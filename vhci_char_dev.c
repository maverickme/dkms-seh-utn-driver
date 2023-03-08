/*
 * Copyright (C) 2013-2014 SEH Computertechnik GmbH
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>		/* everything... */
#include <linux/errno.h>	/* error codes */
#include <linux/types.h>	/* size_t */
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>	/* O_ACCMODE */
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/ioctl.h>


#include "vhci_common.h"
#include "vhci.h"
#include "vhci_event.h"
#include "vhci_to_service.h"
#include "vhci_char_dev.h"
#include "vhci_xmit.h"


int32_t seh_char_dev_major = 0;
char drv_proto_version = 0;

const unsigned char Base64[] = {
    'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P',
    'Q','R','S','T','U','V','W','X','Y','Z','a','b','c','d','e','f',
    'g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v',
    'w','x','y','z','0','1','2','3','4','5','6','7','8','9','+','/'};


module_param(seh_char_dev_major, int, 0);


/* FIXME: Do we need this here??  It be ugly  */
int32_t seh_char_read_procmem(char *buf, char **start, off_t offset,
    int count, int *eof, void *data)
    {
    return 0;
    }
/*give back the urb on failure*/
static void inline urb_give_back (struct urb *urb, int status){
    struct usb_hcd *hcd = bus_to_hcd(urb->dev->bus);
    unsigned long flags;
    urb->status = status;
    HCD_LOCK_IRQSAVE(hcd_to_vhci(hcd), flags);
    usb_hcd_unlink_urb_from_ep(hcd, urb);
    HCD_UNLOCK_IRQRESTORE(hcd_to_vhci(hcd), flags);
    usb_hcd_giveback_urb(hcd, urb, urb->status);
    }        

/*give back the urb on failure*/
static void inline priv_give_back (struct vhci_priv *priv, int status){
    struct urb *urb = priv->urb;
    unsigned long flags;
    if (urb) {
        struct usb_hcd *hcd = bus_to_hcd(urb->dev->bus);
        urb->status = status;
        HCD_LOCK_IRQSAVE(hcd_to_vhci(hcd), flags);
        usb_hcd_unlink_urb_from_ep(hcd, urb);
        HCD_UNLOCK_IRQRESTORE(hcd_to_vhci(hcd), flags);
        seh_dbg_vhci_data ("unlink N %u unlink %u\n", priv->seqnum, priv->unlink_seqnum);
        usb_hcd_giveback_urb(hcd, urb, urb->status);
        }
    seh_dbg_vhci_cdev("%s kfree:%p\n",__func__,priv);
    kfree(priv);
    }  
/*
 * Open char dev
 */

int32_t seh_cdev_open (struct inode *inode, struct file *filp)
    {
    struct seh_char_dev *dev; /* device information */
    seh_dbg_vhci_cdev("%s ...\n",__func__);
    /*  Find the device */
    dev = container_of(inode->i_cdev, struct seh_char_dev, cdev);
    /* and use filp->private_data to point to the device data */
    filp->private_data = dev;
    down(&dev->sem);
    if (dev->b_opened == true){
        pr_err("already opened\n");
        return -EFAULT;
        }
    dev->b_opened  = true;
    up(&dev->sem);
    seh_dbg_vhci_cdev("Exit dev %p OK\n", dev);
    return 0;          /* success */
    }

/*
 * close chardev
 */
int32_t seh_cdev_release (struct inode *inode, struct file *filp)
    {
    uint32_t hcd_index, rhport;
    struct vhci_device *vdev;
    struct seh_char_dev *dev; /* device information */
    seh_dbg_vhci_cdev("%s ...\n",__func__);
    /*  Find the device */
    dev = container_of(inode->i_cdev, struct seh_char_dev, cdev);
    down(&dev->sem);
    if (dev->b_opened == false){
        pr_err("not opened\n");
        up(&dev->sem);
        return -EFAULT;
        }
    dev->b_opened = false;
    dev->is_data_tx_bitmap = 0;
    /*return back URB*/
    if (dev->priv_to_send){
        priv_give_back(dev->priv_to_send, -ENOENT);
        dev->priv_to_send = NULL;
        }
    for (hcd_index = 0; hcd_index < VHCI_CONTROLLERS; hcd_index++){
        struct vhci_hcd *hcd_controller = controllers[hcd_index];
        for (rhport = 0; rhport < VHCI_NPORTS; rhport++){
            vdev = port_to_vdev(hcd_controller, rhport);
            if (vdev->device_status == VDEV_ST_NULL)
                continue;
            vdev->device_status = VDEV_ST_STOP_PENDING;
            /* for all devices */
            vhci_device_unlink_all (vdev);
            rh_port_disconnect(get_hcd_from_vhci_device(vdev), hcd_port_index (vdev->rh_port_id));
            vhci_device_reset(vdev);
            }
        }
    up(&dev->sem);
    seh_dbg_vhci_cdev("Exit dev %p OK\n", dev);
    return 0;
    }



static uint32_t seh_cdev_poll(struct file *filp,  struct poll_table_struct *wait)
    {
    struct seh_char_dev *dev = filp->private_data;
    uint32_t mask = 0;
    down(&dev->sem);
    poll_wait(filp, &dev->txq, wait);
    if (dev->is_data_tx_bitmap){
        mask |= POLLIN | POLLRDNORM;
        /* readable */
        mask |= POLLOUT | POLLWRNORM;
        }
    /* writable */
    up(&dev->sem);
    return mask;
    }

/*
 * send URB to userspace
 * find any URB from any device
 * we read at 2 passes 
 * 	1. read size_t
 * 	2. read body
 * than move urb to sent
 */
ssize_t seh_cdev_read (struct file *filp, char __user *buf, size_t count,
    loff_t *f_pos)
    {
    vusb_read_urb_size __user *from_user = (vusb_read_urb_size __user *)buf;
    struct seh_char_dev *dev = filp->private_data;
    struct vhci_priv *priv = NULL;

    int32_t iRhPort = 0;
    long ret;
    bool is_more;

    seh_dbg_vhci_cdev("%s  dev %p buf %p count %zu pos %p %lld ...\n",__func__, dev, buf, count, f_pos, *f_pos );
    /* access */
    if (unlikely(!access_ok_seh(VERIFY_WRITE, buf, count))){
        pr_err("Access write fail\n");
        return -EFAULT;
        }
    /*check size*/
    if (unlikely(count < sizeof (vusb_read_urb_size))){
        pr_err("Err: size for data %zu\n", count);
        return -ENOBUFS;
        }

    if (down_interruptible(&dev->sem))
        return -ERESTARTSYS;

    /*walk all tx queues and check if data*/  

    while (dev->is_data_tx_bitmap == 0) { /* nothing to read */
        up(&dev->sem); /* release the lock */

        if (filp->f_flags & O_NONBLOCK)
            return -EAGAIN;

        if (wait_event_interruptible(dev->txq, dev->is_data_tx_bitmap ))
            return -ERESTARTSYS; /* signal: tell the fs layer to handle it */
        /* otherwise loop, but first reacquire the lock */
        if (down_interruptible(&dev->sem))
            return -ERESTARTSYS;
        }
    if (dev->priv_to_send){
        pr_err("Err: Data from URB is not read %d\n", iRhPort);
        count = -EFAULT;
        goto exit;
        }

    /*read from next hcd*/
    do{
        dev->last_read_hcd = (dev->last_read_hcd + 1) % VHCI_CONTROLLERS;
        }while (!(dev->is_data_tx_bitmap & (1 << dev->last_read_hcd)));

    iRhPort =  vhci_get_from_hcd_tx_queue(dev->last_read_hcd, &priv, &is_more);
    if (unlikely(!priv)){
        count = -EFAULT;
        clear_bit(dev->last_read_hcd, &dev->is_data_tx_bitmap);
        pr_err("Err: cannot find URB %d\n", iRhPort);
        goto exit;
        }
    if (is_more == false){
        clear_bit(dev->last_read_hcd, &dev->is_data_tx_bitmap);
        seh_dbg_vhci_data1("CLEAR   hcd %d %d  is_data_tx_bitmap %lx \n", dev->last_read_hcd, priv->seqnum, vhci_char_dev->is_data_tx_bitmap);
        }
    seh_dbg_vhci_data1("READ hcd %d %d  is_data_tx_bitmap %lx \n", dev->last_read_hcd, priv->seqnum, vhci_char_dev->is_data_tx_bitmap);
    dev->priv_to_send      = priv;
    dev->priv_to_send_size = vhci_get_priv_send_size (priv);
    seh_dbg_vhci_data("get HCD %d port %d, priv %p number %d\n", dev->last_read_hcd, iRhPort, priv, priv->seqnum);
    ret = __put_user(dev->last_read_hcd , (uint32_t __user *)&from_user->hcd_index)|
        __put_user(dev->priv_to_send_size , (uint32_t __user *)&from_user->size) ;

    if (unlikely(ret)) {
        pr_err("Err: __put_user fail %p\n", dev);
        priv_give_back(dev->priv_to_send, -ENOENT);
        dev->priv_to_send = NULL;
        count = -EFAULT;
        goto exit;
        }
    count = sizeof (vusb_read_urb_size);
    exit:
    up (&dev->sem);
    seh_dbg_vhci_cdev("%s Exit data to sent to SVC=%lx\n", __func__,(long)count);
    return count;
    }
/*
 * get URB from userspace
 */
ssize_t seh_cdev_write (struct file *filp, const char __user *buf, size_t count,
    loff_t *f_pos)
    {
    uint32_t i, bus, addr, ip, seqnum;
    struct vhci_device *vdev;
    struct urb *urb;
    bool post_action = false;
    struct vhci_priv *priv;

    seh_dbg_vhci_cdev("%s filp:%p buf:%p count:%zu pos:%lld\n",__func__,filp, buf, count, *f_pos);
    /*get vdev*/
    if (unlikely(get_indexes_from_buf (buf, count, &bus, &addr, &ip, &seqnum) == false)){
        pr_err("Cannot Get indexes \n");
        return 0;
        }
    /*find priv*/
    priv = vhci_get_urb_from_rx_queue (bus, addr, ip, seqnum);
    if (!priv){
        //pr_err("Priv not found (bus %d addr %d ip %x sequence number %d)\n",bus , addr, ip, seqnum);
#ifdef SEH_HACK_NO_IN_WAIT
        return count;
#else
        return 0;
#endif
        }
    seh_dbg_vhci_tx("Found vdev %p URB %p unlink %d bus %d addr %d ip %x sequence number %d)\n",
        priv->vdev, priv->urb, is_unlink(priv), bus , addr, ip, seqnum);
    vdev = priv->vdev;
    /*we can have 3 tyes  unlink, urb, command*/
    if (likely(priv->urb)){
        urb = priv->urb;
        /* recv transfer buffer */
        if (vhci_get_from_service(priv, buf, count, urb) < 0){
            pr_info("Fail to get data, status %d\n", urb->status);
            urb->status = -EPROTO;
            }
        if (urb->status == -ENODEV){
            pr_err("No device, status %d\n", urb->status);
            post_action = true;
            }
        if (urb->status == -EINPROGRESS) {
            /* This request is successfully completed. */
            /* If not -EINPROGRESS, possibly unlinked. */
            urb->status = 0;
            }
        seh_dbg_vhci_data("%s giveBack URB seq:%d to USB stack urb:%p status:%d len:%d\n", __func__,
            priv->seqnum,urb, urb->status, urb->actual_length);

        if (urb->status == 0 && usb_pipeisoc(urb->pipe) &&  ! usb_pipein(urb->pipe)) {
            seh_dbg_vhci_data("%s makeHappy isofrags:%d actLen:%d->%d flags:%#x\n", __func__,
                urb->number_of_packets,
                urb->actual_length,
                urb->transfer_buffer_length,
                urb->transfer_flags);
            
            for (i=0; i < urb->number_of_packets; i++) {
                seh_dbg_vhci_data("%s clear OUT frame[%2d] off:%5d len:%d/%3d stat:%d->0\n",__func__,i,
                    urb->iso_frame_desc[i].offset,
                    urb->iso_frame_desc[i].actual_length,
                    urb->iso_frame_desc[i].length,
                    urb->iso_frame_desc[i].status);
                urb->iso_frame_desc[i].status = 0; // success
                urb->iso_frame_desc[i].actual_length = urb->iso_frame_desc[i].length; // test killme
                }
            urb->actual_length = urb->transfer_buffer_length;
            }
        urb_give_back(urb, urb->status);
        free_priv(priv);
        }
    else if (is_unlink(priv)){
        struct vhci_priv *priv_unlink = priv;
        if (vhci_get_cmd_reply_from_service(buf, count) != StatOk){
            post_action = 1;
            }
        priv = get_priv_from_rx_by_num (vdev, priv_unlink->unlink_seqnum);
        free_priv(priv_unlink);   
        if (priv){
            if (priv->urb){
                urb = priv->urb;
                if (post_action)/*fail*/{
                    urb->status = -ENODEV;
                    seh_dbg_vhci_tx("Unlink OK status %d\n", urb->status);
                    }else/*OK*/{
                    urb->status = -ECONNRESET;
                    seh_dbg_vhci_tx("Unlink OK status %d\n", urb->status);
                    }
                urb_give_back(urb, urb->status);
                }
            free_priv(priv);  
            }
        }
    else if (is_vers2(priv)){
        int32_t err;
        uint32_t nw_hndl;
        uint16_t comp_id;
        if ((err = vhci_get_vers2_reply_from_service(buf, count, &nw_hndl, &comp_id)) != StatOk){
            post_action = 1;
            seh_dbg_vhci_tx("VERS2 reply %d\n", err);
            }
        else { /*StatOk*/
            if (comp_id % 100 >= 8 && strlen(vdev->hndl_plus_prtkey + 4)) {
                vdev->hndl_plus_prtkey[0] = Base64[(nw_hndl>>24) % sizeof(Base64)];
                vdev->hndl_plus_prtkey[1] = Base64[(nw_hndl>>16) % sizeof(Base64)];
                vdev->hndl_plus_prtkey[2] = Base64[(nw_hndl>> 8) % sizeof(Base64)];
                vdev->hndl_plus_prtkey[3] = Base64[(nw_hndl>> 0) % sizeof(Base64)];
                seh_dbg_vhci_tx("base64 %c%c%c%c\n", vdev->hndl_plus_prtkey[0], vdev->hndl_plus_prtkey[1],
                        vdev->hndl_plus_prtkey[2],vdev->hndl_plus_prtkey[3]);
                seh_vusb_add_cmd_to_svc(hcd_index(vdev->rh_port_id), vdev, CLAIM3);
                }
            else {
                seh_vusb_add_cmd_to_svc(hcd_index(vdev->rh_port_id), vdev, CLAIM);
                }
            seh_vusb_add_cmd_to_svc(hcd_index(vdev->rh_port_id), vdev, IF_STAT);
            }
        }
    else/*no urb - this means some command*/{
        int32_t err;
        if ((err = vhci_get_cmd_reply_from_service(buf, count)) != StatOk){
            post_action = 1;
            seh_dbg_vhci_tx("Command reply %d\n", err);
            }
        }
    if (post_action){
        unpluging_device(vdev);
        }
    seh_dbg_vhci_cdev("Exit %zu", count);
    return count;
    }
/*
 * ioctl_to_str()
 */
static int8_t *ioctl_to_str (uint32_t cmd)
    {
    static char doof[16];
    switch (cmd){
    case SEH_IOC_MAGIC:
        return "SEH Magic";
    case LINUX_IOCTL_VUSB_PLUGIN_HARDWARE:
        return "LINUX_IOCTL_VUSB_PLUGIN_HARDWARE";
    case LINUX_IOCTL_VUSB_EJECT_HARDWARE:
        return "LINUX_IOCTL_VUSB_EJECT_HARDWARE";
    case LINUX_IOCTL_VUSB_UNPLUG_HARDWARE:
        return "LINUX_IOCTL_VUSB_UNPLUG_HARDWARE";
    case LINUX_IOCTL_VUSB_GET_INSTANCE_ID:
        return "LINUX_IOCTL_VUSB_GET_INSTANCE_ID";
    case LINUX_IOCTL_VUSB_GET_INTERFACE_STATUS:
        return "LINUX_IOCTL_VUSB_GET_INTERFACE_STATUS";
    case LINUX_IOCTL_VUSB_HEARTBEAT:
        return "LINUX_IOCTL_VUSB_HEARTBEAT";
    case LINUX_IOCTL_VUSB_SET_HEARTBEAT_TIMEOUT:
        return "LINUX_IOCTL_VUSB_SET_HEARTBEAT_TIMEOUT";
    case LINUX_IOCTL_VUSB_GET_VERSION:
        return "LINUX_IOCTL_VUSB_GET_VERSION";
    default:
        sprintf(doof,"0x%x",cmd);
        return doof;
        }
    }

/*
 * The ioctl() implementation
 */
long seh_cdev_ioctl (struct file *filp,uint32_t cmd, unsigned long arg)
    {
#define get_param_ptr(A) ((uintptr_t)param[A])
#define get_param_u32(A) ((uint32_t)param[A])
    uint64_t param[5];
    int32_t ret = 0;
    uint32_t in_size = 0;
    uint32_t out_max_size = 0;
    int unused __attribute__((unused));

    seh_dbg_vhci_cdev("%s(%s) %#x\n", __func__,ioctl_to_str(cmd),cmd);

    /* don't even decode wrong cmds: better returning  ENOTTY than EFAULT */
    if (_IOC_TYPE(cmd) != SEH_IOC_MAGIC || _IOC_NR(cmd) > SEH_VHCI_IOC_MAXNR){
        pr_err("wrong cmd %s\n", ioctl_to_str(cmd));
        return -ENOTTY;
        }

    if (!access_ok_seh(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd))){
        pr_err("Cannot read user data\n");
        return -EFAULT;
        }
    /*GET DATA*/
    if (cmd == LINUX_IOCTL_VUSB_GET_URB_DATA){
        struct seh_char_dev *dev = filp->private_data;
        ret = seh_vusb_get_urb_data (dev, (void __user *)arg);
        seh_dbg_vhci_cdev("%s URB_DATA done %d\n", __func__,ret);
        return ret;
        }

    unused=__copy_from_user(param, (void __user *)arg, sizeof (param));
    in_size = get_param_u32(1);
    out_max_size = get_param_u32(3);

    switch(cmd) {
    case LINUX_IOCTL_VUSB_PLUGIN_HARDWARE:{
        vusb_plugin_hardware hw;
        /*check access*/
        if (!access_ok_seh(VERIFY_READ, (void __user *)get_param_ptr(0), in_size)){
            pr_err("Cannot read user data\n");
            return -EFAULT;
            }
        /*get from user space*/
        unused=__copy_from_user(&hw, (void __user *)get_param_ptr(0), sizeof (vusb_plugin_hardware));
        /*validate*/
        if (in_size != sizeof (vusb_plugin_hardware) || hw.size != sizeof (vusb_plugin_hardware)){
            pr_err("Cannot read user data- size do not match\n");
            return -EINVAL;
            }
        ret = seh_vusb_plug_hw (&hw);
        }
        break;
    case LINUX_IOCTL_VUSB_UNPLUG_HARDWARE:{
        vusb_plugout_hardware hw;
        /*check access*/
        if (!access_ok_seh(VERIFY_READ, (void __user *)get_param_ptr(0), in_size)){
            pr_err("Cannot read user data\n");
            return -EFAULT;
            }
        /*get from user space*/
        unused=__copy_from_user(&hw, (void __user *)get_param_ptr(0), sizeof (vusb_plugout_hardware));
        /*validate*/
        if (in_size != sizeof (vusb_plugout_hardware) || hw.size != sizeof (vusb_plugout_hardware)){
            pr_err("Cannot read user data- size fo not match\n");
            return -EINVAL;
            }
        ret = seh_vusb_unplug_hw (&hw);
        }
        break;
    case LINUX_IOCTL_VUSB_EJECT_HARDWARE:{
        vusb_plugout_hardware hw;
        /*check access*/
        if (!access_ok_seh(VERIFY_READ, (void __user *)get_param_ptr(0), in_size)){
            pr_err("Cannot read user data\n");
            return -EFAULT;
            }
        /*get from user space*/
        unused=__copy_from_user(&hw, (void __user *)get_param_ptr(0), sizeof (vusb_plugout_hardware));
        /*validate*/
        if (in_size != sizeof (vusb_plugout_hardware) || hw.size != sizeof (vusb_plugout_hardware)){
            pr_err("Cannot read user data- size fo not match\n");
            return -EINVAL;
            }
        ret = seh_vusb_eject_hw  (&hw);
        }
        break;
    case LINUX_IOCTL_VUSB_GET_INSTANCE_ID: {
        vusb_get_instance_id hw;
        uint32_t Id = 0;
        /*check in data*/
        if (!access_ok_seh(VERIFY_READ, (void __user *)get_param_ptr(0), in_size)){
            pr_err("Cannot read user data\n");
            return -EFAULT;
            }
        /*get from user space*/
        unused=__copy_from_user(&hw, (void __user *)get_param_ptr(0), sizeof (vusb_get_instance_id));
        /*validate IN*/
        if (in_size != sizeof (vusb_get_instance_id) || hw.size != sizeof (vusb_get_instance_id)){
            pr_err("Cannot read user data- size fo not match\n");
            return -EINVAL;
            }

        /*check out data*/
        if (!access_ok_seh(VERIFY_WRITE, (void __user *)get_param_ptr(2), out_max_size)){
            pr_err("Cannot read user data\n");
            return -EFAULT;
            }
        if (out_max_size != sizeof (Id)){
            pr_err("Cannot read user data- size fo not match\n");
            return -EINVAL;
            }

        ret = seh_vusb_get_inst_id (&hw, &Id);

        /*now return data*/
        __put_user(Id, (uint32_t __user *)get_param_ptr(2));
        /*size written*/
        __put_user(sizeof (Id), (uint32_t __user *)get_param_ptr(4));
        }
        break;
    case LINUX_IOCTL_VUSB_GET_INTERFACE_STATUS: {
        vusb_get_if_status hw;
        int32_t Status;
        /*check in data*/
        if (!access_ok_seh(VERIFY_READ, (void __user *)get_param_ptr(0), in_size)){
            pr_err("Cannot read user data\n");
            return -EFAULT;
            }
        /*get from user space*/
        unused=__copy_from_user(&hw, (void __user *)get_param_ptr(0), sizeof (vusb_get_if_status));
        /*validate IN*/
        if (in_size != sizeof (vusb_get_if_status) || hw.size != sizeof (vusb_get_if_status)){
            pr_err("Cannot read user data- size fo not match\n");
            return -EINVAL;
            }

        /*check out data*/
        if (!access_ok_seh(VERIFY_WRITE, (void __user *)get_param_ptr(2), out_max_size)){
            pr_err("Cannot read user data\n");
            return -EFAULT;
            }
        if (out_max_size != sizeof (Status)){
            pr_err("Cannot read user data- size fo not match\n");
            return -EINVAL;
            }
        ret = seh_vusb_get_if_status (&hw, &Status);
        /*now return data*/
        __put_user(Status, (uint32_t __user *)get_param_ptr(2));
        /*size written*/
        __put_user(sizeof (Status), (uint32_t __user *)get_param_ptr(4));
        }
        break;
    case LINUX_IOCTL_VUSB_SET_HEARTBEAT_TIMEOUT:
        ret = seh_vusb_set_hb ();
        break;
    case LINUX_IOCTL_VUSB_HEARTBEAT:
        ret = seh_vusb_hartbeat ();
        // ? no break?
        break;
    case LINUX_IOCTL_VUSB_GET_VERSION:{

        if (!access_ok_seh(VERIFY_WRITE, (void __user *)get_param_ptr(2), sizeof (drv_proto_version))){
            pr_err("Cannot read user data\n");
            return -EFAULT;
            }
        /*now return data*/
        __put_user(drv_proto_version, (char __user *)get_param_ptr(2));
        /*size written*/
        __put_user(sizeof (drv_proto_version), (uint32_t __user *)get_param_ptr(4));

        }
        break;

    default:  /* redundant, as cmd was checked against MAXNR */
        pr_err("Unknown cmd %x\n", cmd);
        return -ENOTTY;
        }
    seh_dbg_vhci_cdev("%s done %d\n", __func__,ret);
    return ret;
    }

/*
 * The fops
 */

struct file_operations seh_vhci_fops = {
    .owner = THIS_MODULE,
    .read  = seh_cdev_read,
    .write = seh_cdev_write,
    .poll  = seh_cdev_poll,
    .unlocked_ioctl = seh_cdev_ioctl,
#ifdef CONFIG_COMPAT  
    .compat_ioctl   = seh_cdev_ioctl,
#endif  
    .open =		seh_cdev_open,
    .release =  seh_cdev_release,
    };

/*
 * seh_chardev_create
 * create char device
 */
int32_t seh_chardev_create (void)
    {
    int32_t result;
    dev_t dev = MKDEV(seh_char_dev_major, 0);
    struct seh_char_dev *seh_char_dev;
    seh_dbg_vhci_cdev("%s ...\n",__func__);
    /*
     * Register your major, and accept a dynamic number.
     */
    if (seh_char_dev_major)
        result = register_chrdev_region(dev, 1, SEH_CHAR_DEV_NAME);
    else {
        result = alloc_chrdev_region(&dev, 0, 1, SEH_CHAR_DEV_NAME);
        seh_char_dev_major = MAJOR(dev);
        }
    if (result < 0){
        pr_err("fail to get major number\n");
        return result;
        }

    seh_char_dev = kzalloc(sizeof (struct seh_char_dev), GFP_KERNEL);
    if (!seh_char_dev) {
        result = -ENOMEM;
        pr_err("no memory to allocate char dev\n");
        goto fail_malloc;
        }
    /*link to other structures*/
    vhci_char_dev = seh_char_dev;

    /*inti char dev*/ 
    cdev_init(&seh_char_dev->cdev, &seh_vhci_fops);
    seh_char_dev->cdev.owner = THIS_MODULE;
    seh_char_dev->cdev.ops = &seh_vhci_fops;
    result = cdev_add (&seh_char_dev->cdev, dev, 1);
    /* Fail gracefully if need be */
    if (result)
        pr_err("Error %d adding seh char device", result);
    seh_dbg_vhci_cdev("Add device major %d, result %d\n", seh_char_dev_major, result);
    /*init wait queue*/
    init_waitqueue_head(&seh_char_dev->txq);
    sema_init (&seh_char_dev->sem, 1);
    seh_char_dev->is_data_tx_bitmap = 0;

    /* 
     * allocate the devices -- we can't have them static, as the number
     * can be specified at load time
     */
    // tbd create_proc_read_entry(SEH_CHAR_PROC, 0, NULL, seh_char_read_procmem, NULL);

    seh_dbg_vhci_cdev("Exit \n");

    return 0; /* succeed */

    fail_malloc:
    unregister_chrdev_region(dev, 1);
    return result;
    }


/*
 * seh_chardev_delete
 * eliminate char device
 */
int32_t seh_chardev_delete(void)
    {
    seh_dbg_vhci_cdev("%s ...\n",__func__);

    cdev_del(&vhci_char_dev->cdev);
    if (vhci_char_dev){
        seh_dbg_vhci_cdev("%s free char_dev %p\n",__func__,vhci_char_dev);
        kfree(vhci_char_dev);
        }
    vhci_char_dev = NULL;
    // tbd remove_proc_entry(SEH_CHAR_PROC, NULL);
    unregister_chrdev_region(MKDEV (seh_char_dev_major, 0), 1);
    seh_dbg_vhci_cdev("Exit \n");
    return 0;
    }
