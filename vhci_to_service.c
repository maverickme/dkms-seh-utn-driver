/*
 * Copyright (C) 2013-2014 SEH Computertechnik GmbH
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307,
 * USA.
 */

#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/utsname.h>


#include "vhci_common.h"
#include "vhci.h"
#include "vhci_event.h"
#include "vhci_char_dev.h"
#include "vhci_to_service.h"
#include "vhci_sd_boost.h"
#include "sha1.h"

#define KERNEL
#define __OS_LINUX__

/*DUMMY */
// #define usb_device_descriptor void*
// #define vusbResponseHeader void *

#include "service_defs.h"

#define USBDEVFS_URB_TYPE_ISO	   0
#define USBDEVFS_URB_TYPE_INTERRUPT	   1
#define USBDEVFS_URB_TYPE_CONTROL	   2
#define USBDEVFS_URB_TYPE_BULK	   3

static int unused __attribute__((unused));
void vhci_device_unlink_all(struct vhci_device *vdev);

static inline void seh_vusb_get_dev_ids (__u32 *bus, __u32 *addr, __u32 *ip, vusb_device_addr *indexes){
    *bus = indexes->busno;
    *addr = indexes->addr;
    *ip = indexes->ipl;
    }


/*
 *  seh_vusb_plug_hw
 *
 *
 *
 *
 */

int32_t seh_vusb_plug_hw (vusb_plugin_hardware *hw){
    uint32_t rhport = 0, addr, ip, bus, n, hcd_index, hcd_end;
    struct vhci_device *vdev, *vdev_free = NULL;
    struct vhci_hcd *hcd_controller, *hcd_controller_free = NULL;

    seh_dbg_vhci_service("%s ip:%#x bus:%d addr:%d...\n", __func__, htonl(hw->addr.ipl), hw->addr.busno, hw->addr.addr);
    seh_vusb_get_dev_ids (&bus, &addr, &ip, &hw->addr); 

#if 0
    ....this is gone to vusblib/src/... for legacy reasons
    if (hw->bcdUSB >= 0x0300) {
        speed  = USB_SPEED_SUPER;
        hcd_index = VHCI_FIRST_USB3;
        hcd_end = VHCI_CONTROLLERS;
        }
    else{
        if (hw->bcdUSB >= 0x0200)
            speed  = USB_SPEED_HIGH;
        else if (hw->bcdUSB >= 0x0110)
            speed  = USB_SPEED_FULL;
        else if (hw->bcdUSB >= 0x0100)
            speed  = USB_SPEED_LOW;
        hcd_index = 0;
        hcd_end   = VHCI_FIRST_USB3;
        }
#endif
        
    if (hw->bcdUSB >= 0x0300) {
        hcd_index = VHCI_FIRST_USB3;
        hcd_end   = VHCI_CONTROLLERS;
        }
    else{
        hcd_index = 0;
        hcd_end   = VHCI_FIRST_USB3;
        }

    for (; hcd_index < hcd_end; hcd_index++)
        {
        hcd_controller = controllers[hcd_index];
        /*check if not assigned by any other and store first free*/
        for (rhport = 0; rhport < VHCI_NPORTS; rhport++){
            vdev = port_to_vdev(hcd_controller, rhport);
            seh_dbg_vhci_service(">>>[hcd:%d rhport:%d] ip:%#x bus:%d addr:%d stat:%d\n", hcd_index, rhport,
                                 ntohl(vdev->ip), vdev->bus, vdev->addr, vdev->device_status);
            if (vdev->device_status == VDEV_ST_NULL){
                if (!vdev_free) 
                    vdev_free = vdev;
                if (!hcd_controller_free) 
                    hcd_controller_free = hcd_controller;
                }

            if (vdev->bus == bus && vdev->addr == addr && vdev->ip == ip &&
                vdev->device_status != VDEV_ST_NULL) {
                /* end of the lock */
                dev_err(NULL, "port %d already used(dev %d/%d/%x stat %d\n",rhport, bus,addr, ip, vdev->device_status);
                return -EINVAL;
                }

            }
        }
    if (!vdev_free){
        pr_err ("Root hub no free ports found\n");
        return 0;
        }
    rhport = hcd_port_index(vdev_free->rh_port_id);
    seh_dbg_vhci_service("vdev_free:           %p\n",vdev_free);
    seh_dbg_vhci_service("hcd_controller_free: %p\n",hcd_controller_free);
    
	vdev_free->bus               = bus;
	vdev_free->addr              = addr;
	vdev_free->ip                = ip;
	vdev_free->speed             = hw->speed;
	vdev_free->device_status     = VDEV_ST_NOTASSIGNED;
	vdev_free->useSSL            = hw->useSSL;
    vdev_free->buffersHackBoosted = 0;
    
	n = sizeof(vdev_free->username);
	strncpy(vdev_free->username, hw->username, n);
	if (n > 0)
		vdev_free->username[n-1] = '\0';

    /* 4 bytes (sizeof(UINT) reserved for network handle */
    memcpy(vdev_free->hndl_plus_prtkey + 4, hw->prtKey, sizeof(hw->prtKey));

	rh_port_connect(hcd_controller_free, rhport, hw->speed);
	
    seh_dbg_vhci_service("!!![hcd:%d rhport:%u] ip:%#x bus:%d addr:%d speed:%u stat:%d\n", hcd_controller_free->id, rhport,
                         ntohl(ip), bus, addr, hw->speed, vdev_free->device_status);
	/*now send to service command - a little bit stupit protocol but...*/
	
    seh_vusb_add_cmd_to_svc(hcd_controller_free->id, vdev_free, VERS2);
    
	seh_dbg_vhci_service("%s done\n",__FUNCTION__);
	return 0;
  
}

/*
 * Unplug
 */
int32_t seh_vusb_unplug_hw (vusb_plugout_hardware *hw){
    uint32_t hcd_index, rhport;
    struct vhci_device *vdev;
    bool found = false;

    seh_dbg_vhci_service("%s ip:%#x bus:%d addr:%d...\n", __func__, ntohl(hw->addr.ipl), hw->addr.busno, hw->addr.addr);

    for (hcd_index = 0; hcd_index < VHCI_CONTROLLERS; hcd_index++){
        for (rhport = 0; rhport < VHCI_NPORTS; rhport++){
            vdev = port_to_vdev(controllers[hcd_index], rhport);
            if (vdev->bus == hw->addr.busno && vdev->addr == hw->addr.addr && vdev->ip == hw->addr.ipl){
                found = true;
                break; 
                }
            }
        if (found) break;
        }
    if (!found){
        pr_err ("Root hub port not found\n");
        return -EINVAL;
        }

    if (vdev->device_status == VDEV_ST_NULL) {
        pr_err("not connected %d\n", vdev->device_status);
        return -EINVAL;
        }
    unpluging_device(vdev);

    seh_dbg_vhci_service("%s Exit [hcd:%d rhport:%#x] stat:%d\n", __func__, hcd_index, vdev->rh_port_id, vdev->device_status);
    return 0;
    }

/***********************************************************************
 * Eject
 ***********************************************************************/
int32_t seh_vusb_eject_hw (vusb_plugout_hardware *hw){
    return seh_vusb_unplug_hw (hw);
    }

/***********************************************************************
 * Get instance ID
 **********************************************************************/
int32_t seh_vusb_get_inst_id (vusb_get_instance_id *hw, uint32_t *Id){

    uint32_t rhport = -1;
    uint32_t hcd_index;
    struct vhci_device *vdev;

    seh_dbg_vhci_service("%s ...\n", __FUNCTION__);

    for (hcd_index = 0; hcd_index < VHCI_CONTROLLERS; hcd_index++){
        for (rhport = 0; rhport < VHCI_NPORTS; rhport++){
            vdev = port_to_vdev(controllers[hcd_index], rhport);
            if (vdev->bus == hw->addr.busno && vdev->addr == hw->addr.addr && vdev->ip == hw->addr.ipl){
                rhport = rhport_get(hcd_index, rhport);
                break;
                }
            }
        if (rhport != -1) break;
        }
    if (rhport == -1){
        pr_err ("Root hub port not found\n");
        return -EINVAL;
        }
    *Id = rhport;
    seh_dbg_vhci_service("Exit instance ID %d\n", rhport);
    return 0;
    }

/***************************************************************************
 * Get if status
 ***************************************************************************/
int32_t seh_vusb_get_if_status(vusb_get_if_status *hw, uint32_t *Status)
    {
    uint32_t hcd_index, rhport;
    struct vhci_device *vdev;
    bool found = false;
    seh_dbg_vhci_service("%s ip:%#x bus:%d addr:%d...\n",__func__, ntohl(hw->addr.ipl), hw->addr.busno, hw->addr.addr);

    for (hcd_index = 0; hcd_index < VHCI_CONTROLLERS; hcd_index++){
        for (rhport = 0; rhport < VHCI_NPORTS; rhport++){
            vdev = port_to_vdev(controllers[hcd_index], rhport);
            if (vdev->bus == hw->addr.busno && vdev->addr == hw->addr.addr && vdev->ip == hw->addr.ipl){
                found = true;
                break; 
                }
            }
        if (found) break;
        }
    if (!found){
        *Status = PNP_Unknown;
        seh_dbg_vhci_service("Exit status %d port rhport unknown\n", *Status);
        return 0;
        }
    /* [fl] assign corresponding windows pnp states used by utnservice */
    if (vdev->device_status == VDEV_ST_NOTASSIGNED) 
        *Status = PNP_NotStarted;
    else if (vdev->device_status == VDEV_ST_USED) 
        *Status = PNP_Started;
    /*stop pending means we wait service to read the status
     * than we make real reset*/
    else if (vdev->device_status == VDEV_ST_STOP_PENDING){
        *Status = PNP_StopPending;
        }
    else if (vdev->device_status == VDEV_ST_STOPPED){
        unplug_device(vdev);
        *Status = PNP_Deleted;
        }
    else
        *Status = PNP_Unknown;
    seh_dbg_vhci_service("%s Exit [hcd:%d rhport:%#x] ip:%#x bus:%d addr:%d status:%d stat:%d\n", __func__, hcd_index, vdev->rh_port_id,
                         ntohl(vdev->ip), vdev->bus, vdev->addr, *Status, vdev->device_status);
    return 0;
    }
int32_t seh_vusb_set_hb (void){
    return 0;
    }

int32_t seh_vusb_hartbeat (void){
    seh_vusb_boost_storage_devices();
    return 0;
    }
//-----------------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------------

static int isoToService(UCHAR __user *data, struct urb *urb, int *hdrLen)
    {
    int i,len = 0;
    
    if (urb->number_of_packets < 1 || urb->number_of_packets > 256) {
        printk("%s error iso Header with %d packets?!\n",__func__,urb->number_of_packets);
        return -1;
        }
        
    seh_dbg_vhci_data("%s ISO Frags:%d buf:%d/%d flags:%#x\n", 
        __func__, urb->number_of_packets,
        urb->actual_length,
        urb->transfer_buffer_length,
        urb->transfer_flags);
    
    //   if (data && ! usb_pipein(urb->pipe)) {  // OUT iso data
    if (data) {  // OUT and IN iso data
        struct IsoPa isofirst, *isoip = NULL;
        int    fralen = 0;
        
        fralen = sizeof(struct IsoPa) * urb->number_of_packets;
        isoip  = urb->number_of_packets == 1 ? &isofirst : (struct IsoPa*) kmalloc(fralen, GFP_KERNEL);

        seh_dbg_vhci_data("%s %s size:%d %p %s\n",__func__,
            usb_pipein(urb->pipe) ? "IN" : "OUT", fralen,isoip,isoip == &isofirst ? "stack" : "alloc");
        
        for (i=0; i < urb->number_of_packets; i++) {
            int l = urb->iso_frame_desc[i].actual_length;
            // OUT TEST TAKE ALL ALL ALL
            isoip[i].status = urb->iso_frame_desc[i].status;
            isoip[i].length = urb->iso_frame_desc[i].length;
            isoip[i].offs   = urb->iso_frame_desc[i].offset;
            isoip[i].pad    = 0xaffebeaf; // unused ... urb->iso_frame_desc[i].length;

            // only debug output
            seh_dbg_vhci_data("%s frame[%2d] off:%5d len:%3d/%3d stat:%d\n",__func__,i,
                urb->iso_frame_desc[i].offset, l,
                    urb->iso_frame_desc[i].length, 
                urb->iso_frame_desc[i].status);
            }
        
        i = copy_to_user (data,isoip,fralen);
        seh_dbg_vhci_data("%s cp2usr:%p rest:%d len:%#x\n",__func__,data,i,fralen);
        *hdrLen += fralen;
        data    += fralen;
        seh_dbg_vhci_data("%s %s frags:%d len:%d added\n",__func__,
            usb_pipein(urb->pipe) ? "IN" : "OUT", urb->number_of_packets,fralen);

        if ( ! usb_pipein(urb->pipe)) {  // OUT iso data, deliver it
            i = copy_to_user (data, urb->transfer_buffer, urb->transfer_buffer_length);
            seh_dbg_vhci_data("%s cp2usr:%p rest:%d len:%#x\n",__func__,data,i, urb->transfer_buffer_length);
            len = urb->transfer_buffer_length;
            seh_dbg_vhci_data("%s OUT buffer len:%d added\n",__func__,len);
            }
        
        if (isoip && isoip != &isofirst) {
            seh_dbg_vhci_data("%s kfree:%p\n",__func__,isoip);
            kfree(isoip);
            }
        }
    else   //  no data, should never reached
        printk("%s error: no data?!\n",__func__);
    
    seh_dbg_vhci_data("%s len:%d (%#x) hdrLen:%d done\n",__func__,len,len, *hdrLen); 
    return len;
    }


/******************************************************************************
 * URB processing 
 ******************************************************************************/
static inline int fill_urb_to_svc(struct vhci_priv *priv, struct urb *urb,
    struct usb_host_endpoint	*ep,
    struct usb_ctrlrequest *setup, VUSB_GET_URB __user *pServiceUrb)
    {
    uint32_t len                = 0;
    uint32_t hdrLen             = sizeof (struct _VUSB_URB_HEADER);
    struct _VUSB_URB_HEADER __user *uh = &pServiceUrb->data.header;
    UCHAR __user *data          = NULL;
    const char *ept             = "Error"; // DEBUG INFO ONLY

    /*Fill data*/
    __put_user (IO_TYPE_URB, &pServiceUrb->dataType );
    __put_user (urb->number_of_packets, &pServiceUrb->dataCntr);

    /*fill header*/
    __put_user (priv->vdev->addr,&uh->addr);
    __put_user (priv->vdev->bus, &uh->busnr);
    
    /*__put_user (usb_pipeendpoint(urb->pipe), &uh->endpoint);*/
    __put_user (ep->desc.bEndpointAddress, &uh->endpoint);

    switch (usb_endpoint_type(&ep->desc)) {
    case USB_ENDPOINT_XFER_ISOC :
        __put_user (USBDEVFS_URB_TYPE_ISO, &uh->type);
        ept="Isoc";
        break;
    case USB_ENDPOINT_XFER_INT:
        __put_user (USBDEVFS_URB_TYPE_INTERRUPT, &uh->type);
        ept="Intr";
        break;
    case USB_ENDPOINT_XFER_CONTROL:
        __put_user (USBDEVFS_URB_TYPE_CONTROL, &uh->type);
        ept="Ctrl";
        break;
    case USB_ENDPOINT_XFER_BULK:
        __put_user (USBDEVFS_URB_TYPE_BULK, &uh->type);
        ept="Bulk";
        break;
        }
    seh_dbg_vhci_data("%s EpType:%s\n",__func__,ept);

    /*put max len if out*/
    if (usb_pipein(urb->pipe))
        __put_user (htonl(urb->transfer_buffer_length), &uh->maxLength);
    else
        __put_user (0, &uh->maxLength);

    len = 0;
    if (USB_ENDPOINT_XFER_CONTROL == usb_endpoint_type(&ep->desc)) {
        VUSB_CTRL_TRANSFER   *rq  = &pServiceUrb->data.urbCtrlRequest.ctrl;
        __put_user (setup->bRequestType,  &rq->bmRequestType );
        __put_user (setup->bRequest,      &rq->bRequest );
        __put_user (htons(setup->wValue), &rq->wValue );
        __put_user (htons(setup->wIndex), &rq->wIndex );
        __put_user (htons(setup->wLength),&rq->wLength );
        __put_user ((setup->wValue>>8),   &pServiceUrb->descriptorType); 
        len += sizeof (struct _VUSB_CTRL_TRANSFER);
        data = (char __user*) &pServiceUrb->data.urbCtrlRequest.data;

        seh_dbg_vhci_data("%s CONTROL  \n\tReqType %d,bRequest %d, tWValue %d \n\tWIndex %d, WLength %d,DescType %d, Len %u\n", 
            __func__,setup->bRequestType,
            setup->bRequest, setup->wValue, setup->wIndex , setup->wLength , (setup->wValue>>8), len);
        }
    else 
        data = (char __user *) &pServiceUrb->data.urbBulkOrInterruptRequest.data;
    
    if (usb_pipeisoc(urb->pipe)) {
        len =  isoToService(data, urb, &hdrLen);
        if (len < 0)  // the only exit
            return -1;
        }
    else if ( ! usb_pipein(urb->pipe)) {
        
        if ( ! urb->num_sgs) {
            if (urb->transfer_buffer_length && data){
                len   += urb->transfer_buffer_length;
                unused = copy_to_user (data, urb->transfer_buffer, urb->transfer_buffer_length);
                seh_dbg_vhci_data("%s noIso cp2usr:%p len:%#x\n",__func__,data,urb->transfer_buffer_length);
                }
            }
        else
            {
            struct sg_mapping_iter *miter = &priv->miter;
            u32 this_sg;
            bool next_sg;
            u32 trans = 0;
            u32 all_len = urb->transfer_buffer_length;
            u32 flags = SG_MITER_FROM_SG;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 35)         
            sg_miter_start(miter, urb->sg->sg, urb->num_sgs, flags);
#else
            sg_miter_start(miter, urb->sg, urb->num_sgs, flags);
#endif            
            next_sg = sg_miter_next(miter);
            if (next_sg == false) {
                WARN_ON_ONCE(1);
                printk ("urb->num_sgs %d\n", urb->num_sgs);
                sg_miter_stop(miter);
                return -1;
                }
            do {
                this_sg = min_t(u32, all_len, miter->length);
                miter->consumed = this_sg;
                trans += this_sg;
                unused = copy_to_user (data, miter->addr, this_sg);
                seh_dbg_vhci_data("%s ! miter ! error ! cp3usr:%p len:%#x\n",__func__,data,this_sg);
                all_len -= this_sg;

                if (!all_len)
                    break;
                next_sg = sg_miter_next(miter);
                if (next_sg == false) {
                    printk ("urb->num_sgs %d\n", urb->num_sgs);
                    WARN_ON_ONCE(1);
                    return -1;
                    }
                data += this_sg;
                } while (1);
            sg_miter_stop(miter);
            len += urb->transfer_buffer_length;
            }
        }
    
    __put_user (htonl(len), &uh->length); /*only data + control pkt*/
    __put_user (len+hdrLen, &pServiceUrb->dataLength );

    seh_dbg_vhci_data("%s %s seq:%u bus:%d addr:%d ep:%#x type:%d buf/len/hdr:%d/%d/%d setup:%p\n",__func__,
        (usb_pipein(urb->pipe) ? "IN":"OUT"),priv->seqnum,
        priv->vdev->bus, priv->vdev->addr, ep->desc.bEndpointAddress, usb_endpoint_type(&ep->desc),
        urb->transfer_buffer_length,len, hdrLen,
        urb->setup_packet);
    
    return 0;
    }

//----------------------------------------------------------------------------------------------------------
//
//----------------------------------------------------------------------------------------------------------

static inline void fill_set_cfg_to_svc(struct vhci_priv *priv, struct urb *urb,
    struct usb_host_endpoint	*ep, struct usb_ctrlrequest *setup,VUSB_GET_URB *pServiceUrb){
    __put_user (IO_TYPE_SET_CONFIGURATION, &pServiceUrb->dataType);
    __put_user (priv->vdev->addr, &pServiceUrb->buffer[0]);
    __put_user (priv->vdev->bus, &pServiceUrb->buffer[1]);
    __put_user ((setup->wValue), &pServiceUrb->buffer[2]);
    __put_user (0, &pServiceUrb->buffer[3]);
    __put_user (4, &pServiceUrb->dataLength);
    seh_dbg_vhci_data("Sent SET_CONFIG priv %p Num %u, urb %p\n", priv, priv->seqnum, priv->urb);

    seh_dbg_vhci_rx("CONTROL  \n\tReqType %d,tRequest %d, tWValue %d \n\tWIndex %d, WLength %d,DescType %d, DATA %d", 
        setup->bRequestType,setup->bRequest, setup->wValue, setup->wIndex , 
        setup->wLength , (setup->wValue>>8), priv->urb->transfer_buffer_length);
    seh_dbg_vhci_data("Sent SET CFG PIPE Bus %d Addr %d, EP %d\n", priv->vdev->bus, priv->vdev->addr, ep->desc.bEndpointAddress);
    }	
static inline void fill_reset_pipe_to_svc(struct vhci_priv *priv, struct urb *urb,
    struct usb_host_endpoint	*ep, struct usb_ctrlrequest *setup,VUSB_GET_URB *pServiceUrb){
    __put_user (IO_TYPE_RESET_PIPE, &pServiceUrb->dataType);
    __put_user (priv->vdev->addr, &pServiceUrb->buffer[0]);
    __put_user (priv->vdev->bus, &pServiceUrb->buffer[1]);
    __put_user ((setup->wIndex), &pServiceUrb->buffer[2]);
    __put_user (3, &pServiceUrb->dataLength);
    seh_dbg_vhci_data("Sent RESET PIPE Bus %d Addr %d, EP %d\n", priv->vdev->bus, priv->vdev->addr, ep->desc.bEndpointAddress);
    }
static inline void fill_abort_pipe_to_svc(struct vhci_priv *priv, struct urb *urb,
    struct usb_host_endpoint	*ep, struct usb_ctrlrequest *setup,
    VUSB_GET_URB *pServiceUrb){
    __put_user (IO_TYPE_ABORT_PIPE, &pServiceUrb->dataType);
    __put_user (priv->vdev->addr, &pServiceUrb->buffer[0]);
    __put_user (priv->vdev->bus, &pServiceUrb->buffer[1]);
    __put_user (ep->desc.bEndpointAddress, &pServiceUrb->buffer[2]);
    __put_user (3, &pServiceUrb->dataLength);
    seh_dbg_vhci_data("Sent ABORT PIPE Bus %d Addr %d, EP %d\n", priv->vdev->bus, priv->vdev->addr, ep->desc.bEndpointAddress);
    }

static inline void fill_unlink_to_svc(struct vhci_priv *priv, VUSB_GET_URB *pServiceUrb){
    __put_user (IO_TYPE_UNLINK_URB, &pServiceUrb->dataType);
    __put_user (priv->unlink_seqnum, (uint32_t *)&pServiceUrb->buffer[0]);
    __put_user (sizeof (uint32_t), &pServiceUrb->dataLength);
    seh_dbg_vhci_data("Sent UNLINK\n");
    }
static inline void fill_claim_to_svc(struct vhci_priv *priv, VUSB_GET_URB *pServiceUrb){
    __put_user (IO_TYPE_CLAIMDEVICE, &pServiceUrb->dataType);
    __put_user ((char)priv->vdev->addr, &pServiceUrb->buffer[0]);
    __put_user ((char)priv->vdev->bus, &pServiceUrb->buffer[1]);
    __put_user (2, &pServiceUrb->dataLength);
    seh_dbg_vhci_data("Sent CLAIM priv %p addr %d bus %d\n", priv, priv->vdev->addr, priv->vdev->bus);
    }
static inline void fill_claim2_to_svc(struct vhci_priv *priv, VUSB_GET_URB *pServiceUrb){
#define NAME_SIZE 32	
    int32_t name_len = 0;
    __put_user (IO_TYPE_CLAIMDEVICE2, &pServiceUrb->dataType);
    __put_user ((char)priv->vdev->addr, &pServiceUrb->buffer[0]);
    __put_user ((char)priv->vdev->bus, &pServiceUrb->buffer[1]);

    name_len = sizeof (priv->vdev->username);
    name_len = (NAME_SIZE < name_len) ? NAME_SIZE:name_len;
    unused=copy_to_user (&pServiceUrb->buffer[2], priv->vdev->username, name_len);
    __put_user (2+name_len, &pServiceUrb->dataLength);

    seh_dbg_vhci_data("Sent CLAIM2 priv %p addr %d bus %d\n", priv, priv->vdev->addr, priv->vdev->bus);
    }
static inline void fill_claim3_to_svc(struct vhci_priv *priv, VUSB_GET_URB *pServiceUrb){
#define SHA1_KEY_SIZE 20
    int32_t name_len = 0;
    char    sha1key[SHA1_KEY_SIZE];
    __put_user (IO_TYPE_CLAIMDEVICE3, &pServiceUrb->dataType);
    __put_user ((char)priv->vdev->addr, &pServiceUrb->buffer[0]);
    __put_user ((char)priv->vdev->bus, &pServiceUrb->buffer[1]);

    name_len = sizeof (priv->vdev->username);
    name_len = (NAME_SIZE < name_len) ? NAME_SIZE:name_len;
    unused=copy_to_user (&pServiceUrb->buffer[2], priv->vdev->username, name_len);

    SHA1(priv->vdev->hndl_plus_prtkey, strlen(priv->vdev->hndl_plus_prtkey), sha1key);
    copy_to_user (pServiceUrb->buffer+2+name_len, sha1key, sizeof(sha1key));
    __put_user (2+name_len+sizeof(sha1key), &pServiceUrb->dataLength);

    seh_dbg_vhci_data("Sent CLAIM3 priv %p addr %d bus %d\n", priv, priv->vdev->addr, priv->vdev->bus);
    }
static inline void fill_ifstatus_to_svc(struct vhci_priv *priv, VUSB_GET_URB *pServiceUrb){
    __put_user (IO_TYPE_UNKNOWN, &pServiceUrb->dataType);
    __put_user ((char)priv->vdev->addr, &pServiceUrb->buffer[0]);
    __put_user ((char)priv->vdev->bus, &pServiceUrb->buffer[1]);
    __put_user (2, &pServiceUrb->dataLength);
    seh_dbg_vhci_data("Sent ISTATUS priv %p addr %d bus %d\n", priv, priv->vdev->addr, priv->vdev->bus);
    }
static inline void fill_vers2_to_svc(struct vhci_priv *priv, VUSB_GET_URB *pServiceUrb){
    __put_user(IO_TYPE_GET_VERSION2, &pServiceUrb->dataType);
    __put_user(0, &pServiceUrb->dataLength);
    }

//-----------------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------------
/*
 * vhci_get_priv_send_size
 * calculate size in bytes requered for sending one URb to service
 * 
 * NOTE as there are unions - we calculate always more data than real - but that's ok
 */ 
int32_t vhci_get_priv_send_size(struct vhci_priv *priv){
    size_t size = sizeof (VUSB_GET_URB);
    
    if (is_urb(priv)) {
        
        if (priv->urb->transfer_buffer_length && ! usb_pipein(priv->urb->pipe))
            size += priv->urb->transfer_buffer_length;
        
        if (usb_pipeisoc(priv->urb->pipe)) {
            // size += sizeof(struct IsoFraHdr);
            if (priv->urb->transfer_buffer_length > 0)
                size += priv->urb->number_of_packets * sizeof(struct IsoPa);
            }
        seh_dbg_vhci_data("%s %s urb bufsize:%d iso:%d frags:%d HdrSize:%ld\n",__func__,
            usb_pipein(priv->urb->pipe) ? "IN" : "OUT",
            priv->urb->transfer_buffer_length,usb_pipeisoc(priv->urb->pipe),
            priv->urb->number_of_packets,size);
        }
    else {
        if (is_claim(priv))
            size += NAME_SIZE;
        else if (is_claim3(priv)) {
            size += NAME_SIZE;
            size += SHA1_KEY_SIZE;
            }
        size += 8;
        }
    return size;
    }

#if 1
//--------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------

void dumpUrb(const struct urb *urb)
    {
    int i;
    
    if (! urb || ! urb->dev) {
        seh_dbg_vhci_data("%s(%p) ERROR zero urb or dev?!\n",__func__,urb);
        return;
        }
    
    seh_dbg_vhci_data("%s(%p) dev:%p %s %s...\n",__func__,urb,urb->dev,
        usb_pipein(urb->pipe) ? "IN" : "OUT",
        usb_pipebulk(urb->pipe) ? "Bulk" :
        usb_pipeisoc(urb->pipe) ? "IsoC" :
        usb_pipecontrol(urb->pipe) ? "Ctrl" :
        usb_pipeint(urb->pipe)  ? "Int" : "UNKNOWN");

    seh_dbg_vhci_data("pipe                   %08X\n", urb->pipe);
    seh_dbg_vhci_data("stream_id              %08X\n", urb->stream_id);
    seh_dbg_vhci_data("status                 %08X\n", urb->status);
    seh_dbg_vhci_data("transfer_flags         %08X\n", urb->transfer_flags);
    seh_dbg_vhci_data("transfer_buffer        %p\n"  , urb->transfer_buffer);
    seh_dbg_vhci_data("transfer_dma           %08llX\n",urb-> transfer_dma);
    seh_dbg_vhci_data("sg                     %p\n " , urb->sg);
    seh_dbg_vhci_data("num_sgs                %08X\n", urb->num_sgs);
    seh_dbg_vhci_data("transfer_buffer_length %08X\n", urb->transfer_buffer_length);
    seh_dbg_vhci_data("actual_length          %08X\n", urb->actual_length);
    seh_dbg_vhci_data("setup_packet           %p\n"  , urb->setup_packet);
    seh_dbg_vhci_data("setup_dma              %08llx\n", urb->setup_dma);
    seh_dbg_vhci_data("start_frame            %08X\n", urb->start_frame);
    seh_dbg_vhci_data("number_of_packets      %08X\n", urb->number_of_packets);
    seh_dbg_vhci_data("interval               %08X\n", urb->interval);
    seh_dbg_vhci_data("error_count            %08X\n", urb->error_count);
	seh_dbg_vhci_data("context                %p\n"  , urb->context);
	seh_dbg_vhci_data("complete               %p\n"  , urb->complete);
    
    if (usb_pipeisoc(urb->pipe)) 
        for (i=0; i < urb->number_of_packets; i++) 
            seh_dbg_vhci_data("frame[%2d] off:%5d len:%d/%4d stat:%d (%#x)\n",i,
                urb->iso_frame_desc[i].offset,
                urb->iso_frame_desc[i].actual_length,
                urb->iso_frame_desc[i].length,
                urb->iso_frame_desc[i].status,
                urb->iso_frame_desc[i].status);
    }
#endif
//-----------------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------------
/*
 * vhci_send_to_service
 * send URB to service
 */
int32_t vhci_send_to_service (struct vhci_priv *priv,size_t priv_size, uint8_t __user *buf, size_t count)
    {
    VUSB_GET_URB __user    *pServiceUrb = (VUSB_GET_URB __user *) buf;  
    struct urb             *urb;
    struct usb_ctrlrequest *setup;

    urb = priv->urb;

    if (unlikely(!access_ok_seh(VERIFY_WRITE, (void __user *)pServiceUrb, count))){
        pr_err("Access write fail\n");
        return 0;
        }

    __put_user (priv_size,          &pServiceUrb->size);
    __put_user (0,                  &pServiceUrb->descriptorType); 
    __put_user (priv->vdev->bus,    &pServiceUrb->busno);
    __put_user (priv->vdev->addr,   &pServiceUrb->addr);
    __put_user (priv->vdev->ip,     &pServiceUrb->ipl);
    __put_user (priv->seqnum,       &pServiceUrb->pIoRequestHandle);
    __put_user (priv->vdev->useSSL, &pServiceUrb->useSSL);
    __put_user (0,                  &pServiceUrb->data.header.maxLength);
    __put_user (0,                  &pServiceUrb->dataCntr);

    if (is_urb(priv)) {
        struct usb_host_endpoint *ep;
        struct usb_device		 *dev;

        dev = urb->dev;
        if (unlikely(!dev)){
            pr_err("no dev\n");
            return 0;
            }
        // dumpUrb(urb);
        // ep = (usb_pipein(urb->pipe) ? dev->ep_in : dev->ep_out)[usb_pipeendpoint(urb->pipe)];

        ep = usb_pipe_endpoint(dev,urb->pipe);
        
        if (!ep){
            pr_err("%s no ep\n",__func__);
            return 0;
            }
        setup = (struct usb_ctrlrequest *) urb->setup_packet;

        /*check for SET configuration*/
        if (usb_endpoint_type(&ep->desc) ==  USB_ENDPOINT_XFER_CONTROL && setup) {

            if (setup->bRequestType == 0 && setup->bRequest == USB_REQ_SET_CONFIGURATION){
                /*Set configuration request*/
                fill_set_cfg_to_svc (priv, urb, ep, setup, pServiceUrb);
                }
            else if (setup->bRequest == USB_REQ_CLEAR_FEATURE 
                && setup->bRequestType == USB_RECIP_ENDPOINT 
                && setup->wValue == USB_ENDPOINT_HALT){
                /*Reset Pipe request*/
                fill_reset_pipe_to_svc (priv, urb, ep, setup, pServiceUrb);
                }
            else if (setup->bRequest == USB_REQ_SET_FEATURE 
                && setup->bRequestType == USB_RECIP_ENDPOINT 
                && setup->wValue == USB_ENDPOINT_HALT){
                /*Abort Pipe request*/
                fill_abort_pipe_to_svc (priv, urb, ep, setup, pServiceUrb);
                }
            else {
                if (fill_urb_to_svc (priv, urb, ep, setup, pServiceUrb) == -1)
                    return 0;
                }
            }
        else {
            
            if (fill_urb_to_svc (priv, urb, ep, setup, pServiceUrb) == -1)
                return 0;
            }
        }
    else if (is_unlink(priv))
        fill_unlink_to_svc (priv, pServiceUrb);
    else if (is_claim(priv))
        fill_claim2_to_svc (priv, pServiceUrb);
    else if (is_claim3(priv))
        fill_claim3_to_svc(priv, pServiceUrb);
    else if (is_ifstatus(priv))
        fill_ifstatus_to_svc (priv, pServiceUrb);
    else if (is_vers2(priv))
        fill_vers2_to_svc(priv, pServiceUrb);

    return priv_size;
    }


//-----------------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------------

bool get_indexes_from_buf (const void __user *buf, size_t count, __u32 *bus, __u32 *addr, __u32 *ip, __u32 *seqnum){
    VUSB_SET_URB service_urb;

    if (unlikely(!access_ok_seh(VERIFY_READ, (void __user *)buf, sizeof (VUSB_SET_URB)))){
        pr_err("Cannot read user data count %zu\n", sizeof (VUSB_SET_URB));
        return false;
        }

    if (unlikely(unused=__copy_from_user(&service_urb, (void __user *)buf, sizeof (VUSB_SET_URB)))){
        pr_err("Copy from user fail\n");
        return false;
        }

    if (unlikely(service_urb.size != count)){
        pr_err ("Bad incomming URB, data size %u vs %zu\n", service_urb.size, count);
        return false;
        }
    *bus = service_urb.busno;
    *addr = service_urb.addr;
    *ip = service_urb.ipl;
    *seqnum = service_urb.pIoRequestHandle;

    return true;
    }


//-----------------------------------------------------------------------------------------
//
//  Allowed transfer_flags  | value      | control | interrupt | bulk     | isochronous
// -------------------------+------------+---------+-----------+----------+-------------
//  URB_SHORT_NOT_OK        | 0x00000001 | only in | only in   | only in  | no
//  URB_ISO_ASAP            | 0x00000002 | no      | no        | no       | yes
//  URB_NO_TRANSFER_DMA_MAP | 0x00000004 | yes     | yes       | yes      | yes
//  URB_NO_FSBR             | 0x00000020 | yes     | no        | no       | no
//  URB_ZERO_PACKET         | 0x00000040 | no      | no        | only out | no
//  URB_NO_INTERRUPT        | 0x00000080 | yes     | yes       | yes      | yes
//  URB_FREE_BUFFER         | 0x00000100 | yes     | yes       | yes      | yes
//  URB_DIR_MASK            | 0x00000200 | yes     | yes       | yes      | yes
//-----------------------------------------------------------------------------------------

static bool isoFromService(const char __user *u, int datLen, struct urb *urb)
    {
    struct IsoPa     isofirst, *isoip;
    // struct IsoFraHdr isohdr;
    const  char __user *end = u + datLen; // boarder test only
    const  UCHAR *dump;
    int i,miss,fralen;
    int frags = urb->number_of_packets;

#if 0
    unused = __copy_from_user (&isohdr, u, sizeof(struct IsoFraHdr));
    u      += sizeof(struct IsoFraHdr);
    datLen -= sizeof(struct IsoFraHdr);
    
    seh_dbg_vhci_rx("%s isoHdr %#x %#x %#x %#x\n", __func__,
        isohdr.start,
        isohdr.number,
        isohdr.interv,
        isohdr.errors);
#endif
    
    if (frags < 1 || frags > 256) {
        printk("%s error iso Header %d packets?!\n",__func__,frags);
        return false;
        }

    fralen = sizeof(struct IsoPa) * frags;
    
    if (datLen < fralen || datLen > (urb->transfer_buffer_length + fralen)) {
        printk("%s error iso odd datlen:%d fraLen:%d bufLen:%d?!\n",__func__,datLen,
            fralen,urb->transfer_buffer_length);
        return false;
        }
                         
    isoip = (frags == 1) ? &isofirst : (struct IsoPa*) kmalloc(fralen, GFP_KERNEL);

    seh_dbg_vhci_rx("%s alloc(%d):%p (stack:%p)\n",__func__,fralen,isoip,&isofirst);

    unused = __copy_from_user (isoip, u, fralen);
    u      += fralen;
    datLen -= fralen;
    
    seh_dbg_vhci_rx("%s netto/buffer len:%d/%d flags:%#x frags:%d/%d\n", __func__,
        datLen,urb->transfer_buffer_length,urb->transfer_flags,
        frags,urb->number_of_packets);

    // not good: urb->transfer_flags |= URB_ISO_ASAP |  URB_NO_TRANSFER_DMA_MAP;
    urb->transfer_flags = URB_ISO_ASAP; // enforce non DMA and ASAP
    
    for (i = 0; i < frags ; i++) {
        
        // urb->iso_frame_desc[i].offset        = len; 
        urb->iso_frame_desc[i].actual_length = isoip[i].length;
        urb->iso_frame_desc[i].status        = isoip[i].status;
        
        // ! need to reFragement the arrived condense fragmented data
        
        miss = __copy_from_user (urb->transfer_buffer + urb->iso_frame_desc[i].offset, u, isoip[i].length);
        u += isoip[i].length;
        
        if (u > end) {
            seh_dbg_vhci_data("%s ERROR skip odd length fra[%2d] off:%d/%d +len:%d/%d > %d/%d stat:%#x\n",__func__,i,
                urb->iso_frame_desc[i].offset,isoip[i].offs,
                urb->iso_frame_desc[i].actual_length,
                urb->iso_frame_desc[i].length,
                urb->transfer_buffer_length, datLen,
                urb->iso_frame_desc[i].status);
            break;
            }
        
        seh_dbg_vhci_data("%s frame[%2d] off:%5d/%5d fill:%d/%d miss:%d stat:%d %p/%p\n",__func__,i,
            urb->iso_frame_desc[i].offset, isoip[i].offs,
            isoip[i].length,
            urb->iso_frame_desc[i].length, miss,
            urb->iso_frame_desc[i].status,u,end);
#if 1
        if (isoip[i].length > 3) {
            dump = (UCHAR*) urb->transfer_buffer;
            seh_dbg_vhci_rx("%s %02x %02x %02x %02x ... %02x %02x %02x %02x\n", __func__,
                dump[0], dump[1], dump[2], dump[3],
                dump[isoip[i].length-4], dump[isoip[i].length-3],
                dump[isoip[i].length-2], dump[isoip[i].length-1]);
            }
#endif
        };

    if (isoip && isoip != &isofirst) {
        seh_dbg_vhci_data("%s kfree:%p\n",__func__,isoip);
        kfree(isoip);
        }

    return true;
    }

//-----------------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------------
/*
 * Get data from Serivce
 */
int32_t vhci_get_from_service (struct vhci_priv *priv, const void __user *buf, size_t count, struct urb *urb)
    {
    VUSB_SET_URB __user *pServiceUrb = (VUSB_SET_URB __user *)buf; 
    int32_t error;
    int32_t status;   
    int32_t datLen;
    int     miss;

    seh_dbg_vhci_rx("%s buf:%p frags:%d size:%zu urb:%p ...\n", __func__,buf, urb->number_of_packets, count, urb);

    if (unlikely(!access_ok_seh(VERIFY_READ, (void __user *)buf, count))){
        pr_err("Cannot read user data count %zu\n", count);
        return 0;
        }

    __get_user(error,  &pServiceUrb->error);
    __get_user(status, &pServiceUrb->status);
    __get_user(datLen, &pServiceUrb->dataLength);

    if (unlikely(status == StatSysErr || status == StatAbortPipe)){
        seh_dbg_vhci_rx("ERROR: error %u, status %u\n", error, status);
        if (urb->status == 71)/*no idea why*/
            urb->status = -EPROTO;
        else
            urb->status = -error;
        } else
        urb->status = status;

    if (unlikely(error != 0 || status != 0)){
        seh_dbg_vhci_rx("ERROR: error %u, status %u\n", error, status);
        }
    
    if (usb_pipein(urb->pipe) && datLen) {
        seh_dbg_vhci_rx("%s expLen:%u dataLen:%d segs:%d\n", __func__,
            urb->transfer_buffer_length, datLen,urb->num_sgs);
        
        if (!urb->num_sgs) {
            if (urb->transfer_buffer && pServiceUrb->data) {
                if (usb_pipeisoc(urb->pipe)) {
                    __get_user(urb->number_of_packets,&pServiceUrb->dataCntr);
                    if (! isoFromService(pServiceUrb->data, datLen, urb)) // including buffer_length check
                        return -EPROTO;
                    }
                else {
                    if (urb->transfer_buffer_length >= datLen) {
                        miss = __copy_from_user (urb->transfer_buffer, pServiceUrb->data, min(datLen, (int) urb->transfer_buffer_length));
                        seh_dbg_vhci_rx("%s cpMiss:%d/%d (max:%d)\n", __func__,datLen,miss,urb->transfer_buffer_length);
                        }
                    else { 
                        pr_err("ERROR: non iso data len %d, buffer len %u status %d\n", urb->transfer_buffer_length, datLen, urb->status);
                        return -EPROTO;
                        }
                    }
                }
            }
        else {  // ?? obsolette ??  isochrone is missing at all
                
            struct sg_mapping_iter *miter = &priv->miter;
            u32 this_sg;
            bool next_sg;
            u32 trans         = 0;
            u32 all_len       = datLen;
            void __user *ubuf = pServiceUrb->data;
            u32 flags         = SG_MITER_TO_SG;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 35)            
            sg_miter_start(miter, urb->sg->sg, urb->num_sgs, flags);
#else
            sg_miter_start(miter, urb->sg, urb->num_sgs, flags);
#endif            
            next_sg = sg_miter_next(miter);
            if (next_sg == false) {
                WARN_ON_ONCE(1);
                pr_err ("urb->num_sgs %d\n", urb->num_sgs);
                return 0;
                }
            do {
                this_sg = min_t(u32, all_len, miter->length);
                miter->consumed = this_sg;
                trans += this_sg;
                unused = __copy_from_user (miter->addr, ubuf, this_sg);
                all_len -= this_sg;
                if (!all_len){
                    break;
                    }
                next_sg = sg_miter_next(miter);
                if (next_sg == false) {
                    WARN_ON_ONCE(1);
                    pr_err ("urb->num_sgs %d\n", urb->num_sgs);
                    return 0;
                    }

                ubuf += this_sg;
                } while (1);
            sg_miter_stop(miter);
            }
        }
    
    if (usb_pipein(urb->pipe) && ! usb_pipeisoc(urb->pipe)){
        urb->actual_length = datLen;
        }
    else {
        urb->actual_length = urb->transfer_buffer_length;
        }
    
    seh_dbg_vhci_rx("%s URB Len %d, status %d done\n", __func__,urb->actual_length, urb->status);
    return 0;
    }

//-----------------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------------
int32_t vhci_get_cmd_reply_from_service (const void __user *buf, size_t count)
    {
    int32_t error;
    int32_t status;
    VUSB_SET_URB __user *pServiceUrb = (VUSB_SET_URB *)buf; 
      
    seh_dbg_vhci_rx("%s buf %p , size %zu\n",__func__, buf, count);

    if (unlikely(!access_ok_seh(VERIFY_READ, (void __user *)buf, count))){
        pr_err("Cannot read user data count %zu\n", count);
        return 0;
        }

    __get_user(error, &pServiceUrb->error);
    __get_user(status, &pServiceUrb->status);
    seh_dbg_vhci_rx("Status %x error %x\n", status, error);
    return status;
    }

//-----------------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------------
int32_t vhci_get_vers2_reply_from_service (const void __user *buf, size_t count,
                                           uint32_t *nw_hndl, uint16_t *comp_id)
    {
    int32_t error;
    int32_t status;
    int32_t datLen;
    VUSB_SET_URB __user *pServiceUrb = (VUSB_SET_URB *)buf;

    seh_dbg_vhci_rx("%s buf %p , size %zu\n",__func__, buf, count);

    if (unlikely(!access_ok_seh(VERIFY_READ, (void __user *)buf, count))){
        pr_err("Cannot read user data count %zu\n", count);
        return 0;
        }

    __get_user(error, &pServiceUrb->error);
    __get_user(status, &pServiceUrb->status);

    if (status) {
        seh_dbg_vhci_rx("Status %x error %x\n", status, error);
        return status;
        }

    __get_user(datLen, &pServiceUrb->dataLength);
    __get_user(*nw_hndl, &pServiceUrb->dataCntr);

    if (datLen >= sizeof(uint16_t)) {
         __copy_from_user (comp_id, (void __user *)pServiceUrb->data, sizeof(uint16_t));
         *comp_id = ntohs(*comp_id);
        }

    seh_dbg_vhci_rx("Status %x error %x nw_hndl %#x compat %d\n", status, error, *nw_hndl, *comp_id);
    return status;
    }

//-----------------------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------------------

int32_t get_size_from_buf (uint8_t __user *buf, uint32_t *hcd_index, size_t *size){
    VUSB_GET_URB *pServiceUrb = (VUSB_GET_URB *)buf;

    if (unlikely(!access_ok_seh(VERIFY_READ, (void __user *)&pServiceUrb->size, sizeof (pServiceUrb->size)))){
        pr_err("Access read fail\n");
        return -1;
        }
    if (unlikely(!access_ok_seh(VERIFY_READ, (void __user *)&pServiceUrb->descriptorType, sizeof (pServiceUrb->descriptorType)))){
        pr_err("Access read fail\n");
        return -1;
        }
    __get_user(*size, &pServiceUrb->size);
    __get_user(*hcd_index, &pServiceUrb->descriptorType);
    return 0;
    }



