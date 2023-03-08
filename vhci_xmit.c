/*
 * Copyright (C) 2013-2014 SEH Computertechnik GmbH
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/kthread.h>
#include <linux/slab.h>

#include "vhci_common.h"
#include "vhci_to_service.h"
#include "vhci.h"
#include "vhci_xmit.h"
#include "vhci_event.h"

/*
 * vhci_get_from_hcd_tx_queue
 * find one URB from tx list
 */
int32_t vhci_get_from_hcd_tx_queue(uint32_t hcd_index, struct vhci_priv **priv_out, bool *is_more){
    unsigned long flags;
    struct vhci_hcd *hcd_controller;
	uint32_t irhPort, start_port;
	struct vhci_device *vdev;
	struct vhci_priv *priv = NULL;

    *is_more = false;
    *priv_out = NULL;
    hcd_controller = controllers[hcd_index];
    irhPort = start_port = ( hcd_controller->device_index_last_used_urb+1)% VHCI_NPORTS;
    /*find first URB on next port
     * at same time check if on other ports there is some URB to send*/
	do {
		vdev = port_to_vdev (hcd_controller, irhPort);
        if (vdev->device_status != VDEV_ST_NULL) {
            seh_dbg_vhci_rx("Check hcd %d port %d %p empty?%d\n", hcd_index, irhPort, vdev, list_empty(&vdev->priv_tx));
            DEV_LOCK_IRQSAVE(vdev, flags);
            /*check if tx list is empty*/
            if (!list_empty(&vdev->priv_tx)){
                /*we took the priv alreay, but
                 * on same device there is no more peivs
                 * so we continue loop searching new one*/
                if (*priv_out != NULL){
                    *is_more = true;
                    DEV_UNLOCK_IRQRESTORE(vdev, flags);
                    return hcd_controller->device_index_last_used_urb;
                }
                hcd_controller->device_index_last_used_urb = irhPort;
                /*move priv out from list*/
                priv = list_first_entry(&vdev->priv_tx, struct vhci_priv, list);
                list_del_init(&priv->list);
                *priv_out = priv;
                seh_dbg_vhci_rx("Got hcd %d vdev %p priv %p port %d number %u urb %p\n", hcd_controller->id, vdev, priv, irhPort, priv->seqnum, priv->urb);
                /*If there is one more URB than we can break*/
                if (!list_empty(&vdev->priv_tx)){
                    *is_more = true;
                    DEV_UNLOCK_IRQRESTORE(vdev, flags);	
                    return hcd_controller->device_index_last_used_urb;
                }
            }
            DEV_UNLOCK_IRQRESTORE(vdev, flags);
        }
        irhPort = (irhPort+1) % VHCI_NPORTS;
	}while (irhPort != start_port);
    if (*priv_out)
        return hcd_controller->device_index_last_used_urb;
    return -1;
}

struct vhci_priv *get_priv_from_rx_by_num (struct vhci_device *vdev, uint32_t seqnum){
    struct vhci_priv *priv;
    unsigned long flags;
    DEV_LOCK_IRQSAVE(vdev, flags);   
    list_for_each_entry(priv, &vdev->priv_rx, list){
        if (priv->seqnum == seqnum){
            list_del(&priv->list);
            DEV_UNLOCK_IRQRESTORE(vdev, flags);           
            return priv;
        }
    }
    DEV_UNLOCK_IRQRESTORE(vdev, flags);
    return NULL;
}
/*
 * vhci_get_from_hcd_tx_queue
 * find one URB from tx list
 */
struct vhci_priv *vhci_get_urb_from_rx_queue(uint32_t bus, uint32_t addr, uint32_t ip, uint32_t seqnum){
   uint32_t hcd_index, rhport;
   struct vhci_priv *priv;
   struct vhci_device *vdev;
   
   for (hcd_index = 0; hcd_index < VHCI_CONTROLLERS; hcd_index++){
        struct vhci_hcd *hcd_controller = controllers[hcd_index];
        for (rhport = 0; rhport < VHCI_NPORTS; rhport++){
            vdev = port_to_vdev(hcd_controller, rhport);
            if (vdev->device_status != VDEV_ST_NULL &&
                vdev->bus == bus && vdev->addr == addr && vdev->ip == ip) {
                    priv = get_priv_from_rx_by_num(vdev, seqnum);
                    if (!priv){
                        //pr_err("Priv seq num %d not found\n", seqnum);
                        return NULL;
                    }
                    return priv;
            }
        }
    }
    //pr_err("Priv seq num %d not found\n", seqnum);
	return NULL;

}


int32_t seh_vusb_get_urb_data(struct seh_char_dev *cdev, uint8_t __user *buf)
{
  size_t count;
  uint32_t hcd_index;
  struct vhci_device *vdev; 
  unsigned long flags;   
  struct vhci_priv *priv = cdev->priv_to_send;
    
	/*priv_to_send is priv_lock*/
	if (unlikely(!priv)){
        pr_err("No priv to send\n");
        return -EFAULT;
    }
    if (unlikely(get_size_from_buf (buf, &hcd_index, &count))){
        pr_err("Bad data from service\n");
        return -EFAULT;
    }
    if (unlikely(cdev->priv_to_send_size > count)){
        pr_err("Buffer too small\n");
        return -EFAULT;
    }
    vdev = cdev->priv_to_send->vdev;
	cdev->priv_to_send = NULL;
    
    count = vhci_send_to_service (priv, cdev->priv_to_send_size, buf, count);
	if (likely(count)){
		if (unlikely(is_ifstatus(priv)))
        {
            if (vdev->device_status == VDEV_ST_STOP_PENDING){
                vdev->device_status = VDEV_ST_STOPPED;
            }
			free_priv(priv);
            return 0;
		}else{
#ifdef SEH_HACK_NO_IN_WAIT         
            if ( priv->urb && usb_pipebulk (priv->urb->pipe) && !usb_pipein(priv->urb->pipe)){
                
                struct usb_hcd *hcd = bus_to_hcd(vdev->udev->bus);
                HCD_LOCK_IRQSAVE(get_hcd_from_vhci_device(vdev), flags);
                usb_hcd_unlink_urb_from_ep(hcd, priv->urb);
                HCD_UNLOCK_IRQRESTORE(get_hcd_from_vhci_device(vdev), flags);
                priv->urb->status = 0; 
                priv->urb->actual_length = priv->urb->transfer_buffer_length;
                usb_hcd_giveback_urb(hcd, priv->urb, priv->urb->status);
                priv->urb = NULL;
                free_priv(priv);
                return 0;
            }
#endif   
            DEV_LOCK_IRQSAVE(vdev, flags);
            list_add_tail(&priv->list, &vdev->priv_rx);
            DEV_UNLOCK_IRQRESTORE(vdev, flags);
            seh_dbg_vhci_rx("Move in waiting reply queue priv %p Num %u, urb %p\n", priv, priv->seqnum, priv->urb);
            return 0;
		}
	}
    if (priv->urb && vdev->udev){
        struct usb_hcd *hcd = bus_to_hcd(vdev->udev->bus);
        HCD_LOCK_IRQSAVE(get_hcd_from_vhci_device(vdev), flags);
        usb_hcd_unlink_urb_from_ep(hcd, priv->urb);
        HCD_UNLOCK_IRQRESTORE(get_hcd_from_vhci_device(vdev), flags);
         
        usb_hcd_giveback_urb(hcd, priv->urb, priv->urb->status);
        pr_err("URB sent back to USB stack \n");
    }
    pr_err("Error Priv not sent to service, free priv\n");
    free_priv(priv);
    return -EFAULT;
}



void vhci_device_unlink_all(struct vhci_device *vdev)
{
    struct vhci_priv *priv;
    unsigned long flags;
    struct usb_hcd *hcd = NULL;

    if (vdev->udev)
        hcd = bus_to_hcd(vdev->udev->bus);

    DEV_LOCK_IRQSAVE(vdev, flags);
    while (!list_empty(&vdev->priv_rx)) {
        struct urb *urb;
        priv = list_first_entry(&vdev->priv_rx, struct vhci_priv, list);
        list_del(&priv->list);
        
        DEV_UNLOCK_IRQRESTORE(vdev, flags);
        urb = priv->urb;
        if (urb && hcd) {
            urb->status = -ENOENT;
            
            HCD_LOCK_IRQSAVE(hcd_to_vhci(hcd), flags);
            usb_hcd_unlink_urb_from_ep(bus_to_hcd(urb->dev->bus), urb);
            HCD_UNLOCK_IRQRESTORE(hcd_to_vhci(hcd), flags);
            usb_hcd_giveback_urb(bus_to_hcd(urb->dev->bus), urb,
                                 urb->status);

        }
        seh_dbg_vhci_rx("Free RX priv %p Num %u, urb %p\n", priv, priv->seqnum, priv->urb);
        free_priv(priv);
        DEV_LOCK_IRQSAVE(vdev, flags);
    }

    while (!list_empty(&vdev->priv_tx)) {
        struct urb *urb;
        priv = list_first_entry(&vdev->priv_tx, struct vhci_priv, list);
        list_del(&priv->list);
        DEV_UNLOCK_IRQRESTORE(vdev, flags);
        urb = priv->urb;
        if (urb && hcd) {
            urb->status = -ENOENT;

            HCD_LOCK_IRQSAVE(hcd_to_vhci(hcd), flags);
            usb_hcd_unlink_urb_from_ep(bus_to_hcd(urb->dev->bus), urb);
            HCD_UNLOCK_IRQRESTORE(hcd_to_vhci(hcd), flags);
            usb_hcd_giveback_urb(bus_to_hcd(urb->dev->bus), urb,
                                 urb->status);

        }
        seh_dbg_vhci_rx("Free TX priv %p Num %u, urb %p\n", priv, priv->seqnum, priv->urb);
        free_priv(priv);
        DEV_LOCK_IRQSAVE(vdev, flags);

    }
    DEV_UNLOCK_IRQRESTORE(vdev, flags);

}



