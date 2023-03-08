/*
 * Copyright (C) 2013-2014 SEH Computertechnik GmbH
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __SEH_XMIT_H
#define __SEH_XMIT_H

int32_t vhci_get_from_hcd_tx_queue(uint32_t hcd_index, struct vhci_priv **priv_out, bool *is_more);
int32_t vhci_get_priv_send_size(struct vhci_priv *priv); 
int32_t vhci_send_to_service (struct vhci_priv *priv, size_t priv_size, uint8_t __user *buf, uint32_t size);
int32_t usbip_recv_iso (const void __user *buf, size_t count, struct urb *urb);
void vhci_device_unlink_all(struct vhci_device *vdev);
struct vhci_priv *vhci_get_urb_from_rx_queue(uint32_t bus, uint32_t addr, uint32_t ip, uint32_t seqnum);
struct vhci_priv *get_priv_from_rx_by_num (struct vhci_device *vdev, uint32_t seqnum);
#endif
