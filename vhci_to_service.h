/*
 * Copyright (C) 2013-2014 SEH Computertechnik GmbH
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __SEH_TO_SERVICE_H
#define __SEH_TO_SERVICE_H
#include "vhci_char_dev.h"


int32_t seh_vusb_plug_hw (vusb_plugin_hardware *hw);
int32_t seh_vusb_unplug_hw (vusb_plugout_hardware *hw);
int32_t seh_vusb_eject_hw (vusb_plugout_hardware *hw);
int32_t seh_vusb_get_inst_id (vusb_get_instance_id *hw, uint32_t *Id);
int32_t seh_vusb_get_if_status(vusb_get_if_status *hw, uint32_t *Status);
int32_t seh_vusb_set_hb (void);
int32_t seh_vusb_hartbeat (void);
int32_t vhci_get_from_service (struct vhci_priv *priv, const void __user *buf, size_t count, struct urb *urb);
int32_t vhci_get_cmd_reply_from_service (const void __user *buf, size_t count);
int32_t vhci_get_vers2_reply_from_service (const void __user *buf, size_t count, uint32_t *nw_hndl, uint16_t *comp_id);
bool get_indexes_from_buf (const void __user *buf, size_t count, __u32 *bus, __u32 *addr, __u32 *ip, __u32 *seqnum);
int32_t get_size_from_buf (uint8_t __user *buf, uint32_t *hcd_index, size_t *size);

int32_t seh_vusb_get_urb_data(struct seh_char_dev *vdev, uint8_t __user *buf);

enum{          
    StatOk,                             //< status O.K.
    StatPending,
    StatAbortPipe,
    StatSysErr,                          //< errno set in error field
    StatDevRemoved,
    StatIntErr,                          //< Internal error, vusbErrorCode in error field
    StatCompressed                       //< compressed data
    };

#endif
