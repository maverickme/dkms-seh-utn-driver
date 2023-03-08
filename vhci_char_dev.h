/*
 * Copyright (C) 2013-2014 SEH Computertechnik GmbH
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __SEH_CHAR_DEV_H
#define __SEH_CHAR_DEV_H


#define SEH_CHAR_DEV_NAME	"seh_vhci_dev"

#ifdef __KERNEL__
#include <linux/cdev.h>


#define SEH_CHAR_PROC		"seh_vhci_dev"



struct seh_char_dev {
  volatile long unsigned int is_data_tx_bitmap;
  
  wait_queue_head_t txq;
  struct semaphore sem;
  struct cdev cdev;
  bool b_opened;
  
  uint32_t last_read_hcd;
  
  struct vhci_priv *priv_to_send;
  size_t priv_to_send_size;
};


int32_t seh_chardev_create(void);
int32_t seh_chardev_delete(void);

#endif

#define SEH_IOC_MAGIC 'k'
/* Please use a different 8-bit number in your code */
#define LINUX_IOCTL_VUSB_EJECT_HARDWARE 	(_IOWR(SEH_IOC_MAGIC, 0, uint64_t [5]))
#define LINUX_IOCTL_VUSB_PLUGIN_HARDWARE 	(_IOWR(SEH_IOC_MAGIC, 1, uint64_t [5]))
#define LINUX_IOCTL_VUSB_UNPLUG_HARDWARE	(_IOWR(SEH_IOC_MAGIC, 2, uint64_t [5]))
#define LINUX_IOCTL_VUSB_GET_INSTANCE_ID	(_IOWR(SEH_IOC_MAGIC, 3, uint64_t [5]))
#define LINUX_IOCTL_VUSB_GET_INTERFACE_STATUS (_IOWR(SEH_IOC_MAGIC, 4, uint64_t [5]))
#define LINUX_IOCTL_VUSB_HEARTBEAT			(_IOWR(SEH_IOC_MAGIC, 5, uint64_t [5]))
#define LINUX_IOCTL_VUSB_SET_HEARTBEAT_TIMEOUT (_IOWR(SEH_IOC_MAGIC, 6, uint64_t [5]))
#define LINUX_IOCTL_VUSB_GET_URB_DATA 		(_IOWR(SEH_IOC_MAGIC, 7, uint64_t [5]))
#define LINUX_IOCTL_VUSB_GET_VERSION 		(_IOWR(SEH_IOC_MAGIC, 8, uint64_t [5]))
#define SEH_VHCI_IOC_MAXNR 14

typedef struct __vusb_read_urb_size{
    uint32_t hcd_index;
    uint32_t size;
} vusb_read_urb_size;

typedef struct _vusb_device_addr {
    uint32_t   ipl;
	uint8_t   busno;       // USB Busnummer
	uint8_t   addr;        // USB Gerätenummer
	uint8_t   padding[2];
} vusb_device_addr ;


typedef struct __vusb_plugin_hardware{
    uint32_t size;
    vusb_device_addr addr;
    uint32_t bcdUSB;
    uint16_t speed;
    uint16_t useSSL;
    char     username[32];
    char     prtKey[65];
} vusb_plugin_hardware;


typedef struct __vusb_plugout_hardware{
    uint32_t size;
    vusb_device_addr addr;
} vusb_plugout_hardware;


typedef struct __vusb_eject_hardware{
    uint32_t size;
    vusb_device_addr addr;
} vusb_eject_hardware;


typedef struct __vusb_get_if_status{
    uint32_t size;
    vusb_device_addr addr;
} vusb_get_if_status;

typedef struct __vusb_get_instance_id{
    uint32_t size;
    vusb_device_addr addr;
} vusb_get_instance_id;



#endif
