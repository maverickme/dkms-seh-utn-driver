/*
 * Copyright (C) 2013-2014 SEH Computertechnik GmbH
 *
 * This code is based on drivers/staging/usbip/vhci.h
 *
 * Copyright (C) 2003-2008 Takahiro Hirofuchi
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __SEH_VHCI_H
#define __SEH_VHCI_H

#include <linux/version.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/usb.h>
#include <linux/scatterlist.h>

#ifdef HAVE_HCD_H
#include <linux/usb/hcd.h>
#else
#ifdef HAVE_HCD_H_IN_SRC
#include <linux/autoconf.h>
#include <drivers/usb/core/hcd.h>
#else
#include "hcd.h"
#warning "Using custom headers !!!"
#endif
#endif

#include <linux/wait.h>

#define VDEV_EVENT_ERROR_MALLOC (1<<0)
#define VDEV_EVENT_MYUTN_DOWN	(1<<1)
#define VDEV_EVENT_SVC_DOWN		(1<<2)
#define VDEV_EVENT_DEV_DOWN		(1<<3)
#define VDEV_EVENT_FREE_DEV		(1<<4)
#define VDEV_EVENT_CLAIM_DEV	(1<<5)
#define VDEV_EVENT_NOTHING_TO_SEND		(1<<6)

typedef uint32_t rhport_id_t;

#define HCD_INDEX_SHIFT 16
#define HCD_PORT_MASK (0xffff)


#ifndef USB_SSP_CAP_TYPE
#define USB_SSP_CAP_TYPE 0xa
#endif

#ifndef HUB_CHAR_INDV_PORT_LPSM
#define HUB_CHAR_INDV_PORT_LPSM 0x0001 
#endif

#ifndef HUB_CHAR_INDV_PORT_OCPM
#define HUB_CHAR_INDV_PORT_OCPM 0x0008
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(5,0,0)
	#define access_ok_seh(A,B,C) access_ok(B,C)
#elif RHEL_MAJOR == 8 && RHEL_MINOR > 0
	#define access_ok_seh(A,B,C) access_ok(B,C)
#else
	#define access_ok_seh(A,B,C) access_ok(A,B,C)
#endif

enum seh_status {
        /* vdev does not connect a remote device. */
        VDEV_ST_NULL,
        /* vdev is used, but the USB address is not assigned yet */
        VDEV_ST_NOTASSIGNED,
        VDEV_ST_USED,
		VDEV_ST_STOP_PENDING,
		VDEV_ST_STOPPED,
        VDEV_ST_ERROR
};

struct vhci_device {
	struct usb_device *udev;

	/*
	 * the 3 ids specifies a remote usb device uniquely instead
	 * of combination of busnum and devnum.
	 */
	uint32_t bus;
	uint32_t addr;
	uint32_t ip;
	
	/* speed of a remote device */
	enum usb_device_speed speed;

	/* vhci root-hub port to which this device is attached */
	rhport_id_t rh_port_id;

	enum seh_status device_status;

    uint32_t buffersHackBoosted;

    uint32_t useSSL;
    char username[32];
    char hndl_plus_prtkey[72];

	/* vhci_priv is linked to one of them. */
    spinlock_t dev_lock;
    
	struct list_head priv_tx;
	struct list_head priv_rx;
};

typedef enum {
    URB,
    UNLINK_URB,
    CLAIM,
    IF_STAT,
    VERS2,
    CLAIM3,
} priv_type;

/* urb->hcpriv, use container_of() */
struct vhci_priv {
	uint32_t seqnum;
	struct list_head list;
	struct vhci_device *vdev;	
	struct urb *urb;
	uint32_t unlink_seqnum;
	priv_type type;
    struct sg_mapping_iter	miter;
};

#define is_urb(priv) 		(priv->type == URB)
#define is_unlink(priv) 	(priv->type == UNLINK_URB)
#define is_claim(priv) 		(priv->type == CLAIM)
#define is_ifstatus(priv) 	(priv->type == IF_STAT)
#define is_vers2(priv)      (priv->type == VERS2)
#define is_claim3(priv) 	(priv->type == CLAIM3)

static inline rhport_id_t rhport_get(uint32_t hcd_index, uint32_t hcd_port_index){return (hcd_index<<HCD_INDEX_SHIFT)|hcd_port_index ;}
static inline uint32_t hcd_index(rhport_id_t port){return (port>>HCD_INDEX_SHIFT);}
static inline uint32_t hcd_port_index (rhport_id_t port){ return (port&HCD_PORT_MASK); }

/*
 * USB_MAXCHILDREN is statically defined to 16 in usb.h.  Its maximum value
 * would be 31 because the event_bits[1] of struct usb_hub is defined as
 * unsigned long in hub.h
 */
#define VHCI_NPORTS 15

#define VHCI_PLATFORM_DEVICES 6                      /* set ONLY THIS VALUE to n to get n x VHCI_NPORTS USB 2.0 AND n x VHCI_NPORTS 3.0 devices */
#define VHCI_CONTROLLERS (VHCI_PLATFORM_DEVICES * 2)
#define VHCI_FIRST_USB3  (VHCI_CONTROLLERS      / 2) /*half usb2 and half usb3, Yes it is VHCI_PLATFORM_DEVICES */

#define IS_USB3(CONTR) (CONTR->id >= VHCI_FIRST_USB3)

/* for usb_bus.hcpriv */
struct vhci_hcd {
    /*id of this HCD*/
	uint32_t id;
    
    spinlock_t hcd_lock;
	uint32_t port_status[VHCI_NPORTS];

	uint32_t resuming:1;
	unsigned long re_timeout;

	/*
	 * NOTE:
	 * wIndex shows the port number and begins from 1.
	 * But, the index of this array begins from 0.
	 */
	struct vhci_device vdev[VHCI_NPORTS];
        
	/*flags that indicate that there is something to be sent/receved
	 from any queue from andy device*/
	uint32_t device_index_last_used_urb;
};

extern struct vhci_hcd *controllers[VHCI_CONTROLLERS];
extern struct seh_char_dev *vhci_char_dev;
extern atomic_t glob_urb_seqnum;

extern void dumpUrb(const struct urb *urb);


/* vhci_hcd.c */
void rh_port_connect(struct vhci_hcd *hcd_controller, int32_t rhport, enum usb_device_speed speed);
void rh_port_disconnect(struct vhci_hcd *hcd_controller, int32_t rhport);

static inline struct vhci_device *port_to_vdev(struct vhci_hcd *vhcd, uint32_t port_index)
{
	return &vhcd->vdev[port_index];
}

static inline struct vhci_hcd *hcd_to_vhci(struct usb_hcd *hcd)
{
	return (struct vhci_hcd *) (hcd->hcd_priv);
}

static inline struct usb_hcd *vhci_to_hcd(struct vhci_hcd *vhci)
{
	return container_of((void *) vhci, struct usb_hcd, hcd_priv);
}
static inline struct vhci_hcd *get_hcd_from_vhci_device(struct vhci_device *vdev){return controllers[hcd_index(vdev->rh_port_id)];}

inline struct vhci_priv *kzalloc_priv(void);
void free_priv(struct vhci_priv *priv);
void seh_vusb_add_cmd_to_svc (int hcd_index, struct vhci_device *vdev, uint32_t cmd);

#ifdef DEBUG
#ifdef DEBUG_STATISTIC
extern uint64_t global_sema_overhead_ns ;	
extern uint64_t global_sema_misses ;	
/*macros for locking the controller */
#define MEASURE_LOCK(LOCK, FUNC) {struct timespec now, later; \
                                                    now = current_kernel_time(); \
                                                    if (spin_is_locked(LOCK)) \
                                                        global_sema_misses = atomic_inc_return((atomic_t *)&global_sema_misses); \
                                                    FUNC;\
                                                    later = current_kernel_time();\
                                                    global_sema_overhead_ns += (later.tv_sec - now.tv_sec)*1000 + (later.tv_nsec - now.tv_nsec)/1000000; \
                                                 }

#define HCD_LOCK_IRQSAVE(CONTROLLER,flags)		    {MEASURE_LOCK(&CONTROLLER->hcd_lock, spin_lock_irqsave(&CONTROLLER->hcd_lock, flags) )}
#define HCD_UNLOCK_IRQRESTORE(CONTROLLER,flags)	    {spin_unlock_irqrestore (&CONTROLLER->hcd_lock, flags);}

#define DEV_LOCK_IRQSAVE(DEV,flags) 		        {MEASURE_LOCK(&DEV->dev_lock, spin_lock_irqsave(&DEV->dev_lock, flags) )}
#define DEV_UNLOCK_IRQRESTORE(DEV,flags)	        {spin_unlock_irqrestore (&DEV->dev_lock, flags);}

#define DEV_LOCK(DEV) 		                        {MEASURE_LOCK(&DEV->dev_lock, spin_lock(&DEV->dev_lock))}
#define DEV_UNLOCK(DEV)	                            {spin_unlock(&DEV->dev_lock);}

#else

#define HCD_LOCK_IRQSAVE(CONTROLLER,flags) 		    {if seh_dbg_flag_vhci_sema printk ("%s %d, CON GET  IRQ %p\n", __FUNCTION__, __LINE__, CONTROLLER);spin_lock_irqsave (&CONTROLLER->hcd_lock, flags);}
#define HCD_UNLOCK_IRQRESTORE(CONTROLLER,flags)	    {if seh_dbg_flag_vhci_sema printk ("%s %d, CON GIVE IRQ %p\n", __FUNCTION__, __LINE__, CONTROLLER);spin_unlock_irqrestore (&CONTROLLER->hcd_lock, flags);}

#define DEV_LOCK_IRQSAVE(DEV,flags) 		        {if seh_dbg_flag_vhci_sema printk ("%s %d, DEV GET  IRQ %p\n", __FUNCTION__, __LINE__, DEV);spin_lock_irqsave (&DEV->dev_lock, flags);}
#define DEV_UNLOCK_IRQRESTORE(DEV,flags)	        {if seh_dbg_flag_vhci_sema printk ("%s %d, DEV GIVE IRQ %p\n", __FUNCTION__, __LINE__, DEV);spin_unlock_irqrestore (&DEV->dev_lock, flags);}

#define DEV_LOCK(DEV) 		                        {if seh_dbg_flag_vhci_sema printk ("%s %d, DEV GET  %p\n", __FUNCTION__, __LINE__, DEV);spin_lock  (&DEV->dev_lock);}
#define DEV_UNLOCK(DEV)	                            {if seh_dbg_flag_vhci_sema printk ("%s %d, DEV GIVE %p\n", __FUNCTION__, __LINE__, DEV);spin_unlock(&DEV->dev_lock);}

#endif

#else
#define HCD_LOCK_IRQSAVE(CONTROLLER,flags) 		    spin_lock_irqsave (&CONTROLLER->hcd_lock, flags)
#define HCD_UNLOCK_IRQRESTORE(CONTROLLER,flags)	    spin_unlock_irqrestore (&CONTROLLER->hcd_lock, flags)

#define DEV_LOCK_IRQSAVE(DEV,flags) 		    spin_lock_irqsave (&DEV->dev_lock, flags)
#define DEV_UNLOCK_IRQRESTORE(DEV,flags)	    spin_unlock_irqrestore (&DEV->dev_lock, flags)

#define DEV_LOCK(DEV) 		                    spin_lock  (&DEV->dev_lock)
#define DEV_UNLOCK(DEV)	                        spin_unlock(&DEV->dev_lock)

#endif

#endif /* __SEH_VHCI_H */
