/*
 * Copyright (C) 2013-2014 SEH Computertechnik GmbH
 *
 * This code is based on drivers/staging/usbip/vhci_hcd.c
 *
 * Copyright (C) 2003-2008 Takahiro Hirofuchi
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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <asm/unaligned.h>

#include "vhci_common.h"
#include "vhci.h"
#include "vhci_event.h"
#include "vhci_char_dev.h"
#include "vhci_xmit.h"
// #include <linux/usb/ch9.h>

#ifndef HCD_HW_ACCESSIBLE
#define HCD_HW_ACCESSIBLE(hcd)	((hcd)->flags & (1U << HCD_FLAG_HW_ACCESSIBLE))
#endif

#define DRIVER_AUTHOR "SEH Computertechnik GmbH"
#define DRIVER_DESC "SEH 'Virtual' Host Controller (VHCI) Driver"

/*
 * TODO
 *	- update root hub emulation
 *	- move the emulation code to userland ?
 *		porting to other operating systems
 *		minimize kernel code
 *	- add suspend/resume code
 *	- clean up everything
 */

/* See usb gadget dummy hcd */

static int32_t vhci_hub_status(struct usb_hcd *hcd, char *buff);
static int32_t vhci_hub_control(struct usb_hcd *hcd, u16 typeReq, u16 wValue,
			    u16 wIndex, char *buff, u16 wLength);
static int32_t vhci_urb_enqueue(struct usb_hcd *hcd, struct urb *urb,
			    gfp_t mem_flags);
static int32_t vhci_urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int32_t status);
static int32_t vhci_start(struct usb_hcd *vhci_hcd);
static void vhci_stop(struct usb_hcd *hcd);
static int32_t vhci_get_frame_number(struct usb_hcd *hcd);

static const uint8_t driver_name[] = "seh_vhcd";
static const uint8_t driver_desc[] = "SEH Virtual Host Controller";

uint32_t seh_debug_flag = /*seh_debug_vhci_rh |  \
                  seh_debug_vhci_hc | \
                  seh_debug_vhci_rx | \
                  seh_debug_vhci_tx | \
                  seh_debug_vhci_service | \
                  seh_debug_vhci_eh | \
                  seh_debug_vhci_cdev | \
                  seh_debug_vhci_data | \
                  seh_debug_vhci_data1 | \
                  seh_debug_vhci_sema | */0;


struct vhci_hcd *controllers[VHCI_CONTROLLERS];
struct seh_char_dev *vhci_char_dev;
atomic_t glob_urb_seqnum;

struct list_head glob_priv_cache;
int32_t glob_cache_count;
spinlock_t glob_cache_lock;


static const uint8_t * const bit_desc[] = {
	"CONNECTION",		/*0*/
	"ENABLE",		/*1*/
	"SUSPEND",		/*2*/
	"OVER_CURRENT",		/*3*/
	"RESET",		/*4*/
	"R5",			/*5*/
	"R6",			/*6*/
	"R7",			/*7*/
	"POWER",		/*8*/
	"LOWSPEED",		/*9*/
	"HIGHSPEED",		/*10*/
	"PORT_TEST",		/*11*/
	"INDICATOR",		/*12*/
	"R13",			/*13*/
	"R14",			/*14*/
	"R15",			/*15*/
	"C_CONNECTION",		/*16*/
	"C_ENABLE",		/*17*/
	"C_SUSPEND",		/*18*/
	"C_OVER_CURRENT",	/*19*/
	"C_RESET",		/*20*/
	"R21",			/*21*/
	"R22",			/*22*/
	"R23",			/*23*/
	"R24",			/*24*/
	"R25",			/*25*/
	"R26",			/*26*/
	"R27",			/*27*/
	"R28",			/*28*/
	"R29",			/*29*/
	"R30",			/*30*/
	"R31",			/*31*/
};
#define MAX_PRIV	100



/* USB 3 BOS descriptor and a capability descriptors, combined.
 * Fields will be adjusted and added later in xhci_create_usb3_bos_desc()
 */
static u8 usb_bos_descriptor [] = {
	USB_DT_BOS_SIZE,		/*  __u8 bLength, 5 bytes */
	USB_DT_BOS,			/*  __u8 bDescriptorType */
	0x0F, 0x00,			/*  __le16 wTotalLength, 15 bytes */
	0x1,				/*  __u8 bNumDeviceCaps */
	/* First device capability, SuperSpeed */
	USB_DT_USB_SS_CAP_SIZE,		/*  __u8 bLength, 10 bytes */
	USB_DT_DEVICE_CAPABILITY,	/* Device Capability */
	USB_SS_CAP_TYPE,		/* bDevCapabilityType, SUPERSPEED_USB */
	0x00,				/* bmAttributes, LTM off by default */
	USB_5GBPS_OPERATION, 0x00,	/* wSpeedsSupported, 5Gbps only */
	0x03,				/* bFunctionalitySupport,
					   USB 3.0 speed only */
	0x00,				/* bU1DevExitLat, set later. */
	0x00, 0x00,			/* __le16 bU2DevExitLat, set later. */
	/* Second device capability, SuperSpeedPlus */
	0x1c,				/* bLength 28, will be adjusted later */
	USB_DT_DEVICE_CAPABILITY,	/* Device Capability */
	USB_SSP_CAP_TYPE,		/* bDevCapabilityType SUPERSPEED_PLUS */
	0x00,				/* bReserved 0 */
	0x23, 0x00, 0x00, 0x00,		/* bmAttributes, SSAC=3 SSIC=1 */
	0x01, 0x00,			/* wFunctionalitySupport */
	0x00, 0x00,			/* wReserved 0 */
	/* Default Sublink Speed Attributes, overwrite if custom PSI exists */
	0x34, 0x00, 0x05, 0x00,		/* 5Gbps, symmetric, rx, ID = 4 */
	0xb4, 0x00, 0x05, 0x00,		/* 5Gbps, symmetric, tx, ID = 4 */
	0x35, 0x40, 0x0a, 0x00,		/* 10Gbps, SSP, symmetric, rx, ID = 5 */
	0xb5, 0x40, 0x0a, 0x00,		/* 10Gbps, SSP, symmetric, tx, ID = 5 */
};
#ifdef DEBUG_STATISTIC

void print_cache_stat(void){
	printk ("Cache count: %d\n", glob_cache_count);
}

#endif
#ifdef CONFIG_SYSFS
#include <linux/sysfs.h>
static ssize_t seh_vhci_sys_debug_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE,"%u\n", seh_debug_flag);
}
static ssize_t seh_vhci_sys_debug_store(struct device *dev,
				   struct device_attribute *attr, const char *buf, size_t count)
{
    int32_t val;
    ssize_t result;
    if (count > 11)
        return -EINVAL;
    result = sscanf(buf, "%x\n", &val);
    if (result == 1) {
       seh_debug_flag = val;
       result = count;
    } else {
       result = -EINVAL;
    }
    return result;
}

#ifndef DEVICE_ATTR_RW
#ifndef __ATTR_RW
#define __ATTR_RW(_name) __ATTR(_name, (S_IWUSR | S_IRUGO),		\
			 _name##_show, _name##_store)
#endif
#define DEVICE_ATTR_RW(_name) \
	struct device_attribute dev_attr_##_name = __ATTR_RW(_name)
#endif

static DEVICE_ATTR_RW(seh_vhci_sys_debug);

static int seh_vhci_add_debug_entry(struct device *dev)
{
	int err;

	err = device_create_file(dev, &dev_attr_seh_vhci_sys_debug);
	if (err < 0)
		goto fail;

fail:
	return err;
}

static void seh_vhci_del_debug_entry(struct device *dev)
{
	device_remove_file(dev, &dev_attr_seh_vhci_sys_debug);
}

#else
#define seh_vhci_add_debug_entry(dev) 0
#define seh_vhci_del_debug_entry(dev) do {} while (0)
#endif /* CONFIG_SYSFS */



inline struct vhci_priv *kzalloc_priv(){
	struct vhci_priv *priv;
    unsigned long flags;
    
    spin_lock_irqsave(&glob_cache_lock, flags);
	if (list_empty(&glob_priv_cache)){
		priv = kmalloc(sizeof(struct vhci_priv), GFP_ATOMIC);
	}else{
		priv = list_first_entry(&glob_priv_cache, struct vhci_priv, list);
		list_del_init(&glob_priv_cache);
		glob_cache_count--;
	}
	spin_unlock_irqrestore(&glob_cache_lock, flags);
	return priv;
}

inline void free_priv(struct vhci_priv *priv){
    unsigned long flags;
    spin_lock_irqsave(&glob_cache_lock, flags);
	if (glob_cache_count > MAX_PRIV) {
        seh_dbg_vhci_rh("%s kfree:%p\n",__func__,priv);
		kfree(priv);
        }
	else{
		list_add_tail(&priv->list, &glob_priv_cache);
		glob_cache_count++;
	}
    spin_unlock_irqrestore(&glob_cache_lock, flags);
}

static void dump_port_status_diff(u32 prev_status, u32 new_status)
{
	int32_t i = 0;
	u32 bit = 1;

	pr_debug("status prev -> new: %08x -> %08x\n", prev_status, new_status);
	while (bit) {
		u32 prev = prev_status & bit;
		u32 new = new_status & bit;
		uint8_t change;

		if (!prev && new)
			change = '+';
		else if (prev && !new)
			change = '-';
		else
			change = ' ';

		if (prev || new)
			pr_debug(" %c%s\n", change, bit_desc[i]);
		bit <<= 1;
		i++;
	}
	pr_debug("\n");
}

void rh_port_connect(struct vhci_hcd *hcd_controller, int32_t rhport, enum usb_device_speed speed)
{
	unsigned long flags;

    seh_dbg_vhci_rh("%s(%p, %d, %d)\n", __func__, hcd_controller, rhport, speed);

	HCD_LOCK_IRQSAVE(hcd_controller, flags);

	hcd_controller->port_status[rhport] |= USB_PORT_STAT_CONNECTION
		| (1 << USB_PORT_FEAT_C_CONNECTION);

	switch (speed) {
	case USB_SPEED_SUPER:
        hcd_controller->port_status[rhport]  |= (USB_PORT_STAT_CONNECTION |
						 USB_PORT_STAT_SPEED_5GBPS);
		break;
	case USB_SPEED_HIGH:
		hcd_controller->port_status[rhport] |= USB_PORT_STAT_HIGH_SPEED;
		break;
	case USB_SPEED_FULL:
	case USB_SPEED_LOW:
		hcd_controller->port_status[rhport] |= USB_PORT_STAT_LOW_SPEED;
		break;
	default:
		break;
	}

	HCD_UNLOCK_IRQRESTORE(hcd_controller, flags);

	usb_hcd_poll_rh_status(vhci_to_hcd(hcd_controller));
}

void rh_port_disconnect(struct vhci_hcd *hcd_controller, int32_t rhport)
{
	unsigned long flags;

	seh_dbg_vhci_rh("rh_port_disconnect %d\n", rhport);

	HCD_LOCK_IRQSAVE(hcd_controller, flags);
	
	/* stop_activity(dum, driver); */
	hcd_controller->port_status[rhport] &= ~USB_PORT_STAT_CONNECTION;
	hcd_controller->port_status[rhport] |=
					(1 << USB_PORT_FEAT_C_CONNECTION);

	/* not yet complete the disconnection
	 * spin_lock(&vdev->ud.lock);
	 * vdev->ud.status = VHC_ST_DISCONNECT;
	 * spin_unlock(&vdev->ud.lock); */

	HCD_UNLOCK_IRQRESTORE(hcd_controller, flags);
	usb_hcd_poll_rh_status(vhci_to_hcd(hcd_controller));
}

#define PORT_C_MASK				\
	((USB_PORT_STAT_C_CONNECTION		\
	  | USB_PORT_STAT_C_ENABLE		\
	  | USB_PORT_STAT_C_SUSPEND		\
	  | USB_PORT_STAT_C_OVERCURRENT		\
	  | USB_PORT_STAT_C_RESET) << 16)

/*
 * This function is almostly the same as dummy_hcd.c:dummy_hub_status() without
 * suspend/resume support. But, it is modified to provide multiple ports.
 *
 * @buf: a bitmap to show which port status has been changed.
 *  bit  0: reserved or used for another purpose?
 *  bit  1: the status of port 0 has been changed.
 *  bit  2: the status of port 1 has been changed.
 *  ...
 *  bit  7: the status of port 6 has been changed.
 *  bit  8: the status of port 7 has been changed.
 *  ...
 *  bit 15: the status of port 14 has been changed.
 *
 * So, the maximum number of ports is 31 ( port 0 to port 30) ?
 *
 * The return value is the actual transferred length in byte. If nothing has
 * been changed, return 0. In the case that the number of ports is less than or
 * equal to 6 (VHCI_NPORTS==7), return 1.
 *
 */
static int32_t vhci_hub_status(struct usb_hcd *hcd, char *buf)
{
	struct vhci_hcd	*hcd_controller;
	unsigned long	flags;
	int32_t		retval = 0;

	/* the enough buffer is allocated according to USB_MAXCHILDREN */
	uint32_t	*event_bits = (uint32_t *) buf;
	int32_t		rhport;
	int32_t		changed = 0;

	*event_bits = 0;

	hcd_controller = hcd_to_vhci(hcd);

	HCD_LOCK_IRQSAVE(hcd_controller, flags);
	if (!HCD_HW_ACCESSIBLE(hcd)) {
		seh_dbg_vhci_rh("hw accessible flag in on?\n");
		goto done;
	}

	/* check pseudo status register for each port */
	for (rhport = 0; rhport < VHCI_NPORTS; rhport++) {
		if ((hcd_controller->port_status[rhport] & PORT_C_MASK)) {
			/* The status of a port has been changed, */
			seh_dbg_vhci_rh("port %d is changed\n", rhport);

			*event_bits |= 1 << (rhport + 1);
			changed = 1;
		}
	}

   // pr_info("changed %d\n", changed);

	if (hcd->state == HC_STATE_SUSPENDED)
		usb_hcd_resume_root_hub(hcd);

	if (changed)
		retval = 1 + (VHCI_NPORTS / 8);
	else
		retval = 0;

done:
	HCD_UNLOCK_IRQRESTORE(hcd_controller, flags);
	return retval;
}
static void vhci_common_hub_descriptor(struct usb_hub_descriptor *desc)
{
	desc->bPwrOn2PwrGood = 10;	/* xhci section 5.4.9 says 20ms max */
	desc->bHubContrCurrent = 0;
	desc->bNbrPorts = VHCI_NPORTS;
	desc->wHubCharacteristics = cpu_to_le16(HUB_CHAR_INDV_PORT_LPSM | HUB_CHAR_INDV_PORT_OCPM);
    
}
/* Fill in the USB 3.0 roothub descriptor */
static void vhci_usb3_hub_descriptor(struct usb_hub_descriptor *desc)
{
	u16 port_removable;
	unsigned int i;
	
	vhci_common_hub_descriptor(desc);
	desc->bDescriptorType = USB_DT_SS_HUB;
	desc->bDescLength = USB_DT_SS_HUB_SIZE;

	/* header decode latency should be zero for roothubs,
	 * see section 4.23.5.2.
	 */
	desc->u.ss.bHubHdrDecLat = 0;
	desc->u.ss.wHubDelay = 0;

	port_removable = 0;
	/* bit 0 is reserved, bit 1 is for port 1, etc. */
	for (i = 0; i < VHCI_NPORTS; i++) {
		port_removable |= 1 << (i + 1);
	}
	desc->u.ss.DeviceRemovable = cpu_to_le16(port_removable);
}

/* Fill in the USB 2.0 roothub descriptor */
static void vhci_usb2_hub_descriptor(struct usb_hub_descriptor *desc)
{
	__u8 port_removable[(USB_MAXCHILDREN + 1 + 7) / 8];
	unsigned int i;
 
	vhci_common_hub_descriptor(desc);
	desc->bDescriptorType = USB_DT_HUB;
	desc->bDescLength = USB_DT_HUB_NONVAR_SIZE + 2 * (1 + (VHCI_NPORTS / 8));

	memset(port_removable, 0, sizeof(port_removable));
	for (i = 0; i < VHCI_NPORTS; i++) {
		port_removable[(i + 1) / 8] |= 1 << ((i + 1) % 8);
	}
	memset(desc->u.hs.DeviceRemovable, 0xff,
			sizeof(desc->u.hs.DeviceRemovable));
	memset(desc->u.hs.PortPwrCtrlMask, 0xff,
			sizeof(desc->u.hs.PortPwrCtrlMask));

	for (i = 0; i < (VHCI_NPORTS + 1 + 7) / 8; i++)
		memset(&desc->u.hs.DeviceRemovable[i], port_removable[i],
				sizeof(__u8));

}
/* See hub_configure in hub.c */
static inline void hub_descriptor(struct usb_hcd *hcd, struct usb_hub_descriptor *desc)
{
    if (hcd->speed >= HCD_USB3)
		vhci_usb3_hub_descriptor(desc);
	else
		vhci_usb2_hub_descriptor(desc);
}


static int vhci_create_usb3_bos_desc(char *buf, u16 wLength)
{
	u16 desc_size, ssp_cap_size, ssa_size = 0;

	desc_size = USB_DT_BOS_SIZE + USB_DT_USB_SS_CAP_SIZE;
	ssp_cap_size = sizeof(usb_bos_descriptor) - desc_size;

	memcpy(buf, &usb_bos_descriptor, min(desc_size, wLength));

	if (wLength < USB_DT_BOS_SIZE + USB_DT_USB_SS_CAP_SIZE)
		return wLength;
	/* ssa_size is 0 for other than usb 3.1 hosts */
	return desc_size + ssa_size;
}

static int32_t vhci_hub_control(struct usb_hcd *hcd, u16 typeReq, u16 wValue,
			    u16 wIndex, char *buf, u16 wLength)
{
	struct vhci_hcd	*hcd_controller;
	int32_t             retval = 0;
	unsigned long   flags;
	int		rhport;

	u32 prev_port_status[VHCI_NPORTS];

	if (!HCD_HW_ACCESSIBLE(hcd))
		return -ETIMEDOUT;

	/*
	 * NOTE:
	 * wIndex shows the port number and begins from 1.
	 */
	seh_dbg_vhci_rh("typeReq %x wValue %x wIndex %x\n", typeReq, wValue,
			  wIndex);
	
	rhport = ((__u8)(wIndex & 0x00ff)) - 1;

    if (rhport > VHCI_NPORTS){
        WARN_ON_ONCE(1);
		pr_err("invalid port number %d\n", wIndex);
        return -EINVAL;
    }

	hcd_controller = hcd_to_vhci(hcd);

	HCD_LOCK_IRQSAVE(hcd_controller, flags);

	/* store old status and compare now and old later */
	if (seh_dbg_flag_vhci_rh) {
		memcpy(prev_port_status, hcd_controller->port_status,
			sizeof(prev_port_status));
	}

	switch (typeReq) {
        case GetHubStatus:
            /* No power source, over-current reported per port */
            memset(buf, 0, 4);
            break;
        case GetHubDescriptor:
            seh_dbg_vhci_rh(" GetHubDescriptor\n");
            if (hcd->speed >= HCD_USB3 &&
				(wLength < USB_DT_SS_HUB_SIZE ||
				 wValue != (USB_DT_SS_HUB << 8))) {
                seh_dbg_vhci_rh("Wrong hub descriptor type for "
                        "USB 3.0 roothub.\n");
                retval = -EPIPE;
                goto error;
            }
            hub_descriptor(hcd, (struct usb_hub_descriptor *) buf);
            break;
        case DeviceRequest | USB_REQ_GET_DESCRIPTOR:
            seh_dbg_vhci_rh(" DeviceRequest USB_REQ_GET_DESCRIPTOR\n");
            if ((wValue & 0xff00) != (USB_DT_BOS << 8))
                goto error;
    
            if (hcd->speed < HCD_USB3)
                goto error;
    
            retval = vhci_create_usb3_bos_desc(buf, wLength);
            HCD_UNLOCK_IRQRESTORE(hcd_controller, flags);
            
            return retval;
    case GetPortStatus: 
		seh_dbg_vhci_rh(" GetPortStatus port %x\n", wIndex);
		/* we do no care of resume. */

		/* whoever resets or resumes must GetPortStatus to
		 * complete it!!
		 *                                   */
		if (hcd_controller->resuming && time_after(jiffies, hcd_controller->re_timeout)) {
			hcd_controller->port_status[rhport] |=
				(1 << USB_PORT_FEAT_C_SUSPEND);
			hcd_controller->port_status[rhport] &=
				~(1 << USB_PORT_FEAT_SUSPEND);
			hcd_controller->resuming = 0;
			hcd_controller->re_timeout = 0;
			/* if (dum->driver && dum->driver->resume) {
			 *	spin_unlock (&dum->lock);
			 *	dum->driver->resume (&dum->gadget);
			 *	spin_lock (&dum->lock);
			 * } */
		}

		if ((hcd_controller->port_status[rhport] & (1 << USB_PORT_FEAT_RESET)) !=
		    0 && time_after(jiffies, hcd_controller->re_timeout)) {
			hcd_controller->port_status[rhport] |=
				(1 << USB_PORT_FEAT_C_RESET);
			hcd_controller->port_status[rhport] &=
				~(1 << USB_PORT_FEAT_RESET);
			hcd_controller->re_timeout = 0;

			if (hcd_controller->vdev[rhport].device_status ==
			    VDEV_ST_NOTASSIGNED) {
				seh_dbg_vhci_rh(" enable rhport %d "
						  "(status %u)\n",
						  rhport,
						  hcd_controller->vdev[rhport].device_status);
				hcd_controller->port_status[rhport] |=
					USB_PORT_STAT_ENABLE;

				switch (hcd_controller->vdev[rhport].speed) {
                    case USB_SPEED_SUPER: 
                        seh_dbg_vhci_rh(" GetPortStatus USB_SPEED_SUPER\n");
                        hcd_controller->port_status[rhport] |= USB_SS_PORT_STAT_POWER;
                        hcd_controller->port_status[rhport] &= ~USB_SS_PORT_STAT_SPEED;
                        break;
					case USB_SPEED_HIGH:
						hcd_controller->port_status[rhport] |=
						      USB_PORT_STAT_HIGH_SPEED;
						break;
					case USB_SPEED_LOW:
						hcd_controller->port_status[rhport] |=
							USB_PORT_STAT_LOW_SPEED;
						break;
					default:
						hcd_controller->port_status[rhport] |=
							USB_SPEED_FULL;
						break;
					}
                }
		}
		((u16 *) buf)[0] = cpu_to_le16(hcd_controller->port_status[rhport]);
		((u16 *) buf)[1] = cpu_to_le16(hcd_controller->port_status[rhport] >> 16);

		seh_dbg_vhci_rh(" GetPortStatus bye %x %x\n", ((u16 *)buf)[0],
				  ((u16 *)buf)[1]);
		break;
    case SetPortFeature:
		switch (wValue) {
		case USB_PORT_FEAT_SUSPEND:
			seh_dbg_vhci_rh(" SetPortFeature: "
					  "USB_PORT_FEAT_SUSPEND\n");
			break;
		case USB_PORT_FEAT_RESET:
            seh_dbg_vhci_rh(" SetPortFeature: "
					  "USB_PORT_FEAT_RESET\n");
			/* if it's already running, disconnect first */
			if (hcd_controller->port_status[rhport] & USB_PORT_STAT_ENABLE) {
				hcd_controller->port_status[rhport] &=
					~(USB_PORT_STAT_ENABLE |
					  USB_PORT_STAT_LOW_SPEED |
					  USB_PORT_STAT_HIGH_SPEED);
				/* FIXME test that code path! */
			}
			/* 50msec reset signaling */
			hcd_controller->re_timeout = jiffies + msecs_to_jiffies(50);
            if (hcd_controller->vdev[rhport].device_status == VDEV_ST_USED
                    || hcd_controller->vdev[rhport].device_status == VDEV_ST_ERROR) {
                /* [fl] Avoid re-changing the device status after a device_reset has been conducted (VDEV_ST_NULL)
                 * or the device is currently in the process of being unplugged (DEV_ST_STOP_PENDING, VDEV_ST_STOPPED)
                 */
                hcd_controller->vdev[rhport].device_status = VDEV_ST_NOTASSIGNED;
            }
            seh_dbg_vhci_rh("%s [hcd:%d rhport:%d] stat:%d\n", __func__, hcd_controller->id, rhport, hcd_controller->vdev[rhport].device_status);
			/* FALLTHROUGH */
		default:
			seh_dbg_vhci_rh(" SetPortFeature: default %d\n",
					  wValue);
			hcd_controller->port_status[rhport] |= (1 << wValue);
			break;
		}
		break;    
        
        
	case ClearHubFeature:
		seh_dbg_vhci_rh(" ClearHubFeature\n");
		break;
	case ClearPortFeature:
		switch (wValue) {
		case USB_PORT_FEAT_SUSPEND:
			if (hcd_controller->port_status[rhport] & USB_PORT_STAT_SUSPEND) {
				/* 20msec signaling */
				hcd_controller->resuming = 1;
				hcd_controller->re_timeout =
					jiffies + msecs_to_jiffies(20);
			}
			break;
		case USB_PORT_FEAT_POWER:
			seh_dbg_vhci_rh(" ClearPortFeature: "
					  "USB_PORT_FEAT_POWER\n");
			hcd_controller->port_status[rhport] = 0;
			/* dum->address = 0; */
			/* dum->hdev = 0; */
			hcd_controller->resuming = 0;
			break;
		case USB_PORT_FEAT_C_RESET:
			seh_dbg_vhci_rh(" ClearPortFeature: "
					  "USB_PORT_FEAT_C_RESET\n");
			switch (hcd_controller->vdev[rhport].speed) {
			case USB_SPEED_HIGH:
				hcd_controller->port_status[rhport] |=
					USB_PORT_STAT_HIGH_SPEED;
				break;
			case USB_SPEED_LOW:
				hcd_controller->port_status[rhport] |=
					USB_PORT_STAT_LOW_SPEED;
				break;
			default:
				break;
			}
		default:
			seh_dbg_vhci_rh(" ClearPortFeature: default %x\n",
					  wValue);
			hcd_controller->port_status[rhport] &= ~(1 << wValue);
			break;
		}
		break;

	
	case SetHubFeature:
		seh_dbg_vhci_rh(" SetHubFeature\n");
		retval = -EPIPE;
		break;
	

	default:
error:
		pr_err("default: no such request\n");
		/* dev_dbg (hardware,
		 *		"hub control req%04x v%04x i%04x l%d\n",
		 *		typeReq, wValue, wIndex, wLength); */

		/* "protocol stall" on error */
		retval = -EPIPE;
	}

	if (seh_dbg_flag_vhci_rh) {
		pr_debug("port %d\n", rhport);
		/* Only dump valid port status */
		if (rhport >= 0) {
			dump_port_status_diff(prev_port_status[rhport],
					      hcd_controller->port_status[rhport]);
		}
	}
	seh_dbg_vhci_rh(" bye\n");

	HCD_UNLOCK_IRQRESTORE(hcd_controller, flags);

	return retval;
}

/* called from locked context*/
static inline void vhci_tx_urb(int hcd_index, struct vhci_device *vdev, struct urb *urb)
{
	struct vhci_priv *priv;
    
	if (unlikely(!vdev)) {
		pr_err("could not get virtual device");
		/* BUG(); */
		return;
	}

    dumpUrb(urb);
    
    priv = kzalloc_priv();
    if (unlikely(!priv)) {
		dev_err(&urb->dev->dev, "malloc vhci_priv\n");
		return;
	}
	
    priv->seqnum = atomic_inc_return((atomic_t *)&glob_urb_seqnum);
	priv->vdev = vdev;
	priv->urb = urb;
	priv->unlink_seqnum = 0;
	priv->type = URB;
	urb->hcpriv = (void *) priv;
    
    DEV_LOCK (vdev);
	list_add_tail(&priv->list, &vdev->priv_tx);
    DEV_UNLOCK (vdev);
	
    /*notify chardev that there is data  set (atomically) */
    set_bit(hcd_index, &vhci_char_dev->is_data_tx_bitmap);      

    seh_dbg_vhci_data1("%s add Urb hcd %d %d  is_data_tx_bitmap %lx \n", __func__,
        hcd_index, priv->seqnum, vhci_char_dev->is_data_tx_bitmap);
	wake_up(&vhci_char_dev->txq);
	
	seh_dbg_vhci_data("Enqueue URB HCD %d priv %p URB %p, number %u\n", hcd_index, priv, urb, priv->seqnum);
}

void seh_vusb_add_cmd_to_svc (int hcd_index, struct vhci_device *vdev, uint32_t cmd){
    unsigned long flags;
    struct vhci_priv *priv;

    /* setup CMD_UNLINK pdu */
    priv = kzalloc_priv();
    if (!priv) {
		pr_err("malloc vhci_priv fail\n");
		return ;
    }
    priv->seqnum = atomic_inc_return(&glob_urb_seqnum);
    priv->vdev = vdev;
    priv->urb = NULL;
    priv->type = cmd;
	priv->unlink_seqnum = 0;
	
    seh_dbg_vhci_data("Enqueue Command priv %p CMD %d, number %u controller %d vdev %p:%x\n", priv, cmd, priv->seqnum, hcd_index, vdev, vdev->rh_port_id);
    DEV_LOCK_IRQSAVE (vdev, flags);
    list_add_tail(&priv->list, &vdev->priv_tx);
    DEV_UNLOCK_IRQRESTORE(vdev, flags);
    /*notify chardev that there is data*/
    set_bit(hcd_index, &vhci_char_dev->is_data_tx_bitmap);
    seh_dbg_vhci_data1("COMMAND hcd %d %d  is_data_tx_bitmap %lx \n",hcd_index, priv->seqnum, vhci_char_dev->is_data_tx_bitmap);
    wake_up(&vhci_char_dev->txq);
    
}
static int32_t vhci_urb_enqueue(struct usb_hcd *hcd, struct urb *urb, gfp_t mem_flags)
    {
    struct device *dev = &urb->dev->dev;
    int32_t i,ret = 0;
    unsigned long flags;
    struct vhci_device *vdev;
    struct vhci_hcd *hcd_controller = hcd_to_vhci(hcd);

    seh_dbg_vhci_hc( "%s buf:%p dma:%p len:%d flags:%x sgs:%d\n",
        __func__,urb->transfer_buffer , (void *)urb->transfer_dma ,
        urb->transfer_buffer_length,urb->transfer_flags, urb->num_sgs);

    for (i = 0; i < urb->number_of_packets;i++)
        seh_dbg_vhci_hc("%s frag[%d] len:%d/%d status:%d", __func__,i,
            urb->iso_frame_desc[i].actual_length,
            urb->iso_frame_desc[i].length,
            urb->iso_frame_desc[i].status);
    
    HCD_LOCK_IRQSAVE(hcd_controller, flags);
    
    if (unlikely(urb->status != -EINPROGRESS)) {
        dev_err(dev, "URB already unlinked!, status %d\n", urb->status);
        HCD_UNLOCK_IRQRESTORE(hcd_controller, flags);
        return urb->status;
        }

  vdev = port_to_vdev(hcd_controller, urb->dev->portnum-1);

  ret = usb_hcd_link_urb_to_ep(hcd, urb);
  if (ret)
    goto no_need_unlink;

/*
 * The enumeration process is as follows;
 *
 *  1. Get_Descriptor request to DevAddrs(0) EndPoint(0)
 *     to get max packet length of default pipe
 *
 *  2. Set_Address request to DevAddr(0) EndPoint(0)
 *
 */
  if (usb_pipedevice(urb->pipe) == 0) {
      __u8 type = usb_pipetype(urb->pipe);
      struct usb_ctrlrequest *ctrlreq = (struct usb_ctrlrequest *) urb->setup_packet;
      
      if (type != PIPE_CONTROL || !ctrlreq) {
          dev_err(dev, "invalid request to devnum 0\n");
          ret = -EINVAL;
          goto no_need_xmit;
          }

      switch (ctrlreq->bRequest) {
      case USB_REQ_SET_ADDRESS:
          /* set_address may come when a device is reset */
          dev_info(dev, "SetAddress Request (%d) to hcd(%d) port(%d)\n",
              ctrlreq->wValue, hcd_index(vdev->rh_port_id), hcd_port_index(vdev->rh_port_id));
          
          if (vdev->udev)
              usb_put_dev(vdev->udev);
          vdev->udev = usb_get_dev(urb->dev);
          if (vdev->device_status == VDEV_ST_NOTASSIGNED)
              vdev->device_status = VDEV_ST_USED;
          if (urb->status == -EINPROGRESS) {
              urb->status = 0;
              }
          seh_vusb_add_cmd_to_svc(hcd_controller->id, vdev, IF_STAT);
          
          goto no_need_xmit;
          
      case USB_REQ_GET_DESCRIPTOR:
          if (ctrlreq->wValue == (USB_DT_DEVICE << 8))
              seh_dbg_vhci_hc("Not yet?: "
                  "Get_Descriptor to device 0 "
                  "(get max pipe size)\n");
          
          if (vdev->udev)
              usb_put_dev(vdev->udev);
		vdev->udev = usb_get_dev(urb->dev);
        goto out;
        
      default:
          /* NOT REACHED */
          dev_err(dev, "invalid request to devnum 0 bRequest %u, "
              "wValue %u\n", ctrlreq->bRequest,
              ctrlreq->wValue);
          ret =  -EINVAL;
          goto no_need_xmit;
          }
      
      }

out:
  /* refuse enqueue for dead connection */
  /*Only not assigned and used are allow to send data*/
  if (unlikely( vdev->device_status != VDEV_ST_NOTASSIGNED &&
          vdev->device_status != VDEV_ST_USED)) {
      usb_hcd_unlink_urb_from_ep(hcd, urb);
      urb->status = -ENODEV;
      HCD_UNLOCK_IRQRESTORE(hcd_controller, flags);
      return -ENODEV;
      }
  vhci_tx_urb(hcd_controller->id, vdev, urb);
	
  HCD_UNLOCK_IRQRESTORE(hcd_controller, flags);
  
  return 0;
  
    no_need_xmit:
  usb_hcd_unlink_urb_from_ep(hcd, urb);
    no_need_unlink:
  HCD_UNLOCK_IRQRESTORE(hcd_controller, flags);
  if (!ret)
      usb_hcd_giveback_urb(hcd, urb, urb->status);
  return ret;
    }

/*
 * vhci_rx gives back the urb after receiving the reply of the urb.  If an
 * unlink pdu is sent or not, vhci_rx receives a normal return pdu and gives
 * back its urb. For the driver unlinking the urb, the content of the urb is
 * not important, but the calling to its completion handler is important; the
 * completion of unlinking is notified by the completion handler.
 *
 *
 * CLIENT SIDE
 *
 * - When vhci_hcd receives RET_SUBMIT,
 *
 *	- case 1a). the urb of the pdu is not unlinking.
 *		- normal case
 *		=> just give back the urb
 *
 *	- case 1b). the urb of the pdu is unlinking.
 *		- usbip.ko will return a reply of the unlinking request.
 *		=> give back the urb now and go to case 2b).
 *
 * - When vhci_hcd receives RET_UNLINK,
 *
 *	- case 2a). a submit request is still pending in vhci_hcd.
 *		- urb was really pending in usbip.ko and urb_unlink_urb() was
 *		  completed there.
 *		=> free a pending submit request
 *		=> notify unlink completeness by giving back the urb
 *
 *	- case 2b). a submit request is *not* pending in vhci_hcd.
 *		- urb was already given back to the core driver.
 *		=> do not give back the urb
 *
 *
 * SERVER SIDE
 *
 * - When usbip receives CMD_UNLINK,
 *
 *	- case 3a). the urb of the unlink request is now in submission.
 *		=> do usb_unlink_urb().
 *		=> after the unlink is completed, send RET_UNLINK.
 *
 *	- case 3b). the urb of the unlink request is not in submission.
 *		- may be already completed or never be received
 *		=> send RET_UNLINK
 *
 */
static int32_t vhci_urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int32_t status)
{
	unsigned long flags;
	struct vhci_priv *priv;
	struct vhci_device *vdev;
	struct vhci_hcd *hcd_controller;

    
	bool send_unlink_to_svc = false;
	seh_dbg_vhci_hc("dequeue a urb %p\n", urb);
    
	hcd_controller = hcd_to_vhci(hcd);

    HCD_LOCK_IRQSAVE(hcd_controller, flags);
	
	priv = urb->hcpriv;
	if (!priv) {
		/* URB was never linked! or will be soon given back by
		 * vhci_rx. */
		HCD_UNLOCK_IRQRESTORE(hcd_controller, flags);
        return -EIDRM;
	}

	{
		int32_t ret = 0;
		ret = usb_hcd_check_unlink_urb(hcd, urb, status);
		if (ret) {
			HCD_UNLOCK_IRQRESTORE(hcd_controller, flags);
			return ret;
		}
	}

	 /* send unlink request here? */
	vdev = priv->vdev;
	/*check if to process dequeue*/
	
	send_unlink_to_svc = (vdev->device_status == VDEV_ST_NOTASSIGNED ||
						 vdev->device_status == VDEV_ST_USED);
	
	if (!send_unlink_to_svc) {
		/* tcp connection is closed */

		pr_info("device %p seems to be disconnected\n", vdev);
		list_del(&priv->list);
		free_priv(priv);
		urb->hcpriv = NULL;

		/*
		 * If tcp connection is alive, we have sent CMD_UNLINK.
		 * vhci_rx will receive RET_UNLINK and give back the URB.
		 * Otherwise, we give back it here.
		 */
		pr_info("gives back urb %p\n", urb);

		usb_hcd_unlink_urb_from_ep(hcd, urb);

		HCD_UNLOCK_IRQRESTORE(hcd_controller, flags);
		usb_hcd_giveback_urb(hcd, urb, urb->status);
		HCD_LOCK_IRQSAVE(hcd_controller, flags);

	} else {
		/* tcp connection is alive */
		struct vhci_priv *unlink;
		/* setup CMD_UNLINK pdu */
		unlink = kzalloc_priv();
		if (!unlink) {
			pr_err("malloc vhci_priv\n");
			HCD_UNLOCK_IRQRESTORE(hcd_controller, flags);
			return -ENOMEM;
		}
		unlink->seqnum = atomic_inc_return(&glob_urb_seqnum);
		if (unlink->seqnum == 0xffff)
			pr_info("seqnum max\n");

		unlink->unlink_seqnum = priv->seqnum;
		unlink->urb = NULL;
		unlink->type = UNLINK_URB;
		unlink->vdev = vdev;
		seh_dbg_vhci_hc("device %p seems to be still connected\n", vdev);

		/* send cmd_unlink and try to cancel the pending URB in the
		 * peer */
        DEV_LOCK (vdev);
		list_add_tail(&unlink->list, &vdev->priv_tx);
        DEV_UNLOCK(vdev);
		/*notify chardev that there is data*/
		set_bit(hcd_controller->id, &vhci_char_dev->is_data_tx_bitmap);
        seh_dbg_vhci_data1("ADD UNL hcd %d %d  is_data_tx_bitmap %lx \n",hcd_controller->id, priv->seqnum, vhci_char_dev->is_data_tx_bitmap);
		wake_up(&vhci_char_dev->txq);
		seh_dbg_vhci_data("Enqueue UNLINK priv %p URB %p, number %u unlink Number %u\n", unlink, urb, unlink->seqnum, priv->seqnum);
	}
	
	HCD_UNLOCK_IRQRESTORE(hcd_controller, flags);

	seh_dbg_vhci_hc("leave\n");
	return 0;
}

static void vhci_device_init(struct vhci_device *vdev)
{
	memset(vdev, 0, sizeof(*vdev));

	vdev->device_status = VDEV_ST_NULL;

    spin_lock_init(&vdev->dev_lock);

	INIT_LIST_HEAD(&vdev->priv_rx);
	INIT_LIST_HEAD(&vdev->priv_tx);
}

static int vhci_setup(struct usb_hcd *hcd)
{
    int32_t rhport;
    struct vhci_hcd *vhci = hcd_to_vhci(hcd);
	hcd->self.sg_tablesize = ~0;
#if LINUX_VERSION_CODE > KERNEL_VERSION(3,12,0)
    hcd->self.no_sg_constraint = 1;
    hcd->self.no_stop_on_short = 1;
#endif    
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,4,0)
/*/*/*    hcd->self.uses_dma = 0; */ */ */
#endif    
	if (usb_hcd_is_primary_hcd(hcd)) {
		/*
		 * Mark the first roothub as being USB 2.0.
		 * The USB 3.0 roothub will be registered later by
		 * dummy_hcd_probe()
		 */
		hcd->speed = HCD_USB2;
		hcd->self.root_hub->speed = USB_SPEED_HIGH;
	} else {
		hcd->speed = HCD_USB3;
		hcd->self.root_hub->speed = USB_SPEED_SUPER;
	}
    /* initialize private data of usb_hcd */

	for (rhport = 0; rhport < VHCI_NPORTS; rhport++) {
		struct vhci_device *vdev = port_to_vdev(vhci, rhport);
		vhci_device_init(vdev);
		vdev->rh_port_id = rhport_get(vhci->id, rhport);
        seh_dbg_vhci_hc ("device %p port %x vhci->id %x\n", vdev, vdev->rh_port_id, vhci->id);
	}

	spin_lock_init(&vhci->hcd_lock);

	hcd->power_budget = 0; /* no limit */
    hcd->uses_new_polling = 1;

	return 0;
}
static int32_t vhci_start(struct usb_hcd *hcd)
{
	
	

	seh_dbg_vhci_hc("enter vhci_start\n");
	hcd->state  = HC_STATE_RUNNING;
	
	return 0;
}
/*called on unload from usb_remove_hcd() */
static void vhci_stop(struct usb_hcd *hcd)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);
	int rhport = 0;
    struct vhci_priv *priv;
    
	seh_dbg_vhci_hc("stop VHCI controller %p %p\n", hcd, vhci);
    
	for (rhport = 0 ; rhport < VHCI_NPORTS; rhport++) {
		struct vhci_device *vdev = port_to_vdev(vhci, rhport);
        
        while (!list_empty(&vdev->priv_rx)) {
                priv = list_first_entry(&vdev->priv_rx, struct vhci_priv, list);
                list_del(&priv->list);
                free_priv(priv);
        }
        while (!list_empty(&vdev->priv_tx)) {
                priv = list_first_entry(&vdev->priv_tx, struct vhci_priv, list);
                list_del(&priv->list);
                free_priv(priv);
        }
            
	}
    
}

static int32_t vhci_get_frame_number(struct usb_hcd *hcd)
    {
#if 0
    seh_dbg_vhci_hc("%s(%p) fake return 0\n",__func__,hcd); 
    return 0; // not implemented like usbip?
#endif
    /* there are both host and device side versions of this call ... */
    int32_t fNo;
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,4,0)
    struct timespec ts;
    ktime_get_ts(&ts);
#else
    struct timespec64 ts;  // ubuntu 16.04, at least newer than 12.04
    ktime_get_ts64(&ts);   // but it doesn't matter if we have 64 version or not
#endif
    fNo =  (ts.tv_nsec / NSEC_PER_MSEC) & 0x7fffffff;
    
    seh_dbg_vhci_hc("%s(%p) return:%#x\n",__func__,hcd,fNo); 
    return fNo;
    }


/* Change a group of bulk endpoints to support multiple stream IDs */
static int vhci_alloc_streams(struct usb_hcd *hcd, struct usb_device *udev,
	struct usb_host_endpoint **eps, unsigned int num_eps,
	unsigned int num_streams, gfp_t mem_flags)
{
    pr_err("vhci_alloc_streams\n");
	return -ENOSYS;
}

/* Reverts a group of bulk endpoints back to not using stream IDs. */
static int vhci_free_streams(struct usb_hcd *hcd, struct usb_device *udev,
	struct usb_host_endpoint **eps, unsigned int num_eps,
	gfp_t mem_flags)
{
    pr_err("vhci_free_streams\n");
	return -ENOSYS;
}


#if 0 /*def CONFIG_PM*/

/* FIXME: suspend/resume */
static int32_t vhci_bus_suspend(struct usb_hcd *hcd)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);

	dev_dbg(&hcd->self.root_hub->dev, "%s\n", __func__);

	spin_lock_irq(vhci->hcd_lock);
	/* vhci->rh_state = DUMMY_RH_SUSPENDED;
	 * set_link_state(vhci); */
	hcd->state = HC_STATE_SUSPENDED;
	spin_unlock_irq(vhci->hcd_lock);

	return 0;
}

static int32_t vhci_bus_resume(struct usb_hcd *hcd)
{
	struct vhci_hcd *vhci = hcd_to_vhci(hcd);
    
	int32_t rc = 0;

	dev_dbg(&hcd->self.root_hub->dev, "%s\n", __func__);

	spin_lock_irq(vhci->hcd_lock);
	if (!HCD_HW_ACCESSIBLE(hcd)) {
		rc = -ESHUTDOWN;
	} else {
		/* vhci->rh_state = DUMMY_RH_RUNNING;
		 * set_link_state(vhci);
		 * if (!list_empty(&vhci->urbp_list))
		 *	mod_timer(&vhci->timer, jiffies); */
		hcd->state = HC_STATE_RUNNING;
	}
	spin_unlock_irq(vhci->hcd_lock);

	return rc;
}

#else
#define vhci_bus_suspend      NULL
#define vhci_bus_resume       NULL
#endif

//------------------------------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------------------------------

int vhci_discover_or_reset_device(struct usb_hcd *hcd, struct usb_device *udev)
    {
	seh_dbg_vhci_hc("%s fake\n",__func__);
    return 0;
    }


#if 0
//              no more fakes today ....
//------------------------------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------------------------------
int vhci_drop_endpoint(struct usb_hcd *hcd, struct usb_device *udev,struct usb_host_endpoint *ep)    
    {
	seh_dbg_vhci_hc("%s(%p,%p,%p) fake\n",__func__,hcd,udev,ep);
    if (ep) {
        struct usb_endpoint_descriptor *ed =  &ep->desc;
        seh_dbg_vhci_hc("%s EndP l:%#x t:%#x addr:%#x attr:%#x max:%d intv:%d\n",__func__,
            ed->bLength, ed->bDescriptorType, ed->bEndpointAddress,
            ed->bmAttributes, ed->wMaxPacketSize ,ed->bInterval);
        /* NOTE:  these two are _only_ in audio endpoints.   */
        /* use USB_DT_ENDPOINT*_SIZE in bLength, not sizeof. */
        if (ed->bLength == USB_DT_ENDPOINT_AUDIO_SIZE){
            seh_dbg_vhci_hc("%s AudioEndP refresh:%#x syncAddrt:%#x\n",__func__,
                ed->bRefresh, ed->bSynchAddress);
            }
        }
    return 0;
    }

//------------------------------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------------------------------
int vhci_add_endpoint(struct usb_hcd *hcd, struct usb_device *udev,
		struct usb_host_endpoint *ep)
    {
	seh_dbg_vhci_hc("%s(%p,%p,%p) fake\n",__func__,hcd,udev,ep);
    if (ep) {
        struct usb_endpoint_descriptor *ed =  &ep->desc;
        seh_dbg_vhci_hc("%s EndP l:%#x t:%#x addr:%#x attr:%#x max:%d intv:%d\n",__func__,
            ed->bLength, ed->bDescriptorType, ed->bEndpointAddress,
            ed->bmAttributes, ed->wMaxPacketSize ,ed->bInterval);
        /* NOTE:  these two are _only_ in audio endpoints.   */
        /* use USB_DT_ENDPOINT*_SIZE in bLength, not sizeof. */
        if (ed->bLength == USB_DT_ENDPOINT_AUDIO_SIZE){
            seh_dbg_vhci_hc("%s AudioEndP refresh:%#x syncAddrt:%#x\n",__func__,
                ed->bRefresh, ed->bSynchAddress);
            }
        }
    return 0;
    }

//------------------------------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------------------------------
int vhci_check_bandwidth(struct usb_hcd *hcd, struct usb_device *udev)
    {


    
	seh_dbg_vhci_hc("%s(%p,%p) fake\n",__func__,hcd,udev);
    return 0;
    }

//------------------------------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------------------------------
void vhci_reset_bandwidth(struct usb_hcd *hcd, struct usb_device *udev)
    {
	seh_dbg_vhci_hc("%s(%p,%p) fake\n",__func__,hcd,udev);
    }

//------------------------------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------------------------------
void vhci_endpoint_reset(struct usb_hcd *hcd,struct usb_host_endpoint *ep)
    {
	seh_dbg_vhci_hc("%s fake\n",__func__);
    }
#endif
//------------------------------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------------------------------


// static const struct hc_driver vhci_hc_driver = {
static struct hc_driver vhci_hc_driver = {
	.description	= driver_name,
	.product_desc	= driver_desc,
	.hcd_priv_size	= sizeof(struct vhci_hcd),

	.flags		= HCD_USB3 | HCD_SHARED,

    .reset     	= vhci_setup,
	.start		= vhci_start,
	.stop		= vhci_stop,

	.urb_enqueue	= vhci_urb_enqueue,
	.urb_dequeue	= vhci_urb_dequeue,

	.get_frame_number = vhci_get_frame_number,

	.hub_status_data = vhci_hub_status,
	.hub_control     = vhci_hub_control,
	.bus_suspend	 = vhci_bus_suspend,
	.bus_resume	     = vhci_bus_resume,
    
    .alloc_streams   = vhci_alloc_streams,
	.free_streams    = vhci_free_streams,
#if 0
    .add_endpoint    =	vhci_add_endpoint,
	.drop_endpoint   =	vhci_drop_endpoint,
	.endpoint_reset  =	vhci_endpoint_reset,
	.check_bandwidth =	vhci_check_bandwidth,
	.reset_bandwidth =	vhci_reset_bandwidth,
#endif
    };

static int32_t vhci_hcd_probe(struct platform_device *pdev)
{
	struct usb_hcd		*hs_hcd;
    struct usb_hcd		*ss_hcd;
	int32_t			ret;
    int32_t			usb3_id = VHCI_FIRST_USB3 + pdev->id;
    seh_dbg_vhci_hc("name %s id %d usb3 %d\n", pdev->name, pdev->id, usb3_id);

	/* will be removed */
    /* [fl] Hmmm...it now is.
     * For kernel >= 5.4.0 dma_mask is not null. This maybe related to uses_dma being no longer present
     * in struct usb_bus. (see vhci_setup())
     */
//	if (pdev->dev.dma_mask) {
//		dev_info(&pdev->dev, "vhci_hcd DMA not supported\n");
//		return -EINVAL;
//	}

	/*
	 * Allocate and initialize hcd.
	 * Our private data is also allocated automatically.
	 */
    if (pdev->id < VHCI_FIRST_USB3){
        vhci_hc_driver.flags = HCD_USB2;
    }
	hs_hcd = usb_create_hcd(&vhci_hc_driver, &pdev->dev, dev_name(&pdev->dev));
    if (!hs_hcd) {
		pr_err("create hcd failed\n");
		return -ENOMEM;
	}
    hs_hcd->has_tt = 1;
	/* this is private data for vhci_hcd */
	controllers[pdev->id] = hcd_to_vhci(hs_hcd);
    /*seh the id of this hcd*/
    controllers[pdev->id]->id = pdev->id;
    
    seh_dbg_vhci_hc("HCD %p priv HCD %p ID %d\n", controllers[pdev->id], hs_hcd, pdev->id);
    
    ret = usb_add_hcd(hs_hcd, -1, 0);
	if (ret != 0) {
		goto put_usb2_hcd;
	}
    
    ss_hcd = usb_create_shared_hcd(&vhci_hc_driver, &pdev->dev,
					dev_name(&pdev->dev), hs_hcd);
    if (!ss_hcd) {
		ret = -ENOMEM;
		goto dealloc_usb2_hcd;
	}
    controllers[usb3_id] = hcd_to_vhci(ss_hcd);
    /*seh the id of this hcd*/
    controllers[usb3_id]->id = usb3_id;
    seh_dbg_vhci_hc("HCD %p priv HCD %p ID %d\n", controllers[usb3_id], ss_hcd, usb3_id);
    
    ret = usb_add_hcd(ss_hcd, 0, 0);
    if (ret)
		goto put_usb3_hcd;
    
    seh_vhci_add_debug_entry (&pdev->dev);
	seh_dbg_vhci_hc("bye\n");
	return 0;
    
put_usb3_hcd:
	usb_put_hcd(ss_hcd);
dealloc_usb2_hcd:
	usb_remove_hcd(hs_hcd);
put_usb2_hcd:
	usb_put_hcd(hs_hcd);
	controllers[pdev->id] = NULL;
    controllers[usb3_id] = NULL;
	return ret;
}

static int32_t vhci_hcd_remove(struct platform_device *pdev)
{
	struct usb_hcd	*hs_hcd;
    struct usb_hcd	*ss_hcd;
    int32_t			usb3_id = VHCI_FIRST_USB3 + pdev->id;
 seh_dbg_vhci_hc("vhci_hcd_remove\n");   
    seh_vhci_del_debug_entry (&pdev->dev);
    
	hs_hcd = platform_get_drvdata(pdev);
	if (!hs_hcd)
		return 0;
seh_dbg_vhci_hc("vhci_hcd_remove\n");          
    ss_hcd = vhci_to_hcd(controllers[usb3_id]);
    seh_dbg_vhci_hc("vhci_hcd_remove %p\n", ss_hcd); 
    if (ss_hcd){
        usb_remove_hcd(ss_hcd);
        usb_put_hcd(ss_hcd);
        controllers[usb3_id] = NULL;
    }
seh_dbg_vhci_hc("vhci_hcd_remove\n");      
	/*
	 * Disconnects the root hub,
	 * then reverses the effects of usb_add_hcd(),
	 * invoking the HCD's stop() methods.
	 */
seh_dbg_vhci_hc("vhci_hcd_remove %p %p\n", hs_hcd, hs_hcd->self.controller );       
	usb_remove_hcd(hs_hcd);
seh_dbg_vhci_hc("vhci_hcd_remove\n");      
	usb_put_hcd(hs_hcd);
seh_dbg_vhci_hc("vhci_hcd_remove\n");      
	controllers[pdev->id] = NULL;
seh_dbg_vhci_hc("vhci_hcd_remove END\n");   
	return 0;
}

#ifdef CONFIG_PM

/* what should happen for USB/IP under suspend/resume? */
static int32_t vhci_hcd_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct usb_hcd *hcd;
    struct vhci_hcd *hcd_controller;
	int32_t rhport = 0;
	int32_t connected = 0;
	int32_t ret = 0;
    unsigned long flags;
    int32_t			usb3_id = VHCI_FIRST_USB3 + pdev->id;
 seh_dbg_vhci_hc("vhci_hcd_suspend\n");      
	hcd = platform_get_drvdata(pdev);
    if (!hcd)
		return 0;
    hcd_controller = hcd_to_vhci(hcd);
    
	HCD_LOCK_IRQSAVE(hcd_controller, flags);

	for (rhport = 0; rhport < VHCI_NPORTS; rhport++)
		if (hcd_controller->port_status[rhport] &
		    USB_PORT_STAT_CONNECTION)
			connected += 1;
   
	HCD_UNLOCK_IRQRESTORE(hcd_controller, flags);

	if (connected > 0) {
		dev_info(&pdev->dev, "We have %d active connection%s. Do not "
			 "suspend.\n", connected, (connected == 1 ? "" : "s"));
		ret =  -EBUSY;
	} else {
		dev_info(&pdev->dev, "suspend vhci_hcd");
		clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	}
    /*USB 3*/
    hcd_controller = controllers[usb3_id];
    if (!hcd_controller)
        return 0;
    hcd = vhci_to_hcd(hcd_controller);
    if (!hcd)
		return 0;
    
	HCD_LOCK_IRQSAVE(hcd_controller, flags);

	for (rhport = 0; rhport < VHCI_NPORTS; rhport++)
		if (hcd_controller->port_status[rhport] &
		    USB_PORT_STAT_CONNECTION)
			connected += 1;
   
	HCD_UNLOCK_IRQRESTORE(hcd_controller, flags);

	if (connected > 0) {
		dev_info(&pdev->dev, "We have %d active connection%s. Do not "
			 "suspend.\n", connected, (connected == 1 ? "" : "s"));
		ret =  -EBUSY;
	} else {
		dev_info(&pdev->dev, "suspend vhci_hcd");
		clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	}

	return ret;
}

static int32_t vhci_hcd_resume(struct platform_device *pdev)
{
	struct usb_hcd *hcd;
    int32_t			usb3_id = VHCI_FIRST_USB3 + pdev->id;
	dev_dbg(&pdev->dev, "%s\n", __func__);
 seh_dbg_vhci_hc("vhci_hcd_suspend\n"); 
	hcd = platform_get_drvdata(pdev);
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	usb_hcd_poll_rh_status(hcd);
    /*USB 3*/
    hcd = vhci_to_hcd(controllers[usb3_id]);
    if (!hcd)
		return 0;
	hcd = platform_get_drvdata(pdev);
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	usb_hcd_poll_rh_status(hcd);
    
	return 0;
}

#else

#define vhci_hcd_suspend	NULL
#define vhci_hcd_resume		NULL

#endif

static struct platform_driver vhci_driver = {
	.probe	= vhci_hcd_probe,
	.remove	= vhci_hcd_remove,
	.suspend = vhci_hcd_suspend,
	.resume	= vhci_hcd_resume,
	.driver	= {
		.name = (uint8_t *) driver_name,
		.owner = THIS_MODULE,
	},
};

/*
 * The VHCI 'device' is 'virtual'; not a real plug&play hardware.
 * We need to add this virtual device as a platform device arbitrarily:
 *	1. platform_device_register()
 */
static void the_pdev_release(struct device *dev)
{
	return;
}

static struct platform_device the_pdev[VHCI_PLATFORM_DEVICES] ;

static int32_t __init vhci_hcd_init(void)
{
	int32_t ret, i, j;
    
	if (usb_disabled())
		return -ENODEV;

    /*init cache*/
    INIT_LIST_HEAD(&glob_priv_cache);
    spin_lock_init(&glob_cache_lock);
	ret = platform_driver_register(&vhci_driver);
	if (ret < 0)
		goto err_driver_register;
    
    for (i = 0;i < VHCI_PLATFORM_DEVICES; i++){
        the_pdev[i].name = (uint8_t *) driver_name;
        the_pdev[i].id = i;
        the_pdev[i].dev.release = the_pdev_release;

        ret = platform_device_register(&the_pdev[i]);
        if (ret < 0)
            goto err_platform_device_register;
    }
    
	pr_info(DRIVER_DESC " v" SEH_VHCI_VERSION "\n");
	
	ret = seh_chardev_create ();
	if (ret) {
		pr_err("create connection to service files\n");
		return ret;
	}
    
	return ret;

err_platform_device_register:
    for (j = 0;i && j < i; j++){
        platform_device_unregister(&the_pdev[j]);
    }
	platform_driver_unregister(&vhci_driver);
err_driver_register:
	return ret;
}

static void __exit vhci_hcd_exit(void)
{
    int i;
    struct vhci_priv *priv, *tmp;
    /*no lock is requred here*/
    list_for_each_entry_safe(priv, tmp, &glob_priv_cache, list)
    {
		list_del(&priv->list);
        seh_dbg_vhci_rh("%s kfree:%p\n",__func__,priv);
		kfree(priv);
    }
    
    seh_chardev_delete();
    for (i = 0;i < VHCI_PLATFORM_DEVICES; i++){
        platform_device_unregister(&the_pdev[i]);
    }
	platform_driver_unregister(&vhci_driver);
}

module_init(vhci_hcd_init);
module_exit(vhci_hcd_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_VERSION(SEH_VHCI_VERSION);


