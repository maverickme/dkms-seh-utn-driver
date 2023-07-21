/*
 * Copyright (C) 2013-2014 SEH Computertechnik GmbH
 *
 * This code is based on drivers/staging/usbip/usbip_common.h
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

#ifndef __SEH_COMMON_H
#define __SEH_COMMON_H
#include <linux/version.h>

#include <linux/compiler.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/net.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 37)
#include <config/printk.h>
#else
#include <linux/printk.h>
#endif
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/usb.h>
#include <linux/wait.h>

// $Format: "#define SEH_VHCI_VERSION "3.3.10-$ProjectVersion$\""$
#define SEH_VHCI_VERSION 

#undef pr_fmt

#ifdef DEBUG
#define pr_fmt(fmt)     KBUILD_MODNAME ": %s:%d: " fmt, __func__, __LINE__
#else
#define pr_fmt(fmt)     KBUILD_MODNAME ": " fmt
#endif

enum {
	seh_debug_vhci_rh	= (1 << 0),
	seh_debug_vhci_hc	= (1 << 1),
	seh_debug_vhci_rx	= (1 << 2),
	seh_debug_vhci_tx	= (1 << 3),
	seh_debug_vhci_service  = (1 << 4),
	seh_debug_vhci_eh	= (1 << 5),
	seh_debug_vhci_cdev	= (1 << 6),
	seh_debug_vhci_data	= (1 << 7),
	seh_debug_vhci_sema = (1 << 8),
    seh_debug_vhci_data1 = (1 << 9)
};

#define seh_dbg_flag_vhci_rh	(seh_debug_flag & seh_debug_vhci_rh)
#define seh_dbg_flag_vhci_hc	(seh_debug_flag & seh_debug_vhci_hc)
#define seh_dbg_flag_vhci_rx	(seh_debug_flag & seh_debug_vhci_rx)
#define seh_dbg_flag_vhci_tx	(seh_debug_flag & seh_debug_vhci_tx)
#define seh_dbg_flag_vhci_service  (seh_debug_flag & seh_debug_vhci_service)
#define seh_dbg_flag_vhci_cdev  (seh_debug_flag & seh_debug_vhci_cdev)
#define seh_dbg_flag_vhci_data  (seh_debug_flag & seh_debug_vhci_data)
#define seh_dbg_flag_vhci_sema  (seh_debug_flag & seh_debug_vhci_sema)
#define seh_dbg_flag_vhci_data1 (seh_debug_flag & seh_debug_vhci_data1)

extern uint32_t seh_debug_flag;

#if 0
#warning --------- all debugs are on -------------
#define seh_dbg_with_flag(flag, fmt, args...)		\
	do {                        \
        pr_info(fmt, ##args);   \
	} while (0)
#else
#define seh_dbg_with_flag(flag, fmt, args...)		\
	do {						\
		if (flag & seh_debug_flag)		\
			pr_info(fmt, ##args);		\
	} while (0)
#endif

#define seh_dbg_vhci_rh(fmt, args...)	\
	seh_dbg_with_flag(seh_debug_vhci_rh, fmt , ##args)
#define seh_dbg_vhci_hc(fmt, args...)	\
	seh_dbg_with_flag(seh_debug_vhci_hc, fmt , ##args)
#define seh_dbg_vhci_rx(fmt, args...)	\
	seh_dbg_with_flag(seh_debug_vhci_rx, fmt , ##args)
#define seh_dbg_vhci_tx(fmt, args...)	\
	seh_dbg_with_flag(seh_debug_vhci_tx, fmt , ##args)
#define seh_dbg_vhci_service(fmt, args...) \
	seh_dbg_with_flag(seh_debug_vhci_service, fmt , ##args)
#define seh_dbg_vhci_eh(fmt, args...)	\
	seh_dbg_with_flag(seh_debug_vhci_eh, fmt , ##args)
#define seh_dbg_vhci_cdev(fmt, args...)	\
	seh_dbg_with_flag(seh_debug_vhci_cdev, fmt , ##args)
#define seh_dbg_vhci_data(fmt, args...)	\
	seh_dbg_with_flag(seh_debug_vhci_data, fmt , ##args)	
#define seh_dbg_vhci_data1(fmt, args...)	\
	seh_dbg_with_flag(seh_debug_vhci_data1, fmt , ##args)    

/*
 * This is the same as usb_iso_packet_descriptor but packed for pdu.
 */
struct __unused__usbip_iso_packet_descriptor {
	__u32 offset;
	__u32 length;			/* expected length */
	__u32 actual_length;
	__u32 status;
} __attribute__ ((packed));

#if 0
obsolete with prot version 6
struct IsoFraHdr {
	__u32 start;
	__u32 number;
	__u32 interv;
	__u32 errors;
} __attribute__ ((packed));
#endif

struct IsoPa {  // wireshart network order
    u_int32_t status;
    u_int32_t offs;
    u_int32_t length;
    u_int32_t pad;
    } __attribute__ ((packed));


	
#endif /* __USBIP_COMMON_H */
