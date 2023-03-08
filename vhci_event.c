/*
 * Copyright (C) 2013-2014 SEH Computertechnik GmbH
 *
 * This code is based on drivers/staging/usbip/usbip_event.c
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

#include <linux/kthread.h>

#include "vhci_common.h"
#include "vhci.h"
#include "vhci_event.h"
#include "vhci_xmit.h"

#ifdef DEBUG
void print_cache_stat(void);
uint64_t global_sema_overhead_ns;
uint64_t global_sema_misses;
#endif




/*
 note controller lock is taken
 */
void vhci_device_reset(struct vhci_device *vdev)
{
    seh_dbg_vhci_service("%s ip:%#x bus:%d addr:%d \n", __func__, ntohl(vdev->ip), vdev->bus, vdev->addr);

    vdev->speed  = 0;
	vdev->bus  = 0;
	vdev->addr  = 0;
	vdev->ip  = 0;
	
    pr_info("reset device\n");

	if (vdev->udev)
		usb_put_dev(vdev->udev);

	vdev->udev = NULL;

	vdev->device_status = VDEV_ST_NULL;
	
#ifdef DEBUG_STATISTIC

	{
		struct vhci_priv *priv, *tmp;
		int32_t rx_count=0;
		int32_t tx_count=0;
		
		print_cache_stat();
		list_for_each_entry_safe(priv, tmp, &vdev->priv_tx, list) {
			rx_count++;
		}
		list_for_each_entry_safe(priv, tmp, &vdev->priv_tx, list) {
			tx_count++;
		}
		printk ("Priv RX %d\nPriv TX %d\n",rx_count , tx_count);
		printk ("Time waiting sema %ld, was locked %ld\n", (unsigned long)global_sema_overhead_ns, (unsigned long)global_sema_misses);
		
	}
#endif	
	
}

void unplug_device (struct vhci_device *vdev){
    rh_port_disconnect(get_hcd_from_vhci_device(vdev), hcd_port_index (vdev->rh_port_id));
    vhci_device_reset(vdev);
}


void unpluging_device (struct vhci_device *vdev){

    if (!vdev || vdev->device_status == VDEV_ST_NULL)
        return;
    vdev->device_status = VDEV_ST_STOP_PENDING;
    vhci_device_unlink_all (vdev);
    seh_vusb_add_cmd_to_svc(hcd_index(vdev->rh_port_id), vdev, IF_STAT);

}

