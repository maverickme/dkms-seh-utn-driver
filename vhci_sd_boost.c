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
#include "vhci.h"
#include "vhci_event.h"

#include <scsi/scsi.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_devinfo.h>
#include <scsi/scsi_device.h>
#include <scsi/scsi_eh.h>
#include <scsi/scsi_host.h>

#ifdef REDHAT
#define PATCH_MINOR_VER 14
#else
#define PATCH_MINOR_VER 15
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, PATCH_MINOR_VER,0) 
#include <linux/backing-dev-defs.h>
#if __has_include(<linux/genhd.h>)
#include <linux/genhd.h>
#endif
//#define queue_to_disk_seh(q)    (dev_to_disk(kobj_to_dev((q)->kobj.parent)))
//#define queue_has_disk_seh(q)   ((q)->kobj.parent != NULL)
#define queue_to_disk_seh(q)    (dev_to_disk(kobj_to_dev((q)->mq_kobj)))
#define queue_has_disk_seh(q)   ((q)->mq_kobj != NULL)
#endif

#define BUFFER_SECTOR 2048
#define BUFFER_READ_AHEAD 1024
/*
#define BUFFER_SECTOR 1024
#define BUFFER_READ_AHEAD 512*/

int seh_vusb_try_set_buffers(struct usb_device *udev){
    struct usb_interface *ifc;
    int ret = -EINVAL;
    if (!usb_trylock_device(udev)) {
        // [fl] Try again later... (patch for trac #1507)
        //printk("%s usb_trylock_device(%p)...FAILED!\n", __func__, udev);
        return ret;
    }
    if (udev->actconfig){
        int i;
        for (i=0; i<USB_MAXINTERFACES; i++){
            ifc = udev->actconfig->interface[i];
            if (!ifc || !ifc->dev.driver)
                continue;
            
            if (!strcmp (ifc->dev.driver->name, "usb-storage")){
                struct Scsi_Host *scsi_host;
                struct scsi_device *sdev;
                struct us_data *us  = usb_get_intfdata(ifc);
                if (!us)
                    continue;
                ret = 1;
                scsi_host = container_of((void *) us, struct Scsi_Host, hostdata);
                shost_for_each_device(sdev, scsi_host) {
                    if (sdev){
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,PATCH_MINOR_VER,0)                          
                        struct backing_dev_info *dst;
                        blk_queue_max_hw_sectors(sdev->request_queue, BUFFER_SECTOR);
                        /*PATCH für kernel backing_dev_info ist from structure to pointer changed
                         * the patch is ugly but should work*/
                        dst = (struct backing_dev_info *)&sdev->request_queue->backing_dev_info;
                        if (sizeof (sdev->request_queue->backing_dev_info) == sizeof (struct backing_dev_info)){
                            dst->ra_pages = BUFFER_READ_AHEAD >> (PAGE_SHIFT - 10);
                        }else{
                            struct backing_dev_info **d = (struct backing_dev_info **)dst;
                            (*d)->ra_pages = BUFFER_READ_AHEAD >> (PAGE_SHIFT - 10);
                        }
#else

                        blk_queue_max_hw_sectors(sdev->request_queue, BUFFER_SECTOR);
                        if (queue_has_disk_seh(sdev->request_queue))
                            queue_to_disk_seh(sdev->request_queue)->bdi->ra_pages = BUFFER_READ_AHEAD >> (PAGE_SHIFT - 10);
#endif                         
                        dev_info (&sdev->sdev_gendev, "Set max_sectors to %d",BUFFER_SECTOR);
                        dev_info (&sdev->sdev_gendev, "Set read_ahead_kb to %d k", BUFFER_READ_AHEAD);
                        ret = 0;
                    }
                }
                

            }else if (!strcmp (ifc->dev.driver->name, "uas")){
                   dev_err (&udev->dev, "SEH UTN do not support \"uas\" driver for storage devices.\n"
                 "Possible solution is to add a file ignore_uas.conf inside /etc/modprobe.d \n"
                 "with \"options usb-storage quirks=%x:%x:u\"\n", udev->descriptor.idVendor, udev->descriptor.idProduct); 
                 usb_unlock_device(udev) ;
                 ret = -EPROTO;
	        }else{
                dev_info (&ifc->dev, "Not storage-device");
                ret = 0;
		    }
            
        }
    }
    usb_unlock_device(udev) ;
    return ret;
}

int seh_vusb_boost_storage_devices(void){
    int ret;
    uint32_t hcd_index, rhport;
    struct vhci_device *vdev;
    /*get all devices we have*/
    for (hcd_index = 0; hcd_index < VHCI_CONTROLLERS; hcd_index++){
        for (rhport = 0; rhport < VHCI_NPORTS; rhport++){
            //printk("%s controller[%d] %p port:%d ...\n",__func__,hcd_index,controllers[hcd_index],rhport);
            vdev = port_to_vdev(controllers[hcd_index], rhport);
            if (vdev && vdev->device_status == VDEV_ST_USED){
                if (vdev->buffersHackBoosted < 2){ // we try twice, as we do not kwno when hartbeat will hit
                    ret = seh_vusb_try_set_buffers(vdev->udev); 
                    if (ret == 0){
                        vdev->buffersHackBoosted = 1; // success - no more Trys
                    }
                    else if (ret == -EPROTO){
                        unpluging_device(vdev);
                        vdev->buffersHackBoosted = 1; // no more Trys
                    }
                    vdev->buffersHackBoosted++;
                    
                    return ret;
                    }
                } else{
                    vdev->buffersHackBoosted = 0;
                }
        }
    }
    return 0;
}
