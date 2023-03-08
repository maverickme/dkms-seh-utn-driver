/*
 * Copyright (C) 2013-2014 SEH Computertechnik GmbH
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __SEH_EVENT_H
#define __SEH_EVENT_H
void unplug_device (struct vhci_device *vdev);
void unpluging_device (struct vhci_device *vdev);
void vhci_device_reset(struct vhci_device *vdev);
#endif
