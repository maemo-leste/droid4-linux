// SPDX-License-Identifier: (GPL-2.0 OR MIT)
/*
 * Linux DRM Driver for PoverVR SGX omap variants
 *
 * Copyright (C) 2019 Tony Lindgren <tony@atomide.com>
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/pm.h>

#include <drm/drm_file.h>
#include <drm/drm_drv.h>
#include <drm/omap_drm.h>

#include "pvr-drv.h"

static struct omap_drm_plugin plugin;

static void pvr_omap4_release(struct drm_device *dev, struct drm_file *file)
{
	PVRSRVRelease(file->driver_priv);

	file->driver_priv = NULL;
}

int pvr_quirk_omap4_init(struct device *dev, struct drm_device *ddev)
{
	int error;

	plugin.dev = ddev;
	plugin.name = dev->driver->name;
	plugin.open = ddev->driver->open;
	plugin.release = pvr_omap4_release;
	plugin.ioctls = ddev->driver->ioctls;
	plugin.num_ioctls = ddev->driver->num_ioctls;

	error = omap_drm_register_plugin(&plugin);
	if (error)
		pr_err("%s: omap_drm_register_plugin failed: %i", __func__, error);

	return error;
}

void pvr_quirk_omap4_cleanup(void)
{
	int error;

	error = omap_drm_unregister_plugin(&plugin);
	if (error)
		pr_err("%s: failed: %i\n", __func__, error);
}
