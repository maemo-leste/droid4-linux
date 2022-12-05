// SPDX-License-Identifier: GPL-2.0-only
/*
 * Generic DSI Command Mode panel driver
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - https://www.ti.com/
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_connector.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#include <video/mipi_display.h>

#define DCS_GET_ID1		0xda
#define DCS_GET_ID2		0xdb
#define DCS_GET_ID3		0xdc

#define DCS_REGULATOR_SUPPLY_NUM 2

static const struct of_device_id dsicm_of_match[];

struct dsic_panel_data {
	u32 xres;
	u32 yres;
	u32 refresh;
	u32 width_mm;
	u32 height_mm;
	u32 max_hs_rate;
	u32 max_lp_rate;
	bool te_support;
	bool amoled_bl;
};

struct panel_drv_data {
	struct mipi_dsi_device *dsi;
	struct drm_panel panel;
	struct drm_display_mode mode;

	struct mutex lock;

	struct backlight_device *bldev;
	struct backlight_device *extbldev;

	unsigned long	hw_guard_end;	/* next value of jiffies when we can
					 * issue the next sleep in/out command
					 */
	unsigned long	hw_guard_wait;	/* max guard time in jiffies */

	const struct dsic_panel_data *panel_data;

	struct gpio_desc *reset_gpio;

	struct regulator_bulk_data supplies[DCS_REGULATOR_SUPPLY_NUM];

	u8 vendor;			/* Panel manufacturer ID */
	u8 controller;			/* Panel controller version */
	u8 driver;			/* Panel controller driver version */

	bool use_dsi_backlight;

	/* runtime variables */
	bool enabled;

	bool intro_printed;
};

static inline struct panel_drv_data *panel_to_ddata(struct drm_panel *panel)
{
	return container_of(panel, struct panel_drv_data, panel);
}

static void dsicm_bl_power(struct panel_drv_data *ddata, bool enable)
{
	struct backlight_device *backlight;

	if (ddata->bldev)
		backlight = ddata->bldev;
	else if (ddata->extbldev)
		backlight = ddata->extbldev;
	else
		return;

	if (enable)
		backlight_enable(backlight);
	else
		backlight_disable(backlight);
}

static void hw_guard_start(struct panel_drv_data *ddata, int guard_msec)
{
	ddata->hw_guard_wait = msecs_to_jiffies(guard_msec);
	ddata->hw_guard_end = jiffies + ddata->hw_guard_wait;
}

static void hw_guard_wait(struct panel_drv_data *ddata)
{
	unsigned long wait = ddata->hw_guard_end - jiffies;

	if ((long)wait > 0 && wait <= ddata->hw_guard_wait) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(wait);
	}
}

static int dsicm_dcs_read_1(struct panel_drv_data *ddata, u8 dcs_cmd, u8 *data)
{
	return mipi_dsi_dcs_read(ddata->dsi, dcs_cmd, data, 1);
}

static int dsicm_dcs_write_1(struct panel_drv_data *ddata, u8 dcs_cmd, u8 param)
{
	return mipi_dsi_dcs_write(ddata->dsi, dcs_cmd, &param, 1);
}

static int dsicm_sleep_in(struct panel_drv_data *ddata)

{
	int r;

	hw_guard_wait(ddata);

	r = mipi_dsi_dcs_enter_sleep_mode(ddata->dsi);
	if (r)
		return r;

	hw_guard_start(ddata, 120);

	usleep_range(5000, 10000);

	return 0;
}

static int dsicm_sleep_out(struct panel_drv_data *ddata)
{
	int r;

	hw_guard_wait(ddata);

	r = mipi_dsi_dcs_exit_sleep_mode(ddata->dsi);
	if (r)
		return r;

	hw_guard_start(ddata, 120);

	usleep_range(5000, 10000);

	return 0;
}

static int dsicm_init_id(struct panel_drv_data *ddata)
{
	int r;

	if (ddata->vendor || ddata->controller || ddata->driver)
		return 0;

	r = dsicm_dcs_read_1(ddata, DCS_GET_ID1, &ddata->vendor);
	if (r)
		return r;
	r = dsicm_dcs_read_1(ddata, DCS_GET_ID2, &ddata->controller);
	if (r)
		return r;
	r = dsicm_dcs_read_1(ddata, DCS_GET_ID3, &ddata->driver);
	if (r)
		return r;

	return 0;
}

static int dsicm_set_update_window(struct panel_drv_data *ddata, int x_offset)
{
	struct mipi_dsi_device *dsi = ddata->dsi;
	int r;

	r = mipi_dsi_dcs_set_column_address(dsi, x_offset,
					    ddata->mode.hdisplay + x_offset - 1);
	if (r < 0)
		return r;

	r = mipi_dsi_dcs_set_page_address(dsi, 0, ddata->mode.vdisplay - 1);
	if (r < 0)
		return r;

	return 0;
}

static int dsicm_amoled_set_backlight(struct panel_drv_data *ddata, u8 level);

static int dsicm_bl_update_status(struct backlight_device *dev)
{
	struct panel_drv_data *ddata = dev_get_drvdata(&dev->dev);
	int r = 0;
	int level = backlight_get_brightness(dev);

	dev_dbg(&ddata->dsi->dev, "update brightness to %d\n", level);

	mutex_lock(&ddata->lock);

	if (!ddata->enabled)
		goto out_unlock;

	if (ddata->panel_data->amoled_bl)
		r = dsicm_amoled_set_backlight(ddata, level);
	else
		r = dsicm_dcs_write_1(ddata, MIPI_DCS_SET_DISPLAY_BRIGHTNESS,
				      level);

out_unlock:
	mutex_unlock(&ddata->lock);

	return r;
}

static int dsicm_bl_get_intensity(struct backlight_device *dev)
{
	return backlight_get_brightness(dev);
}

static const struct backlight_ops dsicm_bl_ops = {
	.get_brightness = dsicm_bl_get_intensity,
	.update_status  = dsicm_bl_update_status,
};

static ssize_t num_dsi_errors_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	u8 errors = 0;
	int r = -ENODEV;

	mutex_lock(&ddata->lock);

	if (ddata->enabled)
		r = dsicm_dcs_read_1(ddata, MIPI_DCS_GET_ERROR_COUNT_ON_DSI, &errors);

	mutex_unlock(&ddata->lock);

	if (r)
		return r;

	return sysfs_emit(buf, "%d\n", errors);
}

static ssize_t hw_revision_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%02x.%02x.%02x\n",
			  ddata->vendor, ddata->controller, ddata->driver);
}

static DEVICE_ATTR_RO(num_dsi_errors);
static DEVICE_ATTR_RO(hw_revision);

static struct attribute *dsicm_attrs[] = {
	&dev_attr_num_dsi_errors.attr,
	&dev_attr_hw_revision.attr,
	NULL,
};

static const struct attribute_group dsicm_attr_group = {
	.attrs = dsicm_attrs,
};

static void dsicm_hw_reset(struct panel_drv_data *ddata)
{
	gpiod_set_value(ddata->reset_gpio, 1);
	udelay(10);
	/* reset the panel */
	gpiod_set_value(ddata->reset_gpio, 0);
	/* assert reset */
	udelay(10);
	gpiod_set_value(ddata->reset_gpio, 1);
	/* wait after releasing reset */
	usleep_range(5000, 10000);
}

/*
 * Amoled panel backlight settings with gamma, taken from Motorola Android
 * kernel panel-mapphone.c.
 */
static u8 amoled_bl_data_cs_03[][26] = {
	{0xfa, 0x02, 0x10, 0x10, 0x10, 0xec, 0x9e, 0xee, 0xd5, 0x93,
	 0xc7, 0xe3, 0xc1, 0xde, 0xb9, 0x96, 0xc1, 0xd9, 0xcb, 0xde,
	 0x00, 0x44, 0x00, 0x2c, 0x00, 0x4d}, /* 10 nits */
	{0xfa, 0x02, 0x10, 0x10, 0x10, 0xec, 0x9e, 0xee, 0xd5, 0x93,
	 0xc7, 0xe3, 0xc1, 0xde, 0xb9, 0x9e, 0xc1, 0xd7, 0xcf, 0xda,
	 0x00, 0x51, 0x00, 0x39, 0x00, 0x5a}, /* 20 nits */
	{0xfa, 0x02, 0x10, 0x10, 0x10, 0xec, 0xb4, 0xee, 0xd5, 0x97,
	 0xc7, 0xe3, 0xc1, 0xde, 0xb9, 0xa5, 0xc3, 0xd5, 0xd0, 0xd4,
	 0x00, 0x5b, 0x00, 0x42, 0x00, 0x64}, /* 30 nits */
	{0xfa, 0x02, 0x10, 0x10, 0x10, 0xec, 0xb4, 0xee, 0xd5, 0x97,
	 0xc7, 0xe3, 0xd1, 0xde, 0xb8, 0xac, 0xc3, 0xd3, 0xd0, 0xcf,
	 0x00, 0x6a, 0x00, 0x50, 0x00, 0x76}, /* 50 nits */
	{0xfa, 0x02, 0x10, 0x10, 0x10, 0xec, 0xb4, 0xee, 0xd6, 0xa6,
	 0xc7, 0xe4, 0xd6, 0xe1, 0xb6, 0xaf, 0xbf, 0xd1, 0xd0, 0xcd,
	 0x00, 0x74, 0x00, 0x59, 0x00, 0x83}, /* 70 nits */
	{0xfa, 0x02, 0x10, 0x10, 0x10, 0xec, 0xb4, 0xee, 0xd7, 0xb3,
	 0xc8, 0xe4, 0xd7, 0xe4, 0xb6, 0xb0, 0xbc, 0xcf, 0xcf, 0xcc,
	 0x00, 0x7a, 0x00, 0x5e, 0x00, 0x89}, /* 80 nits */
	{0xfa, 0x02, 0x10, 0x10, 0x10, 0xec, 0xb4, 0xee, 0xd7, 0xb6,
	 0xc9, 0xe3, 0xd7, 0xe5, 0xb5, 0xb2, 0xbb, 0xce, 0xce, 0xcb,
	 0x00, 0x7f, 0x00, 0x62, 0x00, 0x8e}, /* 90 nits */
	{0xfa, 0x02, 0x10, 0x10, 0x10, 0xec, 0xb4, 0xee, 0xd4, 0xb5,
	 0xc6, 0xe4, 0xd8, 0xe5, 0xb4, 0xb1, 0xb9, 0xcd, 0xcd, 0xc9,
	 0x00, 0x83, 0x00, 0x65, 0x00, 0x93}, /* 100 nits */
	{0xfa, 0x02, 0x10, 0x10, 0x10, 0xec, 0xb4, 0xee, 0xd5, 0xb9,
	 0xc7, 0xe4, 0xd9, 0xe6, 0xb2, 0xb2, 0xb7, 0xcd, 0xcc, 0xc7,
	 0x00, 0x87, 0x00, 0x68, 0x00, 0x98}, /* 110 nits */
	{0xfa, 0x02, 0x10, 0x10, 0x10, 0xec, 0xb4, 0xee, 0xd5, 0xbb,
	 0xc7, 0xe4, 0xd9, 0xe6, 0xb2, 0xb1, 0xb6, 0xcd, 0xcc, 0xc7,
	 0x00, 0x8a, 0x00, 0x6c, 0x00, 0x9c}, /* 120 nits */
	{0xfa, 0x02, 0x10, 0x10, 0x10, 0xec, 0xb4, 0xee, 0xd5, 0xbc,
	 0xcb, 0xe4, 0xd9, 0xe8, 0xb2, 0xb1, 0xb2, 0xcc, 0xcc, 0xc5,
	 0x00, 0x8e, 0x00, 0x6f, 0x00, 0xa2}, /* 130 nits */
	{0xfa, 0x02, 0x10, 0x10, 0x10, 0xec, 0xb4, 0xee, 0xd6, 0xbc,
	 0xcd, 0xe2, 0xdb, 0xe5, 0xb2, 0xb1, 0xb2, 0xcc, 0xcb, 0xc5,
	 0x00, 0x91, 0x00, 0x72, 0x00, 0xa6}, /* 140 nits */
	{0xfa, 0x02, 0x10, 0x10, 0x10, 0xec, 0xb4, 0xee, 0xd4, 0xbf,
	 0xcc, 0xe3, 0xdb, 0xe5, 0xb1, 0xb1, 0xb1, 0xca, 0xcb, 0xc5,
	 0x00, 0x95, 0x00, 0x74, 0x00, 0xa9}, /* 150 nits */
	{0xfa, 0x02, 0x10, 0x10, 0x10, 0xec, 0xb4, 0xee, 0xd5, 0xc1,
	 0xcc, 0xe3, 0xdc, 0xe5, 0xb1, 0xb0, 0xb0, 0xc9, 0xcb, 0xc3,
	 0x00, 0x98, 0x00, 0x77, 0x00, 0xae}, /* 160 nits */
	{0xfa, 0x02, 0x10, 0x10, 0x10, 0xec, 0xb4, 0xee, 0xd6, 0xc2,
	 0xcf, 0xe2, 0xdb, 0xe4, 0xb0, 0xb0, 0xaf, 0xc9, 0xca, 0xc2,
	 0x00, 0x9b, 0x00, 0x7a, 0x00, 0xb2}, /* 170 nits */
	{0xfa, 0x02, 0x10, 0x10, 0x10, 0xec, 0xb4, 0xee, 0xd5, 0xc2,
	 0xcd, 0xe2, 0xdc, 0xe4, 0xb0, 0xb0, 0xae, 0xc8, 0xca, 0xc2,
	 0x00, 0x9e, 0x00, 0x7c, 0x00, 0xb5}, /* 180 nits */
	{0xfa, 0x02, 0x10, 0x10, 0x10, 0xec, 0xb4, 0xee, 0xd4, 0xc3,
	 0xcf, 0xe3, 0xdb, 0xe4, 0xaf, 0xb0, 0xad, 0xc9, 0xc9, 0xc2,
	 0x00, 0xa0, 0x00, 0x7f, 0x00, 0xb8}, /* 190 nits */
	{0xfa, 0x02, 0x10, 0x10, 0x10, 0xec, 0xb4, 0xee, 0xd4, 0xc7,
	 0xd1, 0xe2, 0xdb, 0xe2, 0xae, 0xb1, 0xad, 0xc8, 0xc8, 0xc0,
	 0x00, 0xa3, 0x00, 0x81, 0x00, 0xbc}, /* 200 nits */
	{0xfa, 0x02, 0x10, 0x10, 0x10, 0xec, 0xb4, 0xee, 0xd3, 0xc4,
	 0xd0, 0xe3, 0xdd, 0xe3, 0xae, 0xb0, 0xac, 0xc7, 0xc8, 0xbe,
	 0x00, 0xa6, 0x00, 0x83, 0x00, 0xc0}, /* 210 nits */
	{0xfa, 0x02, 0x10, 0x10, 0x10, 0xec, 0xb4, 0xee, 0xd3, 0xc5,
	 0xd2, 0xe2, 0xdd, 0xe2, 0xae, 0xaf, 0xa9, 0xc5, 0xc8, 0xbe,
	 0x00, 0xac, 0x00, 0x87, 0x00, 0xc7}, /* 230 nits */
	{0xfa, 0x02, 0x10, 0x10, 0x10, 0xec, 0xb4, 0xee, 0xd3, 0xc7,
	 0xd2, 0xe1, 0xdd, 0xe1, 0xae, 0xaf, 0xa9, 0xc5, 0xc6, 0xbe,
	 0x00, 0xaf, 0x00, 0x8b, 0x00, 0xcb}, /* 250 nits */
	{0xfa, 0x02, 0x10, 0x10, 0x10, 0xec, 0xb4, 0xee, 0xd2, 0xc8,
	 0xd2, 0xe1, 0xde, 0xe0, 0xac, 0xaf, 0xa8, 0xc5, 0xc5, 0xbb,
	 0x00, 0xb4, 0x00, 0x8f, 0x00, 0xd3}, /* 270 nits */
	{0xfa, 0x02, 0x10, 0x10, 0x10, 0xec, 0xb4, 0xee, 0xd3, 0xc9,
	 0xd3, 0xe1, 0xdd, 0xdf, 0xac, 0xaf, 0xa7, 0xc3, 0xc6, 0xbb,
	 0x00, 0xb8, 0x00, 0x91, 0x00, 0xd7}, /* 290 nits */
	{0xfa, 0x02, 0x10, 0x10, 0x10, 0xec, 0xb4, 0xee, 0xd4, 0xc9,
	 0xd6, 0xe1, 0xde, 0xde, 0xac, 0xaf, 0xa7, 0xc3, 0xc5, 0xbb,
	 0x00, 0xb9, 0x00, 0x93, 0x00, 0xd9} /* 300 nits */
};

static int dsicm_amoled_set_backlight(struct panel_drv_data *ddata, u8 level)
{
	const int max_brightness = 255;
	u8 (*bl_data)[26];
	int num_bl_steps;
	int index;
	int range;
	int r = 0;

	if (!ddata->controller) {
		dev_err(&ddata->dsi->dev, "unsupported controller version\n");
		return -ENODEV;
	}

	/* All of the p1c gamma settings */
	bl_data = amoled_bl_data_cs_03;
	num_bl_steps = (sizeof(amoled_bl_data_cs_03) /
			sizeof(amoled_bl_data_cs_03[0]));

	range = (max_brightness / num_bl_steps) + 1;
	index = level / range;
	if (index >= num_bl_steps)
		index = num_bl_steps - 1;

	r = mipi_dsi_dcs_write_buffer(ddata->dsi, bl_data[index],
				      sizeof(bl_data[index]));
	if (r)
		goto err;

	/* Gamma set update enable */
	r = dsicm_dcs_write_1(ddata, 0xfa, 0x03);
	if (r)
		goto err;

	return 0;

err:
	dev_info(&ddata->dsi->dev, "failed to set backlight: index: %d r: %d\n",
		 index, r);
	return r;
}

static int dsicm_amoled_set_acl(struct panel_drv_data *ddata, bool enable)
{
	return dsicm_dcs_write_1(ddata, 0xc0, enable);
}

static int dsicm_amoled_power_on(struct panel_drv_data *ddata)
{
	u8 buf[30];
	int r;

	dsicm_hw_reset(ddata);

	ddata->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	r = dsicm_sleep_out(ddata);
	if (r)
		goto err;

	r = dsicm_init_id(ddata);
	if (r)
		goto err;

	/* ETC condition set 1 */
	buf[0] = 0xf0;
	buf[1] = 0x5a;
	buf[2] = 0x5a;
	r = mipi_dsi_dcs_write_buffer(ddata->dsi, buf, 3);
	if (r)
		goto err;

	buf[0] = 0xf1;
	buf[1] = 0x5a;
	buf[2] = 0x5a;
	r = mipi_dsi_dcs_write_buffer(ddata->dsi, buf, 3);
	if (r)
		goto err;

	buf[0] = 0xfc;
	buf[1] = 0x5a;
	buf[2] = 0x5a;
	r = mipi_dsi_dcs_write_buffer(ddata->dsi, buf, 3);
	if (r)
		goto err;

	/* Gamma condition set */
	r = dsicm_amoled_set_backlight(ddata, 255);
	if (r)
		goto err;

	/* Panel Condition set */
	buf[0]  = 0xf8;
	buf[1]  = 0x27;
	buf[2]  = 0x27;
	buf[3]  = 0x08;
	buf[4]  = 0x08;
	buf[5]  = 0x4e;
	buf[6]  = 0xaa;
	buf[7]  = 0x5e;
	buf[8]  = 0x8a;
	buf[9]  = 0x10;
	buf[10] = 0x3f;
	buf[11] = 0x10;
	buf[12] = 0x10;
	buf[13] = 0x00;
	r = mipi_dsi_dcs_write_buffer(ddata->dsi, buf, 14);
	if (r)
		goto err;

	/* ETC condition set 2 */
	buf[0]  = 0xf6;
	buf[1]  = 0x00;
	buf[2]  = 0x84;
	buf[3]  = 0x09;
	r = mipi_dsi_dcs_write_buffer(ddata->dsi, buf, 4);
	if (r)
		goto err;

	/* Set the starting address of the parameter of next coming command */
	r = dsicm_dcs_write_1(ddata, 0xb0, 0x09);
	if (r)
		goto err;

	r = dsicm_dcs_write_1(ddata, 0xd5, 0x64);
	if (r)
		goto err;

	r = dsicm_dcs_write_1(ddata, 0xb0, 0x0b);
	if (r)
		goto err;

	buf[0]  = 0xd5;
	buf[1]  = 0xa4;
	buf[2]  = 0x7e;
	buf[3]  = 0x20;
	r = mipi_dsi_dcs_write_buffer(ddata->dsi, buf, 4);
	if (r)
		goto err;

	r = dsicm_dcs_write_1(ddata, 0xb0, 0x08);
	if (r)
		goto err;

	r = dsicm_dcs_write_1(ddata, 0xfd, 0xf8);
	if (r)
		goto err;

	r = dsicm_dcs_write_1(ddata, 0xb0, 0x04);
	if (r)
		goto err;

	r = dsicm_dcs_write_1(ddata, 0xf2, 0x4d);
	if (r)
		goto err;

	r = dsicm_dcs_write_1(ddata, 0xb0, 0x05);
	if (r)
		goto err;

	r = dsicm_dcs_write_1(ddata, 0xfd, 0x1f);
	if (r)
		goto err;

	buf[0]  = 0xb1;
	buf[1]  = 0x01;
	buf[2]  = 0x00;
	buf[3]  = 0x16;
	r = mipi_dsi_dcs_write_buffer(ddata->dsi, buf, 4);
	if (r)
		goto err;

	buf[0]  = 0xb2;
	buf[1]  = 0x06;
	buf[2]  = 0x06;
	buf[3]  = 0x06;
	buf[4]  = 0x06;
	r = mipi_dsi_dcs_write_buffer(ddata->dsi, buf, 5);
	if (r)
		goto err;

	r = dsicm_sleep_out(ddata);
	if (r)
		goto err;

	msleep(120);

	r = dsicm_dcs_write_1(ddata, 0xd1, 0x8a);
	if (r)
		goto err;

	r = dsicm_amoled_set_acl(ddata, true);
	if (r)
		goto err;

	/* 70% ACL */
	buf[0]  = 0xc1;
	buf[1]  = 0x47;
	buf[2]  = 0x53;
	buf[3]  = 0x13;
	buf[4]  = 0x53;
	buf[5]  = 0x00;
	buf[6]  = 0x00;
	buf[7]  = 0x01;
	buf[8]  = 0xdf;
	buf[9]  = 0x00;
	buf[10] = 0x00;
	buf[11] = 0x03;
	buf[12] = 0x1f;
	buf[13] = 0x00;
	buf[14] = 0x00;
	buf[15] = 0x00;
	buf[16] = 0x00;
	buf[17] = 0x00;
	buf[18] = 0x01;
	buf[19] = 0x02;
	buf[20] = 0x03;
	buf[21] = 0x07;
	buf[22] = 0x0e;
	buf[23] = 0x14;
	buf[24] = 0x1c;
	buf[25] = 0x24;
	buf[26] = 0x2d;
	buf[27] = 0x2d;
	buf[28] = 0x00;
	r = mipi_dsi_dcs_write_buffer(ddata->dsi, buf, 29);
	if (r)
		goto err;

	r = mipi_dsi_dcs_set_pixel_format(ddata->dsi, MIPI_DCS_PIXEL_FMT_24BIT);
	if (r)
		goto err;

	r = dsicm_set_update_window(ddata, 30);
	if (r)
		goto err;

	r = mipi_dsi_dcs_set_display_on(ddata->dsi);
	if (r)
		goto err;

	ddata->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
	ddata->enabled = 1;

	return 0;

err:
	dev_err(&ddata->dsi->dev, "error while enabling panel, issuing HW reset\n");

	dsicm_hw_reset(ddata);

	return r;
}

static int dsicm_power_on(struct panel_drv_data *ddata)
{
	int r;

	dsicm_hw_reset(ddata);

	ddata->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	dsi_mipi_cm_400_540_960_m2_v1_panel_enable_1(ddata->dsi);

	r = dsicm_sleep_out(ddata);
	if (r)
		goto err;

	r = dsicm_init_id(ddata);
	if (r)
		goto err;

	r = dsicm_dcs_write_1(ddata, MIPI_DCS_SET_DISPLAY_BRIGHTNESS, 0xff);
	if (r)
		goto err;

	r = dsicm_dcs_write_1(ddata, MIPI_DCS_WRITE_CONTROL_DISPLAY,
			(1<<2) | (1<<5));	/* BL | BCTRL */
	if (r)
		goto err;

	r = mipi_dsi_dcs_set_pixel_format(ddata->dsi, MIPI_DCS_PIXEL_FMT_24BIT);
	if (r)
		goto err;

	r = dsicm_set_update_window(ddata, 0);
	if (r)
		goto err;

	r = mipi_dsi_dcs_set_display_on(ddata->dsi);
	if (r)
		goto err;

	dsi_mipi_cm_400_540_960_m2_v1_panel_enable_2(ddata->dsi);

	if (ddata->panel_data->te_support) {
		r = mipi_dsi_dcs_set_tear_on(ddata->dsi,
					MIPI_DSI_DCS_TEAR_MODE_VBLANK);
		if (r)
			goto err;
		if (ddata->panel_data->te_scan_line) {
			r = mipi_dsi_dcs_set_tear_scanline(ddata->dsi,
					ddata->panel_data->te_scan_line);
			if (r)
				goto err;
		}
	}

	/* possible panel bug */
	msleep(100);

	ddata->enabled = true;

	if (!ddata->intro_printed) {
		dev_info(&ddata->dsi->dev, "panel revision %02x.%02x.%02x\n",
			ddata->vendor, ddata->controller, ddata->driver);
		ddata->intro_printed = true;
	}

	ddata->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	return 0;
err:
	dev_err(&ddata->dsi->dev, "error while enabling panel, issuing HW reset\n");

	dsicm_hw_reset(ddata);

	return r;
}

static int dsicm_power_off(struct panel_drv_data *ddata)
{
	int r;

	ddata->enabled = false;

	r = mipi_dsi_dcs_set_display_off(ddata->dsi);
	if (!r)
		r = dsicm_sleep_in(ddata);

	if (r) {
		dev_err(&ddata->dsi->dev,
				"error disabling panel, issuing HW reset\n");
		dsicm_hw_reset(ddata);
	}

	return r;
}

static int dsicm_prepare(struct drm_panel *panel)
{
	struct panel_drv_data *ddata = panel_to_ddata(panel);
	int r;

	r = regulator_bulk_enable(ARRAY_SIZE(ddata->supplies), ddata->supplies);
	if (r)
		dev_err(&ddata->dsi->dev, "failed to enable supplies: %d\n", r);

	return r;
}

static int dsicm_enable(struct drm_panel *panel)
{
	struct panel_drv_data *ddata = panel_to_ddata(panel);
	int r;

	mutex_lock(&ddata->lock);

	if (ddata->panel_data->amoled_bl)
		r = dsicm_amoled_power_on(ddata);
	else
		r = dsicm_power_on(ddata);
	if (r)
		goto err;

	mutex_unlock(&ddata->lock);

	dsicm_bl_power(ddata, true);

	return 0;
err:
	dev_err(&ddata->dsi->dev, "enable failed (%d)\n", r);
	mutex_unlock(&ddata->lock);
	return r;
}

static int dsicm_unprepare(struct drm_panel *panel)
{
	struct panel_drv_data *ddata = panel_to_ddata(panel);
	int r;

	r = regulator_bulk_disable(ARRAY_SIZE(ddata->supplies), ddata->supplies);
	if (r)
		dev_err(&ddata->dsi->dev, "failed to disable supplies: %d\n", r);

	return r;
}

static int dsicm_disable(struct drm_panel *panel)
{
	struct panel_drv_data *ddata = panel_to_ddata(panel);
	int r;

	dsicm_bl_power(ddata, false);

	mutex_lock(&ddata->lock);

	r = dsicm_power_off(ddata);

	mutex_unlock(&ddata->lock);

	return r;
}

static int dsicm_get_modes(struct drm_panel *panel,
			   struct drm_connector *connector)
{
	struct panel_drv_data *ddata = panel_to_ddata(panel);
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &ddata->mode);
	if (!mode) {
		dev_err(&ddata->dsi->dev, "failed to add mode %ux%ux@%u kHz\n",
			ddata->mode.hdisplay, ddata->mode.vdisplay,
			ddata->mode.clock);
		return -ENOMEM;
	}

	connector->display_info.width_mm = ddata->panel_data->width_mm;
	connector->display_info.height_mm = ddata->panel_data->height_mm;

	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs dsicm_panel_funcs = {
	.unprepare = dsicm_unprepare,
	.disable = dsicm_disable,
	.prepare = dsicm_prepare,
	.enable = dsicm_enable,
	.get_modes = dsicm_get_modes,
};

static int dsicm_probe_of(struct mipi_dsi_device *dsi)
{
	struct backlight_device *backlight;
	struct panel_drv_data *ddata = mipi_dsi_get_drvdata(dsi);
	int err;
	struct drm_display_mode *mode = &ddata->mode;

	ddata->reset_gpio = devm_gpiod_get(&dsi->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ddata->reset_gpio)) {
		err = PTR_ERR(ddata->reset_gpio);
		dev_err(&dsi->dev, "reset gpio request failed: %d", err);
		return err;
	}

	mode->hdisplay = mode->hsync_start = mode->hsync_end = mode->htotal =
		ddata->panel_data->xres;
	mode->vdisplay = mode->vsync_start = mode->vsync_end = mode->vtotal =
		ddata->panel_data->yres;
	mode->clock = ddata->panel_data->xres * ddata->panel_data->yres *
		ddata->panel_data->refresh / 1000;
	mode->width_mm = ddata->panel_data->width_mm;
	mode->height_mm = ddata->panel_data->height_mm;
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_set_name(mode);

	ddata->supplies[0].supply = "vpnl";
	ddata->supplies[1].supply = "vddi";
	err = devm_regulator_bulk_get(&dsi->dev, ARRAY_SIZE(ddata->supplies),
				      ddata->supplies);
	if (err)
		return err;

	backlight = devm_of_find_backlight(&dsi->dev);
	if (IS_ERR(backlight))
		return PTR_ERR(backlight);

	/* If no backlight device is found assume native backlight support */
	if (backlight)
		ddata->extbldev = backlight;
	else
		ddata->use_dsi_backlight = true;

	return 0;
}

static int dsicm_probe(struct mipi_dsi_device *dsi)
{
	struct panel_drv_data *ddata;
	struct backlight_device *bldev = NULL;
	struct device *dev = &dsi->dev;
	int r;

	dev_dbg(dev, "probe\n");

	ddata = devm_kzalloc(dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ddata);
	ddata->dsi = dsi;

	ddata->panel_data = of_device_get_match_data(dev);
	if (!ddata->panel_data)
		return -ENODEV;

	r = dsicm_probe_of(dsi);
	if (r)
		return r;

	mutex_init(&ddata->lock);

	dsicm_hw_reset(ddata);

	drm_panel_init(&ddata->panel, dev, &dsicm_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	if (ddata->use_dsi_backlight) {
		struct backlight_properties props = { 0 };
		props.max_brightness = 255;
		props.type = BACKLIGHT_RAW;

		bldev = devm_backlight_device_register(dev, dev_name(dev),
			dev, ddata, &dsicm_bl_ops, &props);
		if (IS_ERR(bldev)) {
			r = PTR_ERR(bldev);
			goto err_bl;
		}

		ddata->bldev = bldev;
	}

	r = sysfs_create_group(&dev->kobj, &dsicm_attr_group);
	if (r) {
		dev_err(dev, "failed to create sysfs files\n");
		goto err_bl;
	}

	dsi->lanes = 2;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS |
			  MIPI_DSI_MODE_NO_EOT_PACKET;
	dsi->hs_rate = ddata->panel_data->max_hs_rate;
	dsi->lp_rate = ddata->panel_data->max_lp_rate;

	drm_panel_add(&ddata->panel);

	r = mipi_dsi_attach(dsi);
	if (r < 0)
		goto err_dsi_attach;

	return 0;

err_dsi_attach:
	drm_panel_remove(&ddata->panel);
	sysfs_remove_group(&dsi->dev.kobj, &dsicm_attr_group);
err_bl:
	if (ddata->extbldev)
		put_device(&ddata->extbldev->dev);

	return r;
}

static void dsicm_remove(struct mipi_dsi_device *dsi)
{
	struct panel_drv_data *ddata = mipi_dsi_get_drvdata(dsi);

	dev_dbg(&dsi->dev, "remove\n");

	mipi_dsi_detach(dsi);

	drm_panel_remove(&ddata->panel);

	sysfs_remove_group(&dsi->dev.kobj, &dsicm_attr_group);

	if (ddata->extbldev)
		put_device(&ddata->extbldev->dev);
}

static const struct dsic_panel_data taal_data = {
	.xres = 864,
	.yres = 480,
	.refresh = 60,
	.width_mm = 0,
	.height_mm = 0,
	.max_hs_rate = 300000000,
	.max_lp_rate = 10000000,
	.te_support = true,
};

static const struct dsic_panel_data himalaya_data = {
	.xres = 480,
	.yres = 864,
	.refresh = 60,
	.width_mm = 49,
	.height_mm = 88,
	.max_hs_rate = 300000000,
	.max_lp_rate = 10000000,
	.te_support = false,
};

static const struct dsic_panel_data droid4_data = {
	.xres = 540,
	.yres = 960,
	.refresh = 60,
	.width_mm = 50,
	.height_mm = 89,
	.max_hs_rate = 300000000,
	.max_lp_rate = 10000000,
	.te_support = true,
	.te_scan_line = 300,
};

static const struct dsic_panel_data razr_xt9xx_data = {
	.xres = 540,
	.yres = 960,
	.refresh = 60,
	.width_mm = 53,
	.height_mm = 95,
	.max_hs_rate = 300000000,
	.max_lp_rate = 10000000,
	.te_support = true,
	.amoled_bl = true,
};

static const struct of_device_id dsicm_of_match[] = {
	{ .compatible = "tpo,taal", .data = &taal_data },
	{ .compatible = "nokia,himalaya", &himalaya_data },
	{ .compatible = "motorola,droid4-panel", &droid4_data },
	{ .compatible = "motorola,razr-xt910-panel", &razr_xt9xx_data },
	{},
};

MODULE_DEVICE_TABLE(of, dsicm_of_match);

static struct mipi_dsi_driver dsicm_driver = {
	.probe = dsicm_probe,
	.remove = dsicm_remove,
	.driver = {
		.name = "panel-dsi-cm",
		.of_match_table = dsicm_of_match,
	},
};
module_mipi_dsi_driver(dsicm_driver);

MODULE_AUTHOR("Tomi Valkeinen <tomi.valkeinen@ti.com>");
MODULE_DESCRIPTION("Generic DSI Command Mode Panel Driver");
MODULE_LICENSE("GPL");
