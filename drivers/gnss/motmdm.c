// SPDX-License-Identifier: GPL-2.0
/*
 * Motorola Modem TS 27.010 serdev GNSS driver
 *
 * Copyright (C) 2018 Tony Lindgren <tony@atomide.com>
 */

#include <linux/errno.h>
#include <linux/gnss.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/serdev.h>
#include <linux/serdev-gsm.h>
#include <linux/slab.h>
#include <linux/wait.h>

#include <linux/mfd/motorola-mdm.h>

#define MOTMDM_GNSS_TIMEOUT	1000
#define MOTMDM_GNSS_RATE	1000

/*
 * Motorola MDM GNSS device talks AT commands on a dedicated TS 27.010
 * channel.
 */
#define MOTMDM_GNSS_MPD_LEN	4				/* +MPD */
#define MOTMDM_GNSS_STATUS_LEN	(MOTMDM_GNSS_MPD_LEN + 7)	/* STATUS= */
#define MOTMDM_GNSS_NMEA_LEN	(MOTMDM_GNSS_MPD_LEN + 8)	/* NMEA=NN, */

enum motmdm_gnss_status {
	MOTMDM_GNSS_UNKNOWN,
	MOTMDM_GNSS_INITIALIZED,
	MOTMDM_GNSS_DATA_OR_TIMEOUT,
	MOTMDM_GNSS_STARTED,
	MOTMDM_GNSS_STOPPED,
};

struct motmdm_gnss_data {
	struct gnss_device *gdev;
	struct device *modem;
	struct motmdm_dlci mot_dlci;
	struct delayed_work restart_work;
	ktime_t last_update;
	int status;
	unsigned char *buf;
	size_t len;
	unsigned char *dbg;
	size_t dbglen;
};

static unsigned int rate_ms = MOTMDM_GNSS_RATE;
module_param(rate_ms, uint, 0644);
MODULE_PARM_DESC(rate_ms, "GNSS refresh rate between 1000 and 16000 ms (default 1000 ms)");

#ifdef DEBUG
#define MOTMDM_GNSS_DEBUG_BUF_LEN	64
static void motmdm_gnss_debug(struct motmdm_gnss_data *ddata,
				   const unsigned char *buf, size_t len,
				   const char *msg)
{
	struct gnss_device *gdev = ddata->gdev;

	if (!ddata->dbg)
		return;

	memset(ddata->dbg, '\0', ddata->dbglen);
	snprintf(ddata->dbg, min(ddata->dbglen - 1, len), buf);
	dev_dbg(&gdev->dev, "%s: %s\n", msg, ddata->dbg);
}
#else
#define MOTMDM_GNSS_DEBUG_BUF_LEN	0
static void motmdm_gnss_debug(struct motmdm_gnss_data *ddata,
				   const unsigned char *buf, size_t len,
				   const char *msg)
{
}
#endif

/*
 * Android uses AT+MPDSTART=0,1,100,0 which starts GNSS for a while
 * and then GNSS needs to be kicked with an AT command based on a
 * status message.
 */
static void motmdm_gnss_restart(struct work_struct *work)
{
	struct motmdm_gnss_data *ddata =
		container_of(work, struct motmdm_gnss_data,
			     restart_work.work);
	struct motmdm_dlci *mot_dlci = &ddata->mot_dlci;
	struct gnss_device *gdev = ddata->gdev;
	const unsigned char *cmd = "AT+MPDSTART=0,1,100,0";
	int error;

	ddata->last_update = ktime_get();

	error = motmdm_send_command(ddata->modem, mot_dlci,
					 MOTMDM_GNSS_TIMEOUT,
					 cmd, strlen(cmd),
					 ddata->buf, ddata->len);
	if (error < 0) {
		/* Timeouts seem common.. Don't warn and request the data again */
		if (error != -ETIMEDOUT)
			dev_warn(&gdev->dev, "%s: could not start: %i\n",
				 __func__, error);

		schedule_delayed_work(&ddata->restart_work,
				      msecs_to_jiffies(MOTMDM_GNSS_RATE));

		return;
	}
}

static void motmdm_gnss_start(struct gnss_device *gdev, int delay_ms)
{
	struct motmdm_gnss_data *ddata = gnss_get_drvdata(gdev);
	ktime_t now, next, delta;
	int next_ms;

	now = ktime_get();
	next = ktime_add_ms(ddata->last_update, delay_ms);
	delta = ktime_sub(next, now);
	next_ms = ktime_to_ms(delta);

	if (next_ms < 0)
		next_ms = 0;
	if (next_ms > delay_ms)
		next_ms = delay_ms;

	schedule_delayed_work(&ddata->restart_work, msecs_to_jiffies(next_ms));
}

static int motmdm_gnss_stop(struct gnss_device *gdev)
{
	struct motmdm_gnss_data *ddata = gnss_get_drvdata(gdev);
	struct motmdm_dlci *mot_dlci = &ddata->mot_dlci;
	const unsigned char *cmd = "AT+MPDSTOP";

	cancel_delayed_work_sync(&ddata->restart_work);

	return motmdm_send_command(ddata->modem, mot_dlci,
					MOTMDM_GNSS_TIMEOUT,
					cmd, strlen(cmd),
					ddata->buf, ddata->len);
}

static int motmdm_gnss_init(struct gnss_device *gdev)
{
	struct motmdm_gnss_data *ddata = gnss_get_drvdata(gdev);
	struct motmdm_dlci *mot_dlci = &ddata->mot_dlci;
	const unsigned char *cmd = "AT+MPDINIT=1";
	int error;

	error = motmdm_send_command(ddata->modem, mot_dlci,
					 MOTMDM_GNSS_TIMEOUT,
					 cmd, strlen(cmd),
					 ddata->buf, ddata->len);
	if (error < 0)
		return error;

	motmdm_gnss_start(gdev, 0);

	return 0;
}

static int motmdm_gnss_finish(struct gnss_device *gdev)
{
	struct motmdm_gnss_data *ddata = gnss_get_drvdata(gdev);
	struct motmdm_dlci *mot_dlci = &ddata->mot_dlci;
	const unsigned char *cmd = "AT+MPDINIT=0";
	int error;

	error = motmdm_gnss_stop(gdev);
	if (error < 0)
		return error;

	return motmdm_send_command(ddata->modem, mot_dlci,
				   MOTMDM_GNSS_TIMEOUT,
				   cmd, strlen(cmd),
				   ddata->buf, ddata->len);
}

static int motmdm_gnss_receive_data(struct motmdm_dlci *mot_dlci,
					 const unsigned char *buf,
					 size_t len)
{
	struct gnss_device *gdev = mot_dlci->drvdata;
	struct motmdm_gnss_data *ddata = gnss_get_drvdata(gdev);
	const unsigned char *msg;
	size_t msglen;
	int error = 0;

	if (len <= MOTMDM_GNSS_MPD_LEN)
		return 0;

	switch (buf[MOTMDM_GNSS_MPD_LEN]) {
	case 'N':	/* UNNNN~+MPDNMEA=NN, */
		msg = buf + MOTMDM_GNSS_NMEA_LEN;
		msglen = len - MOTMDM_GNSS_NMEA_LEN;

		/*
		 * Firmware bug: Strip out extra duplicate line break always
		 * in the data
		 */
		msglen--;

		/*
		 * Firmware bug: Strip out extra data based on an
		 * earlier line break in the data
		 */
		if (msg[msglen - 5 - 1] == 0x0a)
			msglen -= 5;

		error = gnss_insert_raw(gdev, msg, msglen);
		break;
	case 'S':	/* UNNNN~+MPDSTATUS=N,NN */
		msg = buf + MOTMDM_GNSS_STATUS_LEN;
		msglen = len - MOTMDM_GNSS_STATUS_LEN;
		motmdm_gnss_debug(ddata, msg, msglen, "status");

		switch (msg[0]) {
		case '1':
			ddata->status = MOTMDM_GNSS_INITIALIZED;
			break;
		case '2':
			ddata->status = MOTMDM_GNSS_DATA_OR_TIMEOUT;
			if (rate_ms < MOTMDM_GNSS_RATE)
				rate_ms = MOTMDM_GNSS_RATE;
			if (rate_ms > 16 * MOTMDM_GNSS_RATE)
				rate_ms = 16 * MOTMDM_GNSS_RATE;
			motmdm_gnss_start(gdev, rate_ms);
			break;
		case '3':
			ddata->status = MOTMDM_GNSS_STARTED;
			break;
		case '4':
			ddata->status = MOTMDM_GNSS_STOPPED;
			break;
		default:
			ddata->status = MOTMDM_GNSS_UNKNOWN;
			break;
		}
		break;
	case 'X':	/* What does UNNNN~+MPDXREQ=N mean? */
	default:
		motmdm_gnss_debug(ddata, buf, len, "unhandled");
		break;
	}

	return 0;
}

static int motmdm_gnss_open(struct gnss_device *gdev)
{
	struct motmdm_gnss_data *ddata = gnss_get_drvdata(gdev);
	struct motmdm_dlci *mot_dlci = &ddata->mot_dlci;
	int error;

	mot_dlci->drvdata = gdev;
	mot_dlci->line = MOTMDM_DLCI4;
	mot_dlci->receive_data = motmdm_gnss_receive_data;

	error = motmdm_register_dlci(ddata->modem, mot_dlci);
	if (error)
		return error;

	error = motmdm_gnss_init(gdev);
	if (error) {
		motmdm_unregister_dlci(ddata->modem, mot_dlci);

		return error;
	}

	return 0;
}

static void motmdm_gnss_close(struct gnss_device *gdev)
{
	struct motmdm_gnss_data *ddata = gnss_get_drvdata(gdev);
	struct motmdm_dlci *mot_dlci = &ddata->mot_dlci;
	int error;

	mot_dlci->receive_data = NULL;
	error = motmdm_gnss_finish(gdev);
	if (error < 0)
		dev_warn(&gdev->dev, "%s: close failed: %i\n",
			 __func__, error);

	motmdm_unregister_dlci(ddata->modem, mot_dlci);
}

static int motmdm_gnss_write_raw(struct gnss_device *gdev,
				      const unsigned char *buf,
				      size_t count)
{
	struct motmdm_gnss_data *ddata = gnss_get_drvdata(gdev);
	struct motmdm_dlci *mot_dlci = &ddata->mot_dlci;

	return motmdm_send_command(ddata->modem, mot_dlci,
					MOTMDM_GNSS_TIMEOUT,
					buf, count, ddata->buf, ddata->len);
}

static const struct gnss_operations motmdm_gnss_ops = {
	.open		= motmdm_gnss_open,
	.close		= motmdm_gnss_close,
	.write_raw	= motmdm_gnss_write_raw,
};

static int motmdm_gnss_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct motmdm_gnss_data *ddata;
	struct gnss_device *gdev;
	int ret;

	ddata = devm_kzalloc(dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	INIT_DELAYED_WORK(&ddata->restart_work, motmdm_gnss_restart);
	ddata->modem = dev->parent;
	ddata->len = PAGE_SIZE;
	ddata->dbglen = MOTMDM_GNSS_DEBUG_BUF_LEN;

	ddata->buf = devm_kzalloc(dev, ddata->len, GFP_KERNEL);
	if (!ddata->buf)
		return -ENOMEM;

	if (ddata->dbglen) {
		ddata->dbg = devm_kzalloc(dev, ddata->dbglen, GFP_KERNEL);
		if (!ddata->dbg)
			return -ENOMEM;
	}

	platform_set_drvdata(pdev, ddata);

	gdev = gnss_allocate_device(dev);
	if (!gdev)
		return -ENOMEM;

	gdev->type = GNSS_TYPE_NMEA;
	gdev->ops = &motmdm_gnss_ops;
	gnss_set_drvdata(gdev, ddata);
	ddata->gdev = gdev;

	ret = gnss_register_device(gdev);
	if (ret)
		goto err_put_device;

	return 0;

err_put_device:
	gnss_put_device(ddata->gdev);

	return ret;
}

static int motmdm_gnss_remove(struct platform_device *pdev)
{
	struct motmdm_gnss_data *data = platform_get_drvdata(pdev);

	gnss_deregister_device(data->gdev);
	gnss_put_device(data->gdev);

	return 0;
};

#ifdef CONFIG_OF
static const struct of_device_id motmdm_gnss_of_match[] = {
	{ .compatible = "motorola,mapphone-mdm6600-gnss" },
	{},
};
MODULE_DEVICE_TABLE(of, motmdm_gnss_of_match);
#endif

static struct platform_driver motmdm_gnss_driver = {
	.driver	= {
		.name		= "gnss-mot-mdm6600",
		.of_match_table	= of_match_ptr(motmdm_gnss_of_match),
	},
	.probe	= motmdm_gnss_probe,
	.remove	= motmdm_gnss_remove,
};
module_platform_driver(motmdm_gnss_driver);

MODULE_AUTHOR("Tony Lindgren <tony@atomide.com>");
MODULE_DESCRIPTION("Motorola Mapphone MDM6600 GNSS receiver driver");
MODULE_LICENSE("GPL v2");
