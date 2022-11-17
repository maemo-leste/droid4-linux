// SPDX-License-Identifier: GPL-2.0
/*
 * Motorola Modem TS 27.010 serdev GNSS driver
 *
 * Copyright (C) 2018 - 2020 Tony Lindgren <tony@atomide.com>
 *
 * Based on drivers/gnss/sirf.c driver example:
 * Copyright (C) 2018 Johan Hovold <johan@kernel.org>
 */

#include <linux/errno.h>
#include <linux/gnss.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/serdev-gsm.h>
#include <linux/slab.h>

#define MOTMDM_GNSS_TIMEOUT	1000
#define MOTMDM_GNSS_RATE	1000

/*
 * Motorola MDM GNSS device communicates over a dedicated TS 27.010 channel
 * using custom data packets. The packets look like AT commands embedded into
 * a Motorola invented packet using format like "U1234AT+MPDSTART=0,1,100,0".
 * But it's not an AT compatible serial interface, it's a packet interface
 * using AT like commands.
 */
#define MOTMDM_GNSS_HEADER_LEN	5				/* U1234 */
#define MOTMDM_GNSS_RESP_LEN	(MOTMDM_GNSS_HEADER_LEN + 4)	/* U1234+MPD */
#define MOTMDM_GNSS_DATA_LEN	(MOTMDM_GNSS_RESP_LEN + 1)	/* U1234~+MPD */
#define MOTMDM_GNSS_STATUS_LEN	(MOTMDM_GNSS_DATA_LEN + 7)	/* STATUS= */
#define MOTMDM_GNSS_NMEA_LEN	(MOTMDM_GNSS_DATA_LEN + 8)	/* NMEA=NN, */

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
	struct gsm_serdev_dlci dlci;
	struct delayed_work restart_work;
	struct mutex mutex;	/* For modem commands */
	ktime_t last_update;
	int status;
	unsigned char *buf;
	size_t len;
	wait_queue_head_t read_queue;
	unsigned int parsed:1;
	unsigned int active:1;
};

static unsigned int rate_ms = MOTMDM_GNSS_RATE;
module_param(rate_ms, uint, 0644);
MODULE_PARM_DESC(rate_ms, "GNSS refresh rate between 1000 and 16000 ms (default 1000 ms)");

/*
 * Note that multiple commands can be sent in series with responses coming
 * out-of-order. For GNSS, we don't need to care about the out-of-order
 * responses, and can assume we have at most one command active at a time.
 * For the commands, can use just a jiffies base packet ID and let the modem
 * sort out the ID conflicts with the modem's unsolicited message ID
 * numbering.
 */
int motmdm_gnss_send_command(struct motmdm_gnss_data *ddata,
			     const u8 *buf, int len)
{
	struct gnss_device *gdev = ddata->gdev;
	const int timeout_ms = 1000;
	unsigned char cmd[128];
	int ret, cmdlen;

	cmdlen = len + 5 + 1;
	if (cmdlen > 128)
		return -EINVAL;

	mutex_lock(&ddata->mutex);
	memset(ddata->buf, 0, ddata->len);
	ddata->parsed = false;
	snprintf(cmd, cmdlen, "U%04li%s", jiffies % 10000, buf);
	ret = serdev_ngsm_write(ddata->modem, &ddata->dlci, cmd, cmdlen);
	if (ret < 0)
		goto out_unlock;

	ret = wait_event_timeout(ddata->read_queue, ddata->parsed,
				 msecs_to_jiffies(timeout_ms));
	if (ret == 0) {
		ret = -ETIMEDOUT;
		goto out_unlock;
	} else if (ret < 0) {
		goto out_unlock;
	}

	if (!strstr(ddata->buf, ":OK")) {
		dev_err(&gdev->dev, "command %s error %s\n",
			cmd, ddata->buf);
		ret = -EPIPE;
	}

	ret = len;

out_unlock:
	mutex_unlock(&ddata->mutex);

	return ret;
}

/*
 * Android uses AT+MPDSTART=0,1,100,0 which starts GNSS for a while,
 * and then GNSS needs to be kicked with an AT command based on a
 * status message.
 */
static void motmdm_gnss_restart(struct work_struct *work)
{
	struct motmdm_gnss_data *ddata =
		container_of(work, struct motmdm_gnss_data,
			     restart_work.work);
	struct gnss_device *gdev = ddata->gdev;
	const unsigned char *cmd = "AT+MPDSTART=0,1,100,0";
	int error;

	if (!ddata->active)
		return;

	ddata->last_update = ktime_get();

	error = motmdm_gnss_send_command(ddata, cmd, strlen(cmd));
	if (error < 0) {
		/* Timeouts can happen, don't warn and try again */
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

	ddata->active = 1;
	schedule_delayed_work(&ddata->restart_work, msecs_to_jiffies(next_ms));
}

static int motmdm_gnss_stop(struct gnss_device *gdev)
{
	struct motmdm_gnss_data *ddata = gnss_get_drvdata(gdev);
	const unsigned char *cmd = "AT+MPDSTOP";
	int error;

	ddata->active = 0;
	cancel_delayed_work_sync(&ddata->restart_work);

	error = motmdm_gnss_send_command(ddata, cmd, strlen(cmd));
	if (error < 0)
		return error;

	return 0;
}

static int motmdm_gnss_init(struct gnss_device *gdev)
{
	struct motmdm_gnss_data *ddata = gnss_get_drvdata(gdev);
	const unsigned char *cmd = "AT+MPDINIT=1";
	int error;

	error = motmdm_gnss_send_command(ddata, cmd, strlen(cmd));
	if (error < 0)
		return error;

	ddata->active = 1;
	motmdm_gnss_start(gdev, 0);

	return 0;
}

static int motmdm_gnss_finish(struct gnss_device *gdev)
{
	struct motmdm_gnss_data *ddata = gnss_get_drvdata(gdev);
	const unsigned char *cmd = "AT+MPDINIT=0";
	int error;

	ddata->active = 0;
	error = motmdm_gnss_stop(gdev);
	if (error < 0)
		return error;

	error = motmdm_gnss_send_command(ddata, cmd, strlen(cmd));
	if (error < 0)
		return error;

	return 0;
}

static int motmdm_gnss_receive_data(struct gsm_serdev_dlci *dlci,
				    const unsigned char *buf,
				    size_t len)
{
	struct gnss_device *gdev = dlci->drvdata;
	struct motmdm_gnss_data *ddata = gnss_get_drvdata(gdev);
	const unsigned char *msg;
	size_t msglen;
	int error = 0;

	if (len <= MOTMDM_GNSS_RESP_LEN)
		return 0;

	/* Handle U1234+MPD style command response */
	if (buf[MOTMDM_GNSS_HEADER_LEN] != '~') {
		msg = buf + MOTMDM_GNSS_RESP_LEN;
		strncpy(ddata->buf, msg, len - MOTMDM_GNSS_RESP_LEN);
		ddata->parsed = true;
		wake_up(&ddata->read_queue);

		return len;
	}

	if (len <= MOTMDM_GNSS_DATA_LEN)
		return 0;

	/* Handle U1234~+MPD style unsolicted message */
	switch (buf[MOTMDM_GNSS_DATA_LEN]) {
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

		switch (msg[0]) {
		case '1':
			ddata->status = MOTMDM_GNSS_INITIALIZED;
			break;
		case '2':
			if (!ddata->active)
				break;
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
	case 'X':	/* UNNNN~+MPDXREQ=N for updated xtra2.bin needed */
	default:
		break;
	}

	return len;
}

static int motmdm_gnss_open(struct gnss_device *gdev)
{
	struct motmdm_gnss_data *ddata = gnss_get_drvdata(gdev);
	struct gsm_serdev_dlci *dlci = &ddata->dlci;
	int error;

	dlci->drvdata = gdev;
	dlci->receive_buf = motmdm_gnss_receive_data;

	error = serdev_ngsm_register_dlci(ddata->modem, dlci);
	if (error)
		return error;

	error = motmdm_gnss_init(gdev);
	if (error) {
		serdev_ngsm_unregister_dlci(ddata->modem, dlci);

		return error;
	}

	return 0;
}

static void motmdm_gnss_close(struct gnss_device *gdev)
{
	struct motmdm_gnss_data *ddata = gnss_get_drvdata(gdev);
	struct gsm_serdev_dlci *dlci = &ddata->dlci;
	int error;

	error = motmdm_gnss_finish(gdev);
	if (error < 0)
		dev_warn(&gdev->dev, "%s: close failed: %i\n",
			 __func__, error);

	dlci->receive_buf = NULL;
	serdev_ngsm_unregister_dlci(ddata->modem, dlci);
}

static int motmdm_gnss_write_raw(struct gnss_device *gdev,
				 const unsigned char *buf,
				 size_t count)
{
	struct motmdm_gnss_data *ddata = gnss_get_drvdata(gdev);

	return serdev_ngsm_write(ddata->modem, &ddata->dlci, buf, count);
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
	u32 line;
	int ret;

	ddata = devm_kzalloc(dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	ret = of_property_read_u32(dev->of_node, "reg", &line);
	if (ret)
		return ret;

	if (!line)
		return -EINVAL;

	ddata->dlci.line = line;
	ddata->modem = dev->parent;
	ddata->len = PAGE_SIZE;
	mutex_init(&ddata->mutex);
	INIT_DELAYED_WORK(&ddata->restart_work, motmdm_gnss_restart);
	init_waitqueue_head(&ddata->read_queue);

	ddata->buf = devm_kzalloc(dev, ddata->len, GFP_KERNEL);
	if (!ddata->buf)
		return -ENOMEM;

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
