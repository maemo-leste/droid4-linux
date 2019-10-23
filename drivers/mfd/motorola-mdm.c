// SPDX-License-Identifier: GPL-2.0
/*
 * Motorola TS 27.010 serial line discipline serdev driver
 * Copyright (C) 2018 Tony Lindgren <tony@atomide.com>
 */

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/serdev.h>
#include <linux/serdev-gsm.h>

#include <linux/mfd/core.h>
#include <linux/mfd/motorola-mdm.h>
#include <linux/phy/phy.h>

#include <uapi/linux/gsmmux.h>

#define MOTMDM_C_N2		3	/* TS27.010 default value */
#define MOTMDM_DLCI_MIN		1
#define MOTMDM_DLCI_MAX		12
#define MOTMDM_DLCI_MASK	0x1ffe	/* MOTMDM_DLCI1 to 12 */
#define MOTMDM_ID_LEN		5	/* U + unsigned short */
#define MOTMDM_CMD_LEN(x)	(MOTMDM_ID_LEN + (x) + 1)
#define MOTMDM_WRITE_BUF_SIZE	1024
#define MOTMDM_READ_FIFO_SIZE	4096

struct motmdm_cfg {
	unsigned long cdevmask;
	unsigned int aggressive_pm:1;
	int modem_dlci;
	int codec_dlci;
};

struct motmdm {
	struct device *dev;
	struct phy *phy;
	struct gsm_serdev gsd;
	const struct motmdm_cfg *cfg;
	struct class *class;
	struct list_head dlcis;
	struct list_head cdevs;
	dev_t dev_id;
	u16 cmdid;
};

struct motmdm_response {
	struct list_head node;
	u16 id;
	const unsigned char *cmd;
	size_t cmdlen;
	size_t reslen;
	unsigned char *buf;
	size_t len;
	unsigned int handled:1;
};

struct motmdm_cdev {
	struct motmdm *ddata;
	struct list_head node;
	struct motmdm_dlci *dlci;
	struct device *dev;
	struct cdev cdev;
	struct rw_semaphore rwsem;
	unsigned int count;
	unsigned int disconnected:1;
	struct mutex read_mutex;	/* char dev write lock */
	struct mutex write_mutex;	/* char dev read lock */
	char *write_buf;
	size_t write_buf_sz;
	size_t write_offset;
};

static const char * const motmdm_driver_name = "motmdm";

static int motmdm_runtime_suspend(struct device *dev)
{
	struct motmdm *ddata = gsm_serdev_get_drvdata(dev);
	int err;

	if (IS_ERR_OR_NULL(ddata->phy))
		return 0;

	err = phy_pm_runtime_put(ddata->phy);
	if (err < 0)
		dev_warn(dev, "%s: phy_pm_runtime_put: %i\n",
			 __func__, err);

	atomic_set(&ddata->gsd.asleep, 1);

	return 0;
}

static int motmdm_runtime_resume(struct device *dev)
{
	struct motmdm *ddata = gsm_serdev_get_drvdata(dev);
	int err;

	if (IS_ERR_OR_NULL(ddata->phy))
		return 0;

	err = phy_pm_runtime_get_sync(ddata->phy);
	if (err < 0)
		dev_warn(dev, "%s: phy_pm_runtime_get: %i\n",
			 __func__, err);

	atomic_set(&ddata->gsd.asleep, 0);
	gsm_serdev_data_kick(&ddata->gsd);

	return 0;
}

static const struct dev_pm_ops motmdm_pm_ops = {
	SET_RUNTIME_PM_OPS(motmdm_runtime_suspend,
			   motmdm_runtime_resume,
			   NULL)
};

/*
 * Motorola MDM6600 devices have GPIO wake pins shared between the USB PHY and
 * the TS 27.010 interface. So for PM, we need to use the phy_pm_runtime
 * runtime calls. Otherwise the modem won't respond to anything on the UART
 * and will never idle either.
 */
static int motmdm_init_phy(struct device *dev)
{
	struct motmdm *ddata = gsm_serdev_get_drvdata(dev);
	int err;

	ddata->phy = devm_of_phy_get(dev, dev->of_node, NULL);
	if (IS_ERR(ddata->phy)) {
		err = PTR_ERR(ddata->phy);
		if (err != -EPROBE_DEFER)
			dev_err(dev, "%s: no phy: %i\n", __func__, err);

		return err;
	}

	return 0;
}

/*
 * Motorola MDM6600 devices add a custom packet numbering layer on top of the
 * TS 27.010 DLCI channels. This is a layering violation as all the devices
 * are on dedicated channels already. For some reason the packet numbering is
 * not specific to each DLCI.. It is for all the DLCI instead. As both ends
 * can increase packet IDs, conflicts are guaranteed but seem to be harmless.
 * Who knows, maybe the packet IDs were specified by frustrated developers to
 * debug buggy modem code reponding on a wrong DLCI. Valid packet numbers are
 * from 0000 to 9999 decimal. We just parse the modem sent packet number to
 * match the response to sent commands and don't use modem sent packet
 * numbers it for new command packets we send out.
 */
static int motmdm_read_packet_id(struct gsm_serdev_dlci *gsm_dlci,
				 const unsigned char *buf,
				 size_t len)
{
	struct motmdm_dlci *mot_dlci =
		container_of(gsm_dlci, struct motmdm_dlci,
			     gsm_dlci);
	struct motmdm *ddata = mot_dlci->privdata;
	unsigned char tmp[MOTMDM_ID_LEN];
	int err;
	u16 id;

	if (WARN(!ddata, "%s no ddata?\n", __func__))
		return 0;

	if (len < MOTMDM_ID_LEN || buf[0] != 'U')
		return -ECOMM;

	snprintf(tmp, MOTMDM_ID_LEN, "%s", buf + 1);
	err = kstrtou16(tmp, 10, &id);
	if (err)
		return -ECOMM;

	return id;
}

/*
 * For new packets, we just use jiffies based numbering and let the modem
 * deal with any possible numbering conflicts across the DLCI.
 */
static int motmdm_new_packet_id(void)
{
	return jiffies % 10000;
}

/*
 * The modem DLCI1 provides also modem status information. We just forward
 * DLCI1 as a character device to userspace. However in a bit of a layering
 * violation, we also need to parse the modem state from DLCI1 for modem
 * state notifications.
 *
 * The notifications start with a '~' character and are separate from modem
 * commands. So let's try to stick to just parsing the notifications here.
 *
 * This is needed for Alsa ASoC codec driver to reconfigure clocks and codec
 * hardware with set_tdm_slot() for voice calls automatically. And it can be
 * later on used for things like signal strength etc.
 */
static void motmdm_read_state(struct motmdm_dlci *mot_dlci,
			      const unsigned char *buf,
			      size_t len)
{
	struct motmdm *ddata = mot_dlci->privdata;
	struct motmdm_dlci *tmp;
	enum motmdm_state state = MOTMDM_STATE_IDLE;

	switch (len) {
	case 12 + 1:
		if (buf[0] != '~')
			break;
		if (!strncmp(buf, "~+CIEV=1,1,0", 12))
			state = MOTMDM_STATE_CONNECTING;
		if (!strncmp(buf, "~+CIEV=1,4,0", 12))
			state = MOTMDM_STATE_INCOMING;
		else if (!strncmp(buf, "~+CIEV=1,2,0", 12))
			state = MOTMDM_STATE_CONNECTED;
		else if (!strncmp(buf, "~+CIEV=1,0,0", 12))
			state = MOTMDM_STATE_DISCONNECTED;
		break;
	default:
		return;
	}

	if (state == MOTMDM_STATE_IDLE)
		return;

	list_for_each_entry(tmp, &ddata->dlcis, node) {
		if (!tmp->notify)
			continue;

		tmp->notify(tmp, state);
	}
}

/* Fix line breaks for apps if needed and feed kfifo */
static int motmdm_dlci_feed_kfifo(struct motmdm_dlci *mot_dlci,
				  const unsigned char *buf,
				  size_t len)
{
	size_t newlen = len;
	int err;

	if (len && buf[len - 1] == '\n') {
		if (len > 1 && buf[len - 2] != '\r')
			newlen--;
		else if (len == 1)
			newlen--;
	}

	err = kfifo_in(&mot_dlci->read_fifo, buf, newlen);
	if (err != newlen)
		return -ENOSPC;

	if (newlen != len) {
		err = kfifo_in(&mot_dlci->read_fifo, "\r\n", 2);
		if (err != 2)
			err = -ENOSPC;
		else
			newlen += err;
	}

	return newlen;
}

/*
 * Read handling for Motorola custom layering on top of TS 27.010
 */
static int motmdm_dlci_receive_buf(struct gsm_serdev_dlci *gsm_dlci,
				   const unsigned char *buf,
				   size_t len)
{
	struct motmdm_dlci *mot_dlci =
		container_of(gsm_dlci, struct motmdm_dlci,
			     gsm_dlci);
	struct motmdm *ddata = mot_dlci->privdata;
	const unsigned char *msg;
	size_t msglen;
	int id, err;

	if (len < (MOTMDM_ID_LEN + 1) || buf[0] != 'U')
		return 0;

	id = motmdm_read_packet_id(gsm_dlci, buf, len);
	if (id < 0)
		return 0;

	/* Strip out Motorola custom packet numbering */
	msg = buf + MOTMDM_ID_LEN;
	msglen = len - MOTMDM_ID_LEN;

	if (mot_dlci->line == ddata->cfg->modem_dlci)
		motmdm_read_state(mot_dlci, msg, msglen);

	/* Motorola custom data or a command ack? */
	if (buf[MOTMDM_ID_LEN] == '~' && mot_dlci->receive_data)
		mot_dlci->receive_data(mot_dlci, msg + 1, msglen - 1);
	else if (mot_dlci->handle_command)
		mot_dlci->handle_command(mot_dlci, id, msg, msglen);

	if (kfifo_initialized(&mot_dlci->read_fifo)) {
		err = motmdm_dlci_feed_kfifo(mot_dlci, msg, msglen);
		if (err < 0)
			goto err_kfifo;
	}

	err = msglen;

	wake_up(&mot_dlci->read_queue);

err_kfifo:

	return err;
}

/*
 * Helper for child device drivers to send a command to a DLCI and wait
 * for result with a matching packet ID.
 */
static int motmdm_dlci_send_command(struct device *dev,
				    struct motmdm_dlci *mot_dlci,
				    unsigned long timeout_ms,
				    const unsigned char *cmd, size_t cmdlen,
				    unsigned char *rsp, size_t rsplen)
{
	struct motmdm_response *resp, *tmp;
	struct list_head *pos, *q;
	unsigned char *delim;
	int err;

	resp = kzalloc(sizeof(*resp), GFP_KERNEL);
	if (!resp)
		return -ENOMEM;

	memset(rsp, 0, rsplen);

	resp->cmd = cmd;
	resp->cmdlen = cmdlen;
	resp->buf = rsp;
	resp->len = rsplen;
	resp->id = motmdm_new_packet_id();

	/* Firmware will return only the command without value */
	if (cmd[cmdlen - 1] != '=') {
		delim = strchr(cmd, '=');
		if (delim)
			resp->cmdlen -= strlen(delim);
	}

	list_add_tail(&resp->node, &mot_dlci->list);

	err = mot_dlci->write(dev, mot_dlci, resp->id,
				cmd, cmdlen);
	if (err < 0)
		goto unregister;

	err = wait_event_timeout(mot_dlci->read_queue, resp->handled,
				 msecs_to_jiffies(timeout_ms));
	if (err < 0)
		goto unregister;

	if (err == 0) {
		err = -ETIMEDOUT;
		goto unregister;
	}

	dev_dbg(dev, "%s: %s got %s\n", __func__, cmd, resp->buf);

	err = resp->reslen;

unregister:
	list_for_each_safe(pos, q, &mot_dlci->list) {
		tmp = list_entry(pos, struct motmdm_response, node);
		if (tmp->id == resp->id)
			list_del(pos);
	}

	kfree(resp);

	return err;
}

/*
 * Helper for child device drivers to parse the command response from a DLCI
 */
static int motmdm_dlci_handle_command(struct motmdm_dlci *mot_dlci, int id,
				      const unsigned char *buf, size_t len)
{
	struct motmdm_response *resp = NULL;
	struct list_head *pos, *q;
	int resp_start;

	list_for_each_safe(pos, q, &mot_dlci->list) {
		resp = list_entry(pos, struct motmdm_response, node);
		if (resp->id == id)
			break;
	}

	if (!resp || !resp->buf)
		return -ENODEV;

	/* Firmware leaves out AT from the commands */
	resp_start = resp->cmdlen - 2;
	if (len < resp_start)
		return -EPIPE;

	/* Only some firmware messages start with ':' */
	if (buf[resp_start] == ':')
		resp_start++;

	resp->reslen = min3(len - resp_start, resp->len, len);
	strncpy(resp->buf, buf + resp_start, resp->reslen);

	/* Leave out trailing line break if there */
	if (resp->reslen > 1 && resp->buf[resp->reslen - 1] == '\n') {
		resp->buf[resp->reslen - 1] = '\0';
		resp->reslen--;
	}

	resp->handled = true;

	return 0;
}

/*
 * Write handling for Motorola custom layering on top of TS 27.010
 */
static int motmdm_dlci_write(struct device *dev, struct motmdm_dlci *mot_dlci,
			     int cmdid, const unsigned char *buf, size_t count)
{
	struct motmdm *ddata;
	struct gsm_serdev *gsd;
	int err, cmdlen;
	char *cmd;

	if (!dev || !mot_dlci || !buf || !count)
		return -EINVAL;

	ddata = gsm_serdev_get_drvdata(dev);
	if (!ddata)
		return -ENODEV;

	gsd = &ddata->gsd;

	err = pm_runtime_get_sync(dev);
	if ((err != -EINPROGRESS) && err < 0) {
		pm_runtime_put_noidle(dev);

		return err;
	}

	cmdlen = MOTMDM_CMD_LEN(count);
	cmd = kmalloc(cmdlen, GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	switch (cmdid) {
	case -ENOENT:
		/* No ID number, just U for continuation messages */
		snprintf(cmd, cmdlen, "U%s\r", buf);
		break;
	case 0 ... 9999:
		/* Valid ID */
		mot_dlci->id = cmdid;
		snprintf(cmd, cmdlen, "U%04i%s\r", mot_dlci->id, buf);
		break;
	default:
		/* Assign ID */
		mot_dlci->id = motmdm_new_packet_id();
		snprintf(cmd, cmdlen, "U%04i%s\r", mot_dlci->id, buf);
		break;
	}

	err = gsm_serdev_write(gsd, &mot_dlci->gsm_dlci, cmd, cmdlen);
	if (err == cmdlen)
		err = count;

	kfree(cmd);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return err;
}



int motmdm_register_dlci(struct device *dev, struct motmdm_dlci *mot_dlci)
{
	struct motmdm *ddata;
	struct gsm_serdev *gsd;
	struct gsm_serdev_dlci *gsm_dlci;
	int err;

	if (!dev || !mot_dlci || !mot_dlci->line)
		return -EINVAL;

	err = pm_runtime_get_sync(dev);
	if ((err != -EINPROGRESS) && err < 0) {
		pm_runtime_put_noidle(dev);

		return err;
	}

	ddata = gsm_serdev_get_drvdata(dev);
	gsd = &ddata->gsd;
	gsm_dlci = &mot_dlci->gsm_dlci;
	mot_dlci->write = motmdm_dlci_write;
	mot_dlci->send_command = motmdm_dlci_send_command;
	mot_dlci->handle_command = motmdm_dlci_handle_command;
	INIT_LIST_HEAD(&mot_dlci->list);
	init_waitqueue_head(&mot_dlci->read_queue);
	gsm_dlci->line = mot_dlci->line;
	gsm_dlci->receive_buf = motmdm_dlci_receive_buf;

	err = gsm_serdev_register_dlci(gsd, gsm_dlci);
	if (err) {
		dev_warn(dev, "error registering dlci%i: %i\n",
			 mot_dlci->line, err);
		kfifo_free(&mot_dlci->read_fifo);
		memset(gsm_dlci, 0, sizeof(*gsm_dlci));
	} else {
		mot_dlci->privdata = ddata;
	}

	list_add_tail(&mot_dlci->node, &ddata->dlcis);

	pm_runtime_put(dev);

	return err;
}
EXPORT_SYMBOL_GPL(motmdm_register_dlci);

void motmdm_unregister_dlci(struct device *dev, struct motmdm_dlci *mot_dlci)
{
	struct motmdm *ddata;
	struct gsm_serdev *gsd;
	struct gsm_serdev_dlci *gsm_dlci;
	struct list_head *pos, *q;
	struct motmdm_dlci *tmp;
	int err;

	if (!dev || !mot_dlci || !mot_dlci->line)
		return;

	err = pm_runtime_get_sync(dev);
	if ((err != -EINPROGRESS) && err < 0) {
		pm_runtime_put_noidle(dev);
		return;
	}

	ddata = gsm_serdev_get_drvdata(dev);

	list_for_each_safe(pos, q, &ddata->dlcis) {
		tmp = list_entry(pos, struct motmdm_dlci, node);
		if (tmp == mot_dlci)
			list_del(pos);
	}

	gsd = &ddata->gsd;
	gsm_dlci = &mot_dlci->gsm_dlci;
	gsm_serdev_unregister_dlci(gsd, gsm_dlci);
	gsm_dlci->receive_buf = NULL;
	mot_dlci->notify = NULL;
	mot_dlci->privdata = NULL;

	pm_runtime_put(dev);
}
EXPORT_SYMBOL_GPL(motmdm_unregister_dlci);

/*
 * Character devices for DLCI channels with no serdev drivers
 */
static int motmdm_cdev_open(struct inode *inode, struct file *file)
{
	struct motmdm_cdev *cdata;
	int ret = 0;

	cdata = container_of(inode->i_cdev, struct motmdm_cdev, cdev);
	get_device(cdata->dev);
	nonseekable_open(inode, file);
	file->private_data = cdata;

	down_write(&cdata->rwsem);
	if (cdata->disconnected) {
		ret = -ENODEV;
		goto unlock;
	}

unlock:
	up_write(&cdata->rwsem);

	if (ret)
		put_device(cdata->dev);

	return ret;
}

static int motmdm_cdev_release(struct inode *inode, struct file *file)
{
	struct motmdm_cdev *cdata = file->private_data;

	down_write(&cdata->rwsem);
	if (cdata->disconnected)
		goto unlock;

unlock:
	up_write(&cdata->rwsem);

	put_device(cdata->dev);

	return 0;
}

static ssize_t motmdm_cdev_read(struct file *file, char __user *buf,
				size_t count, loff_t *pos)
{
	struct motmdm_cdev *cdata = file->private_data;
	struct motmdm_dlci *mot_dlci = cdata->dlci;
	unsigned int copied;
	int err;

	mutex_lock(&cdata->read_mutex);
	while (kfifo_is_empty(&mot_dlci->read_fifo)) {
		mutex_unlock(&cdata->read_mutex);

		if (cdata->disconnected)
			return 0;

		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		err = wait_event_interruptible(mot_dlci->read_queue,
				cdata->disconnected ||
				!kfifo_is_empty(&mot_dlci->read_fifo));
		if (err)
			return -ERESTARTSYS;

		mutex_lock(&cdata->read_mutex);
	}

	err = kfifo_to_user(&mot_dlci->read_fifo, buf, count, &copied);
	if (err == 0)
		err = copied;

	mutex_unlock(&cdata->read_mutex);

	return err;
}

static ssize_t motmdm_cdev_write_packet(struct motmdm_cdev *cdata, int cmdid)
{
	struct motmdm_dlci *mot_dlci = cdata->dlci;
	struct motmdm *ddata = mot_dlci->privdata;

	return mot_dlci->write(ddata->dev, mot_dlci, cmdid,
			cdata->write_buf, cdata->write_offset - 1);
}

static ssize_t motmdm_cdev_write(struct file *file, const char __user *buf,
				 size_t count, loff_t *pos)
{
	struct motmdm_cdev *cdata = file->private_data;
	size_t written = 0;
	int err, flag = -ENOMSG;

	if (cdata->disconnected)
		return -EIO;

	if (!count)
		return 0;

	err = mutex_lock_interruptible(&cdata->write_mutex);
	if (err)
		return -ERESTARTSYS;

	for (;;) {
		size_t n;
		bool packet = false;

		n = min(count, cdata->write_buf_sz -
			cdata->write_offset - count);
		if (copy_from_user(cdata->write_buf + cdata->write_offset,
				   buf, n)) {
			err = -EFAULT;
			goto out_unlock;
		}

		cdata->write_offset += n;
		cdata->write_buf[cdata->write_offset] = '\0';
		if (cdata->write_offset) {
			u8 last = cdata->write_buf[cdata->write_offset - 1];

			switch (last) {
			case 0x1a:	/* Continuation packets end with ^Z */
				flag = -ENOENT;
				/* Fallthrough */
			case '\n':
			case '\r':
				packet = true;
				break;
			default:
				break;
			}
		}

		down_read(&cdata->rwsem);

		if (cdata->disconnected) {
			err = -EIO;
			goto err_write;
		}

		if (packet) {
			err = motmdm_cdev_write_packet(cdata, flag);
			if (err < 0)
				goto err_write;

			cdata->write_offset = 0;
		}

		err = n;

err_write:
		up_read(&cdata->rwsem);

		if (err < 0)
			break;

		written += err;
		buf += err;

		if (written == count)
			break;
	}

	if (written)
		err = written;

out_unlock:
	mutex_unlock(&cdata->write_mutex);

	return err;
}

static __poll_t motmdm_cdev_poll(struct file *file, poll_table *wait)
{
	struct motmdm_cdev *cdata = file->private_data;
	struct motmdm_dlci *mot_dlci = cdata->dlci;
	__poll_t mask = 0;

	poll_wait(file, &mot_dlci->read_queue, wait);

	if (!kfifo_is_empty(&mot_dlci->read_fifo))
		mask |= EPOLLIN | EPOLLRDNORM;
	if (cdata->write_offset < cdata->write_buf_sz)
		mask |= EPOLLOUT | EPOLLWRNORM;
	if (cdata->disconnected)
		mask |= EPOLLHUP;

	return mask;
}

static const struct file_operations motmdm_fops = {
	.owner		= THIS_MODULE,
	.open		= motmdm_cdev_open,
	.release	= motmdm_cdev_release,
	.read		= motmdm_cdev_read,
	.write		= motmdm_cdev_write,
	.poll		= motmdm_cdev_poll,
	.llseek		= no_llseek,
};

static int motmdm_cdev_init_one(struct motmdm *ddata, int index)
{
	struct motmdm_cdev *cdata;
	struct motmdm_dlci *mot_dlci;
	int err;

	mot_dlci = kzalloc(sizeof(*mot_dlci), GFP_KERNEL);
	if (!mot_dlci)
		return -ENOMEM;

	mot_dlci->drvdata = ddata;
	mot_dlci->line = index;

	err = kfifo_alloc(&mot_dlci->read_fifo, MOTMDM_READ_FIFO_SIZE,
			  GFP_KERNEL);
	if (err)
		goto err_free_dlci;

	cdata = kzalloc(sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		goto err_free_kfifo;

	cdata->dlci = mot_dlci;
	init_rwsem(&cdata->rwsem);
	mutex_init(&cdata->read_mutex);
	mutex_init(&cdata->write_mutex);
	cdata->write_buf_sz = MOTMDM_WRITE_BUF_SIZE;
	cdata->write_buf = kzalloc(cdata->write_buf_sz, GFP_KERNEL);
	if (!cdata->write_buf)
		goto err_free_cdata;

	err = motmdm_register_dlci(ddata->dev, cdata->dlci);
	if (err)
		goto err_free_write_buf;

	cdata->dev = device_create(ddata->class, ddata->dev,
				   MKDEV(MAJOR(ddata->dev_id), index),
				   mot_dlci, "%s%i", motmdm_driver_name,
				   index);
	if (IS_ERR(cdata->dev)) {
		err = PTR_ERR(cdata->dev);
		goto err_unregister;
	}

	cdev_init(&cdata->cdev, &motmdm_fops);
	err = cdev_add(&cdata->cdev, MKDEV(MAJOR(ddata->dev_id), index),
		       MOTMDM_DLCI_MAX);
	if (err)
		goto err_device_destroy;

	list_add_tail(&cdata->node, &ddata->cdevs);

	return 0;

err_device_destroy:
	device_destroy(ddata->class, MKDEV(MAJOR(ddata->dev_id), index));
err_unregister:
	motmdm_unregister_dlci(ddata->dev, cdata->dlci);
err_free_write_buf:
	kfree(cdata->write_buf);
err_free_cdata:
	kfree(cdata);
err_free_kfifo:
	kfifo_free(&mot_dlci->read_fifo);
err_free_dlci:
	kfree(mot_dlci);

	return err;
}

static void motmdm_cdev_cleanup(struct device *dev);

static int motmdm_cdev_init(struct device *dev)
{
	struct motmdm *ddata = gsm_serdev_get_drvdata(dev);
	int bit, err;

	err = alloc_chrdev_region(&ddata->dev_id, 0, MOTMDM_DLCI_MAX,
				  motmdm_driver_name);
	if (err)
		return err;

	ddata->class = class_create(THIS_MODULE, motmdm_driver_name);
	if (IS_ERR(ddata->class)) {
		err = PTR_ERR(ddata->class);
		motmdm_cdev_cleanup(dev);

		return err;
	}

	for_each_set_bit(bit, &ddata->cfg->cdevmask, BITS_PER_LONG) {
		err = motmdm_cdev_init_one(ddata, bit);
		if (err) {
			motmdm_cdev_cleanup(dev);

			return err;
		}
	}

	return 0;
}

static void motmdm_cdev_free_one(struct motmdm_cdev *cdata)
{
	struct motmdm_dlci *mot_dlci = cdata->dlci;
	struct motmdm *ddata = mot_dlci->privdata;

	down_write(&cdata->rwsem);
	cdata->disconnected = true;
	if (cdata->count)
		wake_up(&mot_dlci->read_queue);
	up_write(&cdata->rwsem);

	cdev_del(&cdata->cdev);
	device_destroy(ddata->class,
		       MKDEV(MAJOR(ddata->dev_id), mot_dlci->line));
	kfree(cdata->write_buf);

	motmdm_unregister_dlci(ddata->dev, cdata->dlci);
	kfifo_free(&mot_dlci->read_fifo);
	kfree(cdata->dlci);
	kfree(cdata);
}

static void motmdm_cdev_cleanup(struct device *dev)
{
	struct motmdm *ddata = gsm_serdev_get_drvdata(dev);
	struct motmdm_cdev *cdata, *tmp;

	list_for_each_entry_safe(cdata, tmp, &ddata->cdevs, node) {
		motmdm_cdev_free_one(cdata);
		list_del(&cdata->node);
	}

	class_destroy(ddata->class);

	unregister_chrdev_region(ddata->dev_id, MOTMDM_DLCI_MAX);
}

static int motmdm_check_revision(struct device *dev)
{
	struct motmdm *ddata = gsm_serdev_get_drvdata(dev);
	struct motmdm_dlci *mot_dlci;
	const unsigned char *cmd = "AT+VERSION=";
	unsigned char *buf;
	int retries = 3, err;

	mot_dlci = kzalloc(sizeof(*mot_dlci), GFP_KERNEL);
	if (!mot_dlci)
		return -ENOMEM;

	mot_dlci->drvdata = ddata;
	mot_dlci->line = MOTMDM_DLCI6;

	buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buf) {
		err = -ENOMEM;
		goto free_dlci;
	}

	err = motmdm_register_dlci(dev, mot_dlci);
	if (err)
		goto free_buf;

	while (retries--) {
		err = motmdm_dlci_send_command(dev, mot_dlci, 1000,
					       cmd, strlen(cmd),
					       buf, PAGE_SIZE);
		if (err >= 0) {
			msleep(100);
			break;
		}

		msleep(500);
	}

	if (err < 0) {
		dev_err(dev, "Could not connect: %i\n", err);
	} else if (!strncmp(buf, "ERROR", 5)) {
		dev_err(dev, "Firmware error: %s\n", buf);
		err = -ENODEV;
	} else {
		dev_info(dev, "Firmware: %s\n", buf);
		err = 0;
	}

	motmdm_unregister_dlci(dev, mot_dlci);

free_buf:
	kfree(buf);
free_dlci:
	kfree(mot_dlci);

	return err;
}

static int motmdm_set_config(struct device *dev, int retransmissions)
{
	struct motmdm *ddata = gsm_serdev_get_drvdata(dev);
	struct gsm_serdev *gsd = &ddata->gsd;
	struct gsm_config c;
	int err;

	err = gsm_serdev_get_config(gsd, &c);
	if (err)
		return err;

	c.i = 1;		/* 1 = UIH, 2 = UI */
	c.initiator = 1;
	c.encapsulation = 0;	/* basic mode */
	c.adaption = 1;
	c.mru = 1024;		/* from android TS 27010 driver */
	c.mtu = 1024;		/* from android TS 27010 driver */
	c.t1 = 10;		/* ack timer, default 10ms */
	c.t2 = 34;		/* response timer, default 34 */
	c.n2 = retransmissions;	/* retransmissions, default 3 */

	err = gsm_serdev_set_config(gsd, &c);
	if (err)
		return err;

	return 0;
}

static int motmdm_output(struct gsm_serdev *gsd, u8 *data, int len)
{
	struct serdev_device *serdev = gsd->serdev;
	struct device *dev = &serdev->dev;
	int err;

	err = pm_runtime_get(dev);
	if ((err != -EINPROGRESS) && err < 0) {
		pm_runtime_put_noidle(dev);

		return err;
	}

	serdev_device_write_buf(serdev, data, len);

	pm_runtime_put(dev);

	return len;
}

static const struct motmdm_cfg mapphone_mdm6600_data = {
	.cdevmask = MOTMDM_DLCI_MASK & ~(BIT(MOTMDM_DLCI2) | BIT(MOTMDM_DLCI4)),
	.aggressive_pm = true,
	.modem_dlci = MOTMDM_DLCI1,
	.codec_dlci = MOTMDM_DLCI2,
};

static const struct of_device_id motmdm_id_table[] = {
	{
		.compatible = "motorola,mapphone-mdm6600-serdev",
		.data = &mapphone_mdm6600_data,
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, motmdm_id_table);

static const struct mfd_cell motmdm_mfd_devices[] = {
	{
		.name = "mot-mdm6600-codec",
	},
	{
		.name = "gnss-mot-mdm6600",
		.of_compatible = "motorola,mapphone-mdm6600-gnss",
	},
};

static int motmdm_probe(struct serdev_device *serdev)
{
	struct device *dev = &serdev->dev;
	const struct of_device_id *match;
	struct gsm_serdev *gsd;
	struct motmdm *ddata;
	int err;

	match = of_match_device(of_match_ptr(motmdm_id_table), dev);
	if (!match)
		return -ENODEV;

	ddata = devm_kzalloc(dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	ddata->dev = dev;
	ddata->cfg = match->data;

	INIT_LIST_HEAD(&ddata->dlcis);
	INIT_LIST_HEAD(&ddata->cdevs);
	gsd = &ddata->gsd;
	gsd->serdev = serdev;
	gsd->output = motmdm_output;
	serdev_device_set_drvdata(serdev, gsd);
	gsm_serdev_set_drvdata(dev, ddata);

	err = motmdm_init_phy(dev);
	if (err)
		return err;

	pm_runtime_set_autosuspend_delay(dev, 200);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);
	err = pm_runtime_get_sync(dev);
	if (err < 0) {
		pm_runtime_put_noidle(dev);

		return err;
	}

	err = gsm_serdev_register_device(gsd);
	if (err)
		goto disable;

	err = serdev_device_open(gsd->serdev);
	if (err)
		goto disable;

	serdev_device_set_baudrate(gsd->serdev, 115200);
	serdev_device_set_rts(gsd->serdev, true);
	serdev_device_set_flow_control(gsd->serdev, true);

	/*
	 * Getting dlci0 connected quirk: We set initial retransmissions
	 * value high to get n_gsm to send SABM packets. Then after about
	 * three seconds we'll get a "reassembly overrun" error from firmware
	 * on dlci0 followed by DM(P) packets and then we're connected in ADM
	 * mode. Note we will set the retransmissions back to default value
	 * later on.
	 */
	err = motmdm_set_config(dev, MOTMDM_C_N2 * 10);
	if (err)
		goto close;

	msleep(3000);

	err = motmdm_check_revision(dev);
	if (err)
		goto close;

	msleep(500);

	err = motmdm_cdev_init(dev);
	if (err)
		goto close;

	pm_runtime_put_sync(dev);

	err = devm_mfd_add_devices(dev, 0, motmdm_mfd_devices,
				   ARRAY_SIZE(motmdm_mfd_devices),
				   NULL, 0, NULL);
	if (err)
		goto cdev_cleanup;

	/* Set initial retransmissions back to default value */
	err = motmdm_set_config(dev, MOTMDM_C_N2);
	if (err)
		goto cdev_cleanup;

	if (ddata->cfg->aggressive_pm) {
		/*
		 * Configure SoC 8250 device for 700 ms autosuspend delay, values
		 * around 600 ms and shorter cause spurious wake-up events at least
		 * on droid 4.
		 */
		pm_runtime_set_autosuspend_delay(serdev->ctrl->dev.parent, 700);

		/* Allow parent serdev device to idle when open, balanced in remove*/
		pm_runtime_put(&serdev->ctrl->dev);

		/*
		 * Keep parent SoC 8250 device active during use because of the OOB
		 * GPIO wake-up signaling shared with USB PHY.
		 */
		pm_suspend_ignore_children(&serdev->ctrl->dev, false);
	}

	return 0;

cdev_cleanup:
	motmdm_cdev_cleanup(dev);

close:
	serdev_device_close(serdev);

disable:
	pm_runtime_dont_use_autosuspend(dev);
	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);
	gsm_serdev_unregister_device(gsd);

	return err;
}

static void motmdm_remove(struct serdev_device *serdev)
{
	struct gsm_serdev *gsd = serdev_device_get_drvdata(serdev);
	struct device *dev = &serdev->dev;
	struct motmdm *ddata = gsm_serdev_get_drvdata(dev);
	int err;

	/* Balance the put done in probe for UART */
	if (ddata->cfg->aggressive_pm)
		pm_runtime_get(&serdev->ctrl->dev);

	err = pm_runtime_get_sync(dev);
	if (err < 0)
		dev_warn(dev, "%s: PM runtime: %i\n", __func__, err);

	motmdm_cdev_cleanup(dev);
	serdev_device_close(serdev);
	gsm_serdev_unregister_device(gsd);

	pm_runtime_dont_use_autosuspend(dev);
	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);
}

static struct serdev_device_driver motmdm_driver = {
	.driver = {
		.name = "motmdm",
		.of_match_table = of_match_ptr(motmdm_id_table),
		.pm = &motmdm_pm_ops,
	},
	.probe = motmdm_probe,
	.remove = motmdm_remove,
};

module_serdev_device_driver(motmdm_driver);

MODULE_ALIAS("platform:motorola-mdm");
MODULE_DESCRIPTION("Motorola Modem TS 27.010 serdev driver");
MODULE_AUTHOR("Tony Lindgren <tony@atomide.com");
MODULE_LICENSE("GPL v2");
