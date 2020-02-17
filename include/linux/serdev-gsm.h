/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _LINUX_SERDEV_GSM_H
#define _LINUX_SERDEV_GSM_H

#include <linux/device.h>
#include <linux/serdev.h>
#include <linux/types.h>

struct gsm_serdev_dlci;
struct gsm_config;

/**
 * struct gsm_serdev - serdev-gsm instance
 * @serdev:		serdev instance
 * @gsm:		ts 27.010 n_gsm instance
 * @asleep:		device is in idle state
 * @drvdata:		serdev-gsm consumer driver data
 * @get_config:		get ts 27.010 config
 * @set_config:		set ts 27.010 config
 * @register_dlci:	register ts 27.010 channel
 * @unregister_dlci:	unregister ts 27.010 channel
 * @output:		read data from ts 27.010 channel
 * @write:		write data to a ts 27.010 channel
 * @kick:		indicate more data is ready
 *
 * Currently only serdev and output must be initialized, the rest are
 * are initialized by gsm_serdev_register_dlci().
 */
struct gsm_serdev {
	struct serdev_device *serdev;
	struct gsm_mux *gsm;
	atomic_t asleep;
	void *drvdata;
	int (*get_config)(struct gsm_serdev *gsd, struct gsm_config *c);
	int (*set_config)(struct gsm_serdev *gsd, struct gsm_config *c);
	int (*register_dlci)(struct gsm_serdev *gsd,
			     struct gsm_serdev_dlci *ops);
	void (*unregister_dlci)(struct gsm_serdev *gsd,
				struct gsm_serdev_dlci *ops);
	int (*output)(struct gsm_serdev *gsd, u8 *data, int len);
	int (*write)(struct gsm_serdev *gsd, struct gsm_serdev_dlci *ops,
		     const u8 *buf, int len);
	void (*kick)(struct gsm_serdev *gsd);
};

/**
 * struct gsm_serdev_dlci - serdev-gsm ts 27.010 channel data
 * @line:		ts 27.010 channel, control channel 0 is not available
 * @receive_buf:	function to handle data received for the channel
 */
struct gsm_serdev_dlci {
	int line;
	int (*receive_buf)(struct gsm_serdev_dlci *ops,
			   const unsigned char *buf,
			   size_t len);
};

#ifdef CONFIG_SERIAL_DEV_BUS

int gsm_serdev_register_device(struct gsm_serdev *gsd);
void gsm_serdev_unregister_device(struct gsm_serdev *gsd);

static inline void *gsm_serdev_get_drvdata(struct device *dev)
{
	struct serdev_device *serdev = to_serdev_device(dev);
	struct gsm_serdev *gsd = serdev_device_get_drvdata(serdev);

	if (gsd)
		return gsd->drvdata;

	return NULL;
}

static inline void gsm_serdev_set_drvdata(struct device *dev, void *drvdata)
{
	struct serdev_device *serdev = to_serdev_device(dev);
	struct gsm_serdev *gsd = serdev_device_get_drvdata(serdev);

	if (gsd)
		gsd->drvdata = drvdata;
}

/**
 * gsm_serdev_get_config - read ts 27.010 config
 * @gsd:	serdev-gsm instance
 * @c:		ts 27.010 config data
 *
 * See gsm_copy_config_values() for more information.
 */
static inline
int gsm_serdev_get_config(struct gsm_serdev *gsd, struct gsm_config *c)
{
	return gsd->get_config(gsd, c);
}

/**
 * gsm_serdev_set_config - set ts 27.010 config
 * @gsd:	serdev-gsm instance
 * @c:		ts 27.010 config data
 *
 * See gsm_config() for more information.
 */
static inline
int gsm_serdev_set_config(struct gsm_serdev *gsd, struct gsm_config *c)
{
	if (gsd && gsd->set_config)
		return gsd->set_config(gsd, c);

	return -ENODEV;
}

/**
 * gsm_serdev_register_dlci - register a ts 27.010 channel
 * @gsd:	serdev-gsm instance
 * @ops:	channel ops
 */
static inline
int gsm_serdev_register_dlci(struct gsm_serdev *gsd,
			     struct gsm_serdev_dlci *ops)
{
	if (gsd && gsd->register_dlci)
		return gsd->register_dlci(gsd, ops);

	return -ENODEV;
}

/**
 * gsm_serdev_unregister_dlci - unregister a ts 27.010 channel
 * @gsd:	serdev-gsm instance
 * @ops:	channel ops
 */
static inline
void gsm_serdev_unregister_dlci(struct gsm_serdev *gsd,
				struct gsm_serdev_dlci *ops)
{
	if (gsd && gsd->unregister_dlci)
		gsd->unregister_dlci(gsd, ops);
}

/**
 * gsm_serdev_write - write data to a ts 27.010 channel
 * @gsd:	serdev-gsm instance
 * @ops:	channel ops
 * @buf:	write buffer
 * @len:	buffer length
 */
static inline
int gsm_serdev_write(struct gsm_serdev *gsd, struct gsm_serdev_dlci *ops,
		     const u8 *buf, int len)
{
	if (gsd && gsd->write)
		return gsd->write(gsd, ops, buf, len);

	return -ENODEV;
}

/**
 * gsm_serdev_data_kick - indicate more data can be trasmitted
 * @gsd:	serdev-gsm instance
 *
 * See gsm_data_kick() for more information.
 */
static inline
void gsm_serdev_data_kick(struct gsm_serdev *gsd)
{
	if (gsd && gsd->kick)
		gsd->kick(gsd);
}

#else	/* CONFIG_SERIAL_DEV_BUS */

static inline
int gsm_serdev_register_device(struct gsm_serdev *gsd)
{
	return -ENODEV;
}

static inline
void gsm_serdev_unregister_device(struct gsm_serdev *gsd)
{
}

static inline void *gsm_serdev_get_drvdata(struct device *dev)
{
	return NULL;
}

static inline
void gsm_serdev_set_drvdata(struct device *dev, void *drvdata)
{
}

static inline
int gsm_serdev_get_config(struct gsm_serdev *gsd, struct gsm_config *c)
{
	return -ENODEV;
}

static inline
int gsm_serdev_set_config(struct gsm_serdev *gsd, struct gsm_config *c)
{
	return -ENODEV;
}

static inline
int gsm_serdev_register_dlci(struct gsm_serdev *gsd,
			     struct gsm_serdev_dlci *ops)
{
	return -ENODEV;
}

static inline
void gsm_serdev_unregister_dlci(struct gsm_serdev *gsd,
				struct gsm_serdev_dlci *ops)
{
}

static inline
int gsm_serdev_write(struct gsm_serdev *gsd, struct gsm_serdev_dlci *ops,
		     const u8 *buf, int len)
{
	return -ENODEV;
}

static inline
void gsm_serdev_data_kick(struct gsm_serdev *gsd)
{
}

#endif	/* CONFIG_SERIAL_DEV_BUS */
#endif	/* _LINUX_SERDEV_GSM_H */
