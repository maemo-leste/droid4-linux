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
 * @drvdata:		serdev-gsm consumer driver data
 * @output:		read data from ts 27.010 channel
 *
 * Currently only serdev and output must be initialized, the rest are
 * are initialized by gsm_serdev_register_dlci().
 */
struct gsm_serdev {
	struct serdev_device *serdev;
	struct gsm_mux *gsm;
	void *drvdata;
	int (*output)(struct gsm_serdev *gsd, u8 *data, int len);
};

/**
 * struct gsm_serdev_dlci - serdev-gsm ts 27.010 channel data
 * @gsd:		serdev-gsm instance
 * @line:		ts 27.010 channel, control channel 0 is not available
 * @receive_buf:	function to handle data received for the channel
 * @drvdata:		dlci specific consumer driver data
 */
struct gsm_serdev_dlci {
	struct gsm_serdev *gsd;
	int line;
	int (*receive_buf)(struct gsm_serdev_dlci *ops,
			   const unsigned char *buf,
			   size_t len);
	void *drvdata;
};

#if IS_ENABLED(CONFIG_N_GSM) && IS_ENABLED(CONFIG_SERIAL_DEV_BUS)

extern int gsm_serdev_register_device(struct gsm_serdev *gsd);
extern void gsm_serdev_unregister_device(struct gsm_serdev *gsd);
extern int gsm_serdev_register_tty_port(struct gsm_serdev *gsd, int line);
extern void gsm_serdev_unregister_tty_port(struct gsm_serdev *gsd, int line);

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

extern int gsm_serdev_get_config(struct gsm_serdev *gsd, struct gsm_config *c);
extern int gsm_serdev_set_config(struct gsm_serdev *gsd, struct gsm_config *c);
extern int
gsm_serdev_register_dlci(struct gsm_serdev *gsd, struct gsm_serdev_dlci *ops);
extern void
gsm_serdev_unregister_dlci(struct gsm_serdev *gsd, struct gsm_serdev_dlci *ops);
extern int gsm_serdev_write(struct gsm_serdev *gsd, struct gsm_serdev_dlci *ops,
			    const u8 *buf, int len);
extern void gsm_serdev_data_kick(struct gsm_serdev *gsd);

#else	/* CONFIG_SERIAL_DEV_BUS */

static inline
int gsm_serdev_register_device(struct gsm_serdev *gsd)
{
	return -ENODEV;
}

static inline void gsm_serdev_unregister_device(struct gsm_serdev *gsd)
{
}

static inline int
gsm_serdev_register_tty_port(struct gsm_serdev *gsd, int line)
{
	return -ENODEV;
}

static inline
void gsm_serdev_unregister_tty_port(struct gsm_serdev *gsd, int line)
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

#endif	/* CONFIG_N_GSM && CONFIG_SERIAL_DEV_BUS */
#endif	/* _LINUX_SERDEV_GSM_H */
