// SPDX-License-Identifier: GPL-2.0-only
/*
 * Motorola TI MSP430 sensor hub driver
 *
 * The Motorola mapphone RAZR (XT910/XT912) has no individual sensor chips on
 * the OMAP I2C bus 4. Instead, all sensors (accelerometer, gyroscope,
 * magnetometer, pressure, temperature) are attached to a Texas Instruments
 * MSP430 microcontroller at I2C address 0x48, which acts as a sensor hub and
 * exposes aggregated data over a custom register protocol. The hub raises an
 * interrupt on a GPIO when new sensor data is available.
 *
 * This is a clean IIO re-implementation of the old Android 3.0.8
 * drivers/misc/msp430.c driver, using modern kernel interfaces (devm, DT
 * GPIO/regulator API, threaded IRQ, IIO triggered buffer).
 *
 * Copyright (C) 2010 Motorola, Inc.
 * Copyright (C) 2026 Merlijn Wajer <merlijn@wizzup.org>
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#define MSP430_DRV_NAME			"msp430"

#define MSP430_I2C_RETRIES		5
#define MSP430_I2C_RETRY_DELAY_MS	5

/* MSP430 register map */
#define MSP430_REG_REV_ID		0x01	/* firmware version, 1 byte */

#define MSP430_REG_ACCEL_UPDATE_RATE	0x16	/* accel poll interval */
#define MSP430_REG_MAG_UPDATE_RATE	0x17	/* magn poll interval */
#define MSP430_REG_PRESSURE_UPDATE_RATE	0x18	/* pressure poll interval */
#define MSP430_REG_GYRO_UPDATE_RATE	0x19	/* gyro poll interval */
#define MSP430_REG_MODULE_CONFIG	0x1A	/* sensor enable bitmask */

#define MSP430_REG_INTERRUPT_STATUS	0x3A	/* 1 byte bitmask */

#define MSP430_REG_ACCEL_X		0x3B	/* 6 bytes = 3x s16 BE */
#define MSP430_REG_TEMPERATURE		0x41	/* 2 bytes s16 BE */
#define MSP430_REG_GYRO_X		0x43	/* 6 bytes = 3x s16 BE */
#define MSP430_REG_MAG_HX		0x49	/* 6 bytes = 3x s16 BE */
#define MSP430_REG_CURRENT_PRESSURE	0x56	/* 2 bytes s16 BE */
#define MSP430_REG_CURRENT_ALTITUDE	0x58	/* 2 bytes s16 BE */

/* Interrupt status / MODULE_CONFIG bits */
#define MSP430_INT_ACCEL		BIT(0)
#define MSP430_INT_GYRO			BIT(1)
#define MSP430_INT_PRESSURE		BIT(2)
#define MSP430_INT_ECOMPASS		BIT(3)
#define MSP430_INT_TEMPERATURE		BIT(4)
#define MSP430_INT_TAP_SENSING		BIT(5)
#define MSP430_INT_PEDOMETER		BIT(6)
#define MSP430_INT_ACTIVITY_CHANGE	BIT(7)

/* All traditional (non-algorithm) sensors */
#define MSP430_SENSOR_MASK		(MSP430_INT_ACCEL | MSP430_INT_GYRO | \
					 MSP430_INT_PRESSURE | \
					 MSP430_INT_ECOMPASS | \
					 MSP430_INT_TEMPERATURE)

enum msp430_sensor {
	MSP430_ACCEL,
	MSP430_GYRO,
	MSP430_MAGN,
	MSP430_TEMP,
	MSP430_PRESSURE,
	MSP430_NUM_SENSORS,
};

struct msp430_data {
	struct i2c_client *client;
	struct iio_dev *indio_dev;
	struct iio_trigger *trig;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *test_gpio;
	struct mutex lock;

	/* Ensure the scan buffer is naturally aligned for the timestamp */
	struct {
		s16 accel[3];
		s16 gyro[3];
		s16 magn[3];
		s16 temp;
		s16 pressure;
		aligned_s64 timestamp;
	} scan;

	u8 enabled_mask;	/* MODULE_CONFIG value */
	int scale[MSP430_NUM_SENSORS][2];
	int version;		/* REV_ID */
	int64_t timestamp;
	bool buffer_on;
};

/* Nominal per-LSB scale factors; placeholders to be calibrated against the
 * physical part, writable via sysfs so no recompile is needed.
 * Format: IIO_VAL_INT_PLUS_MICRO (val, val2).
 */
static const int msp430_default_scale[MSP430_NUM_SENSORS][2] = {
	[MSP430_ACCEL]		= { 0, 1000 },	/* 0.001 */
	[MSP430_GYRO]		= { 0, 1000 },	/* 0.001 */
	[MSP430_MAGN]		= { 0, 1000 },	/* 0.001 */
	[MSP430_TEMP]		= { 0, 1000 },	/* 0.001 */
	[MSP430_PRESSURE]	= { 0, 1000 },	/* 0.001 */
};

/*
 * I2C helpers.
 *
 * The MSP430 uses a write-then-read protocol: write the 1-byte register
 * address, then read back N bytes. This must be a single transaction with a
 * repeated start, so we use i2c_transfer() with two messages like the old
 * driver did.
 */
static int msp430_read_block(struct msp430_data *data, u8 reg, u8 *buf, int len)
{
	struct i2c_client *client = data->client;
	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};
	int ret, tries;

	for (tries = 0; tries < MSP430_I2C_RETRIES; tries++) {
		ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
		if (ret == ARRAY_SIZE(msgs))
			return 0;
		msleep(MSP430_I2C_RETRY_DELAY_MS);
	}

	dev_err(&client->dev, "read of reg 0x%02x failed (%d)\n", reg, ret);
	return -EIO;
}

static int msp430_read_s16(struct msp430_data *data, u8 reg, s16 *val)
{
	__be16 raw;
	int ret;

	ret = msp430_read_block(data, reg, (u8 *)&raw, sizeof(raw));
	if (ret)
		return ret;

	*val = (s16)be16_to_cpu(raw);
	return 0;
}

static int msp430_write_reg(struct msp430_data *data, u8 reg, u8 val)
{
	struct i2c_client *client = data->client;
	u8 buf[2] = { reg, val };
	int ret, tries;

	for (tries = 0; tries < MSP430_I2C_RETRIES; tries++) {
		ret = i2c_master_send(client, buf, sizeof(buf));
		if (ret == sizeof(buf))
			return 0;
		msleep(MSP430_I2C_RETRY_DELAY_MS);
	}

	dev_err(&client->dev, "write of reg 0x%02x failed (%d)\n", reg, ret);
	return -EIO;
}

/* Bring the hub out of reset into normal operating mode. */
static int msp430_set_normal_mode(struct msp430_data *data)
{
	/* Test line low selects normal mode (high selects bootloader) */
	if (data->test_gpio)
		gpiod_set_value_cansleep(data->test_gpio, 0);

	if (!data->reset_gpio)
		return 0;

	/* Pulse the (active low) reset line, ending deasserted */
	gpiod_set_value_cansleep(data->reset_gpio, 1);
	msleep(10);
	gpiod_set_value_cansleep(data->reset_gpio, 0);
	msleep(10);
	gpiod_set_value_cansleep(data->reset_gpio, 1);

	return 0;
}

static int msp430_enable_sensors(struct msp430_data *data)
{
	/* MODULE_CONFIG is a bitmask of enabled sensors and selects which
	 * sensors drive the interrupt line.
	 */
	return msp430_write_reg(data, MSP430_REG_MODULE_CONFIG,
			       data->enabled_mask);
}

static int msp430_disable_sensors(struct msp430_data *data)
{
	return msp430_write_reg(data, MSP430_REG_MODULE_CONFIG, 0);
}

static int msp430_get_version(struct msp430_data *data)
{
	u8 rev;
	int ret;

	ret = msp430_read_block(data, MSP430_REG_REV_ID, &rev, 1);
	if (ret)
		return ret;

	data->version = rev;
	return 0;
}

static int msp430_chan_to_sensor(const struct iio_chan_spec *chan)
{
	switch (chan->type) {
	case IIO_ACCEL:
		return MSP430_ACCEL;
	case IIO_ANGL_VEL:
		return MSP430_GYRO;
	case IIO_MAGN:
		return MSP430_MAGN;
	case IIO_TEMP:
		return MSP430_TEMP;
	case IIO_PRESSURE:
		return MSP430_PRESSURE;
	default:
		return -EINVAL;
	}
}

static int msp430_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct msp430_data *data = iio_priv(indio_dev);
	int type = msp430_chan_to_sensor(chan);
	int ret;

	if (type < 0)
		return -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&data->lock);
		ret = msp430_read_s16(data, chan->address, (s16 *)val);
		mutex_unlock(&data->lock);
		if (ret)
			return ret;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = data->scale[type][0];
		*val2 = data->scale[type][1];
		return IIO_VAL_INT_PLUS_MICRO;

	default:
		return -EINVAL;
	}
}

static int msp430_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	struct msp430_data *data = iio_priv(indio_dev);
	int type = msp430_chan_to_sensor(chan);

	if (mask != IIO_CHAN_INFO_SCALE)
		return -EINVAL;
	if (type < 0 || val < 0 || val2 < 0)
		return -EINVAL;

	data->scale[type][0] = val;
	data->scale[type][1] = val2;

	return 0;
}

#define MSP430_AXIS_CHANNEL(_type, _mod, _addr, _idx) {		\
	.type = _type,						\
	.modified = 1,						\
	.channel2 = IIO_MOD_##_mod,				\
	.address = _addr,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
	.scan_index = _idx,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 16,					\
		.storagebits = 16,				\
		.endianness = IIO_BE,				\
	},							\
}

#define MSP430_SCALAR_CHANNEL(_type, _addr, _idx) {		\
	.type = _type,						\
	.address = _addr,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
	.scan_index = _idx,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 16,					\
		.storagebits = 16,				\
		.endianness = IIO_BE,				\
	},							\
}

static const struct iio_chan_spec msp430_channels[] = {
	MSP430_AXIS_CHANNEL(IIO_ACCEL, X, MSP430_REG_ACCEL_X, 0),
	MSP430_AXIS_CHANNEL(IIO_ACCEL, Y, MSP430_REG_ACCEL_X + 2, 1),
	MSP430_AXIS_CHANNEL(IIO_ACCEL, Z, MSP430_REG_ACCEL_X + 4, 2),
	MSP430_AXIS_CHANNEL(IIO_ANGL_VEL, X, MSP430_REG_GYRO_X, 3),
	MSP430_AXIS_CHANNEL(IIO_ANGL_VEL, Y, MSP430_REG_GYRO_X + 2, 4),
	MSP430_AXIS_CHANNEL(IIO_ANGL_VEL, Z, MSP430_REG_GYRO_X + 4, 5),
	MSP430_AXIS_CHANNEL(IIO_MAGN, X, MSP430_REG_MAG_HX, 6),
	MSP430_AXIS_CHANNEL(IIO_MAGN, Y, MSP430_REG_MAG_HX + 2, 7),
	MSP430_AXIS_CHANNEL(IIO_MAGN, Z, MSP430_REG_MAG_HX + 4, 8),
	MSP430_SCALAR_CHANNEL(IIO_TEMP, MSP430_REG_TEMPERATURE, 9),
	MSP430_SCALAR_CHANNEL(IIO_PRESSURE, MSP430_REG_CURRENT_PRESSURE, 10),
	IIO_CHAN_SOFT_TIMESTAMP(11),
};

static const unsigned long msp430_scan_masks[] = {
	GENMASK(MSP430_NUM_SENSORS - 1, 0), 0
};

static int msp430_buffer_preenable(struct iio_dev *indio_dev)
{
	struct msp430_data *data = iio_priv(indio_dev);

	return msp430_enable_sensors(data);
}

static int msp430_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct msp430_data *data = iio_priv(indio_dev);

	return msp430_disable_sensors(data);
}

static const struct iio_buffer_setup_ops msp430_buffer_setup_ops = {
	.preenable	= msp430_buffer_preenable,
	.postdisable	= msp430_buffer_postdisable,
};

static irqreturn_t msp430_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct msp430_data *data = iio_priv(indio_dev);
	u8 status;

	mutex_lock(&data->lock);

	if (msp430_read_block(data, MSP430_REG_INTERRUPT_STATUS,
			      &status, 1))
		goto done;

	if (status & MSP430_INT_ACCEL)
		msp430_read_block(data, MSP430_REG_ACCEL_X,
				  (u8 *)data->scan.accel,
				  sizeof(data->scan.accel));
	if (status & MSP430_INT_GYRO)
		msp430_read_block(data, MSP430_REG_GYRO_X,
				  (u8 *)data->scan.gyro,
				  sizeof(data->scan.gyro));
	if (status & MSP430_INT_ECOMPASS)
		msp430_read_block(data, MSP430_REG_MAG_HX,
				  (u8 *)data->scan.magn,
				  sizeof(data->scan.magn));
	if (status & MSP430_INT_TEMPERATURE)
		msp430_read_s16(data, MSP430_REG_TEMPERATURE, &data->scan.temp);
	if (status & MSP430_INT_PRESSURE)
		msp430_read_s16(data, MSP430_REG_CURRENT_PRESSURE,
				&data->scan.pressure);

	iio_push_to_buffers_with_ts(indio_dev, &data->scan,
				    sizeof(data->scan), data->timestamp);

done:
	mutex_unlock(&data->lock);
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int msp430_trigger_set_state(struct iio_trigger *trig, bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct msp430_data *data = iio_priv(indio_dev);
	int ret;

	mutex_lock(&data->lock);
	if (state)
		ret = msp430_enable_sensors(data);
	else
		ret = msp430_disable_sensors(data);
	data->buffer_on = state;
	mutex_unlock(&data->lock);

	return ret;
}

static const struct iio_trigger_ops msp430_trigger_ops = {
	.set_trigger_state = msp430_trigger_set_state,
};

static irqreturn_t msp430_irq_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct msp430_data *data = iio_priv(indio_dev);

	if (!data->buffer_on)
		return IRQ_NONE;

	data->timestamp = iio_get_time_ns(indio_dev);
	iio_trigger_poll(data->trig);

	return IRQ_HANDLED;
}

static ssize_t firmware_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct msp430_data *data = iio_priv(indio_dev);

	return sysfs_emit(buf, "%d\n", data->version);
}

static IIO_DEVICE_ATTR_RO(firmware_version, 0);

static struct attribute *msp430_attributes[] = {
	&iio_dev_attr_firmware_version.dev_attr.attr,
	NULL,
};

static const struct attribute_group msp430_attrs_group = {
	.attrs = msp430_attributes,
};

static const struct iio_info msp430_iio_info = {
	.attrs		= &msp430_attrs_group,
	.read_raw	= msp430_read_raw,
	.write_raw	= msp430_write_raw,
};

static int msp430_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct msp430_data *data;
	struct iio_dev *indio_dev;
	int ret, i;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->client = client;
	data->indio_dev = indio_dev;
	data->enabled_mask = MSP430_SENSOR_MASK;
	for (i = 0; i < MSP430_NUM_SENSORS; i++) {
		data->scale[i][0] = msp430_default_scale[i][0];
		data->scale[i][1] = msp430_default_scale[i][1];
	}
	mutex_init(&data->lock);

	i2c_set_clientdata(client, indio_dev);

	ret = devm_regulator_get_enable(dev, "vdd");
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get vdd regulator\n");

	data->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(data->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(data->reset_gpio),
				     "Failed to get reset GPIO\n");

	data->test_gpio = devm_gpiod_get_optional(dev, "test", GPIOD_OUT_LOW);
	if (IS_ERR(data->test_gpio))
		return dev_err_probe(dev, PTR_ERR(data->test_gpio),
				     "Failed to get test GPIO\n");

	msp430_set_normal_mode(data);

	ret = msp430_get_version(data);
	if (ret)
		dev_warn(dev, "Failed to read firmware version: %d\n", ret);
	else
		dev_info(dev, "MSP430 sensor hub firmware version %d\n",
			 data->version);

	indio_dev->channels = msp430_channels;
	indio_dev->num_channels = ARRAY_SIZE(msp430_channels);
	indio_dev->available_scan_masks = msp430_scan_masks;
	indio_dev->name = MSP430_DRV_NAME;
	indio_dev->info = &msp430_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	if (client->irq > 0) {
		data->trig = devm_iio_trigger_alloc(dev, "%s-dev%d",
						    indio_dev->name,
						    iio_device_id(indio_dev));
		if (!data->trig)
			return -ENOMEM;

		data->trig->ops = &msp430_trigger_ops;
		iio_trigger_set_drvdata(data->trig, indio_dev);
		ret = devm_iio_trigger_register(dev, data->trig);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to register trigger\n");

		ret = devm_request_threaded_irq(dev, client->irq,
						msp430_irq_handler, NULL,
						IRQF_TRIGGER_RISING,
						MSP430_DRV_NAME, indio_dev);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to request interrupt\n");
	}

	ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
					      &iio_pollfunc_store_time,
					      msp430_trigger_handler,
					      &msp430_buffer_setup_ops);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to set up triggered buffer\n");

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register IIO device\n");

	dev_info(dev, "MSP430 sensor hub probed (irq %d)\n", client->irq);

	return 0;
}

static int msp430_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct msp430_data *data = iio_priv(indio_dev);
	int ret;

	mutex_lock(&data->lock);
	ret = msp430_disable_sensors(data);
	mutex_unlock(&data->lock);

	return ret;
}

static int msp430_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct msp430_data *data = iio_priv(indio_dev);
	int ret = 0;

	mutex_lock(&data->lock);
	if (data->buffer_on)
		ret = msp430_enable_sensors(data);
	mutex_unlock(&data->lock);

	return ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(msp430_pm_ops, msp430_suspend, msp430_resume);

static const struct i2c_device_id msp430_id[] = {
	{ "msp430", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, msp430_id);

static const struct of_device_id msp430_of_match[] = {
	{ .compatible = "motorola,msp430-sensors" },
	{}
};
MODULE_DEVICE_TABLE(of, msp430_of_match);

static struct i2c_driver msp430_driver = {
	.driver = {
		.name		= MSP430_DRV_NAME,
		.of_match_table	= msp430_of_match,
		.pm		= pm_sleep_ptr(&msp430_pm_ops),
	},
	.probe		= msp430_probe,
	.id_table	= msp430_id,
};
module_i2c_driver(msp430_driver);

MODULE_DESCRIPTION("Motorola TI MSP430 sensor hub driver");
MODULE_AUTHOR("Merlijn Wajer <merlijn@wizzup.org>");
MODULE_LICENSE("GPL");
