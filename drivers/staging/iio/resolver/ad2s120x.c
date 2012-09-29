/*
 * ad2s120x.c simple support for the ADI Resolver to Digital Converters: AD2S1200/1205
 *
 * Copyright (c) 2010-2010 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include "../iio.h"
#include "../sysfs.h"

#define DRV_NAME "ad2s120x"

/* input pin sample and rdvel is controlled by driver */
#define AD2S120X_PN	2

/* input clock on serial interface */
#define AD2S120X_HZ	8192000
/* clock period in nano second */
#define AD2S120X_TSCLK	(1000000000/AD2S120X_HZ)

struct ad2s120x_state {
	struct mutex lock;
<<<<<<< HEAD
	struct iio_dev *idev;
	struct spi_device *sdev;
	unsigned short sample;
	unsigned short rdvel;
	u8 rx[2];
	u8 tx[2];
};

static ssize_t ad2s120x_show_pos_vel(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct spi_message msg;
	struct spi_transfer xfer;
=======
	struct spi_device *sdev;
	int sample;
	int rdvel;
	u8 rx[2] ____cacheline_aligned;
};

static ssize_t ad2s120x_show_val(struct device *dev,
			struct device_attribute *attr, char *buf)
{
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	int ret = 0;
	ssize_t len = 0;
	u16 pos;
	s16 vel;
	u8 status;
<<<<<<< HEAD
	struct iio_dev *idev = dev_get_drvdata(dev);
	struct ad2s120x_state *st = idev->dev_data;

	xfer.len = 1;
	xfer.tx_buf = st->tx;
	xfer.rx_buf = st->rx;
=======
	struct ad2s120x_state *st = iio_priv(dev_get_drvdata(dev));
	struct iio_dev_attr *iattr = to_iio_dev_attr(attr);

>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	mutex_lock(&st->lock);

	gpio_set_value(st->sample, 0);
	/* delay (6 * AD2S120X_TSCLK + 20) nano seconds */
	udelay(1);
	gpio_set_value(st->sample, 1);
<<<<<<< HEAD

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	ret = spi_sync(st->sdev, &msg);
	if (ret)
		goto error_ret;
	status = st->rx[1];
	pos = (((u16)(st->rx[0])) << 4) | ((st->rx[1] & 0xF0) >> 4);
	len = sprintf(buf, "%d %c%c%c%c ", pos,
				(status & 0x8) ? 'P' : 'V',
				(status & 0x4) ? 'd' : '_',
				(status & 0x2) ? 'l' : '_',
				(status & 0x1) ? '1' : '0');

	/* delay 18 ns */
	/* ndelay(18); */

	gpio_set_value(st->rdvel, 0);
	/* ndelay(5);*/

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	ret = spi_sync(st->sdev, &msg);
	if (ret)
		goto error_ret;
	status = st->rx[1];
	vel = (st->rx[0] & 0x80) ? 0xf000 : 0;
	vel |= (((s16)(st->rx[0])) << 4) | ((st->rx[1] & 0xF0) >> 4);
	len += sprintf(buf + len, "%d %c%c%c%c\n", vel,
				(status & 0x8) ? 'P' : 'V',
				(status & 0x4) ? 'd' : '_',
				(status & 0x2) ? 'l' : '_',
				(status & 0x1) ? '1' : '0');
error_ret:
	gpio_set_value(st->rdvel, 1);
	/* delay (2 * AD2S120X_TSCLK + 20) ns for sample pulse */
	udelay(1);
	mutex_unlock(&st->lock);

	return ret ? ret : len;
}

static ssize_t ad2s120x_show_pos(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct spi_message msg;
	struct spi_transfer xfer;
	int ret = 0;
	ssize_t len = 0;
	u16 pos;
	u8 status;
	struct iio_dev *idev = dev_get_drvdata(dev);
	struct ad2s120x_state *st = idev->dev_data;

	xfer.len = 1;
	xfer.tx_buf = st->tx;
	xfer.rx_buf = st->rx;
	mutex_lock(&st->lock);

	gpio_set_value(st->sample, 0);
	/* delay (6 * AD2S120X_TSCLK + 20) nano seconds */
	udelay(1);
	gpio_set_value(st->sample, 1);
	gpio_set_value(st->rdvel, 1);

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	ret = spi_sync(st->sdev, &msg);
	if (ret)
		goto error_ret;
	status = st->rx[1];
	pos = (((u16)(st->rx[0])) << 4) | ((st->rx[1] & 0xF0) >> 4);
	len = sprintf(buf, "%d %c%c%c%c ", pos,
				(status & 0x8) ? 'P' : 'V',
				(status & 0x4) ? 'd' : '_',
				(status & 0x2) ? 'l' : '_',
				(status & 0x1) ? '1' : '0');
error_ret:
	/* delay (2 * AD2S120X_TSCLK + 20) ns for sample pulse */
	udelay(1);
	mutex_unlock(&st->lock);

	return ret ? ret : len;
}

static ssize_t ad2s120x_show_vel(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct spi_message msg;
	struct spi_transfer xfer;
	int ret = 0;
	ssize_t len = 0;
	s16 vel;
	u8 status;
	struct iio_dev *idev = dev_get_drvdata(dev);
	struct ad2s120x_state *st = idev->dev_data;

	xfer.len = 1;
	xfer.tx_buf = st->tx;
	xfer.rx_buf = st->rx;
	mutex_lock(&st->lock);

	gpio_set_value(st->sample, 0);
	/* delay (6 * AD2S120X_TSCLK + 20) nano seconds */
	udelay(1);
	gpio_set_value(st->sample, 1);

	gpio_set_value(st->rdvel, 0);
	/* ndelay(5);*/

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	ret = spi_sync(st->sdev, &msg);
	if (ret)
		goto error_ret;
	status = st->rx[1];
	vel = (st->rx[0] & 0x80) ? 0xf000 : 0;
	vel |= (((s16)(st->rx[0])) << 4) | ((st->rx[1] & 0xF0) >> 4);
	len += sprintf(buf + len, "%d %c%c%c%c\n", vel,
=======
	gpio_set_value(st->rdvel, iattr->address);
	ret = spi_read(st->sdev, st->rx, 2);
	if (ret < 0)
		goto error_ret;
	status = st->rx[1];
	if (iattr->address)
		pos = (((u16)(st->rx[0])) << 4) | ((st->rx[1] & 0xF0) >> 4);
	else {
		vel = (st->rx[0] & 0x80) ? 0xf000 : 0;
		vel |= (((s16)(st->rx[0])) << 4) | ((st->rx[1] & 0xF0) >> 4);
	}
	len = sprintf(buf, "%d %c%c%c%c ", iattr->address ? pos : vel,
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
				(status & 0x8) ? 'P' : 'V',
				(status & 0x4) ? 'd' : '_',
				(status & 0x2) ? 'l' : '_',
				(status & 0x1) ? '1' : '0');
error_ret:
<<<<<<< HEAD
	gpio_set_value(st->rdvel, 1);
=======
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	/* delay (2 * AD2S120X_TSCLK + 20) ns for sample pulse */
	udelay(1);
	mutex_unlock(&st->lock);

	return ret ? ret : len;
}

<<<<<<< HEAD
static IIO_CONST_ATTR(description,
	"12-Bit R/D Converter with Reference Oscillator");
static IIO_DEVICE_ATTR(pos_vel, S_IRUGO, ad2s120x_show_pos_vel, NULL, 0);
static IIO_DEVICE_ATTR(pos, S_IRUGO, ad2s120x_show_pos, NULL, 0);
static IIO_DEVICE_ATTR(vel, S_IRUGO, ad2s120x_show_vel, NULL, 0);

static struct attribute *ad2s120x_attributes[] = {
	&iio_const_attr_description.dev_attr.attr,
	&iio_dev_attr_pos_vel.dev_attr.attr,
=======
static IIO_DEVICE_ATTR(pos, S_IRUGO, ad2s120x_show_val, NULL, 1);
static IIO_DEVICE_ATTR(vel, S_IRUGO, ad2s120x_show_val, NULL, 0);

static struct attribute *ad2s120x_attributes[] = {
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	&iio_dev_attr_pos.dev_attr.attr,
	&iio_dev_attr_vel.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad2s120x_attribute_group = {
	.attrs = ad2s120x_attributes,
};

static const struct iio_info ad2s120x_info = {
	.attrs = &ad2s120x_attribute_group,
	.driver_module = THIS_MODULE,
};

static int __devinit ad2s120x_probe(struct spi_device *spi)
{
	struct ad2s120x_state *st;
<<<<<<< HEAD
	int pn, ret = 0;
	unsigned short *pins = spi->dev.platform_data;

	for (pn = 0; pn < AD2S120X_PN; pn++) {
		if (gpio_request(pins[pn], DRV_NAME)) {
=======
	struct iio_dev *indio_dev;
	int pn, ret = 0;
	unsigned short *pins = spi->dev.platform_data;

	for (pn = 0; pn < AD2S120X_PN; pn++)
		if (gpio_request_one(pins[pn], GPIOF_DIR_OUT, DRV_NAME)) {
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
			pr_err("%s: request gpio pin %d failed\n",
						DRV_NAME, pins[pn]);
			goto error_ret;
		}
<<<<<<< HEAD
		gpio_direction_output(pins[pn], 1);
	}

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (st == NULL) {
		ret = -ENOMEM;
		goto error_ret;
	}
	spi_set_drvdata(spi, st);

=======
	indio_dev = iio_allocate_device(sizeof(*st));
	if (indio_dev == NULL) {
		ret = -ENOMEM;
		goto error_ret;
	}
	spi_set_drvdata(spi, indio_dev);
	st = iio_priv(indio_dev);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	mutex_init(&st->lock);
	st->sdev = spi;
	st->sample = pins[0];
	st->rdvel = pins[1];

<<<<<<< HEAD
	st->idev = iio_allocate_device(0);
	if (st->idev == NULL) {
		ret = -ENOMEM;
		goto error_free_st;
	}
	st->idev->dev.parent = &spi->dev;

	st->idev->info = &ad2s120x_info;
	st->idev->dev_data = (void *)(st);
	st->idev->modes = INDIO_DIRECT_MODE;

	ret = iio_device_register(st->idev);
=======
	indio_dev->dev.parent = &spi->dev;
	indio_dev->info = &ad2s120x_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = iio_device_register(indio_dev);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
	if (ret)
		goto error_free_dev;

	spi->max_speed_hz = AD2S120X_HZ;
	spi->mode = SPI_MODE_3;
	spi_setup(spi);

	return 0;

error_free_dev:
<<<<<<< HEAD
	iio_free_device(st->idev);
error_free_st:
	kfree(st);
=======
	iio_free_device(indio_dev);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7
error_ret:
	for (--pn; pn >= 0; pn--)
		gpio_free(pins[pn]);
	return ret;
}

static int __devexit ad2s120x_remove(struct spi_device *spi)
{
<<<<<<< HEAD
	struct ad2s120x_state *st = spi_get_drvdata(spi);

	iio_device_unregister(st->idev);
	kfree(st);
=======
	iio_device_unregister(spi_get_drvdata(spi));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	return 0;
}

static struct spi_driver ad2s120x_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
	},
	.probe = ad2s120x_probe,
	.remove = __devexit_p(ad2s120x_remove),
};

static __init int ad2s120x_spi_init(void)
{
	return spi_register_driver(&ad2s120x_driver);
}
module_init(ad2s120x_spi_init);

static __exit void ad2s120x_spi_exit(void)
{
	spi_unregister_driver(&ad2s120x_driver);
}
module_exit(ad2s120x_spi_exit);

MODULE_AUTHOR("Graff Yang <graff.yang@gmail.com>");
MODULE_DESCRIPTION("Analog Devices AD2S1200/1205 Resolver to Digital SPI driver");
MODULE_LICENSE("GPL v2");
