/*
 * STMicroelectronics accelerometers driver
 *
 * Copyright 2012-2013 STMicroelectronics Inc.
 *
 * Denis Ciocca <denis.ciocca@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/buffer.h>

#include <linux/i2c/sensors/common/st_sensors_htc.h>
#include "st_accel.h"
#include <linux/notifier.h>
#include <linux/qpnp_vibrator.h>

#define D(x...) printk(KERN_DEBUG "[GSNR][LIS2DH12] " x)
#define I(x...) printk(KERN_INFO "[GSNR][LIS2DH12] " x)
#define E(x...) printk(KERN_ERR "[GSNR][LIS2DH12 ERROR] " x)

#define CALIBRATION_DATA_PATH "/calibration_data"
#define G_SENSOR_FLASH_DATA "gs_flash"

#define ST_ACCEL_DEFAULT_OUT_X_L_ADDR		0x28
#define ST_ACCEL_DEFAULT_OUT_Y_L_ADDR		0x2a
#define ST_ACCEL_DEFAULT_OUT_Z_L_ADDR		0x2c

#define ST_ACCEL_FS_AVL_2G			2
#define ST_ACCEL_FS_AVL_4G			4
#define ST_ACCEL_FS_AVL_6G			6
#define ST_ACCEL_FS_AVL_8G			8
#define ST_ACCEL_FS_AVL_16G			16

#define ST_ACCEL_1_WAI_EXP			0x33
#define ST_ACCEL_1_ODR_ADDR			0x20
#define ST_ACCEL_1_ODR_MASK			0xf0
#define ST_ACCEL_1_ODR_AVL_1HZ_VAL		0x01
#define ST_ACCEL_1_ODR_AVL_10HZ_VAL		0x02
#define ST_ACCEL_1_ODR_AVL_25HZ_VAL		0x03
#define ST_ACCEL_1_ODR_AVL_50HZ_VAL		0x04
#define ST_ACCEL_1_ODR_AVL_100HZ_VAL		0x05
#define ST_ACCEL_1_ODR_AVL_200HZ_VAL		0x06
#define ST_ACCEL_1_ODR_AVL_400HZ_VAL		0x07
#define ST_ACCEL_1_ODR_AVL_1600HZ_VAL		0x08
#define ST_ACCEL_1_FS_ADDR			0x23
#define ST_ACCEL_1_FS_MASK			0x30
#define ST_ACCEL_1_FS_AVL_2_VAL			0x00
#define ST_ACCEL_1_FS_AVL_4_VAL			0x01
#define ST_ACCEL_1_FS_AVL_8_VAL			0x02
#define ST_ACCEL_1_FS_AVL_16_VAL		0x03
#define ST_ACCEL_1_FS_AVL_2_GAIN		IIO_G_TO_M_S_2(1000)
#define ST_ACCEL_1_FS_AVL_4_GAIN		IIO_G_TO_M_S_2(2000)
#define ST_ACCEL_1_FS_AVL_8_GAIN		IIO_G_TO_M_S_2(4000)
#define ST_ACCEL_1_FS_AVL_16_GAIN		IIO_G_TO_M_S_2(12000)
#define ST_ACCEL_1_BDU_ADDR			0x23
#define ST_ACCEL_1_BDU_MASK			0x80
#define ST_ACCEL_1_DRDY_IRQ_ADDR		0x22
#define ST_ACCEL_1_DRDY_IRQ_MASK		0x10
#define ST_ACCEL_1_MULTIREAD_BIT		true

#define ST_ACCEL_2_WAI_EXP			0x32
#define ST_ACCEL_2_ODR_ADDR			0x20
#define ST_ACCEL_2_ODR_MASK			0x18
#define ST_ACCEL_2_ODR_AVL_50HZ_VAL		0x00
#define ST_ACCEL_2_ODR_AVL_100HZ_VAL		0x01
#define ST_ACCEL_2_ODR_AVL_400HZ_VAL		0x02
#define ST_ACCEL_2_ODR_AVL_1000HZ_VAL		0x03
#define ST_ACCEL_2_PW_ADDR			0x20
#define ST_ACCEL_2_PW_MASK			0xe0
#define ST_ACCEL_2_FS_ADDR			0x23
#define ST_ACCEL_2_FS_MASK			0x30
#define ST_ACCEL_2_FS_AVL_2_VAL			0X00
#define ST_ACCEL_2_FS_AVL_4_VAL			0X01
#define ST_ACCEL_2_FS_AVL_8_VAL			0x03
#define ST_ACCEL_2_FS_AVL_2_GAIN		IIO_G_TO_M_S_2(1000)
#define ST_ACCEL_2_FS_AVL_4_GAIN		IIO_G_TO_M_S_2(2000)
#define ST_ACCEL_2_FS_AVL_8_GAIN		IIO_G_TO_M_S_2(3900)
#define ST_ACCEL_2_BDU_ADDR			0x23
#define ST_ACCEL_2_BDU_MASK			0x80
#define ST_ACCEL_2_DRDY_IRQ_ADDR		0x22
#define ST_ACCEL_2_DRDY_IRQ_MASK		0x02
#define ST_ACCEL_2_MULTIREAD_BIT		true

#define ST_ACCEL_3_WAI_EXP			0x40
#define ST_ACCEL_3_ODR_ADDR			0x20
#define ST_ACCEL_3_ODR_MASK			0xf0
#define ST_ACCEL_3_ODR_AVL_3HZ_VAL		0x01
#define ST_ACCEL_3_ODR_AVL_6HZ_VAL		0x02
#define ST_ACCEL_3_ODR_AVL_12HZ_VAL		0x03
#define ST_ACCEL_3_ODR_AVL_25HZ_VAL		0x04
#define ST_ACCEL_3_ODR_AVL_50HZ_VAL		0x05
#define ST_ACCEL_3_ODR_AVL_100HZ_VAL		0x06
#define ST_ACCEL_3_ODR_AVL_200HZ_VAL		0x07
#define ST_ACCEL_3_ODR_AVL_400HZ_VAL		0x08
#define ST_ACCEL_3_ODR_AVL_800HZ_VAL		0x09
#define ST_ACCEL_3_ODR_AVL_1600HZ_VAL		0x0a
#define ST_ACCEL_3_FS_ADDR			0x24
#define ST_ACCEL_3_FS_MASK			0x38
#define ST_ACCEL_3_FS_AVL_2_VAL			0X00
#define ST_ACCEL_3_FS_AVL_4_VAL			0X01
#define ST_ACCEL_3_FS_AVL_6_VAL			0x02
#define ST_ACCEL_3_FS_AVL_8_VAL			0x03
#define ST_ACCEL_3_FS_AVL_16_VAL		0x04
#define ST_ACCEL_3_FS_AVL_2_GAIN		IIO_G_TO_M_S_2(61)
#define ST_ACCEL_3_FS_AVL_4_GAIN		IIO_G_TO_M_S_2(122)
#define ST_ACCEL_3_FS_AVL_6_GAIN		IIO_G_TO_M_S_2(183)
#define ST_ACCEL_3_FS_AVL_8_GAIN		IIO_G_TO_M_S_2(244)
#define ST_ACCEL_3_FS_AVL_16_GAIN		IIO_G_TO_M_S_2(732)
#define ST_ACCEL_3_BDU_ADDR			0x20
#define ST_ACCEL_3_BDU_MASK			0x08
#define ST_ACCEL_3_DRDY_IRQ_ADDR		0x23
#define ST_ACCEL_3_DRDY_IRQ_MASK		0x80
#define ST_ACCEL_3_IG1_EN_ADDR			0x23
#define ST_ACCEL_3_IG1_EN_MASK			0x08
#define ST_ACCEL_3_MULTIREAD_BIT		false

static const struct iio_chan_spec st_accel_12bit_channels[] = {
	ST_SENSORS_LSM_CHANNELS(IIO_ACCEL, ST_SENSORS_SCAN_X, IIO_MOD_X, IIO_LE,
		ST_SENSORS_DEFAULT_12_REALBITS, ST_ACCEL_DEFAULT_OUT_X_L_ADDR),
	ST_SENSORS_LSM_CHANNELS(IIO_ACCEL, ST_SENSORS_SCAN_Y, IIO_MOD_Y, IIO_LE,
		ST_SENSORS_DEFAULT_12_REALBITS, ST_ACCEL_DEFAULT_OUT_Y_L_ADDR),
	ST_SENSORS_LSM_CHANNELS(IIO_ACCEL, ST_SENSORS_SCAN_Z, IIO_MOD_Z, IIO_LE,
		ST_SENSORS_DEFAULT_12_REALBITS, ST_ACCEL_DEFAULT_OUT_Z_L_ADDR),
	IIO_CHAN_SOFT_TIMESTAMP(3)
};

static const struct iio_chan_spec st_accel_16bit_channels[] = {
	ST_SENSORS_LSM_CHANNELS(IIO_ACCEL, ST_SENSORS_SCAN_X, IIO_MOD_X, IIO_LE,
		ST_SENSORS_DEFAULT_16_REALBITS, ST_ACCEL_DEFAULT_OUT_X_L_ADDR),
	ST_SENSORS_LSM_CHANNELS(IIO_ACCEL, ST_SENSORS_SCAN_Y, IIO_MOD_Y, IIO_LE,
		ST_SENSORS_DEFAULT_16_REALBITS, ST_ACCEL_DEFAULT_OUT_Y_L_ADDR),
	ST_SENSORS_LSM_CHANNELS(IIO_ACCEL, ST_SENSORS_SCAN_Z, IIO_MOD_Z, IIO_LE,
		ST_SENSORS_DEFAULT_16_REALBITS, ST_ACCEL_DEFAULT_OUT_Z_L_ADDR),
	IIO_CHAN_SOFT_TIMESTAMP(3)
};

static const struct st_sensors st_accel_sensors[] = {
	{
		.wai = ST_ACCEL_1_WAI_EXP,
		.sensors_supported = {
			[0] = LIS3DH_ACCEL_DEV_NAME,
			[1] = LSM303DLHC_ACCEL_DEV_NAME,
			[2] = LSM330D_ACCEL_DEV_NAME,
			[3] = LSM330DL_ACCEL_DEV_NAME,
			[4] = LSM330DLC_ACCEL_DEV_NAME,
			[5] = LIS2DH12_ACCEL_DEV_NAME,
		},
		.ch = (struct iio_chan_spec *)st_accel_12bit_channels,
		.odr = {
			.addr = ST_ACCEL_1_ODR_ADDR,
			.mask = ST_ACCEL_1_ODR_MASK,
			.odr_avl = {
				{ 1, ST_ACCEL_1_ODR_AVL_100HZ_VAL, },
				{ 10, ST_ACCEL_1_ODR_AVL_100HZ_VAL, },
				{ 25, ST_ACCEL_1_ODR_AVL_100HZ_VAL, },
				{ 50, ST_ACCEL_1_ODR_AVL_100HZ_VAL, },
				{ 100, ST_ACCEL_1_ODR_AVL_100HZ_VAL, },
				{ 200, ST_ACCEL_1_ODR_AVL_200HZ_VAL, },
				{ 400, ST_ACCEL_1_ODR_AVL_400HZ_VAL, },
				{ 1600, ST_ACCEL_1_ODR_AVL_1600HZ_VAL, },
			},
		},
		.pw = {
			.addr = ST_ACCEL_1_ODR_ADDR,
			.mask = ST_ACCEL_1_ODR_MASK,
			.value_off = ST_SENSORS_DEFAULT_POWER_OFF_VALUE,
		},
		.enable_axis = {
			.addr = ST_SENSORS_DEFAULT_AXIS_ADDR,
			.mask = ST_SENSORS_DEFAULT_AXIS_MASK,
		},
		.fs = {
			.addr = ST_ACCEL_1_FS_ADDR,
			.mask = ST_ACCEL_1_FS_MASK,
			.fs_avl = {
				[0] = {
					.num = ST_ACCEL_FS_AVL_2G,
					.value = ST_ACCEL_1_FS_AVL_2_VAL,
					.gain = ST_ACCEL_1_FS_AVL_2_GAIN,
				},
				[1] = {
					.num = ST_ACCEL_FS_AVL_4G,
					.value = ST_ACCEL_1_FS_AVL_4_VAL,
					.gain = ST_ACCEL_1_FS_AVL_4_GAIN,
				},
				[2] = {
					.num = ST_ACCEL_FS_AVL_8G,
					.value = ST_ACCEL_1_FS_AVL_8_VAL,
					.gain = ST_ACCEL_1_FS_AVL_8_GAIN,
				},
				[3] = {
					.num = ST_ACCEL_FS_AVL_16G,
					.value = ST_ACCEL_1_FS_AVL_16_VAL,
					.gain = ST_ACCEL_1_FS_AVL_16_GAIN,
				},
			},
		},
		.bdu = {
			.addr = ST_ACCEL_1_BDU_ADDR,
			.mask = ST_ACCEL_1_BDU_MASK,
		},
		.drdy_irq = {
			.addr = ST_ACCEL_1_DRDY_IRQ_ADDR,
			.mask = ST_ACCEL_1_DRDY_IRQ_MASK,
		},
		.multi_read_bit = ST_ACCEL_1_MULTIREAD_BIT,
		.bootime = 2,
	},
	{
		.wai = ST_ACCEL_2_WAI_EXP,
		.sensors_supported = {
			[0] = LIS331DLH_ACCEL_DEV_NAME,
			[1] = LSM303DL_ACCEL_DEV_NAME,
			[2] = LSM303DLH_ACCEL_DEV_NAME,
			[3] = LSM303DLM_ACCEL_DEV_NAME,
		},
		.ch = (struct iio_chan_spec *)st_accel_12bit_channels,
		.odr = {
			.addr = ST_ACCEL_2_ODR_ADDR,
			.mask = ST_ACCEL_2_ODR_MASK,
			.odr_avl = {
				{ 50, ST_ACCEL_2_ODR_AVL_50HZ_VAL, },
				{ 100, ST_ACCEL_2_ODR_AVL_100HZ_VAL, },
				{ 400, ST_ACCEL_2_ODR_AVL_400HZ_VAL, },
				{ 1000, ST_ACCEL_2_ODR_AVL_1000HZ_VAL, },
			},
		},
		.pw = {
			.addr = ST_ACCEL_2_PW_ADDR,
			.mask = ST_ACCEL_2_PW_MASK,
			.value_on = ST_SENSORS_DEFAULT_POWER_ON_VALUE,
			.value_off = ST_SENSORS_DEFAULT_POWER_OFF_VALUE,
		},
		.enable_axis = {
			.addr = ST_SENSORS_DEFAULT_AXIS_ADDR,
			.mask = ST_SENSORS_DEFAULT_AXIS_MASK,
		},
		.fs = {
			.addr = ST_ACCEL_2_FS_ADDR,
			.mask = ST_ACCEL_2_FS_MASK,
			.fs_avl = {
				[0] = {
					.num = ST_ACCEL_FS_AVL_2G,
					.value = ST_ACCEL_2_FS_AVL_2_VAL,
					.gain = ST_ACCEL_2_FS_AVL_2_GAIN,
				},
				[1] = {
					.num = ST_ACCEL_FS_AVL_4G,
					.value = ST_ACCEL_2_FS_AVL_4_VAL,
					.gain = ST_ACCEL_2_FS_AVL_4_GAIN,
				},
				[2] = {
					.num = ST_ACCEL_FS_AVL_8G,
					.value = ST_ACCEL_2_FS_AVL_8_VAL,
					.gain = ST_ACCEL_2_FS_AVL_8_GAIN,
				},
			},
		},
		.bdu = {
			.addr = ST_ACCEL_2_BDU_ADDR,
			.mask = ST_ACCEL_2_BDU_MASK,
		},
		.drdy_irq = {
			.addr = ST_ACCEL_2_DRDY_IRQ_ADDR,
			.mask = ST_ACCEL_2_DRDY_IRQ_MASK,
		},
		.multi_read_bit = ST_ACCEL_2_MULTIREAD_BIT,
		.bootime = 2,
	},
	{
		.wai = ST_ACCEL_3_WAI_EXP,
		.sensors_supported = {
			[0] = LSM330_ACCEL_DEV_NAME,
		},
		.ch = (struct iio_chan_spec *)st_accel_16bit_channels,
		.odr = {
			.addr = ST_ACCEL_3_ODR_ADDR,
			.mask = ST_ACCEL_3_ODR_MASK,
			.odr_avl = {
				{ 3, ST_ACCEL_3_ODR_AVL_3HZ_VAL },
				{ 6, ST_ACCEL_3_ODR_AVL_6HZ_VAL, },
				{ 12, ST_ACCEL_3_ODR_AVL_12HZ_VAL, },
				{ 25, ST_ACCEL_3_ODR_AVL_25HZ_VAL, },
				{ 50, ST_ACCEL_3_ODR_AVL_50HZ_VAL, },
				{ 100, ST_ACCEL_3_ODR_AVL_100HZ_VAL, },
				{ 200, ST_ACCEL_3_ODR_AVL_200HZ_VAL, },
				{ 400, ST_ACCEL_3_ODR_AVL_400HZ_VAL, },
				{ 800, ST_ACCEL_3_ODR_AVL_800HZ_VAL, },
				{ 1600, ST_ACCEL_3_ODR_AVL_1600HZ_VAL, },
			},
		},
		.pw = {
			.addr = ST_ACCEL_3_ODR_ADDR,
			.mask = ST_ACCEL_3_ODR_MASK,
			.value_off = ST_SENSORS_DEFAULT_POWER_OFF_VALUE,
		},
		.enable_axis = {
			.addr = ST_SENSORS_DEFAULT_AXIS_ADDR,
			.mask = ST_SENSORS_DEFAULT_AXIS_MASK,
		},
		.fs = {
			.addr = ST_ACCEL_3_FS_ADDR,
			.mask = ST_ACCEL_3_FS_MASK,
			.fs_avl = {
				[0] = {
					.num = ST_ACCEL_FS_AVL_2G,
					.value = ST_ACCEL_3_FS_AVL_2_VAL,
					.gain = ST_ACCEL_3_FS_AVL_2_GAIN,
				},
				[1] = {
					.num = ST_ACCEL_FS_AVL_4G,
					.value = ST_ACCEL_3_FS_AVL_4_VAL,
					.gain = ST_ACCEL_3_FS_AVL_4_GAIN,
				},
				[2] = {
					.num = ST_ACCEL_FS_AVL_6G,
					.value = ST_ACCEL_3_FS_AVL_6_VAL,
					.gain = ST_ACCEL_3_FS_AVL_6_GAIN,
				},
				[3] = {
					.num = ST_ACCEL_FS_AVL_8G,
					.value = ST_ACCEL_3_FS_AVL_8_VAL,
					.gain = ST_ACCEL_3_FS_AVL_8_GAIN,
				},
				[4] = {
					.num = ST_ACCEL_FS_AVL_16G,
					.value = ST_ACCEL_3_FS_AVL_16_VAL,
					.gain = ST_ACCEL_3_FS_AVL_16_GAIN,
				},
			},
		},
		.bdu = {
			.addr = ST_ACCEL_3_BDU_ADDR,
			.mask = ST_ACCEL_3_BDU_MASK,
		},
		.drdy_irq = {
			.addr = ST_ACCEL_3_DRDY_IRQ_ADDR,
			.mask = ST_ACCEL_3_DRDY_IRQ_MASK,
			.ig1 = {
				.en_addr = ST_ACCEL_3_IG1_EN_ADDR,
				.en_mask = ST_ACCEL_3_IG1_EN_MASK,
			},
		},
		.multi_read_bit = ST_ACCEL_3_MULTIREAD_BIT,
		.bootime = 2,
	},
};

extern void set_wa_values(bool enable, int duration);
static int qpnp_vibrator_status_handler_func(struct notifier_block *this, unsigned long status, void *unused)
{
	bool enable = status & 0x01;
	int duration = (status & 0xFFFF) >> 1;

	printk(KERN_INFO "%s: duration is %d, vib status is %d\n", __func__, duration, enable);
	set_wa_values(enable, duration);
	return NOTIFY_OK;
}

static struct notifier_block qpnp_vibrator_status_handler = {
	.notifier_call = qpnp_vibrator_status_handler_func,
};

static int st_accel_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *ch, int *val,
							int *val2, long mask)
{
	int err;
	struct st_sensor_data *adata = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		err = st_sensors_read_info_raw(indio_dev, ch, val);
		if (err < 0)
			goto read_error;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = adata->current_fullscale->gain;
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}

read_error:
	return err;
}

static int st_accel_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	int err;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		err = st_sensors_set_fullscale_by_gain(indio_dev, val2);
		break;
	default:
		return -EINVAL;
	}

	return err;
}

static int st_accel_registerAttr(struct iio_dev *indio_dev)
{
	int ret;
	struct class *htc_accelerometer_class;
	struct device *accelerometer_dev;
#if 0
#ifdef CONFIG_CIR_ALWAYS_READY
	 struct class *st_accel_powerkey_class = NULL;
	struct device *st_accel_powerkey_dev = NULL;
#endif
#endif
	htc_accelerometer_class = class_create(THIS_MODULE,
					"htc_g_sensor");
	if (IS_ERR(htc_accelerometer_class)) {
		ret = PTR_ERR(htc_accelerometer_class);
		htc_accelerometer_class = NULL;
		goto err_create_class;
	}

	accelerometer_dev = device_create(htc_accelerometer_class,
				NULL, 0, "%s", "lis2dh12");
	if (unlikely(IS_ERR(accelerometer_dev))) {
		ret = PTR_ERR(accelerometer_dev);
		accelerometer_dev = NULL;
		goto err_create_accelerometer_device;
	}
#if 0 
#ifdef CONFIG_CIR_ALWAYS_READY
	ret = device_create_file(accelerometer_dev, &dev_attr_enable_cir_interrupt);

	if (ret)
		goto err_create_accelerometer_debug_en_device_file;

	st_accel_powerkey_class = class_create(THIS_MODULE, "st_accel_powerkey");
	if (IS_ERR(st_accel_powerkey_class)) {
	    ret = PTR_ERR(st_accel_powerkey_class);
	    st_accel_powerkey_class = NULL;
	    E("%s: could not allocate st_accel_powerkey_class\n", __func__);
	    goto err_create_st_accel_powerkey_class_failed;
	}
	st_accel_powerkey_dev= device_create(st_accel_powerkey_class,
		NULL, 0, "%s", "st_accel");

	ret = device_create_file(st_accel_powerkey_dev, &dev_attr_clear_powerkey_flag);

	if (ret)
		goto err_create_accelerometer_clear_powerkey_flag_device_file;
#endif
#endif
	ret = sysfs_create_link(&accelerometer_dev->kobj,
				&indio_dev->dev.kobj, "iio");
	if (ret < 0)
		goto err_sysfs_create_link;

	return 0;

err_sysfs_create_link:
#if 0
#ifdef CONFIG_CIR_ALWAYS_READY
	device_unregister(st_accel_powerkey_dev);
err_create_st_accel_powerkey_class_failed:
	class_destroy(st_accel_powerkey_class);
err_create_accelerometer_clear_powerkey_flag_device_file:
	device_remove_file(accelerometer_dev, &dev_attr_enable_cir_interrupt);
#endif
err_create_accelerometer_debug_en_device_file:
err_create_accelerometer_device_file:
#endif
	device_unregister(accelerometer_dev);
err_create_accelerometer_device:
	class_destroy(htc_accelerometer_class);
err_create_class:

	return ret;
}

static void st_accel_work_report(struct work_struct *work)
{
	struct st_sensor_data *adata = container_of((struct delayed_work *)work,
			struct st_sensor_data, work);

		irq_work_queue(&adata->iio_irq_work);
		schedule_delayed_work(&adata->work, msecs_to_jiffies(adata->pollrate));
}

static void iio_trigger_work(struct irq_work *work)
{
	struct st_sensor_data *adata = container_of((struct irq_work *)work,
					struct st_sensor_data, iio_irq_work);

	iio_trigger_poll(adata->trig, iio_get_time_ns());
}

static void st_accel_get_offset_data(struct st_sensor_data *adata)
{
	struct device_node *offset = NULL;
	int cali_size = 0;
	unsigned char *cali_data = NULL;
	int i = 0;

	adata->has_offset = false;
	memset(adata->calib_data, 0, 4);
	if ((offset = of_find_node_by_path(CALIBRATION_DATA_PATH))) {
		cali_data = (unsigned char*) of_get_property(offset, G_SENSOR_FLASH_DATA, &cali_size);
		if (cali_data) {
			for (i = 0; (i < cali_size) && (i < 4); i++) {
				adata->calib_data[i] = cali_data[i] ;
			}
		}
		if(adata->calib_data[3] == 0x67)
			adata->has_offset = true;
		else {
			adata->has_offset = false;
			memset(adata->calib_data, 0, 3);
		}
	}
}

ssize_t st_accel_get_k_value(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct st_sensor_data *adata = iio_priv(dev_get_drvdata(dev));

	return sprintf(buf, "%d %d %d\n", adata->calib_data[0], adata->calib_data[1], adata->calib_data[2]);
}

ssize_t st_accel_set_k_value(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	char *str_buf;
	char *running;
	long temp_calib_data = 0;
	int error;
	char *token;

	struct st_sensor_data *adata = iio_priv(dev_get_drvdata(dev));

	str_buf = kstrndup(buf, size, GFP_KERNEL);
	if (str_buf == NULL) {
		return -ENOMEM;
	}
	running = str_buf;

	token = strsep(&running, " ");

	if (token)
		error = kstrtol(token, 10, &temp_calib_data);
	else {
		if (token == NULL)
			return -EINVAL;
	}
	if (error) {
		kfree(str_buf);
		return error;
	}
	if(((temp_calib_data >> 24) & 0xFF) == 0x67) {
		adata->calib_data[0] = temp_calib_data >> 16 & 0xFF;
		adata->calib_data[1] = temp_calib_data >> 8 & 0xFF;
		adata->calib_data[2] = temp_calib_data & 0xFF;
		adata->has_offset = true;
	}

	return size;
}


static ST_SENSOR_DEV_ATTR_SAMP_FREQ();
static ST_SENSORS_DEV_ATTR_SAMP_FREQ_AVAIL();
static ST_SENSORS_DEV_ATTR_SCALE_AVAIL(in_accel_scale_available);
static IIO_DEVICE_ATTR(set_k_value, S_IWUSR | S_IRUSR, st_accel_get_k_value, st_accel_set_k_value, 0);
static struct attribute *st_accel_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_accel_scale_available.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_set_k_value.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_accel_attribute_group = {
	.attrs = st_accel_attributes,
};

static const struct iio_info accel_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_accel_attribute_group,
	.read_raw = &st_accel_read_raw,
	.write_raw = &st_accel_write_raw,
};

#ifdef CONFIG_IIO_TRIGGER
static const struct iio_trigger_ops st_accel_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = ST_ACCEL_TRIGGER_SET_STATE,
};
#define ST_ACCEL_TRIGGER_OPS (&st_accel_trigger_ops)
#else
#define ST_ACCEL_TRIGGER_OPS NULL
#endif

int st_accel_common_probe(struct iio_dev *indio_dev)
{
	int err;
	struct st_sensor_data *adata = iio_priv(indio_dev);

	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &accel_info;

	err = st_sensors_check_device_support(indio_dev,
				ARRAY_SIZE(st_accel_sensors), st_accel_sensors);
	if (err < 0) {
		I("Not use ST G-sensor\n");
		goto st_accel_common_probe_error;
	}

	adata->multiread_bit = adata->sensor->multi_read_bit;
	indio_dev->channels = adata->sensor->ch;
	indio_dev->num_channels = ST_SENSORS_NUMBER_ALL_CHANNELS;

	adata->current_fullscale = (struct st_sensor_fullscale_avl *)
						&adata->sensor->fs.fs_avl[0];

	adata->odr = 100;
	adata->pollrate = 200;
	st_accel_get_offset_data(adata);

	err = st_sensors_init_sensor(indio_dev);
	if (err < 0)
		goto st_accel_common_probe_error;

	if (adata->get_irq_data_ready(indio_dev) > 0) {
		err = st_accel_allocate_ring(indio_dev);
		if (err < 0)
			goto st_accel_common_probe_error;

		err = st_sensors_allocate_trigger(indio_dev,
						 ST_ACCEL_TRIGGER_OPS);
		if (err < 0)
			goto st_accel_probe_trigger_error;
	}

	err = iio_device_register(indio_dev);
	if (err)
		goto st_accel_device_register_error;

	err = st_accel_registerAttr(indio_dev);
	if (err)
		goto st_accel_registerAttr_error;

	qpnp_vibrator_register_notifier(&qpnp_vibrator_status_handler);

	INIT_DELAYED_WORK(&adata->work, st_accel_work_report);
	init_irq_work(&adata->iio_irq_work, iio_trigger_work);

	return err;


st_accel_registerAttr_error:
	iio_device_unregister(indio_dev);
st_accel_device_register_error:
	if (adata->get_irq_data_ready(indio_dev) > 0)
		st_sensors_deallocate_trigger(indio_dev);
st_accel_probe_trigger_error:
	if (adata->get_irq_data_ready(indio_dev) > 0)
		st_accel_deallocate_ring(indio_dev);
st_accel_common_probe_error:
	return err;
}
EXPORT_SYMBOL(st_accel_common_probe);

void st_accel_common_remove(struct iio_dev *indio_dev)
{
	struct st_sensor_data *adata = iio_priv(indio_dev);

	qpnp_vibrator_unregister_notifier(&qpnp_vibrator_status_handler);

	iio_device_unregister(indio_dev);
	if (adata->get_irq_data_ready(indio_dev) > 0) {
		st_sensors_deallocate_trigger(indio_dev);
		st_accel_deallocate_ring(indio_dev);
	}
	iio_device_free(indio_dev);
}
EXPORT_SYMBOL(st_accel_common_remove);

MODULE_AUTHOR("Denis Ciocca <denis.ciocca@st.com>");
MODULE_DESCRIPTION("STMicroelectronics accelerometers driver");
MODULE_LICENSE("GPL v2");
