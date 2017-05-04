/*
 * STMicroelectronics sensors library driver
 *
 * Copyright 2012-2013 STMicroelectronics Inc.
 *
 * Denis Ciocca <denis.ciocca@st.com>
 *
 * Licensed under the GPL-2.
 */

#ifndef ST_SENSORS_H
#define ST_SENSORS_H

#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/irqreturn.h>
#include <linux/iio/trigger.h>
#include <linux/bitops.h>
#include <linux/irq_work.h>

#define ST_SENSORS_TX_MAX_LENGTH		2
#define ST_SENSORS_RX_MAX_LENGTH		6

#define ST_SENSORS_ODR_LIST_MAX			10
#define ST_SENSORS_FULLSCALE_AVL_MAX		10

#define ST_SENSORS_NUMBER_ALL_CHANNELS		4
#define ST_SENSORS_NUMBER_DATA_CHANNELS		3
#define ST_SENSORS_ENABLE_ALL_AXIS		0x07
#define ST_SENSORS_BYTE_FOR_CHANNEL		2
#define ST_SENSORS_SCAN_X			0
#define ST_SENSORS_SCAN_Y			1
#define ST_SENSORS_SCAN_Z			2
#define ST_SENSORS_DEFAULT_12_REALBITS		12
#define ST_SENSORS_DEFAULT_16_REALBITS		16
#define ST_SENSORS_DEFAULT_POWER_ON_VALUE	0x01
#define ST_SENSORS_DEFAULT_POWER_OFF_VALUE	0x00
#define ST_SENSORS_DEFAULT_WAI_ADDRESS		0x0f
#define ST_SENSORS_DEFAULT_AXIS_ADDR		0x20
#define ST_SENSORS_DEFAULT_AXIS_MASK		0x07
#define ST_SENSORS_DEFAULT_AXIS_N_BIT		3

#define ST_SENSORS_MAX_NAME			17
#define ST_SENSORS_MAX_4WAI			7

#define ST_SENSORS_LSM_CHANNELS(device_type, index, mod, endian, bits, addr) \
{ \
	.type = device_type, \
	.modified = 1, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
			BIT(IIO_CHAN_INFO_SCALE), \
	.scan_index = index, \
	.channel2 = mod, \
	.address = addr, \
	.scan_type = { \
		.sign = 's', \
		.realbits = bits, \
		.shift = 16 - bits, \
		.storagebits = 16, \
		.endianness = endian, \
	}, \
}

#define ST_SENSOR_DEV_ATTR_SAMP_FREQ() \
		IIO_DEV_ATTR_SAMP_FREQ(S_IWUSR | S_IRUGO, \
			st_sensors_sysfs_get_sampling_frequency, \
			st_sensors_sysfs_set_sampling_frequency)

#define ST_SENSORS_DEV_ATTR_SAMP_FREQ_AVAIL() \
		IIO_DEV_ATTR_SAMP_FREQ_AVAIL( \
			st_sensors_sysfs_sampling_frequency_avail)

#define ST_SENSORS_DEV_ATTR_SCALE_AVAIL(name) \
		IIO_DEVICE_ATTR(name, S_IRUGO, \
			st_sensors_sysfs_scale_avail, NULL , 0);

struct st_sensor_odr_avl {
	unsigned int hz;
	u8 value;
};

struct st_sensor_odr {
	u8 addr;
	u8 mask;
	struct st_sensor_odr_avl odr_avl[ST_SENSORS_ODR_LIST_MAX];
};

struct st_sensor_power {
	u8 addr;
	u8 mask;
	u8 value_off;
	u8 value_on;
};

struct st_sensor_axis {
	u8 addr;
	u8 mask;
};

struct st_sensor_fullscale_avl {
	unsigned int num;
	u8 value;
	unsigned int gain;
	unsigned int gain2;
};

struct st_sensor_fullscale {
	u8 addr;
	u8 mask;
	struct st_sensor_fullscale_avl fs_avl[ST_SENSORS_FULLSCALE_AVL_MAX];
};

struct st_sensor_bdu {
	u8 addr;
	u8 mask;
};

struct st_sensor_data_ready_irq {
	u8 addr;
	u8 mask;
	struct {
		u8 en_addr;
		u8 en_mask;
	} ig1;
};

struct st_sensor_transfer_buffer {
	struct mutex buf_lock;
	u8 rx_buf[ST_SENSORS_RX_MAX_LENGTH];
	u8 tx_buf[ST_SENSORS_TX_MAX_LENGTH] ____cacheline_aligned;
};

struct st_sensor_transfer_function {
	int (*read_byte) (struct st_sensor_transfer_buffer *tb,
				struct device *dev, u8 reg_addr, u8 *res_byte);
	int (*write_byte) (struct st_sensor_transfer_buffer *tb,
				struct device *dev, u8 reg_addr, u8 data);
	int (*read_multiple_byte) (struct st_sensor_transfer_buffer *tb,
		struct device *dev, u8 reg_addr, int len, u8 *data,
							bool multiread_bit);
};

struct st_sensors {
	u8 wai;
	char sensors_supported[ST_SENSORS_MAX_4WAI][ST_SENSORS_MAX_NAME];
	struct iio_chan_spec *ch;
	struct st_sensor_odr odr;
	struct st_sensor_power pw;
	struct st_sensor_axis enable_axis;
	struct st_sensor_fullscale fs;
	struct st_sensor_bdu bdu;
	struct st_sensor_data_ready_irq drdy_irq;
	bool multi_read_bit;
	unsigned int bootime;
};

struct st_sensor_data {
	struct device *dev;
	struct iio_trigger *trig;
	struct st_sensors *sensor;
	struct st_sensor_fullscale_avl *current_fullscale;

	bool enabled;
	bool multiread_bit;

	char *buffer_data;

	unsigned int odr;

	unsigned int (*get_irq_data_ready) (struct iio_dev *indio_dev);

	const struct st_sensor_transfer_function *tf;
	struct st_sensor_transfer_buffer tb;

	struct delayed_work work;
	struct irq_work iio_irq_work;
	unsigned int pollrate;
	bool has_offset;
	int8_t calib_data[4];
};

#ifdef CONFIG_IIO_BUFFER
irqreturn_t st_sensors_trigger_handler(int irq, void *p);

int st_sensors_get_buffer_element(struct iio_dev *indio_dev, u8 *buf);
#endif

#ifdef CONFIG_IIO_TRIGGER
int st_sensors_allocate_trigger(struct iio_dev *indio_dev,
				const struct iio_trigger_ops *trigger_ops);

void st_sensors_deallocate_trigger(struct iio_dev *indio_dev);

#else
static inline int st_sensors_allocate_trigger(struct iio_dev *indio_dev,
				const struct iio_trigger_ops *trigger_ops)
{
	return 0;
}
static inline void st_sensors_deallocate_trigger(struct iio_dev *indio_dev)
{
	return;
}
#endif

int st_sensors_init_sensor(struct iio_dev *indio_dev);

int st_sensors_set_enable(struct iio_dev *indio_dev, bool enable);

int st_sensors_set_axis_enable(struct iio_dev *indio_dev, u8 axis_enable);

int st_sensors_set_odr(struct iio_dev *indio_dev, unsigned int odr);

int st_sensors_set_dataready_irq(struct iio_dev *indio_dev, bool enable);

int st_sensors_set_fullscale_by_gain(struct iio_dev *indio_dev, int scale);

int st_sensors_read_info_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *ch, int *val);

int st_sensors_check_device_support(struct iio_dev *indio_dev,
			int num_sensors_list, const struct st_sensors *sensors);

ssize_t st_sensors_sysfs_get_sampling_frequency(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t st_sensors_sysfs_set_sampling_frequency(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);

ssize_t st_sensors_sysfs_sampling_frequency_avail(struct device *dev,
				struct device_attribute *attr, char *buf);

ssize_t st_sensors_sysfs_scale_avail(struct device *dev,
				struct device_attribute *attr, char *buf);

#endif 
