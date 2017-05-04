/*
 * STMicroelectronics sensors buffer library driver
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
#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>
#include <linux/interrupt.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/irqreturn.h>

#include <linux/i2c/sensors/common/st_sensors_htc.h>
#include <linux/qpnp_vibrator.h>

#define WA             (15)
static int filtered_data[3] = { 0 };

#define VIB_TIMEOUT    (1 * HZ)
static bool vib_enable = 0;
static int vib_duration = 0;
static unsigned long vib_jiffies = 0;

bool drop_event = false;

void set_wa_values(bool enable, int duration)
{
	vib_enable = enable;
	vib_duration = duration;
	if (!enable)
		vib_jiffies = jiffies;
	return;
}

static void st_sensors_add_offset(struct st_sensor_data *sdata, u8 *buf)
{
	int16_t temp_buf[3];
	int i = 0;

	if(!sdata->has_offset)
		return;

	for(i = 0; i < 3 ; i++){
		temp_buf[i] = (buf[2*i+1] << 8) + buf[2*i];
		temp_buf[i] += (sdata->calib_data[i] << 4);
		buf[2*i] = temp_buf[i] & 0xFF;
		buf[2*i+1] = (temp_buf[i] >> 8) & 0xFF;
	}
}

int st_sensors_get_buffer_element(struct iio_dev *indio_dev, u8 *buf)
{
	int i, n = 0, len, t;
	int current_data[3];
	u8 addr[ST_SENSORS_NUMBER_DATA_CHANNELS];
	struct st_sensor_data *sdata = iio_priv(indio_dev);

	for (i = 0; i < ST_SENSORS_NUMBER_DATA_CHANNELS; i++) {
		if (test_bit(i, indio_dev->active_scan_mask)) {
			addr[n] = indio_dev->channels[i].address;
			n++;
		}
	}
	switch (n) {
	case 1:
		len = sdata->tf->read_multiple_byte(&sdata->tb, sdata->dev,
			addr[0], ST_SENSORS_BYTE_FOR_CHANNEL, buf,
			sdata->multiread_bit);

		if (len > 0) {
			current_data[0] = (s16)*((u16 *)&buf[0]);
			filtered_data[0] = (WA * current_data[0] + ((100 - WA) * filtered_data[0])) / 100;
			*((u16 *)&buf[0]) = (vib_enable || !time_after(jiffies, vib_jiffies + VIB_TIMEOUT)) ? (u16)((s16)filtered_data[0]) : (u16)((s16)current_data[0]);
		}

		break;
	case 2:
		if ((addr[1] - addr[0]) == ST_SENSORS_BYTE_FOR_CHANNEL) {
			len = sdata->tf->read_multiple_byte(&sdata->tb,
					sdata->dev, addr[0],
					ST_SENSORS_BYTE_FOR_CHANNEL*n,
					buf, sdata->multiread_bit);
		} else {
			u8 rx_array[ST_SENSORS_BYTE_FOR_CHANNEL*
				    ST_SENSORS_NUMBER_DATA_CHANNELS];
			len = sdata->tf->read_multiple_byte(&sdata->tb,
				sdata->dev, addr[0],
				ST_SENSORS_BYTE_FOR_CHANNEL*
				ST_SENSORS_NUMBER_DATA_CHANNELS,
				rx_array, sdata->multiread_bit);
			if (len < 0)
				goto read_data_channels_error;

			for (i = 0; i < n * ST_SENSORS_NUMBER_DATA_CHANNELS;
									i++) {
				if (i < n)
					buf[i] = rx_array[i];
				else
					buf[i] = rx_array[n + i];
			}
			len = ST_SENSORS_BYTE_FOR_CHANNEL*n;
		}

		if (len > 0) {
			for (t = 0; t < 2; t++) {
				current_data[t] = (s16)*((u16 *)&buf[t * 2]);
				filtered_data[t] = (WA * current_data[t] + ((100 - WA) * filtered_data[t])) / 100;
				*((u16 *)&buf[t * 2]) = (vib_enable || !time_after(jiffies, vib_jiffies + VIB_TIMEOUT)) ? (u16)((s16)filtered_data[t]) : (u16)((s16)current_data[t]);
			}
		}

		break;
	case 3:
		len = sdata->tf->read_multiple_byte(&sdata->tb, sdata->dev,
			addr[0], ST_SENSORS_BYTE_FOR_CHANNEL*
			ST_SENSORS_NUMBER_DATA_CHANNELS,
			buf, sdata->multiread_bit);
		st_sensors_add_offset(sdata, buf);

		
		if (len > 0) {
			for (t = 0; t < 3; t++) {
				current_data[t] = (s16)*((u16 *)&buf[t * 2]);
				filtered_data[t] = (WA * current_data[t] + ((100 - WA) * filtered_data[t])) / 100;
				*((u16 *)&buf[t * 2]) = (vib_enable || !time_after(jiffies, vib_jiffies + VIB_TIMEOUT)) ? (u16)((s16)filtered_data[t]) : (u16)((s16)current_data[t]);
			}
		}

		break;
	default:
		len = -EINVAL;
		goto read_data_channels_error;
	}
	if (len != ST_SENSORS_BYTE_FOR_CHANNEL*n) {
		len = -EIO;
		goto read_data_channels_error;
	}

read_data_channels_error:
	return len;
}
EXPORT_SYMBOL(st_sensors_get_buffer_element);

irqreturn_t st_sensors_trigger_handler(int irq, void *p)
{
	int len;
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct st_sensor_data *sdata = iio_priv(indio_dev);

	len = st_sensors_get_buffer_element(indio_dev, sdata->buffer_data);
	if (len < 0)
		goto st_sensors_get_buffer_element_error;

	if (indio_dev->scan_timestamp)
		*(s64 *)((u8 *)sdata->buffer_data +
				ALIGN(len, sizeof(s64))) = pf->timestamp;

	
	if (drop_event) {
		drop_event = false;
	} else {
		iio_push_to_buffers(indio_dev, sdata->buffer_data);
	}

st_sensors_get_buffer_element_error:
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}
EXPORT_SYMBOL(st_sensors_trigger_handler);

MODULE_AUTHOR("Denis Ciocca <denis.ciocca@st.com>");
MODULE_DESCRIPTION("STMicroelectronics ST-sensors buffer");
MODULE_LICENSE("GPL v2");
