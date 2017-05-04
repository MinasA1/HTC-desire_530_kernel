/*
 * LM3697 BL Driver
 *
 * Simple driver for TEXAS Instruments LM3697 Backlight driver chip
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/debug_display.h>
#include <linux/delay.h>

#define LM3697_LED_DEV "LM3697-BL"
#define LM3697_NAME "lm3697-bl"

#define MAX_BRIGHTNESS	255
#define BANK_NONE	0x00
#define BANK_A		0x01
#define BANK_B		0x02

/* LM3697 Register */
#define LM3697_REVISION_REG			0x00
#define LM3697_SW_RESET_REG			0x01
#define LM3697_HVLED_CURR_SINK_OUT_CFG_REG	0x10
#define LM3697_CTL_A_RAMP_TIME_REG		0x11
#define LM3697_CTL_B_RAMP_TIME_REG		0x12
#define LM3697_CTL_RUNTIME_RAMP_TIME_REG	0x13
#define LM3697_CTL_RUNTIME_RAMP_CFG_REG		0x14
#define LM3697_BRIGHTNESS_CFG_REG		0x16
#define LM3697_CTL_A_FULL_SCALE_CURR_REG	0x17
#define LM3697_CTL_B_FULL_SCALE_CURR_REG	0x18
#define LM3697_HVLED_CURR_SINK_FEEDBACK_REG	0x19
#define LM3697_BOOST_CTL_REG			0x1A
#define LM3697_AUTO_FREQ_THRESHOLD_REG		0x1B
#define LM3697_PWM_CFG_REG			0x1C
#define LM3697_CTL_A_BRIGHTNESS_LSB_REG		0x20
#define LM3697_CTL_A_BRIGHTNESS_MSB_REG		0x21
#define LM3697_CTL_B_BRIGHTNESS_LSB_REG		0x22
#define LM3697_CTL_B_BRIGHTNESS_MSB_REG		0x23
#define LM3697_CTL_B_BANK_EN_REG		0x24

/**
 * struct lm3697_data
 * @led_dev: led class device
 * @client: i2c client
 * @adapter: i2c adapter
 * @lock: mutex lock
 * @work: work
 * @addr: i2c address
 * @brighness: previous brightness value
 * @enable: enable/disable
 * @bank_A: enable/disable
 * @bank_B: enable/disable
 * @pwm_cfg: pwm configuration
 * @boost_en: Boost Control
 * @ctl_bank_en: Control Bank Enables
 * @hvled_curr_sink_feedback: HVLED Current Sink Feedback Enables
 */
struct lm3697_data {
	struct led_classdev led_dev;
	struct i2c_client *client;
	struct i2c_adapter *adapter;
	unsigned short addr;
	struct mutex		lock;
	struct work_struct	work;
	enum led_brightness brightness;
	bool enable;
	bool bank_A;
	bool bank_B;
	u8 pwm_cfg;
	u8 boost_ctl;
	u8 brightness_cfg;
	u8 ctl_bank_en;
	u8 hvled_curr_sink_feedback;
};

static inline int platform_write_i2c_block(struct i2c_adapter *i2c_bus
								, u8 page
								, u8 offset
								, u16 count
								, u8 *values
								)
{
	struct i2c_msg msg;
	u8 *buffer;
	int ret;

	buffer = kmalloc(count + 1, GFP_KERNEL);
	if (!buffer) {
		pr_err("%s:%d buffer allocation failed\n",__FUNCTION__,__LINE__);
		return -ENOMEM;
	}

	buffer[0] = offset;
	memmove(&buffer[1], values, count);

	msg.flags = 0;
	msg.addr = page >> 1;
	msg.buf = buffer;
	msg.len = count + 1;

	ret = i2c_transfer(i2c_bus, &msg, 1);

	kfree(buffer);

	if (ret != 1) {
		pr_err("%s:%d I2c write failed 0x%02x:0x%02x\n"
				,__FUNCTION__,__LINE__, page, offset);
		ret = -EIO;
	} else
		ret = 0;

	return ret;
}

void lm3697_init_registers(struct lm3697_data *drvdata)
{
	platform_write_i2c_block(drvdata->adapter, drvdata->addr, LM3697_BOOST_CTL_REG, 0x01, &drvdata->boost_ctl);
	platform_write_i2c_block(drvdata->adapter, drvdata->addr, LM3697_PWM_CFG_REG, 0x01, &drvdata->pwm_cfg);
	platform_write_i2c_block(drvdata->adapter, drvdata->addr, LM3697_BRIGHTNESS_CFG_REG, 0x01, &drvdata->brightness_cfg);
#ifdef CONFIG_MACH_A12
	/* Disable HVLED2 for A12 */
	platform_write_i2c_block(drvdata->adapter, drvdata->addr, LM3697_HVLED_CURR_SINK_FEEDBACK_REG, 0x01, &drvdata->hvled_curr_sink_feedback);
	/* Delay before backlight ON to avoid flicker when resumes for A12 */
	usleep(120000);
#endif
	platform_write_i2c_block(drvdata->adapter, drvdata->addr, LM3697_CTL_B_BANK_EN_REG, 0x01, &drvdata->ctl_bank_en);

	drvdata->enable = true;
	PR_DISP_INFO("%s: \n",__func__);
}

void lm3697_set_brightness(struct lm3697_data *drvdata, int brt_val)
{
	u8 brt_LSB = 0xff;
	u8 brt_MSB = brt_val & 0xff;

	if (!drvdata->enable && brt_val)
		lm3697_init_registers(drvdata);

	if (!brt_val) {
		brt_LSB = 0;
		brt_MSB = 0;
	}
	/* set the brightness in brightness control register */
	if (drvdata->bank_B && drvdata->enable) {
		platform_write_i2c_block(drvdata->adapter, drvdata->addr, LM3697_CTL_B_BRIGHTNESS_LSB_REG, 0x01, &brt_LSB);
		platform_write_i2c_block(drvdata->adapter, drvdata->addr, LM3697_CTL_B_BRIGHTNESS_MSB_REG, 0x01, &brt_MSB);

		drvdata->brightness = brt_val;

		if (!drvdata->brightness)
			drvdata->enable = false;

		PR_DISP_INFO("%s: btr_val=%d \n",__func__, brt_val);
	}
}

static void __lm3697_work(struct lm3697_data *led,
				enum led_brightness value)
{
	mutex_lock(&led->lock);
	lm3697_set_brightness(led, value);
	mutex_unlock(&led->lock);
}

static void lm3697_work(struct work_struct *work)
{
	struct lm3697_data *drvdata = container_of(work,
					struct lm3697_data, work);

	__lm3697_work(drvdata, drvdata->led_dev.brightness);

	return;
}

static void lm3697_brightness_set(struct led_classdev *led_cdev,
				     enum led_brightness brt_val)
{
	struct lm3697_data *drvdata;

	drvdata = container_of(led_cdev, struct lm3697_data, led_dev);

	schedule_work(&drvdata->work);
}

static int lm3697_get_dt_data(struct device *dev, struct lm3697_data *drvdata)
{
	int rc;
	u32 tmp;
	struct device_node *of_node = NULL;
	of_node = dev->of_node;

	rc = of_property_read_u32(of_node, "boost-ctl", &tmp);
	if (rc) {
		pr_err("%s:%d, dt not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	drvdata->boost_ctl = (!rc ? tmp : 0);
	PR_DISP_DEBUG("%s : boost_ctl=0x%x\n",__func__, drvdata->boost_ctl);

	rc = of_property_read_u32(of_node, "pwm-cfg", &tmp);
	if (rc) {
		pr_err("%s:%d, dt not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	drvdata->pwm_cfg = (!rc ? tmp : 0);
	PR_DISP_DEBUG("%s : pwm_cfg=0x%x\n",__func__, drvdata->pwm_cfg);

	rc = of_property_read_u32(of_node, "brightness-cfg", &tmp);
	if (rc) {
		pr_err("%s:%d, dt not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	drvdata->brightness_cfg = (!rc ? tmp : 0);
	PR_DISP_DEBUG("%s : brightness_cfg=0x%x\n",__func__, drvdata->brightness_cfg);

	rc = of_property_read_u32(of_node, "ctl-bank-en", &tmp);
	if (rc) {
		pr_err("%s:%d, dt not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	drvdata->ctl_bank_en = (!rc ? tmp : 0);
	PR_DISP_DEBUG("%s : ctl_bank_en=0x%x\n",__func__, drvdata->ctl_bank_en);

#ifdef CONFIG_MACH_A12
	rc = of_property_read_u32(of_node, "hvled-curr-sink-feedback", &tmp);
	if (rc) {
		pr_err("%s:%d, dt not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	drvdata->hvled_curr_sink_feedback = (!rc ? tmp : 0);
	PR_DISP_DEBUG("%s : hvled_curr_sink_feedback=0x%x\n",__func__, drvdata->hvled_curr_sink_feedback);
#endif

	/* select bank */
	if (drvdata->ctl_bank_en & 0x01)
		drvdata->bank_A = true;
	if (drvdata->ctl_bank_en & 0x02)
		drvdata->bank_B = true;

	PR_DISP_INFO("%s : bank_A=%d bank_B=%d\n",__func__, drvdata->bank_A, drvdata->bank_B);

	return 0;
}

static int lm3697_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct lm3697_data *drvdata;
	int err = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s : I2C_FUNC_I2C not supported\n", __func__);
		err = -EIO;
		goto err_out;
	}

	if (!client->dev.of_node) {
		pr_err("%s : no device node\n", __func__);
		err = -ENOMEM;
		goto err_out;
	}

	drvdata = kzalloc(sizeof(struct lm3697_data), GFP_KERNEL);
	if (drvdata == NULL) {
		pr_err("%s : kzalloc failed\n", __func__);
		err = -ENOMEM;
		goto err_out;
	}

	drvdata->client = client;
	drvdata->adapter = client->adapter;
	drvdata->addr = client->addr;
	drvdata->brightness = LED_OFF;
	drvdata->enable = true;  /* set to false if cont-splash is disabled */
	drvdata->led_dev.default_trigger = "bl-led-i2c-trigger";
	drvdata->led_dev.name = LM3697_LED_DEV;
	drvdata->led_dev.brightness_set = lm3697_brightness_set;
	drvdata->led_dev.max_brightness = MAX_BRIGHTNESS;

	mutex_init(&drvdata->lock);
	INIT_WORK(&drvdata->work, lm3697_work);

	err = lm3697_get_dt_data(&client->dev, drvdata);
	if(err < 0) {
		pr_err("%s : get dt failed\n", __func__);
		err = -ENOMEM;
		goto err_init;
	}

	i2c_set_clientdata(client, drvdata);

	err = led_classdev_register(&client->dev, &drvdata->led_dev);
	if (err < 0) {
		pr_err("%s : Register led class failed\n", __func__);
		err = -ENODEV;
		goto err_init;
	}

	PR_DISP_INFO("%s \n",__func__);
	return 0;

err_init:
	kfree(drvdata);
err_out:
	return err;
}

static int lm3697_remove(struct i2c_client *client)
{
	struct lm3697_data *drvdata = i2c_get_clientdata(client);

	led_classdev_unregister(&drvdata->led_dev);

	kfree(drvdata);
	return 0;
}

static const struct i2c_device_id lm3697_id[] = {
	{LM3697_NAME, 0},
	{}
};
static struct of_device_id match_table[] = {
       {.compatible = "ti-lm3697",}
};

MODULE_DEVICE_TABLE(i2c, lm3697_id);

static struct i2c_driver lm3697_i2c_driver = {
	.probe = lm3697_probe,
	.remove = lm3697_remove,
	.id_table = lm3697_id,
	.driver = {
		.name = LM3697_NAME,
		.owner = THIS_MODULE,
		.of_match_table = match_table,
	},
};

module_i2c_driver(lm3697_i2c_driver);
MODULE_DESCRIPTION("Back Light driver for LM3697");
MODULE_LICENSE("GPL v2");
