/* Copyright (c) 2014 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)	"[BATT][SMB] %s: " fmt, __func__
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>
#include <linux/qpnp/qpnp-adc.h>

#ifdef CONFIG_HTC_BATT_8960
#include <mach/htc_charger.h>
#include <mach/htc_gauge.h>
#include <linux/smb358-charger.h>
#include <linux/usb/cable_detect.h>
#include <linux/htc_flags.h>
#include <linux/qpnp/qpnp-vm-bms.h>
#include <linux/wakelock.h>
#include <linux/of_batterydata.h>
#include <mach/htc_battery_cell.h>
#include <mach/htc_battery_common.h>
#endif

#ifdef pr_debug
#undef pr_debug
#endif
#define pr_debug(fmt, args...) do { \
		if (flag_enable_bms_charger_log) \
			printk(KERN_INFO pr_fmt(fmt), ## args); \
	} while (0)

#define _SMB358_MASK(BITS, POS) \
	((unsigned char)(((1 << (BITS)) - 1) << (POS)))
#define SMB358_MASK(LEFT_BIT_POS, RIGHT_BIT_POS) \
		_SMB358_MASK((LEFT_BIT_POS) - (RIGHT_BIT_POS) + 1, \
			(RIGHT_BIT_POS))

#define CHG_CURRENT_CTRL_REG		0x0
#define CHG_OTH_CURRENT_CTRL_REG	0x1
#define VARIOUS_FUNC_REG		0x2
#define VFLOAT_REG			0x3
#define CHG_CTRL_REG			0x4
#define STAT_AND_TIMER_CTRL_REG		0x5
#define CHG_PIN_EN_CTRL_REG		0x6
#define THERM_A_CTRL_REG		0x7
#define SYSOK_AND_USB3_REG		0x8
#define OTHER_CTRL_REG			0x9
#define FAULT_INT_REG			0xC
#define STATUS_INT_REG			0xD

#define CFG_SFY_TIMER_CTRL_REG		0x05
#define SAFETY_TIME_DISABLE_BIT		(BIT(2) | BIT(3))
#define SAFETY_TIME_MINUTES_SHIFT	2
#define SAFETY_TIME_MINUTES_MASK	SMB358_MASK(3, 2)
#define AICL_CONTROL_REG	0x2
#define AICL_ENABLED_BIT	BIT(4)

#define CMD_A_REG			0x30
#define CMD_B_REG			0x31

#define IRQ_A_REG			0x35
#define IRQ_B_REG			0x36
#define IRQ_C_REG			0x37
#define IRQ_D_REG			0x38
#define IRQ_E_REG			0x39
#define IRQ_E_USBIN_OV_BIT		BIT(2)
#define IRQ_E_USBIN_UV_BIT		BIT(0)

#define IRQ_F_REG			0x3A
#define IRQ_F_POWER_OK_BIT		BIT(0)

#define STATUS_A_REG			0x3B
#define STATUS_A_SOFT_LIMIT_BIT	BIT(7)
#define STATUS_C_REG			0x3D
#define STATUS_C_FAST_CHG_BIT	BIT(0)
#define STATUS_C_FAST_CHG_BIT	BIT(0)

#define STATUS_D_REG			0x3E
#define STATUS_E_REG			0x3F

enum {
	IRQ_A_REG_SEQUENCE,
	IRQ_B_REG_SEQUENCE,
	IRQ_C_REG_SEQUENCE,
	IRQ_D_REG_SEQUENCE,
	IRQ_E_REG_SEQUENCE,
	IRQ_F_REG_SEQUENCE,

	TOTAL_IRQ_REG_SEQUENCE
};

#define CHG_INHI_EN_MASK			BIT(1)
#define CHG_INHI_EN_BIT				BIT(1)
#define CMD_A_CHG_ENABLE_BIT			BIT(1)
#define CMD_A_VOLATILE_W_PERM_BIT		BIT(7)
#define CMD_A_CHG_SUSP_EN_BIT			BIT(2)
#define CMD_A_CHG_SUSP_EN_MASK			BIT(2)
#define CMD_A_OTG_ENABLE_BIT			BIT(4)
#define CMD_A_OTG_ENABLE_MASK			BIT(4)
#define CMD_B_CHG_HC_ENABLE_BIT			BIT(0)
#define USB3_ENABLE_BIT				BIT(5)
#define USB3_ENABLE_MASK			BIT(5)
#define CMD_B_CHG_USB_500_900_ENABLE_BIT	BIT(1)
#define CHG_CTRL_AUTO_RECHARGE_ENABLE_BIT	0x0
#define CHG_CTRL_CURR_TERM_END_CHG_BIT		0x0
#define CHG_CTRL_BATT_MISSING_DET_THERM_IO	SMB358_MASK(5, 4)
#define CHG_CTRL_AUTO_RECHARGE_MASK		BIT(7)
#define CHG_AUTO_RECHARGE_DIS_BIT		BIT(7)
#define CHG_CTRL_CURR_TERM_END_MASK		BIT(6)
#define CHG_CTRL_BATT_MISSING_DET_MASK		SMB358_MASK(5, 4)
#define CHG_CTRL_APSD_EN_BIT			BIT(2)
#define CHG_CTRL_APSD_EN_MASK			BIT(2)
#define CHG_ITERM_MASK				0x07
#define CHG_PIN_CTRL_USBCS_REG_BIT		0x0
#define CHG_PIN_CTRL_CHG_EN_LOW_PIN_BIT		SMB358_MASK(6, 5)
#define CHG_PIN_CTRL_CHG_EN_LOW_REG_BIT		0x0
#define CHG_PIN_CTRL_CHG_EN_MASK		SMB358_MASK(6, 5)

#define CHG_LOW_BATT_THRESHOLD \
				SMB358_MASK(3, 0)
#define CHG_PIN_CTRL_USBCS_REG_MASK		BIT(4)
#define CHG_PIN_CTRL_APSD_IRQ_BIT		BIT(1)
#define CHG_PIN_CTRL_APSD_IRQ_MASK		BIT(1)
#define CHG_PIN_CTRL_CHG_ERR_IRQ_BIT		BIT(2)
#define CHG_PIN_CTRL_CHG_ERR_IRQ_MASK		BIT(2)
#define VARIOUS_FUNC_USB_SUSP_EN_REG_BIT	BIT(6)
#define VARIOUS_FUNC_USB_SUSP_MASK		BIT(6)
#define FAULT_INT_HOT_COLD_HARD_BIT		BIT(7)
#define FAULT_INT_HOT_COLD_SOFT_BIT		BIT(6)
#define FAULT_INT_INPUT_OV_BIT			BIT(3)
#define FAULT_INT_INPUT_UV_BIT			BIT(2)
#define FAULT_INT_AICL_COMPLETE_BIT		BIT(1)
#define STATUS_INT_CHG_TIMEOUT_BIT		BIT(7)
#define STATUS_INT_OTG_DETECT_BIT		BIT(6)
#define STATUS_INT_BATT_OV_BIT			BIT(5)
#define STATUS_INT_CHGING_BIT			BIT(4)
#define STATUS_INT_CHG_INHI_BIT			BIT(3)
#define STATUS_INT_INOK_BIT			BIT(2)
#define STATUS_INT_MISSING_BATT_BIT		BIT(1)
#define STATUS_INT_LOW_BATT_BIT			BIT(0)
#define THERM_A_THERM_MONITOR_EN_BIT		0x0
#define THERM_A_THERM_MONITOR_EN_MASK		BIT(4)
#define VFLOAT_MASK				0x3F

#define IRQ_A_HOT_HARD_BIT			BIT(6)
#define IRQ_A_COLD_HARD_BIT			BIT(4)
#define IRQ_A_HOT_SOFT_BIT			BIT(2)
#define IRQ_A_COLD_SOFT_BIT			BIT(0)
#define IRQ_B_BATT_MISSING_BIT			BIT(4)
#define IRQ_B_BATT_LOW_BIT			BIT(2)
#define IRQ_B_BATT_OV_BIT			BIT(6)
#define IRQ_B_PRE_FAST_CHG_BIT			BIT(0)
#define IRQ_C_RECHARGE_BIT			BIT(4)
#define IRQ_C_TAPER_CHG_BIT			BIT(2)
#define IRQ_C_TERM_BIT				BIT(0)
#define IRQ_C_INT_OVER_TEMP_BIT			BIT(6)
#define IRQ_D_COMPLETE_CHG_TIMEOUT_BIT		BIT(2)
#define IRQ_D_CHG_TIMEOUT_BIT			(BIT(0) | BIT(2))
#define IRQ_D_AICL_DONE_BIT			BIT(4)
#define IRQ_D_APSD_COMPLETE			BIT(6)
#define IRQ_E_INPUT_UV_BIT			BIT(0)
#define IRQ_E_INPUT_OV_BIT			BIT(2)
#define IRQ_E_AFVC_ACTIVE                       BIT(4)
#define IRQ_F_OTG_VALID_BIT			BIT(2)
#define IRQ_F_OTG_BATT_FAIL_BIT			BIT(4)
#define IRQ_F_OTG_OC_BIT			BIT(6)
#define IRQ_F_POWER_OK				BIT(0)

#define CHG_STATUS_SHIFT				1
#define STATUS_C_CHARGING_MASK			SMB358_MASK(2, 1)
#define STATUS_C_FAST_CHARGING			BIT(2)
#define STATUS_C_PRE_CHARGING			BIT(1)
#define STATUS_C_TAPER_CHARGING			SMB358_MASK(2, 1)
#define STATUS_C_CHG_ERR_STATUS_BIT		BIT(6)
#define STATUS_C_CHG_ENABLE_STATUS_BIT		BIT(0)
#define STATUS_C_CHG_HOLD_OFF_BIT		BIT(3)
#define STATUS_D_CHARGING_PORT_MASK \
				SMB358_MASK(3, 0)
#define STATUS_D_PORT_ACA_DOCK			BIT(3)
#define STATUS_D_PORT_SDP			BIT(2)
#define STATUS_D_PORT_DCP			BIT(1)
#define STATUS_D_PORT_CDP			BIT(0)
#define STATUS_D_PORT_OTHER			SMB358_MASK(1, 0)
#define STATUS_D_PORT_ACA_A			(BIT(2) | BIT(0))
#define STATUS_D_PORT_ACA_B			SMB358_MASK(2, 1)
#define STATUS_D_PORT_ACA_C			SMB358_MASK(2, 0)

#define USB2_MIN_CURRENT_MA		100
#define USB2_MAX_CURRENT_MA		500
#define USB3_MIN_CURRENT_MA		150
#define USB3_MAX_CURRENT_MA		900
#define AC_CHG_CURRENT_MASK		0x70
#define AC_CHG_CURRENT_SHIFT		4
#define SMB358_IRQ_REG_COUNT		6
#define SMB358_FAST_CHG_MIN_MA		200
#define SMB358_FAST_CHG_MAX_MA		2000
#define SMB358_FAST_CHG_SHIFT		5
#define SMB_FAST_CHG_CURRENT_MASK	0xE0
#define SMB358_DEFAULT_BATT_CAPACITY	50
#define SMB358_BATT_GOOD_THRE_2P5	0x1

enum {
	USER	= BIT(0),
	THERMAL = BIT(1),
	CURRENT = BIT(2),
	SOC	= BIT(3),
};

struct smb358_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct smb358_charger {
	struct i2c_client	*client;
	struct device		*dev;

	bool			inhibit_disabled;
	bool			recharge_disabled;
	int			recharge_mv;
	bool			iterm_disabled;
	int			iterm_ma;
	int			batt_full_criteria;
	int			vfloat_mv;
	int			chg_valid_gpio;
	int			chg_valid_act_low;
	int			chg_present;
	int			fake_battery_soc;
	int			safety_time;
	bool			chg_autonomous_mode;
	bool			disable_apsd;
	bool			using_pmic_therm;
	bool			battery_missing;
	bool			is_embeded_batt;
	const char		*bms_psy_name;
	bool			resume_completed;
	bool			irq_waiting;
	bool			bms_controlled_charging;
	struct mutex		read_write_lock;
	struct mutex		path_suspend_lock;
	struct mutex		charging_disable_lock;
	struct mutex		irq_complete;
	u8			irq_cfg_mask[2];
	int			irq_gpio;
	int			charging_disabled;
	int			fastchg_current_max_ma;
	int			input_current_ma;
#if defined(CONFIG_HTC_BATT_GPIO_OVP)
	int			batt_ovp_irq;
#endif
	unsigned int		cool_bat_ma;
	unsigned int		warm_bat_ma;
	unsigned int		cool_bat_mv;
	unsigned int		warm_bat_mv;

	
#if defined(CONFIG_DEBUG_FS)
	struct dentry		*debug_root;
	u32			peek_poke_address;
#endif
	
	bool			batt_full;
	bool			batt_hot;
	bool			batt_cold;
	bool			batt_warm;
	bool			batt_cool;
	bool			jeita_supported;
	int			charging_disabled_status;
	int			usb_suspended;

	
	struct power_supply	*usb_psy;
	struct power_supply	*bms_psy;
	struct power_supply	batt_psy;

	
	struct smb358_regulator	otg_vreg;

	
	struct qpnp_vadc_chip	*vadc_dev;
	struct qpnp_adc_tm_chip	*adc_tm_dev;
	struct qpnp_adc_tm_btm_param	adc_param;
	int			cold_bat_decidegc;
	int			hot_bat_decidegc;
	int			cool_bat_decidegc;
	int			warm_bat_decidegc;
	int			bat_present_decidegc;
	
	struct regulator	*vcc_i2c;
	enum htc_ftm_power_source_type	ftm_src;
#if defined(CONFIG_HTC_BATT_GPIO_OVP)
	struct delayed_work     check_external_ovp_work;
#endif
	struct delayed_work		update_ovp_uvp_work;
	struct delayed_work		eoc_work;
	struct delayed_work		set_hsml_work;
	struct delayed_work		retry_aicl_work;

	struct wake_lock		eoc_worker_lock;
#ifdef CONFIG_HTC_BATT_8960
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_init;
#endif
};

struct smb_irq_info {
	const char		*name;
	int			(*smb_irq)(struct smb358_charger *chip,
							u8 rt_stat);
	int			high;
	int			low;
};

struct irq_handler_info {
	u8			stat_reg;
	u8			val;
	u8			prev_val;
	struct smb_irq_info	irq_info[4];
};

static int chg_current[] = {
	300, 500, 700, 1000, 1200, 1500, 1800, 2000,
};

static int fast_chg_current[] = {
	200, 450, 600, 900, 1300, 1500, 1800, 2000,
};

#ifdef CONFIG_HTC_BATT_8960
static struct smb358_charger *the_chip;
enum htc_power_source_type pwr_src;
static void update_ovp_uvp_worker(struct work_struct *work);
#if defined(CONFIG_HTC_BATT_GPIO_OVP)
static void check_external_ovp_worker(struct work_struct *work);
#endif
int smb358_chg_recharge_threshold_set(struct smb358_charger *chip);
int smb358_chg_safety_timer_set(struct smb358_charger *chip);
static int is_ac_online(void);
static void start_eoc_work(struct smb358_charger *chip);
static int smb358_get_aicl_result(struct smb358_charger *chip, int *aicl_result, bool *aicl_done);
int smb358_chg_aicl_enable(struct smb358_charger *chip, bool enable);
static void smb358_idic_detection(struct smb358_charger *chip);
static int64_t read_vbus_voltage(struct smb358_charger *chip);
void smb358_set_charger_configuration(void);
bool smb358_is_hw_reload_happened(struct smb358_charger *chip);
static bool flag_keep_charge_on;
static bool flag_force_ac_chg;
static bool flag_pa_fake_batt_temp;
static bool flag_disable_safety_timer;
static bool flag_disable_temp_protection;
static bool flag_enable_bms_charger_log;
static int test_power_monitor;
static int test_ftm_mode;
static int test_download_mode;
static int hsml_target_ma;
static int ovp = false;
static int uvp = false;
#if defined(CONFIG_HTC_BATT_GPIO_OVP)
static int start_ovp = 1;
#endif
static bool is_batt_full = false;
static bool is_batt_full_eoc_stop = false;
static int eoc_count; 
static int eoc_count_by_curr; 
static bool is_ac_safety_timeout = false;
static unsigned int chg_limit_current; 

static bool first_vbus_irq = true; 
static int iusb_limit_reason;
static bool iusb_limit_enable = false;
static int retry_aicl_cnt = 0;

static bool gs_is_bad_aicl_result = false;

#define IDIC_VOLTAGE_SAMPLE    3
static bool is_idic_detect_done = false;
static bool is_hv_battery = true;
static int vbat_sample[IDIC_VOLTAGE_SAMPLE] = {0};

#define SMB358_EOC_CHECK_PERIOD_MS	60000
#define TIMER_382MINS		0x00
#define TIMER_764MINS		0x04
#define TIMER_1527MINS		0x08

#define SMB358_CHG_I_MIN_MA		200
#define SMB358_INPUT_LIMIT_MA 700

static bool chg_enable_status;
static bool pwrsrc_enable_status;
#endif

static inline long ABS(long x) { return x >= 0 ? x : -x; }

static unsigned long dischg_last_time_ms;
static bool is_limit_IUSB = false;

#ifndef CONFIG_HTC_BATT_8960
static char *pm_batt_supplied_to[] = {
	"bms",
};
#endif

#define I2C_RETRY_TIMES		10
static int __smb358_read_reg(struct smb358_charger *chip, u8 reg, u8 *val)
{
	s32 ret;
#ifdef CONFIG_HTC_BATT_8960
	int retry;

#endif

	ret = i2c_smbus_read_byte_data(chip->client, reg);
	if (ret < 0) {
		dev_err(chip->dev,
			"i2c read fail: can't read from %02x: %d\n", reg, ret);
#ifdef CONFIG_HTC_BATT_8960
		if (ret == -ENOTCONN) {
			for (retry = 0; retry < I2C_RETRY_TIMES && ret == -ENOTCONN; retry++) {
				ret = 0;
				msleep(10);
				ret = i2c_smbus_read_byte_data(chip->client, reg);
				pr_warn("retry %d times ret=%d\n", retry, ret);
			}
			if (retry >= I2C_RETRY_TIMES) {
				pr_err("retry over %d times still fail ret=%d\n",
						I2C_RETRY_TIMES, ret);
				return ret;
			}
		}
#else
		return ret;
#endif
	} else {
		*val = ret;
	}

	return 0;
}

static int __smb358_write_reg(struct smb358_charger *chip, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		dev_err(chip->dev,
			"i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);

		return ret;
	}

	return 0;
}

static int smb358_read_reg(struct smb358_charger *chip, int reg,
						u8 *val)
{
	int rc;

	mutex_lock(&chip->read_write_lock);
	rc = __smb358_read_reg(chip, reg, val);
	mutex_unlock(&chip->read_write_lock);

	return rc;
}

#ifdef CONFIG_HTC_BATT_8960
static int smb358_read_bytes(struct smb358_charger *chip, int reg,
						u8 *val, u8 bytes)
{
	s32 rc;

	mutex_lock(&chip->read_write_lock);
	rc = i2c_smbus_read_i2c_block_data(chip->client, reg, bytes, val);
	if (rc < 0)
		dev_err(chip->dev,
			"i2c read fail: can't read %d bytes from %02x: %d\n",
							bytes, reg, rc);
	mutex_unlock(&chip->read_write_lock);

	return (rc < 0) ? rc : 0;
}
#endif 

static int smb358_write_reg(struct smb358_charger *chip, int reg,
						u8 val)
{
	int rc;

	mutex_lock(&chip->read_write_lock);
	rc = __smb358_write_reg(chip, reg, val);
	mutex_unlock(&chip->read_write_lock);

	return rc;
}

static int smb358_masked_write(struct smb358_charger *chip, int reg,
							u8 mask, u8 val)
{
	s32 rc;
	u8 temp;

	mutex_lock(&chip->read_write_lock);
	rc = __smb358_read_reg(chip, reg, &temp);
	if (rc) {
		dev_err(chip->dev,
			"smb358_read_reg Failed: reg=%03X, rc=%d\n", reg, rc);
		goto out;
	}
	temp &= ~mask;
	temp |= val & mask;
	rc = __smb358_write_reg(chip, reg, temp);
	if (rc) {
		dev_err(chip->dev,
			"smb358_write Failed: reg=%03X, rc=%d\n", reg, rc);
	}
out:
	mutex_unlock(&chip->read_write_lock);
	return rc;
}

static int smb358_enable_volatile_writes(struct smb358_charger *chip)
{
	int rc;

	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_VOLATILE_W_PERM_BIT,
						CMD_A_VOLATILE_W_PERM_BIT);
	if (rc)
		dev_err(chip->dev, "Couldn't write VOLATILE_W_PERM_BIT rc=%d\n",
				rc);

	return rc;
}

static int smb358_fastchg_current_set(struct smb358_charger *chip,
					unsigned int fastchg_current)
{
	int i;

	if ((fastchg_current < SMB358_FAST_CHG_MIN_MA) ||
		(fastchg_current >  SMB358_FAST_CHG_MAX_MA)) {
		dev_err(chip->dev, "bad fastchg current mA=%d asked to set\n",
						fastchg_current);
		return -EINVAL;
	}

	for (i = ARRAY_SIZE(fast_chg_current) - 1; i >= 0; i--) {
		if (fast_chg_current[i] <= fastchg_current)
			break;
	}

	if (i < 0) {
		dev_err(chip->dev, "Invalid current setting %dmA\n",
						fastchg_current);
		i = 0;
	}

	i = i << SMB358_FAST_CHG_SHIFT;
	pr_debug("fastchg limit=%d setting %02x\n",
					fastchg_current, i);

	return smb358_masked_write(chip, CHG_CURRENT_CTRL_REG,
				SMB_FAST_CHG_CURRENT_MASK, i);
}

#define MIN_FLOAT_MV		3500
#define MAX_FLOAT_MV		4500
#define VFLOAT_STEP_MV		20
#define VFLOAT_4350MV		4350
static int smb358_float_voltage_set(struct smb358_charger *chip, int vfloat_mv)
{
	u8 temp;

	if ((vfloat_mv < MIN_FLOAT_MV) || (vfloat_mv > MAX_FLOAT_MV)) {
		dev_err(chip->dev, "bad float voltage mv =%d asked to set\n",
					vfloat_mv);
		return -EINVAL;
	}

	if (VFLOAT_4350MV == vfloat_mv)
		temp = 0x2B;
	else if (vfloat_mv > VFLOAT_4350MV)
		temp = (vfloat_mv - MIN_FLOAT_MV) / VFLOAT_STEP_MV - 1;
	else
		temp = (vfloat_mv - MIN_FLOAT_MV) / VFLOAT_STEP_MV;

	return smb358_masked_write(chip, VFLOAT_REG, VFLOAT_MASK, temp);
}

#define CHG_ITERM_30MA			0x00
#define CHG_ITERM_40MA			0x01
#define CHG_ITERM_60MA			0x02
#define CHG_ITERM_80MA			0x03
#define CHG_ITERM_100MA			0x04
#define CHG_ITERM_125MA			0x05
#define CHG_ITERM_150MA			0x06
#define CHG_ITERM_200MA			0x07
static int smb358_term_current_set(struct smb358_charger *chip)
{
	u8 reg = 0;
	int rc;

	if (chip->iterm_ma != -EINVAL) {
		if (chip->iterm_disabled)
			dev_err(chip->dev, "Error: Both iterm_disabled and iterm_ma set\n");

		if (chip->iterm_ma <= 30)
			reg = CHG_ITERM_30MA;
		else if (chip->iterm_ma <= 40)
			reg = CHG_ITERM_40MA;
		else if (chip->iterm_ma <= 60)
			reg = CHG_ITERM_60MA;
		else if (chip->iterm_ma <= 80)
			reg = CHG_ITERM_80MA;
		else if (chip->iterm_ma <= 100)
			reg = CHG_ITERM_100MA;
		else if (chip->iterm_ma <= 125)
			reg = CHG_ITERM_125MA;
		else if (chip->iterm_ma <= 150)
			reg = CHG_ITERM_150MA;
		else
			reg = CHG_ITERM_200MA;

		rc = smb358_masked_write(chip, CHG_CURRENT_CTRL_REG,
							CHG_ITERM_MASK, reg);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't set iterm rc = %d\n", rc);
			return rc;
		}
	}

	if (chip->iterm_disabled) {
		rc = smb358_masked_write(chip, CHG_CTRL_REG,
					CHG_CTRL_CURR_TERM_END_MASK,
					CHG_CTRL_CURR_TERM_END_MASK);
		if (rc) {
			dev_err(chip->dev, "Couldn't set iterm rc = %d\n",
								rc);
			return rc;
		}
	} else {
		rc = smb358_masked_write(chip, CHG_CTRL_REG,
					CHG_CTRL_CURR_TERM_END_MASK, 0);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't enable iterm rc = %d\n", rc);
			return rc;
		}
	}

	return 0;
}

#define VFLT_300MV			0x0C
#define VFLT_200MV			0x08
#define VFLT_100MV			0x04
#define VFLT_50MV			0x00
#define VFLT_MASK			0x0C
static int smb358_recharge_and_inhibit_set(struct smb358_charger *chip)
{
	u8 reg = 0;
	int rc;

	if (chip->recharge_disabled)
		rc = smb358_masked_write(chip, CHG_CTRL_REG,
		CHG_CTRL_AUTO_RECHARGE_MASK, CHG_AUTO_RECHARGE_DIS_BIT);
	else
		rc = smb358_masked_write(chip, CHG_CTRL_REG,
			CHG_CTRL_AUTO_RECHARGE_MASK, 0x0);
	if (rc) {
		dev_err(chip->dev,
			"Couldn't set auto recharge en reg rc = %d\n", rc);
	}

	if (chip->inhibit_disabled)
		rc = smb358_masked_write(chip, CHG_OTH_CURRENT_CTRL_REG,
					CHG_INHI_EN_MASK, 0x0);
	else
		rc = smb358_masked_write(chip, CHG_OTH_CURRENT_CTRL_REG,
					CHG_INHI_EN_MASK, CHG_INHI_EN_BIT);
	if (rc) {
		dev_err(chip->dev,
			"Couldn't set inhibit en reg rc = %d\n", rc);
	}

	if (chip->recharge_mv != -EINVAL) {
		if (chip->recharge_mv <= 50)
			reg = VFLT_50MV;
		else if (chip->recharge_mv <= 100)
			reg = VFLT_100MV;
		else if (chip->recharge_mv <= 200)
			reg = VFLT_200MV;
		else
			reg = VFLT_300MV;

		pr_info("setting:%dmV, reg:0x%X\n", chip->recharge_mv, reg);
		rc = smb358_masked_write(chip, CHG_OTH_CURRENT_CTRL_REG,
						VFLT_MASK, reg);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't set inhibit threshold rc = %d\n", rc);
			return rc;
		}
	}

	return 0;
}

static int smb358_chg_otg_regulator_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smb358_charger *chip = rdev_get_drvdata(rdev);

	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_OTG_ENABLE_BIT,
							CMD_A_OTG_ENABLE_BIT);
	if (rc)
		dev_err(chip->dev, "Couldn't enable OTG mode rc=%d, reg=%2x\n",
								rc, CMD_A_REG);
	return rc;
}

static int smb358_chg_otg_regulator_disable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smb358_charger *chip = rdev_get_drvdata(rdev);

	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_OTG_ENABLE_BIT, 0);
	if (rc)
		dev_err(chip->dev, "Couldn't disable OTG mode rc=%d, reg=%2x\n",
								rc, CMD_A_REG);
	return rc;
}

static int smb358_chg_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	u8 reg = 0;
	struct smb358_charger *chip = rdev_get_drvdata(rdev);

	rc = smb358_read_reg(chip, CMD_A_REG, &reg);
	if (rc) {
		dev_err(chip->dev,
			"Couldn't read OTG enable bit rc=%d, reg=%2x\n",
							rc, CMD_A_REG);
		return rc;
	}

	return  (reg & CMD_A_OTG_ENABLE_BIT) ? 1 : 0;
}

struct regulator_ops smb358_chg_otg_reg_ops = {
	.enable		= smb358_chg_otg_regulator_enable,
	.disable	= smb358_chg_otg_regulator_disable,
	.is_enabled	= smb358_chg_otg_regulator_is_enable,
};

static int smb358_regulator_init(struct smb358_charger *chip)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};

	init_data = of_get_regulator_init_data(chip->dev, chip->dev->of_node);
	if (!init_data) {
		dev_err(chip->dev, "Allocate memory failed\n");
		return -ENOMEM;
	}

	
	if (init_data->constraints.name) {
		chip->otg_vreg.rdesc.owner = THIS_MODULE;
		chip->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->otg_vreg.rdesc.ops = &smb358_chg_otg_reg_ops;
		chip->otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = chip->dev->of_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->otg_vreg.rdev = regulator_register(
					&chip->otg_vreg.rdesc, &cfg);
		if (IS_ERR(chip->otg_vreg.rdev)) {
			rc = PTR_ERR(chip->otg_vreg.rdev);
			chip->otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev,
					"OTG reg failed, rc=%d\n", rc);
		}
	}
	return rc;
}

static int __smb358_charging_disable(struct smb358_charger *chip, bool disable)
{
	int rc;

	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_CHG_ENABLE_BIT,
			disable ? 0 : CMD_A_CHG_ENABLE_BIT);
	if (rc < 0)
		pr_err("Couldn't set CHG_ENABLE_BIT diable = %d, rc = %d\n",
				disable, rc);
	return rc;
}

#ifndef CONFIG_HTC_BATT_8960
static int smb358_charging_disable(struct smb358_charger *chip,
						int reason, int disable)
{
	int rc = 0;
	int disabled;

	disabled = chip->charging_disabled_status;

	pr_debug("reason = %d requested_disable = %d disabled_status = %d\n",
						reason, disable, disabled);

	if (disable == true)
		disabled |= reason;
	else
		disabled &= ~reason;

	if (!!disabled == !!chip->charging_disabled_status)
		goto skip;

	rc = __smb358_charging_disable(chip, !!disabled);
	if (rc) {
		pr_err("Failed to disable charging rc = %d\n", rc);
		return rc;
	} else {
	
#ifndef CONFIG_HTC_BATT_8960
		power_supply_changed(&chip->batt_psy);
#endif
	}

skip:
	chip->charging_disabled_status = disabled;
	return rc;
}
#else
static int batt_charging_disabled; 
#define BATT_CHG_DISABLED_BIT_EOC	(1)
#define BATT_CHG_DISABLED_BIT_KDRV	(1<<1)
#define BATT_CHG_DISABLED_BIT_USR	(1<<2)
#define BATT_CHG_DISABLED_BIT_TEMP	(1<<3)
#define BATT_CHG_DISABLED_BIT_BAT	(1<<4)
#define BATT_CHG_DISABLED_BIT_SAFETY (1<<5)

static int smb358_chg_disable_charger(struct smb358_charger *chip,
		int disable, int reason)
{
	int rc;

	pr_info("disable=%d, reason=0x%X\n", disable, reason);
	mutex_lock(&chip->charging_disable_lock);
	if (disable)
		batt_charging_disabled |= reason;	
	else
		batt_charging_disabled &= ~reason;	
	rc = __smb358_charging_disable(chip, !!batt_charging_disabled);
	if (rc)
		pr_err("Failed rc=%d\n", rc);
	mutex_unlock(&chip->charging_disable_lock);

	return rc;
}
#endif 

static int smb358_hw_init(struct smb358_charger *chip)
{
	int rc;
	u8 reg = 0, mask = 0;

	if (chip->chg_autonomous_mode) {
		pr_debug("Charger configured for autonomous mode\n");
		return 0;
	}

	rc = smb358_enable_volatile_writes(chip);
	if (rc) {
		dev_err(chip->dev, "Couldn't configure volatile writes rc=%d\n",
				rc);
		return rc;
	}

	
	reg = CHG_CTRL_BATT_MISSING_DET_THERM_IO;
	mask = CHG_CTRL_BATT_MISSING_DET_MASK;
	rc = smb358_masked_write(chip, CHG_CTRL_REG, mask, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set CHG_CTRL_REG rc=%d\n", rc);
		return rc;
	}
	
	reg = CHG_PIN_CTRL_USBCS_REG_BIT | CHG_PIN_CTRL_CHG_EN_LOW_REG_BIT |
		CHG_PIN_CTRL_APSD_IRQ_BIT | CHG_PIN_CTRL_CHG_ERR_IRQ_BIT;
	mask = CHG_PIN_CTRL_CHG_EN_MASK | CHG_PIN_CTRL_USBCS_REG_MASK |
		CHG_PIN_CTRL_APSD_IRQ_MASK | CHG_PIN_CTRL_CHG_ERR_IRQ_MASK;
	rc = smb358_masked_write(chip, CHG_PIN_EN_CTRL_REG, mask, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set CHG_PIN_EN_CTRL_REG rc=%d\n",
				rc);
		return rc;
	}

	
	rc = smb358_masked_write(chip, VARIOUS_FUNC_REG,
		VARIOUS_FUNC_USB_SUSP_MASK, VARIOUS_FUNC_USB_SUSP_EN_REG_BIT);
	if (rc) {
		dev_err(chip->dev, "Couldn't set VARIOUS_FUNC_REG rc=%d\n",
				rc);
		return rc;
	}

	if (!chip->disable_apsd)
		reg = CHG_CTRL_APSD_EN_BIT;
	else
		reg = 0;

	rc = smb358_masked_write(chip, CHG_CTRL_REG,
				CHG_CTRL_APSD_EN_MASK, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set CHG_CTRL_REG rc=%d\n",
				rc);
		return rc;
	}
	
	reg = FAULT_INT_HOT_COLD_HARD_BIT | FAULT_INT_HOT_COLD_SOFT_BIT
		| FAULT_INT_INPUT_UV_BIT | FAULT_INT_AICL_COMPLETE_BIT
		| FAULT_INT_INPUT_OV_BIT;
	rc = smb358_write_reg(chip, FAULT_INT_REG, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set FAULT_INT_REG rc=%d\n", rc);
		return rc;
	}
	reg = STATUS_INT_CHG_TIMEOUT_BIT | STATUS_INT_OTG_DETECT_BIT |
		STATUS_INT_BATT_OV_BIT | STATUS_INT_CHGING_BIT |
		STATUS_INT_CHG_INHI_BIT | STATUS_INT_INOK_BIT |
		STATUS_INT_LOW_BATT_BIT | STATUS_INT_MISSING_BATT_BIT;
	rc = smb358_write_reg(chip, STATUS_INT_REG, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set STATUS_INT_REG rc=%d\n", rc);
		return rc;
	}
	
	rc = smb358_masked_write(chip, THERM_A_CTRL_REG,
		THERM_A_THERM_MONITOR_EN_MASK, THERM_A_THERM_MONITOR_EN_BIT);
	if (rc) {
		dev_err(chip->dev, "Couldn't set THERM_A_CTRL_REG rc=%d\n",
				rc);
		return rc;
	}
	
	rc = smb358_fastchg_current_set(chip, chip->fastchg_current_max_ma);
	if (rc) {
		dev_err(chip->dev, "Couldn't set fastchg current rc=%d\n", rc);
		return rc;
	}

	
	rc = smb358_float_voltage_set(chip, chip->vfloat_mv);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't set float voltage rc = %d\n", rc);
		return rc;
	}

	
	rc = smb358_term_current_set(chip);
	if (rc)
		dev_err(chip->dev, "Couldn't set term current rc=%d\n", rc);

	
	rc = smb358_recharge_and_inhibit_set(chip);
	if (rc)
		dev_err(chip->dev, "Couldn't set recharge para rc=%d\n", rc);

	
	if (chip->charging_disabled) {
#ifndef CONFIG_HTC_BATT_8960
		rc = smb358_charging_disable(chip, USER, 1);
#else
		rc = smb358_chg_disable_charger(chip, 1,
							BATT_CHG_DISABLED_BIT_USR);
#endif
		if (rc)
			dev_err(chip->dev, "Couldn't '%s' charging rc = %d\n",
			chip->charging_disabled ? "disable" : "enable", rc);
	} else {
		rc = __smb358_charging_disable(chip, 0);
		if (rc)
			dev_err(chip->dev, "Couldn't enable charging\n");
	}

	rc = smb358_masked_write(chip, OTHER_CTRL_REG, CHG_LOW_BATT_THRESHOLD,
						SMB358_BATT_GOOD_THRE_2P5);
	if (rc)
		dev_err(chip->dev, "Couldn't write OTHER_CTRL_REG, rc = %d\n",
								rc);
#ifdef CONFIG_HTC_BATT_8960
	
	if (chip->safety_time == 0 || flag_keep_charge_on || flag_disable_safety_timer) {
		
		rc = smb358_masked_write(chip, CFG_SFY_TIMER_CTRL_REG,
				SAFETY_TIME_MINUTES_MASK, SAFETY_TIME_DISABLE_BIT);
		if (rc)
			dev_err(chip->dev, "Couldn't write CFG_SFY_TIMER_CTRL_REG, rc = %d\n",
								rc);
	} else if (chip->safety_time != -EINVAL) {
		if (chip->safety_time <= 382)
			reg = TIMER_382MINS;
		else if (chip->safety_time <= 764)
			reg = TIMER_764MINS;
		else if (chip->safety_time <= 1527)
			reg = TIMER_1527MINS;
		else
			reg = TIMER_1527MINS;

		pr_info("Safety_timer: %dmins, reg:0x%X\n", chip->safety_time, reg);
		rc = smb358_masked_write(chip, CFG_SFY_TIMER_CTRL_REG,
				SAFETY_TIME_MINUTES_MASK, reg);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set safety timer rc = %d\n",
								rc);
			return rc;
		}
	}
#endif

	return rc;
}

#ifndef CONFIG_HTC_BATT_8960
static enum power_supply_property smb358_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};
#endif

static int smb358_get_prop_batt_status(struct smb358_charger *chip)
{
	int rc;
	u8 reg = 0;

#ifndef CONFIG_HTC_BATT_8960
	if (chip->batt_full)
		return POWER_SUPPLY_STATUS_FULL;
#else
	if (is_batt_full_eoc_stop)
		return POWER_SUPPLY_STATUS_FULL;
#endif

	rc = smb358_read_reg(chip, STATUS_C_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read STAT_C rc = %d\n", rc);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	pr_debug("STATUS_C_REG=%x\n", reg);

	if (reg & STATUS_C_CHG_HOLD_OFF_BIT)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

	if ((reg & STATUS_C_CHARGING_MASK) &&
			!(reg & STATUS_C_CHG_ERR_STATUS_BIT))
		return POWER_SUPPLY_STATUS_CHARGING;

	return POWER_SUPPLY_STATUS_DISCHARGING;
}

static int smb358_get_prop_batt_present(struct smb358_charger *chip)
{
	return !chip->battery_missing;
}

#ifndef CONFIG_HTC_BATT_8960
static int smb358_get_prop_batt_capacity(struct smb358_charger *chip)
{
	union power_supply_propval ret = {0, };

	if (chip->fake_battery_soc >= 0)
		return chip->fake_battery_soc;

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, &ret);
		return ret.intval;
	}

	pr_warn("Couldn't get bms_psy, return default capacity\n");
	return SMB358_DEFAULT_BATT_CAPACITY;
}
#endif

static int smb358_get_prop_charge_type(struct smb358_charger *chip)
{
	int rc;
	u8 reg = 0;

	rc = smb358_read_reg(chip, STATUS_C_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read STAT_C rc = %d\n", rc);
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}

	pr_debug("STATUS_C_REG=%x\n", reg);

	reg &= STATUS_C_CHARGING_MASK;

	if (reg == STATUS_C_FAST_CHARGING)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	else if (reg == STATUS_C_TAPER_CHARGING)
		return POWER_SUPPLY_CHARGE_TYPE_TAPER;
	else if (reg == STATUS_C_PRE_CHARGING)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	else
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

#ifndef CONFIG_HTC_BATT_8960
static int smb358_get_prop_batt_health(struct smb358_charger *chip)
{
	union power_supply_propval ret = {0, };

	if (chip->batt_hot)
		ret.intval = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (chip->batt_cold)
		ret.intval = POWER_SUPPLY_HEALTH_COLD;
	else if (chip->batt_warm)
		ret.intval = POWER_SUPPLY_HEALTH_WARM;
	else if (chip->batt_cool)
		ret.intval = POWER_SUPPLY_HEALTH_COOL;
	else
		ret.intval = POWER_SUPPLY_HEALTH_GOOD;

	return ret.intval;
}
#endif 

static int
smb358_get_prop_battery_voltage_now(struct smb358_charger *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	rc = qpnp_vadc_read(chip->vadc_dev, VBAT_SNS, &results);
	if (rc) {
		pr_err("Unable to read vbat rc=%d\n", rc);
		return 0;
	}
	return results.physical;
}

#define DEFAULT_TEMP 250
static int smb358_get_prop_batt_temp(struct smb358_charger *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

#ifndef CONFIG_HTC_BATT_8960
	if (!smb358_get_prop_batt_present(chip))
		return DEFAULT_TEMP;
#endif

	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM, &results);
	if (rc) {
		pr_err("Unable to read batt temperature rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	pr_debug("get_bat_temp %d, %lld\n",
		results.adc_code, results.physical);

	return (int)results.physical;
}

static int __smb358_path_suspend(struct smb358_charger *chip, bool suspend)
{
	int rc;

	rc = smb358_masked_write(chip, CMD_A_REG,
			CMD_A_CHG_SUSP_EN_MASK,
				suspend ? CMD_A_CHG_SUSP_EN_BIT : 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set CMD_A reg, rc = %d\n", rc);
	return rc;
}

#ifndef CONFIG_HTC_BATT_8960
static int smb358_path_suspend(struct smb358_charger *chip, int reason,
							bool suspend)
{
	int rc = 0;
	int suspended;

	mutex_lock(&chip->path_suspend_lock);
	suspended = chip->usb_suspended;

	if (suspend == false)
		suspended &= ~reason;
	else
		suspended |= reason;

	if (!chip->usb_suspended && suspended) {
		rc = __smb358_path_suspend(chip, true);
		chip->usb_suspended = suspended;
		power_supply_set_online(chip->usb_psy, !chip->usb_suspended);
		power_supply_changed(chip->usb_psy);
	} else if (chip->usb_suspended && !suspended) {
		rc = __smb358_path_suspend(chip, false);
		chip->usb_suspended = suspended;
		power_supply_set_online(chip->usb_psy, !chip->usb_suspended);
		power_supply_changed(chip->usb_psy);
	}

	if (rc)
		dev_err(chip->dev, "Couldn't set/unset suspend rc = %d\n", rc);

	mutex_unlock(&chip->path_suspend_lock);
	return rc;
}
#else
#define PWRSRC_DISABLED_BIT_KDRV	(1)
#define PWRSRC_DISABLED_BIT_USER	(1<<1)
#define PWRSRC_DISABLED_BIT_AICL	(1<<2)
#define PWRSRC_DISABLED_BIT_FTM		(1<<3)
#define PWRSRC_DISABLED_BIT_EOC		(1<<4)
#define PWRSRC_DISABLED_BIT_IDIC	(1<<5)

static int pwrsrc_disabled; 

static int smb358_chg_disable_pwrsrc(struct smb358_charger *chip,
		int disable, int reason)
{
	int rc;

	pr_info("disable=%d, reason=0x%X\n", disable, reason);
	mutex_lock(&chip->path_suspend_lock);
	if (disable)
		pwrsrc_disabled |= reason;	
	else
		pwrsrc_disabled &= ~reason;	
	rc = __smb358_path_suspend(chip, !!pwrsrc_disabled);
	if (rc)
		pr_err("Failed rc=%d\n", rc);
	mutex_unlock(&chip->path_suspend_lock);

	return rc;
}
#endif 

static int smb358_set_usb_chg_current(struct smb358_charger *chip,
		int current_ma)
{
	int i, rc = 0;
	u8 reg1 = 0, reg2 = 0, mask = 0;

	pr_debug("USB current_ma = %d\n", current_ma);

	if (chip->chg_autonomous_mode) {
		pr_debug("Charger in autonmous mode\n");
		return 0;
	}

	if (current_ma < USB3_MIN_CURRENT_MA && current_ma != 2)
		current_ma = USB2_MIN_CURRENT_MA;

	if (current_ma == USB2_MIN_CURRENT_MA) {
		
		reg1 &= ~USB3_ENABLE_BIT;
		reg2 &= ~CMD_B_CHG_USB_500_900_ENABLE_BIT;
	} else if (current_ma == USB2_MAX_CURRENT_MA) {
		
		reg1 &= ~USB3_ENABLE_BIT;
		reg2 |= CMD_B_CHG_USB_500_900_ENABLE_BIT;
	} else if (current_ma == USB3_MAX_CURRENT_MA) {
		
		reg1 |= USB3_ENABLE_BIT;
		reg2 |= CMD_B_CHG_USB_500_900_ENABLE_BIT;
	} else if (current_ma > USB2_MAX_CURRENT_MA) {
		
		reg2 |= CMD_B_CHG_HC_ENABLE_BIT;

		for (i = ARRAY_SIZE(chg_current) - 1; i >= 0; i--) {
			if (chg_current[i] <= current_ma)
				break;
		}
		if (i < 0) {
			dev_err(chip->dev, "Cannot find %dmA\n", current_ma);
			i = 0;
		}

		pr_info("current_limit = %d\n", chg_current[i]);

		i = i << AC_CHG_CURRENT_SHIFT;
		rc = smb358_masked_write(chip, CHG_OTH_CURRENT_CTRL_REG,
						AC_CHG_CURRENT_MASK, i);
		if (rc)
			dev_err(chip->dev, "Couldn't set input mA rc=%d\n", rc);
	}

	mask = CMD_B_CHG_HC_ENABLE_BIT | CMD_B_CHG_USB_500_900_ENABLE_BIT;
	rc = smb358_masked_write(chip, CMD_B_REG, mask, reg2);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set charging mode rc = %d\n", rc);

	mask = USB3_ENABLE_MASK;
	rc = smb358_masked_write(chip, SYSOK_AND_USB3_REG, mask, reg1);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set USB3 mode rc = %d\n", rc);

#ifndef CONFIG_HTC_BATT_8960
	
	if (current_ma == 2 && chip->chg_present) {
		rc = smb358_path_suspend(chip, CURRENT, true);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't suspend rc = %d\n", rc);
	} else {
		rc = smb358_path_suspend(chip, CURRENT, false);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't set susp rc = %d\n", rc);
	}
#endif

	return rc;
}

#ifndef CONFIG_HTC_BATT_8960
static int
smb358_batt_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CAPACITY:
		return 1;
	default:
		break;
	}

	return 0;
}

static int bound_soc(int soc)
{
	soc = max(0, soc);
	soc = min(soc, 100);
	return soc;
}

static int smb358_battery_set_property(struct power_supply *psy,
					enum power_supply_property prop,
					const union power_supply_propval *val)
{
	int rc;
	struct smb358_charger *chip = container_of(psy,
				struct smb358_charger, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		if (!chip->bms_controlled_charging)
			return -EINVAL;
		switch (val->intval) {
		case POWER_SUPPLY_STATUS_FULL:
			rc = smb358_charging_disable(chip, SOC, true);
			if (rc < 0) {
				dev_err(chip->dev,
					"Couldn't set charging disable rc = %d\n",
					rc);
			} else {
				chip->batt_full = true;
				pr_debug("status = FULL, batt_full = %d\n",
							chip->batt_full);
			}
			break;
		case POWER_SUPPLY_STATUS_DISCHARGING:
			chip->batt_full = false;
#ifndef CONFIG_HTC_BATT_8960
			power_supply_changed(&chip->batt_psy);
#endif
			pr_debug("status = DISCHARGING, batt_full = %d\n",
							chip->batt_full);
			break;
		case POWER_SUPPLY_STATUS_CHARGING:
			rc = smb358_charging_disable(chip, SOC, false);
			if (rc < 0) {
				dev_err(chip->dev,
				"Couldn't set charging disable rc = %d\n",
								rc);
			} else {
				chip->batt_full = false;
				pr_debug("status = CHARGING, batt_full = %d\n",
							chip->batt_full);
			}
			break;
		default:
			return -EINVAL;
		}
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		smb358_charging_disable(chip, USER, !val->intval);
		smb358_path_suspend(chip, USER, !val->intval);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		chip->fake_battery_soc = bound_soc(val->intval);
#ifndef CONFIG_HTC_BATT_8960
		power_supply_changed(&chip->batt_psy);
#endif
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int smb358_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct smb358_charger *chip = container_of(psy,
				struct smb358_charger, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = smb358_get_prop_batt_status(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = smb358_get_prop_batt_present(chip);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = smb358_get_prop_batt_capacity(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = !(chip->charging_disabled_status & USER);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = smb358_get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = smb358_get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "SMB358";
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = smb358_get_prop_batt_temp(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = smb358_get_prop_battery_voltage_now(chip);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
#endif 

static int safety_timeout(struct smb358_charger *chip, u8 status)
{
	pr_info("status = 0x%02x, chg_src = %d\n", status, pwr_src);

	if(status & IRQ_D_COMPLETE_CHG_TIMEOUT_BIT) {
		pr_info("Safety timer (%dmins) timeout with AC cable = %d\n",
				chip->safety_time, is_ac_online());

		is_ac_safety_timeout = true;
		htc_charger_event_notify(HTC_CHARGER_EVENT_SAFETY_TIMEOUT);
	}

	return 0;
}

#ifdef CONFIG_HTC_BATT_8960
static int downgrade_aicl_result(struct smb358_charger *chip, int mA)
{
	int rc = 0;

	pr_info("re-run AICL by %dmA\n", mA);

	
	rc = smb358_chg_aicl_enable(chip, false);
	if (rc < 0) {
		pr_err("enable/disable AICL failed rc=%d\n", rc);
		return rc;
	}

	
	rc = smb358_set_usb_chg_current(chip, mA);
	if (rc < 0) {
		pr_err("set usb chg current fail rc=%d\n", rc);
		return rc;
	}

	
	rc = smb358_chg_aicl_enable(chip, true);
	if (rc < 0) {
		pr_err("enable/disable AICL failed rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static void retry_aicl_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct smb358_charger *chip = container_of(dwork,
				struct smb358_charger, retry_aicl_work);

	downgrade_aicl_result(chip, USB_MA_1500);
	retry_aicl_cnt++;
}
#endif 

#define RETRY_AICL_TOTAL	2
#define RETRY_AICL_INTERVAL_MS	180000
static int aicl_complete(struct smb358_charger *chip, u8 status)
{
	bool aicl_done;
	int aicl_result;
	int usbin = -EINVAL;
	u8 reg = 0;

	smb358_get_aicl_result(chip, &aicl_result, &aicl_done);
	smb358_read_reg(chip, STATUS_A_REG, &reg);
#ifdef CONFIG_HTC_BATT_8960
	usbin = (int)read_vbus_voltage(chip);
#endif

	pr_info("AICL done=%d,rt_status=0x%X,aicl_result=%d, reg=0x%X, usbin=%d\n",
			aicl_done, status, aicl_result, reg, usbin);

#ifdef CONFIG_HTC_BATT_8960
	
	if ((status & IRQ_D_AICL_DONE_BIT) && aicl_done && is_ac_online()) {
		if (aicl_result == USB_MA_1200) {
			downgrade_aicl_result(chip, USB_MA_1000);
		} else if (aicl_result == USB_MA_1500
				&& !(reg & STATUS_A_SOFT_LIMIT_BIT)) {
			downgrade_aicl_result(chip, USB_MA_1000);
		} else if (aicl_result <= USB_MA_500  &&
				retry_aicl_cnt < RETRY_AICL_TOTAL) {
			
			if (delayed_work_pending(&chip->retry_aicl_work))
				cancel_delayed_work_sync(&chip->retry_aicl_work);
			pr_info("Trigger re-run AICL after 3 minutes, cnt=%d\n", retry_aicl_cnt);
			schedule_delayed_work(&chip->retry_aicl_work,
					msecs_to_jiffies(RETRY_AICL_INTERVAL_MS));
		}
		if (aicl_result < USB_MA_500 &&
				retry_aicl_cnt == RETRY_AICL_TOTAL) {
			
			gs_is_bad_aicl_result = true;
		}
	}
#endif 

	return 0;
}

static int apsd_complete(struct smb358_charger *chip, u8 status)
{
	int rc;
	u8 reg = 0;
	enum power_supply_type type = POWER_SUPPLY_TYPE_UNKNOWN;

	pr_info("status = 0x%02x\n", status);
	if (chip->disable_apsd || status == 0) {
		pr_info("APSD %s, status = %d\n",
			chip->disable_apsd ? "disabled" : "enabled", !!status);
		return 0;
	}

	rc = smb358_read_reg(chip, STATUS_D_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read STATUS D rc = %d\n", rc);
		return rc;
	}

	pr_info("STATUS_D_REG=%x\n", reg);

	switch (reg & STATUS_D_CHARGING_PORT_MASK) {
	case STATUS_D_PORT_ACA_DOCK:
	case STATUS_D_PORT_ACA_C:
	case STATUS_D_PORT_ACA_B:
	case STATUS_D_PORT_ACA_A:
		type = POWER_SUPPLY_TYPE_USB_ACA;
		break;
	case STATUS_D_PORT_CDP:
		type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	case STATUS_D_PORT_DCP:
		type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case STATUS_D_PORT_SDP:
		type = POWER_SUPPLY_TYPE_USB;
		break;
	case STATUS_D_PORT_OTHER:
		type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	default:
		type = POWER_SUPPLY_TYPE_USB;
		break;
	}

	chip->chg_present = !!status;

	pr_info("APSD complete. USB type detected=%d chg_present=%d",
						type, chip->chg_present);
#ifndef CONFIG_HTC_BATT_8960
	power_supply_set_charge_type(chip->usb_psy, type);

	 
	pr_info("%s updating usb_psy present=%d", __func__,
			chip->chg_present);
	power_supply_set_present(chip->usb_psy, chip->chg_present);
#endif

	return 0;
}

#define AC_1A_WA_DIFF_TIME_MS	1000
#define AC_1A_WA_TRIGGER		3
static int chg_uv(struct smb358_charger *chip, u8 status)
{
#ifndef CONFIG_HTC_BATT_8960
	int rc;
#else
	struct timespec xtime;
	unsigned long dischg_time_ms;
	unsigned long cur_time_ms;
	static int count_same_dischg = 0;
	static bool first = true;
	int cable_in;
	int usbin;
	long diff;
	static int prev_cable_in = -1;
	static unsigned long prev_dischg_time_ms = 0;

	xtime = CURRENT_TIME;
	cur_time_ms = xtime.tv_sec * MSEC_PER_SEC + xtime.tv_nsec / NSEC_PER_MSEC;

	usbin = (int)read_vbus_voltage(chip);
#endif

	pr_info("chip->chg_present = %d-> %d, status = 0x%02x, usbin = %d, is_limit_IUSB=%d\n",
			chip->chg_present, !!!status, status, usbin, is_limit_IUSB);

#ifndef CONFIG_HTC_BATT_8960
	
	if (chip->disable_apsd && status == 0) {
		chip->chg_present = true;
		pr_info("%s updating usb_psy present=%d",
				__func__, chip->chg_present);
		power_supply_set_supply_type(chip->usb_psy,
						POWER_SUPPLY_TYPE_USB);
		power_supply_set_present(chip->usb_psy, chip->chg_present);

		if (chip->bms_controlled_charging) {
#ifndef CONFIG_HTC_BATT_8960
			rc = smb358_charging_disable(chip, SOC, false);
#else
			rc = smb358_chg_disable_charger(chip, false,
									BATT_CHG_DISABLED_BIT_EOC);
#endif
			if (rc < 0)
				dev_err(chip->dev,
					"Couldn't disable usb suspend rc = %d\n",
									rc);
		}
	}

	if (status != 0) {
		chip->chg_present = false;
		pr_info("%s updating usb_psy present=%d",
				__func__, chip->chg_present);
	

		power_supply_set_present(chip->usb_psy, chip->chg_present);
	}
	power_supply_changed(chip->usb_psy);

	pr_info("chip->chg_present = %d\n", chip->chg_present);
#endif 

	if (first_vbus_irq) {
		pr_info("Run before BMS init!! Do Nothing.\n");
		return 0;
	}

#ifdef CONFIG_HTC_BATT_8960

	if (status)
		cable_in = 0;
	else
		cable_in = 1;

	if ((prev_cable_in == 1) && (cable_in == 0)) {
		
		dischg_last_time_ms = cur_time_ms;
		dischg_time_ms = 0;
		first = false;
	} else if ((!first) && (prev_cable_in == 0) && (cable_in == 1)){
		
		dischg_time_ms = cur_time_ms - dischg_last_time_ms;

		
		
		diff = dischg_time_ms - prev_dischg_time_ms;

		pr_info("status = %d. "
				"prev_dischg_time_ms(%lu) dischg_time_ms(%lu) "
				"diff(%ld) count_same_dischg(%d)\n",
				status,
				prev_dischg_time_ms,
				dischg_time_ms,
				diff,
				count_same_dischg);

		prev_dischg_time_ms = dischg_time_ms;

		if (ABS(diff) < AC_1A_WA_DIFF_TIME_MS) {
			if(++count_same_dischg == AC_1A_WA_TRIGGER) {
				is_limit_IUSB = true;
				pr_info("Start to limit USB current\n");
			}
		} else {
			is_limit_IUSB = false;
			count_same_dischg = 0;
		}
	}

	prev_cable_in = cable_in;
#endif

	cable_detection_vbus_irq_handler();
	return 0;
}

static int chg_ov(struct smb358_charger *chip, u8 status)
{
#ifndef CONFIG_HTC_BATT_8960
	u8 psy_health_sts;
#endif

	pr_info("status = 0x%02x\n", status);

	schedule_delayed_work(&chip->update_ovp_uvp_work,
			msecs_to_jiffies(1000));

#ifndef CONFIG_HTC_BATT_8960
	if (status)
		psy_health_sts = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	else
		psy_health_sts = POWER_SUPPLY_HEALTH_GOOD;

	power_supply_set_health_state(
				chip->usb_psy, psy_health_sts);
	power_supply_changed(chip->usb_psy);
#endif

	return 0;
}

static int power_ok_handler(struct smb358_charger *chip, u8 status)
{
	pr_info("status = 0x%02x\n", status);

	return 0;
}

#define STATUS_FAST_CHARGING BIT(6)
static int fast_chg(struct smb358_charger *chip, u8 status)
{
	pr_info("status = 0x%02x\n", status);

	if (status & STATUS_FAST_CHARGING) {
		start_eoc_work(chip);
		chip->batt_full = false;
	}
	return 0;
}

static int chg_term(struct smb358_charger *chip, u8 status)
{
	int temp;
	bool is_hw_reload = false;

	temp = smb358_get_prop_batt_temp(chip);
	pr_info("status=0x%02x, is_batt_full=%d, temp=%dC\n",
			status, is_batt_full, temp);

	if (status & IRQ_C_TERM_BIT)
		is_hw_reload = smb358_is_hw_reload_happened(chip);

	if((temp > chip->bat_present_decidegc) && (status & IRQ_C_TERM_BIT)
			&& !is_hw_reload) {
		if(!is_idic_detect_done && !chip->is_embeded_batt)
			smb358_idic_detection(chip);
		is_batt_full_eoc_stop = is_batt_full = true;
		htc_gauge_event_notify(HTC_GAUGE_EVENT_EOC_STOP_CHG);
		
		if (is_hv_battery && !flag_keep_charge_on)
			smb358_chg_disable_pwrsrc(chip, true,
							PWRSRC_DISABLED_BIT_EOC);
	}

	if (!chip->iterm_disabled && !is_hw_reload)
		chip->batt_full = !!status;
	return 0;
}

#ifndef CONFIG_HTC_BATT_8960
static int taper_chg(struct smb358_charger *chip, u8 status)
{
	pr_info("status = 0x%02x\n", status);
	return 0;
}
#endif

static int chg_recharge(struct smb358_charger *chip, u8 status)
{
	pr_info("status = 0x%02x, is_batt_full = %d\n", status, is_batt_full);

	if(is_batt_full && (status & IRQ_C_RECHARGE_BIT)) {
		is_batt_full_eoc_stop = false;
	}
	
	chip->batt_full = !status;
	return 0;
}

static void smb358_chg_set_appropriate_battery_current(
				struct smb358_charger *chip)
{
	int rc;
	unsigned int current_max = chip->fastchg_current_max_ma;

	if (chip->batt_cool)
		current_max =
			min(current_max, chip->cool_bat_ma);
	if (chip->batt_warm)
		current_max =
			min(current_max, chip->warm_bat_ma);

	if (chg_limit_current != 0)
		current_max = min(current_max, chg_limit_current);

	pr_info("setting %dmA", current_max);
	rc = smb358_fastchg_current_set(chip, current_max);
	if (rc)
		dev_err(chip->dev,
			"Couldn't set charging current rc = %d\n", rc);
}

static void smb358_chg_set_appropriate_vddmax(
				struct smb358_charger *chip)
{
	int rc;
	unsigned int vddmax = chip->vfloat_mv;

	if (chip->batt_cool)
		vddmax = min(vddmax, chip->cool_bat_mv);
	if (chip->batt_warm)
		vddmax = min(vddmax, chip->warm_bat_mv);
	if (chip->batt_hot)
		vddmax = min(vddmax, chip->warm_bat_mv);

	pr_info("setting %dmV\n", vddmax);
	rc = smb358_float_voltage_set(chip, vddmax);
	if (rc)
		dev_err(chip->dev,
			"Couldn't set float voltage rc = %d\n", rc);
}

#define HYSTERESIS_DECIDEGC 20
static void smb_chg_adc_notification(enum qpnp_tm_state state, void *ctx)
{
	struct smb358_charger *chip = ctx;
	bool bat_hot = 0, bat_cold = 0, bat_present = 0, bat_warm = 0,
							bat_cool = 0;
	int temp;

	if (state >= ADC_TM_STATE_NUM) {
		pr_err("invallid state parameter %d\n", state);
		return;
	}

	temp = smb358_get_prop_batt_temp(chip);

	pr_info("temp = %d state = %s\n", temp,
				state == ADC_TM_WARM_STATE ? "hot" : "cold");

	if (state == ADC_TM_WARM_STATE) {
		if (temp >= chip->hot_bat_decidegc) {
			bat_hot = true;
			bat_warm = false;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;

			chip->adc_param.low_temp =
				chip->hot_bat_decidegc - HYSTERESIS_DECIDEGC;
			chip->adc_param.state_request =
				ADC_TM_COOL_THR_ENABLE;
		} else if (temp >=
			chip->warm_bat_decidegc && chip->jeita_supported) {
			bat_hot = false;
			bat_warm = true;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;

			chip->adc_param.low_temp =
				chip->warm_bat_decidegc - HYSTERESIS_DECIDEGC;
			chip->adc_param.high_temp =
				chip->hot_bat_decidegc;
		} else if (temp >=
			chip->cool_bat_decidegc && chip->jeita_supported) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;

			chip->adc_param.low_temp =
				chip->cool_bat_decidegc - HYSTERESIS_DECIDEGC;
			chip->adc_param.high_temp =
				chip->warm_bat_decidegc;
		} else if (temp >=
			chip->cold_bat_decidegc) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = false;
			bat_cool = true;
			bat_present = true;

			chip->adc_param.low_temp =
				chip->cold_bat_decidegc - HYSTERESIS_DECIDEGC;
			if (chip->jeita_supported)
				chip->adc_param.high_temp =
						chip->cool_bat_decidegc;
			else
				chip->adc_param.high_temp =
						chip->hot_bat_decidegc;
			chip->adc_param.state_request =
					ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (temp >= chip->bat_present_decidegc) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = true;
			bat_cool = false;
			bat_present = true;

			chip->adc_param.high_temp = chip->cold_bat_decidegc;
			chip->adc_param.low_temp = chip->bat_present_decidegc
							- HYSTERESIS_DECIDEGC;
			chip->adc_param.state_request =
					ADC_TM_HIGH_LOW_THR_ENABLE;
		}
	} else {
		if (temp <= chip->bat_present_decidegc) {
			bat_cold = true;
			bat_cool = false;
			bat_hot = false;
			bat_warm = false;
			bat_present = false;
			chip->adc_param.high_temp = chip->bat_present_decidegc
							+ HYSTERESIS_DECIDEGC;
			chip->adc_param.state_request =
				ADC_TM_WARM_THR_ENABLE;
		} else if (temp <= chip->cold_bat_decidegc) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = true;
			bat_cool = false;
			bat_present = true;
			chip->adc_param.high_temp =
				chip->cold_bat_decidegc + HYSTERESIS_DECIDEGC;
			
			chip->adc_param.low_temp =
				chip->bat_present_decidegc;
			chip->adc_param.state_request =
				ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (temp <= chip->cool_bat_decidegc &&
					chip->jeita_supported) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = false;
			bat_cool = true;
			bat_present = true;
			chip->adc_param.high_temp =
				chip->cool_bat_decidegc + HYSTERESIS_DECIDEGC;
			chip->adc_param.low_temp =
				chip->cold_bat_decidegc;
			chip->adc_param.state_request =
				ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (temp <= chip->warm_bat_decidegc &&
					chip->jeita_supported) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;
			chip->adc_param.high_temp =
				chip->warm_bat_decidegc + HYSTERESIS_DECIDEGC;
			chip->adc_param.low_temp =
				chip->cool_bat_decidegc;
			chip->adc_param.state_request =
				ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (temp <= chip->hot_bat_decidegc) {
			bat_hot = false;
			bat_warm = true;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;
			if (chip->jeita_supported)
				chip->adc_param.low_temp =
					chip->warm_bat_decidegc;
			else
				chip->adc_param.low_temp =
					chip->cold_bat_decidegc;
			chip->adc_param.high_temp =
				chip->hot_bat_decidegc + HYSTERESIS_DECIDEGC;
			chip->adc_param.state_request =
					ADC_TM_HIGH_LOW_THR_ENABLE;
		}
	}

	if (bat_present)
		chip->battery_missing = false;
	else if (!chip->is_embeded_batt)
		chip->battery_missing = true;

	if (bat_hot ^ chip->batt_hot || bat_cold ^ chip->batt_cold) {
		chip->batt_hot = bat_hot;
		chip->batt_cold = bat_cold;
		
		if (bat_hot || bat_cold || chip->battery_missing)
#ifndef CONFIG_HTC_BATT_8960
			smb358_charging_disable(chip, THERMAL, 1);
#else
			smb358_chg_disable_charger(chip, 1,
						BATT_CHG_DISABLED_BIT_TEMP);
#endif
		else
#ifndef CONFIG_HTC_BATT_8960
			smb358_charging_disable(chip, THERMAL, 0);
#else
			smb358_chg_disable_charger(chip, 0,
						BATT_CHG_DISABLED_BIT_TEMP);
#endif
		if (chip->battery_missing) {
			pr_info("Battery is removed, temp=%dC,bat_cold=%d,bat_present_degc=%dC\n",
					temp, bat_cold, chip->bat_present_decidegc);

			htc_gauge_event_notify(HTC_GAUGE_EVENT_BATT_REMOVED);
		}
	}

	if ((chip->batt_warm ^ bat_warm || chip->batt_cool ^ bat_cool)
						&& chip->jeita_supported) {
		chip->batt_warm = bat_warm;
		chip->batt_cool = bat_cool;
		smb358_chg_set_appropriate_battery_current(chip);
		smb358_chg_set_appropriate_vddmax(chip);
		htc_gauge_event_notify(HTC_GAUGE_EVENT_TEMP_ZONE_CHANGE);
	}

	pr_info("hot %d, cold %d, warm %d, cool %d, jeita supported %d, missing %d, low = %d deciDegC, high = %d deciDegC\n",
		chip->batt_hot, chip->batt_cold, chip->batt_warm,
		chip->batt_cool, chip->jeita_supported, chip->battery_missing,
		chip->adc_param.low_temp, chip->adc_param.high_temp);
	if (qpnp_adc_tm_channel_measure(chip->adc_tm_dev, &chip->adc_param))
		pr_err("request ADC error\n");
}

#ifndef CONFIG_HTC_BATT_8960
static int hot_hard_handler(struct smb358_charger *chip, u8 status)
{
	pr_info("%s: status = 0x%02x\n", __func__, status);
	chip->batt_hot = !!status;
	return 0;
}
static int cold_hard_handler(struct smb358_charger *chip, u8 status)
{
	pr_info("%s: status = 0x%02x\n", __func__, status);
	chip->batt_cold = !!status;
	return 0;
}
static int hot_soft_handler(struct smb358_charger *chip, u8 status)
{
	pr_info("%s: status = 0x%02x\n", __func__, status);
	chip->batt_warm = !!status;
	return 0;
}
static int cold_soft_handler(struct smb358_charger *chip, u8 status)
{
	pr_info("%s: status = 0x%02x\n", __func__, status);
	chip->batt_cool = !!status;
	return 0;
}

static int battery_missing(struct smb358_charger *chip, u8 status)
{
	pr_info("%s: status = 0x%02x\n", __func__, status);
	chip->battery_missing = !!status;
	return 0;
}
#endif 

static struct irq_handler_info handlers[] = {
	[0] = {
		.stat_reg	= IRQ_A_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "cold_soft",
			},
			{
				.name		= "hot_soft",
			},
			{
				.name		= "cold_hard",
			},
			{
				.name		= "hot_hard",
			},
		},
	},
	[1] = {
		.stat_reg	= IRQ_B_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "chg_hot",
			},
			{
				.name		= "vbat_low",
			},
			{
				.name		= "battery_missing",
			},
			{
				.name		= "battery_ov",
			},
		},
	},
	[2] = {
		.stat_reg	= IRQ_C_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "chg_term",
				.smb_irq	= chg_term,
			},
			{
				.name		= "taper",
			},
			{
				.name		= "recharge",
				.smb_irq	= chg_recharge,
			},
			{
				.name		= "fast_chg",
				.smb_irq	= fast_chg,
			},
		},
	},
	[3] = {
		.stat_reg	= IRQ_D_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "prechg_timeout",
			},
			{
				.name		= "safety_timeout",
				.smb_irq	= safety_timeout,
			},
			{
				.name		= "aicl_complete",
				.smb_irq	= aicl_complete,
			},
			{
				.name		= "src_detect",
				.smb_irq	= apsd_complete,
			},
		},
	},
	[4] = {
		.stat_reg	= IRQ_E_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "usbin_uv",
				.smb_irq        = chg_uv,
			},
			{
				.name		= "usbin_ov",
				.smb_irq	= chg_ov,
			},
			{
				.name		= "unknown",
			},
			{
				.name		= "unknown",
			},
		},
	},
	[5] = {
		.stat_reg	= IRQ_F_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "power_ok",
				.smb_irq	= power_ok_handler,
			},
			{
				.name		= "otg_det",
			},
			{
				.name		= "otg_batt_uv",
			},
			{
				.name		= "otg_oc",
			},
		},
	},
};

#define IRQ_LATCHED_MASK	0x02
#define IRQ_STATUS_MASK		0x01
#define BITS_PER_IRQ		2
static irqreturn_t smb358_chg_stat_handler(int irq, void *dev_id)
{
	struct smb358_charger *chip = dev_id;
	int i, j;
	u8 triggered;
	u8 changed;
	u8 rt_stat, prev_rt_stat;
	int rc;
	int handler_count = 0;

	mutex_lock(&chip->irq_complete);

	chip->irq_waiting = true;
	if (!chip->resume_completed) {
		pr_warn("IRQ triggered before device-resume\n");
		disable_irq_nosync(irq);
		mutex_unlock(&chip->irq_complete);
		return IRQ_HANDLED;
	}
	chip->irq_waiting = false;

	for (i = 0; i < ARRAY_SIZE(handlers); i++) {
		rc = smb358_read_reg(chip, handlers[i].stat_reg,
						&handlers[i].val);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't read %d rc = %d\n",
					handlers[i].stat_reg, rc);
			continue;
		}

		for (j = 0; j < ARRAY_SIZE(handlers[i].irq_info); j++) {
			triggered = handlers[i].val
			       & (IRQ_LATCHED_MASK << (j * BITS_PER_IRQ));
			rt_stat = handlers[i].val
				& (IRQ_STATUS_MASK << (j * BITS_PER_IRQ));
			prev_rt_stat = handlers[i].prev_val
				& (IRQ_STATUS_MASK << (j * BITS_PER_IRQ));
			changed = prev_rt_stat ^ rt_stat;

			if (triggered || changed)
				rt_stat ? handlers[i].irq_info[j].high++ :
						handlers[i].irq_info[j].low++;

			if ((triggered || changed)
				&& handlers[i].irq_info[j].smb_irq != NULL) {
				handler_count++;
				rc = handlers[i].irq_info[j].smb_irq(chip,
								rt_stat);
				
				if (!((i == IRQ_C_REG_SEQUENCE && (IRQ_STATUS_MASK << (j * BITS_PER_IRQ)) == IRQ_C_TAPER_CHG_BIT))) {
					pr_info("IRQ 0x%02X reg =0x%02X, %s irq is triggered=0x%02X, rt_stat=0x%02X, changed=0x%02X\n",
							handlers[i].stat_reg, handlers[i].val,
							handlers[i].irq_info[j].name, triggered, rt_stat, changed);
					if (rc < 0)
						dev_err(chip->dev,
							"Couldn't handle %d irq for reg 0x%02x rc = %d\n",
							j, handlers[i].stat_reg, rc);
				}
			}
		}
		handlers[i].prev_val = handlers[i].val;
	}

	pr_info("handler count = %d\n", handler_count);
#ifndef CONFIG_HTC_BATT_8960
	if (handler_count) {
		pr_debug("batt psy changed\n");
		power_supply_changed(&chip->batt_psy);
	}
#endif

	mutex_unlock(&chip->irq_complete);

	return IRQ_HANDLED;
}

static irqreturn_t smb358_chg_valid_handler(int irq, void *dev_id)
{
	struct smb358_charger *chip = dev_id;
	int present;

	present = gpio_get_value_cansleep(chip->chg_valid_gpio);
	if (present < 0) {
		dev_err(chip->dev, "Couldn't read chg_valid gpio=%d\n",
						chip->chg_valid_gpio);
		return IRQ_HANDLED;
	}
	present ^= chip->chg_valid_act_low;

	pr_info("%s: chg_present = %d\n", __func__, present);

	if (present != chip->chg_present) {
		chip->chg_present = present;
		pr_info("%s updating usb_psy present=%d",
				__func__, chip->chg_present);
		power_supply_set_present(chip->usb_psy, chip->chg_present);
	}

	return IRQ_HANDLED;
}

#if defined(CONFIG_HTC_BATT_GPIO_OVP)
static irqreturn_t batt_ovp_irq_handler(int irq, void *dev_id)
{
	int ovp_now = gpio_get_value(the_chip->batt_ovp_irq);
	pr_info("Status = %d -> %d\n", start_ovp, ovp_now);
	if (start_ovp == ovp_now)
		goto endOVP;
	if (!delayed_work_pending(&the_chip->check_external_ovp_work))
		schedule_delayed_work(&the_chip->check_external_ovp_work, msecs_to_jiffies(200));
endOVP:
	return IRQ_HANDLED;
}
#endif

#ifdef CONFIG_HTC_BATT_8960
int smb358_get_batt_temperature(int *result)
{
	if (!the_chip) {
		pr_warn("called before init\n");
		return -EINVAL;
	}

	*result = smb358_get_prop_batt_temp(the_chip);

	if (*result >= 650 &&
			(flag_keep_charge_on || flag_pa_fake_batt_temp))
		*result  = 650;

	return 0;
}

int smb358_get_charging_source(int *result)
{
	*result = pwr_src;
	return 0;
}

int smb358_get_charging_enabled(int *result)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (smb358_get_prop_batt_status(the_chip) == POWER_SUPPLY_STATUS_CHARGING)
		return smb358_get_charging_source(result);
	else
		*result = HTC_PWR_SOURCE_TYPE_BATT;
	return 0;
}

int smb358_charger_enable(bool enable)
{
	int rc = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		rc = -EINVAL;
	} else {
		rc = smb358_chg_disable_charger(the_chip, !enable,
									BATT_CHG_DISABLED_BIT_KDRV);
		if (rc)
			dev_err(the_chip->dev, "Couldn't control charger!!\n");
	}

	return rc;
}

int smb358_pwrsrc_enable(bool enable)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	return smb358_chg_disable_pwrsrc(the_chip, !enable,
								PWRSRC_DISABLED_BIT_KDRV);
}

int smb358_set_ftm_charge_enable_type(enum htc_ftm_power_source_type ftm_src)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (the_chip->ftm_src != ftm_src) {
		pr_info("(%d -> %d)\n", the_chip->ftm_src, ftm_src);
		the_chip->ftm_src = ftm_src;
	}

	return 0;
}

static u32 htc_fake_charger_for_ftm(enum htc_power_source_type src)
{
	unsigned int new_src = src;

	if((src <= HTC_PWR_SOURCE_TYPE_9VAC) && (src != HTC_PWR_SOURCE_TYPE_BATT)) {
		if (the_chip->ftm_src == HTC_FTM_PWR_SOURCE_TYPE_USB)
			new_src = HTC_PWR_SOURCE_TYPE_USB;
		else if (the_chip->ftm_src == HTC_FTM_PWR_SOURCE_TYPE_AC)
			new_src = HTC_PWR_SOURCE_TYPE_AC;

		if (src != new_src)
			pr_info("(%d -> %d)\n", src , new_src);
	}

	return new_src;
}

static int get_prop_usb_valid_status(struct smb358_charger *chip,
					int *ov, int *v, int *uv)
{
	int rc = 0;
	u8 reg = 0;

	rc = smb358_read_reg(chip, IRQ_E_REG, &reg);
	if (rc < 0) {
		pr_err("Couldn't read irq E rc = %d\n", rc);
		return rc;
	}
	*ov = (reg & IRQ_E_USBIN_OV_BIT) ? true : false;
	*uv = (reg & IRQ_E_USBIN_UV_BIT) ? true : false;

	if(!(*ov) && !(*uv))
		*v = true;
	else
		*v = false;

#if defined(CONFIG_HTC_BATT_GPIO_OVP)
	if (!gpio_get_value(the_chip->batt_ovp_irq)) {
		pr_info("v=%d, ov=%d, uv=%d OVP by external OVP irq: ", *v, *ov, *uv);
		*ov = true;
		*uv = false;
	}
#endif
	pr_info("v=%d, ov=%d, uv=%d\n", *v, *ov, *uv);
	return 0;
}

int smb358_is_pwr_src_plugged_in(void)
{
	int ov = false, uv = false, v = false;
	int rc = 0;

	
	if (!the_chip) {
		pr_warn("called before init\n");
		return -EINVAL;
	}

	rc = get_prop_usb_valid_status(the_chip, &ov, &v, &uv);
	if (rc < 0) {
		pr_err("Couldn't get vbus state rc = %d\n", rc);
		return rc;
	}

	if (uv) {
		
		mdelay(100);
		rc = get_prop_usb_valid_status(the_chip, &ov, &v, &uv);
		if (rc < 0) {
			pr_err("Couldn't get vbus state rc = %d\n", rc);
			return rc;
		}
	}

	return v;
}
EXPORT_SYMBOL(smb358_is_pwr_src_plugged_in);

static u32 htc_fake_charger_for_testing(enum htc_power_source_type src)
{
	
	enum htc_power_source_type new_src = HTC_PWR_SOURCE_TYPE_AC;

	if((src > HTC_PWR_SOURCE_TYPE_9VAC) || (src == HTC_PWR_SOURCE_TYPE_BATT))
		return src;

	pr_info("(%d -> %d)\n", src , new_src);
	return new_src;
}

static int smb358_enable_cmd_register_control(struct smb358_charger *chip)
{
	int rc;
	u8 reg = 0;
	u8 mask = 0;

	
	reg = CHG_PIN_CTRL_USBCS_REG_BIT | CHG_PIN_CTRL_CHG_EN_LOW_REG_BIT |
		CHG_PIN_CTRL_APSD_IRQ_BIT | CHG_PIN_CTRL_CHG_ERR_IRQ_BIT;
	mask = CHG_PIN_CTRL_CHG_EN_MASK | CHG_PIN_CTRL_USBCS_REG_MASK |
		CHG_PIN_CTRL_APSD_IRQ_MASK | CHG_PIN_CTRL_CHG_ERR_IRQ_MASK;
	rc = smb358_masked_write(chip, CHG_PIN_EN_CTRL_REG, mask, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set CHG_PIN_EN_CTRL_REG rc=%d\n",
				rc);
		return rc;
	}

	return 0;
}

static void start_eoc_work(struct smb358_charger *chip)
{
	wake_lock(&chip->eoc_worker_lock);
	if (delayed_work_pending(&chip->eoc_work))
		cancel_delayed_work_sync(&chip->eoc_work);
	schedule_delayed_work(&chip->eoc_work,
			msecs_to_jiffies(SMB358_EOC_CHECK_PERIOD_MS));
}

static void handle_usb_present_change(struct smb358_charger *chip,
				int chg_present)
{
	if (chip->chg_present ^ chg_present) {
		chip->chg_present = chg_present;
		is_ac_safety_timeout = false;

		if (!chg_present) {
			
			if (!strcmp(htc_get_bootmode(), "offmode_charging"))
				pm8909_bms_store_battery_data_emmc();

			
			eoc_count_by_curr = eoc_count = 0;
			is_batt_full = false;
			is_batt_full_eoc_stop = false;
			
			if (!flag_keep_charge_on)
				smb358_set_charger_after_eoc(true);
			memset(vbat_sample, 0, sizeof(vbat_sample));
			is_idic_detect_done = false;
			hsml_target_ma = 0;
			
			if (delayed_work_pending(&chip->retry_aicl_work))
				cancel_delayed_work_sync(&chip->retry_aicl_work);
			retry_aicl_cnt = 0;
			gs_is_bad_aicl_result = false;
			
			if (delayed_work_pending(&chip->eoc_work))
				cancel_delayed_work_sync(&chip->eoc_work);
			schedule_delayed_work(&chip->eoc_work, msecs_to_jiffies(0));
		} else {
			start_eoc_work(chip);
		}
	}

	
	if (chg_present && iusb_limit_enable)
		smb358_limit_input_current(
			iusb_limit_enable, iusb_limit_reason);
}

void smb358_set_charger_configuration(void)
{
	struct smb358_charger *chip = the_chip;
	int mA = chip->input_current_ma;

	smb358_enable_volatile_writes(chip);

	
	smb358_enable_cmd_register_control(chip);

	
	smb358_set_usb_chg_current(chip, mA);

	
	smb358_chg_set_appropriate_battery_current(chip);

	
	smb358_chg_set_appropriate_vddmax(chip);

	
	smb358_chg_disable_charger(chip, !chg_enable_status,
								BATT_CHG_DISABLED_BIT_KDRV);

	
	smb358_chg_disable_pwrsrc(chip, !pwrsrc_enable_status,
								PWRSRC_DISABLED_BIT_KDRV);

	
	smb358_chg_recharge_threshold_set(chip);

	
	smb358_chg_safety_timer_set(chip);

	return;
}

bool smb358_is_chg_current_reload(struct smb358_charger *chip)
{
	unsigned int current_max = chip->fastchg_current_max_ma;
	int i;
	u8 reg_value;

	smb358_read_reg(chip, CHG_CURRENT_CTRL_REG, &reg_value);

	if (chip->batt_cool)
		current_max =
			min(current_max, chip->cool_bat_ma);
	if (chip->batt_warm)
		current_max =
			min(current_max, chip->warm_bat_ma);

	if (chg_limit_current != 0)
		current_max = min(current_max, chg_limit_current);

	for (i = ARRAY_SIZE(fast_chg_current) - 1; i >= 0; i--) {
		if (fast_chg_current[i] <= current_max)
			break;
	}

	if ((reg_value >> SMB358_FAST_CHG_SHIFT)!= i)
		return true;
	else
		return false;

}

bool smb358_is_vfloat_reload(struct smb358_charger *chip)
{
	unsigned int vddmax = chip->vfloat_mv;
	u8 reg_value, vfloat_value;

	smb358_read_reg(chip, VFLOAT_REG, &reg_value);

	if (chip->batt_cool)
		vddmax = min(vddmax, chip->cool_bat_mv);
	if (chip->batt_warm)
		vddmax = min(vddmax, chip->warm_bat_mv);
	if (chip->batt_hot)
		vddmax = min(vddmax, chip->warm_bat_mv);

	if (VFLOAT_4350MV == vddmax)
		vfloat_value = 0x2B;
	else if (vddmax > VFLOAT_4350MV)
		vfloat_value = (vddmax - MIN_FLOAT_MV) / VFLOAT_STEP_MV - 1;
	else
		vfloat_value = (vddmax - MIN_FLOAT_MV) / VFLOAT_STEP_MV;

	if ((reg_value & VFLOAT_MASK) != vfloat_value)
		return true;
	else
		return false;

}

bool smb358_is_hw_reload_happened(struct smb358_charger *chip)
{
	int	vbat_mv;
	bool is_chg_current_reload = smb358_is_chg_current_reload(chip);
	bool is_vfloat_reload = smb358_is_vfloat_reload(chip);

	vbat_mv = smb358_get_prop_battery_voltage_now(chip)/1000;

	if (is_chg_current_reload || is_vfloat_reload) {
		smb358_set_charger_configuration();
		pr_info("Re-config charger again.");
		return true;
	} else {
		return false;
	}
}

int smb358_set_pwrsrc_and_charger_enable(enum htc_power_source_type src,
			bool chg_enable, bool pwrsrc_enable)
{
	static int pre_pwr_src;
	int mA = 0;
	struct smb358_charger *chip = the_chip;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	
	gs_is_bad_aicl_result = false;

	pr_info("src=%d, pre_pwr_src=%d, chg_enable=%d, pwrsrc_enable=%d, ftm_src=%d\n",
				src, pre_pwr_src, chg_enable, pwrsrc_enable, chip->ftm_src);

	if (flag_force_ac_chg)
		src = htc_fake_charger_for_testing(src);

	pwr_src = src;

	if (chip->ftm_src == HTC_FTM_PWR_SOURCE_TYPE_USB ||
			chip->ftm_src == HTC_FTM_PWR_SOURCE_TYPE_AC)
		src = htc_fake_charger_for_ftm(src);

	switch (src) {
	case HTC_PWR_SOURCE_TYPE_BATT:
		mA = USB_MA_0;
		break;
	case HTC_PWR_SOURCE_TYPE_WIRELESS:
	case HTC_PWR_SOURCE_TYPE_DETECTING:
	case HTC_PWR_SOURCE_TYPE_UNKNOWN_USB:
	case HTC_PWR_SOURCE_TYPE_CABLE_INSERT:
	case HTC_PWR_SOURCE_TYPE_USB:
		
		if (chip->ftm_src == HTC_FTM_PWR_SOURCE_TYPE_AC)
			mA = USB_MA_1000;
		else
			mA = USB_MA_500;
		break;
	case HTC_PWR_SOURCE_TYPE_AC:
	case HTC_PWR_SOURCE_TYPE_9VAC:
	case HTC_PWR_SOURCE_TYPE_MHL_AC:
		if (flag_force_ac_chg || is_limit_IUSB)
			mA = USB_MA_1000;
		else
			mA = USB_MA_1500;
		break;
	default:
		mA = USB_MA_0;
		break;
	}

	chip->input_current_ma = mA;

	chg_enable_status = chg_enable;
	pwrsrc_enable_status = pwrsrc_enable;
	smb358_set_charger_configuration();

	if (chip->ftm_src == HTC_FTM_PWR_SOURCE_TYPE_NONE_STOP)
		smb358_chg_disable_pwrsrc(chip, true, PWRSRC_DISABLED_BIT_FTM);
	else
		smb358_chg_disable_pwrsrc(chip, false, PWRSRC_DISABLED_BIT_FTM);

	if (HTC_PWR_SOURCE_TYPE_BATT == src)
		handle_usb_present_change(chip, 0);
	else
		handle_usb_present_change(chip, 1);

	return 0;
}

int smb358_fake_chg_uv_irq_handler(void)
{
	int rc = 0;
	u8 reg = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	rc = smb358_read_reg(the_chip, IRQ_E_REG, &reg);
	if (rc) {
		dev_err(the_chip->dev, "Couldn't read IRQ_E rc = %d\n", rc);
		return rc;
	}

	first_vbus_irq = false;

	if (reg & IRQ_E_INPUT_UV_BIT) {
		chg_uv(the_chip, 1);
	} else {
		chg_uv(the_chip, 0);
		apsd_complete(the_chip, 1);
	}
	pr_info("Trigger fake vbus irq during kernel init, reg:0x%X\n", reg);

	return 0;
}

static int64_t read_battery_id(struct smb358_charger *chip)
{
	int rc;
	struct qpnp_vadc_result result;

	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX2_BAT_ID, &result);
	if (rc) {
		pr_err("error reading batt id channel = %d, rc = %d\n",
					LR_MUX2_BAT_ID, rc);
		return rc;
	}

	return result.physical;
}

static int64_t read_vbus_voltage(struct smb358_charger *chip)
{
	int rc;
	struct qpnp_vadc_result result;

	rc = qpnp_vadc_read(chip->vadc_dev, USBIN, &result);
	if (rc) {
		pr_err("error reading USBIN channel = %d, rc = %d\n",
                                       USBIN, rc);
		return rc;
	}

	return result.physical;
}

static int smb358_get_usb_chg_current(struct smb358_charger *chip)
{
	int i, rc = 0;
	u8 reg = 0;

	rc = smb358_read_reg(chip, CHG_OTH_CURRENT_CTRL_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read input mA rc = %d\n", rc);
		return rc;
	}

	i = reg >> AC_CHG_CURRENT_SHIFT;

	pr_debug("reg=0x%X, i=%d, current_limit = %d\n", reg, i, chg_current[i]);
	return chg_current[i];
}

static int smb358_get_fastchg_current(struct smb358_charger *chip)
{
	int i, rc = 0;
	u8 reg = 0;

	rc = smb358_read_reg(chip, CHG_CURRENT_CTRL_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read fastchg mA rc = %d\n", rc);
		return rc;
	}

	i = reg >> SMB358_FAST_CHG_SHIFT;

	pr_debug("reg=0x%X, i=%d, fastchg_current = %d\n", reg, i, fast_chg_current[i]);
	return fast_chg_current[i];
}

#define AICL_COMPLETE_MASK BIT(4)
#define AICL_RESULT_MASK SMB358_MASK(3, 0)
static int smb358_get_aicl_result(struct smb358_charger *chip, int *aicl_result, bool *aicl_done)
{
	int i, rc = 0;
	u8 reg = 0, mask = 0;

	rc = smb358_read_reg(chip, STATUS_E_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read input mA rc = %d\n", rc);
		return rc;
	}

	if(reg & AICL_COMPLETE_MASK)
		*aicl_done = true;
	else
		*aicl_done = false;

	mask = AICL_RESULT_MASK;
	i = reg & mask;
	*aicl_result = chg_current[i];

	pr_debug("reg=0x%X, i=%d, aicl_result=%d, aicl_done=%d\n",
				reg, i, chg_current[i], *aicl_done);
	return 0;
}

#define FIRST_CNFG_REG	0x00
#define FIRST_CMD_REG	0x30
#define FIRST_STATUS_REG	0x35
static void dump_charger_regs(struct smb358_charger *chip)
{
	static u8 config_regs[20];
	static u8 cmd_regs[4];
	static u8 status_regs[11];

	memset(config_regs, 0, sizeof(config_regs));
	memset(cmd_regs, 0, sizeof(cmd_regs));
	memset(status_regs, 0, sizeof(status_regs));

	
	smb358_read_bytes(chip, FIRST_CNFG_REG, config_regs, sizeof(config_regs));
	
	smb358_read_bytes(chip, FIRST_CMD_REG, cmd_regs, sizeof(cmd_regs));
	
	smb358_read_bytes(chip, FIRST_STATUS_REG, status_regs, sizeof(status_regs));

	printk(KERN_INFO "[BATT][SMB] CONFIG_REG<00h~13h>:"
		"[00h]=[0x%02X,0x%02X,0x%02X,0x%02X,0x%02X],"
		"[05h]=[0x%02X,0x%02X,0x%02X,0x%02X,0x%02X],"
		"[0Ah]=[0x%02X,0x%02X,0x%02X,0x%02X,0x%02X],"
		"[0Fh]=[0x%02X,0x%02X,0x%02X,0x%02X,0x%02X],"
		"CMD_REG<30h~33h>:[0x%02X,0x%02X,0x%02X,0x%02X], "
		"STATUS_REG<35h~3Fh>:"
		"[35h]=[0x%02X,0x%02X,0x%02X,0x%02X,0x%02X],"
		"[3Ah]=[0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X]\n",
		config_regs[0x0], config_regs[0x1], config_regs[0x2], config_regs[0x3], config_regs[0x4],
		config_regs[0x5], config_regs[0x6], config_regs[0x7], config_regs[0x8], config_regs[0x9],
		config_regs[0xA], config_regs[0xB], config_regs[0xC], config_regs[0xD], config_regs[0xE],
		config_regs[0xF], config_regs[0x10], config_regs[0x11], config_regs[0x12], config_regs[0x13],
		cmd_regs[0], cmd_regs[1], cmd_regs[2], cmd_regs[3],
		status_regs[0], status_regs[1], status_regs[2], status_regs[3], status_regs[4],
		status_regs[5], status_regs[6], status_regs[7], status_regs[8], status_regs[9], status_regs[10]);
}

static void dump_irq_rt_status(struct smb358_charger *chip)
{
	printk(KERN_INFO "[BATT][SMB] "
		"[IRQ_A]:0x%02X,[IRQ_B]:0x%02X,[IRQ_C]:0x%02X,"
		"[IRQ_D]:0x%02X,[IRQ_E]:0x%02X,[IRQ_F]:0x%02X\n",
		handlers[0].val, handlers[1].val, handlers[2].val,
		handlers[3].val, handlers[4].val, handlers[5].val);
}
static void dump_all(int more)
{
	int vbat_mv, ibat_ma, tbat_deg, soc, usbin;
	int id_mv, iusb_max, ibat_max, aicl_result;
	bool aicl_done;
	struct smb358_charger *chip = the_chip;

	pm8909_get_batt_voltage(&vbat_mv);
	pm8909_get_batt_soc(&soc);
	pm8909_get_batt_current(&ibat_ma);
	tbat_deg = smb358_get_prop_batt_temp(chip);
	id_mv = (int)read_battery_id(chip)/1000;
	iusb_max = smb358_get_usb_chg_current(chip);
	ibat_max = smb358_get_fastchg_current(chip);
	smb358_get_aicl_result(chip, &aicl_result, &aicl_done);
	usbin = (int)read_vbus_voltage(chip);

	printk(KERN_INFO "[BATT][SMB] "
		"V=%dmV,I=%dmA,T=%dC,SoC=%d%%,id=%dmV,usbin=%d,"
		"iusb_max=%dmA,ibat_max=%dmA,aicl_done=%d,aicl_result=%dmA,"
		"batfet_dis=0x%x,pwrsrc_dis=0x%x,"
		"bat_hot/cold=%d/%d,bat_warm/cool=%d/%d,"
		"flag=%d%d%d%d%d,OVP=%d,UVP=%d,"
		"is_ac_ST=%d,vbat_sample=%d/%d/%d,is_HV=%d,"
		"hsml_ma=%d\n",
		vbat_mv, ibat_ma, tbat_deg, soc, id_mv, usbin,
		iusb_max, ibat_max, aicl_done, aicl_result,
		batt_charging_disabled, pwrsrc_disabled,
		chip->batt_hot, chip->batt_cold, chip->batt_warm, chip->batt_cool,
		flag_keep_charge_on, flag_force_ac_chg, flag_pa_fake_batt_temp,
		flag_disable_safety_timer, flag_disable_temp_protection, ovp, uvp,
		is_ac_safety_timeout, vbat_sample[0],vbat_sample[1],vbat_sample[2], is_hv_battery,
		hsml_target_ma);

	dump_charger_regs(chip);
	dump_irq_rt_status(chip);
	pm8909_bms_dump_all();
}

int smb358_dump_all(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	dump_all(0);

	return 0;
}

int smb358_is_batt_temp_fault_disable_chg(int *result)
{
	int vbat_mv, is_vbatt_over_vddmax;
	int is_cold = 0, is_hot = 0;
	int is_warm = 0;
	struct smb358_charger *chip = the_chip;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	vbat_mv = smb358_get_prop_battery_voltage_now(chip) / 1000;

	is_hot = chip->batt_hot;
	is_cold = chip->batt_cold;
	is_warm = chip->batt_warm;

	if(vbat_mv >= chip->warm_bat_mv)
		is_vbatt_over_vddmax = true;
	else
		is_vbatt_over_vddmax = false;

	pr_debug("is_cold=%d, is_hot=%d, is_warm=%d, is_vbatt_over_vddmax=%d, warm_bat_mv:%d\n",
			is_cold, is_hot, is_warm, is_vbatt_over_vddmax, chip->warm_bat_mv);
	if ((is_cold || is_hot || (is_warm && is_vbatt_over_vddmax)) &&
			!flag_keep_charge_on && !flag_disable_temp_protection)
		*result = 1;
	else
		*result = 0;

	return 0;
}

int smb358_is_batt_temperature_fault(int *result)
{
	int is_cold = 0, is_warm = 0, is_hot = 0;
	struct smb358_charger *chip = the_chip;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	is_cold = chip->batt_cold;
	is_warm = chip->batt_warm;
	is_hot = chip->batt_hot;

	pr_debug("is_cold=%d,is_warm=%d,is_hot=%d\n", is_cold, is_warm, is_hot);
	if (is_cold || is_warm || is_hot)
		*result = 1;
	else
		*result = 0;
	return 0;
}

static void update_ovp_uvp_state(int ov, int v, int uv)
{
	if ( ov && !v && !uv) {
		if (!ovp) {
			ovp = 1;
			pr_info("OVP: 0 -> 1, USB_Valid: %d\n", v);
			htc_charger_event_notify(HTC_CHARGER_EVENT_OVP);
			
			cable_detection_vbus_irq_handler();
		}
		if (uvp) {
			uvp = 0;
			pr_debug("UVP: 1 -> 0, USB_Valid: %d\n", v);
		}
	} else if ( !ov && !v && uv) {
		if (ovp) {
			ovp = 0;
			pr_info("OVP: 1 -> 0, USB_Valid: %d\n", v);
			htc_charger_event_notify(HTC_CHARGER_EVENT_OVP_RESOLVE);
		}
		if (!uvp) {
			uvp = 1;
			pr_debug("UVP: 0 -> 1, USB_Valid: %d\n", v);
		}
	} else {
		
		if (ovp) {
			ovp = 0;
			pr_info("OVP: 1 -> 0, USB_Valid: %d\n", v);
			htc_charger_event_notify(HTC_CHARGER_EVENT_OVP_RESOLVE);
			
			cable_detection_vbus_irq_handler();
		}
		if (uvp) {
			uvp = 0;
			pr_debug("UVP: 1 -> 0, USB_Valid: %d\n", v);
		}
	}

	pr_debug("ovp=%d, uvp=%d [%d,%d,%d]\n", ovp, uvp, ov, v, uv);
}

#if defined(CONFIG_HTC_BATT_GPIO_OVP)
static void check_external_ovp_worker(struct work_struct *work)
{
	if (gpio_get_value(the_chip->batt_ovp_irq) != start_ovp) {
		if (delayed_work_pending(&the_chip->update_ovp_uvp_work))
			cancel_delayed_work(&the_chip->update_ovp_uvp_work);
		schedule_delayed_work(&the_chip->update_ovp_uvp_work, msecs_to_jiffies(800));
	}
	start_ovp = gpio_get_value(the_chip->batt_ovp_irq);
}
#endif
static void update_ovp_uvp_worker(struct work_struct *work)
{
	int ov = false, uv = false, v = false;

	if (!the_chip) {
		pr_err("called before init\n");
		return;
	}

	get_prop_usb_valid_status(the_chip, &ov, &v, &uv);
	update_ovp_uvp_state(ov, v, uv);
}

int smb358_is_charger_ovp(int* result)
{
	int ov = false, uv = false, v = false;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	get_prop_usb_valid_status(the_chip, &ov, &v, &uv);

	update_ovp_uvp_state(ov, v, uv);
	*result = ovp;
	return 0;
}

static int smb358_is_fastchg_on(struct smb358_charger *chip)
{
	int rc;
	u8 reg = 0;

	rc = smb358_read_reg(chip, STATUS_C_REG, &reg);
	if (rc) {
		pr_err("Couldn't read STATUS_C_REG_FAST rc=%d\n", rc);
		return 0;
	}

	return (reg & STATUS_C_FAST_CHARGING) ? 1 : 0;
}

static int smb358_chg_is_taper(struct smb358_charger *chip)
{
	int rc;
	u8 reg = 0;

	rc = smb358_read_reg(chip, STATUS_C_REG, &reg);
	if (rc) {
		pr_err("Couldn't read STATUS_C_REG_TAPER rc=%d\n", rc);
		return 0;
	}

	reg &= STATUS_C_CHARGING_MASK;

	if (reg == STATUS_C_TAPER_CHARGING)
		return true;
	else
		return false;
}

#define VBAT_TOLERANCE_MV	70
#define CONSECUTIVE_COUNT	3
#define CLEAR_FULL_STATE_BY_LEVEL_THR	95
#define LOW_CURRENT_TOLERANCE	10
static void
smb358_eoc_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct smb358_charger *chip = container_of(dwork,
				struct smb358_charger, eoc_work);
	int ibat_ma, vbat_mv;
	u8 chg_state = 0;
	int soc;
	bool is_hw_reload;

	wake_lock(&chip->eoc_worker_lock);
	if (!smb358_is_pwr_src_plugged_in()){
		pr_info("no chg connected, stopping\n");
		is_batt_full = false;
		is_batt_full_eoc_stop = false;
		goto stop_eoc;
	}

	smb358_read_reg(chip, IRQ_C_REG, &chg_state);
	pm8909_get_batt_current(&ibat_ma);
	ibat_ma /= 1000;
	vbat_mv = smb358_get_prop_battery_voltage_now(chip)/1000;
	is_hw_reload = smb358_is_hw_reload_happened(chip);

	pr_info("ibat_ma=%d, vbat_mv=%d, vfloat_mv:%d, batt_full_cri=%d, "
			"eoc_ma=%d, eoc_count=%d, eoc_count_by_curr=%d, chg_state=0x%X, "
			"fastchg=%d, chgtaper=%d, hw_reload=%d\n",
			ibat_ma, vbat_mv, chip->vfloat_mv, chip->batt_full_criteria,
			chip->iterm_ma, eoc_count, eoc_count_by_curr, chg_state,
			smb358_is_fastchg_on(chip), smb358_chg_is_taper(chip),
			is_hw_reload);

	if (smb358_is_fastchg_on(chip)) {
		is_batt_full_eoc_stop = false;
		if (!smb358_chg_is_taper(chip)) {
			
			eoc_count_by_curr = eoc_count = 0;
		} else if (ibat_ma > (0 + LOW_CURRENT_TOLERANCE)) {
			
			eoc_count_by_curr = eoc_count = 0;
		} else if (vbat_mv < (chip->vfloat_mv - VBAT_TOLERANCE_MV)) {
			
			eoc_count_by_curr = eoc_count = 0;
		} else if ((ibat_ma * -1) > chip->batt_full_criteria) {
			
			eoc_count_by_curr = eoc_count = 0;
		} else if ((ibat_ma * -1) > chip->iterm_ma) {
			
			eoc_count_by_curr = 0;
			eoc_count++;

			if (eoc_count == CONSECUTIVE_COUNT) {
				is_batt_full = true;
				htc_gauge_event_notify(HTC_GAUGE_EVENT_EOC);
			}
		} else {
			eoc_count++;
			
			if (eoc_count_by_curr == CONSECUTIVE_COUNT) {
				pr_info("End of Charging\n");
				is_batt_full_eoc_stop = true;
				
				if (is_hv_battery && !flag_keep_charge_on)
					smb358_chg_disable_pwrsrc(chip, true,
									PWRSRC_DISABLED_BIT_EOC);
				goto stop_eoc;
			} else {
				if (eoc_count == CONSECUTIVE_COUNT && !is_batt_full) {
					is_batt_full = true;
					htc_gauge_event_notify(HTC_GAUGE_EVENT_EOC);
				}
				eoc_count_by_curr += 1;
			}
		}

		if (is_batt_full) {
			pm8909_get_batt_soc(&soc);
			if (soc <= CLEAR_FULL_STATE_BY_LEVEL_THR) {
				
				if (chip->vfloat_mv &&
					(vbat_mv > (chip->vfloat_mv - 100))) {
					pr_info("Not satisfy overloading battery voltage"
						" critiria (%dmV < %dmV).\n", vbat_mv,
						(chip->vfloat_mv - 100));
				} else {
					is_batt_full = false;
					eoc_count = eoc_count_by_curr = 0;
					pr_info("Clear is_batt_full & eoc_count due to"
						" Overloading happened, soc=%d%%\n", soc);
					htc_gauge_event_notify(HTC_GAUGE_EVENT_EOC);
				}
			}
		}
	} else {
		if((!is_batt_full_eoc_stop || !is_batt_full)
				&& (chg_state & IRQ_C_TERM_BIT)){
			is_batt_full_eoc_stop = is_batt_full = true;
			htc_gauge_event_notify(HTC_GAUGE_EVENT_EOC_STOP_CHG);
			
			if (is_hv_battery && !flag_keep_charge_on)
				smb358_chg_disable_pwrsrc(chip, true,
								PWRSRC_DISABLED_BIT_EOC);
			pr_info("smb358 chg_term irq not fired but charger is EoC\n");
		}
		pr_info("not charging\n");
			goto stop_eoc;
	}

	schedule_delayed_work(&chip->eoc_work,
		msecs_to_jiffies(SMB358_EOC_CHECK_PERIOD_MS));
	return;

stop_eoc:
	eoc_count_by_curr = eoc_count = 0;
	wake_unlock(&chip->eoc_worker_lock);

}

int smb358_is_batt_full(int *result)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	*result = is_batt_full;
	return 0;
}

int smb358_is_batt_full_eoc_stop(int *result)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	*result = is_batt_full_eoc_stop;
	return 0;
}

int smb358_chg_recharge_threshold_set(struct smb358_charger *chip)
{
	u8 reg = 0;
	int rc;

	if (flag_keep_charge_on || !is_hv_battery)
		chip->recharge_mv = 100;

	if (chip->recharge_mv != -EINVAL) {
		if (chip->recharge_mv <= 50)
			reg = VFLT_50MV;
		else if (chip->recharge_mv <= 100)
			reg = VFLT_100MV;
		else if (chip->recharge_mv <= 200)
			reg = VFLT_200MV;
		else
			reg = VFLT_300MV;

		pr_info("setting:%dmV, reg:0x%X\n", chip->recharge_mv, reg);
		rc = smb358_masked_write(chip, CHG_OTH_CURRENT_CTRL_REG,
						VFLT_MASK, reg);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't set inhibit threshold rc = %d\n", rc);
			return rc;
		}
	}

	return 0;
}

int smb358_chg_safety_timer_set(struct smb358_charger *chip)
{
	u8 reg = 0;
	int rc;

	
	if (!is_ac_online() || chip->safety_time == 0 ||
			flag_keep_charge_on || flag_disable_safety_timer) {
		
		rc = smb358_masked_write(chip, CFG_SFY_TIMER_CTRL_REG,
				SAFETY_TIME_MINUTES_MASK, SAFETY_TIME_DISABLE_BIT);
		if (rc)
			dev_err(chip->dev, "Couldn't write CFG_SFY_TIMER_CTRL_REG, rc = %d\n",
								rc);
	} else if (chip->safety_time != -EINVAL) {
		if (chip->safety_time <= 382)
			reg = TIMER_382MINS;
		else if (chip->safety_time <= 764)
			reg = TIMER_764MINS;
		else if (chip->safety_time <= 1527)
			reg = TIMER_1527MINS;
		else
			reg = TIMER_1527MINS;

		pr_info("Safety_timer: %dmins, reg:0x%X\n", chip->safety_time, reg);
		rc = smb358_masked_write(chip, CFG_SFY_TIMER_CTRL_REG,
				SAFETY_TIME_MINUTES_MASK, reg);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set safety timer rc = %d\n",
								rc);
			return rc;
		}
	}

	return 0;
}

int smb358_get_charge_type(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return smb358_get_prop_charge_type(the_chip);
}

static int is_ac_online(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	return smb358_is_pwr_src_plugged_in() &&
				(pwr_src == HTC_PWR_SOURCE_TYPE_AC ||
							pwr_src == HTC_PWR_SOURCE_TYPE_9VAC ||
							pwr_src == HTC_PWR_SOURCE_TYPE_MHL_AC);
}

int smb358_is_chg_safety_timer_timeout(int *result)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	*result = is_ac_safety_timeout;
	return 0;
}

static int smb358_load_battery_data(struct smb358_charger *chip)
{
	int64_t battery_id;
	int rc = 0;
	struct bms_battery_data *batt_data;
	struct device_node *node;
	int id_result;

	
	battery_id = (int)read_battery_id(chip) / 1000;
	if (battery_id < 0) {
		pr_err("cannot read battery id err = %lld\n", battery_id);
		return battery_id;
	}

	
	id_result = htc_battery_cell_find_and_set_id_auto(battery_id);
	pr_info("batt ID vol= %lldmv, id_result= %d\n", battery_id, id_result);

	node = of_find_node_by_name(chip->dev->of_node,
					"qcom,battery-data");
	if (!node) {
			pr_err("No available batterydata\n");
			return -EINVAL;
	}

	batt_data = devm_kzalloc(chip->dev,
			sizeof(struct bms_battery_data), GFP_KERNEL);
	if (!batt_data) {
		pr_err("Could not alloc battery data\n");
		return -EINVAL;
	}

	batt_data->max_voltage_uv = -1;
	batt_data->fastchg_current_max_ma = -1;
	batt_data->cool_bat_ma = -1;
	batt_data->warm_bat_ma = -1;

	rc = of_batterydata_read_data_by_id_result(node, batt_data, id_result);
	if (rc) {
		pr_err("Could not load battery data by id_result\n");
		goto fail_load_battdata;
	}

	
	if ((!test_power_monitor && !test_ftm_mode && !test_download_mode) &&
			batt_data->max_voltage_uv >= 0)
		chip->vfloat_mv = batt_data->max_voltage_uv / 1000;
	if (batt_data->fastchg_current_max_ma >= 0)
		chip->fastchg_current_max_ma = batt_data->fastchg_current_max_ma;
	if (batt_data->warm_bat_ma >= 0)
		chip->warm_bat_ma = batt_data->warm_bat_ma;
	if (batt_data->cool_bat_ma >= 0)
		chip->cool_bat_ma = batt_data->cool_bat_ma;

	pr_info("vfloat-mv=%d,fastchg-current=%d,warm-bat-ma=%d,cool-bat-ma=%d\n",
			batt_data->max_voltage_uv, batt_data->fastchg_current_max_ma,
			batt_data->warm_bat_ma, batt_data->cool_bat_ma);

	return 0;

fail_load_battdata:
	devm_kfree(chip->dev, batt_data);
	return rc;
}

int smb358_limit_charge_enable(bool enable)
{
	pr_info("limit_charge=%d\n", enable);
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (enable)
		chg_limit_current = SMB358_CHG_I_MIN_MA;
	else
		chg_limit_current = 0;

	smb358_chg_set_appropriate_battery_current(the_chip);
	return 0;
}

int smb358_pinctrl_control(struct smb358_charger *chip)
{
	int ret = 0;

	
	chip->pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		pr_err("Target does not use pinctrl\n");
		ret = PTR_ERR(chip->pinctrl);
		chip->pinctrl = NULL;
		return ret;
	}

	chip->gpio_state_init = pinctrl_lookup_state(chip->pinctrl, "batt_ovp_1_init");
	if (IS_ERR_OR_NULL(chip->gpio_state_init)) {
		pr_err("Cannot get pintctrl state\n");
		ret = PTR_ERR(chip->gpio_state_init);
		chip->pinctrl = NULL;
		return ret;
	}

	ret = pinctrl_select_state(chip->pinctrl, chip->gpio_state_init);
	if (ret) {
		pr_err("Cannot init INT gpio\n");
		return ret;
	}

	return 0;
}

bool smb358_is_battery_present(void)
{
	if (!the_chip) {
		pr_warn("called before init\n");
		return true;
	}

	return !the_chip->battery_missing;
}

int smb358_get_battery_status(void)
{
	if (!the_chip) {
		pr_warn("called before init\n");
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	return smb358_get_prop_batt_status(the_chip);
}

int smb358_chg_aicl_enable(struct smb358_charger *chip, bool enable)
{
	int rc;
	u8 reg;

	pr_debug("charger AICL enable: %d\n", enable);
	if (enable)
		reg = AICL_ENABLED_BIT;
	else
		reg = 0x00;
	
	rc = smb358_masked_write(chip, AICL_CONTROL_REG, AICL_ENABLED_BIT, reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set AICL_ENABLED_BIT rc=%d\n",
				rc);
		return rc;
	}
	return 0;
}

int smb358_limit_input_current(bool enable, int reason)
{
	int rc = 0;
	int limit_intput_current = 0;
	struct smb358_charger *chip = the_chip;
	iusb_limit_enable = enable;
	iusb_limit_reason = reason;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	pr_debug("enable=%d, input_current=%dmA\n", enable, chip->input_current_ma);

	if(chip->input_current_ma && is_ac_online()) {
		limit_intput_current = chip->input_current_ma;

		
		rc = smb358_chg_aicl_enable(chip, false);
		if (rc < 0) {
			pr_err("enable/disable AICL failed rc=%d\n", rc);
			return rc;
		}

		if (enable) {
			
			if (iusb_limit_reason & HTC_BATT_CHG_LIMIT_BIT_THRML)
				limit_intput_current =
					min(limit_intput_current, SMB358_INPUT_LIMIT_MA);
		}

		pr_info("Set input current limit to %dmA due to reason=0x%X, "
				"input_current=%dmA\n",
				limit_intput_current, iusb_limit_reason,
				chip->input_current_ma);

		
		rc = smb358_set_usb_chg_current(chip, limit_intput_current);
		if (rc < 0) {
			pr_err("set usb chg current fail rc=%d\n", rc);
			return rc;
		}
		
		rc = smb358_chg_aicl_enable(chip, true);
		if (rc < 0) {
			pr_err("enable/disable AICL failed rc=%d\n", rc);
			return rc;
		}
	}
	return 0;
}

int smb358_charger_get_attr_text(char *buf, int size)
{
	int ibat_max, iusb_max, len = 0, i = 0;
	int ov = false, uv = false, v = false;
	int aicl_result, usbin;
	bool aicl_done;
	u8 status_a_reg = 0;

	struct smb358_charger *chip = the_chip;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	smb358_get_aicl_result(chip, &aicl_result, &aicl_done);
	iusb_max = smb358_get_usb_chg_current(chip);
	ibat_max = smb358_get_fastchg_current(chip);
	smb358_read_reg(chip, STATUS_A_REG, &status_a_reg);
	usbin = (int)read_vbus_voltage(chip);

	get_prop_usb_valid_status(chip, &ov, &v, &uv);

	len += scnprintf(buf + len, size - len,
			"OVP: %d;\n", ov);

	len += scnprintf(buf + len, size - len,
			"V: %d;\n", v);

	len += scnprintf(buf + len, size - len,
			"UV: %d;\n", uv);

	len += scnprintf(buf + len, size - len,
			"USBIN: %d;\n", usbin);

#if defined(CONFIG_HTC_BATT_GPIO_OVP)
	len += scnprintf(buf + len, size - len,
			"GPIO49: %d;\n", gpio_get_value(the_chip->batt_ovp_irq));
#endif

	len += scnprintf(buf + len, size - len,
			"aicl_done: %d;\n", aicl_done);

	len += scnprintf(buf + len, size - len,
			"aicl_result: %d;\n", aicl_result);

	len += scnprintf(buf + len, size - len,
			"iusb_max: %d;\n", iusb_max);

	len += scnprintf(buf + len, size - len,
			"ibat_max: %d;\n", ibat_max);

	len += scnprintf(buf + len, size - len,
			"status_a_reg: 0x%X;\n", status_a_reg);

	len += scnprintf(buf + len, size - len,
			"is_idic_detect_done: %d;\n", is_idic_detect_done);

	for (i = 0; i < IDIC_VOLTAGE_SAMPLE; i++) {
		len += scnprintf(buf + len, size - len,
			"vbat_sample[%d]: %d;\n", i, vbat_sample[i]);
	}

	len += scnprintf(buf + len, size - len,
			"is_hv_battery: %d;\n", is_hv_battery);

	len += scnprintf(buf + len, size - len,
			"vfloat(mV): %d;\n", chip->vfloat_mv);

	return len;
}

int smb358_is_bad_cable_used(int *result)
{
	int batt_temp = 0,aicl_result = 0;
	bool is_temp_fault = false,aicl_done = 0;
	*result = 0;
	if(!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	is_temp_fault = (the_chip->batt_hot || the_chip->batt_cold);

	if(pwr_src != 2 || is_temp_fault){
		*result = 0;
		pr_info("chg_src:%d, is_bad_aicl_result:%d, temp_fault:%d,charging_disabled:%d,result:%d\n",
			pwr_src,gs_is_bad_aicl_result,is_temp_fault,the_chip->charging_disabled,*result);
		return 0;
	}

	batt_temp = smb358_get_prop_batt_temp(the_chip);
	smb358_get_aicl_result(the_chip, &aicl_result, &aicl_done);

	if(	!is_batt_full &&
		pwr_src == 2 &&
		gs_is_bad_aicl_result &&
		!the_chip->charging_disabled ){

	   *result = 1;
	}
	pr_info("chg_src:%d,is_batt_full:%d,is_bad_aicl_result:%d,charging_disabled:%d,Temp:%d,AICL:%d,result:%d\n",
			pwr_src,is_batt_full,gs_is_bad_aicl_result,the_chip->charging_disabled,batt_temp,aicl_result,*result);

	return 0;
}

int smb358_set_charger_after_eoc(bool enable)
{
	int rc = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	
	smb358_chg_disable_charger(the_chip, !enable,
								BATT_CHG_DISABLED_BIT_EOC);

	
	smb358_chg_disable_pwrsrc(the_chip, false,
								PWRSRC_DISABLED_BIT_EOC);

	return rc;
}

int smb358_is_hv_battery_detection(int *result)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	if (the_chip->is_embeded_batt)
		*result = true;
	else
		*result = is_hv_battery;

	return 0;
}

#define SW_IDIC_VBAT_THRESHOLD 4300
#define NON_HV_BATT_VFLOAT_MV	4200
static void smb358_idic_detection(struct smb358_charger *chip)
{
	int i = 0;

	if (!smb358_is_pwr_src_plugged_in()
			|| chip->batt_warm || chip->batt_cool
			|| chip->batt_hot || chip->batt_cold){
		pr_info("stopping\n");
		goto stop_idic_detect;
	}

	
	for (i = 0; i < IDIC_VOLTAGE_SAMPLE; i ++) {
		is_idic_detect_done = true;
		vbat_sample[i] = smb358_get_prop_battery_voltage_now(chip)/1000;
		if (vbat_sample[i] < SW_IDIC_VBAT_THRESHOLD)
			is_hv_battery = false;
		else {
			is_hv_battery = true;
			goto stop_idic_detect;
		}
		mdelay(500);
	}

	if (!is_hv_battery) {
		chip->vfloat_mv = NON_HV_BATT_VFLOAT_MV;
		smb358_chg_set_appropriate_vddmax(chip);
		smb358_chg_recharge_threshold_set(chip);
		smb358_chg_disable_pwrsrc(chip, true,
								PWRSRC_DISABLED_BIT_IDIC);
		mdelay(20);
		smb358_chg_disable_pwrsrc(chip, false,
								PWRSRC_DISABLED_BIT_IDIC);
	}

stop_idic_detect:
	pr_info("3 times vbatt(mv)=%d,%d,%d,vfloat_mv=%d,recharge_mv=%d\n",
				vbat_sample[0],vbat_sample[1],vbat_sample[2],
				chip->vfloat_mv,chip->recharge_mv);
}

static void smb358_set_hsml_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct smb358_charger *chip = container_of(dwork,
				struct smb358_charger, set_hsml_work);

	downgrade_aicl_result(chip, hsml_target_ma);
}

int pm8909_set_hsml_target_ma(int target_ma)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	pr_info("target_ma= %d\n", target_ma);
	hsml_target_ma = target_ma;
	if((hsml_target_ma != 0) && (pwr_src == HTC_PWR_SOURCE_TYPE_USB)) {
		schedule_delayed_work(&the_chip->set_hsml_work,
				msecs_to_jiffies(1000));
	}

	return 0;
}
#endif 

#ifndef CONFIG_HTC_BATT_8960
static void smb358_external_power_changed(struct power_supply *psy)
{
	struct smb358_charger *chip = container_of(psy,
				struct smb358_charger, batt_psy);
	union power_supply_propval prop = {0,};
	int rc, current_limit = 0;

	if (chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);

	rc = chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (rc)
		dev_err(chip->dev,
			"Couldn't read USB current_max property, rc=%d\n", rc);
	else
		current_limit = prop.intval / 1000;


	smb358_enable_volatile_writes(chip);
	smb358_set_usb_chg_current(chip, current_limit);

	pr_info("current_limit = %d\n", current_limit);
}
#endif 

#if defined(CONFIG_DEBUG_FS)
#define LAST_CNFG_REG	0x13
static int show_cnfg_regs(struct seq_file *m, void *data)
{
	struct smb358_charger *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = 0; addr <= LAST_CNFG_REG; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int cnfg_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb358_charger *chip = inode->i_private;

	return single_open(file, show_cnfg_regs, chip);
}

static const struct file_operations cnfg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= cnfg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define FIRST_CMD_REG	0x30
#define LAST_CMD_REG	0x33
static int show_cmd_regs(struct seq_file *m, void *data)
{
	struct smb358_charger *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = FIRST_CMD_REG; addr <= LAST_CMD_REG; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int cmd_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb358_charger *chip = inode->i_private;

	return single_open(file, show_cmd_regs, chip);
}

static const struct file_operations cmd_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= cmd_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define FIRST_STATUS_REG	0x35
#define LAST_STATUS_REG		0x3F
static int show_status_regs(struct seq_file *m, void *data)
{
	struct smb358_charger *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = FIRST_STATUS_REG; addr <= LAST_STATUS_REG; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int status_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb358_charger *chip = inode->i_private;

	return single_open(file, show_status_regs, chip);
}

static const struct file_operations status_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= status_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int show_irq_count(struct seq_file *m, void *data)
{
	int i, j, total = 0;

	for (i = 0; i < ARRAY_SIZE(handlers); i++)
		for (j = 0; j < 4; j++) {
			seq_printf(m, "%s=%d\t(high=%d low=%d)\n",
						handlers[i].irq_info[j].name,
						handlers[i].irq_info[j].high
						+ handlers[i].irq_info[j].low,
						handlers[i].irq_info[j].high,
						handlers[i].irq_info[j].low);
			total += (handlers[i].irq_info[j].high
					+ handlers[i].irq_info[j].low);
		}

	seq_printf(m, "\n\tTotal = %d\n", total);

	return 0;
}

static int irq_count_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb358_charger *chip = inode->i_private;

	return single_open(file, show_irq_count, chip);
}

static const struct file_operations irq_count_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= irq_count_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int get_reg(void *data, u64 *val)
{
	struct smb358_charger *chip = data;
	int rc;
	u8 temp;

	rc = smb358_read_reg(chip, chip->peek_poke_address, &temp);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't read reg %x rc = %d\n",
			chip->peek_poke_address, rc);
		return -EAGAIN;
	}
	*val = temp;
	return 0;
}

static int set_reg(void *data, u64 val)
{
	struct smb358_charger *chip = data;
	int rc;
	u8 temp;

	temp = (u8) val;
	rc = smb358_write_reg(chip, chip->peek_poke_address, temp);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't write 0x%02x to 0x%02x rc= %d\n",
			chip->peek_poke_address, temp, rc);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(poke_poke_debug_ops, get_reg, set_reg, "0x%02llx\n");

static int force_irq_set(void *data, u64 val)
{
	struct smb358_charger *chip = data;

	smb358_chg_stat_handler(chip->client->irq, data);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(force_irq_ops, NULL, force_irq_set, "0x%02llx\n");
#endif

#ifdef DEBUG
static void dump_regs(struct smb358_charger *chip)
{
	int rc;
	u8 reg;
	u8 addr;

	for (addr = 0; addr <= LAST_CNFG_REG; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (rc)
			dev_err(chip->dev, "Couldn't read 0x%02x rc = %d\n",
					addr, rc);
		else
			pr_debug("0x%02x = 0x%02x\n", addr, reg);
	}

	for (addr = FIRST_STATUS_REG; addr <= LAST_STATUS_REG; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (rc)
			dev_err(chip->dev, "Couldn't read 0x%02x rc = %d\n",
					addr, rc);
		else
			pr_debug("0x%02x = 0x%02x\n", addr, reg);
	}

	for (addr = FIRST_CMD_REG; addr <= LAST_CMD_REG; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (rc)
			dev_err(chip->dev, "Couldn't read 0x%02x rc = %d\n",
					addr, rc);
		else
			pr_debug("0x%02x = 0x%02x\n", addr, reg);
	}
}
#else
static void dump_regs(struct smb358_charger *chip)
{
}
#endif

static int smb_parse_dt(struct smb358_charger *chip)
{
	int rc;
	enum of_gpio_flags gpio_flags;
	struct device_node *node = chip->dev->of_node;
	int batt_present_degree_negative;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}

	chip->charging_disabled = of_property_read_bool(node,
					"qcom,charger-disabled");

	chip->inhibit_disabled = of_property_read_bool(node,
					"qcom,chg-inhibit-disabled");
	chip->chg_autonomous_mode = of_property_read_bool(node,
					"qcom,chg-autonomous-mode");

	chip->disable_apsd = of_property_read_bool(node, "qcom,disable-apsd");

	chip->using_pmic_therm = of_property_read_bool(node,
						"qcom,using-pmic-therm");
	chip->bms_controlled_charging = of_property_read_bool(node,
						"qcom,bms-controlled-charging");

	rc = of_property_read_string(node, "qcom,bms-psy-name",
						&chip->bms_psy_name);
	if (rc)
		chip->bms_psy_name = NULL;

	chip->chg_valid_gpio = of_get_named_gpio_flags(node,
				"qcom,chg-valid-gpio", 0, &gpio_flags);
	if (!gpio_is_valid(chip->chg_valid_gpio))
		pr_warn("Invalid chg-valid-gpio");
	else
		chip->chg_valid_act_low = gpio_flags & OF_GPIO_ACTIVE_LOW;

	rc = of_property_read_u32(node, "qcom,fastchg-current-max-ma",
						&chip->fastchg_current_max_ma);
	if (rc)
		chip->fastchg_current_max_ma = SMB358_FAST_CHG_MAX_MA;

	chip->iterm_disabled = of_property_read_bool(node,
					"qcom,iterm-disabled");

	rc = of_property_read_u32(node, "qcom,iterm-ma", &chip->iterm_ma);
	if (rc < 0)
		chip->iterm_ma = -EINVAL;

	rc = of_property_read_u32(node, "htc,batt-full-criteria",
					&chip->batt_full_criteria);
	if (rc < 0)
		chip->batt_full_criteria = -EINVAL;

	rc = of_property_read_u32(node, "qcom,float-voltage-mv",
						&chip->vfloat_mv);
	if (rc < 0) {
		chip->vfloat_mv = -EINVAL;
		pr_err("float-voltage-mv property missing, exit\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(node, "qcom,recharge-mv",
						&chip->recharge_mv);
	if (rc < 0)
		chip->recharge_mv = -EINVAL;

	chip->recharge_disabled = of_property_read_bool(node,
					"qcom,recharge-disabled");

	rc = of_property_read_u32(node, "qcom,cold-bat-decidegc",
						&chip->cold_bat_decidegc);
	if (rc < 0)
		chip->cold_bat_decidegc = -EINVAL;

	rc = of_property_read_u32(node, "qcom,hot-bat-decidegc",
						&chip->hot_bat_decidegc);
	if (rc < 0)
		chip->hot_bat_decidegc = -EINVAL;

	rc = of_property_read_u32(node, "qcom,warm-bat-decidegc",
						&chip->warm_bat_decidegc);

	rc |= of_property_read_u32(node, "qcom,cool-bat-decidegc",
						&chip->cool_bat_decidegc);

	if (!rc) {
		rc = of_property_read_u32(node, "qcom,cool-bat-mv",
						&chip->cool_bat_mv);

		rc |= of_property_read_u32(node, "qcom,warm-bat-mv",
						&chip->warm_bat_mv);

		rc |= of_property_read_u32(node, "qcom,cool-bat-ma",
						&chip->cool_bat_ma);

		rc |= of_property_read_u32(node, "qcom,warm-bat-ma",
						&chip->warm_bat_ma);
		if (rc)
			chip->jeita_supported = false;
		else
			chip->jeita_supported = true;
	}

	pr_info("jeita_supported = %d\n", chip->jeita_supported);

	rc = of_property_read_u32(node, "qcom,bat-present-decidegc",
						&batt_present_degree_negative);
	if (rc < 0)
		chip->bat_present_decidegc = -EINVAL;
	else
		chip->bat_present_decidegc = -batt_present_degree_negative;

	if (of_get_property(node, "qcom,vcc-i2c-supply", NULL)) {
		chip->vcc_i2c = devm_regulator_get(chip->dev, "vcc-i2c");
		if (IS_ERR(chip->vcc_i2c)) {
			dev_err(chip->dev,
				"%s: Failed to get vcc_i2c regulator\n",
								__func__);
			return PTR_ERR(chip->vcc_i2c);
		}
	}

	chip->is_embeded_batt = of_property_read_bool(node,
					"htc,is-embeded-batt");

	rc = of_property_read_u32(node, "qcom,charging-timeout",
						&chip->safety_time);
	if (rc < 0)
		chip->safety_time = -EINVAL;

	pr_info("inhibit-disabled = %d, recharge-disabled = %d, recharge-mv = %d\n",
		chip->inhibit_disabled, chip->recharge_disabled,
						chip->recharge_mv);
	pr_info("vfloat-mv = %d, iterm-disabled = %d\n",
			chip->vfloat_mv, chip->iterm_disabled);
	pr_info("fastchg-current = %d, charging-disabled = %d\n",
			chip->fastchg_current_max_ma,
					chip->charging_disabled);
	pr_info("disable-apsd = %d bms = %s cold-bat-degree = %d\n",
		chip->disable_apsd, chip->bms_psy_name,
					chip->cold_bat_decidegc);
	pr_info("hot-bat-degree = %d, bat-present-decidegc = %d\n",
		chip->hot_bat_decidegc, chip->bat_present_decidegc);
	pr_info("is-embeded-batt = %d, charging-timeout = %d\n",
		chip->is_embeded_batt, chip->safety_time);
	return 0;
}

static int determine_initial_state(struct smb358_charger *chip)
{
	int rc;
	u8 reg = 0;

	rc = smb358_read_reg(chip, IRQ_B_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read IRQ_B rc = %d\n", rc);
		goto fail_init_status;
	}

	rc = smb358_read_reg(chip, IRQ_C_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read IRQ_C rc = %d\n", rc);
		goto fail_init_status;
	}
	chip->batt_full = (reg & IRQ_C_TERM_BIT) ? true : false;

	rc = smb358_read_reg(chip, IRQ_A_REG, &reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read irq A rc = %d\n", rc);
		return rc;
	}

#ifndef CONFIG_HTC_BATT_8960
	
	if (reg & IRQ_A_HOT_HARD_BIT)
		chip->batt_hot = true;
	if (reg & IRQ_A_COLD_HARD_BIT)
		chip->batt_cold = true;
	if (reg & IRQ_A_HOT_SOFT_BIT)
		chip->batt_warm = true;
	if (reg & IRQ_A_COLD_SOFT_BIT)
		chip->batt_cool = true;

	rc = smb358_read_reg(chip, IRQ_E_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read IRQ_E rc = %d\n", rc);
		goto fail_init_status;
	}

	if (reg & IRQ_E_INPUT_UV_BIT) {
		chg_uv(chip, 1);
	} else {
		chg_uv(chip, 0);
		apsd_complete(chip, 1);
	}
#endif

	pr_info("hot:%d, cold:%d, warm:%d, cool:%d, batt_full:%d\n",
		chip->batt_hot, chip->batt_cold, chip->batt_warm,
		chip->batt_cool, chip->batt_full);

	return 0;

fail_init_status:
	dev_err(chip->dev, "Couldn't determine initial status\n");
	return rc;
}

#if defined(CONFIG_DEBUG_FS)
static void smb358_debugfs_init(struct smb358_charger *chip)
{
	int rc;
	chip->debug_root = debugfs_create_dir("smb358", NULL);
	if (!chip->debug_root)
		dev_err(chip->dev, "Couldn't create debug dir\n");

	if (chip->debug_root) {
		struct dentry *ent;

		ent = debugfs_create_file("config_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &cnfg_debugfs_ops);
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(chip->dev,
				"Couldn't create cnfg debug file rc = %d\n",
				rc);
		}

		ent = debugfs_create_file("status_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &status_debugfs_ops);
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(chip->dev,
				"Couldn't create status debug file rc = %d\n",
				rc);
		}

		ent = debugfs_create_file("cmd_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &cmd_debugfs_ops);
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(chip->dev,
				"Couldn't create cmd debug file rc = %d\n",
				rc);
		}

		ent = debugfs_create_x32("address", S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root,
					  &(chip->peek_poke_address));
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(chip->dev,
				"Couldn't create address debug file rc = %d\n",
				rc);
		}

		ent = debugfs_create_file("data", S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root, chip,
					  &poke_poke_debug_ops);
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(chip->dev,
				"Couldn't create data debug file rc = %d\n",
				rc);
		}

		ent = debugfs_create_file("force_irq",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root, chip,
					  &force_irq_ops);
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(chip->dev,
				"Couldn't create force_irq debug file rc =%d\n",
				rc);
		}

		ent = debugfs_create_file("irq_count", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &irq_count_debugfs_ops);
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(chip->dev,
				"Couldn't create cnfg irq_count file rc = %d\n",
				rc);
		}
	}
}
#else
static void smb358_debugfs_init(struct smb358_charger *chip)
{
}
#endif

#define SMB_I2C_VTG_MIN_UV 1800000
#define SMB_I2C_VTG_MAX_UV 1800000
static int smb358_charger_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int rc, irq;
	struct smb358_charger *chip;
#ifndef CONFIG_HTC_BATT_8960
	struct power_supply *usb_psy;
#endif
	u8 reg = 0;

	flag_keep_charge_on =
		(get_kernel_flag() & KERNEL_FLAG_KEEP_CHARG_ON) ? 1 : 0;
	flag_force_ac_chg =
		(get_kernel_flag() & KERNEL_FLAG_ENABLE_FAST_CHARGE) ? 1 : 0;
	flag_pa_fake_batt_temp =
		(get_kernel_flag() & KERNEL_FLAG_FOR_PA_TEST) ? 1 : 0;
	flag_disable_safety_timer =
		(get_kernel_flag() & KERNEL_FLAG_DISABLE_SAFETY_TIMER) ? 1 : 0;
	flag_disable_temp_protection =
		(get_kernel_flag() & KERNEL_FLAG_DISABLE_TBATT_PROTECT) ? 1 : 0;
	flag_enable_bms_charger_log =
		(get_kernel_flag() & KERNEL_FLAG_ENABLE_BMS_CHARGER_LOG) ? 1 : 0;
	test_power_monitor =
		(get_kernel_flag() & KERNEL_FLAG_TEST_PWR_SUPPLY) ? 1 : 0;
	test_ftm_mode =
		(!strcmp(htc_get_bootmode(),"ftm")) ? 1 : 0;
	test_download_mode =
		(!strcmp(htc_get_bootmode(),"download")) ? 1 : 0;

#ifndef CONFIG_HTC_BATT_8960
	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_warn("USB psy not found; deferring probe\n");
		return -EPROBE_DEFER;
	}
#endif

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "Couldn't allocate memory\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->dev = &client->dev;
#ifndef CONFIG_HTC_BATT_8960
	chip->usb_psy = usb_psy;
#endif
	chip->fake_battery_soc = -EINVAL;

	
	chip->vadc_dev = qpnp_get_vadc(chip->dev, "chg");
	if (IS_ERR(chip->vadc_dev)) {
		rc = PTR_ERR(chip->vadc_dev);
		if (rc != -EPROBE_DEFER)
			pr_err("vadc property missing\n");
		return rc;
	}

	rc = smb_parse_dt(chip);
	if (rc) {
		dev_err(&client->dev, "Couldn't parse DT nodes rc=%d\n", rc);
		return rc;
	}
	
	if (chip->vcc_i2c) {
		if (regulator_count_voltages(chip->vcc_i2c) > 0) {
			rc = regulator_set_voltage(chip->vcc_i2c,
				SMB_I2C_VTG_MIN_UV, SMB_I2C_VTG_MAX_UV);
			if (rc) {
				dev_err(&client->dev,
				"regulator vcc_i2c set failed, rc = %d\n",
								rc);
				return rc;
			}
		}

		rc = regulator_enable(chip->vcc_i2c);
		if (rc) {
			dev_err(&client->dev,
				"Regulator vcc_i2c enable failed rc = %d\n",
									rc);
			goto err_set_vtg_i2c;
		}
	}

	mutex_init(&chip->irq_complete);
	mutex_init(&chip->read_write_lock);
	mutex_init(&chip->path_suspend_lock);
	mutex_init(&chip->charging_disable_lock);

	
	rc = smb358_read_reg(chip, CHG_OTH_CURRENT_CTRL_REG, &reg);
	if (rc) {
		pr_err("Failed to detect SMB358, device absent, rc = %d\n", rc);
		goto err_set_vtg_i2c;
	}

	
	if (chip->using_pmic_therm) {
		chip->adc_tm_dev = qpnp_get_adc_tm(chip->dev, "chg");
		if (IS_ERR(chip->adc_tm_dev)) {
			rc = PTR_ERR(chip->adc_tm_dev);
			if (rc != -EPROBE_DEFER)
				pr_err("adc_tm property missing\n");
			return rc;
		}
	}

	i2c_set_clientdata(client, chip);

#ifndef CONFIG_HTC_BATT_8960
	chip->batt_psy.name		= "battery";
	chip->batt_psy.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->batt_psy.get_property	= smb358_battery_get_property;
	chip->batt_psy.set_property	= smb358_battery_set_property;
	chip->batt_psy.property_is_writeable =
					smb358_batt_property_is_writeable;
	chip->batt_psy.properties	= smb358_battery_properties;
	chip->batt_psy.num_properties	= ARRAY_SIZE(smb358_battery_properties);
	chip->batt_psy.external_power_changed = smb358_external_power_changed;
	chip->batt_psy.supplied_to = pm_batt_supplied_to;
	chip->batt_psy.num_supplicants = ARRAY_SIZE(pm_batt_supplied_to);
#endif 
	chip->resume_completed = true;

#ifndef CONFIG_HTC_BATT_8960
	rc = power_supply_register(chip->dev, &chip->batt_psy);
	if (rc < 0) {
		dev_err(&client->dev, "Couldn't register batt psy rc = %d\n",
				rc);
		goto err_set_vtg_i2c;
	}
#endif 

	dump_regs(chip);

	
	rc = smb358_load_battery_data(chip);
	if (rc) {
		pr_err("Unable to read battery data %d\n", rc);
		goto fail_smb358_hw_init;
	}

	rc = smb358_regulator_init(chip);
	if  (rc) {
		dev_err(&client->dev,
			"Couldn't initialize smb358 ragulator rc=%d\n", rc);
		goto fail_regulator_register;
	}

	rc = smb358_hw_init(chip);
	if (rc) {
		dev_err(&client->dev,
			"Couldn't intialize hardware rc=%d\n", rc);
		goto fail_smb358_hw_init;
	}

	the_chip = chip;

	rc = determine_initial_state(chip);
	if (rc) {
		dev_err(&client->dev,
			"Couldn't determine initial state rc=%d\n", rc);
		goto fail_smb358_hw_init;
	}
#if defined(CONFIG_HTC_BATT_GPIO_OVP)
	INIT_DELAYED_WORK(&chip->check_external_ovp_work, check_external_ovp_worker);
#endif
	INIT_DELAYED_WORK(&chip->update_ovp_uvp_work, update_ovp_uvp_worker);
	INIT_DELAYED_WORK(&chip->eoc_work, smb358_eoc_work);
	INIT_DELAYED_WORK(&chip->set_hsml_work, smb358_set_hsml_work);
	INIT_DELAYED_WORK(&chip->retry_aicl_work, retry_aicl_worker);

	wake_lock_init(&chip->eoc_worker_lock, WAKE_LOCK_SUSPEND,
			"smb358_eoc_worker_lock");

	
	if (gpio_is_valid(chip->chg_valid_gpio)) {
		rc = gpio_request(chip->chg_valid_gpio, "smb358_chg_valid");
		if (rc) {
			dev_err(&client->dev,
				"gpio_request for %d failed rc=%d\n",
				chip->chg_valid_gpio, rc);
			goto fail_chg_valid_irq;
		}
		irq = gpio_to_irq(chip->chg_valid_gpio);
		if (irq < 0) {
			dev_err(&client->dev,
				"Invalid chg_valid irq = %d\n", irq);
			goto fail_chg_valid_irq;
		}
		rc = devm_request_threaded_irq(&client->dev, irq,
				NULL, smb358_chg_valid_handler,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"smb358_chg_valid_irq", chip);
		if (rc) {
			dev_err(&client->dev,
				"Failed request_irq irq=%d, gpio=%d rc=%d\n",
				irq, chip->chg_valid_gpio, rc);
			goto fail_chg_valid_irq;
		}
		smb358_chg_valid_handler(irq, chip);
		enable_irq_wake(irq);
	}

	chip->irq_gpio = of_get_named_gpio_flags(chip->dev->of_node,
				"qcom,irq-gpio", 0, NULL);

	
	if (gpio_is_valid(chip->irq_gpio)) {
		rc = gpio_request(chip->irq_gpio, "smb358_irq");
		if (rc) {
			dev_err(&client->dev,
					"irq gpio request failed, rc=%d", rc);
			goto fail_smb358_hw_init;
		}
		rc = gpio_direction_input(chip->irq_gpio);
		if (rc) {
			dev_err(&client->dev,
					"set_direction for irq gpio failed\n");
			goto fail_irq_gpio;
		}

		irq = gpio_to_irq(chip->irq_gpio);
		if (irq < 0) {
			dev_err(&client->dev,
				"Invalid irq_gpio irq = %d\n", irq);
			goto fail_irq_gpio;
		}
		rc = devm_request_threaded_irq(&client->dev, irq, NULL,
				smb358_chg_stat_handler,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"smb358_chg_stat_irq", chip);
		if (rc) {
			dev_err(&client->dev,
				"Failed STAT irq=%d request rc = %d\n",
				irq, rc);
			goto fail_irq_gpio;
		}
		enable_irq_wake(irq);
	} else {
		goto fail_irq_gpio;
	}

#ifdef CONFIG_HTC_BATT_8960
	rc = smb358_pinctrl_control(chip);
	if (rc) {
		pr_err("smb358 pinctrl error, rc=%d\n", rc);
		goto fail_irq_gpio;
	}
#endif
#if defined(CONFIG_HTC_BATT_GPIO_OVP)
	chip->batt_ovp_irq = 49 + 911;
        
        if (gpio_is_valid(chip->batt_ovp_irq)) {
                rc = gpio_request(chip->batt_ovp_irq, "Batt_ovp_irq");
                if (rc) {
                        dev_err(&client->dev, "Batt_OVP_irq request failed, rc=%d", rc);
                        goto fail_batt_irq_gpio;
                }
                rc = gpio_direction_input(chip->batt_ovp_irq);
                if (rc) {
                        dev_err(&client->dev,"set_direction for Batt_OVP_irq failed\n");
                        goto fail_batt_irq_gpio;
                }
                irq = gpio_to_irq(chip->batt_ovp_irq);
                if (irq < 0) {
                        dev_err(&client->dev, "Invalid Batt_OVP_irq irq = %d\n", irq);
                        goto fail_batt_irq_gpio;
                }
                rc = devm_request_threaded_irq(&client->dev, irq,
                                batt_ovp_irq_handler, NULL,
                                IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                                "batt_ovp_irq", chip);
                if (rc) {
                        dev_err(&client->dev, "Failed BATT OVP irq=%d request rc = %d\n", irq, rc);
                        goto fail_batt_irq_gpio;
                }
                enable_irq_wake(irq);
        } else {
                goto fail_batt_irq_gpio;
        }
	start_ovp = gpio_get_value(the_chip->batt_ovp_irq);
	pr_info("external OVP irq init value:%d\n", start_ovp);
#endif
	
	if (!flag_keep_charge_on && !flag_disable_temp_protection &&
			chip->using_pmic_therm) {
		if (!chip->jeita_supported) {
			
			chip->adc_param.low_temp = chip->cold_bat_decidegc;
			chip->adc_param.high_temp = chip->hot_bat_decidegc;
		} else {
			chip->adc_param.low_temp = chip->cool_bat_decidegc;
			chip->adc_param.high_temp = chip->warm_bat_decidegc;
		}
		chip->adc_param.timer_interval = ADC_MEAS2_INTERVAL_1S;
		chip->adc_param.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
		chip->adc_param.btm_ctx = chip;
		chip->adc_param.threshold_notification =
				smb_chg_adc_notification;
		chip->adc_param.channel = LR_MUX1_BATT_THERM;

		
		rc = qpnp_adc_tm_channel_measure(chip->adc_tm_dev,
							&chip->adc_param);
		if (rc)
			pr_err("requesting ADC error %d\n", rc);

		pr_info("PMIC thermal monitor register done!\n");
	}

	htc_charger_event_notify(HTC_CHARGER_EVENT_READY);

	smb358_debugfs_init(chip);

	dump_regs(chip);

	pr_info("SMB358 successfully probed. charger=%d, batt=%d\n",
			chip->chg_present, smb358_get_prop_batt_present(chip));
	return 0;

fail_chg_valid_irq:
	if (gpio_is_valid(chip->chg_valid_gpio))
		gpio_free(chip->chg_valid_gpio);
fail_irq_gpio:
	if (gpio_is_valid(chip->irq_gpio))
		gpio_free(chip->irq_gpio);
#if defined(CONFIG_HTC_BATT_GPIO_OVP)
fail_batt_irq_gpio:
	if (gpio_is_valid(chip->batt_ovp_irq))
		gpio_free(chip->batt_ovp_irq);
#endif
fail_smb358_hw_init:
	regulator_unregister(chip->otg_vreg.rdev);
fail_regulator_register:
#ifndef CONFIG_HTC_BATT_8960
	power_supply_unregister(&chip->batt_psy);
#endif
err_set_vtg_i2c:
	if (chip->vcc_i2c)
		if (regulator_count_voltages(chip->vcc_i2c) > 0)
			regulator_set_voltage(chip->vcc_i2c, 0,
						SMB_I2C_VTG_MAX_UV);
	return rc;
}

static int smb358_charger_remove(struct i2c_client *client)
{
	struct smb358_charger *chip = i2c_get_clientdata(client);

#ifndef CONFIG_HTC_BATT_8960
	power_supply_unregister(&chip->batt_psy);
#endif
	if (gpio_is_valid(chip->chg_valid_gpio))
		gpio_free(chip->chg_valid_gpio);

	if (chip->vcc_i2c)
		regulator_disable(chip->vcc_i2c);

	mutex_destroy(&chip->irq_complete);
	debugfs_remove_recursive(chip->debug_root);
	return 0;
}

static int smb358_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smb358_charger *chip = i2c_get_clientdata(client);
	int rc;
	int i;

	for (i = 0; i < 2; i++) {
		rc = smb358_read_reg(chip, FAULT_INT_REG + i,
					&chip->irq_cfg_mask[i]);
		if (rc)
			dev_err(chip->dev,
				"Couldn't save irq cfg regs rc = %d\n", rc);
	}

	
	rc = smb358_write_reg(chip, FAULT_INT_REG,
			FAULT_INT_HOT_COLD_HARD_BIT | FAULT_INT_INPUT_UV_BIT);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set fault_irq_cfg rc = %d\n", rc);

	rc = smb358_write_reg(chip, STATUS_INT_REG,
			STATUS_INT_LOW_BATT_BIT | STATUS_INT_MISSING_BATT_BIT |
			STATUS_INT_CHGING_BIT | STATUS_INT_INOK_BIT |
			STATUS_INT_OTG_DETECT_BIT | STATUS_INT_CHG_INHI_BIT);
	if (rc < 0)
		dev_err(chip->dev,
			"Couldn't set status_irq_cfg rc = %d\n", rc);

	mutex_lock(&chip->irq_complete);
	if (chip->vcc_i2c) {
		rc = regulator_disable(chip->vcc_i2c);
		if (rc) {
			dev_err(chip->dev,
				"Regulator vcc_i2c disable failed rc=%d\n", rc);
			mutex_unlock(&chip->irq_complete);
			return rc;
		}
	}

	chip->resume_completed = false;
	mutex_unlock(&chip->irq_complete);
	return 0;
}

static int smb358_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smb358_charger *chip = i2c_get_clientdata(client);

	if (chip->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int smb358_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smb358_charger *chip = i2c_get_clientdata(client);
	int rc;
	int i;

	if (chip->vcc_i2c) {
		rc = regulator_enable(chip->vcc_i2c);
		if (rc) {
			dev_err(chip->dev,
				"Regulator vcc_i2c enable failed rc=%d\n", rc);
			return rc;
		}
	}

	
	for (i = 0; i < 2; i++) {
		rc = smb358_write_reg(chip, FAULT_INT_REG + i,
					chip->irq_cfg_mask[i]);
		if (rc)
			dev_err(chip->dev,
				"Couldn't restore irq cfg regs rc=%d\n", rc);
	}

	mutex_lock(&chip->irq_complete);
	chip->resume_completed = true;
	mutex_unlock(&chip->irq_complete);
	if (chip->irq_waiting) {
		smb358_chg_stat_handler(client->irq, chip);
		enable_irq(client->irq);
	}
	return 0;
}

static const struct dev_pm_ops smb358_pm_ops = {
	.suspend	= smb358_suspend,
	.suspend_noirq	= smb358_suspend_noirq,
	.resume		= smb358_resume,
};

static struct of_device_id smb358_match_table[] = {
	{ .compatible = "qcom,smb358-charger",},
	{ },
};

static const struct i2c_device_id smb358_charger_id[] = {
	{"smb358-charger", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, smb358_charger_id);

static struct i2c_driver smb358_charger_driver = {
	.driver		= {
		.name		= "smb358-charger",
		.owner		= THIS_MODULE,
		.of_match_table = smb358_match_table,
		.pm		= &smb358_pm_ops,
	},
	.probe		= smb358_charger_probe,
	.remove		= smb358_charger_remove,
	.id_table	= smb358_charger_id,
};

static int __init smb358_charger_init(void)
{
	return i2c_add_driver(&smb358_charger_driver);
}
late_initcall(smb358_charger_init);



MODULE_DESCRIPTION("SMB358 Charger");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:smb358-charger");
