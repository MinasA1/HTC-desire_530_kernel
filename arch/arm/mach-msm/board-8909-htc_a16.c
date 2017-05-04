/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <asm/mach/arch.h>
#include <soc/qcom/socinfo.h>
#include <linux/usb/board.h>
#include <mach/msm_memtypes.h>
#include <soc/qcom/rpm-smd.h>
#include <soc/qcom/smd.h>
#include <soc/qcom/smem.h>
#include <soc/qcom/spm.h>
#include <soc/qcom/pm.h>
#include "board-dt.h"
#include "platsmp.h"
#include <mach/devices_cmdline.h>
#ifdef CONFIG_HTC_BATT_8960
#include "mach/htc_battery_8960.h"
#include "mach/htc_battery_cell.h"
#include <linux/smb358-charger.h>
#include <linux/qpnp/qpnp-linear-charger.h>
#include <linux/qpnp/qpnp-vm-bms.h>
#endif
#include <linux/usb/cable_detect.h>
#include <linux/usb/android.h>

#ifdef CONFIG_HTC_BUILD_EDIAG
#include <linux/android_ediagpmem.h>
#endif

#define HTC_8909_USB1_HS_ID_GPIO 11 + 911
#define HTC_8909_ADC_SWITCH_GPIO 65 + 911


#ifdef CONFIG_HTC_BUILD_EDIAG
#define MSM_HTC_PMEM_EDIAG_BASE 0x8E339000
#define MSM_HTC_PMEM_EDIAG_SIZE SZ_64K
#define MSM_HTC_PMEM_EDIAG1_BASE MSM_HTC_PMEM_EDIAG_BASE
#define MSM_HTC_PMEM_EDIAG1_SIZE MSM_HTC_PMEM_EDIAG_SIZE
#define MSM_HTC_PMEM_EDIAG2_BASE MSM_HTC_PMEM_EDIAG_BASE
#define MSM_HTC_PMEM_EDIAG2_SIZE MSM_HTC_PMEM_EDIAG_SIZE
#define MSM_HTC_PMEM_EDIAG3_BASE MSM_HTC_PMEM_EDIAG_BASE
#define MSM_HTC_PMEM_EDIAG3_SIZE MSM_HTC_PMEM_EDIAG_SIZE

static struct android_pmem_platform_data android_pmem_ediag_pdata = {
	.name = "pmem_ediag",
	.start = MSM_HTC_PMEM_EDIAG_BASE,
	.size = MSM_HTC_PMEM_EDIAG_SIZE,
	.no_allocator = 0,
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_ediag1_pdata = {
	.name = "pmem_ediag1",
	.start = MSM_HTC_PMEM_EDIAG1_BASE,
	.size = MSM_HTC_PMEM_EDIAG1_SIZE,
	.no_allocator = 0,
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_ediag2_pdata = {
	.name = "pmem_ediag2",
	.start = MSM_HTC_PMEM_EDIAG2_BASE,
	.size = MSM_HTC_PMEM_EDIAG2_SIZE,
	.no_allocator = 0,
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_ediag3_pdata = {
	.name = "pmem_ediag3",
	.start = MSM_HTC_PMEM_EDIAG3_BASE,
	.size = MSM_HTC_PMEM_EDIAG3_SIZE,
	.no_allocator = 0,
	.cached = 0,
};

static struct platform_device android_pmem_ediag_device = {
	.name = "ediag_pmem",	.id = 1,
	.dev = { .platform_data = &android_pmem_ediag_pdata },
};

static struct platform_device android_pmem_ediag1_device = {
	.name = "ediag_pmem",	.id = 2,
	.dev = { .platform_data = &android_pmem_ediag1_pdata },
};

static struct platform_device android_pmem_ediag2_device = {
	.name = "ediag_pmem",	.id = 3,
	.dev = { .platform_data = &android_pmem_ediag2_pdata },
};

static struct platform_device android_pmem_ediag3_device = {
	.name = "ediag_pmem",	.id = 4,
	.dev = { .platform_data = &android_pmem_ediag3_pdata },
};
#endif

extern int smb358_is_pwr_src_plugged_in(void);
static int htc_get_usbid(void)
{
	int usbid_gpio;
	usbid_gpio = HTC_8909_USB1_HS_ID_GPIO;
	pr_debug("%s: usbid_gpio=%d\n", __func__, usbid_gpio);
	return usbid_gpio;
}

static int htc_get_adc_switch(void)
{
	int adc_switch_gpio;
	adc_switch_gpio = HTC_8909_ADC_SWITCH_GPIO;
	pr_info("%s: adc_switch_gpio=%d\n", __func__, adc_switch_gpio);
	return adc_switch_gpio;
}

static struct cable_detect_platform_data cable_detect_pdata = {
       .detect_type            = CABLE_TYPE_PMIC_ADC,
       .usb_id_pin_type        = CABLE_TYPE_APP_GPIO,
       .usb_id_pin_gpio        = htc_get_usbid,
       .adc_switch_gpio        = htc_get_adc_switch,
       .is_pwr_src_plugged_in  = smb358_is_pwr_src_plugged_in,
       .vbus_debounce_retry    = 1,
};

static struct platform_device cable_detect_device = {
       .name	= "cable_detect",
       .id	= -1,
       .dev	= {
               .platform_data = &cable_detect_pdata,
       },
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id			= 0x0bb4,
	.product_id			= 0x0dff, 
	.product_name			= "Android Phone",
	.manufacturer_name		= "HTC",
	.serial_number			= "123456789012",
	.usb_core_id			= 0,
	.rmnet_transports_interface	= "qti,bam",
	.diag_client_interface		= "diag",
	.fserial_init_string		= "smd:modem,tty,tty:autobot,tty:serial,tty:autobot,tty:acm",
	.nluns 				= 1,
	.cdrom_lun 			= 0x1,
	.vzw_unmount_cdrom 		= 0,
};

#define QCT_ANDROID_USB_REGS 0x086000c8
#define QCT_ANDROID_USB_SIZE 0xc8
static struct resource resources_android_usb[] = {
	{
		.start  = QCT_ANDROID_USB_REGS,
		.end    = QCT_ANDROID_USB_REGS + QCT_ANDROID_USB_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device android_usb_device = {
	.name   = "android_usb",
	.id     = -1,
	.num_resources  = ARRAY_SIZE(resources_android_usb),
	.resource       = resources_android_usb,
	.dev    = {
		.platform_data = &android_usb_pdata,
	},
};

static void msm8909_add_usb_devices(void)
{
	
	platform_device_register(&android_usb_device);
}

static void __init msm8909_dt_reserve(void)
{
	of_scan_flat_dt(dt_scan_for_memory_reserve, NULL);
}

static void __init msm8909_map_io(void)
{
	msm_map_msm8909_io();
}

static struct of_dev_auxdata msm8909_auxdata_lookup[] __initdata = {
	{}
};

static void msm8909_cable_detect_register(void){
	platform_device_register(&cable_detect_device);
}

#if defined(CONFIG_HTC_BATT_8960)
static int critical_alarm_voltage_mv[] = {3000, 3200, 3400};

static struct htc_battery_platform_data htc_battery_pdev_data = {
	.guage_driver = 0,
	.ibat_limit_active_mask = HTC_BATT_CHG_LIMIT_BIT_TALK |
								HTC_BATT_CHG_LIMIT_BIT_NAVI |
								HTC_BATT_CHG_LIMIT_BIT_THRML,
	.iusb_limit_active_mask = 0,

	.critical_low_voltage_mv = 3200,
	.critical_alarm_vol_ptr = critical_alarm_voltage_mv,
	.critical_alarm_vol_cols = sizeof(critical_alarm_voltage_mv) / sizeof(int),
	.overload_vol_thr_mv = 4000,
	.overload_curr_thr_ma = 0,
	.smooth_chg_full_delay_min = 2,
	.decreased_batt_level_check = 0,
	.force_shutdown_batt_vol = 3000,
	.usb_temp_monitor_enable = 1,
	
	.usb_overheat_rising_threshold = 300,
	.usb_temp_overheat_threshold = 650,
	.disable_pwrpath_after_eoc = 1,

	.icharger.name = "smb358",
	.icharger.get_charging_source = smb358_get_charging_source,
	.icharger.get_charging_enabled = smb358_get_charging_enabled,
	.icharger.set_charger_enable = smb358_charger_enable,
	.icharger.set_pwrsrc_enable = smb358_pwrsrc_enable,
	.icharger.set_ftm_charge_enable_type = smb358_set_ftm_charge_enable_type,
	.icharger.set_pwrsrc_and_charger_enable =
						smb358_set_pwrsrc_and_charger_enable,
	.icharger.set_limit_charge_enable = smb358_limit_charge_enable,
	.icharger.set_chg_iusbmax = pm8916_set_chg_iusbmax,
	.icharger.set_chg_vin_min = pm8916_set_chg_vin_min,
	.icharger.is_ovp = smb358_is_charger_ovp,
	.icharger.is_batt_temp_fault_disable_chg =
						smb358_is_batt_temp_fault_disable_chg,
	.icharger.charger_change_notifier_register =
						cable_detect_register_notifier,
	.icharger.is_safty_timer_timeout = smb358_is_chg_safety_timer_timeout,
	.icharger.get_attr_text = smb358_charger_get_attr_text,
	.icharger.max_input_current = pm8909_set_hsml_target_ma,
	.icharger.is_battery_full_eoc_stop = smb358_is_batt_full_eoc_stop,
	.icharger.get_charge_type = smb358_get_charge_type,
	.icharger.get_chg_usb_iusbmax = pm8916_get_chg_usb_iusbmax,
	.icharger.get_chg_vinmin = pm8916_get_chg_vinmin,
	.icharger.get_input_voltage_regulation =
						pm8916_get_input_voltage_regulation,
	.icharger.set_charger_after_eoc = smb358_set_charger_after_eoc,
	.icharger.is_hv_battery_detection = smb358_is_hv_battery_detection,
	.icharger.dump_all = smb358_dump_all,
	.icharger.is_bad_cable_used = smb358_is_bad_cable_used,

	.igauge.name = "pm8909",
	.igauge.get_battery_voltage = pm8909_get_batt_voltage,
	.igauge.get_battery_current = pm8909_get_batt_current,
	.igauge.get_battery_temperature = smb358_get_batt_temperature,
	.igauge.get_battery_id = pm8909_get_batt_id,
	.igauge.get_battery_id_mv = pm8909_get_batt_id_mv,
	.igauge.get_battery_soc = pm8909_get_batt_soc,
	.igauge.get_battery_cc = pm8916_get_batt_cc,
	.igauge.is_battery_full = smb358_is_batt_full,
	.igauge.is_battery_temp_fault = smb358_is_batt_temperature_fault,
	.igauge.get_attr_text = pm8909_gauge_get_attr_text,
	.igauge.get_usb_temperature = pm8909_get_usb_temperature,
	.igauge.set_lower_voltage_alarm_threshold =
						pm8909_batt_lower_alarm_threshold_set,
	.igauge.store_battery_data = pm8909_bms_store_battery_data_emmc,
	.igauge.store_battery_ui_soc = pm8909_bms_store_battery_ui_soc,
	.igauge.get_battery_ui_soc = pm8909_bms_get_battery_ui_soc,
};
static struct platform_device htc_battery_pdev = {
	.name = "htc_battery",
	.id = -1,
	.dev    = {
		.platform_data = &htc_battery_pdev_data,
	},
};

static void msm8909_add_batt_devices(void)
{
	platform_device_register(&htc_battery_pdev);
}

static struct platform_device htc_battery_cell_pdev = {
	.name = "htc_battery_cell",
	.id = -1,
};

int __init htc_batt_cell_register(void)
{
	platform_device_register(&htc_battery_cell_pdev);
	return 0;
}
#endif 

static void __init msm8909_add_drivers(void)
{
	msm_smd_init();
	msm_rpm_driver_init();
	msm_spm_device_init();
	msm_pm_sleep_status_init();
#if defined(CONFIG_HTC_BATT_8960)
	htc_batt_cell_register();
	msm8909_add_batt_devices();
#endif
	msm8909_cable_detect_register();
	msm8909_add_usb_devices();
}

static void __init msm8909_init(void)
{
	struct of_dev_auxdata *adata = msm8909_auxdata_lookup;

	of_platform_populate(NULL, of_default_bus_match_table, adata, NULL);
	msm_smem_init();

	if (socinfo_init() < 0)
		pr_err("%s: socinfo_init() failed\n", __func__);

	msm8909_add_drivers();

#ifdef CONFIG_HTC_BUILD_EDIAG
	platform_device_register(&android_pmem_ediag_device);
	platform_device_register(&android_pmem_ediag1_device);
	platform_device_register(&android_pmem_ediag2_device);
	platform_device_register(&android_pmem_ediag3_device);
#endif
}

static const char *msm8909_dt_match[] __initconst = {
	"htc,msm8909-a16",
	NULL
};

DT_MACHINE_START(MSM8909_DT,
	"HTC Corporation. MSM8909-PM8909 A16")
	.map_io = msm8909_map_io,
	.init_machine = msm8909_init,
	.dt_compat = msm8909_dt_match,
	.reserve = msm8909_dt_reserve,
	.smp = &msm8916_smp_ops,
MACHINE_END
