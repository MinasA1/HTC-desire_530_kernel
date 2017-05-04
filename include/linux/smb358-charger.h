/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
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

#ifndef __SMB358_CHARGER_H
#define __SMB358_CHARGER_H

#include <linux/errno.h>
#include <linux/power_supply.h>
#ifdef CONFIG_HTC_BATT_8960
#include <mach/htc_charger.h>
#endif

#define FALSE       0
#define TRUE        1

#define USB_MA_0       (0)
#define USB_MA_2       (2)
#define USB_MA_100     (100)
#define USB_MA_200     (200)
#define USB_MA_300     (300)
#define USB_MA_400     (400)
#define USB_MA_450     (450)
#define USB_MA_500     (500)
#define USB_MA_900     (900)
#define USB_MA_1000    (1000)
#define USB_MA_1100    (1100)
#define USB_MA_1200	(1200)
#define USB_MA_1500	(1500)
#define USB_MA_1600	(1600)

#ifdef CONFIG_SMB358_CHARGER
#ifdef CONFIG_HTC_BATT_8960
int smb358_get_batt_temperature(int *result);
int smb358_get_charging_source(int *result);
int smb358_get_charging_enabled(int *result);
int smb358_is_bad_cable_used(int *result);
int smb358_charger_enable(bool enable);
int smb358_pwrsrc_enable(bool enable);
int smb358_set_ftm_charge_enable_type(enum htc_ftm_power_source_type ftm_src);
int smb358_set_pwrsrc_and_charger_enable(enum htc_power_source_type src,
			bool chg_enable, bool pwrsrc_enable);
int smb358_fake_chg_uv_irq_handler(void);
int smb358_dump_all(void);
int smb358_is_batt_temp_fault_disable_chg(int *result);
int smb358_is_batt_temperature_fault(int *result);
int smb358_is_charger_ovp(int* result);
int smb358_is_batt_full(int *result);
int smb358_is_batt_full_eoc_stop(int *result);
int smb358_get_charge_type(void);
int smb358_is_chg_safety_timer_timeout(int *result);
int smb358_limit_charge_enable(bool enable);
bool smb358_is_battery_present(void);
int smb358_get_battery_status(void);
int smb358_limit_input_current(bool enable, int reason);
int smb358_charger_get_attr_text(char *buf, int size);
int smb358_set_charger_after_eoc(bool enable);
int smb358_is_hv_battery_detection(int *result);
int pm8909_set_hsml_target_ma(int target_ma);
#endif
#else 
#ifdef CONFIG_HTC_BATT_8960
static inline int smb358_get_batt_temperature(int *result)
{
	return -ENXIO;
}
static inline int smb358_get_charging_source(int *result)
{
	return -ENXIO;
}
static inline int smb358_get_charging_enabled(int *result)
{
	return -ENXIO;
}
static inline int smb358_is_bad_cable_used(int *result)
{
	return -ENXIO;
}
static inline int smb358_charger_enable(bool enable)
{
	return -ENXIO;
}
static inline int smb358_pwrsrc_enable(bool enable)
{
	return -ENXIO;
}
static inline int smb358_set_ftm_charge_enable_type(enum htc_ftm_power_source_type ftm_src)
{
	return -ENXIO;
}
static inline int smb358_set_pwrsrc_and_charger_enable(enum htc_power_source_type src,
			bool chg_enable, bool pwrsrc_enable)
{
	return -ENXIO;
}
static inline int smb358_fake_chg_uv_irq_handler(void)
{
	return -ENXIO;
}
static inline int smb358_dump_all(void)
{
	return -ENXIO;
}
static inline int smb358_is_batt_temp_fault_disable_chg(int *result)
{
	return -ENXIO;
}
static inline int smb358_is_batt_temperature_fault(int *result)
{
	return -ENXIO;
}
static inline int smb358_is_charger_ovp(int* result)
{
	return -ENXIO;
}
static inline int smb358_is_batt_full(int *result)
{
	return -ENXIO;
}
static inline int smb358_is_batt_full_eoc_stop(int *result)
{
	return -ENXIO;
}
static inline int smb358_get_charge_type(void)
{
	return -ENXIO;
}
static inline int smb358_is_chg_safety_timer_timeout(int *result)
{
	return -ENXIO;
}
static inline int smb358_limit_charge_enable(bool enable)
{
	return -ENXIO;
}
static inline bool smb358_is_battery_present(void)
{
	return -ENXIO;
}
static inline int smb358_get_battery_status(void)
{
	return -ENXIO;
}
static inline int smb358_limit_input_current(bool enable, int reason)
{
	return -ENXIO;
}
static inline int smb358_charger_get_attr_text(char *buf, int size)
{
	return -ENXIO;
}
static inline int smb358_set_charger_after_eoc(bool enable)
{
	return -ENXIO;
}
static inline int smb358_is_hv_battery_detection(int *result)
{
	return -ENXIO;
}
static inline int pm8909_set_hsml_target_ma(int target_ma)
{
	return -ENXIO;
}
#endif 
#endif 
#endif 

