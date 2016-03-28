/*
 * drivers/power/huawei_charger.h
 *
 *huawei charger driver
 *
 * Copyright (C) 2012-2015 HUAWEI, Inc.
 * Author: HUAWEI, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/charger_core.h>
#include <linux/qpnp/qpnp-adc.h>

#ifndef _HUAWEI_CHARGER
#define _HUAWEI_CHARGER

/*************************marco define area***************************/
#ifndef TRUE
#define TRUE (1)
#endif
#ifndef FALSE
#define FALSE (0)
#endif

/* Macros for running test charging control */
/*Running test result*/
/*And charge abnormal info*/
#define CHARGE_STATUS_FAIL 	(0<<0) //Indicate running test charging status fail
#define CHARGE_STATUS_PASS 	(1<<0) //Indicate running test charging status pass
#define BATTERY_FULL 			(1<<1)
#define USB_NOT_PRESENT 		(1<<2)
#define REGULATOR_BOOST		(1<<3)
#define CHARGE_LIMIT			(1<<4)
#define BATTERY_HEALTH 		(1<<5)
#define CHARGER_OVP			(1<<6)
#define OCP_ABNORML			(1<<7)
#define BATTERY_VOL_ABNORML	(1<<8)
#define BATTERY_TEMP_ABNORML	(1<<9)
#define BATTERY_ABSENT			(1<<10)

#define CHARGE_OCP_THR	-2500000 //charge current abnormal threshold
#define BATTERY_OCP_THR 	5000000 //discharge current abnormal threshold
#define BATTERY_VOL_THR_HI	4500000 //battery voltage abnormal high threshold
#define BATTERY_VOL_THR_LO	2500000 //battery voltage abnormal low threshold
#define BATTERY_TEMP_HI	780 //battery high temp threshold
#define BATTERY_TEMP_LO	-100 //battery low temp threshold
#define WARM_VOL_BUFFER	100 //cfg_warm_bat_mv need have a 100mV buffer
#define WARM_TEMP_THR		390 //battery warm temp threshold for running test
#define HOT_TEMP_THR		600 //battery hot temp threshold for running test
#define BATT_FULL			100 //battery full capactiy
#define USB_VALID_MASK		0xC0
#define USB_VALID_OVP_VALUE    0x40
#define COLD_HOT_TEMP_BUFFER		200
#define PASS_MASK			0x1E    //Running test pass mask
#define FAIL_MASK			0x7E0   //Running test fail mask
#define WARM_TEMP_BUFFER		30

/* Linear peripheral subtype values */
#define USB_PTH_STS_REG				    0x09
#define PERP_SUBTYPE_REG			    0x05
#define BAT_IF_PRES_STATUS_REG          0x08
#define BAT_IF_TEMP_STATUS_REG			0x09
#define INT_RT_STS_REG				    0x10
#define LBC_CHGR_SUBTYPE			    0x15
#define LBC_BAT_IF_SUBTYPE			    0x16
#define LBC_USB_PTH_SUBTYPE			    0x17
#define LBC_MISC_SUBTYPE			    0x18
#define CHG_CTRL_REG				    0x49
#define CHG_STATUS_REG				0x09

#define CHG_VIN_MIN_LOOP_BIT			BIT(3)
#define BATT_PRES_MASK				    BIT(7)
#define VBUS_PRES_MASK                  BIT(1)

#define BATT_DEFAULT_TEMP		250
#define BATT_ABSENT_TEMP		90
#define BATT_DEFAULT_VOL		3800
#define LED_CHECK_PERIOD_MS		3000

#define MAX_SIZE    2048

enum charge_sysfs_type
{
    CHARGE_SYSFS_IIN_THERMAL = 0,         /* set input current for thermal*/
    CHARGE_SYSFS_IIN_RUNNINGTEST,       /* set input current for runningtest*/
    CHARGE_SYSFS_ENABLE_CHARGER,        /* enabe/disable charger*/
    CHARGE_SYSFS_SHUTDOWN_Q4,            /* shutdown Q4*/
    CHARGE_SYSFS_SHUTDOWN_WD,           /* shutdown watchdog*/
    CHARGE_SYSFS_USB_CURRENT,             /* usb_current */
    CHARGE_SYSFS_RUNNING_TEST_STATUS,    /* for runningtest to get the result*/
    CHARGE_SYSFS_CAPACITY_FCC,
    CHARGE_SYSFS_CHARGELOG_HEAD,
    CHARGE_SYSFS_CHARGELOG,
};

struct charge_sysfs_data
{
    int iin_rt;
    unsigned int iin_thl;
    unsigned int capacity_fcc;
    int shutdown_q4;
    int shutdown_wd;
    int usb_current;
    char reg_value[CHARGELOG_SIZE];
};

struct charge_device_info
{
    struct device   *dev;
    spinlock_t			hw_access_lock;

    struct delayed_work		nff_work;
    struct charge_device_ops *ops;
    struct charge_core_data *core_data;
    struct charge_sysfs_data sysfs_data;

    struct qpnp_adc_tm_btm_param	adc_param;
    struct qpnp_vadc_chip		*vadc_dev;
    struct qpnp_adc_tm_chip		*adc_tm_dev;
    struct spmi_device      *spmi;
    struct power_supply		*usb_psy;
    struct power_supply		*batt_psy;
    struct power_supply     *bms_psy;
    struct huawei_charger_irq		irqs;
    struct wake_lock		chg_wake_lock;
    struct wake_lock		led_wake_lock;

    bool		usb_present;
    int			chgr_base;
    int			bat_if_base;
    int			usb_chgpth_base;
    int			misc_base;
    int			bms_base;
    int          chrg_config;
    int          shutdown_q4;
    int          shutdown_wd;
    int          usb_current;
    int          warm_bat_decidegc;
    int          warm_bat_chg_ma;
    int          warm_bat_mv;
    int          cool_bat_decidegc;
    int          cool_bat_chg_ma;
    int          cool_bat_mv;
    int          hot_bat_decidegc;
    int          cold_bat_decidegc;
    int          running_test_settled_status;
};

/****************variable and function declarationn area******************/

int charge_ops_register(struct charge_device_ops *ops);

int is_usb_chg_exist(void);
int huawei_charger_get_battery_temperature(void);
int huawei_charger_get_battery_voltage_now(void);
int huawei_charger_battery_is_exist(void);
int qpnp_lbc_is_in_vin_min_loop(void);
int get_running_test_status(void); /* for running test to get test result */
int set_running_test_flag(int value); /* for running test to set charging flag */

extern bool otg_enabled;

extern int is_vbus_otg_regulator_enabled(void);
extern void do_max77819_dcin_valid_irq(int present);
#endif
