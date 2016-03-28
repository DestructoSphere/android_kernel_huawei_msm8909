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

#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/usb/msm_hsusb.h>

#ifndef _CHARGER_CORE
#define _CHARGER_CORE
/*options of charge states from chip*/
#define CHARGELOG_SIZE      (2048)


#define MAX_CHARGER_TYPE_STRING_LEN        20
#define CHARGER_IC_QCOM                    0
#define CHARGER_IC_NO_DPM                  1
#define CHARGER_IC_HAVE_DPM                2

#define QCOM_LINEAR_CHARGER        0
#define TI_BQ24152_CHARGER         1
#define TI_BQ24192_CHARGER         2
#define TI_BQ24296_CHARGER      3
#define MAX77819_CHARGER      4
#define UNKNOW_CHARGER             10
#define DEFAULT_AC_IIN_CURRENT             1000

/*************************struct define area***************************/
struct charger_ic_type{
    char *charger_type_string;
    int  charger_index;
};

struct charge_core_data{
    unsigned int iin_ac;
};

struct huawei_charger_irq
{
    int		irq;
    unsigned long	disabled;
    bool            is_wake;
};

struct charger_core_info{
    struct charger_ic_type  charger_type_info;
    struct charge_core_data data;
    struct qpnp_vadc_chip   *vadc_dev;
    struct huawei_charger_irq		irqs;
};
struct charge_device_ops
{
    int (*set_runningtest)(int value);   /* for running test charging control*/
    int (*set_enable_charger)(int value); /* for factory diag function */
    int (*set_in_thermal)(int value); /* for power genius to set current limit */
    int (*shutdown_q4)(int value); /* shutdown q4 */
    int (*shutdown_wd)(int value); /* shutdown watchdog */
    int (*set_usb_current)(int value); /* set usb input current */
    int (*get_register_head)(char *reg_head);
    int (*dump_register)(char *reg_value); /* dump charger regs*/
};

/****************variable and function declarationn area******************/
extern struct charger_core_info *charge_core_get_params(void);
extern int charge_ops_register(struct charge_device_ops *ops);
#endif
