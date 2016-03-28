/*
 * drivers/power/huawei_charger.c
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/wakelock.h>
#include <linux/usb/otg.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/power_supply.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/notifier.h>
#include <linux/mutex.h>
#include <linux/power/huawei_charger.h>
#include <linux/power/bq24296m_charger.h>
#include <linux/spmi.h>
#include <linux/wakelock.h>
#include <linux/charger_core.h>
#include <linux/irqchip/qpnp-int.h>
#include <linux/of_batterydata.h>
#include <linux/qpnp/qpnp-adc.h>

#ifdef CONFIG_HUAWEI_PMU_DSM
#include <linux/power/huawei_dsm_charger.h>

/*8916 pmic LBC_CHGR registers*/
static u8 LBC_CHGR[] =
{
    0x08,
    0x09,
    0x0A,
    0x0B,
    0x10,
    0x11,
    0x12,
    0x13,
    0x14,
    0x15,
    0x16,
    0x18,
    0x19,
    0x1A,
    0x1B,
    0x40,
    0x41,
    0x43,
    0x44,
    0x45,
    0x47,
    0x49,
    0x4A,
    0x4C,
    0x4D,
    0x50,
    0x52,
    0x55,
    0x5B,
    0x5E,
    0x5F,
    0x60,
    0x61,
    0x62,
    0x63,
    0x64,
    0x65,
    0x66,
    0x69,
    0x6A
};
/*8916 pmic LBC_BAT_IF registers*/
static u8 LBC_BAT_IF[] =
{
    0x08,
    0x09,
    0x0A,
    0x10,
    0x11,
    0x12,
    0x13,
    0x14,
    0x15,
    0x16,
    0x18,
    0x19,
    0x1A,
    0x1B,
    0x48,
    0x49,
    0x4A,
    0x4F,
    0xD0
};
/*8916 pmic LBC_USB registers*/
static u8 LBC_USB[] =
{
    0x08,
    0x09,
    0x10,
    0x11,
    0x12,
    0x13,
    0x14,
    0x15,
    0x16,
    0x18,
    0x19,
    0x1A,
    0x1B,
    0x42,
    0x47,
    0x4E,
    0x4F,
    0xD0
};
/*8916 pmic LBC_MISC registers*/
static u8 LBC_MISC[] =
{
    0x40,
    0x41,
    0x42,
    0x43,
    0x49,
    0xCD,
    0xCE,
    0xD0
};

/*8916 pmic VM_BMS registers*/
static u8 VM_BMS[] =
{
    0x08,
    0x09,
    0x10,
    0x11,
    0x12,
    0x13,
    0x14,
    0x15,
    0x16,
    0x18,
    0x19,
    0x1A,
    0x1B,
    0x40,
    0x42,
    0x43,
    0x44,
    0x46,
    0x47,
    0x50,
    0x51,
    0x53,
    0x55,
    0x56,
    0x57,
    0x58,
    0x5A,
    0x5B,
    0x5C,
    0x5D,
    0x5E,
    0x5F,
    0x60,
    0x61,
    0x62,
    0x63,
    0x64,
    0x65,
    0x66,
    0x67,
    0x6A,
    0x6B,
    0xB0,
    0xB1,
    0xB2,
    0xB3,
    0xB4,
    0xB5,
    0xB6,
    0xB7,
    0xB8,
    0xB9,
    0xC0,
    0xC1,
    0xC2,
    0xC3,
    0xC4,
    0xC5,
    0xC6,
    0xC7,
    0xC8,
    0xC9,
    0xCA,
    0xCB,
    0xCC,
    0xCD,
    0xCE,
    0xCF,
    0xD0,
    0xD8,
    0xD9,
    0xDA,
    0xDB
};
#endif

struct charge_device_ops *g_ops = NULL;
struct device *charge_dev = NULL;

struct charger_core_info *g_charger_core_para = NULL;
struct charge_device_info *g_charger_device_para = NULL;

/* used for usbin_valid irq, if otg is enabled, usbin_valid irq return directly */
bool otg_enabled = false;

static int huawei_charger_pmic_reg_read(struct charge_device_info *di, u16 base, u8 *val, int count)
{
    int rc = 0;
    struct spmi_device *spmi = di->spmi;
    unsigned long flags;

    if (base == 0)
    {
        pr_err("base cannot be zero base=0x%02x sid=0x%02x rc=%d\n",base, spmi->sid, rc);
        return -EINVAL;
    }
    spin_lock_irqsave(&di->hw_access_lock, flags);
    rc = spmi_ext_register_readl(spmi->ctrl, spmi->sid, base, val, count);
    spin_unlock_irqrestore(&di->hw_access_lock, flags);
    if (rc)
    {
        pr_err("SPMI read failed base=0x%02x sid=0x%02x rc=%d\n",base, spmi->sid, rc);
#ifdef CONFIG_HUAWEI_PMU_DSM
        /* if spmi read fail, record this log, and notify to the dsm server*/
        //DSM_PMU_LOG(charger_dclient, DSM_SPMI_ABNORMAL_ERROR_NO,
                    //"[%s]spmi read failed, rc=%d\n",__func__, rc);

#endif
    }

    return rc;
}

int huawei_charger_battery_is_exist(void)
{
    u8 reg_val;
    int rc;
    struct charge_device_info *di = g_charger_device_para;

    if(NULL == di)
    {
        pr_err("get battery battery present status failed!\n");
        return -1;
    }

    rc = huawei_charger_pmic_reg_read(di, di->bat_if_base + BAT_IF_PRES_STATUS_REG,&reg_val, 1);
    if (rc)
    {
        pr_err("Failed to read battery status read failed rc=%d\n",rc);
        return 0;
    }

    return (reg_val & BATT_PRES_MASK) ? 1 : 0;

}

static int huawei_charger_vbus_is_exist(struct charge_device_info *di)
{
    u8 reg_val = 0;
    int rc = 0;

    rc = huawei_charger_pmic_reg_read(di, di->usb_chgpth_base + INT_RT_STS_REG, &reg_val, 1);
    if (rc)
    {
        pr_err("Failed to read battery status read failed rc=%d\n",rc);
        return -1;
    }

    return (reg_val & VBUS_PRES_MASK) ? 1 : 0;
}

int is_usb_chg_exist(void)
{
    if (!g_charger_device_para)
    {
        pr_err("called before init\n");
        return -EINVAL;
    }
    return huawei_charger_vbus_is_exist(g_charger_device_para);
}

int huawei_charger_get_battery_temperature(void)
{
    int rc = 0;
    struct qpnp_vadc_result results;
    struct charge_device_info *di = NULL;

    di = g_charger_device_para;
    if(NULL == di)
    {
        pr_err("get battery temp failed!\n");
        return BATT_DEFAULT_TEMP;
    }

    if (!huawei_charger_battery_is_exist())
    {
        return BATT_ABSENT_TEMP;
    }

    rc = qpnp_vadc_read(di->vadc_dev, LR_MUX1_BATT_THERM, &results);
    if (rc)
    {
        pr_err("Unable to read batt temperature rc=%d\n", rc);
        return BATT_DEFAULT_TEMP;
    }

    return (int)results.physical;
}

int huawei_charger_get_battery_voltage_now(void)
{
    int rc = 0;
    struct qpnp_vadc_result results;
    struct charge_device_info *di = NULL;

    di = g_charger_device_para;
    if(NULL == di)
    {
        pr_err("get battery vol failed!\n");
        return BATT_DEFAULT_VOL;
    }

    rc = qpnp_vadc_read(di->vadc_dev, VBAT_SNS, &results);
    if (rc)
    {
        pr_err("Unable to read vbat rc=%d\n", rc);
        return 0;
    }

    return results.physical;
}

/*===========================================
FUNCTION: qpnp_lbc_is_in_vin_min_loop
DESCRIPTION: to get pmic8916 vin_min loop value
IPNUT:	charge_device_info *di
RETURN:	a int value, 1 means in vin_min loop; 0 means not in vin_min loop
=============================================*/
int qpnp_lbc_is_in_vin_min_loop(void)
{
    u8 reg_val = 0;
    int rc;
    struct charge_device_info *di = NULL;

    di = g_charger_device_para;
    if(NULL == di)
    {
        pr_err("get battery vol failed!\n");
        return 0;
    }
    rc = huawei_charger_pmic_reg_read(di, di->chgr_base + CHG_STATUS_REG,
                                      &reg_val, 1);
    if (rc)
    {
        pr_err("Failed to read chg status rc=%d\n", rc);
        return rc;
    }

    pr_debug("CHG_STATUS_REG %x\n", reg_val);
    return (reg_val & CHG_VIN_MIN_LOOP_BIT) ? 1 : 0;
}

/*===========================================
FUNCTION: get_running_test_result
DESCRIPTION: For running test apk to get the running test result and status
IPNUT:	struct charge_device_info *di
RETURN:	a int value, we use bit0 to bit10 to tell running test apk the test
result and status, if bit0 is 0, the result is fail, bit5 to bit11 is the
failed reason
if bit0 is 1, the result is pass.
=============================================*/
static int get_running_test_result(struct charge_device_info *di)
{
    int result = 0;
    int cur_status = 0;
    int is_temp_vol_current_ok = 1;
    int vol = 0, temp = 250, health = 0, current_ma =0, capacity;
    u8 usbin_valid_rt_sts = 0;
    int rc;
    int mode = 0;
    int battery_present = 0;

    union power_supply_propval val = {0,};

    if(di->batt_psy)
    {
        di->batt_psy->get_property(di->batt_psy,POWER_SUPPLY_PROP_STATUS,&val);
        cur_status = val.intval;
        di->batt_psy->get_property(di->batt_psy,POWER_SUPPLY_PROP_CURRENT_NOW,&val);
        current_ma = val.intval;
        di->batt_psy->get_property(di->batt_psy,POWER_SUPPLY_PROP_VOLTAGE_NOW,&val);
        vol = val.intval;
        di->batt_psy->get_property(di->batt_psy,POWER_SUPPLY_PROP_TEMP,&val);
        temp = val.intval;
        di->batt_psy->get_property(di->batt_psy,POWER_SUPPLY_PROP_CAPACITY,&val);
        capacity = val.intval;
        di->batt_psy->get_property(di->batt_psy,POWER_SUPPLY_PROP_HEALTH,&val);
        health = val.intval;
    }

    battery_present = huawei_charger_battery_is_exist();
    mode = is_vbus_otg_regulator_enabled();

    pr_debug("get_running_test_result info: usb=%d batt_pres=%d batt_volt=%d batt_temp=%d"
             " cur_status=%d current_ma=%d setting status=%d\n",
             huawei_charger_vbus_is_exist(di),
             battery_present,
             vol,
             temp,
             cur_status,
             current_ma,
             di->running_test_settled_status
            );

    /* mask current_ma logic, as we do not use hardware current in A/R projects */
    /* curren_ma is caculated by qcom software, sometimes it is too large */
    /*if((CHARGE_OCP_THR > current_ma) || (BATTERY_OCP_THR < current_ma)){
    	result |= OCP_ABNORML;
    	is_temp_vol_current_ok = 0;
    	pr_info("Find OCP! current_ma is %d\n", current_ma);
    }*/

    if((BATTERY_VOL_THR_HI < vol) || (BATTERY_VOL_THR_LO > vol))
    {
        result |= BATTERY_VOL_ABNORML;
        is_temp_vol_current_ok = 0;
        pr_err("Battery voltage is abnormal! voltage is %d\n", vol);
    }

    if((BATTERY_TEMP_HI < temp) || (BATTERY_TEMP_LO > temp))
    {
        result |= BATTERY_TEMP_ABNORML;
        is_temp_vol_current_ok = 0;
        pr_err("Battery temperature is abnormal! temp is %d\n", temp);
    }

    if(!is_temp_vol_current_ok)
    {
        result |= CHARGE_STATUS_FAIL;
        pr_err("running test find abnormal battery status, the result is 0x%x\n", result);
        return result;
    }

    if(cur_status == di->running_test_settled_status)
    {
        result |= CHARGE_STATUS_PASS;
        return result;

    }
    else if((POWER_SUPPLY_STATUS_CHARGING == cur_status)
    &&(POWER_SUPPLY_STATUS_DISCHARGING == di->running_test_settled_status))
    {
        result |= CHARGE_STATUS_FAIL;
        pr_err("running test failed!!! the result is 0x%x\n", result);
        return result;

    }
    else if(POWER_SUPPLY_STATUS_CHARGING == di->running_test_settled_status)
    {
        if((POWER_SUPPLY_STATUS_DISCHARGING == cur_status)
        && (BATT_FULL == capacity) && (battery_present)
        && huawei_charger_vbus_is_exist(di))
        {
            cur_status = POWER_SUPPLY_STATUS_FULL;
        }

        if(POWER_SUPPLY_STATUS_FULL == cur_status)
        {
            result |= BATTERY_FULL;
        }

        if(!huawei_charger_vbus_is_exist(di))
        {
            result |= USB_NOT_PRESENT;
        }

        if((g_charger_core_para->charger_type_info.charger_index != QCOM_LINEAR_CHARGER)
        && (1 == mode))
        {
            result |= REGULATOR_BOOST;
        }

        if((vol >= ((di->warm_bat_mv - WARM_VOL_BUFFER)*1000))
        && (WARM_TEMP_THR <= temp))
        {
            result |= CHARGE_LIMIT;
        }

        if((POWER_SUPPLY_STATUS_NOT_CHARGING == cur_status)&& (HOT_TEMP_THR > temp)
        &&(g_charger_core_para->charger_type_info.charger_index == QCOM_LINEAR_CHARGER))
        {
            result |= CHARGE_LIMIT;
            pr_err("settled_status = %d cur_status = %d temp = %d\n",
                    di->running_test_settled_status, cur_status, temp);
        }

        if(((POWER_SUPPLY_STATUS_DISCHARGING == cur_status)
        || (POWER_SUPPLY_STATUS_NOT_CHARGING == cur_status))
        && (g_charger_core_para->charger_type_info.charger_index != QCOM_LINEAR_CHARGER))
        {
            if(((temp >= (di->hot_bat_decidegc - COLD_HOT_TEMP_BUFFER))&&(HOT_TEMP_THR > temp))
            ||(temp <= (di->cold_bat_decidegc + COLD_HOT_TEMP_BUFFER)))
            {
                result |= CHARGE_LIMIT;
                pr_err("settled_status %d cur_status=%d temp=%d "
                        "di->hot_bat_decidegc =%d "
                        "di->cold_bat_decidegc =%d\n",
                        di->running_test_settled_status, cur_status, temp,
                        di->hot_bat_decidegc,
                        di->cold_bat_decidegc);
            }
        }

        if((POWER_SUPPLY_HEALTH_OVERHEAT == health)
        || (POWER_SUPPLY_HEALTH_COLD ==health))
        {
            result |= BATTERY_HEALTH;
        }

        rc = huawei_charger_pmic_reg_read(di, di->usb_chgpth_base + USB_PTH_STS_REG, &usbin_valid_rt_sts, 1);
        if (rc)
        {
            pr_err("spmi read failed: addr=%03X, rc=%d\n",di->usb_chgpth_base + USB_PTH_STS_REG, rc);
        }
        else
        {
            if ((usbin_valid_rt_sts & USB_VALID_MASK)== USB_VALID_OVP_VALUE)
            {
                result |= CHARGER_OVP;
            }
        }

        if(!battery_present)
        {
            result |= BATTERY_ABSENT;
        }

        /* add FAIL_MASK, if pass and fail reasons are */
        /* meet at the same time, report fail */
        if((result & PASS_MASK) && (!(result & FAIL_MASK)))
        {
            result |= CHARGE_STATUS_PASS;
        }
        else
        {
            result |= CHARGE_STATUS_FAIL;
            pr_info("get_running_test_result: usb=%d batt_pres=%d batt_volt=%d batt_temp=%d"
                    " capacity=%d cur_status=%d current_ma=%d setting status=%d result=0x%x\n",
                    huawei_charger_vbus_is_exist(di),
                    battery_present,
                    vol,
                    temp,
                    capacity,
                    cur_status,
                    current_ma,
                    di->running_test_settled_status,
                    result
                   );
        }

        return result;
    }
    else
    {
        pr_info("other else status!");
        /* if the setting status is discharging, meanwhile */
        /* if(cur_status != POWER_SUPPLY_STATUS_CHARGING*/
        /* && cur_status != POWER_SUPPLY_STATUS_DISCHARGING) */
        /* We return 1(PASS) directly, as when set discharging*/
        /* it do not need to care high temperature, battery full or unknow*/
        pr_info("usb=%d batt_pres=%d batt_volt=%d batt_temp=%d"
                " cur_status=%d current_ma=%d setting status=%d\n",
                huawei_charger_vbus_is_exist(di),
                battery_present,
                vol,
                temp,
                cur_status,
                current_ma,
                di->running_test_settled_status
               );
        return 1;
    }
}

int set_running_test_flag(int value)
{
    struct charge_device_info *di = NULL;

    di = g_charger_device_para;
    if(NULL == di)
    {
        pr_err("charge_device is not ready! cannot set runningtest flag\n");
        return -1;
    }
    if(value)
    {
        di->running_test_settled_status = POWER_SUPPLY_STATUS_DISCHARGING;
    }
    else
    {
        di->running_test_settled_status = POWER_SUPPLY_STATUS_CHARGING;
    }
    return 0;
}

int get_running_test_status(void)
{
    struct charge_device_info *di = NULL;

    di = g_charger_device_para;
    if(NULL == di)
    {
        pr_err("charge_device is not ready!\n");
        return 0;
    }
    
    return get_running_test_result(di);
}


static void charge_nff_work(struct work_struct *work)
{
    //struct charge_device_info *di = container_of(work, struct charge_device_info, nff_work.work);

    /* to do*/

    return ;
}

static int charger_dump_loginfo(char *reg_value)
{
    int string_length = 0;
    char buff[100] = {0};
    struct charge_device_info *di = g_charger_device_para;
    union power_supply_propval ret = {0,};

    if (di->usb_psy)
    {
         di->usb_psy->get_property(di->usb_psy,POWER_SUPPLY_PROP_TYPE, &ret);
    }
    switch(ret.intval)
    {
        case 0:
            snprintf(buff, 100, "%-11s","UNKNOW");
            break;
        case 4:
            snprintf(buff, 100, "%-11s","USB");
            break;
        case 5:
            snprintf(buff, 100, "%-11s","DCP");
            break;
        default:
            snprintf(buff, 100, "%-11s","NON_STAND");
            break;
    }
    strncat(reg_value, buff, strlen(buff));
    string_length += strlen(buff);

    if (di->bms_psy)
    {
        di->bms_psy->get_property(di->bms_psy,POWER_SUPPLY_PROP_CURRENT_NOW, &ret);
    }
    snprintf(buff, 100, "%-7d",ret.intval/1000);
    strncat(reg_value, buff, strlen(buff));
    string_length += strlen(buff);

    if (di->batt_psy)
    {
        di->batt_psy->get_property(di->batt_psy,POWER_SUPPLY_PROP_VOLTAGE_NOW, &ret);
    }
    snprintf(buff, 100, "%-6d",ret.intval/1000);
    strncat(reg_value, buff, strlen(buff));
    string_length += strlen(buff);

    if (di->bms_psy)
    {
        di->bms_psy->get_property(di->bms_psy,POWER_SUPPLY_PROP_VOLTAGE_OCV, &ret);
    }
    snprintf(buff, 100, "%-6d",ret.intval/1000);
    strncat(reg_value, buff, strlen(buff));
    string_length += strlen(buff);

    if (di->batt_psy)
    {
        di->batt_psy->get_property(di->batt_psy,POWER_SUPPLY_PROP_CHARGE_TYPE, &ret);
    }
    snprintf(buff, 100, "%-7d",ret.intval);
    strncat(reg_value, buff, strlen(buff));
    string_length += strlen(buff);

    snprintf(buff, 100, "%-10d",di->chrg_config);
    strncat(reg_value, buff, strlen(buff));
    string_length += strlen(buff);

    if (di->batt_psy)
    {
        di->batt_psy->get_property(di->batt_psy,POWER_SUPPLY_PROP_HEALTH, &ret);
    }
    snprintf(buff, 100, "%-9d",ret.intval);
    strncat(reg_value, buff, strlen(buff));
    string_length += strlen(buff);

    if (di->batt_psy)
    {
        di->batt_psy->get_property(di->batt_psy,POWER_SUPPLY_PROP_STATUS, &ret);
    }
    snprintf(buff, 100, "%-9d",ret.intval);
    strncat(reg_value, buff, strlen(buff));
    string_length += strlen(buff);

    if (di->bms_psy)
    {
        di->bms_psy->get_property(di->bms_psy,POWER_SUPPLY_PROP_CAPACITY, &ret);
    }
    snprintf(buff, 100, "%-11d",ret.intval);
    strncat(reg_value, buff, strlen(buff));
    string_length += strlen(buff);

    if (di->bms_psy)
    {
        di->bms_psy->get_property(di->bms_psy,POWER_SUPPLY_PROP_BMS_LOG, &ret);
    }
    snprintf(buff, 100, "%s",ret.strval);
    strncat(reg_value, buff, strlen(buff));
    string_length += strlen(buff);

    return string_length;
}

static int charger_log_head(char *reg_head)
{
    char buff[26] = {0};
    int string_length = 0;

    snprintf(buff, 26, "chgr_type  ");
    strncat(reg_head, buff, strlen(buff));
    string_length += strlen("chgr_type  ");

    snprintf(buff, 26, "curr   ");
    strncat(reg_head, buff, strlen(buff));
    string_length += strlen("curr   ");

    snprintf(buff, 26, "vol   ");
    strncat(reg_head, buff, strlen(buff));
    string_length += strlen("vol   ");

    snprintf(buff, 26, "ocv   ");
    strncat(reg_head, buff, strlen(buff));
    string_length += strlen("ocv   ");

    snprintf(buff, 26, "type   ");
    strncat(reg_head, buff, strlen(buff));
    string_length += strlen("type   ");

    snprintf(buff, 26, "enabled   ");
    strncat(reg_head, buff, strlen(buff));
    string_length += strlen("enabled   ");

    snprintf(buff, 26, "health   ");
    strncat(reg_head, buff, strlen(buff));
    string_length += strlen("health   ");

    snprintf(buff, 26, "status   ");
    strncat(reg_head, buff, strlen(buff));
    string_length += strlen("status   ");

    snprintf(buff, 26, "capacity   ");
    strncat(reg_head, buff, strlen(buff));
    string_length += strlen("capcity   ");

    snprintf(buff, 26, "rbat   ");
    strncat(reg_head, buff, strlen(buff));
    string_length += strlen("rbat   ");

    snprintf(buff, 26, "temp   ");
    strncat(reg_head, buff, strlen(buff));
    string_length += strlen("temp   ");

    snprintf(buff, 26, "soc_ocv   ");
    strncat(reg_head, buff, strlen(buff));
    string_length += strlen("soc_ocv   ");

    snprintf(buff, 26, "soc_uuc   ");
    strncat(reg_head, buff, strlen(buff));
    string_length += strlen("soc_uuc   ");

    snprintf(buff, 26, "soc_acc   ");
    strncat(reg_head, buff, strlen(buff));
    string_length += strlen("soc_acc   ");

    snprintf(buff, 26, "acc    ");
    strncat(reg_head, buff, strlen(buff));
    string_length += strlen("acc    ");

    snprintf(buff, 26, "fcc    ");
    strncat(reg_head, buff, strlen(buff));
    string_length += strlen("fcc    ");

    return string_length;
}

#if CONFIG_SYSFS
#define CHARGE_SYSFS_FIELD(_name, n, m, store)                \
{                                                   \
    .attr = __ATTR(_name, m, charge_sysfs_show, store),    \
    .name = CHARGE_SYSFS_##n,          \
}

#define CHARGE_SYSFS_FIELD_RW(_name, n)               \
        CHARGE_SYSFS_FIELD(_name, n, S_IWUSR | S_IRUGO,       \
                charge_sysfs_store)

#define CHARGE_SYSFS_FIELD_RO(_name, n)               \
        CHARGE_SYSFS_FIELD(_name, n, S_IRUGO, NULL)

static ssize_t charge_sysfs_show(struct device *dev,
                                 struct device_attribute *attr, char *buf);
static ssize_t charge_sysfs_store(struct device *dev,
                                  struct device_attribute *attr, const char *buf, size_t count);

struct charge_sysfs_field_info
{
    struct device_attribute	attr;
    char name;
};

static struct charge_sysfs_field_info charge_sysfs_field_tbl[] =
{
    CHARGE_SYSFS_FIELD_RW(iin_thermal,       IIN_THERMAL),
    CHARGE_SYSFS_FIELD_RW(iin_runningtest,    IIN_RUNNINGTEST),
    CHARGE_SYSFS_FIELD_RW(enable_charger,    ENABLE_CHARGER),
    CHARGE_SYSFS_FIELD_RW(shutdown_q4,       SHUTDOWN_Q4),
    CHARGE_SYSFS_FIELD_RW(shutdown_wd,    SHUTDOWN_WD),
    CHARGE_SYSFS_FIELD_RW(usb_current,    USB_CURRENT),
    CHARGE_SYSFS_FIELD_RO(running_test_status,    RUNNING_TEST_STATUS),
    CHARGE_SYSFS_FIELD_RO(capacity_fcc,    CAPACITY_FCC),
    CHARGE_SYSFS_FIELD_RO(chargelog_head,    CHARGELOG_HEAD),
    CHARGE_SYSFS_FIELD_RO(chargelog,    CHARGELOG),
};

static struct attribute *charge_sysfs_attrs[ARRAY_SIZE(charge_sysfs_field_tbl) + 1];

static const struct attribute_group charge_sysfs_attr_group =
{
    .attrs = charge_sysfs_attrs,
};

/**********************************************************
*  Function:       charge_sysfs_init_attrs
*  Discription:    initialize charge_sysfs_attrs[] for charge attribute
*  Parameters:   NULL
*  return value:  NULL
**********************************************************/
static void charge_sysfs_init_attrs(void)
{
    int i, limit = ARRAY_SIZE(charge_sysfs_field_tbl);

    for (i = 0; i < limit; i++)
    {
        charge_sysfs_attrs[i] = &charge_sysfs_field_tbl[i].attr.attr;
    }

    charge_sysfs_attrs[limit] = NULL; /* Has additional entry for this */
}
/**********************************************************
*  Function:       charge_sysfs_field_lookup
*  Discription:    get the current device_attribute from charge_sysfs_field_tbl by attr's name
*  Parameters:   name:device attribute name
*  return value:  charge_sysfs_field_tbl[]
**********************************************************/
static struct charge_sysfs_field_info *charge_sysfs_field_lookup(const char *name)
{
    int i, limit = ARRAY_SIZE(charge_sysfs_field_tbl);

    for (i = 0; i < limit; i++)
    {
        if (!strcmp(name, charge_sysfs_field_tbl[i].attr.attr.name))
            break;
    }

    if (i >= limit)
    {
        return NULL;
    }

    return &charge_sysfs_field_tbl[i];
}

/**********************************************************
*  Function:       charge_sysfs_show
*  Discription:    show the value for all charge device's node
*  Parameters:   dev:device
*                      attr:device_attribute
*                      buf:string of node value
*  return value:  0-sucess or others-fail
**********************************************************/
static ssize_t charge_sysfs_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
    struct charge_sysfs_field_info *info = NULL;
    union power_supply_propval val = {0,};
    int string_length = 0;

    struct charge_device_info *di = dev_get_drvdata(dev);
    info = charge_sysfs_field_lookup(attr->attr.name);
    if (!info)
    {
        return -EINVAL;
    }

    switch(info->name)
    {
        case CHARGE_SYSFS_IIN_THERMAL:
        {
            return snprintf(buf, MAX_SIZE, "%u\n", di->sysfs_data.iin_thl);
        }
        case CHARGE_SYSFS_IIN_RUNNINGTEST:
        {
            return snprintf(buf, MAX_SIZE, "%u\n", di->sysfs_data.iin_rt);
        }
        case CHARGE_SYSFS_RUNNING_TEST_STATUS:
        {
            return snprintf(buf, MAX_SIZE, "%u\n", get_running_test_result(di));
        }
        case CHARGE_SYSFS_CHARGELOG_HEAD:
        {
            memset(di->sysfs_data.reg_value, 0, CHARGELOG_SIZE);
            string_length = charger_log_head(di->sysfs_data.reg_value);
            di->ops->get_register_head(di->sysfs_data.reg_value + string_length);
            return snprintf(buf, MAX_SIZE, "%s", di->sysfs_data.reg_value);
        }
        case CHARGE_SYSFS_CHARGELOG:
        {
            memset(di->sysfs_data.reg_value, 0, CHARGELOG_SIZE);
            string_length = charger_dump_loginfo(di->sysfs_data.reg_value);
            di->ops->dump_register(di->sysfs_data.reg_value + string_length);
            return snprintf(buf, MAX_SIZE, "%s", di->sysfs_data.reg_value);
        }
        case CHARGE_SYSFS_CAPACITY_FCC:
        {
            if(di->batt_psy && di->batt_psy->get_property)
            {
                di->batt_psy->get_property(di->batt_psy,POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,&val);
                di->sysfs_data.capacity_fcc = val.intval;
            }
            else
            {
                di->sysfs_data.capacity_fcc  = 0;
            }
            return snprintf(buf, MAX_SIZE, "%u\n", di->sysfs_data.capacity_fcc);
        }
        case CHARGE_SYSFS_ENABLE_CHARGER:
        {
            return snprintf(buf, MAX_SIZE, "%u\n", di->chrg_config);
        }
        case CHARGE_SYSFS_SHUTDOWN_Q4: /* shutdown charge ic Q4*/
        {
            return snprintf(buf, MAX_SIZE, "%u\n", di->shutdown_q4);
        }
        case CHARGE_SYSFS_SHUTDOWN_WD: /* shutdown watchdog*/
        {
            return snprintf(buf, MAX_SIZE, "%u\n", di->shutdown_wd);
        }
        case CHARGE_SYSFS_USB_CURRENT: /* usb_current*/
        {
            return snprintf(buf, MAX_SIZE, "%u\n", di->usb_current);
        }
        
        default:
        {
            pr_err("(%s)NODE ERR!!HAVE NO THIS NODE:(%d)\n",__func__,info->name);
            break;
        }
    }

    return 0;
}
/**********************************************************
*  Function:       charge_sysfs_store
*  Discription:    set the value for charge_data's node which is can be written
*  Parameters:   dev:device
*                      attr:device_attribute
*                      buf:string of node value
*                      count:unused
*  return value:  0-sucess or others-fail
**********************************************************/
static ssize_t charge_sysfs_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
    struct charge_sysfs_field_info *info = NULL;
    struct charge_device_info *di = dev_get_drvdata(dev);
    static bool running_test_flag = false;
    int iin_thermal = 0;
    long val = 0;

    info = charge_sysfs_field_lookup(attr->attr.name);
    if (!info)
    {
        return -EINVAL;
    }

    switch(info->name)
    {
        case CHARGE_SYSFS_IIN_THERMAL: /* hot current limit function*/
        {
            if((strict_strtol(buf, 10, &val) < 0)||(val < 0)||(val > 3000))
            {
                return -EINVAL;
            }

            if((val >= 1)&&(val <= 100))
            {
                di->sysfs_data.iin_thl = 100;
                iin_thermal = 100;
            }
            else
            {
                if(val == 0) /* if set 0, means recovery to normal input current*/
                {
                    di->sysfs_data.iin_thl = di->core_data->iin_ac;
                }
                else
                {
                    di->sysfs_data.iin_thl = val;
                }
                iin_thermal = val;
            }

            di->ops->set_in_thermal(iin_thermal);
            pr_info("THERMAL set input current = %d\n", di->sysfs_data.iin_thl);

            break;
        }

        case CHARGE_SYSFS_IIN_RUNNINGTEST: /* running test charging/discharge function*/
        {
            if((strict_strtol(buf, 10, &val) < 0)||(val < 0)||(val > 3000))
                return -EINVAL;
            if((val == 0)||(val == 1))
                di->sysfs_data.iin_rt = di->core_data->iin_ac;
            else if((val > 1)&&(val <= 100))
                di->sysfs_data.iin_rt = 100;
            else
                di->sysfs_data.iin_rt = val;

            pr_info("IIN_RUNNINGTEST set input current = %d\n", di->sysfs_data.iin_rt);

            if(di->sysfs_data.iin_rt <= 100)
            {
                running_test_flag = true;
            }
            else
            {
                running_test_flag = false;
            }

            if(running_test_flag)
            {
                di->running_test_settled_status = POWER_SUPPLY_STATUS_DISCHARGING;
            }
            else
            {
                di->running_test_settled_status = POWER_SUPPLY_STATUS_CHARGING;
            }
            di->ops->set_runningtest(!running_test_flag);
            break;
        }

        case CHARGE_SYSFS_ENABLE_CHARGER:  /* factory diag function*/
        {
            if((strict_strtol(buf, 10, &val) < 0)||(val < 0) || (val > 1))
            {
                return -EINVAL;
            }

            di->chrg_config = val;
            pr_info("ENABLE_CHARGER set chrg_config = %d\n", di->chrg_config);

            /* as we set rt discharging in set_runningtest, so if set rt discharging, do not set again*/
            if(running_test_flag)
            {
                break;
            }
            di->ops->set_enable_charger(di->chrg_config);
            break;
        }
        /* shutdown q4, set 1 shutdown q4, set 0 to recovery*/
        case CHARGE_SYSFS_SHUTDOWN_Q4:
        {
            if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 1))
                return -EINVAL;

            di->shutdown_q4 = val;
            pr_info("SHUTDOWN_Q4 set shutdown_q4 = %d\n", di->shutdown_q4);
            if(di->ops->shutdown_q4)
            {
                di->ops->shutdown_q4(di->shutdown_q4);
                break;
            }
            else
            {
                return -EINVAL;
            }
        }
        /* shutdown watchdog, set 1 shutdown watchdog, set 0 to recovery*/
        case CHARGE_SYSFS_SHUTDOWN_WD:
        {
            if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 1))
                return -EINVAL;

            di->shutdown_wd = val;
            pr_info("SHUTDOWN_WD set shutdown_q4 = %d\n", di->shutdown_wd);
            if(di->ops->shutdown_wd)
            {
                di->ops->shutdown_wd(di->shutdown_wd);
                break;
            }
            else
            {
                return -EINVAL;
            }
        }
        /* set usb_current, set 0 recovery to normal */
        case CHARGE_SYSFS_USB_CURRENT:
        {
            if ((strict_strtol(buf, 10, &val) < 0) 
            || ((val > 1) && (val < IINLIM_100)) || (val > IINLIM_3000))
                return -EINVAL;

            di->usb_current = val;
            pr_info("USB_CURRENT set usb_current = %d\n", di->usb_current);
            if(di->ops->set_usb_current)
            {
                di->ops->set_usb_current(di->usb_current);
                break;
            }
            else
            {
                return -EINVAL;
            }
        }
        default:
        {
            pr_err("(%s)NODE ERR!!HAVE NO THIS NODE:(%d)\n",__func__,info->name);
            break;
        }
    }
    return count;
}

/**********************************************************
*  Function:       huawei_charger_load_battery_data
*  Discription:    load battery data from dtsi
*  Parameters:   di:charge_device_info
*  return value:  0-sucess or others-fail
**********************************************************/
static int huawei_charger_load_battery_data(struct charge_device_info *di)
{
    struct bms_battery_data batt_data;
    struct device_node *node;
    struct qpnp_vadc_result result;
    int rc;

    node = of_find_node_by_name(di->spmi->dev.of_node,
                                "qcom,battery-data");
    if (node)
    {
        memset(&batt_data, 0, sizeof(struct bms_battery_data));
        rc = qpnp_vadc_read(di->vadc_dev, LR_MUX2_BAT_ID, &result);
        if (rc)
        {
            pr_err("error reading batt id channel = %d, rc = %d\n",
                   LR_MUX2_BAT_ID, rc);
            return rc;
        }

        batt_data.max_voltage_uv = -1;
        batt_data.iterm_ua = -1;
        rc = of_batterydata_read_data(node,
                                      &batt_data, result.physical);
        if (rc)
        {
            pr_err("failed to read battery data: %d\n", rc);
            batt_data = palladium_1500_data;
        }

        if (batt_data.warm_bat_decidegc || batt_data.cool_bat_decidegc)
        {
            di->warm_bat_decidegc = batt_data.warm_bat_decidegc;
            di->warm_bat_chg_ma = batt_data.warm_bat_chg_ma;
            di->warm_bat_mv = batt_data.warm_bat_mv;

            di->cool_bat_decidegc = batt_data.cool_bat_decidegc;
            di->cool_bat_chg_ma = batt_data.cool_bat_chg_ma;
            di->cool_bat_mv = batt_data.cool_bat_mv;

            di->hot_bat_decidegc = batt_data.hot_bat_decidegc;
            di->cold_bat_decidegc = batt_data.cold_bat_decidegc;

            pr_info("use special temp-cv parameters\n");
        }

        if(TI_BQ24296_CHARGER == g_charger_core_para->charger_type_info.charger_index)
        {
            bq24296_temp_info.cold_bat_degree = di->cold_bat_decidegc;
            bq24296_temp_info.cool_bat_degree = di->cool_bat_decidegc;
            bq24296_temp_info.imaxma_cool_bat = di->cool_bat_chg_ma;
            bq24296_temp_info.vmaxmv_cool_bat = di->cool_bat_mv;
            bq24296_temp_info.warm_bat_degree = di->warm_bat_decidegc;
            bq24296_temp_info.imaxma_warm_bat = di->warm_bat_chg_ma;
            bq24296_temp_info.vmaxmv_warm_bat = di->warm_bat_mv;
            bq24296_temp_info.hot_bat_degree = di->hot_bat_decidegc;
        }

#ifdef CONFIG_HUAWEI_PMU_DSM
        temp_info.cold_bat_degree = di->cold_bat_decidegc;
        temp_info.cool_bat_degree = di->cool_bat_decidegc;
        temp_info.imaxma_cool_bat = di->cool_bat_chg_ma;
        temp_info.vmaxmv_cool_bat = di->cool_bat_mv;
        temp_info.warm_bat_degree = di->warm_bat_decidegc;
        temp_info.imaxma_warm_bat = di->warm_bat_chg_ma;
        temp_info.vmaxmv_warm_bat = di->warm_bat_mv;
        temp_info.hot_bat_degree = di->hot_bat_decidegc;
#endif

        pr_info("warm_bat_decidegc=%d "
                "warm_bat_chg_ma=%d "
                "warm_bat_mv=%d "
                "cool_bat_decidegc=%d "
                "cool_bat_chg_ma=%d "
                "cool_bat_mv=%d "
                "hot_bat_decidegc=%d "
                "cold_bat_decidegc=%d \n",
                di->warm_bat_decidegc,
                di->warm_bat_chg_ma,
                di->warm_bat_mv,
                di->cool_bat_decidegc,
                di->cool_bat_chg_ma,
                di->cool_bat_mv,
                di->hot_bat_decidegc,
                di->cold_bat_decidegc);
    }

    return 0;
}

#ifdef CONFIG_HUAWEI_PMU_DSM
/*int get_iin_runningtest(void)
{
    struct charge_device_info *di = NULL;

    di = g_charger_device_para;
    if(NULL == di)
    {
        pr_err("charge_device is not ready!\n");
        return -1;
    }
    return di->sysfs_data.iin_rt;
}

int get_factory_diag(void)
{
    struct charge_device_info *di = NULL;

    di = g_charger_device_para;
    if(NULL == di)
    {
        pr_err("charge_device is not ready!\n");
        return -1;
    }
    return di->chrg_config;
}*/

static int get_battery_id(struct charge_device_info *di)
{
    int rc = 0;
    struct qpnp_vadc_result results;

    rc = qpnp_vadc_read(di->vadc_dev, LR_MUX2_BAT_ID, &results);
    if (rc)
    {
        pr_err("Unable to read battery_id rc=%d\n", rc);
        return 0;
    }

    return results.physical;
}

static int get_vbus_uv(struct charge_device_info *di)
{
    int rc = 0;
    struct qpnp_vadc_result results;

    rc = qpnp_vadc_read(di->vadc_dev, USBIN, &results);
    if (rc)
    {
        pr_err("Unable to read vbus rc=%d\n", rc);
        return 0;
    }

    return results.physical;
}

static int get_vchg_uv(struct charge_device_info *di)
{
    int rc = 0;
    struct qpnp_vadc_result results;

    rc = qpnp_vadc_read(di->vadc_dev, VCHG_SNS, &results);
    if (rc)
    {
        pr_err("Unable to read vchg rc=%d\n", rc);
        return 0;
    }

    return results.physical;
}

static int get_vcoin_uv(struct charge_device_info *di)
{
    int rc = 0;
    struct qpnp_vadc_result results;

    rc = qpnp_vadc_read(di->vadc_dev, VCOIN, &results);
    if (rc)
    {
        pr_err("Unable to read vcoin rc=%d\n", rc);
        return 0;
    }

    return results.physical;
}

int dump_qpnp_adc_values(struct dsm_client *dclient)
{
    int batt_id, vusb_uv, vchg_uv, vcoin_uv;

    struct charge_device_info *di = NULL;

    di = g_charger_device_para;
    if(NULL == di)
    {
        pr_err("charge_device is not ready!\n");
        return -1;
    }

    batt_id = get_battery_id(di);
    vusb_uv = get_vbus_uv(di);
    vchg_uv = get_vchg_uv(di);
    vcoin_uv = get_vcoin_uv(di);

    dsm_client_record(dclient,
                      "ADC values: "
                      "batt_id=%d vusb=%d vchg=%d vcoin=%d\n",
                      batt_id, vusb_uv, vchg_uv, vcoin_uv);

    pr_info("ADC values:  "
            "batt_id=%d vusb=%d vchg=%d vcoin=%d\n",
            batt_id, vusb_uv, vchg_uv, vcoin_uv);
    return 0;
}

int is_usb_ovp(void)
{
    int rc = 0;
    u8 usbin_valid_rt_sts = 0;
    struct charge_device_info *di = NULL;

    di = g_charger_device_para;
    if(NULL == di)
    {
        pr_err("charge_device is not ready!\n");
        return 0;
    }
    rc = huawei_charger_pmic_reg_read(di, di->usb_chgpth_base + USB_PTH_STS_REG,
                                      &usbin_valid_rt_sts, 1);
    if (rc)
    {
        pr_err("spmi read failed: addr=%03X, rc=%d\n",
               di->usb_chgpth_base + USB_PTH_STS_REG, rc);
        return rc;
    }

    return ((usbin_valid_rt_sts & USB_VALID_MASK)== USB_VALID_OVP_VALUE) ? 1 : 0;

}

int vm_bms_dump_registers(struct dsm_client *dclient)
{
    int i = 0;
    int ret = 0;
    u8 reg_val = 0;

    struct charge_device_info *di = NULL;

    di = g_charger_device_para;
    if(NULL == di)
    {
        pr_err("charge_device is not ready!\n");
        return -1;
    }

    dsm_client_record(dclient, "[VM_BMS] regs:\n");
    pr_debug("[VM_BMS] regs:\n");
    for(i = 0; i < ARRAY_SIZE(VM_BMS); i++)
    {
        ret = huawei_charger_pmic_reg_read(di, di->bms_base + VM_BMS[i], &reg_val, 1);
        if (ret)
        {
            pr_err("Failed to read bms_base registers %d\n", ret);
            return -1;
        }
        dsm_client_record(dclient,
            "0x%x, 0x%x\n",
            di->bms_base+VM_BMS[i], reg_val);
        pr_debug("0x%x, 0x%x\n",
                 di->bms_base+VM_BMS[i], reg_val);
    }

    return 0;
}

int lbc_dump_registers(struct dsm_client *dclient)
{
    int i = 0;
    int rc = 0;
    u8 reg_val = 0;
    struct charge_device_info *di = NULL;

    di = g_charger_device_para;
    if(NULL == di)
    {
        pr_err("charge_device is not ready!\n");
        return -1;
    }

    /* First dump LBC_CHGR registers */
    dsm_client_record(dclient, "[LBC_CHGR] regs:\n");
    pr_debug("[LBC_CHGR] regs:\n");
    for(i = 0; i < ARRAY_SIZE(LBC_CHGR); i++)
    {
        rc = huawei_charger_pmic_reg_read(di, di->chgr_base + LBC_CHGR[i],
                       &reg_val, 1);
        if (rc)
        {
            pr_err("Failed to read chgr_base registers %d\n", rc);
            return -1;
        }
        dsm_client_record(dclient,
                  "0x%x, 0x%x\n",
                  di->chgr_base+LBC_CHGR[i], reg_val);
        pr_debug("0x%x, 0x%x\n",
                 di->chgr_base+LBC_CHGR[i], reg_val);
    }

    /*Then dump LBC_BAT_IF registers*/
    dsm_client_record(dclient, "[LBC_BAT_IF] regs:\n");
    pr_debug("[LBC_BAT_IF] regs:\n");
    for(i = 0; i < ARRAY_SIZE(LBC_BAT_IF); i++)
    {
        rc = huawei_charger_pmic_reg_read(di, di->bat_if_base + LBC_BAT_IF[i],
                       &reg_val, 1);
        if (rc)
        {
            pr_err("Failed to read bat_if_base registers %d\n", rc);
            return -1;
        }
        dsm_client_record(dclient,
                  "0x%x, 0x%x\n",
                  di->bat_if_base+LBC_BAT_IF[i], reg_val);
        pr_debug("0x%x, 0x%x\n",
                 di->bat_if_base+LBC_BAT_IF[i], reg_val);
    }

    /*Third dump LBC_USB registers*/
    dsm_client_record(dclient, "[LBC_USB] regs:\n");
    pr_debug("[LBC_USB] regs:\n");
    for(i = 0; i < ARRAY_SIZE(LBC_USB); i++)
    {
        rc = huawei_charger_pmic_reg_read(di, di->usb_chgpth_base + LBC_USB[i],
                       &reg_val, 1);
        if (rc)
        {
            pr_err("Failed to read usb_chgpth_base registers %d\n", rc);
            return -1;
        }
        dsm_client_record(dclient,
                  "0x%x, 0x%x\n",
                  di->usb_chgpth_base+LBC_USB[i], reg_val);
        pr_debug("0x%x, 0x%x\n",
                 di->usb_chgpth_base+LBC_USB[i], reg_val);
    }

    /*Fourth dump LBC_MISC registers*/
    dsm_client_record(dclient, "[LBC_MISC] regs:\n");
    pr_debug("[LBC_MISC] regs:\n");
    for(i = 0; i < ARRAY_SIZE(LBC_MISC); i++)
    {
        rc = huawei_charger_pmic_reg_read(di, di->misc_base + LBC_MISC[i],
                       &reg_val, 1);
        if (rc)
        {
            pr_err("Failed to read misc_base registers %d\n", rc);
            return -1;
        }
        dsm_client_record(dclient,
                  "0x%x, 0x%x\n",
                  di->misc_base+LBC_MISC[i], reg_val);
        pr_debug("0x%x, 0x%x\n",
                 di->misc_base+LBC_MISC[i], reg_val);
    }
    return 0;
}

struct dsm_charger_ops lbc_dsm_ops =
{
    .charger_dump_register = lbc_dump_registers,
};

struct dsm_bms_ops vm_bms_dsm_ops =
{
    .bms_dump_register = vm_bms_dump_registers,
};
#endif


/**********************************************************
*  Function:       charge_sysfs_create_group
*  Discription:    create the charge device sysfs group
*  Parameters:   di:charge_device_info
*  return value:  0-sucess or others-fail
**********************************************************/
static int charge_sysfs_create_group(struct charge_device_info *di)
{
    charge_sysfs_init_attrs();
    return sysfs_create_group(&di->dev->kobj, &charge_sysfs_attr_group);
}
/**********************************************************
*  Function:       charge_sysfs_remove_group
*  Discription:    remove the charge device sysfs group
*  Parameters:   di:charge_device_info
*  return value:  NULL
**********************************************************/
static inline void charge_sysfs_remove_group(struct charge_device_info *di)
{
    sysfs_remove_group(&di->dev->kobj, &charge_sysfs_attr_group);
}
#else
static int charge_sysfs_create_group(struct charge_device_info *di)
{
    return 0;
}
static inline void charge_sysfs_remove_group(struct charge_device_info *di) {}
#endif

/**********************************************************
*  Function:       charge_ops_register
*  Discription:    register the handler ops for chargerIC
*  Parameters:   ops:operations interface of charge device
*  return value:  0-sucess or others-fail
**********************************************************/
int charge_ops_register(struct charge_device_ops *ops)
{
    int ret = 0;

    if(ops != NULL)
    {
        g_ops = ops;
    }
    else
    {
        pr_err("charge ops register fail!\n");
        ret = -EPERM;
    }

    return ret;
}

/* Get/Set initial state of charger */
static void huawei_charger_initial_status(struct charge_device_info *di)
{
    di->usb_present = huawei_charger_vbus_is_exist(di);
    power_supply_set_present(di->usb_psy, di->usb_present);
    if(di->usb_present)
    {
        /*
        * Set USB psy online to avoid userspace from shutting down if battery
        * capacity is at zero and no chargers online.
        */
        power_supply_set_online(di->usb_psy, 1);
        wake_lock(&di->chg_wake_lock);
    }
}

static irqreturn_t huawei_charger_usbin_valid_irq_handler(int irq, void *data)
{
    struct charge_device_info *di = data;
    int usb_present;

    usb_present = huawei_charger_vbus_is_exist(di);
    pr_info("usbin-valid triggered: %d\n", usb_present);

    /* if otg is enabled, usbin_valid irq return directly */
    if(otg_enabled)
        return IRQ_HANDLED;
#ifdef CONFIG_HUAWEI_PMU_DSM
    usbin_valid_count_invoke();
#endif

    if (di->usb_present ^ usb_present)
    {
        di->usb_present = usb_present;
        if (!usb_present)
        {
            /* wake_lock for update the state of led,prevent phone enter sleep at once  */
            wake_lock_timeout(&di->led_wake_lock,msecs_to_jiffies(LED_CHECK_PERIOD_MS));
            wake_unlock(&di->chg_wake_lock);
        }
        else
        {
            wake_lock(&di->chg_wake_lock);
        }

        power_supply_set_present(di->usb_psy, di->usb_present);
    }
#ifdef CONFIG_HUAWEI_PMU_DSM
    /* if usb online status is abnormal*/
    /*dump pmic registers and adc values, and notify to dsm server */
    else
    {
        dsm_dump_work_invoke(DSM_CHARGER_ONLINE_ABNORMAL_ERROR_NO);
    }
#endif

    return IRQ_HANDLED;
}

static int huawei_lbc_request_irqs(struct charge_device_info *di)
{
    int rc = 0;

    rc = devm_request_irq(di->dev, di->irqs.irq, huawei_charger_usbin_valid_irq_handler,IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,"usbin_valid", di);
    if(rc)
    {
        pr_err("request usbin_valid failed!\n");
        return -1;
    }
    enable_irq_wake(di->irqs.irq);
    di->irqs.is_wake = true;
    return 0;
}

static int huawei_charger_reg_base_props(struct device_node* np, struct charge_device_info *di)
{
    int ret = 0;

    ret = of_property_read_u32(np, "chgr-base", &(di->chgr_base));
    if(ret)
    {
        pr_err("get chgr-base failed\n");
        return -EINVAL;
    }
    pr_debug("chgr-base = %d\n",di->chgr_base);

    ret = of_property_read_u32(np, "bat-if-base", &(di->bat_if_base));
    if(ret)
    {
        pr_err("get bat_if_base failed\n");
        return -EINVAL;
    }
    pr_debug("bat_if_base = %d\n",di->bat_if_base);

    ret = of_property_read_u32(np, "usb-chgpth-base", &(di->usb_chgpth_base));
    if(ret)
    {
        pr_err("get usb_chgpth_base failed\n");
        return -EINVAL;
    }
    pr_debug("usb_chgpth_base = %d\n",di->usb_chgpth_base);

    ret = of_property_read_u32(np, "misc-base", &(di->misc_base));
    if(ret)
    {
        pr_err("get usb_chgpth_base failed\n");
        return -EINVAL;
    }
    pr_debug("misc-base = %d\n",di->misc_base);

    ret = of_property_read_u32(np, "bms-base", &(di->bms_base));
    if(ret)
    {
        pr_err("get usb_chgpth_base failed\n");
        return -EINVAL;
    }
    pr_debug("bms-base = %d\n",di->bms_base);

    return 0;
}

static void free_charger_core_info(void)
{
    struct charger_core_info *di;
    di = g_charger_core_para;

    if(NULL != di)
    {
        kfree(di->charger_type_info.charger_type_string);
        di->charger_type_info.charger_type_string = NULL;
    }
    kfree(di);

    return ;
}

static void free_ops_info(void)
{
    struct charge_device_info *di;
    di = g_charger_device_para;

    if(NULL != di)
    {
        di->ops->dump_register = NULL;
        di->ops->set_in_thermal = NULL;
        di->ops->set_enable_charger = NULL;
        di->ops->set_runningtest = NULL;
    }

    di->ops = NULL;

    return ;
}

/**********************************************************
*  Function:       charge_probe
*  Discription:    chargre module probe
*  Parameters:   pdev:platform_device
*  return value:  0-sucess or others-fail
**********************************************************/
static int charge_probe(struct spmi_device *pdev)
{
    struct device_node* np = NULL;
    struct charge_device_info *di = NULL;
    struct class *power_class = NULL;
    struct power_supply *usb_psy;
    struct power_supply *batt_psy;
    struct power_supply *bms_psy;

    int ret = 0;
#ifdef CONFIG_HUAWEI_PMU_DSM
    struct dsm_charger_ops *chg_ops = NULL;
    struct dsm_bms_ops *bms_ops = NULL;
#endif
    pr_err("huawei charger probe!\n");

    usb_psy = power_supply_get_by_name("usb");
    if (!usb_psy)
    {
        pr_err("usb supply not found deferring probe\n");
        return -EPROBE_DEFER;
    }
    batt_psy = power_supply_get_by_name("battery");
    if (!batt_psy)
    {
        pr_err("batt supply not found deferring probe\n");
        return -EPROBE_DEFER;
    }

    bms_psy = power_supply_get_by_name("bms");
    if (!bms_psy)
    {
        pr_err("bms supply not found deferring probe\n");
        return -EPROBE_DEFER;
    }

    di = devm_kzalloc(&pdev->dev, sizeof(struct charge_device_info),GFP_KERNEL);
    if (!di)
    {
        pr_err("memory allocation failed.\n");
        return -ENOMEM;
    }

    di->core_data = (struct charge_core_data*)kzalloc(sizeof(struct charge_core_data), GFP_KERNEL);
    if(NULL == di->core_data)
    {
        pr_err("alloc di->core_data failed\n");
        goto charge_fail_0;
    }
	
    np = pdev->dev.of_node;
    di->dev = &pdev->dev;
    di->spmi = pdev;
    di->vadc_dev = g_charger_core_para->vadc_dev;
    di->irqs.irq = g_charger_core_para->irqs.irq;
    dev_set_drvdata(&pdev->dev, di);
    di->usb_psy = usb_psy;
    di->batt_psy = batt_psy;
    di->bms_psy = bms_psy;
    device_init_wakeup(&pdev->dev, 1);
	
    /* move to the end of probe */
    if(QCOM_LINEAR_CHARGER != g_charger_core_para->charger_type_info.charger_index)
    {
        huawei_charger_load_battery_data(di);
    }
    di->running_test_settled_status = POWER_SUPPLY_STATUS_CHARGING;
    di->ops = g_ops;
	
    if((NULL == di->ops->set_runningtest)||(NULL == di->ops->set_enable_charger)
            ||(NULL == di->ops->set_in_thermal)||(NULL == di->ops->dump_register)
            || (NULL == di->ops->get_register_head))
    {
        pr_err("charge ops is NULL!\n");
        goto charge_fail_1;
    }
	
    spin_lock_init(&di->hw_access_lock);
    wake_lock_init(&di->chg_wake_lock, WAKE_LOCK_SUSPEND, "charge_wakelock");
    wake_lock_init(&di->led_wake_lock, WAKE_LOCK_SUSPEND, "charge_led");

    huawei_charger_reg_base_props(np, di);

    /*delte unused code */
    di->core_data = &g_charger_core_para->data;
    di->chrg_config = 1;
    di->sysfs_data.iin_thl = di->core_data->iin_ac;
    di->sysfs_data.iin_rt = di->core_data->iin_ac;
    di->sysfs_data.shutdown_q4 = 0;
    di->sysfs_data.shutdown_wd = 0;
    di->sysfs_data.usb_current = 0;
  
    INIT_DELAYED_WORK(&di->nff_work, charge_nff_work); //For nff log, to do
    if(QCOM_LINEAR_CHARGER != g_charger_core_para->charger_type_info.charger_index)
    {
        huawei_charger_initial_status(di);
        ret = huawei_lbc_request_irqs(di);
        if (ret)
        {
            pr_err("can't create charge sysfs entries\n");
            goto charge_fail_2;
        }
    }

    ret = charge_sysfs_create_group(di);
    if (ret)
    {
        pr_err("can't create charge sysfs entries\n");
        free_ops_info();
        goto charge_fail_2;
    }

    power_class = class_create(THIS_MODULE, "hw_power");
    if(power_class)
    {
        if(charge_dev == NULL)
        {
            charge_dev = device_create(power_class, NULL, 0, NULL,"charger");
        }

        ret = sysfs_create_link(&charge_dev->kobj, &di->dev->kobj, "charge_data");
        if(ret)
        {
            pr_err("create link to charge_data fail.\n");
        }
    }

#ifdef CONFIG_HUAWEI_PMU_DSM
    if(QCOM_LINEAR_CHARGER == g_charger_core_para->charger_type_info.charger_index)
    {
        chg_ops = &lbc_dsm_ops;
        ret = dsm_charger_ops_register(chg_ops);
        if(ret)
        {
            pr_err("register dsm charge ops failed!\n");
            goto charge_fail_2;
        }
        bms_ops = &vm_bms_dsm_ops;
        ret = dsm_bms_ops_register(bms_ops);
        if(ret)
        {
            pr_err("register dsm charge ops failed!\n");
            goto charge_fail_2;
        }
    }
    if(TI_BQ24296_CHARGER == g_charger_core_para->charger_type_info.charger_index)
    {
        /* ATH and RIO use qcom vm_bms as fuel gauge, so register bms ops here*/
        bms_ops = &vm_bms_dsm_ops;
        ret = dsm_bms_ops_register(bms_ops);
        if(ret)
        {
            pr_err("register dsm charge ops failed!\n");
            goto charge_fail_2;
        }
    }
#endif

    g_charger_device_para = di;
    pr_info("huawei charger probe ok!\n");
    return 0;

charge_fail_2:
    wake_lock_destroy(&di->led_wake_lock);
    wake_lock_destroy(&di->chg_wake_lock);
charge_fail_1:
    if(NULL != di)
    {
        kfree(di->core_data);
        di->core_data = NULL;
    }
    dev_set_drvdata(&pdev->dev, NULL);

charge_fail_0:
    kfree(di);
    di = NULL;
    free_charger_core_info();

    return 0;
}
/**********************************************************
*  Function:       charge_remove
*  Discription:    charge module remove
*  Parameters:   pdev:platform_device
*  return value:  0-sucess or others-fail
**********************************************************/
static int charge_remove(struct spmi_device *pdev)
{
    struct charge_device_info *di;
    di = g_charger_device_para;
    di->dev = &pdev->dev;

    if (di == NULL)
    {
        pr_err("[%s]di is NULL!\n",__func__);
        return -ENODEV;
    }

    charge_sysfs_remove_group(di);
    cancel_delayed_work_sync(&di->nff_work);
    wake_lock_destroy(&di->led_wake_lock);
    wake_lock_destroy(&di->chg_wake_lock);

    if (NULL != di->ops)
    {
        di->ops = NULL;
        g_ops = NULL;
    }
    kfree(di);
    di = NULL;

    return 0;
}
/**********************************************************
*  Function:       charge_shutdown
*  Discription:    charge module shutdown
*  Parameters:   pdev:platform_device
*  return value:  NULL
**********************************************************/
static void charge_shutdown(struct spmi_device  *pdev)
{
    struct charge_device_info *di;
    di = g_charger_device_para;
    di->dev = &pdev->dev;

    return;
}

#ifdef CONFIG_PM
/**********************************************************
*  Function:       charge_suspend
*  Discription:    charge module suspend
*  Parameters:   pdev:platform_device
*                      state:unused
*  return value:  0-sucess or others-fail
**********************************************************/
static int charge_suspend(struct spmi_device *pdev, pm_message_t state)
{
    return 0;
}
/**********************************************************
*  Function:       charge_resume
*  Discription:    charge module resume
*  Parameters:   pdev:platform_device
*  return value:  0-sucess or others-fail
**********************************************************/
static int charge_resume(struct spmi_device *pdev)
{
    return 0;
}
#endif

static struct of_device_id charge_match_table[] =
{
    {
        .compatible = "huawei,charger",
        .data = NULL,
    },
    {
    },
};

static struct spmi_driver charge_driver =
{
    .probe = charge_probe,
    .remove = charge_remove,
    .suspend = charge_suspend,
    .resume = charge_resume,
    .shutdown = charge_shutdown,
    .driver =
    {
        .name = "huawei,charger",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(charge_match_table),
    },
};


/**********************************************************
*  Function:       charge_init
*  Discription:    charge module initialization
*  Parameters:   NULL



*  return value:  0-sucess or others-fail
**********************************************************/
static int __init charge_init(void)
{
    struct charger_core_info *di = NULL;

    pr_info("huawei_charger.c driver init!\n");

    di = charge_core_get_params();
    if(NULL == di)
    {
        pr_err("err:[%s]di is NULL!\n",__func__);
        kfree(di);
        di = NULL;
        return -EINVAL;
    }

    g_charger_core_para = di;

    spmi_driver_register(&charge_driver);

    return 0;
}
/**********************************************************
*  Function:       charge_exit
*  Discription:    charge module exit
*  Parameters:   NULL
*  return value:  NULL
**********************************************************/
static void __exit charge_exit(void)
{
    spmi_driver_unregister(&charge_driver);
}
late_initcall(charge_init);
module_exit(charge_exit);


MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("huawei charger module driver");
MODULE_AUTHOR("HUAWEI Inc");

