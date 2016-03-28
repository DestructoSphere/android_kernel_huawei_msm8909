
/************************************************************
*
* Copyright (C), 1988-2015, Huawei Tech. Co., Ltd.
* FileName: huawei_dsm_charger.c
* Author: jiangfei(00270021)       Version : 0.1      Date:  2015-03-17
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*  Description:    .c adatper file for charger radar
*  Version:
*  Function List:
*  History:
*  <author>  <time>   <version >   <desc>
***********************************************************/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <asm/irq.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/syscalls.h>
#include <linux/power_supply.h>
#include <linux/rtc.h>
#include <linux/power/huawei_dsm_charger.h>
#include <linux/power/bq24296m_charger.h>
#include <linux/power/huawei_charger.h>

struct hw_dsm_charger_info
{
    struct device        *dev;
    struct delayed_work   check_charging_batt_status_work;
    struct delayed_work		dump_work;
    struct work_struct	usbin_valid_count_work;
    struct power_supply *batt_psy;
    struct power_supply *usb_psy;
    struct mutex		dsm_dump_lock;
    struct mutex		dsm_soc_lock;
    struct hw_batt_temp_info *temp_ctrl;
    struct dsm_charger_ops *chg_ops;
    struct dsm_bms_ops *bms_ops;
    struct dsm_err_info *dsm_err;
    bool    charging_disabled;
    bool    factory_diag;
    int    error_no;
    /* remove unused member*/
};

struct hw_dsm_charger_info *dsm_info = NULL;
struct charger_core_info *g_charger_core = NULL;
struct dsm_charger_ops *charger_ops = NULL;
struct dsm_bms_ops *bms_ops = NULL;

/*bms dsm client definition */
struct dsm_dev dsm_bms =
{
    .name = "dsm_bms", // dsm client name
    .fops = NULL,
    .buff_size = 4096, // buffer size
};
struct dsm_client *bms_dclient = NULL;
EXPORT_SYMBOL(dsm_bms);

/*charger dsm client definition */
struct dsm_dev dsm_charger =
{
    .name = "dsm_charger", // dsm client name
    .fops = NULL,
    .buff_size = 4096, // buffer size
};
struct dsm_client *charger_dclient = NULL;
EXPORT_SYMBOL(charger_dclient);

struct hw_batt_temp_info temp_info =
{
    .cold_bat_degree = 0, //default cold bat degree: 0 degree
    .cool_bat_degree = 100, //default cool bat degree: 10 degree
    .imaxma_cool_bat = 600, //default cool bat charge current: 600mA
    .vmaxmv_cool_bat = 4350, //default cool bat max voltage: 4350mV
    .warm_bat_degree = 420, //default warm bat degree: 42 degree
    .imaxma_warm_bat = 800, //default warm bat charge current: 600mA
    .vmaxmv_warm_bat = 4100, //default warm bat max voltage: 4100mV
    .hot_bat_degree = 520, //default hot bat degree: 52 degree
};

/* radar error count struct, same error report 3 time*/
struct dsm_err_info dsm_err;

static int usbin_irq_invoke_flag = 0;
static unsigned long usbin_start_tm_sec = 0;
static bool dsm_factory_flag = false;

int dsm_charger_ops_register(struct dsm_charger_ops *ops)
{
    int ret = 0;

    if(ops != NULL)
    {
        charger_ops = ops;
    }
    else
    {
        pr_err("dsm charge ops register fail!\n");
        ret = -EPERM;
    }

    return ret;
}

int dsm_bms_ops_register(struct dsm_bms_ops *ops)
{
    int ret = 0;

    if(ops != NULL)
    {
        bms_ops = ops;
    }
    else
    {
        pr_err("dsm bms ops register fail!\n");
        ret = -EPERM;
    }

    return ret;
}

static void print_basic_info_before_dump(struct dsm_client *dclient, const int type)
{
    int error_type;

    error_type = type;
    switch(error_type)
    {
    case DSM_BMS_NORMAL_SOC_CHANGE_MUCH:
        dsm_client_record(dclient,
                  "soc changed a lot in one minute "
                  "the difference is more than 5 percent\n");
        pr_info("soc changed a lot in one minute "
                "the difference is more than 5 percent\n");
        break;

    case DSM_BMS_VOL_SOC_DISMATCH_1:
        dsm_client_record(dclient,
                  "battery voltage is over 3.7V, but soc is "
                  "below 2 percent, not match\n");
        pr_info("battery voltage is over 3.7V, but soc is "
                "below 2 percent, not match\n");
        break;

    case DSM_BMS_VOL_SOC_DISMATCH_2:
        dsm_client_record(dclient,
                  "battery voltage is over 3.6V, but soc is "
                  "0 percent, not match\n");
        pr_info("battery voltage is over 3.6V, but soc is "
                "0 percent, not match\n");
        break;

    case DSM_BMS_VOL_SOC_DISMATCH_3:
        dsm_client_record(dclient,
                  "battery voltage is over 4.32V, but  "
                  "soc is below 95 percent\n");
        pr_info("battery voltage is over 4.32V, but  "
                "soc is below 95 percent\n");
        break;

    case DSM_VM_BMS_VOL_SOC_DISMATCH_4:
        dsm_client_record(dclient,
                  "battery voltage is too low when "
                  "soc is 2 percent\n");
        pr_info("battery voltage is too low when "
                "soc is 2 percent\n");
        break;

    case DSM_BMS_SOC_CHANGE_PLUG_INOUT:
        dsm_client_record(dclient,
                  "soc changed more than 1 percent when plug in or out charger\n");
        pr_info("soc changed more than 1 percent when plug in or out charger\n");
        break;

    case DSM_BMS_POWON_SOC_CHANGE_MUCH:
        dsm_client_record(dclient,
                  "poweron soc is different with last "
                   "shutdown soc, the difference is more than 10 percent\n");
        pr_info("poweron soc is different with last "
                "shutdown soc, the difference is more than 10 percent\n");
        break;

    case DSM_NOT_CHARGE_WHEN_ALLOWED:
        dsm_client_record(dclient,
                  "cannot charging when allowed charging\n");
        pr_info("cannot charging when allowed charging\n");
        break;

    case DSM_CHG_OVP_ERROR_NO:
        dsm_client_record(dclient,
                   "CHG_OVP happend!\n");
        pr_info("CHG_OVP happend!\n");
        break;

    case DSM_BATT_PRES_ERROR_NO:
        dsm_client_record(dclient,
                  "battery is absent!\n");
        pr_info("battery is absent!\n");
        break;

    case DSM_WARM_CURRENT_LIMIT_FAIL:
        dsm_client_record(dclient,
                  "set battery warm charge current failed\n");
        pr_info("set battery warm charge current failed\n");
        break;

    case DSM_COOL_CURRENT_LIMIT_FAIL:
        dsm_client_record(dclient,
                   "set battery cool charge current failed\n");
        pr_info("set battery cool charge current failed\n");
        break;

    case DSM_FULL_WHEN_CHARGER_ABSENT:
        dsm_client_record(dclient,
                  "battery status is full when charger is absent\n");
        pr_info("battery status is full when charger is absent\n");
        break;

    case DSM_CHARGER_ONLINE_ABNORMAL_ERROR_NO:
        dsm_client_record(dclient,
                  "charger online status abnormal!\n");
        pr_info("charger online status abnormal!\n");
        break;

    case DSM_BATT_VOL_OVER_4400:
        dsm_client_record(dclient,
                  "battery voltage is over 4.4V\n");
        pr_info("battery voltage is over 4.4V\n");
        break;

    case DSM_FAKE_FULL:
        dsm_client_record(dclient,
                   "report charging full when actual soc is below 95 percent\n");
        pr_info("report charging full when actual soc is below 95 percent\n");
        break;

    case DSM_ABNORMAL_CHARGE_STATUS:
        dsm_client_record(dclient,
                  "charging status is charging while charger is not online\n");
        pr_info("charging status is charging while charger is not online\n");
        break;

    case DSM_BATT_VOL_TOO_LOW:
        dsm_client_record(dclient,
                   "battery voltage is too low(below 2.5V)\n");
        pr_info("battery voltage is too low(below 2.5V)\n");
        break;

    case DSM_STILL_CHARGE_WHEN_HOT:
        dsm_client_record(dclient,
                   "still charge when battery is hot\n");
        pr_info("still charge when battery is hot\n");
        break;

    case DSM_STILL_CHARGE_WHEN_COLD:
        dsm_client_record(dclient,
                   "still charge when battery is cold\n");
        pr_info("still charge when battery is cold\n");
        break;

    case DSM_STILL_CHARGE_WHEN_SET_DISCHARGE:
        dsm_client_record(dclient,
                   "still charge when we set discharge\n");
        pr_info("still charge when we set discharge\n");
        break;

    case DSM_STILL_CHARGE_WHEN_VOL_OVER_4350:
        dsm_client_record(dclient,
                   "still charge when we battery voltage reach or over 4.35V\n");
        pr_info("still charge when we battery voltage reach or over 4.35V\n");
        break;

    case DSM_HEATH_OVERHEAT:
        dsm_client_record(dclient,
                   "battery health is overheat\n");
        pr_info("battery health is overheat\n");
        break;

    case DSM_BATTERY_ID_UNKNOW:
        dsm_client_record(dclient,
                   "battery id is unknow\n");
        pr_info("battery id is unknow\n");
        break;

    case DSM_BATT_TEMP_JUMP:
        dsm_client_record(dclient,
                   "battery temperature change more than 5 degree in short time\n");
        pr_info("battery temperature change more than 5 degree in short time\n");
        break;

    case DSM_BATT_TEMP_BELOW_0:
        dsm_client_record(dclient,
                   "battery temperature is below 0 degree\n");
        pr_info("battery temperature is below 0 degree\n");
        break;

    case DSM_BATT_TEMP_OVER_60:
        dsm_client_record(dclient,
                   "battery temperature is over 60 degree\n");
        pr_info("battery temperature is over 60 degree\n");
        break;

    case DSM_NOT_CHARGING_WHEN_HOT:
        dsm_client_record(dclient,
                   "battery is hot, not charging\n");
        pr_info("battery is hot, not charging\n");
        break;

    case DSM_NOT_CHARGING_WHEN_COLD:
        dsm_client_record(dclient,
                   "battery is cold, not charging\n");
        pr_info("battery is cold, not charging\n");
        break;

    case DSM_CHARGE_DISABLED_IN_USER_VERSION:
        dsm_client_record(dclient,
                   "set runningtest discharging in user version, it is not allowed\n");
        pr_info("set charging_enabled as 0 in user version, it is not allowed\n");
        break;

    case DSM_SET_FACTORY_DIAG_IN_USER_VERSION:
        dsm_client_record(dclient,
                   "set factory_diag as discharging in user version, it is not allowed\n");
        pr_info("set factory_diag as 1 in user version, it is not allowed\n");
        break;

    case DSM_USBIN_IRQ_INVOKE_TOO_QUICK:
        dsm_client_record(dclient,
                   "usbin vaild irq invoke too quickly, more than 20 times in 30s\n");
        pr_info("usbin vaild irq invoke too quickly, more than 20 times in 30s\n");
        break;

    case DSM_BQ_WATCHDOG_FAULT:
        dsm_client_record(dclient,
                   "BQ: WATCHDOG_FAULT\n");
        pr_info("BQ: WATCHDOG_FAULT\n");
        break;

    case DSM_BQ_OTG_FAULT:
        dsm_client_record(dclient,
                   "BQ: VBUS OVP, or overload or battery is too low when OTG enabled\n");
        pr_info("BQ: VBUS OVP, or overload or battery is too low when OTG enabled\n");
        break;

    case DSM_BQ_POWER_SUPPLY_OVP:
        dsm_client_record(dclient,
                   "BQ: POWER_SUPPLY_OVP\n");
        pr_info("BQ: POWER_SUPPLY_OVP\n");
        break;

    case DSM_BQ_THERMAL_SHUTDOWM:
        dsm_client_record(dclient,
                   "BQ: THERMAL_SHUTDOWM\n");
        pr_info("BQ: THERMAL_SHUTDOWM\n");
        break;

    case DSM_BQ_CHRG_TIMER_EXPIRED:
        dsm_client_record(dclient,
                   "BQ: TIMER_EXPIRED\n");
        pr_info("BQ: TIMER_EXPIRED\n");
        break;

    case DSM_BQ_BAT_FAULT_OVP:
        dsm_client_record(dclient,
                   "BQ: battery OVP\n");
        pr_info("BQ: battery OVP\n");
        break;

    case DSM_BQ_ENABLE_OTG_FAIL:
        dsm_client_record(dclient,
                   "BQ: enable OTG fail\n");
        pr_info("BQ: enable OTG fail\n");
        break;

    case DSM_BQ_CHARGE_WHEN_OTGEN:
        dsm_client_record(dclient,
                   "BQ: still show charge status when OTG is enabled\n");
        pr_info("BQ: still show charge status when OTG is enabled\n");
        break;

    case DSM_LINEAR_USB_OVERTEMP:
        dsm_client_record(dclient,
                   "linear charger: USB overtemp!\n");
        pr_info("linear charger: USB overtemp!\n");
        break;

    case DSM_LINEAR_CHG_FAILED:
        dsm_client_record(dclient,
                   "linear charger: charge failed!\n");
        pr_info("linear charger: charge failed!\n");
        break;

    case DSM_LINEAR_CHG_OVERCURRENT:
        dsm_client_record(dclient,
                   "linear charger: battery charge current is exceeded 1.5A\n");
        pr_info("linear charger: battery charge current is exceeded 1.5A\n");
        break;

    case DSM_LINEAR_BAT_OVERCURRENT:
        dsm_client_record(dclient,
                   "linear charger: battery discharge current is exceeded 5A\n");
        pr_info("linear charger: battery discharge current is exceeded 5A\n");
        break;

    default:
        break;
    }
}

/* dump charger ic, bms registers and some adc values, and notify to dsm*/
static int dump_registers_and_adc(struct hw_dsm_charger_info *di, struct dsm_client *dclient, int type)
{
    int ret = 0;
    int vbat_uv = 0, batt_temp = 250, current_ma = 0;
    union power_supply_propval val = {0,};

    if( NULL == dclient )
    {
        pr_err("%s: there is no dclient!\n", __func__);
        return -1;
    }

    mutex_lock(&di->dsm_dump_lock);

    /* try to get permission to use the buffer */
    if(dsm_client_ocuppy(dclient))
    {
        /* buffer is busy */
        pr_err("%s: buffer is busy!\n", __func__);
        mutex_unlock(&di->dsm_dump_lock);
        return -1;
    }

    print_basic_info_before_dump(dclient, type);

    ret = di->chg_ops->charger_dump_register(dclient); /* dump charger registers*/
    if(ret)
    {
        mutex_unlock(&di->dsm_dump_lock);
        return -1;
    }

    if ((DSM_BMS_NORMAL_SOC_CHANGE_MUCH <= type)
        && (DSM_BMS_POWON_SOC_CHANGE_MUCH >= type)) /* only care bms abnormal*/
    {
        ret = di->bms_ops->bms_dump_register(dclient); /* if bms abnormal, dump bms registers*/
        if(ret)
       {
            mutex_unlock(&di->dsm_dump_lock);
            return -1;
       }
    }

    /*save battery vbat, current, temp values and so on*/
    di->batt_psy->get_property(di->batt_psy,POWER_SUPPLY_PROP_TEMP,&val);
    batt_temp = val.intval;
    di->batt_psy->get_property(di->batt_psy,POWER_SUPPLY_PROP_VOLTAGE_NOW,&val);
    vbat_uv = val.intval;
    di->batt_psy->get_property(di->batt_psy,POWER_SUPPLY_PROP_CURRENT_NOW,&val);
    current_ma = val.intval;

    dsm_client_record(dclient,
                      "ADC values: vbat=%d current=%d batt_temp=%d\n",
                      vbat_uv, current_ma, batt_temp);

    pr_info("ADC values: vbat=%d current=%d batt_temp=%d\n",
            vbat_uv, current_ma, batt_temp);

    /* dump qpnp adc values, such as usb_in, vcoin, battery id voltage*/
    ret = dump_qpnp_adc_values(dclient);
    if(ret)
    {
        mutex_unlock(&di->dsm_dump_lock);
        return -1;
    }

    dsm_client_notify(dclient, type);
    mutex_unlock(&di->dsm_dump_lock);
    return 0;
}

/* interface for be called to dump and notify*/
int dsm_dump_log(struct dsm_client *dclient, int err_no)
{
    struct hw_dsm_charger_info *di = NULL;

    di = dsm_info;
    if( NULL == di )
    {
        pr_err("%s: dsm_charger is not ready!\n", __func__);
        return -1;
    }
    if((PMU_ERR_NO_MIN > err_no) 
    || (PMU_ERR_NO_MAX < err_no))
    {
        pr_err("%s: err_no is exceed available number, do nothing!\n", __func__);
        return -1;
    }
    di->dsm_err->err_no = err_no - DSM_BMS_NORMAL_SOC_CHANGE_MUCH;
    if(di->dsm_err->count[di->dsm_err->err_no]++ < REPORT_MAX)
    {
        dump_registers_and_adc(di, dclient, err_no);
    }
    else
    {
        di->dsm_err->count[di->dsm_err->err_no] = REPORT_MAX;
    }
    return 0;
}
EXPORT_SYMBOL(dsm_dump_log);

static int get_current_time(unsigned long *now_tm_sec)
{
    struct rtc_time tm;
    struct rtc_device *rtc;
    int rc;

    rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
    if (rtc == NULL)
    {
        pr_err("%s: unable to open rtc device (%s)\n",
               __FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
        return -EINVAL;
    }

    rc = rtc_read_time(rtc, &tm);
    if (rc)
    {
        pr_err("Error reading rtc device (%s) : %d\n",
               CONFIG_RTC_HCTOSYS_DEVICE, rc);
        goto close_time;
    }

    rc = rtc_valid_tm(&tm);
    if (rc)
    {
        pr_err("Invalid RTC time (%s): %d\n",
               CONFIG_RTC_HCTOSYS_DEVICE, rc);
        goto close_time;
    }
    rtc_tm_to_time(&tm, now_tm_sec);

close_time:
    rtc_class_close(rtc);
    return rc;
}

/* this func is used to count usbin_valid irq invoke times*/
static void usbin_valid_count_work_func(struct work_struct *work)
{
    static int usbin_irq_invoke_count = 0;
    static int start_flag = 0;
    static unsigned long start_tm_sec = 0;
    unsigned long now_tm_sec = 0;

    struct hw_dsm_charger_info	*chip =
        container_of(work, struct hw_dsm_charger_info, usbin_valid_count_work);

    if(!usbin_irq_invoke_flag)
    {
        mutex_lock(&chip->dsm_soc_lock);
        get_current_time(&usbin_start_tm_sec);
        usbin_irq_invoke_flag = 1;
        mutex_unlock(&chip->dsm_soc_lock);
    }

    /* if usbin_valid irq invokes more than 20 times in 30 seconds*/
    /*dump pmic registers and adc values, and notify to dsm server */
    if(!start_flag)
    {
        get_current_time(&start_tm_sec);
        start_flag = 1;
    }

    get_current_time(&now_tm_sec);
    if(HALF_MINUTE >= (now_tm_sec - start_tm_sec))
    {
        usbin_irq_invoke_count++;
        pr_debug("usbin_valid_count_work_func is invoked! usbin_irq_invoke_count is %d\n",usbin_irq_invoke_count);
        if(MAX_COUNT <= usbin_irq_invoke_count)
        {
            usbin_irq_invoke_count = 0;
            dsm_dump_log(charger_dclient, DSM_USBIN_IRQ_INVOKE_TOO_QUICK);
        }
    }
    else
    {
        start_flag = 0;
        start_tm_sec = 0;
        usbin_irq_invoke_count = 0;
    }
}

/* if soc jump more than 1 percent when usbin irq invoke(plug in or out)*/
static void monitor_soc_jump_when_usbinirq_invoke(struct hw_dsm_charger_info *chip)
{
    unsigned long now_tm_sec = 0;
    int cur_capacity;
    static int prev_capacity;
    union power_supply_propval val = {0,};

    if(!usbin_irq_invoke_flag)
    {
        if (chip->batt_psy && chip->batt_psy->get_property)
        {
            chip->batt_psy->get_property(chip->batt_psy, POWER_SUPPLY_PROP_CAPACITY,&val);
            prev_capacity = val.intval;
        }

    }

    /* if soc jump more than 2 percent in 30 seconds after usbin irq invoke*/
    /*dump pmic registers and adc values, and notify to dsm server */
    get_current_time(&now_tm_sec);
    if((HALF_MINUTE >= (now_tm_sec - usbin_start_tm_sec))
            && usbin_start_tm_sec)
    {
        if (chip->batt_psy && chip->batt_psy->get_property)
        {
            chip->batt_psy->get_property(chip->batt_psy, POWER_SUPPLY_PROP_CAPACITY,&val);
            cur_capacity = val.intval;
        }
        if(abs(cur_capacity - prev_capacity) > SOC_JUMP_USBIN)
        {
            pr_info("soc changed more than 2 percent in 30s after usbin irq invoke "
                    "cur_capacity=%d prev_capacity=%d\n",
                    cur_capacity,
                    prev_capacity);
            dsm_dump_log(charger_dclient, DSM_BMS_SOC_CHANGE_PLUG_INOUT);
        }
    }
    else
    {
        mutex_lock(&chip->dsm_soc_lock);
        usbin_irq_invoke_flag = 0;
        usbin_start_tm_sec = 0;
        mutex_unlock(&chip->dsm_soc_lock);
    }

}

void usbin_valid_count_invoke(void)
{
    if (!dsm_info)
    {
        pr_info("dsm_charger device not init,do nothing!\n");
        return;
    }
    schedule_work(&dsm_info->usbin_valid_count_work);
}

/* this work is lanch by batt_pres abnormal, and online status error and others*/
static void dsm_dump_work(struct work_struct *work)
{
    struct hw_dsm_charger_info	*chip =
        container_of(work, struct hw_dsm_charger_info, dump_work.work);

    pr_info("dsm_dump_work is invoked! error type is %d\n",chip->error_no);
    dsm_dump_log(charger_dclient, chip->error_no);
}

void dsm_dump_work_invoke(int error_no)
{
    if (!dsm_info)
    {
        pr_info("hwcharger device not init,do nothing!\n");
        return;
    }
    dsm_info->error_no = error_no;
    schedule_delayed_work(&dsm_info->dump_work,
                          msecs_to_jiffies(DUMP_WORK_DELAY_MS));
}

static void check_charging_batt_status_work(struct work_struct *work)
{
    int vbat_uv, batt_temp, bat_present, cur_status;
    int health, batt_level, usb_present, input_current=0;
    static int cannot_charge_count = 0;
    static int start_dismatch_detect = 0;
    static int hot_charging_count = 0, cold_charging_count = 0;
    static int warm_exceed_limit_count = 0,  cool_exceed_limit_count = 0;
    static int previous_temp = INIT_TEMP;
    static int status_not_match_count = 0;
    int mode = 0;
    int current_max = 0, current_ma = 0;
    int voltage_regulation = 0;
    static unsigned long previous_tm_sec = 0;
    unsigned long now_tm_sec = 0;
    union power_supply_propval val = {0,};
    struct hw_dsm_charger_info *chip =
        container_of(work, struct hw_dsm_charger_info, check_charging_batt_status_work.work);

    usb_present = is_usb_chg_exist();
    mode = is_vbus_otg_regulator_enabled(); //whether vbus otg is enabled
    /* mask these code temporarily, will recovery them for some time */
    /*iin_rt = get_iin_runningtest();
    if(IIN_RT == iin_rt)
    {
        chip->charging_disabled = true;
    }
    else
    {
        chip->charging_disabled = false;
    }
    chip->factory_diag = get_factory_diag();*/

    if(chip->batt_psy && chip->batt_psy->get_property)
    {
        chip->batt_psy->get_property(chip->batt_psy,POWER_SUPPLY_PROP_CAPACITY,&val);
        batt_level = val.intval;
        chip->batt_psy->get_property(chip->batt_psy,POWER_SUPPLY_PROP_PRESENT,&val);
        bat_present = val.intval;
        chip->batt_psy->get_property(chip->batt_psy,POWER_SUPPLY_PROP_TEMP,&val);
        batt_temp = val.intval;
        chip->batt_psy->get_property(chip->batt_psy,POWER_SUPPLY_PROP_VOLTAGE_NOW,&val);
        vbat_uv = val.intval;
        chip->batt_psy->get_property(chip->batt_psy,POWER_SUPPLY_PROP_CURRENT_NOW,&val);
        current_ma = val.intval;
        chip->batt_psy->get_property(chip->batt_psy,POWER_SUPPLY_PROP_HEALTH,&val);
        health = val.intval;
        chip->batt_psy->get_property(chip->batt_psy,POWER_SUPPLY_PROP_CHARGING_ENABLED,&val);
        chip->charging_disabled = !(val.intval);//get running_test flag
        chip->batt_psy->get_property(chip->batt_psy,POWER_SUPPLY_PROP_FACTORY_DIAG,&val);
        chip->factory_diag = val.intval; //get factory_diag
    }

    if(TI_BQ24296_CHARGER == g_charger_core->charger_type_info.charger_index)
    {
        /* bq24296 should use STATUS reg08 to determine whether charging or not */
        cur_status = get_bq_charge_status();
    }
    else
    {
        chip->batt_psy->get_property(chip->batt_psy,POWER_SUPPLY_PROP_STATUS,&val);
        cur_status = val.intval;
    }

    if(QCOM_LINEAR_CHARGER == g_charger_core->charger_type_info.charger_index)
    /* input current max node only for qcom linear charger*/
    {
        chip->batt_psy->get_property(chip->batt_psy,POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,&val);
        input_current = val.intval;
    }

    /* if usb ovp happens*/
    /*dump pmic registers and adc values, and notify to dsm server */
    if(is_usb_ovp())
    {
        pr_info("USB OVP!\n");
        dsm_dump_log(charger_dclient, DSM_CHG_OVP_ERROR_NO);
    }

    /* get /sys/class/power_supply/usb/current_max value */
    chip->usb_psy->get_property(chip->usb_psy,
                                POWER_SUPPLY_PROP_CURRENT_MAX, &val);
    current_max = val.intval / 1000;
    /*if all the charging conditions are avaliable, but it still cannot charge, */
    /*we report the error and dump the registers values and ADC values  */
    if((((chip->temp_ctrl->hot_bat_degree - TEMP_BUFFER) > batt_temp)
        && (vbat_uv <= (chip->temp_ctrl->vmaxmv_warm_bat - WARM_VOL_BUFFER))
        && (batt_temp >= (chip->temp_ctrl->warm_bat_degree - TEMP_BUFFER)))
        || (((chip->temp_ctrl->cold_bat_degree + TEMP_BUFFER) < batt_temp)
        && (batt_temp < (chip->temp_ctrl->warm_bat_degree - TEMP_BUFFER))))
    {

        if((POWER_SUPPLY_STATUS_DISCHARGING == cur_status)
           && (BATT_FULL_LEVEL != batt_level) && usb_present
           && (current_max > 2) && (0 == mode)
           && (!chip->charging_disabled) && (chip->factory_diag)
           && bat_present && (POWER_SUPPLY_STATUS_FULL != cur_status))
        {

            if(cannot_charge_count++ < NOT_CHARGE_COUNT)
            {
                pr_info("cannot charge when allowed, count is %d\n", cannot_charge_count);
            }
            else
            {
                cannot_charge_count = 0;
                pr_info("cannot charge when allowed!\n");
                dsm_dump_log(charger_dclient, DSM_NOT_CHARGE_WHEN_ALLOWED);
            }
        }
        else
        {
            cannot_charge_count = 0;
        }
    }

    if(!bat_present)  /* battery absent */
    {
        pr_info("battery is absent!\n");
        dsm_dump_log(charger_dclient, DSM_BATT_PRES_ERROR_NO);
    }
    else
    {
        if(!usb_present)  /* usb absent, discharging */
        {
            if(START_DISMATCH_COUNT <= start_dismatch_detect++)
            {
                start_dismatch_detect = START_DISMATCH_COUNT;
                if((VOL_THR1 <= vbat_uv) && (SOC_ZERO == batt_level))
                {
                    dsm_dump_log(charger_dclient, DSM_BMS_VOL_SOC_DISMATCH_1);
                }

                if((VOL_THR2 <= vbat_uv) && (SOC_ZERO != batt_level)
                        && (SOC_THR1 >= batt_level))
                {
                    dsm_dump_log(charger_dclient, DSM_BMS_VOL_SOC_DISMATCH_2);
                }

                if((VOL_HIGH <= vbat_uv) && (SOC_ZERO != batt_level)
                        && (SOC_HIGH >= batt_level))
                {
                    dsm_dump_log(charger_dclient, DSM_BMS_VOL_SOC_DISMATCH_3);
                }

                if((VOL_TOO_LOW >= vbat_uv) && (SOC_THR1 == batt_level))
                {
                    dsm_dump_log(charger_dclient, DSM_VM_BMS_VOL_SOC_DISMATCH_4);
                }
            }

            if(HIGH_VOL <= vbat_uv)
            {
                dsm_dump_log(charger_dclient, DSM_BATT_VOL_OVER_4400);
            }

		if(TI_BQ24296_CHARGER == g_charger_core->charger_type_info.charger_index)
		{
			if(POWER_SUPPLY_STATUS_CHARGING == cur_status)
            		{
                		dsm_dump_log(charger_dclient, DSM_ABNORMAL_CHARGE_STATUS);
           			}

            		if(POWER_SUPPLY_STATUS_FULL == cur_status)
            		{
               		dsm_dump_log(charger_dclient, DSM_FULL_WHEN_CHARGER_ABSENT);
            		}
		}
		else if(QCOM_LINEAR_CHARGER == g_charger_core->charger_type_info.charger_index)
		{
			if(STATUS_NOT_MATCH_COUNT <= status_not_match_count++){
				status_not_match_count = 0;
				
				if(POWER_SUPPLY_STATUS_CHARGING == cur_status)
		            	{
		               	 dsm_dump_log(charger_dclient, DSM_ABNORMAL_CHARGE_STATUS);
		            	}

		            	if(POWER_SUPPLY_STATUS_FULL == cur_status)
		            	{
		                	dsm_dump_log(charger_dclient, DSM_FULL_WHEN_CHARGER_ABSENT);
		            	}
			}
		}
        }

        /* usb present and not otg mode*/
        if(usb_present && (0 == mode))
        {
            if((POWER_SUPPLY_STATUS_FULL == cur_status)
                    && (SOC_HIGH_THR >= batt_level))
            {
                dsm_dump_log(charger_dclient, DSM_FAKE_FULL);
            }
            /* move the code to other place */
            /* chip->charging_disabled is true*/
            if(chip->charging_disabled)
            {
                if(POWER_SUPPLY_STATUS_CHARGING == cur_status)
                {
                    dsm_dump_log(charger_dclient, DSM_STILL_CHARGE_WHEN_SET_DISCHARGE);
                }
                /* as C199s use charging_enabled node to control flash light */
                /* so C199s ignore this monitor point, this point keeps for others*/
                if(!dsm_factory_flag
                        && (MAX77819_CHARGER != g_charger_core->charger_type_info.charger_index))
                {
                    dsm_dump_log(charger_dclient, DSM_CHARGE_DISABLED_IN_USER_VERSION);
                }
            }
            else
            {
                if( !dsm_factory_flag && !chip->factory_diag)
                {
                    dsm_dump_log(charger_dclient, DSM_SET_FACTORY_DIAG_IN_USER_VERSION);
                }
                voltage_regulation = VOL_REGULATION_MAX;
                if((voltage_regulation <= vbat_uv)&&(POWER_SUPPLY_STATUS_CHARGING == cur_status))
                {
                    dsm_dump_log(charger_dclient, DSM_STILL_CHARGE_WHEN_VOL_OVER_4350);
                }

                if(((chip->temp_ctrl->hot_bat_degree + TEMP_BUFFER) < batt_temp)
                        &&((POWER_SUPPLY_STATUS_DISCHARGING == cur_status)
                           ||(POWER_SUPPLY_STATUS_NOT_CHARGING == cur_status)))
                {
                    dsm_dump_log(charger_dclient, DSM_NOT_CHARGING_WHEN_HOT);
                }

                if(((chip->temp_ctrl->cold_bat_degree - TEMP_BUFFER) > batt_temp)
                        &&((POWER_SUPPLY_STATUS_DISCHARGING == cur_status)
                           ||(POWER_SUPPLY_STATUS_NOT_CHARGING == cur_status)))
                {
                    dsm_dump_log(charger_dclient, DSM_NOT_CHARGING_WHEN_COLD);
                }

                if(((chip->temp_ctrl->hot_bat_degree + TEMP_BUFFER) < batt_temp)
                        &&(POWER_SUPPLY_STATUS_CHARGING == cur_status))
                {
                    if(hot_charging_count++ < DSM_COUNT)
                    {
                        pr_info("still charge when battery is hot, count is %d\n", hot_charging_count);
                    }
                    else
                    {
                        hot_charging_count = 0;
                        dsm_dump_log(charger_dclient, DSM_STILL_CHARGE_WHEN_HOT);
                    }
                }
                else
                {
                    hot_charging_count = 0;
                }

                if(((chip->temp_ctrl->cold_bat_degree - TEMP_BUFFER) > batt_temp)
                        &&(POWER_SUPPLY_STATUS_CHARGING == cur_status))
                {
                    if(cold_charging_count++ < DSM_COUNT)
                    {
                        pr_info("still charge when battery is cold, count is %d\n", cold_charging_count);
                    }
                    else
                    {
                        cold_charging_count = 0;
                        dsm_dump_log(charger_dclient, DSM_STILL_CHARGE_WHEN_COLD);
                    }
                }
                else
                {
                    cold_charging_count = 0;
                }

                if(((chip->temp_ctrl->warm_bat_degree + TEMP_BUFFER) < batt_temp)
                        && (QCOM_LINEAR_CHARGER == g_charger_core->charger_type_info.charger_index)
                        &&(WARM_COOL_CURRENT_LIMIT < input_current))
                {
                    if(warm_exceed_limit_count++ < DSM_COUNT)
                    {
                        pr_info("current is over warm current limit when warm, count is %d\n", warm_exceed_limit_count);
                    }
                    else
                    {
                        warm_exceed_limit_count = 0;
                        dsm_dump_log(charger_dclient, DSM_WARM_CURRENT_LIMIT_FAIL);
                    }
                }
                else
                {
                    warm_exceed_limit_count = 0;
                }

                if(((chip->temp_ctrl->cool_bat_degree - TEMP_BUFFER) > batt_temp)
                        && (QCOM_LINEAR_CHARGER == g_charger_core->charger_type_info.charger_index)
                        &&(WARM_COOL_CURRENT_LIMIT < input_current))
                {
                    if(cool_exceed_limit_count++ < DSM_COUNT)
                    {
                        pr_info("current is over cool current limit when cool, count is %d\n", cool_exceed_limit_count);
                    }
                    else
                    {
                        cool_exceed_limit_count = 0;
                        dsm_dump_log(charger_dclient, DSM_COOL_CURRENT_LIMIT_FAIL);
                    }
                }
                else
                {
                    cool_exceed_limit_count = 0;
                }
            }
        }

        /* monitor whether soc jump more than 2 percent when plug in out charger(in 30 seconds)*/
        monitor_soc_jump_when_usbinirq_invoke(chip);

        /* only care 20 to 40 temperature zone in centigrade, */
        /* if temp jumps in this zone in 30 seconds, notify to dsm server*/
        get_current_time(&now_tm_sec);
        if((abs(previous_temp - batt_temp) >= TEMP_DELTA)
            && (TEMP_LOWER_THR <= batt_temp)
            && (TEMP_UPPER_THR >= batt_temp)
            && (TEMP_LOWER_THR <= previous_temp)
            && (TEMP_UPPER_THR >= previous_temp)
            && (INIT_TEMP != previous_temp)
            && (HALF_MINUTE >=(now_tm_sec -previous_tm_sec)))
        {
                dsm_dump_log(charger_dclient, DSM_BATT_TEMP_JUMP);
        }
        previous_temp = batt_temp;
        previous_tm_sec = now_tm_sec;

        if(LOW_VOL >= vbat_uv)
        {
            dsm_dump_log(charger_dclient, DSM_BATT_VOL_TOO_LOW);
        }

        if(POWER_SUPPLY_HEALTH_OVERHEAT == health)
        {
            dsm_dump_log(charger_dclient, DSM_HEATH_OVERHEAT);
        }

        if(HOT_TEMP < batt_temp)
        {
            dsm_dump_log(charger_dclient, DSM_BATT_TEMP_OVER_60);
        }

        if(LOW_TEMP > batt_temp)
        {
            dsm_dump_log(charger_dclient, DSM_BATT_TEMP_BELOW_0);
        }

        if(QCOM_LINEAR_CHARGER == g_charger_core->charger_type_info.charger_index)
        {
            if(CHARGE_CURRENT_MAX > current_ma)
            {
                dsm_dump_log(charger_dclient, DSM_LINEAR_CHG_OVERCURRENT);
            }

            if(OVER_CURRENT < current_ma)
            {
                dsm_dump_log(charger_dclient, DSM_LINEAR_BAT_OVERCURRENT);
            }
        }

        if(!chip->charging_disabled && mode
                && (POWER_SUPPLY_STATUS_CHARGING == cur_status))
        {
            dsm_dump_log(charger_dclient, DSM_BQ_CHARGE_WHEN_OTGEN);
        }
    }

    schedule_delayed_work(&chip->check_charging_batt_status_work,
                          msecs_to_jiffies(CHECKING_TIME));
}

static int __init early_parse_factory_flag(char * p)
{
    if(p)
    {
        if(!strcmp(p,"factory"))
        {
            dsm_factory_flag = true;
        }
    }
    return 0;
}
early_param("androidboot.huawei_swtype",early_parse_factory_flag);

static int huawei_dsm_charger_probe(struct platform_device *pdev)
{
    struct hw_dsm_charger_info *chip;
    struct power_supply *usb_psy;
    struct power_supply *batt_psy;

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

    pr_info("%s: ------entry.\n", __func__);

    chip = kzalloc(sizeof(*chip), GFP_KERNEL);
    if (!chip)
    {
        pr_err("alloc mem failed!\n");
        return -ENOMEM;
    }
    chip->usb_psy = usb_psy;
    chip->batt_psy = batt_psy;
    chip->dev = &pdev->dev;
    dev_set_drvdata(chip->dev, chip);

    chip->temp_ctrl = &temp_info;
    chip->dsm_err = &dsm_err;
    chip->chg_ops = charger_ops;
    chip->bms_ops = bms_ops;
    if((NULL == chip->chg_ops) || (NULL == chip->bms_ops)
    ||(NULL == chip->chg_ops->charger_dump_register)
    ||(NULL == chip->bms_ops->bms_dump_register))
    {
        pr_err("charge ops is NULL!\n");
        goto dsm_fail;
    }
    mutex_init(&chip->dsm_dump_lock);
    mutex_init(&chip->dsm_soc_lock);

    dsm_info = chip;

    if (!bms_dclient)
    {
        bms_dclient = dsm_register_client(&dsm_bms);
    }

    if (!charger_dclient)
    {
        charger_dclient = dsm_register_client(&dsm_charger);
    }

    INIT_WORK(&chip->usbin_valid_count_work, usbin_valid_count_work_func);
    INIT_DELAYED_WORK(&chip->check_charging_batt_status_work,
                      check_charging_batt_status_work);
    INIT_DELAYED_WORK(&chip->dump_work, dsm_dump_work);

    schedule_delayed_work(&chip->check_charging_batt_status_work,
                          msecs_to_jiffies(DELAY_TIME));

    pr_info("%s: ------OK.\n", __func__);
    return 0;
dsm_fail:
    dev_set_drvdata(&pdev->dev, NULL);
    kfree(chip);
    chip = NULL;
    return 0;
}

static int  huawei_dsm_charger_remove(struct platform_device *pdev)
{
    struct hw_dsm_charger_info *chip = dev_get_drvdata(&pdev->dev);
    cancel_delayed_work_sync(&chip->check_charging_batt_status_work);
    mutex_destroy(&chip->dsm_dump_lock);
    mutex_destroy(&chip->dsm_soc_lock);
    return 0;
}

static int dsm_charger_suspend(struct device *dev)
{
    struct hw_dsm_charger_info *chip = dev_get_drvdata(dev);

    cancel_delayed_work_sync(&chip->check_charging_batt_status_work);

    return 0;
}
static int dsm_charger_resume(struct device *dev)
{
    struct hw_dsm_charger_info *chip = dev_get_drvdata(dev);

    schedule_delayed_work(&chip->check_charging_batt_status_work,
                          msecs_to_jiffies(0));

    return 0;
}

static const struct dev_pm_ops hw_dsm_pm_ops =
{
    .suspend	= dsm_charger_suspend,
    .resume	=dsm_charger_resume,
};


static struct of_device_id platform_hw_charger_ids[] =
{
    {
        .compatible = "huawei,dsm_charger",
        .data = NULL,
    },
    {
    },
};

static struct platform_driver huawei_dsm_charger_driver =
{
    .probe        = huawei_dsm_charger_probe,
    .remove        = huawei_dsm_charger_remove,
    .driver        = {
        .name    = "dsm_charger",
        .owner    = THIS_MODULE,
        .of_match_table = of_match_ptr(platform_hw_charger_ids),
        .pm		= &hw_dsm_pm_ops,
    },
};

static int __init huawei_dsm_charger_init(void)
{
    int ret = 0;
    struct charger_core_info *di = NULL;

    di = charge_core_get_params();
    if(NULL == di)
    {
        pr_err("err:[%s]di is NULL!\n",__func__);
        kfree(di);
        di = NULL;
        return -EINVAL;
    }

    g_charger_core = di;
    ret = platform_driver_register(&huawei_dsm_charger_driver);

    return ret;
}
late_initcall_sync(huawei_dsm_charger_init);

static void __exit huawei_dsm_charger_exit(void)
{
    platform_driver_unregister(&huawei_dsm_charger_driver);
}

module_exit(huawei_dsm_charger_exit);

MODULE_AUTHOR("jiangfei<fei.jiang@huawei.com>");
MODULE_DESCRIPTION("hw dsm charger driver");
MODULE_LICENSE("GPL");