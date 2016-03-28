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
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/charger_core.h>
#include <linux/of_batterydata.h>
#include <linux/platform_device.h>

struct charger_ic_type charger_type_table[] =
{
    {"Linear_charger",     QCOM_LINEAR_CHARGER},
    {"Ti_bq24152",     TI_BQ24152_CHARGER},
    {"Ti_bq24192",     TI_BQ24192_CHARGER},
    {"Ti_bq24296",     TI_BQ24296_CHARGER},
    {"Max77819",     MAX77819_CHARGER},
    {"",     UNKNOW_CHARGER},
};

struct charger_core_info *g_charger_core_info = NULL;

static int get_charge_ic_type(struct charger_core_info *di,const char *name)
{
    int i = 0;

    while(UNKNOW_CHARGER != charger_type_table[i].charger_index)
    {
        if (0 == strcmp(charger_type_table[i].charger_type_string, name) )
        {
            di->charger_type_info.charger_index = charger_type_table[i].charger_index;
            di->charger_type_info.charger_type_string = kzalloc(MAX_CHARGER_TYPE_STRING_LEN, GFP_KERNEL);
            if (!di)
            {
                pr_err("huawei_charger_info is NULL!\n");
                return -EINVAL;
            }

            memset(di->charger_type_info.charger_type_string, 0, MAX_CHARGER_TYPE_STRING_LEN);
            memcpy(di->charger_type_info.charger_type_string, \
                   charger_type_table[di->charger_type_info.charger_index].charger_type_string,MAX_CHARGER_TYPE_STRING_LEN);
            break;
        }

        i++;
    }

    if(UNKNOW_CHARGER == di->charger_type_info.charger_index)
    {
        pr_err("unknow charger is detected!\n");
        return -EINVAL;
    } 
    return 0;
}

static int huawei_charger_read_dt_props(struct device_node* np, struct charger_core_info *di)
{
    int ret = 0;

    ret = of_property_read_u32(np, "iin_ac", &(di->data.iin_ac));
    if(ret)
    {
        di->data.iin_ac = DEFAULT_AC_IIN_CURRENT;
        pr_err("get iin_ac failed\n");
    }

    pr_info("iin_ac = %d\n",di->data.iin_ac);

    return 0;
}


struct charger_core_info *charge_core_get_params(void)
{
    struct charger_core_info *di = g_charger_core_info;

    if((NULL == di) || (NULL == di->charger_type_info.charger_type_string))
    {
        pr_err("get charger core info failed!\n");
        return NULL;
    }

    return di;
}

static int charge_core_probe(struct platform_device *pdev)
{
    struct device_node* np = NULL;
    struct charger_core_info *di = NULL;
    const  char *charger_type_string;
    int    rc = 0;

    pr_info("charge_core_probe probe init!\n");

    np = pdev->dev.of_node;

    di = kzalloc(sizeof(*di), GFP_KERNEL);
    if (!di)
    {
        pr_err("huawei_charger_info is NULL!\n");
        return -ENOMEM;
    }

    memset(di, 0, sizeof(struct charger_core_info));

    g_charger_core_info = di;

    platform_set_drvdata(pdev, di);

    rc = of_property_read_string(np, "charger_type", &charger_type_string);
    if (rc)
    {
        pr_err("Failed to get charger type from dts ,rc=%d\n", rc);
        goto core_fail_0;
    }

    rc = get_charge_ic_type(di, charger_type_string);
    if (rc)
    {
        pr_err("Failed to find charger type info,rc=%d\n", rc);
        goto core_fail_1;
    }

    huawei_charger_read_dt_props(np, di);

    pr_info("charge_core_probe probe ok!\n");

    return 0;

core_fail_1:
    if(NULL != di)
    {
        kfree(di->charger_type_info.charger_type_string);
        di->charger_type_info.charger_type_string = NULL;
    }
core_fail_0:
    platform_set_drvdata(pdev, NULL);
    kfree(di);
    di = NULL;

    return 0;
}

static int charge_core_remove(struct platform_device *pdev)
{
    struct charger_core_info *di = platform_get_drvdata(pdev);

    if (NULL == di)
    {
        pr_err("[%s]di is NULL!\n",__func__);
        return -EINVAL;
    }

    kfree(di->charger_type_info.charger_type_string);
    di->charger_type_info.charger_type_string = NULL;
    g_charger_core_info = NULL;
    kfree(di);
    di = NULL;

    return 0;
}


static struct of_device_id charge_core_match_table[] =
{
    {
        .compatible = "huawei,charger_core",
        .data = NULL,
    },
    {
    },
};

static struct platform_driver charge_core_driver =
{
    .probe = charge_core_probe,
    .remove = charge_core_remove,
    .driver =
    {
        .name = "huawei,charger_core",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(charge_core_match_table),
    },
};

static int __init charge_core_init(void)
{
    return platform_driver_register(&charge_core_driver);
}

static void __exit charge_core_exit(void)
{
    platform_driver_unregister(&charge_core_driver);
}

core_initcall_sync(charge_core_init);
module_exit(charge_core_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("huawei charger module driver");
MODULE_AUTHOR("HUAWEI Inc");
