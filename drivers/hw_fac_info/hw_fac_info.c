/*
 * Copyright (C) huawei company
 *
 * This    program    is free    software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public    License    version    2 as
 * published by    the Free Software Foundation.
 */
 
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/slab.h> 
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/of.h>
#include <linux/config_interface.h>
#include <hw_fac_info.h>
int hw_fac_info_debug_mask = FAC_INFO;

#define NAME_LEN 32

char product_name[NAME_LEN];
char hardware_ver[NAME_LEN];
char software_ver[NAME_LEN];


static ssize_t product_name_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf)
{
    if(strlen(product_name) == 0)
    {
        return sprintf(buf, "%s\n", "unkmown product name");
    }
    
    return sprintf(buf, "%s\n", product_name);
}

static struct kobj_attribute product_name_attribute =__ATTR(product_name, 0400, product_name_show, NULL);

static ssize_t hardware_ver_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf)
{
    if(strlen(hardware_ver) == 0)
    {
        return sprintf(buf, "%s\n", "unkmown hardware version");
    }
    
    return sprintf(buf, "%s\n", hardware_ver);
}

static struct kobj_attribute hardware_ver_attribute =__ATTR(hardware_version, 0400, hardware_ver_show, NULL);

static ssize_t software_ver_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf)
{
    if(strlen(software_ver) == 0)
    {
        return sprintf(buf, "%s\n", "unkmown software version");
    }
    
    return sprintf(buf, "%s\n", software_ver);
}

static struct kobj_attribute software_ver_attribute =__ATTR(software_version, 0400, software_ver_show, NULL);


static struct attribute *fac_attrs[] = {
    &product_name_attribute.attr,
    &hardware_ver_attribute.attr,
    &software_ver_attribute.attr,
    NULL,    /* need to NULL terminate the list of attributes */
};

static int  fac_info_probe(struct platform_device *pdev)
{
         int ret = 0;
        ret = get_product_name(product_name,NAME_LEN);
        if(ret)
        {
            fac_log_err("%s:get product name fail!\n", __func__);
        }
        ret = get_hardware_ver(hardware_ver,NAME_LEN);
        if(ret)
        {
            fac_log_err("%s:get hardware version fail!\n", __func__);
        }
        ret = get_software_ver(software_ver,NAME_LEN);
        if(ret)
        {
            fac_log_err("%s:get software version fail!\n", __func__);
        }
		ret = set_sbl1_ver_to_appinfo();
        if(ret)
        {
            fac_log_err("%s:get sbl1 version fail!\n", __func__);
        }
		ret = set_hardware_ver_to_appinfo();
        if(ret)
        {
            fac_log_err("%s:get hardware version fail!\n", __func__);
        }
		ret = set_appboot_ver_to_appinfo();
        if(ret)
        {
            fac_log_err("%s:get appboot version fail!\n", __func__);
        }		
		ret = set_product_name_to_appinfo();
        if(ret)
        {
            fac_log_err("%s:get product name fail!\n", __func__);
        }
		ret = set_software_ver_to_appinfo();
        if(ret)
        {
            fac_log_err("%s:get software ver fail!\n", __func__);
        }
        return ret;
}

struct attribute_group fac_group = {
                                 .name ="hw_fac_info",
                                 .attrs = fac_attrs,
                                                   }; 
static const struct attribute_group *groups[]={
                              &fac_group,
                           NULL,
    
};
static struct of_device_id fac_info_match_table[] = {
    { .compatible = "huawei,hw_fac_info",},
    { },
};

static struct platform_driver fac_info_driver = {
    .driver = {
                    .name  = "hw_fac_info",
                    .owner  = THIS_MODULE,
                    .groups = groups,
                    .of_match_table = fac_info_match_table,
                },

       .probe = fac_info_probe,
       .remove = NULL,
};

static int __init  fac_info_init(void)
{
    return platform_driver_register(&fac_info_driver);
}

static void __exit fac_info_exit(void)
{
    platform_driver_unregister(&fac_info_driver);
}

module_init(fac_info_init);
module_exit(fac_info_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("for getting Factory Info");
MODULE_AUTHOR("zhaorenjie <zhaorenjie@huawei.com>");
