/*
 * Copyright (C) huawei company
 *
 * This	program	is free	software; you can redistribute it and/or modify
 * it under	the	terms of the GNU General Public	License	version	2 as
 * published by	the	Free Software Foundation.
 */
#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/slab.h> 
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/hw_dev_dec.h>
#include <linux/hw_dev_array.h>
#include <linux/err.h>
#define DEV_NAME_LENGTH 20

static uint64_t dev_flag_long = 0;

/* set device flag */
void set_hw_dev_flag( int dev_id )
{
	if( (dev_id >= 0) && ( dev_id < DEV_I2C_MAX ) )
	{
		dev_flag_long = dev_flag_long | (1 << dev_id);
	}
	else
	{
		pr_err("Device %s  is unknown in list \n",hw_dec_device_array[dev_id].devices_name);
	}
}
static ssize_t dev_flag_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (dev == NULL) {
		pr_err("dev_flag dev is null\n");
		return -EINVAL;
	}
	return sprintf((char *)buf, "%llx\n",dev_flag_long);
}
static DEVICE_ATTR(dev_flag, S_IRUGO,
					dev_flag_show, NULL);

static struct attribute *dev_dct_attributes[] = {
	&dev_attr_dev_flag.attr,
	NULL
};

static const struct attribute_group dev_dct_attr_group= {
	.attrs = dev_dct_attributes,
};

/* set default value for each device */
static int hw_set_default_dev_flag(struct device_node *core_node)
{
	u32 value = 0;
	int dev_id;
	char i2c_dev_name[DEV_NAME_LENGTH] = {0};
	for(dev_id = 0; dev_id < DEV_I2C_MAX;dev_id ++){
		snprintf(i2c_dev_name,DEV_NAME_LENGTH,"i2c,%s",hw_dec_device_array[dev_id].devices_name);
		if(!of_property_read_u32(core_node,i2c_dev_name,&value)){
			if(!value)
				dev_flag_long = dev_flag_long | (1 << dev_id);
		}
		else
			return -1;
	}
	return 0;
}

static int dev_dct_probe(struct platform_device *pdev)
{
	int ret = 0;
	ret = sysfs_create_group(&pdev->dev.kobj, &dev_dct_attr_group);
	if(ret)
		pr_err("%s:sysfs_create_group failed\n",__func__);
	ret = hw_set_default_dev_flag(pdev->dev.of_node);
	if(ret)
		pr_err("%s: set default hw dev detect flag failed.\n",__func__);

	return 0;
}

static struct of_device_id hw_i2c_detect_of_match[] = 
{
	{ .compatible = "huawei,hw-dev-detect", },
	{ }
};

static struct platform_driver dev_dct_driver = {
	.driver	= {
		.name	= "hw-dev-detect",
		.owner  = THIS_MODULE,
		.of_match_table = hw_i2c_detect_of_match,
	},
	.probe		= dev_dct_probe,
	.remove		= NULL,
};

static int __init hw_dev_dct_init(void)
{
	return platform_driver_register(&dev_dct_driver);
}

static void __exit hw_dev_dct_exit(void)
{
	platform_driver_unregister(&dev_dct_driver);
}
/* priority is 7s */
late_initcall_sync(hw_dev_dct_init);
module_exit(hw_dev_dct_exit);
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:dev_dct");
