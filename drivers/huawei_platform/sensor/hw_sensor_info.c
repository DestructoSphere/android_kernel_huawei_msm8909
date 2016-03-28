/*
 *
 * Copyright (C) 2013 HUAWEI, Inc.
 *File Name: kernel/drivers/misc/hw_sensor_info.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/of_device.h>

#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <huawei_platform/sensor/hw_sensor_info.h>

struct sensor_info{
	struct platform_driver sensor_info_drv;
	struct platform_device *sensor_info_dev;
};

static struct device_node *this_node = NULL;
static const char *product_name = NULL;
const char * get_sensor_info_of_product_name(void)
{
	if(NULL != product_name)
		return product_name ;
	else
		return "error";
}
int sensor_info_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct sensor_info *info = NULL ;
	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (info == NULL) {
		ret = -ENOMEM;
		goto error1;
	}
	info->sensor_info_dev = pdev;
	this_node = info->sensor_info_dev->dev.of_node;
	ret = of_property_read_string(this_node, "product_name", &product_name);
	if(ret)
	{
		pr_err("%s ,sensor info get failed,ret = %d\n",__func__,ret);
		goto error2;
	}
	pr_info("%s ,huawei product name is  %s\n",__func__,product_name);
	return 0;

error2:
	pr_info("%s ,error2,huawei product name is  %s\n",__func__,product_name);
	kfree(info);
error1:
	pr_info("%s ,error1,huawei product name is  %s\n",__func__,product_name);
	return ret;
}

static struct of_device_id sensor_info_match_table[] = {
	{
		.compatible = "huawei,hw_sensor_info",
	},
};

static struct sensor_info sensor_info_instance = {
	.sensor_info_drv = {
		.probe = sensor_info_probe,
		.remove	= NULL,
		.driver = {
			.name = "huawei_sensor_info",
			.owner = THIS_MODULE,
			.of_match_table = sensor_info_match_table,
		},
	},
	.sensor_info_dev = 	NULL,
};


static int __init hw_sensor_info_init(void)
{
	int err = 0;

	err = platform_driver_register(&sensor_info_instance.sensor_info_drv);
	if (err){
		printk("sensor_info_drv regiset error %d\n", err);
	}

	return err;
}

static void __exit hw_sensor_info_exit(void)
{
	platform_driver_unregister(&sensor_info_instance.sensor_info_drv);
}

MODULE_AUTHOR("huawei");
MODULE_DESCRIPTION("name infomation for sensor");
MODULE_LICENSE("GPL");

module_init(hw_sensor_info_init);
module_exit(hw_sensor_info_exit);
