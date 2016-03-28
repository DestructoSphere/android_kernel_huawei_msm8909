/*
 * Copyright (C) huawei company
 *
 * This	program	is free	software; you can redistribute it and/or modify
 * it under	the	terms of the GNU General Public	License	version	2 as
 * published by	the	Free Software Foundation.
 */
/*==============================================================================
History

Problem NO.         Name        Time         Reason

==============================================================================*/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/sensors.h>
#include <linux/module.h>
#include <misc/app_info.h>
static char *sensor_binder_input[SENSOR_MAX] = {NULL};

int set_sensor_input(enum input_name name, const char *input_num)
{
	if (name >= SENSOR_MAX || input_num == NULL) {
		pr_err("set_sensor_input name =%d input_num = %s\n",
				name, input_num);
		return -EINVAL;
	}
	sensor_binder_input[name] = (char *)input_num;

	return 0;
}
EXPORT_SYMBOL(set_sensor_input);

static struct platform_device sensor_input_info = {
	.name = "huawei_sensor_input_info",
	.id = -1,
};

static ssize_t sensor_show_akm_input(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (dev == NULL) {
		pr_err("sensor_show_akm_input dev is null\n");
		return -EINVAL;
	}

	return snprintf(buf,512, "%s\n", sensor_binder_input[AKM]);

}
static DEVICE_ATTR(akm_input, S_IRUGO,
					sensor_show_akm_input, NULL);

static ssize_t sensor_show_acc_input(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (dev == NULL) {
		pr_err("sensor_show_acc_input dev is null\n");
		return -EINVAL;
	}
	return snprintf(buf, 512,"%s\n", sensor_binder_input[ACC]);
}
static DEVICE_ATTR(acc_input, S_IRUGO,
					sensor_show_acc_input, NULL);

static ssize_t sensor_show_gyro_input(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (dev == NULL) {
		pr_err("sensor_show_gyro_input dev is null\n");
		return -EINVAL;
	}
	return snprintf(buf,512,"%s\n", sensor_binder_input[GYRO]);
}
static DEVICE_ATTR(gyro_input, S_IRUGO,
					sensor_show_gyro_input, NULL);

static ssize_t sensor_show_ps_input(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (dev == NULL) {
		pr_err("sensor_show_ps_input dev is null\n");
		return -EINVAL;
	}
	return snprintf(buf,512,"%s\n", sensor_binder_input[PS]);

}
static DEVICE_ATTR(ps_input, S_IRUGO,
					sensor_show_ps_input, NULL);
static ssize_t sensor_show_hall_input(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (dev == NULL) {
		pr_err("sensor_show_hall_input dev is null\n");
		return -EINVAL;
	}
	return snprintf(buf,512, "%s\n", sensor_binder_input[HALL]);
}
static DEVICE_ATTR(hall_input, S_IRUGO,
					sensor_show_hall_input, NULL);
static ssize_t sensor_show_als_input(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (dev == NULL) {
		pr_err("sensor_show_als_input dev is null\n");
		return -EINVAL;
	}
	return snprintf(buf,512, "%s\n", sensor_binder_input[ALS]);
}
static DEVICE_ATTR(als_input, S_IRUGO,
					sensor_show_als_input, NULL);

/*--------------------------------------------------------*/
/*Next code is for kernel debug about probe result*/
static int sensors_list = 0;/* record sensors which probe success */

/* set sensors_list, called by sensor driver */
void set_sensors_list(int sensor)
{
	sensors_list |= sensor;
}

static int get_sensors_list(void)
{
	return sensors_list;
}

char *get_sensors_list_name(void)
{
	int sensors_list = G_SENSOR + L_SENSOR + P_SENSOR + M_SENSOR;
	char *list_name = NULL;
	sensors_list = get_sensors_list();

	switch(sensors_list)
	{
		case G_SENSOR + L_SENSOR + P_SENSOR + M_SENSOR + GY_SENSOR:
			list_name = "G_L_P_M_GY_SENSORS";
			break;

		case G_SENSOR + L_SENSOR + P_SENSOR + M_SENSOR:
			list_name = "G_L_P_M_SENSORS";
			break;

		case G_SENSOR + L_SENSOR + P_SENSOR:
			list_name = "G_L_P_SENSORS";
			break;

		case M_SENSOR + L_SENSOR + P_SENSOR:
			list_name = "M_L_P_SENSORS";
			break;

		case G_SENSOR:
			list_name = "G_SENSORS";
			break;

		case M_SENSOR:
			list_name = "M_SENSORS";
			 break;

		case G_SENSOR + M_SENSOR + GY_SENSOR:
			list_name = "G_M_GY_SENSORS";
			break;

		case G_SENSOR + M_SENSOR:
			list_name = "G_M_SENSORS";
			break;

		case L_SENSOR + P_SENSOR:
			list_name = "L_P_SENSORS";
			break;

		case NONE_SENSOR:
			list_name = "NONE_SENSORS";
			break;

		default:
			list_name = "G_L_P_M_SENSORS";
			break;
	}
	printk("name is %s\n",list_name);
	return list_name;
}
static ssize_t sensor_probe_list_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	char *list_name = NULL;
	if (dev == NULL) {
		pr_err("sensor_probe_list_show dev is null\n");
		return -EINVAL;
	}
	list_name = get_sensors_list_name();
	return snprintf(buf, 512,"%s\n", list_name);
}
static DEVICE_ATTR(sensor_probe_list, S_IRUGO,
					sensor_probe_list_show, NULL);
bool sensorDT_mode=false;//sensor DT mode
int Gsensor_data_count=0;//G-sensor upload data times
int als_data_count=0;    //ALS sensor upload data times
int ps_data_count=0;     //ps sensor upload data times
int compass_data_count=0;//compass sensor upload data times


static ssize_t store_sensor_DT_test(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
       unsigned long val = 0;
	if (strict_strtoul(buf, 10, &val)) {
		pr_err("store_sensor_DT_test val, val = %ld\n", val);
		return -EINVAL;
	}
	pr_err("store_sensor_DT_test val=%ld\n",val);
	if(1 == val)
	{
		sensorDT_mode=true;
		Gsensor_data_count=0;
		als_data_count=0;
		ps_data_count=0;
		compass_data_count=0;
	}
	else
	{
		sensorDT_mode=false;
		Gsensor_data_count=0;
		als_data_count=0;
		ps_data_count=0;
		compass_data_count=0;
	}
	return count;
}
static ssize_t show_sensor_DT_test(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (dev == NULL) {
		pr_err("als_chip info is null\n");
		return -EINVAL;
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", sensorDT_mode);
}

static DEVICE_ATTR(sensor_test, 0660,show_sensor_DT_test, store_sensor_DT_test);
static ssize_t show_gsensor_test_result(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (dev == NULL) {
		pr_err("als_chip info is null\n");
		return -EINVAL;
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", Gsensor_data_count);
}
static DEVICE_ATTR(acc_read_data, S_IRUGO,show_gsensor_test_result, NULL);

static ssize_t show_compass_test_result(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (dev == NULL) {
		pr_err("als_chip info is null\n");
		return -EINVAL;
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", compass_data_count);
}
static DEVICE_ATTR(mag_read_data, S_IRUGO,show_compass_test_result, NULL);

static ssize_t show_als_test_result(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (dev == NULL) {
		pr_err("als_chip info is null\n");
		return -EINVAL;
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", als_data_count);
}
static DEVICE_ATTR(als_read_data, S_IRUGO,show_als_test_result, NULL);
static ssize_t show_ps_test_result(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (dev == NULL) {
		pr_err("als_chip info is null\n");
		return -EINVAL;
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", ps_data_count);
}
static DEVICE_ATTR(ps_read_data, S_IRUGO,show_ps_test_result, NULL);
static struct attribute *sensor_input_attributes[] = {
	&dev_attr_ps_input.attr,
	&dev_attr_als_input.attr,
	&dev_attr_acc_input.attr,
	&dev_attr_akm_input.attr,
	&dev_attr_gyro_input.attr,
	&dev_attr_hall_input.attr,
	&dev_attr_sensor_probe_list.attr,
	&dev_attr_sensor_test.attr,
	&dev_attr_als_read_data.attr,
	&dev_attr_mag_read_data.attr,
	&dev_attr_acc_read_data.attr,
	&dev_attr_ps_read_data.attr,
	NULL
};
static const struct attribute_group sensor_input = {
	.attrs = sensor_input_attributes,
};
static int __init sensor_input_info_init(void)
{
	int ret = 0;
	char *list_name = get_sensors_list_name();
	ret = platform_device_register(&sensor_input_info);
	if (ret) {
		pr_err("%s: platform_device_register failed, ret:%d.\n",
				__func__, ret);
		goto REGISTER_ERR;
	}

	ret = sysfs_create_group(&sensor_input_info.dev.kobj, &sensor_input);
	if (ret) {
		pr_err("sensor_input_info_init sysfs_create_group error ret =%d", ret);
		goto SYSFS_CREATE_CGOUP_ERR;
	}
	app_info_set("Sensor_list",list_name);
	return 0;
SYSFS_CREATE_CGOUP_ERR:
	platform_device_unregister(&sensor_input_info);
REGISTER_ERR:
	return ret;

}

late_initcall(sensor_input_info_init);
MODULE_DESCRIPTION("sensor input info");
MODULE_LICENSE("GPL");
