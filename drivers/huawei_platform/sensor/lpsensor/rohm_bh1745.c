/*huawei kernel driver for rgb_bh1745*/
/*
 * rgb_bh1745.c - Linux kernel modules for ambient light + proximity sensor
 *
 * Copyright (C) 2012 Lee Kai Koon <kai-koon.lee@avagotech.com>
 * Copyright (C) 2012 Avago Technologies
 * Copyright (C) 2013 LGE Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <huawei_platform/sensor/rohm_bh1745.h>
#include <huawei_platform/sensor/hw_sensor_info.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/sensors.h>
#include <linux/wakelock.h>

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif

#include <misc/app_info.h>
#include <linux/debugfs.h>

#define PARSE_DTSI_NUMBER                               (22)
#define JUDEG_COEFF   (1000)

static unsigned short bh1745_atime[6] = {160, 320, 640, 1280, 2560, 5120};
static unsigned char bh1745_again[3] = {1, 2, 16};
static int dim_flag = 0;
static long cofficient_judge = 1514;
static long cofficient_red[2]={5771, -1109};
static long cofficient_green[2] = {26879, 31316};
static long cofficient_blue[2] = {0,0};
enum tp_color_id{
	GOLDEN = 0,
	WHITE,
	BLACK,
	TP_COLOR_NUMBER,
};
#define MODULE_MANUFACTURE_NUMBER		3
#define VALID_FLAG							0x5555

#define BH1745_DRV_NAME	"bh1745"
#define DRIVER_VERSION		"1.0.0"

#define BH1745_REG_LEN 0x0a

#define	SENSORS_I2C_SCL	909
#define	SENSORS_I2C_SDA	908

#define BH1745_I2C_RETRY_COUNT		3 	/* Number of times to retry i2c */

/*wait more time to try read or write to avoid potencial risk*/
#define BH1745_I2C_RETRY_TIMEOUT	3	/* Timeout between retry (miliseconds) */

#define BH1745_I2C_BYTE 0
#define BH1745_I2C_WORD 1
/*dynamic debug mask to control log print,you can echo value to rgb_bh1745_debug to control*/
static int rgb_bh1745_debug_mask= 3;
module_param_named(rgb_bh1745_debug, rgb_bh1745_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

#define BH1745_ERR(x...) do {\
    if (rgb_bh1745_debug_mask >=0) \
        printk(KERN_ERR x);\
    } while (0)

#define BH1745_INFO(x...) do {\
    if ((rgb_bh1745_debug_mask >=0)) \
        printk(KERN_ERR x);\
    } while (0)
#define BH1745_FLOW(x...) do {\
    if (KERNEL_HWFLOW && (rgb_bh1745_debug_mask >=1)) \
        printk(KERN_ERR x);\
    } while (0)


static const char *data_array_name[MODULE_MANUFACTURE_NUMBER] = {
	[0] = "bh1745,cal_data0",
	[1] = "bh1745,cal_data1",
	[2] = "bh1745,cal_data2"
};
typedef struct rgb_bh1745_rgb_data {
    int red;
    int green;
    int blue;
    int clear;
    int lx;
    int color_temp;
} rgb_bh1745_rgb_data;

struct lux_cal_parameter{
	long judge;
	long cw_r_gain;
	long other_r_gain;

	long cw_g_gain;
	long other_g_gain;

	long cw_b_gain;
	long other_b_gain;
}lux_cal_parameter;

struct tp_lx_cal_parameter{
	long tp_module_id;
	struct lux_cal_parameter  gold_lux_cal_parameter;
	struct lux_cal_parameter  white_lux_cal_parameter;
	struct lux_cal_parameter  black_lux_cal_parameter;
}tp_lx_cal_parameter;

struct tp_lx_cal_parameter tp_module_parameter[MODULE_MANUFACTURE_NUMBER] = {{.tp_module_id = 0x55},{.tp_module_id = 0x55},{.tp_module_id = 0x55}};


struct rgb_bh1745_data {
	struct i2c_client *client;
	/*to protect the i2c read and write operation*/
	struct mutex update_lock;
	/*to protect only one thread to control the device register*/
	struct mutex single_lock;
	struct work_struct	als_dwork;	/* for ALS polling */

	struct input_dev *input_dev_als;

	/* regulator data */
	bool power_on;
	struct regulator *vdd;
	struct regulator *vio;

	/* pinctrl data*/
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
	/*sensor class for als and ps*/
	struct sensors_classdev als_cdev;
	struct rgb_bh1745_platform_data *platform_data;

	struct rgb_bh1745_rgb_data rgb_data;

	int irq;

	struct hrtimer timer;
	unsigned int enable;
	unsigned int irq_control;
	unsigned int ailt;
	unsigned int aiht;
	unsigned int pers;
	unsigned int config;
	unsigned int control;
	unsigned int measure_time;

	/* control flag from HAL */
	unsigned int enable_als_sensor;
	/*to record the open or close state of als before suspend*/
	unsigned int enable_als_state;

	/* ALS parameters */
	unsigned int als_threshold_l;	/* low threshold */
	unsigned int als_threshold_h;	/* high threshold */
	unsigned int als_data;		/* to store ALS data from CH0 or CH1*/
	int als_prev_lux;		/* to store previous lux value */
	unsigned int als_poll_delay;	/* needed for light sensor polling : micro-second (us) */
	bool device_exist;
};

static struct sensors_classdev sensors_light_cdev = {
	.name = "bh1745-light",
	.vendor = "rohm",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "10000",
	.resolution = "0.0125",
	.sensor_power = "0.20",
	.min_delay = 1000, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
/*
 * Global data
 */
static struct workqueue_struct *rgb_bh1745_workqueue = NULL;

/*init the register of device function for probe and every time the chip is powered on*/
static int rgb_bh1745_init_client(struct i2c_client *client);

/*we use the unified the function for i2c write and read operation*/
static int rgb_bh1745_i2c_write(struct i2c_client*client, u8 reg, u16 value,bool flag)
{
	int err,loop;

	struct rgb_bh1745_data *data = i2c_get_clientdata(client);

	loop = BH1745_I2C_RETRY_COUNT;
	/*we give three times to repeat the i2c operation if i2c errors happen*/
	while(loop) {
		mutex_lock(&data->update_lock);
		/*0 is i2c_smbus_write_byte_data,1 is i2c_smbus_write_word_data*/
		if(flag == BH1745_I2C_BYTE)
		{
			err = i2c_smbus_write_byte_data(client, reg, (u8)value);
		}
		else if(flag == BH1745_I2C_WORD)
		{
			err = i2c_smbus_write_word_data(client, reg, value);
		}
		else
		{
			BH1745_ERR("%s,line %d:attention: i2c write wrong flag\n",__func__,__LINE__);
			mutex_unlock(&data->update_lock);
			return -EINVAL;
		}
		mutex_unlock(&data->update_lock);
		if(err < 0){
			loop--;
			msleep(BH1745_I2C_RETRY_TIMEOUT);
		}
		else
			break;
	}
	/*after three times,we print the register and regulator value*/
	if(loop == 0){
		BH1745_ERR("%s,line %d:attention:i2c write err = %d\n",__func__,__LINE__,err);
	}

	return err;
}

static int rgb_bh1745_i2c_read(struct i2c_client*client, u8 reg,bool flag)
{
	int err,loop;

	struct rgb_bh1745_data *data = i2c_get_clientdata(client);

	loop = BH1745_I2C_RETRY_COUNT;
	/*we give three times to repeat the i2c operation if i2c errors happen*/
	while(loop) {
		mutex_lock(&data->update_lock);
		/*0 is i2c_smbus_read_byte_data,1 is i2c_smbus_read_word_data*/
		if(flag == BH1745_I2C_BYTE)
		{
			err = i2c_smbus_read_byte_data(client, reg);
		}
		else if(flag == BH1745_I2C_WORD)
		{
			err = i2c_smbus_read_word_data(client, reg);
		}
		else
		{
			BH1745_ERR("%s,line %d:attention: i2c read wrong flag\n",__func__,__LINE__);
			mutex_unlock(&data->update_lock);
			return -EINVAL;
		}
		mutex_unlock(&data->update_lock);
		if(err < 0){
			loop--;
			msleep(BH1745_I2C_RETRY_TIMEOUT);
		}
		else
			break;
	}
	/*after three times,we print the register and regulator value*/
	if(loop == 0){
		BH1745_ERR("%s,line %d:attention: i2c read err = %d,reg=0x%x\n",__func__,__LINE__,err,reg);
	}

	return err;
}

/*
*	print the registers value with proper format
*/
static int dump_reg_buf(struct rgb_bh1745_data *data,char *buf, int size,int enable)
{
	int i=0;

	if(enable)
		printk("[enable]");
	else
		printk("[disable]");
	printk(" reg_buf= ");
	for(i = 0;i < size; i++){
		printk("0x%2x  ",buf[i]);
	}
	
	printk("\n");
	return 0;
}
static int rgb_bh1745_regs_debug_print(struct rgb_bh1745_data *data,int enable)
{
	int i=0;
	char reg_buf[BH1745_REG_LEN];
	u8 reg = 0;
	struct i2c_client *client = data->client;

	/* read registers[0x0~0x1a] value*/
	for(i = 0; i < BH1745_REG_LEN; i++ )
	{
		reg = 0x50+i;
		reg_buf[i] = rgb_bh1745_i2c_read(client, reg, BH1745_I2C_BYTE);

		if(reg_buf[i] <0){
			BH1745_ERR("%s,line %d:read %d reg failed\n",__func__,__LINE__,i);
			return reg_buf[i] ;
		}
	}

	/* print the registers[0x0~0x1a] value in proper format*/
	dump_reg_buf(data,reg_buf,BH1745_REG_LEN,enable);

	return 0;
}

static void rgb_bh1745_dump_register(struct i2c_client *client)
{
	int sys_ctl,mode_ctl1,mode_ctl2,mode_ctl3,irq_ctl,pers;
	sys_ctl = rgb_bh1745_i2c_read(client, BH1745_SYSTEMCONTROL,BH1745_I2C_BYTE);
	mode_ctl1= rgb_bh1745_i2c_read(client, BH1745_MODECONTROL1,BH1745_I2C_BYTE);
	mode_ctl2 =rgb_bh1745_i2c_read(client, BH1745_MODECONTROL2,BH1745_I2C_BYTE);
	mode_ctl3=rgb_bh1745_i2c_read(client, BH1745_MODECONTROL3,BH1745_I2C_BYTE);
	irq_ctl=rgb_bh1745_i2c_read(client, BH1745_INTERRUPT,BH1745_I2C_BYTE);
	pers = rgb_bh1745_i2c_read(client, BH1745_PERSISTENCE,BH1745_I2C_BYTE);

	BH1745_INFO("%s,line %d:sys_ctl = 0x%x,mode_ctl1=0x%x,mode_ctl2=0x%x\n",__func__,__LINE__,sys_ctl,mode_ctl1,mode_ctl2);
	BH1745_INFO("%s,line %d:mode_ctl3 = 0x%x,irq_ctl=0x%x,pers=0x%x\n",__func__,__LINE__,mode_ctl3,irq_ctl,pers);
}

/******************************************************************************
 * NAME       : rgb_driver_reset
 * FUNCTION   : reset BH1745 register
 * REMARKS    :
 *****************************************************************************/
static int rgb_bh1745_driver_reset(struct i2c_client *client)
{
    int ret;

    /* set soft ware reset */
    ret = rgb_bh1745_i2c_write(client, BH1745_SYSTEMCONTROL, (SW_RESET | INT_RESET), BH1745_I2C_BYTE);
	if (ret < 0){
		BH1745_ERR("%s,line %d: i2c error,rgb_bh1745_driver_reset fail %d\n",__func__,__LINE__,ret);
		return ret;
	}
	BH1745_FLOW("%s,line %d:rgb_bh1745 reset\n",__func__,__LINE__);
	/*wait for device reset sucess*/
	mdelay(1);
    return (ret);
}

static int rgb_bh1745_set_enable(struct i2c_client *client, int enable)
{
	int ret;

	ret = rgb_bh1745_i2c_write(client, BH1745_MODECONTROL2, enable,BH1745_I2C_BYTE);
	if (ret < 0){
		BH1745_ERR("%s,line %d:i2c error,enable = %d\n",__func__,__LINE__,enable);
		return ret;
	}
	BH1745_FLOW("%s,line %d:rgb_bh1745 enable = %d\n",__func__,__LINE__,enable);
	return ret;
}

static int rgb_bh1745_set_pers(struct i2c_client *client, int pers)
{
	struct rgb_bh1745_data *data = i2c_get_clientdata(client);
	int ret;

	ret = rgb_bh1745_i2c_write(client,BH1745_PERSISTENCE, pers,BH1745_I2C_BYTE);
	if (ret < 0){
		BH1745_ERR("%s,line %d:i2c error,pers = %d\n",__func__,__LINE__,pers);
		return ret;
	}

	data->pers = pers;
	BH1745_FLOW("%s,line %d:rgb_bh1745 pers = %d\n",__func__,__LINE__,pers);
	return ret;
}

static int rgb_bh1745_set_interrupt(struct i2c_client *client, int irq_control)
{
	struct rgb_bh1745_data *data = i2c_get_clientdata(client);
	int ret;

	ret = rgb_bh1745_i2c_write(client,BH1745_INTERRUPT, irq_control,BH1745_I2C_BYTE);
	if (ret < 0){
		BH1745_ERR("%s,line %d:i2c error,irq_control = %d\n",__func__,__LINE__,irq_control);
		return ret;
	}

	data->irq_control = irq_control;
	BH1745_FLOW("%s,line %d:rgb_bh1745 irq_control = %d\n",__func__,__LINE__,irq_control);
	return ret;
}

static int rgb_bh1745_set_control(struct i2c_client *client, int control)
{
	struct rgb_bh1745_data *data = i2c_get_clientdata(client);
	int ret;

	ret = rgb_bh1745_i2c_write(client,BH1745_MODECONTROL3, control,BH1745_I2C_BYTE);
	if (ret < 0){
		BH1745_ERR("%s,line %d:i2c error,control = %d\n",__func__,__LINE__,control);
		return ret;
	}

	data->control = control;
	BH1745_FLOW("%s,line %d:rgb_bh1745 control = %d\n",__func__,__LINE__,control);
	return ret;
}

static int rgb_bh1745_set_measure_time(struct i2c_client *client, int measure_time)
{
	struct rgb_bh1745_data *data = i2c_get_clientdata(client);
	int ret;

	ret = rgb_bh1745_i2c_write(client,BH1745_MODECONTROL1, measure_time,BH1745_I2C_BYTE);
	if (ret < 0){
		BH1745_ERR("%s,line %d:i2c error,measure_time = %d\n",__func__,__LINE__,measure_time);
		return ret;
	}

	data->measure_time = measure_time;
	BH1745_FLOW("%s,line %d:rgb_bh1745 measure_time = %d\n",__func__,__LINE__,measure_time);
	return ret;
}

/******************************************************************************
 * FUNCTION   : Calculate lux
 * REMARKS    :
 * INPUT      : data  : each data value from IC (Red, Green, Blue, Clear)
 *            : gain  : gain's value of when sensor is gotten the data
 *            : itime : time's value of when sensor is getten data (unit is ms)
 * RETURN     : Lux value
 *****************************************************************************/
static int rgb_bh1745_calc_lx(struct i2c_client *client, struct rgb_bh1745_rgb_data *data, unsigned char gain, unsigned short itime)
{
	long long int lx;
	long long int lx_tmp;
	int ret ;

	if ((data->red >= BH1745_RGB_DATA_MAX) 
		|| (data->green >= BH1745_RGB_DATA_MAX)
		|| (data->blue >= BH1745_RGB_DATA_MAX))
	{
		lx = BH1745_LUX_MAX;
		return lx;
	}

	if(data->green < 1)
	{
		lx_tmp = 0;
	}
	else if((data->clear * JUDEG_COEFF) <( cofficient_judge*data->green))
	{
		lx_tmp = data->green*cofficient_green[0] + data->red *cofficient_red[0];
		BH1745_FLOW("%s,line %d:lx_temp 1: %lld\n", __func__, __LINE__, lx_tmp);
	}
	else
	{
		lx_tmp = data->green*cofficient_green[1]+data->red *cofficient_red[1];
		BH1745_FLOW("%s,line %d:lx_temp 1: %lld\n", __func__, __LINE__, lx_tmp);
	}

	BH1745_FLOW("%s,line %d: cal lx_tmp is %lld\n", __func__,__LINE__, lx_tmp);

	if (lx_tmp < 0)
		lx_tmp = 0;

	lx= lx_tmp/(gain/16)/(itime/160)/1000;

	if (lx < 200)
	{
		if(!dim_flag)
		{
			ret = rgb_bh1745_i2c_write(client,BH1745_MODECONTROL1, MEASURE_640MS , BH1745_I2C_BYTE);
			if (ret < 0)
				BH1745_ERR("%s,line %d:i2c change measurement error = %d\n",__func__,__LINE__,ret);
			else
				dim_flag = 1;
		}
	}
	else
	{
		if(dim_flag)
		{
			ret = rgb_bh1745_i2c_write(client,BH1745_MODECONTROL1, MEASURE_320MS , BH1745_I2C_BYTE);
			if (ret < 0)
				BH1745_ERR("%s,line %d:i2c change measurement error = %d\n",__func__,__LINE__,ret);
			else
				dim_flag = 0;
		}

	}

	BH1745_FLOW("%s,line %d:gain = %d, itime=%d, lux = %lld\n",__func__,__LINE__, gain, itime, lx);
	BH1745_FLOW("%s,line %d:judge = %ld ,red[0] = %ld, red[1]=%ld, green[0] = %ld, green[1]=%ld, blue[0]=%ld,blue[1]=%ld\n",__func__,__LINE__,cofficient_judge,cofficient_red[0], cofficient_red[1],cofficient_green[0], cofficient_green[1],cofficient_blue[0],cofficient_blue[1]);

	return ((int)lx);
}

/* delete rgb_bh1745_reschedule_work, we use queue_work to replase queue_delayed_work, because flush_delayed_work
   may cause system stop work */
/* ALS polling routine */
static void rgb_bh1745_als_polling_work_handler(struct work_struct *work)
{
	struct rgb_bh1745_data *data = container_of(work, struct rgb_bh1745_data,als_dwork);
	struct i2c_client *client=data->client;
	int  luxValue=0;
	unsigned char gain = 0;
	unsigned short time = 0;
	int tmp = 0;

	unsigned char lux_is_valid=1;

	data->rgb_data.red      = rgb_bh1745_i2c_read(client, BH1745_RED_DATA_LSB, BH1745_I2C_WORD);
	data->rgb_data.green  = rgb_bh1745_i2c_read(client, BH1745_GREEN_DATA_LSB, BH1745_I2C_WORD);
	data->rgb_data.blue     = rgb_bh1745_i2c_read(client, BH1745_BLUE_DATA_LSB, BH1745_I2C_WORD);
	data->rgb_data.clear    = rgb_bh1745_i2c_read(client, BH1745_CLEAR_DATA_LSB, BH1745_I2C_WORD);


	BH1745_FLOW("%s,line %d:rgb bh1745 data->rgb_data.red(%d); data->rgb_data.green(%d);data->rgb_data.blue(%d);data->rgb_data.clear(%d)\n",
		__func__,
		__LINE__,
		data->rgb_data.red,
		data->rgb_data.green,
		data->rgb_data.blue,
		data->rgb_data.clear);

	if((data->rgb_data.red < 0) 
		|| (data->rgb_data.green < 0)
		|| (data->rgb_data.blue < 0)
		|| (data->rgb_data.clear < 0))
	{
		/* don't report, this is invalid lux value */
		lux_is_valid = 0;
		luxValue = data->als_prev_lux;
		BH1745_ERR("%s,line %d i2c read fail, rgb bh1745 data->rgb_data.red(%d); data->rgb_data.green(%d);data->rgb_data.blue(%d);data->rgb_data.clear(%d)\n",
		__func__,
		__LINE__,
		data->rgb_data.red,
		data->rgb_data.green,
		data->rgb_data.blue,
		data->rgb_data.clear);
	}
	else
	{

		tmp = rgb_bh1745_i2c_read(client, BH1745_MODECONTROL1, BH1745_I2C_BYTE);
		if (tmp < 0){
			BH1745_ERR("%s:%d i2c read error tmp = %d\n", __func__,__LINE__,tmp);
			tmp = 0;
		}
		tmp = tmp & 0x7;
		time = bh1745_atime[tmp];
		tmp = rgb_bh1745_i2c_read(client, BH1745_MODECONTROL2, BH1745_I2C_BYTE);
		if (tmp < 0){
			BH1745_ERR("%s:%d i2c read error tmp = %d\n", __func__,__LINE__,tmp);
			tmp = 0;
		}
		tmp = tmp & 0x3;
		gain = bh1745_again[tmp];
		luxValue = rgb_bh1745_calc_lx(client, &(data->rgb_data), gain, time);
	}

	if (luxValue >= 0)
	{
		luxValue = luxValue < BH1745_LUX_MAX? luxValue : BH1745_LUX_MAX;
		data->als_prev_lux = luxValue;
	}
	else
	{
		BH1745_ERR("%s:%d cal lux error, luxValue = %d lux_is_valid =%d\n",__FUNCTION__,__LINE__,luxValue,lux_is_valid);
		/* don't report, this is invalid lux value */
		lux_is_valid = 0;
		luxValue = data->als_prev_lux;
	}
	/*remove it because we use other judge method to decide if pls close event is triggered by sunlight*/
	if (lux_is_valid) {
		/* report the lux level */
		input_report_abs(data->input_dev_als, ABS_MISC, luxValue);
		input_sync(data->input_dev_als);
		BH1745_FLOW("%s,line %d:rgb bh1745 lux=%d\n",__func__,__LINE__,luxValue);
	}
	/* restart timer */
	/* start a work after 200ms */
	if (0 != hrtimer_start(&data->timer,
							ktime_set(0, data->als_poll_delay * 1000000), HRTIMER_MODE_REL) )
	{
		BH1745_ERR("%s: hrtimer_start fail! nsec=%d\n", __func__, data->als_poll_delay);
	}
}

/*****************************************************************
Parameters    :  timer
Return        :  HRTIMER_NORESTART
Description   :  hrtimer_start call back function,
				 use to report als data
*****************************************************************/
static enum hrtimer_restart rgb_bh1745_als_timer_func(struct hrtimer *timer)
{
	struct rgb_bh1745_data* data = container_of(timer,struct rgb_bh1745_data,timer);
	
	queue_work(rgb_bh1745_workqueue, &data->als_dwork);
	return HRTIMER_NORESTART;
}
/*
 * IOCTL support
 */
static int rgb_bh1745_enable_als_sensor(struct i2c_client *client, int val)
{
	struct rgb_bh1745_data *data = i2c_get_clientdata(client);
	struct rgb_bh1745_platform_data *pdata = data->platform_data;
	int ret;

	BH1745_FLOW("%s,line %d:enable als val=%d\n",__func__,__LINE__,val);

	mutex_lock(&data->single_lock);
	if (val == 1) {
		/* turn on light  sensor */
		BH1745_INFO("%s:%d pdata->panel_id = %d pdata->tp_color = %d\n", __func__,__LINE__,pdata->panel_id,pdata->tp_color);
		BH1745_INFO("%s:%d lux cal parameter from dtsi  is judge[%ld], red[%ld], red[%ld], green[%ld] , green[%ld], blue[%ld],  blue[%ld]\n", __FUNCTION__, __LINE__,cofficient_judge, cofficient_red[0],cofficient_red[1],cofficient_green[0],cofficient_green[1],cofficient_blue[0],cofficient_blue[1]);
		if (data->enable_als_sensor == 0) {
			/* Power on and initalize the device */
			if (pdata->power_on)
				pdata->power_on(true,data);

			ret = rgb_bh1745_init_client(client);
			if (ret) {
				BH1745_ERR("%s:line:%d,Failed to init rgb_bh1745\n", __func__, __LINE__);
				mutex_unlock(&data->single_lock);
				return ret;
			}
			data->enable_als_sensor = 1;
			data->enable = (data->enable)|RGBC_EN_ON;
			rgb_bh1745_set_enable(client, data->enable);
			BH1745_INFO("%s: line:%d enable als sensor,data->enable=0x%x\n", __func__, __LINE__, data->enable);
			/* enable als sensor, start data report hrtimer */
			ret = hrtimer_start(&data->timer, ktime_set(0, data->als_poll_delay * 1000000), HRTIMER_MODE_REL);
			if (ret != 0) {
				BH1745_ERR("%s: hrtimer_start fail! nsec=%d\n", __func__, data->als_poll_delay);
			}
		}
	} else {
		/*
		 * turn off light sensor
		 */
		 if(data->enable_als_sensor == 1)
		 {
			data->enable_als_sensor = 0;
			data->enable =  ADC_GAIN_X16|RGBC_EN_OFF;
			rgb_bh1745_set_enable(client, data->enable);

			BH1745_INFO("%s: line:%d,disable rgb bh1745 als sensor,data->enable = 0x%x\n", __func__, __LINE__,data->enable);
			/* disable als sensor, cancne data report hrtimer */
			hrtimer_cancel(&data->timer);
			cancel_work_sync(&data->als_dwork);
			/*avoid hrtimer restart in data->als_dwork*/
			hrtimer_cancel(&data->timer);
		 }

	}
	/* Vote off  regulators if both light and prox sensor are off */
	if ((data->enable_als_sensor == 0)&&(pdata->power_on)){
		pdata->power_on(false,data);
	}
	mutex_unlock(&data->single_lock);
	BH1745_FLOW("%s: line:%d,enable als sensor success\n", __func__, __LINE__);
	return 0;
}
/*
 * SysFS support
 */
 static int rgb_bh1745_als_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	int ret = 0;
	static int als_enalbe_count=0;
	

	struct rgb_bh1745_data *data = container_of(sensors_cdev,struct rgb_bh1745_data, als_cdev);
	struct i2c_client *client = data->client;

	if ((enable != 0) && (enable != 1)) {
		BH1745_ERR("%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}
	BH1745_FLOW("%s,line %d:rgb bh1745 als enable=%d\n",__func__,__LINE__,enable);

	/*for debug and print registers value when enable/disable the als every time*/
	if(enable == 0)
	{
		rgb_bh1745_enable_als_sensor(data->client, enable);

		if(rgb_bh1745_debug_mask >= 1){
			BH1745_FLOW("attention:before als_disable %d times\n", als_enalbe_count);
			rgb_bh1745_regs_debug_print(data,enable);
		}
		rgb_bh1745_dump_register(client);
	}else{

		rgb_bh1745_enable_als_sensor(data->client, enable);

		if(rgb_bh1745_debug_mask >= 1){
			BH1745_FLOW("attention: after als_enable %d times\n",++als_enalbe_count);
		}
		rgb_bh1745_dump_register(client);		
	 }
	 return ret;
}

/*use this function to reset the poll_delay time(ms),val is the time parameter*/
static int rgb_bh1745_set_als_poll_delay(struct i2c_client *client,
		unsigned int val)
{
	struct rgb_bh1745_data *data = i2c_get_clientdata(client);
	int ret;

	/* minimum 10ms */
	if (val < 10)
		val = 10;
	data->als_poll_delay = 320;

	BH1745_INFO("%s,line %d:poll delay %d",__func__,__LINE__, data->als_poll_delay);

	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	cancel_work_sync(&data->als_dwork);
	hrtimer_cancel(&data->timer);
	ret = hrtimer_start(&data->timer, ktime_set(0, data->als_poll_delay * 1000000), HRTIMER_MODE_REL);
	if (ret != 0) {
		BH1745_ERR("%s,line%d: hrtimer_start fail! nsec=%d\n", __func__, __LINE__,data->als_poll_delay);
		return ret;
	}
	return 0;
}

static int rgb_bh1745_als_poll_delay(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
	struct rgb_bh1745_data *data = container_of(sensors_cdev,
			struct rgb_bh1745_data, als_cdev);
	rgb_bh1745_set_als_poll_delay(data->client, delay_msec);
	return 0;
}
static ssize_t rgb_bh1745_show_red_data(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int red_data;

	red_data = rgb_bh1745_i2c_read(client,BH1745_RED_DATA_LSB,BH1745_I2C_WORD);

	return snprintf(buf,32,"%d\n", red_data);
}

static DEVICE_ATTR(red_data, S_IRUGO, rgb_bh1745_show_red_data, NULL);

static ssize_t rgb_bh1745_show_green_data(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int green_data;

	green_data = rgb_bh1745_i2c_read(client,BH1745_GREEN_DATA_LSB,BH1745_I2C_WORD);

	return snprintf(buf,32, "%d\n", green_data);
}

static DEVICE_ATTR(green_data, S_IRUGO, rgb_bh1745_show_green_data, NULL);

static ssize_t rgb_bh1745_show_blue_data(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int blue_data;

	blue_data = rgb_bh1745_i2c_read(client, BH1745_BLUE_DATA_LSB,BH1745_I2C_WORD);
	if(blue_data <0){
		BH1745_ERR("%s,line %d:read blue_data failed\n",__func__,__LINE__);
	}

 	return snprintf(buf,32, "%d\n", blue_data);
 }

static DEVICE_ATTR(blue_data, S_IRUGO, rgb_bh1745_show_blue_data, NULL);

static ssize_t rgb_bh1745_show_clear_data(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int clear_data;

	clear_data = rgb_bh1745_i2c_read(client, BH1745_CLEAR_DATA_LSB, BH1745_I2C_WORD);
	if(clear_data <0){
		BH1745_ERR("%s,line %d:read clear_data failed\n",__func__,__LINE__);
	}

 	return snprintf(buf,32, "%d\n", clear_data);
 }

static DEVICE_ATTR(clear_data, S_IRUGO, rgb_bh1745_show_clear_data, NULL);

/*
* set the register's value from userspace
* Usage: echo "0x08|0x12" > dump_reg
*			"reg_address|reg_value"
*/
static ssize_t rgb_bh1745_write_reg(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int val_len_max = 4;
	char* input_str =NULL;
	char reg_addr_str[10]={'\0'};
	char reg_val_str[10]={'\0'};
	long reg_addr,reg_val;
	int addr_lenth=0,value_lenth=0,buf_len=0,ret = -1;
	char* strtok=NULL;

	buf_len = strlen(buf);
	input_str = kzalloc(buf_len, GFP_KERNEL);
	if (!input_str)
	{
		pr_err("%s:kmalloc fail!\n",__func__);
		return -ENOMEM;
	}

	snprintf(input_str, 10,"%s", buf);
	/*Split the string when encounter "|", for example "0x08|0x12" will be splited "0x18" "0x12" */
	strtok=strsep(&input_str, "|");
	if(strtok!=NULL)
	{
		addr_lenth = strlen(strtok);
		memcpy(reg_addr_str,strtok,((addr_lenth > (val_len_max))?(val_len_max):addr_lenth));
	}
	else
	{
		pr_err("%s: buf name Invalid:%s", __func__,buf);
		goto parse_fail_exit;
	}
	strtok=strsep(&input_str, "|");
	if(strtok!=NULL)
	{
		value_lenth = strlen(strtok);
		memcpy(reg_val_str,strtok,((value_lenth > (val_len_max))?(val_len_max):value_lenth));
	}
	else
	{
		pr_err("%s: buf value Invalid:%s", __func__,buf);
		goto parse_fail_exit;
	}

	/* transform string to long int */
	ret = kstrtol(reg_addr_str,16,&reg_addr);
	if(ret)
		goto parse_fail_exit;

	ret = kstrtol(reg_val_str,16,&reg_val);
	if(ret)
		goto parse_fail_exit;

	/* write the parsed value in the register*/
	ret = rgb_bh1745_i2c_write(client,(char)reg_addr,(char)reg_val,BH1745_I2C_BYTE);
	if (ret < 0){
		goto parse_fail_exit;
	}
	return count;

parse_fail_exit:
	if (input_str)
		kfree(input_str);

	return ret;
}

/*
* show all registers' value to userspace
*/
static ssize_t rgb_bh1745_print_reg_buf(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int i;
	char reg[BH1745_REG_LEN];
	struct i2c_client *client = to_i2c_client(dev);

	/* read all register value and print to user*/
	for(i = 0; i < BH1745_REG_LEN; i++ )
	{
		reg[i] = rgb_bh1745_i2c_read(client, (0x50+i), BH1745_I2C_BYTE);
		if(reg[i] <0){
			BH1745_ERR("%s,line %d:read %d reg failed\n",__func__,__LINE__,i);
			return reg[i] ;
		}
	}

	return snprintf(buf,512,"reg[0x0~0x8]=0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x\n"
			"reg[0x09~0x11]0x%2x\n",
			reg[0x00],reg[0x01],reg[0x02],reg[0x03],reg[0x04],reg[0x05],reg[0x06],reg[0x07],reg[0x08],reg[0x09]);
}
/*
 *	panel_id represent junda ofilm jdi.
 *	tp_color represent golden white black
 *
 */
static ssize_t write_module_tpcolor(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rgb_bh1745_data *data = i2c_get_clientdata(client);
	struct rgb_bh1745_platform_data *pdata = data->platform_data;
	int err;
	u32 val;
	int valid_flag;
	int i;
	err = kstrtoint(buf, 0, &val);
	if (err < 0) {
		BH1745_ERR("%s:%d kstrtoint failed\n", __func__,__LINE__);
		return count;
	}
	valid_flag = val & 0xffff;
	pdata->panel_id = (val >> 16) & 0xff;
	pdata->tp_color = (val >> 24) & 0xff;
	if (valid_flag != VALID_FLAG){
		BH1745_ERR("%s:%d  valid flag error\n", __func__,__LINE__);
		return count;
	}
	for (i = 0;i < MODULE_MANUFACTURE_NUMBER;i++){
		if (pdata->panel_id == tp_module_parameter[i].tp_module_id){
			if (pdata->tp_color == GOLDEN){
				cofficient_judge = tp_module_parameter[i].gold_lux_cal_parameter.judge;
				cofficient_red[0] = tp_module_parameter[i].gold_lux_cal_parameter.cw_r_gain;
				cofficient_red[1] = tp_module_parameter[i].gold_lux_cal_parameter.other_r_gain;
				cofficient_green[0] = tp_module_parameter[i].gold_lux_cal_parameter.cw_g_gain;
				cofficient_green[1] = tp_module_parameter[i].gold_lux_cal_parameter.other_g_gain;
				cofficient_blue[0] = tp_module_parameter[i].gold_lux_cal_parameter.cw_b_gain;
				cofficient_blue[1] = tp_module_parameter[i].gold_lux_cal_parameter.other_b_gain;
			}else if (pdata->tp_color == WHITE){
				cofficient_judge = tp_module_parameter[i].white_lux_cal_parameter.judge;
				cofficient_red[0] = tp_module_parameter[i].white_lux_cal_parameter.cw_r_gain;
				cofficient_red[1] = tp_module_parameter[i].white_lux_cal_parameter.other_r_gain;
				cofficient_green[0] = tp_module_parameter[i].white_lux_cal_parameter.cw_g_gain;
				cofficient_green[1] = tp_module_parameter[i].white_lux_cal_parameter.other_g_gain;
				cofficient_blue[0] = tp_module_parameter[i].white_lux_cal_parameter.cw_b_gain;
				cofficient_blue[1] = tp_module_parameter[i].white_lux_cal_parameter.other_b_gain;
			}else if (pdata->tp_color == BLACK){
				cofficient_judge = tp_module_parameter[i].black_lux_cal_parameter.judge;
				cofficient_red[0] = tp_module_parameter[i].black_lux_cal_parameter.cw_r_gain;
				cofficient_red[1] = tp_module_parameter[i].black_lux_cal_parameter.other_r_gain;
				cofficient_green[0] = tp_module_parameter[i].black_lux_cal_parameter.cw_g_gain;
				cofficient_green[1] = tp_module_parameter[i].black_lux_cal_parameter.other_g_gain;
				cofficient_blue[0] = tp_module_parameter[i].black_lux_cal_parameter.cw_b_gain;
				cofficient_blue[1] = tp_module_parameter[i].black_lux_cal_parameter.other_b_gain;
			}
		}
	}
	BH1745_INFO("%s:%d lux cal black parameter from dtsi  is judge[%ld], red[%ld], red[%ld], green[%ld] , green[%ld], blue[%ld],  blue[%ld]\n", __FUNCTION__, __LINE__,cofficient_judge, cofficient_red[0],cofficient_red[1],cofficient_green[0],cofficient_green[1],cofficient_blue[0],cofficient_blue[1]);
	return count;
}
static ssize_t read_tp_parameters(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return snprintf(buf,2048,"golden0 judge[%ld],cw_r_gain[%ld],other_r_gain[%ld],cw_g_gain[%ld],other_g_gain[%ld],cw_b_gain[%ld],other_b_gain[%ld]\n"
	                                    "white0 judge[%ld],cw_r_gain[%ld],other_r_gain[%ld],cw_g_gain[%ld],other_g_gain[%ld],cw_b_gain[%ld],other_b_gain[%ld]\n"
	                                    "black0 judge[%ld],cw_r_gain[%ld],other_r_gain[%ld],cw_g_gain[%ld],other_g_gain[%ld],cw_b_gain[%ld],other_b_gain[%ld]\n"
	                                    "golden1 judge[%ld],cw_r_gain[%ld],other_r_gain[%ld],cw_g_gain[%ld],other_g_gain[%ld],cw_b_gain[%ld],other_b_gain[%ld]\n"
	                                    "white1 judge[%ld],cw_r_gain[%ld],other_r_gain[%ld],cw_g_gain[%ld],other_g_gain[%ld],cw_b_gain[%ld],other_b_gain[%ld]\n"
	                                    "black1 judge[%ld],cw_r_gain[%ld],other_r_gain[%ld],cw_g_gain[%ld],other_g_gain[%ld],cw_b_gain[%ld],other_b_gain[%ld]\n",
	tp_module_parameter[0].gold_lux_cal_parameter.judge,tp_module_parameter[0].gold_lux_cal_parameter.cw_r_gain,tp_module_parameter[0].gold_lux_cal_parameter.other_r_gain,
	tp_module_parameter[0].gold_lux_cal_parameter.cw_g_gain,tp_module_parameter[0].gold_lux_cal_parameter.other_g_gain,tp_module_parameter[0].gold_lux_cal_parameter.cw_b_gain,tp_module_parameter[0].gold_lux_cal_parameter.other_b_gain,
	tp_module_parameter[0].white_lux_cal_parameter.judge,tp_module_parameter[0].white_lux_cal_parameter.cw_r_gain,tp_module_parameter[0].white_lux_cal_parameter.other_r_gain,
	tp_module_parameter[0].white_lux_cal_parameter.cw_g_gain,tp_module_parameter[0].white_lux_cal_parameter.other_g_gain,tp_module_parameter[0].white_lux_cal_parameter.cw_b_gain,tp_module_parameter[0].white_lux_cal_parameter.other_b_gain,
	tp_module_parameter[0].black_lux_cal_parameter.judge,tp_module_parameter[0].black_lux_cal_parameter.cw_r_gain,tp_module_parameter[0].black_lux_cal_parameter.other_r_gain,
	tp_module_parameter[0].black_lux_cal_parameter.cw_g_gain,tp_module_parameter[0].black_lux_cal_parameter.other_g_gain,tp_module_parameter[0].black_lux_cal_parameter.cw_b_gain,tp_module_parameter[0].black_lux_cal_parameter.other_b_gain,
	tp_module_parameter[1].gold_lux_cal_parameter.judge,tp_module_parameter[1].gold_lux_cal_parameter.cw_r_gain,tp_module_parameter[1].gold_lux_cal_parameter.other_r_gain,
	tp_module_parameter[1].gold_lux_cal_parameter.cw_g_gain,tp_module_parameter[1].gold_lux_cal_parameter.other_g_gain,tp_module_parameter[1].gold_lux_cal_parameter.cw_b_gain,tp_module_parameter[1].gold_lux_cal_parameter.other_b_gain,
	tp_module_parameter[1].white_lux_cal_parameter.judge,tp_module_parameter[1].white_lux_cal_parameter.cw_r_gain,tp_module_parameter[1].white_lux_cal_parameter.other_r_gain,
	tp_module_parameter[1].white_lux_cal_parameter.cw_g_gain,tp_module_parameter[1].white_lux_cal_parameter.other_g_gain,tp_module_parameter[1].white_lux_cal_parameter.cw_b_gain,tp_module_parameter[1].white_lux_cal_parameter.other_b_gain,
	tp_module_parameter[1].black_lux_cal_parameter.judge,tp_module_parameter[1].black_lux_cal_parameter.cw_r_gain,tp_module_parameter[1].black_lux_cal_parameter.other_r_gain,
	tp_module_parameter[1].black_lux_cal_parameter.cw_g_gain,tp_module_parameter[1].black_lux_cal_parameter.other_g_gain,tp_module_parameter[1].black_lux_cal_parameter.cw_b_gain,tp_module_parameter[1].black_lux_cal_parameter.other_b_gain);
}

static DEVICE_ATTR(dump_reg ,S_IRUGO|S_IWUSR|S_IWGRP, rgb_bh1745_print_reg_buf, rgb_bh1745_write_reg);
static DEVICE_ATTR(module_tpcolor ,S_IRUGO|S_IWUSR, NULL, write_module_tpcolor);
static DEVICE_ATTR(dump_tp_parameters ,S_IRUGO, read_tp_parameters, NULL);
static struct attribute *rgb_bh1745_attributes[] = {
	&dev_attr_red_data.attr,
	&dev_attr_green_data.attr,
	&dev_attr_blue_data.attr,
	&dev_attr_clear_data.attr,
	&dev_attr_dump_reg.attr,
	&dev_attr_module_tpcolor.attr,
	&dev_attr_dump_tp_parameters.attr,
	NULL
};

static const struct attribute_group rgb_bh1745_attr_group = {
	.attrs = rgb_bh1745_attributes,
};

/*
 * Initialization function
 */
static int rgb_bh1745_read_device_id(struct i2c_client *client)
{
	int id;
	int err;
	id    = rgb_bh1745_i2c_read(client, BH1745_SYSTEMCONTROL, BH1745_I2C_BYTE);
	id &= 0x3f;
	if (id == 0x0b) {
		BH1745_INFO("%s: ROHM BH1745\n", __func__);
		err = app_info_set("L-Sensor", "BH1745");
		if (err < 0)/*failed to add app_info*/
		{
		    BH1745_ERR("%s %d:failed to add app_info\n", __func__, __LINE__);
		}
	} else {
		BH1745_INFO("%s: ROHM BH1745 Does not exist \n", __func__);
		return -ENODEV;
	}
	return 0;
}
static int rgb_bh1745_init_client(struct i2c_client *client)
{
	struct rgb_bh1745_data *data = i2c_get_clientdata(client);
	int err;

	rgb_bh1745_driver_reset(client);
	
	data->enable = ADC_GAIN_X16|RGBC_EN_OFF;
	err = rgb_bh1745_set_enable(client, data->enable);
	if (err < 0)
	{
		BH1745_ERR("%s,line%d:rgb_bh1745_set_enable FAIL ",__func__,__LINE__);
		return err;
	}

	err = rgb_bh1745_set_interrupt(client, BH1745_IRQ_DISABLE);
	if (err < 0)
	{
		BH1745_ERR("%s,line%d:rgb_bh1745_set_interrupt FAIL ",__func__,__LINE__);
		return err;
	}

	err = rgb_bh1745_set_measure_time(client, MEASURE_320MS);
	if (err < 0)
	{
		BH1745_ERR("%s,line%d:rgb_bh1745_set_measure_time FAIL ",__func__,__LINE__);
		return err;
	}

	err = rgb_bh1745_set_pers(client, BH1745_PPERS_1);
	if (err < 0)
	{
		BH1745_ERR("%s,line%d:rgb_bh1745_set_pers FAIL ",__func__,__LINE__);
		return err;
	}

	err = rgb_bh1745_set_control(client, MODE_CTL_FIX_VAL);
	if (err < 0)
	{
		BH1745_ERR("%s,line%d:rgb_bh1745_set_pers FAIL ",__func__,__LINE__);
		return err;
	}
	

	return 0;
}
/*qualcom updated the regulator configure functions and we add them all*/
static int sensor_regulator_configure(struct rgb_bh1745_data *data, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0, BH1745_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0, BH1745_VIO_MAX_UV);

		regulator_put(data->vio);

	} else {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			BH1745_ERR("%s,line%d:Regulator get failed vdd rc=%d\n",__func__,__LINE__, rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd, BH1745_VDD_MIN_UV, BH1745_VDD_MAX_UV);
			if (rc) {
				BH1745_ERR("%s,line%d:Regulator set failed vdd rc=%d\n",__func__,__LINE__,rc);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->client->dev, "vio");
		if (IS_ERR(data->vio)) {
			rc = PTR_ERR(data->vio);
			BH1745_ERR("%s,line%d:Regulator get failed vio rc=%d\n",__func__,__LINE__, rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			rc = regulator_set_voltage(data->vio,
				BH1745_VIO_MIN_UV, BH1745_VIO_MAX_UV);
			if (rc) {
				BH1745_ERR("%s,line%d:Regulator set failed vio rc=%d\n",__func__,__LINE__, rc);
				goto reg_vio_put;
			}
		}

	}

	return 0;
reg_vio_put:
	regulator_put(data->vio);

reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, BH1745_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}
/*In suspend and resume function,we only control the als,leave pls alone*/
static int rgb_bh1745_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct rgb_bh1745_data *data = i2c_get_clientdata(client);
	int rc;
	BH1745_INFO("%s,line%d:BH1745 SUSPEND\n",__func__,__LINE__);

	data->enable_als_state = data->enable_als_sensor;
	if(data->enable_als_sensor){
		BH1745_INFO("%s,line%d:BH1745 SUSPEND and disable als\n",__func__,__LINE__);
		rc = rgb_bh1745_enable_als_sensor(data->client, 0);
		if (rc){
			BH1745_ERR("%s,line%d:Disable rgb light sensor fail! rc=%d\n",__func__,__LINE__, rc);
		}
	}

	return 0;
}

static int rgb_bh1745_resume(struct i2c_client *client)
{
	struct rgb_bh1745_data *data = i2c_get_clientdata(client);
	int ret = 0;

	BH1745_INFO("%s,line%d:BH1745 RESUME\n",__func__,__LINE__);

	if (data->enable_als_state) {
		ret = rgb_bh1745_enable_als_sensor(data->client, 1);
		if (ret){
			BH1745_ERR("%s,line%d:enable rgb  light sensor fail! rc=%d\n",__func__,__LINE__, ret);
		}
	}

	return 0;
}
/*pamameter subfunction of probe to reduce the complexity of probe function*/
static int rgb_bh1745_sensorclass_init(struct rgb_bh1745_data *data,struct i2c_client* client)
{
	int err;
	/* Register to sensors class */
	data->als_cdev = sensors_light_cdev;
	data->als_cdev.sensors_enable = rgb_bh1745_als_set_enable;
	data->als_cdev.sensors_poll_delay = rgb_bh1745_als_poll_delay;

	err = sensors_classdev_register(&client->dev, &data->als_cdev);
	if (err) {
		BH1745_ERR("%s: Unable to register to sensors class: %d\n",__func__, err);
	}

	return err;
}
static void rgb_bh1745_parameter_init(struct rgb_bh1745_data *data)
{
	struct rgb_bh1745_platform_data *pdata = data->platform_data;
	data->enable = ADC_GAIN_X16|RGBC_EN_OFF;	/* default mode is standard */
	data->enable_als_sensor = 0;	// default to 0
	data->als_poll_delay = 320;	// default to 320ms
	data->als_prev_lux = 300;
	pdata->panel_id = -1;
	pdata->tp_color = -1;
}
/*input init subfunction of probe to reduce the complexity of probe function*/
static int rgb_bh1745_input_init(struct rgb_bh1745_data *data)
{
	int err = 0;
	/* Register to Input Device */
	data->input_dev_als = input_allocate_device();
	if (!data->input_dev_als) {
		err = -ENOMEM;
		BH1745_ERR("%s: Failed to allocate input device als\n", __func__);
		goto exit;
	}

	set_bit(EV_ABS, data->input_dev_als->evbit);
	input_set_abs_params(data->input_dev_als, ABS_MISC, 0, 10000, 0, 0);

	data->input_dev_als->name = "light";

	err = input_register_device(data->input_dev_als);
	if (err) {
		err = -ENOMEM;
		BH1745_ERR("%s: Unable to register input device als: %s\n",
				__func__, data->input_dev_als->name);
		goto input_register_err;
	}
	return err;

input_register_err:
	input_free_device(data->input_dev_als);
exit:
	return err;
}

static int sensor_regulator_power_on(struct rgb_bh1745_data *data, bool on)
{
	int rc = 0;

	if (!on) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			BH1745_ERR("%s: Regulator vdd disable failed rc=%d\n", __func__, rc);
			return rc;
		}

		rc = regulator_disable(data->vio);
		if (rc) {
			BH1745_ERR("%s: Regulator vdd disable failed rc=%d\n", __func__, rc);
			rc = regulator_enable(data->vdd);
			BH1745_ERR("%s:Regulator vio re-enabled rc=%d\n",__func__, rc);
			/*
			 * Successfully re-enable regulator.
			 * Enter poweron delay and returns error.
			 */
			if (!rc) {
				rc = -EBUSY;
				goto enable_delay;
			}
		}
		return rc;
	} else {
		rc = regulator_enable(data->vdd);
		if (rc) {
			BH1745_ERR("%s:Regulator vdd enable failed rc=%d\n",__func__, rc);
			return rc;
		}

		rc = regulator_enable(data->vio);
		if (rc) {
			BH1745_ERR("%s:Regulator vio enable failed rc=%d\n", __func__,rc);
			rc = regulator_disable(data->vdd);
			return rc;
		}
	}
enable_delay:
	msleep(130);
	BH1745_FLOW("%s:Sensor regulator power on =%d\n",__func__, on);
	return rc;
}

static int sensor_platform_hw_power_on(bool on,struct rgb_bh1745_data *data)
{
	int err = 0;

	if (data->power_on != on) {
		if (!IS_ERR_OR_NULL(data->pinctrl)) {
			if (on)
				/*after poweron,set the INT pin the default state*/
				err = pinctrl_select_state(data->pinctrl,
					data->pin_default);
			if (err)
				BH1745_ERR("%s,line%d:Can't select pinctrl state\n", __func__, __LINE__);
		}

		err = sensor_regulator_power_on(data, on);
		if (err)
			BH1745_ERR("%s,line%d:Can't configure regulator!\n", __func__, __LINE__);
		else
			data->power_on = on;
	}

	return err;
}
static int sensor_platform_hw_init(struct rgb_bh1745_data *data)
{
	int error;

	error = sensor_regulator_configure(data, true);
	if (error < 0) {
		BH1745_ERR("%s,line %d:unable to configure regulator\n",__func__,__LINE__);
		return error;
	}

	return 0;
}

static void sensor_platform_hw_exit(struct rgb_bh1745_data *data)
{
	int error;
	error = sensor_regulator_configure(data, false);
	if (error < 0) {
		BH1745_ERR("%s,line %d:unable to configure regulator\n",__func__,__LINE__);
	}
}
static int rgb_bh1745_pinctrl_init(struct rgb_bh1745_data *data)
{
	struct i2c_client *client = data->client;

	data->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(data->pinctrl)) {
		BH1745_ERR("%s,line %d:Failed to get pinctrl\n",__func__,__LINE__);
		return PTR_ERR(data->pinctrl);
	}
	/*we have not set the sleep state of INT pin*/
	data->pin_default =
		pinctrl_lookup_state(data->pinctrl, "default");
	if (IS_ERR_OR_NULL(data->pin_default)) {
		BH1745_ERR("%s,line %d:Failed to look up default state\n",__func__,__LINE__);
		return PTR_ERR(data->pin_default);
	}

	return 0;
}

static int sensor_parse_dt(struct device *dev,
		struct rgb_bh1745_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	unsigned int tmp;
	int tp_moudle_count = 0;
	int index =0;
	int rc = 0;
	int array_len = 0;
	int retval = 0;
	int i = 0;
	const char *raw_data0_dts = NULL;
	long *ptr = NULL;


	/* set functions of platform data */
	pdata->init = sensor_platform_hw_init;
	pdata->exit = sensor_platform_hw_exit;
	pdata->power_on = sensor_platform_hw_power_on;

	rc = of_property_read_u32(np, "bh1745,tp_moudle_count", &tmp);
	if (rc) {
		BH1745_ERR("%s,line %d:Unable to read ga_a_value\n",__func__,__LINE__);
		return rc;
	}
	tp_moudle_count = tmp;

	BH1745_INFO("%s:%d read lux cal parameter count from dtsi  is %d\n", __FUNCTION__, __LINE__, tp_moudle_count);

	if(tp_moudle_count > MODULE_MANUFACTURE_NUMBER){
		BH1745_ERR("%s,line %d:tp_moudle_count from dtsi too large: %d\n",__func__,__LINE__, tp_moudle_count);
		return  -EINVAL;
	}

	for(i=0; i<tp_moudle_count; i++){
		array_len = of_property_count_strings(np, data_array_name[i]);
		if (array_len != PARSE_DTSI_NUMBER) {
			BH1745_ERR("%s:%d bh1745,junda_data0 length invaild or dts number is larger than:%d\n",__FUNCTION__,__LINE__,array_len);
			return array_len;
		}
		BH1745_INFO("%s:%d read lux cal parameter count from dtsi  is %d\n", __FUNCTION__, __LINE__, array_len);

		ptr = (long *)&tp_module_parameter[i];

		for(index = 0; index < array_len; index++){
			retval = of_property_read_string_index(np, data_array_name[i], index, &raw_data0_dts);
			if (retval) {
				BH1745_ERR("%s:%d read index = %d,raw_data0_dts = %s,retval = %d error,\n",__FUNCTION__,__LINE__,index, raw_data0_dts, retval);
				return retval;
			}
			ptr[index]  = simple_strtol(raw_data0_dts, NULL, 10);
			BH1745_INFO("%s:%d lux cal parameter from dtsi  is %ld\n", __FUNCTION__, __LINE__, ptr[index]);
		}
	}

	return 0;
}

/*
 * I2C init/probing/exit functions
 */
static struct i2c_driver rgb_bh1745_driver;
static int rgb_bh1745_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct rgb_bh1745_data *data;
	struct rgb_bh1745_platform_data *pdata;
	int err = 0;

	BH1745_INFO("%s,line %d:PROBE START.\n",__func__,__LINE__);

	if (client->dev.of_node) {
		/*Memory allocated with this function is automatically freed on driver detach.*/
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct rgb_bh1745_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			BH1745_ERR("%s,line %d:Failed to allocate memory\n",__func__,__LINE__);
			err =-ENOMEM;
			goto exit;
		}

		client->dev.platform_data = pdata;
		err = sensor_parse_dt(&client->dev, pdata);
		if (err) {
			BH1745_ERR("%s: sensor_parse_dt() err\n", __func__);
			goto exit_parse_dt;
		}
	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			BH1745_ERR("%s,line %d:No platform data\n",__func__,__LINE__);
			err = -ENODEV;
			goto exit;
		}
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		BH1745_ERR("%s,line %d:Failed to i2c_check_functionality\n",__func__,__LINE__);
		err = -EIO;
		goto exit_parse_dt;
	}

	data = kzalloc(sizeof(struct rgb_bh1745_data), GFP_KERNEL);
	if (!data) {
		BH1745_ERR("%s,line %d:Failed to allocate memory\n",__func__,__LINE__);
		err = -ENOMEM;
		goto exit_parse_dt;
	}

	data->platform_data = pdata;
	data->client = client;
	data->device_exist = false;

	/* h/w initialization */
	if (pdata->init)
		err = pdata->init(data);

	if (pdata->power_on)
		err = pdata->power_on(true,data);

	i2c_set_clientdata(client, data);
	rgb_bh1745_parameter_init(data);
	/* initialize pinctrl */
	err = rgb_bh1745_pinctrl_init(data);
	if (err) {
		BH1745_ERR("%s,line %d:Can't initialize pinctrl\n",__func__,__LINE__);
		data->pinctrl = NULL;
	}else{
		BH1745_ERR("%s,line %d:RGB BH1745 use pinctrl\n",__func__,__LINE__);
	}

	if (!IS_ERR_OR_NULL(data->pinctrl)){
		err = pinctrl_select_state(data->pinctrl, data->pin_default);
		if (err) {
			BH1745_ERR("%s,line %d:Can't select pinctrl default state\n",__func__,__LINE__);
		}
		BH1745_ERR("%s,line %d:RGB BH1745 select pinctrl default state\n",__func__,__LINE__);
	}

	mutex_init(&data->update_lock);
	mutex_init(&data->single_lock);

	INIT_WORK(&data->als_dwork, rgb_bh1745_als_polling_work_handler);
	
	/* Initialize the BH1745 chip and judge who am i*/
	err = rgb_bh1745_read_device_id(client);
	if (err) {
		BH1745_ERR("%s: Failed to read rgb_bh1745\n", __func__);
		goto exit_power_off;
	}
	err = rgb_bh1745_init_client(client);
	if (err) {
		BH1745_ERR("%s: Failed to init rgb_bh1745\n", __func__);
		goto exit_power_off;
	}

	err = rgb_bh1745_input_init(data);
	if(err)
		goto exit_power_off;

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &rgb_bh1745_attr_group);
	if (err)
		goto exit_unregister_dev_als;

	err = rgb_bh1745_sensorclass_init(data,client);
	if (err) {
		BH1745_ERR("%s: Unable to register to sensors class: %d\n", __func__, err);
		goto exit_remove_sysfs_group;
	}

	rgb_bh1745_workqueue = create_workqueue("rgb_bh1745_work_queue");
	if (!rgb_bh1745_workqueue)
	{
		BH1745_ERR("%s: Create ps_workqueue fail.\n", __func__);
		goto exit_unregister_sensorclass;
	}

	/* init hrtimer and call back function */
	hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->timer.function = rgb_bh1745_als_timer_func;
	set_sensors_list(L_SENSOR);
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
	set_hw_dev_flag(DEV_I2C_L_SENSOR);
#endif
	err = set_sensor_input(ALS, data->input_dev_als->dev.kobj.name);
	if (err) {
		BH1745_ERR("%s set_sensor_input ALS failed\n", __func__);
	}

	if (pdata->power_on)
		err = pdata->power_on(false,data);

	BH1745_INFO("%s: Support ver. %s enabled\n", __func__, DRIVER_VERSION);
	data->device_exist = true;
	return 0;

exit_unregister_sensorclass:
	sensors_classdev_unregister(&data->als_cdev);
exit_remove_sysfs_group:
	sysfs_remove_group(&client->dev.kobj, &rgb_bh1745_attr_group);
exit_unregister_dev_als:
	input_unregister_device(data->input_dev_als);
exit_power_off:
	if (pdata->power_on)
		pdata->power_on(false,data);
	if (pdata->exit)
		pdata->exit(data);
	kfree(data);

exit_parse_dt:
exit:
	return err;
}

static int rgb_bh1745_remove(struct i2c_client *client)
{
	struct rgb_bh1745_data *data = i2c_get_clientdata(client);
	struct rgb_bh1745_platform_data *pdata = data->platform_data;

	data->enable = ADC_GAIN_X16|RGBC_EN_OFF;
	rgb_bh1745_set_enable(client, data->enable);
	sysfs_remove_group(&client->dev.kobj, &rgb_bh1745_attr_group);

	input_unregister_device(data->input_dev_als);

	free_irq(client->irq, data);
	hrtimer_cancel(&data->timer);

	if (pdata->power_on)
		pdata->power_on(false,data);

	if (pdata->exit)
		pdata->exit(data);

	kfree(data);

	return 0;
}

static const struct i2c_device_id rgb_bh1745_id[] = {
	{ "bh1745", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rgb_bh1745_id);

static struct of_device_id rgb_bh1745_match_table[] = {
	{ .compatible = "rohm,bh1745",},
	{ },
};

static struct i2c_driver rgb_bh1745_driver = {
	.driver = {
		.name   = BH1745_DRV_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = rgb_bh1745_match_table,
	},
	.probe  = rgb_bh1745_probe,
	.remove = rgb_bh1745_remove,
	.suspend = rgb_bh1745_suspend,
	.resume = rgb_bh1745_resume,
	.id_table = rgb_bh1745_id,
};

static int __init rgb_bh1745_init(void)
{
	return i2c_add_driver(&rgb_bh1745_driver);
}

static void __exit rgb_bh1745_exit(void)
{
	if (rgb_bh1745_workqueue) {
		destroy_workqueue(rgb_bh1745_workqueue);
		rgb_bh1745_workqueue = NULL;
	}

	i2c_del_driver(&rgb_bh1745_driver);
}

MODULE_DESCRIPTION("BH1745 ambient light sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(rgb_bh1745_init);
module_exit(rgb_bh1745_exit);
