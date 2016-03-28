/*huawei kernel driver for rpr521*/
/*
 * rpr521.c - Linux kernel modules for ambient light + proximity sensor
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
#include <huawei_platform/sensor/rohm_rpr521.h>
#include <huawei_platform/sensor/hw_sensor_info.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/sensors.h>
#include <linux/wakelock.h>
#include <huawei_platform/touchscreen/hw_tp_common.h>
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif
#ifdef CONFIG_HUAWEI_DSM
#include 	<linux/dsm_pub.h>
#endif

#include <misc/app_info.h>

#include <linux/debugfs.h>

/*************** Global Data ******************/
/* parameter for als calculation */
#define COEFFICIENT               (4)
//read these coefficient from dtsi
unsigned long *data0_coefficient;//[COEFFICIENT] = {7768, 4388, 2627, 1971};
unsigned long *data1_coefficient;//[COEFFICIENT] = {5066, 2315, 1106, 687};
unsigned long *judge_coefficient;//[COEFFICIENT] = {1032, 1605, 1904, 2864};

static int origin_prox = 822;
#define RPR521_DRV_NAME		"rpr521"
#define DRIVER_VERSION		"1.0.0"

#define RPR521_REG_LEN 0xf 

/* Register Value define : CONTROL */

#define	SENSORS_I2C_SCL	909
#define	SENSORS_I2C_SDA	908
#define RPR521_I2C_RETRY_COUNT		3 	/* Number of times to retry i2c */ 
#define RPR521_I2C_RETRY_TIMEOUT	1	/* Timeout between retry (miliseconds) */

#define RPR521_I2C_BYTE 0
#define RPR521_I2C_WORD 1
/*keep 400ms wakeup after the ps report the far or near state*/
#define PS_WAKEUP_TIME 700
#define DISTINCTION_TP  1
/*If diff_tp valu of true,distinction tp,otherwise,no distinction tp */
static bool diff_tp = false;
/*prase_parameter vlue of true, call sensor_parse_parameter */
static bool  prase_parameter = true;
/*it is like min_proximity_value in 8x12 and 8930,to adjust the dynamic proximity ditance*/
/*del Invalid global variables*/
u8 gain0 = 2;
u8 gain1 = 2;
u16 atime = 100;
/*delete the wakelock,so system can sleep when the device is in the calling*/
/*pls parameters,it is still different for every devices*/
static uint32_t rpr521_pwave_value = 100;
static uint32_t rpr521_pwindow_value = 200;
extern bool power_key_ps ;    //the value is true means powerkey is pressed, false means not pressed
/*Ambient light array of values*/
static unsigned int  lux_arr[] = {10, 20,30,50,75,200,400, 800, 1500, 3000,5000,8000,10000};
static unsigned long  jiffies_save= 0;
static unsigned char  i_save = 0;
/*Ambient light each enabled print log*/
static bool als_print = false;
/*dynamic debug mask to control log print,you can echo value to rpr521_debug to control*/
static int rpr521_debug_mask= 1;
module_param_named(rpr521_debug, rpr521_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);
module_param_named(rpr521_pwindow, rpr521_pwindow_value, int, S_IRUGO | S_IWUSR | S_IWGRP);
module_param_named(rpr521_pwave, rpr521_pwave_value, int, S_IRUGO | S_IWUSR | S_IWGRP);

#define RPR521_ERR(x...) do {\
    if (rpr521_debug_mask >=0) \
        printk(KERN_ERR x);\
    } while (0)
/*KERNEL_HWFLOW is for radar using to control all the log of devices*/
#define RPR521_INFO(x...) do {\
    if (rpr521_debug_mask >=0) \
        printk(KERN_ERR x);\
    } while (0)
    #define RPR521_DEBUG(x...) do {\
    if (rpr521_debug_mask >=1) \
        printk(KERN_ERR x);\
    } while (0)
#define RPR521_FLOW(x...) do {\
    if (KERNEL_HWFLOW && (rpr521_debug_mask >=2)) \
        printk(KERN_ERR x);\
    } while (0)

/*delete it, no use check_type*/

/*
 * Structs
 */
 struct ls_test_excep{
	int i2c_scl_val;		/* when i2c transfer err, read the gpio value*/
	int i2c_sda_val;
	int vdd_mv;
	int vdd_status;
	int vio_mv;
	int vio_status;
	int i2c_err_num;
	int excep_num;
	atomic_t ps_enable_flag;
	int ps_enable_cnt;
	int ps_report_flag;		/*check if this is one irq occur or not after enable ps */
	int last_high_threshold;
	int last_low_threshold;
	int cur_high_threshold;
	int cur_low_threshold;
	/*delete it, no use check_type*/
	int irq_val;
#ifdef CONFIG_HUAWEI_DSM
	struct mutex dsm_lock;
#endif
	char *reg_buf;
};
struct rpr521_data {
	struct i2c_client *client;
	/*to protect the i2c read and write operation*/
	struct mutex update_lock;
	/*to protect only one thread to control the device register*/
	struct mutex single_lock;
	struct work_struct	dwork;		/* for PS interrupt */
	struct work_struct	als_dwork;	/* for ALS polling */
	struct delayed_work    powerkey_work;
	struct input_dev *input_dev_als;
	struct input_dev *input_dev_ps;

	/* regulator data */
	bool power_on;
	struct regulator *vdd;
	struct regulator *vio;

	/* pinctrl data*/
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
	/*sensor class for als and ps*/
	struct sensors_classdev als_cdev;
	struct sensors_classdev ps_cdev;
	struct rpr521_platform_data *platform_data;
#ifdef CONFIG_HUAWEI_DSM
	/*for capture the i2c and other errors*/
	struct ls_test_excep ls_test_exception;
	struct delayed_work dsm_work;
	struct delayed_work dsm_irq_work;
#endif
	int irq;
	int count;		/* the interrupt counter*/
	/*hrtimer is removed from platform struct to here*/
	struct hrtimer timer;
	/*wake lock for not losing ps event reporting*/
	struct wake_lock ps_report_wk;
	unsigned int enable;
	unsigned int ailt;
	unsigned int aiht;
	unsigned int pilt;
	unsigned int piht;
	unsigned int pers;
	unsigned int config;
	unsigned int control;

	/* control flag from HAL */
	unsigned int enable_ps_sensor;
	unsigned int enable_als_sensor;
	/*to record the open or close state of als before suspend*/
	unsigned int enable_als_state;
	/*delete it,we use data->ps_detection to save ps close or far state*/

	/* PS parameters */
	unsigned int ps_min_threshold; 	/*it is the min_proximity_value */
	unsigned int ps_detection;		/* 5 = near-to-far; 0 = far-to-near */
	unsigned int ps_data;/* to store PS data,it is pdata */

	/* ALS parameters */
	unsigned int als_threshold_l;	/* low threshold */
	unsigned int als_threshold_h;	/* high threshold */
	unsigned int als_data;		/* to store ALS data from CH0 or CH1*/
	int als_prev_lux;		/* to store previous lux value */
	unsigned int als_poll_delay;	/* needed for light sensor polling : micro-second (us) */
	bool device_exist;
};

static struct sensors_classdev sensors_light_cdev = {
	.name = "rpr521-light",
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
static struct sensors_classdev sensors_proximity_cdev = {
	.name = "rpr521-proximity",
	.vendor = "rohm",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "1",
	.resolution = "1.0",
	.sensor_power = "3",
	.min_delay = 0, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
static int sensor_parse_dt(struct device *,struct rpr521_platform_data *);
static int sensor_parse_parameter(struct device *);
/*
 * Global data
 */
/*delete two global data to avoid potencial risks*/
/* Proximity sensor use this work queue to report data */
static struct workqueue_struct *rpr521_workqueue = NULL;
/*changeable als gain and ADC time,we don't use*/

static int cur_pthreshold_h=0;
static int cur_pthreshold_l=0;

static int far_init=549;
static int near_init=550;
static int als_polling_count=0;
/*
 * Management functions,they are used to set registers in booting and enable
 */
/*init the register of device function for probe and every time the chip is powered on*/
static int rpr521_init_client(struct i2c_client *client);
static void operate_irq(struct rpr521_data *data, int enable, bool sync);
static int rpr521_i2c_read(struct i2c_client*client, u8 reg,bool flag);
static ssize_t rpr521_print_reg_buf(struct device *dev,
			struct device_attribute *attr, char *buf);
static ssize_t rpr521_write_reg(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count);

#ifdef CONFIG_HUAWEI_DSM

static struct dsm_client *rpr521_lps_dclient = NULL;
#define CLIENT_NAME_LPS_RPR			"dsm_lps_rpr"

/* dsm client for lp-sensor */
static struct dsm_dev dsm_lps_rpr = {
	.name 		= CLIENT_NAME_LPS_RPR,		// dsm client name
	.fops 		= NULL,						// options
	.buff_size 	= DSM_SENSOR_BUF_MAX,			// buffer size
};


static struct device_attribute rpr_show_regs =
		__ATTR(dump_reg, 0440, rpr521_print_reg_buf, rpr521_write_reg);

static int rpr_dsm_report_i2c(struct rpr521_data *data);
static int rpr_dsm_report_err(int errno,struct rpr521_data *data);

static void rpr_dsm_read_regs(struct rpr521_data *data)
{
	struct ls_test_excep *excep = &data->ls_test_exception;

	/*
	* read all regs to buf
	*/
	rpr521_print_reg_buf(&data->client->dev,&rpr_show_regs, excep->reg_buf);
}

static void rpr_dsm_save_threshold(struct rpr521_data *data, int high, int low)
{
	struct ls_test_excep *excep = &data->ls_test_exception;

	mutex_lock(&excep->dsm_lock);
	excep->last_high_threshold = high;
	excep->last_low_threshold = low;
	mutex_unlock(&excep->dsm_lock);
}

/*************************************************
  Function:        rpr_dsm_irq_excep_work
  Description:    this func is to report dsm err, no irq occured
                       after ps enabled
  Input:            work
  Output:          none
  Return:          0
*************************************************/

static void rpr_dsm_irq_excep_work(struct work_struct *work)
{
	struct rpr521_data *data =
		container_of((struct delayed_work *)work, struct rpr521_data, dsm_irq_work);
	struct ls_test_excep *excep = &data->ls_test_exception;
	if(!excep->ps_report_flag){
		/*
		* report dsm err, no irq occured after ps enabled.
		*/
		rpr_dsm_report_err(DSM_LPS_ENABLED_IRQ_ERROR,data);
	}
}

static void rpr_dsm_excep_work(struct work_struct *work)
{
	struct rpr521_data *data =
		container_of((struct delayed_work *)work, struct rpr521_data, dsm_work);
	struct i2c_client *client = data->client;
	
	int high_threshold;
	int low_threshold;
	/*
	*	read high and low threshold,  save them.
	*/
	low_threshold = rpr521_i2c_read(client,REG_PSTL,RPR521_I2C_WORD);
	high_threshold = rpr521_i2c_read(client,REG_PSTH,RPR521_I2C_WORD);
	rpr_dsm_save_threshold(data, high_threshold, low_threshold);

	/*
	* report dsm err, high and low threshold don't changed after ps irq.
	*/
	rpr_dsm_report_err(DSM_LPS_THRESHOLD_ERROR,data);
}

static void rpr_dsm_no_update_threhold_check(struct rpr521_data *data)
{
	schedule_delayed_work(&data->dsm_work, msecs_to_jiffies(10));
}

static void rpr_dsm_no_irq_check(struct rpr521_data *data)
{
	struct ls_test_excep *excep = &data->ls_test_exception;


	/* add this code segment to enable ps func
	*	irq gpio status
	*/
	atomic_set(&excep->ps_enable_flag, 1);

	/*delete it, no use check_type*/
	schedule_delayed_work(&data->dsm_irq_work, msecs_to_jiffies(120));
}


static void rpr_dsm_change_ps_enable_status(struct rpr521_data *data)
{
	/*
	*	add this code segment to report ps event.
	*/
	if (atomic_cmpxchg(&data->ls_test_exception.ps_enable_flag, 1, 0)){
		data->ls_test_exception.ps_report_flag = 1;
	}
}


/*
* if i2c transfer error, we check sda/scl value and regulator's value
*/
static int dump_i2c_exception_status(struct rpr521_data *data)
{
	int ret = 0;
	/* print pm status and i2c gpio status*/
	struct ls_test_excep *excep = &data->ls_test_exception;

	if (data->vdd == NULL) {
		return -ENXIO;
	}

	if (data->vio == NULL) {
		return -ENXIO;
	}

	/* read i2c_sda i2c_scl gpio value*/
	mutex_lock(&data->update_lock);
	excep->i2c_scl_val = gpio_get_value(data->platform_data->i2c_scl_gpio);
	excep->i2c_sda_val = gpio_get_value(data->platform_data->i2c_sda_gpio);
	mutex_unlock(&data->update_lock);

	/* get regulator's status*/
	excep->vdd_status = regulator_is_enabled(data->vdd);
	if(excep->vdd_status < 0){
		RPR521_ERR("%s,line %d:regulator_is_enabled vdd failed\n",__func__,__LINE__);
	}
	excep->vio_status = regulator_is_enabled(data->vio);
	if(excep->vio_status < 0){
		RPR521_ERR("%s,line %d:regulator_is_enabled vio failed\n",__func__,__LINE__);
	}

	/* get regulator's value*/
	excep->vdd_mv = regulator_get_voltage(data->vdd)/1000;
	if(excep->vdd_mv < 0){
		RPR521_ERR("%s,line %d:regulator_get_voltage vdd failed\n",__func__,__LINE__);
	}

	excep->vio_mv = regulator_get_voltage(data->vio)/1000;
	if(excep->vio_mv < 0){
		RPR521_ERR("%s,line %d:regulator_get_voltage vio failed\n",__func__,__LINE__);
	}

	/* report i2c err info */
	ret = rpr_dsm_report_err(DSM_LPS_I2C_ERROR,data);

	RPR521_INFO("%s,line %d:i2c_scl_val=%d,i2c_sda_val=%d,vdd = %d, vdd_status = %d\n"
			"vio=%d, vio_status=%d, excep_num=%d, i2c_err_num=%d",__func__,__LINE__
			,excep->i2c_scl_val,excep->i2c_sda_val,excep->vdd_mv,excep->vdd_status
			,excep->vio_mv,excep->vio_status,excep->excep_num,excep->i2c_err_num);

	excep->i2c_err_num = 0;

	return ret;

}
static void rpr_report_i2c_info(struct rpr521_data* data, int ret)
{
	data->ls_test_exception.i2c_err_num = ret;
	dump_i2c_exception_status(data);
}

static int rpr_dsm_init(struct rpr521_data *data)
{
	rpr521_lps_dclient = dsm_register_client(&dsm_lps_rpr);
	if (!rpr521_lps_dclient) {
		RPR521_ERR("%s@%d register dsm rpr521_lps_dclient failed!\n",__func__,__LINE__);
		return -ENOMEM;
	}

	rpr521_lps_dclient->driver_data = data;

	data->ls_test_exception.reg_buf = kzalloc(512, GFP_KERNEL);
	if(!data->ls_test_exception.reg_buf){
		RPR521_ERR("%s@%d alloc dsm reg_buf failed!\n",__func__,__LINE__);
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&data->dsm_work, rpr_dsm_excep_work);
	INIT_DELAYED_WORK(&data->dsm_irq_work, rpr_dsm_irq_excep_work);

	mutex_init(&data->ls_test_exception.dsm_lock);

	return 0;
}

static void rpr_dsm_exit(void)
{
	dsm_unregister_client(rpr521_lps_dclient,&dsm_lps_rpr);
}

static int rpr_dsm_report_i2c(struct rpr521_data *data)
{
	struct ls_test_excep *excep = &data->ls_test_exception;

	ssize_t size = 0;

	size = dsm_client_record(rpr521_lps_dclient,
				"i2c_scl_val=%d,i2c_sda_val=%d,vdd = %d, vdd_status = %d\n"
				"vio=%d, vio_status=%d, excep_num=%d, i2c_err_num=%d\n"
				,excep->i2c_scl_val,excep->i2c_sda_val,excep->vdd_mv,excep->vdd_status
				,excep->vio_mv,excep->vio_status,excep->excep_num,excep->i2c_err_num);

	/*if device is not probe successfully or client is null, don't notify dsm work func*/
	if(data->device_exist == false || rpr521_lps_dclient == NULL){
		return -ENODEV;
	}

	return size;

}

static int rpr_dsm_report_wrong_irq(struct rpr521_data *data)
{
	struct ls_test_excep *excep = &data->ls_test_exception;
	int irq_gpio = data->platform_data->irq_gpio;
	ssize_t size;


	/*
	*	read  regs and irq gpio
	*/
	rpr_dsm_read_regs(data);
	excep->irq_val = gpio_get_value(irq_gpio);


	size = dsm_client_record(rpr521_lps_dclient,"irq_pin = %d\n regs:%s \n",
		excep->irq_val, excep->reg_buf);

	RPR521_ERR("dsm error-> irq_pin = %d\n regs:%s\n",
				excep->irq_val, excep->reg_buf);


	return 0;

}

static int rpr_dsm_report_no_irq(struct rpr521_data *data)
{
	ssize_t size = 0;

	size = rpr_dsm_report_wrong_irq(data);

	return size;
}

static int rpr_dsm_report_not_change_threshold(struct rpr521_data *data)
{
	struct ls_test_excep *excep = &data->ls_test_exception;
	ssize_t size = 0;

	rpr_dsm_read_regs(data);
	excep->irq_val = gpio_get_value(data->platform_data->irq_gpio);

	size = dsm_client_record(rpr521_lps_dclient, "irq_pin = %d high_thrhd = %d, low_thrhd = %d\n regs:%s",
							 excep->irq_val, excep->last_high_threshold,
							 excep->last_low_threshold, excep->reg_buf);

	RPR521_ERR("dsm error->""irq_pin = %d high_thrhd = %d, low_thrhd = %d\n regs:%s",
							 excep->irq_val, excep->last_high_threshold,
							 excep->last_low_threshold, excep->reg_buf);

	return size;
}
static int  rpr521_dsm_report_flag(struct rpr521_data *data)
{
	struct ls_test_excep *excep = &data->ls_test_exception;
	ssize_t size = 0;
	rpr_dsm_read_regs(data);
	size = dsm_client_record(rpr521_lps_dclient," regs:%s, enable_ps_sensor = %d\n",
		excep->reg_buf,data->enable_ps_sensor);
	RPR521_ERR("dsm error->""regs:%s,enable_ps_sensor = %d\n",
		excep->reg_buf,data->enable_ps_sensor);
	return size;
}

static int rpr521_dsm_report_ps_threshold(struct rpr521_data *data)
{
	struct ls_test_excep *excep = &data->ls_test_exception;
	ssize_t size = 0;
	rpr_dsm_read_regs(data);
	size = dsm_client_record(rpr521_lps_dclient," regs:%s,pilt = %d,piht = %d\n",
		excep->reg_buf,data->pilt,data->piht);
	RPR521_ERR("dsm error->""regs:%s,pilt = %d,piht = %d\n",
		excep->reg_buf,data->pilt,data->piht);
	return size;
}

static void rpr521_dsm_threhold_size_check(struct rpr521_data *data)
{
	//If pilt >= piht,repot threhold error
	if(data->pilt >= data->piht)
	{
		RPR521_ERR("%s,far threhold greater than near threhold\n",__func__);
		rpr_dsm_report_err(DSM_LPS_THRESHOLD_SIZE_ERROR,data);
	}

}

static void check_hardware_software_flag(struct rpr521_data *data )
{
	int ret = 0;
	ret = rpr521_i2c_read(data->client, REG_MODECONTROL, RPR521_I2C_BYTE);
	if(ret < 0)
	{
		RPR521_ERR("%s:read reg fail\n",__func__);
		return;
	}
	//Check ps enable bit
	if(1 == data ->enable_ps_sensor  &&  (ret  & ( 1 << 6)) != 0x40)
	{
		RPR521_ERR("%s:Software enable and hardware enable mismatch\n",__func__);
		rpr_dsm_report_err(DSM_LPS_ENABLED_ERROR,data);
	}
}
static int rpr_dsm_report_err(int errno,struct rpr521_data *data)
{
	int size = 0;

	if(dsm_client_ocuppy(rpr521_lps_dclient))
	{
		/* buffer is busy */
		RPR521_ERR("%s: buffer is busy!, errno = %d\n", __func__,errno);
		return -EBUSY;
	}

	RPR521_INFO("dsm error, errno = %d \n", errno);

	switch(errno){
		case DSM_LPS_I2C_ERROR:
			size = rpr_dsm_report_i2c(data);
			break;

		case DSM_LPS_WRONG_IRQ_ERROR:
			size = rpr_dsm_report_wrong_irq(data);
			break;

		case DSM_LPS_THRESHOLD_ERROR:
			size = rpr_dsm_report_not_change_threshold(data);
			break;

		case DSM_LPS_ENABLED_IRQ_ERROR:
			size = rpr_dsm_report_no_irq(data);
			break;
		case DSM_LPS_ENABLED_ERROR:
			size = rpr521_dsm_report_flag(data);
			break;
		case DSM_LPS_THRESHOLD_SIZE_ERROR:
			size = rpr521_dsm_report_ps_threshold(data);
			break;
		default:

			break;

	}
	if( size  > -1)
	{
		dsm_client_notify(rpr521_lps_dclient, errno);
		RPR521_ERR("%s:line:%d,size = %d\n",__func__,__LINE__,size);
	}
	return size;
}

#endif



/*we use the unified the function for i2c write and read operation*/
static int rpr521_i2c_write(struct i2c_client*client, u8 reg, u16 value,bool flag)
{
	int err,loop;

	struct rpr521_data *data = i2c_get_clientdata(client);

	loop = RPR521_I2C_RETRY_COUNT;
	/*we give three times to repeat the i2c operation if i2c errors happen*/
	while(loop) {
		mutex_lock(&data->update_lock);
		/*0 is i2c_smbus_write_byte_data,1 is i2c_smbus_write_word_data*/
		if(flag == RPR521_I2C_BYTE)
		{
			err = i2c_smbus_write_byte_data(client, reg, (u8)value);
		}
		else if(flag == RPR521_I2C_WORD)
		{
			err = i2c_smbus_write_word_data(client, reg, value);
		}
		else
		{
			RPR521_ERR("%s,line %d:attention: i2c write wrong flag",__func__,__LINE__);
			mutex_unlock(&data->update_lock);
			return -EINVAL;
		}
		mutex_unlock(&data->update_lock);
		if(err < 0){
			loop--;
			mdelay(RPR521_I2C_RETRY_TIMEOUT);
		}
		else
			break;
	}
	/*after three times,we print the register and regulator value*/
	if(loop == 0){
		RPR521_ERR("%s,line %d:attention:i2c write err = %d",__func__,__LINE__,err);
#ifdef CONFIG_HUAWEI_DSM
		rpr_report_i2c_info(data,err);
#endif
	}

	return err;
}

static int rpr521_i2c_read(struct i2c_client*client, u8 reg,bool flag)
{
	int err,loop;

	struct rpr521_data *data = i2c_get_clientdata(client);

	loop = RPR521_I2C_RETRY_COUNT;
	/*we give three times to repeat the i2c operation if i2c errors happen*/
	while(loop) {
		mutex_lock(&data->update_lock);
		/*0 is i2c_smbus_read_byte_data,1 is i2c_smbus_read_word_data*/
		if(flag == RPR521_I2C_BYTE)
		{
			err = i2c_smbus_read_byte_data(client, reg); 
		}
		else if(flag == RPR521_I2C_WORD)
		{
			err = i2c_smbus_read_word_data(client, reg);
		}
		else
		{
			RPR521_ERR("%s,line %d:attention: i2c read wrong flag",__func__,__LINE__);
			mutex_unlock(&data->update_lock);
			return -EINVAL;
		}
		mutex_unlock(&data->update_lock);
		if(err < 0){
			loop--;
			mdelay(RPR521_I2C_RETRY_TIMEOUT);
		}
		else
			break;
	}
	/*after three times,we print the register and regulator value*/
	if(loop == 0){
		RPR521_ERR("%s,line %d:attention: i2c read err = %d,reg=0x%x",__func__,__LINE__,err,reg);
#ifdef CONFIG_HUAWEI_DSM
		rpr_report_i2c_info(data,err);
#endif
	}

	return err;
}

//grace modify in 2014.7.31 begin
static int rpr521_i2c_read_buf(struct i2c_client*client, u8 reg, u8 length, u8 *buf)
{
	int err,loop;
	u8 *values = buf;

	struct rpr521_data *data = i2c_get_clientdata(client);

	loop = RPR521_I2C_RETRY_COUNT;
	/*we give three times to repeat the i2c operation if i2c errors happen*/
	while(loop) {
		mutex_lock(&data->update_lock);
		
		err = i2c_smbus_read_i2c_block_data(client, reg, length, values);
		
		mutex_unlock(&data->update_lock);
		if(err !=  length){
			loop--;
			mdelay(RPR521_I2C_RETRY_TIMEOUT);
		}
		else
			break;
	}
	/*after three times,we print the register and regulator value*/
	if(loop == 0){
		RPR521_ERR("%s,line %d:attention: i2c read err = %d,reg=0x%x",__func__,__LINE__,err,reg);
#ifdef CONFIG_HUAWEI_DSM
		dump_i2c_exception_status(data);
#endif
	}

	return err;
}
//grace modify in 2014.7.31 end

/*
*	print the registers value with proper format
*/
static int dump_reg_buf(struct rpr521_data *data,char *buf, int size,int enable)
{
	int i=0;

	mutex_lock(&data->update_lock);

	if(enable)
		printk("[enable]");
	else
		printk("[disable]");
	printk(" reg_buf= ");
	for(i = 0;i < size; i++){
		printk("0x%2x  ",buf[i]);
	}
	mutex_unlock(&data->update_lock);

	printk("\n");
	return 0;
}
static int rpr521_regs_debug_print(struct rpr521_data *data,int enable)
{
	int i=0;
	char reg_buf[RPR521_REG_LEN];
	u8 reg = 0;
	struct i2c_client *client = data->client;

	/* read registers[0x0~0x1a] value*/
	for(i = 0; i < RPR521_REG_LEN; i++ )
	{
		reg = 0x40+i;
		reg_buf[i] = rpr521_i2c_read(client,reg,RPR521_I2C_BYTE);

		if(reg_buf[i] <0){
			RPR521_ERR("%s,line %d:read %d reg failed\n",__func__,__LINE__,i);
			return reg_buf[i] ;
		}
	}

	/* print the registers[0x0~0x1a] value in proper format*/
	dump_reg_buf(data,reg_buf,RPR521_REG_LEN,enable);

	return 0;
}



static int rpr521_set_enable(struct i2c_client *client, int enable)
{
	int ret;

	ret = rpr521_i2c_write(client, REG_MODECONTROL, enable,RPR521_I2C_BYTE);
	if (ret < 0){
		RPR521_ERR("%s,line %d:i2c error,enable = %d\n",__func__,__LINE__,enable);
		return ret;
	}
	/*remove it,we have save the value of enable in another way*/
	RPR521_FLOW("%s,line %d:rpr521 enable = %d\n",__func__,__LINE__,enable);
	return ret;
}

static int rpr521_set_pilt(struct i2c_client *client, int threshold)
{
	struct rpr521_data *data = i2c_get_clientdata(client);
	int ret;

	ret = rpr521_i2c_write(client,REG_PSTL, threshold,RPR521_I2C_WORD);
	if (ret < 0){
		RPR521_ERR("%s,line %d:i2c error,threshold = %d\n",__func__,__LINE__,threshold);
		return ret;
	}

	data->pilt = threshold;
	RPR521_INFO("%s,line %d:set rpr521 pilt =%d\n", __func__, __LINE__,threshold);

	return ret;
}

static int rpr521_set_piht(struct i2c_client *client, int threshold)
{
	struct rpr521_data *data = i2c_get_clientdata(client);
	int ret;

	ret = rpr521_i2c_write(client,REG_PSTH, threshold,RPR521_I2C_WORD);
	if (ret < 0){
		RPR521_ERR("%s,line %d:i2c error,threshold = %d\n",__func__,__LINE__,threshold);
		return ret;
	}

	data->piht = threshold;
	RPR521_INFO("%s,line %d:set rpr521 piht =%d\n", __func__,__LINE__,threshold);
	return ret;
}

static int rpr521_set_pers(struct i2c_client *client, int pers)
{
	struct rpr521_data *data = i2c_get_clientdata(client);
	int ret;

	ret = rpr521_i2c_write(client,REG_PERSISTENCE, pers,RPR521_I2C_BYTE);
	if (ret < 0){
		RPR521_ERR("%s,line %d:i2c error,pers = %d\n",__func__,__LINE__,pers);
		return ret;
	}

	data->pers = pers;
	RPR521_FLOW("%s,line %d:rpr521 pers = %d\n",__func__,__LINE__,pers);
	return ret;
}

static int rpr521_set_config(struct i2c_client *client, int config)
{
	struct rpr521_data *data = i2c_get_clientdata(client);
	int ret;

	ret = rpr521_i2c_write(client,REG_INTERRUPT, config,RPR521_I2C_BYTE);
	if (ret < 0){
		RPR521_ERR("%s,line %d:i2c error,config = %d\n",__func__,__LINE__,config);
		return ret;
	}

	data->config = config;
	RPR521_FLOW("%s,line %d:rpr521 config = %d\n",__func__,__LINE__,config);
	return ret;
}

static int rpr521_set_control(struct i2c_client *client, int control)
{
	struct rpr521_data *data = i2c_get_clientdata(client);
	int ret;

	ret = rpr521_i2c_write(client,REG_ALSPSCONTROL , control,RPR521_I2C_BYTE);
	if (ret < 0){
		RPR521_ERR("%s,line %d:i2c error,control = %d\n",__func__,__LINE__,control);
		return ret;
	}

	data->control = control;
	RPR521_FLOW("%s,line %d:rpr521 control = %d\n",__func__,__LINE__,control);
	return ret;
}

static void rpr521_ps_report_event(struct i2c_client *client, int irq_status)
{
	int ch0data = 0;
	int sunlight_detect = 0;
	/*PS interrupt generation flags*/
	int ps_int_occur = 0;
	struct rpr521_data *data = i2c_get_clientdata(client);

	int ret;
	unsigned char ps_info[3]; //grace modify in 2014.7.31
	
	ch0data = rpr521_i2c_read(client, REG_ALSDATA0_LSB, RPR521_I2C_WORD);
	if(ch0data < 0)
	{
		RPR521_ERR("%s,line %d:rpr521_i2c_read ch0data err:%d\n", __func__, __LINE__, ch0data);
	}
	else if(ch0data > RPR521_SUNLIGHT_CHODATA)
	{
		sunlight_detect = 0x1;
	}

	RPR521_INFO("%s, line %d: read irq status reg data: 0x%x; ch0data: %d, sunlight_detect:%d\n",
		__func__, __LINE__, irq_status, ch0data, sunlight_detect);	

	if((irq_status >= 0) && ((irq_status & (PS_INT_MASK)) == PS_INT_MASK))
	{
		ret = rpr521_i2c_read_buf(client, REG_PERSISTENCE, 3, ps_info);
		if( ret < 0 )
		{
		/* the number "200" is a value to make sure there is a valid value */
			data->ps_data = 200 ;
			RPR521_ERR("%s, line %d: pdate<0, reset to %d\n", __func__, __LINE__, data->ps_data);
		}else{
			data->ps_data = (ps_info[2] << 8) | ps_info[1];
			ps_int_occur = 1;
			RPR521_FLOW("%s, line %d:read ps data->ps_data:%d, ps_int_occur:%d\n",  
				__func__, __LINE__, data->ps_data, ps_int_occur);
		}
		/*remove to wrong interrupts branch because when sunlight_detect is equal to 1, threhold would not update*/
	}
	else
	{
			/* the number "200" is a value to make sure there is a valid value */
			data->ps_data = 200 ;
			RPR521_FLOW("%s, line %d:ps intruppt not occur data->ps_data:%d\n",  __func__, __LINE__, data->ps_data);
	}
	RPR521_FLOW("%s,line %d:rpr521 ps_data=%d\n",__func__,__LINE__,data->ps_data);
	RPR521_FLOW("%s,line %d:RPR521 ps_data=%d,ps_min_threshold=%d, ps_int_occur:%d\n",__func__,__LINE__,
		data->ps_data,data->ps_min_threshold, ps_int_occur);
	if (ps_info[0] >> 6 == 0)
	{
		if (((data->ps_data + rpr521_pwave_value) < (data->ps_min_threshold)) && ps_int_occur)
		{
			data->ps_min_threshold = data->ps_data + rpr521_pwave_value;
			ret = rpr521_i2c_write(client,REG_PSTL,data->ps_min_threshold,RPR521_I2C_WORD);
			ret += rpr521_i2c_write(client,REG_PSTH,data->ps_min_threshold + rpr521_pwindow_value,RPR521_I2C_WORD);
			if (ret < 0)
			{
				RPR521_ERR("%s,line %d:data->pilt = %d,data->piht=%d, i2c wrong\n",__func__,__LINE__,data->pilt,data->piht);
				goto exit;
			}
			data->pilt = data->ps_min_threshold;
			data->piht = data->ps_min_threshold + rpr521_pwindow_value;
			RPR521_INFO("%s,line %d:change threshoid,data->ps_data =%d,data->pilt=%d,data->piht=%d,\n", __func__,__LINE__,data->ps_data, data->pilt, data->piht);
			if(data->pilt != cur_pthreshold_l || data->piht != cur_pthreshold_h)
			{
				cur_pthreshold_h = data->piht;
				cur_pthreshold_l = data->pilt;
				RPR521_FLOW("%s,line %d:data->ps_data=%d data->pilt=%d data->piht=%d\n", __func__, __LINE__, data->ps_data, data->pilt, data->piht);
			}
		}
	}
	data->pilt = rpr521_i2c_read(client,REG_PSTL,RPR521_I2C_WORD);
	data->piht = rpr521_i2c_read(client,REG_PSTH,RPR521_I2C_WORD);
	if (data->pilt < 0 || data->piht < 0){
		RPR521_ERR("%s,line %d:data->pilt = %d,data->piht=%d,read i2c wrong\n",__func__,__LINE__,data->pilt,data->piht);
		goto exit;
	}
#ifdef CONFIG_HUAWEI_DSM
	rpr521_dsm_threhold_size_check(data);
#endif
	if ((data->ps_data >= data->piht) && (!sunlight_detect) && ps_int_occur) {
		/* far-to-near detected */
		data->ps_detection = RPR521_CLOSE_FLAG;

		/* FAR-to-NEAR detection */
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, RPR521_CLOSE_FLAG);
		input_sync(data->input_dev_ps);

#ifdef CONFIG_HUAWEI_DSM
		rpr_dsm_change_ps_enable_status(data);
#endif
		RPR521_INFO("%s,line %d:PROXIMITY close event, data->ps_data =%d,data->pilt=%d,data->piht=%d,\n", __func__,__LINE__,data->ps_data, data->pilt, data->piht);

		ret = rpr521_i2c_write(client,REG_PSTL,data->ps_min_threshold,RPR521_I2C_WORD);
		ret += rpr521_i2c_write(client,REG_PSTH, REG_PSTH_MAX, RPR521_I2C_WORD);
		if (ret < 0){
			RPR521_ERR("%s,line %d:data->pilt = %d,data->piht=%d, i2c wrong\n",__func__,__LINE__,data->pilt,data->piht);
			goto exit;
		}
		data->pilt = data->ps_min_threshold;
		data->piht = REG_PSTH_MAX;
		RPR521_INFO("%s,line %d:change threshoid,data->ps_data =%d,data->pilt=%d,data->piht=%d,\n", __func__,__LINE__,data->ps_data, data->pilt, data->piht);
		if(data->pilt != cur_pthreshold_l || data->piht != cur_pthreshold_h)
		{
			cur_pthreshold_h = data->piht;
			cur_pthreshold_l = data->pilt;
			RPR521_FLOW("%s,line %d:data->ps_data=%d data->pilt=%d data->piht=%d\n", __func__, __LINE__, data->ps_data, data->pilt, data->piht);
		}
	} else if (((data->ps_data <= data->pilt)) ||(sunlight_detect)) {
		/* near-to-far detected */
		data->ps_detection = RPR521_FAR_FLAG;

		/* NEAR-to-FAR detection */
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, RPR521_FAR_FLAG);
		input_sync(data->input_dev_ps);
#ifdef CONFIG_HUAWEI_DSM
		rpr_dsm_change_ps_enable_status(data);
#endif
		RPR521_INFO("%s,line %d:PROXIMITY far event, data->ps_data=%d,data->pilt=%d,data->piht=%d,\n", __func__,__LINE__,data->ps_data, data->pilt, data->piht);

		ret = rpr521_i2c_write(client,REG_PSTH,
				data->ps_min_threshold + rpr521_pwindow_value,RPR521_I2C_WORD); 
		ret += rpr521_i2c_write(client,REG_PSTL,0,RPR521_I2C_WORD); 
		if (ret < 0){
			RPR521_ERR("%s,line %d:data->pilt = %d,data->piht=%d, i2c wrong\n",__func__,__LINE__,data->pilt,data->piht);
			goto exit;
		}

		data->piht = data->ps_min_threshold + rpr521_pwindow_value;
		data->pilt  = 0;
		RPR521_INFO("%s,line %d:change threshoid,data->ps_data =%d,data->pilt=%d,data->piht=%d,\n", __func__,__LINE__,data->ps_data, data->pilt, data->piht);
		if(data->pilt != cur_pthreshold_l || data->piht != cur_pthreshold_h)
		{
			cur_pthreshold_h = data->piht;
			cur_pthreshold_l = data->pilt;
			RPR521_FLOW("%s:data->ps_data=%d data->pilt=%d data->piht=%d\n", __func__, data->ps_data, data->pilt, data->piht);
		}
		if(1 == sunlight_detect)
		{
			/*To avoid Always generated light interrupt*/
			msleep(300);
		}
	}
	else{
		RPR521_ERR("%s,line %d:data->ps_data=%d,data->pilt = %d,data->piht=%d,wrong interrupts\n",__func__,__LINE__,data->ps_data, data->pilt,data->piht);
#ifdef CONFIG_HUAWEI_DSM
		rpr_dsm_no_update_threhold_check(data);
#endif
	}
	return ;
exit:
	/*if i2c error happens,we report far event*/
	if(data->ps_detection == RPR521_CLOSE_FLAG)
	{
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, RPR521_FAR_FLAG);
		input_sync(data->input_dev_ps);
		data->ps_detection= RPR521_FAR_FLAG;
		RPR521_ERR("%s:i2c error happens, report far event, data->ps_data:%d\n", __func__,data->ps_data);
		if(1 == sunlight_detect)
		{
			/*To avoid Always generated light interrupt*/
			msleep(300);
		}
		return ;
	}
}


/******************************************************************************
 * NAME       : long_long_divider
 * FUNCTION   : calc divider of unsigned long long int or unsgined long
 * REMARKS    :
 *****************************************************************************/
static int long_long_divider(long long data, unsigned long base_divier, unsigned long *answer, unsigned long long *overplus)
{
    volatile long long divier;
    volatile long      unit_sft;

    if ((data < 0) || (base_divier == 0)) {
        *answer   = 0;
        *overplus = 0;
        return (CALC_ERROR);
    }

    divier = base_divier;
    if (data > MASK_LONG) {
        unit_sft = 0;
        while ((data > divier) && (divier > 0)) {
            unit_sft++;
            divier = divier << 1;
        }
        while ((data > base_divier) && (unit_sft > 0)) {
            if (data > divier) {
                *answer += 1 << unit_sft;
                data    -= divier;
            }
            unit_sft--;
            divier = divier >> 1;
        }
        *overplus = data;
    } else {
        *answer = (unsigned long)(data & MASK_LONG) / base_divier;
        /* calculate over plus and shift 16bit */
        *overplus = (unsigned long long)(data - (*answer * base_divier));
    }

    return (0);
}


/******************************************************************************
 * NAME       : calc_rohm_als_data
 * FUNCTION   : calculate illuminance data for rpr521
 * REMARKS    : final_data is 1000 times, which is defined as CUT_UNIT, of the actual lux value
 *****************************************************************************/
static int calc_rohm_als_data(unsigned short data0, unsigned short data1, unsigned char gain0, unsigned char gain1, unsigned short time) 
{
#define DECIMAL_BIT      (15)
#define JUDGE_FIXED_COEF (1000)
#define MAX_OUTRANGE    65535// (11357)
#define MAXRANGE_NMODE   (0xFFFF)
#define MAXSET_CASE      (4)

	int                result; //grace modified in 2014.4.2
	int                final_data;
	CALC_DATA          calc_data;
	CALC_ANS           calc_ans;
	unsigned long      calc_judge;
	unsigned char      set_case;
	unsigned long      div_answer;
	unsigned long long div_overplus;
	unsigned long long overplus;
	unsigned long      max_range;

	/* set the value of measured als data */
	calc_data.als_data0  = data0;
	calc_data.als_data1  = data1;
	calc_data.gain_data0 = gain0;

	/* set max range */
	if (calc_data.gain_data0 == 0) 
	{
		/* issue error value when gain is 0 */
		return (CALC_ERROR);
	}
	else
	{
		max_range = MAX_OUTRANGE / calc_data.gain_data0;
	}
	
	/* calculate data */
	if (calc_data.als_data0 == MAXRANGE_NMODE) 
	{
		calc_ans.positive = max_range;
		calc_ans.decimal  = 0;
	} 
	else 
	{
		/* get the value which is measured from power table */
		calc_data.als_time = time;
		if (calc_data.als_time == 0) 
		{
			/* issue error value when time is 0 */
			return (CALC_ERROR);
		}

		calc_judge = calc_data.als_data1 * JUDGE_FIXED_COEF;
		if (calc_judge < (calc_data.als_data0 * judge_coefficient[0])) 
		{
			set_case = 0;
		} 
		else if (calc_judge < (calc_data.als_data0 * judge_coefficient[1]))
		{
			set_case = 1;
		} 
		else if (calc_judge < (calc_data.als_data0 * judge_coefficient[2])) 
		{
			set_case = 2;
		}
		else if (calc_judge < (calc_data.als_data0 * judge_coefficient[3])) 
		{
			 set_case = 3;
		} 
		else
		{
			set_case = MAXSET_CASE;
		}
		calc_ans.positive = 0;
		if (set_case >= MAXSET_CASE) 
		{
			calc_ans.decimal = 0;	//which means that lux output is 0
		}
		else
		{
			calc_data.gain_data1 = gain1;
			if (calc_data.gain_data1 == 0) 
			{
				/* issue error value when gain is 0 */
				return (CALC_ERROR);
			}
			calc_data.data0      = (unsigned long long )(data0_coefficient[set_case] * calc_data.als_data0) * calc_data.gain_data1;
			calc_data.data1      = (unsigned long long )(data1_coefficient[set_case] * calc_data.als_data1) * calc_data.gain_data0;
			if(calc_data.data0 < calc_data.data1)	//In this case, data will be less than 0. As data is unsigned long long, it will become extremely big.
			{
				return (CALC_ERROR);
			}
			else
			{
				calc_data.data       = (calc_data.data0 - calc_data.data1);
			}
			calc_data.dev_unit   = calc_data.gain_data0 * calc_data.gain_data1 * calc_data.als_time * 10;	//24 bit at max (128 * 128 * 100 * 10)
			if (calc_data.dev_unit == 0) 
			{
				/* issue error value when dev_unit is 0 */
				return (CALC_ERROR);
			}

			/* calculate a positive number */
			div_answer   = 0;
			div_overplus = 0;

			result = long_long_divider(calc_data.data, calc_data.dev_unit, &div_answer, &div_overplus);
      			if (result == CALC_ERROR)
      			{
        			 return (result);
      			}
			calc_ans.positive = div_answer;
			/* calculate a decimal number */
			calc_ans.decimal = 0;
			overplus         = div_overplus;
			if (calc_ans.positive < max_range)
			{
				if (overplus != 0)
				{
					overplus     = overplus << DECIMAL_BIT;
					div_answer   = 0;
					div_overplus = 0;

					result = long_long_divider(overplus, calc_data.dev_unit, &div_answer, &div_overplus);
					if (result == CALC_ERROR)
      					{
         					return (result);
      					}
					calc_ans.decimal = div_answer;
				}
			}

			else
			{
				calc_ans.positive = max_range;
			}
		}
	}
	
	final_data = calc_ans.positive * CUT_UNIT + ((calc_ans.decimal * CUT_UNIT) >> DECIMAL_BIT);
					
	return (final_data);

#undef DECIMAL_BIT
#undef JUDGE_FIXED_COEF
#undef MAX_OUTRANGE
#undef MAXRANGE_NMODE
#undef MAXSET_CASE
}


/* delete rpr521_reschedule_work, we use queue_work to replase queue_delayed_work, because flush_delayed_work
   may cause system stop work */
/* ALS polling routine */
static void rpr521_als_polling_work_handler(struct work_struct *work)
{
	struct rpr521_data *data = container_of(work, struct rpr521_data,als_dwork);
	struct i2c_client *client=data->client;
	int ch0data, ch1data, pdata;
	int luxValue=0;
	
	unsigned char lux_is_valid=1;
	int  i;
	ch0data = rpr521_i2c_read(client,
			REG_ALSDATA0,RPR521_I2C_WORD); 
	ch1data = rpr521_i2c_read(client,
			REG_ALSDATA1,RPR521_I2C_WORD);
	pdata = rpr521_i2c_read(client,
			REG_PSDATA,RPR521_I2C_WORD);
	if(ch0data < 0 || ch1data < 0 || pdata < 0)
	{
		/* don't report, this is invalid lux value */
		lux_is_valid = 0;
		luxValue = data->als_prev_lux;
	}
	else
	{
		luxValue = calc_rohm_als_data(ch0data, ch1data, gain0, gain1, atime); 
	}

	if ((luxValue >= 0) && (luxValue!=CALC_ERROR)) //grace modify in 2014.7.31
	{

		luxValue = luxValue < RPR521_LUX_MAX? luxValue : RPR521_LUX_MAX;

		data->als_prev_lux = luxValue;
	}
	else
	{
		/* don't report, this is invalid lux value */
		lux_is_valid = 0;
		luxValue = data->als_prev_lux;
	}
	/*Every 30s  to print once log */
	if (time_after_eq(jiffies, jiffies_save + PRINT_LUX_TIME * HZ ))
	{
		jiffies_save = jiffies;
		RPR521_DEBUG("[ALS_PS]: the cycle luxValue = %d,ch0data = %d,ch1data = %d\n", luxValue,ch0data,ch1data);

	}
	else
	{
		for(i = 0;i<ARR_NUM;i++)
		{
			if(luxValue < lux_arr[i])
				break;

		}
		/*als value appears to jump or enable als to print log*/
		if( (i_save != i)  || (true == als_print ))
		{
			i_save = i;
			RPR521_DEBUG("[ALS_PS]: the skip  luxValue = %d,ch0data = %d,ch1data = %d,als_print = %d\n", luxValue,ch0data,ch1data,als_print);
			als_print  = false;
		}
	}
	/*remove it because we use other judge method to decide if pls close event is triggered by sunlight*/
	if( als_polling_count <5 )
	{
		if(luxValue == RPR521_LUX_MAX)
		{
			luxValue = luxValue - als_polling_count%2;
		}
		else
		{
			luxValue = luxValue + als_polling_count%2;
		}
		als_polling_count++;
	}
	if (lux_is_valid) {
		/* report the lux level */
		input_report_abs(data->input_dev_als, ABS_MISC, luxValue);
		input_sync(data->input_dev_als);
		RPR521_FLOW("%s,line %d:rpr521 lux=%d\n",__func__,__LINE__,luxValue);
	}

	if (sensorDT_mode)
	{
		als_data_count++;
	}

 	RPR521_FLOW("%s: line:%d rpr521 light ch0data=%d, ch1data=%d, luxValue=%d, mode=%x\n", __func__, __LINE__, ch0data, ch1data,luxValue,rpr521_i2c_read(client,
			REG_ALSPSCONTROL,RPR521_I2C_BYTE));
	/* restart timer */
	/* start a work after 200ms */
	if (0 != hrtimer_start(&data->timer,
							ktime_set(0, data->als_poll_delay * 1000000), HRTIMER_MODE_REL) )
	{
		RPR521_ERR("%s: hrtimer_start fail! nsec=%d\n", __func__, data->als_poll_delay);
	}
}

/*****************************************************************
Parameters    :  timer
Return        :  HRTIMER_NORESTART
Description   :  hrtimer_start call back function,
				 use to report als data
*****************************************************************/
static enum hrtimer_restart rpr521_als_timer_func(struct hrtimer *timer)
{
	struct rpr521_data* data = container_of(timer,struct rpr521_data,timer); //wanghang modify
	queue_work(rpr521_workqueue, &data->als_dwork);  //wanghang modify
	return HRTIMER_NORESTART;
}

/* PS interrupt routine */

static void rpr521_work_handler(struct work_struct *work)
{
	struct rpr521_data *data = container_of(work, struct rpr521_data, dwork);
	struct i2c_client *client=data->client;
	int status;
	int control,mode,pers; 
	mutex_lock(&data->single_lock);
	status = rpr521_i2c_read(client, REG_INTERRUPT,RPR521_I2C_BYTE);
	
	RPR521_INFO("%s: line:%d rpr521 interrupt handler status =%x\n", __func__, __LINE__, status); 
	
	if(rpr521_debug_mask > 1)
	{
		control=rpr521_i2c_read(client, REG_ALSPSCONTROL,RPR521_I2C_BYTE);
		mode = rpr521_i2c_read(client, REG_MODECONTROL,RPR521_I2C_BYTE);
		pers= rpr521_i2c_read(client, REG_PERSISTENCE,RPR521_I2C_BYTE);

		RPR521_FLOW("%s,line %d:status = 0x%x\n",__func__,__LINE__,status);
		RPR521_FLOW("%s,line %d:control = 0x%x,mode=0x%x,pers=0x%x\n",__func__,__LINE__,control,mode,pers);
	}
	
	if (((status & PS_INT_MASK) == PS_INT_MASK) || ((status  & ALS_INT_MASK) == ALS_INT_MASK)) { 
		/* only PS is interrupted */
		RPR521_FLOW("%s,line %d:only PLS is detected.\n",__func__,__LINE__);
		rpr521_ps_report_event(client, status);


	} else{
		RPR521_ERR("%s,line %d:wrong interrupts,RPR521_STATUS_REG is 0X%x\n",__func__,__LINE__,status);
	}
	/*clear interrupt */
	rpr521_i2c_read(client, REG_INTERRUPT,RPR521_I2C_BYTE);
	mutex_unlock(&data->single_lock);
	if (data->irq)
	{
		operate_irq(data,1,true);
	}

	if(sensorDT_mode)
	{
		ps_data_count++;
	}
}

static void operate_irq(struct rpr521_data *data, int enable, bool sync)
{
	RPR521_FLOW("%s,line %d:operate irq type enable:%d, data->count:%d.\n",__func__,__LINE__,enable, data->count);
	if (data->irq)
	{
		if(enable)
		{
			data->count++;
			RPR521_FLOW("%s:line:%d, enable_irq, data->count:%d\n", __FUNCTION__, __LINE__, data->count);
			enable_irq(data->irq);
		}
		else
		{
			if(data->count > 0)
			{
				if(sync)
				{
					RPR521_FLOW("%s:line:%d, disable_irq data->count:%d\n", __FUNCTION__, __LINE__, data->count);
					disable_irq(data->irq);
				}
				else
				{
					RPR521_FLOW("%s:line:%d, disable_irq_nosync data->count:%d\n", __FUNCTION__, __LINE__, data->count);
					disable_irq_nosync(data->irq);
				}
				data->count--;
				RPR521_FLOW("%s:line:%d, after disable_irq data->count:%d\n", __FUNCTION__, __LINE__, data->count);
			}
		}
	}
}

/* assume this is ISR */
static irqreturn_t rpr521_interrupt(int vec, void *info)
{
	struct i2c_client *client=(struct i2c_client *)info;
	struct rpr521_data *data = i2c_get_clientdata(client);

	RPR521_FLOW("%s: line:%d rpr521 interrupt\n", __func__, __LINE__); //grace modify in 2014.7.28
	operate_irq(data,0,false);
	wake_lock_timeout(&data->ps_report_wk, PS_WAKEUP_TIME);
	/* and ps data report function to workqueue */
	queue_work(rpr521_workqueue, &data->dwork);

	return IRQ_HANDLED;
}

/*
 * IOCTL support
 */
static int rpr521_enable_als_sensor(struct i2c_client *client, int val)
{
	struct rpr521_data *data = i2c_get_clientdata(client);
	struct rpr521_platform_data *pdata = data->platform_data;
	int ret;
	/*Not in use power_value*/

	RPR521_FLOW("%s,line %d:enable als val=%d\n",__func__,__LINE__,val);
	/*Not in use power_value*/
	mutex_lock(&data->single_lock);
	if (val == 1) {
		/* turn on light  sensor */
		if (data->enable_als_sensor == 0) {
			if(data->enable_ps_sensor == 0){
				/* Power on and initalize the device */
				if (pdata->power_on)
					pdata->power_on(true,data);

				ret = rpr521_init_client(client);
				if (ret) {
					RPR521_ERR("%s:line:%d,Failed to init rpr521\n", __func__, __LINE__);
					mutex_unlock(&data->single_lock);
					return ret;
				}
				if(prase_parameter)
				{
					ret = sensor_parse_parameter(&client->dev);
					if(ret ) 
					{
						RPR521_ERR("%s: sensor_parse_parameter() error\n", __func__);
						return ret;
					}
					prase_parameter = false;
				}
			}
			als_print = true;
			als_polling_count=1;
			data->enable_als_sensor = 1;
			data->enable = data->enable|ALS_EN; 
			rpr521_set_enable(client, data->enable);

			RPR521_INFO("%s: line:%d enable als sensor,data->enable=%d\n", __func__, __LINE__, data->enable);
			/* enable als sensor, start data report hrtimer */
			ret = hrtimer_start(&data->timer, ktime_set(0, data->als_poll_delay * 1000000), HRTIMER_MODE_REL);
			if (ret != 0) {
				RPR521_ERR("%s: hrtimer_start fail! nsec=%d\n", __func__, data->als_poll_delay);
			}
		}
	} else {
		/*
		 * turn off light sensor
		 * what if the p sensor is active?
		 */
		 if(data->enable_als_sensor == 1)
		 {
			data->enable_als_sensor = 0;
			if(data->enable_ps_sensor ==1)
			{
				/*don't disable als sensor,ps sensor need it to check Ambient light Infra-red noise*/
				RPR521_INFO("%s: line:%d,don't disable als sensor,ps sensor need it to check Ambient light Infra-red noise\n", 
						__func__, __LINE__);
			}
			else
			{
				data->enable = PS_ALS_SET_MODE_CONTROL;
				rpr521_set_enable(client, data->enable);
			}

			RPR521_INFO("%s: line:%d,disable als sensor,data->enable = 0x%x\n", __func__, __LINE__,data->enable);
			/* disable als sensor, cancne data report hrtimer */
			hrtimer_cancel(&data->timer);
			cancel_work_sync(&data->als_dwork);
			hrtimer_cancel(&data->timer);
			
		 }

	}
	/* Vote off  regulators if both light and prox sensor are off */
	if ((data->enable_als_sensor == 0) &&(data->enable_ps_sensor == 0) &&(pdata->power_on)){
		pdata->power_on(false,data);
	}
	mutex_unlock(&data->single_lock);
	RPR521_FLOW("%s: line:%d,enable als sensor success\n", __func__, __LINE__);
	return 0;
}
static int rpr521_open_ps_sensor(struct rpr521_data *data, struct i2c_client *client)
{
	int ret = 0;
	int irq_val = 0;
	/* turn on p sensor */
	if (data->enable_ps_sensor==0) {
		/* Power on and initalize the device */
		if (data->platform_data->power_on)
			data->platform_data->power_on(true,data);

		ret = rpr521_init_client(client);
		if (ret) {
			RPR521_ERR("%s:line:%d,Failed to init rpr521\n", __func__, __LINE__);
			return ret;
		}

		data->enable_ps_sensor= 1;
		/*initialize the ps_min_threshold,to update data->piht and data->pilt*/
		data->ps_min_threshold = origin_prox;
		RPR521_FLOW("%s,line %d:change threshoid,data->ps_min_threshold =%d\n",__func__,__LINE__,data->ps_min_threshold);
		/* init threshold for proximity */
		ret = rpr521_set_pilt(client, far_init);
		ret += rpr521_set_piht(client, near_init);
		/*we use ch0data high thresh hold to check sunlight*/
		ret += rpr521_i2c_write(client,REG_ALSDATA0TL, 0, RPR521_I2C_WORD);
		ret += rpr521_i2c_write(client,REG_ALSDATA0TH, RPR521_SUNLIGHT_CHODATA, RPR521_I2C_WORD);
		if (ret < 0)
			return ret;
		RPR521_INFO("%s,line %d:change threshoid,data->ps_data =%d,data->pilt=%d,data->piht=%d,\n", __func__,__LINE__,data->ps_data, data->pilt, data->piht);

#ifdef CONFIG_HUAWEI_DSM
		rpr_dsm_save_threshold(data, far_init, near_init);
#endif
		/* enable both ALS and PS irq*/
		ret = rpr521_set_config(client, PS_ALS_SET_INTR | MODE_BOTH);
		if (ret < 0)
		{
			RPR521_ERR("%s,line%d:rpr521_set_config FAIL ",__func__,__LINE__);
		}

		/*clear interrupt by read rpr521 irq status reg before enable AP irq*/
		irq_val = rpr521_i2c_read(client, REG_INTERRUPT,RPR521_I2C_BYTE);
		if(irq_val < 0 )
		{
			RPR521_INFO("%s,line %d: clear rpr521 interrupt fail; err(%d)\n", __func__,__LINE__,irq_val);
		}else{
			RPR521_INFO("%s,line %d: clear rpr521 interrupt success; REG_INTERRUPT(%d)\n", __func__,__LINE__,irq_val);
		}
		
		if (data->irq)
		{
			operate_irq(data,1,true);
			/*set the property of pls irq,so the pls irq can wake up the sleeping system */
			irq_set_irq_wake(data->irq, 1);
		}
		/*we use our own calibration algorithm,more details of the algorithm you can check rpr521_ps_report_event*/
		/* Infrared detection results under the impact of ambient light , so need open ambient light*/
		data->enable = data->enable |PS_EN|ALS_EN;
		rpr521_set_enable(client, data->enable);
		RPR521_INFO("%s: line:%d,enable pls sensor.data->enable = 0x%x\n", __func__, __LINE__,data->enable);
		/* 0 is close, 1 is far */
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, RPR521_FAR_FLAG);
		input_sync(data->input_dev_ps);
		data->ps_detection = RPR521_FAR_FLAG;
		RPR521_INFO("%s,line %d:input_report_abs report ABS_DISTANCE, far event, data->ps_data:%d\n", __func__,__LINE__,data->ps_data);
		/* < move this codes before operate_irq  */
	}
	return ret;
}
static int rpr521_enable_ps_sensor(struct i2c_client *client,unsigned int val)
{
	struct rpr521_data *data = i2c_get_clientdata(client);
	int ret;
	/*Not in use power_value*/

	RPR521_FLOW("%s,line %d:val=%d\n",__func__,__LINE__,val);
	if ((val != 0) && (val != 1)) {
		RPR521_ERR("%s: invalid value=%d\n", __func__, val);
		return -EINVAL;
	}
	/*Not in use power_value*/
	if (val == 1) {
		mutex_lock(&data->single_lock);
		ret = rpr521_open_ps_sensor(data, client);
		mutex_unlock(&data->single_lock);
		if(ret)
		{
			RPR521_ERR("%s,line %d:read power_value failed,open ps fail\n",__func__,__LINE__);
			return ret;
		}
#ifdef CONFIG_HUAWEI_DSM
		rpr_dsm_no_irq_check(data);
#endif
		power_key_ps = false;
		schedule_delayed_work(&data->powerkey_work, msecs_to_jiffies(100));
	} else {
		/*
		 * turn off p sensor - kk 25 Apr 2011
		 * we can't turn off the entire sensor,
		 * the light sensor may be needed by HAL
		 */
		if (data->enable_ps_sensor==1) {
			mutex_lock(&data->single_lock);
			data->enable_ps_sensor = 0;
			/*close both als and ps IRQ*/
			ret = rpr521_set_config(client, PS_ALS_SET_INTR);
			if (ret < 0)
			{
				RPR521_ERR("%s,line%d:rpr521_set_config FAIL ",__func__,__LINE__);
			}			
			if(data->enable_als_sensor ==1)
			{
				data->enable = data->enable &(~PS_EN);
				rpr521_set_enable(client, data->enable);
			}
			else
			{
				data->enable= PS_ALS_SET_MODE_CONTROL;
				rpr521_set_enable(client, data->enable);
			}
			mutex_unlock(&data->single_lock);
			RPR521_INFO("%s: line:%d,disable pls sensor,data->enable = 0x%x\n", __func__, __LINE__,data->enable);
#ifdef CONFIG_HUAWEI_DSM
			rpr_dsm_change_ps_enable_status(data);
#endif
			cancel_work_sync(&data->dwork);
			cancel_delayed_work(&data->powerkey_work);
			if (data->irq)
			{
				/*when close the pls,make the wakeup property diabled*/
				irq_set_irq_wake(data->irq, 0);
				operate_irq(data,0,true);
			}
		}
	}
	/* Vote off  regulators if both light and prox sensor are off */
	if ((data->enable_als_sensor == 0) &&(data->enable_ps_sensor == 0) &&(data->platform_data->power_on)){
		data->platform_data->power_on(false,data);
	}
	return 0;
}
/*
 * SysFS support
 */
 static int rpr521_als_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	int ret = 0;
	static int als_enalbe_count=0;

	struct rpr521_data *data = container_of(sensors_cdev,struct rpr521_data, als_cdev);
	if ((enable != 0) && (enable != 1)) {
		RPR521_ERR("%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}
	RPR521_FLOW("%s,line %d:rpr521 als enable=%d\n",__func__,__LINE__,enable);

	/*for debug and print registers value when enable/disable the als every time*/
	if(enable == 0)
	{
		if(rpr521_debug_mask >= 1){
		RPR521_FLOW("attention:before als_disable %d times\n", als_enalbe_count);
			rpr521_regs_debug_print(data,enable);
		}
		rpr521_enable_als_sensor(data->client, enable);

	}else{

		rpr521_enable_als_sensor(data->client, enable);

		if(rpr521_debug_mask >= 1){
		RPR521_FLOW("attention: after als_enable %d times\n",++als_enalbe_count);
			rpr521_regs_debug_print(data,enable);
		}
	 }
	 return ret;
}

static int rpr521_ps_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct rpr521_data *data = container_of(sensors_cdev,
			struct rpr521_data, ps_cdev);
	if ((enable != 0) && (enable != 1)) {
		RPR521_ERR("%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}
	RPR521_FLOW("%s,line %d:rpr521 enable ps value=(%d)\n",__func__,__LINE__, enable);
	return rpr521_enable_ps_sensor(data->client, enable);
}
/*use this function to reset the poll_delay time(ms),val is the time parameter*/
static int rpr521_set_als_poll_delay(struct i2c_client *client,
		unsigned int val)
{
	struct rpr521_data *data = i2c_get_clientdata(client);
	//int ret;

	/* minimum 10ms */
	if (val < 10)
		val = 10;
	data->als_poll_delay = 200;
	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	 /*
	hrtimer_cancel(&data->timer);
	cancel_work_sync(&data->als_dwork);
	hrtimer_cancel(&data->timer);

	ret = hrtimer_start(&data->timer, ktime_set(0, data->als_poll_delay * 1000000), HRTIMER_MODE_REL);
	if (ret != 0) {
		RPR521_ERR("%s,line%d: hrtimer_start fail! nsec=%d\n", __func__, __LINE__,data->als_poll_delay);
		return ret;
	}
	*/
	return 0;
}

static int rpr521_als_poll_delay(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
	struct rpr521_data *data = container_of(sensors_cdev,
			struct rpr521_data, als_cdev);
	rpr521_set_als_poll_delay(data->client, delay_msec);
	return 0;
}
static ssize_t rpr521_show_ch0data(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ch0data;

	ch0data = rpr521_i2c_read(client,REG_ALSDATA0,RPR521_I2C_WORD);

	return snprintf(buf,32,"%d\n", ch0data);
}

static DEVICE_ATTR(ch0data, S_IRUGO, rpr521_show_ch0data, NULL);

static ssize_t rpr521_show_ch1data(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ch1data;

	ch1data = rpr521_i2c_read(client,REG_ALSDATA1,RPR521_I2C_WORD);

	return snprintf(buf,32, "%d\n", ch1data);
}

static DEVICE_ATTR(ch1data, S_IRUGO, rpr521_show_ch1data, NULL);

static ssize_t rpr521_show_pdata(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int pdata;

	pdata = rpr521_i2c_read(client, REG_PSDATA,RPR521_I2C_WORD);
	if(pdata <0){
		RPR521_ERR("%s,line %d:read pdata failed\n",__func__,__LINE__);
	}

	return snprintf(buf,32, "%d\n", pdata);
}

static DEVICE_ATTR(pdata, S_IRUGO, rpr521_show_pdata, NULL);


/*
* set the register's value from userspace
* Usage: echo "0x08|0x12" > dump_reg
*			"reg_address|reg_value"
*/
static ssize_t rpr521_write_reg(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rpr521_data *data = i2c_get_clientdata(client);
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

	mutex_lock(&data->update_lock);
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
	ret = rpr521_i2c_write(client,(char)reg_addr,(char)reg_val,RPR521_I2C_BYTE);
	if (ret < 0){
		goto parse_fail_exit;
	}
	mutex_unlock(&data->update_lock);
	return count;

parse_fail_exit:
	mutex_unlock(&data->update_lock);
	if (input_str)
		kfree(input_str);

	return ret;
}

/*
* show all registers' value to userspace
*/
static ssize_t rpr521_print_reg_buf(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int i;
	char reg[RPR521_REG_LEN];
	int regs = 0;
	struct i2c_client *client = to_i2c_client(dev);

	/* read all register value and print to user*/
	for(i = 0; i < RPR521_REG_LEN; i++ )
	{
		regs = 0x40+i;
		reg[i] = rpr521_i2c_read(client,regs,RPR521_I2C_BYTE); 
		if(reg[i] <0){
			RPR521_ERR("%s,line %d:read %d reg failed\n",__func__,__LINE__,i);
			return reg[i] ;
		}
	}

	return snprintf(buf,512,"reg[0x40~0x48]=0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x\n"
				      "reg[0x49~0x4e]0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x\n",
			reg[0x0],reg[0x1],reg[0x2],reg[0x3],reg[0x4],reg[0x5],reg[0x6],reg[0x7],reg[0x8],
			reg[0x9],reg[0xa],reg[0xb],reg[0xc],reg[0xd],reg[0xe]); 
}

static DEVICE_ATTR(dump_reg ,S_IRUGO|S_IWUSR|S_IWGRP, rpr521_print_reg_buf, rpr521_write_reg);
#ifdef CONFIG_HUAWEI_DSM

/*
*  test data or i2c error interface for device monitor
*/
static ssize_t rpr521_sysfs_dsm_test(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rpr521_data *data = i2c_get_clientdata(client);
	long mode;
	int ret = 0;

	if (strict_strtol(buf, 10, &mode))
			return -EINVAL;

	switch(mode){
		case DSM_LPS_I2C_ERROR:
			ret = dump_i2c_exception_status(data);
			break;
		case DSM_LPS_WRONG_IRQ_ERROR:
		case DSM_LPS_THRESHOLD_ERROR:
		case DSM_LPS_ENABLED_IRQ_ERROR:
			ret = rpr_dsm_report_err(mode, data);
			break;

		default:
			RPR521_ERR("%s unsupport err_no = %ld \n", __func__, mode);
			break;

	}

	return ret;
}

static DEVICE_ATTR(dsm_excep,S_IWUSR|S_IWGRP, NULL, rpr521_sysfs_dsm_test);
#endif

static struct attribute *rpr521_attributes[] = {
	&dev_attr_ch0data.attr,
	&dev_attr_ch1data.attr,
	&dev_attr_pdata.attr,
	&dev_attr_dump_reg.attr,
#ifdef CONFIG_HUAWEI_DSM
	&dev_attr_dsm_excep.attr,
#endif
	NULL
};

static const struct attribute_group rpr521_attr_group = {
	.attrs = rpr521_attributes,
};

/*
 * Initialization function
 */
static int rpr521_read_device_id(struct i2c_client *client)
{
	int id;
	int err;

	id = rpr521_i2c_read(client, REG_SYSTEMCONTROL,RPR521_I2C_BYTE);
	if (id == 0xa) {
		RPR521_INFO("%s: RPR521\n", __func__);
		err = app_info_set("LP-Sensor", "RPR521");
		if (err < 0)/*failed to add app_info*/
		{
		    RPR521_ERR("%s %d:failed to add app_info\n", __func__, __LINE__);
		}
	} else {
		RPR521_INFO("%s: Neither RPR521,id = %d \n", __func__,id);
		return -ENODEV;
	}
	return 0;
}
static int rpr521_init_client(struct i2c_client *client)
{
	struct rpr521_data *data = i2c_get_clientdata(client);
	int err;
	unsigned int tmp = 0;
	data->enable = PS_ALS_SET_MODE_CONTROL;
	err = rpr521_set_enable(client, PS_ALS_SET_MODE_CONTROL);
	if (err < 0)
	{
		RPR521_ERR("%s,line%d:rpr521_set_enable FAIL ",__func__,__LINE__);
		return err;
	}

	err = rpr521_set_config(client, PS_ALS_SET_INTR);
	if (err < 0)
	{
		RPR521_ERR("%s,line%d:rpr521_set_config FAIL ",__func__,__LINE__);
		return err;
	}

	err = rpr521_set_control(client,PS_ALS_SET_ALSPS_CONTROL);
	if (err < 0)
	{
		RPR521_ERR("%s,line%d:rpr521_set_control FAIL ",__func__,__LINE__);
		return err;
	}
	/* init threshold for proximity */
	err = rpr521_set_pilt(client, far_init);
	if (err < 0)
	{
		RPR521_ERR("%s,line%d:rpr521_set_pilt FAIL ",__func__,__LINE__);
		return err;
	}

	err = rpr521_set_piht(client, near_init);
	if (err < 0)
	{
		RPR521_ERR("%s,line%d:rpr521_set_piht FAIL ",__func__,__LINE__);
		return err;
	}
	data->ps_detection = RPR521_FAR_FLAG; /* initial value = far*/

	/* 1 consecutive Interrupt persistence */
	switch(data->platform_data->ps_gain)
	{
		case PSGAIN1:
			tmp = PS_ALS_SET_INTR_PERSIST;
			break;
		case PSGAIN2:
			tmp = PSGAIN_X2 |0x1;
			break;
		case PSGAIN4:
			tmp = PSGAIN_X4 |0x1;
			break;
		default:
			tmp = PS_ALS_SET_INTR_PERSIST;
			break;
	}
	err = rpr521_set_pers(client, tmp);
	if (err < 0)
	{
		RPR521_ERR("%s,line%d:rpr521_set_pers FAIL ",__func__,__LINE__);
		return err;
	}

	/* sensor is in disabled mode but all the configurations are preset */
	return 0;
}
/*qualcom updated the regulator configure functions and we add them all*/
static int sensor_regulator_configure(struct rpr521_data *data, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0,
				RPR521_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0,
				RPR521_VIO_MAX_UV);

		regulator_put(data->vio);

	} else {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			RPR521_ERR("%s,line%d:Regulator get failed vdd rc=%d\n",__func__,__LINE__, rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd,
				RPR521_VDD_MIN_UV, RPR521_VDD_MAX_UV);
			if (rc) {
				RPR521_ERR("%s,line%d:Regulator set failed vdd rc=%d\n",__func__,__LINE__,rc);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->client->dev, "vio");
		if (IS_ERR(data->vio)) {
			rc = PTR_ERR(data->vio);
			RPR521_ERR("%s,line%d:Regulator get failed vio rc=%d\n",__func__,__LINE__, rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			rc = regulator_set_voltage(data->vio,
				RPR521_VIO_MIN_UV, RPR521_VIO_MAX_UV);
			if (rc) {
				RPR521_ERR("%s,line%d:Regulator set failed vio rc=%d\n",__func__,__LINE__, rc);
				goto reg_vio_put;
			}
		}

	}

	return 0;
reg_vio_put:
	regulator_put(data->vio);

reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, RPR521_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}
/*In suspend and resume function,we only control the als,leave pls alone*/
static int rpr521_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct rpr521_data *data = i2c_get_clientdata(client);
	int rc;
	RPR521_INFO("%s,line%d:RPR521 SUSPEND\n",__func__,__LINE__);
	/*hrtimer and work will canceled at rpr521_enable_als_sensor*/
	/*
	* Save sensor state and disable them,
	* this is to ensure internal state flags are set correctly.
	* device will power off after both sensors are disabled.
	* P sensor will not be disabled because it  is a wakeup sensor.
	*/
	data->enable_als_state = data->enable_als_sensor;

	if(data->enable_als_sensor){
		RPR521_INFO("%s,line%d:RPR521 SUSPEND and disable als\n",__func__,__LINE__);
		rc = rpr521_enable_als_sensor(data->client, 0);
		if (rc){
			RPR521_ERR("%s,line%d:Disable light sensor fail! rc=%d\n",__func__,__LINE__, rc);
		}
	}

	return 0;
}

static int rpr521_resume(struct i2c_client *client)
{
	struct rpr521_data *data = i2c_get_clientdata(client);
	int ret = 0;

	RPR521_INFO("%s,line%d:RPR521 RESUME\n",__func__,__LINE__);
	if (data->enable_als_state) {
		ret = rpr521_enable_als_sensor(data->client, 1);
		if (ret){
			RPR521_ERR("%s,line%d:Disable light sensor fail! rc=%d\n",__func__,__LINE__, ret);
		}
	}

	return 0;
}
/*pamameter subfunction of probe to reduce the complexity of probe function*/
static int rpr521_sensorclass_init(struct rpr521_data *data,struct i2c_client* client)
{
	int err;
	/* Register to sensors class */
	data->als_cdev = sensors_light_cdev;
	data->als_cdev.sensors_enable = rpr521_als_set_enable;
	data->als_cdev.sensors_poll_delay = rpr521_als_poll_delay;

	data->ps_cdev = sensors_proximity_cdev;
	data->ps_cdev.sensors_enable = rpr521_ps_set_enable;
	data->ps_cdev.sensors_poll_delay = NULL;

	err = sensors_classdev_register(&data->input_dev_als->dev, &data->als_cdev);
	if (err) {
		RPR521_ERR("%s: Unable to register to sensors class: %d\n",__func__, err);
		goto exit;
	}
	err = sensors_classdev_register(&data->input_dev_ps->dev, &data->ps_cdev);
	if (err) {
		RPR521_ERR("%s: Unable to register to sensors class: %d\n",__func__, err);
		goto exit_unregister_als_class;
	}
	goto exit;
exit_unregister_als_class:
	sensors_classdev_unregister(&data->als_cdev);
exit:
	return err;
}
static void rpr521_parameter_init(struct rpr521_data *data)
{
	/* Set the default parameters */
	
	rpr521_pwave_value= data->platform_data->pwave;
	rpr521_pwindow_value= data->platform_data->pwindow;

	data->enable = PS_ALS_SET_MODE_CONTROL;	/* default mode is standard */
	data->ps_min_threshold = origin_prox;
	RPR521_FLOW("%s:set origin_prox to data->ps_min_threshold=%d\n", __func__, data->ps_min_threshold);
	data->enable_als_sensor = 0;	// default to 0
	data->enable_ps_sensor = 0;	// default to 0
	data->als_poll_delay = 200;	// default to 200ms
	data->als_prev_lux = 300;
	data->count = 1;	// disable_irq is before enable_irq, so the initial value should more than zero
}
/*input init subfunction of probe to reduce the complexity of probe function*/
static int rpr521_input_init(struct rpr521_data *data)
{
	int err = 0;
	/* Register to Input Device */
	data->input_dev_als = input_allocate_device();
	if (!data->input_dev_als) {
		err = -ENOMEM;
		RPR521_ERR("%s: Failed to allocate input device als\n", __func__);
		goto exit;
	}

	data->input_dev_ps = input_allocate_device();
	if (!data->input_dev_ps) {
		err = -ENOMEM;
		input_free_device(data->input_dev_als);
		RPR521_ERR("%s: Failed to allocate input device ps\n", __func__);
		goto exit;
	}

	set_bit(EV_ABS, data->input_dev_als->evbit);
	set_bit(EV_ABS, data->input_dev_ps->evbit);

	input_set_abs_params(data->input_dev_als, ABS_MISC, 0, 10000, 0, 0);
	input_set_abs_params(data->input_dev_ps, ABS_DISTANCE, 0, 5, 0, 0);

	data->input_dev_als->name = "light";
	data->input_dev_ps->name = "proximity";

	err = input_register_device(data->input_dev_als);
	if (err) {
		err = -ENOMEM;
		input_free_device(data->input_dev_als);
		input_free_device(data->input_dev_ps);
		RPR521_ERR("%s: Unable to register input device als: %s\n",
				__func__, data->input_dev_als->name);
		goto exit;
	}

	err = input_register_device(data->input_dev_ps);
	if (err) {
		err = -ENOMEM;
		RPR521_ERR("%s: Unable to register input device ps: %s\n",
				__func__, data->input_dev_ps->name);
		goto unregister_als;
	}
	goto exit;
unregister_als:
	input_unregister_device(data->input_dev_als);
	input_free_device(data->input_dev_ps);
exit:
	return err;
}
/*irq request subfunction of probe to reduce the complexity of probe function*/
static int rpr521_irq_init(struct rpr521_data *data,struct i2c_client *client)
{
	int ret = 0;
	if (data->platform_data->irq_gpio)
	{
		ret = gpio_request(data->platform_data->irq_gpio,"rpr521_irq_gpio");
		if (ret)
		{
			RPR521_ERR("%s, line %d:unable to request gpio [%d]\n", __func__, __LINE__,data->platform_data->irq_gpio);
			return ret;
		}
		else
		{
			ret = gpio_direction_input(data->platform_data->irq_gpio);
			if(ret)
			{
				RPR521_ERR("%s, line %d: Failed to set gpio %d direction\n", __func__, __LINE__,data->platform_data->irq_gpio);
				return ret;
			}
		}
	}
	client->irq = gpio_to_irq(data->platform_data->irq_gpio);
	if (client->irq < 0) {
		ret = -EINVAL;
		RPR521_ERR("%s, line %d:gpio_to_irq FAIL! IRQ=%d\n", __func__, __LINE__,data->platform_data->irq_gpio);
		return ret;
	}
	data->irq = client->irq;
	if (client->irq)
	{
		/*AP examination of low level to prevent lost interrupt*/
		if (request_irq(data->irq, rpr521_interrupt,IRQF_TRIGGER_LOW|IRQF_ONESHOT|IRQF_NO_SUSPEND, RPR521_DRV_NAME, (void *)client) >= 0)
		{
			RPR521_FLOW("%s, line %d:Received IRQ!\n", __func__, __LINE__);
			operate_irq(data,0,true);
		}
		else
		{
			RPR521_ERR("%s, line %d:Failed to request IRQ!\n", __func__, __LINE__);
			ret = -EINVAL;
			return ret;
		}
	}
	return ret;
}
static int sensor_regulator_power_on(struct rpr521_data *data, bool on)
{
	int rc = 0;

	if (!on) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			RPR521_ERR("%s: Regulator vdd disable failed rc=%d\n", __func__, rc);
			return rc;
		}

		rc = regulator_disable(data->vio);
		if (rc) {
			RPR521_ERR("%s: Regulator vdd disable failed rc=%d\n", __func__, rc);
			rc = regulator_enable(data->vdd);
			RPR521_ERR("%s:Regulator vio re-enabled rc=%d\n",__func__, rc);
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
			RPR521_ERR("%s:Regulator vdd enable failed rc=%d\n",__func__, rc);
			return rc;
		}

		rc = regulator_enable(data->vio);
		if (rc) {
			RPR521_ERR("%s:Regulator vio enable failed rc=%d\n", __func__,rc);
			rc = regulator_disable(data->vdd);
			return rc;
		}
	}
enable_delay:
	msleep(130);
	RPR521_FLOW("%s:Sensor regulator power on =%d\n",__func__, on);
	return rc;
}

static int sensor_platform_hw_power_on(bool on,struct rpr521_data *data)
{
	int err = 0;

	if (data->power_on != on) {
		if (!IS_ERR_OR_NULL(data->pinctrl)) {
			if (on)
				/*after poweron,set the INT pin the default state*/
				err = pinctrl_select_state(data->pinctrl,
					data->pin_default);
			if (err)
				RPR521_ERR("%s,line%d:Can't select pinctrl state\n", __func__, __LINE__);
		}

		err = sensor_regulator_power_on(data, on);
		if (err)
			RPR521_ERR("%s,line%d:Can't configure regulator!\n", __func__, __LINE__);
		else
			data->power_on = on;
	}

	return err;
}
static int sensor_platform_hw_init(struct rpr521_data *data)
{
	int error;

	error = sensor_regulator_configure(data, true);
	if (error < 0) {
		RPR521_ERR("%s,line %d:unable to configure regulator\n",__func__,__LINE__);
		return error;
	}

	return 0;
}

static void sensor_platform_hw_exit(struct rpr521_data *data)
{
	int error;
	error = sensor_regulator_configure(data, false);
	if (error < 0) {
		RPR521_ERR("%s,line %d:unable to configure regulator\n",__func__,__LINE__);
	}
	/*remove to the error handler program in probe because the gpio is not requested.*/
}
static int rpr521_pinctrl_init(struct rpr521_data *data)
{
	struct i2c_client *client = data->client;

	data->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(data->pinctrl)) {
		RPR521_ERR("%s,line %d:Failed to get pinctrl\n",__func__,__LINE__);
		return PTR_ERR(data->pinctrl);
	}
	/*we have not set the sleep state of INT pin*/
	data->pin_default =
		pinctrl_lookup_state(data->pinctrl, "default");
	if (IS_ERR_OR_NULL(data->pin_default)) {
		RPR521_ERR("%s,line %d:Failed to look up default state\n",__func__,__LINE__);
		return PTR_ERR(data->pin_default);
	}

	return 0;
}

static unsigned long *create_and_get_array(struct device_node *dev_node,
		const char *name, int *size)
{
	const __be32 *values;
	unsigned long *val_array;
	int len;
	int sz;
	int rc;
	int i;

	values = of_get_property(dev_node, name, &len);
	if (values == NULL)
	{
		RPR521_ERR("%s:value is NULL\n",__func__);
		return NULL;
	}
	sz = len / sizeof(u32);
	pr_debug("%s: %s size:%d\n", __func__, name, sz);

	val_array = kzalloc(sz * sizeof(unsigned long), GFP_KERNEL);
	if (val_array == NULL) {
		rc = -ENOMEM;
		goto fail;
	}

	for (i = 0; i < sz; i++)
		val_array[i] = (unsigned long)be32_to_cpup(values++);

	*size = sz;

	return val_array;

fail:
	return ERR_PTR(rc);
}


static unsigned long *create_and_get_als_judge_array(struct device_node *dev_node)
{
	unsigned long *judge;
	int size;
	int rc;
	int tp_type = -1;
	if(diff_tp )
	{
		tp_type=get_tp_type();
		RPR521_ERR("%s:tp_type = %d\n",__func__,tp_type);
		switch(tp_type)
		{
			case  FW_OFILM:
				{
					judge = create_and_get_array(dev_node, "rpr521,judge_array_of", &size);
					break;
				}
			case  FW_EELY:
				{
					judge = create_and_get_array(dev_node, "rpr521,judge_array_ee", &size);
					break;
				}
			case  FW_TRULY:
				{
					judge = create_and_get_array(dev_node, "rpr521,judge_array_tr", &size);
					break;
				}
			case  FW_GIS:
				{
					judge = create_and_get_array(dev_node, "rpr521,judge_array_gi", &size);
					break;
				}
			case  FW_JUNDA:
				{
					judge = create_and_get_array(dev_node, "rpr521,judge_array_ju", &size);
					break;
				}
			case  FW_LENSONE:
				{
					judge = create_and_get_array(dev_node, "rpr521,judge_array_le", &size);
					break;
				}
			case  FW_YASSY:
				{
					judge = create_and_get_array(dev_node, "rpr521,judge_array_ya", &size);
					break;
				}
			default:
				{
					RPR521_ERR("%s,prase dt error\n",__func__);
					judge = create_and_get_array(dev_node, "rpr521,judge_array", &size);
					break;
				}
		}
	}
	else
		judge = create_and_get_array(dev_node, "rpr521,judge_array", &size);
	if (IS_ERR_OR_NULL(judge))
		judge = create_and_get_array(dev_node, "rpr521,judge_array", &size);
	if (IS_ERR_OR_NULL(judge))
		return (void *)judge;

	/* Check for valid judge size */
	if (size % COEFFICIENT) {
		rc = -EINVAL;
		goto fail_free_judge;
	}

	return judge;

fail_free_judge:
	kfree(judge);

	return ERR_PTR(rc);
}


static unsigned long *create_and_get_als_data1_array(struct device_node *dev_node)
{
	unsigned long *data1;
	int size;
	int rc;
	int tp_type = -1;
	if(diff_tp )
	{
		tp_type=get_tp_type();
		switch(tp_type)
		{
			case  FW_OFILM:
				{
					data1 = create_and_get_array(dev_node, "rpr521,data1_array_of", &size);
					break;
				}
			case  FW_EELY:
				{
					data1= create_and_get_array(dev_node, "rpr521,data1_array_ee", &size);
					break;
				}
			case  FW_TRULY:
				{
					data1 = create_and_get_array(dev_node, "rpr521,data1_array_tr", &size);
					break;
				}
			case  FW_GIS:
				{
					data1 = create_and_get_array(dev_node, "rpr521,data1_array_gi", &size);
					break;
				}
			case  FW_JUNDA:
				{
					data1 = create_and_get_array(dev_node, "rpr521,data1_array_ju", &size);
					break;
				}
			case  FW_LENSONE:
				{
					data1 = create_and_get_array(dev_node, "rpr521,data1_array_le", &size);
					break;
				}
			case  FW_YASSY:
				{
					data1 = create_and_get_array(dev_node, "rpr521,data1_array_ya", &size);
					break;
				}
			default:
				{
					RPR521_ERR("%s,prase dt error\n",__func__);
					data1 = create_and_get_array(dev_node, "rpr521,data1_array", &size);
					break;
				}
		}
	}
	else
		data1 = create_and_get_array(dev_node, "rpr521,data1_array", &size);
	if (IS_ERR_OR_NULL(data1))
		data1 = create_and_get_array(dev_node, "rpr521,data1_array", &size);
	if (IS_ERR_OR_NULL(data1))
		return (void *)data1;

	/* Check for valid data1 size */
	if (size % COEFFICIENT) {
		rc = -EINVAL;
		goto fail_free_data1;
	}

	return data1;

fail_free_data1:
	kfree(data1);

	return ERR_PTR(rc);
}


static unsigned long *create_and_get_als_data0_array(struct device_node *dev_node)
{
	unsigned long *data0;
	int size;
	int rc;
	int tp_type = -1;
	if(diff_tp )
	{
		tp_type=get_tp_type();
		switch(tp_type)
		{
			case  FW_OFILM:
				{
					data0 = create_and_get_array(dev_node, "rpr521,data0_array_of", &size);
					break;
				}
			case  FW_EELY:
				{
					data0= create_and_get_array(dev_node, "rpr521,data0_array_ee", &size);
					break;
				}
			case  FW_TRULY:
				{
					data0 = create_and_get_array(dev_node, "rpr521,data0_array_tr", &size);
					break;
				}
			case  FW_GIS:
				{
					data0 = create_and_get_array(dev_node, "rpr521,data0_array_gi", &size);
					break;
				}
			case  FW_JUNDA:
				{
					data0 = create_and_get_array(dev_node, "rpr521,data0_array_ju", &size);
					break;
				}
			case  FW_LENSONE:
				{
					data0 = create_and_get_array(dev_node, "rpr521,data0_array_le", &size);
					break;
				}
			case  FW_YASSY:
				{
					data0 = create_and_get_array(dev_node, "rpr521,data0_array_ya", &size);
					break;
				}
			default:
				{
					RPR521_ERR("%s,prase dt error\n",__func__);
					data0 = create_and_get_array(dev_node, "rpr521,data0_array", &size);
					break;
				}
		}
	}
	else
		data0 = create_and_get_array(dev_node, "rpr521,data0_array", &size);
	if (IS_ERR_OR_NULL(data0))
		data0 = create_and_get_array(dev_node, "rpr521,data0_array", &size);
	if (IS_ERR_OR_NULL(data0))
		return (void *)data0;

	/* Check for valid data0 size */
	if (size % COEFFICIENT) {
		rc = -EINVAL;
		goto fail_free_data0;
	}

	return data0;

fail_free_data0:
	kfree(data0);

	return ERR_PTR(rc);
}
/******************************************************************************
Function:       sensor_parse_parameter
Description:  Wrapper function, Parsing dtsi ambient light array parameter
Parameters:  struct device *
Return:          int   vlue
******************************************************************************/
static int sensor_parse_parameter(struct device *dev)
{
	
	struct device_node *np = dev->of_node;
	int rc = 0;
	//read data0 from dtsi for calculate als data
	data0_coefficient = create_and_get_als_data0_array(np);
	if (data0_coefficient == NULL)
	{
		rc = -EINVAL;
		RPR521_ERR("Unable to read data0_array,  rc = %d \n",rc);
		return rc;
	}

	data1_coefficient = create_and_get_als_data1_array(np);
	if (data1_coefficient == NULL)
	{
		rc = -EINVAL;
		RPR521_ERR("Unable to read data1_array,  rc = %d \n",rc);
		kfree(data0_coefficient);
		data0_coefficient = NULL;
		return rc;
	}

	judge_coefficient = create_and_get_als_judge_array(np);
	if (judge_coefficient == NULL)
	{
		rc = -EINVAL;
		RPR521_ERR("Unable to read judge_array,  rc = %d \n",rc);
		kfree(data0_coefficient);
		data0_coefficient = NULL;
		kfree(data1_coefficient);
		data1_coefficient = NULL;
		return rc;
	}
	return 0;
}
static int sensor_parse_dt(struct device *dev,
		struct rpr521_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	unsigned int tmp;
	int rc = 0;

	/* set functions of platform data */
	pdata->init = sensor_platform_hw_init;
	pdata->exit = sensor_platform_hw_exit;
	pdata->power_on = sensor_platform_hw_power_on;

	/* irq gpio */

	rc = of_get_named_gpio_flags(dev->of_node, "rpr521,irq-gpio", 0, NULL);
    	if (rc < 0)
    	{
        	RPR521_ERR("Unable to read irq gpio\n");
        	return rc;
    	}
	pdata->irq_gpio = rc;

	rc = of_property_read_u32(np, "rpr521,wave", &tmp);
	if (rc) {
		RPR521_ERR("Unable to read pwave_value\n");
		return rc;
	}
	pdata ->pwave= tmp;
	
	rc = of_property_read_u32(np, "rpr521,window", &tmp);
	if (rc) {
		RPR521_ERR("Unable to read pwindow_value\n");
		return rc;
	}
	pdata ->pwindow= tmp;
	pdata->i2c_scl_gpio = of_get_named_gpio_flags(np, "rpr521,i2c-scl-gpio", 0, NULL);
	if (!gpio_is_valid(pdata->i2c_scl_gpio)) {
		RPR521_ERR("gpio i2c-scl pin %d is invalid\n", pdata->i2c_scl_gpio);
		return -EINVAL;
	}
	
	pdata->i2c_sda_gpio = of_get_named_gpio_flags(np, "rpr521,i2c-sda-gpio", 0, NULL);
	if (!gpio_is_valid(pdata->i2c_sda_gpio)) {
		RPR521_ERR("gpio i2c-sda pin %d is invalid\n", pdata->i2c_sda_gpio);
		return -EINVAL;
	}
	rc = of_property_read_u32(np, "rpr521,diff_tp", &tmp);
	if(rc)
		RPR521_ERR("Unable to read diff_tp_value,tmp=%d\n",tmp);
	else
	{	/*If rpr521, diff_tp value of 1, diff_tp = true */
		if(DISTINCTION_TP == tmp )
			diff_tp= true;
	}
	pdata->ps_gain = 0;
	rc = of_property_read_u32(np, "rpr521,ps_gain", &tmp);
	if (rc)
		RPR521_ERR("Unable to read ps_gain,tmp=%d\n",tmp);
	else
	{
		switch(tmp)
		{
			case  PSGAIN1 :
			case  PSGAIN2 :
			case  PSGAIN4 :
				pdata->ps_gain = tmp;
				break;
			default:
				RPR521_ERR("ps_gain error,tmp = %d\n",tmp);
				break;
		}
	}
	return 0;
}


static void rpr521_powerkey_screen_handler(struct work_struct *work)
{
	struct rpr521_data *data = container_of((struct delayed_work *)work, struct rpr521_data, powerkey_work);
	if(power_key_ps &&  (1== data ->enable_ps_sensor))
	{
		RPR521_INFO("%s : power_key_ps (%d) press\n",__func__, power_key_ps);
		rpr521_regs_debug_print(data,1);
		power_key_ps=false;
#ifdef  CONFIG_HUAWEI_DSM
		check_hardware_software_flag(data);
#endif
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, RPR521_FAR_FLAG);
		input_sync(data->input_dev_ps);
	}
	if(1== data ->enable_ps_sensor)
		schedule_delayed_work(&data->powerkey_work, msecs_to_jiffies(500));
}
/*
 * I2C init/probing/exit functions
 */
static struct i2c_driver rpr521_driver;
static int rpr521_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct rpr521_data *data;
	struct rpr521_platform_data *pdata;
	int err = 0;

	RPR521_INFO("%s,line %d:PROBE START.\n",__func__,__LINE__);

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct rpr521_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			RPR521_ERR("%s,line %d:Failed to allocate memory\n",__func__,__LINE__);
			err =-ENOMEM;
			goto exit;
		}

		client->dev.platform_data = pdata;
		err = sensor_parse_dt(&client->dev, pdata);
		if (err) {
			RPR521_ERR("%s: sensor_parse_dt() err\n", __func__);
			goto exit_parse_dt;
		}
	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			RPR521_ERR("%s,line %d:No platform data\n",__func__,__LINE__);
			err = -ENODEV;
			goto exit;
		}
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		RPR521_ERR("%s,line %d:Failed to i2c_check_functionality\n",__func__,__LINE__);
		err = -EIO;
		goto exit_parse_dt;
	}

	data = kzalloc(sizeof(struct rpr521_data), GFP_KERNEL);
	if (!data) {
		RPR521_ERR("%s,line %d:Failed to allocate memory\n",__func__,__LINE__);
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
#ifdef CONFIG_HUAWEI_DSM
	err = rpr_dsm_init(data);
	if(err < 0)
		goto exit_uninit;
#endif
	i2c_set_clientdata(client, data);
	rpr521_parameter_init(data);
	/* initialize pinctrl */
	err = rpr521_pinctrl_init(data);
	if (err) {
		RPR521_ERR("%s,line %d:Can't initialize pinctrl\n",__func__,__LINE__);
			goto exit_unregister_dsm;
	}
	err = pinctrl_select_state(data->pinctrl, data->pin_default);
	if (err) {
		RPR521_ERR("%s,line %d:Can't select pinctrl default state\n",__func__,__LINE__);
		goto exit_unregister_dsm;
	}

	mutex_init(&data->update_lock);
	mutex_init(&data->single_lock);
	INIT_WORK(&data->dwork, rpr521_work_handler);

	INIT_WORK(&data->als_dwork, rpr521_als_polling_work_handler);
	INIT_DELAYED_WORK(&data->powerkey_work, rpr521_powerkey_screen_handler);
	/* Initialize the RPR521 chip and judge who am i*/
	err=rpr521_read_device_id(client);
	if (err) {
		RPR521_ERR("%s: Failed to read rpr521\n", __func__);
		goto exit_unregister_dsm;
	}
	err = rpr521_init_client(client);
	if (err) {
		RPR521_ERR("%s: Failed to init rpr521\n", __func__);
		goto exit_unregister_dsm;
	}

	err = rpr521_input_init(data);
	if(err)
		goto exit_unregister_dsm;

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &rpr521_attr_group);
	if (err)
		goto exit_unregister_dev_ps;

	wake_lock_init(&data->ps_report_wk, WAKE_LOCK_SUSPEND, "psensor_wakelock");

	err=rpr521_irq_init(data,client);
	if(err)
		goto exit_remove_sysfs_group;

	device_init_wakeup(&(client->dev), true);

	err = rpr521_sensorclass_init(data,client);
	if (err) {
		RPR521_ERR("%s: Unable to register to sensors class: %d\n",
	__func__, err);
		goto exit_free_irq;
	}

	rpr521_workqueue = create_workqueue("rpr521_work_queue");
	if (!rpr521_workqueue)
	{
		RPR521_ERR("%s: Create ps_workqueue fail.\n", __func__);
		goto exit_unregister_sensorclass;
	}

	/* init hrtimer and call back function */
	hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->timer.function = rpr521_als_timer_func;
	set_sensors_list(L_SENSOR + P_SENSOR);
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
	set_hw_dev_flag(DEV_I2C_APS);
	set_hw_dev_flag(DEV_I2C_L_SENSOR);
#endif
	err = set_sensor_input(PS, data->input_dev_ps->dev.kobj.name);
	if (err) {
		RPR521_ERR("%s set_sensor_input PS failed\n", __func__);
	}
	err = set_sensor_input(ALS, data->input_dev_als->dev.kobj.name);
	if (err) {
		RPR521_ERR("%s set_sensor_input ALS failed\n", __func__);
	}

	rpr521_regs_debug_print(data,1);

	if (pdata->power_on)
		err = pdata->power_on(false,data);
	RPR521_INFO("%s: Support ver. %s enabled\n", __func__, DRIVER_VERSION);
	data->device_exist = true;
	return 0;
	/* delete this,use one work queue*/
exit_unregister_sensorclass:
	sensors_classdev_unregister(&data->als_cdev);
	sensors_classdev_unregister(&data->ps_cdev);
exit_free_irq:
	if (gpio_is_valid(data->platform_data->irq_gpio))
		gpio_free(data->platform_data->irq_gpio);
	free_irq(data->irq, client);
exit_remove_sysfs_group:
	wake_lock_destroy(&data->ps_report_wk);
	sysfs_remove_group(&client->dev.kobj, &rpr521_attr_group);
exit_unregister_dev_ps:
	input_unregister_device(data->input_dev_ps);
	input_unregister_device(data->input_dev_als);
#ifdef CONFIG_HUAWEI_DSM
exit_unregister_dsm:
	rpr_dsm_exit();
exit_uninit:
#else
exit_unregister_dsm:
#endif
	if (pdata->power_on)
		pdata->power_on(false,data);
	if (pdata->exit)
		pdata->exit(data);
	kfree(data);

exit_parse_dt:
	/*remove it,because devm_kzalloc will release the memory automatically
	if don't remove,there will be null pointer which lead to booting crush*/
exit:
	return err;
}

static int rpr521_remove(struct i2c_client *client)
{
	struct rpr521_data *data = i2c_get_clientdata(client);
	struct rpr521_platform_data *pdata = data->platform_data;

	/* Power down the device */
	data->enable = PS_ALS_SET_MODE_CONTROL;
	rpr521_set_enable(client, PS_ALS_SET_MODE_CONTROL);

	wake_lock_destroy(&data->ps_report_wk);
	sysfs_remove_group(&client->dev.kobj, &rpr521_attr_group);

	input_unregister_device(data->input_dev_ps);
	input_unregister_device(data->input_dev_als);

	free_irq(client->irq, data);
	hrtimer_cancel(&data->timer);
#ifdef CONFIG_HUAWEI_DSM
	rpr_dsm_exit();
#endif
	if (pdata->power_on)
		pdata->power_on(false,data);

	if (pdata->exit)
		pdata->exit(data);

	kfree(data);

	return 0;
}

static const struct i2c_device_id rpr521_id[] = {
	{ "rpr521", 0 }, 
	{ }
};
MODULE_DEVICE_TABLE(i2c, rpr521_id);

static struct of_device_id rpr521_match_table[] = {
	{ .compatible = "rohm,rpr521",}, 
	{ },
};

static struct i2c_driver rpr521_driver = {
	.driver = {
		.name   = RPR521_DRV_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = rpr521_match_table,
	},
	.probe  = rpr521_probe,
	.remove = rpr521_remove,
	.suspend = rpr521_suspend,
	.resume = rpr521_resume,
	.id_table = rpr521_id,
};

static int __init rpr521_init(void)
{
	return i2c_add_driver(&rpr521_driver);
}

static void __exit rpr521_exit(void)
{
	/* destroy als and ps work queue */
	if (rpr521_workqueue) {
		destroy_workqueue(rpr521_workqueue);
		rpr521_workqueue = NULL;
	}

	i2c_del_driver(&rpr521_driver);
}

MODULE_DESCRIPTION("RPR521 ambient light + proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(rpr521_init);
module_exit(rpr521_exit);
