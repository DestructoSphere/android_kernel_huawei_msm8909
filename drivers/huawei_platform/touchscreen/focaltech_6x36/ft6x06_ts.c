/* drivers/input/touchscreen/ft6x06_ts.c
 *
 * FocalTech ft6x06 TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
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
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <mach/irqs.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/syscalls.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kobject.h>
#include <linux/workqueue.h>
#ifdef CONFIG_APP_INFO
#include <misc/app_info.h> 
#endif
#include <linux/of.h>
#include "ft6x06_ts.h"
/*delete focaltech debug logs to use common debug logs*/
#if !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#ifdef CONFIG_FOCALTECH_FOCALTECH_USERDATA
#include "focaltech_ctl.h"
#endif
#ifdef CONFIG_FOCALTECH_FT6X06_SYSDEBUG
#include "ft6x06_sysdebug.h"
#endif
#ifdef CONFIG_FOCALTECH_FT6X06_UPDATA_FW
#include "ft6x06_ex_fun.h"
#endif

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif

#define IS_TP_SUSPENDED    1
#define NOT_TP_SUSPENDED   0
/*delete focaltech_debug_mask in order to use common debug mask*/
struct regulator *vdd_focaltech = NULL;
struct regulator *vbus_focaltech = NULL;
static char touch_info[APP_INFO_VALUE_LENTH] = {0};
static struct i2c_client *g_i2c_client = NULL;

struct i2c_client * i2c_client=NULL;

static unsigned int reversal_data = 0;

#define FTS_APK_DEBUG
#define TP_GPIO_I2C_SDA   2
#define TP_GPIO_I2C_SCL   3
#define FOCALTECH_VRITUAL_KEY
#define FT6X06_REG_PMODE		0xA5
#define FT6X06_PMODE_HIBERNATE	0x03
#define FTS_POINT_UP		0x01
#define FTS_POINT_DOWN		0x00
#define FTS_POINT_CONTACT	0x02


struct ft6x06_ts_data * g_ft6x06_ts = NULL;
extern struct dsm_dev dsm_i2c;
extern struct dsm_client *tp_dclient;


/*
*ft6x06_tp_dump - called when force dump dsm err
*/
int ft6x06_tp_dump (int type, void *buff, int size)
{
    int used_size = 0;
    struct dsm_client * dsm_client = tp_dclient;
    if(NULL == dsm_client)
    {
        tp_log_err("%s %d: dsm_client is NULL!\n", __func__, __LINE__);
        return used_size;
    }

    /* save tp basice infomation */
    used_size = ft6x06_dsm_record_basic_info(g_ft6x06_ts, dsm_client);

    if( used_size > 0 )
    {
        tp_log_info("%s %d: force dump tp error! \n",__func__, __LINE__);
        snprintf( buff, dsm_client->used_size, dsm_client->dump_buff );
    }

    return dsm_client->used_size;
}


size_t ft6x06_dsm_record_basic_info(struct ft6x06_ts_data * ft6x06_data, struct dsm_client * dsm_client)
{
    ssize_t size = 0;
    ssize_t total_size = 0; 

    if(!ft6x06_data)
    {
        tp_log_err("%s %d: ft6x06_data is null!\n", __func__, __LINE__);
        return -1;
    }

    if(!dsm_client)
    {
        tp_log_err("%s %d: dsm_client is null!\n", __func__, __LINE__);
        return -1;
    }

    /* power status: mode, enable, voltage*/ 
    size = dsm_client_record(dsm_client,
                "[vbus power] mode:%d, enable:%d, vol:%d\n"
                "[vdd power]  mode:%d, enable:%d, vol:%d\n",
                regulator_get_mode(vbus_focaltech), 
                regulator_is_enabled(vbus_focaltech),
                regulator_get_voltage(vbus_focaltech),
                regulator_get_mode(vdd_focaltech),
                regulator_is_enabled(vdd_focaltech),
                regulator_get_voltage(vdd_focaltech));
    total_size += size;

    /* gpio status: irq, reset*/
    size =dsm_client_record(dsm_client, 
                "[irq gpio]   num:%d, irq gpio status:%d\n"
                "[reset gpio] num:%d, reset gpio status:%d\n",
                ft6x06_data->pdata->irq, gpio_get_value(ft6x06_data->pdata->reset),
                ft6x06_data->pdata->reset, gpio_get_value(ft6x06_data->pdata->reset));
    total_size += size;

    return total_size;
}

void ft6x06_report_dsm_erro(struct ft6x06_ts_data * ft6x06_data, struct dsm_client * dsm_client, 
                                                                int type, int err_num)
{
    if(!dsm_client)
    {
        tp_log_err("%s %d: dsm_client is null!\n", __func__, __LINE__);
        return;
    }

    if(dsm_client_ocuppy(dsm_client))
    {
        tp_log_err("%s %d: dsm buffer is busy!\n", __func__, __LINE__);
        return;
    }

    switch(type)
    {
        case DSM_TP_I2C_RW_ERROR_NO:
            dsm_client_record(dsm_client,"%s %d: i2c_trans_erro, rc = %d\n", __func__,__LINE__, err_num); 
            break;
        case DSM_TP_FW_ERROR_NO:
            dsm_client_record(dsm_client,"%s %d: firmware upgrade erro!\n", __func__,__LINE__); 
            break;
        default:
            break;
    }

    ft6x06_dsm_record_basic_info(ft6x06_data, dsm_client);
    dsm_client_notify(dsm_client, type);

    return;
}


/*
*ft6x06_i2c_Read-read data and write data by i2c
*@client: handle of i2c
*@writebuf: Data that will be written to the slave
*@writelen: How many bytes to write
*@readbuf: Where to store data read from slave
*@readlen: How many bytes to read
*
*Returns negative errno, else the number of messages executed
*
*
*/
int ft6x06_i2c_Read(struct i2c_client *client, char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret;
	tp_log_debug("in ft6x06_i2c_Read\n");
	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
	}
	tp_log_debug("out ft6x06_i2c_Read");
    
	if(ret <= 0)
	{
            tp_log_err("%s %d: i2c_transfer return: %d\n", __func__, __LINE__,ret);

            ft6x06_report_dsm_erro(g_ft6x06_ts, tp_dclient, DSM_TP_I2C_RW_ERROR_NO, ret);

            return -1;
	}
    
	return ret;
}
/*write data by i2c*/
int ft6x06_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(client->adapter, msg, 1);
    
    if (ret <= 0)
    {
        tp_log_err("%s %d: i2c_transfer return: %d\n", __func__, __LINE__,ret);

        ft6x06_report_dsm_erro(g_ft6x06_ts, tp_dclient, DSM_TP_I2C_RW_ERROR_NO, ret);
        
        return -1;
    }
		
	return ret;
}
int ft6x06_download_i2c_Read(unsigned char *writebuf,
		    int writelen, unsigned char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = 0x38,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = 0x38,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(g_i2c_client->adapter, msgs, 2);
		if (ret < 0)
			tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,ret);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = 0x38,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(g_i2c_client->adapter, msgs, 1);
		if (ret < 0)
			tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,ret);
	}
	
	if(0 == ret)
	{
		tp_log_err("%s %d: i2c_transfer return: %d\n", __func__, __LINE__,ret);
	    return -1;
	}
	
	return ret;
}
/*write data by i2c*/
int ft6x06_download_i2c_Write(unsigned char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = 0x38,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(g_i2c_client->adapter, msg, 1);
	if (ret < 0)
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,ret);

	if(0 == ret)
	{
		tp_log_err("%s %d: i2c_transfer return: %d\n", __func__, __LINE__,ret);
	    return -1;
	}
		
	return ret;
}

/*to reduce the ft6x06_ts_release because in report_data has been done*/

/*Read touch point information when the interrupt  is asserted.*/
static int ft6x06_read_Touchdata(struct ft6x06_ts_data *data)
{
	struct ts_event *event = &data->event;
	u8 buf[POINT_READ_BUF] = { 0 };
	int ret = -1;
	int i = 0;
	u8 pointid = FT_MAX_ID;
	tp_log_debug("in ft6x06_read_Touchdata\n");
	ret = ft6x06_i2c_Read(data->client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) {
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,ret);
		return ret;
	}
	memset(event, 0, sizeof(struct ts_event)); 

	//when ep wake up, it will report (0,0), those code use to filtrate te point
	if (((buf[2] & 0x0f) == 0) && ((buf[3] & 0xc0) == 0))
	{
		tp_log_debug("%s %d:error data:buf[2] = 0x%2x, buf[3] = 0x%2x\n", 
				__func__, __LINE__, buf[2], buf[3]);
		return -EINVAL;
	}

	event->touch_point = 0;
	for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++) {
		pointid = (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		if (pointid >= FT_MAX_ID)
			break;
		else
			event->touch_point++;
		event->au16_x[i] =
		    (s16) (buf[FT_TOUCH_X_H_POS + FT_TOUCH_STEP * i] & 0x0F) <<
		    8 | (s16) buf[FT_TOUCH_X_L_POS + FT_TOUCH_STEP * i];
		event->au16_y[i] =
		    (s16) (buf[FT_TOUCH_Y_H_POS + FT_TOUCH_STEP * i] & 0x0F) <<
		    8 | (s16) buf[FT_TOUCH_Y_L_POS + FT_TOUCH_STEP * i];
		event->au8_touch_event[i] =
		    buf[FT_TOUCH_EVENT_POS + FT_TOUCH_STEP * i] >> 6;
		event->au8_finger_id[i] =
		    (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
	}
	
	event->pressure = FT_PRESS;
	tp_log_debug("out ft6x06_read_Touchdata\n");
	return 0;
}

/*
*report the point information
*/
u8 g_key_state = 0; // 1-down 0-up
u8 g_key_value = 0;
static void ft6x06_report_value(struct ft6x06_ts_data *data)
{
	struct ts_event *event = &data->event;
	int i = 0;
	u8 uppoint = 0;
	
	tp_log_debug("in ft6x06_report_value\n");
	tp_log_debug("event->touch_point = %d\n",event->touch_point);
	//u8 key_value;
	for (i = 0; i < event->touch_point; i++) {
		/* LCD view area */
		tp_log_debug("touch_point[%d]\n",i);
		//check if the data should be reversal
		if (reversal_data)
		{
			//if the data is touch area, reversal,
			//otherwise, do not reversal
			if (event->au16_x[i] < data->x_max
				&& event->au16_y[i] < data->y_max)
			{
				event->au16_x[i] = data->x_max - event->au16_x[i];
				event->au16_y[i] = data->y_max - event->au16_y[i];
			}
		}
		
		if (event->au8_touch_event[i] == FTS_POINT_DOWN || event->au8_touch_event[i] == FTS_POINT_CONTACT)
		{
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,event->au16_x[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,event->au16_y[i]);
			input_report_abs(data->input_dev, ABS_MT_PRESSURE,event->pressure);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID,event->au8_finger_id[i]);
			input_report_abs(data->input_dev,ABS_MT_TOUCH_MAJOR,event->pressure);
		} 	
		else 
		{
			uppoint++;
		}
		
		input_mt_sync(data->input_dev);
		tp_log_debug("input_mt_sync ok\n");
	}
	/*if touch_point equal to the point_up numbers*/
	if(event->touch_point == uppoint)
	{
		input_report_key(data->input_dev,BTN_TOUCH, 0);
	}
	else
	{
		input_report_key(data->input_dev,BTN_TOUCH, event->touch_point > 0);
	}
	
	input_sync(data->input_dev);
}

/*The ft6x06 device will signal the host about TRIGGER_FALLING.
*Processed when the interrupt is asserted.
*/
static irqreturn_t ft6x06_ts_interrupt(int irq, void *dev_id)
{
	struct ft6x06_ts_data *ft6x06_ts = dev_id;
	int ret = 0;
	tp_log_debug("in ft6x06_ts_interrupt\n");
	disable_irq_nosync(ft6x06_ts->irq);

	ret = ft6x06_read_Touchdata(ft6x06_ts);
	if (ret == 0)
		ft6x06_report_value(ft6x06_ts);

	enable_irq(ft6x06_ts->irq);
	tp_log_debug("out ft6x06_ts_interrupt\n");
	return IRQ_HANDLED;
}

static struct of_device_id focaltech_i2c_of_match[] = {
	{ .compatible = "ft6x06,ft6x06_i2c_adapter", }, { }
};
MODULE_DEVICE_TABLE(of, focaltech_i2c_of_match);
/*
*ft6x06_ts_suspend-it is suspended when the tp will be sleep,if you would the tp sleep,
*you can send REG and data to ft6x06.
*@dev the struct of user defined.
*
*Returns -1 on errno, else 0.
*
*
*/
#if defined(CONFIG_PM_RUNTIME)
static int ft6x06_ts_suspend(struct device *dev)
{
	int ret = 0;
	unsigned char tx_buf[2] = {0,0};
	struct ft6x06_ts_data * ts = container_of(dev, struct ft6x06_ts_data, dev);
	tp_log_info("in %s irq = %d\n",__func__,ts->irq);
	if(ts->is_suspended == FT6X06_FLAG_WAKE_LOCK)
		return 0;
	disable_irq(ts->irq);
    
	/*send REG and DATA to FT6X06 to goto sleep.*/
	tx_buf[0] = FT6X06_REG_PMODE;
	tx_buf[1] = FT6X06_PMODE_HIBERNATE;
	ret = ft6x06_i2c_Write(ts->client, tx_buf, sizeof(tx_buf));
	tp_log_debug("write to suspend irq = %d ret = %d\n",ts->irq,ret);
	if(ret < 0)
	{
		ret = ft6x06_i2c_Write(ts->client, tx_buf, sizeof(tx_buf));
		tp_log_debug("write to suspend again irq = %d ret = %d\n",ts->irq,ret);
	}
	return 0;
}
/*
*ft6x06_ts_resume-it is sesume when the tp will be unsleep,if you would the tp unsleep,
*you can control the ic which is ft6x06 to reset,but it is also depend on LCD. 
*@dev the struct of user defined.
*
*Returns -1 on errno, else 0.
*
*
*/
static int ft6x06_ts_resume(struct device *dev)
{
	
	struct ft6x06_ts_data *ts = container_of(dev,struct ft6x06_ts_data, dev);
	tp_log_info("in %s irq = %d\n",__func__,ts->irq);
	if(ts->is_suspended == FT6X06_FLAG_WAKE_LOCK)
		return 0;
	gpio_set_value(ts->pdata->reset, 0);
	msleep(10);
	gpio_set_value(ts->pdata->reset, 1);
	
	msleep(100);
    
	enable_irq(ts->irq);
	return 0;
}
#endif
/*
*fb_notifier_callback-it is the callback,FB_BLANK_UNBLANK and FB_BLANK_POWERDOWN by lcd
*@self 	the notifier_block input 
*@event the event define by lcd,FB_EVENT_BLANK and FB_BLANK_POWERDOWN
*@data   the data comes from lcd
*Returns -1 on errno, else 0.
*
*
*/
#if !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
	unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank = NULL;
	struct ft6x06_ts_data  *ft6x06_ts =container_of(self, struct ft6x06_ts_data, fb_notif);

	tp_log_info("%s %d:evdata = %d, evdata->data = %d, event = %d, ft6x06_ts = %d, suspend_flag = %d.\n",
        __func__, __LINE__, (int)evdata, (int)(evdata->data), (int)event, (int)ft6x06_ts, (int)ft6x06_ts->suspend_resume_flag);
	
	if (evdata && evdata->data && event == FB_EVENT_BLANK && ft6x06_ts) {
		blank = evdata->data;
		tp_log_info( "%s:%d blank = %d\n",__func__,__LINE__, *blank);
		/*In case of resume we get BLANK_UNBLANK and for suspen BLANK_POWERDOWN*/
		
		if (*blank == FB_BLANK_UNBLANK)
		{
			if(IS_TP_SUSPENDED == ft6x06_ts->suspend_resume_flag)
			{
			    ft6x06_ts_resume(&ft6x06_ts->dev);
				ft6x06_ts->suspend_resume_flag = NOT_TP_SUSPENDED;
			}
		}
		else if (*blank == FB_BLANK_POWERDOWN)
		{
		    if(NOT_TP_SUSPENDED == ft6x06_ts->suspend_resume_flag)
			{
		        ft6x06_ts_suspend(&ft6x06_ts->dev);
				ft6x06_ts->suspend_resume_flag = IS_TP_SUSPENDED;
			}
		}
	}
	return 0;
}
 /*
*configure_sleep-register the callback to the FB
*@ft6x06_ts the struct of ft6x06_ts_data.
*
*
*/
static int configure_sleep(struct ft6x06_ts_data *ft6x06_ts)
{
	int rc = 0;
	tp_log_info("%s,%d\n",__func__,__LINE__);
	ft6x06_ts->fb_notif.notifier_call = fb_notifier_callback;

	/*register the callback to the FB*/
	rc = fb_register_client(&ft6x06_ts->fb_notif);
	if (rc)
	{	
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		return -EINVAL;
	}
	return 0;
}
#endif
 
static const struct dev_pm_ops ft6x06_core_pm_ops = {
	SET_RUNTIME_PM_OPS(ft6x06_ts_suspend, ft6x06_ts_resume,
			NULL)
};
static void ft6x06_power_and_gpio_del(struct device *dev,struct ft6x06_platform_data *pdata)
{
	gpio_free(pdata->reset);
	gpio_free(pdata->irq);
	if (!IS_ERR(vbus_focaltech))
		regulator_disable(vbus_focaltech);
	if (!IS_ERR(vbus_focaltech))
		regulator_put(vbus_focaltech);
	if (!IS_ERR(vdd_focaltech))
		regulator_disable(vdd_focaltech);
	if (!IS_ERR(vdd_focaltech))
		regulator_put(vdd_focaltech);
}
static int ft6x06_power_and_gpio_init(struct device *dev,struct ft6x06_platform_data *pdata)
{
	int rc =0;
	/* VDD power on */
	vdd_focaltech= regulator_get(dev, pdata->power_pin_vdd);
	if (IS_ERR(vdd_focaltech)) {
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		rc =  -EINVAL;
		goto regulator_get_vdd_fail;
	}

	rc = regulator_set_voltage(vdd_focaltech,2850000,2850000);
	if(rc < 0){
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		rc =  -EINVAL;
		goto regulator_set_voltage_vdd_fail;
	}

	rc = regulator_enable(vdd_focaltech);
	if (rc < 0) {
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		rc =  -EINVAL;
		goto regulator_enable_vdd_fail;
	}
	
	/* VBUS power on */
	vbus_focaltech = regulator_get(dev, pdata->power_pin_vbus);
	if (IS_ERR(vbus_focaltech)) {
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		rc = -EINVAL;
		goto regulator_get_vbus_fail;
	}
	rc = regulator_set_voltage(vbus_focaltech,1800000,1800000);
	if(rc < 0){
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		rc = -EINVAL;
		goto regulator_set_voltage_vbus_fail;
	}
 
	rc = regulator_enable(vbus_focaltech);
	if (rc < 0) {
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		rc =  -EINVAL;
		goto regulator_enable_vbus_fail;
	}
 	
	mdelay(1);
	/* init irq gpio */
    rc = gpio_request(pdata->irq, "focaltech");
	if (rc < 0) {
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		gpio_free(pdata->irq);
		rc = gpio_request(pdata->irq,"focaltech");
	}

	if (rc < 0) {
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		rc =  -EINVAL;
		goto gpio_request_irq_fail;
	} else {
		/*rc = gpio_tlmm_config(GPIO_CFG(pdata->irq,0,GPIO_CFG_INPUT,GPIO_CFG_PULL_UP,GPIO_CFG_8MA),GPIO_CFG_ENABLE);
		if(rc < 0) {
			tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
			rc =  -EINVAL;
			goto gpio_config_irq_fail;
		}*/
		rc = gpio_direction_input(pdata->irq);
		if(rc < 0) {
			tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
			rc =  -EINVAL;
			goto gpio_direction_input_irq_fail;
		}
	}
	/*init reset gpio*/
	rc = gpio_request(pdata->reset, "focaltech_reset");
	if (rc < 0) {
		gpio_free(pdata->reset);
		rc = gpio_request(pdata->reset,"focaltech_reset");
	}

	if (rc < 0) {
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		rc =  -EINVAL;
		goto gpio_request_reset_fail;
	} else {
		/*rc = gpio_tlmm_config(GPIO_CFG(pdata->reset,0,GPIO_CFG_OUTPUT,GPIO_CFG_PULL_UP,GPIO_CFG_8MA),GPIO_CFG_ENABLE);
		if(rc < 0) {
			tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
			rc =  -EINVAL;
			goto gpio_config_reset_fail;
		}*/
		rc = gpio_direction_output(pdata->reset,1);
		if(rc < 0) {
			tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
			rc =  -EINVAL;
			goto gpio_direction_output_reset_fail;
		}
		msleep(10);
		rc = gpio_direction_output(pdata->reset,0);
		if(rc < 0) {
			tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
			rc = -EINVAL;
			goto gpio_direction_output_reset_fail;
		}
		msleep(10);
		rc = gpio_direction_output(pdata->reset,1);
		if(rc < 0) {
			tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
			rc = -EINVAL;
			goto gpio_direction_output_reset_fail;
		}
		msleep(2);
	}

	pdata->irqflags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;

	tp_log_debug("power_pin_vdd=%s,power_pin_vbus=%s,irq_gpio_num=%d,tp_reset_num=%d,tp_x_max = %d,tp_y_max = %d\n", \
		pdata->power_pin_vdd,pdata->power_pin_vbus,pdata->irq,pdata->reset, \
		pdata->x_max,pdata->y_max);
	return 0;
gpio_direction_output_reset_fail:
//gpio_config_reset_fail:
	gpio_free(pdata->reset);
gpio_request_reset_fail:
gpio_direction_input_irq_fail:
//gpio_config_irq_fail:
	gpio_free(pdata->irq);
gpio_request_irq_fail:
	if (!IS_ERR(vbus_focaltech))
		regulator_disable(vbus_focaltech);
regulator_enable_vbus_fail:
regulator_set_voltage_vbus_fail:
	if (!IS_ERR(vbus_focaltech))
		regulator_put(vbus_focaltech);
regulator_get_vbus_fail:
	if (!IS_ERR(vdd_focaltech))
		regulator_disable(vdd_focaltech);
regulator_enable_vdd_fail:
regulator_set_voltage_vdd_fail:
	if (!IS_ERR(vdd_focaltech))
		regulator_put(vdd_focaltech);
regulator_get_vdd_fail:
	return rc;
}
/*
*ft6x06_parse_dt- read some data which is defined by self from devicetreee .
*@dev the client->dev which is read from DTS.
*@ft6x06_platform_data the platform struct .
*
*/
static int ft6x06_parse_dt(struct device *dev,struct ft6x06_platform_data *pdata)
{
	int rc = 0;
	
	rc = of_property_read_string(dev->of_node,"ft6x06,vdd", &pdata->power_pin_vdd);
	if (rc) {
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		goto error_alloc_data_failed;
	}
	
	rc= of_property_read_string(dev->of_node,"ft6x06,vbus", &pdata->power_pin_vbus);
	if (rc) {
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		goto error_alloc_data_failed;
	}
	
	/*rc = of_property_read_u32(dev->of_node,"ft6x06,irq_gpio_num", &pdata->irq);
	if (rc) {
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		goto error_alloc_data_failed;
	}
	
	rc = of_property_read_u32(dev->of_node,"ft6x06,tp_reset_num", &pdata->reset);
	if (rc) {
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		goto error_alloc_data_failed;
	}*/
	pdata->reset = of_get_named_gpio_flags(dev->of_node,
			"ft6x06,tp_reset_num", 0, &pdata->reset_flags);

	pdata->irq = of_get_named_gpio_flags(dev->of_node,
			"ft6x06,irq_gpio_num", 0, &pdata->irq_flags);

	
	tp_log_info("Info:%s,line=%d,reset=%d, irq=%d, reset_flag = %d, irq_flag = %d.\n", 
		__func__, __LINE__,pdata->reset, pdata->irq, pdata->reset_flags, pdata->irq_flags);


	rc = of_property_read_u32(dev->of_node,"ft6x06,tp_x_max", &pdata->x_max);
	if (rc) {
		tp_log_info("Info:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		goto error_alloc_data_failed;
	}
	rc = of_property_read_u32(dev->of_node,"ft6x06,tp_y_max", &pdata->y_max);
	if (rc) {
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		goto error_alloc_data_failed;
	}
	rc = of_property_read_u32(dev->of_node, "ft6x06,reversal_data", &reversal_data);
	if (rc) {
		tp_log_info("Info:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		reversal_data = 0;
	}

        rc = of_property_read_string(dev->of_node,"ft6x06,product_name", &pdata->product_name);
        if(rc)
        {
            tp_log_info("%s %d: no product_id configer in dtsi, use default value\n", __func__, __LINE__);
            pdata->product_name = "unkown_product";
        }
	
	return 0;
error_alloc_data_failed:
	return rc;
}



struct kobject *ft6x06_virtual_key_properties_kobj = NULL;

static ssize_t ft6x06_virtual_keys_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int index = 0, size = 0,i = 0;
	u32  *data = NULL;
	struct ft6x06_virtual_keys_data *virtual_keys_pdata = NULL;
	virtual_keys_pdata = container_of(attr,
		struct ft6x06_virtual_keys_data, kobj_attr);
	tp_log_info("%s %d\n",__func__,__LINE__);
	size = virtual_keys_pdata->size;
	data = virtual_keys_pdata->data;
	tp_log_info("data[0] = %d,size = %d\n",data[0],size);
	for (i = 0; i < size; i += FT6X06_VIRTUAL_KEY_ELEMENT_SIZE)
		index += scnprintf(buf + index, FT6X06_MAX_VKEY_LEN - index,
			"0x01:%d:%d:%d:%d:%d\n",
			data[i], data[i+1], data[i+2], data[i+3], data[i+4]);
	tp_log_info("%s %d\n",__func__,__LINE__);
	return index;
}

static int ft6x06_init_virtual_key(struct device_node *dev_node,
		const char *inp_dev_name, struct ft6x06_core_data *ft6x06_core_pdata)
{
	int rc = 0;
	char *name = NULL;
	u32 *data = NULL;
	size_t size = 3*FT6X06_VIRTUAL_KEY_ELEMENT_SIZE;
	
	struct ft6x06_virtual_keys_data *virtual_keys_pdata = NULL;
	
	if(!dev_node)
	{
		rc = -EINVAL;
		goto dev_node_invaild;
	}
	tp_log_info("%s,%d inp_dev_name = %s\n",__func__,__LINE__,inp_dev_name);
	name = kzalloc(FT6X06_VIRTUAL_NAME_MAX_LENGTH, GFP_KERNEL);
	if (name == NULL) {
		rc = -ENOMEM;
		goto fail_alloc_data;
	}
	snprintf(name, FT6X06_VIRTUAL_NAME_MAX_LENGTH, "virtualkeys.%s", inp_dev_name);
	
	virtual_keys_pdata = kzalloc(sizeof(struct ft6x06_virtual_keys_data), GFP_KERNEL);
	if(virtual_keys_pdata == NULL)
	{
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		rc = -ENOMEM;
		goto alloc_vritual_size_fail;
	}
	data = kzalloc(size*sizeof(u32), GFP_KERNEL);
	if(data == NULL)
	{
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		rc = -ENOMEM;
		goto alloc_size_fail;
	}
	rc = of_property_read_u32_array(dev_node,"ft6x06,virtual_keys",data,size);
	if (rc) {
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		rc = -EINVAL;
		goto read_devtree_fail;
	}
	for(rc = 0;rc < 15; rc++)
		tp_log_debug("data[%d] = %d\n",rc,data[rc]); 
	
	virtual_keys_pdata->data = data;
	virtual_keys_pdata->size  = size;
	
	if (ft6x06_virtual_key_properties_kobj == NULL)
		ft6x06_virtual_key_properties_kobj =
			kobject_create_and_add("board_properties", NULL);
	if (ft6x06_virtual_key_properties_kobj == NULL) {
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		rc = -EINVAL;
		goto kobject_creat_properties_fail;
	}
	/* Initialize ft6x06 SysFs attribute */
	sysfs_attr_init(&virtual_keys_pdata->kobj_attr.attr);
	virtual_keys_pdata->kobj_attr.attr.name = name;
	virtual_keys_pdata->kobj_attr.attr.mode = S_IRUGO;
	virtual_keys_pdata->kobj_attr.show     = ft6x06_virtual_keys_show;
	rc = sysfs_create_file(ft6x06_virtual_key_properties_kobj, &virtual_keys_pdata->kobj_attr.attr);
	if (rc)
	{
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		rc = -EINVAL;
		goto fail_del_kobj;
	}
	
  	ft6x06_core_pdata->virtual_keys_pdata = virtual_keys_pdata;

	
	return 0;

fail_del_kobj:
	virtual_keys_pdata->kobj_attr.attr.name = NULL;
	kobject_del(ft6x06_virtual_key_properties_kobj);
kobject_creat_properties_fail:
	virtual_keys_pdata->data = NULL;
read_devtree_fail:
	kfree(data);
alloc_size_fail:
	kfree(virtual_keys_pdata);
alloc_vritual_size_fail:
	kfree(name);
fail_alloc_data:
dev_node_invaild:
	return rc;

}
/*del the ft6x06_init_virtual_key of struct */
static void ft6x06_del_virtual_key(struct device_node *dev_node,
		const char *inp_dev_name, struct ft6x06_core_data *ft6x06_core_pdata)
{
	sysfs_remove_file(ft6x06_virtual_key_properties_kobj, &ft6x06_core_pdata->virtual_keys_pdata->kobj_attr.attr);
	kobject_del(ft6x06_virtual_key_properties_kobj);
	
	kfree(ft6x06_core_pdata->virtual_keys_pdata->data);
	ft6x06_core_pdata->virtual_keys_pdata->data = NULL;
	ft6x06_core_pdata->virtual_keys_pdata->size  = 0;
	kfree(ft6x06_core_pdata->virtual_keys_pdata->kobj_attr.attr.name);
	ft6x06_core_pdata->virtual_keys_pdata->kobj_attr.attr.name = NULL;
	kfree(ft6x06_core_pdata->virtual_keys_pdata);
}

static void ft6x06_unregister_virtual_key_device(struct device_node *dev_node,struct ft6x06_ts_data *ft6x06_ts)
{
	ft6x06_del_virtual_key(dev_node,NULL,ft6x06_ts->ft6x06_core_pdata);
	
	kfree(ft6x06_ts->ft6x06_core_pdata);
	ft6x06_ts->ft6x06_core_pdata = NULL;
}
static void ft6x06_devtree_unregister_devices(struct device *dev,struct ft6x06_ts_data *ft6x06_ts)
{
	ft6x06_unregister_virtual_key_device(dev->of_node,ft6x06_ts);
}
static int ft6x06_register_virtual_key_device(struct device_node *dev_node,struct ft6x06_ts_data *ft6x06_ts)
{
	char const *name;
	int rc;
	struct ft6x06_core_data *ft6x06_core_pdata = NULL;
	if (!dev_node)
	{
		rc = -EINVAL;
		goto dev_node_invaild;
	}
	ft6x06_core_pdata	= kzalloc(sizeof(*ft6x06_core_pdata), GFP_KERNEL);
	if (ft6x06_core_pdata == NULL) {
		rc = -ENOMEM;
		goto alloc_core_data_fail;
	}
	rc = of_property_read_string(dev_node, "ft6x06,virtual_keys_name", &name);
	if (rc) {
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		rc = -EINVAL;
		goto of_read_string_fail;
	} else
		tp_log_debug("%s: OF ft6x06,virtual_keys_name: %s\n", __func__, name);
	/*init virtual keys */
	
    rc = ft6x06_init_virtual_key(dev_node,name,ft6x06_core_pdata);
	
	if(rc)
	{
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		rc = -EINVAL;
		goto init_virtual_key_fail;
	}
	ft6x06_ts->ft6x06_core_pdata = ft6x06_core_pdata;
	return 0;
init_virtual_key_fail:
of_read_string_fail:
	kfree(ft6x06_core_pdata);
alloc_core_data_fail:
dev_node_invaild:
	return rc;
}

/*register vritual device and cap device,so on*/
static int ft6x06_devtree_register_devices(struct device *dev,struct ft6x06_ts_data *ft6x06_ts)
{
	int rc;
	struct device_node *dev_node = dev->of_node;
	tp_log_info("%s,%d\n",__func__,__LINE__);
	if (!dev_node)
	{
		rc = -EINVAL;
		goto dev_node_invaild;
	}
	rc = ft6x06_register_virtual_key_device(dev_node,ft6x06_ts);
	if(rc)
	{
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		rc =  -EINVAL;
		goto ft6x06_register_virtual_key_fail;
	}
	tp_log_info("%s,%d\n",__func__,__LINE__);
	return 0; 
	
ft6x06_register_virtual_key_fail:
dev_node_invaild:
	return rc ;
}
static int ft6x06_set_input_dev(struct ft6x06_ts_data *ft6x06_ts)
{
	int rc =0;
	struct input_dev *input_dev = NULL;
	
	input_dev = input_allocate_device();
	if (!input_dev) {
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		rc = -ENOMEM;
		goto exit_input_dev_alloc_failed;
	}
	ft6x06_ts->input_dev = input_dev;

	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_PRESSURE, input_dev->absbit);
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
	//set_bit(BTN_TOOL_FINGER, input_dev->keybit);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, ft6x06_ts->x_max -1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, ft6x06_ts->y_max -1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, CFG_MAX_TOUCH_POINTS, 0, 0);

	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_SYN, input_dev->evbit);
	input_dev->name = FT6X06_I2C_NAME;
	rc = input_register_device(input_dev);
	if (rc) { 
		tp_log_err("error:%s,line=%d,rc=%d,input device:%s\n", __func__, __LINE__,rc,dev_name(&ft6x06_ts->client->dev));
		goto exit_input_register_device_failed;
	}
	return 0;

	
exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
	return rc;
}

/********************************************************************************************
Parameters    :  Vendor_ID   :vendor id
                 module_name :buffer to store module name 
                 size        :size of module_name buffer
Return        :  the length of moudule name
Description   :  get module name by vendor id, if vendor id is not in this function, 
				 the module name will be set to Vendor_ID:@Vendor_ID
*********************************************************************************************/
static int ft6x06_get_touch_module_name(unsigned char  Vendor_ID, char *module_name, int size)
{
	int length = 0;

	switch(Vendor_ID)
	{
		case JUN_VENDOR_ID:
			length = snprintf(module_name, size - 1, "%s", "Junda");
			break;
        case LANSI_VENDOR_ID:
		case LANSI_VENDOR_ID2:
			length = snprintf(module_name, size - 1, "%s", "Lansi");
            break;
		default:
			length = snprintf(module_name, size - 1, "ID:%d", Vendor_ID);			
			break;
	}

	module_name[length] = 0x00;
	return length;
}
void ft6x06_set_app_info_touchpanel(struct ft6x06_ts_data *ft6x06_ts)
{
	int rc = 0;
	char module_name[APP_INFO_VALUE_LENTH] = {0};
	
	ft6x06_get_touch_module_name(ft6x06_ts->vendor_id_value, module_name, APP_INFO_VALUE_LENTH);
	snprintf(touch_info, sizeof(touch_info),"Focaltech_Ft6x36_%s.%d",module_name,ft6x06_ts->fw_ver_reg_value);
	#ifdef CONFIG_APP_INFO
	rc = app_info_set("touch_panel", touch_info);
	#endif
	if (rc) 
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
}

static int ft6x06_check_device(struct i2c_client *client, struct ft6x06_ts_data *ft6x06_ts)
{
    int rc = 0;
    
    if(!ft6x06_ts)
    {
        tp_log_err("%s %d: ft6x06_ts is null!", __func__, __LINE__);
        return -1;
    }
    
    rc = ft6x36_get_factory_id(i2c_client, &ft6x06_ts->vendor_id_value);
    if(rc < 0)
    {
        tp_log_err("%s %d: get vendor id fail: rc = %d\n", __func__, __LINE__,rc);
    }
    
    return rc;
}

/*
*ft6x06_ts_probe- the probe function .
*@client	 the client which will be matching by platform.
*@i2c_device_id the i2c_device_id struct .
*
*/
static int ft6x06_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct ft6x06_platform_data *pdata = NULL;
	struct ft6x06_ts_data *ft6x06_ts = NULL;
	struct device *dev = &client->dev;
	unsigned char uc_reg_addr = 0;
	unsigned char uc_reg_value = 0;
	int rc = 0;

	i2c_client=client;

	tp_log_info( "%s: Starting %s probe...\n", __func__, FT6X06_I2C_NAME);
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENODEV;
		goto exit_check_functionality_failed;
	}
	if (client->dev.of_node) 
	{
		pdata = kzalloc(sizeof(struct ft6x06_platform_data), GFP_KERNEL);
		if(!pdata)
		{
			rc = -ENOMEM;
			goto exit_alloc_pdata_failed;
		}
		rc = ft6x06_parse_dt(dev,pdata);
		if(rc)
			goto parse_dt_fail;
	}
	else 
	{
		pdata = client->dev.platform_data;
		if (!pdata) 
		{
			tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
			rc  = -EINVAL;
			goto get_platform_data_fail;
		}
	}
	/*enable power and request gpio*/
	rc = ft6x06_power_and_gpio_init(dev,pdata);
	if(rc)
	{
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		goto power_and_gpio_init_fail;
	}
	/*it can read IC-REG after  ft6x06 init over */
	msleep(150);
	
	ft6x06_ts = kzalloc(sizeof(struct ft6x06_ts_data), GFP_KERNEL);
	if (!ft6x06_ts) {
		rc = -ENOMEM;
		goto exit_alloc_data_failed;
	}

        if(ft6x06_check_device(client, ft6x06_ts))
        {
            tp_log_err("%s %d: check device fail!", __func__, __LINE__);
            goto exit_check_device_fail;
        }
        else
        {
            tp_log_info("%s %d: check device success: vendor id=%d\n", 
            __func__, __LINE__, ft6x06_ts->vendor_id_value);
        }

	rc = ft6x06_devtree_register_devices(dev,ft6x06_ts);
	if(rc)
	{	
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		goto  devtree_register_devices_fail;
	}
	else
		tp_log_debug("%s success: line=%d rc=%d\n", __func__, __LINE__,rc);
	/* Initialize IRQ */	
	client->irq  = gpio_to_irq(pdata->irq);
	if (client->irq < 0) 
	{
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		rc = -EINVAL;
		goto gpio_to_irq_fail;
	}
	else
		tp_log_debug("client->irq = %d,client->addr = 0x%x\n",client->irq,client->addr);
	
	ft6x06_ts->irq = client->irq;
	ft6x06_ts->client = client;
	ft6x06_ts->pdata = pdata;
	ft6x06_ts->x_max = pdata->x_max ;
	ft6x06_ts->y_max = pdata->y_max ;

	/*init is_suspended*/
	ft6x06_ts->is_suspended = FT6X06_FLAG_WAKE_UNLOCK;
	
	ft6x06_ts->suspend_resume_flag = NOT_TP_SUSPENDED;
	
	i2c_set_clientdata(client, ft6x06_ts);
	dev_set_drvdata(&ft6x06_ts->dev, ft6x06_ts);
	
	#if !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_FB)
	rc = configure_sleep(ft6x06_ts);
	if(rc)
	{
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		goto configure_sleep_fail;
	}
	#endif

	rc = ft6x06_set_input_dev(ft6x06_ts);
	if(rc)
	{
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		goto ft6x06_set_input_dev_fail;
	}
	
	rc = request_threaded_irq(client->irq, NULL,ft6x06_ts_interrupt,
				   pdata->irqflags, "focaltech_irq",
				   ft6x06_ts);
	if (rc < 0) {
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		goto request_irq_fail;
	}
	disable_irq(client->irq);
	
	/*make sure CTP already finish startup process */
	//msleep(150);
	/*debug */
	if(hw_tp_common_debug_mask >= TP_DBG)
	{
		/*get some register information */
		uc_reg_addr = FT6x06_REG_FW_VER;
		rc = ft6x06_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
		if(rc < 0)
			tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		else 
			tp_log_debug("Firmware version = 0x%x\n", uc_reg_value);
		
		uc_reg_addr = FT6x06_REG_POINT_RATE;
		rc = ft6x06_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
		if(rc < 0)
			tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		else 
			tp_log_debug("report rate is %dHz.\n",uc_reg_value * 10);

		uc_reg_addr = FT6x06_REG_THGROUP;
		rc = ft6x06_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
		if(rc < 0)
			tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		else
			tp_log_debug("touch threshold is %d.\n",uc_reg_value * 4);
	}
#ifdef CONFIG_FOCALTECH_FT6X06_SYSDEBUG
	ft6x06_create_sysfs(client);
#endif

	ft6x06_create_sscap_sysfs(client);
	
#ifdef CONFIG_FOCALTECH_FOCALTECH_USERDATA
	if (ft_rw_iic_drv_init(client) < 0) 
		tp_log_err("error:%s,line=%d\n", __func__, __LINE__);
#endif 
	/*set app_info of touchpanel*/
	//ft6x06_set_app_info_touchpanel(ft6x06_ts);

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
    set_hw_dev_flag(DEV_I2C_TOUCH_PANEL);
#endif

	enable_irq(client->irq);
#ifdef CONFIG_FOCALTECH_FT6X06_UPDATA_FW
	
	INIT_DELAYED_WORK(&ft6x06_ts->ft6x06_fw_config_delay, fts_ctpm_auto_upgrade);
	schedule_delayed_work(&ft6x06_ts->ft6x06_fw_config_delay, msecs_to_jiffies(FT6X06_UPDATE_WAIT_TIMEOUT));

#endif

    g_ft6x06_ts = ft6x06_ts;
    if (!tp_dclient) {
        dsm_i2c.fops->dump_func = ft6x06_tp_dump;
        tp_dclient = dsm_register_client(&dsm_i2c);
        tp_log_info("%s %d: Successful register dsm_client!\n", __func__, __LINE__);
    }
    
	tp_log_info( "%s: Successful probe %s\n", __func__, FT6X06_I2C_NAME);
	return 0;

request_irq_fail:
	input_unregister_device(ft6x06_ts->input_dev);
	input_free_device(ft6x06_ts->input_dev);
ft6x06_set_input_dev_fail:
#if !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_FB)
	fb_unregister_client(&ft6x06_ts->fb_notif);
configure_sleep_fail:
#endif
	i2c_set_clientdata(client, NULL);
	dev_set_drvdata(&ft6x06_ts->dev, NULL);
	ft6x06_ts->pdata = NULL;
	ft6x06_ts->client = NULL;
gpio_to_irq_fail:
	ft6x06_devtree_unregister_devices(dev,ft6x06_ts);
devtree_register_devices_fail:
exit_check_device_fail:
	kfree(ft6x06_ts);
exit_alloc_data_failed:
        ft6x06_power_and_gpio_del(dev,pdata);
power_and_gpio_init_fail:
parse_dt_fail:
	kfree(pdata);
exit_alloc_pdata_failed:
get_platform_data_fail:
exit_check_functionality_failed:
	return rc;
}

/*
*ft6x06_ts_remove- the remove function.
*@client	 the client which will be matching by platform.
*
*/
static int  ft6x06_ts_remove(struct i2c_client *client)
{
	struct ft6x06_ts_data *ft6x06_ts;
	ft6x06_ts = i2c_get_clientdata(client);
	
	#ifdef CONFIG_FOCALTECH_FT6X06_UPDATA_FW
	cancel_delayed_work_sync(&ft6x06_ts->ft6x06_fw_config_delay);
	#endif
	
	#ifdef CONFIG_FOCALTECH_FOCALTECH_USERDATA
	ft_rw_iic_drv_exit();
	#endif 
	
	#ifdef CONFIG_FOCALTECH_FT6X06_SYSDEBUG
	ft6x06_release_sysfs(client);
	#endif
	
	ft6x06_release_sscap_sysfs(client);
	
	input_unregister_device(ft6x06_ts->input_dev);
	input_free_device(ft6x06_ts->input_dev);
	
	#if !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_FB)
	fb_unregister_client(&ft6x06_ts->fb_notif);
	#endif

	i2c_set_clientdata(client, NULL);
	free_irq(client->irq, ft6x06_ts);
	ft6x06_devtree_unregister_devices(&client->dev,ft6x06_ts);
	kfree(ft6x06_ts);
	return 0;
}

static const struct i2c_device_id ft6x06_ts_id[] = {
	{FT6X06_I2C_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ft6x06_ts_id);


static struct i2c_driver ft6x06_ts_driver = {
	.probe = ft6x06_ts_probe,
	.remove = ft6x06_ts_remove,
	.id_table = ft6x06_ts_id,
	.driver = {
		   .name = FT6X06_I2C_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = focaltech_i2c_of_match,
		   #if !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_FB)
		   .pm = &ft6x06_core_pm_ops,
		   #endif
		   },
};

static int __init ft6x06_ts_init(void)
{
	int ret;
	ret = i2c_add_driver(&ft6x06_ts_driver);
	if (ret) {
		tp_log_err("Adding ft6x06 driver failed (errno = %d)\n", ret);
	} else {
		tp_log_info("Successfully added driver %s\n", ft6x06_ts_driver.driver.name);
	}
	return ret;
}

static void __exit ft6x06_ts_exit(void)
{
	i2c_del_driver(&ft6x06_ts_driver);
}

module_init(ft6x06_ts_init);
module_exit(ft6x06_ts_exit);

MODULE_AUTHOR("<luowj>");
MODULE_DESCRIPTION("FocalTech ft6x06 TouchScreen driver");
MODULE_LICENSE("GPL");
