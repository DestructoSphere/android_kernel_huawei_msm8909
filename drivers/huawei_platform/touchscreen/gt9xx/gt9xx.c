/* drivers/input/touchscreen/gt9xx_hw/gt9xx.c
 *
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * Linux Foundation chooses to take subject only to the GPLv2 license
 * terms, and distributes only under these terms.
 *
 * 2010 - 2013 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Version: 1.8
 * Authors: andrew@goodix.com, meta@goodix.com
 * Release Date: 2013/04/25
 * Revision record:
 *      V1.0:
 *          first Release. By Andrew, 2012/08/31
 *      V1.2:
 *          modify gtp_reset_guitar,slot report,tracking_id & 0x0F.
 *                  By Andrew, 2012/10/15
 *      V1.4:
 *          modify gt9xx_update.c. By Andrew, 2012/12/12
 *      V1.6:
 *          1. new heartbeat/esd_protect mechanism(add external watchdog)
 *          2. doze mode, sliding wakeup
 *          3. 3 more cfg_group(GT9 Sensor_ID: 0~5)
 *          3. config length verification
 *          4. names & comments
 *                  By Meta, 2013/03/11
 *      V1.8:
 *          1. pen/stylus identification
 *          2. read double check & fixed config support
 *          2. new esd & slide wakeup optimization
 *                  By Meta, 2013/06/08
 */

#include <linux/regulator/consumer.h>
#include "gt9xx.h"

#include <linux/of_gpio.h>
#ifdef CONFIG_APP_INFO
#include <misc/app_info.h>
#endif
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif
#if GTP_ICS_SLOT_REPORT
#include <linux/input/mt.h>
#endif
#include <linux/pm_runtime.h>
static int runtime_count = 0;
/*move hw_tp_common.h to gt9xx.h and merge huawei_tp_adapter.h to hw_tp_common.h*/
#define GOODIX_DEV_NAME	"Goodix Capacitive TouchScreen"

#if USE_IRQ_THREAD
#else
static struct workqueue_struct *goodix_wq;
#endif

struct i2c_client * i2c_connect_client = NULL; 
#define CFG_MAX_TOUCH_POINTS	5
#define GOODIX_COORDS_ARR_SIZE	4
#define MAX_BUTTONS		4
/*delete goodix_debug_mask in order to use common debug mask*/


/* HIGH: 0x28/0x29, LOW: 0xBA/0xBB */
#define GTP_I2C_ADDRESS_HIGH	0x14
#define GTP_I2C_ADDRESS_LOW	0x5D
#define CFG_GROUP_LEN(p_cfg_grp)  (sizeof(p_cfg_grp) / sizeof(p_cfg_grp[0]))

#define GOODIX_VTG_MIN_UV	2850000
#define GOODIX_VTG_MAX_UV	2850000
#define GOODIX_I2C_VTG_MIN_UV	1800000
#define GOODIX_I2C_VTG_MAX_UV	1800000
#define GOODIX_VDD_LOAD_MIN_UA	0
#define GOODIX_VDD_LOAD_MAX_UA	10000
#define GOODIX_VIO_LOAD_MIN_UA	0
#define GOODIX_VIO_LOAD_MAX_UA	10000

#define RESET_DELAY_T3_US	200	/* T3: > 100us */
#define RESET_DELAY_T4		20	/* T4: > 5ms */

#define	PHY_BUF_SIZE		32

#define GTP_MAX_TOUCH		5
#define GTP_ESD_CHECK_CIRCLE_MS	2000
u8 config[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH]
                = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};

#if GTP_HAVE_TOUCH_KEY
static const u16 touch_key_array[] = {KEY_BACK, KEY_HOMEPAGE, KEY_MENU};
#define GTP_MAX_KEY_NUM  (sizeof(touch_key_array)/sizeof(touch_key_array[0]))

#if GTP_DEBUG_ON
    static const int  key_codes[] = {KEY_BACK, KEY_HOMEPAGE, KEY_MENU};
    static const char *key_names[] = {"Key_Back", "Key_Home", "Key_Menu"};
    #define GTP_MAX_KEY_CODE  (sizeof(key_codes)/sizeof(key_codes[0]))
#endif
    
#endif

static s8 gtp_i2c_test(struct i2c_client *client);
void gtp_reset_guitar(struct i2c_client *client, s32 ms);
s32 gtp_send_cfg(struct i2c_client *client);
void gtp_int_sync(s32 ms);
static int goodix_ts_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int goodix_ts_remove(struct i2c_client *client);

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void goodix_ts_early_suspend(struct early_suspend *h);
static void goodix_ts_late_resume(struct early_suspend *h);
#endif

#if GTP_CREATE_WR_NODE
extern s32 init_wr_node(struct i2c_client*);
extern void uninit_wr_node(void);
#endif

#if GTP_AUTO_UPDATE
extern u8 gup_init_update_proc(struct goodix_ts_data *);
#endif

extern s32 gtp_test_sysfs_init(void); 
extern void gtp_test_sysfs_deinit(void); 


#if GTP_ESD_PROTECT
static struct delayed_work gtp_esd_check_work;
static struct workqueue_struct *gtp_esd_check_workqueue = NULL;
static void gtp_esd_check_func(struct work_struct *);
static s32 gtp_init_ext_watchdog(struct i2c_client *client);
void gtp_esd_switch(struct i2c_client *, s32);
#endif

//*********** For GT9XXF Start **********//
#if GTP_COMPATIBLE_MODE
extern s32 i2c_read_bytes(struct i2c_client *client, u16 addr, u8 *buf, s32 len);
extern s32 i2c_write_bytes(struct i2c_client *client, u16 addr, u8 *buf, s32 len);
extern s32 gup_clk_calibration(void);
extern s32 gup_fw_download_proc(void *dir, u8 dwn_mode);
extern u8 gup_check_fs_mounted(char *path_name);
void gtp_recovery_reset(struct i2c_client *client);
static s32 gtp_esd_recovery(struct i2c_client *client);
s32 gtp_fw_startup(struct i2c_client *client);
static s32 gtp_main_clk_proc(struct goodix_ts_data *ts);
static s32 gtp_bak_ref_proc(struct goodix_ts_data *ts, u8 mode);
#endif

#if GTP_SLIDE_WAKEUP
typedef enum
{
    DOZE_DISABLED = 0,
    DOZE_ENABLED = 1,
    DOZE_WAKEUP = 2,
}DOZE_T;
static DOZE_T doze_status = DOZE_DISABLED;
static s8 gtp_enter_doze(struct goodix_ts_data *ts);
#endif

static u8 chip_gt9xxs = 0;  // true if ic is gt9xxs, like gt915s
u8 grp_cfg_version = 0;

/*******************************************************
Function:
	Read data from the i2c slave device.
Input:
	client:     i2c device.
	buf[0~1]:   read start address.
	buf[2~len-1]:   read data buffer.
	len:    GTP_ADDR_LENGTH + read bytes count
Output:
	numbers of i2c_msgs to transfer:
		2: succeed, otherwise: failed
*********************************************************/
s32 gtp_i2c_read(struct i2c_client *client, u8 *buf, s32 len)
{
    struct i2c_msg msgs[2];
    s32 ret=-1;
    s32 retries = 0;

	GTP_DEBUG_FUNC();

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr = client->addr;
	msgs[0].len = GTP_ADDR_LENGTH;
	msgs[0].buf = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr = client->addr;
	msgs[1].len = len - GTP_ADDR_LENGTH;
	msgs[1].buf = &buf[GTP_ADDR_LENGTH];

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, msgs, 2);
        if(ret == 2)break;
        retries++;
    }
    if((retries >= 5))
    {
    #if GTP_COMPATIBLE_MODE
        struct goodix_ts_data *ts = i2c_get_clientdata(client);
    #endif
        
    #if GTP_SLIDE_WAKEUP
        // reset chip would quit doze mode
        if (DOZE_ENABLED == doze_status)
        {
            return ret;
        }
    #endif
        tp_log_err("I2C Read: 0x%04X, %d bytes failed, errcode: %d! Process reset.", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
    #if GTP_COMPATIBLE_MODE
        if (CHIP_TYPE_GT9F == ts->chip_type)
        {
            gtp_recovery_reset(client);
        }
        else
    #endif
    	{
			gtp_reset_guitar(client, RESET_TIME_10_MS);
    	}
		
	}
	return ret;
}

/*******************************************************
Function:
	Write data to the i2c slave device.
Input:
	client:     i2c device.
	buf[0~1]:   write start address.
	buf[2~len-1]:   data buffer
	len:    GTP_ADDR_LENGTH + write bytes count
Output:
	numbers of i2c_msgs to transfer:
	1: succeed, otherwise: failed
*********************************************************/
s32 gtp_i2c_write(struct i2c_client *client,u8 *buf,s32 len)
{
    struct i2c_msg msg;
    s32 ret = -1;
    s32 retries = 0;

	GTP_DEBUG_FUNC();

	msg.flags = !I2C_M_RD;
	msg.addr = client->addr;
	msg.len = len;
	msg.buf = buf;

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, &msg, 1);
        if (ret == 1)break;
        retries++;
    }
    if((retries >= 5))
    {
    #if GTP_COMPATIBLE_MODE
        struct goodix_ts_data *ts = i2c_get_clientdata(client);
    #endif
    
    #if GTP_SLIDE_WAKEUP
        if (DOZE_ENABLED == doze_status)
        {
            return ret;
        }
    #endif
        tp_log_err("I2C Write: 0x%04X, %d bytes failed, errcode: %d! Process reset.", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
    #if GTP_COMPATIBLE_MODE
        if (CHIP_TYPE_GT9F == ts->chip_type)
        {
            gtp_recovery_reset(client);
        }
        else
    #endif
    	{
			gtp_reset_guitar(client, RESET_TIME_10_MS);
    	}
	
	}
	return ret;
}
/*******************************************************
Function:
	i2c read twice, compare the results
Input:
	client:  i2c device
	addr:    operate address
	rxbuf:   read data to store, if compare successful
	len:     bytes to read
Output:
	FAIL:    read failed
	SUCCESS: read successful
*********************************************************/
s32 gtp_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
    u8 buf[16] = {0};
    u8 confirm_buf[16] = {0};
    u8 retry = 0;
    s32 ret = -1;
	
    while (retry++ < 3)
    {
        memset(buf, 0xAA, 16);
        buf[0] = (u8)(addr >> 8);
        buf[1] = (u8)(addr & 0xFF);
        ret = gtp_i2c_read(client, buf, len + 2);
        if(ret < 0)
        {
            tp_log_err("%s:failed to read register firstly!line=%d\n",__func__,__LINE__);
            goto exit_dbl_check;
        }
        
        memset(confirm_buf, 0xAB, 16);
        confirm_buf[0] = (u8)(addr >> 8);
        confirm_buf[1] = (u8)(addr & 0xFF);
        ret = gtp_i2c_read(client, confirm_buf, len + 2);
        if(ret < 0)
        {
            tp_log_err("%s:failed to read register secondly!line=%d\n",__func__,__LINE__);
            goto exit_dbl_check;
        }        
        if (!memcmp(buf, confirm_buf, len+2))
        {
            memcpy(rxbuf, confirm_buf+2, len);
            return SUCCESS;
        }
    }  
exit_dbl_check:
    tp_log_err("I2C read 0x%04X, %d bytes, double check failed!", addr, len);
    return FAIL;
}

/*******************************************************
Function:
	Send config data.
Input:
	client: i2c device.
Output:
	result of i2c write operation.
	> 0: succeed, otherwise: failed
*********************************************************/
s32 gtp_send_cfg(struct i2c_client *client)
{
    s32 ret = 2;

#if GTP_DRIVER_SEND_CFG
    s32 retry = 0;
    struct goodix_ts_data *ts = i2c_get_clientdata(client);

    if (ts->fixed_cfg)
    {
        tp_log_info("%s, Ic fixed config, no config sent!\n", __func__);
        return 0;
    }
    else if (ts->pnl_init_error)
    {
        tp_log_info("%s, Error occured in init_panel, no config sent\n", __func__);
        return 0;
    }
    
    tp_log_info("%s, driver begin to send config.\n", __func__);
    for (retry = 0; retry < 5; retry++)
    {
        ret = gtp_i2c_write(client, config , GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
        if (ret > 0)
        {
            break;
        }
    }
#endif

	return ret;
}
/*******************************************************
Function:
	Disable irq function
Input:
	ts: goodix i2c_client private data
Output:
	None.
*********************************************************/
void gtp_irq_disable(struct goodix_ts_data *ts)
{
	unsigned long irqflags;

	GTP_DEBUG_FUNC();

    spin_lock_irqsave(&ts->irq_lock, irqflags);
    if (!ts->irq_is_disable)
    {
        ts->irq_is_disable = 1; 
        disable_irq_nosync(ts->client->irq);
    }
    spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
	Enable irq function
Input:
	ts: goodix i2c_client private data
Output:
	None.
*********************************************************/
void gtp_irq_enable(struct goodix_ts_data *ts)
{
	unsigned long irqflags = 0;

    GTP_DEBUG_FUNC();
    
    spin_lock_irqsave(&ts->irq_lock, irqflags);
    if (ts->irq_is_disable) 
    {
		/*the interrupt wake up the cpu when the cpu is sleep*/
        enable_irq(ts->client->irq);
        ts->irq_is_disable = 0; 
    }
    spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
	Report touch point event
Input:
	ts: goodix i2c_client private data
	id: trackId
	x:  input x coordinate
	y:  input y coordinate
	w:  input pressure
Output:
	None.
*********************************************************/
static void gtp_touch_down(struct goodix_ts_data* ts,s32 id,s32 x,s32 y,s32 w)
{
#if GTP_CHANGE_X2Y
	GTP_SWAP(x, y);
#endif

#if GTP_ICS_SLOT_REPORT
	input_mt_slot(ts->input_dev, id);
	input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
#else
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
	input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
	input_mt_sync(ts->input_dev);
#endif

	//tp_log_debug("ID:%d, X:%d, Y:%d, W:%d", id, x, y, w);
}

/*******************************************************
Function:
	Report touch release event
Input:
	ts: goodix i2c_client private data
Output:
	None.
*********************************************************/
static void gtp_touch_up(struct goodix_ts_data* ts, s32 id)
{
#if GTP_ICS_SLOT_REPORT
	input_mt_slot(ts->input_dev, id);
	input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
	//tp_log_debug("Touch id[%2d] release!", id);
#else
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
	input_mt_sync(ts->input_dev);
#endif
}



/*******************************************************
Function:
	Goodix touchscreen work function
Input:
	work: work struct of goodix_workqueue
Output:
	None.
*********************************************************/
#if USE_IRQ_THREAD 
static void goodix_ts_work_func(struct goodix_ts_data *ts)
#else
static void goodix_ts_work_func(struct work_struct *work)
#endif
{
    u8  end_cmd[3] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF, 0};
    u8  point_data[2 + 1 + 8 * GTP_MAX_TOUCH + 1]={GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF};
    u8  touch_num = 0;
    u8  finger = 0;
    static u16 pre_touch = 0;
    static u8 pre_key = 0;
#if GTP_WITH_PEN
    static u8 pre_pen = 0;
#endif
    u8  key_value = 0;
    u8* coor_data = NULL;
    s32 input_x = 0;
    s32 input_y = 0;
    s32 input_w = 0;
    s32 id = 0;
    s32 i  = 0;
    s32 ret = -1;
    /*add the gesture handler in tp work function*/
    unsigned int reprot_gesture_key_value = 0;

#if GTP_COMPATIBLE_MODE
    u8 rqst_buf[3] = {0x80, 0x43};  // for GT9XXF
#endif

#if GTP_SLIDE_WAKEUP
    u8 doze_buf[3] = {0x81, 0x4B};
#endif

#if USE_IRQ_THREAD 
	
#else
	struct goodix_ts_data *ts = NULL;
	ts = container_of(work, struct goodix_ts_data, work);
#endif

	tp_log_debug("%s :tp work function begin line = %d\n",__func__,__LINE__);

    if (ts->enter_update)
    {
        return;
    }
    #if GTP_SLIDE_WAKEUP
        
        if (DOZE_ENABLED == doze_status)
        {
            ret = gtp_i2c_read(ts->client, doze_buf, 3);
            tp_log_debug("0x814B = 0x%02X \n", doze_buf[2]);
            if (ret > 0)
            {               
                if (0xAA == doze_buf[2])
                {
                    if(IS_APP_ENABLE_GESTURE(GESTURE_SLIDE_L2R)&ts->easy_wakeup_gesture)
                    {
                        reprot_gesture_key_value = KEY_F2;
                        tp_log_debug("%s:from left to right to light up the screen!easy_wakeup_gesture=0x%04x!\n",__func__,ts->easy_wakeup_gesture);
                	}   
                }
                else if (0xBB == doze_buf[2])
                {
                    if(IS_APP_ENABLE_GESTURE(GESTURE_SLIDE_R2L)&ts->easy_wakeup_gesture)
                    {
                        reprot_gesture_key_value = KEY_F3;
                        tp_log_debug("%s:from right to left to light up the screen!easy_wakeup_gesture=0x%04x!\n",__func__,ts->easy_wakeup_gesture);
                	}
                }
                else if (0xAB == doze_buf[2])
                {
                    if(IS_APP_ENABLE_GESTURE(GESTURE_SLIDE_T2B)&ts->easy_wakeup_gesture)
                    {
                        reprot_gesture_key_value = KEY_F4;
                        tp_log_debug("%s:from top to bottom to light up the screen!easy_wakeup_gesture=0x%04x!\n",__func__,ts->easy_wakeup_gesture);
                	}
                }
                else if (0xBA == doze_buf[2])
                {
                    if(IS_APP_ENABLE_GESTURE(GESTURE_SLIDE_B2T)&ts->easy_wakeup_gesture)
                    {
                        reprot_gesture_key_value = KEY_F5;
                        tp_log_debug("%s:from bottom to top to light up the screen!easy_wakeup_gesture=0x%04x!\n",__func__,ts->easy_wakeup_gesture);
                    }
                }
                else if (0xCC == doze_buf[2]) // double click wakeup
                {
                
                    if(!(IS_APP_ENABLE_GESTURE(GESTURE_DOUBLE_CLICK)&ts->easy_wakeup_gesture))
                    {
                        tp_log_debug("%s:DOUBLE_CLICK_WAKEUP not enabled!!\n",__func__);
                    }
                    else
                    {
                        reprot_gesture_key_value = KEY_F1;
                        tp_log_debug("%s:Double click to light up the screen!easy_wakeup_gesture=0x%04x!\n",__func__,ts->easy_wakeup_gesture);
                    }
                }
                else
                {
                    gtp_enter_doze(ts);
                }
                if(0 != reprot_gesture_key_value)
                {
                    doze_status = DOZE_WAKEUP;
                    tp_log_debug("%s: reprot_gesture_key_value = %d \n",__func__, reprot_gesture_key_value);
                    input_report_key(ts->input_dev, reprot_gesture_key_value, 1);
                    input_sync(ts->input_dev);
                    input_report_key(ts->input_dev, reprot_gesture_key_value, 0);
                    input_sync(ts->input_dev);
                    // clear 0x814B
                    doze_buf[2] = 0x00;
                    gtp_i2c_write(ts->client, doze_buf, 3);					
                }
                else
                {
                    // clear 0x814B
                    doze_buf[2] = 0x00;
                    gtp_i2c_write(ts->client, doze_buf, 3);
                    gtp_enter_doze(ts);
                }
            }
        	return;
        }
        
    #endif
	tp_log_debug("%s: begin to read point data!line=%d\n",__func__,__LINE__);
    ret = gtp_i2c_read(ts->client, point_data, 12);
    if (ret < 0)
    {
        tp_log_err("%s: I2C transfer error. errno:%d,line=%d\n " ,__func__, ret,__LINE__);
        goto exit_work_func;
    }

	finger = point_data[GTP_ADDR_LENGTH];
	
#if GTP_COMPATIBLE_MODE
    // GT9XXF
    if ((finger == 0x00) && (CHIP_TYPE_GT9F == ts->chip_type))     // request arrived
    {
        ret = gtp_i2c_read(ts->client, rqst_buf, 3);
        if (ret < 0)
        {
           tp_log_err("Read request status error!");
           goto exit_work_func;
        } 
        
        switch (rqst_buf[2] & 0x0F)
        {
        case GTP_RQST_CONFIG:
            tp_log_info("Request for config.");
            ret = gtp_send_cfg(ts->client);
            if (ret < 0)
            {
                tp_log_err("Request for config unresponded!");
            }
            else
            {
                rqst_buf[2] = GTP_RQST_RESPONDED;
                gtp_i2c_write(ts->client, rqst_buf, 3);
                tp_log_info("Request for config responded!");
            }
            break;
            
        case GTP_RQST_BAK_REF:
            tp_log_info("Request for backup reference.");
            ret = gtp_bak_ref_proc(ts, GTP_BAK_REF_SEND);
            if (SUCCESS == ret)
            {
                rqst_buf[2] = GTP_RQST_RESPONDED;
                gtp_i2c_write(ts->client, rqst_buf, 3);
                tp_log_info("Request for backup reference responded!");
            }
            else
            {
                tp_log_err("Requeset for backup reference unresponed!");
            }
            break;
            
        case GTP_RQST_RESET:
            tp_log_info("Request for reset.");
            gtp_recovery_reset(ts->client);
            break;
            
        case GTP_RQST_MAIN_CLOCK:
            tp_log_info("Request for main clock.");
            ts->rqst_processing = 1;
            ret = gtp_main_clk_proc(ts);
            if (FAIL == ret)
            {
                tp_log_err("Request for main clock unresponded!");
            }
            else
            {
                tp_log_info("Request for main clock responded!");
                rqst_buf[2] = GTP_RQST_RESPONDED;
                gtp_i2c_write(ts->client, rqst_buf, 3);
                ts->rqst_processing = 0;
                ts->clk_chk_fs_times = 0;
            }
            break;
            
        case GTP_RQST_IDLE:
        default:
            break;
        }
    }
#endif

    if((finger & 0x80) == 0)
    {
        goto exit_work_func;
    }

    touch_num = finger & 0x0f;
    if (touch_num > GTP_MAX_TOUCH)
    {
        goto exit_work_func;
    }

    if (touch_num > 1)
    {
        u8 buf[8 * GTP_MAX_TOUCH] = {(GTP_READ_COOR_ADDR + 10) >> 8, (GTP_READ_COOR_ADDR + 10) & 0xff};

        ret = gtp_i2c_read(ts->client, buf, 2 + 8 * (touch_num - 1)); 
        memcpy(&point_data[12], &buf[2], 8 * (touch_num - 1));
    }

#if GTP_HAVE_TOUCH_KEY
    key_value = point_data[3 + 8 * touch_num];
    
    if(key_value || pre_key)
    {
        for (i = 0; i < GTP_MAX_KEY_NUM; i++)
        {
        #if GTP_DEBUG_ON
            for (ret = 0; ret < GTP_MAX_KEY_CODE; ++ret)
            {
                if (key_codes[ret] == touch_key_array[i])
                {
                    tp_log_debug("Key:%d %s %s", __LINE__,key_names[ret], (key_value & (0x01 << i)) ? "Down" : "Up");
                    break;
                }
            }
        #endif
            tp_log_debug("%s,touch_key_array[%d] = %d,key_value=%d,\n",__func__, i , touch_key_array[i],key_value);
            input_report_key(ts->input_dev, touch_key_array[i], key_value & (0x01<<i));  
        }
        touch_num = 0;
        pre_touch = 0;
    }
#endif
	pre_key = key_value;

	tp_log_debug("%s,pre_touch:%02x, finger:%02x.pre_key = %d\n",__func__, pre_touch, finger,pre_key);

#if GTP_ICS_SLOT_REPORT
#if GTP_WITH_PEN
    if (pre_pen && (touch_num == 0))
    {
        tp_log_debug("Pen touch UP(Slot)!");
        input_report_key(ts->input_dev, BTN_TOOL_PEN, 0);
        input_mt_slot(ts->input_dev, 5);
        input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
        pre_pen = 0;
    }
#endif
    if (pre_touch || touch_num)
    {
        s32 pos = 0;
        u16 touch_index = 0;
        u8 report_num = 0;
        coor_data = &point_data[3];
        
        if(touch_num)
        {
            id = coor_data[pos] & 0x0F;
        
        #if GTP_WITH_PEN
            id = coor_data[pos];
            if ((id & 0x80))  
            {
                tp_log_debug("Pen touch DOWN(Slot)!");
                input_x  = coor_data[pos + 1] | (coor_data[pos + 2] << 8);
                input_y  = coor_data[pos + 3] | (coor_data[pos + 4] << 8);
                input_w  = coor_data[pos + 5] | (coor_data[pos + 6] << 8);
                
                input_report_key(ts->input_dev, BTN_TOOL_PEN, 1);
                input_mt_slot(ts->input_dev, 5);
                input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 5);
                input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
                input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
                input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
                tp_log_debug("Pen/Stylus: (%d, %d)[%d]", input_x, input_y, input_w);
                pre_pen = 1;
                pre_touch = 0;
            }    
        #endif
        
            touch_index |= (0x01<<id);
        }
        
        tp_log_debug("%s,id = %d,touch_index = 0x%x, pre_touch = 0x%x\n",__func__,id, touch_index,pre_touch);
        for (i = 0; i < GTP_MAX_TOUCH; i++)
        {
        #if GTP_WITH_PEN
            if (pre_pen == 1)
            {
                break;
            }
        #endif
        
            if ((touch_index & (0x01<<i)))
            {
            	tp_log_debug("%s,touch_index = 0x%x,i = %d,line = %d\n",__func__,touch_index,i,__LINE__);
                input_x  = coor_data[pos + 1] | (coor_data[pos + 2] << 8);
                input_y  = coor_data[pos + 3] | (coor_data[pos + 4] << 8);
                input_w  = coor_data[pos + 5] | (coor_data[pos + 6] << 8);

                gtp_touch_down(ts, id, input_x, input_y, input_w);
                pre_touch |= 0x01 << i;
                
                report_num++;
                if (report_num < touch_num)
                {
                    pos += 8;
				id = coor_data[pos] & 0x0F;
				touch_index |= (0x01<<id);
                }
            }
            else
            {
                gtp_touch_up(ts, i);
                pre_touch &= ~(0x01 << i);
            }
        }
    }
#else
    input_report_key(ts->input_dev, BTN_TOUCH, (touch_num || key_value));
    if (touch_num)
    {
        for (i = 0; i < touch_num; i++)
        {
            coor_data = &point_data[i * 8 + 3];

            id = coor_data[0] & 0x0F;
            input_x  = coor_data[1] | (coor_data[2] << 8);
            input_y  = coor_data[3] | (coor_data[4] << 8);
            input_w  = coor_data[5] | (coor_data[6] << 8);
        	tp_log_debug("%s id = %d,line = %d\n",__func__,id,__LINE__);
        #if GTP_WITH_PEN
            id = coor_data[0];
            if (id & 0x80)
            {
                tp_log_debug("Pen touch DOWN!");
                input_report_key(ts->input_dev, BTN_TOOL_PEN, 1);
                pre_pen = 1;
                id = 0;   
            }
        #endif
        
            gtp_touch_down(ts, id, input_x, input_y, input_w);
        }
    }
    else if (pre_touch)
    {
    
    #if GTP_WITH_PEN
        if (pre_pen == 1)
        {
            tp_log_debug("Pen touch UP!");
            input_report_key(ts->input_dev, BTN_TOOL_PEN, 0);
            pre_pen = 0;
        }
    #endif
    
        tp_log_debug("%s,Touch Release!line = %d\n",__func__,__LINE__);
        gtp_touch_up(ts, 0);
    }

	pre_touch = touch_num;
#endif
	tp_log_debug("%s pre_touch = %d,line = %d\n",__func__,pre_touch,__LINE__);
	input_sync(ts->input_dev);

exit_work_func:
    if(!ts->gtp_rawdiff_mode)
    {
        ret = gtp_i2c_write(ts->client, end_cmd, 3);
        if (ret < 0)
        {
            tp_log_info("%s: I2C write end_cmd error!LINE=%d\n",__func__,__LINE__);
        }
    }
	
#if USE_IRQ_THREAD
#else
    if (ts->use_irq)
    {
        gtp_irq_enable(ts);
    }
#endif
}

/*******************************************************
Function:
	Timer interrupt service routine for polling mode.
Input:
	timer: timer struct pointer
Output:
	Timer work mode.
	HRTIMER_NORESTART: no restart mode
*********************************************************/
static enum hrtimer_restart goodix_ts_timer_handler(struct hrtimer *timer)
{
    struct goodix_ts_data *ts = container_of(timer, struct goodix_ts_data, timer);

    GTP_DEBUG_FUNC();
	
#if USE_IRQ_THREAD
#else
    queue_work_on(WORK_CPU_UNBOUND,goodix_wq, &ts->work);
#endif
	
    hrtimer_start(&ts->timer, ktime_set(0, (GTP_POLL_TIME+6)*1000000), HRTIMER_MODE_REL);
    return HRTIMER_NORESTART;
}

/*******************************************************
Function:
	External interrupt service routine for interrupt mode.
Input:
	irq:  interrupt number.
	dev_id: private data pointer
Output:
	Handle Result.
	IRQ_HANDLED: interrupt handled successfully
*********************************************************/
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
    struct goodix_ts_data *ts = dev_id;

    GTP_DEBUG_FUNC();
    /* Add dynamic log interface for goodix tp*/
    //tp_log_debug("%s interrupt handler begin ... line = %d\n",__func__,__LINE__);
#if USE_IRQ_THREAD
    goodix_ts_work_func(ts);
#else
    gtp_irq_disable(ts);
    queue_work_on(WORK_CPU_UNBOUND,goodix_wq, &ts->work);
#endif
    
    return IRQ_HANDLED;
}
/*******************************************************
Function:
	Synchronization.
Input:
	ms: synchronization time in millisecond.
Output:
	None.
*******************************************************/
void gtp_int_sync(s32 ms)
{
    int ret = 0;
    struct goodix_ts_data *ts;
    ts = i2c_get_clientdata(i2c_connect_client);
	
    ret = gpio_direction_output(ts->pdata->irq_gpio, 0);
    if(ret)
    {
        tp_log_err("%s:Failed to set gpio INT for output low,line=%d\n", __func__, __LINE__);
    }
    msleep(ms);
    ret = gpio_direction_input(ts->pdata->irq_gpio);
    if(ret)
    {
        tp_log_err("%s:Failed to set gpio INT for input,line=%d\n", __func__, __LINE__);
    }
}

/*******************************************************
Function:
	Reset chip.
Input:
	ms: reset time in millisecond, must >10ms
Output:
	None.
*******************************************************/
void gtp_reset_guitar(struct i2c_client *client, s32 ms)
{
    int ret = 0;
    struct goodix_ts_data *ts = i2c_get_clientdata(client);
    GTP_DEBUG_FUNC();

    /* This reset sequence will selcet I2C slave address */
    tp_log_debug("%s enter,reset time=%d ms,line=%d",__func__,ms,__LINE__);
    ret = gpio_direction_output(ts->pdata->reset_gpio, 0);
    if(ret)
    {
        tp_log_err("%s:Failed to set gpio reset for output low,line=%d\n", __func__, __LINE__);
    }

    msleep(ms);

    if (ts->client->addr == GTP_I2C_ADDRESS_HIGH)
        ret = gpio_direction_output(ts->pdata->irq_gpio, 1);
    else
        ret = gpio_direction_output(ts->pdata->irq_gpio, 0);
    if(ret)
    {
        tp_log_err("%s:Failed to set gpio INT for output line=%d\n", __func__, __LINE__);
    }

    usleep(RESET_DELAY_T3_US);
    ret = gpio_direction_output(ts->pdata->reset_gpio, 1);
    if(ret)
    {
        tp_log_err("%s:Failed to set gpio reset for output high,line=%d\n", __func__, __LINE__);
    }

    msleep(RESET_DELAY_T4);

    ret = gpio_direction_input(ts->pdata->reset_gpio);
    if(ret)
    {
        tp_log_err("%s:Failed to set gpio reset for input line=%d\n", __func__, __LINE__);
    }
	
#if GTP_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        return;
    }
#endif
	gtp_int_sync(50);

#if GTP_ESD_PROTECT
	gtp_init_ext_watchdog(client);
#endif
}

#if defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_FB)
#if GTP_SLIDE_WAKEUP
/*******************************************************
Function:
	Enter doze mode for sliding wakeup.
Input:
	ts: goodix tp private data
Output:
	1: succeed, otherwise failed
*******************************************************/
static s8 gtp_enter_doze(struct goodix_ts_data *ts)
{
    s8 ret = -1;
    s8 retry = 0;
    u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 8};

    GTP_DEBUG_FUNC();

#if GTP_DBL_CLK_WAKEUP
	i2c_control_buf[2] = 0x09;
#endif

    gtp_irq_disable(ts);
    
    tp_log_info("%s, begin line=%d \n", __func__, __LINE__);
    while(retry++ < 5)
    {
        i2c_control_buf[0] = 0x80;
        i2c_control_buf[1] = 0x46;
        ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
        if (ret < 0)
        {
            tp_log_err("%s, failed to set doze flag into 0x8046, %d\n", __func__, retry);
            continue;
        }
        i2c_control_buf[0] = 0x80;
        i2c_control_buf[1] = 0x40;
        ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
        if (ret > 0)
        {
            doze_status = DOZE_ENABLED;
            tp_log_info("%s, GTP has been working in doze mode!\n", __func__);
            gtp_irq_enable(ts);
            return ret;
        }
        msleep(10);
    }
    tp_log_err("GTP send doze cmd failed.");
    gtp_irq_enable(ts);
    return ret;
}
#endif
/*******************************************************
Function:
	Enter sleep mode.
Input:
	ts: private data.
Output:
	Executive outcomes.
	1: succeed, otherwise failed.
*******************************************************/
static s8 gtp_enter_sleep(struct goodix_ts_data  *ts)
{
    s8 ret = -1;
    s8 retry = 0;
    u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 5};

#if GTP_COMPATIBLE_MODE
    u8 status_buf[3] = {0x80, 0x44};
#endif
    
    GTP_DEBUG_FUNC();
    tp_log_info("%s, begin line=%d \n", __func__, __LINE__);
#if GTP_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        // GT9XXF: host interact with ic
        ret = gtp_i2c_read(ts->client, status_buf, 3);
        if (ret < 0)
        {
            tp_log_err("failed to get backup-reference status");
        }
        
        if (status_buf[2] & 0x80)
        {
            ret = gtp_bak_ref_proc(ts, GTP_BAK_REF_STORE);
            if (FAIL == ret)
            {
                tp_log_err("failed to store bak_ref");
            }
        }
    }
#endif
    /*pull down the INT before IC enter sleep*/
    ret = gpio_direction_output(ts->pdata->irq_gpio, 0);
    msleep(5);
    
    while(retry++ < 5)
    {
        ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
        if (ret > 0)
        {
            tp_log_info("%s, GTP has been sleep!\n", __func__);            
            return ret;
        }
        msleep(10);
    }
    tp_log_err("%s, GTP send sleep cmd failed.\n", __func__);
    return ret;
}

/*******************************************************
Function:
	Wakeup from sleep.
Input:
	ts: private data.
Output:
	Executive outcomes.
	>0: succeed, otherwise: failed.
*******************************************************/
static s8 gtp_wakeup_sleep(struct goodix_ts_data *ts)
{
	u8 retry = 0;
	s8 ret = -1;
	

	GTP_DEBUG_FUNC();
	
#if GTP_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        u8 opr_buf[3] = {0x41, 0x80};
        
        gpio_direction_output(ts->pdata->irq_gpio, 1);
        msleep(5);
    
        for (retry = 0; retry < 20; ++retry)
        {
            // hold ss51 & dsp
            opr_buf[2] = 0x0C;
            ret = gtp_i2c_write(ts->client, opr_buf, 3);
            if (FAIL == ret)
            {
                tp_log_err("failed to hold ss51 & dsp!");
                continue;
            }
            opr_buf[2] = 0x00;
            ret = gtp_i2c_read(ts->client, opr_buf, 3);
            if (FAIL == ret)
            {
                tp_log_err("failed to get ss51 & dsp status!");
                continue;
            }
            if (0x0C != opr_buf[2])
            {
                tp_log_debug("ss51 & dsp not been hold, %d", retry+1);
                continue;
            }
            tp_log_debug("ss51 & dsp confirmed hold");
            
            ret = gtp_fw_startup(ts->client);
            if (FAIL == ret)
            {
                tp_log_err("failed to startup GT9XXF, process recovery");
                gtp_esd_recovery(ts->client);
            }
            break;
        }
        if (retry >= 10)
        {
            tp_log_err("failed to wakeup, processing esd recovery");
            gtp_esd_recovery(ts->client);
        }
        else
        {
            tp_log_info("GT9XXF gtp wakeup success");
        }
        return ret;
    }
#endif

#if GTP_POWER_CTRL_SLEEP

    while(retry++ < 5)
    {
        gtp_reset_guitar(ts->client, RESET_TIME_20_MS);
        
        tp_log_info("GTP wakeup sleep.");
        return 1;
    }
#else
    while(retry++ < 10)
    {
        /*wake up by gtp reset if we enable the gesture wakeup*/
    #if GTP_SLIDE_WAKEUP
        if (DOZE_WAKEUP != doze_status)       // wakeup not by slide 
        {
            tp_log_debug("wakeup by power, reset guitar");
        }
        else              // wakeup by slide 
        {
            tp_log_debug("wakeup by slide/double-click, reset guitar");
        }

        doze_status = DOZE_DISABLED;   
        gtp_irq_disable(ts);
        gtp_reset_guitar(ts->client, RESET_TIME_10_MS);
        gtp_irq_enable(ts);
        
    #else
        if (chip_gt9xxs == 1)
        {
           gtp_reset_guitar(ts->client, RESET_TIME_10_MS);
        }
        else
        {
          tp_log_debug("%s, wakeup chip_gt9xxs=%d,line=%d", __func__, chip_gt9xxs, __LINE__);
          gpio_direction_output(ts->pdata->irq_gpio, 1);
          msleep(5);
        }
    #endif
        /*if the i2c is good, tp IC has been waked up*/	
        ret = gtp_i2c_test(ts->client);
        if (ret > 0)
        {
            tp_log_info("%s, GTP has been waked up.LINE = %d\n", __func__, __LINE__);
            
        #if (!GTP_SLIDE_WAKEUP)
            if (chip_gt9xxs == 0)
            {
                gtp_int_sync(25);
            #if GTP_ESD_PROTECT
                gtp_init_ext_watchdog(ts->client);
            #endif
            }
        #endif
            return ret;
        }
        gtp_reset_guitar(ts->client, RESET_TIME_20_MS);
    }
#endif
    tp_log_err("%s, GTP wakeup sleep failed.", __func__);
    return ret;
}
#if GTP_DRIVER_SEND_CFG
static s32 gtp_get_info(struct goodix_ts_data *ts)
{
    u8 opr_buf[6] = {0};
    s32 ret = 0;
    
    opr_buf[0] = (u8)((GTP_REG_CONFIG_DATA+1) >> 8);
    opr_buf[1] = (u8)((GTP_REG_CONFIG_DATA+1) & 0xFF);
    
    ret = gtp_i2c_read(ts->client, opr_buf, 6);
    if (ret < 0)
    {
        return FAIL;
    }
    
    ts->abs_x_max = (opr_buf[3] << 8) + opr_buf[2];
    ts->abs_y_max = (opr_buf[5] << 8) + opr_buf[4];
    
    opr_buf[0] = (u8)((GTP_REG_CONFIG_DATA+6) >> 8);
    opr_buf[1] = (u8)((GTP_REG_CONFIG_DATA+6) & 0xFF);
    
    ret = gtp_i2c_read(ts->client, opr_buf, 3);
    if (ret < 0)
    {
        return FAIL;
    }
    ts->int_trigger_type = opr_buf[2] & 0x03;
    
    tp_log_info("X_MAX = %d, Y_MAX = %d, TRIGGER = 0x%02x",
            ts->abs_x_max,ts->abs_y_max,ts->int_trigger_type);
    
    return SUCCESS;    
}
#endif 

/*******************************************************
Function:
	Initialize gtp.
Input:
	ts: goodix private data
Output:
	Executive outcomes.
	> =0: succeed, otherwise: failed
*******************************************************/
static s32 gtp_init_panel(struct goodix_ts_data *ts)
{
    s32 ret = -1;

#if GTP_DRIVER_SEND_CFG
    s32 i = 0;
    u8 check_sum = 0;
    u8 opr_buf[16] = {0};
    u8 sensor_id = 0; 
    
    u8 cfg_info_group1[] = CTP_CFG_GROUP1;
    u8 cfg_info_group2[] = CTP_CFG_GROUP2;
    u8 cfg_info_group3[] = CTP_CFG_GROUP3;
    u8 cfg_info_group4[] = CTP_CFG_GROUP4;
    u8 cfg_info_group5[] = CTP_CFG_GROUP5;
    u8 cfg_info_group6[] = CTP_CFG_GROUP6;
    u8 *send_cfg_buf[] = {cfg_info_group1, cfg_info_group2, cfg_info_group3,
                        cfg_info_group4, cfg_info_group5, cfg_info_group6};
    u8 cfg_info_len[] = { CFG_GROUP_LEN(cfg_info_group1),
                          CFG_GROUP_LEN(cfg_info_group2),
                          CFG_GROUP_LEN(cfg_info_group3),
                          CFG_GROUP_LEN(cfg_info_group4),
                          CFG_GROUP_LEN(cfg_info_group5),
                          CFG_GROUP_LEN(cfg_info_group6)};

    GTP_DEBUG_FUNC();
    tp_log_debug("Config Groups\' Lengths: %d, %d, %d, %d, %d, %d", 
        cfg_info_len[0], cfg_info_len[1], cfg_info_len[2], cfg_info_len[3],
        cfg_info_len[4], cfg_info_len[5]);

    
#if GTP_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        ts->fw_error = 0;
    }
    else
#endif
    {
    	/*not need to send config if ic Firmware is error */
        ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_FW_UPDATE, opr_buf, 1);
        if (SUCCESS == ret) 
        {
            if (opr_buf[0] != 0xBE)
            {
                ts->fw_error = 1;
                tp_log_err("Firmware error, no config sent!");
                return -1;
            }
        }
    }

	/*get the config id */
    if ((!cfg_info_len[1]) && (!cfg_info_len[2]) && 
        (!cfg_info_len[3]) && (!cfg_info_len[4]) && 
        (!cfg_info_len[5]))
    {
        sensor_id = 0; 
    }
    else
    {
    #if GTP_COMPATIBLE_MODE
        msleep(50);
    #endif
        ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_SENSOR_ID, &sensor_id, 1);
        if (SUCCESS == ret)
        {
            if (sensor_id >= 0x06)
            {
                tp_log_err("Invalid sensor_id(0x%02X), No Config Sent!", sensor_id);
                ts->pnl_init_error = 1;
            #if GTP_COMPATIBLE_MODE
                if (CHIP_TYPE_GT9F == ts->chip_type)
                {
                    return -1;
                }
                else
            #endif
                {
                    gtp_get_info(ts);
                }
                return 0;
            }
        }
        else
        {
            tp_log_err("Failed to get sensor_id, No config sent!");
            ts->pnl_init_error = 1;
            return -1;
        }
        tp_log_info("Sensor_ID: %d", sensor_id);
    }
    ts->gtp_cfg_len = cfg_info_len[sensor_id];
    tp_log_info("CTP_CONFIG_GROUP%d used, config length: %d", sensor_id + 1, ts->gtp_cfg_len);
    
    if (ts->gtp_cfg_len < GTP_CONFIG_MIN_LENGTH)
    {
        tp_log_err("Config Group%d is INVALID CONFIG GROUP(Len: %d)! NO Config Sent! You need to check you header file CFG_GROUP section!", sensor_id+1, ts->gtp_cfg_len);
        ts->pnl_init_error = 1;
        return -1;
    }
#if GTP_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        ts->fixed_cfg = 0;
    }
    else
#endif
    {
        ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_CONFIG_DATA, &opr_buf[0], 1);
        
        if (ret == SUCCESS)
        {   
            tp_log_debug("CFG_GROUP%d Config Version: %d, 0x%02X; IC Config Version: %d, 0x%02X", sensor_id+1, 
                        send_cfg_buf[sensor_id][0], send_cfg_buf[sensor_id][0], opr_buf[0], opr_buf[0]);
            
            if (opr_buf[0] < 90)    
            {
                grp_cfg_version = send_cfg_buf[sensor_id][0];       // backup group config version
                //send_cfg_buf[sensor_id][0] = 0x00;
                ts->fixed_cfg = 0;
            }
            else        // treated as fixed config, not send config
            {
                tp_log_info("Ic fixed config with config version(%d, 0x%02X)", opr_buf[0], opr_buf[0]);
                ts->fixed_cfg = 1;
                gtp_get_info(ts);
                return 0;
            }
        }
        else
        {
            tp_log_err("Failed to get ic config version!No config sent!");
            return -1;
        }
    }
    
    memset(&config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
    memcpy(&config[GTP_ADDR_LENGTH], send_cfg_buf[sensor_id], ts->gtp_cfg_len);

#if GTP_CUSTOM_CFG
    config[RESOLUTION_LOC]     = (u8)GTP_MAX_WIDTH;
    config[RESOLUTION_LOC + 1] = (u8)(GTP_MAX_WIDTH>>8);
    config[RESOLUTION_LOC + 2] = (u8)GTP_MAX_HEIGHT;
    config[RESOLUTION_LOC + 3] = (u8)(GTP_MAX_HEIGHT>>8);
    
    if (GTP_INT_TRIGGER == 0)  //RISING
    {
        config[TRIGGER_LOC] &= 0xfe; 
    }
    else if (GTP_INT_TRIGGER == 1)  //FALLING
    {
        config[TRIGGER_LOC] |= 0x01;
    }
#endif  // GTP_CUSTOM_CFG
    
    check_sum = 0;
    for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++)
    {
        check_sum += config[i];
    }
    config[ts->gtp_cfg_len] = (~check_sum) + 1;

#else // driver not send config

    ts->gtp_cfg_len = GTP_CONFIG_MAX_LENGTH;
    ret = gtp_i2c_read(ts->client, config, ts->gtp_cfg_len + GTP_ADDR_LENGTH);
    if (ret < 0)
    {
        tp_log_err("Read Config Failed, Using Default Resolution & INT Trigger!");
        ts->abs_x_max = GTP_MAX_WIDTH;
        ts->abs_y_max = GTP_MAX_HEIGHT;
        ts->int_trigger_type = GTP_INT_TRIGGER;
    }
#endif // GTP_DRIVER_SEND_CFG
	tp_log_debug("abs_x_max=%d,abs_y_max=%d,line=%d \n",ts->abs_x_max,ts->abs_y_max,__LINE__);
    if ((ts->abs_x_max == 0) && (ts->abs_y_max == 0))
    {
        ts->abs_x_max = (config[RESOLUTION_LOC + 1] << 8) + config[RESOLUTION_LOC];
        ts->abs_y_max = (config[RESOLUTION_LOC + 3] << 8) + config[RESOLUTION_LOC + 2];
        ts->int_trigger_type = (config[TRIGGER_LOC]) & 0x03; 
    }

#if GTP_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == ts->chip_type)
    {
        u8 sensor_num = 0;
        u8 driver_num = 0;
        u8 have_key = 0;
        
        have_key = (config[GTP_REG_HAVE_KEY - GTP_REG_CONFIG_DATA + 2] & 0x01);
        
        if (1 == ts->is_950)
        {
            driver_num = config[GTP_REG_MATRIX_DRVNUM - GTP_REG_CONFIG_DATA + 2];
            sensor_num = config[GTP_REG_MATRIX_SENNUM - GTP_REG_CONFIG_DATA + 2];
            if (have_key)
            {
                driver_num--;
            }
            ts->bak_ref_len = (driver_num * (sensor_num - 1) + 2) * 2 * 6;
        }
        else
        {
            driver_num = (config[CFG_LOC_DRVA_NUM] & 0x1F) + (config[CFG_LOC_DRVB_NUM]&0x1F);
            if (have_key)
            {
                driver_num--;
            }
            sensor_num = (config[CFG_LOC_SENS_NUM] & 0x0F) + ((config[CFG_LOC_SENS_NUM] >> 4) & 0x0F);
            ts->bak_ref_len = (driver_num * (sensor_num - 2) + 2) * 2;
        }
    
        tp_log_info("Drv * Sen: %d * %d(key: %d), X_MAX: %d, Y_MAX: %d, TRIGGER: 0x%02x",
           driver_num, sensor_num, have_key, ts->abs_x_max,ts->abs_y_max,ts->int_trigger_type);
        return 0;
    }
    else
#endif
    {
    #if GTP_DRIVER_SEND_CFG
        ret = gtp_send_cfg(ts->client);
        if (ret < 0)
        {
            tp_log_err("Send config error.");
        }
        // set config version to CTP_CFG_GROUP, for resume to send config
        config[GTP_ADDR_LENGTH] = grp_cfg_version;
        check_sum = 0;
        for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++)
        {
            check_sum += config[i];
        }
        config[ts->gtp_cfg_len] = (~check_sum) + 1;
    #endif
        tp_log_info("X_MAX: %d, Y_MAX: %d, TRIGGER: 0x%02x", ts->abs_x_max,ts->abs_y_max,ts->int_trigger_type);
    }
    
    msleep(10);
	
    return 0;
}

/*******************************************************
Function:
	Read chip version.
Input:
	client:  i2c device
	version: buffer to keep ic firmware version
Output:
	read operation return.
	2: succeed, otherwise: failed
*******************************************************/
s32 gtp_read_version(struct i2c_client *client, u16 *version)
{
    s32 ret = -1;
    u8 buf[8] = {GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff};

    GTP_DEBUG_FUNC();
    tp_log_info("%s start line=%d\n",__func__,__LINE__);
    ret = gtp_i2c_read(client, buf, sizeof(buf));
    if (ret < 0)
    {
        tp_log_err("GTP read version failed");
        return ret;
    }

    if (version)
    {
        *version = (buf[7] << 8) | buf[6];
    }
    
    if (buf[5] == 0x00)
    {
        tp_log_info("IC Version: %c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[7], buf[6]);
    }
    else
    {
        if (buf[5] == 'S' || buf[5] == 's')
        {
            chip_gt9xxs = 1;
        }
        tp_log_info("IC Version: %c%c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);
    }
    return ret;
}

/*******************************************************
Function:
	I2c test Function.
Input:
	client:i2c client.
Output:
	Executive outcomes.
	2: succeed, otherwise failed.
*******************************************************/
static s8 gtp_i2c_test(struct i2c_client *client)
{
    u8 test[3] = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
    u8 retry = 0;
    s8 ret = -1;
  
    GTP_DEBUG_FUNC();
    tp_log_info("%s start line=%d\n",__func__,__LINE__);
    while(retry++ < 5)
    {
        ret = gtp_i2c_read(client, test, 3);
        if (ret > 0)
        {
            return ret;
        }
        tp_log_err("GTP i2c test failed time %d.",retry);
        msleep(10);
    }
    return ret;
}

/*******************************************************
Function:
	Request gpio(INT & RST) ports.
Input:
	ts: private data.
Output:
	Executive outcomes.
	= 0: succeed, != 0: failed
*******************************************************/
static int gtp_request_io_port(struct goodix_ts_data *ts)
{
	struct goodix_ts_platform_data *pdata = ts->pdata;
	int ret;
	tp_log_info("%s start line=%d\n",__func__,__LINE__);
	if(NULL == pdata)
	{
		tp_log_err("%s,pdata is NULL,exit request.\n", __func__);
		goto pwr_off;
	}
	else
	{
		tp_log_info("%s,pdata->irq_gpio = %d .\n", __func__,pdata->irq_gpio);
	}
	
	/*request irq and set irq gpio direction if the gpio is valid*/
	if (gpio_is_valid(pdata->irq_gpio)) {

		ret = gpio_request(pdata->irq_gpio, "goodix_ts_irq_gpio");
		if (ret) {
			tp_log_err("irq gpio request failed\n");
			goto pwr_off;
		}

		ret = gpio_direction_input(pdata->irq_gpio);
		if (ret) {
			tp_log_err("set direction for irq gpio failed\n");
			goto free_irq_gpio;
		}
	} else {
		tp_log_err("irq gpio is invalid!\n");
		ret = -EINVAL;
		goto free_irq_gpio;
	}

	/*request reset and set reset gpio direction if the gpio is valid*/
	if (gpio_is_valid(pdata->reset_gpio)) {
		ret = gpio_request(pdata->reset_gpio, "goodix_ts__reset_gpio");
		if (ret) {
			tp_log_err("reset gpio request failed\n");
			goto free_irq_gpio;
		}

		ret = gpio_direction_output(pdata->reset_gpio, 0);
		if (ret) {
			tp_log_err("set direction for reset gpio failed\n");
			goto free_reset_gpio;
		}
	} else {
		tp_log_err("reset gpio is invalid!\n");
		ret = -EINVAL;
		goto free_reset_gpio;
	}
	ret = gpio_direction_input(pdata->reset_gpio);
	if(ret)
	{
		tp_log_err("%s:Failed to set reset input line=%d\n",__func__,__LINE__);
		goto free_reset_gpio;
	}     

	return ret;

free_reset_gpio:
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
free_irq_gpio:
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);
pwr_off:
	return ret;
}

/*******************************************************
Function:
	Request interrupt.
Input:
	ts: private data.
Output:
	Executive outcomes.
	0: succeed, -1: failed.
*******************************************************/
static int gtp_request_irq(struct goodix_ts_data *ts)
{
	int ret;
	const u8 irq_table[] = GTP_IRQ_TAB;
	unsigned long irq_flags;

	tp_log_info("INT trigger type:%x, irq=%d", ts->int_trigger_type,
			ts->client->irq);
	ts->client->irq = gpio_to_irq(ts->pdata->irq_gpio);
	tp_log_info("after INT trigger type:%x, irq=%d", ts->int_trigger_type,
			ts->client->irq);
	irq_flags = irq_table[ts->int_trigger_type] |IRQF_ONESHOT;
	ret = request_threaded_irq(ts->client->irq,NULL, 
								goodix_ts_irq_handler,irq_flags,
								ts->client->name, ts);
	if (ret) {
		tp_log_err( "Request IRQ failed!ERRNO:%d.\n",
				ret);
		ret = gpio_direction_input(ts->pdata->irq_gpio);
		if(ret)
		{
			tp_log_err("%s:Failed to set gpio %d direction line=%d\n",__func__,ts->pdata->irq_gpio,__LINE__);
			return ret;
		}  

		hrtimer_init(&ts->timer, CLOCK_MONOTONIC,
				HRTIMER_MODE_REL);
		ts->timer.function = goodix_ts_timer_handler;
		hrtimer_start(&ts->timer, ktime_set(1, 0),
				HRTIMER_MODE_REL);
		ts->use_irq = false;
		return ret;
	} else {
		gtp_irq_disable(ts);
		ts->use_irq = true;
		return 0;
	}
}

/*add the function of interface for the gesture*/
static ssize_t hw_goodix_easy_wakeup_gesture_show(struct kobject *dev,
		struct kobj_attribute *attr, char *buf)
{
	struct goodix_ts_data *ts ;
	ssize_t ret;

	ts= i2c_get_clientdata(i2c_connect_client);
	if(NULL == ts)
	{
		tp_log_err("%s: ts pointer is null .\n", __func__);
		return -EINVAL;
	}
	mutex_lock(&ts->gtp_sysfs_mutex);
	tp_log_debug("%s: easy_wakeup_gesture=0x%04x", __func__, ts->easy_wakeup_gesture);
	ret = snprintf(buf, PAGE_SIZE, "0x%04x\n",ts->easy_wakeup_gesture);
	mutex_unlock(&ts->gtp_sysfs_mutex);
	return ret;
}

static ssize_t hw_goodix_easy_wakeup_gesture_store(struct kobject *dev,
		struct kobj_attribute *attr, const char *buf, size_t size)
{
	struct goodix_ts_data *ts ;
	unsigned long value = 0;
	ssize_t ret;

	ts= i2c_get_clientdata(i2c_connect_client);
	if(NULL == ts)
	{
		tp_log_err("%s: ts pointer is null .\n", __func__);
		return -EINVAL;
	}
	
	ret = kstrtoul(buf, 10, &value);
	if (ret < 0)
		return ret;

	tp_log_debug("%s:the value=0x%04x", __func__, (unsigned int)value);
	if (value > 0xFFFF) {
		tp_log_err("%s: Invalid value:%ld", __func__, value);
		return -EINVAL;
	}
	
	tp_log_debug("%s:line = %d , rt_counter=%d\n",__func__,__LINE__, ts->client->dev.power.usage_count.counter);
	pm_runtime_get_sync(&ts->client->dev);
	tp_log_debug("%s:line = %d , rt_counter=%d\n",__func__,__LINE__, ts->client->dev.power.usage_count.counter);

	mutex_lock(&ts->gtp_sysfs_mutex);
	ts->easy_wakeup_gesture = value;
	tp_log_debug("%s: easy_wakeup_gesture=%d", __func__, ts->easy_wakeup_gesture);
	mutex_unlock(&ts->gtp_sysfs_mutex);
	
	tp_log_debug("%s:line = %d , rt_counter=%d\n",__func__,__LINE__, ts->client->dev.power.usage_count.counter);
	pm_runtime_put(&ts->client->dev);
	tp_log_debug("%s:line = %d , rt_counter=%d\n",__func__,__LINE__, ts->client->dev.power.usage_count.counter);

	return size;
}
static ssize_t hw_goodix_easy_wakeup_supported_gestures_show(struct kobject *dev,
		struct kobj_attribute *attr, char *buf)
{
	struct goodix_ts_data *ts ;
	ssize_t ret;

	ts= i2c_get_clientdata(i2c_connect_client);
	if(NULL == ts)
	{
		tp_log_err("%s: ts pointer is null .\n", __func__);
		return -EINVAL;
	}
	
	mutex_lock(&ts->gtp_sysfs_mutex);
	tp_log_debug("%s: easy_wakeup_supported_gestures=0x%04x\n", __func__, ts->pdata->easy_wakeup_supported_gestures);
	ret = snprintf(buf, PAGE_SIZE, "0x%04x\n",ts->pdata->easy_wakeup_supported_gestures);
	mutex_unlock(&ts->gtp_sysfs_mutex);
	
	tp_log_debug("%s:the returned byte length ret = %d \n",__func__,ret);
	
	return ret;

}
static inline ssize_t hw_goodix_rmi4_store_error(struct kobject *dev,
		struct kobj_attribute *attr, const char *buf, size_t size)
{
		return -EINVAL;
}

static struct kobj_attribute easy_wakeup_gesture = {
	.attr = {.name = "easy_wakeup_gesture", .mode = (S_IRUGO | S_IWUGO)},
	.show = hw_goodix_easy_wakeup_gesture_show,
	.store = hw_goodix_easy_wakeup_gesture_store,
};

static struct kobj_attribute easy_wakeup_supported_gestures = {
	.attr = {.name = "easy_wakeup_supported_gestures", .mode = S_IRUGO},
	.show = hw_goodix_easy_wakeup_supported_gestures_show,
	.store = hw_goodix_rmi4_store_error,
};

/*******************************************************
Function:
	add the node for easy_wakeup_geture
Input:
	void.
return:
    Executive outcomes. 0---succeed.
*******************************************************/

static int add_easy_wakeup_interfaces(void)
{
	int error = 0;
	struct kobject *properties_kobj;
	
	properties_kobj = tp_get_touch_screen_obj();
	if( NULL == properties_kobj )
	{
		tp_log_err("%s: Error, get kobj failed!\n", __func__);
		return -1;
	}
	
	/*add the node easy_wakeup_gesture_supported for apk to read*/
	error = sysfs_create_file(properties_kobj, &easy_wakeup_supported_gestures.attr);
	if (error)
	{
		kobject_put(properties_kobj);
		tp_log_err("%s: goodix_easy_wakeup_gesture_supported create file error\n", __func__);
		return -ENODEV;
	}
	
	/*add the node easy_wakeup_gesture apk to write*/
	error = sysfs_create_file(properties_kobj, &easy_wakeup_gesture.attr);
	if (error)
	{
		kobject_put(properties_kobj);
		tp_log_err("%s: goodix_easy_wakeup_gesture create file error\n", __func__);
		return -ENODEV;
	}
	return 0;
}

/*******************************************************
Function:
	Request input device Function.
Input:
	ts:private data.
Output:
	Executive outcomes.
	0: succeed, otherwise: failed.
*******************************************************/
static s8 gtp_request_input_dev(struct goodix_ts_data *ts)
{
    s8 ret = -1;
    s8 phys[32];
#if GTP_HAVE_TOUCH_KEY
    u8 index = 0;
#endif
  
    GTP_DEBUG_FUNC();
  
    ts->input_dev = input_allocate_device();
    if (ts->input_dev == NULL)
    {
        tp_log_err("Failed to allocate input device.");
        return -ENOMEM;
    }

    ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
#if GTP_ICS_SLOT_REPORT
    __set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
    input_mt_init_slots(ts->input_dev, 16,0);     // in case of "out of memory"
#else
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif

#if GTP_HAVE_TOUCH_KEY
    for (index = 0; index < GTP_MAX_KEY_NUM; index++)
    {
        input_set_capability(ts->input_dev, EV_KEY, touch_key_array[index]);  
    }
#endif

#if GTP_SLIDE_WAKEUP
	input_set_capability(ts->input_dev, EV_KEY, KEY_POWER);
	/* report the event that the driver supported to the input subsystem by set_bit*/
	input_set_capability(ts->input_dev, EV_KEY, KEY_F1);
	input_set_capability(ts->input_dev, EV_KEY, KEY_F2);
	input_set_capability(ts->input_dev, EV_KEY, KEY_F3);
	input_set_capability(ts->input_dev, EV_KEY, KEY_F4);
	input_set_capability(ts->input_dev, EV_KEY, KEY_F5);
#endif

#if GTP_WITH_PEN
	/* pen support */
	__set_bit(BTN_TOOL_PEN, ts->input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
	__set_bit(INPUT_PROP_POINTER, ts->input_dev->propbit);
#endif

#if GTP_CHANGE_X2Y
	GTP_SWAP(ts->abs_x_max, ts->abs_y_max);
#endif

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
				0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
				0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR,
				0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,
				0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID,
				0, 255, 0, 0);

	snprintf(phys, PHY_BUF_SIZE, "input/ts");
	ts->input_dev->name = GOODIX_DEV_NAME;
	ts->input_dev->phys = phys;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0xDEAD;
	ts->input_dev->id.product = 0xBEEF;
	ts->input_dev->id.version = 10427;

	ret = input_register_device(ts->input_dev);
    if (ret)
    {
        tp_log_err("Register %s input device failed", ts->input_dev->name);
        return -ENODEV;
    }
    
#ifdef CONFIG_HAS_EARLYSUSPEND
    ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    ts->early_suspend.suspend = goodix_ts_early_suspend;
    ts->early_suspend.resume = goodix_ts_late_resume;
    register_early_suspend(&ts->early_suspend);
#endif

    return 0;
}

//************** For GT9XXF Start *************//
#if GTP_COMPATIBLE_MODE

s32 gtp_fw_startup(struct i2c_client *client)
{
    u8 opr_buf[4];
    s32 ret = 0;
    
    //init sw WDT
	opr_buf[0] = 0xAA;
	ret = i2c_write_bytes(client, 0x8041, opr_buf, 1);
    if (ret < 0)
    {
        return FAIL;
    }
    
    //release SS51 & DSP
    opr_buf[0] = 0x00;
    ret = i2c_write_bytes(client, 0x4180, opr_buf, 1);
    if (ret < 0)
    {
        return FAIL;
    }
    //int sync
    gtp_int_sync(25);  
    
    //check fw run status
    ret = i2c_read_bytes(client, 0x8041, opr_buf, 1);
    if (ret < 0)
    {
        return FAIL;
    }
    if(0xAA == opr_buf[0])
    {
        tp_log_err("IC works abnormally,startup failed.");
        return FAIL;
    }
    else
    {
        tp_log_info("IC works normally, Startup success.");
        opr_buf[0] = 0xAA;
        i2c_write_bytes(client, 0x8041, opr_buf, 1);
        return SUCCESS;
    }
}

static s32 gtp_esd_recovery(struct i2c_client *client)
{
    s32 retry = 0;
    s32 ret = 0;
    struct goodix_ts_data *ts;
    
    ts = i2c_get_clientdata(client);
    
    gtp_irq_disable(ts);
    
    tp_log_info("GT9XXF esd recovery mode");
    gtp_reset_guitar(client, RESET_TIME_20_MS);       // reset & select I2C addr
    for (retry = 0; retry < 5; ++retry)
    {
        ret = gup_fw_download_proc(NULL, GTP_FL_ESD_RECOVERY); 
        if (FAIL == ret)
        {
            tp_log_err("esd recovery failed %d", retry+1);
            continue;
        }
        ret = gtp_fw_startup(ts->client);
        if (FAIL == ret)
        {
            tp_log_err("GT9XXF start up failed %d", retry+1);
            continue;
        }
        break;
    }
    gtp_irq_enable(ts);
    
    if (retry >= 5)
    {
        tp_log_err("failed to esd recovery");
        return FAIL;
    }
    
    tp_log_info("Esd recovery successful");
    return SUCCESS;
}

void gtp_recovery_reset(struct i2c_client *client)
{
#if GTP_ESD_PROTECT
    gtp_esd_switch(client, SWITCH_OFF);
#endif
    GTP_DEBUG_FUNC();
    
    gtp_esd_recovery(client); 
    
#if GTP_ESD_PROTECT
    gtp_esd_switch(client, SWITCH_ON);
#endif
}

static s32 gtp_bak_ref_proc(struct goodix_ts_data *ts, u8 mode)
{
    s32 ret = 0;
    s32 i = 0;
    s32 j = 0;
    u16 ref_sum = 0;
    u16 learn_cnt = 0;
    u16 chksum = 0;
    s32 ref_seg_len = 0;
    s32 ref_grps = 0;
    struct file *ref_filp = NULL;
    u8 *p_bak_ref;
    
    ret = gup_check_fs_mounted("/data");
    if (FAIL == ret)
    {
        ts->ref_chk_fs_times++;
        tp_log_debug("Ref check /data times/MAX_TIMES: %d / %d", ts->ref_chk_fs_times, GTP_CHK_FS_MNT_MAX);
        if (ts->ref_chk_fs_times < GTP_CHK_FS_MNT_MAX)
        {
            msleep(50);
            tp_log_info("/data not mounted.");
            return FAIL;
        }
        tp_log_info("check /data mount timeout...");
    }
    else
    {
        tp_log_info("/data mounted!!!(%d/%d)", ts->ref_chk_fs_times, GTP_CHK_FS_MNT_MAX);
    }
    
    p_bak_ref = (u8 *)kzalloc(ts->bak_ref_len, GFP_KERNEL);
    
    if (NULL == p_bak_ref)
    {
        tp_log_err("Allocate memory for p_bak_ref failed!");
        return FAIL;   
    }
    
    if (ts->is_950)
    {
        ref_seg_len = ts->bak_ref_len / 6;
        ref_grps = 6;
    }
    else
    {
        ref_seg_len = ts->bak_ref_len;
        ref_grps = 1;
    }
    ref_filp = filp_open(GTP_BAK_REF_PATH, O_RDWR | O_CREAT, 0666);
    if (IS_ERR(ref_filp))
    {
        tp_log_info("%s is unavailable, default backup-reference used", GTP_BAK_REF_PATH);
        goto bak_ref_default;
    }
    
    switch (mode)
    {
    case GTP_BAK_REF_SEND:
        tp_log_info("Send backup-reference");
        ref_filp->f_op->llseek(ref_filp, 0, SEEK_SET);
        ret = ref_filp->f_op->read(ref_filp, (char*)p_bak_ref, ts->bak_ref_len, &ref_filp->f_pos);
        if (ret < 0)
        {
            tp_log_err("failed to read bak_ref info from file, sending defualt bak_ref");
            goto bak_ref_default;
        }
        for (j = 0; j < ref_grps; ++j)
        {
            ref_sum = 0;
            for (i = 0; i < (ref_seg_len); i += 2)
            {
                ref_sum += (p_bak_ref[i + j * ref_seg_len] << 8) + p_bak_ref[i+1 + j * ref_seg_len];
            }
            learn_cnt = (p_bak_ref[j * ref_seg_len + ref_seg_len -4] << 8) + (p_bak_ref[j * ref_seg_len + ref_seg_len -3]);
            chksum = (p_bak_ref[j * ref_seg_len + ref_seg_len -2] << 8) + (p_bak_ref[j * ref_seg_len + ref_seg_len -1]);
            tp_log_debug("learn count = %d", learn_cnt);
            tp_log_debug("chksum = %d", chksum);
            tp_log_debug("ref_sum = 0x%04X", ref_sum & 0xFFFF);
            // Sum(1~ref_seg_len) == 1
            if (1 != ref_sum)
            {
                tp_log_info("wrong chksum for bak_ref, reset to 0x00 bak_ref");
                memset(&p_bak_ref[j * ref_seg_len], 0, ref_seg_len);
                p_bak_ref[ref_seg_len + j * ref_seg_len - 1] = 0x01;
            }
            else
            {
                if (j == (ref_grps - 1))
                {
                    tp_log_info("backup-reference data in %s used", GTP_BAK_REF_PATH);
                }
            }
        }
        ret = i2c_write_bytes(ts->client, GTP_REG_BAK_REF, p_bak_ref, ts->bak_ref_len);
        if (FAIL == ret)
        {
            tp_log_err("failed to send bak_ref because of iic comm error");
            filp_close(ref_filp, NULL);
            return FAIL;
        }
        break;
        
    case GTP_BAK_REF_STORE:
        tp_log_info("Store backup-reference");
        ret = i2c_read_bytes(ts->client, GTP_REG_BAK_REF, p_bak_ref, ts->bak_ref_len);
        if (ret < 0)
        {
            tp_log_err("failed to read bak_ref info, sending default back-reference");
            goto bak_ref_default;
        }
        ref_filp->f_op->llseek(ref_filp, 0, SEEK_SET);
        ref_filp->f_op->write(ref_filp, (char*)p_bak_ref, ts->bak_ref_len, &ref_filp->f_pos);
        break;
        
    default:
        tp_log_err("invalid backup-reference request");
        break;
    }
    filp_close(ref_filp, NULL);
    return SUCCESS;

bak_ref_default:
    
    for (j = 0; j < ref_grps; ++j)
    {
        memset(&p_bak_ref[j * ref_seg_len], 0, ref_seg_len);
        p_bak_ref[j * ref_seg_len + ref_seg_len - 1] = 0x01;  // checksum = 1     
    }
    ret = i2c_write_bytes(ts->client, GTP_REG_BAK_REF, p_bak_ref, ts->bak_ref_len);
    if (!IS_ERR(ref_filp))
    {
        tp_log_info("write backup-reference data into %s", GTP_BAK_REF_PATH);
        ref_filp->f_op->llseek(ref_filp, 0, SEEK_SET);
        ref_filp->f_op->write(ref_filp, (char*)p_bak_ref, ts->bak_ref_len, &ref_filp->f_pos);
        filp_close(ref_filp, NULL);
    }
    if (ret == FAIL)
    {
        tp_log_err("failed to load the default backup reference");
        return FAIL;
    }
    return SUCCESS;
}


static s32 gtp_verify_main_clk(u8 *p_main_clk)
{
    u8 chksum = 0;
    u8 main_clock = p_main_clk[0];
    s32 i = 0;
    
    if (main_clock < 50 || main_clock > 120)    
    {
        return FAIL;
    }
    
    for (i = 0; i < 5; ++i)
    {
        if (main_clock != p_main_clk[i])
        {
            return FAIL;
        }
        chksum += p_main_clk[i];
    }
    chksum += p_main_clk[5];
    if ( (chksum) == 0)
    {
        return SUCCESS;
    }
    else
    {
        return FAIL;
    }
}

static s32 gtp_main_clk_proc(struct goodix_ts_data *ts)
{
    s32 ret = 0;
    s32 i = 0;
    s32 clk_chksum = 0;
    struct file *clk_filp = NULL;
    u8 p_main_clk[6] = {0};

    ret = gup_check_fs_mounted("/data");
    if (FAIL == ret)
    {
        ts->clk_chk_fs_times++;
        tp_log_debug("Clock check /data times/MAX_TIMES: %d / %d", ts->clk_chk_fs_times, GTP_CHK_FS_MNT_MAX);
        if (ts->clk_chk_fs_times < GTP_CHK_FS_MNT_MAX)
        {
            msleep(50);
            tp_log_info("/data not mounted.");
            return FAIL;
        }
        tp_log_info("Check /data mount timeout!");
    }
    else
    {
        tp_log_info("/data mounted!!!(%d/%d)", ts->clk_chk_fs_times, GTP_CHK_FS_MNT_MAX);
    }
    
    clk_filp = filp_open(GTP_MAIN_CLK_PATH, O_RDWR | O_CREAT, 0666);
    if (IS_ERR(clk_filp))
    {
        tp_log_err("%s is unavailable, calculate main clock", GTP_MAIN_CLK_PATH);
    }
    else
    {
        clk_filp->f_op->llseek(clk_filp, 0, SEEK_SET);
        clk_filp->f_op->read(clk_filp, (char *)p_main_clk, 6, &clk_filp->f_pos);
       
        ret = gtp_verify_main_clk(p_main_clk);
        if (FAIL == ret)
        {
            // recalculate main clock & rewrite main clock data to file
            tp_log_err("main clock data in %s is wrong, recalculate main clock", GTP_MAIN_CLK_PATH);
        }
        else
        { 
            tp_log_info("main clock data in %s used, main clock freq: %d", GTP_MAIN_CLK_PATH, p_main_clk[0]);
            filp_close(clk_filp, NULL);
            goto update_main_clk;
        }
    }
    
#if GTP_ESD_PROTECT
    gtp_esd_switch(ts->client, SWITCH_OFF);
#endif
    ret = gup_clk_calibration();
    gtp_esd_recovery(ts->client);
    
#if GTP_ESD_PROTECT
    gtp_esd_switch(ts->client, SWITCH_ON);
#endif

    tp_log_info("calibrate main clock: %d", ret);
    if (ret < 50 || ret > 120)
    {
        tp_log_err("wrong main clock: %d", ret);
        goto exit_main_clk;
    }
    
    // Sum{0x8020~0x8025} = 0
    for (i = 0; i < 5; ++i)
    {
        p_main_clk[i] = ret;
        clk_chksum += p_main_clk[i];
    }
    p_main_clk[5] = 0 - clk_chksum;
    
    if (!IS_ERR(clk_filp))
    {
        tp_log_debug("write main clock data into %s", GTP_MAIN_CLK_PATH);
        clk_filp->f_op->llseek(clk_filp, 0, SEEK_SET);
        clk_filp->f_op->write(clk_filp, (char *)p_main_clk, 6, &clk_filp->f_pos);
        filp_close(clk_filp, NULL);
    }
    
update_main_clk:
    ret = i2c_write_bytes(ts->client, GTP_REG_MAIN_CLK, p_main_clk, 6);
    if (FAIL == ret)
    {
        tp_log_err("update main clock failed!");
        return FAIL;
    }
    return SUCCESS;
    
exit_main_clk:
    if (!IS_ERR(clk_filp))
    {
        filp_close(clk_filp, NULL);
    }
    return FAIL;
}

s32 gtp_gt9xxf_init(struct i2c_client *client)
{
    s32 ret = 0;
    
    ret = gup_fw_download_proc(NULL, GTP_FL_FW_BURN); 
    if (FAIL == ret)
    {
        return FAIL;
    }
    
    ret = gtp_fw_startup(client);
    if (FAIL == ret)
    {
        return FAIL;
    }
    return SUCCESS;
}

void gtp_get_chip_type(struct goodix_ts_data *ts)
{
    u8 opr_buf[10] = {0x00};
    s32 ret = 0;
    
    msleep(10);
    
    ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_CHIP_TYPE, opr_buf, 10);
    
    if (FAIL == ret)
    {
        tp_log_err("Failed to get chip-type, set chip type default: GOODIX_GT9");
        ts->chip_type = CHIP_TYPE_GT9;
        return;
    }
    
    if (!memcmp(opr_buf, "GOODIX_GT9", 10))
    {
        ts->chip_type = CHIP_TYPE_GT9;
    }
    else // GT9XXF
    {
        ts->chip_type = CHIP_TYPE_GT9F;
    }
    tp_log_info("Chip Type: %s", (ts->chip_type == CHIP_TYPE_GT9) ? "GOODIX_GT9" : "GOODIX_GT9F");
}

#endif
//************* For GT9XXF End ************//
static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_uA) : 0;
}

/**
 * goodix_power_on - Turn device power ON
 * @ts: driver private data
 *
 * Returns zero on success, else an error.
 */
static int goodix_power_on(struct goodix_ts_data *ts)
{
	int ret=0;
	tp_log_info("%s start line=%d\n",__func__,__LINE__);

	/*set the voltage for power vdd*/
	if (!IS_ERR(ts->vdd)) {
		ret = regulator_set_voltage(ts->vdd, GOODIX_VTG_MIN_UV,
					   GOODIX_VTG_MAX_UV);
		if (ret) {
			tp_log_err("%s:Regulator set_vtg failed vdd ret=%d\n",__func__, ret);
			return -1;
		}
		ret = reg_set_optimum_mode_check(ts->vdd,
			GOODIX_VDD_LOAD_MAX_UA);
		if (ret < 0) {
			tp_log_err("%s:Regulator vdd set_opt failed ret=%d\n",__func__, ret);
			goto err_set_opt_vdd;
		}
		ret = regulator_enable(ts->vdd);
		if (ret) {
			tp_log_err("%s:Regulator vdd enable failed ret=%d\n",__func__, ret);
			goto err_enable_vdd;
		}
	}
	else{
		return -ENODEV;
	}
	
    /*set the voltage for power vcc_i2c*/
	if (!IS_ERR(ts->vcc_i2c)) {
		ret = regulator_set_voltage(ts->vcc_i2c, GOODIX_I2C_VTG_MIN_UV,
					   GOODIX_I2C_VTG_MAX_UV);
		if (ret) {
			tp_log_err("%s:Regulator set_vtg failed vcc_i2c ret=%d\n",__func__, ret);
			goto err_set_vtg_vcc_i2c;
		}
		ret = reg_set_optimum_mode_check(ts->vcc_i2c,
			GOODIX_VIO_LOAD_MAX_UA);
		if (ret < 0) {
			tp_log_err("%s:Regulator vcc_i2c set_opt failed ret=%d\n",__func__,ret);
			goto err_set_opt_vcc_i2c;
		}
		ret = regulator_enable(ts->vcc_i2c);
		if (ret) {
			tp_log_err("%s:Regulator vcc_i2c enable failed ret=%d\n",__func__,ret);
			goto err_enable_vcc_i2c;
		}
	}
	else{
		return -ENODEV;
	}	

	return ret;

err_enable_vcc_i2c:
err_set_opt_vcc_i2c:
	if (!IS_ERR(ts->vcc_i2c))
		regulator_set_voltage(ts->vcc_i2c, 0, GOODIX_I2C_VTG_MAX_UV);
err_set_vtg_vcc_i2c:
	if (!IS_ERR(ts->vdd))
		regulator_disable(ts->vdd);
err_enable_vdd:
err_set_opt_vdd:
	if (!IS_ERR(ts->vdd))
		regulator_set_voltage(ts->vdd, 0, GOODIX_VTG_MAX_UV);
//err_set_vtg_vdd:
	//if (!IS_ERR(ts->vdd))
		//regulator_disable(ts->vdd);
//err_enable_avdd:
//err_set_opt_avdd:
	return ret;
}

/**
 * goodix_power_off - Turn device power OFF
 * @ts: driver private data
 *
 * Returns zero on success, else an error.
 */
static int goodix_power_off(struct goodix_ts_data *ts)
{
	int ret;

	if (!IS_ERR(ts->vcc_i2c)) {
		ret = regulator_set_voltage(ts->vcc_i2c, 0,
			GOODIX_I2C_VTG_MAX_UV);
		if (ret < 0)
			tp_log_err(
				"Regulator vcc_i2c set_vtg failed ret=%d\n",
				ret);
		ret = regulator_disable(ts->vcc_i2c);
		if (ret)
			tp_log_err(
				"Regulator vcc_i2c disable failed ret=%d\n",
				ret);
	}

	if (!IS_ERR(ts->vdd)) {
		ret = regulator_set_voltage(ts->vdd, 0, GOODIX_VTG_MAX_UV);
		if (ret < 0)
			tp_log_err(
				"Regulator vdd set_vtg failed ret=%d\n", ret);
		ret = regulator_disable(ts->vdd);
		if (ret)
			tp_log_err(
				"Regulator vdd disable failed ret=%d\n", ret);
	}
	return 0;
}

/**
 * goodix_power_init - Initialize device power
 * @ts: driver private data
 *
 * Returns zero on success, else an error.
 */
static int goodix_power_init(struct goodix_ts_data *ts)
{
	char const *power_pin_vdd;
	char const *power_pin_vbus;
	int ret=0;
	struct device *dev=&ts->client->dev;
	tp_log_info("%s start line=%d\n",__func__,__LINE__);
	
	/*get the power name from the dtsi*/
	ret = of_property_read_string(dev->of_node,"goodix,vdd", &power_pin_vdd);
	if (ret) {
		tp_log_err("%s: OF error vdd ret=%d\n", __func__, ret);
		return -ENODEV;
	}
	ret = of_property_read_string(dev->of_node,"goodix,vbus", &power_pin_vbus);
	if (ret) {
		tp_log_err("%s: OF error vbus ret=%d\n", __func__, ret);
		return -ENODEV;
	}
	
	/*get the power regulator */
	ts->vdd = regulator_get(dev, power_pin_vdd);
	if (IS_ERR(ts->vdd)) {
		ret = PTR_ERR(ts->vdd);
		tp_log_err("Regulator get failed vdd ret=%d\n", ret);
		return -ENODEV;
	}
	ts->vcc_i2c = regulator_get(dev, power_pin_vbus);
	if (IS_ERR(ts->vcc_i2c)) {
		ret = PTR_ERR(ts->vcc_i2c);
		tp_log_err("Regulator get failed vcc_i2c ret=%d\n", ret);
		return -ENODEV;
	}

	return ret;
}

/**
 * goodix_power_deinit - Deinitialize device power
 * @ts: driver private data
 *
 * Returns zero on success, else an error.
 */
static int goodix_power_deinit(struct goodix_ts_data *ts)
{
	regulator_put(ts->vdd);
	regulator_put(ts->vcc_i2c);

	return 0;
}
static int gtp_pinctrl_init(struct goodix_ts_data *ts)
{
	struct i2c_client *client = ts->client;
    tp_log_info("%s, begin \n", __func__);
	ts->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(ts->pinctrl)) {
		dev_err(&client->dev, "Failed to get pinctrl\n");
		return PTR_ERR(ts->pinctrl);
	}

	ts->pin_default =
		pinctrl_lookup_state(ts->pinctrl, "default");
	if (IS_ERR_OR_NULL(ts->pin_default)) {
		dev_err(&client->dev, "Failed to look up default state\n");
		return PTR_ERR(ts->pin_default);
	}

	ts->pin_sleep =
		pinctrl_lookup_state(ts->pinctrl, "sleep");
	if (IS_ERR_OR_NULL(ts->pin_sleep)) {
		dev_err(&client->dev, "Failed to look up sleep state\n");
		return PTR_ERR(ts->pin_sleep);
	}

	return 0;
}
static int goodix_parse_dt(struct device *dev,
			struct goodix_ts_platform_data *pdata)
{
	int ret=0;
	struct device_node *np = dev->of_node;
	u32 temp_val;

	tp_log_info("%s start line=%d\n",__func__,__LINE__);
	/*get reset and irq value from the dtsi*/
	pdata->reset_gpio = of_get_named_gpio_flags(np, "goodix,reset-gpio", 0, NULL);
	/*if (ret)
	{
		tp_log_err("%s pdata->reset_gpio line=%d\n",__func__,__LINE__);
		return ret ;
	}*/
	//pdata->reset_gpio=temp_val;
	pdata->irq_gpio = of_get_named_gpio_flags(np, "goodix,irq-gpio", 0, NULL);
	/*if (ret)
	{
		tp_log_err("%s pdata->irq_gpio line=%d\n",__func__,__LINE__);
		return ret ;
	}*/
	//pdata->irq_gpio=temp_val;
	
	ret = of_property_read_u32(np, "goodix,easy_wakeup_supported_gestures", &temp_val);
	if (!ret)
		pdata->easy_wakeup_supported_gestures = (u32)temp_val;
	return ret;
}

/*******************************************************
Function:
	I2c probe.
Input:
	client: i2c device struct.
	id: device id.
Output:
	Executive outcomes.
	0: succeed.
*******************************************************/
 static int goodix_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct goodix_ts_platform_data *pdata;
	struct goodix_ts_data *ts;
	int ret = -1;
	int retval;
	GTP_DEBUG_FUNC();
	tp_log_warning("%s start ... line=%d\n",__func__,__LINE__);
	tp_log_debug("GTP Driver Version: %s\n", GTP_DRIVER_VERSION);
	tp_log_debug("GTP Driver Built@%s, %s\n", __TIME__, __DATE__);
	tp_log_debug("GTP I2C Address: 0x%02x\n", client->addr);

	/*it will not come in the goodix probe,if the touch driver has existed*/
	if(touch_hw_data.read_touch_probe_flag)
	{
		retval = touch_hw_data.read_touch_probe_flag();
		if(retval)
		{
			tp_log_err("%s:it's not the first touch driver! \n",__func__);
			return -EPERM;
		}
		else
		{
			tp_log_info("%s:it's the first touch driver!line=%d\n",__func__,__LINE__);
		}
	}
	
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct goodix_ts_platform_data), GFP_KERNEL);
		if (!pdata) {
			tp_log_err("GTP Failed to allocate memory for pdata\n");
			return -ENOMEM;
		}
		ret = goodix_parse_dt(&client->dev, pdata);		
		if (ret)
		{
			tp_log_info("%s :goodix_parse_dt fail line=%d\n",__func__,__LINE__);
			goto exit_free_pdata;
		}
	} 
	else {
		pdata = client->dev.platform_data;
	}
	if (!pdata) {		

		tp_log_err("GTP invalid pdata\n");
		return -EINVAL;	
	}
	
	i2c_connect_client = client;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		tp_log_err("GTP I2C not supported\n");
		ret = -ENODEV;
		goto exit_free_pdata;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts) {
		tp_log_err("GTP not enough memory for ts\n");
		ret = -ENODEV;
		goto exit_free_pdata;
	}
	
#if USE_IRQ_THREAD 
#else
	INIT_WORK(&ts->work, goodix_ts_work_func);
#endif

	ts->client = client;
	ts->pdata = pdata;
	/*close the gesture when the driver probe*/
	ts->easy_wakeup_gesture = false;
	mutex_init(&(ts->gtp_sysfs_mutex));
	spin_lock_init(&ts->irq_lock);
	
#if GTP_ESD_PROTECT
	ts->clk_tick_cnt = 2 * HZ;      // HZ: clock ticks in 1 second generated by system
	tp_log_debug("Clock ticks for an esd cycle: %d\n", ts->clk_tick_cnt);  
	spin_lock_init(&ts->esd_lock);
#endif

	i2c_set_clientdata(client, ts);
	ts->gtp_rawdiff_mode = 0;
	/* Initialize device power*/
	ret = goodix_power_init(ts);
	if (ret) {
		tp_log_err("GTP power init failed\n");
		goto exit_free_client_data;
	}
	/* Turn device power vdd and vcc_i2c on*/
	ret = goodix_power_on(ts);
	if (ret) {
		tp_log_err("GTP power on failed\n");
		goto exit_deinit_power;
	}
	/* Request gpio(INT and RST) ports*/
	ret = gtp_request_io_port(ts);
	if (ret < 0) {
		tp_log_err("GTP request IO port failed.\n");
		ret = -ENODEV;
		goto exit_power_off;
	}
	/* initialize pinctrl */
	ret = gtp_pinctrl_init(ts);
	if (ret) {
		dev_err(&client->dev, "Can't initialize pinctrl\n");
			goto exit_power_off;
	}
	ret = pinctrl_select_state(ts->pinctrl, ts->pin_default);
	if (ret) {
		dev_err(&client->dev,
			"Can't select pinctrl default state\n");
		goto exit_power_off;
	}
	/* Reset touch panel IC.*/
	gtp_reset_guitar(ts->client, RESET_TIME_50_MS);
	
#if GTP_COMPATIBLE_MODE
	gtp_get_chip_type(ts);
	
	if (CHIP_TYPE_GT9F == ts->chip_type)
	{
		ret = gtp_gt9xxf_init(ts->client);
		if (FAIL == ret)
		{
			tp_log_info("Failed to init GT9XXF.\n");
		}
	}
#endif
	/*check the I2c communication function is good or not */
	ret = gtp_i2c_test(client);
	if (ret < 0)
	{
		tp_log_err("I2C communication ERROR!\n");
		ret = -ENODEV;
		goto exit_power_off;
	}
	/*initialize the config and send config to ic */
	tp_log_debug("abs_x_max=%d,abs_y_max=%d,line=%d \n",ts->abs_x_max,ts->abs_y_max,__LINE__);
	ret = gtp_init_panel(ts);
	if (ret < 0)
    {
		tp_log_err("GTP init panel failed.\n");
		ts->abs_x_max = GTP_MAX_WIDTH;
		ts->abs_y_max = GTP_MAX_HEIGHT;
		ts->int_trigger_type = GTP_INT_TRIGGER;
	}
	/*To create the fw update thread after enable pm runtime*/

	/*Request input device function*/
	ret = gtp_request_input_dev(ts);
	if (ret) {
		tp_log_err("GTP request input dev failed.\n");
		ret = -ENODEV;
		goto exit_power_off;
	}

	/*Request interrupt.*/
	tp_log_debug("abs_x_max=%d,abs_y_max=%d,int_trigger_type=%d,line=%d\n",ts->abs_x_max,ts->abs_y_max,ts->int_trigger_type,__LINE__);
	ret = gtp_request_irq(ts);
	if (ret < 0)
		tp_log_err("%s:GTP works in polling mode.\n",__func__);
	else
		tp_log_info("%s:GTP works in interrupt mode.\n",__func__);

	if (ts->use_irq)
		gtp_irq_enable(ts);

#if GTP_CREATE_WR_NODE
	init_wr_node(client);
#endif

#if defined(CONFIG_FB)
	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if (ret){
		tp_log_err("Unable to register fb_notifier: %d\n",ret);
		ret = -ENODEV;
		goto exit_free_irq;		
	}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = goodix_ts_early_suspend;
	ts->early_suspend.resume = goodix_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
	/*enable pm runtime*/
	pm_runtime_enable(&client->dev);
#if GTP_AUTO_UPDATE
	/*enter the update process if the GTP_AUTO_UPDATE is on (1)*/
	ret = gup_init_update_proc(ts);
	if (ret < 0)
	{
		tp_log_err("Create update thread error.\n");
	}
#endif
	/*set touch flag. avoid to probe repeatly!*/
	if(touch_hw_data.set_touch_probe_flag)
	{
		touch_hw_data.set_touch_probe_flag(1);
	}
	
	/*init the openshort test sysfs and init the gesture sysfs*/
	ret = gtp_test_sysfs_init();
	if (ret) 
	{
		tp_log_err( "%s: Error, goodix openshort test init sysfs fail! \n",__func__);
		goto exit_disable_pm;
	}

	ret = add_easy_wakeup_interfaces();
	if (ret < 0) 
	{
		tp_log_err( "%s: Error, easy wakeup init sysfs fail! \n",__func__);
		goto exit_free_sysfs;
	}
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
	/* detect current device successful, set the flag as present */
	set_hw_dev_flag(DEV_I2C_TOUCH_PANEL);
#endif
	tp_log_info("%s goodix probe success line=%d\n",__func__,__LINE__);

	return 0;

exit_free_sysfs:
	gtp_test_sysfs_deinit();
exit_disable_pm:
	pm_runtime_disable(&client->dev);
exit_free_irq:
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	cancel_work_sync(&ts->work);
	
#if USE_IRQ_THREAD 
#else	
	flush_workqueue(ts->goodix_wq);
	destroy_workqueue(ts->goodix_wq);
#endif

	input_unregister_device(ts->input_dev);
	if (ts->input_dev) {
		input_free_device(ts->input_dev);
		ts->input_dev = NULL;
	}
exit_power_off:
	goodix_power_off(ts);
exit_deinit_power:
	goodix_power_deinit(ts);
exit_free_client_data:
	i2c_set_clientdata(client, NULL);
	kfree(ts);
exit_free_pdata:
	if (client->dev.of_node)
		devm_kfree(&client->dev, pdata);
	return ret;
}

/*******************************************************
Function:
	Goodix touchscreen driver release function.
Input:
	client: i2c device struct.
Output:
	Executive outcomes. 0---succeed.
*******************************************************/
static int goodix_ts_remove(struct i2c_client *client)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	GTP_DEBUG_FUNC();
	/*del touch_screen sysfs*/
	gtp_test_sysfs_deinit();
	pm_runtime_disable(&client->dev);
#if defined(CONFIG_FB)
	if (fb_unregister_client(&ts->fb_notif))
		tp_log_err("Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#endif


#if GTP_CREATE_WR_NODE
	uninit_wr_node();
#endif

#if GTP_ESD_PROTECT
	//cancel_work_sync(gtp_esd_check_workqueue);
	//flush_workqueue(gtp_esd_check_workqueue);
	destroy_workqueue(gtp_esd_check_workqueue);
#endif

	if (ts) {
		if (ts->use_irq)
			free_irq(client->irq, ts);
		else
			hrtimer_cancel(&ts->timer);

		cancel_work_sync(&ts->work);
		
	#if	USE_IRQ_THREAD 
	#else	
		flush_workqueue(ts->goodix_wq);
		destroy_workqueue(ts->goodix_wq);
	#endif
	
		input_unregister_device(ts->input_dev);
		if (ts->input_dev) {
			input_free_device(ts->input_dev);
			ts->input_dev = NULL;
		}
		//kfree(ts->config_data);

		if (gpio_is_valid(ts->pdata->reset_gpio))
			gpio_free(ts->pdata->reset_gpio);
		if (gpio_is_valid(ts->pdata->irq_gpio))
			gpio_free(ts->pdata->irq_gpio);

		goodix_power_off(ts);
		goodix_power_deinit(ts);
		i2c_set_clientdata(client, NULL);
		kfree(ts);
	}

	return 0;
}

#if defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_FB)
/*******************************************************
Function:
	Early suspend function.
Input:
	h: early_suspend struct.
Output:
	None.
*******************************************************/
static int goodix_ts_suspend(struct device *dev)
{

	struct goodix_ts_data *ts ;
	int ret = -1,i;

	GTP_DEBUG_FUNC();
	ts= i2c_get_clientdata(i2c_connect_client);
	if(NULL == ts)
	{
		tp_log_err("%s: ts pointer is null line = %d.\n", __func__,__LINE__);
		return -EAGAIN;
	}
	/*no need to protect the fw update by judging the mask enter_update*/
	tp_log_warning( "%s : in ,easy_wakeup_gesture = 0x%04x.line=%d\n",__func__, ts->easy_wakeup_gesture,__LINE__);
#if GTP_ESD_PROTECT
	gtp_esd_switch(ts->client, SWITCH_OFF);
#endif

	ts->gtp_is_suspend = 1;
	
#if GTP_SLIDE_WAKEUP
	if((0x1f)&(ts->easy_wakeup_gesture))
	{
		ret = gtp_enter_doze(ts);
	}
	else	
#endif
	{
		if (ts->use_irq)
		{
			gtp_irq_disable(ts);
		}
		else
		{
			hrtimer_cancel(&ts->timer);
		}
		for (i = 0; i < GTP_MAX_TOUCH; i++)
		{
			gtp_touch_up(ts, i);
		}
		
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_sync(ts->input_dev);
		

		ret = gtp_enter_sleep(ts);
	}
	if (ret < 0)
		tp_log_err( "GTP early suspend failed.\n");
	/* to avoid waking up while not sleeping,
	 * delay 48 + 10ms to ensure reliability
	 */
	msleep(58);
	return 0;

}

/*******************************************************
Function:
	Late resume function.
Input:
	h: early_suspend struct.
Output:
	None.
*******************************************************/
static int goodix_ts_resume(struct device *dev)
{
	struct goodix_ts_data *ts ;
	int ret = -1;

	GTP_DEBUG_FUNC();
	ts= i2c_get_clientdata(i2c_connect_client);
	if(NULL == ts)
	{
		tp_log_err("%s: ts pointer is null line = %d .\n", __func__,__LINE__);
		return -EAGAIN;
	}

	/*no need to protect the fw update by judging the mask enter_update*/

	tp_log_warning("%s,in .LINE = %d. \n", __func__, __LINE__);
	ret = gtp_wakeup_sleep(ts);

#if GTP_SLIDE_WAKEUP
	doze_status = DOZE_DISABLED;
#endif

	if (ret < 0)
	{
		tp_log_err("%s,GTP later resume failed.\n",__func__);
	}
	
#if (GTP_COMPATIBLE_MODE)
	if (CHIP_TYPE_GT9F == ts->chip_type)
	{
		// do nothing
	}
	else
#endif
	{
		gtp_send_cfg(ts->client);
	}

	if (ts->use_irq)
	{
		gtp_irq_enable(ts);
	}
	else
	{
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}
	ts->gtp_is_suspend = 0;
#if GTP_ESD_PROTECT
	//ts->gtp_is_suspend = 0;
	gtp_esd_switch(ts->client, SWITCH_ON);
#endif
	return 0;
}
#endif
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank,ret = 0;
	struct goodix_ts_data *ts =
		container_of(self, struct goodix_ts_data, fb_notif);
	tp_log_debug("fb_notifier_callback enter line = %d \n",__LINE__);
	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
			ts && ts->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
		{
			tp_log_debug("%s:line = %d , rt_counter=%d\n",__func__,__LINE__, ts->client->dev.power.usage_count.counter);
			if (0 == runtime_count)
			{
				ret = pm_runtime_get_sync(&ts->client->dev);
				runtime_count = 1;//in case that fb_callback more than once
				tp_log_debug("%s:line = %d , rt_counter=%d, ret = %d\n",__func__,__LINE__,ts->client->dev.power.usage_count.counter,ret);
			}
			else
			{
				tp_log_info("%s:line = %d , runtime_count=%d,TP already resumed!\n",__func__,__LINE__, runtime_count);
			}
		}
		else if (*blank == FB_BLANK_POWERDOWN)
		{
			tp_log_debug("%s:line = %d , rt_counter=%d\n",__func__,__LINE__, ts->client->dev.power.usage_count.counter);
			if (1 == runtime_count)
			{
				ret = pm_runtime_put(&ts->client->dev);
				runtime_count = 0;//in case that fb_callback more than once
				tp_log_debug("%s:line = %d , rt_counter=%d, ret = %d\n",__func__,__LINE__,ts->client->dev.power.usage_count.counter,ret);
			}
			else
			{
				tp_log_info("%s:line = %d , runtime_count=%d,TP already suspended!\n",__func__,__LINE__, runtime_count);
			}
		}
		
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
/*******************************************************
Function:
	Early suspend function.
Input:
	h: early_suspend struct.
Output:
	None.
*******************************************************/
static void goodix_ts_early_suspend(struct early_suspend *h)
{
	struct goodix_ts_data *ts;

	ts = container_of(h, struct goodix_ts_data, early_suspend);
	goodix_ts_suspend(&ts->client->dev);
	return;
}

/*******************************************************
Function:
	Late resume function.
Input:
	h: early_suspend struct.
Output:
	None.
*******************************************************/
static void goodix_ts_late_resume(struct early_suspend *h)
{
	struct goodix_ts_data *ts;

	ts = container_of(h, struct goodix_ts_data, early_suspend);
	goodix_ts_resume(&ts->client->dev);
	return;
}
#endif
#endif /* !CONFIG_HAS_EARLYSUSPEND && !CONFIG_FB*/

#if GTP_ESD_PROTECT
s32 gtp_i2c_read_no_rst(struct i2c_client *client, u8 *buf, s32 len)
{
    struct i2c_msg msgs[2];
    s32 ret=-1;
    s32 retries = 0;

    GTP_DEBUG_FUNC();

    msgs[0].flags = !I2C_M_RD;
    msgs[0].addr  = client->addr;
    msgs[0].len   = GTP_ADDR_LENGTH;
    msgs[0].buf   = &buf[0];
    //msgs[0].scl_rate = 300 * 1000;    // for Rockchip, etc.
    
    msgs[1].flags = I2C_M_RD;
    msgs[1].addr  = client->addr;
    msgs[1].len   = len - GTP_ADDR_LENGTH;
    msgs[1].buf   = &buf[GTP_ADDR_LENGTH];
    //msgs[1].scl_rate = 300 * 1000;

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, msgs, 2);
        if(ret == 2)break;
        retries++;
    }
    if ((retries >= 5))
    {    
        tp_log_err("I2C Read: 0x%04X, %d bytes failed, errcode: %d!", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
    }
    return ret;
}

s32 gtp_i2c_write_no_rst(struct i2c_client *client,u8 *buf,s32 len)
{
    struct i2c_msg msg;
    s32 ret = -1;
    s32 retries = 0;

    GTP_DEBUG_FUNC();

    msg.flags = !I2C_M_RD;
    msg.addr  = client->addr;
    msg.len   = len;
    msg.buf   = buf;
    //msg.scl_rate = 300 * 1000;    // for Rockchip, etc

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, &msg, 1);
        if (ret == 1)break;
        retries++;
    }
    if((retries >= 5))
    {
        tp_log_err("I2C Write: 0x%04X, %d bytes failed, errcode: %d!", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
    }
    return ret;
}
/*******************************************************
Function:
	switch on & off esd delayed work
Input:
	client:  i2c device
	on:	SWITCH_ON / SWITCH_OFF
Output:
	void
*********************************************************/
void gtp_esd_switch(struct i2c_client *client, s32 on)
{
    struct goodix_ts_data *ts;
    
    ts = i2c_get_clientdata(client);
    spin_lock(&ts->esd_lock);
    
    if (SWITCH_ON == on)     // switch on esd 
    {
        if (!ts->esd_running)
        {
            ts->esd_running = 1;
            spin_unlock(&ts->esd_lock);
            tp_log_info("%s,Esd started \n", __func__);
            queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, ts->clk_tick_cnt);
        }
        else
        {
            spin_unlock(&ts->esd_lock);
        }
    }
    else    // switch off esd
    {
        if (ts->esd_running)
        {
            ts->esd_running = 0;
            spin_unlock(&ts->esd_lock);
            tp_log_info("%s,Esd cancelled \n", __func__);
            cancel_delayed_work_sync(&gtp_esd_check_work);
        }
        else
        {
            spin_unlock(&ts->esd_lock);
        }
    }
}
/*******************************************************
Function:
	Initialize external watchdog for esd protect
Input:
	client:  i2c device.
Output:
	result of i2c write operation.
		1: succeed, otherwise: failed
*********************************************************/
static s32 gtp_init_ext_watchdog(struct i2c_client *client)
{
    u8 opr_buffer[3] = {0x80, 0x41, 0xAA};
    tp_log_debug("[Esd]Init external watchdog");
    return gtp_i2c_write_no_rst(client, opr_buffer, 3);
}

/*******************************************************
Function:
	Esd protect function.
	Added external watchdog by meta, 2013/03/07
Input:
	work: delayed work
Output:
	None.
*******************************************************/
static void gtp_esd_check_func(struct work_struct *work)
{
    s32 i,j;
    s32 ret = -1;
    struct goodix_ts_data *ts = NULL;
    u8 esd_buf[4] = {0x80, 0x40};
    u8 hard_reset[6] = {0x42,0x26,0x01,0x01,0x01};
	
    GTP_DEBUG_FUNC();
   
    ts = i2c_get_clientdata(i2c_connect_client);

    if (ts->gtp_is_suspend ||ts->enter_update)
    {
        tp_log_info("Esd suspended!");
        return;
    }
    
    for (i = 0; i < 3; i++)
    {
        ret = gtp_i2c_read_no_rst(ts->client, esd_buf, 4);
        
        tp_log_debug("[Esd]0x8040 = 0x%02X, 0x8041 = 0x%02X", esd_buf[2], esd_buf[3]);
        if ((ret < 0))
        {
            // IIC communication problem
            continue;
        }
        else
        { 
            if ((esd_buf[2] == 0xAA) || (esd_buf[3] != 0xAA))
            {
                // IC works abnormally..
                u8 chk_buf[4] = {0x80, 0x40};
                
                gtp_i2c_read_no_rst(ts->client, chk_buf, 4);
                
                tp_log_debug("[Check]0x8040 = 0x%02X, 0x8041 = 0x%02X", chk_buf[2], chk_buf[3]);
                
                if ((chk_buf[2] == 0xAA) || (chk_buf[3] != 0xAA))
                {
                    i = 3;
                    break;
                }
                else
                {
                    continue;
                }
            }
            else 
            {
                // IC works normally, Write 0x8040 0xAA, feed the dog
                esd_buf[2] = 0xAA; 
                gtp_i2c_write_no_rst(ts->client, esd_buf, 3);
                break;
            }
        }
    }
    if (i >= 3)
    {
    #if GTP_COMPATIBLE_MODE
        if (CHIP_TYPE_GT9F == ts->chip_type)
        {        
            if (ts->rqst_processing)
            {
                tp_log_info("Request processing, no esd recovery");
            }
            else
            {
                tp_log_err("IC working abnormally! Process esd recovery.");
                gtp_esd_recovery(ts->client);
            }
        }
        else
    #endif
        {
            tp_log_err("IC working abnormally! Process reset guitar.");
            /* modified by tony	at 2013/12/07 */
            ret = -1;
            for(j=0;j<5;j++)
            {
            	ret = gtp_i2c_write(ts->client, hard_reset, 5);
            	if(ret >= 0)
            		{
            			tp_log_info("Success to hard reset for Guitar ,cmd status=%d",ret);
            			break;
            		}else{
            			tp_log_err("Failed to hard reset for Guitar ,cmd status=%d",ret);
            			}
            	}
            msleep(30);
            /* modified by tony at 2013/12/07 */
            gtp_reset_guitar(ts->client, RESET_TIME_50_MS);
        }
    }

    if(!ts->gtp_is_suspend)
    {
        queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, ts->clk_tick_cnt);
    }
    else
    {
        tp_log_info("Esd suspended!");
    }
    return;
}
#endif
#ifdef CONFIG_PM_RUNTIME
static const struct dev_pm_ops goodix_ts_dev_pm_rt_ops = {
	SET_RUNTIME_PM_OPS(goodix_ts_suspend, goodix_ts_resume,NULL)
};
#endif/*CONFIG_PM_RUNTIME*/

static const struct i2c_device_id goodix_ts_id[] = {
	{ GTP_I2C_NAME, 0 },
	{ }
};

static struct of_device_id goodix_match_table[] = {
	{ .compatible = "goodix,Goodix-TS", },
	{ },
};

static struct i2c_driver goodix_ts_driver = {
	.probe      = goodix_ts_probe,
	.remove     = goodix_ts_remove,
#ifdef CONFIG_HAS_EARLYSUSPEND
	.suspend    = goodix_ts_early_suspend,
	.resume     = goodix_ts_late_resume,
#endif

	.id_table   = goodix_ts_id,
	.driver = {
		.name     = GTP_I2C_NAME,
		.owner    = THIS_MODULE,
		.of_match_table = goodix_match_table,
		
#ifdef CONFIG_PM_RUNTIME
		.pm = &goodix_ts_dev_pm_rt_ops,
#endif
	},
};
/*******************************************************
Function:
    Driver Install function.
Input:
    None.
Output:
    Executive Outcomes. 0---succeed.
********************************************************/
/*put the Creat workqueue at the behind of  i2c_add_driver*/
static int __init goodix_ts_init(void)
{
    int ret;

    GTP_DEBUG_FUNC();   
    tp_log_info("GTP driver installing...");

#if GTP_ESD_PROTECT
    INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
    gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
#endif

#if	USE_IRQ_THREAD 
#else	
    goodix_wq = create_singlethread_workqueue("goodix_wq");
    if (!goodix_wq)
    {
        tp_log_err("Creat workqueue failed.");
        return -ENOMEM;
    }
#endif

    ret = i2c_add_driver(&goodix_ts_driver);


    return ret;
}
/*******************************************************
Function:
	Driver uninstall function.
Input:
	None.
Output:
	Executive Outcomes. 0---succeed.
********************************************************/
static void __exit goodix_ts_exit(void)
{
    GTP_DEBUG_FUNC();
    tp_log_info("GTP driver exited.");
    i2c_del_driver(&goodix_ts_driver);
	
#if	USE_IRQ_THREAD 
#else	
    if (goodix_wq)
    {
        destroy_workqueue(goodix_wq);
    }
#endif

}

module_init(goodix_ts_init);
module_exit(goodix_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");
