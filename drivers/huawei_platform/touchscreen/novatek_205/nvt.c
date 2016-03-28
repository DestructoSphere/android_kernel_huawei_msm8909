/* drivers/input/touchscreen/novatek/nvt.c
 *
 * Copyright (C) 2010 - 2014 Novatek, Inc.
 * 
 * History:
 * V1.0 : First Released
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/version.h>
 
#include "nvt.h"
#include "nvt_input.h"

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif

struct nvt_ts_data *ts;

//external references
extern int nvt_parse_dts(struct device *, struct nvt_ts_data *);

extern uint16_t get_vendor_id(void);                  //get vendor id of tp module
extern uint16_t get_fw_version(void);                //get fw_version

extern int nvt_input_init(struct nvt_ts_data *);    //input system init
extern void nvt_input_report(struct nvt_ts_data *, uint8_t *); //do input report

extern int nvt_proc_flash_init(void);                     //create proc-nodes for novatek debug
extern int nvt_sysfs_fwup_init(void);                   //create sysfs-nodes fw_update
extern void nvt_sysfs_test_init(void);                  //create sysfs-nodes for scap_test
extern void nvt_sysfs_easy_wakeup_init(void);   //create sysfs-nodes for easy_wakeup



 //IC Reset using GPIO trigger 
 void nvt_hw_reset(void)
 {
     gpio_direction_output(ts->reset_gpio, 1);    
     msleep(20);
     gpio_direction_output(ts->reset_gpio, 0);    
     msleep(20);
     gpio_direction_output(ts->reset_gpio, 1);    
     msleep(500);
 }

 //I2C Read & Write
int nvt_i2c_read(struct i2c_client *client, uint8_t address, uint8_t *buf, uint8_t len)
{
    struct i2c_msg msgs[2];
    int ret = -1;
    
    msgs[0].flags =  (client->flags & I2C_M_TEN);
    msgs[0].addr  = address;
    msgs[0].len   = 1;
    msgs[0].buf   = &buf[0];    
    ret = i2c_transfer(client->adapter, msgs, 1);
    if(ret != 1){
        tp_log_err("%s line%d: ret=%d, i2c read (write offset) failed!", __func__,__LINE__,ret);
        return ret;
    }

    msgs[0].flags =  (client->flags & I2C_M_TEN) | I2C_M_RD;
    msgs[0].addr  = address;
    msgs[0].len   = len-1;
    msgs[0].buf   = &buf[1];
    ret = i2c_transfer(client->adapter, msgs, 1);
    if(ret != 1){
        tp_log_err("%s line%d: ret=%d, i2c read (read data) failed!", __func__,__LINE__,ret);
        return ret;
    }

    return 2;
}
void nvt_i2c_write (struct i2c_client *client, uint8_t address, uint8_t *data, uint8_t len)
{
    struct i2c_msg msg;
    int ret = -1;
    int retries = 0;

    msg.flags = !I2C_M_RD;
    msg.addr  = address;
    msg.len   = len;
    msg.buf   = data;        
    
    while(retries < 5){
        ret = i2c_transfer(client->adapter, &msg, 1);
        if(ret == 1){
            break;
        }
        tp_log_err("%s line%d: ret=%d, times=%d, i2c write failed!",__func__,__LINE__,ret,retries);
        retries++;
    }
    return;
}

static int nvt_esd_check(void)
{
    uint8_t I2C_Buf[16];
    int ret = 0;

    tp_log_debug("%s %d: start esd check\n", __func__, __LINE__);
    I2C_Buf[0] = 0x86;
    ret = nvt_i2c_read(ts->client, I2C_FW_Address, I2C_Buf, 2);
    if(ret < 0){
        tp_log_info("%s: i2c read failed! ret=%d\n ",__func__,ret);
        return ret;
    }

    if(I2C_Buf[1] != 5){
        tp_log_info("%s: data not match!",__func__);
        return -1;
    }

    return 0;
}

static void nvt_esd_reset(void)
{
    disable_irq_nosync(ts->client->irq);
    nvt_hw_reset();
    enable_irq(ts->client->irq);
}
/*
static void nvt_try_reset(void)
{
    u8 i = 0;
    u8 retry = 3;

    for(i = 1; i <= retry; i++)
    {
        nvt_esd_reset();
        if(nvt_esd_check())
        {
           tp_log_info("%s,%d: try to reset %d times, failed!\n",__func__,__LINE__,i);
        }
        else
        {
           tp_log_info("%s,%d: try to reset %d times, succeed!\n",__func__,__LINE__,i);
           break;
        }
    }
}*/

void nvt_start_esd_timer(void)
{
    tp_log_debug("%s %d: start esd timer\n", __func__, __LINE__);
    mod_timer(&ts->esd_timer, jiffies + msecs_to_jiffies(NVT_ESD_TIMEOUT));
}

void nvt_stop_esd_timer(void)
{
    tp_log_debug("%s %d: stop esd timer\n", __func__, __LINE__);

    //----cancel the timer_work if it is running...
    cancel_work_sync(&ts->timer_work);
    del_timer_sync(&ts->esd_timer);
}

static void nvt_timer_work(struct work_struct *work)
{
    u8 i;
    u8 retry = 3;
    for(i=0;i<retry;i++){
        if(!nvt_esd_check()) break;
    }

    if(i == 3)
    {
        tp_log_info("%s,%d: Esd check failed, starting nt11205 restart!\n",__func__,__LINE__);
        nvt_esd_reset();
    }
    else
    {
        tp_log_debug("%s,%d: Esd check pass...\n",__func__,__LINE__);
    }

    nvt_start_esd_timer();
}

static void nvt_esd_timer(unsigned long handle)
{
    tp_log_debug( "%s: Timer triggered\n", __func__);

    if (!work_pending(&ts->timer_work))
    {
        schedule_work(&ts->timer_work);
    }else
    {
        tp_log_info("%s %d: timer_work is pending!\n", __func__, __LINE__);
    }

    return;
}


//work_func: reprt abs events here
static void nvt_ts_work_func(struct work_struct *work)
{        
    int ret=-1;
    uint8_t  point_data[63]={0};
    
    ret = nvt_i2c_read(ts->client, I2C_FW_Address, point_data, 63);
    if(ret < 0){
        tp_log_err("%s line%d: nvt_i2c_read failed.\n", __func__,__LINE__);
    } else {
        nvt_input_report(ts,point_data);
    }

    enable_irq(ts->client->irq);
}

static irqreturn_t nvt_ts_irq_handler(int irq, void *dev_id)
{    
    //note: _nosync must be used here!!!
    disable_irq_nosync(ts->client->irq);
    queue_work(ts->nvt_wq, &ts->work);
    return IRQ_HANDLED;
}

static int nv_hw_power_on(struct device *dev)
{
    int ret = 0;
  
    ret = gpio_direction_output(ts->vdd_gpio, 1);
    if(ret){
        tp_log_err("%s line%d: ret=%d, set_gpio failed!\n",__func__,__LINE__,ret);
    }
    msleep(200);

    return ret;
}
static int nv_hw_power_off(struct device *dev)
{
    int ret = 0;

    ret = gpio_direction_output(ts->vdd_gpio, 0);
    if(ret){
        tp_log_err("%s line%d: ret=%d, set_gpio failed!\n",__func__,__LINE__,ret);
    }
    msleep(200);

    return ret;
}
static int nv_request_gpios(struct device *dev)
{
    int ret = 0;
    
    ret = gpio_request(ts->vdd_gpio, "novatek-vdd");
    if (ret) {
        tp_log_err("%s line%d: Failed to get vdd gpio %d (code: %d)",
            __func__,__LINE__,ts->vdd_gpio, ret);
        goto request_gpio_vdd_falied;
    }
    
    ret |= gpio_request(ts->irq_gpio, "novatek-irq");
    if (ret) {
        tp_log_err("%s line%d: Failed to get irq gpio %d (code: %d)",
            __func__,__LINE__,ts->irq_gpio, ret);
        goto request_gpio_irq_falied;
    }
    
    ret |= gpio_request(ts->reset_gpio, "novatek-reset");
    if (ret) {
        tp_log_err("%s line%d: Failed to get reset gpio %d (code: %d)",
            __func__,__LINE__,ts->reset_gpio, ret);
        goto request_gpio_reset_falied;
    }
    ret = gpio_direction_output(ts->reset_gpio, 0);
    if(ret){
        tp_log_err("%s line%d: ret=%d, set_gpio failed!\n",__func__,__LINE__,ret);
    }
    return ret;
    
request_gpio_reset_falied:
    gpio_free(ts->irq_gpio);
request_gpio_irq_falied:
    gpio_free(ts->vdd_gpio);
request_gpio_vdd_falied:    
    return ret;
}

static int nv_release_gpios(void)
{
    gpio_free(ts->vdd_gpio);
    gpio_free(ts->irq_gpio);
    gpio_free(ts->reset_gpio);
    return 0;
}

//----nvt core suspend
static int nvt_core_suspend(struct nvt_ts_data * ts)
{    
    uint8_t buf[4]={0};  

    if (IS_GESTURE_SUPPORTED(ts->easy_wakeup_supported_gestures)
        && IS_GESTURE_ENBALED(ts->easy_wakeup_gestures)){
        tp_log_info("%s line%d: Enable touch wakeup gesture.\n",__func__,__LINE__);
        
        //---write i2c command to enter "wakeup gesture mode"---
        buf[0]=0xA1;
        buf[1]=(uint8_t)(ts->easy_wakeup_gestures_nova>>8);
        buf[2]=(uint8_t)(ts->easy_wakeup_gestures_nova&0x00FF);
        nvt_i2c_write(ts->client, I2C_FW_Address, buf, 3);
        tp_log_info("%s line%d: easy_wakeup_gestures H=0x%02X, L=0x%02X\n",
                            __func__,__LINE__,buf[1], buf[2]);
        
        //---write i2c command to enter "wakeup gesture mode"---
        buf[0]=0x88;
        buf[1]=0x55;
        buf[2]=0xAA;
        buf[3]=0xA6;
        nvt_i2c_write(ts->client, I2C_FW_Address, buf, 4);
        
        enable_irq_wake(ts->client->irq);
        //irq_set_irq_type(ts->client->irq,IRQF_TRIGGER_LOW | IRQF_NO_SUSPEND | IRQF_ONESHOT);
        tp_log_info("%s line%d: suspend to easy wakeup mode!\n",__func__,__LINE__);
    }else{
        //---write i2c command to enter "deep sleep mode"---
        buf[0]=0x88;
        buf[1]=0x55;
        buf[2]=0xAA;
        buf[3]=0xA5;
        nvt_i2c_write (ts->client,I2C_FW_Address, buf, 4);
        disable_irq(ts->client->irq);
        tp_log_info("%s line%d: normal suspended.\n", __func__,__LINE__);  
    }
    
    msleep(50);
    return 0;
}
//----nvt core resume
static int nvt_core_resume(struct nvt_ts_data * ts)
{
    nvt_hw_reset();    
    if(IS_GESTURE_SUPPORTED(ts->easy_wakeup_supported_gestures)
        && IS_GESTURE_ENBALED(ts->easy_wakeup_gestures)){
        //irq_set_irq_type(ts->client->irq,IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND | IRQF_ONESHOT);
        tp_log_info("%s line%d: resume from easy wakeup mode!\n",__func__,__LINE__);
    }else{
        enable_irq(ts->client->irq);
        tp_log_info("%s line%d: normal resumed.\n", __func__,__LINE__);          
    }
    
    return 0;
}

int nvt_check_device(struct nvt_ts_data * ts)
{
    int ret = 0;
    int retry = 3;
    uint8_t buf[7] = {0x00,};
    
    while(retry--){
        ret = nvt_i2c_read(ts->client, I2C_HW_Address, buf, 5);
        tp_log_info("%s line%d: buf[1]=%d, buf[2]=%d, buf[3]=%d, buf[4]=%d, buf[5]=%d\n",
            __func__,__LINE__,buf[1], buf[2], buf[3], buf[4], buf[5]);        
        if(ret <= 0){
            tp_log_err("%s line%d: i2c read test failed at retry=%d, ret = %d\n", 
                __func__,__LINE__,retry, ret);
        } else {
            tp_log_info("%s line%d: i2c read test succeed.\n",__func__,__LINE__);
            return 0;
        }
        msleep(5);
    }    
    
    return -1;
}

//---define runtime-suspend
#if defined(CONFIG_PM_RUNTIME)
static int nvt_rt_suspend(struct device *dev)
{
    struct nvt_ts_data * ts = dev_get_drvdata(dev);
    nvt_core_suspend(ts);
    tp_log_info("%s %d:runtime suspend succeed.\n", __func__, __LINE__);
    return 0;
}
static int nvt_rt_resume(struct device *dev)
{
    struct nvt_ts_data * ts = dev_get_drvdata(dev);
    nvt_core_resume(ts);
    tp_log_info("%s %d:runtime resume succeed.\n", __func__, __LINE__);
    return 0;
}

//---package of runtime suspend
void nvt_pm_runtime_get(struct nvt_ts_data * ts)
{
    tp_log_info( "%s line%d:system count:%d.\n", 
        __func__,__LINE__, ts->dev->power.usage_count.counter);
    pm_runtime_get_sync(ts->dev);
    tp_log_info( "%s line%d:system count:%d.\n", 
        __func__,__LINE__, ts->dev->power.usage_count.counter);
}
void nvt_pm_runtime_put(struct nvt_ts_data * ts)
{
    tp_log_info( "%s line%d: system count:%d.\n", 
        __func__,__LINE__,ts->dev->power.usage_count.counter);
    pm_runtime_put(ts->dev);
    tp_log_info( "%s line%d: system count:%d.\n",
        __func__, __LINE__,ts->dev->power.usage_count.counter);
}
#endif

//---fb_notifier_callback: do suspend and resume here
#if !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank = NULL;
    struct nvt_ts_data *ts =container_of(self, struct nvt_ts_data, fb_notif);

    tp_log_info("%s line%d:evdata = %d, evdata->data = %d, event = %d, ts = %d.\n",
        __func__, __LINE__, (int)evdata, (int)(evdata->data), (int)event, (int)ts);
    if (evdata && evdata->data && event == FB_EVENT_BLANK && ts) 
    {
        blank = evdata->data;
        /*In case of resume we get BLANK_UNBLANK and for suspen BLANK_POWERDOWN*/
        if (FB_BLANK_UNBLANK == *blank && NVT_STATE_RESUME != ts->suspend_state)
        {       
            nvt_pm_runtime_get(ts);
            ts->suspend_state = NVT_STATE_RESUME;
            tp_log_info("%s line%d: UNBLANK!\n", __func__,__LINE__);
            nvt_start_esd_timer();
        }
        else if (FB_BLANK_POWERDOWN == *blank && NVT_STATE_SUSPEND != ts->suspend_state)
        {
            nvt_stop_esd_timer();

            ts->suspend_state = NVT_STATE_SUSPEND;            
            nvt_pm_runtime_put(ts);
            tp_log_info("%s line%d: POWERDOWN!\n", __func__,__LINE__);
        }     
    }

    return 0;
}
#endif

void nvt_set_app_info(struct nvt_ts_data * ts)
{
    int ret = 0;
    char touch_info[100] = "";
    char vendor_name[20] = "";
    get_vendor_name_by_id(ts->vendor_id,vendor_name,sizeof(vendor_name));
    snprintf(touch_info,sizeof(touch_info),"nvt205_%s_V%d.%d",
                 vendor_name,ts->vendor_id, ts->fw_version);
    ret = app_info_set("touch_panel", touch_info);
    if (ret)
    {
        tp_log_err( "%s line%d: set_touch_info error\n",__func__,__LINE__);
    }
}


static int nvt_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret = 0;

    //if there is already another driver running
    if(already_has_tp_driver_running()){
            tp_log_warning("%s: Another tp driver is running! \n",__func__);
            return 0;        
    }

    tp_log_info("%s line%d: probe start...\n",__func__,__LINE__);

    //malloc for ts_data
    ts = kmalloc(sizeof(struct nvt_ts_data), GFP_KERNEL);
    if(ts == NULL){
        tp_log_err("%s line%d: probe: TS malloc fail!\n",__func__,__LINE__);
        ret = -ENOMEM;
        goto err_malloc_ts_data;
    }
    mutex_init(&ts->nvt_mutex);
    tp_log_info("%s line%d: probe: ts_data kmalloc succeed.\n", __func__, __LINE__);

    //parse dts configs to ts
    ret = nvt_parse_dts(&client->dev, ts);
    if(ret){
        tp_log_err("%s line%d: probe: ret=%d, parse dts failed!\n", __func__,__LINE__, ret);
        goto err_parse_dts_data;
    }
    tp_log_info("%s line%d: probe: parse dts succeed.\n", __func__, __LINE__);

    //check i2c func and set client data
    if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        tp_log_err("%s line%d: probe: i2c_check_functionality failed!\n",__func__,__LINE__);
        ret = -ENODEV;
        goto err_check_functionality_failed;
    }
    ts->client = client;
    ts->dev = &client->dev;
    i2c_set_clientdata(client, ts);
    tp_log_info("%s line%d: probe: check i2c-func&set_client_data succeed.\n",__func__,__LINE__);

    //requst gpios
    ret = nv_request_gpios(&client->dev);
    if (ret) {
        tp_log_err("%s line%d: probe: request gpio resource failed\n", __func__, __LINE__);
        goto err_request_gpio_failed;
    }
    tp_log_info("%s line%d: probe: request gpio succeed.\n", __func__, __LINE__);  
    ret = nv_hw_power_off(&client->dev);
    if (ret) {
        tp_log_err("%s line%d: probe: nv_hw_power_off failed!\n", __func__, __LINE__);
        goto err_hw_power_on_failed;
    }

    ret = gpio_direction_output(ts->reset_gpio, 1);
    if(ret){
        tp_log_err("%s line%d: ret=%d, set_gpio failed!\n",__func__,__LINE__,ret);
    }
    //power on
    ret = nv_hw_power_on(&client->dev);
    if (ret) {
        tp_log_err("%s line%d: probe: nv_hw_power_on failed!\n", __func__, __LINE__);
        goto err_hw_power_on_failed;        
    }
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
        set_hw_dev_flag(DEV_I2C_TOUCH_PANEL);
#endif
    tp_log_info("%s line%d: probe: power on succeed.\n", __func__, __LINE__);
    //Reset IC
    nvt_hw_reset();
    //check i2c read for 3 times till success
    ret = nvt_check_device(ts);
    if(ret){
        tp_log_err("%s line%d: probe: check devcie failed!\n", __func__, __LINE__);        
        goto err_check_device_failed;
    }
    tp_log_info("%s line%d: probe: check devcie succeed.\n", __func__, __LINE__);  

    //get vendor_id & fw_version       
    ts->vendor_id = get_vendor_id();
    ts->fw_version = get_fw_version(); 
    //set app_info
#ifdef CONFIG_APP_INFO
    nvt_set_app_info(ts);
    tp_log_info("%s line%d: probe: set app_info done.\n", __func__, __LINE__);
#endif

    //create work queue and init works
    ts->nvt_wq = create_workqueue("nvt205_wq");
        if(!ts->nvt_wq){
            tp_log_err("%s line%d: probe: create workqueue failed.\n", __func__,__LINE__);
            ret = -ENOMEM;    
            goto err_create_nvt_wq;
    }
    INIT_WORK(&ts->work, nvt_ts_work_func);
    tp_log_info("%s line%d: probe: init nvt_wkq succeed.\n", __func__, __LINE__);  

    ret = nvt_input_init(ts);
    if(ret){
        tp_log_err("%s line%d: probe: nvt_input_init failed!\n", __func__, __LINE__);
        goto err_nvt_input_init_failed;
    }
    tp_log_info("%s line%d: probe: nvt_input_init succeed.\n", __func__, __LINE__);

    //init pm_runtime
#if defined(CONFIG_PM_RUNTIME)    
    pm_runtime_get_noresume(ts->dev);
    pm_runtime_set_active(ts->dev);
    pm_runtime_enable(ts->dev);
    tp_log_info("%s line%d: probe: pm_runtime init done.\n", __func__, __LINE__);
#endif

    //register the callback to the FB
#if !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_FB)
    ts->suspend_state =NVT_STATE_RESUME;
    ts->fb_notif.notifier_call = fb_notifier_callback;
    ret = fb_register_client(&ts->fb_notif);
    if (ret)
    {
        tp_log_err("%s line%d: probe: ret=%d, fb_notifier register failed!\n",
            __func__, __LINE__,ret);
        goto err_fb_notifier_reg;
    }
    tp_log_info("%s line%d: probe: fb_notifier register succeed.\n", __func__, __LINE__);
#endif

    //set int-pin & request irq
    ts->int_trigger_type = INT_TRIGGER_TYPE;
    client->irq = gpio_to_irq(ts->irq_gpio);
    if(client->irq){
        tp_log_info("%s line%d: probe: int_trigger_type=%d\n",
            __func__,__LINE__,ts->int_trigger_type);
        ret = request_irq(client->irq, nvt_ts_irq_handler, ts->int_trigger_type, client->name, ts);
        if (ret){
            tp_log_err("%s line%d: probe: ret=%d, request irq failed.\n", __func__, __LINE__, ret);
            goto err_int_request_failed;
        }
        else {    
            enable_irq(ts->client->irq);
        }
    }    
    tp_log_info("%s line%d: probe: irq req and enable succeed.\n", __func__, __LINE__);

    INIT_WORK(&ts->timer_work, nvt_timer_work);
    setup_timer(&ts->esd_timer, nvt_esd_timer, (unsigned long)ts);
    //set device node
    nvt_proc_flash_init();
    //for test
    nvt_sysfs_test_init();
    //for fwupdate
    nvt_sysfs_fwup_init();
    //wakeup init
    nvt_sysfs_easy_wakeup_init();

    tp_log_info("%s line%d: sysfs init done.\n",__func__,__LINE__);

    //set this driver as running driver
    set_tp_driver_running();
    
#ifdef CONFIG_HAS_EARLYSUSPEND
    ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    ts->early_suspend.suspend = nvt_ts_early_suspend;
    ts->early_suspend.resume = nvt_ts_late_resume;
    register_early_suspend(&ts->early_suspend);
#endif

    tp_log_info("%s line%d: finished.\n", __func__,__LINE__);
    return 0;

err_int_request_failed:
err_fb_notifier_reg:    
err_nvt_input_init_failed:    
err_create_nvt_wq:
err_check_device_failed:
err_hw_power_on_failed:
    nv_release_gpios();
err_request_gpio_failed:
    i2c_set_clientdata(client, NULL);
err_check_functionality_failed:
err_parse_dts_data:   
    kfree(ts);    
err_malloc_ts_data:    
    return ret;
}

static int nvt_ts_remove(struct i2c_client *client)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&ts->early_suspend);
#endif
    
    tp_log_info("removing driver...\n");
    free_irq(client->irq, ts);
    input_unregister_device(ts->input_dev);
    i2c_set_clientdata(client, NULL);
    kfree(ts);    
    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void nvt_ts_early_suspend(struct early_suspend *h)
{    
    nvt_core_suspend(ts);
}

static void nvt_ts_late_resume(struct early_suspend *h)
{    
    nvt_core_resume(ts);
}
#endif

static const struct i2c_device_id nvt_ts_id[] = {
    { NVT_I2C_NAME, 0 },
    { }
};

#ifdef CONFIG_OF
static struct of_device_id novatek_match_table[] = {
        { .compatible = "nvt,nvt205_i2c_adapter",},
        { },
};
#endif

const struct dev_pm_ops nvt_pm_ops = {
#ifdef CONFIG_PM_RUNTIME
    SET_RUNTIME_PM_OPS(nvt_rt_suspend, nvt_rt_resume, NULL)
#endif
};

static struct i2c_driver nvt_i2c_driver = {
    .probe        = nvt_ts_probe,
    .remove        = nvt_ts_remove,
#ifdef CONFIG_HAS_EARLYSUSPEND
    .suspend = nvt_ts_early_suspend,
    .resume = nvt_ts_late_resume,
#endif    
    .id_table    = nvt_ts_id,
    .driver = {
        .name    = NVT_I2C_NAME,
        .owner    = THIS_MODULE,
        .of_match_table = novatek_match_table,
        .pm = &nvt_pm_ops,
    },
};

static int __init nvt_driver_init(void)
{
    int ret;
    
    //add i2c driver
    ret = i2c_add_driver(&nvt_i2c_driver);
    if(ret){
        tp_log_err("%s : failed to add i2c driver.", __func__);
        goto err_driver;
    }
    
    tp_log_info("%s : finished.\n", __func__);    

err_driver:
    return ret; 
}
static void __exit nvt_driver_exit(void)
{
    i2c_del_driver(&nvt_i2c_driver);
    i2c_unregister_device(ts->client);
    
    if(ts->nvt_wq)
        destroy_workqueue(ts->nvt_wq);
}


late_initcall(nvt_driver_init);
module_exit(nvt_driver_exit);

MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");
