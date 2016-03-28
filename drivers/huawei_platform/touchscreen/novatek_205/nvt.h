/* drivers/input/touchscreen/novatek/nvt.h
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
 *
 */
 
#ifndef     _LINUX_NVT_TOUCH_H
#define     _LINUX_NVT_TOUCH_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/proc_fs.h>
#include <linux/unistd.h>
#include <linux/of_gpio.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/firmware.h>

#include <linux/pm_runtime.h>
#include <linux/version.h>
#include <misc/app_info.h>
#include <linux/mutex.h>

#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <huawei_platform/touchscreen/hw_tp_common.h>
//---INT trigger mode---
#define INT_TRIGGER_TYPE IRQ_TYPE_EDGE_FALLING

//---I2C driver info.---
#define NVT_I2C_NAME "nvt205"
#define I2C_FW_Address 0x01
#define I2C_HW_Address 0x70

//---Input device info.---
#define NVT_TS_NAME "NVTCapacitiveTouchScreen"

//---Touch info---
#define TOUCH_FINGER_NUM 10
#define TOUCH_KEY_NUM 0

//---Customerized func.---
#define MANUAL_FIRMWARE_NAME "manual_novatek_fw.bin"

#define NVT_ESD_TIMEOUT 3000

enum nvt_suspend_state {
    NVT_STATE_NONE,
    NVT_STATE_RESUME,
    NVT_STATE_SUSPEND,
};

struct nvt_flash_data{
    rwlock_t lock;
    unsigned char bufferIndex;
    unsigned int length;
    struct i2c_client *client;
};

/* modify */
struct nvt_ts_data{
    struct i2c_client *client;
    struct input_dev *input_dev;
    struct device * dev;
    struct work_struct work;
    struct workqueue_struct *nvt_wq;
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend;
#elif defined(CONFIG_FB)
    struct notifier_block fb_notif;
#endif
    struct notifier_block pm_notifier;
    struct mutex nvt_mutex;
    struct mutex nvt_irq_mutex;
    enum nvt_suspend_state suspend_state;
    int irq_gpio;
    int reset_gpio;
    int vdd_gpio;
    uint16_t abs_x_max;
    uint16_t abs_y_max;
    uint8_t max_touch_num;
    uint8_t max_key_num;
    uint32_t int_trigger_type;
    uint16_t easy_wakeup_supported_gestures;
    uint16_t easy_wakeup_gestures;
    uint16_t easy_wakeup_gestures_nova;
    uint32_t easy_wakeup_gesture_keys[12];
    uint16_t gesture_positions[12];
    uint8_t gesture_id;
    bool isIrqEnabled;
    //bool isInEasyWakeUpMode;
    uint16_t * touch_key_array;
    char const *product_name;
    uint16_t vendor_id;
    uint16_t fw_version;
    struct timer_list esd_timer;
    struct work_struct timer_work;
};

//extern struct nvt_ts_data *ts;
extern struct nvt_ts_data *ts;

//customed log tag for nvt205
#ifdef TP_LOG_NAME
#undef TP_LOG_NAME
#define TP_LOG_NAME "[NVT205]"
#endif

#if defined(CONFIG_PM_RUNTIME)
void nvt_pm_runtime_get(struct nvt_ts_data *);
void nvt_pm_runtime_put(struct nvt_ts_data *);
#endif

void nvt_hw_reset(void);
int nvt_i2c_read(struct i2c_client *client, uint8_t address, uint8_t *buf, uint8_t len);
void nvt_i2c_write (struct i2c_client *client, uint8_t address, uint8_t *data, uint8_t len);

#endif /* _LINUX_NVT_TOUCH_H */
