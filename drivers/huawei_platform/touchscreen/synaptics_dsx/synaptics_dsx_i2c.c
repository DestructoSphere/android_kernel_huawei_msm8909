/*Add synaptics new driver "Synaptics DSX I2C V2.0"*/
/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
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
 */
//modify some logs, and del software_reset for it's not work for mmi_test
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include "synaptics_dsx.h"
#include "synaptics_dsx_i2c.h"

#ifdef KERNEL_ABOVE_2_6_38
#include <linux/input/mt.h>
#endif
#include <linux/of_gpio.h>

#ifdef CONFIG_HUAWEI_KERNEL
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif
#if !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#include <linux/pm_runtime.h>

#ifdef CONFIG_HUAWEI_DSM
#include <linux/dsm_pub.h>
#endif

#include "synaptics_dsx_esd.h"

#define MAX_F12_TOUCH_WIDTH 255

#define USE_IRQ_THREAD 1
#define USE_BTN_TOOL_FINGER 0
#define LCD_X_DEFAULT 480
#define LCD_Y_DEFAULT 800
#define LCD_ALL_DEFAULT 854

/* delay 50ms to execute the easy wake process */
#define GENSTRUE_WORK_DELAY 50

#define USE_WAKEUP_GESTURE 1
#define REPORT_GESTURE_LOCUS 1

#if USE_WAKEUP_GESTURE
#define GESTURE_FROM_APP(_x) (_x)

#define F11_2D_DATA28_OFFSET 53
#define F11_2D_DATA38_OFFSET 54
#define F11_2D_DATA39_OFFSET 55
#define F51_CUSTOM_DATA24_OFFSET 24

/* Doze wakeup threshold reg */
#define F01_RMI_CTRL03_OFFSET 3

#define F51_LOCUS_DATA_NUM 24
#define F51_LOCUS_DATA_LENS 4
#define F51_LETTER_LOCUS_NUM 6
#define F51_LINEAR2_LOCUS_NUM 4
#define F51_LINEAR_LOCUS_NUM 2
#define F51_DOUTAP_LOCUS_NUM 2

/*Gesture register(F11_2D_data38) value*/
#define DOUBLE_CLICK_WAKEUP  (1<<0)
#define LINEAR_SLIDE_DETECTED  (1<<1)
#define CIRCLE_SLIDE_DETECTED  (1<<3)
#define SPECIFIC_LETTER_DETECTED  (1<<6)

/*Linear esture register(F51_custom_data0) value*/
/*Change reg define for diff fw*/
#define LINEAR_SLIDE_LEFT_TO_RIGHT 0x1
#define LINEAR_SLIDE_RIGHT_TO_LEFT 0x2
#define LINEAR_SLIDE_TOP_TO_BOTTOM 0x4
#define LINEAR_SLIDE_BOTTOM_TO_TOP 0x8
#define LINEAR_SLIDE_TOP_TO_BOTTOM2 0x80
/* Reg defination for cirlce gesture of clockwise and counterclockwise */
#define CIRCLE_SLIDE_CLOCKWISE 0x10
#define CIRCLE_SLIDE_COUNTERCLOCKWISE 0x20

/*Letter gesture register(F11_2D_DATA39 (04)/01) value*/
#define SPECIFIC_LETTER_c 0x63
#define SPECIFIC_LETTER_e 0x65
#define SPECIFIC_LETTER_m 0x6D
#define SPECIFIC_LETTER_w 0x77

/* For case that need driver to force calibrate TP */
#define FORCE_TIMEOUT_10MS 10
#define F54_FORCE_CAL 1<<1
#define GESTURE_TIMER_INTERVAL  HZ //jiffies 1S

enum synaptics_gesture_num {
    GESTURE_DOUBLE_CLICK = 0,
    GESTURE_SLIDE_L2R,
    GESTURE_SLIDE_R2L,
    GESTURE_SLIDE_T2B,
    GESTURE_SLIDE_B2T,
    GESTURE_SLIDE_T2B2,
    GESTURE_CIRCLE_SLIDE,
    GESTURE_LETTER_c,
    GESTURE_LETTER_e,
    GESTURE_LETTER_m,
    GESTURE_LETTER_w,
    GESTURE_PALM_COVERED,
    GESTURE_MAX,
};

#define APP_ENABLE_GESTURE(x)  ((u32)(1<<(x)))
#define APP_ENABLE_LINEAR  ((u32)((1<<GESTURE_SLIDE_L2R)|\
                                        (1<<GESTURE_SLIDE_R2L)|\
                                        (1<<GESTURE_SLIDE_T2B)|\
                                        (1<<GESTURE_SLIDE_B2T)|\
                                        (1<<GESTURE_SLIDE_T2B2)))
#define APP_ENABLE_LETTERS  ((u32)((1<<GESTURE_LETTER_c)|\
                                    (1<<GESTURE_LETTER_e)|\
                                    (1<<GESTURE_LETTER_m)|\
                                    (1<<GESTURE_LETTER_w)))

#define APP_ENABLE_WAKEUP_GESTURE  ((u32)(APP_ENABLE_GESTURE(GESTURE_DOUBLE_CLICK))|\
                                        (APP_ENABLE_LINEAR)|\
                                        (APP_ENABLE_GESTURE(GESTURE_CIRCLE_SLIDE))|\
                                        (APP_ENABLE_LETTERS))

#define IS_GESTURE_ENABLE(gesture, var) (APP_ENABLE_GESTURE((gesture)) & (var))

/*Use to switch between reduce mode and gesture mode*/
#define F11_REDUCE_MODE (1 << 0)
#define F11_GESTURE_MODE (1 << 2)
#endif/*USE_WAKEUP_GESTURE*/

static struct device *core_dev_ptr = NULL;

/*For gestures the max report point num is 6*/
#define MAX_LOTUS_NUM  6
u32 easywake_position[MAX_LOTUS_NUM] = {0};
#define DOUBLE_TAP_ZONE_BYTES  6

/*store the default double tap zone defined by fw */
u8 double_tap_zone[DOUBLE_TAP_ZONE_BYTES+1] = {0};

/* indicate if the easy wakeup process is running */
static bool easy_wakeup_process = false;

/*delay 8s for permitting glove to finger on the edge of TP, when tp resumed*/
#define DELAY_TIME_ENABLE_EDGE_GLOVE_SWITCH 8000
#define F51_CTRL_EDGE_GLOVE_OFFSET 20
static bool edge_glove_switch_process = false;
static void synaptics_edge_glove_to_finger_delay_work(struct work_struct *work);
#define F51_CTRL_GRIP_ALGORITHM_MODE_OFFSET 21
#define F51_CTRL_GRIP_ALGORITHM_DISTANCE_OFFSET 22
#define GRIP_ALGORITHM_BYTES_NUM  4
static bool grip_algorithm_state = false;
#if USE_WAKEUP_GESTURE
static u32 gesture_count[GESTURE_MAX] = {0};
#endif

#define DSM_BASIC_INFO_LEN (512)
/* Used for lcd to control power state */

#ifdef CONFIG_HUAWEI_DSM
ssize_t synaptics_dsm_record_basic_err_info( void );

/* force dump some tp infomation */
int dump (int type, void *buff, int size)
{
    int used_size = 0;
    struct dsm_client *tp_client;

    /* get tp dsm client */
    tp_client = tp_dsm_get_client();

    /* save tp basice infomation */
    used_size = synaptics_dsm_record_basic_err_info();

    if( used_size > 0 )
    {
        tp_log_debug("%s: force dump tp error! \n",__func__);
        snprintf( buff, tp_client->used_size, tp_client->dump_buff );
    }

    return tp_client->used_size;

}

/* add tp dump_func */
struct dsm_client_ops tp_ops={
    .poll_state = NULL,
    .dump_func = dump,
};

/* dsm client for tp */
struct dsm_dev dsm_i2c = {
    .name = "dsm_i2c_bus",    // dsm client name
    .fops = &tp_ops,        // options
    .buff_size = TP_RADAR_BUF_MAX,        // buffer size
};
struct dsm_client *tp_dclient=NULL;

/* move this var out of CONFIG_HUAWEI_DSM */
static struct synaptics_dsx_platform_data *tp_data;
#endif/*CONFIG_HUAWEI_DSM*/

#ifndef USE_IRQ_THREAD
static struct workqueue_struct *synaptics_wq;
#endif

#ifdef ENABLE_VIRTUAL_KEYS
static struct kobject *board_properties_kobj;
#endif
#endif /*CONFIG_HUAWEI_KERNEL*/



#define DRIVER_NAME "synaptics_dsx_i2c"
#define INPUT_PHYS_NAME "synaptics_dsx_i2c/input0"

#ifdef KERNEL_ABOVE_2_6_38
#define TYPE_B_PROTOCOL
#endif

#define NO_0D_WHILE_2D
#define REPORT_2D_Z
#define REPORT_2D_W
#define F12_DATA_15_WORKAROUND

#ifdef CONFIG_HUAWEI_KERNEL
#define IGNORE_FN_INIT_FAILURE
#endif /*CONFIG_HUAWEI_KERNEL*/

#define RPT_TYPE (1 << 0)
#define RPT_X_LSB (1 << 1)
#define RPT_X_MSB (1 << 2)
#define RPT_Y_LSB (1 << 3)
#define RPT_Y_MSB (1 << 4)
#define RPT_Z (1 << 5)
#define RPT_WX (1 << 6)
#define RPT_WY (1 << 7)
#define RPT_DEFAULT (RPT_TYPE | RPT_X_LSB | RPT_X_MSB | RPT_Y_LSB | RPT_Y_MSB)

#define EXP_FN_WORK_DELAY_MS 1000 /* ms */

#define SYN_I2C_RETRY_TIMES 5
#define MAX_F11_TOUCH_WIDTH 15
#define CHECK_STATUS_TIMEOUT_MS 100

#define F01_STD_QUERY_LEN 21
#define F01_BUID_ID_OFFSET 18
#define F11_STD_QUERY_LEN 9
#define F11_STD_CTRL_LEN 10
#define F11_STD_DATA_LEN 12

#define STATUS_NO_ERROR 0x00
#define STATUS_RESET_OCCURRED 0x01
#define STATUS_INVALID_CONFIG 0x02
#define STATUS_DEVICE_FAILURE 0x03
#define STATUS_CONFIG_CRC_FAILURE 0x04
#define STATUS_FIRMWARE_CRC_FAILURE 0x05
#define STATUS_CRC_IN_PROGRESS 0x06

#define NORMAL_OPERATION (0 << 0)
#define SENSOR_SLEEP (1 << 0)
#define NO_SLEEP_OFF (0 << 2)
#define NO_SLEEP_ON (1 << 2)
#define CONFIGURED (1 << 7)

static int synaptics_rmi4_i2c_read(struct synaptics_rmi4_data *rmi4_data,
        unsigned short addr, unsigned char *data,
        unsigned short length);

static int synaptics_rmi4_i2c_write(struct synaptics_rmi4_data *rmi4_data,
        unsigned short addr, unsigned char *data,
        unsigned short length);

static int synaptics_rmi4_f12_set_enables(struct synaptics_rmi4_data *rmi4_data,
        unsigned short ctrl28);

static int synaptics_rmi4_free_fingers(struct synaptics_rmi4_data *rmi4_data);
static int synaptics_rmi4_reinit_device(struct synaptics_rmi4_data *rmi4_data);
static int synaptics_rmi4_reset_device(struct synaptics_rmi4_data *rmi4_data);
static int synaptics_gpio_setup(int gpio, bool configure, int dir, int state);

#ifdef CONFIG_HAS_EARLYSUSPEND
static ssize_t synaptics_rmi4_full_pm_cycle_show(struct device *dev,
        struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_full_pm_cycle_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count);

static void synaptics_rmi4_early_suspend(struct early_suspend *h);

static void synaptics_rmi4_late_resume(struct early_suspend *h);
#endif

static int synaptics_rmi4_suspend(struct device *dev);

static int synaptics_rmi4_resume(struct device *dev);

static ssize_t synaptics_rmi4_f01_reset_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_f01_productinfo_show(struct device *dev,
        struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_f01_buildid_show(struct device *dev,
        struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_f01_flashprog_show(struct device *dev,
        struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_0dbutton_show(struct device *dev,
        struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_0dbutton_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_suspend_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count);
static ssize_t synaptics_easy_wakeup_supported_gestures_show(struct device *dev,
        struct device_attribute *attr, char *buf);
static ssize_t synaptics_easy_wakeup_gesture_show(struct device *dev,
        struct device_attribute *attr, char *buf);
static ssize_t synaptics_easy_wakeup_gesture_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size);

/*For apk to read gesture locus point position*/
static ssize_t synaptics_easy_wakeup_position_show(struct device *dev,
        struct device_attribute *attr, char *buf);
static ssize_t synaptics_easy_wakeup_position_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size);

struct synaptics_rmi4_f01_device_status {
    union {
        struct {
            unsigned char status_code:4;
            unsigned char reserved:2;
            unsigned char flash_prog:1;
            unsigned char unconfigured:1;
        } __packed;
        unsigned char data[1];
    };
};

struct synaptics_rmi4_f12_query_5 {
    union {
        struct {
            unsigned char size_of_query6;
            struct {
                unsigned char ctrl0_is_present:1;
                unsigned char ctrl1_is_present:1;
                unsigned char ctrl2_is_present:1;
                unsigned char ctrl3_is_present:1;
                unsigned char ctrl4_is_present:1;
                unsigned char ctrl5_is_present:1;
                unsigned char ctrl6_is_present:1;
                unsigned char ctrl7_is_present:1;
            } __packed;
            struct {
                unsigned char ctrl8_is_present:1;
                unsigned char ctrl9_is_present:1;
                unsigned char ctrl10_is_present:1;
                unsigned char ctrl11_is_present:1;
                unsigned char ctrl12_is_present:1;
                unsigned char ctrl13_is_present:1;
                unsigned char ctrl14_is_present:1;
                unsigned char ctrl15_is_present:1;
            } __packed;
            struct {
                unsigned char ctrl16_is_present:1;
                unsigned char ctrl17_is_present:1;
                unsigned char ctrl18_is_present:1;
                unsigned char ctrl19_is_present:1;
                unsigned char ctrl20_is_present:1;
                unsigned char ctrl21_is_present:1;
                unsigned char ctrl22_is_present:1;
                unsigned char ctrl23_is_present:1;
            } __packed;
            struct {
                unsigned char ctrl24_is_present:1;
                unsigned char ctrl25_is_present:1;
                unsigned char ctrl26_is_present:1;
                unsigned char ctrl27_is_present:1;
                unsigned char ctrl28_is_present:1;
                unsigned char ctrl29_is_present:1;
                unsigned char ctrl30_is_present:1;
                unsigned char ctrl31_is_present:1;
            } __packed;
        };
        unsigned char data[5];
    };
};

struct synaptics_rmi4_f12_query_8 {
    union {
        struct {
            unsigned char size_of_query9;
            struct {
                unsigned char data0_is_present:1;
                unsigned char data1_is_present:1;
                unsigned char data2_is_present:1;
                unsigned char data3_is_present:1;
                unsigned char data4_is_present:1;
                unsigned char data5_is_present:1;
                unsigned char data6_is_present:1;
                unsigned char data7_is_present:1;
            } __packed;
            struct {
                unsigned char data8_is_present:1;
                unsigned char data9_is_present:1;
                unsigned char data10_is_present:1;
                unsigned char data11_is_present:1;
                unsigned char data12_is_present:1;
                unsigned char data13_is_present:1;
                unsigned char data14_is_present:1;
                unsigned char data15_is_present:1;
            } __packed;
        };
        unsigned char data[3];
    };
};

struct synaptics_rmi4_f12_ctrl_8 {
    union {
        struct {
            unsigned char max_x_coord_lsb;
            unsigned char max_x_coord_msb;
            unsigned char max_y_coord_lsb;
            unsigned char max_y_coord_msb;
            unsigned char rx_pitch_lsb;
            unsigned char rx_pitch_msb;
            unsigned char tx_pitch_lsb;
            unsigned char tx_pitch_msb;
            unsigned char low_rx_clip;
            unsigned char high_rx_clip;
            unsigned char low_tx_clip;
            unsigned char high_tx_clip;
            unsigned char num_of_rx;
            unsigned char num_of_tx;
        };
        unsigned char data[14];
    };
};

struct synaptics_rmi4_f12_ctrl_23 {
    union {
        struct {
            unsigned char obj_type_enable;
            unsigned char max_reported_objects;
        };
        unsigned char data[2];
    };
};

struct synaptics_rmi4_f12_finger_data {
    unsigned char object_type_and_status;
    unsigned char x_lsb;
    unsigned char x_msb;
    unsigned char y_lsb;
    unsigned char y_msb;
#ifdef REPORT_2D_Z
    unsigned char z;
#endif
#ifdef REPORT_2D_W
    unsigned char wx;
    unsigned char wy;
#endif
};

struct synaptics_rmi4_f1a_query {
    union {
        struct {
            unsigned char max_button_count:3;
            unsigned char reserved:5;
            unsigned char has_general_control:1;
            unsigned char has_interrupt_enable:1;
            unsigned char has_multibutton_select:1;
            unsigned char has_tx_rx_map:1;
            unsigned char has_perbutton_threshold:1;
            unsigned char has_release_threshold:1;
            unsigned char has_strongestbtn_hysteresis:1;
            unsigned char has_filter_strength:1;
        } __packed;
        unsigned char data[2];
    };
};

struct synaptics_rmi4_f1a_control_0 {
    union {
        struct {
            unsigned char multibutton_report:2;
            unsigned char filter_mode:2;
            unsigned char reserved:4;
        } __packed;
        unsigned char data[1];
    };
};

struct synaptics_rmi4_f1a_control {
    struct synaptics_rmi4_f1a_control_0 general_control;
    unsigned char button_int_enable;
    unsigned char multi_button;
    unsigned char *txrx_map;
    unsigned char *button_threshold;
    unsigned char button_release_threshold;
    unsigned char strongest_button_hysteresis;
    unsigned char filter_strength;
};

struct synaptics_rmi4_f1a_handle {
    int button_bitmask_size;
    unsigned char max_count;
    unsigned char valid_button_count;
    unsigned char *button_data_buffer;
    unsigned char *button_map;
    struct synaptics_rmi4_f1a_query button_query;
    struct synaptics_rmi4_f1a_control button_control;
};

struct synaptics_rmi4_exp_fhandler {
    struct synaptics_rmi4_exp_fn *exp_fn;
    bool insert;
    bool remove;
    struct list_head link;
};

struct synaptics_rmi4_exp_fn_data {
    bool initialized;
    bool queue_work;
    struct mutex mutex;
    struct list_head list;
    struct delayed_work work;
    struct workqueue_struct *workqueue;
    struct synaptics_rmi4_data *rmi4_data;
};

static struct synaptics_rmi4_exp_fn_data exp_data;

/*To fix CTS issue*/
static struct device_attribute attrs[] = {
#ifdef CONFIG_HAS_EARLYSUSPEND
    __ATTR(full_pm_cycle, (S_IRUGO | S_IWUSR|S_IWGRP),
            synaptics_rmi4_full_pm_cycle_show,
            synaptics_rmi4_full_pm_cycle_store),
#endif
    __ATTR(reset, S_IWUSR|S_IWGRP,
            synaptics_rmi4_show_error,
            synaptics_rmi4_f01_reset_store),
    __ATTR(productinfo, S_IRUGO,
            synaptics_rmi4_f01_productinfo_show,
            synaptics_rmi4_store_error),
    __ATTR(buildid, S_IRUGO,
            synaptics_rmi4_f01_buildid_show,
            synaptics_rmi4_store_error),
    __ATTR(flashprog, S_IRUGO,
            synaptics_rmi4_f01_flashprog_show,
            synaptics_rmi4_store_error),
    __ATTR(0dbutton, (S_IRUGO | S_IWUSR|S_IWGRP),
            synaptics_rmi4_0dbutton_show,
            synaptics_rmi4_0dbutton_store),
    __ATTR(suspend, S_IWUSR|S_IWGRP,
            synaptics_rmi4_show_error,
            synaptics_rmi4_suspend_store),
    __ATTR(easy_wakeup_gesture, (S_IRUGO | S_IWUSR|S_IWGRP),
            synaptics_easy_wakeup_gesture_show,
            synaptics_easy_wakeup_gesture_store),
    __ATTR(easy_wakeup_supported_gestures, S_IRUGO,
            synaptics_easy_wakeup_supported_gestures_show,
            synaptics_rmi4_store_error),
};

/* get tp dsm client handle */
#ifdef CONFIG_HUAWEI_DSM
struct dsm_client* tp_dsm_get_client(void)
{
    return tp_dclient;
}

/* TP basic infomation:  power, irq, reset, i2c gpio error */
ssize_t synaptics_dsm_record_basic_err_info( void )
{

    ssize_t size = 0;
    ssize_t total_size = 0;
    char str_tmp[DSM_BASIC_INFO_LEN] = {0};

    tp_log_debug("%s: entry!\n", __func__);

    /* power status,include mode, is enable, voltage */
    hw_tp_power_get_info(str_tmp,sizeof(str_tmp));
    tp_log_debug("%s: record power!\n", __func__);
    size = dsm_client_record(tp_dclient,str_tmp);
    total_size += size;
    
    /* irq reset gpio status */
    tp_log_debug("%s: record irq and reset gpio!\n", __func__);
    size =dsm_client_record(tp_dclient, 
                "[irq gpio] num:%d, irq gpio status:%d\n"
                "[reset gpio] num:%d, reset gpio status:%d\n",
                tp_data->irq_gpio, gpio_get_value(tp_data->irq_gpio),
                tp_data->reset_gpio, gpio_get_value(tp_data->reset_gpio) );
    total_size += size;
    tp_log_err("[irq gpio] num:%d, irq gpio status:%d\n"
                "[reset gpio] num:%d, reset gpio status:%d\n",
                tp_data->irq_gpio, gpio_get_value(tp_data->irq_gpio),
                tp_data->reset_gpio, gpio_get_value(tp_data->reset_gpio) );
    return total_size;

}

ssize_t synaptics_dsm_record_i2c_err_info( int err_numb )
{

    ssize_t size = 0;
    ssize_t total_size = 0;

    tp_log_err("%s: entry!\n", __func__);
    
    /* err number */
    size =dsm_client_record(tp_dclient, "i2c err number:%d\n", err_numb );
    total_size += size;

    return total_size;

}

int synp_tp_report_dsm_err( int type, int err_numb )
{

    struct dsm_client *tp_dclient = tp_dsm_get_client();

    tp_log_err("%s: entry! type:%d\n", __func__, type);

    if( NULL == tp_dclient )
    {
        tp_log_err("%s: there is not tp_dclient!\n", __func__);
        return -1;
    }

    /* try to get permission to use the buffer */
    if(dsm_client_ocuppy(tp_dclient))
    {
        tp_log_err("%s: buffer is busy!\n", __func__);
        return -1;
    }

    /* tp report err according to err type */
    switch(type)
    {
        case DSM_TP_I2C_RW_ERROR_NO:
            synaptics_dsm_record_basic_err_info();
            synaptics_dsm_record_i2c_err_info(err_numb);
            break;
        case DSM_TP_FW_ERROR_NO:
            synaptics_dsm_record_basic_err_info();
            synaptics_dsm_record_fw_err_info(err_numb);
            break;
        case DSM_TP_ESD_ERROR_NO:
            synaptics_dsm_record_basic_err_info();
            synaptics_dsm_record_esd_err_info(err_numb);
            break;
        case DSM_TP_F34_PDT_ERROR_NO:
            synaptics_dsm_record_basic_err_info();
            synaptics_dsm_f34_pdt_err_info(err_numb);
            break;
        case DSM_TP_F54_PDT_ERROR_NO:
            synaptics_dsm_record_basic_err_info();
            synaptics_dsm_f54_pdt_err_info(err_numb);
            break;
        case DSM_TP_PDT_PROPS_ERROR_NO:
            synaptics_dsm_record_basic_err_info();
            synaptics_dsm_fwu_init_pdt_props_err_info(err_numb);
            break;
        case DSM_TP_F34_READ_QUERIES_ERROR_NO:
            synaptics_dsm_record_basic_err_info();
            synaptics_dsm_f34_read_queries_err_info(err_numb);
            break;
        default:
            break;
    }

    dsm_client_notify(tp_dclient, type);

    return 0;

}
#endif/*CONFIG_HUAWEI_DSM*/

#ifdef CONFIG_HAS_EARLYSUSPEND
static ssize_t synaptics_rmi4_full_pm_cycle_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%u\n",
            rmi4_data->full_pm_cycle);
}

static ssize_t synaptics_rmi4_full_pm_cycle_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned int input;
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

    if (sscanf(buf, "%u", &input) != 1)
        return -EINVAL;

    rmi4_data->full_pm_cycle = input > 0 ? 1 : 0;

    return count;
}
#endif

static ssize_t synaptics_rmi4_f01_reset_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    int retval;
    unsigned int reset;
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

    if (sscanf(buf, "%u", &reset) != 1)
        return -EINVAL;

    if (reset != 1)
        return -EINVAL;

    retval = synaptics_rmi4_reset_device(rmi4_data);
    if (retval < 0) {
        dev_err(dev,
                "%s: Failed to issue reset command, error = %d\n",
                __func__, retval);
        return retval;
    }

    return count;
}

static ssize_t synaptics_rmi4_f01_productinfo_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "0x%02x 0x%02x\n",
            (rmi4_data->rmi4_mod_info.product_info[0]),
            (rmi4_data->rmi4_mod_info.product_info[1]));
}

static ssize_t synaptics_rmi4_f01_buildid_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%u\n",
            rmi4_data->firmware_id);
}

static ssize_t synaptics_rmi4_f01_flashprog_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int retval;
    struct synaptics_rmi4_f01_device_status device_status;
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
    /*for ESD test*/
    unsigned char data[MAX_INTR_REGISTERS + 1];
    unsigned char device_data = 0;
    unsigned char no_sleep_setting = rmi4_data->no_sleep_setting;
    unsigned char glove_mode = 0;

    /*Read doze wakeup threshold param for debug*/
    unsigned char doze_wakeup_threshold = 0;

    unsigned char glove_edge_state = 0;
    unsigned char grip_algorithm_mode = 0;

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f11_data_base_addr+F11_2D_DATA28_OFFSET,
            &device_data,
            sizeof(device_data));
    if (retval < 0) {
        tp_log_err("%s: Failed to read reg(0x%04x),retval=%d\n",
                __func__,rmi4_data->f11_data_base_addr+F11_2D_DATA28_OFFSET,retval);
        return retval;
    }

    tp_log_debug("%s:palm device_data = %d",__func__, device_data);

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f01_data_base_addr,
            data,
            rmi4_data->num_of_intr_regs + 1);
    tp_log_debug("%s: f01_data[0] = %d ,retval = %d \n",__func__,data[0],retval);
    tp_log_debug("%s: f01_data[1] = %d ,retval = %d \n",__func__,data[1],retval);
    if (retval < 0) {
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: Failed to read interrupt status\n",
                __func__);
        return retval;
    }

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f01_data_base_addr,
            device_status.data,
            sizeof(device_status.data));
    if (retval < 0) {
        dev_err(dev,
                "%s: Failed to read device status, error = %d\n",
                __func__, retval);
        return retval;
    }

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f51_ctrl_base_addr,
            &glove_mode,
            sizeof(glove_mode));
    if (retval < 0) {
        dev_err(&(rmi4_data->input_dev->dev),
                "%s: Failed to read glove mode\n",
                __func__);
        return retval;
    }
    tp_log_debug("%s: glove_mode=%d", __func__, glove_mode);

    /*Read doze wakeup threshold param for debug*/
    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f01_ctrl_base_addr + F01_RMI_CTRL03_OFFSET,
            &doze_wakeup_threshold,
            sizeof(doze_wakeup_threshold));
    tp_log_debug("%s: doze_wakeup_threshold = %d\n",__func__,doze_wakeup_threshold);
    if (retval < 0) {
        dev_err(&(rmi4_data->input_dev->dev),
                "%s: Failed to set configured\n",
                __func__);
        return retval;
    }

    /*To read the register of the glove edge switch, 1: disable, 0: enable*/
    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f51_ctrl_base_addr + F51_CTRL_EDGE_GLOVE_OFFSET,
            &glove_edge_state,
            sizeof(glove_edge_state));
    if (retval < 0) {
        dev_err(&(rmi4_data->input_dev->dev),
                "%s: Failed to read glove edge state\n",
                __func__);
        return retval;
    }

    /*To read the register of the grip algorithm mode , 0: disable, 1: enable*/
    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f51_ctrl_base_addr + F51_CTRL_GRIP_ALGORITHM_MODE_OFFSET,
            &grip_algorithm_mode,
            sizeof(grip_algorithm_mode));
    if (retval < 0) {
        dev_err(&(rmi4_data->input_dev->dev),
                "%s: Failed to read grip algorithm state for synaptics\n",
                __func__);
        return retval;
    }
    grip_algorithm_mode = grip_algorithm_mode & MASK_1BIT;

    return snprintf(buf, PAGE_SIZE, "flash_prog=%u\n"
        "f01_data[0]=%d \n"
        "f01_data[1]=%d\n"
        "palm_data=%d\n"
        "no_sleep_setting=%d\n"
        "glove_mode=%d\n"
        "doze_wakeup_threshold=%d\n"
        "glove_edge_state=%d\n"
        "grip_algorithm_mode=%d\n",
        device_status.flash_prog,
        data[0],data[1],device_data,no_sleep_setting,glove_mode,doze_wakeup_threshold,
        glove_edge_state, grip_algorithm_mode);
}

static ssize_t synaptics_rmi4_0dbutton_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%u\n",
            rmi4_data->button_0d_enabled);
}

static ssize_t synaptics_rmi4_0dbutton_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    int retval;
    unsigned int input;
    unsigned char ii;
    unsigned char intr_enable;
    struct synaptics_rmi4_fn *fhandler;
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
    struct synaptics_rmi4_device_info *rmi;

    rmi = &(rmi4_data->rmi4_mod_info);

    if (sscanf(buf, "%u", &input) != 1)
        return -EINVAL;

    input = input > 0 ? 1 : 0;

    if (rmi4_data->button_0d_enabled == input)
        return count;

    if (list_empty(&rmi->support_fn_list))
        return -ENODEV;

    list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
        if (fhandler->fn_number == SYNAPTICS_RMI4_F1A) {
            ii = fhandler->intr_reg_num;

            retval = synaptics_rmi4_i2c_read(rmi4_data,
                    rmi4_data->f01_ctrl_base_addr + 1 + ii,
                    &intr_enable,
                    sizeof(intr_enable));
            if (retval < 0)
                return retval;

            if (input == 1)
                intr_enable |= fhandler->intr_mask;
            else
                intr_enable &= ~fhandler->intr_mask;

            retval = synaptics_rmi4_i2c_write(rmi4_data,
                    rmi4_data->f01_ctrl_base_addr + 1 + ii,
                    &intr_enable,
                    sizeof(intr_enable));
            if (retval < 0)
                return retval;
        }
    }

    rmi4_data->button_0d_enabled = input;

    return count;
}

static ssize_t synaptics_rmi4_suspend_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned int input;

    if (sscanf(buf, "%u", &input) != 1)
        return -EINVAL;

    if (input == 1)
        synaptics_rmi4_suspend(dev);
    else if (input == 0)
        synaptics_rmi4_resume(dev);
    else
        return -EINVAL;

    return count;
}

static ssize_t synaptics_easy_wakeup_supported_gestures_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
    ssize_t ret;

    mutex_lock(&rmi4_data->rmi4_sysfs_mutex);
    
    tp_log_debug("%s: easy_wakeup_supported_gestures=0x%04x \n", __func__, rmi4_data->board->easy_wakeup_supported_gestures);
    ret = snprintf(buf, PAGE_SIZE, "0x%04x\n",rmi4_data->board->easy_wakeup_supported_gestures);
    
    mutex_unlock(&rmi4_data->rmi4_sysfs_mutex);
    return ret;
}

/* Set F12 palm mode */
static int f12_set_palm_mode(struct synaptics_rmi4_data *rmi4_data,bool mode)
{
    int ret;
    static unsigned char ctrl22_palm_default = 0;
    unsigned char device_data = 0;
    struct synaptics_dsx_platform_data *platform_data =
            rmi4_data->board;
    unsigned int ic_type = platform_data->ic_type;

    if (SYNAPTICS_S3320 == ic_type)
    {
        ret = synaptics_rmi4_i2c_read(rmi4_data,
                rmi4_data->f12_ctrl_base_addr+rmi4_data->f12_2d_ctrl22_palm,
                &device_data,
                sizeof(device_data));
        if (ret < 0) {
            tp_log_err("%s:read f12_2d_ctrl22_palm fail !\n",__func__);
            return ret;
        }
        tp_log_debug("%s:ctrl22_palm_swich ,reg = %d ,data = %d\n",__func__, 
            rmi4_data->f12_ctrl_base_addr+rmi4_data->f12_2d_ctrl22_palm,
            device_data);
        if (0 == ctrl22_palm_default)
        {
            ctrl22_palm_default = device_data;//record default value
        }

        if(true == mode)
        {
            device_data = (device_data & 0);//enable palm,set 0x00
        } else {
            device_data = (device_data & 0);
            device_data = (device_data | ctrl22_palm_default);//disable palm,set default
        }
        tp_log_debug("%s:write ctrl22_palm_swich,data = %d\n",
            __func__,device_data);

        ret = synaptics_rmi4_i2c_write(rmi4_data,
                rmi4_data->f12_ctrl_base_addr+rmi4_data->f12_2d_ctrl22_palm,
                &device_data,
                sizeof(device_data));
        if (ret < 0) {
            tp_log_err("%s:read f12_2d_ctrl22_palm fail !\n",__func__);
            return ret;
        }

        ret = synaptics_rmi4_i2c_read(rmi4_data,
                rmi4_data->f12_ctrl_base_addr+rmi4_data->f12_2d_ctrl23_palm,
                &device_data,
                sizeof(device_data));
        if (ret < 0) {
            tp_log_err("%s:read f12_2d_ctrl23_palm fail !\n",__func__);
            return ret;
        }
        tp_log_debug("%s:read ctrl23_palm_swich ,reg = %d ,data = %d\n",__func__, 
            rmi4_data->f12_ctrl_base_addr+rmi4_data->f12_2d_ctrl23_palm,
            device_data);

        if(true == mode)
        {
            device_data = (device_data | MASK_BIT3);//enable palm int
        } else {
            device_data = (device_data & ~MASK_BIT3);//disable palm int

        }
        tp_log_debug("%s:write ctrl23_palm_swich,data = %d\n",
            __func__,device_data);

        ret = synaptics_rmi4_i2c_write(rmi4_data,
                rmi4_data->f12_ctrl_base_addr+rmi4_data->f12_2d_ctrl23_palm,
                &device_data,
                sizeof(device_data));
        if (ret < 0) {
            tp_log_err("%s:read f12_2d_ctrl23_palm fail !\n",__func__);
            return ret;
        }
    }

    return 0;
}

/* add to the information of gesture count */
static ssize_t synaptics_easy_wakeup_gesture_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
    ssize_t ret;
    int i;

    mutex_lock(&rmi4_data->rmi4_sysfs_mutex);

    tp_log_debug("%s: easy_wakeup_gesture=0x%04x \n", __func__, rmi4_data->easy_wakeup_gesture);
    ret = snprintf(buf, PAGE_SIZE, "0x%04x\n",rmi4_data->easy_wakeup_gesture);
    for(i = 0;i < GESTURE_MAX ;i++)
    {
        ret += snprintf(&buf[ret], PAGE_SIZE, "%s %d : %d\n","gesture",i,gesture_count[i]);
    }
    mutex_unlock(&rmi4_data->rmi4_sysfs_mutex);
    return ret;
}

static ssize_t synaptics_easy_wakeup_gesture_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
    unsigned int value = 0;
    unsigned int gestrue_value = 0;
    int ret;

    ret = kstrtouint(buf, 10, &value);
    if (ret < 0){
        tp_log_err("%s: kstrtoul error,ret=%d \n", __func__,ret);
        return ret;
    }

    tp_log_info("%s: value=0x%04x \n", __func__, value);
    if (value > 0xFFFF) {
        tp_log_err("%s: Invalid value: %d \n", __func__, value);
        return -EINVAL;
    }

    tp_log_info("%s: gesture_enabled_0=%d \n", __func__, rmi4_data->gesture_enabled);

    synaptics_pm_rt_get(rmi4_data);

    mutex_lock(&rmi4_data->rmi4_sysfs_mutex);
    gestrue_value = value & rmi4_data->board->easy_wakeup_supported_gestures;
    tp_log_info("%s: gesture_value=0x%04x \n", __func__, gestrue_value);
    if(gestrue_value & APP_ENABLE_WAKEUP_GESTURE) {
        rmi4_data->gesture_enabled = RMI4_EASY_WAKE_UP_GESTURE_ENABLED;
    } else {
        rmi4_data->gesture_enabled = RMI4_EASY_WAKE_UP_GESTURE_DISABLED;
    }
    if(gestrue_value & APP_ENABLE_GESTURE(GESTURE_PALM_COVERED)) {
        rmi4_data->palm_enabled = RMI4_PALM_SLEEP_GESTURE_ENABLED;
    } else {
        rmi4_data->palm_enabled =  RMI4_PALM_SLEEP_GESTURE_DISABLED;
    }
    tp_log_info("%s#%d: gesture_enabled=%d,palm_enabled=%d\n",
        __func__, __LINE__, 
        rmi4_data->gesture_enabled,rmi4_data->palm_enabled);

    ret = f12_set_palm_mode(rmi4_data,rmi4_data->palm_enabled);
    if (ret < 0){
        tp_log_err("%s: f12_set_palm_mode fail! \n", __func__);
    }

    rmi4_data->easy_wakeup_gesture = gestrue_value;
    tp_log_info("%s: easy_wakeup_gesture=0x%04x \n", __func__, rmi4_data->easy_wakeup_gesture);

    mutex_unlock(&rmi4_data->rmi4_sysfs_mutex);

    synaptics_pm_rt_put(rmi4_data);

    if (ret < 0)
        return ret;

    return size;
}

/*
 *For apk to read gesture locus point position:
 *Each point was represent by 8 hex numbers
 *with the 4 MSB hex number for position_x and
 *the 4 LSB hex number for position_y
 *There is no space between point positions
 */
static ssize_t synaptics_easy_wakeup_position_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
    ssize_t ret;
    char temp[9] = {0};
    int i = 0;
    
    mutex_lock(&rmi4_data->rmi4_sysfs_mutex);

    for(i = 0;i < MAX_LOTUS_NUM;i ++)
    {
        tp_log_info("%s: easywake_position[%d] = %d:\n",
                        __func__, i,easywake_position[i]);
        ret = snprintf(temp, (sizeof(u32)*2+1), "%08x",easywake_position[i]);
        strncat(buf, temp ,(sizeof(u32)*2+1));
    }
    strncat(buf,"\n\0",2);

    mutex_unlock(&rmi4_data->rmi4_sysfs_mutex);

    ret = (strlen(buf)+1);
    return ret;
}

static ssize_t synaptics_easy_wakeup_position_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    return 0;
}

static ssize_t hw_synaptics_easy_wakeup_position_show(struct kobject *dev,
        struct kobj_attribute *attr, char *buf)
{
    struct device *cdev = core_dev_ptr;
    if (!cdev){
        tp_log_err("%s: device is null \n", __func__);
        return -EINVAL;
    }
    return synaptics_easy_wakeup_position_show(cdev, NULL, buf);
}

static ssize_t hw_synaptics_easy_wakeup_position_store(struct kobject *dev,
        struct kobj_attribute *attr, const char *buf, size_t size)
{
    struct device *cdev = core_dev_ptr;
    if (!cdev){
        tp_log_err("%s: device is null \n", __func__);
        return -EINVAL;
    }
    return synaptics_easy_wakeup_position_store(cdev, NULL, buf, size);
}


static struct kobj_attribute easy_wakeup_position = {
    .attr = {.name = "easy_wakeup_position", .mode = (S_IRUGO | S_IWUSR | S_IWGRP)},
    .show = hw_synaptics_easy_wakeup_position_show,
    .store = hw_synaptics_easy_wakeup_position_store,
};

static ssize_t hw_synaptics_easy_wakeup_gesture_show(struct kobject *dev,
        struct kobj_attribute *attr, char *buf)
{
    struct device *cdev = core_dev_ptr;
    if (!cdev){
        tp_log_err("%s: device is null \n", __func__);
        return -EINVAL;
    }
    return synaptics_easy_wakeup_gesture_show(cdev, NULL, buf);
}

static ssize_t hw_synaptics_easy_wakeup_gesture_store(struct kobject *dev,
        struct kobj_attribute *attr, const char *buf, size_t size)
{
    struct device *cdev = core_dev_ptr;
    if (!cdev){
        tp_log_err("%s: device is null \n", __func__);
        return -EINVAL;
    }
    return synaptics_easy_wakeup_gesture_store(cdev, NULL, buf, size);
}

static ssize_t hw_synaptics_easy_wakeup_supported_gestures_show(struct kobject *dev,
        struct kobj_attribute *attr, char *buf)
{
    struct device *cdev = core_dev_ptr;
    if (!cdev){
        tp_log_err("%s: device is null \n", __func__);
        return -EINVAL;
    }
    return synaptics_easy_wakeup_supported_gestures_show(cdev, NULL, buf);
}

static ssize_t synaptics_glove_func_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

    unsigned char device_ctrl;
    ssize_t ret;

    synaptics_pm_rt_get(rmi4_data);
    ret = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f12_ctrl_base_addr+rmi4_data->f12_2d_ctrl26_glove,
            &device_ctrl,
            sizeof(device_ctrl));
    if (ret < 0) {
        dev_err(&(rmi4_data->input_dev->dev),
                "%s: Failed to read glove mode\n",
                __func__);
    } else{
        tp_log_info("%s: glove mode=%d \n", __func__, device_ctrl);
    }
    synaptics_pm_rt_put(rmi4_data);

    mutex_lock(&rmi4_data->rmi4_sysfs_mutex);

    tp_log_debug("%s: glove_enabled=%d \n", __func__, rmi4_data->glove_enabled);
    ret = snprintf(buf, PAGE_SIZE, "%d\n",rmi4_data->glove_enabled);
    mutex_unlock(&rmi4_data->rmi4_sysfs_mutex);
    return ret;
}

static int synaptics_force_cal(struct synaptics_rmi4_data *rmi4_data);
static int F12_set_glove_mode(struct synaptics_rmi4_data *rmi4_data,unsigned long value)
{
    int ret;
    unsigned char mode = (bool)!!value;

    mutex_lock(&rmi4_data->rmi4_glove_mutex);
    ret = synaptics_rmi4_i2c_write(rmi4_data,
            rmi4_data->f12_ctrl_base_addr+rmi4_data->f12_2d_ctrl26_glove,
            &mode,
            sizeof(mode));
    mutex_unlock(&rmi4_data->rmi4_glove_mutex);
    if (ret < 0) {
        tp_log_err("%s: Failed, glove mode=%d!\n",
                __func__,mode);
        return ret;
    } else{
        tp_log_warning("%s: Success, glove in %d mode!\n",
                __func__,mode);
        rmi4_data->glove_enabled = !!value;
    }

    return 0;
}

/* For S3207,use F51 reg to control glove mode */
static int F51_set_glove_reg(struct synaptics_rmi4_data *rmi4_data,enum syanptics_glove_mode mode)
{
    int ret;
    unsigned char data = mode;

    /*Set glove mode*/
    mutex_lock(&rmi4_data->rmi4_glove_mutex);
    ret = synaptics_rmi4_i2c_write(rmi4_data,
            rmi4_data->f51_ctrl_base_addr,
            &data,
            sizeof(data));
    if (ret < 0) {
        tp_log_err("%s: Failed to set glove mode=%d!\n",
                __func__,data);
        mutex_unlock(&rmi4_data->rmi4_glove_mutex);
        return ret;
    } else{
        tp_log_warning("%s: glove in %d mode!\n",
                __func__,data);
    }
    mutex_unlock(&rmi4_data->rmi4_glove_mutex);

    return 0;
}

/* For S3207,use F51 to control glove mode */
static int F51_set_glove_mode(struct synaptics_rmi4_data *rmi4_data,unsigned long value)
{
    int ret = 0;

    if (value > 0)
    {
        /* In case that glove has already opened by ui before holster */
        /* covered, we should change glove mode from mode 0 to mode 3 */
        if ((true == rmi4_data->holster_enabled)&&
                (true == rmi4_data->glove_enabled))
        {
            /*TP will only response to glove in holster mode*/
            tp_log_debug("%s: line=%d\n",__func__,__LINE__);
            ret = F51_set_glove_reg(rmi4_data,SYSTEM_LOCKED_TO_GLOVE_MODE);
        }
        /* In case that glove has not been opened by ui before holster */
        /* covered, we do not need to do anyhing here  */
        else if((true == rmi4_data->holster_enabled)&&
                    (false == rmi4_data->glove_enabled))
        {
            /*Do nothing, holster will finish the rest later*/
            tp_log_warning("%s: line=%d\n",__func__,__LINE__);
        }
        /* In other case, we just need to set glove mode 0 */
        /* and record ui state by driver flag glove_enabled */
        else
        {
            /*TP will response to finger and glove,with finger priority*/
            tp_log_debug("%s: line=%d\n",__func__,__LINE__);
            ret = F51_set_glove_reg(rmi4_data,SYSTEM_START_IN_SKIN_MODE);
            rmi4_data->glove_enabled = RMI4_GLOV_FUNC_ENABLED;
        }
    }
    else
    {
        /* In case that holster has been covered, */
        /* we do not need to do anyhing here */
        if (true == rmi4_data->holster_enabled)
        {
            /*Do nothing, holster will finish the rest later*/
            /* holster was opened, do one force cal , as early as possible */
            ret = synaptics_force_cal(rmi4_data);
            if (ret < 0) {
                tp_log_err("%s: Failed to force calibration!ret=%d\n",
                        __func__,ret);
            }
            tp_log_warning("%s: line=%d\n",__func__,__LINE__);
        }
        /* In other case, we just need to set glove mode 2 */
        /* and record ui state by driver flag glove_enabled */
        else
        {
            /*TP will only response to finger*/
            tp_log_debug("%s: line=%d\n",__func__,__LINE__);
            ret = F51_set_glove_reg(rmi4_data,SYSTEM_LOCKED_TO_SKIN_MODE);
            rmi4_data->glove_enabled = RMI4_GLOV_FUNC_DISABLED;
        }
    }

    return ret;
}

static int set_edge_glove_state(struct synaptics_rmi4_data *rmi4_data, int edge_glove_state)
{
    int ret;
    unsigned char data = edge_glove_state;

    /*Set glove edge state*/
    ret = synaptics_rmi4_i2c_write(rmi4_data,
            rmi4_data->f51_ctrl_base_addr + F51_CTRL_EDGE_GLOVE_OFFSET,
            &data,
            sizeof(data));
    if (ret < 0) {
        tp_log_err("%s: Failed to set glove edge state=%d!\n",
                __func__,data);
        return ret;
    } else{
        tp_log_debug("%s: glove edge state is %d!\n",
                __func__,data);
    }

    return 0;
}

/*In EmUi3.0, holster switch will also change glove state, so we also */
/* need to record this state to make sure driver sync with ui state */
static int holster_enable_glove = false;

static ssize_t synaptics_glove_func_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
    struct synaptics_dsx_platform_data *platform_data = rmi4_data->board;
    unsigned int ic_type = platform_data->ic_type;
    unsigned long value;
    int ret;

    ret = kstrtoul(buf, 10, &value);
    /* Normalized input value */
    tp_log_debug("%s: value=%d", __func__, (unsigned int)value);
    if (ret < 0){
        tp_log_err("%s: kstrtoul error,ret=%d", __func__,ret);
        return ret;
    }

    tp_log_debug("%s: glove_enabled=%d", __func__, rmi4_data->glove_enabled);

    synaptics_pm_rt_get(rmi4_data);

    if (SYNAPTICS_S3320 == ic_type)
    {
        /* Set f12 glove mode:1-open,0-close */
        ret = F12_set_glove_mode(rmi4_data,value);
        
    }
    else
    {
        /* Set f51 glove mode */
        ret = F51_set_glove_mode(rmi4_data,value);
    }

    tp_log_debug("%s: glove_enabled=%d", __func__, rmi4_data->glove_enabled);


    synaptics_pm_rt_put(rmi4_data);

    if (ret < 0)
        return ret;

    return size;
}

static ssize_t hw_glove_func_show(struct kobject *dev,
        struct kobj_attribute *attr, char *buf)
{
    struct device *cdev = core_dev_ptr;
    if (!cdev){
        tp_log_err("%s: device is null", __func__);
        return -EINVAL;
    }
    return synaptics_glove_func_show(cdev, NULL, buf);
}

static ssize_t hw_glove_func_store(struct kobject *dev,
        struct kobj_attribute *attr, const char *buf, size_t size)
{
    struct device *cdev = core_dev_ptr;
    if (!cdev){
        tp_log_err("%s: device is null", __func__);
        return -EINVAL;
    }
    return synaptics_glove_func_store(cdev, NULL, buf, size);
}

/*touch window get point location*/
static struct holster_mode synap_holster_info = {0,0,0,0,0};
static struct holster_mode double_tap_info = {0,0,0,0,0};

/**
 * To set the grip algorithm mode by the last bit of f51_custom_ctrl80
 * 1 : enble
 * 0 : disble
 */
static int set_grip_algorithm_mode(struct synaptics_rmi4_data *rmi4_data, unsigned char mode)
{
    unsigned char device_ctrl;
    unsigned char grip_algorithm_mode = mode;
    int ret = 0;

    /*Read F51_CUSTOM_CTRL80 to get grip algorithm mode*/
    ret = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f51_ctrl_base_addr + F51_CTRL_GRIP_ALGORITHM_MODE_OFFSET,
            &device_ctrl,
            sizeof(device_ctrl));
    if (ret < 0) {
        tp_log_err("%s: Failed to read grip algorithm mode\n",
                __func__);
        return ret;
    }
    tp_log_debug("%s: device_ctrl = %d!\n", __func__, device_ctrl);
    device_ctrl = (device_ctrl & (~MASK_1BIT));
    device_ctrl = (device_ctrl | grip_algorithm_mode);
    tp_log_debug("%s: device_ctrl = %d!\n", __func__, device_ctrl);

    /*Write F51_CUSTOM_CTRL80 to set grip algorithm mode*/
    ret = synaptics_rmi4_i2c_write(rmi4_data,
            rmi4_data->f51_ctrl_base_addr + F51_CTRL_GRIP_ALGORITHM_MODE_OFFSET,
            &device_ctrl,
            sizeof(device_ctrl));
    if (ret < 0){
        tp_log_err("%s:failed to set grip algorithm mode!ret = %d\n",__func__,ret);
    }else{
        tp_log_warning("%s: succeeded to set grip algorithm mode! grip_algorithm_mode = %d\n", 
            __func__, grip_algorithm_mode);
    }

    return ret;
}
/*To set the grip algorithm edge distance*/
static int set_grip_algorithm_edge_distance(struct synaptics_rmi4_data *rmi4_data)
{
    int ret = 0;
    unsigned char f51_custom_ctrl81[GRIP_ALGORITHM_BYTES_NUM] = {0};

    if(true == rmi4_data->board->grip_algorithm_enabled){
        /*enble grip algorithm*/
        ret = set_grip_algorithm_mode(rmi4_data, true);
        if(ret < 0){
            tp_log_err("%s:failed to enable grip algorithm!ret = %d\n",__func__,ret);
            return ret;
        }
    }else{
        /*disable grip algorithm, no need to set the grip algorithm edge distance*/
        ret = set_grip_algorithm_mode(rmi4_data, false);
        if(ret < 0){
            tp_log_err("%s:failed to disable grip algorithm!ret = %d\n",__func__,ret);
        }
        return ret;
    }
    /**
    * TP X coordinate pixel size  :  1100, Physical Dimensions : 68mm
    * left edge distance is 3mm for better performance, right edge distance is 3mm too.
    * According to the proportion:3mm is 50 TP pixels for left, 1050 TP pixels for right.
    */
    f51_custom_ctrl81[0] = rmi4_data->board->grip_left_lsb;
    f51_custom_ctrl81[1] = rmi4_data->board->grip_left_msb;
    f51_custom_ctrl81[2] = rmi4_data->board->grip_right_lsb;
    f51_custom_ctrl81[3] = rmi4_data->board->grip_right_msb;

    /* Write grip zone of left and right edge to f51_custom_ctrl81 (81~84)*/
    ret = synaptics_rmi4_i2c_write(rmi4_data,
            rmi4_data->f51_ctrl_base_addr + F51_CTRL_GRIP_ALGORITHM_DISTANCE_OFFSET,
            &f51_custom_ctrl81[0],
            sizeof(f51_custom_ctrl81));
    if (ret < 0){
        tp_log_err("%s:failed to set grip algorithm edge distance!ret = %d\n",__func__,ret);
    }

    return ret;
}

/*
 * Use to define double-tap gesture zone that read from dtsi
*/
static int huawei_init_double_tap_zone(struct synaptics_rmi4_data *rmi4_data)
{
    int x0 = rmi4_data->board->dtz_x0;
    int y0 = rmi4_data->board->dtz_y0;
    int x1 = rmi4_data->board->dtz_x1;
    int y1 = rmi4_data->board->dtz_y1;

    if ((x0 == 0) || (y0 == 0) || (x1 == 0) || (y1 == 0))
    {
        tp_log_err("%s:double_tap_zone invalid!\n",__func__);
        return -1;
    }

    /* init double_tap window info*/
    double_tap_info.top_left_x0 = x0;
    double_tap_info.top_left_y0 = y0;
    double_tap_info.bottom_right_x1 = x1;
    double_tap_info.bottom_right_y1 = y1;

    tp_log_debug("%s: double_tap_zone is initialized!\n", __func__);
    return 0;
}

/*
    this function will set the glove/holster's sensitivity.
    when holster was covered , low sensitivity will be set.
    shen holster was opned and glove was enabled , default sensitivity will be set.
    the default sensitivity value was read when TP probe.
    the low sensitivity value was set in this function.
*/
static int synaptics_set_holster_low_sensitivity(struct synaptics_rmi4_data *rmi4_data, unsigned char enabled)
{
    int ret;
    if (0 == enabled) { // set the default sensitivity value
        tp_log_debug("%s: write reg value, begin(%d)\n", __func__, enabled);
        ret = synaptics_rmi4_i2c_write(rmi4_data,
                PDT_P4_F51_CUSTOM_CTRL_BASE+F51_CTRL_01_OFFSET,
                &rmi4_data->f51_ctrl_01_val,
                sizeof(rmi4_data->f51_ctrl_01_val));
        if (ret < 0) {
            tp_log_err("%s: Failed to write F51_CTRL_01 register!\n",
                    __func__);
            return ret;
        }
        tp_log_debug("%s: write reg value, f51_ctrl_01_val:0x%x \n", __func__, rmi4_data->f51_ctrl_01_val);
        ret = synaptics_rmi4_i2c_write(rmi4_data,
                PDT_P4_F51_CUSTOM_CTRL_BASE+F51_CTRL_04_OFFSET,
                &rmi4_data->f51_ctrl_04_val,
                sizeof(rmi4_data->f51_ctrl_04_val));
        if (ret < 0) {
            tp_log_err("%s: Failed to write F51_CTRL_04 register!\n",
                    __func__);
            return ret;
        }
        tp_log_debug("%s: write reg value, f51_ctrl_04_val:0x%x \n", __func__, rmi4_data->f51_ctrl_04_val);
        ret = synaptics_rmi4_i2c_write(rmi4_data,
                PDT_P0_F01_RMI_CTRL_BASE+F01_CTRL_05_OFFSET,
                &rmi4_data->f01_ctrl_05_val,
                sizeof(rmi4_data->f01_ctrl_05_val));
        if (ret < 0) {
            tp_log_err("%s: Failed to write F01_CTRL_05 register!\n",
                    __func__);
            return ret;
        }
        tp_log_debug("%s: write reg value, f01_ctrl_05_val:0x%x \n", __func__, rmi4_data->f01_ctrl_05_val);
        ret = synaptics_rmi4_i2c_write(rmi4_data,
                PDT_P0_F01_RMI_CTRL_BASE+F01_CTRL_09_OFFSET,
                &rmi4_data->f01_ctrl_09_val,
                sizeof(rmi4_data->f01_ctrl_09_val));
        if (ret < 0) {
            tp_log_err("%s: Failed to write F01_CTRL_09 register!\n",
                    __func__);
            return ret;
        }
        tp_log_debug("%s: write reg value, f01_ctrl_09_val:0x%x \n", __func__, rmi4_data->f01_ctrl_09_val);
        tp_log_debug("%s: write reg value, end\n", __func__);

    } else { // set low sensitivity value, when holster was closed
        tp_log_debug("%s: write reg value, begin(%d)\n", __func__, enabled);
        ret = synaptics_rmi4_i2c_write(rmi4_data,
                PDT_P4_F51_CUSTOM_CTRL_BASE+F51_CTRL_01_OFFSET,
                &rmi4_data->f51_ctrl_01_low_sensitivity_val,
                sizeof(rmi4_data->f51_ctrl_01_low_sensitivity_val));
        if (ret < 0) {
            tp_log_err("%s: Failed to write F51_CTRL_01 register!\n",
                    __func__);
            return ret;
        }
        tp_log_debug("%s: write reg value, f51_ctrl_01_val:0x%x \n", __func__, rmi4_data->f51_ctrl_01_low_sensitivity_val);
        ret = synaptics_rmi4_i2c_write(rmi4_data,
                PDT_P4_F51_CUSTOM_CTRL_BASE+F51_CTRL_04_OFFSET,
                &rmi4_data->f51_ctrl_04_low_sensitivity_val,
                sizeof(rmi4_data->f51_ctrl_04_low_sensitivity_val));
        if (ret < 0) {
            tp_log_err("%s: Failed to write F51_CTRL_04 register!\n",
                    __func__);
            return ret;
        }
        tp_log_debug("%s: write reg value, f51_ctrl_04_val:0x%x \n", __func__, rmi4_data->f51_ctrl_04_low_sensitivity_val);
        ret = synaptics_rmi4_i2c_write(rmi4_data,
                PDT_P0_F01_RMI_CTRL_BASE+F01_CTRL_05_OFFSET,
                &rmi4_data->f01_ctrl_05_low_sensitivity_val,
                sizeof(rmi4_data->f01_ctrl_05_low_sensitivity_val));
        if (ret < 0) {
            tp_log_err("%s: Failed to write F01_CTRL_05 register!\n",
                    __func__);
            return ret;
        }
        tp_log_debug("%s: write reg value, f01_ctrl_05_val:0x%x \n", __func__, rmi4_data->f01_ctrl_05_low_sensitivity_val);
        ret = synaptics_rmi4_i2c_write(rmi4_data,
                PDT_P0_F01_RMI_CTRL_BASE+F01_CTRL_09_OFFSET,
                &rmi4_data->f01_ctrl_09_low_sensitivity_val,
                sizeof(rmi4_data->f01_ctrl_09_low_sensitivity_val));
        if (ret < 0) {
            tp_log_err("%s: Failed to write F01_CTRL_09 register!\n",
                    __func__);
            return ret;
        }
        tp_log_debug("%s: write reg value, f01_ctrl_09_val:0x%x \n", __func__, rmi4_data->f01_ctrl_09_low_sensitivity_val);
        tp_log_debug("%s: write reg value, end\n", __func__);
    }
    return ret;
}

/*Enable force calibration*/
static int synaptics_force_cal(struct synaptics_rmi4_data *rmi4_data)
{
    int ret = 0;
    unsigned char value = 0xFF;
    unsigned char command = 0;
    unsigned char timeout_count;

    ret = synaptics_rmi4_i2c_read(rmi4_data,
        rmi4_data->f54_cmd_base_addr,
        &command,
        sizeof(command));
    if (ret < 0) {
        tp_log_err("%s: Failed to read force cal register!\n",
                __func__);
        return ret;
    }

    /*Setting this bit to '1' requests that a new baseline be taken*/
    command |= F54_FORCE_CAL;

    ret = synaptics_rmi4_i2c_write(rmi4_data,
        rmi4_data->f54_cmd_base_addr,
        &command,
        sizeof(command));
    if (ret < 0) {
        tp_log_err("%s: Failed to write force cal command!\n",
                __func__);
        return ret;
    }

    timeout_count = 0;
    do {
        ret = synaptics_rmi4_i2c_read(rmi4_data,
                rmi4_data->f54_cmd_base_addr,
                &value,
                sizeof(value));
        if (ret < 0) {
            tp_log_err("%s: Failed to read force cal register!\n",
                    __func__);
            return ret;
        }

        /*When the new baseline has been taken , this*/
        /*command bit will automatically clear to '0'*/
        if (value == 0x00)
            break;

        msleep(10);//wait force cal finished
        timeout_count++;
    } while (timeout_count < FORCE_TIMEOUT_10MS);

    if (timeout_count == FORCE_TIMEOUT_10MS) {
        tp_log_err("%s: Timed out waiting for force cal\n",
                __func__);
        return -ETIMEDOUT;
    }

    return 0;
}

/*
 * This window zone is defined in f51_ctrl reg,from F51_CUSTOM_CTRL19 to 26;
 * The 8 bytes is:<x0_LSB><x0_MSB><y0_LSB><y0_MSB><x1_LSB><x1_MSB><y1_LSB><y1_MSB>;
 * The 8 bytes is used to limit double-tap zone and holster window zone,so that we should change it
 * when mode is switch between double-tap gesture mode and holster mode.
 */
static int s3320_set_effective_window(struct synaptics_rmi4_data *rmi4_data,struct holster_mode *effective_window_info)
{
    int ret = 0;
    unsigned char f11_2d_ctrl92[DOUBLE_TAP_ZONE_BYTES + 2] = {0};//record the zone read from/write to TP register
    unsigned char effective_window[DOUBLE_TAP_ZONE_BYTES + 2] = {0};//new zone to be written
    int i = 0;
    int x0 = effective_window_info->top_left_x0;
    int y0 = effective_window_info->top_left_y0;
    int x1 = effective_window_info->bottom_right_x1;
    int y1 = effective_window_info->bottom_right_y1;
    int x_lcd = rmi4_data->board->lcd_x;
    int y_lcd_all = rmi4_data->board->lcd_all;
    int max_x = rmi4_data->sensor_max_x;
    int max_y = rmi4_data->sensor_max_y;

    tp_log_info("%s:in!(%d,%d)(%d,%d)\n",__func__,x0,y0,x1,y1);
    /* Transform LCD resolution to TP resolution*/
    effective_window[0] = (char)(x0*max_x/x_lcd);
    effective_window[1] = (char)(((u16)(x0*max_x/x_lcd) & ~MASK_8BIT)>>8);
    effective_window[2] = (char)(y0*max_y/y_lcd_all);
    effective_window[3] = (char)(((u16)(y0*max_y/y_lcd_all) & ~MASK_8BIT)>>8);
    effective_window[4] = (char)(x1*max_x/x_lcd);
    effective_window[5] = (char)(((u16)(x1*max_x/x_lcd) & ~MASK_8BIT)>>8);
    effective_window[6] = (char)(y1*max_y/y_lcd_all);
    effective_window[7] = (char)(((u16)(y1*max_y/y_lcd_all) & ~MASK_8BIT)>>8);

    tp_log_warning("%s: window reg(0x%4x)!\n", __func__,
            rmi4_data->f51_ctrl_base_addr + F51_CUSTOM_CTRL19_OFFSET);
    mutex_lock(&rmi4_data->rmi4_window_mutex);
    ret = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f51_ctrl_base_addr + F51_CUSTOM_CTRL19_OFFSET,
            &f11_2d_ctrl92[0],
            sizeof(f11_2d_ctrl92));
    if (ret < 0)
    {
        tp_log_err("%s:holster window read error!ret = %d\n",__func__,ret);
        mutex_unlock(&rmi4_data->rmi4_window_mutex);
        return ret;
    }

    for(i = 0;i < (DOUBLE_TAP_ZONE_BYTES + 2);i ++)
    {
        f11_2d_ctrl92[i] = effective_window[i];
        tp_log_debug("%s:new effective_window[%d] = %d\n",__func__,i, effective_window[i]);
    }
    

    ret = synaptics_rmi4_i2c_write(rmi4_data,
            rmi4_data->f51_ctrl_base_addr + F51_CUSTOM_CTRL19_OFFSET,
            &f11_2d_ctrl92[0],
            sizeof(f11_2d_ctrl92));
    if (ret < 0){
        tp_log_err("%s:new effective_window write error!ret = %d\n",__func__,ret);
    }else{
        tp_log_warning("%s: new effective_window is written!\n", __func__);
    }
    mutex_unlock(&rmi4_data->rmi4_window_mutex);

    return ret;
}

/*
 * This window zone is defined in f11_ctrl reg,from F11_2D_CTRL92(01)/00 to (01)/05;
 * The 6 bytes is:<x0_LSB><y0_LSB><y0_MSB+x0_MSB><x1_LSB><y1_LSB><y1_MSB+x1_MSB>;
 * The 6 bytes is used to limit double-tap zone and holster window zone,so that we should change it
 * when mode is switch between double-tap gesture mode and holster mode.
 */
static int s3207_set_effective_window(struct synaptics_rmi4_data *rmi4_data,struct holster_mode *effective_window_info)
{
    int ret = 0;
    unsigned char f11_2d_ctrl92[DOUBLE_TAP_ZONE_BYTES + 1] = {0};//record the zone read from/write to TP register
    unsigned char effective_window[DOUBLE_TAP_ZONE_BYTES + 1] = {0};//new zone to be written
    int i = 0;
    int x0 = effective_window_info->top_left_x0;
    int y0 = effective_window_info->top_left_y0;
    int x1 = effective_window_info->bottom_right_x1;
    int y1 = effective_window_info->bottom_right_y1;
    int x_lcd = rmi4_data->board->lcd_x;
    int y_lcd_all = rmi4_data->board->lcd_all;
    int max_x = rmi4_data->sensor_max_x;
    int max_y = rmi4_data->sensor_max_y;

    tp_log_info("%s:in!(%d,%d)(%d,%d)\n",__func__,x0,y0,x1,y1);
    /* Transform LCD resolution to TP resolution*/
    effective_window[1] = (char)(x0*max_x/x_lcd);
    effective_window[2] = (char)(y0*max_y/y_lcd_all);
    effective_window[3] = (char)(((u16)(y0*max_y/y_lcd_all) & ~MASK_8BIT)>>4 | 
                                ((u16)(x0*max_x/x_lcd) & ~MASK_8BIT)>>8);
    effective_window[4] = (char)(x1*max_x/x_lcd);
    effective_window[5] = (char)(y1*max_y/y_lcd_all);
    effective_window[6] = (char)(((u16)(y1*max_y/y_lcd_all) & ~MASK_8BIT)>>4 | 
                                ((u16)(x1*max_x/x_lcd) & ~MASK_8BIT)>>8);

    mutex_lock(&rmi4_data->rmi4_window_mutex);
    ret = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f11_ctrl_base_addr + F11_2D_CTRL92_OFFSET,
            &f11_2d_ctrl92[0],
            sizeof(f11_2d_ctrl92));
    if (ret < 0)
    {
        tp_log_err("%s:double_tap_zone read error!ret = %d\n",__func__,ret);
        mutex_unlock(&rmi4_data->rmi4_window_mutex);
        return ret;
    }

    for(i = 1;i < (DOUBLE_TAP_ZONE_BYTES + 1);i ++)
    {
        f11_2d_ctrl92[i] = effective_window[i];
        tp_log_debug("%s:new effective_window[%d] = %d\n",__func__,i, effective_window[i]);
    }
    

    /* Write window zone to F11_2D_CTRL92(01)/00~(01)/05*/
    ret = synaptics_rmi4_i2c_write(rmi4_data,
            rmi4_data->f11_ctrl_base_addr + F11_2D_CTRL92_OFFSET,
            &f11_2d_ctrl92[0],
            sizeof(f11_2d_ctrl92));
    if (ret < 0){
        tp_log_err("%s:new effective_window write error!ret = %d\n",__func__,ret);
    }else{
        tp_log_warning("%s: new effective_window is written!\n", __func__);
    }
    mutex_unlock(&rmi4_data->rmi4_window_mutex);

    return ret;
}

static int set_effective_window(struct synaptics_rmi4_data *rmi4_data,struct holster_mode *effective_window_info)
{
    struct synaptics_dsx_platform_data *platform_data = rmi4_data->board;
    unsigned int ic_type = platform_data->ic_type;
    int ret = 0;

    if (SYNAPTICS_S3320 == ic_type)
    {
        /* Set f12 glove mode:1-open,0-close */
        ret = s3320_set_effective_window(rmi4_data,effective_window_info);
    }
    else
    {
        /* Set f51 holster mode */
        ret = s3207_set_effective_window(rmi4_data,effective_window_info);
    }
    if (ret < 0)
    {
        tp_log_err("%s:set_effective_window read error!ret = %d\n",__func__,ret);
    }

    return ret;
}

/*
 * If holster mode is enabled,TP will only response to certain
 * holster window zone defined by set_effective_window();
 * or it will response to whole area as usual when disabled.
*/
static int set_holster_mode(struct synaptics_rmi4_data *rmi4_data,
        unsigned short addr, enum syanptics_holster_flags flag)
{
    int ret;
    unsigned char data = flag;

    tp_log_warning("%s: write(%d)!\n", __func__,data);
    /*Write holster mode reg*/
    mutex_lock(&rmi4_data->rmi4_holster_mutex);
    ret = synaptics_rmi4_i2c_write(rmi4_data,
            addr,
            &data,
            sizeof(data));
    if (ret < 0) {
        tp_log_err("%s: Failed to change holster mode\n",
                __func__);
        mutex_unlock(&rmi4_data->rmi4_holster_mutex);
        return ret;
    } 
    mutex_unlock(&rmi4_data->rmi4_holster_mutex);

    return 0;
}

static ssize_t s3320_set_holster_mode(struct synaptics_rmi4_data *rmi4_data,int enable)
{
    int ret;
    unsigned char data = enable;

    if (enable)
    {
        /* Write holster window zone to F11_2D_CTRL92(01)/00~(01)/05*/
        ret = set_effective_window(rmi4_data,&synap_holster_info);
        if (ret < 0)
        {
            tp_log_err("%s:holster_zwindow_zone write error!ret = %d\n",
                    __func__,ret);
            goto exit;
        }

        tp_log_warning("%s: Enabled reg(0x%4x)!\n", __func__,
                rmi4_data->f51_ctrl_base_addr + F51_CUSTOM_CTRL07_OFFSET);
        /*Enable glove mode by cover*/
        /*TP will only response to certain holster window zone*/
        ret = set_holster_mode(rmi4_data,
                rmi4_data->f51_ctrl_base_addr + F51_CUSTOM_CTRL07_OFFSET,
                RMI4_HOLSTER_FUNC_ENABLED);
        if (ret < 0) {
            tp_log_err("%s: Failed to enable holster mode\n",
                    __func__);
            goto exit;
        } else {
            rmi4_data->holster_enabled = true;
            tp_log_warning("%s: holster is Enabled!\n", __func__);
        }

        tp_log_warning("%s: H-glove reg(0x%4x)!write(%d)!\n", __func__,
                rmi4_data->f51_ctrl_base_addr + F51_CUSTOM_CTRL29_OFFSET,data);
        /*Force Glove In Smart Clover*/
        mutex_lock(&rmi4_data->rmi4_holster_mutex);
        ret = synaptics_rmi4_i2c_write(rmi4_data,
                rmi4_data->f51_ctrl_base_addr + F51_CUSTOM_CTRL29_OFFSET,
                &data,
                sizeof(data));
        mutex_unlock(&rmi4_data->rmi4_holster_mutex);
        if (ret < 0) {
            tp_log_err("%s: Failed to set glove mode!ret=%d\n",
                    __func__,ret);
            goto exit;
        } else {
            /* In case that glove mode has been changed by holster ui, we need */
            /* to record ui state by another driver flag holster_enable_glove */
            tp_log_warning("%s: line=%d\n",__func__,__LINE__);
//            holster_enable_glove = false;
        }
    } else {
        /*Disable holster mode*/
        ret = set_holster_mode(rmi4_data,
                rmi4_data->f51_ctrl_base_addr + F51_CUSTOM_CTRL07_OFFSET,
                RMI4_HOLSTER_FUNC_DISABLED);
        if (ret < 0) {
            tp_log_err("%s: Failed to exit holster mode\n",
                    __func__);
            goto exit;
        } else{
            rmi4_data->holster_enabled = false;
            tp_log_warning("%s: holster is Disabled!\n", __func__);
        }

        /*
         * Change glove mode:
         * If glove has been enabled before holster was covered,
         * we should enable glove again to mode 0 to make sure
         * that glove will still work after holster is removed.
        */
        mutex_lock(&rmi4_data->rmi4_holster_mutex);
        ret = synaptics_rmi4_i2c_write(rmi4_data,
                rmi4_data->f51_ctrl_base_addr + F51_CUSTOM_CTRL29_OFFSET,
                &data,
                sizeof(data));
        mutex_unlock(&rmi4_data->rmi4_holster_mutex);
        if (ret < 0) {
            tp_log_err("%s: Failed to set glove mode!ret=%d\n",
                    __func__,ret);
            goto exit;
        } else {
            /* In case that glove mode has been changed by holster ui, we need */
            /* to record ui state by another driver flag holster_enable_glove */
            tp_log_warning("%s: line=%d\n",__func__,__LINE__);
//            holster_enable_glove = false;
        }
    }

    return 0;
exit:
    return ret;
}

static ssize_t s3207_set_holster_mode(struct synaptics_rmi4_data *rmi4_data,int enable)
{
    int ret;

    if (enable)
    {
        /* Write holster window zone to F11_2D_CTRL92(01)/00~(01)/05*/
        ret = set_effective_window(rmi4_data,&synap_holster_info);
        if (ret < 0)
        {
            tp_log_err("%s:holster_zwindow_zone write error!ret = %d\n",
                    __func__,ret);
            goto exit;
        }

        /*Enable holster mode*/
        /*TP will only response to certain holster window zone*/
        ret = set_holster_mode(rmi4_data,
                rmi4_data->f51_ctrl_base_addr + F51_CUSTOM_CTRL63_OFFSET,
                RMI4_HOLSTER_FUNC_ENABLED);
        if (ret < 0) {
            tp_log_err("%s: Failed to enable holster mode\n",
                    __func__);
            goto exit;
        } else {
            rmi4_data->holster_enabled = true;
            tp_log_warning("%s: holster is Enabled!\n", __func__);
        }

        /*Enable glove mode*/
        /*TP will only response to finger with glove*/
        tp_log_debug("%s: line=%d\n",__func__,__LINE__);
        ret = F12_set_glove_mode(rmi4_data,SYSTEM_LOCKED_TO_GLOVE_MODE);
        if (ret < 0) {
            tp_log_err("%s: Failed to set glove mode!ret=%d\n",
                    __func__,ret);
            goto exit;
        } else {
            /* In case that glove mode has been set to 3 by holster ui, we need */
            /* to record ui state by another driver flag holster_enable_glove */
            holster_enable_glove = true;
            tp_log_warning("%s: glove is Enabled!\n", __func__);
        }

        /* when holster was covered, force cal onec */
        ret = synaptics_force_cal(rmi4_data);
        if (ret < 0) {
            tp_log_err("%s: Failed to force calibration!ret=%d\n",
                    __func__,ret);
            goto exit;
        }

        /* when holster closed , set TP sensitivity to low*/
        ret = synaptics_set_holster_low_sensitivity(rmi4_data, 1);
        if (ret < 0) {
            tp_log_err("%s: Failed to set holster low sensitivity!ret=%d\n",
                    __func__,ret);
            goto exit;
        }
    } else {
        /*Disable holster mode*/
        ret = set_holster_mode(rmi4_data,
                rmi4_data->f51_ctrl_base_addr + F51_CUSTOM_CTRL63_OFFSET,
                RMI4_HOLSTER_FUNC_DISABLED);
        if (ret < 0) {
            tp_log_err("%s: Failed to exit holster mode\n",
                    __func__);
            goto exit;
        } else{
            rmi4_data->holster_enabled = false;
            tp_log_warning("%s: holster is Disabled!\n", __func__);
        }

        /*
         * Change glove mode:
         * If glove has been enabled before holster was covered,
         * we should enable glove again to mode 0 to make sure
         * that glove will still work after holster is removed.
        */
        if(rmi4_data->glove_enabled == RMI4_GLOV_FUNC_ENABLED){
            /*TP will response to finger and glove,with finger default*/
            tp_log_debug("%s: line=%d\n",__func__,__LINE__);
            ret = F51_set_glove_mode(rmi4_data,SYSTEM_START_IN_SKIN_MODE);
        } else {
            /*TP will only response to finger*/
            tp_log_debug("%s: line=%d\n",__func__,__LINE__);
            ret = F51_set_glove_mode(rmi4_data,SYSTEM_LOCKED_TO_SKIN_MODE);
        }
        if (ret < 0) {
            tp_log_err("%s: Failed to set glove mode!ret=%d\n",
                    __func__,ret);
            goto exit;
        } else {
            /* In case that glove mode has been changed by holster ui, we need */
            /* to record ui state by another driver flag holster_enable_glove */
            tp_log_warning("%s: line=%d\n",__func__,__LINE__);
            holster_enable_glove = false;
        }

        /* when holster opened , set TP sensitivity to default */
        ret = synaptics_set_holster_low_sensitivity(rmi4_data, 0);
        if (ret < 0) {
            tp_log_err("%s: Failed to set holster low sensitivity!ret=%d\n",
                    __func__,ret);
            goto exit;
        }
    }

    return 0;
exit:
    return ret;
}


static ssize_t synaptics_holster_func_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
    struct synaptics_dsx_platform_data *platform_data = rmi4_data->board;
    unsigned int ic_type = platform_data->ic_type;
    int ret;
    int enable;
    int x0 = 0;
    int y0 = 0;
    int x1 = 0;
    int y1 = 0;

    ret = sscanf(buf, "%d %d %d %d %d",&enable, &x0, &y0, &x1, &y1);
    if (!ret) {
        tp_log_err("sscanf return invaild :%d\n", ret);
        ret = -EINVAL;
        goto out;
    }
    tp_log_debug("%s:sscanf value is enable=%d, (%d,%d), (%d,%d)\n",__func__, enable, x0, y0, x1, y1);
    
    if (enable && ((x0 < 0) || (y0 < 0) || (x1 <= x0) || (y1 <= y0))) {
        tp_log_err("%s:invalid value is %d (%d,%d), (%d,%d)\n",__func__, enable, x0, y0, x1, y1);
        ret = -EINVAL;
        goto out;
    }

    /* init holster window info*/
    synap_holster_info.top_left_x0 = x0;
    synap_holster_info.top_left_y0 = y0;
    synap_holster_info.bottom_right_x1 = x1;
    synap_holster_info.bottom_right_y1 = y1;
    synap_holster_info.holster_enable = enable;

    synaptics_pm_rt_get(rmi4_data);

    if (SYNAPTICS_S3320 == ic_type)
    {
        /* Set f12 glove mode:1-open,0-close */
        ret = s3320_set_holster_mode(rmi4_data,enable);
    }
    else
    {
        /* Set f51 holster mode */
        ret = s3207_set_holster_mode(rmi4_data,enable);
    }

    synaptics_pm_rt_put(rmi4_data);

out:
    if (ret < 0)
        return ret;

    return size;
}


static ssize_t synaptics_holster_func_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return snprintf(buf,PAGE_SIZE,"%ld %d %d %d %d\n", synap_holster_info.holster_enable, synap_holster_info.top_left_x0,
                synap_holster_info.top_left_y0, synap_holster_info.bottom_right_x1, synap_holster_info.bottom_right_y1);
}

static ssize_t hw_holster_func_store(struct kobject *dev,
        struct kobj_attribute *attr, const char *buf, size_t size)
{
    struct device *cdev = core_dev_ptr;
    if (!cdev){
        tp_log_err("%s: device is null", __func__);
        return -EINVAL;
    }
    return synaptics_holster_func_store(cdev, NULL, buf, size);
}

static ssize_t hw_holster_func_show(struct kobject *dev,
        struct kobj_attribute *attr, char *buf)
{
    struct device *cdev = core_dev_ptr;
    if (!cdev){
        tp_log_err("%s: device is null", __func__);
        return -EINVAL;
    }
    return synaptics_holster_func_show(cdev, NULL, buf);
}

static inline ssize_t hw_synaptics_rmi4_store_error(struct kobject *dev,
        struct kobj_attribute *attr, const char *buf, size_t size)
{
    struct device *cdev = core_dev_ptr;
    if (!cdev){
        tp_log_err("%s: device is null", __func__);
        return -EINVAL;
    }
    return synaptics_rmi4_store_error(cdev, NULL, buf, size);
}

static struct kobj_attribute easy_wakeup_gesture = {
    .attr = {.name = "easy_wakeup_gesture", .mode = (S_IRUGO | S_IWUSR | S_IWGRP)},
    .show = hw_synaptics_easy_wakeup_gesture_show,
    .store = hw_synaptics_easy_wakeup_gesture_store,
};

static struct kobj_attribute easy_wakeup_supported_gestures = {
    .attr = {.name = "easy_wakeup_supported_gestures", .mode = S_IRUGO},
    .show = hw_synaptics_easy_wakeup_supported_gestures_show,
    .store = hw_synaptics_rmi4_store_error,
};

static struct kobj_attribute glove_func = {
    .attr = {.name = "signal_disparity", .mode = (S_IRUGO | S_IWUSR | S_IWGRP)},
    .show = hw_glove_func_show,
    .store = hw_glove_func_store,
};

/*Normalized name*/
static struct kobj_attribute holster_func = {
    .attr = {.name = "holster_touch_window", .mode = (S_IRUGO | S_IWUSR | S_IWGRP)},
    .show = hw_holster_func_show,
    .store = hw_holster_func_store,
};

static int add_easy_wakeup_interfaces(struct device *dev)
{
    int error = 0;
    struct kobject *properties_kobj;
    struct kobject *glove_kobj;
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
    
    properties_kobj = tp_get_touch_screen_obj();
    if( NULL == properties_kobj )
    {
        tp_log_err("%s: Error, get kobj failed!\n", __func__);
        return -1;
    }

    if(0 != rmi4_data->board->easy_wakeup_supported_gestures)
    {
        /*add the node easy_wakeup_gesture_supported for apk to read*/
        error = sysfs_create_file(properties_kobj, &easy_wakeup_supported_gestures.attr);
        if (error)
        {
            kobject_put(properties_kobj);
            tp_log_err("%s: synaptics_easy_wakeup_gesture_supported create file error\n", __func__);
            return -ENODEV;
        }
        
        /*add the node easy_wakeup_gesture apk to write*/
        error = sysfs_create_file(properties_kobj, &easy_wakeup_gesture.attr);
        if (error)
        {
            kobject_put(properties_kobj);
            tp_log_err("%s: synaptics_easy_wakeup_gesture create file error\n", __func__);
            return -ENODEV;
        }

        /*add the node easy_wakeup_position for apk to write*/
        error = sysfs_create_file(properties_kobj, &easy_wakeup_position.attr);
        if (error)
        {
            kobject_put(properties_kobj);
            tp_log_err("%s: easy_wakeup_position create file error\n", __func__);
            return -ENODEV;
        }
    }

    if(true == rmi4_data->board->glove_supported)
    {
        glove_kobj = tp_get_glove_func_obj();
        if( NULL == glove_kobj )
        {
            tp_log_err("%s: Error, get kobj failed!\n", __func__);
            return -1;
        }
        
        /*add the node glove_func/signal_disparity for apk to write*/
        error = sysfs_create_file(glove_kobj, &glove_func.attr);
        if (error)
        {
            kobject_put(glove_kobj);
            error = sysfs_create_file(glove_kobj, &glove_func.attr);
            if (error)
            {
                kobject_put(glove_kobj);
                tp_log_err("%s: glove_func create file error\n", __func__);
                return -ENODEV;
            }
        }
        /*if dtsi enables the glove_edge_switch_supported, initialize the delayed work*/
        if(true == rmi4_data->board->glove_edge_switch_supported){
            INIT_DELAYED_WORK(&rmi4_data->glove_delay_work, synaptics_edge_glove_to_finger_delay_work);
        }
    }

    if(true == rmi4_data->board->holster_supported)
    {
        /*add the node holster_touch_window for apk to write*/
        error = sysfs_create_file(properties_kobj, &holster_func.attr);
        if (error)
        {
            kobject_put(properties_kobj);
            error = sysfs_create_file(properties_kobj, &holster_func.attr);
            if (error)
            {
                kobject_put(properties_kobj);
                tp_log_err("%s: holster_func create file error\n", __func__);
                return -ENODEV;
            }
        }
    }

    return 0;
}

static int synaptics_gpio_setup(int gpio, bool configure, int dir, int state)
{
    int retval = 0;
    unsigned char buf[16];

    if (configure) {
        snprintf(buf, PAGE_SIZE, "dsx_gpio_%u\n", gpio);

        retval = gpio_request(gpio, buf);
        if (retval) {
            tp_log_err("%s: Failed to get gpio %d (code: %d) \n",
                    __func__, gpio, retval);
            return retval;
        }

        if (dir == 0)
            retval = gpio_direction_input(gpio);
        else
            retval = gpio_direction_output(gpio, state);
        if (retval) {
            tp_log_err("%s: Failed to set gpio %d direction \n",
                    __func__, gpio);
            return retval;
        }
    } else {
        tp_log_warning("%s: free = %d \n",__func__, gpio);
        gpio_free(gpio);
    }

    return retval;
}

 /**
 * synaptics_rmi4_set_page()
 *
 * Called by synaptics_rmi4_i2c_read() and synaptics_rmi4_i2c_write().
 *
 * This function writes to the page select register to switch to the
 * assigned page.
 */
static int synaptics_rmi4_set_page(struct synaptics_rmi4_data *rmi4_data,
        unsigned int address)
{
    int retval = 0;
    unsigned char retry;
    unsigned char buf[PAGE_SELECT_LEN];
    unsigned char page;
    struct i2c_client *i2c = rmi4_data->i2c_client;

    page = ((address >> 8) & MASK_8BIT);
    if (page != rmi4_data->current_page) {
        buf[0] = MASK_8BIT;
        buf[1] = page;
        for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
            retval = i2c_master_send(i2c, buf, PAGE_SELECT_LEN);
            if (retval != PAGE_SELECT_LEN) {
                dev_err(&i2c->dev,
                        "%s: I2C retry %d\n",
                        __func__, retry + 1);
                msleep(20);
            } else {
                rmi4_data->current_page = page;
                break;
            }
        }
    } else {
        retval = PAGE_SELECT_LEN;
    }

    return retval;
}

/* For Qualcomm 8916 I2C workaround */
//#define I2C_LIMIT 255 //make sure transfer block less than 255bytes each time
#define I2C_LIMIT 1024
struct i2c_msg *read_msg;

 /**
 * synaptics_rmi4_i2c_read()
 *
 * Called by various functions in this driver, and also exported to
 * other expansion Function modules such as rmi_dev.
 *
 * This function reads data of an arbitrary length from the sensor,
 * starting from an assigned register address of the sensor, via I2C
 * with a retry mechanism.
 */
static int synaptics_rmi4_i2c_read(struct synaptics_rmi4_data *rmi4_data,
        unsigned short addr, unsigned char *data, unsigned short length)
{
    int retval;
    unsigned char retry;
    unsigned char buf;
    unsigned int full = length / I2C_LIMIT;      //how many I2C_LIMIT-bytes this block transfer
    unsigned int partial = length % I2C_LIMIT;//the rest of bytes this block transfer
    unsigned int total = 0;
    unsigned int last = 0;
    int ii = 0;
    static int msg_length = 0;
    if ( (full + 2) > msg_length ){
        kfree(read_msg);
        msg_length = full + 2;
        read_msg = kcalloc(msg_length, sizeof(struct i2c_msg), GFP_KERNEL);
    }

    read_msg[0].addr = rmi4_data->i2c_client->addr;
    read_msg[0].flags = 0;
    read_msg[0].len = 1;
    read_msg[0].buf = &buf;

    if (partial) {
        total = full + 1;
        last = partial;
    } else {
        total = full;
        last = I2C_LIMIT;
    }

    for ( ii = 1; ii <= total; ii++) {
        read_msg[ii].addr = rmi4_data->i2c_client->addr;
        read_msg[ii].flags = I2C_M_RD;
        read_msg[ii].len = ( ii == total ) ? last : I2C_LIMIT;
        read_msg[ii].buf = data + I2C_LIMIT * (ii - 1);
    }

    buf = addr & MASK_8BIT;

    mutex_lock(&(rmi4_data->rmi4_io_ctrl_mutex));

    retval = synaptics_rmi4_set_page(rmi4_data, addr);
    if (retval != PAGE_SELECT_LEN)
        goto exit;

    for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
        if (i2c_transfer(rmi4_data->i2c_client->adapter, read_msg, (total + 1)) == (total + 1)) {
            retval = length;
            break;
        }
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: I2C retry %d\n",
                __func__, retry + 1);
        msleep(20);
    }

    if (retry == SYN_I2C_RETRY_TIMES) {
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: I2C read over retry limit\n",
                __func__);
        retval = -EIO;
    }

exit:
    mutex_unlock(&(rmi4_data->rmi4_io_ctrl_mutex));

    /* if i2c read err, report err */
#ifdef CONFIG_HUAWEI_DSM
    if(retval<0)
    {
        synp_tp_report_dsm_err(DSM_TP_I2C_RW_ERROR_NO, retval);
    }
#endif/*CONFIG_HUAWEI_DSM*/

    return retval;
}

 /**
 * synaptics_rmi4_i2c_write()
 *
 * Called by various functions in this driver, and also exported to
 * other expansion Function modules such as rmi_dev.
 *
 * This function writes data of an arbitrary length to the sensor,
 * starting from an assigned register address of the sensor, via I2C with
 * a retry mechanism.
 */
static int synaptics_rmi4_i2c_write(struct synaptics_rmi4_data *rmi4_data,
        unsigned short addr, unsigned char *data, unsigned short length)
{
    int retval;
    unsigned char retry;
    unsigned char buf[length + 1];
    struct i2c_msg msg[] = {
        {
            .addr = rmi4_data->i2c_client->addr,
            .flags = 0,
            .len = length + 1,
            .buf = buf,
        }
    };

    mutex_lock(&(rmi4_data->rmi4_io_ctrl_mutex));

    retval = synaptics_rmi4_set_page(rmi4_data, addr);
    if (retval != PAGE_SELECT_LEN)
        goto exit;

    buf[0] = addr & MASK_8BIT;
    memcpy(&buf[1], &data[0], length);

    for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
        if (i2c_transfer(rmi4_data->i2c_client->adapter, msg, 1) == 1) {
            retval = length;
            break;
        }
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: I2C retry %d\n",
                __func__, retry + 1);
        msleep(20);
    }

    if (retry == SYN_I2C_RETRY_TIMES) {
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: I2C write over retry limit\n",
                __func__);
        retval = -EIO;
    }

exit:
    mutex_unlock(&(rmi4_data->rmi4_io_ctrl_mutex));

    /* if i2c write err, report err */
#ifdef CONFIG_HUAWEI_DSM
    if(retval<0)
    {
        synp_tp_report_dsm_err( DSM_TP_I2C_RW_ERROR_NO, retval);
    }
#endif/*CONFIG_HUAWEI_DSM*/

    return retval;
}

#ifdef CONFIG_HUAWEI_KERNEL
 static uint16_t check_scope_x(uint16_t x,int lcd_x)
 {
     uint16_t temp = x;
     if (x >= lcd_x -1)
     {
         temp = lcd_x -2;
     }
     if (x <= 1)
     {
         temp = 1;
     }
 
     return temp;
 }

#if USE_WAKEUP_GESTURE
static int init_easy_wakeup_key_value(struct synaptics_rmi4_data *rmi4_data)
{
    int i = 0;
    
    if(NULL  == rmi4_data->board->wakeup_keys)
    {
        tp_log_err("%s: wakeup keys is NULL! \n",__func__);
        return -1;
    }

    if(rmi4_data->board->wakeup_keys->size > GESTURE_MAX)
    {
        tp_log_err("%s: wakeup keys size not match! \n",__func__);
        return -1;
    }

    set_bit(KEY_POWER, rmi4_data->input_dev->keybit);
    
    for (i = 0; i < rmi4_data->board->wakeup_keys->size; ++i){
        tp_log_debug("%s: wakeup_keys[%d] = %d \n",__func__,i,rmi4_data->board->wakeup_keys->keys[i]);
        set_bit(rmi4_data->board->wakeup_keys->keys[i], rmi4_data->input_dev->keybit);
    }

    return 0;
}


#if REPORT_GESTURE_LOCUS
static int easy_wakeup_gesture_report_locus(struct synaptics_rmi4_data *rmi4_data,
        unsigned int reprot_gesture_point_num)
{
    int retval = 0;
    unsigned char f51_custom_data[F51_LOCUS_DATA_NUM] = {0};
    int x = 0;
    int y = 0;
    int i = 0;
    int x_lcd = 0;
    int y_lcd_all = 0;
    
    x_lcd = rmi4_data->board->lcd_x;
    y_lcd_all = rmi4_data->board->lcd_all;
    
    if (reprot_gesture_point_num != 0)
    {
        retval = synaptics_rmi4_i2c_read(rmi4_data,
                rmi4_data->f51_data_base_addr,
                f51_custom_data,
                (reprot_gesture_point_num*F51_LOCUS_DATA_LENS));
        if (retval < 0)
        {
            tp_log_err("%s:f51_data_base_addr read error!retval = %d \n",__func__,retval);
            return retval;
        }
    
        mutex_lock(&rmi4_data->rmi4_sysfs_mutex);
        memset(easywake_position,0,sizeof(easywake_position));
        for(i = 0;i < reprot_gesture_point_num;i ++)
        {
            /*
              *F51_CUSTOM_DATA[00]~[23],every 4 bytes save 1 x/y position of the gesture locus;
              *i.e.Point 1 has 4 bytes from [00] to [03] :
              * [00] means LSB of x position,[01] means MSB of x position,
              * [02] means LSB of y position,[03] means MSB of y position,
              * The most points num is 6,point from 1(lower address) to 6(higher address) means:
              * 1.beginning 2.end 3.top 4.leftmost 5.bottom 6.rightmost
              * The only one exception is the 4 points for GESTURE_SLIDE_T2B2:
              * 1.first finger beginning 2.first finger end 3.second finger beginning 4.second finger end 5.NA 6.NA
              */
            x = ((f51_custom_data[F51_LOCUS_DATA_LENS*i + 1] << 8) | (f51_custom_data[F51_LOCUS_DATA_LENS*i + 0]));
            y = ((f51_custom_data[F51_LOCUS_DATA_LENS*i + 3] << 8) | (f51_custom_data[F51_LOCUS_DATA_LENS*i + 2]));
            tp_log_debug("%s:f51_custom_data[%d] = 0x%x",__func__,(4*i + 0),f51_custom_data[4*i + 0]);
            tp_log_debug("%s:f51_custom_data[%d] = 0x%x",__func__,(4*i + 1),f51_custom_data[4*i + 1]);
            tp_log_debug("%s:f51_custom_data[%d] = 0x%x",__func__,(4*i + 2),f51_custom_data[4*i + 2]);
            tp_log_debug("%s:f51_custom_data[%d] = 0x%x",__func__,(4*i + 3),f51_custom_data[4*i + 3]);
            
            x = x * x_lcd / rmi4_data->sensor_max_x;
            x = check_scope_x(x,x_lcd);
            y = y * y_lcd_all / rmi4_data->sensor_max_y;
            
            easywake_position[i]=x<<16|y;        
            tp_log_debug("%s: easywake_position[%d] = %d:\n",
                            __func__, i,easywake_position[i]);
    
            tp_log_debug("%s: Point %d:\n"
                            "x = %d\n"
                            "y = %d\n",
                            __func__, i,x, y);
        }
        mutex_unlock(&rmi4_data->rmi4_sysfs_mutex);

    }
    return retval;
}
#endif/*REPORT_GESTURE_LOCUS*/

static void synaptics_rmi_check_and_save(
    struct synaptics_rmi4_data *rmi4_data,
    enum synaptics_gesture_num gesture,
    unsigned int *reprot_gesture_key_value,
    unsigned int *reprot_gesture_point_num,
    unsigned int locus_num)
{
    if(rmi4_data == NULL)
        return;

    if (IS_GESTURE_ENABLE(gesture, rmi4_data->easy_wakeup_gesture)) {
        *reprot_gesture_key_value =
            rmi4_data->board->wakeup_keys->keys[gesture];
        gesture_count[gesture]++;
        tp_log_debug("%s: Gesture detected,gesture_count[%d]=%d!\n",__func__, gesture,gesture_count[gesture]);
    #if REPORT_GESTURE_LOCUS
        *reprot_gesture_point_num = locus_num;
    #endif/*REPORT_GESTURE_LOCUS*/
    } else {
        tp_log_err("%s: TP Gesture: %d not enabled!!\n",__func__, gesture);
    }
}

static int synaptics_rmi_check_char_gesture(
    struct synaptics_rmi4_data *rmi4_data,
    unsigned char letter_gesture_type,
    unsigned int *reprot_gesture_key_value,
    unsigned int *reprot_gesture_point_num)
{
    switch(letter_gesture_type) {
    case SPECIFIC_LETTER_c:
        synaptics_rmi_check_and_save(rmi4_data,
            GESTURE_LETTER_c,
            reprot_gesture_key_value,
            reprot_gesture_point_num,
            F51_LETTER_LOCUS_NUM);
        tp_log_debug("%s:Gesture letter_C detected!\n",__func__);
        break;
    case SPECIFIC_LETTER_e:
        synaptics_rmi_check_and_save(rmi4_data,
            GESTURE_LETTER_e,
            reprot_gesture_key_value,
            reprot_gesture_point_num,
            F51_LETTER_LOCUS_NUM);
        tp_log_debug("%s:Gesture letter_e detected!\n",__func__);
        break;
    case SPECIFIC_LETTER_m:
        synaptics_rmi_check_and_save(rmi4_data,
            GESTURE_LETTER_m,
            reprot_gesture_key_value,
            reprot_gesture_point_num,
            F51_LETTER_LOCUS_NUM);
        tp_log_debug("%s:Gesture letter_M detected!\n",__func__);
        break;
    case SPECIFIC_LETTER_w:
        synaptics_rmi_check_and_save(rmi4_data,
            GESTURE_LETTER_w,
            reprot_gesture_key_value,
            reprot_gesture_point_num,
            F51_LETTER_LOCUS_NUM);
        tp_log_debug("%s:Gesture letter_W detected!\n",__func__);
        break;
    default:
        tp_log_err("%s:unknow letter!letter_gesture_type = 0x%x\n",
            __func__,letter_gesture_type);
        return -1;
    }
    return 0;
}

static int synaptics_rmi_check_slide_gesture(
    struct synaptics_rmi4_data *rmi4_data,
    unsigned int *reprot_gesture_key_value,
    unsigned int *reprot_gesture_point_num,
    char slide_gesture_type)
{
    switch(slide_gesture_type) {
    case LINEAR_SLIDE_LEFT_TO_RIGHT:
        synaptics_rmi_check_and_save(rmi4_data,
            GESTURE_SLIDE_L2R,
            reprot_gesture_key_value,
            reprot_gesture_point_num,
            F51_LINEAR_LOCUS_NUM);
        break;
    case LINEAR_SLIDE_RIGHT_TO_LEFT:
        synaptics_rmi_check_and_save(rmi4_data,
            GESTURE_SLIDE_R2L,
            reprot_gesture_key_value,
            reprot_gesture_point_num,
            F51_LINEAR_LOCUS_NUM);
        break;
    case LINEAR_SLIDE_TOP_TO_BOTTOM:
        synaptics_rmi_check_and_save(rmi4_data,
            GESTURE_SLIDE_T2B,
            reprot_gesture_key_value,
            reprot_gesture_point_num,
            F51_LINEAR_LOCUS_NUM);
        break;
    case LINEAR_SLIDE_BOTTOM_TO_TOP:
        synaptics_rmi_check_and_save(rmi4_data,
            GESTURE_SLIDE_B2T,
            reprot_gesture_key_value,
            reprot_gesture_point_num,
            F51_LINEAR_LOCUS_NUM);
        break;
    case LINEAR_SLIDE_TOP_TO_BOTTOM2:
        synaptics_rmi_check_and_save(rmi4_data,
            GESTURE_SLIDE_T2B2,
            reprot_gesture_key_value,
            reprot_gesture_point_num,
            F51_LINEAR2_LOCUS_NUM);
        break;
    default:
        tp_log_debug("%s:unknow LINEAR!slide_gesture_type = 0x%x\n",
            __func__,slide_gesture_type);
        return -1;
    }
    return 0;
}

/* Change define for function */
static int synaptics_rmi_f11_check_slide_gesture(
    struct synaptics_rmi4_data *rmi4_data,
    unsigned int *reprot_gesture_key_value,
    unsigned int *reprot_gesture_point_num)
{
    int retval = 0;
    unsigned char f51_custom_data24 = 0;

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f51_data_base_addr + F51_CUSTOM_DATA24_OFFSET,
            &f51_custom_data24,
            sizeof(f51_custom_data24));
    if (retval < 0) {
        tp_log_err("%s:LINEAR_SLIDE read error!retval = %d\n",
            __func__, retval);
        return retval;
    }

    retval = synaptics_rmi_check_slide_gesture(rmi4_data,
                reprot_gesture_key_value,
                reprot_gesture_point_num,
                f51_custom_data24);

    return retval;
}

static int synaptics_rmi_check_circle_gesture(
    struct synaptics_rmi4_data *rmi4_data,
    unsigned int *reprot_gesture_key_value,
    unsigned int *reprot_gesture_point_num)
{
    int retval;
    unsigned char f51_custom_data24 = 0;

    if(!(APP_ENABLE_GESTURE(GESTURE_CIRCLE_SLIDE)&rmi4_data->easy_wakeup_gesture))
    {
        tp_log_debug("%s:CIRCLE_SLIDE not enabled! \n",__func__);
        return -1;
    }
    tp_log_debug("%s:CIRCLE_SLIDE detected! \n",__func__);

    /* This reg show clock or counterclock */
    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f51_data_base_addr + F51_CUSTOM_DATA24_OFFSET,
            &f51_custom_data24,
            sizeof(f51_custom_data24));
    if (retval < 0) {
        tp_log_err("%s:CIRCLE_SLIDE read clockwise error!retval = %d\n",
            __func__, retval);
        return retval;
    }
    switch(f51_custom_data24) {
    case CIRCLE_SLIDE_COUNTERCLOCKWISE:
        synaptics_rmi_check_and_save(rmi4_data,
            GESTURE_CIRCLE_SLIDE,
            reprot_gesture_key_value,
            reprot_gesture_point_num,
            F51_LETTER_LOCUS_NUM);
        break;
    case CIRCLE_SLIDE_CLOCKWISE:
        if (IS_GESTURE_ENABLE(GESTURE_CIRCLE_SLIDE, rmi4_data->easy_wakeup_gesture)) {
            tp_log_info("%s:Gesture: %d CLOCKWISE detected!\n",__func__, GESTURE_CIRCLE_SLIDE);
            *reprot_gesture_key_value =
                rmi4_data->board->wakeup_keys->keys[GESTURE_SLIDE_T2B2];//use this key to report clockwise
            #if REPORT_GESTURE_LOCUS
            *reprot_gesture_point_num = F51_LETTER_LOCUS_NUM;
            #endif/*REPORT_GESTURE_LOCUS*/
        } else {
            tp_log_debug("%s: TP Gesture: %d not enabled!!\n",__func__, GESTURE_CIRCLE_SLIDE);
        }
        break;
    default:
        tp_log_debug("%s:unknow CIRCLE!f51_custom_data24 = 0x%x\n",
            __func__,f51_custom_data24);
        return -1;
    }
    return 0;
}

static int synaptics_rmi4_f12_gesture_report(struct synaptics_rmi4_data *rmi4_data,
        struct synaptics_rmi4_fn *fhandler)
{
    int retval;
    unsigned char device_data[5] = {0};
    unsigned int reprot_gesture_key_value = 0;
    unsigned long lTimerCur = jiffies;
    static unsigned long lTimerPre = 0;
    static unsigned long lTimerNext = 0;
    unsigned int reprot_gesture_point_num = 0;
    static int first_int_flag = true;

    if (true == first_int_flag) {
        lTimerPre = jiffies;
        lTimerCur = jiffies;
        first_int_flag = false;
        tp_log_debug("%s:Gesture first interrupt!\n",__func__);
    }

    if (time_before(lTimerCur, lTimerPre + lTimerNext)) {
        tp_log_debug("%s:Gesture Timer Interval is less than 1 senond!\n",__func__);
        return 0;
    } else {
        lTimerNext = GESTURE_TIMER_INTERVAL;
        lTimerPre = jiffies;
    }

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f12_data_base_addr + rmi4_data->f12_2d_data04_gesture,
            &device_data[0],
            sizeof(device_data));
    if (retval < 0){
        tp_log_err("%s:f12_data_base_addr(%d) read error!retval = %d\n",
                __func__, (rmi4_data->f12_data_base_addr + rmi4_data->f12_2d_data04_gesture),retval);
        return 0;
    }

    tp_log_debug("%s:rmi4_data->f12_2d_data04_gesture = %d \n",
            __func__, rmi4_data->f12_2d_data04_gesture);
    tp_log_debug("%s:f12_data_base_addr = %d,device_data[0] = %d\n",
            __func__, rmi4_data->f12_data_base_addr,device_data[0]);

    switch(device_data[0]) {
        //case DOUBLE_CLICK_WAKEUP:
        case 0x3:
            synaptics_rmi_check_and_save(rmi4_data,
                GESTURE_DOUBLE_CLICK,
                &reprot_gesture_key_value,
                &reprot_gesture_point_num,
                F51_DOUTAP_LOCUS_NUM);
            break;
        //case LINEAR_SLIDE_DETECTED:
        case 0x7:
            if (!(APP_ENABLE_LINEAR & rmi4_data->easy_wakeup_gesture)) {
                tp_log_debug("%s:LINEAR_SLIDE not enabled!\n",__func__);
                return 0;
            }
            tp_log_debug("%s:LINEAR_SLIDE detected!\n",__func__);
            if (synaptics_rmi_check_slide_gesture(rmi4_data,
                &reprot_gesture_key_value,
                &reprot_gesture_point_num,
                device_data[1])){
                return 0;
            }
            break;
        case CIRCLE_SLIDE_DETECTED:
            /*parse which circle gesture it is*/
            if (synaptics_rmi_check_circle_gesture(rmi4_data,
                &reprot_gesture_key_value,
                &reprot_gesture_point_num)){
                return 0;
            }
            break;
        //case SPECIFIC_LETTER_DETECTED:
        case 0xB:
            if (!(APP_ENABLE_LETTERS&rmi4_data->easy_wakeup_gesture)) {
                tp_log_debug("%s:SPECIFIC_LETTER not enabled!\n",__func__);
                return 0;
            }
            tp_log_debug("%s:SPECIFIC_LETTER detected!\n",__func__);

            if (synaptics_rmi_check_char_gesture(rmi4_data,
                device_data[2],
                &reprot_gesture_key_value,
                &reprot_gesture_point_num))
                return 0;
            break;
        default:
            tp_log_info("%s:unknow gesture detected! device_data[0]=%d\n",__func__ ,device_data[0]);
            return 0;
    }

    /*Gesture detected, report the event.*/
    if (0 != reprot_gesture_key_value) {
    /*Make sure the position be ready before wake up system*/
#if REPORT_GESTURE_LOCUS
        retval = easy_wakeup_gesture_report_locus(rmi4_data,reprot_gesture_point_num);
        if (retval < 0) {
            tp_log_err("%s: report locus error!retval = %d \n",__func__,retval);
            return 0;
        }
#endif/*REPORT_GESTURE_LOCUS*/

        /*
        * New solution, reporting Power_Key is cannceled.
        *input_report_key(rmi4_data->input_dev,KEY_POWER, 1);
        *input_sync(rmi4_data->input_dev);
        *input_report_key(rmi4_data->input_dev,KEY_POWER, 0);
        *input_sync(rmi4_data->input_dev);
        */

        tp_log_warning("%s:reprot_gesture_key_value = %d\n",__func__, reprot_gesture_key_value);
        input_report_key(rmi4_data->input_dev,reprot_gesture_key_value, 1);
        input_sync(rmi4_data->input_dev);
        input_report_key(rmi4_data->input_dev,reprot_gesture_key_value, 0);
        input_sync(rmi4_data->input_dev);
    }
    return 0;
}

static int synaptics_rmi4_f11_gesture_report(struct synaptics_rmi4_data *rmi4_data,
        struct synaptics_rmi4_fn *fhandler)
{
    int retval;
    unsigned char device_data = 0;
    unsigned char f11_2d_data39[9] = {0};
    unsigned int reprot_gesture_key_value = 0;
    unsigned long lTimerCur = jiffies;
    static unsigned long lTimerPre = 0;
    static unsigned long lTimerNext = 0;
    unsigned int reprot_gesture_point_num = 0;
    static int first_int_flag = true;

    if (true == first_int_flag) {
        lTimerPre = jiffies;
        lTimerCur = jiffies;
        first_int_flag = false;
        tp_log_debug("%s:Gesture first interrupt!\n",__func__);
    }

    //tp_log_debug("%s:lTimerCur = %ld,lTimerPre + lTimerNext = %ld\n",
            //__func__,lTimerCur,(lTimerPre + lTimerNext));
    if (time_before(lTimerCur, lTimerPre + lTimerNext)) {
        tp_log_debug("%s:Gesture Timer Interval is less than 1 senond!\n",__func__);
        return 0;
    } else {
        lTimerNext = GESTURE_TIMER_INTERVAL;
        lTimerPre = jiffies;
        //tp_log_debug("%s:lTimerPre + lTimerNext = %ld\n",
                //__func__, (lTimerPre + lTimerNext));
    }

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f11_data_base_addr + F11_2D_DATA38_OFFSET,
            &device_data,
            sizeof(device_data));
    if (retval < 0){
        tp_log_err("%s:f11_data_base_addr(%d) read error!retval = %d\n",
                __func__, (rmi4_data->f11_data_base_addr + F11_2D_DATA38_OFFSET),retval);
        return 0;
    }

    tp_log_vdebug("%s:f11_data_base_addr = %d,device_data = %d\n",
            __func__, rmi4_data->f11_data_base_addr,device_data);

    switch(device_data) {
        case DOUBLE_CLICK_WAKEUP:
            synaptics_rmi_check_and_save(rmi4_data,
                GESTURE_DOUBLE_CLICK,
                &reprot_gesture_key_value,
                &reprot_gesture_point_num,
                0);
            break;
        case LINEAR_SLIDE_DETECTED:
            if (!(APP_ENABLE_LINEAR & rmi4_data->easy_wakeup_gesture)) {
                tp_log_debug("%s:LINEAR_SLIDE not enabled!\n",__func__);
                return 0;
            }
            tp_log_debug("%s:LINEAR_SLIDE detected!\n",__func__);

            /*check and save slide gesture*/
            /* Change defination */
            if (synaptics_rmi_f11_check_slide_gesture(rmi4_data,
                &reprot_gesture_key_value,
                &reprot_gesture_point_num)){
                return 0;
            }
            break;
        case CIRCLE_SLIDE_DETECTED:
            /*parse which circle gesture it is*/
            if (synaptics_rmi_check_circle_gesture(rmi4_data,
                &reprot_gesture_key_value,
                &reprot_gesture_point_num)){
                return 0;
            }
            break;
        case SPECIFIC_LETTER_DETECTED:
            if (!(APP_ENABLE_LETTERS&rmi4_data->easy_wakeup_gesture)) {
                tp_log_debug("%s:SPECIFIC_LETTER not enabled!\n",__func__);
                return 0;
            }
            tp_log_debug("%s:SPECIFIC_LETTER detected!\n",__func__);
            retval = synaptics_rmi4_i2c_read(rmi4_data,
                    rmi4_data->f11_data_base_addr+F11_2D_DATA39_OFFSET,
                    &f11_2d_data39[0],
                    sizeof(f11_2d_data39));
            if (retval < 0) {
                tp_log_err("%s:SPECIFIC_LETTER read error!retval = %d\n",
                    __func__,retval);
                return 0;
            }

            //tp_log_vdebug("%s: read f11_2d_data39[6] = 0x%x\n",
                //__func__,f11_2d_data39[6]);

            if (synaptics_rmi_check_char_gesture(rmi4_data,
                f11_2d_data39[6],
                &reprot_gesture_key_value,
                &reprot_gesture_point_num))
                return 0;
            break;
        default:
            tp_log_info("%s:unknow gesture detected! device_data=%d\n",__func__ ,device_data);
            return 0;
    }

    /*Gesture detected, report the event.*/
    if (0 != reprot_gesture_key_value) {
    /*Make sure the position be ready before wake up system*/
#if REPORT_GESTURE_LOCUS
        retval = easy_wakeup_gesture_report_locus(rmi4_data,reprot_gesture_point_num);
        if (retval < 0) {
            tp_log_err("%s: report locus error!retval = %d",__func__,retval);
            return 0;
        }
#endif/*REPORT_GESTURE_LOCUS*/

        /*
        * New solution, reporting Power_Key is cannceled.
        *input_report_key(rmi4_data->input_dev,KEY_POWER, 1);
        *input_sync(rmi4_data->input_dev);
        *input_report_key(rmi4_data->input_dev,KEY_POWER, 0);
        *input_sync(rmi4_data->input_dev);
        */

        tp_log_warning("%s:reprot_gesture_key_value = %d\n",__func__, reprot_gesture_key_value);
        input_report_key(rmi4_data->input_dev,reprot_gesture_key_value, 1);
        input_sync(rmi4_data->input_dev);
        input_report_key(rmi4_data->input_dev,reprot_gesture_key_value, 0);
        input_sync(rmi4_data->input_dev);
    }
    return 0;
}

static int synaptics_rmi4_f12_palm_report(struct synaptics_rmi4_data *rmi4_data,
        struct synaptics_rmi4_fn *fhandler)
{
    int retval;
    unsigned char device_data = 0;
    unsigned int reprot_gesture_key_value = 0;
    static int palm_is_covered = false;
    
    /* Object Type and Status */
    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f12_data_base_addr,
            &device_data,
            sizeof(device_data));
    if (retval < 0)
        return 0;

    tp_log_vdebug("%s:f12_data_base_addr = %d,device_data = %d \n",
                __func__, rmi4_data->f12_data_base_addr,device_data);
    /*
     * Palm cover sleep gesture
     * 1:has fingers;
     * 3:has palm covered;
     */
    if (device_data == MASK_2BIT)
    {
        if(palm_is_covered == true)
        {
            tp_log_vdebug("%s:GESTURE_PALM_COVERED do not report again! \n",__func__);
            return 0;
        }
        
        if (APP_ENABLE_GESTURE(GESTURE_PALM_COVERED)&rmi4_data->easy_wakeup_gesture)
        {
            tp_log_warning("%s:GESTURE_PALM_COVERED detected! \n",__func__);
            reprot_gesture_key_value = rmi4_data->board->wakeup_keys->keys[GESTURE_PALM_COVERED];
            palm_is_covered = true;
            goto report_key;
        }
    }
    else
    {
        tp_log_vdebug("%s:GESTURE_PALM_COVERED not detected! \n",__func__);
        palm_is_covered = false;
    }

    return 0;

report_key:
    input_report_key(rmi4_data->input_dev,KEY_POWER, 1);
    input_sync(rmi4_data->input_dev);
    input_report_key(rmi4_data->input_dev,KEY_POWER, 0);
    input_sync(rmi4_data->input_dev);

    tp_log_debug("%s:reprot_gesture_key_value = %d \n",__func__, reprot_gesture_key_value);
    input_report_key(rmi4_data->input_dev,reprot_gesture_key_value, 1);
    input_sync(rmi4_data->input_dev);
    input_report_key(rmi4_data->input_dev,reprot_gesture_key_value, 0);
    input_sync(rmi4_data->input_dev);

    return 1;
}


static int synaptics_rmi4_f11_palm_report(struct synaptics_rmi4_data *rmi4_data,
        struct synaptics_rmi4_fn *fhandler)
{
    int retval;
    unsigned char device_data = 0;
    unsigned int reprot_gesture_key_value = 0;
    static int palm_is_covered = false;

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f11_data_base_addr+F11_2D_DATA28_OFFSET,
            &device_data,
            sizeof(device_data));
    if (retval < 0)
        return 0;

    tp_log_vdebug("%s:f11_data_base_addr = %d,device_data = %d \n",
                            __func__, rmi4_data->f11_data_base_addr,device_data);
    /*
     * Palm cover sleep gesture
     * 0:for no palm;
     * 1:for has palm cover;
     */
    if (device_data)
    {
        if(palm_is_covered == true)
        {
            tp_log_vdebug("%s:GESTURE_PALM_COVERED do not report again! \n",__func__);
            return 0;
        }
        
        if (APP_ENABLE_GESTURE(GESTURE_PALM_COVERED)&rmi4_data->easy_wakeup_gesture)
        {
            tp_log_warning("%s:GESTURE_PALM_COVERED detected! \n",__func__);
            reprot_gesture_key_value = rmi4_data->board->wakeup_keys->keys[GESTURE_PALM_COVERED];
            palm_is_covered = true;
            goto report_key;
        }
    }
    else
    {
        tp_log_vdebug("%s:GESTURE_PALM_COVERED not detected! \n",__func__);
        palm_is_covered = false;
    }
    return 0;

report_key:
/*    input_report_key(rmi4_data->input_dev,KEY_POWER, 1);
    input_sync(rmi4_data->input_dev);
    input_report_key(rmi4_data->input_dev,KEY_POWER, 0);
    input_sync(rmi4_data->input_dev);*/

    tp_log_debug("%s:reprot_gesture_key_value = %d \n",__func__, reprot_gesture_key_value);
    input_report_key(rmi4_data->input_dev,reprot_gesture_key_value, 1);
    input_sync(rmi4_data->input_dev);
    input_report_key(rmi4_data->input_dev,reprot_gesture_key_value, 0);
    input_sync(rmi4_data->input_dev);
    return 1;
}
#endif /*USE_WAKEUP_GESTURE*/
#endif /*CONFIG_HUAWEI_KERNEL*/

 /**
 * synaptics_rmi4_f11_abs_report()
 *
 * Called by synaptics_rmi4_report_touch() when valid Function $11
 * finger data has been detected.
 *
 * This function reads the Function $11 data registers, determines the
 * status of each finger supported by the Function, processes any
 * necessary coordinate manipulation, reports the finger data to
 * the input subsystem, and returns the number of fingers detected.
 */
static int synaptics_rmi4_f11_abs_report(struct synaptics_rmi4_data *rmi4_data,
        struct synaptics_rmi4_fn *fhandler)
{
    int retval;
    unsigned char touch_count = 0; /* number of touch points */
    unsigned char reg_index;
    unsigned char finger;
    unsigned char fingers_supported;
    unsigned char num_of_finger_status_regs;
    unsigned char finger_shift;
    unsigned char finger_status;
    unsigned char data_reg_blk_size;
    unsigned char finger_status_reg[3];
    unsigned char data[F11_STD_DATA_LEN];
    unsigned short data_addr;
    unsigned short data_offset;
    int x;
    int y;
    int z;
    int wx;
    int wy;
    int temp;
#ifdef CONFIG_HUAWEI_KERNEL
    int x_lcd,y_lcd_all;
    x_lcd = rmi4_data->board->lcd_x;
    y_lcd_all = rmi4_data->board->lcd_all;
#endif /*CONFIG_HUAWEI_KERNEL*/

    tp_log_debug("%s: in \n",__func__);
#if USE_WAKEUP_GESTURE
    
    if(false != rmi4_data->easy_wakeup_gesture)
    {
        if(true == rmi4_data->sleep_gesture_flag)//work only when suspended
        {
            tp_log_vdebug("%s:wake up gesture mode  in!sleep_gesture_flag = %d \n",__func__, rmi4_data->sleep_gesture_flag);
            return synaptics_rmi4_f11_gesture_report(rmi4_data,fhandler);
        }
        else if(true == rmi4_data->palm_enabled)//work only when resumed
        {
            tp_log_vdebug("%s:palm sleep gesture mode  in! \n",__func__);
            retval = synaptics_rmi4_f11_palm_report(rmi4_data,fhandler);
            if (1 == retval)
                return 0;
        }
        else
        {
            tp_log_vdebug("%s:easy wake up gesture not enabled! \n",__func__);
        }

    }
#endif /*USE_WAKEUP_GESTURE*/
    /*
     * The number of finger status registers is determined by the
     * maximum number of fingers supported - 2 bits per finger. So
     * the number of finger status registers to read is:
     * register_count = ceil(max_num_of_fingers / 4)
     */
    fingers_supported = fhandler->num_of_data_points;
    num_of_finger_status_regs = (fingers_supported + 3) / 4;
    data_addr = fhandler->full_addr.data_base;
    data_reg_blk_size = fhandler->size_of_data_register_block;

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            data_addr,
            finger_status_reg,
            num_of_finger_status_regs);
    if (retval < 0)
        return 0;
    tp_log_debug("%s(line%d): finger_status_reg = %d \n",__func__,__LINE__,finger_status_reg[0]);

    for (finger = 0; finger < fingers_supported; finger++) {
        reg_index = finger / 4;
        finger_shift = (finger % 4) * 2;
        finger_status = (finger_status_reg[reg_index] >> finger_shift)
                & MASK_2BIT;

        /*
         * Each 2-bit finger status field represents the following:
         * 00 = finger not present
         * 01 = finger present and data accurate
         * 10 = finger present but data may be inaccurate
         * 11 = reserved
         */
#ifdef TYPE_B_PROTOCOL
        input_mt_slot(rmi4_data->input_dev, finger);
        input_mt_report_slot_state(rmi4_data->input_dev,
                MT_TOOL_FINGER, finger_status);
#endif

        if (finger_status) {
            data_offset = data_addr +
                    num_of_finger_status_regs +
                    (finger * data_reg_blk_size);
            tp_log_debug("%s(line%d): retval = %d \n",__func__,__LINE__,retval);
            retval = synaptics_rmi4_i2c_read(rmi4_data,
                    data_offset,
                    data,
                    data_reg_blk_size);
            if (retval < 0)
                return 0;

            x = (data[0] << 4) | (data[2] & MASK_4BIT);
            y = (data[1] << 4) | ((data[2] >> 4) & MASK_4BIT);
            wx = (data[3] & MASK_4BIT);
            wy = (data[3] >> 4) & MASK_4BIT;
            z = data[4];

            if (rmi4_data->board->swap_axes) {
                temp = x;
                x = y;
                y = temp;
                temp = wx;
                wx = wy;
                wy = temp;
            }

            if (rmi4_data->board->x_flip)
                x = rmi4_data->sensor_max_x - x;
            if (rmi4_data->board->y_flip)
                y = rmi4_data->sensor_max_y - y;

#ifdef CONFIG_HUAWEI_KERNEL
            tp_log_debug("%s: x_lcd = %d,y_lcd_all = %d  \n",__func__, x_lcd,y_lcd_all);
            tp_log_debug("%s: max_x = %d,max_y = %d  \n",__func__, rmi4_data->sensor_max_x,rmi4_data->sensor_max_y);
            x = x * x_lcd / rmi4_data->sensor_max_x;
            x = check_scope_x(x,x_lcd);
            y = y * y_lcd_all / rmi4_data->sensor_max_y;
#endif /*CONFIG_HUAWEI_KERNEL*/
            input_report_key(rmi4_data->input_dev,
                    BTN_TOUCH, 1);
#if USE_BTN_TOOL_FINGER//huawei 11-25
            input_report_key(rmi4_data->input_dev,
                    BTN_TOOL_FINGER, 1);
#endif//huawei 11-25
            input_report_abs(rmi4_data->input_dev,
                    ABS_MT_POSITION_X, x);
            input_report_abs(rmi4_data->input_dev,
                    ABS_MT_POSITION_Y, y);
            input_report_abs(rmi4_data->input_dev,
                    ABS_MT_PRESSURE, z);
#ifdef REPORT_2D_W
            input_report_abs(rmi4_data->input_dev,
                    ABS_MT_TOUCH_MAJOR, max(wx, wy));
            input_report_abs(rmi4_data->input_dev,
                    ABS_MT_TOUCH_MINOR, min(wx, wy));
#endif
#ifndef TYPE_B_PROTOCOL
            input_mt_sync(rmi4_data->input_dev);
#endif

            tp_log_debug("%s: Finger %d:\n"
                    "status = 0x%02x\n"
                    "x = %d\n"
                    "y = %d\n"
                    "z = %d\n"
                    "wx = %d\n"
                    "wy = %d\n",
                    __func__, finger,
                    finger_status,
                    x, y,z, wx, wy);

            touch_count++;
        }
    }

    if (touch_count == 0) {
        input_report_key(rmi4_data->input_dev,
                BTN_TOUCH, 0);
#if USE_BTN_TOOL_FINGER//huawei 11-25
        input_report_key(rmi4_data->input_dev,
                BTN_TOOL_FINGER, 0);
#endif//huawei 11-25
#ifndef TYPE_B_PROTOCOL
        tp_log_vdebug("%s:NO 2-6-38  \n",__func__);
        input_mt_sync(rmi4_data->input_dev);
#endif
    }

    input_sync(rmi4_data->input_dev);

    tp_log_vdebug("%s: out  \n",__func__);

    return touch_count;
}

 /**
 * synaptics_rmi4_f12_abs_report()
 *
 * Called by synaptics_rmi4_report_touch() when valid Function $12
 * finger data has been detected.
 *
 * This function reads the Function $12 data registers, determines the
 * status of each finger supported by the Function, processes any
 * necessary coordinate manipulation, reports the finger data to
 * the input subsystem, and returns the number of fingers detected.
 */
static int synaptics_rmi4_f12_abs_report(struct synaptics_rmi4_data *rmi4_data,
        struct synaptics_rmi4_fn *fhandler)
{
    int retval;
    unsigned char touch_count = 0; /* number of touch points */
    unsigned char finger;
    unsigned char fingers_to_process;
    unsigned char finger_status;
    unsigned char size_of_2d_data;
    unsigned short data_addr;
    int x;
    int y;
    unsigned char  z;
    int wx;
    int wy;
    int temp;
    struct synaptics_rmi4_f12_extra_data *extra_data;
    struct synaptics_rmi4_f12_finger_data *data;
    struct synaptics_rmi4_f12_finger_data *finger_data;
#ifdef F12_DATA_15_WORKAROUND
    static unsigned char fingers_already_present;
#endif
    int x_lcd,y_lcd,y_lcd_all;
    tp_log_debug("%s:fhandler->fn_number=%d \n",
    __func__,fhandler->fn_number);
    x_lcd = rmi4_data->board->lcd_x;
    y_lcd = rmi4_data->board->lcd_y;
    y_lcd_all = rmi4_data->board->lcd_all;

#if USE_WAKEUP_GESTURE
    
    tp_log_debug("%s:sleep_gesture_flag = %d\n",__func__, rmi4_data->sleep_gesture_flag);
    tp_log_debug("%s:easy_wakeup_gesture = %d\n",__func__, rmi4_data->easy_wakeup_gesture);
    if(false != rmi4_data->easy_wakeup_gesture)
    {
        if(true == rmi4_data->sleep_gesture_flag)//work only when suspended
        {
            tp_log_vdebug("%s:wake up gesture mode  in!sleep_gesture_flag = %d\n",__func__, rmi4_data->sleep_gesture_flag);
            return synaptics_rmi4_f12_gesture_report(rmi4_data,fhandler);
        }
        else if(true == rmi4_data->palm_enabled)//work only when resumed
        {
            tp_log_vdebug("%s:palm sleep gesture mode  in!\n",__func__);
            retval = synaptics_rmi4_f12_palm_report(rmi4_data,fhandler);
            if (1 == retval)
                return 0;
        }
        else
        {
            tp_log_vdebug("%s:easy wake up gesture not enabled!\n",__func__);
        }

    }
#endif /*USE_WAKEUP_GESTURE*/

    fingers_to_process = fhandler->num_of_data_points;
    data_addr = fhandler->full_addr.data_base;
    extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
    size_of_2d_data = sizeof(struct synaptics_rmi4_f12_finger_data);


    /* Determine the total number of fingers to process */
    if (extra_data->data15_size) {
        retval = synaptics_rmi4_i2c_read(rmi4_data,
                data_addr + extra_data->data15_offset,
                extra_data->data15_data,
                extra_data->data15_size);
        if (retval < 0)
            return 0;

        /* Start checking from the highest bit */
        temp = extra_data->data15_size - 1; /* Highest byte */
        finger = (fingers_to_process - 1) % 8; /* Highest bit */
        do {
            if (extra_data->data15_data[temp] & (1 << finger))
                break;

            if (finger) {
                finger--;
            } else {
                temp--; /* Move to the next lower byte */
                finger = 7;
            }

            fingers_to_process--;
        } while (fingers_to_process);

        dev_dbg(&rmi4_data->i2c_client->dev,
            "%s: Number of fingers to process = %d\n",
            __func__, fingers_to_process);
    }

#ifdef F12_DATA_15_WORKAROUND
    fingers_to_process = max(fingers_to_process, fingers_already_present);
#endif

    if (!fingers_to_process) {
        synaptics_rmi4_free_fingers(rmi4_data);
        return 0;
    }

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            data_addr + extra_data->data1_offset,
            (unsigned char *)fhandler->data,
            fingers_to_process * size_of_2d_data);
    if (retval < 0)
        return 0;

    data = (struct synaptics_rmi4_f12_finger_data *)fhandler->data;

    for (finger = 0; finger < fingers_to_process; finger++) {
        finger_data = data + finger;
        finger_status = finger_data->object_type_and_status;// & MASK_1BIT;

#ifdef TYPE_B_PROTOCOL
        input_mt_slot(rmi4_data->input_dev, finger);
        input_mt_report_slot_state(rmi4_data->input_dev,
                MT_TOOL_FINGER, finger_status);
#endif

        if (finger_status) {
#ifdef F12_DATA_15_WORKAROUND
            fingers_already_present = finger + 1;
#endif

            x = (finger_data->x_msb << 8) | (finger_data->x_lsb);
            y = (finger_data->y_msb << 8) | (finger_data->y_lsb);
            z = finger_data->z;
#ifdef REPORT_2D_W
            wx = finger_data->wx;
            wy = finger_data->wy;
#endif

            if (rmi4_data->board->swap_axes) {
                temp = x;
                x = y;
                y = temp;
                temp = wx;
                wx = wy;
                wy = temp;
            }

            if (rmi4_data->board->x_flip)
                x = rmi4_data->sensor_max_x - x;
            if (rmi4_data->board->y_flip)
                y = rmi4_data->sensor_max_y - y;
            x = x * x_lcd / rmi4_data->sensor_max_x;
            x = check_scope_x(x,x_lcd);
            y = y * y_lcd_all / rmi4_data->sensor_max_y;
            input_report_key(rmi4_data->input_dev,
                    BTN_TOUCH, 1);
#if USE_BTN_TOOL_FINGER//huawei 11-25
            input_report_key(rmi4_data->input_dev,
                    BTN_TOOL_FINGER, 1);
#endif//huawei 11-25
            input_report_abs(rmi4_data->input_dev,
                    ABS_MT_POSITION_X, x);
            input_report_abs(rmi4_data->input_dev,
                    ABS_MT_POSITION_Y, y);
            input_report_abs(rmi4_data->input_dev,
                    ABS_MT_PRESSURE, z);
#ifdef REPORT_2D_W
            if (rmi4_data->wedge_sensor) {
                input_report_abs(rmi4_data->input_dev,
                        ABS_MT_TOUCH_MAJOR, wx);
                input_report_abs(rmi4_data->input_dev,
                        ABS_MT_TOUCH_MINOR, wx);
            } else {
                input_report_abs(rmi4_data->input_dev,
                        ABS_MT_TOUCH_MAJOR,
                        max(wx, wy));
                input_report_abs(rmi4_data->input_dev,
                        ABS_MT_TOUCH_MINOR,
                        min(wx, wy));
            }
#endif
#ifndef TYPE_B_PROTOCOL
            input_mt_sync(rmi4_data->input_dev);
#endif

            tp_log_vdebug("%s: Finger %d:\n"
                    "status = 0x%02x\n"
                    "x = %d\n"
                    "y = %d\n"
                    "z = %d\n"
                    "wx = %d\n"
                    "wy = %d\n",
                    __func__, finger,
                    finger_status,
                    x, y,z, wx, wy);
            dev_dbg(&rmi4_data->i2c_client->dev,
                    "%s: Finger %d:\n"
                    "status = 0x%02x\n"
                    "x = %d\n"
                    "y = %d\n"
                    "wx = %d\n"
                    "wy = %d\n",
                    __func__, finger,
                    finger_status,
                    x, y, wx, wy);

            touch_count++;
        }
    }

    if (touch_count == 0) {
        input_report_key(rmi4_data->input_dev,
                BTN_TOUCH, 0);
#if USE_BTN_TOOL_FINGER//huawei 11-25
        input_report_key(rmi4_data->input_dev,
                BTN_TOOL_FINGER, 0);
#endif//huawei 11-25
#ifndef TYPE_B_PROTOCOL
        input_mt_sync(rmi4_data->input_dev);
#endif
    }

    input_sync(rmi4_data->input_dev);

    return touch_count;
}

static void synaptics_rmi4_f1a_report(struct synaptics_rmi4_data *rmi4_data,
        struct synaptics_rmi4_fn *fhandler)
{
    int retval;
    unsigned char touch_count = 0;
    unsigned char button;
    unsigned char index;
    unsigned char shift;
    unsigned char status;
    unsigned char *data;
    unsigned short data_addr = fhandler->full_addr.data_base;
    struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;
    static unsigned char do_once = 1;
    static bool current_status[MAX_NUMBER_OF_BUTTONS];
#ifdef NO_0D_WHILE_2D
    static bool before_2d_status[MAX_NUMBER_OF_BUTTONS];
    static bool while_2d_status[MAX_NUMBER_OF_BUTTONS];
#endif

    if (do_once) {
        memset(current_status, 0, sizeof(current_status));
#ifdef NO_0D_WHILE_2D
        memset(before_2d_status, 0, sizeof(before_2d_status));
        memset(while_2d_status, 0, sizeof(while_2d_status));
#endif
        do_once = 0;
    }

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            data_addr,
            f1a->button_data_buffer,
            f1a->button_bitmask_size);
    if (retval < 0) {
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: Failed to read button data registers\n",
                __func__);
        return;
    }

    data = f1a->button_data_buffer;

    for (button = 0; button < f1a->valid_button_count; button++) {
        index = button / 8;
        shift = button % 8;
        status = ((data[index] >> shift) & MASK_1BIT);

        if (current_status[button] == status)
            continue;
        else
            current_status[button] = status;

        dev_dbg(&rmi4_data->i2c_client->dev,
                "%s: Button %d (code %d) ->%d\n",
                __func__, button,
                f1a->button_map[button],
                status);
#ifdef NO_0D_WHILE_2D
        if (rmi4_data->fingers_on_2d == false) {
            if (status == 1) {
                before_2d_status[button] = 1;
            } else {
                if (while_2d_status[button] == 1) {
                    while_2d_status[button] = 0;
                    continue;
                } else {
                    before_2d_status[button] = 0;
                }
            }
            touch_count++;
            input_report_key(rmi4_data->input_dev,
                    f1a->button_map[button],
                    status);
        } else {
            if (before_2d_status[button] == 1) {
                before_2d_status[button] = 0;
                touch_count++;
                input_report_key(rmi4_data->input_dev,
                        f1a->button_map[button],
                        status);
            } else {
                if (status == 1)
                    while_2d_status[button] = 1;
                else
                    while_2d_status[button] = 0;
            }
        }
#else
        touch_count++;
        input_report_key(rmi4_data->input_dev,
                f1a->button_map[button],
                status);
#endif
    }

    if (touch_count)
        input_sync(rmi4_data->input_dev);

    return;
}

 /**
 * synaptics_rmi4_report_touch()
 *
 * Called by synaptics_rmi4_sensor_report().
 *
 * This function calls the appropriate finger data reporting function
 * based on the function handler it receives and returns the number of
 * fingers detected.
 */
static void synaptics_rmi4_report_touch(struct synaptics_rmi4_data *rmi4_data,
        struct synaptics_rmi4_fn *fhandler)
{
    unsigned char touch_count_2d;

    tp_log_debug("%s: Func %02x reporting\n",__func__, fhandler->fn_number);
    tp_log_debug("%s: fn_number=%d\n",__func__,fhandler->fn_number);

    switch (fhandler->fn_number) {
    case SYNAPTICS_RMI4_F11:
        touch_count_2d = synaptics_rmi4_f11_abs_report(rmi4_data,fhandler);

        if (touch_count_2d) {
            rmi4_data->fingers_on_2d = true;
        } else {
            rmi4_data->fingers_on_2d = false;
        }
        break;
    case SYNAPTICS_RMI4_F12:
        touch_count_2d = synaptics_rmi4_f12_abs_report(rmi4_data,fhandler);

        if (touch_count_2d) {
            rmi4_data->fingers_on_2d = true;
        } else {
            rmi4_data->fingers_on_2d = false;
        }
        break;
    case SYNAPTICS_RMI4_F1A:
        synaptics_rmi4_f1a_report(rmi4_data, fhandler);
        break;
    default:
        break;
    }

    return;
}
 
 /**
 * synaptics_rmi4_sensor_report()
 *
 * Called by synaptics_rmi4_irq().
 *
 * This function determines the interrupt source(s) from the sensor
 * and calls synaptics_rmi4_report_touch() with the appropriate
 * function handler for each function with valid data inputs.
 */
#if USE_IRQ_THREAD //huawei 11-25
static void synaptics_rmi4_sensor_report(struct synaptics_rmi4_data *rmi4_data)
#else //huawei 11-25
static void synaptics_rmi4_sensor_report(struct work_struct *work)
#endif //huawei 11-25
{
    int retval;
    unsigned char data[MAX_INTR_REGISTERS + 1];
    unsigned char *intr = &data[1];
    struct synaptics_rmi4_f01_device_status status;
    struct synaptics_rmi4_fn *fhandler;
    struct synaptics_rmi4_exp_fhandler *exp_fhandler;
    struct synaptics_rmi4_device_info *rmi;
#if USE_IRQ_THREAD //huawei 11-25
#else //huawei 11-25
    struct synaptics_rmi4_data *rmi4_data = NULL;
    rmi4_data = container_of(work,struct synaptics_rmi4_data, work);
#endif //huawei 11-25

    tp_log_debug("%s#%d: in!\n",__func__,__LINE__);
    rmi = &(rmi4_data->rmi4_mod_info);

    /*
     * Get interrupt status information from F01 Data1 register to
     * determine the source(s) that are flagging the interrupt.
     */
    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f01_data_base_addr,
            data,rmi4_data->num_of_intr_regs + 1);

    tp_log_debug("%s#%d: data[0]=%d,data[1]=%d,retval = %d \n",
        __func__,__LINE__,data[0],data[1],retval);

    if (retval < 0) {
        tp_log_err("%s#%d: Read interrupt status fail!\n",__func__,__LINE__);
        goto REPORT_ERROR;
    }

    tp_log_debug("%s:rmi4_data->f01_data_base_addr=%d,"
        "rmi4_data->num_of_intr_regs=%d  \n",
        __func__,rmi4_data->f01_data_base_addr,rmi4_data->num_of_intr_regs);

    status.data[0] = data[0];
    if (status.unconfigured && !status.flash_prog) {
        tp_log_warning("%s: spontaneous reset detected\n", __func__);
        retval = synaptics_rmi4_reinit_device(rmi4_data);
        if (retval < 0) {
            tp_log_err("%s#%d: Reinit devcie fail!\n",__func__,__LINE__);
        }
        goto REPORT_ERROR;
    }

    /*
     * Traverse the function handler list and service the source(s)
     * of the interrupt accordingly.
     */
    if (!list_empty(&rmi->support_fn_list)) {
        list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
            if (fhandler->num_of_data_sources) {
                if (fhandler->intr_mask & intr[fhandler->intr_reg_num]) {
                    tp_log_debug("%s#%d: service irq of fn_number=%d! \n",
                        __func__,__LINE__,fhandler->fn_number);
                    synaptics_rmi4_report_touch(rmi4_data,fhandler);
                }
            }
        }
    }

    mutex_lock(&exp_data.mutex);
    if (!list_empty(&exp_data.list)) {
        list_for_each_entry(exp_fhandler, &exp_data.list, link) {
            if (!exp_fhandler->insert &&
                    !exp_fhandler->remove &&
                    (exp_fhandler->exp_fn->attn != NULL))
                exp_fhandler->exp_fn->attn(rmi4_data, intr[0]);
                tp_log_debug("%s#%d: service irq of fn_type=%d! \n",
                    __func__,__LINE__,exp_fhandler->exp_fn->fn_type);
        }
    }
    mutex_unlock(&exp_data.mutex);

REPORT_ERROR:
    if (rmi4_data->irq_enabled)
    {
        tp_log_debug("%s#%d: rq_enabled!  \n",__func__,__LINE__);
        enable_irq(rmi4_data->irq);
    }
    
    return;
}

/* the function for processing the easy wakeup work */
static void synaptics_rmi4_sensor_report_delay_work(struct work_struct *work)
{

    struct synaptics_rmi4_data *rmi4_data = NULL;
    rmi4_data = container_of(work,struct synaptics_rmi4_data, delay_work.work);

    tp_log_debug("%s#%d: in\n",__func__,__LINE__);
    synaptics_rmi4_sensor_report( rmi4_data );

    /* easy wakeup process finish */
    easy_wakeup_process = false;

    return;
}
 static void synaptics_edge_glove_to_finger_delay_work(struct work_struct *work)
{
    int retval = 0;
    struct synaptics_rmi4_data *rmi4_data = NULL;
    rmi4_data = container_of(work, struct synaptics_rmi4_data, glove_delay_work.work);

    /*disable glove mode to finger mode on the edge of TP*/
    retval = set_edge_glove_state(rmi4_data, true);
    if(retval < 0)
    {
        tp_log_err("%s: Failed to disable edge glove state\n",
        __func__);
    }
    edge_glove_switch_process = false;

    return;
}
 /**
 * synaptics_rmi4_irq()
 *
 * Called by the kernel when an interrupt occurs (when the sensor
 * asserts the attention irq).
 *
 * This function is the ISR thread and handles the acquisition
 * and the reporting of finger data when the presence of fingers
 * is detected.
 */
static irqreturn_t synaptics_rmi4_irq(int irq, void *data)
{
    struct synaptics_rmi4_data *rmi4_data = data;

    tp_log_debug("%s:in,(line%d): touch_stopped=%d\n",__func__,__LINE__,rmi4_data->touch_stopped);
    if (rmi4_data->board->esd_support){
        synaptics_dsx_esd_suspend();
    }
    if (!rmi4_data->touch_stopped)
#if USE_IRQ_THREAD //huawei 11-25
    /* if the irq come from easy wakeup, then schedule a delay work to process it */
    /* just when easy wakeup process is not running, schedule the easy wakeup work */
    {
        if((false != rmi4_data->easy_wakeup_gesture)
            && (true == rmi4_data->sleep_gesture_flag))
        {
            if( false == easy_wakeup_process )
            {
                disable_irq_nosync(irq);
                /* indicate the easy wakeup process is running */
                easy_wakeup_process = true;
                schedule_delayed_work(&rmi4_data->delay_work, msecs_to_jiffies(GENSTRUE_WORK_DELAY));
            }
        }
        else
        {
            disable_irq_nosync(irq);
            synaptics_rmi4_sensor_report(rmi4_data);
        }

    }
#else //huawei 11-25
    {
        disable_irq_nosync(irq);
        queue_work(synaptics_wq, &rmi4_data->work);
    }
#endif //huawei 11-25
    if (rmi4_data->board->esd_support){
        synaptics_dsx_esd_resume();
    }

    return IRQ_HANDLED;
}

 /**
 * synaptics_rmi4_irq_enable()
 *
 * Called by synaptics_rmi4_probe() and the power management functions
 * in this driver and also exported to other expansion Function modules
 * such as rmi_dev.
 *
 * This function handles the enabling and disabling of the attention
 * irq including the setting up of the ISR thread.
 */
static int synaptics_rmi4_irq_enable(struct synaptics_rmi4_data *rmi4_data,
        bool enable)
{
    int retval = 0;
    unsigned char intr_status[MAX_INTR_REGISTERS];
    //const struct synaptics_dsx_platform_data *platform_data =
    //        rmi4_data->i2c_client->dev.platform_data;

    const struct synaptics_dsx_platform_data *platform_data =
            rmi4_data->board;
    tp_log_debug("%s: enable = %d,irq_enabled = %d \n",__func__,enable,rmi4_data->irq_enabled);
    if (enable) {
        if (rmi4_data->irq_enabled)
            return retval;

        /* Clear interrupts first */
        retval = synaptics_rmi4_i2c_read(rmi4_data,
                rmi4_data->f01_data_base_addr + 1,
                intr_status,
                rmi4_data->num_of_intr_regs);

        tp_log_debug("%s: intr_status[0] = %d ,retval = %d \n",__func__,intr_status[0],retval);
        if (retval < 0)
            return retval;

        retval = request_threaded_irq(rmi4_data->irq, NULL,
                synaptics_rmi4_irq, platform_data->irq_flags,
                DRIVER_NAME, rmi4_data);
        tp_log_debug("%s: request_threaded_irq retval= %d \n",__func__,retval);
        if (retval < 0) {
            dev_err(&rmi4_data->i2c_client->dev,
                    "%s: Failed to create irq thread\n",
                    __func__);
            return retval;
        }

        /* Clear interrupts first */
        retval = synaptics_rmi4_i2c_read(rmi4_data,
                rmi4_data->f01_data_base_addr + 1,
                intr_status,
                rmi4_data->num_of_intr_regs);
        tp_log_debug("%s: intr_status[0] = %d ,retval = %d \n",__func__,intr_status[0],retval);
        if (retval < 0)
            return retval;

        rmi4_data->irq_enabled = true;
    } else {
        if (rmi4_data->irq_enabled) {
            disable_irq(rmi4_data->irq);
            free_irq(rmi4_data->irq, rmi4_data);
            rmi4_data->irq_enabled = false;
            /* Clear interrupts first */
            retval = synaptics_rmi4_i2c_read(rmi4_data,
                    rmi4_data->f01_data_base_addr + 1,
                    intr_status,
                    rmi4_data->num_of_intr_regs);
            tp_log_debug("%s: intr_status[0] = %d ,retval = %d \n",__func__,intr_status[0],retval);
            if (retval < 0)
                return retval;
        }
    }

    return retval;
}

static void synaptics_rmi4_set_intr_mask(struct synaptics_rmi4_fn *fhandler,
        struct synaptics_rmi4_fn_desc *fd,
        unsigned int intr_count)
{
    unsigned char ii;
    unsigned char intr_offset;

    fhandler->intr_reg_num = (intr_count + 7) / 8;
    if (fhandler->intr_reg_num != 0)
        fhandler->intr_reg_num -= 1;

    /* Set an enable bit for each data source */
    intr_offset = intr_count % 8;
    fhandler->intr_mask = 0;
    for (ii = intr_offset;
            ii < ((fd->intr_src_count & MASK_3BIT) +
            intr_offset);
            ii++)
        fhandler->intr_mask |= 1 << ii;

    return;
}

static int synaptics_rmi4_f01_init(struct synaptics_rmi4_data *rmi4_data,
        struct synaptics_rmi4_fn *fhandler,
        struct synaptics_rmi4_fn_desc *fd,
        unsigned int intr_count)
{
    fhandler->fn_number = fd->fn_number;
    fhandler->num_of_data_sources = fd->intr_src_count;
    fhandler->data = NULL;
    fhandler->extra = NULL;

    synaptics_rmi4_set_intr_mask(fhandler, fd, intr_count);

    rmi4_data->f01_query_base_addr = fd->query_base_addr;
    rmi4_data->f01_ctrl_base_addr = fd->ctrl_base_addr;
    rmi4_data->f01_data_base_addr = fd->data_base_addr;
    rmi4_data->f01_cmd_base_addr = fd->cmd_base_addr;

    return 0;
}

 /**
 * synaptics_rmi4_f11_init()
 *
 * Called by synaptics_rmi4_query_device().
 *
 * This funtion parses information from the Function 11 registers
 * and determines the number of fingers supported, x and y data ranges,
 * offset to the associated interrupt status register, interrupt bit
 * mask, and gathers finger data acquisition capabilities from the query
 * registers.
 */
static int synaptics_rmi4_f11_init(struct synaptics_rmi4_data *rmi4_data,
        struct synaptics_rmi4_fn *fhandler,
        struct synaptics_rmi4_fn_desc *fd,
        unsigned int intr_count)
{
    int retval;
    unsigned char abs_data_size;
    unsigned char abs_data_blk_size;
    unsigned char query[F11_STD_QUERY_LEN];
    unsigned char control[F11_STD_CTRL_LEN];

    fhandler->fn_number = fd->fn_number;
    fhandler->num_of_data_sources = fd->intr_src_count;

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            fhandler->full_addr.query_base,
            query,
            sizeof(query));
    if (retval < 0)
        return retval;

    /* Maximum number of fingers supported */
    if ((query[1] & MASK_3BIT) <= 4)
        fhandler->num_of_data_points = (query[1] & MASK_3BIT) + 1;
    else if ((query[1] & MASK_3BIT) == 5)
        fhandler->num_of_data_points = 10;

    rmi4_data->num_of_fingers = fhandler->num_of_data_points;

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            fhandler->full_addr.ctrl_base,
            control,
            sizeof(control));
    if (retval < 0)
        return retval;

    /* Maximum x and y */
    rmi4_data->sensor_max_x = ((control[6] & MASK_8BIT) << 0) |
            ((control[7] & MASK_4BIT) << 8);
    rmi4_data->sensor_max_y = ((control[8] & MASK_8BIT) << 0) |
            ((control[9] & MASK_4BIT) << 8);
    dev_dbg(&rmi4_data->i2c_client->dev,
            "%s: Function %02x max x = %d max y = %d\n",
            __func__, fhandler->fn_number,
            rmi4_data->sensor_max_x,
            rmi4_data->sensor_max_y);

    rmi4_data->max_touch_width = MAX_F11_TOUCH_WIDTH;

    synaptics_rmi4_set_intr_mask(fhandler, fd, intr_count);

    abs_data_size = query[5] & MASK_2BIT;
    abs_data_blk_size = 3 + (2 * (abs_data_size == 0 ? 1 : 0));
    fhandler->size_of_data_register_block = abs_data_blk_size;
    fhandler->data = NULL;
    fhandler->extra = NULL;

    rmi4_data->f11_query_base_addr = fd->query_base_addr;
    rmi4_data->f11_ctrl_base_addr = fd->ctrl_base_addr;
    rmi4_data->f11_data_base_addr = fd->data_base_addr;
    rmi4_data->f11_cmd_base_addr = fd->cmd_base_addr;

    return retval;
}

static int synaptics_rmi4_f12_set_enables(struct synaptics_rmi4_data *rmi4_data,
        unsigned short ctrl28)
{
    int retval;
    static unsigned short ctrl_28_address;

    if (ctrl28)
        ctrl_28_address = ctrl28;

    retval = synaptics_rmi4_i2c_write(rmi4_data,
            ctrl_28_address,
            &rmi4_data->report_enable,
            sizeof(rmi4_data->report_enable));
    if (retval < 0)
        return retval;

    return retval;
}

 /**
 * synaptics_rmi4_f12_init()
 *
 * Called by synaptics_rmi4_query_device().
 *
 * This funtion parses information from the Function 12 registers and
 * determines the number of fingers supported, offset to the data1
 * register, x and y data ranges, offset to the associated interrupt
 * status register, interrupt bit mask, and allocates memory resources
 * for finger data acquisition.
 */
static int synaptics_rmi4_f12_init(struct synaptics_rmi4_data *rmi4_data,
        struct synaptics_rmi4_fn *fhandler,
        struct synaptics_rmi4_fn_desc *fd,
        unsigned int intr_count)
{
    int retval;
    unsigned char size_of_2d_data;
    unsigned char size_of_query8;
    unsigned char ctrl_8_offset;
    unsigned char ctrl_20_offset;
    unsigned char ctrl_22_offset;
    unsigned char ctrl_23_offset;
    unsigned char ctrl_26_offset;
    unsigned char ctrl_28_offset;
    unsigned char data_04_offset;
    unsigned char ctrl_31_offset;
    unsigned char num_of_fingers;
    struct synaptics_rmi4_f12_extra_data *extra_data;
    struct synaptics_rmi4_f12_query_5 query_5;
    struct synaptics_rmi4_f12_query_8 query_8;
    struct synaptics_rmi4_f12_ctrl_8 ctrl_8;
    struct synaptics_rmi4_f12_ctrl_23 ctrl_23;
    struct synaptics_rmi4_f12_ctrl_31 ctrl_31;

    fhandler->fn_number = fd->fn_number;
    fhandler->num_of_data_sources = fd->intr_src_count;
    fhandler->extra = kmalloc(sizeof(*extra_data), GFP_KERNEL);
    extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
    size_of_2d_data = sizeof(struct synaptics_rmi4_f12_finger_data);

    rmi4_data->f12_ctrl_base_addr = fd->ctrl_base_addr;
    rmi4_data->f12_data_base_addr = fd->data_base_addr;
    tp_log_info("%s:f12_ctrl_base_addr(%d); f12_data_base_addr(%d)\n",
            __func__, rmi4_data->f12_ctrl_base_addr, rmi4_data->f12_data_base_addr);
    
    retval = synaptics_rmi4_i2c_read(rmi4_data,
            fhandler->full_addr.query_base + 5,
            query_5.data,
            sizeof(query_5.data));
    if (retval < 0)
        return retval;

    ctrl_8_offset = query_5.ctrl0_is_present +
            query_5.ctrl1_is_present +
            query_5.ctrl2_is_present +
            query_5.ctrl3_is_present +
            query_5.ctrl4_is_present +
            query_5.ctrl5_is_present +
            query_5.ctrl6_is_present +
            query_5.ctrl7_is_present;

    ctrl_20_offset = ctrl_8_offset +
            query_5.ctrl8_is_present +
            query_5.ctrl9_is_present +
            query_5.ctrl10_is_present +
            query_5.ctrl11_is_present +
            query_5.ctrl12_is_present +
            query_5.ctrl13_is_present +
            query_5.ctrl14_is_present +
            query_5.ctrl15_is_present +
            query_5.ctrl16_is_present +
            query_5.ctrl17_is_present +
            query_5.ctrl18_is_present +
            query_5.ctrl19_is_present;
    rmi4_data->f12_2d_ctrl20_lpm = ctrl_20_offset;

    ctrl_22_offset = ctrl_20_offset +
            query_5.ctrl20_is_present +
            query_5.ctrl21_is_present;
    rmi4_data->f12_2d_ctrl22_palm = ctrl_22_offset;

    ctrl_23_offset = ctrl_22_offset +
            query_5.ctrl22_is_present;
    rmi4_data->f12_2d_ctrl23_palm = ctrl_23_offset;

    ctrl_26_offset = ctrl_23_offset +
            query_5.ctrl23_is_present +
            query_5.ctrl24_is_present +
            query_5.ctrl25_is_present;
    rmi4_data->f12_2d_ctrl26_glove = ctrl_26_offset;

    ctrl_28_offset = ctrl_26_offset +
            query_5.ctrl26_is_present +
            query_5.ctrl27_is_present;

    ctrl_31_offset = ctrl_28_offset +
            query_5.ctrl28_is_present +
            query_5.ctrl29_is_present +
            query_5.ctrl30_is_present;

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            fhandler->full_addr.ctrl_base + ctrl_23_offset,
            ctrl_23.data,
            sizeof(ctrl_23.data));
    if (retval < 0)
        return retval;

    /* Maximum number of fingers supported */
    fhandler->num_of_data_points = min(ctrl_23.max_reported_objects,
            (unsigned char)F12_FINGERS_TO_SUPPORT);

    num_of_fingers = fhandler->num_of_data_points;
    rmi4_data->num_of_fingers = num_of_fingers;

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            fhandler->full_addr.query_base + 7,
            &size_of_query8,
            sizeof(size_of_query8));
    if (retval < 0)
        return retval;

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            fhandler->full_addr.query_base + 8,
            query_8.data,
            size_of_query8);
    if (retval < 0)
        return retval;

    /* Determine the presence of the Data0 register */
    extra_data->data1_offset = query_8.data0_is_present;

    if ((size_of_query8 >= 3) && (query_8.data15_is_present)) {
        extra_data->data15_offset = query_8.data0_is_present +
                query_8.data1_is_present +
                query_8.data2_is_present +
                query_8.data3_is_present +
                query_8.data4_is_present +
                query_8.data5_is_present +
                query_8.data6_is_present +
                query_8.data7_is_present +
                query_8.data8_is_present +
                query_8.data9_is_present +
                query_8.data10_is_present +
                query_8.data11_is_present +
                query_8.data12_is_present +
                query_8.data13_is_present +
                query_8.data14_is_present;
        extra_data->data15_size = (num_of_fingers + 7) / 8;
    } else {
        extra_data->data15_size = 0;
    }
    data_04_offset = query_8.data0_is_present +
                query_8.data1_is_present +
                query_8.data2_is_present +
                query_8.data3_is_present;
    rmi4_data->f12_2d_data04_gesture = data_04_offset;

    rmi4_data->report_enable = RPT_DEFAULT;
#ifdef REPORT_2D_Z
    rmi4_data->report_enable |= RPT_Z;
#endif
#ifdef REPORT_2D_W
    rmi4_data->report_enable |= (RPT_WX | RPT_WY);
#endif

    retval = synaptics_rmi4_f12_set_enables(rmi4_data,
            fhandler->full_addr.ctrl_base + ctrl_28_offset);
    if (retval < 0)
        return retval;

    tp_log_info("%s,line=%d,query_5.ctrl8_is_present=%d,ctrl_8_offset=%d \n",
        __func__,__LINE__,query_5.ctrl8_is_present,ctrl_8_offset);
    if (query_5.ctrl8_is_present) {
        rmi4_data->wedge_sensor = false;

        retval = synaptics_rmi4_i2c_read(rmi4_data,
                fhandler->full_addr.ctrl_base + ctrl_8_offset,
                ctrl_8.data,
                sizeof(ctrl_8.data));
        if (retval < 0)
            return retval;

        /* Maximum x and y */
        rmi4_data->sensor_max_x =
            ((unsigned short)ctrl_8.max_x_coord_lsb << 0) |
            ((unsigned short)ctrl_8.max_x_coord_msb << 8);
        rmi4_data->sensor_max_y =
            ((unsigned short)ctrl_8.max_y_coord_lsb << 0) |
            ((unsigned short)ctrl_8.max_y_coord_msb << 8);

        rmi4_data->max_touch_width = MAX_F12_TOUCH_WIDTH;
    } else {
        rmi4_data->wedge_sensor = true;
        tp_log_info("%s,line=%d \n",__func__,__LINE__);
        retval = synaptics_rmi4_i2c_read(rmi4_data,
                fhandler->full_addr.ctrl_base + ctrl_31_offset,
                ctrl_31.data,
                sizeof(ctrl_31.data));
        if (retval < 0)
            return retval;

        /* Maximum x and y */
        rmi4_data->sensor_max_x =
                ((unsigned short)ctrl_31.max_x_coord_lsb << 0) |
                ((unsigned short)ctrl_31.max_x_coord_msb << 8);
        rmi4_data->sensor_max_y =
                ((unsigned short)ctrl_31.max_y_coord_lsb << 0) |
                ((unsigned short)ctrl_31.max_y_coord_msb << 8);

        rmi4_data->max_touch_width = MAX_F12_TOUCH_WIDTH;
    }
    tp_log_debug("%s,%d ctrl_31_offset=0x%x,ctrl_base=0x%x\n" \
        "rmi4_data->sensor_max_x=%d,rmi4_data->sensor_max_y=%d\n", \
        __func__,__LINE__,ctrl_31_offset,fhandler->full_addr.ctrl_base, \
        rmi4_data->sensor_max_x,rmi4_data->sensor_max_y);

    dev_dbg(&rmi4_data->i2c_client->dev,
            "%s: Function %02x max x = %d max y = %d\n",
            __func__, fhandler->fn_number,
            rmi4_data->sensor_max_x,
            rmi4_data->sensor_max_y);

    /*rmi4_data->num_of_rx = ctrl_8.num_of_rx;
    rmi4_data->num_of_tx = ctrl_8.num_of_tx;
    rmi4_data->max_touch_width = max(rmi4_data->num_of_rx,
            rmi4_data->num_of_tx);*/

    synaptics_rmi4_set_intr_mask(fhandler, fd, intr_count);

    /* Allocate memory for finger data storage space */
    fhandler->data_size = num_of_fingers * size_of_2d_data;
    fhandler->data = kmalloc(fhandler->data_size, GFP_KERNEL);

    return retval;
}

static int synaptics_rmi4_f1a_alloc_mem(struct synaptics_rmi4_data *rmi4_data,
        struct synaptics_rmi4_fn *fhandler)
{
    int retval;
    struct synaptics_rmi4_f1a_handle *f1a;

    f1a = kzalloc(sizeof(*f1a), GFP_KERNEL);
    if (!f1a) {
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: Failed to alloc mem for function handle\n",
                __func__);
        return -ENOMEM;
    }

    fhandler->data = (void *)f1a;
    fhandler->extra = NULL;

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            fhandler->full_addr.query_base,
            f1a->button_query.data,
            sizeof(f1a->button_query.data));
    if (retval < 0) {
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: Failed to read query registers\n",
                __func__);
        return retval;
    }

    f1a->max_count = f1a->button_query.max_button_count + 1;

    f1a->button_control.txrx_map = kzalloc(f1a->max_count * 2, GFP_KERNEL);
    if (!f1a->button_control.txrx_map) {
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: Failed to alloc mem for tx rx mapping\n",
                __func__);
        return -ENOMEM;
    }

    f1a->button_bitmask_size = (f1a->max_count + 7) / 8;

    f1a->button_data_buffer = kcalloc(f1a->button_bitmask_size,
            sizeof(*(f1a->button_data_buffer)), GFP_KERNEL);
    if (!f1a->button_data_buffer) {
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: Failed to alloc mem for data buffer\n",
                __func__);
        return -ENOMEM;
    }

    f1a->button_map = kcalloc(f1a->max_count,
            sizeof(*(f1a->button_map)), GFP_KERNEL);
    if (!f1a->button_map) {
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: Failed to alloc mem for button map\n",
                __func__);
        return -ENOMEM;
    }

    return 0;
}

static int synaptics_rmi4_f1a_button_map(struct synaptics_rmi4_data *rmi4_data,
        struct synaptics_rmi4_fn *fhandler)
{
    int retval;
    unsigned char ii;
    unsigned char mapping_offset = 0;
    struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;
    const struct synaptics_dsx_platform_data *pdata = rmi4_data->board;

    mapping_offset = f1a->button_query.has_general_control +
            f1a->button_query.has_interrupt_enable +
            f1a->button_query.has_multibutton_select;

    if (f1a->button_query.has_tx_rx_map) {
        retval = synaptics_rmi4_i2c_read(rmi4_data,
                fhandler->full_addr.ctrl_base + mapping_offset,
                f1a->button_control.txrx_map,
                sizeof(f1a->button_control.txrx_map));
        if (retval < 0) {
            dev_err(&rmi4_data->i2c_client->dev,
                    "%s: Failed to read tx rx mapping\n",
                    __func__);
            return retval;
        }

        rmi4_data->button_txrx_mapping = f1a->button_control.txrx_map;
    }

    if (!pdata->cap_button_map) {
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: cap_button_map is NULL in board file\n",
                __func__);
        //return -ENODEV;
    } else if (!pdata->cap_button_map->map) {
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: Button map is missing in board file\n",
                __func__);
        //return -ENODEV;
    } else {
        if (pdata->cap_button_map->nbuttons != f1a->max_count) {
            f1a->valid_button_count = min(f1a->max_count,
                    pdata->cap_button_map->nbuttons);
        } else {
            f1a->valid_button_count = f1a->max_count;
        }

        for (ii = 0; ii < f1a->valid_button_count; ii++)
            f1a->button_map[ii] = pdata->cap_button_map->map[ii];
    }

    return 0;
}

static void synaptics_rmi4_f1a_kfree(struct synaptics_rmi4_fn *fhandler)
{
    struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;

    if (f1a) {
        kfree(f1a->button_control.txrx_map);
        kfree(f1a->button_data_buffer);
        kfree(f1a->button_map);
        kfree(f1a);
        fhandler->data = NULL;
    }

    return;
}

static int synaptics_rmi4_f1a_init(struct synaptics_rmi4_data *rmi4_data,
        struct synaptics_rmi4_fn *fhandler,
        struct synaptics_rmi4_fn_desc *fd,
        unsigned int intr_count)
{
    int retval;

    fhandler->fn_number = fd->fn_number;
    fhandler->num_of_data_sources = fd->intr_src_count;

    synaptics_rmi4_set_intr_mask(fhandler, fd, intr_count);

    retval = synaptics_rmi4_f1a_alloc_mem(rmi4_data, fhandler);
    if (retval < 0)
        goto error_exit;

    retval = synaptics_rmi4_f1a_button_map(rmi4_data, fhandler);
    if (retval < 0)
        goto error_exit;

    rmi4_data->button_0d_enabled = 1;

    return 0;

error_exit:
    synaptics_rmi4_f1a_kfree(fhandler);

    return retval;
}

static void synaptics_rmi4_empty_fn_list(struct synaptics_rmi4_data *rmi4_data)
{
    struct synaptics_rmi4_fn *fhandler;
    struct synaptics_rmi4_fn *fhandler_temp;
    struct synaptics_rmi4_device_info *rmi;

    rmi = &(rmi4_data->rmi4_mod_info);

    if (!list_empty(&rmi->support_fn_list)) {
        list_for_each_entry_safe(fhandler,
                fhandler_temp,
                &rmi->support_fn_list,
                link) {
            if (fhandler->fn_number == SYNAPTICS_RMI4_F1A) {
                synaptics_rmi4_f1a_kfree(fhandler);
            } else {
                kfree(fhandler->extra);
                kfree(fhandler->data);
            }
            list_del(&fhandler->link);
            kfree(fhandler);
        }
    }
    INIT_LIST_HEAD(&rmi->support_fn_list);

    return;
}

static int synaptics_rmi4_check_status(struct synaptics_rmi4_data *rmi4_data,
        bool *was_in_bl_mode)
{
    int retval;
    int timeout = CHECK_STATUS_TIMEOUT_MS;
    unsigned char command = 0x01;
    unsigned char intr_status;
    struct synaptics_rmi4_f01_device_status status;

    /* Do a device reset first */
    retval = synaptics_rmi4_i2c_write(rmi4_data,
            rmi4_data->f01_cmd_base_addr,
            &command,
            sizeof(command));
    if (retval < 0)
        return retval;

    msleep(rmi4_data->board->reset_delay_ms);

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f01_data_base_addr,
            status.data,
            sizeof(status.data));
    if (retval < 0)
        return retval;

    while (status.status_code == STATUS_CRC_IN_PROGRESS) {
        if (timeout > 0)
            msleep(20);
        else
            return -1;

        retval = synaptics_rmi4_i2c_read(rmi4_data,
                rmi4_data->f01_data_base_addr,
                status.data,
                sizeof(status.data));
        if (retval < 0)
            return retval;

        timeout -= 20;
    }

    if (timeout != CHECK_STATUS_TIMEOUT_MS)
        *was_in_bl_mode = true;

    if (status.flash_prog == 1) {
        rmi4_data->flash_prog_mode = true;
        tp_log_info("%s: In flash prog mode, status = 0x%02x\n",
                __func__,status.status_code);
    } else {
        rmi4_data->flash_prog_mode = false;
    }

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f01_data_base_addr + 1,
            &intr_status,
            sizeof(intr_status));
    tp_log_debug("%s: intr_status = %d ,retval = %d \n",__func__,intr_status,retval);
    if (retval < 0) {
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: Failed to read interrupt status\n",
                __func__);
        return retval;
    }

    return 0;
}

/* Get doze wakeup threshold of gesture for different modules */
static int synaptics_get_gesture_doze_wakeup_threshold(struct synaptics_rmi4_data *rmi4_data)
{
    int len = 0;
    unsigned char module_threshold = 0;
    struct synaptics_rmi4_device_info *rmi = &(rmi4_data->rmi4_mod_info);
    struct synaptics_dsx_platform_data *pdata = rmi4_data->board;
    char *product_id = rmi->product_id_string;

    if(NULL == product_id)
    {
        tp_log_err("%s: product_id = NULL ,LINE = %d\n", __func__,__LINE__);
        return -1;
    }
    len = strlen(product_id);

    /*To get the last three characters of product_id , if the length of product id is longer than three */
    if (len > MODULE_STR_LEN) {
        product_id += (len - MODULE_STR_LEN);
        tp_log_info("%s: the last three characters of product_id = %s\n", __func__,product_id);
    } else {
        tp_log_err("%s: product_id too short,len = %d\n", __func__,len);
        return -1;
    }

    /* To get the module-dwt according to the product_id */
    if (0 == strcasecmp(product_id, FW_OFILM_STR))
    {
        module_threshold = pdata->ofilm_dwt;
        tp_log_info("%s: ofilm_dwt = %d\n", __func__,module_threshold);
    }
    else if(0 == strcasecmp(product_id, FW_EELY_STR))
    {
        module_threshold = pdata->eely_dwt;
        tp_log_info("%s: eely_dwt = %d\n", __func__,module_threshold);
    }
    else if(0 == strcasecmp(product_id, FW_TRULY_STR))
    {
        module_threshold = pdata->truly_dwt;
        tp_log_info("%s: truly_dwt = %d\n", __func__,module_threshold);
    }
    else if(0 == strcasecmp(product_id, FW_JUNDA_STR))
    {
        module_threshold = pdata->junda_dwt;
        tp_log_info("%s: junda_dwt = %d\n", __func__,module_threshold);
    }
    else if(0 == strcasecmp(product_id, FW_LENSONE_STR))
    {
        module_threshold = pdata->lensone_dwt;
        tp_log_info("%s: lensone_dwt = %d\n", __func__,module_threshold);
    }
    else
    {
        tp_log_err("%s: unknow product_id = %s\n", __func__,product_id);
        return -1;
    }

    /* Record the module-dwt for gesture */
    rmi4_data->dwt_gesture = module_threshold;
    return 0;
}

static void synaptics_rmi4_set_configured(struct synaptics_rmi4_data *rmi4_data)
{
    int retval;
    unsigned char device_ctrl;

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f01_ctrl_base_addr,
            &device_ctrl,
            sizeof(device_ctrl));
    if (retval < 0) {
        tp_log_err("%s#%d: Failed to set configured\n",__func__,__LINE__);
        return;
    }

    rmi4_data->no_sleep_setting = device_ctrl & NO_SLEEP_ON;
    device_ctrl |= CONFIGURED;

    retval = synaptics_rmi4_i2c_write(rmi4_data,
            rmi4_data->f01_ctrl_base_addr,
            &device_ctrl,
            sizeof(device_ctrl));
    if (retval < 0) {
        tp_log_err("%s#%d: Failed to set configured\n",__func__,__LINE__);
        return;
    }

    /* F01_RMI_CTRL03:Doze Wakeup Threshold*/
    if(0 == rmi4_data->dwt_default) {
        retval = synaptics_rmi4_i2c_read(rmi4_data,
                rmi4_data->f01_ctrl_base_addr + F01_RMI_CTRL03_OFFSET,
                &device_ctrl,
                sizeof(device_ctrl));
        tp_log_debug("%s(line%d): Doze Wakeup Threshold = %d\n",__func__,__LINE__,device_ctrl);
        if (retval < 0) {
            tp_log_err("%s#%d: Failed to set configured\n",__func__,__LINE__);
            return;
        }
        /* Record the default-dwt */
        rmi4_data->dwt_default = device_ctrl;
    }

    return;
}

static int synaptics_rmi4_alloc_fh(struct synaptics_rmi4_fn **fhandler,
        struct synaptics_rmi4_fn_desc *rmi_fd, int page_number)
{
    *fhandler = kmalloc(sizeof(**fhandler), GFP_KERNEL);
    if (!(*fhandler))
        return -ENOMEM;

    (*fhandler)->full_addr.data_base =
            (rmi_fd->data_base_addr |
            (page_number << 8));
    (*fhandler)->full_addr.ctrl_base =
            (rmi_fd->ctrl_base_addr |
            (page_number << 8));
    (*fhandler)->full_addr.cmd_base =
            (rmi_fd->cmd_base_addr |
            (page_number << 8));
    (*fhandler)->full_addr.query_base =
            (rmi_fd->query_base_addr |
            (page_number << 8));

    return 0;
}

 static int synaptics_rmi4_f51_init(struct synaptics_rmi4_data *rmi4_data,
         struct synaptics_rmi4_fn_desc *fd,
         unsigned char page)
 {
     rmi4_data->f51_query_base_addr = fd->query_base_addr | (page << 8);
     rmi4_data->f51_ctrl_base_addr = fd->ctrl_base_addr | (page << 8);
     rmi4_data->f51_data_base_addr = fd->data_base_addr | (page << 8);
     rmi4_data->f51_cmd_base_addr = fd->cmd_base_addr | (page << 8);

     tp_log_debug("%s(line%d): rmi4_data->f51_query_base_addr = 0x%x \n",
            __func__,__LINE__,rmi4_data->f51_query_base_addr);
     tp_log_debug("%s(line%d): rmi4_data->f51_ctrl_base_addr = 0x%x \n",
            __func__,__LINE__,rmi4_data->f51_ctrl_base_addr);
     tp_log_debug("%s(line%d): rmi4_data->f51_data_base_addr = 0x%x \n",
            __func__,__LINE__,rmi4_data->f51_data_base_addr);
     tp_log_debug("%s(line%d): rmi4_data->f51_cmd_base_addr = 0x%x \n",
            __func__,__LINE__,rmi4_data->f51_cmd_base_addr);
 
     return 0;
 }

static int synaptics_rmi4_f54_init(struct synaptics_rmi4_data *rmi4_data,
        struct synaptics_rmi4_fn_desc *fd,
        unsigned char page)
{
    rmi4_data->f54_query_base_addr = fd->query_base_addr | (page << 8);
    rmi4_data->f54_ctrl_base_addr = fd->ctrl_base_addr | (page << 8);
    rmi4_data->f54_data_base_addr = fd->data_base_addr | (page << 8);
    rmi4_data->f54_cmd_base_addr = fd->cmd_base_addr | (page << 8);

    tp_log_debug("%s(line%d): rmi4_data->f54_query_base_addr = 0x%x \n",
            __func__,__LINE__,rmi4_data->f54_query_base_addr);
    tp_log_debug("%s(line%d): rmi4_data->f54_ctrl_base_addr = 0x%x \n",
            __func__,__LINE__,rmi4_data->f54_ctrl_base_addr);
    tp_log_debug("%s(line%d): rmi4_data->f54_data_base_addr = 0x%x \n",
            __func__,__LINE__,rmi4_data->f54_data_base_addr);
    tp_log_debug("%s(line%d): rmi4_data->f54_cmd_base_addr = 0x%x \n",
            __func__,__LINE__,rmi4_data->f54_cmd_base_addr);

    return 0;
}

 /**
 * synaptics_rmi4_query_device()
 *
 * Called by synaptics_rmi4_probe().
 *
 * This funtion scans the page description table, records the offsets
 * to the register types of Function $01, sets up the function handlers
 * for Function $11 and Function $12, determines the number of interrupt
 * sources from the sensor, adds valid Functions with data inputs to the
 * Function linked list, parses information from the query registers of
 * Function $01, and enables the interrupt sources from the valid Functions
 * with data inputs.
 */
static int synaptics_rmi4_query_device(struct synaptics_rmi4_data *rmi4_data)
{
    int retval;
    unsigned char ii;
    unsigned char page_number;
    unsigned char intr_count;
    unsigned char f01_query[F01_STD_QUERY_LEN];
    unsigned short pdt_entry_addr;
    unsigned short intr_addr;
    bool was_in_bl_mode;
    struct synaptics_rmi4_fn_desc rmi_fd;
    struct synaptics_rmi4_fn *fhandler;
    struct synaptics_rmi4_device_info *rmi;

    rmi = &(rmi4_data->rmi4_mod_info);

rescan_pdt:
    was_in_bl_mode = false;
    intr_count = 0;
    INIT_LIST_HEAD(&rmi->support_fn_list);

    /* Scan the page description tables of the pages to service */
    for (page_number = 0; page_number < PAGES_TO_SERVICE; page_number++) {
        for (pdt_entry_addr = PDT_START; pdt_entry_addr > PDT_END;
                pdt_entry_addr -= PDT_ENTRY_SIZE) {
            pdt_entry_addr |= (page_number << 8);

            retval = synaptics_rmi4_i2c_read(rmi4_data,
                    pdt_entry_addr,(unsigned char *)&rmi_fd,sizeof(rmi_fd));
            if (retval < 0)
                return retval;

            fhandler = NULL;

            if (rmi_fd.fn_number == 0) {
                tp_log_info("%s#%d: Reached end of PDT\n",__func__,__LINE__);
                break;
            }

            tp_log_info("%s#%d: F%02x found(page %d)\n",
                __func__,__LINE__,rmi_fd.fn_number,page_number);

            switch (rmi_fd.fn_number) {
            case SYNAPTICS_RMI4_F01:
                if (rmi_fd.intr_src_count == 0)
                    break;

                retval = synaptics_rmi4_alloc_fh(&fhandler,&rmi_fd, page_number);
                if (retval < 0) {
                    tp_log_err("%s#%d: Malloc for F%d fail!\n",
                        __func__,__LINE__,rmi_fd.fn_number);
                    return retval;
                }

                retval = synaptics_rmi4_f01_init(rmi4_data, fhandler, &rmi_fd, intr_count);
                if (retval < 0)
                    return retval;

                retval = synaptics_rmi4_check_status(rmi4_data,&was_in_bl_mode);
                if (retval < 0) {
                    tp_log_err("%s#%d: Check status fail!\n",__func__,__LINE__);
                    return retval;
                }

                if (was_in_bl_mode) {
                    kfree(fhandler);
                    fhandler = NULL;
                    goto rescan_pdt;
                }

                if (rmi4_data->flash_prog_mode)
                    goto flash_prog_mode;

                break;
            case SYNAPTICS_RMI4_F11:
                if (rmi_fd.intr_src_count == 0)
                    break;

                retval = synaptics_rmi4_alloc_fh(&fhandler,&rmi_fd, page_number);
                if (retval < 0) {
                    tp_log_err("%s#%d: Malloc for F%d fail!\n",
                        __func__,__LINE__,rmi_fd.fn_number);
                    return retval;
                }

                retval = synaptics_rmi4_f11_init(rmi4_data,fhandler, &rmi_fd, intr_count);
                if (retval < 0)
                    return retval;
                break;
            case SYNAPTICS_RMI4_F12:
                if (rmi_fd.intr_src_count == 0)
                    break;

                retval = synaptics_rmi4_alloc_fh(&fhandler,&rmi_fd, page_number);
                if (retval < 0) {
                    tp_log_err("%s#%d: Malloc for F%d fail!\n",
                        __func__,__LINE__,rmi_fd.fn_number);
                    return retval;
                }

                retval = synaptics_rmi4_f12_init(rmi4_data,fhandler, &rmi_fd, intr_count);
                if (retval < 0)
                    return retval;
                break;
            case SYNAPTICS_RMI4_F1A:
                if (rmi_fd.intr_src_count == 0)
                    break;

                retval = synaptics_rmi4_alloc_fh(&fhandler,&rmi_fd, page_number);
                if (retval < 0) {
                    tp_log_err("%s#%d: Malloc for F%d fail!\n",
                        __func__,__LINE__,rmi_fd.fn_number); 
                    return retval;
                }

                retval = synaptics_rmi4_f1a_init(rmi4_data,fhandler, &rmi_fd, intr_count);
                if (retval < 0) {
#ifdef IGNORE_FN_INIT_FAILURE
                    kfree(fhandler);
                    fhandler = NULL;
#else
                    return retval;
#endif
                }
                break;
            case SYNAPTICS_RMI4_F51:
                retval = synaptics_rmi4_f51_init(rmi4_data, &rmi_fd, page_number);
                if (retval < 0)
                    return retval;
                break;
            case SYNAPTICS_RMI4_F54:
                retval = synaptics_rmi4_f54_init(rmi4_data,&rmi_fd, page_number);
                if (retval < 0)
                    return retval;
                break;
            default:
                tp_log_info("%s#%d: unkown fn_number(%d)\n",
                    __func__,__LINE__,rmi_fd.fn_number);
                break;
            }

            /* Accumulate the interrupt count */
            intr_count += (rmi_fd.intr_src_count & MASK_3BIT);

            if (fhandler && rmi_fd.intr_src_count) {
                list_add_tail(&fhandler->link,&rmi->support_fn_list);
            }
        }
    }

flash_prog_mode:
    rmi4_data->num_of_intr_regs = (intr_count + 7) / 8;
    tp_log_info("%s#%d: Numb of irq regs = %d\n",
        __func__,__LINE__,rmi4_data->num_of_intr_regs);
    
    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f01_query_base_addr,
            f01_query,
            sizeof(f01_query));
    if (retval < 0)
        return retval;

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
    /* detect current device successful, set the flag as present */
    set_hw_dev_flag(DEV_I2C_TOUCH_PANEL);
#endif

    /* RMI Version 4.0 currently supported */
    rmi->version_major = 4;
    rmi->version_minor = 0;

    rmi->manufacturer_id = f01_query[0];
    rmi->product_props = f01_query[1];
    rmi->product_info[0] = f01_query[2] & MASK_7BIT;
    rmi->product_info[1] = f01_query[3] & MASK_7BIT;
    rmi->date_code[0] = f01_query[4] & MASK_5BIT;
    rmi->date_code[1] = f01_query[5] & MASK_4BIT;
    rmi->date_code[2] = f01_query[6] & MASK_5BIT;
    rmi->tester_id = ((f01_query[7] & MASK_7BIT) << 8) |
            (f01_query[8] & MASK_7BIT);
    rmi->serial_number = ((f01_query[9] & MASK_7BIT) << 8) |
            (f01_query[10] & MASK_7BIT);
    memcpy(rmi->product_id_string, &f01_query[11], 10);

    if (rmi->manufacturer_id != 1) {
        tp_log_err("%s#%d: Non-synap devcie found, manufacturer ID = %d\n",
            __func__,__LINE__,rmi->manufacturer_id);
        return -EINVAL;
    }
    set_tp_type((rmi->product_id_string[strnlen(rmi->product_id_string,10) - 1] - '0'));
    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f01_query_base_addr + F01_BUID_ID_OFFSET,
            rmi->build_id,
            sizeof(rmi->build_id));
    if (retval < 0)
        return retval;

    rmi4_data->firmware_id = (unsigned int)rmi->build_id[0] +
            (unsigned int)rmi->build_id[1] * 0x100 +
            (unsigned int)rmi->build_id[2] * 0x10000;

    memset(rmi4_data->intr_mask, 0x00, sizeof(rmi4_data->intr_mask));

    /*
     * Map out the interrupt bit masks for the interrupt sources
     * from the registered function handlers.
     */
    if (!list_empty(&rmi->support_fn_list)) {
        list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
            if (fhandler->num_of_data_sources) {
                rmi4_data->intr_mask[fhandler->intr_reg_num] |=
                        fhandler->intr_mask;
            }
        }
    }

    /* Enable the interrupt sources */
    for (ii = 0; ii < rmi4_data->num_of_intr_regs; ii++) {
        if (rmi4_data->intr_mask[ii] != 0x00) {
            tp_log_info("%s#%d: Interrupt enable mask %d = 0x%02x\n\n",
                __func__,__LINE__,ii,rmi4_data->intr_mask[ii]);
            intr_addr = rmi4_data->f01_ctrl_base_addr + 1 + ii;
            retval = synaptics_rmi4_i2c_write(rmi4_data,
                    intr_addr,
                    &(rmi4_data->intr_mask[ii]),
                    sizeof(rmi4_data->intr_mask[ii]));
            if (retval < 0)
                return retval;
        }
    }

    synaptics_rmi4_set_configured(rmi4_data);

    return 0;
}

static void synaptics_rmi4_set_params(struct synaptics_rmi4_data *rmi4_data)
{
    unsigned char ii;
    struct synaptics_rmi4_f1a_handle *f1a;
    struct synaptics_rmi4_fn *fhandler;
    struct synaptics_rmi4_device_info *rmi;

    rmi = &(rmi4_data->rmi4_mod_info);

    tp_log_debug("%s: sensor_max_x = %d,sensor_max_y = %d \n",__func__, rmi4_data->sensor_max_x,rmi4_data->sensor_max_y);
    input_set_abs_params(rmi4_data->input_dev,
            ABS_MT_POSITION_X, 0,
#ifndef CONFIG_HUAWEI_KERNEL
            rmi4_data->sensor_max_x, 0, 0);
#else /*CONFIG_HUAWEI_KERNEL*/
            rmi4_data->board->lcd_x-1, 0, 0);
#endif /*CONFIG_HUAWEI_KERNEL*/
    input_set_abs_params(rmi4_data->input_dev,
            ABS_MT_POSITION_Y, 0,
#ifndef CONFIG_HUAWEI_KERNEL
            rmi4_data->sensor_max_y, 0, 0);
#else /*CONFIG_HUAWEI_KERNEL*/
            rmi4_data->board->lcd_y-1, 0, 0);
#endif /*CONFIG_HUAWEI_KERNEL*/
    input_set_abs_params(rmi4_data->input_dev,
            ABS_MT_PRESSURE, 0, 255, 0, 0);
#ifdef REPORT_2D_W
    input_set_abs_params(rmi4_data->input_dev,
            ABS_MT_TOUCH_MAJOR, 0,
            rmi4_data->max_touch_width, 0, 0);
    input_set_abs_params(rmi4_data->input_dev,
            ABS_MT_TOUCH_MINOR, 0,
            rmi4_data->max_touch_width, 0, 0);
#endif

#ifdef TYPE_B_PROTOCOL
    input_mt_init_slots(rmi4_data->input_dev,
            rmi4_data->num_of_fingers,0);
#endif

    f1a = NULL;
    if (!list_empty(&rmi->support_fn_list)) {
        list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
            if (fhandler->fn_number == SYNAPTICS_RMI4_F1A)
                f1a = fhandler->data;
        }
    }

    if (f1a) {
        for (ii = 0; ii < f1a->valid_button_count; ii++) {
            set_bit(f1a->button_map[ii],
                    rmi4_data->input_dev->keybit);
            input_set_capability(rmi4_data->input_dev,
                    EV_KEY, f1a->button_map[ii]);
        }
    }

    return;
}

static int synaptics_rmi4_set_input_dev(struct synaptics_rmi4_data *rmi4_data)
{
    int retval;
    int temp;

    rmi4_data->input_dev = input_allocate_device();
    if (rmi4_data->input_dev == NULL) {
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: Failed to allocate input device\n",
                __func__);
        retval = -ENOMEM;
        goto err_input_device;
    }

    retval = synaptics_rmi4_query_device(rmi4_data);
    if (retval < 0) {
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: Failed to query device\n",
                __func__);
        goto err_query_device;
    }

    rmi4_data->input_dev->name = DRIVER_NAME;
    rmi4_data->input_dev->phys = INPUT_PHYS_NAME;
    rmi4_data->input_dev->id.product = SYNAPTICS_DSX_DRIVER_PRODUCT;
    rmi4_data->input_dev->id.version = SYNAPTICS_DSX_DRIVER_VERSION;
    rmi4_data->input_dev->id.bustype = BUS_I2C;
    rmi4_data->input_dev->dev.parent = &rmi4_data->i2c_client->dev;
    input_set_drvdata(rmi4_data->input_dev, rmi4_data);

    set_bit(EV_SYN, rmi4_data->input_dev->evbit);
    set_bit(EV_KEY, rmi4_data->input_dev->evbit);
    set_bit(EV_ABS, rmi4_data->input_dev->evbit);
    set_bit(BTN_TOUCH, rmi4_data->input_dev->keybit);
#if USE_BTN_TOOL_FINGER//huawei 11-25
    set_bit(BTN_TOOL_FINGER, rmi4_data->input_dev->keybit);
#endif//huawei 11-25
#ifdef INPUT_PROP_DIRECT
    set_bit(INPUT_PROP_DIRECT, rmi4_data->input_dev->propbit);
#endif
#if USE_WAKEUP_GESTURE
    retval = init_easy_wakeup_key_value(rmi4_data);
    if (retval < 0) {
        tp_log_err( "%s: Error, init easy wakeup key value fail!\n",
            __func__);
        rmi4_data->board->easy_wakeup_supported_gestures = 0;
    }
    /* Double tap zone defined in dtsi */
    retval = huawei_init_double_tap_zone(rmi4_data);
    if (retval < 0) {
        tp_log_err( "%s: Error, init double_tap_zone fail!\n",
            __func__);
    }
#endif /*USE_WAKEUP_GESTURE*/

    /* Record the module-dwt of gesture */
    if(0 == rmi4_data->dwt_gesture)
    {
        retval = synaptics_get_gesture_doze_wakeup_threshold(rmi4_data);
        if(retval < 0)
        {
            tp_log_err("%s: Failed to get gesture dwt\n", __func__);
        }
    }

    if (rmi4_data->board->swap_axes) {
        temp = rmi4_data->sensor_max_x;
        rmi4_data->sensor_max_x = rmi4_data->sensor_max_y;
        rmi4_data->sensor_max_y = temp;
    }

    synaptics_rmi4_set_params(rmi4_data);

    retval = input_register_device(rmi4_data->input_dev);
    if (retval) {
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: Failed to register input device\n",
                __func__);
        goto err_register_input;
    }

    return 0;

err_register_input:
err_query_device:
    synaptics_rmi4_empty_fn_list(rmi4_data);
    input_free_device(rmi4_data->input_dev);

err_input_device:
    return retval;
}

static void synaptics_rmi4_release_gpio(struct synaptics_dsx_platform_data *platform_data)
{
    int irq_gpio;
    int rst_gpio;

    irq_gpio = platform_data->irq_gpio;
    rst_gpio = platform_data->reset_gpio;

    if(irq_gpio >= 0) {
        synaptics_gpio_setup(irq_gpio,false, 0, 0);
    }

    if(rst_gpio >= 0) {
        synaptics_gpio_setup(rst_gpio,false, 0, 0);
    }
}

static int synaptics_rmi4_set_gpio(struct synaptics_dsx_platform_data *platform_data)
{
    int retval;
    int irq_gpio;
    int rst_gpio;
    int reset_on;

    reset_on = platform_data->reset_on_state;
    irq_gpio = platform_data->irq_gpio;
    rst_gpio = platform_data->reset_gpio;

    if (irq_gpio >= 0) {
        retval = synaptics_gpio_setup(irq_gpio,true, 0, 0);
        if (retval < 0) {
            tp_log_err("%s: Failed to configure attention GPIO\n",
                    __func__);
            goto err_gpio_irq;
        }
    }

    if (rst_gpio >= 0) {
        retval = synaptics_gpio_setup(rst_gpio,true, 1, !reset_on);
        if (retval < 0) {
            tp_log_err("%s: Failed to configure reset GPIO\n",
                    __func__);
            goto err_gpio_reset;
        }
        retval = gpio_direction_output(rst_gpio, !reset_on);//pull up
        mdelay(5);
        retval = gpio_direction_output(rst_gpio, reset_on);//pull down
        mdelay(platform_data->reset_active_ms);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
        retval = gpio_direction_output(rst_gpio, !reset_on);//pull up
        mdelay(platform_data->reset_delay_ms);
    }

    return 0;

err_gpio_reset:
    tp_log_err("%s: set_reset_gpio_false  \n",__func__);
    synaptics_gpio_setup(
            platform_data->reset_gpio,
            false, 0, 0);
err_gpio_irq:
        tp_log_err("%s: set_irq_gpio_failse  \n",__func__);
    synaptics_gpio_setup(
            platform_data->irq_gpio,
            false, 0, 0);

    return retval;
}

static int synaptics_rmi4_free_fingers(struct synaptics_rmi4_data *rmi4_data)
{
    unsigned char ii;

#ifdef TYPE_B_PROTOCOL
    for (ii = 0; ii < rmi4_data->num_of_fingers; ii++) {
        input_mt_slot(rmi4_data->input_dev, ii);
        input_mt_report_slot_state(rmi4_data->input_dev,
                MT_TOOL_FINGER, 0);
    }
#endif
    input_report_key(rmi4_data->input_dev,
            BTN_TOUCH, 0);
#if USE_BTN_TOOL_FINGER//huawei 11-25
    input_report_key(rmi4_data->input_dev,
            BTN_TOOL_FINGER, 0);
#endif//huawei 11-25
#ifndef TYPE_B_PROTOCOL
    input_mt_sync(rmi4_data->input_dev);
#endif
    input_sync(rmi4_data->input_dev);

    rmi4_data->fingers_on_2d = false;

    return 0;
}

static int synaptics_rmi4_reinit_device(struct synaptics_rmi4_data *rmi4_data)
{
    int retval;
    unsigned char ii;
    unsigned short intr_addr;
    struct synaptics_rmi4_fn *fhandler;
    struct synaptics_rmi4_exp_fhandler *exp_fhandler;
    struct synaptics_rmi4_device_info *rmi;
    unsigned char intr_status[MAX_INTR_REGISTERS];

#ifndef CONFIG_OF
    if (*platform_data->regulator_name != 0x00) {
        rmi4_data->regulator = regulator_get(&client->dev,
                platform_data->regulator_name);
    }
#else
    const struct synaptics_dsx_platform_data *platform_data =
#ifndef CONFIG_HUAWEI_KERNEL
            rmi4_data->i2c_client->dev.platform_data;
#else /*CONFIG_HUAWEI_KERNEL*/
            rmi4_data->board;
#endif /*CONFIG_HUAWEI_KERNEL*/
    if (platform_data->reg_en) {
        rmi4_data->regulator = regulator_get(&rmi4_data->i2c_client->dev,
                "vdd");
    }
#endif

    tp_log_info("%s#%d: begin\n",__func__,__LINE__);

    rmi = &(rmi4_data->rmi4_mod_info);

    mutex_lock(&(rmi4_data->rmi4_reset_mutex));

    synaptics_rmi4_free_fingers(rmi4_data);
    
    /* Clear interrupts first */
    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f01_data_base_addr + 1,
            intr_status,
            rmi4_data->num_of_intr_regs);
    if (retval < 0){
        tp_log_err("%s#%d: reinit fail, ret=%d\n",__func__,__LINE__,retval);
        goto exit;
    }
    /**
     *if product dts supported grip algorithm and state of the grip algorithm 
     *is false ,we begin to set the grip algorithm edge distance
     */
    if((true == rmi4_data->board->grip_algorithm_supported)
        &&(false == grip_algorithm_state)){
        retval = set_grip_algorithm_edge_distance(rmi4_data);
        if(retval < 0){
            tp_log_err("%s:failed to set grip algorithm edge distance! \n",__func__);
            goto exit;
        }
        grip_algorithm_state = true;
    }
    if (!list_empty(&rmi->support_fn_list)) {
        list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
            if (fhandler->fn_number == SYNAPTICS_RMI4_F12) {
                synaptics_rmi4_f12_set_enables(rmi4_data, 0);
                break;
            }
        }
    }

    for (ii = 0; ii < rmi4_data->num_of_intr_regs; ii++) {
        if (rmi4_data->intr_mask[ii] != 0x00) {
            tp_log_debug("%s#%d: Interrupt enable mask %d = 0x%02x\n",
                    __func__, __LINE__,ii, rmi4_data->intr_mask[ii]);
            intr_addr = rmi4_data->f01_ctrl_base_addr + 1 + ii;
            retval = synaptics_rmi4_i2c_write(rmi4_data,
                    intr_addr,
                    &(rmi4_data->intr_mask[ii]),
                    sizeof(rmi4_data->intr_mask[ii]));
            if (retval < 0) {
                tp_log_err("%s#%d: reinit fail, ret=%d\n",__func__,__LINE__,retval);
                goto exit;
            }
        }
    }

    mutex_lock(&exp_data.mutex);
    if (!list_empty(&exp_data.list)) {
        list_for_each_entry(exp_fhandler, &exp_data.list, link)
            if (exp_fhandler->exp_fn->reinit != NULL)
                exp_fhandler->exp_fn->reinit(rmi4_data);
    }
    mutex_unlock(&exp_data.mutex);

    synaptics_rmi4_set_configured(rmi4_data);

    retval = 0;

exit:
    mutex_unlock(&(rmi4_data->rmi4_reset_mutex));
    tp_log_info("%s#%d: done!\n",__func__,__LINE__);    
    return retval;
}

static int synaptics_rmi4_reset_device(struct synaptics_rmi4_data *rmi4_data)
{
    int retval;
    int temp;
    unsigned char command = 0x01;
    struct synaptics_rmi4_exp_fhandler *exp_fhandler;

    mutex_lock(&(rmi4_data->rmi4_reset_mutex));

    /* After reset there will be 2 interrupts which may cause system too busy */
    disable_irq_nosync(rmi4_data->irq);

    rmi4_data->touch_stopped = true;

    retval = synaptics_rmi4_i2c_write(rmi4_data,
            rmi4_data->f01_cmd_base_addr,
            &command,
            sizeof(command));
    if (retval < 0) {
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: Failed to issue reset command, error = %d\n",
                __func__, retval);
        enable_irq(rmi4_data->irq);
        mutex_unlock(&(rmi4_data->rmi4_reset_mutex));
        return retval;
    }

    msleep(rmi4_data->board->reset_delay_ms);

    synaptics_rmi4_free_fingers(rmi4_data);

    synaptics_rmi4_empty_fn_list(rmi4_data);

    retval = synaptics_rmi4_query_device(rmi4_data);
    if (retval < 0) {
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: Failed to query device\n",
                __func__);
        enable_irq(rmi4_data->irq);
        mutex_unlock(&(rmi4_data->rmi4_reset_mutex));
        return retval;
    }

    /* F01_RMI_CTRL03:Doze Wakeup Threshold*/
    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f01_ctrl_base_addr + F01_RMI_CTRL03_OFFSET,
            &command,
            sizeof(command));
    tp_log_info("%s(line%d): Doze Wakeup Threshold = %d\n",__func__,__LINE__,command);
    if (retval < 0) {
        dev_err(&(rmi4_data->input_dev->dev),
                "%s: Failed to read default dwt!\n",
                __func__);
        enable_irq(rmi4_data->irq);
        mutex_unlock(&(rmi4_data->rmi4_reset_mutex));
        return retval;
    }
    
    /**
     *if product dts supported grip algorithm,
     *we begin to set the grip algorithm edge distance
     */
    if(true == rmi4_data->board->grip_algorithm_supported){
        retval = set_grip_algorithm_edge_distance(rmi4_data);
        if(retval < 0){
            tp_log_err("%s:failed to set grip algorithm edge distance! \n",__func__);
            return retval;
        }
        grip_algorithm_state = true;
    }
    /* Refresh the default-dwt */
    rmi4_data->dwt_default = command;
    tp_log_info("%s(line%d): rmi4_data->dwt_default = %d\n",__func__,__LINE__,rmi4_data->dwt_default);

    if (rmi4_data->board->swap_axes) {
        temp = rmi4_data->sensor_max_x;
        rmi4_data->sensor_max_x = rmi4_data->sensor_max_y;
        rmi4_data->sensor_max_y = temp;
    }

    synaptics_rmi4_set_params(rmi4_data);

    mutex_lock(&exp_data.mutex);
    if (!list_empty(&exp_data.list)) {
        /* Add necessary log for debug */
        tp_log_info("%s: line(%d)\n", __func__,__LINE__);
        list_for_each_entry(exp_fhandler, &exp_data.list, link){
            tp_log_info("%s#d%d: exp_fn_type = %d\n",
                __func__,__LINE__,exp_fhandler->exp_fn->fn_type);
            if (exp_fhandler->exp_fn->reset != NULL)
            {
                tp_log_info("%s#%d: exp_fn_type=%d, reset!\n", 
                    __func__,__LINE__,exp_fhandler->exp_fn->fn_type);
                exp_fhandler->exp_fn->reset(rmi4_data);
            }
        }
    }
    mutex_unlock(&exp_data.mutex);

    rmi4_data->touch_stopped = false;

    enable_irq(rmi4_data->irq);
    mutex_unlock(&(rmi4_data->rmi4_reset_mutex));

    return 0;
}

/**
* synaptics_rmi4_exp_fn_work()
*
* Called by the kernel at the scheduled time.
*
* This function is a work thread that checks for the insertion and
* removal of other expansion Function modules such as rmi_dev and calls
* their initialization and removal callback functions accordingly.
*/
static void synaptics_rmi4_exp_fn_work(struct work_struct *work)
{
    int retval;
    struct synaptics_rmi4_exp_fhandler *exp_fhandler;
    struct synaptics_rmi4_exp_fhandler *exp_fhandler_temp;
    struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;

    mutex_lock(&exp_data.mutex);
    if (!list_empty(&exp_data.list)) {
        list_for_each_entry_safe(exp_fhandler,
                exp_fhandler_temp,
                &exp_data.list,
                link) {
            if ((exp_fhandler->exp_fn->init != NULL) &&
                    exp_fhandler->insert) {
                tp_log_info("%s#%d: init fn_type=%d\n", 
                    __func__,__LINE__,exp_fhandler->exp_fn->fn_type);
                retval = exp_fhandler->exp_fn->init(rmi4_data);
                /* In flash program mode,del exp_fn from list will cause fn cant init again. */
                if ((retval < 0) && (!rmi4_data->flash_prog_mode)) {
                    tp_log_err("%s#%d: ret=%d, init fn_type=%d fail!\n", 
                        __func__,__LINE__,retval,exp_fhandler->exp_fn->fn_type);
                    list_del(&exp_fhandler->link);
                    kfree(exp_fhandler);
                } else {
                    exp_fhandler->insert = false;
                }
            } else if ((exp_fhandler->exp_fn->remove != NULL) &&
                    exp_fhandler->remove) {
                exp_fhandler->exp_fn->remove(rmi4_data);
                list_del(&exp_fhandler->link);
                kfree(exp_fhandler);
            }
        }
    }
    mutex_unlock(&exp_data.mutex);

    return;
}

/**
* synaptics_rmi4_new_func()
*
* Called by other expansion Function modules in their module init and
* module exit functions.
*
* This function is used by other expansion Function modules such as
* rmi_dev to register themselves with the driver by providing their
* initialization and removal callback function pointers so that they
* can be inserted or removed dynamically at module init and exit times,
* respectively.
*/
void synaptics_rmi4_new_func(struct synaptics_rmi4_exp_fn *exp_fn,
        bool insert)
{
    struct synaptics_rmi4_exp_fhandler *exp_fhandler;

    if (!exp_data.initialized) {
        mutex_init(&exp_data.mutex);
        INIT_LIST_HEAD(&exp_data.list);
        exp_data.initialized = true;
    }

    mutex_lock(&exp_data.mutex);
    if (insert) {
        exp_fhandler = kzalloc(sizeof(*exp_fhandler), GFP_KERNEL);
        if (!exp_fhandler) {
            tp_log_err("%s#%d: Malloc for exp_func fail!\n",__func__,__LINE__);
            goto exit;
        }
        exp_fhandler->exp_fn = exp_fn;
        exp_fhandler->insert = true;
        exp_fhandler->remove = false;
        list_add_tail(&exp_fhandler->link, &exp_data.list);
    } else if (!list_empty(&exp_data.list)) {
        list_for_each_entry(exp_fhandler, &exp_data.list, link) {
            if (exp_fhandler->exp_fn->fn_type == exp_fn->fn_type) {
                exp_fhandler->insert = false;
                exp_fhandler->remove = true;
                goto exit;
            }
        }
    }

exit:
    mutex_unlock(&exp_data.mutex);

    if (exp_data.queue_work) {
        queue_delayed_work(exp_data.workqueue,
                &exp_data.work,msecs_to_jiffies(EXP_FN_WORK_DELAY_MS));
    }

    return;
}
EXPORT_SYMBOL(synaptics_rmi4_new_func);

#ifdef ENABLE_VIRTUAL_KEYS//huawei 11-25

#define VIRTUAL_KEY_ELEMENT_SIZE    5
#define  MAX_NAME_LENGTH 256
#define DSX_MAX_PRBUF_SIZE        PIPE_BUF

static ssize_t virtual_keys_show(struct kobject *kobj,
        struct kobj_attribute *attr, char *buf)
{
    struct syanptics_virtual_keys *vkeys = container_of(attr,
        struct syanptics_virtual_keys, kobj_attr);
    u16 *data = vkeys->data;
    int size = vkeys->size;
    int index;
    int i;

    index = 0;
    for (i = 0; i < size; i += VIRTUAL_KEY_ELEMENT_SIZE)
        index += scnprintf(buf + index, DSX_MAX_PRBUF_SIZE - index,
            "0x01:%d:%d:%d:%d:%d\n",
            data[i], data[i+1], data[i+2], data[i+3], data[i+4]);

    return index;
}

static u16 *create_and_get_u16_array(struct device_node *dev_node,
        const char *name, int *size)
{
    const __be32 *values;
    u16 *val_array;
    int len;
    int sz;
    int rc;
    int i;

    values = of_get_property(dev_node, name, &len);
    if (values == NULL)
        return NULL;

    sz = len / sizeof(u32);

    val_array = kzalloc(sz * sizeof(u16), GFP_KERNEL);
    if (val_array == NULL) {
        rc = -ENOMEM;
        tp_log_err("%s:fail line=%d\n", __func__,__LINE__);
        goto fail;
    }

    for (i = 0; i < sz; i++)
        val_array[i] = (u16)be32_to_cpup(values++);

    *size = sz;

    return val_array;

fail:
    return ERR_PTR(rc);
}

static int setup_virtual_keys(struct device_node *dev_node,
        const char *inp_dev_name, struct syanptics_virtual_keys *vkeys)
{
    char *name;
    u16 *data;
    int size = 0;
    int rc = -EINVAL;

    data = create_and_get_u16_array(dev_node, "synaptics,virtual_keys", &size);
    if (data == NULL)
    {
        /*if virtual keys are not supported  return error*/
        return -ENOMEM;
    }
    else if (IS_ERR(data)) {
        rc = PTR_ERR(data);
        goto fail;
    }

    /* Check for valid virtual keys size */
    if (size % VIRTUAL_KEY_ELEMENT_SIZE) {
        rc = -EINVAL;
        goto fail_free_data;
    }

    name = kzalloc(MAX_NAME_LENGTH, GFP_KERNEL);
    if (name == NULL) {
        rc = -ENOMEM;
        goto fail_free_data;
    }

    snprintf(name, MAX_NAME_LENGTH, "virtualkeys.%s", inp_dev_name);

    vkeys->data = data;
    vkeys->size = size;

    /* TODO: Instantiate in board file and export it */
    if (board_properties_kobj == NULL)
        board_properties_kobj =
            tp_get_virtual_key_obj("board_properties");
    if (board_properties_kobj == NULL) {
        tp_log_err("%s: Cannot get board_properties kobject!\n", __func__);
        rc = -EINVAL;
        goto fail_free_name;
    }

    /* Initialize dynamic SysFs attribute */
    sysfs_attr_init(&vkeys->kobj_attr.attr);
    vkeys->kobj_attr.attr.name = name;
    vkeys->kobj_attr.attr.mode = S_IRUGO;
    vkeys->kobj_attr.show = virtual_keys_show;

    rc = sysfs_create_file(board_properties_kobj, &vkeys->kobj_attr.attr);
    if (rc)
        goto fail_del_kobj;

    return 0;

fail_del_kobj:
    kobject_del(board_properties_kobj);
fail_free_name:
    kfree(name);
    vkeys->kobj_attr.attr.name = NULL;
fail_free_data:
    kfree(data);
    vkeys->data = NULL;
    vkeys->size = 0;
fail:
    return rc;
}

static void free_virtual_keys(struct syanptics_virtual_keys *vkeys)
{
    if (board_properties_kobj)
        sysfs_remove_file(board_properties_kobj,
            &vkeys->kobj_attr.attr);

    kfree(vkeys->data);
    vkeys->data = NULL;
    vkeys->size = 0;

    kfree(vkeys->kobj_attr.attr.name);
    vkeys->kobj_attr.attr.name = NULL;
}
#endif /*ENABLE_VIRTUAL_KEYS*/

static struct syanptics_wakeup_keys *create_and_get_wakeup_keys(
        struct device_node *dev_node)
{
    struct syanptics_wakeup_keys *wakeup_keys;
    u16 *keys;
    int size;
    int rc;

    keys = create_and_get_u16_array(dev_node, "synaptics,easy_wakeup_gesture_keys", &size);
    if (IS_ERR_OR_NULL(keys))
        return (void *)keys;

    /* Check for valid abs size */
    if (size > GESTURE_MAX) {
        rc = -EINVAL;
        tp_log_err("%s:fail line=%d\n", __func__,__LINE__);
        goto fail_free_keys;
    }

    wakeup_keys = kzalloc(sizeof(*wakeup_keys), GFP_KERNEL);
    if (wakeup_keys == NULL) {
        rc = -ENOMEM;
        tp_log_err("%s:fail line=%d\n", __func__,__LINE__);
        goto fail_free_keys;
    }

    wakeup_keys->keys = keys;
    wakeup_keys->size = size;

    return wakeup_keys;

fail_free_keys:
    kfree(keys);

    return ERR_PTR(rc);
}

static void free_wakeup_keys(struct syanptics_wakeup_keys *wakeup_keys)
{
    /*If call in failure case when frmwrk is free but abs is not it might crash,
    use check even its called in failure case*/
    if (wakeup_keys)
        kfree(wakeup_keys->keys);
    kfree(wakeup_keys);
}

#ifdef CONFIG_OF

/**
 * get_of_u32_val() - get the u32 type value fro the DTSI.
 * @np: device node
 * @name: The DTSI key name
 * @default_value: If the name is not matched, return a default value.
 */
static u32 get_of_u32_val(struct device_node *np,
    const char *name,u32 default_val)
{
    u32 tmp= 0;
    int err = 0;
    err = of_property_read_u32(np, name, &tmp);
    if (!err)
        return tmp;
    else
        return default_val;
}

static void synaptics_print_dtsi_info(struct synaptics_dsx_platform_data *pdata)
{
    tp_log_debug("synaptics,product_name=%s\n", pdata->product_name);
    tp_log_debug("synaptics,ic_type=%d\n", pdata->ic_type);
    tp_log_debug("synaptics,reset-gpio=%d\n", pdata->reset_gpio);
    tp_log_debug("synaptics,lcd-x=%d; lcd-y=%d; lcd-all=%d\n", pdata->lcd_x,pdata->lcd_y,pdata->lcd_all);
    tp_log_debug("synaptics,reset-on-status=%d power-delay-ms=%d reset-delay-ms=%d reset-active-ms=%d\n",
                    pdata->reset_on_state,pdata->power_delay_ms,pdata->reset_delay_ms,pdata->reset_active_ms);
    tp_log_debug("synaptics,gesture_enabled=%d; easy_wakeup_supported_gestures=0x%x\n",
                    pdata->gesture_enabled,pdata->easy_wakeup_supported_gestures);
    tp_log_debug("synaptics,double tap zone=(%d,%d)(%d,%d)\n", 
                pdata->dtz_x0,pdata->dtz_y0,pdata->dtz_x1,pdata->dtz_y1);
    tp_log_debug("synaptics,glove_enabled=%d\n", pdata->glove_enabled);
    tp_log_debug("synaptics,fast_relax_gesture=%d\n", pdata->fast_relax_gesture);
    tp_log_debug("synaptics,pdata->holster_supported=%d\n", pdata->holster_supported);
    tp_log_debug("synaptics,ofilm_dwt=%d\n", pdata->ofilm_dwt);
    tp_log_debug("synaptics,junda_dwt=%d\n", pdata->junda_dwt);
    tp_log_debug("synaptics,truly_dwt=%d\n", pdata->truly_dwt);
    tp_log_debug("synaptics,eely_dwt=%d\n", pdata->eely_dwt);
    tp_log_debug("synaptics,lensone_dwt=%d\n", pdata->lensone_dwt);
    tp_log_debug("synaptics,glove_edge_switch_supported=%d\n", pdata->glove_edge_switch_supported);
    tp_log_debug("synaptics,grip_algorithm_supported=%d\n", pdata->grip_algorithm_supported);
    tp_log_debug("synaptics,grip_algorithm_enabled=%d\n", pdata->grip_algorithm_enabled);
}

static int synaptics_rmi4_parse_dt(struct device *dev, struct synaptics_dsx_platform_data *pdata)
{
    //int retval; //TODO: error checking
    struct device_node *np = dev->of_node;
    struct property *prop;
    u32 tmp = 0;
    int err = 0;

    pdata->x_flip = of_property_read_bool(np, "synaptics,x-flip");
    pdata->y_flip = of_property_read_bool(np, "synaptics,y-flip");
    pdata->swap_axes = of_property_read_bool(np, "synaptics,swap-axes");

    /* reset, irq gpio info */
    pdata->irq_gpio = of_get_named_gpio_flags(np, "synaptics,irq-gpio", 0, NULL);
    /* In case that system has not enough time to process irq */
    pdata->irq_flags = IRQF_TRIGGER_LOW | IRQF_ONESHOT;

    if (of_find_property(np, "synaptics,reset-gpio", NULL)) {
        pdata->reset_gpio = of_get_named_gpio_flags(np, "synaptics,reset-gpio",
            0, NULL);
    } else {
        /*invalid reset GPIO*/
        pdata->reset_gpio = -1;
    }

    /* Modify JDI tp/lcd power on/off to reduce power consumption */
    if (of_find_property(np, "synaptics,vci-gpio", NULL)) {
        pdata->vci_gpio = of_get_named_gpio_flags(np, "synaptics,vci-gpio",
            0, NULL);
    } else {
        /*invalid vci enable GPIO*/
        pdata->vci_gpio = -1;
    }

    pdata->ic_type = get_of_u32_val(np, "ic_type", SYNAPTICS_S3207);

    /*LCD resolution. Default is WVGA*/
    pdata->lcd_x = get_of_u32_val(np, "synaptics,lcd-x", LCD_X_DEFAULT);
    pdata->lcd_y = get_of_u32_val(np, "synaptics,lcd-y", LCD_Y_DEFAULT);
    pdata->lcd_all = get_of_u32_val(np, "synaptics,lcd-all", LCD_ALL_DEFAULT);

#ifdef ENABLE_VIRTUAL_KEYS
    err = setup_virtual_keys(np, DRIVER_NAME,&pdata->vkeys);
    if (err) {
        /*If the virtual keys are not supported the TP should work fine;*/
        tp_log_err("%s: Cannot setup virtual keys, only TP will work now!err = %d\n",__func__,err);
    }
#endif

    err = of_property_read_string(np, "synaptics,product_name",&pdata->product_name);
    if (err){
        tp_log_err("Unable to read firmware_name\n");            
        pdata->product_name = "Unknow";
    }

    pdata->reset_on_state = get_of_u32_val(np, "synaptics,reset-on-status", 0);
    pdata->power_delay_ms = get_of_u32_val(np, "synaptics,power-delay-ms", 160);
    pdata->reset_delay_ms = get_of_u32_val(np, "synaptics,reset-delay-ms", 10);
    pdata->reset_active_ms = get_of_u32_val(np, "synaptics,reset-active-ms", 100);

    prop = of_find_property(np, "synaptics,button-map", NULL);
    if (prop) {
        pdata->cap_button_map->nbuttons = prop->length / sizeof(tmp);
        err = of_property_read_u32_array(np,
            "synaptics,button-map", (unsigned int *)pdata->cap_button_map->map,
            pdata->cap_button_map->nbuttons);
        if (err) {
            tp_log_err("Unable to read key codes\n");
            pdata->cap_button_map->map = NULL;
        }
    }
    
    pdata->gesture_enabled = get_of_u32_val(np, "synaptics,gesture_enabled", 0);
    pdata->easy_wakeup_supported_gestures = get_of_u32_val(np,
            "synaptics,easy_wakeup_supported_gestures", 0);

    pdata->wakeup_keys = create_and_get_wakeup_keys(np);
    if (IS_ERR_OR_NULL(pdata->wakeup_keys)) {
        tp_log_warning("%s: Wakeup gesture is not configured!\n", __func__);
        pdata->wakeup_keys = NULL;
    } else {
        tp_log_info("%s: Wakeup gesture is configured for %d guestures\n", 
            __func__,pdata->wakeup_keys->size);
    }

    /* Double tap zone defined in dtsi  0 means invalid*/
    pdata->dtz_x0 = get_of_u32_val(np, "huawei,dtz_x0", 0);
    pdata->dtz_y0 = get_of_u32_val(np, "huawei,dtz_y0", 0);
    pdata->dtz_x1 = get_of_u32_val(np, "huawei,dtz_x1", 0);
    pdata->dtz_y1 = get_of_u32_val(np, "huawei,dtz_y1", 0);

    err = of_property_read_u32(np, "synaptics,glove_enabled", &tmp);
    if (!err) {
        pdata->glove_enabled = tmp;
        pdata->glove_supported = true;
    } else {
        pdata->glove_supported = false;
    }
    
    pdata->fast_relax_gesture = get_of_u32_val(np, "synaptics,fast_relax_gesture", 0);

    pdata->holster_supported = (bool)get_of_u32_val(np, "synaptics,holster_supported", 0);
    
    pdata->esd_support = (bool)get_of_u32_val(np, "synaptics,esd_support", 1);
    pdata->esd_delay_time = get_of_u32_val(np,"synaptics,esd_delay_time",3000);

    /* Doze wakeup threshold for gesture mode */
    pdata->ofilm_dwt = get_of_u32_val(np, "synaptics,ofilm_dwt", 0);
    pdata->junda_dwt = get_of_u32_val(np, "synaptics,junda_dwt", 0);
    pdata->truly_dwt = get_of_u32_val(np, "synaptics,truly_dwt", 0);
    pdata->eely_dwt = get_of_u32_val(np, "synaptics,eely_dwt", 0);
    pdata->lensone_dwt = get_of_u32_val(np, "synaptics,lensone_dwt", 0);

    pdata->glove_edge_switch_supported= (bool)get_of_u32_val(np, "synaptics,glove_edge_switch_supported", 0);

    pdata->grip_algorithm_supported= (bool)get_of_u32_val(np, "synaptics,grip_algorithm_supported", 0);
    pdata->grip_algorithm_enabled= (bool)get_of_u32_val(np, "synaptics,grip_algorithm_enabled", 0);
    pdata->grip_left_lsb= get_of_u32_val(np, "huawei,grip_left_lsb", 0);
    pdata->grip_left_msb= get_of_u32_val(np, "huawei,grip_left_msb", 0);
    pdata->grip_right_lsb= get_of_u32_val(np, "huawei,grip_right_lsb", 0x4c);
    pdata->grip_right_msb= get_of_u32_val(np, "huawei,grip_right_msb", 0x04);

    /*DEBUG: print tp dtsi info*/
    synaptics_print_dtsi_info(pdata);

    return 0;
}
#else
static int synaptics_rmi4_parse_dt(struct device *dev, struct synaptics_dsx_platform_data *pdata)
{
    return -ENODEV;
}
#endif

/*****************************************************************
Parameters    :  void
Return        :    
Description   :  reset ic
*****************************************************************/
void synaptics_dsx_hardware_reset(struct synaptics_rmi4_data *rmi4_data)
{
    int ret = 0;
    int reset_on = rmi4_data->board->reset_on_state;
    int rest_gpio = rmi4_data->board->reset_gpio;
    unsigned int reset_low = rmi4_data->board->reset_active_ms;
    unsigned int reset_high = rmi4_data->board->reset_delay_ms;

    
    char buf[SYNAPTICS_HOLSTER_INFO_LENGTH] = {0};
    struct device *cdev = core_dev_ptr;

    disable_irq(rmi4_data->irq);

    ret = gpio_direction_output(rest_gpio, !reset_on);//pull up
    mdelay(5); 
    ret = gpio_direction_output(rest_gpio, reset_on);//pull down
    mdelay(reset_low); 
    ret = gpio_direction_output(rest_gpio, !reset_on);//pull up
    mdelay(reset_high); 

    synaptics_rmi4_reset_device(rmi4_data);

    /* init holster function */
    ret = snprintf(buf, SYNAPTICS_HOLSTER_INFO_LENGTH - 1,
                    "%ld %d %d %d %d", synap_holster_info.holster_enable, 
                    synap_holster_info.top_left_x0, 
                    synap_holster_info.top_left_y0, 
                    synap_holster_info.bottom_right_x1, 
                    synap_holster_info.bottom_right_y1);
    buf[ret] = 0x00;
    if (!cdev){
        tp_log_err("%s %d: device is null \n", __func__, __LINE__);
        return;
    }
    synaptics_holster_func_store(cdev, NULL, buf, ret);

    /* init glove function */
    ret = snprintf(buf, SYNAPTICS_HOLSTER_INFO_LENGTH - 1,
                    "%d", rmi4_data->glove_enabled);
    buf[ret] = 0x00;
    synaptics_glove_func_store(cdev, NULL, buf, ret);
    
    enable_irq(rmi4_data->irq);

    tp_log_info("%s#%d: ic hard reseted!\n",__func__,__LINE__);
    return;
}

#if !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_FB)
 static int fb_notifier_callback(struct notifier_block *self,
                 unsigned long event, void *data)
 {
     struct fb_event *evdata = data;
     int *blank = NULL;
     static bool is_fb_suspend = 0;
     struct synaptics_rmi4_data *rmi4_data =
             container_of(self, struct synaptics_rmi4_data,
             fb_notif);
     
     if (evdata && evdata->data && event == FB_EVENT_BLANK && rmi4_data) {
         blank = evdata->data;
         tp_log_info("%s#%d:blank=%d,is_fb_suspend=%d\n",
            __func__,__LINE__, *blank,is_fb_suspend);
         /*In case of resume we get BLANK_UNBLANK and for suspen BLANK_POWERDOWN*/
         if (*blank == FB_BLANK_UNBLANK)
         {
             if (1 == is_fb_suspend)
             {
                 synaptics_pm_rt_get(rmi4_data);
                 is_fb_suspend = 0;//in case that fb_callback more than once
             }
             else
             {
                 tp_log_warning("%s:line%d , is_fb_suspend=%d,TP already resumed!\n",
                    __func__,__LINE__, is_fb_suspend);
             }
         }
         else if (*blank == FB_BLANK_POWERDOWN)
         {
            if (0 == is_fb_suspend)
            {
                synaptics_pm_rt_put(rmi4_data);
                 is_fb_suspend = 1;//in case that fb_callback more than once
             }
             else
             {
                 tp_log_warning("%s:line%d , is_fb_suspend=%d,TP already suspended!\n",
                    __func__,__LINE__, is_fb_suspend);
             }
         }
     }
     return 0;
 }

 static int configure_sleep(struct synaptics_rmi4_data *rmi4_data)
 {
     int retval = 0;
     struct device *dev = &(rmi4_data->input_dev->dev);
        
     rmi4_data->fb_notif.notifier_call = fb_notifier_callback;
 
     /*register the callback to the FB*/
     retval = fb_register_client(&rmi4_data->fb_notif);
     if (retval){
         dev_info(dev,
             "Unable to register fb_notifier: %d\n", retval);
         return -EINVAL;
     }
     return 0;
 }
#endif

 static int synaptics_pinctrl_init(struct synaptics_rmi4_data *rmi4_data)
 {
     struct i2c_client *client = rmi4_data->i2c_client;
     tp_log_info("%s, begin \n", __func__);
     rmi4_data->pinctrl = devm_pinctrl_get(&client->dev);
     if (IS_ERR_OR_NULL(rmi4_data->pinctrl)) {
         dev_err(&client->dev, "Failed to get pinctrl\n");
         return PTR_ERR(rmi4_data->pinctrl);
     }
 
     /* Modify JDI tp reset gpio pinctrl for power consumption */
     rmi4_data->pin_active =
         pinctrl_lookup_state(rmi4_data->pinctrl, "active");
     if (IS_ERR_OR_NULL(rmi4_data->pin_active)) {
         dev_err(&client->dev, "Failed to look up active state\n");
         return PTR_ERR(rmi4_data->pin_active);
     }
 
     rmi4_data->pin_sleep =
         pinctrl_lookup_state(rmi4_data->pinctrl, "sleep");
     if (IS_ERR_OR_NULL(rmi4_data->pin_sleep)) {
         dev_err(&client->dev, "Failed to look up sleep state\n");
         return PTR_ERR(rmi4_data->pin_sleep);
     }
 
     return 0;
 }
extern int get_product_module_name(unsigned char *product_id);
static void synaptics_rmi4_read_default_glove_holster_sensitivity(struct synaptics_rmi4_data *rmi4_data)
{
    int ret;
    int product_module_name = UNKNOW_PRODUCT_MODULE;
    struct device_node *np = rmi4_data->i2c_client->dev.of_node;
    struct device_node *dev_node = NULL;
    struct synaptics_rmi4_device_info *rmi = &(rmi4_data->rmi4_mod_info);
    char *product_id = rmi->product_id_string;

    /*
       read glove/holster sensitivity register value for default
       the parameters was defined in synaptics_dsx_i2c.h in rmi4_data structure.
     */
    ret = synaptics_rmi4_i2c_read(rmi4_data,
            PDT_P4_F51_CUSTOM_CTRL_BASE+F51_CTRL_01_OFFSET,
            &rmi4_data->f51_ctrl_01_val,
            sizeof(rmi4_data->f51_ctrl_01_val));
    if (ret < 0) {
        tp_log_err("%s: Failed to read F51_CTRL_01 register!\n",
                __func__);
        goto error;
    }
    tp_log_debug("%s: read reg value, f51_ctrl_01_val:0x%x \n", __func__, rmi4_data->f51_ctrl_01_val);
    ret = synaptics_rmi4_i2c_read(rmi4_data,
            PDT_P4_F51_CUSTOM_CTRL_BASE+F51_CTRL_04_OFFSET,
            &rmi4_data->f51_ctrl_04_val,
            sizeof(rmi4_data->f51_ctrl_04_val));
    if (ret < 0) {
        tp_log_err("%s: Failed to read F51_CTRL_04 register!\n",
                __func__);
        goto error;
    }
    tp_log_debug("%s: read reg value, f51_ctrl_04_val:0x%x \n", __func__, rmi4_data->f51_ctrl_04_val);
    ret = synaptics_rmi4_i2c_read(rmi4_data,
            PDT_P0_F01_RMI_CTRL_BASE+F01_CTRL_05_OFFSET,
            &rmi4_data->f01_ctrl_05_val,
            sizeof(rmi4_data->f01_ctrl_05_val));
    if (ret < 0) {
        tp_log_err("%s: Failed to read F01_CTRL_05 register!\n",
                __func__);
        goto error;
    }
    tp_log_debug("%s: read reg value, f01_ctrl_05_val:0x%x \n", __func__, rmi4_data->f01_ctrl_05_val);
    ret = synaptics_rmi4_i2c_read(rmi4_data,
            PDT_P0_F01_RMI_CTRL_BASE+F01_CTRL_09_OFFSET,
            &rmi4_data->f01_ctrl_09_val,
            sizeof(rmi4_data->f01_ctrl_09_val));
    if (ret < 0) {
        tp_log_err("%s: Failed to read F01_CTRL_09 register!\n",
                __func__);
        goto error;
    }
    tp_log_debug("%s: read reg value, f01_ctrl_09_val:0x%x \n", __func__, rmi4_data->f01_ctrl_09_val);
    tp_log_debug("%s: product_id:%s\n",__func__,product_id);

    product_module_name = get_product_module_name(product_id);
    if (product_module_name == UNKNOW_PRODUCT_MODULE) {
        tp_log_err("%s: not able to get module name = %d\n",
                __func__,product_module_name);
        goto error;
    }

    switch(product_module_name) {
        case FW_JUNDA:
            dev_node= of_find_node_by_name(np, "huawei,junda");
            break;
        case FW_OFILM:
            dev_node= of_find_node_by_name(np, "huawei,ofilm");
            break;
        case FW_TRULY:
            dev_node= of_find_node_by_name(np, "huawei,truly");
            break;
        case FW_EELY:
            dev_node= of_find_node_by_name(np, "huawei,eely");
            break;
        case FW_LENSONE:
            dev_node= of_find_node_by_name(np, "huawei,lensone");
            break;
        case FW_TIANMA:
            dev_node= of_find_node_by_name(np, "huawei,tianma");
            break;
        default:
            tp_log_err("%s: got failed,use default!\n",__func__);
            goto error;
    }

    rmi4_data->f51_ctrl_01_low_sensitivity_val = get_of_u32_val(dev_node, "huawei,f51_ctrl_01_low_sensitivity_val", rmi4_data->f51_ctrl_01_val);
    tp_log_debug("%s: read dtsi value, f51_ctrl_01_val:0x%x \n", __func__, rmi4_data->f51_ctrl_01_low_sensitivity_val);
    rmi4_data->f51_ctrl_04_low_sensitivity_val = get_of_u32_val(dev_node, "huawei,f51_ctrl_04_low_sensitivity_val", rmi4_data->f51_ctrl_04_val);
    tp_log_debug("%s: read dtsi value, f51_ctrl_04_val:0x%x \n", __func__, rmi4_data->f51_ctrl_04_low_sensitivity_val);
    rmi4_data->f01_ctrl_05_low_sensitivity_val = get_of_u32_val(dev_node, "huawei,f01_ctrl_05_low_sensitivity_val", rmi4_data->f01_ctrl_05_val);
    tp_log_debug("%s: read dtsi value, f01_ctrl_05_val:0x%x \n", __func__, rmi4_data->f01_ctrl_05_low_sensitivity_val);
    rmi4_data->f01_ctrl_09_low_sensitivity_val = get_of_u32_val(dev_node, "huawei,f01_ctrl_09_low_sensitivity_val", rmi4_data->f01_ctrl_09_val);
    tp_log_debug("%s: read dtsi value, f01_ctrl_09_val:0x%x \n", __func__, rmi4_data->f01_ctrl_09_low_sensitivity_val);

    return;
error:
    tp_log_err("%s: read sensitivity value failed!\n",__func__);

    return;
}
 /**
 * synaptics_rmi4_probe()
 *
 * Called by the kernel when an association with an I2C device of the
 * same name is made (after doing i2c_add_driver).
 *
 * This funtion allocates and initializes the resources for the driver
 * as an input driver, turns on the power to the sensor, queries the
 * sensor for its supported Functions and characteristics, registers
 * the driver to the input subsystem, sets up the interrupt, handles
 * the registration of the early_suspend and late_resume functions,
 * and creates a work queue for detection of other expansion Function
 * modules.
 */

static int synaptics_rmi4_probe(struct i2c_client *client,
        const struct i2c_device_id *dev_id)
{
    int retval;
    int ret = 0;
    int attr_count;
    struct synaptics_rmi4_data *rmi4_data;
    struct synaptics_dsx_platform_data *platform_data =
            client->dev.platform_data;

    if(already_has_tp_driver_running()) {
        tp_log_warning("%s:  Another tp driver is running!\n",__func__);
        return 0;
    }
    
    tp_log_info("%s: probe start...\n",__func__);
    if (!i2c_check_functionality(client->adapter,
            I2C_FUNC_SMBUS_BYTE_DATA)) {
        dev_err(&client->dev,
                "%s: SMBus byte data not supported\n",
                __func__);
        return -EIO;
    }

    if (client->dev.of_node) {
        platform_data = devm_kzalloc(&client->dev,
            sizeof(struct synaptics_dsx_platform_data), GFP_KERNEL);
        if (!platform_data) {
            dev_err(&client->dev, "%s: Failed to allocate memory for pdata\n", __func__);
            return -ENOMEM;
        }

        retval = synaptics_rmi4_parse_dt(&client->dev, platform_data);
        if (retval)
            return retval;
    }
    if (!platform_data) {
        dev_err(&client->dev,
                "%s: No platform data found\n",
                __func__);
        return -EINVAL;
    }

    rmi4_data = kzalloc(sizeof(*rmi4_data) * 2, GFP_KERNEL);
    if (!rmi4_data) {
        dev_err(&client->dev,
                "%s: Failed to alloc mem for rmi4_data\n",
                __func__);
        return -ENOMEM;
    }

    retval = hw_tp_power_init(&client->dev);
    if (retval)
    {
        tp_log_err("%s#%d: power init fail!\n",__func__,__LINE__);
        goto err_power_on;
    }
    hw_tp_power_on();
    
    /*delay at this place will cause INT to half high,please view the dts analyze process*/
    /*msleep(platform_data->power_delay_ms);*/

    retval = synaptics_rmi4_set_gpio(platform_data);
    if (retval < 0) {
        tp_log_err("%s: Failed to set up GPIO's\n",__func__);
        goto err_set_gpio;
    }

#if USE_IRQ_THREAD //huawei 11-25
    /* init a delay work for easy wakeup */
    INIT_DELAYED_WORK(&rmi4_data->delay_work, synaptics_rmi4_sensor_report_delay_work);
#else
    synaptics_wq = create_singlethread_workqueue("rmi_dsx_wq");
    if (!synaptics_wq)
    {
        tp_log_err("Could not create work queue rmi_dsx_wq: no memory");
        retval = -ENOMEM;
        goto error_wq_creat_failed; 
    }
    INIT_WORK(&rmi4_data->work, synaptics_rmi4_sensor_report);
#endif /*USE_IRQ_THREAD*/
    /* save platform data */
#ifdef CONFIG_HUAWEI_DSM
    tp_data = platform_data;
#endif/*CONFIG_HUAWEI_DSM*/

    rmi4_data->i2c_client = client;
    rmi4_data->current_page = MASK_8BIT;
    rmi4_data->board = platform_data;
    rmi4_data->touch_stopped = false;
    rmi4_data->sensor_sleep = false;
    rmi4_data->irq_enabled = false;
    rmi4_data->fingers_on_2d = false;
#if USE_WAKEUP_GESTURE
    rmi4_data->gesture_enabled = platform_data->gesture_enabled;
    rmi4_data->palm_enabled = false;
    rmi4_data->easy_wakeup_gesture = false;
    rmi4_data->sleep_gesture_flag = false;
    rmi4_data->glove_enabled = platform_data->glove_enabled;
    rmi4_data->holster_enabled = false;
    /*record default value from FW*/
    rmi4_data->fast_relax_normal = 0;
#endif /*USE_WAKEUP_GESTURE*/

    rmi4_data->i2c_read = synaptics_rmi4_i2c_read;
    rmi4_data->i2c_write = synaptics_rmi4_i2c_write;
    rmi4_data->irq_enable = synaptics_rmi4_irq_enable;
    rmi4_data->reset_device = synaptics_rmi4_reset_device;

    mutex_init(&(rmi4_data->rmi4_io_ctrl_mutex));
    mutex_init(&(rmi4_data->rmi4_reset_mutex));
    mutex_init(&(rmi4_data->rmi4_sysfs_mutex));
    mutex_init(&(rmi4_data->rmi4_holster_mutex));
    mutex_init(&(rmi4_data->rmi4_glove_mutex));
    mutex_init(&(rmi4_data->rmi4_window_mutex));

    i2c_set_clientdata(client, rmi4_data);

    retval = synaptics_rmi4_set_input_dev(rmi4_data);
    if (retval < 0) {
        dev_err(&client->dev,
                "%s: Failed to set up input device\n",
                __func__);
        goto err_set_input_dev;
    }

    /* registe dsm client */
#ifdef CONFIG_HUAWEI_DSM
    if (!tp_dclient) {
        dsm_i2c.fops->dump_func = dump;
        tp_dclient = dsm_register_client(&dsm_i2c);
    }
#endif/*CONFIG_HUAWEI_DSM*/

#if !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_FB)
    retval = configure_sleep(rmi4_data);
    if(retval)
    {
        tp_log_err("configure_sleep error:%s,line=%d,retval=%d\n", __func__, __LINE__,retval);
        goto err_configure_sleep;
    }
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
    rmi4_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    rmi4_data->early_suspend.suspend = synaptics_rmi4_early_suspend;
    rmi4_data->early_suspend.resume = synaptics_rmi4_late_resume;
    register_early_suspend(&rmi4_data->early_suspend);
#endif

    /* initialize pinctrl */
    retval = synaptics_pinctrl_init(rmi4_data);
    if (retval) {
        dev_err(&client->dev, "Can't initialize pinctrl\n");
            goto err_init_pinctrl;
    }

    retval = pinctrl_select_state(rmi4_data->pinctrl, rmi4_data->pin_active);
    if (retval) {
        dev_err(&client->dev,
            "Can't select pinctrl active state\n");
        goto err_init_pinctrl;
    }

    rmi4_data->irq = gpio_to_irq(platform_data->irq_gpio);

    retval = synaptics_rmi4_irq_enable(rmi4_data, true);
    if (retval < 0) {
        dev_err(&client->dev,
                "%s: Failed to enable attention interrupt\n",
                __func__);
        goto err_enable_irq;
    }

    if (!exp_data.initialized) {
        mutex_init(&exp_data.mutex);
        INIT_LIST_HEAD(&exp_data.list);
        exp_data.initialized = true;
    }

    exp_data.workqueue = create_singlethread_workqueue("dsx_exp_workqueue");

    if(NULL == exp_data.workqueue){
        tp_log_err("%s: failed to create workqueue for initialization and removal of "
                        "synaptics module\n", __func__);
        retval = -ENOMEM;
        goto err_create_workqueue;
    }
    
#if defined(CONFIG_PM) && defined(CONFIG_PM_RUNTIME)
    pm_runtime_get_noresume(&rmi4_data->i2c_client->dev);
    pm_runtime_set_active(&rmi4_data->i2c_client->dev);
    pm_runtime_enable(&rmi4_data->i2c_client->dev);
#endif/*CONFIG_PM*/

    for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
        retval = sysfs_create_file(&rmi4_data->input_dev->dev.kobj,
                &attrs[attr_count].attr);
        if (retval < 0) {
            dev_err(&client->dev,
                    "%s: Failed to create sysfs attributes\n",
                    __func__);
            goto err_sysfs;
        }
    }

#if USE_WAKEUP_GESTURE
    core_dev_ptr = &rmi4_data->input_dev->dev;
    retval = add_easy_wakeup_interfaces(&rmi4_data->input_dev->dev);
    if (retval < 0) {
        tp_log_err( "%s: Error, easy wakeup init sysfs fail! \n",
            __func__);
    }

    device_init_wakeup(&rmi4_data->input_dev->dev, 1);
#endif /*USE_WAKEUP_GESTURE*/
    /* The work should be queued after synaptics probed successfully*/
    INIT_DELAYED_WORK(&exp_data.work, synaptics_rmi4_exp_fn_work);
    exp_data.rmi4_data = rmi4_data;
    exp_data.queue_work = true;
    queue_delayed_work(exp_data.workqueue,
            &exp_data.work,
            msecs_to_jiffies(EXP_FN_WORK_DELAY_MS));
    
    if (platform_data->esd_support)
    {
        tp_log_info("%s %d, Init esd check\n", __func__, __LINE__);
        ret = synaptics_dsx_esd_init(rmi4_data, synaptics_rmi4_i2c_read, 
                                        synaptics_rmi4_i2c_write);
        if (ret == 0) {
            ret = synaptics_dsx_esd_start();
            if (ret < 0) {
            tp_log_err("%s %d:start esd check fail, ret = %d\n", __func__, 
                        __LINE__, ret);
            }
        } else {
            tp_log_err("%s %d:init tp esd fail, ret = %d\n", __func__, 
                        __LINE__, ret);
        }
    }
    else
    {
        tp_log_info("%s %d, not open esd check, flag = %d.\n", __func__, 
                    __LINE__, platform_data->esd_support);
    }

    synaptics_rmi4_read_default_glove_holster_sensitivity(rmi4_data);

    set_tp_driver_running();
    tp_log_info("%s#%d: probe success!\n",__func__,__LINE__);
    
    return retval;

err_sysfs:
    for (attr_count--; attr_count >= 0; attr_count--) {
        sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
                &attrs[attr_count].attr);
    }

#ifdef CONFIG_PM
#ifdef CONFIG_PM_RUNTIME
    pm_runtime_disable(&client->dev);
#endif/*CONFIG_PM_RUNTIME*/
#endif/*CONFIG_PM*/

    flush_workqueue(exp_data.workqueue);
    destroy_workqueue(exp_data.workqueue);
err_create_workqueue:
    synaptics_rmi4_irq_enable(rmi4_data, false);
err_enable_irq:
err_init_pinctrl:
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&rmi4_data->early_suspend);
#endif
#if !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_FB)
    fb_unregister_client(&rmi4_data->fb_notif);
#endif
err_configure_sleep:
    synaptics_rmi4_empty_fn_list(rmi4_data);
    input_unregister_device(rmi4_data->input_dev);
    rmi4_data->input_dev = NULL;
err_set_input_dev:
    synaptics_rmi4_release_gpio(platform_data);

#ifndef CONFIG_HUAWEI_KERNEL
    if (platform_data->power_gpio >= 0) {
        synaptics_gpio_setup(
                platform_data->power_gpio,
                false, 0, 0);
    }
#endif /*CONFIG_HUAWEI_KERNEL*/

#if    USE_IRQ_THREAD //huawei 11-25
#else        
error_wq_creat_failed:
    if (synaptics_wq)
    {
        destroy_workqueue(synaptics_wq);
    }
#endif /*USE_IRQ_THREAD*/

err_set_gpio:
    if (rmi4_data->regulator) {
        regulator_disable(rmi4_data->regulator);
        regulator_put(rmi4_data->regulator);
    }
    hw_tp_power_release();
err_power_on:
#ifdef ENABLE_VIRTUAL_KEYS//huawei 11-25
    free_virtual_keys(&platform_data->vkeys);
#endif//huawei 11-25
    free_wakeup_keys(platform_data->wakeup_keys);
    kfree(rmi4_data);
    if (client->dev.of_node) {
        if (platform_data) {
            devm_kfree(&client->dev, platform_data);
        }
    }

    return retval;
}

 /**
 * synaptics_rmi4_remove()
 *
 * Called by the kernel when the association with an I2C device of the
 * same name is broken (when the driver is unloaded).
 *
 * This funtion terminates the work queue, stops sensor data acquisition,
 * frees the interrupt, unregisters the driver from the input subsystem,
 * turns off the power to the sensor, and frees other allocated resources.
 */
static int synaptics_rmi4_remove(struct i2c_client *client)
{
    unsigned char attr_count;
    struct synaptics_rmi4_data *rmi4_data = i2c_get_clientdata(client);
    const struct synaptics_dsx_platform_data *platform_data =
            rmi4_data->board;

    for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
        sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
                &attrs[attr_count].attr);
    }

#ifdef CONFIG_PM
#ifdef CONFIG_PM_RUNTIME
    pm_runtime_disable(&client->dev);
#endif/*CONFIG_PM_RUNTIME*/
#endif/*CONFIG_PM*/
    cancel_delayed_work_sync(&exp_data.work);
    flush_workqueue(exp_data.workqueue);
    destroy_workqueue(exp_data.workqueue);

    synaptics_rmi4_irq_enable(rmi4_data, false);

#if !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_FB)
    fb_unregister_client(&rmi4_data->fb_notif);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&rmi4_data->early_suspend);
#endif

    synaptics_rmi4_empty_fn_list(rmi4_data);
    input_unregister_device(rmi4_data->input_dev);
    rmi4_data->input_dev = NULL;

    if (platform_data->irq_gpio) {
        synaptics_gpio_setup(
                platform_data->irq_gpio,
                false, 0, 0);
    }

    if (platform_data->reset_gpio >= 0) {
        tp_log_err("%s: reset_false ",__func__);
        synaptics_gpio_setup(
                platform_data->reset_gpio,
                false, 0, 0);
    }

#ifndef CONFIG_HUAWEI_KERNEL
    if (platform_data->power_gpio >= 0) {
        synaptics_gpio_setup(
                platform_data->power_gpio,
                false, 0, 0);
    }
#endif /*CONFIG_HUAWEI_KERNEL*/

    if (rmi4_data->regulator) {
        regulator_disable(rmi4_data->regulator);
        regulator_put(rmi4_data->regulator);
    }

    kfree(rmi4_data);

    return 0;
}

#if USE_WAKEUP_GESTURE
static int synaptics_set_config_reg(struct synaptics_rmi4_data *rmi4_data,unsigned char fast_relax)
{
    int retval;
    unsigned char device_cmd;

    tp_log_debug( "%s:write fast_relax= %d\n", __func__,fast_relax);
    
    retval = synaptics_rmi4_i2c_write(rmi4_data,
            rmi4_data->f54_ctrl_base_addr+F54_ANALOG_CTRL13_OFFSET,
            &fast_relax,
            sizeof(fast_relax));
    if (retval < 0) {
        tp_log_err("%s: Failed to write reg(0x%04x),retval=%d\n",
                __func__,rmi4_data->f54_ctrl_base_addr+F54_ANALOG_CTRL13_OFFSET,retval);
        return retval;
    }

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f54_cmd_base_addr,
            &device_cmd,
            sizeof(device_cmd));
    if (retval < 0) {
        tp_log_err("%s: Failed to read reg(0x%04x),retval=%d\n",
                __func__,rmi4_data->f54_cmd_base_addr,retval);
        return retval;
    }

    /*write 0100 to f54_cmd_00 will force update reg-value to TP ram */
    device_cmd = (device_cmd | 0x04);
    retval = synaptics_rmi4_i2c_write(rmi4_data,
            rmi4_data->f54_cmd_base_addr,
            &device_cmd,
            sizeof(device_cmd));
    if (retval < 0) {
        tp_log_err("%s: Failed to write reg(0x%04x),retval=%d\n",
                __func__,rmi4_data->f54_cmd_base_addr,retval);
        return retval;
    }
    
    return 0;
}

static int synaptics_set_nosleep_mode(struct synaptics_rmi4_data *rmi4_data,unsigned char nosleep)
{
    int retval;
    unsigned char device_ctrl;

    /*Read F01_RMI_CTRL00 to get nosleep mode*/
    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f01_ctrl_base_addr,
            &device_ctrl,
            sizeof(device_ctrl));
    if (retval < 0) {
        tp_log_err("%s: Failed to read sleep mode\n",
                __func__);
        return retval;
    }

    device_ctrl = (device_ctrl & ~MASK_BIT3);
    device_ctrl = (device_ctrl | nosleep);

    /*Write F01_RMI_CTRL00 to set nosleep mode*/
    retval = synaptics_rmi4_i2c_write(rmi4_data,
            rmi4_data->f01_ctrl_base_addr,
            &device_ctrl,
            sizeof(device_ctrl));
    if (retval < 0) {
        tp_log_err("%s: Failed to enter sleep mode\n",
                __func__);
        return retval;
    }

    return 0;
}

/* Set doze wakeup threshold reg for better performance */
static int synaptics_set_doze_wakeup_threshold(struct synaptics_rmi4_data *rmi4_data,unsigned char threshold)
{
    int retval;
    unsigned char device_ctrl = threshold;

    tp_log_debug("%s:set doze_wakeup_threshold = %d\n",
        __func__,threshold);

    /*Write F01_RMI_CTRL03 to set doze_wakeup_threshold*/
    retval = synaptics_rmi4_i2c_write(rmi4_data,
            rmi4_data->f01_ctrl_base_addr + F01_RMI_CTRL03_OFFSET,
            &device_ctrl,
            sizeof(device_ctrl));
    if (retval < 0) {
        tp_log_err("%s: Failed to set doze_wakeup_threshold\n",
                __func__);
        return retval;
    }

    return 0;
}

static void f11_open_easy_wakeup(struct synaptics_rmi4_data *rmi4_data)
{
    int retval;
    unsigned char device_ctrl;
    unsigned char fast_relax;
    unsigned char nosleep = NO_SLEEP_OFF;
    /*Doze wakeup threshold for gesture mode */
    unsigned char threshold = rmi4_data->dwt_gesture;

    tp_log_warning("%s: in \n",__func__);
    if (true == rmi4_data->sleep_gesture_flag)
    {
        tp_log_debug( "%s gesture already enabled,return directly!\n", __func__);
        return;
    }

    if (0 != rmi4_data->board->fast_relax_gesture)
    {
        if (0 == rmi4_data->fast_relax_normal)
        {
            /*Record synap original config,which will never be zero*/
            retval = synaptics_rmi4_i2c_read(rmi4_data,
                    rmi4_data->f54_ctrl_base_addr+F54_ANALOG_CTRL13_OFFSET,
                    &fast_relax,
                    sizeof(fast_relax));
            if (retval < 0) {
                tp_log_err("%s: Failed to read reg(0x%04x),retval=%d\n",
                        __func__,rmi4_data->f54_ctrl_base_addr+F54_ANALOG_CTRL13_OFFSET,retval);
            }
            else{
                rmi4_data->fast_relax_normal = fast_relax;
            }
        }
        
        retval = synaptics_set_config_reg(rmi4_data,rmi4_data->board->fast_relax_gesture);
        if (retval < 0) {
            tp_log_err("%s: Failed to set config reg,retval=%d\n",
                    __func__,retval);
        }
    }

    tp_log_debug("%s: glove_enabled=%d \n", __func__, rmi4_data->glove_enabled);
    /*In easy wakeup mode,exit glove mode if app has enabled it */
    /*In EmUi3.0, holster switch will also change glove state, so we also */
    /* need to recognize this state by driver flag to exit glove mode for */
    /* temporary, to make sure that gesture will only response to finger */
    if ((RMI4_GLOV_FUNC_ENABLED == rmi4_data->glove_enabled)||
        (true == holster_enable_glove))
    {
        /*TP will only response to finger in easy wakeup*/
        tp_log_warning("%s: line=%d\n",__func__,__LINE__);
        retval = F51_set_glove_reg(rmi4_data,SYSTEM_LOCKED_TO_SKIN_MODE);
        if (retval < 0) {
            tp_log_err("%s: Failed to set glove mode!retval=%d\n",
                    __func__,retval);
        }
    }

    /*Exit holster mode in easy wakeup if app has enabled it */
    if (true == rmi4_data->holster_enabled)
    {
        /*Exit holster mode in easy wakeup mode*/
        retval = set_holster_mode(rmi4_data,
                rmi4_data->f51_ctrl_base_addr + F51_CUSTOM_CTRL63_OFFSET,
                RMI4_HOLSTER_FUNC_DISABLED);
        if (retval < 0) {
            tp_log_err("%s: Failed to exit holster mode\n",
                    __func__);
        } else{
            tp_log_warning("%s: holster is Disabled!\n", __func__);
        }
    }

    if(APP_ENABLE_GESTURE(GESTURE_DOUBLE_CLICK)&rmi4_data->easy_wakeup_gesture)
    {
        /* Write double-tap zone to F11_2D_CTRL92(01)/00~(01)/05*/
        retval = set_effective_window(rmi4_data,&double_tap_info);
        if (retval < 0)
        {
            tp_log_err("%s:double_tap_zone set error!ret = %d\n",__func__,retval);
        }
    }

    /*Exit nosleep mode to reduce power_consumption*/
    retval = synaptics_set_nosleep_mode(rmi4_data,nosleep);
    if (retval < 0) {
        tp_log_err("%s: Failed to set_nosleep_mode off,retval=%d\n",
                __func__,retval);
    }

    /*Set doze wakeup threshold reg for gesture mode if necessary */
    if(threshold)
    {
        retval = synaptics_set_doze_wakeup_threshold(rmi4_data,threshold);
        if (retval < 0) {
            tp_log_err("%s: Failed to set doze_wakeup_threshold,retval=%d\n",
                    __func__,retval);
        }
    }
    
    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f11_ctrl_base_addr,
            &device_ctrl,
            sizeof(device_ctrl));
    if (retval < 0) {
        dev_err(&(rmi4_data->input_dev->dev),
                "%s: Failed to read reg,retval=%d\n",
                __func__,retval);
        rmi4_data->sensor_sleep = false;
        return;
    }

    /*write 0100 to f11_ctrl_00 will not sleep but into gesture mode*/
    device_ctrl = (device_ctrl & ~MASK_3BIT);

    /* Into gesture mode */
    device_ctrl = (device_ctrl | F11_GESTURE_MODE);

    retval = synaptics_rmi4_i2c_write(rmi4_data,
            rmi4_data->f11_ctrl_base_addr,
            &device_ctrl,
            sizeof(device_ctrl));
    if (retval < 0) {
        dev_err(&(rmi4_data->input_dev->dev),
                "%s: Failed to write reg,retval=%d\n",
                __func__,retval);
        rmi4_data->sensor_sleep = false;
        return;
    } else {
        rmi4_data->sensor_sleep = true;
    }

    if (device_may_wakeup(&rmi4_data->input_dev->dev)) 
    {
        if (!enable_irq_wake(rmi4_data->irq))
        {
            synaptics_rmi4_irq_enable(rmi4_data, true);
            rmi4_data->sleep_gesture_flag = true;
            rmi4_data->touch_stopped = false;
        }
    } 

    tp_log_debug( "%s out: sleep_gesture_flag=%d\n", __func__,rmi4_data->sleep_gesture_flag);

    return;
}

static void f12_open_easy_wakeup(struct synaptics_rmi4_data *rmi4_data)
{
    int retval;
    unsigned char device_ctrl[4] = {0};

    tp_log_warning("%s: in \n",__func__);

    if (true == rmi4_data->sleep_gesture_flag)
    {
        tp_log_debug( "%s gesture already enabled,return directly!\n", __func__);
        return;
    }

    /*reg Report Wakeup Gesture Only read first*/
    retval = synaptics_rmi4_i2c_read(rmi4_data, 
            rmi4_data->f12_ctrl_base_addr+rmi4_data->f12_2d_ctrl20_lpm, 
            &device_ctrl[0], 
            sizeof(device_ctrl));
    if (retval < 0) {
        tp_log_err("%s:read Wakeup Gesture Only reg F12_2D_CTRL20 fail !\n",__func__);
    } else {
        tp_log_debug("%s:read Wakeup Gesture Only reg F12_2D_CTRL20(00)/00 \n"
            "Motion Suppression : \n"
            "CTRL20(00)/00 : 0x%x  CTRL20(00)/01 :0x%x \n" 
            "CTRL20(01)/00 : 0x%x CTRL20(04)/00 : 0x%x \n",__func__,
            device_ctrl[0], device_ctrl[1], device_ctrl[2], device_ctrl[3]);
    }

    /*Report Wakeup Gesture Only set bit 1 */
    device_ctrl[2] = device_ctrl[2] | MASK_BIT2;

    /*Wakeup Gesture open, bit(01) set 1*/
    retval = synaptics_rmi4_i2c_write(rmi4_data, 
            rmi4_data->f12_ctrl_base_addr+rmi4_data->f12_2d_ctrl20_lpm, 
            &device_ctrl[0], 
            sizeof(device_ctrl));
    if (retval < 0) {
        rmi4_data->sensor_sleep = false;
        tp_log_err("%s:write Wakeup Gesture reg fail!\n",__func__);
    } else {
        rmi4_data->sensor_sleep = true;
        tp_log_info("%s:write Wakeup Gesture reg OK!\n"
            "Reg_address(0x%2x),Reg_value(0x%2x)\n",__func__ ,
            rmi4_data->f12_ctrl_base_addr+rmi4_data->f12_2d_ctrl20_lpm, device_ctrl[2]);
    }

    return;
}
static void synaptics_open_easy_wakeup(struct synaptics_rmi4_data *rmi4_data)
{
    struct synaptics_dsx_platform_data *platform_data =
            rmi4_data->board;
    unsigned int ic_type = platform_data->ic_type;

    tp_log_warning("%s: in \n",__func__);

    if (SYNAPTICS_S3320 == ic_type 
         || SYNAPTICS_S2716 == ic_type) 
    {
        f12_open_easy_wakeup(rmi4_data);
    } else {
        f11_open_easy_wakeup(rmi4_data);//S3207 use default
    }

    if (device_may_wakeup(&rmi4_data->input_dev->dev)) 
    {
        if (!enable_irq_wake(rmi4_data->irq))
        {
            synaptics_rmi4_irq_enable(rmi4_data, true);
            rmi4_data->sleep_gesture_flag = true;
            rmi4_data->touch_stopped = false;
        }
    } 

    tp_log_debug( "%s out: sleep_gesture_flag=%d\n", 
        __func__,rmi4_data->sleep_gesture_flag);

    return;
}

static void f11_close_easy_wakeup(struct synaptics_rmi4_data *rmi4_data)
{
    int retval;
    unsigned char device_ctrl;
    unsigned char command = 0x01;//write command=1 to reset IC
    /*Use default(doze usually) config when wake,to reduce power consumption*/
    unsigned char nosleep = rmi4_data->no_sleep_setting;
    /*Doze wakeup threshold for wakeup mode */
    unsigned char threshold = rmi4_data->dwt_default;

    tp_log_warning("%s: in ",__func__);

    if (false == rmi4_data->sleep_gesture_flag)
    {
        tp_log_debug( "%s gesture already disabled,return directly!\n", __func__);
        return;
    }

    /* Reset to recalibrate TP */
    mutex_lock(&(rmi4_data->rmi4_reset_mutex));
    retval = synaptics_rmi4_i2c_write(rmi4_data,
            rmi4_data->f01_cmd_base_addr,
            &command,
            sizeof(command));
    if (retval < 0) {
        tp_log_err("%s: Failed to issue reset command, error = %d\n",
                __func__, retval);
        mutex_unlock(&(rmi4_data->rmi4_reset_mutex));
        return;
    }
    mutex_unlock(&(rmi4_data->rmi4_reset_mutex));

    /* Waiting for reset finish */
    msleep(rmi4_data->board->reset_delay_ms);

    if (0 != rmi4_data->fast_relax_normal)
    {
        /*Write back synap original config*/
        retval = synaptics_set_config_reg(rmi4_data,rmi4_data->fast_relax_normal);
        if (retval < 0) {
            tp_log_err("%s: Failed to set config reg,retval=%d\n",
                    __func__,retval);
            return;
        }
    }

    /*Enter nosleep mode to increase self_relax*/
    retval = synaptics_set_nosleep_mode(rmi4_data,nosleep);
    if (retval < 0) {
        tp_log_err("%s: Failed to set_nosleep_mode on,retval=%d\n",
                __func__,retval);
    }

    /*Set doze wakeup threshold reg back to default value */
    if(threshold)
    {
        retval = synaptics_set_doze_wakeup_threshold(rmi4_data,threshold);
        if (retval < 0) {
            tp_log_err("%s: Failed to set doze_wakeup_threshold,retval=%d\n",
                    __func__,retval);
        }
    }

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f11_ctrl_base_addr,
            &device_ctrl,
            sizeof(device_ctrl));
    if (retval < 0) {
        tp_log_err("%s: Failed to read reg,retval=%d\n",
                __func__,retval);
        rmi4_data->sensor_sleep = true;
        return;
    }

    /*write 0000 to f11_ctrl_00 will get into normal mode*/
    device_ctrl = (device_ctrl & ~MASK_3BIT);
    /* Into reduce mode which can reduce power consumption*/
    device_ctrl = (device_ctrl | F11_REDUCE_MODE);

    retval = synaptics_rmi4_i2c_write(rmi4_data,
            rmi4_data->f11_ctrl_base_addr,
            &device_ctrl,
            sizeof(device_ctrl));
    if (retval < 0) {
        tp_log_err("%s: Failed to write reg,retval=%d\n",
                __func__,retval);
        rmi4_data->sensor_sleep = true;
        return;
    } else {
        rmi4_data->sensor_sleep = false;
    }

    return;
}

static void f12_close_easy_wakeup(struct synaptics_rmi4_data *rmi4_data)
{
    int retval;
    unsigned char device_ctrl[4] = {0};

    tp_log_warning("%s: in ",__func__);

    if (false == rmi4_data->sleep_gesture_flag)
    {
        tp_log_debug( "%s gesture already disabled,return directly!\n", __func__);
        return;
    }

    /*reg Report Wakeup Gesture Only read first*/
    retval = synaptics_rmi4_i2c_read(rmi4_data, 
            rmi4_data->f12_ctrl_base_addr+rmi4_data->f12_2d_ctrl20_lpm, 
            &device_ctrl[0], 
            sizeof(device_ctrl));
    if (retval < 0) {
        tp_log_err("%s:read Wakeup Gesture Only reg F12_2D_CTRL20 fail !\n",__func__);
    } else {
        tp_log_debug("%s:read Wakeup Gesture Only reg F12_2D_CTRL20(00)/00 \n"
            "Motion Suppression :\n" 
            "CTRL20(00)/00 : 0x%x  CTRL20(00)/01 : 0x%x \n"
            "CTRL20(01)/00 : 0x%x CTRL20(04)/00 : 0x%x \n", __func__,
            device_ctrl[0], device_ctrl[1], device_ctrl[2], device_ctrl[3]);
    }

    /*Report Wakeup Gesture Only set bit 1 */
    device_ctrl[2] = device_ctrl[2] & ~MASK_BIT2;

    /*Wakeup Gesture close,bit(01) set 0*/
    retval = synaptics_rmi4_i2c_write(rmi4_data, 
            rmi4_data->f12_ctrl_base_addr+rmi4_data->f12_2d_ctrl20_lpm, 
            &device_ctrl[0], 
            sizeof(device_ctrl));
    if (retval < 0) {
        rmi4_data->sensor_sleep = true;
        tp_log_err("%s:write Wakeup Gesture reg fail\n",__func__);
    } else {
        rmi4_data->sensor_sleep = false;
        tp_log_info("%s:write Wakeup Gesture reg OK \n" 
            "address(0x%2x) value(0x%2x)\n", __func__,
            rmi4_data->f12_ctrl_base_addr+rmi4_data->f12_2d_ctrl20_lpm, device_ctrl[2]);
    }

    return;
}

static void synaptics_close_easy_wakeup(struct synaptics_rmi4_data *rmi4_data)
{
    struct synaptics_dsx_platform_data *platform_data =
            rmi4_data->board;
    unsigned int ic_type = platform_data->ic_type;

    tp_log_warning("%s: in \n",__func__);

    if (SYNAPTICS_S3320 == ic_type
         || SYNAPTICS_S2716 == ic_type)
    {
        f12_close_easy_wakeup(rmi4_data);
    } else {
        f11_close_easy_wakeup(rmi4_data);//S3207 use default
    }

    if (device_may_wakeup(&rmi4_data->input_dev->dev)) 
    {
        if (!disable_irq_wake(rmi4_data->irq))
        {
            rmi4_data->sleep_gesture_flag = false;
            if( ( true == easy_wakeup_process) )
            {
                cancel_delayed_work_sync(&rmi4_data->delay_work);
                easy_wakeup_process = false;
            }
            synaptics_rmi4_irq_enable(rmi4_data, false);
        }
    } 

    tp_log_debug( "%s out:sleep_gesture_flag=%d\n", 
        __func__,rmi4_data->sleep_gesture_flag);

    return;
}
#endif /*USE_WAKEUP_GESTURE*/

#ifdef CONFIG_PM
 /**
 * synaptics_rmi4_sensor_sleep()
 *
 * Called by synaptics_rmi4_early_suspend() and synaptics_rmi4_suspend().
 *
 * This function stops finger data acquisition and puts the sensor to sleep.
 */
static void synaptics_rmi4_sensor_sleep(struct synaptics_rmi4_data *rmi4_data)
{
    int retval;
    unsigned char device_ctrl;

    tp_log_info("%s: in ",__func__);

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f01_ctrl_base_addr,
            &device_ctrl,
            sizeof(device_ctrl));
    if (retval < 0) {
        tp_log_err("%s: Failed to read sleep mode\n",
                __func__);
        rmi4_data->sensor_sleep = false;
        return;
    }

    device_ctrl = (device_ctrl & ~MASK_3BIT);
    device_ctrl = (device_ctrl | NO_SLEEP_OFF | SENSOR_SLEEP);

    retval = synaptics_rmi4_i2c_write(rmi4_data,
            rmi4_data->f01_ctrl_base_addr,
            &device_ctrl,
            sizeof(device_ctrl));
    if (retval < 0) {
        tp_log_err("%s: Failed to write sleep mode\n",
                __func__);
        rmi4_data->sensor_sleep = false;
        return;
    } else {
        rmi4_data->sensor_sleep = true;
    }

    return;
}

 /**
 * synaptics_rmi4_sensor_wake()
 *
 * Called by synaptics_rmi4_resume() and synaptics_rmi4_late_resume().
 *
 * This function wakes the sensor from sleep.
 */
static void synaptics_rmi4_sensor_wake(struct synaptics_rmi4_data *rmi4_data)
{
    int retval;
    unsigned char device_ctrl;
    unsigned char no_sleep_setting = rmi4_data->no_sleep_setting;

    tp_log_warning("%s: in \n",__func__);

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f01_ctrl_base_addr,
            &device_ctrl,
            sizeof(device_ctrl));
    if (retval < 0) {
        tp_log_err("%s: Failed to read reg,retval=%d\n",
                __func__,retval);
        rmi4_data->sensor_sleep = true;
        return;
    }

    device_ctrl = (device_ctrl & ~MASK_3BIT);
    device_ctrl = (device_ctrl | no_sleep_setting | NORMAL_OPERATION);

    retval = synaptics_rmi4_i2c_write(rmi4_data,
            rmi4_data->f01_ctrl_base_addr,
            &device_ctrl,
            sizeof(device_ctrl));
    if (retval < 0) {
        tp_log_err("%s: Failed to write reg,retval=%d\n",
                __func__,retval);
        rmi4_data->sensor_sleep = true;
        return;
    } else {
        rmi4_data->sensor_sleep = false;
    }

    return;
}


static void synaptics_set_sensor_sleep_mode(struct synaptics_rmi4_data *rmi4_data)
{
#if USE_WAKEUP_GESTURE
    if (true == rmi4_data->gesture_enabled) {
        synaptics_open_easy_wakeup(rmi4_data);/* Enter gesture mode */
        return;
    }
#endif
    synaptics_rmi4_sensor_sleep(rmi4_data);/* Enter sleep mode */
    return;
}

static void synaptics_wake_recovery(struct synaptics_rmi4_data *rmi4_data)
{
    int retval = 0;
    const struct synaptics_dsx_platform_data *platform_data =
            rmi4_data->board;
    unsigned int ic_type = platform_data->ic_type;
    /* If glove has been enabled by app before tp was sleep,              */
    /* we should enable glove again when resume tp.                       */
    /*In EmUi3.0, holster switch will also change glove state, so we also */
    /* need to recognize this state by driver flag to restore glove mode, */
    /* to make sure that glove is in the same mode as before suspend.     */
    if ((RMI4_GLOV_FUNC_ENABLED == rmi4_data->glove_enabled)||
        (true == holster_enable_glove))
    {
        if (true == rmi4_data->holster_enabled)
        {
            /*TP will only response to glove in holster mode*/
            tp_log_debug("%s: line=%d\n",__func__,__LINE__);
            if (SYNAPTICS_S3320 == ic_type)
            {
                retval = F12_set_glove_mode(rmi4_data,RMI4_GLOV_FUNC_ENABLED);
            }
            else
            {
                retval = F51_set_glove_reg(rmi4_data,SYSTEM_LOCKED_TO_GLOVE_MODE);
            }
        }
        else
        {
            /*TP will response to finger and glove,with finger default*/
            tp_log_debug("%s: line=%d\n",__func__,__LINE__);
            if (SYNAPTICS_S3320 == ic_type)
            {
                retval = F12_set_glove_mode(rmi4_data,RMI4_GLOV_FUNC_ENABLED);
            }
            else
            {
                retval = F51_set_glove_reg(rmi4_data,SYSTEM_START_IN_SKIN_MODE);
            }
            /**
             * if dtsi enable glove edge switch and glove state switch of edge is not running ,
             * we need to enable glove mode to finger mode on the edge of TP
             * and schedule a delayed work
            */
            if((true == platform_data->glove_edge_switch_supported) &&
                (false == edge_glove_switch_process))
            {
                /*enable glove mode to finger mode on the edge of TP*/
                retval = set_edge_glove_state(rmi4_data, false);
                if(retval < 0)
                {
                    tp_log_err("%s: Failed to enable edge glove state\n",
                    __func__);
                }
                else
                {
                    edge_glove_switch_process = true;
                    schedule_delayed_work(&rmi4_data->glove_delay_work, msecs_to_jiffies(DELAY_TIME_ENABLE_EDGE_GLOVE_SWITCH));
                }
            }
        }
    }

    /*
     * If holster has been enabled by app before tp was sleep,
     * we should enable holster again when resume tp,
     * in case that press power key times with holster covered .
    */
    if (true == rmi4_data->holster_enabled)
    {
        /* Write holster window zone to F11_2D_CTRL92(01)/00~(01)/05*/
        retval = set_effective_window(rmi4_data,&synap_holster_info);
        if (retval < 0)
        {
            tp_log_err("%s:holster_zwindow_zone write error!ret = %d\n",
                    __func__,retval);
        }

        /*Enable holster mode*/
        retval = set_holster_mode(rmi4_data,
                rmi4_data->f51_ctrl_base_addr + F51_CUSTOM_CTRL63_OFFSET,
                RMI4_HOLSTER_FUNC_ENABLED);
        if (retval < 0) {
            tp_log_err("%s: Failed to enter holster mode\n",
                    __func__);
        } else{
            tp_log_warning("%s: holster is Enabled!\n", __func__);
        }
    }


    return;
}

static void synaptics_set_sensor_wake_mode(struct synaptics_rmi4_data *rmi4_data)
{
    int retval = 0;

    if (true == rmi4_data->gesture_enabled) {
#if USE_WAKEUP_GESTURE
        synaptics_close_easy_wakeup(rmi4_data);/* Exit gesture mode */
#endif
     } else {
        synaptics_rmi4_sensor_wake(rmi4_data);/* Exit sleep mode */
    }

    retval = synaptics_rmi4_reinit_device(rmi4_data);
    if (retval < 0) {
        tp_log_err("%s: Failed to reinit device,retval =%d\n",
            __func__,retval);
        return;
    }
    

    synaptics_wake_recovery(rmi4_data);


    return;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
 /**
 * synaptics_rmi4_early_suspend()
 *
 * Called by the kernel during the early suspend phase when the system
 * enters suspend.
 *
 * This function calls synaptics_rmi4_sensor_sleep() to stop finger
 * data acquisition and put the sensor to sleep.
 */
static void synaptics_rmi4_early_suspend(struct early_suspend *h)
{
    struct synaptics_rmi4_exp_fhandler *exp_fhandler;
    struct synaptics_rmi4_data *rmi4_data =
            container_of(h, struct synaptics_rmi4_data,
            early_suspend);

    if (rmi4_data->stay_awake) {
        rmi4_data->staying_awake = true;
        return;
    } else {
        rmi4_data->staying_awake = false;
    }

    rmi4_data->touch_stopped = true;
    synaptics_rmi4_irq_enable(rmi4_data, false);
    synaptics_rmi4_sensor_sleep(rmi4_data);
    synaptics_rmi4_free_fingers(rmi4_data);

    mutex_lock(&exp_data.mutex);
    if (!list_empty(&exp_data.list)) {
        list_for_each_entry(exp_fhandler, &exp_data.list, link)
            if (exp_fhandler->exp_fn->early_suspend != NULL)
                exp_fhandler->exp_fn->early_suspend(rmi4_data);
    }
    mutex_unlock(&exp_data.mutex);

    if (rmi4_data->full_pm_cycle)
        synaptics_rmi4_suspend(&(rmi4_data->input_dev->dev));

    return;
}

 /**
 * synaptics_rmi4_late_resume()
 *
 * Called by the kernel during the late resume phase when the system
 * wakes up from suspend.
 *
 * This function goes through the sensor wake process if the system wakes
 * up from early suspend (without going into suspend).
 */
static void synaptics_rmi4_late_resume(struct early_suspend *h)
{
    int retval;
    struct synaptics_rmi4_exp_fhandler *exp_fhandler;
    struct synaptics_rmi4_data *rmi4_data =
            container_of(h, struct synaptics_rmi4_data,
            early_suspend);

    if (rmi4_data->staying_awake)
        return;

    if (rmi4_data->full_pm_cycle)
        synaptics_rmi4_resume(&(rmi4_data->input_dev->dev));

    if (rmi4_data->sensor_sleep == true) {
        synaptics_rmi4_sensor_wake(rmi4_data);
        synaptics_rmi4_irq_enable(rmi4_data, true);
        retval = synaptics_rmi4_reinit_device(rmi4_data);
        if (retval < 0) {
            dev_err(&rmi4_data->i2c_client->dev,
                    "%s: Failed to reinit device\n",
                    __func__);
        }
    }

    mutex_lock(&exp_data.mutex);
    if (!list_empty(&exp_data.list)) {
        list_for_each_entry(exp_fhandler, &exp_data.list, link)
            if (exp_fhandler->exp_fn->late_resume != NULL)
                exp_fhandler->exp_fn->late_resume(rmi4_data);
    }
    mutex_unlock(&exp_data.mutex);

    rmi4_data->touch_stopped = false;

    return;
}
#endif

 /**
 * synaptics_rmi4_suspend()
 *
 * Called by the kernel during the suspend phase when the system
 * enters suspend.
 *
 * This function stops finger data acquisition and puts the sensor to
 * sleep (if not already done so during the early suspend phase),
 * disables the interrupt, and turns off the power to the sensor.
 */
static int synaptics_rmi4_suspend(struct device *dev)
{
    struct synaptics_rmi4_exp_fhandler *exp_fhandler;
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

    tp_log_warning("%s: in \n",__func__);
    
    if (rmi4_data->staying_awake)
        return 0;

    if (!rmi4_data->sensor_sleep) {
        rmi4_data->touch_stopped = true;
        synaptics_rmi4_irq_enable(rmi4_data, false);
        synaptics_set_sensor_sleep_mode(rmi4_data);
        synaptics_rmi4_free_fingers(rmi4_data);
    }

    mutex_lock(&exp_data.mutex);
    if (!list_empty(&exp_data.list)) {
        list_for_each_entry(exp_fhandler, &exp_data.list, link)
            if (exp_fhandler->exp_fn->suspend != NULL)
                exp_fhandler->exp_fn->suspend(rmi4_data);
    }
    mutex_unlock(&exp_data.mutex);

    if (rmi4_data->regulator)
        regulator_disable(rmi4_data->regulator);

    tp_log_debug("%s: out\n",__func__);
    return 0;
}

 /**
 * synaptics_rmi4_resume()
 *
 * Called by the kernel during the resume phase when the system
 * wakes up from suspend.
 *
 * This function turns on the power to the sensor, wakes the sensor
 * from sleep, enables the interrupt, and starts finger data
 * acquisition.
 */
static int synaptics_rmi4_resume(struct device *dev)
{
    int retval = 0;
    struct synaptics_rmi4_exp_fhandler *exp_fhandler;
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
    const struct synaptics_dsx_platform_data *platform_data =
            rmi4_data->board;

    tp_log_warning("%s#%d: in ",__func__,__LINE__);
    if (rmi4_data->staying_awake)
        return 0;

    if (rmi4_data->regulator) {
        retval = regulator_enable(rmi4_data->regulator);
        if (retval) {
            dev_err(&rmi4_data->i2c_client->dev,
                "Regulator enable failed rc=%d\n",
                retval);
        }
        msleep(platform_data->reset_delay_ms);
        rmi4_data->current_page = MASK_8BIT;
    }

    synaptics_set_sensor_wake_mode(rmi4_data);
    retval = synaptics_rmi4_reinit_device(rmi4_data);
    if (retval < 0) {
        tp_log_err("%s: Failed to reinit device LINE = %d\n",__func__,__LINE__);
        return retval;
    }

    mutex_lock(&exp_data.mutex);
    if (!list_empty(&exp_data.list)) {
        list_for_each_entry(exp_fhandler, &exp_data.list, link)
            if (exp_fhandler->exp_fn->resume != NULL)
                exp_fhandler->exp_fn->resume(rmi4_data);
    }
    mutex_unlock(&exp_data.mutex);

    rmi4_data->touch_stopped = false;
    synaptics_rmi4_irq_enable(rmi4_data, true);
    tp_log_debug("%s: out\n",__func__);

    return 0;
}

#ifdef CONFIG_PM_RUNTIME
void synaptics_pm_rt_get(struct synaptics_rmi4_data *rmi4_data){
    tp_log_info("%s:line%d , rt_counter=%d\n",__func__,__LINE__, 
        rmi4_data->i2c_client->dev.power.usage_count.counter);
    pm_runtime_get_sync(&rmi4_data->i2c_client->dev);
    tp_log_info("%s:line%d , rt_counter=%d\n",__func__,__LINE__, 
        rmi4_data->i2c_client->dev.power.usage_count.counter);
}

void synaptics_pm_rt_put(struct synaptics_rmi4_data *rmi4_data){
    tp_log_info("%s:line%d , rt_counter=%d\n",__func__,__LINE__, 
        rmi4_data->i2c_client->dev.power.usage_count.counter);
    pm_runtime_put(&rmi4_data->i2c_client->dev);
    tp_log_info("%s:line%d , rt_counter=%d\n",__func__,__LINE__, 
        rmi4_data->i2c_client->dev.power.usage_count.counter);
}

 /**
 * synaptics_rmi4_rt_suspend()
 */
static int synaptics_rmi4_rt_suspend(struct device *dev)
{
    int retval = 0;
    struct synaptics_rmi4_exp_fhandler *exp_fhandler;
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

    tp_log_warning("%s: begin \n",__func__);
    
    if (rmi4_data->staying_awake)
        return 0;
    
    if (rmi4_data->board->esd_support){
        retval = synaptics_dsx_esd_stop();
        if (retval < 0) {
            tp_log_err("%s %d:suspend esd check fail, ret = %d\n", __func__, __LINE__, retval);
        }
    }

    if (!rmi4_data->sensor_sleep) {
        rmi4_data->touch_stopped = true;
        synaptics_rmi4_irq_enable(rmi4_data, false);
        /**
         * if dtsi enable glove edge switch and glove state switch of edge is running ,
         * we need to cancel the delayed work for tp suspend
         * and set the glove state process of edge for false
         */
        if((true == rmi4_data->board->glove_edge_switch_supported)
                    &&(true == edge_glove_switch_process))
        {
            retval = cancel_delayed_work_sync(&rmi4_data->glove_delay_work);
            if(retval < 0){
                tp_log_err("%s %d: failed to cancel delayed work sync retval = %d\n",
                    __func__, __LINE__, retval);
            }
            edge_glove_switch_process = false;
        }
        synaptics_set_sensor_sleep_mode(rmi4_data);
        synaptics_rmi4_free_fingers(rmi4_data);
    }

    mutex_lock(&exp_data.mutex);
    if (!list_empty(&exp_data.list)) {
        list_for_each_entry(exp_fhandler, &exp_data.list, link)
            if (exp_fhandler->exp_fn->suspend != NULL)
                exp_fhandler->exp_fn->suspend(rmi4_data);
    }
    mutex_unlock(&exp_data.mutex);

    if (rmi4_data->regulator)
        regulator_disable(rmi4_data->regulator);

    tp_log_debug("%s: out\n",__func__);
    return 0;
}

 /**
 * synaptics_rmi4_rt_resume()
 */
static int synaptics_rmi4_rt_resume(struct device *dev)
{
    int retval = 0;
    struct synaptics_rmi4_exp_fhandler *exp_fhandler;
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
    const struct synaptics_dsx_platform_data *platform_data =
            rmi4_data->board;

    tp_log_warning("%s: begin \n",__func__);

    if (rmi4_data->regulator) {
        retval = regulator_enable(rmi4_data->regulator);
        if (retval) {
            dev_err(&rmi4_data->i2c_client->dev,
                "Regulator enable failed rc=%d\n",
                retval);
        }
        msleep(platform_data->reset_delay_ms);
        rmi4_data->current_page = MASK_8BIT;
    }

    /* Finish all the work to resume device */
    synaptics_set_sensor_wake_mode(rmi4_data);
    
    mutex_lock(&exp_data.mutex);
    if (!list_empty(&exp_data.list)) {
        list_for_each_entry(exp_fhandler, &exp_data.list, link)
            if (exp_fhandler->exp_fn->resume != NULL)
                exp_fhandler->exp_fn->resume(rmi4_data);
    }
    mutex_unlock(&exp_data.mutex);

    rmi4_data->touch_stopped = false;
    synaptics_rmi4_irq_enable(rmi4_data, true);
    tp_log_debug("%s: out\n",__func__);

    if (rmi4_data->board->esd_support){
        retval = synaptics_dsx_esd_start();
        if (retval < 0) {
            tp_log_err("%s %d:start esd check fail, ret = %d\n", __func__, __LINE__, retval);
        }
    }

    return 0;
}
#endif/*CONFIG_PM_RUNTIME*/

#ifdef CONFIG_PM_RUNTIME
static const struct dev_pm_ops synaptics_rmi4_dev_pm_rt_ops = {
    SET_RUNTIME_PM_OPS(synaptics_rmi4_rt_suspend, synaptics_rmi4_rt_resume,NULL)
};
#elif CONFIG_PM_SLEEP
static const struct dev_pm_ops synaptics_rmi4_dev_pm_ops = {
    SET_SYSTEM_SLEEP_PM_OPS(synaptics_rmi4_suspend, synaptics_rmi4_resume)
};
#endif/*CONFIG_PM_RUNTIME*/
#endif


static const struct i2c_device_id synaptics_rmi4_id_table[] = {
    {DRIVER_NAME, 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, synaptics_rmi4_id_table);

#ifdef CONFIG_OF
static struct of_device_id synaptics_match_table[] = {
    { .compatible = "synaptics,dsx",},
    { },
};
#else
#define synaptics_match_table NULL
#endif

static struct i2c_driver synaptics_rmi4_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .owner = THIS_MODULE,
        .of_match_table = synaptics_match_table,
#ifdef CONFIG_PM
#ifdef CONFIG_PM_RUNTIME
        .pm = &synaptics_rmi4_dev_pm_rt_ops,
#elif CONFIG_PM_SLEEP
        .pm = &synaptics_rmi4_dev_pm_ops,
#endif/*CONFIG_PM_RUNTIME*/
#endif
    },
    .probe = synaptics_rmi4_probe,
    .remove = synaptics_rmi4_remove,
    .id_table = synaptics_rmi4_id_table,
};

 /**
 * synaptics_rmi4_init()
 *
 * Called by the kernel during do_initcalls (if built-in)
 * or when the driver is loaded (if a module).
 *
 * This function registers the driver to the I2C subsystem.
 *
 */
static int __init synaptics_rmi4_init(void)
{
    tp_log_info("%s: begin\n",__func__);
    return i2c_add_driver(&synaptics_rmi4_driver);
}

 /**
 * synaptics_rmi4_exit()
 *
 * Called by the kernel when the driver is unloaded.
 *
 * This funtion unregisters the driver from the I2C subsystem.
 *
 */
static void __exit synaptics_rmi4_exit(void)
{
    i2c_del_driver(&synaptics_rmi4_driver);
#if    USE_IRQ_THREAD //huawei 11-25
#else        
    if (synaptics_wq)
        destroy_workqueue(synaptics_wq);
#endif /*USE_IRQ_THREAD*/

    return;
}

module_init(synaptics_rmi4_init);
module_exit(synaptics_rmi4_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics DSX I2C Touch Driver");
MODULE_LICENSE("GPL v2");
