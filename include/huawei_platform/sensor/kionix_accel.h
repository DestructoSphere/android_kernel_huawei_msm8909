/* include/linux/input/kionix_accel.h - Kionix accelerometer driver
 * Copyright (C) 2012 Kionix, Inc.
 * Written by Kuching Tan <kuchingtan@kionix.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __KIONIX_ACCEL_H__
#define __KIONIX_ACCEL_H__
#include	<linux/kernel.h>
#include	<linux/i2c.h>
#include	<linux/input.h>
#ifdef CONFIG_HUAWEI_DSM 
#include 	<linux/dsm_pub.h>
#endif
#include	<linux/sensors.h>


#define KIONIX_ACCEL_I2C_ADDR		0x0F
#define KIONIX_ACCEL_NAME			"kionix_accel"
#define KIONIX_ACCEL_INPUT_NAME		"accelerometer"
#define KIONIX_ACCEL_IRQ			"kionix-irq"

/* set print debug info interval 10s*/
#define DBG_INTERVAL_10S 			10*HZ
#define DBG_INTERVAL_1S 			1*HZ
#define COUNT_EXCEPTION_TIME_2S     2000
#define EXCEPTION_PERCENTAGE 		80
#define EXCEPTION_BASE_TIMES		24
#define	KIONIX_I2C_SCL	909
#define	KIONIX_I2C_SDA	908
 #ifdef CONFIG_HUAWEI_DSM  
/* for example: base=1g, x_y_z_max = 3g*/
#define base_to_total_max(base) 		(3*base)
/* for example: base=1g, base_to_total_min = 0.6g*/
#define base_to_total_min(base) 		(3*base/5)
/* for example: base=1g, single_axis_max = 1.8g*/
#define base_to_single_axis_max(base) 	(9*base/5)
#endif 
#define KIONIX_ERR(x,arg...) do {\
		printk("[kx023_err]"  x"\n",##arg);\
	} while (0)

#define KIONIX_INFO(x, arg...) do {\
	if (kx023_debug_mask >= 1)\
		printk("[kx023_info]" x"\n",##arg);\
	} while (0)

#define KIONIX_DBG(x,arg...) do {\
	if (kx023_debug_mask > 1)\
		printk("[kx023_debug]" x"\n",##arg);\
	} while (0)


enum {
	accel_ctrl_reg1 = 0,
	accel_data_ctrl,
	accel_int_ctrl,
	accel_regs_count,
};

struct kionix_accel_platform_data;
struct kionix_accel_driver;
#ifdef CONFIG_HUAWEI_DSM

struct kx_gs_dsm_operation{
	int 	(*dump_i2c_status)(struct kionix_accel_driver *acceld,int i2c_err);
	int 	(*check_exception)(int *xyz, struct kionix_accel_driver *acceld);
	void (*judge_same_value_excep)(s16 x,s16 y,s16 z,struct kionix_accel_driver *acceld);
};

struct gsensor_test_excep{
	int total_times;
	int exception_times;
	int i2c_scl_val;			/* when i2c transfer err, read the gpio value*/
	int i2c_sda_val;
	int vdd_mv;
	int vddio_mv;
	int i2c_err_num;			/*i2c transfer err number*/
	int excep_num;				/* the error number the dsm_client identified*/
	unsigned char reg_buf[4]; 	/* 0: ACCEL_INT_REL,		1:ACCEL_CTRL_REG1
								 2: ACCEL_DATA_CTRL ,	3:ACCEL_INT_CTRL1 */
	bool triger_timer_flag;		/*Allow to trigger the timer to judge exception or not */
	bool triger_count_flag;		/*Allow to trigger counting the exception and total times */
	int excep_base; 			/* exception base value to calculate x_y_z_max and single_axis_max */
	int x_y_z_max;				/*exception of	(x+y+z) max value*/
	int x_y_z_min;				/*exception of	(|x|+|y|+|z|) min value*/
	int single_axis_max;		/* single axis max value*/
	int cur_err_x;
	int cur_err_y;
	int cur_err_z;
	s16	pre_xyz[3];
	int same_times;
	int x_same_times;
	int y_same_times;
	int z_same_times;
	int error_times;
};
#endif
struct sensor_regulator {
	struct regulator *vreg;
	const char *name;
	u32 min_uV;
	u32 max_uV;
	/*if we has got the regulator,got_regulator_flag=1,and don't need to get again,else got_regulator_flag=0 */
	bool got_regulator_flag;
};

struct kionix_accel_driver {
	struct i2c_client *client;
	struct kionix_accel_platform_data *accel_pdata;
	struct sensors_classdev cdev;
	struct input_dev *input_dev;
	struct delayed_work accel_work;
	struct workqueue_struct *accel_workqueue;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
	struct pinctrl_state *pin_sleep;
	wait_queue_head_t wqh_suspend;
	int on_before_suspend;

	struct sensor_regulator kionix_acc_vreg[2];
	struct delayed_work debug_work;
	bool	queued_debug_work_flag;
#ifdef CONFIG_HUAWEI_DSM
	struct kx_gs_dsm_operation kx023_dsm_operation;
	struct timer_list gsensor_excep_timer;
	struct gsensor_test_excep gsensor_test_exception;
	struct work_struct	excep_dwork;
	char *dsm_buf;			/* buf to record error or exception */
	struct dsm_client *gsensor_dclient;
#endif
	
	bool device_exist;

	int accel_data[3];
	int accel_cali[3];
	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;
	bool negate_x;
	bool negate_y;
	bool negate_z;
	u8 shift;

	unsigned int poll_interval;
	unsigned int poll_delay;
	u8 *accel_registers;

	atomic_t accel_suspended;
	atomic_t accel_suspend_continue;
	atomic_t accel_enabled;
	atomic_t accel_input_event;
	atomic_t accel_enable_resume;
	struct mutex mutex_earlysuspend;
	struct mutex mutex_resume;
	struct mutex lock_i2c;
	struct mutex lock;
	rwlock_t rwlock_accel_data;

	bool accel_drdy;
	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;
	/* Function callback */
	void (*kionix_accel_report_accel_data)(struct kionix_accel_driver *acceld);
	int (*kionix_accel_update_odr)(struct kionix_accel_driver *acceld, unsigned int poll_interval);
	int (*kionix_accel_power_on_init)(struct kionix_accel_driver *acceld);
	int (*kionix_accel_operate)(struct kionix_accel_driver *acceld);
	int (*kionix_accel_standby)(struct kionix_accel_driver *acceld);
	int (*i2c_read)(struct kionix_accel_driver *acceld, u8 addr, u8 *buf,int len);
};

#ifdef CONFIG_HUAWEI_DSM
int register_kx023_dsm_operations(struct kionix_accel_driver *acceld);
void unregister_kx023_dsm_operations(struct kionix_accel_driver *acceld);
#endif


struct kionix_accel_platform_data {
	/* Although the accelerometer can perform at high ODR,
	 * there is a need to keep the maximum ODR to a lower
	 * value due to power consumption or other concern.
	 * Use this variable to set the minimum allowable
	 * interval for data to be reported from the
	 * accelerometer. Unit is measured in milli-
	 * seconds. Recommended value is 5ms. */
	unsigned int min_interval;
	/* Use this variable to set the default interval for
	 * data to be reported from the accelerometer. This
	 * value will be used during driver setup process,
	 * but can be changed by the system during runtime via
	 * sysfs control. Recommended value is 200ms.*/
	unsigned int poll_interval;
	int gpio_int1;
	int gpio_int2;
	/* This variable controls the corresponding direction
	 * of the accelerometer that is mounted on the board
	 * of the device. Refer to the porting guide for
	 * details. Valid value is 1 to 8. */
	u8 accel_direction;

	/* Use this variable to choose whether or not to use
	 * DRDY hardware interrupt mode to trigger a data
	 * report event instead of using software polling.
	 * Note that for those accelerometer model that does
	 * not support DRDY hardware interrupt, the driver
	 * will revert to software polling mode automatically.
	 * Valid value is 0 or 1.*/
	bool accel_irq_use_drdy;

	/* Use this variable to control the number of
	 * effective bits of the accelerometer output.
	 * Use the macro definition to select the desired
	 * number of effective bits. */
	#define KIONIX_ACCEL_RES_12BIT	0
	#define KIONIX_ACCEL_RES_8BIT	1
	#define KIONIX_ACCEL_RES_6BIT	2
	#define KIONIX_ACCEL_RES_16BIT	3	//KX023
	u8 accel_res;

	/* Use this variable to control the G range of
	 * the accelerometer output. Use the macro definition
	 * to select the desired G range.*/
	#define KIONIX_ACCEL_G_2G		0
	#define KIONIX_ACCEL_G_4G		1
	#define KIONIX_ACCEL_G_6G		2
	#define KIONIX_ACCEL_G_8G		3
	u8 accel_g_range;

	/* Optional callback functions that can be implemented
	 * on per product basis. If these callbacks are defined,
	 * they will be called by the driver. */
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

	/* i2c gpio */
	unsigned int i2c_scl_gpio;
	unsigned int i2c_sda_gpio;
};
/******************************************************************************
 * Accelerometer WHO_AM_I return value
 *****************************************************************************/
#define KIONIX_ACCEL_WHO_AM_I_KX023		0x15

/******************************************************************************
 * Accelerometer Grouping
 *****************************************************************************/

#define KIONIX_ACCEL	7	/* KX023 */

/******************************************************************************
 * Registers for All Accelerometer Group
 *****************************************************************************/
#define ACCEL_WHO_AM_I		0x0F

/*****************************************************************************/
/* Registers for Accelerometer Group 7 */
/*****************************************************************************/
/* Output Registers */
#define ACCEL_XOUT_L		0x06
/* Control Registers */
#define ACCEL_INT_REL		0x17
#define ACCEL_CTRL_REG1		0x18
#define ACCEL_INT_CTRL1		0x1C
#define ACCEL_DATA_CTRL		0x1B	/* ODCNTL */
/* CTRL_REG1 */
#define ACCEL_PC1_OFF		0x7F
#define ACCEL_PC1_ON		(1 << 7)
#define ACCEL_DRDYE			(1 << 5)
#define ACCEL_G_8G			(2 << 3)
#define ACCEL_G_4G			(1 << 3)
#define ACCEL_G_2G			(0 << 3)
#define ACCEL_G_MASK		(3 << 3)
#define ACCEL_RES_8BIT		(0 << 6)
#define ACCEL_RES_16BIT		(1 << 6)
#define ACCEL_RES_MASK		(1 << 6)
/* INT_CTRL1 */
#define ACCEL_IEA			(1 << 4)
#define ACCEL_IEN			(1 << 5)
/* DATA_CTRL_REG */
#define ACCEL_ODR0_781		0x08
#define ACCEL_ODR1_563		0x09
#define ACCEL_ODR3_125		0x0A
#define ACCEL_ODR6_25		0x0B
#define ACCEL_ODR12_5		0x00
#define ACCEL_ODR25			0x01
#define ACCEL_ODR50			0x02
#define ACCEL_ODR100		0x03
#define ACCEL_ODR200		0x04
#define ACCEL_ODR400		0x05
#define ACCEL_ODR800		0x06
#define ACCEL_ODR1600		0x07
/*****************************************************************************/

/* Input Event Constants */
#define ACCEL_G_MAX		8096
#define ACCEL_FUZZ		0
#define ACCEL_FLAT		0
/* I2C Retry Constants */
#define KIONIX_I2C_RETRY_COUNT		3 	/* Number of times to retry i2c */
#define KIONIX_I2C_RETRY_TIMEOUT	1	/* Timeout between retry (miliseconds) */

/* Earlysuspend Contants */
#define KIONIX_ACCEL_EARLYSUSPEND_TIMEOUT	5000	/* Timeout (miliseconds) */

#define GS_MAP_DIRECTION_BOTTOM 1
#define GS_MAP_DIRECTION_TOP		0
#define GS_MAP_DIRECTION_BOTTOM_LEFT_TOP 2

#define DEFAULT_INT1_GPIO 			1017
#define DEFAULT_DIRECT			GS_MAP_DIRECTION_TOP
#define DEFAULT_G_RANGE			0
#define DEFAULT_POLL_INTERVAL	200
#define DEFAULT_MIN_INTERVAL		5
#define DEFAULT_IRQ_USE_DRDY		0
#define DEFAULT_ACCEL_RES			0
#endif /* __KIONIX_ACCEL_H__ */
