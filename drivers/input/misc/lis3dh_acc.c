/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
 *
 * File Name          : lis3dh_acc.c
 * Authors            : MSH - Motion Mems BU - Application Team
 *		      : Matteo Dameno (matteo.dameno@st.com)
 *		      : Carmine Iascone (carmine.iascone@st.com)
 *                    : Samuel Huo (samuel.huo@st.com)
 * Version            : V.1.1.0
 * Date               : 07/10/2012
 * Description        : LIS3DH accelerometer sensor driver
 *
 *******************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
 *
 ******************************************************************************
 Revision 1.0.0 05/11/09
 First Release;
 Revision 1.0.3 22/01/2010
  Linux K&R Compliant Release;
 Revision 1.0.5 16/08/2010
  modified _get_acceleration_data function;
  modified _update_odr function;
  manages 2 interrupts;
 Revision 1.0.6 15/11/2010
  supports sysfs;
  no more support for ioctl;
 Revision 1.0.7 26/11/2010
  checks for availability of interrupts pins
  correction on FUZZ and FLAT values;
 Revision 1.0.8 2010/Apr/01
  corrects a bug in interrupt pin management in 1.0.7
 Revision 1.0.9 07/25/2011
  Romove several unused functions,add 5ms delay in init,change sysfs attributes.
 Revision 1.1.0 07/10/2012
  To replace some deprecated functions for 3.4 kernel; to pass the checkpatch's formatting requirement;
  To add regulator request;

 ******************************************************************************/

#include	<linux/err.h>
#include	<linux/errno.h>
#include	<linux/delay.h>
#include	<linux/fs.h>
#include	<linux/i2c.h>
#include	<linux/input.h>
#include	<linux/uaccess.h>
#include	<linux/workqueue.h>
#include	<linux/irq.h>
#include	<linux/gpio.h>
#include	<linux/interrupt.h>
#include	<linux/slab.h>
#include	<linux/pm.h>
#include	<linux/input/lis3dh.h>
#include	<linux/module.h>
#include	<linux/regulator/consumer.h>
#include	<linux/of_gpio.h>
#include	<linux/sensors.h>
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include	<linux/hw_dev_dec.h>
#endif

#include <misc/app_info.h>


#ifdef CONFIG_HUAWEI_KERNEL
#define DEFAULT_MIN_INTERVAL 5
#define DEFAULT_POLL_INTERVAL 200
#define DEFAULT_G_RANGE		2
/* gsensor map direction : 1--bottom*/
#define GS_MAP_DIRECTION_BOTTOM 1
#define GS_MAP_DIRECTION_TOP 0
#define DEFAULT_GS_MAP_DIRECTION  GS_MAP_DIRECTION_TOP
#define GS_MAP_DIRECTION_BOTTOM_LEFT_TOP 2
#define DBG_INTERVAL_1S 			1000

#endif

#define MAX_ACCEL_VAL 1054   /*9.8065/1024*MAX_ACCEL_VAL=10.1*/
#define MIN_ACCEL_VAL -1054   /*-9.8065/1024*MAX_ACCEL_VAL=-10.1*/

#define	DEBUG	1

/*Calibration*/
#define LIS3DH_CAL_SKIP_COUNT 5
#define LIS3DH_CAL_MAX		(10 + LIS3DH_CAL_SKIP_COUNT)
#define LIS3DH_CAL_NUM		99
#define RAW_TO_1G		16384
#define FACTORY_CALIBRATION_DELAY 100	/* ms */

#define	G_MAX		17000
#define LIS3DH_FIFO_SIZE	32
#define LIS3DH_TIME_MS_TO_NS	1000000L


#define SENSITIVITY_2G		1	/**	mg/LSB	*/
#define SENSITIVITY_4G		2	/**	mg/LSB	*/
#define SENSITIVITY_8G		4	/**	mg/LSB	*/
#define SENSITIVITY_16G		12	/**	mg/LSB	*/

#ifdef CONFIG_HUAWEI_KERNEL
static int lis3dh_debug_mask = 1;
module_param_named(lis3dh_debug, lis3dh_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

#define LIS3DH_ERR(x...) do {\
    if (lis3dh_debug_mask >=0) \
        printk(KERN_ERR x);\
    } while (0)
#define LIS3DH_WARN(x...) do {\
    if (lis3dh_debug_mask >=0) \
        printk(KERN_ERR x);\
    } while (0)
#define LIS3DH_INFO(x...) do {\
    if ((KERNEL_HWFLOW)  && (lis3dh_debug_mask >=1)) \
        printk(KERN_ERR x);\
    } while (0)
#define LIS3DH_DEBUG(x...) do {\
    if ((KERNEL_HWFLOW)  && (lis3dh_debug_mask >=2)) \
        printk(KERN_ERR x);\
    } while (0)
#endif

#ifndef CONFIG_HUAWEI_KERNEL
/* Accelerometer Sensor Operating Mode */
#define LIS3DH_ACC_ENABLE	0x01
#define LIS3DH_ACC_DISABLE	0x00

#define	HIGH_RESOLUTION		0x08

#define	AXISDATA_REG		0x28
#define WHOAMI_LIS3DH_ACC	0x33	/*	Expected content for WAI */

/*	CONTROL REGISTERS	*/
#define WHO_AM_I		0x0F	/*	WhoAmI register		*/
#define	TEMP_CFG_REG		0x1F	/*	temper sens control reg	*/
/* ctrl 1: ODR3 ODR2 ODR ODR0 LPen Zenable Yenable Zenable */
#define	CTRL_REG1		0x20	/*	control reg 1		*/
#define	CTRL_REG2		0x21	/*	control reg 2		*/
#define	CTRL_REG3		0x22	/*	control reg 3		*/
#define	CTRL_REG4		0x23	/*	control reg 4		*/
#define	CTRL_REG5		0x24	/*	control reg 5		*/
#define	CTRL_REG6		0x25	/*	control reg 6		*/

#define	FIFO_CTRL_REG		0x2E	/*	FiFo control reg	*/

#define	INT_CFG1		0x30	/*	interrupt 1 config	*/
#define	INT_SRC1		0x31	/*	interrupt 1 source	*/
#define	INT_THS1		0x32	/*	interrupt 1 threshold	*/
#define	INT_DUR1		0x33	/*	interrupt 1 duration	*/


#define	TT_CFG			0x38	/*	tap config		*/
#define	TT_SRC			0x39	/*	tap source		*/
#define	TT_THS			0x3A	/*	tap threshold		*/
#define	TT_LIM			0x3B	/*	tap time limit		*/
#define	TT_TLAT			0x3C	/*	tap time latency	*/
#define	TT_TW			0x3D	/*	tap time window		*/
/*	end CONTROL REGISTRES	*/


#define ENABLE_HIGH_RESOLUTION	1

#define LIS3DH_ACC_PM_OFF		0x00
#define LIS3DH_ACC_ENABLE_ALL_AXES	0x07


#define PMODE_MASK			0x08
#define ODR_MASK			0XF0

#define ODR1		0x10  /* 1Hz output data rate */
#define ODR10		0x20  /* 10Hz output data rate */
#define ODR25		0x30  /* 25Hz output data rate */
#define ODR50		0x40  /* 50Hz output data rate */
#define ODR100		0x50  /* 100Hz output data rate */
#define ODR200		0x60  /* 200Hz output data rate */
#define ODR400		0x70  /* 400Hz output data rate */
#define ODR1250		0x90  /* 1250Hz output data rate */



#define	IA			0x40
#define	ZH			0x20
#define	ZL			0x10
#define	YH			0x08
#define	YL			0x04
#define	XH			0x02
#define	XL			0x01
/* */
/* CTRL REG BITS*/
#define	CTRL_REG3_I1_AOI1	0x40
#define	CTRL_REG6_I2_TAPEN	0x80
#define	CTRL_REG6_HLACTIVE	0x02
/* */
#define NO_MASK			0xFF
#define INT1_DURATION_MASK	0x7F
#define	INT1_THRESHOLD_MASK	0x7F
#define TAP_CFG_MASK		0x3F
#define	TAP_THS_MASK		0x7F
#define	TAP_TLIM_MASK		0x7F
#define	TAP_TLAT_MASK		NO_MASK
#define	TAP_TW_MASK		NO_MASK


/* TAP_SOURCE_REG BIT */
#define	DTAP			0x20
#define	STAP			0x10
#define	SIGNTAP			0x08
#define	ZTAP			0x04
#define	YTAP			0x02
#define	XTAZ			0x01


#define	FUZZ			0
#define	FLAT			0
#define	I2C_RETRY_DELAY		5
#ifdef CONFIG_HUAWEI_KERNEL
#define	I2C_RETRIES		3
#else
#define	I2C_RETRIES		5
#endif
#define	I2C_AUTO_INCREMENT	0x80

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1		0
#define	RES_CTRL_REG2		1
#define	RES_CTRL_REG3		2
#define	RES_CTRL_REG4		3
#define	RES_CTRL_REG5		4
#define	RES_CTRL_REG6		5

#define	RES_INT_CFG1		6
#define	RES_INT_THS1		7
#define	RES_INT_DUR1		8

#define	RES_TT_CFG		9
#define	RES_TT_THS		10
#define	RES_TT_LIM		11
#define	RES_TT_TLAT		12
#define	RES_TT_TW		13

#define	RES_TEMP_CFG_REG	14
#define	RES_REFERENCE_REG	15
#define	RES_FIFO_CTRL_REG	16

#define	RESUME_ENTRIES		17
/* end RESUME STATE INDICES */
#endif

#define BATCH_MODE_NORMAL		0
#define BATCH_MODE_WAKE_UPON_FIFO_FULL	2

/* interrput mode for sensor max delay ms */
#define LIS_INT_MAX_DELAY	1000

enum {
	LIS3DH_BYPASS_MODE = 0,
	LIS3DH_FIFO_MODE,
	LIS3DH_STREAM_MODE,
	LIS3DH_STREAM2FIFO_MODE,
	LIS3DH_FIFO_MODE_NUM
};

struct odr_config_table {
	unsigned int cutoff_ms;
	unsigned int mask;
} lis3dh_acc_odr_table[] = {
		{    1, ODR1250 },
		{    3, ODR400  },
		{    5, ODR200  },
		{   10, ODR100  },
		{   20, ODR50   },
		{   40, ODR25   },
		{  100, ODR10   },
		{ 1000, ODR1    },
};
#ifndef CONFIG_HUAWEI_KERNEL
struct lis3dh_acc_data {
	struct i2c_client *client;
	struct lis3dh_acc_platform_data *pdata;
	struct sensors_classdev cdev;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
	struct pinctrl_state *pin_sleep;

	struct mutex lock;
	struct delayed_work input_work;

	struct input_dev *input_dev;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;
	int on_before_suspend;
	int pre_enable;

	char calibrate_buf[LIS3DH_CAL_NUM];
	int cal_params[3];
	bool use_cal;
	atomic_t cal_status;

	u8 sensitivity;

	u8 resume_state[RESUME_ENTRIES];

	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;

#ifdef DEBUG
	u8 reg_addr;
#endif
};
#endif

static struct sensors_classdev lis3dh_acc_cdev = {
	.name = "lis3dh-accel",
	.vendor = "STMicroelectronics",
	.version = 1,
	.handle = SENSORS_ACCELERATION_HANDLE,
	.type = SENSOR_TYPE_ACCELEROMETER,
	.max_range = "156.8",
	.resolution = "0.01",
	.sensor_power = "0.01",
	.min_delay = 5000,
	.delay_msec = 200,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.sensors_set_latency = NULL,
	.sensors_flush = NULL,
	.sensors_calibrate = NULL,
	.sensors_write_cal_params = NULL,
	.params = NULL,
};
#ifndef CONFIG_HUAWEI_KERNEL
struct sensor_regulator {
	struct regulator *vreg;
	const char *name;
	u32	min_uV;
	u32	max_uV;
};
#endif

struct sensor_regulator lis3dh_acc_vreg[] = {
	{NULL, "vdd", 1700000, 3600000},
	{NULL, "vddio", 1700000, 3600000},
};

static int lis3dh_acc_get_calibrate(struct lis3dh_acc_data *acc,
					int *xyz);

static inline s64 lis3dh_acc_get_time_ns(void)
{
	struct timespec ts;

	get_monotonic_boottime(&ts);
	return timespec_to_ns(&ts);
}

static unsigned int lis3dh_acc_odr_to_interval(struct lis3dh_acc_data *acc,
				unsigned int odr)
{
	unsigned int odr_mask;
	unsigned int i;

	odr_mask = (odr << REG1_ODR_SHIFT) & REG1_ODR_MASK;

	for (i = ARRAY_SIZE(lis3dh_acc_odr_table) - 1; i > 0; i--) {
		if (lis3dh_acc_odr_table[i].mask == odr_mask)
			break;
	}

	return lis3dh_acc_odr_table[i].cutoff_ms;
}

static int lis3dh_acc_config_regulator(struct lis3dh_acc_data *acc, bool on)
{
	int rc = 0, i;
	int num_reg = sizeof(lis3dh_acc_vreg) / sizeof(struct sensor_regulator);
#ifdef CONFIG_HUAWEI_DSM
	struct sensor_regulator *lis3dh_acc_vreg = acc->lis3dh_acc_vreg;
#endif

	if (on) {
		for (i = 0; i < num_reg; i++) {
			lis3dh_acc_vreg[i].vreg =
				regulator_get(&acc->client->dev,
				lis3dh_acc_vreg[i].name);
			if (IS_ERR(lis3dh_acc_vreg[i].vreg)) {
				rc = PTR_ERR(lis3dh_acc_vreg[i].vreg);
				pr_err("%s:regulator get failed rc=%d\n",
								__func__, rc);
				lis3dh_acc_vreg[i].vreg = NULL;
				goto error_vdd;
			}

			if (regulator_count_voltages(
				lis3dh_acc_vreg[i].vreg) > 0) {
				rc = regulator_set_voltage(
					lis3dh_acc_vreg[i].vreg,
					lis3dh_acc_vreg[i].min_uV,
					lis3dh_acc_vreg[i].max_uV);
				if (rc) {
					pr_err("%s: set voltage failed rc=%d\n",
					__func__, rc);
					regulator_put(lis3dh_acc_vreg[i].vreg);
					lis3dh_acc_vreg[i].vreg = NULL;
					goto error_vdd;
				}
			}

			rc = regulator_enable(lis3dh_acc_vreg[i].vreg);
			if (rc) {
				pr_err("%s: regulator_enable failed rc =%d\n",
					__func__, rc);
				if (regulator_count_voltages(
					lis3dh_acc_vreg[i].vreg) > 0) {
					regulator_set_voltage(
						lis3dh_acc_vreg[i].vreg, 0,
						lis3dh_acc_vreg[i].max_uV);
				}
				regulator_put(lis3dh_acc_vreg[i].vreg);
				lis3dh_acc_vreg[i].vreg = NULL;
				goto error_vdd;
			}
		}
		return rc;
	} else {
		i = num_reg;
	}

error_vdd:
	while (--i >= 0) {
		if (!IS_ERR_OR_NULL(lis3dh_acc_vreg[i].vreg)) {
			if (regulator_count_voltages(
			lis3dh_acc_vreg[i].vreg) > 0) {
				regulator_set_voltage(lis3dh_acc_vreg[i].vreg,
						0, lis3dh_acc_vreg[i].max_uV);
			}
			regulator_disable(lis3dh_acc_vreg[i].vreg);
			regulator_put(lis3dh_acc_vreg[i].vreg);
			lis3dh_acc_vreg[i].vreg = NULL;
		}
	}
	return rc;
}
#ifdef CONFIG_HUAWEI_KERNEL
static int lis3dh_acc_i2c_read(struct lis3dh_acc_data *acc,
				u8 *buf, int len)
{
	int err;
	int tries = 0;
		/*delete it , use short code segment */
	struct i2c_msg	msgs[] = {
		{
			.addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = buf,
		},
		{
			.addr = acc->client->addr,
			.flags = (acc->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	do {
		mutex_lock(&acc->lock_i2c);
		err = i2c_transfer(acc->client->adapter, msgs, 2);
		mutex_unlock(&acc->lock_i2c);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		err = -EIO;
		dev_err(&acc->client->dev,"---------cut here-------\n"
			"WARNING: at %s %d read transfer %s() err=%d\n",__FILE__,__LINE__,__func__,err);
#ifdef CONFIG_HUAWEI_DSM
		acc->lis3dh_dsm_operation.dump_i2c_status(acc, err);
#endif
	} else {
		err = 0;
	}

	return err;
}

static int lis3dh_acc_i2c_write(struct lis3dh_acc_data *acc, u8 *buf, int len)
{
	int err;
	int tries = 0;
		/*delete it , use short code segment */

	struct i2c_msg msgs[] = {
		{
			.addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = len + 1,
			.buf = buf,
		 },
	};

	do {
		mutex_lock(&acc->lock_i2c);
		err = i2c_transfer(acc->client->adapter, msgs, 1);
		mutex_unlock(&acc->lock_i2c);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		err = -EIO;
		dev_err(&acc->client->dev,"---------cut here-------\n"
			"WARNING: at %s %d write transfer %s() err=%d\n",__FILE__,__LINE__,__func__,err);
#ifdef CONFIG_HUAWEI_DSM
		acc->lis3dh_dsm_operation.dump_i2c_status(acc, err);
#endif
	} else {
		err = 0;
	}

	return err;
}
#else

static int lis3dh_acc_i2c_read(struct lis3dh_acc_data *acc,
				u8 *buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg	msgs[] = {
		{
			.addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = buf,
		},
		{
			.addr = acc->client->addr,
			.flags = (acc->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	do {
		err = i2c_transfer(acc->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		dev_err(&acc->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int lis3dh_acc_i2c_write(struct lis3dh_acc_data *acc, u8 *buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
			.addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = len + 1,
			.buf = buf,
		 },
	};

	do {
		err = i2c_transfer(acc->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&acc->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}
#endif

static int lis3dh_acc_hw_init(struct lis3dh_acc_data *acc)
{
	int err = -1;
	u8 buf[7];

	buf[0] = WHO_AM_I;
	err = lis3dh_acc_i2c_read(acc, buf, 1);
	if (err < 0) {
		dev_warn(&acc->client->dev,
		"Error reading WHO_AM_I: is device available/working?\n");
		goto err_firstread;
	} else
		acc->hw_working = 1;
	if (buf[0] != WHOAMI_LIS3DH_ACC) {
		dev_err(&acc->client->dev,
		"device unknown. Expected: 0x%x, Replies: 0x%x\n",
		WHOAMI_LIS3DH_ACC, buf[0]);
		err = -1; /* choose the right coded error */
		goto err_unknown_device;
	}

	buf[0] = CTRL_REG1;
	buf[1] = acc->resume_state[RES_CTRL_REG1];
	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = TEMP_CFG_REG;
	buf[1] = acc->resume_state[RES_TEMP_CFG_REG];
	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = FIFO_CTRL_REG;
	buf[1] = acc->resume_state[RES_FIFO_CTRL_REG];
	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = (I2C_AUTO_INCREMENT | TT_THS);
	buf[1] = acc->resume_state[RES_TT_THS];
	buf[2] = acc->resume_state[RES_TT_LIM];
	buf[3] = acc->resume_state[RES_TT_TLAT];
	buf[4] = acc->resume_state[RES_TT_TW];
	err = lis3dh_acc_i2c_write(acc, buf, 4);
	if (err < 0)
		goto err_resume_state;
	buf[0] = TT_CFG;
	buf[1] = acc->resume_state[RES_TT_CFG];
	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = (I2C_AUTO_INCREMENT | INT_THS1);
	buf[1] = acc->resume_state[RES_INT_THS1];
	buf[2] = acc->resume_state[RES_INT_DUR1];
	err = lis3dh_acc_i2c_write(acc, buf, 2);
	if (err < 0)
		goto err_resume_state;
	buf[0] = INT_CFG1;
	buf[1] = acc->resume_state[RES_INT_CFG1];
	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;


	buf[0] = (I2C_AUTO_INCREMENT | CTRL_REG2);
	buf[1] = acc->resume_state[RES_CTRL_REG2];
	buf[2] = acc->resume_state[RES_CTRL_REG3];
	buf[3] = acc->resume_state[RES_CTRL_REG4];
	buf[4] = acc->resume_state[RES_CTRL_REG5];
	buf[5] = acc->resume_state[RES_CTRL_REG6];
	err = lis3dh_acc_i2c_write(acc, buf, 5);
	if (err < 0)
		goto err_resume_state;

	acc->hw_initialized = 1;
	return 0;

err_firstread:
	acc->hw_working = 0;
err_unknown_device:
err_resume_state:
	acc->hw_initialized = 0;
	dev_err(&acc->client->dev, "hw init error 0x%x,0x%x: %d\n", buf[0],
			buf[1], err);
	return err;
}

static void lis3dh_acc_device_power_off(struct lis3dh_acc_data *acc)
{
	int err;
	u8 buf[2] = { CTRL_REG1, LIS3DH_ACC_PM_OFF };

	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		dev_err(&acc->client->dev, "soft power off failed: %d\n", err);

	lis3dh_acc_config_regulator(acc, false);

	if (acc->hw_initialized) {
		if (gpio_is_valid(acc->pdata->gpio_int1))
			disable_irq_nosync(acc->irq1);
		if (gpio_is_valid(acc->pdata->gpio_int2))
			disable_irq_nosync(acc->irq2);
		acc->hw_initialized = 0;
	}
}

static int lis3dh_acc_device_power_on(struct lis3dh_acc_data *acc)
{
	int err = -1;

	err = lis3dh_acc_config_regulator(acc, true);
	if (err < 0) {
		dev_err(&acc->client->dev,
				"power_on failed: %d\n", err);
		return err;
	}


	msleep(20);

	if (!acc->hw_initialized) {
		err = lis3dh_acc_hw_init(acc);
		if (acc->hw_working == 1 && err < 0) {
			dev_err(&acc->client->dev,"%s:%d lis3dh acc hw init failed: %d\n", __FUNCTION__, __LINE__, err);
			lis3dh_acc_device_power_off(acc);
			return err;
		}
#ifdef CONFIG_HUAWEI_KERNEL
		/*if probe first call this func and can't communicate with this slave address,acc->hw_working == 0 */
		/*we should return error code, and go out of probe function as soon as possible */
		else if(err == -EIO){
			lis3dh_acc_config_regulator(acc, false);
			dev_err(&acc->client->dev,"%s:%d lis3dh acc hw init failed: %d\n", __FUNCTION__, __LINE__, err);
			return err;
		}
#endif
	}

	if (acc->hw_initialized) {
		if (gpio_is_valid(acc->pdata->gpio_int1))
			enable_irq(acc->irq1);
		if (gpio_is_valid(acc->pdata->gpio_int2))
			enable_irq(acc->irq2);
	}
	return 0;
}

static irqreturn_t lis3dh_acc_isr1(int irq, void *dev)
{
	struct lis3dh_acc_data *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq1_work_queue, &acc->irq1_work);

	return IRQ_HANDLED;
}

static irqreturn_t lis3dh_acc_isr2(int irq, void *dev)
{
	struct lis3dh_acc_data *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq2_work_queue, &acc->irq2_work);

	return IRQ_HANDLED;
}

static void lis3dh_acc_irq1_work_func(struct work_struct *work)
{

	struct lis3dh_acc_data *acc =
	container_of(work, struct lis3dh_acc_data, irq1_work);

	goto exit;
exit:
	enable_irq(acc->irq1);
}

static void lis3dh_acc_irq2_work_func(struct work_struct *work)
{

	struct lis3dh_acc_data *acc =
	container_of(work, struct lis3dh_acc_data, irq2_work);

	goto exit;
exit:
	enable_irq(acc->irq2);
}

int lis3dh_acc_update_g_range(struct lis3dh_acc_data *acc, u8 new_g_range)
{
	int err = -1;

	u8 sensitivity;
	u8 buf[2];
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = LIS3DH_ACC_FS_MASK | HIGH_RESOLUTION;

	switch (new_g_range) {
	case LIS3DH_ACC_G_2G:

		sensitivity = SENSITIVITY_2G;
		break;
	case LIS3DH_ACC_G_4G:

		sensitivity = SENSITIVITY_4G;
		break;
	case LIS3DH_ACC_G_8G:

		sensitivity = SENSITIVITY_8G;
		break;
	case LIS3DH_ACC_G_16G:

		sensitivity = SENSITIVITY_16G;
		break;
	default:
		dev_err(&acc->client->dev, "invalid g range requested: %u\n",
				new_g_range);
		return -EINVAL;
	}

	if (atomic_read(&acc->enabled)) {
		/* Updates configuration register 4,
		* which contains g range setting */
		buf[0] = CTRL_REG4;
		err = lis3dh_acc_i2c_read(acc, buf, 1);
		if (err < 0)
			goto error;
		init_val = buf[0];
		acc->resume_state[RES_CTRL_REG4] = init_val;
		new_val = new_g_range | HIGH_RESOLUTION;
		updated_val = ((mask & new_val) | ((~mask) & init_val));
		buf[1] = updated_val;
		buf[0] = CTRL_REG4;
		err = lis3dh_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			goto error;
		acc->resume_state[RES_CTRL_REG4] = updated_val;
		acc->sensitivity = sensitivity;
	}


	return err;
error:
	dev_err(&acc->client->dev, "update g range failed 0x%x,0x%x: %d\n",
			buf[0], buf[1], err);

	return err;
}

int lis3dh_acc_update_odr(struct lis3dh_acc_data *acc, int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config[2];

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(lis3dh_acc_odr_table) - 1; i >= 0; i--) {
		if (lis3dh_acc_odr_table[i].cutoff_ms <= poll_interval_ms)
			break;
	}
	config[1] = lis3dh_acc_odr_table[i].mask;

	config[1] |= LIS3DH_ACC_ENABLE_ALL_AXES;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&acc->enabled)) {
		config[0] = CTRL_REG1;
		err = lis3dh_acc_i2c_write(acc, config, 1);
		if (err < 0)
			goto error;
		acc->resume_state[RES_CTRL_REG1] = config[1];
	}

	return err;

error:
	dev_err(&acc->client->dev, "update odr failed 0x%x,0x%x: %d\n",
			config[0], config[1], err);

	return err;
}



static int lis3dh_acc_register_write(struct lis3dh_acc_data *acc, u8 *buf,
		u8 reg_address, u8 new_value)
{
	int err = -1;

	/* Sets configuration register at reg_address
	 *  NOTE: this is a straight overwrite  */
		buf[0] = reg_address;
		buf[1] = new_value;
		err = lis3dh_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			return err;
	return err;
}

static int lis3dh_acc_get_acceleration_data(struct lis3dh_acc_data *acc,
		int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s16 hw_d[3] = { 0 };

	acc_data[0] = (I2C_AUTO_INCREMENT | AXISDATA_REG);
	err = lis3dh_acc_i2c_read(acc, acc_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = (((s16) ((acc_data[1] << 8) | acc_data[0])) >> 4);
	hw_d[1] = (((s16) ((acc_data[3] << 8) | acc_data[2])) >> 4);
	hw_d[2] = (((s16) ((acc_data[5] << 8) | acc_data[4])) >> 4);

	hw_d[0] = hw_d[0] * acc->sensitivity;
	hw_d[1] = hw_d[1] * acc->sensitivity;
	hw_d[2] = hw_d[2] * acc->sensitivity;


	xyz[0] = ((acc->pdata->negate_x) ? (-hw_d[acc->pdata->axis_map_x])
		   : (hw_d[acc->pdata->axis_map_x]));
	xyz[1] = ((acc->pdata->negate_y) ? (-hw_d[acc->pdata->axis_map_y])
		   : (hw_d[acc->pdata->axis_map_y]));
	xyz[2] = ((acc->pdata->negate_z) ? (-hw_d[acc->pdata->axis_map_z])
		   : (hw_d[acc->pdata->axis_map_z]));

	#ifdef CONFIG_HUAWEI_KERNEL
	if ((xyz[0] == 0) && (xyz[1] == 0) && (xyz[2] == 0)) {
		LIS3DH_WARN("%s:%d accelerate axis x=0,y=0,z=0\n", __FUNCTION__, __LINE__);
	}
	if ((xyz[0] > MAX_ACCEL_VAL) | (xyz[1] > MAX_ACCEL_VAL) | (xyz[2] > MAX_ACCEL_VAL)) {
		LIS3DH_DEBUG("%s:%d read x=%d, y=%d, z=%d\n", __FUNCTION__, __LINE__, xyz[0], xyz[1], xyz[2]);
	}
	if ((xyz[0] < MIN_ACCEL_VAL) | (xyz[1] < MIN_ACCEL_VAL) | (xyz[2] < MIN_ACCEL_VAL)) {
		LIS3DH_DEBUG("%s:%d read x=%d, y=%d, z=%d\n", __FUNCTION__, __LINE__, xyz[0], xyz[1], xyz[2]);
	}
	#endif
	#ifdef DEBUG
	dev_dbg(&acc->client->dev, "%s read x=%d, y=%d, z=%d\n",
				LIS3DH_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
	if (acc->use_cal) {
		err = lis3dh_acc_get_calibrate(acc, xyz);
		if (err < 0) {
			dev_err(&acc->client->dev,
					"get calibrate data falied\n");
			return err;
		}
	}

	return err;
}

static void lis3dh_acc_report_values(struct lis3dh_acc_data *acc,
					int *xyz)
{
#ifdef CONFIG_HUAWEI_DSM
	acc->lis3dh_dsm_operation.judge_check_excep(xyz,acc);
#endif

	ktime_t timestamp;

	timestamp = ktime_get_boottime();
	input_report_abs(acc->input_dev, ABS_X, xyz[0]);
	input_report_abs(acc->input_dev, ABS_Y, xyz[1]);
	input_report_abs(acc->input_dev, ABS_Z, xyz[2]);
	input_event(acc->input_dev,
		EV_SYN, SYN_TIME_SEC,
		ktime_to_timespec(timestamp).tv_sec);
	input_event(acc->input_dev,
		EV_SYN, SYN_TIME_NSEC,
		ktime_to_timespec(timestamp).tv_nsec);
	input_sync(acc->input_dev);

	#ifdef CONFIG_HUAWEI_KERNEL
	//print xyz value one time only when enable
	if(acc->print_xyz_flag)
	{
		acc->print_xyz_flag = false;
		LIS3DH_INFO("%s,xyz[0]=%d, xyz[1]=%d, xyz[2]=%d\n", __func__, xyz[0],  xyz[1],xyz[2]);
	}

	if( (lis3dh_debug_mask > 1) && false == acc->queued_debug_work_flag)
	{
		schedule_delayed_work(&acc->debug_work, msecs_to_jiffies(DBG_INTERVAL_1S));
		acc->queued_debug_work_flag = true;
	}
	#endif
}

static int lis3dh_acc_enable(struct lis3dh_acc_data *acc)
{
	int err;
	#ifdef CONFIG_HUAWEI_KERNEL
	LIS3DH_INFO("%s: acc->enabled=%d",__func__, atomic_read(&acc->enabled));
	acc->print_xyz_flag = true;
	acc->queued_debug_work_flag = false;
	#endif

	if (atomic_read(&acc->cal_status)) {
		dev_err(&acc->client->dev,
			"can not enable when sensor do calibration\n");
		return -EBUSY;
	}
	dev_dbg(&acc->client->dev, "enable acc: state =%d\n",
			atomic_read(&acc->enabled));
	if (!atomic_cmpxchg(&acc->enabled, 0, 1)) {
		if (pinctrl_select_state(acc->pinctrl, acc->pin_default))
			dev_err(&acc->client->dev,
				"Can't select pinctrl default state\n");

		err = lis3dh_acc_device_power_on(acc);
		if (err < 0) {
			dev_err(&acc->client->dev, "%s:%d lis3dh acc device power on failed, err:%d!\n", __FUNCTION__, __LINE__, err);
			atomic_set(&acc->enabled, 0);
			return err;
		}
		schedule_delayed_work(&acc->input_work,
			msecs_to_jiffies(acc->pdata->poll_interval));
	}

	return 0;
}

static int lis3dh_acc_disable(struct lis3dh_acc_data *acc)
{
	int err = 0;

	if (atomic_read(&acc->cal_status)) {
		dev_err(&acc->client->dev,
			"can not disable when sensor do calibration now\n");
		return -EBUSY;
	}
	dev_dbg(&acc->client->dev, "disable state=%d\n",
		atomic_read(&acc->enabled));
	#ifdef CONFIG_HUAWEI_KERNEL
	LIS3DH_INFO("%s: acc->enabled=%d",__func__, atomic_read(&acc->enabled));
	#endif
	if (atomic_cmpxchg(&acc->enabled, 1, 0)) {
		cancel_delayed_work_sync(&acc->input_work);
		#ifdef CONFIG_HUAWEI_KERNEL
		cancel_delayed_work_sync(&acc->debug_work);
		acc->print_xyz_flag = false;
		#endif
#ifdef CONFIG_HUAWEI_DSM
		cancel_work_sync(&acc->excep_work);
#endif

		lis3dh_acc_device_power_off(acc);
		if (pinctrl_select_state(acc->pinctrl, acc->pin_sleep))
			dev_err(&acc->client->dev,
				"Can't select pinctrl sleep state\n");
	}

	return 0;
}


static ssize_t read_single_reg(struct device *dev, char *buf, u8 reg)
{
	ssize_t ret;
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	int err;

	u8 data = reg;
	err = lis3dh_acc_i2c_read(acc, &data, 1);
	if (err < 0)
		return err;
	ret = snprintf(buf, 4, "0x%02x\n", data);
	return ret;

}

static int write_reg(struct device *dev, const char *buf, u8 reg,
		u8 mask, int resumeIndex)
{
	int err = -1;
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	u8 x[2];
	u8 new_val;
	unsigned long val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	new_val = ((u8) val & mask);
	x[0] = reg;
	x[1] = new_val;
	err = lis3dh_acc_register_write(acc, x, reg, new_val);
	if (err < 0)
		return err;
	acc->resume_state[resumeIndex] = new_val;
	return err;
}

static int lis3dh_axis_calibrate(struct lis3dh_acc_data *acc,
				int *cal_xyz)
{
	int xyz[3] = { 0 };
	int arry[3] = { 0 };
	int err;
	int i;

	for (i = 0; i < LIS3DH_CAL_MAX; i++) {
		msleep(FACTORY_CALIBRATION_DELAY);
		err = lis3dh_acc_get_acceleration_data(acc, xyz);
		if (err < 0) {
			dev_err(&acc->client->dev,
					"get_acceleration_data failed\n");
			return err;
		}
		if (i < LIS3DH_CAL_SKIP_COUNT)
			continue;
		arry[0] += xyz[0];
		arry[1] += xyz[1];
		arry[2] += xyz[2];
	}
	cal_xyz[0] = arry[0] / (LIS3DH_CAL_MAX -
			LIS3DH_CAL_SKIP_COUNT);
	cal_xyz[1] = arry[1] / (LIS3DH_CAL_MAX -
			LIS3DH_CAL_SKIP_COUNT);
	cal_xyz[2] = arry[2] / (LIS3DH_CAL_MAX -
			LIS3DH_CAL_SKIP_COUNT);

	return 0;
}

static int lis3dh_calibrate(struct sensors_classdev *sensors_cdev,
		int axis, int apply_now)
{
	struct lis3dh_acc_data *acc = container_of(sensors_cdev,
		struct lis3dh_acc_data, cdev);
	int xyz[3] = { 0 };
	int err;

	acc->use_cal = false;
	acc->pre_enable = atomic_read(&acc->enabled);
	if (acc->pre_enable) {
		err = lis3dh_acc_disable(acc);
		if (err < 0) {
			dev_err(&acc->client->dev, "cannot disable sensor\n");
			return err;
		}
	}
	if (atomic_cmpxchg(&acc->cal_status, 0, 1)) {
		dev_err(&acc->client->dev, "do calibration error\n");
		return -EBUSY;
	}
	err = lis3dh_acc_device_power_on(acc);
	if (err < 0) {
		dev_err(&acc->client->dev, "cannot power on sensor\n");
		return err;
	}
	err = lis3dh_axis_calibrate(acc, xyz);
	if (err < 0) {
		dev_err(&acc->client->dev, "accel calibrate error\n");
		return err;
	}
	switch (axis) {
	case AXIS_X:
		xyz[1] = 0;
		xyz[2] = 0;
		break;
	case AXIS_Y:
		xyz[0] = 0;
		xyz[2] = 0;
		break;
	case AXIS_Z:
		xyz[0] = 0;
		xyz[1] = 0;
		xyz[2] = xyz[2] - RAW_TO_1G;
		break;
	case AXIS_XYZ:
		xyz[2] = xyz[2] - RAW_TO_1G;
		break;
	default:
		dev_err(&acc->client->dev, "can not calibrate accel\n");
		break;
	}
	memset(acc->calibrate_buf, 0, sizeof(acc->calibrate_buf));
	snprintf(acc->calibrate_buf, sizeof(acc->calibrate_buf),
			"%d,%d,%d", xyz[0], xyz[1], xyz[2]);
	if (apply_now) {
		acc->cal_params[0] = xyz[0];
		acc->cal_params[1] = xyz[1];
		acc->cal_params[2] = xyz[2];
		acc->use_cal = true;
	}
	lis3dh_acc_device_power_off(acc);
	atomic_set(&acc->cal_status, 0);
	if (acc->pre_enable) {
		err = lis3dh_acc_enable(acc);
		if (err < 0) {
			dev_err(&acc->client->dev, "cannot enable sensor\n");
			return err;
		}
	}

	return 0;
}

static int lis3dh_write_cal_params(struct sensors_classdev *sensors_cdev,
		struct cal_result_t *cal_result)
{
	struct lis3dh_acc_data *acc = container_of(sensors_cdev,
		struct lis3dh_acc_data, cdev);
	acc->cal_params[0] = cal_result->offset_x;
	acc->cal_params[1] = cal_result->offset_y;
	acc->cal_params[2] = cal_result->offset_z;

	snprintf(acc->calibrate_buf, sizeof(acc->calibrate_buf),
			"%d,%d,%d", acc->cal_params[0], acc->cal_params[1],
			acc->cal_params[2]);
	acc->use_cal = true;
	dev_dbg(&acc->client->dev, "read accel calibrate bias %d,%d,%d\n",
		acc->cal_params[0], acc->cal_params[1], acc->cal_params[2]);

	return 0;
}

static int lis3dh_acc_get_calibrate(struct lis3dh_acc_data *acc, int *xyz)
{
	xyz[0] -=  acc->cal_params[0];
	xyz[1] -=  acc->cal_params[1];
	xyz[2] -=  acc->cal_params[2];
	dev_dbg(&acc->client->dev, "%s read calibrate x=%d, y=%d, z=%d\n",
			LIS3DH_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);

	return 0;
}

static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	mutex_lock(&acc->lock);
	val = acc->pdata->poll_interval;
	mutex_unlock(&acc->lock);
	return snprintf(buf, 8, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (atomic_read(&acc->cal_status)) {
		dev_err(&acc->client->dev,
			"can not set rate when sensor do calibration now\n");
		return -EBUSY;
	}
	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;

	mutex_lock(&acc->lock);
	acc->pdata->poll_interval = interval_ms;
	lis3dh_acc_update_odr(acc, interval_ms);
	mutex_unlock(&acc->lock);
	#ifdef CONFIG_HUAWEI_KERNEL
	LIS3DH_INFO("%s,acc->pdata->poll_interval  = %d",__func__,acc->pdata->poll_interval );
	#endif
	return size;
}

static ssize_t attr_get_range(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	char val;
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	char range = 2;

	mutex_lock(&acc->lock);
	val = acc->pdata->g_range;
	switch (val) {
	case LIS3DH_ACC_G_2G:
		range = 2;
		break;
	case LIS3DH_ACC_G_4G:
		range = 4;
		break;
	case LIS3DH_ACC_G_8G:
		range = 8;
		break;
	case LIS3DH_ACC_G_16G:
		range = 16;
		break;
	}
	mutex_unlock(&acc->lock);
	return snprintf(buf, 4, "%d\n", range);
}

static ssize_t attr_set_range(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;
	if (kstrtoul(buf, 10, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	acc->pdata->g_range = val;
	lis3dh_acc_update_g_range(acc, val);
	mutex_unlock(&acc->lock);
	return size;
}

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	int val = atomic_read(&acc->enabled);
	return snprintf(buf, sizeof(val) + 2, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lis3dh_acc_enable(acc);
	else
		lis3dh_acc_disable(acc);

	return size;
}

static ssize_t attr_set_intconfig1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_CFG1, NO_MASK, RES_INT_CFG1);
}

static ssize_t attr_get_intconfig1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_CFG1);
}

static ssize_t attr_set_duration1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_DUR1, INT1_DURATION_MASK, RES_INT_DUR1);
}

static ssize_t attr_get_duration1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_DUR1);
}

static ssize_t attr_set_thresh1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_THS1, INT1_THRESHOLD_MASK, RES_INT_THS1);
}

static ssize_t attr_get_thresh1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_THS1);
}

static ssize_t attr_get_source1(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, INT_SRC1);
}

static ssize_t attr_set_click_cfg(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_CFG, TAP_CFG_MASK, RES_TT_CFG);
}

static ssize_t attr_get_click_cfg(struct device *dev,
		struct device_attribute *attr,	char *buf)
{

	return read_single_reg(dev, buf, TT_CFG);
}

static ssize_t attr_get_click_source(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_SRC);
}

static ssize_t attr_set_click_ths(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_THS, TAP_THS_MASK, RES_TT_THS);
}

static ssize_t attr_get_click_ths(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_THS);
}

static ssize_t attr_set_click_tlim(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_LIM, TAP_TLIM_MASK, RES_TT_LIM);
}

static ssize_t attr_get_click_tlim(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_LIM);
}

static ssize_t attr_set_click_tlat(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_TLAT, TAP_TLAT_MASK, RES_TT_TLAT);
}

static ssize_t attr_get_click_tlat(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_TLAT);
}

static ssize_t attr_set_click_tw(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_TLAT, TAP_TW_MASK, RES_TT_TLAT);
}

static ssize_t attr_get_click_tw(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_TLAT);
}


#ifdef DEBUG
/* PAY ATTENTION: These DEBUG funtions don't manage resume_state */
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int rc;
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	x[0] = acc->reg_addr;
	mutex_unlock(&acc->lock);
	x[1] = val;
	rc = lis3dh_acc_i2c_write(acc, x, 1);
	/*TODO: error need to be managed */
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&acc->lock);
	data = acc->reg_addr;
	mutex_unlock(&acc->lock);
	rc = lis3dh_acc_i2c_read(acc, &data, 1);
	/* TODO: error need to be managed */
	ret = snprintf(buf, 8, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;
	if (kstrtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	acc->reg_addr = val;
	mutex_unlock(&acc->lock);
	return size;
}
#endif

static struct device_attribute attributes[] = {

	__ATTR(poll_delay, 0664, attr_get_polling_rate,
			attr_set_polling_rate),
	__ATTR(range, 0664, attr_get_range, attr_set_range),
	__ATTR(enable, 0664, attr_get_enable, attr_set_enable),
	__ATTR(int1_config, 0664, attr_get_intconfig1, attr_set_intconfig1),
	__ATTR(int1_duration, 0664, attr_get_duration1, attr_set_duration1),
	__ATTR(int1_threshold, 0664, attr_get_thresh1, attr_set_thresh1),
	__ATTR(int1_source, 0444, attr_get_source1, NULL),
	__ATTR(click_config, 0664, attr_get_click_cfg, attr_set_click_cfg),
	__ATTR(click_source, 0444, attr_get_click_source, NULL),
	__ATTR(click_threshold, 0664, attr_get_click_ths, attr_set_click_ths),
	__ATTR(click_timelimit, 0664, attr_get_click_tlim,
			attr_set_click_tlim),
	__ATTR(click_timelatency, 0664, attr_get_click_tlat,
							attr_set_click_tlat),
	__ATTR(click_timewindow, 0664, attr_get_click_tw, attr_set_click_tw),

#ifdef DEBUG
	__ATTR(reg_value, 0664, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0220, NULL, attr_addr_set),
#endif
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	int err;
	for (i = 0; i < ARRAY_SIZE(attributes); i++) {
		err = device_create_file(dev, attributes + i);
		if (err)
			goto error;
	}
	return 0;

error:
	for (; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return err;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}

static int lis3dh_acc_flush(struct sensors_classdev *sensors_cdev)
{
	struct lis3dh_acc_data *acc = container_of(sensors_cdev,
			struct lis3dh_acc_data, cdev);
	s64 timestamp, sec, ns;
	int err;
	int fifo_cnt;
	int i;
	u32 time_ms;
	int xyz[3] = {0};

	timestamp = lis3dh_acc_get_time_ns();
	time_ms = lis3dh_acc_odr_to_interval(acc,
			(acc->resume_state[RES_CTRL_REG1] >> 4));
	fifo_cnt = lis3dh_acc_get_fifo_lvl(acc);
	dev_dbg(&acc->client->dev, "TS: base=%lld, interval=%d fifo_cnt=%d\n",
			timestamp, time_ms, fifo_cnt);
	timestamp = timestamp -
		((s64)time_ms * LIS3DH_TIME_MS_TO_NS * fifo_cnt);

	for (i = 0; i < fifo_cnt; i++) {
		err = lis3dh_acc_get_acceleration_data(acc, xyz);
		if (err < 0) {
			dev_err(&acc->client->dev,
					"get_acceleration_data failed\n");
			goto exit;
		} else {
			timestamp = timestamp +
				(time_ms * LIS3DH_TIME_MS_TO_NS);
			sec = timestamp;
			ns = do_div(sec, NSEC_PER_SEC);
			input_event(acc->input_dev, EV_SYN, SYN_TIME_SEC, sec);
			input_event(acc->input_dev, EV_SYN, SYN_TIME_NSEC, ns);
			input_report_abs(acc->input_dev, ABS_X, xyz[0]);
			input_report_abs(acc->input_dev, ABS_Y, xyz[1]);
			input_report_abs(acc->input_dev, ABS_Z, xyz[2]);
			input_sync(acc->input_dev);
		}
	}

	input_event(acc->input_dev, EV_SYN, SYN_CONFIG, acc->flush_count++);
	input_sync(acc->input_dev);

	return 0;
exit:
	return err;
}

static int lis3dh_acc_poll_delay_set(struct sensors_classdev *sensors_cdev,
	unsigned int delay_msec)
{
	struct lis3dh_acc_data *acc = container_of(sensors_cdev,
		struct lis3dh_acc_data, cdev);
	int err;

	if (atomic_read(&acc->cal_status)) {
		dev_err(&acc->client->dev,
			"can not set delay when sensor do calibration now\n");
		return -EBUSY;
	}
	dev_dbg(&acc->client->dev, "set poll delay =%d\n", delay_msec);
	mutex_lock(&acc->lock);
	acc->pdata->poll_interval = delay_msec;
	err = lis3dh_acc_update_odr(acc, delay_msec);
	mutex_unlock(&acc->lock);
	#ifdef CONFIG_HUAWEI_KERNEL
	LIS3DH_INFO("%s,acc->pdata->poll_interval  = %d",__func__,acc->pdata->poll_interval );
	#endif
	return err;
}

static int lis3dh_acc_enable_set(struct sensors_classdev *sensors_cdev,
	unsigned int enable)
{
	struct lis3dh_acc_data *acc = container_of(sensors_cdev,
		struct lis3dh_acc_data, cdev);
	int err;

	if (enable)
		err = lis3dh_acc_enable(acc);
	else
		err = lis3dh_acc_disable(acc);
	return err;
}
/**
*	print main registers and xyz vaules
*/
#ifdef CONFIG_HUAWEI_KERNEL
static void dump_regs_xyz_values(struct work_struct *work)
{
	struct lis3dh_acc_data *acc = container_of(cdev,
		struct lis3dh_acc_data, cdev);
	struct i2c_client *client = acc->client;
	int ret;

	mutex_lock(&acceld->lock);
	acceld->queued_debug_work_flag = false;
	reg[0] = CTRL_REG1;
	err = acceld->i2c_read(acceld, reg, 6);
	if(err < 0){
		LIS3DH_ERR("[lis3dh_err]%s,line %d: failed to read regs value",__func__,__LINE__);
		return;
	}

	acc->fifo_timeout_ms = max_latency;
	acc->use_batch = max_latency ? true : false;

	ret = lis3dh_acc_enable_batch(acc, max_latency);
	if (ret) {
		dev_err(&client->dev, "enable batch:%d failed\n", max_latency);
		return ret;
	}

	return 0;
}
#endif

static void lis3dh_acc_input_work_func(struct work_struct *work)
{
	struct lis3dh_acc_data *acc;

	int xyz[3] = { 0 };
	int err;

	acc = container_of((struct delayed_work *)work,
			struct lis3dh_acc_data,	input_work);

	mutex_lock(&acc->lock);
	err = lis3dh_acc_get_acceleration_data(acc, xyz);
	if (err < 0)
		dev_err(&acc->client->dev, "get_acceleration_data failed\n");
	else
		lis3dh_acc_report_values(acc, xyz);

	schedule_delayed_work(&acc->input_work, msecs_to_jiffies(
			acc->pdata->poll_interval));
	mutex_unlock(&acc->lock);
	if (sensorDT_mode)
	{
		Gsensor_data_count++;
	}
}

int lis3dh_acc_input_open(struct input_dev *input)
{
	struct lis3dh_acc_data *acc = input_get_drvdata(input);

	return lis3dh_acc_enable(acc);
}

void lis3dh_acc_input_close(struct input_dev *dev)
{
	struct lis3dh_acc_data *acc = input_get_drvdata(dev);

	lis3dh_acc_disable(acc);
}

static int lis3dh_acc_validate_pdata(struct lis3dh_acc_data *acc)
{
	acc->pdata->poll_interval = max(acc->pdata->poll_interval,
			acc->pdata->min_interval);

	if (acc->pdata->axis_map_x > 2 ||
		acc->pdata->axis_map_y > 2 ||
		 acc->pdata->axis_map_z > 2) {
		dev_err(&acc->client->dev,
			"invalid axis_map value x:%u y:%u z%u\n",
			acc->pdata->axis_map_x,
			acc->pdata->axis_map_y, acc->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (acc->pdata->negate_x > 1 || acc->pdata->negate_y > 1
			|| acc->pdata->negate_z > 1) {
		dev_err(&acc->client->dev,
			"invalid negate value x:%u y:%u z:%u\n",
			acc->pdata->negate_x,
			acc->pdata->negate_y, acc->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (acc->pdata->poll_interval < acc->pdata->min_interval) {
		dev_err(&acc->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lis3dh_acc_input_init(struct lis3dh_acc_data *acc)
{
	int err;

	INIT_DELAYED_WORK(&acc->input_work, lis3dh_acc_input_work_func);
	#ifdef CONFIG_HUAWEI_KERNEL
	/* use delay work to debug x,y,z value, this will make cpu efficiently than old method*/
	INIT_DELAYED_WORK(&acc->debug_work, dump_regs_xyz_values);
	acc->queued_debug_work_flag = false;
	#endif
	acc->input_dev = input_allocate_device();
	if (!acc->input_dev) {
		err = -ENOMEM;
		dev_err(&acc->client->dev, "input device allocation failed\n");
		goto err0;
	}

	acc->input_dev->open = lis3dh_acc_input_open;
	acc->input_dev->close = lis3dh_acc_input_close;
	acc->input_dev->name = ACCEL_INPUT_DEV_NAME;
	acc->input_dev->id.bustype = BUS_I2C;
	acc->input_dev->dev.parent = &acc->client->dev;

	input_set_drvdata(acc->input_dev, acc);

	set_bit(EV_ABS, acc->input_dev->evbit);
	/*	next is used for interruptA sources data if the case */
	set_bit(ABS_MISC, acc->input_dev->absbit);
	/*	next is used for interruptB sources data if the case */
	set_bit(ABS_WHEEL, acc->input_dev->absbit);

	input_set_abs_params(acc->input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(acc->input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(acc->input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);
	/*	next is used for interruptA sources data if the case */
	input_set_abs_params(acc->input_dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
	/*	next is used for interruptB sources data if the case */
	input_set_abs_params(acc->input_dev, ABS_WHEEL, INT_MIN, INT_MAX, 0, 0);


	err = input_register_device(acc->input_dev);
	if (err) {
		dev_err(&acc->client->dev,
				"unable to register input device %s\n",
				acc->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(acc->input_dev);
err0:
	return err;
}

static void lis3dh_acc_input_cleanup(struct lis3dh_acc_data *acc)
{
	input_unregister_device(acc->input_dev);
	input_free_device(acc->input_dev);
}

static int lis3dh_pinctrl_init(struct lis3dh_acc_data *acc)
{
	struct i2c_client *client = acc->client;

	acc->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(acc->pinctrl)) {
		dev_err(&client->dev, "Failed to get pinctrl\n");
		return PTR_ERR(acc->pinctrl);
	}

	acc->pin_default =
		pinctrl_lookup_state(acc->pinctrl, "lis3dh_default");
	if (IS_ERR_OR_NULL(acc->pin_default)) {
		dev_err(&client->dev, "Failed to look up default state\n");
		return PTR_ERR(acc->pin_default);
	}

	acc->pin_sleep =
		pinctrl_lookup_state(acc->pinctrl, "lis3dh_sleep");
	if (IS_ERR_OR_NULL(acc->pin_sleep)) {
		dev_err(&client->dev, "Failed to look up sleep state\n");
		return PTR_ERR(acc->pin_sleep);
	}

	return 0;
}

#ifdef CONFIG_OF
#ifdef CONFIG_HUAWEI_KERNEL
static int lis3dh_parse_dt(struct device *dev,
			struct lis3dh_acc_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;

	rc = of_property_read_u32(np, "st,min-interval", &temp_val);
	if (rc && (rc != -EINVAL)) {
		pdata->min_interval = DEFAULT_MIN_INTERVAL;
	} else {
		pdata->min_interval = temp_val;
	}

	rc = of_property_read_u32(np, "st,init-interval", &temp_val);
	if (rc && (rc != -EINVAL)) {
		pdata->poll_interval = DEFAULT_POLL_INTERVAL;
	} else {
		pdata->poll_interval = temp_val;
	}

	rc = of_property_read_u32(np, "st,axis-map-bottom", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read axis-map-bottom, use default map_direction\n");
		temp_val = DEFAULT_GS_MAP_DIRECTION;
	}

	switch(temp_val){
		case GS_MAP_DIRECTION_BOTTOM:
			pdata->axis_map_x = 0;
			pdata->axis_map_y = 1;
			pdata->axis_map_z = 2;
			pdata->negate_x 	= 0;
			pdata->negate_y	= 1;
			pdata->negate_z 	= 1;
			break;
		case GS_MAP_DIRECTION_BOTTOM_LEFT_TOP:
			pdata->axis_map_x = 1;
			pdata->axis_map_y = 0;
			pdata->axis_map_z = 2;
			pdata->negate_x 	= 0;
			pdata->negate_y	= 0;
			pdata->negate_z 	= 1;
			break;
		default:
			pdata->axis_map_x = 0;
			pdata->axis_map_y = 1;
			pdata->axis_map_z = 2;
			pdata->negate_x 	= 1;
			pdata->negate_y	= 1;
			pdata->negate_z 	= 0;
			break;
	}

	rc = of_property_read_u32(np, "st,g-range", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read g-range, use default g-range\n");
		temp_val = DEFAULT_G_RANGE;
	} else {
		switch (temp_val) {
		case 2:
			pdata->g_range = LIS3DH_ACC_G_2G;
			break;
		case 4:
			pdata->g_range = LIS3DH_ACC_G_4G;
			break;
		case 8:
			pdata->g_range = LIS3DH_ACC_G_8G;
			break;
		case 16:
			pdata->g_range = LIS3DH_ACC_G_16G;
			break;
		default:
			pdata->g_range = LIS3DH_ACC_G_2G;
			break;
		}
	}
	pdata->gpio_int1 = of_get_named_gpio_flags(dev->of_node,
				"st,gpio-int1", 0, NULL);

	pdata->gpio_int2 = of_get_named_gpio_flags(dev->of_node,
				"st,gpio-int2", 0, NULL);

	return 0;
}
#else
static int lis3dh_parse_dt(struct device *dev,
			struct lis3dh_acc_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;

	rc = of_property_read_u32(np, "st,min-interval", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read min-interval\n");
		return rc;
	} else {
		pdata->min_interval = temp_val;
	}

	rc = of_property_read_u32(np, "st,init-interval", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read init-interval\n");
		return rc;
	} else {
		pdata->poll_interval = temp_val;
	}

	rc = of_property_read_u32(np, "st,axis-map-x", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read axis-map_x\n");
		return rc;
	} else {
		pdata->axis_map_x = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "st,axis-map-y", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read axis_map_y\n");
		return rc;
	} else {
		pdata->axis_map_y = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "st,axis-map-z", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read axis-map-z\n");
		return rc;
	} else {
		pdata->axis_map_z = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "st,g-range", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read g-range\n");
		return rc;
	} else {
		switch (temp_val) {
		case 2:
			pdata->g_range = LIS3DH_ACC_G_2G;
			break;
		case 4:
			pdata->g_range = LIS3DH_ACC_G_4G;
			break;
		case 8:
			pdata->g_range = LIS3DH_ACC_G_8G;
			break;
		case 16:
			pdata->g_range = LIS3DH_ACC_G_16G;
			break;
		default:
			pdata->g_range = LIS3DH_ACC_G_2G;
			break;
		}
	}

	pdata->negate_x = of_property_read_bool(np, "st,negate-x");

	pdata->negate_y = of_property_read_bool(np, "st,negate-y");

	pdata->negate_z = of_property_read_bool(np, "st,negate-z");

	pdata->gpio_int1 = of_get_named_gpio_flags(dev->of_node,
				"st,gpio-int1", 0, NULL);

	pdata->gpio_int2 = of_get_named_gpio_flags(dev->of_node,
				"st,gpio-int2", 0, NULL);
	return 0;
}
#endif
#else
static int lis3dh_parse_dt(struct device *dev,
			struct lis3dh_acc_platform_data *pdata)
{
	return -EINVAL;
}
#endif


static int lis3dh_acc_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{

	struct lis3dh_acc_data *acc;
	int err = -1;
	LIS3DH_INFO("%s,line %d: PROBE START.",__func__,__LINE__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	acc = kzalloc(sizeof(struct lis3dh_acc_data), GFP_KERNEL);
	if (acc == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for module data: "
					"%d\n", err);
		goto exit_check_functionality_failed;
	}


	mutex_init(&acc->lock);
	mutex_init(&acc->lock_i2c);
	mutex_lock(&acc->lock);

	acc->client = client;
#ifdef CONFIG_HUAWEI_KERNEL
	acc->device_exist = false;
#endif
	i2c_set_clientdata(client, acc);

	acc->pdata = kmalloc(sizeof(*acc->pdata), GFP_KERNEL);
	if (acc->pdata == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for pdata: %d\n",
				err);
		goto err_mutexunlock;
	}
	memset(acc->pdata, 0 , sizeof(*acc->pdata));

	if (client->dev.of_node) {
		err = lis3dh_parse_dt(&client->dev, acc->pdata);
		if (err) {
			dev_err(&client->dev, "Failed to parse device tree\n");
			err = -EINVAL;
			goto exit_kfree_pdata;
		}
	} else if (client->dev.platform_data != NULL) {
		memcpy(acc->pdata, client->dev.platform_data,
			sizeof(*acc->pdata));
	} else {
		dev_err(&client->dev, "No valid platform data. exiting.\n");
		err = -ENODEV;
		goto exit_kfree_pdata;
	}

	err = lis3dh_acc_validate_pdata(acc);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}
	memcpy(acc->lis3dh_acc_vreg,lis3dh_acc_vreg,2*sizeof(struct sensor_regulator));
	acc->i2c_read = lis3dh_acc_i2c_read;
#ifdef CONFIG_HUAWEI_DSM
	err = lis_dsm_excep_init(acc);
	if(err < 0){
		LIS3DH_ERR("%s %d:lis_dsm_excep_init error! \n", __func__, __LINE__);
		goto err_dsm_excep_exit;
	}
#endif


	/* initialize pinctrl */
	err = lis3dh_pinctrl_init(acc);
	if (err) {
		dev_err(&client->dev, "Can't initialize pinctrl\n");
			goto exit_kfree_pdata;
	}
	err = pinctrl_select_state(acc->pinctrl, acc->pin_default);
	if (err) {
		dev_err(&client->dev,
			"Can't select pinctrl default state\n");
		goto exit_kfree_pdata;
	}

	if (acc->pdata->init) {
		err = acc->pdata->init();
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err_pdata_init;
		}
	}

	if (gpio_is_valid(acc->pdata->gpio_int1))
		acc->irq1 = gpio_to_irq(acc->pdata->gpio_int1);

	if (gpio_is_valid(acc->pdata->gpio_int2))
		acc->irq2 = gpio_to_irq(acc->pdata->gpio_int2);

	memset(acc->resume_state, 0, ARRAY_SIZE(acc->resume_state));

	acc->resume_state[RES_CTRL_REG1] = LIS3DH_ACC_ENABLE_ALL_AXES;
	acc->resume_state[RES_CTRL_REG2] = 0x00;
	acc->resume_state[RES_CTRL_REG3] = 0x00;
	acc->resume_state[RES_CTRL_REG4] = 0x00;
	acc->resume_state[RES_CTRL_REG5] = 0x00;
	acc->resume_state[RES_CTRL_REG6] = 0x00;

	acc->resume_state[RES_TEMP_CFG_REG] = 0x00;
	acc->resume_state[RES_FIFO_CTRL_REG] = 0x00;
	acc->resume_state[RES_INT_CFG1] = 0x00;
	acc->resume_state[RES_INT_THS1] = 0x00;
	acc->resume_state[RES_INT_DUR1] = 0x00;

	acc->resume_state[RES_TT_CFG] = 0x00;
	acc->resume_state[RES_TT_THS] = 0x00;
	acc->resume_state[RES_TT_LIM] = 0x00;
	acc->resume_state[RES_TT_TLAT] = 0x00;
	acc->resume_state[RES_TT_TW] = 0x00;
	acc->cal_params[0] = 0;
	acc->cal_params[1] = 0;
	acc->cal_params[2] = 0;
	acc->use_cal = false;
	atomic_set(&acc->cal_status, 0);

	err = lis3dh_acc_device_power_on(acc);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err_pdata_init;
	}

	atomic_set(&acc->enabled, 1);

	err = lis3dh_acc_update_g_range(acc, acc->pdata->g_range);
	if (err < 0) {
		dev_err(&client->dev, "update_g_range failed\n");
		goto  err_power_off;
	}

	err = lis3dh_acc_update_odr(acc, acc->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto  err_power_off;
	}

	err = lis3dh_acc_input_init(acc);
	if (err < 0) {
		dev_err(&client->dev, "input init failed\n");
		goto err_power_off;
	}

	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		dev_err(&client->dev,
		   "device LIS3DH_ACC_DEV_NAME sysfs register failed\n");
		goto err_input_cleanup;
	}

	acc->cdev = lis3dh_acc_cdev;
	acc->cdev.sensors_enable = lis3dh_acc_enable_set;
	acc->cdev.sensors_poll_delay = lis3dh_acc_poll_delay_set;
	/* batch is possiable only when interrupt is enabled */
	if (gpio_is_valid(acc->pdata->gpio_int1) && acc->pdata->enable_int) {
		acc->cdev.sensors_set_latency = lis3dh_latency_set;
		acc->cdev.sensors_flush = lis3dh_acc_flush;
		acc->cdev.fifo_reserved_event_count = LIS3DH_FIFO_SIZE;
		acc->cdev.fifo_max_event_count = LIS3DH_FIFO_SIZE;
		acc->cdev.max_delay = LIS_INT_MAX_DELAY;
	}
	acc->cdev.sensors_calibrate = lis3dh_calibrate;
	acc->cdev.sensors_write_cal_params = lis3dh_write_cal_params;
	memset(&acc->cdev.cal_result, 0, sizeof(acc->cdev.cal_result));
	acc->cdev.params = acc->calibrate_buf;
	err = sensors_classdev_register(&acc->input_dev->dev, &acc->cdev);
	if (err) {
		dev_err(&client->dev,
			"class device create failed: %d\n", err);
		goto err_remove_sysfs_int;
	}

	lis3dh_acc_device_power_off(acc);

	/* As default, do not report information */
	atomic_set(&acc->enabled, 0);

	if (gpio_is_valid(acc->pdata->gpio_int1)) {
		INIT_WORK(&acc->irq1_work, lis3dh_acc_irq1_work_func);
		acc->irq1_work_queue =
			create_singlethread_workqueue("lis3dh_acc_wq1");
		if (!acc->irq1_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev,
					"cannot create work queue1: %d\n", err);
			goto err_unreg_sensor_class;
		}
		err = request_irq(acc->irq1, lis3dh_acc_isr1,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"lis3dh_acc_irq1", acc);
		if (err < 0) {
			dev_err(&client->dev, "request irq1 failed: %d\n", err);
			goto err_destoyworkqueue1;
		}
		disable_irq_nosync(acc->irq1);
	}

	if (gpio_is_valid(acc->pdata->gpio_int2)) {
		INIT_WORK(&acc->irq2_work, lis3dh_acc_irq2_work_func);
		acc->irq2_work_queue =
			create_singlethread_workqueue("lis3dh_acc_wq2");
		if (!acc->irq2_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev,
					"cannot create work queue2: %d\n", err);
			goto err_free_irq1;
		}
		err = request_irq(acc->irq2, lis3dh_acc_isr2,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"lis3dh_acc_irq2", acc);
		if (err < 0) {
			dev_err(&client->dev, "request irq2 failed: %d\n", err);
			goto err_destoyworkqueue2;
		}
		disable_irq_nosync(acc->irq2);
	}

	if (pinctrl_select_state(acc->pinctrl, acc->pin_sleep))
		dev_err(&client->dev,
			"Can't select pinctrl sleep state\n");

	mutex_unlock(&acc->lock);
	set_sensors_list(G_SENSOR);
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
	set_hw_dev_flag(DEV_I2C_G_SENSOR);
#endif
	err = set_sensor_input(ACC, acc->input_dev->dev.kobj.name);
	if (err) {
		dev_err(&client->dev, "%s set_sensor_input failed\n", __func__);
	}
	err = app_info_set("G-Sensor",  "ST LIS3DH");
	if (err < 0)/*failed to add app_info*/
	{
		dev_err(&acc->client->dev,"%s %d:failed to add app_info\n", __func__, __LINE__);
	}
#ifdef CONFIG_HUAWEI_KERNEL
	acc->device_exist = true;
#endif
	LIS3DH_INFO("%s,line %d: PROBE SUCCESS",__func__,__LINE__);
	return 0;

err_destoyworkqueue2:
	if (gpio_is_valid(acc->pdata->gpio_int2))
		destroy_workqueue(acc->irq2_work_queue);
err_free_irq1:
	free_irq(acc->irq1, acc);
err_destoyworkqueue1:
	if (gpio_is_valid(acc->pdata->gpio_int1))
		destroy_workqueue(acc->irq1_work_queue);
err_unreg_sensor_class:
	sensors_classdev_unregister(&acc->cdev);
err_remove_sysfs_int:
	remove_sysfs_interfaces(&client->dev);
err_input_cleanup:
	lis3dh_acc_input_cleanup(acc);
err_power_off:
	lis3dh_acc_device_power_off(acc);
err_pdata_init:
	if (acc->pdata->exit)
		acc->pdata->exit();

#ifdef CONFIG_HUAWEI_DSM
err_dsm_excep_exit:
	lis_dsm_excep_exit(acc);
#endif

exit_kfree_pdata:
	kfree(acc->pdata);
err_mutexunlock:
	mutex_unlock(&acc->lock);
	kfree(acc);
exit_check_functionality_failed:
	dev_err(&client->dev, "%s: Driver Init failed\n", LIS3DH_ACC_DEV_NAME);
	return err;
}

static int lis3dh_acc_remove(struct i2c_client *client)
{
	struct lis3dh_acc_data *acc = i2c_get_clientdata(client);

	if (gpio_is_valid(acc->pdata->gpio_int1)) {
		free_irq(acc->irq1, acc);
		gpio_free(acc->pdata->gpio_int1);
		destroy_workqueue(acc->irq1_work_queue);
	}

	if (gpio_is_valid(acc->pdata->gpio_int2)) {
		free_irq(acc->irq2, acc);
		gpio_free(acc->pdata->gpio_int2);
		destroy_workqueue(acc->irq2_work_queue);
	}

	sensors_classdev_unregister(&acc->cdev);
	lis3dh_acc_input_cleanup(acc);
	lis3dh_acc_device_power_off(acc);
	remove_sysfs_interfaces(&client->dev);

	if (acc->pdata->exit)
		acc->pdata->exit();
#ifdef CONFIG_HUAWEI_DSM
	lis_dsm_excep_exit(acc);
#endif

	kfree(acc->pdata);
	kfree(acc);

	return 0;
}

#ifdef CONFIG_PM
static int lis3dh_acc_resume(struct i2c_client *client)
{
	struct lis3dh_acc_data *acc = i2c_get_clientdata(client);

	if (acc->on_before_suspend)
		return lis3dh_acc_enable(acc);
	return 0;
}

static int lis3dh_acc_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct lis3dh_acc_data *acc = i2c_get_clientdata(client);

	acc->on_before_suspend = atomic_read(&acc->enabled);
	return lis3dh_acc_disable(acc);
}
#else
#define lis3dh_acc_suspend	NULL
#define lis3dh_acc_resume	NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id lis3dh_acc_id[]
		= { { LIS3DH_ACC_DEV_NAME, 0 }, { }, };

static struct of_device_id lis3dh_acc_match_table[] = {
	{ .compatible = "st,lis3dh", },
	{ },
};

MODULE_DEVICE_TABLE(i2c, lis3dh_acc_id);

static struct i2c_driver lis3dh_acc_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = LIS3DH_ACC_DEV_NAME,
			.of_match_table = lis3dh_acc_match_table,
		  },
	.probe = lis3dh_acc_probe,
	.remove = lis3dh_acc_remove,
	.suspend = lis3dh_acc_suspend,
	.resume = lis3dh_acc_resume,
	.id_table = lis3dh_acc_id,
};

static int __init lis3dh_acc_init(void)
{
	return i2c_add_driver(&lis3dh_acc_driver);
}

static void __exit lis3dh_acc_exit(void)
{
	i2c_del_driver(&lis3dh_acc_driver);
	return;
}

module_init(lis3dh_acc_init);
module_exit(lis3dh_acc_exit);

MODULE_DESCRIPTION("lis3dh digital accelerometer sysfs driver");
MODULE_AUTHOR("Matteo Dameno, Carmine Iascone, Samuel Huo, STMicroelectronics");
MODULE_LICENSE("GPL");


