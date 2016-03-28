/* drivers/input/misc/kionix_accel.c - Kionix accelerometer driver
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
#include	<linux/err.h>
#include	<linux/errno.h>
#include	<linux/delay.h>
#include	<linux/fs.h>
#include	<linux/kernel.h>
#include	<linux/i2c.h>
#include	<linux/input.h>
#include	<linux/uaccess.h>
#include	<linux/interrupt.h>
#include	<linux/workqueue.h>
#include	<linux/module.h>
#include	<linux/irq.h>
#include	<linux/gpio.h>
#include	<linux/slab.h>
#include	<linux/version.h>
#include	<linux/proc_fs.h>
#include	<linux/regulator/consumer.h>
#include	<linux/of_gpio.h>
#include	<linux/sensors.h>
#include	<huawei_platform/sensor/hw_sensor_info.h>
#ifdef	CONFIG_HAS_EARLYSUSPEND
#include	<linux/earlysuspend.h>
#endif /* CONFIG_HAS_EARLYSUSPEND */
#ifdef	CONFIG_HUAWEI_HW_DEV_DCT
#include	<linux/hw_dev_dec.h>
#endif
#include	<huawei_platform/sensor/kionix_accel.h>
#include <misc/app_info.h>
/*move so many Macro definitions and structs to kionix_accel.h*/

int kx023_debug_mask = 1;
module_param_named(kx023_debug, kx023_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);
static int olddata[3] = {0,0,0};
static u8 dataflag = 0;
/* The following table lists the maximum appropriate poll interval for each
 * available output data rate (ODR).
 */
 static const struct {
	unsigned int cutoff;
	u8 mask;
} kionix_accel_odr_table[] = {
	{ 0,	ACCEL_ODR1600 },
	{ 0,	ACCEL_ODR800 },
	{ 0,	ACCEL_ODR400 },
	{ 10,	ACCEL_ODR200 },
	{ 20,	ACCEL_ODR100 },
	{ 40,	ACCEL_ODR50  },
	{ 80,	ACCEL_ODR25  },
	{ 160,	ACCEL_ODR12_5},
	{ 320,	ACCEL_ODR6_25},
	{ 640,	ACCEL_ODR3_125},
	{1280,	ACCEL_ODR1_563},
	{ 0,	ACCEL_ODR0_781},
};


static struct sensors_classdev kionix_acc_cdev = {
	.name = "kx023-accel",
	.vendor = "KIONIXMicroelectronics",
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
};

static struct sensor_regulator kionix_acc_vreg[] = {
	{NULL, "vdd", 1700000, 3600000,false},
	{NULL, "vddio", 1700000, 3600000,false},
};

static void kionix_accel_update_g_range(struct kionix_accel_driver *acceld);
static void kionix_accel_update_direction(struct kionix_accel_driver *acceld);
static int kionix_acc_poll_delay_set(struct sensors_classdev *sensors_cdev,unsigned int delay_msec);
static int kionix_acc_enable_set(struct sensors_classdev *sensors_cdev,unsigned int enable);
static void print_default_config(struct kionix_accel_platform_data *pdata);

static int kionix_acc_config_regulator(struct kionix_accel_driver *acc, bool on)
{
	int rc = 0, i;
	int num_reg = 2;
	struct sensor_regulator *kionix_acc_vreg = acc->kionix_acc_vreg;


	if (on) {
		for (i = 0; i < num_reg; i++) {
			if(false == kionix_acc_vreg[i].got_regulator_flag){
				kionix_acc_vreg[i].vreg = regulator_get(&acc->client->dev,
					kionix_acc_vreg[i].name);
				kionix_acc_vreg[i].got_regulator_flag = true;
			}
			if (IS_ERR(kionix_acc_vreg[i].vreg)) {
				rc = PTR_ERR(kionix_acc_vreg[i].vreg);
				pr_err("%s:regulator get failed rc=%d\n",
								__func__, rc);
				kionix_acc_vreg[i].vreg = NULL;
				goto error_vdd;
			}
			if (regulator_count_voltages(
				kionix_acc_vreg[i].vreg) > 0) {
				rc = regulator_set_voltage(
					kionix_acc_vreg[i].vreg,
					kionix_acc_vreg[i].min_uV,
					kionix_acc_vreg[i].max_uV);
				if (rc) {
					pr_err("%s: set voltage failed rc=%d\n",
					__func__, rc);
					regulator_put(kionix_acc_vreg[i].vreg);
					kionix_acc_vreg[i].vreg = NULL;
					goto error_vdd;
				}
			}

			rc = regulator_enable(kionix_acc_vreg[i].vreg);
			if (rc) {
				pr_err("%s: regulator_enable failed rc =%d\n",
					__func__, rc);
				if (regulator_count_voltages(
					kionix_acc_vreg[i].vreg) > 0) {
					regulator_set_voltage(
						kionix_acc_vreg[i].vreg, 0,
						kionix_acc_vreg[i].max_uV);
				}
				regulator_put(kionix_acc_vreg[i].vreg);
				kionix_acc_vreg[i].vreg = NULL;
				goto error_vdd;
			}
		}
		return rc;
	} else {
		i = num_reg;
	}

error_vdd:
	while (--i >= 0) {
		if (!IS_ERR_OR_NULL(kionix_acc_vreg[i].vreg)) {
			if (regulator_count_voltages(
			kionix_acc_vreg[i].vreg) > 0) {
				regulator_set_voltage(kionix_acc_vreg[i].vreg,
						0, kionix_acc_vreg[i].max_uV);
			}
			regulator_disable(kionix_acc_vreg[i].vreg);
			regulator_put(kionix_acc_vreg[i].vreg);
			kionix_acc_vreg[i].vreg = NULL;
			kionix_acc_vreg[i].got_regulator_flag = false;
		}
	}

	return rc;
}

static int kionix_i2c_write(struct kionix_accel_driver *acceld, u8 reg, u8 value)
{
	int err,loop;
		/*delete it , use short code segment */

	loop = KIONIX_I2C_RETRY_COUNT;
	while(loop) {
		mutex_lock(&acceld->lock_i2c);
		err = i2c_smbus_write_byte_data(acceld->client, reg, value);
		mutex_unlock(&acceld->lock_i2c);
		if(err < 0){
			loop--;
			mdelay(KIONIX_I2C_RETRY_TIMEOUT);
		}
		else
			break;
	}


	if(loop == 0){
		KIONIX_ERR("---------cut here-------\n"
			"WARNING:at %s %d:kx023 acceler data is exception %s() err=%d "
			,__FILE__,__LINE__,__func__,err);
#ifdef CONFIG_HUAWEI_DSM
		acceld->kx023_dsm_operation.dump_i2c_status(acceld,err);
#endif
	}
	return err;
}


static int kionix_i2c_read(struct kionix_accel_driver *acceld, u8 addr, u8 *buf,int len)
{
	int err,loop;
		/*delete it , use short code segment */

	struct i2c_msg msgs[] = {
		{
			.addr = acceld->client->addr,
			.flags = acceld->client->flags,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = acceld->client->addr,
			.flags = acceld->client->flags | I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	loop = KIONIX_I2C_RETRY_COUNT;
	while(loop) {
		mutex_lock(&acceld->lock_i2c);
		err =  i2c_transfer(acceld->client->adapter, msgs, 2);
		mutex_unlock(&acceld->lock_i2c);
		if(err < 0){
			loop--;
			mdelay(KIONIX_I2C_RETRY_TIMEOUT);
		}
		else
			break;
	}

	if(loop == 0){
		KIONIX_ERR("---------cut here-------\n"
			"WARNING:at %s %d:kx023 acceler data is exception %s() err=%d"
			,__FILE__,__LINE__,__func__,err);
#ifdef CONFIG_HUAWEI_DSM
		acceld->kx023_dsm_operation.dump_i2c_status(acceld,err);
#endif
	}

	return err;
}

/* move these check exception functions to kionix_accel_monitor.c*/

static int kionix_strtok(const char *buf, size_t count, char **token, const int token_nr)
{
	char *buf2 = (char *)kzalloc((count + 1) * sizeof(char), GFP_KERNEL);
	char **token2 = token;
	unsigned int num_ptr = 0, num_nr = 0, num_neg = 0;
	int i = 0, start = 0, end = (int)count;

	memcpy(buf2, buf,count);

	/* We need to breakup the string into separate chunks in order for kstrtoint
	 * or strict_strtol to parse them without returning an error. Stop when the end of
	 * the string is reached or when enough value is read from the string */
	while((start < end) && (i < token_nr)) {
		/* We found a negative sign */
		if(*(buf2 + start) == '-') {
			/* Previous char(s) are numeric, so we store their value first before proceed */
			if(num_nr > 0) {
				/* If there is a pending negative sign, we adjust the variables to account for it */
				if(num_neg) {
					num_ptr--;
					num_nr++;
				}
				*token2 = (char *)kzalloc((num_nr + 2) * sizeof(char), GFP_KERNEL);
				strncpy(*token2, (const char *)(buf2 + num_ptr), (size_t) num_nr);
				*(*token2+num_nr) = '\n';
				i++;
				token2++;
				/* Reset */
				num_ptr = num_nr = 0;
			}
			/* This indicates that there is a pending negative sign in the string */
			num_neg = 1;
		}
		/* We found a numeric */
		else if((*(buf2 + start) >= '0') && (*(buf2 + start) <= '9')) {
			/* If the previous char(s) are not numeric, set num_ptr to current char */
			if(num_nr < 1)
				num_ptr = start;
			num_nr++;
		}
		/* We found an unwanted character */
		else {
			/* Previous char(s) are numeric, so we store their value first before proceed */
			if(num_nr > 0) {
				if(num_neg) {
					num_ptr--;
					num_nr++;
				}
				*token2 = (char *)kzalloc((num_nr + 2) * sizeof(char), GFP_KERNEL);
				strncpy(*token2, (const char *)(buf2 + num_ptr), (size_t) num_nr);
				*(*token2+num_nr) = '\n';
				i++;
				token2++;
			}
			/* Reset all the variables to start afresh */
			num_ptr = num_nr = num_neg = 0;
		}
		start++;
	}

	kfree(buf2);

	return (i == token_nr) ? token_nr : -1;
}



static int kionix_accel_init_and_power_on(struct kionix_accel_driver *acceld)
{
	int err;

	/* update output data rate, range and direction*/
	kionix_accel_update_direction(acceld);
	kionix_accel_update_g_range(acceld);
	err = acceld->kionix_accel_update_odr(acceld, acceld->poll_interval);
	if(err < 0)
		KIONIX_ERR("init_and_power_on failed.");

	return 0;
}

static int kionix_accel_operate(struct kionix_accel_driver *acceld)
{
	int err;

	err = kionix_i2c_write(acceld,ACCEL_CTRL_REG1, 0);
	if (err < 0) {
		KIONIX_ERR("%s:%d write reg:%x,value:0,failed:%d\n", __FUNCTION__, __LINE__, ACCEL_CTRL_REG1, err);
		return err;
	}

	err = kionix_i2c_write(acceld,ACCEL_DATA_CTRL, acceld->accel_registers[accel_data_ctrl]);
	if (err < 0) {
		KIONIX_ERR("%s:%d write reg:%x,value:%x,failed:%d\n", __FUNCTION__, __LINE__, ACCEL_DATA_CTRL,\
					 acceld->accel_registers[accel_data_ctrl], err);
		return err;
	}

	err = kionix_i2c_write(acceld,ACCEL_CTRL_REG1, acceld->accel_registers[accel_ctrl_reg1] | ACCEL_PC1_ON);
	if (err < 0) {
		KIONIX_ERR("%s:%d write reg:%x,value:%x,failed:%d\n", __FUNCTION__, __LINE__, ACCEL_CTRL_REG1,\
					 acceld->accel_registers[accel_ctrl_reg1] | ACCEL_PC1_ON, err);
		return err;
	}
	queue_delayed_work(acceld->accel_workqueue, &acceld->accel_work, acceld->poll_delay);

	KIONIX_INFO("enter operate mode, poll_delay = %d",acceld->poll_delay);
	return 0;
}

static int kionix_accel_standby(struct kionix_accel_driver *acceld)
{
	int err;

	/*enter stand-by mode*/
	err = kionix_i2c_write(acceld, ACCEL_CTRL_REG1, 0);
	if (err < 0) {
		KIONIX_ERR("%s:%d write reg:%x,value:0,failed!\n", __FUNCTION__, __LINE__, ACCEL_CTRL_REG1);
		return err;
	}

	KIONIX_INFO("enter standby mode");
	return 0;
}

static void kionix_accel_report_accel_data(struct kionix_accel_driver *acceld)
{
	struct { union {
		s16 accel_data_s16[3];
		s8	accel_data_s8[6];
	}; } accel_data;
	s16 x, y, z;
	int err;
#ifdef CONFIG_HUAWEI_DSM	
	s16 dsm_x,dsm_y,dsm_z;
#endif
		/*delete it , use short code segment */

	/* Only read the output registers if enabled */
	if(atomic_read(&acceld->accel_enabled) > 0) {
		if(atomic_read(&acceld->accel_enable_resume) > 0)
		{
			err = kionix_i2c_read(acceld, ACCEL_XOUT_L, (u8 *)accel_data.accel_data_s16, 6);
			if (err < 0) {
				KIONIX_ERR( "%s: read xyz data output error = %d", __func__, err);
			}
			else {
				write_lock(&acceld->rwlock_accel_data);
#ifdef CONFIG_HUAWEI_DSM
				dsm_x = ((s16)accel_data.accel_data_s16[0]);
				dsm_y = ((s16)accel_data.accel_data_s16[1]);
				dsm_z = ((s16)accel_data.accel_data_s16[2]);

				//acceld->kx023_dsm_operation.judge_same_value_excep(dsm_x,dsm_y,dsm_z,acceld);
#endif
				x = ((s16) le16_to_cpu(accel_data.accel_data_s16[acceld->axis_map_x])) >> acceld->shift;
				y = ((s16) le16_to_cpu(accel_data.accel_data_s16[acceld->axis_map_y])) >> acceld->shift;
				z = ((s16) le16_to_cpu(accel_data.accel_data_s16[acceld->axis_map_z])) >> acceld->shift;

				acceld->accel_data[acceld->axis_map_x] = (acceld->negate_x ? -x : x) + acceld->accel_cali[acceld->axis_map_x];
				acceld->accel_data[acceld->axis_map_y] = (acceld->negate_y ? -y : y) + acceld->accel_cali[acceld->axis_map_y];
				acceld->accel_data[acceld->axis_map_z] = (acceld->negate_z ? -z : z) + acceld->accel_cali[acceld->axis_map_z];

		/*move this code segment down */
				if(olddata[0] == acceld->accel_data[acceld->axis_map_x]) 
				{
					acceld->accel_data[acceld->axis_map_x]++;
					dataflag |= 0x01;
				}

				if (olddata[1] == acceld->accel_data[acceld->axis_map_y])
				{
					acceld->accel_data[acceld->axis_map_y]++;
					dataflag |= 0x02;
				}
					
				if (olddata[2] == acceld->accel_data[acceld->axis_map_z])
				{
					acceld->accel_data[acceld->axis_map_z]++;
					dataflag |= 0x04;
				}
				input_report_abs(acceld->input_dev, ABS_X, acceld->accel_data[acceld->axis_map_x]);
				input_report_abs(acceld->input_dev, ABS_Y, acceld->accel_data[acceld->axis_map_y]);
				input_report_abs(acceld->input_dev, ABS_Z, acceld->accel_data[acceld->axis_map_z]);
				input_sync(acceld->input_dev);
				olddata[0] = acceld->accel_data[acceld->axis_map_x];
				olddata[1] = acceld->accel_data[acceld->axis_map_y];
				olddata[2] = acceld->accel_data[acceld->axis_map_z];
				if(dataflag & 0x01)
				{
					acceld->accel_data[acceld->axis_map_x]--;
				}

				if(dataflag & 0x02)
				{
					acceld->accel_data[acceld->axis_map_y]--;
				}

				if(dataflag & 0x04)
				{
					acceld->accel_data[acceld->axis_map_z]--;
				}
				dataflag = 0;
				if (sensorDT_mode)
				{
					Gsensor_data_count++;
				}
				write_unlock(&acceld->rwlock_accel_data);
			}
		}
		else
		{
			atomic_inc(&acceld->accel_enable_resume);
		}
	}

#ifdef CONFIG_HUAWEI_DSM
	/* use operations pointer to check exceptions*/
	acceld->kx023_dsm_operation.check_exception(&acceld->accel_data[acceld->axis_map_x],acceld);
#endif


	/*print important regs and xyz value every 10 seconds*/
	if( (kx023_debug_mask > 1) && false == acceld->queued_debug_work_flag)
	{
		queue_delayed_work(acceld->accel_workqueue, &acceld->debug_work, DBG_INTERVAL_1S);
		acceld->queued_debug_work_flag = true;
	}
}

static void kionix_accel_update_g_range(struct kionix_accel_driver *acceld)
{
	acceld->accel_registers[accel_ctrl_reg1] &= ~ACCEL_G_MASK;

	switch (acceld->accel_pdata->accel_g_range) {
		case KIONIX_ACCEL_G_8G:
		case KIONIX_ACCEL_G_6G:
			acceld->shift = 2;
			acceld->accel_registers[accel_ctrl_reg1] |= ACCEL_G_8G;
			break;
		case KIONIX_ACCEL_G_4G:
			acceld->shift = 3;
			acceld->accel_registers[accel_ctrl_reg1] |= ACCEL_G_4G;
			break;
		case KIONIX_ACCEL_G_2G:
		default:
			acceld->shift = 4;
			acceld->accel_registers[accel_ctrl_reg1] |= ACCEL_G_2G;
			break;
	}

	return;
}

/**
*	update output data rate
*/
static int kionix_accel_update_odr(struct kionix_accel_driver *acceld, unsigned int poll_interval)
{
	int err;
	int i;
	u8 odr, buf[2];

	/* Use the lowest ODR that can support the requested poll interval */
	for (i = 0; i < ARRAY_SIZE(kionix_accel_odr_table); i++) {
		odr = kionix_accel_odr_table[i].mask;
		if (poll_interval < kionix_accel_odr_table[i].cutoff)
			break;
	}
	KIONIX_DBG("%s@line:%d: accel_data_ctrl=0x%x,accel_ctrl_reg1=0x%x",\
		__func__, __LINE__,acceld->accel_registers[accel_data_ctrl],acceld->accel_registers[accel_ctrl_reg1]);

	/* Do not need to update DATA_CTRL_REG register if the ODR is not changed */
	if(acceld->accel_registers[accel_data_ctrl] == odr)
		return 0;
	else
		acceld->accel_registers[accel_data_ctrl] = odr;

	/* Do not need to update DATA_CTRL_REG register if the sensor is not currently turn on */
	if(atomic_read(&acceld->accel_enabled) > 0) {
		err = kionix_i2c_write(acceld,ACCEL_CTRL_REG1, 0);
		if (err < 0)
			return err;

		err = kionix_i2c_write(acceld,ACCEL_DATA_CTRL, acceld->accel_registers[accel_data_ctrl]);
		if (err < 0)
			return err;

		err = kionix_i2c_write(acceld,ACCEL_CTRL_REG1, acceld->accel_registers[accel_ctrl_reg1] | ACCEL_PC1_ON);
		if (err < 0)
			return err;

		err = kionix_i2c_read(acceld,ACCEL_DATA_CTRL,buf,1);
		if (err < 0)
			return err;
		switch(buf[0]) {
			case ACCEL_ODR0_781:
				KIONIX_DBG("kx023 ODR = 0.781 Hz");
				break;
			case ACCEL_ODR1_563:
				KIONIX_DBG("kx023 ODR = 1.563 Hz");
				break;
			case ACCEL_ODR3_125:
				KIONIX_DBG("kx023 ODR = 3.125 Hz");
				break;
			case ACCEL_ODR6_25:
				KIONIX_DBG("kx023 ODR = 6.25 Hz");
				break;
			case ACCEL_ODR12_5:
				KIONIX_DBG("kx023 ODR = 12.5 Hz");
				break;
			case ACCEL_ODR25:
				KIONIX_DBG("kx023 ODR = 25 Hz");
				break;
			case ACCEL_ODR50:
				KIONIX_DBG("kx023 ODR = 50 Hz");
				break;
			case ACCEL_ODR100:
				KIONIX_DBG("kx023 ODR = 100 Hz");
				break;
			case ACCEL_ODR200:
				KIONIX_DBG("kx023 ODR = 200 Hz");
				break;
			case ACCEL_ODR400:
				KIONIX_DBG("kx023 ODR = 400 Hz");
				break;
			case ACCEL_ODR800:
				KIONIX_DBG("kx023 ODR = 800 Hz");
				break;
			case ACCEL_ODR1600:
				KIONIX_DBG("kx023 ODR = 1600 Hz");
				break;
			default:
				KIONIX_DBG("kx023 Unknown ODR");
				break;
		}
	}

	return 0;
}


static void kionix_accel_work(struct work_struct *work)
{
	struct kionix_accel_driver *acceld = container_of((struct delayed_work *)work,	struct kionix_accel_driver, accel_work);

	acceld->kionix_accel_report_accel_data(acceld);

	queue_delayed_work(acceld->accel_workqueue, &acceld->accel_work, acceld->poll_delay);

}

static void kionix_accel_update_direction(struct kionix_accel_driver *acceld)
{
	unsigned int direction = acceld->accel_pdata->accel_direction;

	write_lock(&acceld->rwlock_accel_data);
	/*this is for bottom*/
	if(direction == GS_MAP_DIRECTION_BOTTOM)
	{
		acceld->axis_map_x = 0;
		acceld->axis_map_y =  1;
		acceld->axis_map_z =  2;
		acceld->negate_z = 1;
		acceld->negate_x = 0;
		acceld->negate_y = 1;
	}
	else if(direction == GS_MAP_DIRECTION_BOTTOM_LEFT_TOP)
	{
		acceld->axis_map_x = 1;
		acceld->axis_map_y =  0;
		acceld->axis_map_z =  2;
		acceld->negate_z = 1;
		acceld->negate_x = 0;
		acceld->negate_y = 0;
	}
	/*this is for top paster*/
	else
	{
		acceld->axis_map_x = 0;
		acceld->axis_map_y =  1;
		acceld->axis_map_z =  2;
		acceld->negate_z = 0;
		acceld->negate_x = 1;
		acceld->negate_y = 1;
	}

	write_unlock(&acceld->rwlock_accel_data);
	return;
}

static int kionix_accel_enable(struct kionix_accel_driver *acceld)
{
	int err = 0;
	long remaining;

	mutex_lock(&acceld->mutex_earlysuspend);
	KIONIX_INFO("%s: acceld->accel_enabled=%d, acceld->accel_suspended=%d",__func__,  
	                          atomic_read(&acceld->accel_enabled),
	                          atomic_read(&acceld->accel_suspended));
	acceld->queued_debug_work_flag = false;
	/* only enable when it is not enabled*/
	if (!atomic_cmpxchg(&acceld->accel_enabled, 0, 1)){

		atomic_set(&acceld->accel_suspend_continue, 0);

		/* Make sure that the sensor had successfully resumed before enabling it */
		if(atomic_read(&acceld->accel_suspended) == 1) {
			KIONIX_INFO("%s: waiting for resume", __func__);
			remaining = wait_event_interruptible_timeout(acceld->wqh_suspend, \
					atomic_read(&acceld->accel_suspended) == 0, \
					msecs_to_jiffies(KIONIX_ACCEL_EARLYSUSPEND_TIMEOUT));

			if(atomic_read(&acceld->accel_suspended) == 1) {
				KIONIX_ERR("%s: timeout waiting for resume", __func__);
				err = -ETIME;
				goto exit;
			}
		}

		err = kionix_acc_config_regulator(acceld,true);
		if(err)
			goto exit;

		/*when enable the device, enter operate mode*/
		if (!IS_ERR_OR_NULL(acceld->pinctrl)) {
			if (pinctrl_select_state(acceld->pinctrl, acceld->pin_default))
				KIONIX_ERR("Can't select pinctrl default state\n");
		}

		err = acceld->kionix_accel_operate(acceld);
		if (err < 0) {
			KIONIX_ERR(
					"%s: kionix_accel_operate returned err = %d", __func__, err);
			goto exit;
		}

		KIONIX_INFO("kionix_accel enabled");
	}

exit:
	mutex_unlock(&acceld->mutex_earlysuspend);

	return err;
}

static int kionix_accel_disable(struct kionix_accel_driver *acceld)
{
	int err = 0;

	mutex_lock(&acceld->mutex_resume);
	KIONIX_INFO("%s: acceld->accel_enabled:%d, acceld->accel_enable_resume:%d\n",__func__, 
		                  atomic_read(&acceld->accel_enabled),
		                  atomic_read(&acceld->accel_suspended));
	/* only enabled set acceld->accel_enabled = 0*/
	if (atomic_cmpxchg(&acceld->accel_enabled, 1, 0)) {

		atomic_set(&acceld->accel_suspend_continue, 1);
		cancel_delayed_work_sync(&acceld->accel_work);
		cancel_delayed_work_sync(&acceld->debug_work);
#ifdef CONFIG_HUAWEI_DSM
		cancel_work_sync(&acceld->excep_dwork);
#endif

		if(atomic_read(&acceld->accel_enable_resume) > 0)
			atomic_set(&acceld->accel_enable_resume, 0);
		/* when disable the device, enter the standby mode*/
		if (!IS_ERR_OR_NULL(acceld->pinctrl)) {
			if (pinctrl_select_state(acceld->pinctrl, acceld->pin_sleep))
				KIONIX_ERR("Can't select pinctrl sleep state\n");
		}

		err = acceld->kionix_accel_standby(acceld);
		if (err < 0) {
			KIONIX_ERR("%s: kionix_accel_standby returned err = %d", __func__, err);
			goto exit;
		}
		wake_up_interruptible(&acceld->wqh_suspend);

		kionix_acc_config_regulator(acceld,false);

		KIONIX_INFO("kionix_accel disabled");
	}

exit:
	mutex_unlock(&acceld->mutex_resume);

	return err;
}

/**
*	init and setup input device
*/
static int  kionix_accel_setup_input_device(struct kionix_accel_driver *acceld)
{
	struct input_dev *input_dev;
	int err;

	input_dev = input_allocate_device();
	if (!input_dev) {
		KIONIX_ERR("input_allocate_device failed");
		return -ENOMEM;
	}
	mutex_init(&input_dev->mutex);
	acceld->input_dev = input_dev;

	input_set_drvdata(input_dev, acceld);

	__set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_X, -ACCEL_G_MAX, ACCEL_G_MAX, ACCEL_FUZZ, ACCEL_FLAT);
	input_set_abs_params(input_dev, ABS_Y, -ACCEL_G_MAX, ACCEL_G_MAX, ACCEL_FUZZ, ACCEL_FLAT);
	input_set_abs_params(input_dev, ABS_Z, -ACCEL_G_MAX, ACCEL_G_MAX, ACCEL_FUZZ, ACCEL_FLAT);


	input_dev->name = KIONIX_ACCEL_INPUT_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &acceld->client->dev;

	err = input_register_device(acceld->input_dev);
	if (err) {
		KIONIX_ERR("%s: input_register_device returned err = %d", __func__, err);
		input_free_device(acceld->input_dev);
		return err;
	}

	return 0;
}

/**
*	 Returns the enable state of device
*/
static ssize_t attr_get_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
	return snprintf(buf,32,"%d\n", atomic_read(&acceld->accel_enabled) > 0 ? 1 : 0);
}
/**
*	 Allow users to enable/disable the device
*/
static ssize_t attr_set_enable(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
	struct input_dev *input_dev = acceld->input_dev;
	char *buf2;
	const int enable_count = 1;
	unsigned long enable;
	int err = 0;

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);

	if(kionix_strtok(buf, count, &buf2, enable_count) < 0) {
		KIONIX_ERR(
				"%s: No enable data being read. " \
				"No enable data will be updated.", __func__);
	}

	else {
		/* Removes any leading negative sign */
		while(*buf2 == '-')
			buf2++;
		#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35))
		err = kstrtouint((const char *)buf2, 10, (unsigned int *)&enable);
		if (err < 0) {
			KIONIX_ERR(\
					"%s: kstrtouint returned err = %d", __func__, err);
			goto exit;
		}
		#else
		err = strict_strtoul((const char *)buf2, 10, &enable);
		if (err < 0) {
			KIONIX_ERR(\
					"%s: strict_strtoul returned err = %d", __func__, err);
			goto exit;
		}
		#endif

		if(enable)
			err = kionix_accel_enable(acceld);
		else
			err = kionix_accel_disable(acceld);
	}

exit:
	mutex_unlock(&input_dev->mutex);

	return (err < 0) ? err : count;
}

/**
*	 Returns currently selected poll interval (in ms)
*/
static ssize_t attr_get_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);

	return snprintf(buf,32,"%d\n", acceld->poll_interval);
}

/**
*	 Allow users to select a new poll interval (in ms)
*/
static ssize_t attr_set_delay(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
	struct input_dev *input_dev = acceld->input_dev;
	char *buf2;
	const int delay_count = 1;
	unsigned long interval;
	int err = 0;

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);

	if(kionix_strtok(buf, count, &buf2, delay_count) < 0) {
		KIONIX_ERR(\
				"%s: No delay data being read. " \
				"No delay data will be updated.", __func__);
	}

	else {
		/* Removes any leading negative sign */
		while(*buf2 == '-')
			buf2++;
		#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35))
		err = kstrtouint((const char *)buf2, 10, (unsigned int *)&interval);
		if (err < 0) {
			KIONIX_ERR(\
					"%s: kstrtouint returned err = %d", __func__, err);
			goto exit;
		}
		#else
		err = strict_strtoul((const char *)buf2, 10, &interval);
		if (err < 0) {
			KIONIX_ERR("%s: strict_strtoul returned err = %d", __func__, err);
			goto exit;
		}
		#endif

		acceld->poll_interval = max((unsigned int)interval, acceld->accel_pdata->min_interval);
		acceld->poll_delay = msecs_to_jiffies(acceld->poll_interval);

		err = acceld->kionix_accel_update_odr(acceld, acceld->poll_interval);

		KIONIX_INFO("set new acceld->poll_interval=%d",acceld->poll_interval);

	}

exit:
	mutex_unlock(&input_dev->mutex);

	return (err < 0) ? err : count;
}

/**
*	 Returns the direction of device
*/
static ssize_t attr_get_direct(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);

	return snprintf(buf,32, "%d\n", acceld->accel_pdata->accel_direction);
}

/**
*	 Allow users to change the direction the device
*/
static ssize_t attr_set_direct(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
	struct input_dev *input_dev = acceld->input_dev;
	char *buf2;
	const int direct_count = 1;
	unsigned long direction;
	int err = 0;

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);

	if(kionix_strtok(buf, count, &buf2, direct_count) < 0) {
		KIONIX_ERR("%s: No direction data being read. " \
				"No direction data will be updated.", __func__);
	}

	else {
		/* Removes any leading negative sign */
		while(*buf2 == '-')
			buf2++;
		#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35))
		err = kstrtouint((const char *)buf2, 10, (unsigned int *)&direction);
		if (err < 0) {
			KIONIX_ERR("%s: kstrtouint returned err = %d", __func__, err);
			goto exit;
		}
		#else
		err = strict_strtoul((const char *)buf2, 10, &direction);
		if (err < 0) {
			KIONIX_ERR("%s: strict_strtoul returned err = %d", __func__, err);
			goto exit;
		}
		#endif

		if(direction < 1 || direction > 8)
			KIONIX_ERR("%s: invalid direction = %d", __func__, (unsigned int) direction);

		else {
			acceld->accel_pdata->accel_direction = (u8) direction;
			kionix_accel_update_direction(acceld);
		}
	}

exit:
	mutex_unlock(&input_dev->mutex);

	return (err < 0) ? err : count;
}

/**
*	 Returns the data output of device
*/
static ssize_t attr_get_data(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
	int x, y, z;

	read_lock(&acceld->rwlock_accel_data);

	x = acceld->accel_data[acceld->axis_map_x];
	y = acceld->accel_data[acceld->axis_map_y];
	z = acceld->accel_data[acceld->axis_map_z];

	read_unlock(&acceld->rwlock_accel_data);

	return snprintf(buf,128, "%d %d %d\n", x, y, z);
}

/**
*	 Returns the calibration value of the device
*/
static ssize_t attr_get_cali(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
	int calibration[3];

	read_lock(&acceld->rwlock_accel_data);

	calibration[0] = acceld->accel_cali[acceld->axis_map_x];
	calibration[1] = acceld->accel_cali[acceld->axis_map_y];
	calibration[2] = acceld->accel_cali[acceld->axis_map_z];

	read_unlock(&acceld->rwlock_accel_data);

	return snprintf(buf, 128,"%d %d %d\n", calibration[0], calibration[1], calibration[2]);
}

/**
*	 Allow users to change the calibration value of the device
*/
static ssize_t attr_set_cali(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
	struct input_dev *input_dev = acceld->input_dev;
	const int cali_count = 3; /* How many calibration that we expect to get from the string */
	char **buf2;
	long calibration[cali_count];
	int err = 0, i = 0;

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);

	buf2 = (char **)kzalloc(cali_count * sizeof(char *), GFP_KERNEL);

	if(kionix_strtok(buf, count, buf2, cali_count) < 0) {
		KIONIX_ERR(\
				"%s: Not enough calibration data being read. " \
				"No calibration data will be updated.", __func__);
	}
	else {
		/* Convert string to integers  */
		for(i = 0 ; i < cali_count ; i++) {
			#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35))
			err = kstrtoint((const char *)*(buf2+i), 10, (int *)&calibration[i]);
			if(err < 0) {
				KIONIX_ERR(\
						"%s: kstrtoint returned err = %d." \
						"No calibration data will be updated.", __func__ , err);
				goto exit;
			}
			#else
			err = strict_strtol((const char *)*(buf2+i), 10, &calibration[i]);
			if(err < 0) {
				KIONIX_ERR(\
						"%s: strict_strtol returned err = %d." \
						"No calibration data will be updated.", __func__ , err);
				goto exit;
			}
			#endif
		}

		write_lock(&acceld->rwlock_accel_data);

		acceld->accel_cali[acceld->axis_map_x] = (int)calibration[0];
		acceld->accel_cali[acceld->axis_map_y] = (int)calibration[1];
		acceld->accel_cali[acceld->axis_map_z] = (int)calibration[2];

		write_unlock(&acceld->rwlock_accel_data);
	}

exit:
	for(i = 0 ; i < cali_count ; i++)
		kfree(*(buf2+i));

	kfree(buf2);

	mutex_unlock(&input_dev->mutex);

	return (err < 0) ? err : count;
}

/* *
*	Returns  regs value and kionix config we care
*/
static ssize_t attr_show_regs(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned char reg_buf[4];
	int err;
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
	struct kionix_accel_platform_data *pdata = acceld->accel_pdata;

	err = kionix_i2c_read(acceld, ACCEL_INT_REL,reg_buf, 2);
	if(err < 0){
		KIONIX_DBG("failed to read ACCEL_INT_REL regs value");
		return err;
	}

	err = kionix_i2c_read(acceld, ACCEL_DATA_CTRL,&reg_buf[2], 2);
	if(err < 0){
		KIONIX_DBG("failed to read ACCEL_DATA_CTRL regs value");
		return err;
	}
	return snprintf(buf, 512,"ACCEL_INT_REL = 0x%2x,ACCEL_CTRL_REG1 = 0x%2x\n"\
					"ACCEL_DATA_CTRL = 0x%2x,ACCEL_INT_CTRL1 = 0x%2x\n"\
					"gpio_int1=%d,accel_direction=%d,accel_g_range=%d\n"\
					"poll_interval=%d,min_interval=%d,accel_res=%d\n"\
					,reg_buf[0],reg_buf[1],reg_buf[2],reg_buf[3],\
					pdata->gpio_int1,pdata->accel_direction,pdata->accel_g_range,\
					pdata->poll_interval,pdata->min_interval,pdata->accel_res);
}

#ifdef CONFIG_HUAWEI_DSM
static ssize_t attr_get_excep_base(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
	struct gsensor_test_excep *test_exception = &acceld->gsensor_test_exception;

	return snprintf(buf, 200,"excep_base=%d,x_y_z_max=%d,x_y_z_min=%d,single_axis_max=%d\n",
			test_exception->excep_base,test_exception->x_y_z_max,
			test_exception->x_y_z_min,test_exception->single_axis_max);
}

static ssize_t attr_set_excep_base(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
	struct gsensor_test_excep *test_exception = &acceld->gsensor_test_exception;
	unsigned long excep_base;

	if (kstrtoul(buf, 10, &excep_base))
		return -EINVAL;

	/*recalculate and refresh x_y_z_max  x_y_z_min single_axis_max value*/
	test_exception->excep_base = excep_base;
	test_exception->x_y_z_max   = base_to_total_max(test_exception->excep_base);
	test_exception->x_y_z_min   = base_to_total_min(test_exception->excep_base);
	test_exception->single_axis_max	    =  base_to_single_axis_max(test_exception->excep_base);

	return count;
}

#endif

static struct device_attribute attributes[] = {

	__ATTR(poll_delay, 0664, attr_get_delay,attr_set_delay),
	__ATTR(enable, 0664, attr_get_enable, attr_set_enable),
	__ATTR(data, 0444, attr_get_data, NULL),
	__ATTR(cali, 0664, attr_get_cali, attr_set_cali),
	__ATTR(direct, 0664, attr_get_direct, attr_set_direct),
	__ATTR(dump_regs, 0444, attr_show_regs, NULL),
#ifdef CONFIG_HUAWEI_DSM
	__ATTR(excep_base, 0664, attr_get_excep_base,attr_set_excep_base),
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
	KIONIX_ERR( "%s:Unable to create interface", __func__);
	return err;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}

static int kx023_pinctrl_init(struct kionix_accel_driver *acc)
{
	struct i2c_client *client = acc->client;

	acc->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(acc->pinctrl)) {
		KIONIX_ERR("Failed to get pinctrl\n");
		return PTR_ERR(acc->pinctrl);
	}

	acc->pin_default =
		pinctrl_lookup_state(acc->pinctrl, "kx023_default");
	if (IS_ERR_OR_NULL(acc->pin_default)) {
		KIONIX_ERR("Failed to look up default state\n");
		return PTR_ERR(acc->pin_default);
	}

	acc->pin_sleep =
		pinctrl_lookup_state(acc->pinctrl, "kx023_sleep");
	if (IS_ERR_OR_NULL(acc->pin_sleep)) {
		KIONIX_ERR("Failed to look up sleep state\n");
		return PTR_ERR(acc->pin_sleep);
	}

	return 0;
}


/**
*	verify chipid and set report function, and some control functions
*/
static int  kionix_verify_and_set_callback_func(struct kionix_accel_driver *acceld)
{
	int err;
	u8 who_am_i;
	/* read WHO_AM_I register and verify */
	err = kionix_i2c_read(acceld, ACCEL_WHO_AM_I,&who_am_i,1);
	if(err < 0)
	{
		KIONIX_ERR("failed to read who_am_i register. return ");
		return err;
	}

	/* Setup group specific configuration and function callback */
	switch (who_am_i) {
		case KIONIX_ACCEL_WHO_AM_I_KX023:

			/* report data */
			acceld->kionix_accel_report_accel_data	= kionix_accel_report_accel_data;
			/* update output data rate*/
			acceld->kionix_accel_update_odr		= kionix_accel_update_odr;
			/* init and power on*/
			acceld->kionix_accel_power_on_init		= kionix_accel_init_and_power_on;
			/* enter operate mode*/
			acceld->kionix_accel_operate			= kionix_accel_operate;
			/* enter stand-by mode*/
			acceld->kionix_accel_standby			= kionix_accel_standby;
			acceld->i2c_read						= kionix_i2c_read;

			acceld->cdev = kionix_acc_cdev;
			acceld->cdev.sensors_enable 			= kionix_acc_enable_set;
			acceld->cdev.sensors_poll_delay 		= kionix_acc_poll_delay_set;

			/*get poll_interval and set poll_delay*/
			acceld->poll_interval = acceld->accel_pdata->poll_interval;
			acceld->poll_delay = msecs_to_jiffies(acceld->poll_interval);

			atomic_set(&acceld->accel_suspended, 0);
			atomic_set(&acceld->accel_suspend_continue, 1);
			/* don't enable kionix accelerometer In the process of initialization*/
			atomic_set(&acceld->accel_enabled, 0);
			atomic_set(&acceld->accel_input_event, 0);
			atomic_set(&acceld->accel_enable_resume, 0);
			break;
		default:
			KIONIX_ERR("%s: unsupported device, who am i = %d. Abort.", __func__, who_am_i);
			return -ENODEV;
	}

	KIONIX_INFO("this accelerometer is a KX023.");
	return 0;
}

static int kionix_acc_poll_delay_set(struct sensors_classdev *sensors_cdev,
	unsigned int delay_msec)
{
	struct kionix_accel_driver*acc = container_of(sensors_cdev,
		struct kionix_accel_driver, cdev);
	int err;
	unsigned char reg_buf[4];
	err = kionix_i2c_read(acc, ACCEL_INT_REL,reg_buf, 2);
	if(err < 0){
		KIONIX_ERR("failed to read ACCEL_INT_REL regs value");
		return err;
	}

	err = kionix_i2c_read(acc, ACCEL_DATA_CTRL,&reg_buf[2], 2);
	if(err < 0){
		KIONIX_ERR("failed to read ACCEL_DATA_CTRL regs value");
		return err;
	}

	KIONIX_INFO("%s, ACCEL_INT_REL = 0x%2x,ACCEL_CTRL_REG1 = 0x%2x," \
		"ACCEL_DATA_CTRL = 0x%2x,ACCEL_INT_CTRL1 = 0x%2x\n",
		__func__,reg_buf[0],reg_buf[1],reg_buf[2],reg_buf[3]);

	mutex_lock(&acc->lock);

	acc->poll_interval = max(delay_msec,acc->accel_pdata->min_interval);
	KIONIX_INFO("%s acc->poll_interval =%d.",__func__,acc->poll_interval );
	acc->poll_delay = msecs_to_jiffies(acc->poll_interval);

	err = acc->kionix_accel_update_odr(acc, acc->poll_interval);

	mutex_unlock(&acc->lock);
	return err;
}

static int kionix_acc_enable_set(struct sensors_classdev *sensors_cdev,
	unsigned int enable)
{
	struct kionix_accel_driver*acc = container_of(sensors_cdev,
		struct kionix_accel_driver, cdev);
	int err;

	if (enable){
		err = kionix_accel_enable(acc);
		if(err < 0){
			KIONIX_ERR("app enalbe gsensor failed!");
			return err;
		}
		KIONIX_INFO("app enalbe gsensor");
	}
	else{
		err = kionix_accel_disable(acc);
		if(err < 0){
			KIONIX_ERR("app disalbe gsensor failed!");
			return err;
		}
		KIONIX_INFO("app disalbe gsensor");
	}


	return err;
}

/**
*	read parameters from device tree, if failed, use default vaule
*/
static u32 get_of_u32_val(struct device_node *np,
	const char *name,u32 default_val)
{
	u32 tmp= 0;
	int err = 0;
	err = of_property_read_u32(np, name, &tmp);
	if (!err)
		return tmp;
	else {
		KIONIX_ERR("%s:FAILED get the data %s from the dtsi default_val=%d!",__func__,name,default_val);
		return default_val;
	}
}
/**
*	print the default config from dts
*/
static void print_default_config(struct kionix_accel_platform_data *pdata)
{
	KIONIX_INFO("%s,line:%d,gpio_int1=%d",__func__,__LINE__,pdata->gpio_int1);
	KIONIX_INFO("%s,line:%d,accel_direction=%d",__func__,__LINE__,pdata->accel_direction);
	KIONIX_INFO("%s,line:%d,accel_g_range=%d",__func__,__LINE__,pdata->accel_g_range);
	KIONIX_INFO("%s,line:%d,poll_interval=%d",__func__,__LINE__,pdata->poll_interval);
	KIONIX_INFO("%s,line:%d,min_interval=%d",__func__,__LINE__,pdata->min_interval);
	KIONIX_INFO("%s,line:%d,accel_res=%d sucessfully",__func__,__LINE__,pdata->accel_res);

}

static int kionix_parse_dt(struct device *dev,
				struct kionix_accel_driver *acceld)
{

	struct device_node *np = dev->of_node;
	struct kionix_accel_platform_data *pdata= acceld->accel_pdata;
	int rc;
	pdata->accel_direction= get_of_u32_val(np, "gs,direct", DEFAULT_DIRECT);

	/* get default g_range from dts*/
	rc = get_of_u32_val(np, "gs,accel_g_range", DEFAULT_G_RANGE);
	switch (rc) {
		case 0:
			pdata->accel_g_range = KIONIX_ACCEL_G_2G;
			break;
		case 1:
			pdata->accel_g_range = KIONIX_ACCEL_G_4G;
			break;
		case 2:
			pdata->accel_g_range = KIONIX_ACCEL_G_6G;
			break;
		case 3:
			pdata->accel_g_range = KIONIX_ACCEL_G_8G;
			break;
		default:
			pdata->accel_g_range = KIONIX_ACCEL_G_2G;
			break;
		}

	/*get default poll_interval from dts*/
	pdata->poll_interval = get_of_u32_val(np, "gs,poll_interval", DEFAULT_POLL_INTERVAL);

	/*get default min_interval form dts*/
	pdata->min_interval = get_of_u32_val(np, "gs,min_interval", DEFAULT_MIN_INTERVAL);

	/*get default min_interval form dts*/
	rc = get_of_u32_val(np, "gs,accel_res", DEFAULT_ACCEL_RES);
	switch (rc) {
			case 0:
				pdata->accel_res = KIONIX_ACCEL_RES_12BIT;
				acceld->accel_registers[accel_ctrl_reg1] |= ACCEL_RES_16BIT;
				break;
			case 1:
				pdata->accel_res = KIONIX_ACCEL_RES_8BIT;
				acceld->accel_registers[accel_ctrl_reg1] |= ACCEL_RES_8BIT;
				break;
			case 2:
				pdata->accel_res = KIONIX_ACCEL_RES_6BIT;
				acceld->accel_registers[accel_ctrl_reg1] |= ACCEL_RES_8BIT;
				break;
			case 3:
				pdata->accel_res = KIONIX_ACCEL_RES_16BIT;
				acceld->accel_registers[accel_ctrl_reg1] |= ACCEL_RES_16BIT;
				break;
			default:
				pdata->accel_res = KIONIX_ACCEL_RES_12BIT;
				acceld->accel_registers[accel_ctrl_reg1] |= ACCEL_RES_16BIT;
				break;
	}

	/*get default interrupt gpio form dts*/
	pdata->gpio_int1 = of_get_named_gpio_flags(dev->of_node,"gs,int1_gpio", 0, NULL);
	KIONIX_ERR("%s: default gpio value = %d\n",__func__, pdata->gpio_int1);
	if(pdata->gpio_int1 < 0) {
		KIONIX_ERR("%s: gpio int1 is not available\n",__func__);
	}
	print_default_config(pdata);

	acceld->accel_pdata->i2c_scl_gpio = of_get_named_gpio_flags(np, "gs,i2c-scl-gpio", 0, NULL);
	if (!gpio_is_valid(acceld->accel_pdata->i2c_scl_gpio)) {
		KIONIX_ERR("%s: i2c scl gpio %d is not configered\n", __func__, acceld->accel_pdata->i2c_scl_gpio);
		return -EINVAL;
	}

	acceld->accel_pdata->i2c_sda_gpio = of_get_named_gpio_flags(np, "gs,i2c-sda-gpio", 0, NULL);
	if (!gpio_is_valid(acceld->accel_pdata->i2c_sda_gpio)) {
		KIONIX_ERR("%s: i2c sda gpio %d is not configered\n", __func__, acceld->accel_pdata->i2c_sda_gpio);
		return -EINVAL;
	}

	return 0;
}

/**
*	create and init delay work queue "kionix_accel_work" to report data
*/
static int  kionix_accel_init_workfunc(struct kionix_accel_driver *acceld)
{

	acceld->accel_workqueue = create_workqueue("Kionix Accel Workqueue");
	if (!acceld->accel_workqueue )
	{
		KIONIX_ERR("%s: Unable to create accel_workqueue",__func__);
			return -ENOMEM;
	}

	INIT_DELAYED_WORK(&acceld->accel_work, kionix_accel_work);
	init_waitqueue_head(&acceld->wqh_suspend);

	return 0;
}


static int  kionix_accel_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct kionix_accel_driver *acceld;
	int err=0;

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA)) {
		KIONIX_ERR("%s:line%d:client is not i2c capable. Abort.",__func__,__LINE__);
		err = -EINVAL;
		goto err_probe_start;
	}

	acceld = kzalloc(sizeof(*acceld), GFP_KERNEL);
	if (acceld == NULL) {
		KIONIX_ERR("%s:line%d:failed to allocate memory for module data. Abort.",__func__,__LINE__);
		err = -ENOMEM;
		goto  err_probe_start;
	}

	mutex_init(&acceld->lock);
	mutex_init(&acceld->lock_i2c);
	mutex_init(&acceld->mutex_earlysuspend);
	mutex_init(&acceld->mutex_resume);
	rwlock_init(&acceld->rwlock_accel_data);

	mutex_lock(&acceld->lock);

	acceld->client = client;
	acceld->device_exist = false;

	i2c_set_clientdata(client, acceld);

	acceld->accel_pdata = kzalloc(sizeof(*acceld->accel_pdata), GFP_KERNEL);
	if (acceld->accel_pdata == NULL) {
		err = -ENOMEM;
		KIONIX_ERR("%s:line%d:failed to allocate memory for pdata: %d",__func__,__LINE__,err);
		goto err_kfree_acc;
	}

	acceld->accel_registers = kzalloc(sizeof(u8)*accel_regs_count, GFP_KERNEL);
	if (acceld->accel_registers == NULL) {
		err =  -ENOMEM;
		KIONIX_ERR("failed to allocate memory for accel_registers. Abort.");
		goto err_kfree_pdata;
	}


	if (client->dev.of_node) {
		err = kionix_parse_dt(&client->dev, acceld);
		if (err) {
			err = -EINVAL;
			KIONIX_ERR("%s:line%d:Failed to parse device tree",__func__,__LINE__);
			goto err_kfree_acc_regs;
		}
	}else {
			KIONIX_ERR( "%s:line%d:No valid platform data. exiting",__func__,__LINE__);
			err = -ENODEV;
			goto err_kfree_acc_regs;
	}

	memcpy(acceld->kionix_acc_vreg,kionix_acc_vreg,2*sizeof(struct sensor_regulator));

	err = kionix_acc_config_regulator(acceld,true);
	if (err < 0) {
		goto err_kfree_acc_regs;
	}
#ifdef CONFIG_HUAWEI_DSM
	err = register_kx023_dsm_operations(acceld);
	if (err < 0) {
		goto err_config_regulator;
	}
#endif


	/*verify who_am_i and set a series of callback functions*/
	err = kionix_verify_and_set_callback_func(acceld);
	if (err < 0) {
		goto err_dsm_exit;
	}

	/* initialize pinctrl */
	err = kx023_pinctrl_init(acceld);
	if (err) {
		KIONIX_ERR("Can't initialize pinctrl, use poll mode\n");
		//goto err_dsm_exit;
		acceld->pinctrl = NULL;
	} else {
		KIONIX_ERR("%s: Initialize pinctrl success, use interrupt mode\n", __func__);
	}

	if (!IS_ERR_OR_NULL(acceld->pinctrl)) {
		err = pinctrl_select_state(acceld->pinctrl, acceld->pin_default);
		if (err) {
			KIONIX_ERR("Can't select pinctrl default state\n");
			goto err_dsm_exit;
		}
	} else {
		KIONIX_ERR("%s: pinctrl is null", __func__);
	}
	/*init queue_delayed_work func to report accel data events*/
	err = kionix_accel_init_workfunc(acceld);
	if(err < 0)
		goto err_destory_queue;

	/* alloc and setup input devices */
	err = kionix_accel_setup_input_device(acceld);
	if (err)
		goto err_destory_queue;

	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		KIONIX_ERR("device KIONIX_ACC_DEV_NAME sysfs register failed err=%d",err);
		goto err_free_input;
	}

	err = sensors_classdev_register(&acceld->input_dev->dev, &acceld->cdev);
	if (err) {
		KIONIX_ERR("sensors_classdev_register failed: %d", err);
		goto err_remove_sysfs_int;
	}

	/* init the registers of accelerometer, don't enable in the initialization process */
	err = acceld->kionix_accel_power_on_init(acceld);
	if (err) {
		KIONIX_ERR("%s: kionix_accel_power_on_init returned err = %d. Abort.", __func__, err);
		goto err_free_sensor_class;
	}

	set_sensors_list(G_SENSOR);
	err = app_info_set("G-Sensor", "ROHM KX023");
	if (err < 0)/*failed to add app_info*/
	{
		KIONIX_ERR("%s %d:failed to add app_info\n", __func__, __LINE__);
	}
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
	set_hw_dev_flag(DEV_I2C_G_SENSOR);
#endif
	err = set_sensor_input(ACC, acceld->input_dev->dev.kobj.name);
	if (err) {
		KIONIX_ERR("%s set_sensor_input failed", __func__);
		goto err_free_sensor_class;
	}
	if (!IS_ERR_OR_NULL(acceld->pinctrl)) {
		if (pinctrl_select_state(acceld->pinctrl, acceld->pin_sleep))
			KIONIX_ERR("Can't select pinctrl sleep state\n");
	}

	mutex_unlock(&acceld ->lock);

	KIONIX_INFO("kionix sensor kx023 probe success,client->addr = 0x%x",client->addr);
	acceld->device_exist = true;
	return 0;

err_free_sensor_class:
	sensors_classdev_unregister(&acceld->cdev);
err_remove_sysfs_int:
	remove_sysfs_interfaces(&client->dev);
err_free_input:
	input_unregister_device(acceld->input_dev);
err_destory_queue:
	destroy_workqueue(acceld->accel_workqueue);
#ifdef CONFIG_HUAWEI_DSM
err_dsm_exit:
unregister_kx023_dsm_operations(acceld);
err_config_regulator:
#else
err_dsm_exit:
#endif
	kionix_acc_config_regulator(acceld,false);
err_kfree_acc_regs:
	kfree(acceld->accel_registers );
err_kfree_pdata:
	kfree(acceld->accel_pdata);
err_kfree_acc:
	mutex_unlock(&acceld->lock);
	kfree(acceld);
err_probe_start:
	return err;
}

static int  kionix_accel_remove(struct i2c_client *client)
{
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);

	if (gpio_is_valid(acceld->accel_pdata->gpio_int1)) {
		gpio_free(acceld->accel_pdata->gpio_int1);
	}

	sensors_classdev_unregister(&acceld->cdev);
	remove_sysfs_interfaces(&client->dev);
	input_unregister_device(acceld->input_dev);
	destroy_workqueue(acceld->accel_workqueue);
#ifdef CONFIG_HUAWEI_DSM
	unregister_kx023_dsm_operations(acceld);
#endif

	kionix_acc_config_regulator(acceld,false);

	kfree(acceld->accel_registers);
	if (acceld->accel_pdata->exit)
		acceld->accel_pdata->exit();
	kfree(acceld->accel_pdata);
	kfree(acceld);

	return 0;
}

static int kionix_acc_resume(struct i2c_client *client)
{
	int ret = 0;
	struct kionix_accel_driver*acc = i2c_get_clientdata(client);

	if (acc->on_before_suspend){
		/* add necessary log */
		ret = kionix_accel_enable(acc);
		if( ret < 0){
			KIONIX_ERR("resume enable acceld failed.");
		}else{
			KIONIX_INFO("resume enable acceld.");
		}
	}
	return ret;
}

static int kionix_acc_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct kionix_accel_driver *acc = i2c_get_clientdata(client);

	acc->on_before_suspend = atomic_read(&acc->accel_enabled);
	/* add necessary log */
	ret =  kionix_accel_disable(acc);
	if(ret < 0){
		KIONIX_ERR("suspend disable acceld failed.");
	}else{
		KIONIX_INFO("resume enable acceld.");
	}

	return ret;

}

static const struct i2c_device_id kionix_accel_id[] = {
	{ KIONIX_ACCEL_NAME, 0 },
	{ },
};
static struct of_device_id kionix_acc_match_table[] = {
	{ .compatible = "kionix,kx023", },
	{ },};
MODULE_DEVICE_TABLE(i2c, kionix_accel_id);

static struct i2c_driver kionix_accel_driver = {
	.driver = {
		.name	= KIONIX_ACCEL_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = kionix_acc_match_table,
	},
	.probe = kionix_accel_probe,
	.remove = kionix_accel_remove,
	.suspend = kionix_acc_suspend,
	.resume = kionix_acc_resume,
	.id_table	= kionix_accel_id,

};

static int __init kionix_accel_init(void)
{
	return i2c_add_driver(&kionix_accel_driver);
}

static void __exit kionix_accel_exit(void)
{
	i2c_del_driver(&kionix_accel_driver);
}

module_init(kionix_accel_init);
module_exit(kionix_accel_exit);

MODULE_DESCRIPTION("Kionix accelerometer driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("3.3.0");


