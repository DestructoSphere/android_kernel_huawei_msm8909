/**
*\mainpage
* ADUX1050 Generic Controller Driver
\n
* @copyright 2014 Analog Devices Inc.
\n
* Licensed under the GPL version 2 or later.
* \date      November-2014
* \version   Driver 1.1
* \version   Linux 3.4.0 and above
* \version   Android 4.4.2 [KK]
*/

/**
* \file adux1050.c
* This file is the core driver part of ADUX1050 Capacitive sensor.
* It also has routines for interrupt handling,
* suspend, resume, initialization routines etc.
<br>
* ADUX1050 Generic Controller Driver
<br>
* Copyright 2014 Analog Devices Inc.
<br>
* Licensed under the GPL version 2 or later.
*/

#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/wakelock.h>
#include <linux/wait.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#endif
#ifdef CONFIG_ADUX1050_POLL
#include <linux/kthread.h>
#endif
#include <huawei_platform/sensor/adux1050.h>
#include <huawei_platform/sensor/hw_sensor_info.h>
#ifdef	CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif
#include <misc/app_info.h>

/**
 This is the path for the DAC calibration data file storage
 The File name prefix and the location of the file can be changed
 as per the platform used.
*/
#define CAL_DATA_FILE_PATH	"/data/misc/adux1050_cal_data"
#define CONFIG_DATA_FILE_PATH	"/data/misc/ADUX1050EvalBoard.txt"
#define CONFIG_ADUX1050_DEBUG 1
#define MIN_VDD 0
#define MAX_VDD 2700000

static bool adux1050_flag;

/**
 Local platform data used if no platform data is found in board file or
 Device tree.
*/
static struct adux1050_platform_data local_plat_data = {
	.init_regs = {
		0x00010103,	0x00028530,	0x000301EE,	0x00050F51,
		0x00061902,	0x00078200,	0x00080200,	0x00090000,
		0x000a000C,	0x00798000,
		0x000b9999,	0x000c03e8,	0x000d0200,	0x000e0000,
		0x000f000C,	0x007a8000,
		0x00109999,	0x001105e8,	0x00120200,	0x00130000,
		0x0014000C,	0x007b8000,
		0x00159999,	0x001607e8,	0x00170200,	0x00180000,
		0x0019000C,	0x007c8000,
	},
	.req_stg0_base = 32000,
	.req_stg1_base = 20000,
	.req_stg2_base = 30000,
	.req_stg3_base = 40000,
};

static struct sensors_classdev sar_cdev = {
	.name = "adux1050",
	.vendor = "ADI",
	.version = 1,
	.handle = SENSORS_PROX_HANDLE,
	.type = SENSOR_TYPE_CAP_PROX,
	.max_range = "3",
	.resolution = "1",
	.sensor_power = "0.002",
	.min_delay = 5000,
	.delay_msec = 200,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

/**
* \fn int adux1050_i2c_write(struct device *dev, u8 reg,u16 *data, u16 data_cnt)
* Writes to the device register through I2C interface.
* Used to write the data to the I2C client's Register through the i2c protocol
* Used i2c_tranfer api's for the bus transfer
@param dev The i2c client's device structure
@param reg The register address to be written
@param data The data buffer which holds the data to be written to the device
@param data_cnt The number of data to be written to the device from buffer.
@return Number of messages transfered, default 2

@see adux1050_i2c_read
*/
static int adux1050_i2c_write(struct device *dev, u8 reg,
		u16 *data, u16 data_cnt)
{

	struct i2c_client *client = to_i2c_client(dev);
	u8 device_addr = client->addr;
	u16 loop_cnt = 0;
	u8 tx[(MAX_ADUX1050_WR_LEN*sizeof(short)) + 1] = {0};
	u16 *head;
	s32 ret = -EIO;
	struct i2c_msg adux1050_wr_msg = {
		.addr = device_addr,
		.buf = (u8 *)tx,
		.len = ((data_cnt*sizeof(data_cnt))+1),
		.flags = 0,
	};
	if (!data)
		return -EINVAL;
	tx[0] = reg;
	head = (unsigned short *)&tx[1];
	for (loop_cnt = 0; loop_cnt < data_cnt; loop_cnt++)
		*(head++) = cpu_to_be16(*(data++));

	for (loop_cnt = 0; loop_cnt < I2C_RETRY_CNT; loop_cnt++) {
		ret = i2c_transfer(client->adapter, &adux1050_wr_msg, 1);
		if (unlikely(ret < 1))
			ADUX1050_ERR("I2C write error %d\n", ret);
		else
			break;
	}
	return ret;
}

/**
* \fn int adux1050_i2c_read(struct device *dev, u8 reg,u16 *data, u16 data_cnt)
* This is used to read the data from the ADUX1050's register through
I2C interface
* This function uses i2c protocol and its api's to read data from register
@param dev The i2c client device Structure.
@param reg The register address to be read.
@param data The buffer's pointer to store the register's value.
@param data_cnt The number of registers to be read.
@return The number of messages transfered as an integer

@see adux1050_i2c_write
*/
static int adux1050_i2c_read(struct device *dev, u8 reg,
			     u16 *data, u16 data_cnt)
{
	struct i2c_client *client = to_i2c_client(dev);
	u16 loop_cnt = 0;
	u16 rx[MAX_ADUX1050_WR_LEN] = {};
	s8  device_addr = client->addr;
	s32 ret = 0;
	struct i2c_msg adux1050_rd_msg[I2C_WRMSG_LEN] = {
			{
				.addr = device_addr,
				.buf = (u8 *)&reg,
				.len = sizeof(reg),
				.flags = 0,
			},
			{
				.addr = device_addr,
				.buf = (u8 *)rx,
				.len = data_cnt * sizeof(short),
				.flags = I2C_M_RD,
			}
	};
	for (loop_cnt = 0; loop_cnt < I2C_RETRY_CNT; loop_cnt++) {
		ret = i2c_transfer(client->adapter, adux1050_rd_msg,
				   I2C_WRMSG_LEN);
		if (unlikely(ret < I2C_WRMSG_LEN)) {
			ADUX1050_ERR("[ADUX1050]: I2C READ error %d\n", ret);
			if (loop_cnt >= (I2C_RETRY_CNT - 1))
				memset(data, 0, data_cnt * sizeof(short));
		} else {
			for (loop_cnt = 0; loop_cnt < data_cnt; loop_cnt++)
				data[loop_cnt] = be16_to_cpu(rx[loop_cnt]);
			break;
		}
	}
	return ret;
}

/**
* \fn inline u16 set_dac_offset(s16 new_offset_value)
* Function to set the DAC positive and negative offset based on given offset
@param new_offset_value value to be set as the offset.
@return Combined +ve and -ve value to be set to the DAC_OFFSET_STGx register
*/
static inline u16 set_dac_offset(s16 new_offset_value)
{
	u16 offset_val = 0;
	if (new_offset_value >= 0)
		offset_val = ST_POS_DAC_OFFSET(new_offset_value);
	else
		offset_val = ST_NEG_DAC_OFFSET(new_offset_value);

	return offset_val;
}

/**
* \fn static inline u16 set_swap_state(struct adux1050_chip *adux1050, u16 stg_num, u16 *swap_state, s16 curr_val)
* Function to set the swap bits based on the calibration DAC offset
@param adux1050 chip structure of ADUX1050 driver.
@param stg_num The stage to which swap is to be done for DAC offset value.
@param swap_state The swap state as set in control register
	[ctrl reg holds the swap state]
@param curr_val Current value of DAC_offest for the stage provided [stg_num]
@return Zero on success.
*/
static inline s16 set_swap_state(struct adux1050_chip *adux1050, u16 stg_num,
		   u16 *swap_state, s16 curr_val) {
	s16 err = 0;

	if (!swap_state)
		return -EINVAL;
	if (curr_val > (MAX_OFFSET/2)) {
		*swap_state = ((*swap_state & CLR_POS_SWAP) | SET_NEG_SWAP);
		err = adux1050->write(adux1050->dev, GET_CONFIG_REG(stg_num),
				swap_state, DEF_WR);
		if (err < DEF_WR) {
			ADUX1050_ERR("%s: write GET_CONFIG_REG failed, line = %d, ret = %d\n",
					__func__, __LINE__, err);
			return err;
		}

		ADUX1050_INFO("%s - Swap set %x", __func__, *swap_state);
	} else if (curr_val < -(MAX_OFFSET/2)) {
		*swap_state = ((*swap_state & CLR_NEG_SWAP) | SET_POS_SWAP);
		err = adux1050->write(adux1050->dev, GET_CONFIG_REG(stg_num),
				      swap_state, DEF_WR);
		if (err < DEF_WR) {
			ADUX1050_ERR("%s: write GET_CONFIG_REG failed, line = %d, ret = %d\n",
					__func__, __LINE__, err);
			return err;
		}

		ADUX1050_INFO("%s - Swap set %x",
			__func__, *swap_state);
	} else {
		ADUX1050_INFO("%s - No Swap to be set", __func__);
		*swap_state = ((*swap_state & CLR_NEG_SWAP) & CLR_POS_SWAP);
		err = adux1050->write(adux1050->dev, GET_CONFIG_REG(stg_num),
				swap_state, DEF_WR);
		if (err < DEF_WR) {
			ADUX1050_ERR("%s: write GET_CONFIG_REG failed, line = %d, ret = %d\n",
					__func__, __LINE__, err);
			return err;
		}
	}
	return err;
}

/**
* \fn void adux1050_force_cal(struct adux1050_chip *adux1050,int cal_time)
* Internal function to perform force calibration of the Chip.
@param adux1050 The chip structure of adux1050 driver
@param cal_time Sleep time required after the force calibration.
@return void
*/
static inline void adux1050_force_cal(struct adux1050_chip *adux1050,
				      int cal_time)
{
	u16 data = 0;
	s16 ret;

	ret = adux1050->read(adux1050->dev, BASELINE_CTRL_REG, &data, DEF_WR);
	if (ret < DEF_WR) {
		ADUX1050_ERR("%s: read BASELINE_CTRL_REG failed, ret = %d\n",
				__func__, ret);
		return;
	}

	data = data | FORCE_CAL_MASK;
	ret = adux1050->write(adux1050->dev, BASELINE_CTRL_REG, &data, DEF_WR);
	if (ret < DEF_WR) {
		ADUX1050_ERR("%s: write BASELINE_CTRL_REG failed, ret = %d\n",
				__func__, ret);
		return;
	}
	if (cal_time != 0)
		msleep(cal_time);
}

/**
* \fn inline s16 get_conv_time(struct adux1050_chip *adux1050, int mul_flag)
* To get the required conversion time for the current seting
@adux1050 Chip structure.
@mul_flag Multiplier flag
@return Returns the conversion time required for an updated configurtion
*/
static inline s16 get_conv_time(struct adux1050_chip *adux1050, int mul_flag)
{
	u16 pwr_ctrl_reg = 0;
	u16 cv_time_ctrl = 0;
	u16 pwr_mode = 0;
	u16 stg_num = 0;
	u16 delay_in_ctoc = 0;
	u16 avg = 0;
	u16 osr = 0;
	u16 phase = 0;
	u16 base_time = 0;
	u16 temp_base_time = 0;
	u16 stg_cfg = 0;
	u16 lp_cnt = 0;
	s16 ret;

	ret = adux1050->read(adux1050->dev, CTRL_REG, &pwr_ctrl_reg, DEF_WR);
	if (ret < DEF_WR) {
		ADUX1050_ERR("%s: read CTRL_REG failed, ret = %d\n", __func__, ret);
		return ret;
	}
	ret = adux1050->read(adux1050->dev, CONV_TIME_CTRL_REG, &cv_time_ctrl,
			DEF_WR);
	if (ret < DEF_WR) {
		ADUX1050_ERR("%s: read CONV_TIME_CTRL_REG failed, ret = %d\n", __func__, ret);
		return ret;
	}
	pwr_mode = GET_PWR_MODE(pwr_ctrl_reg);
	stg_num = GET_NUM_STG(pwr_ctrl_reg);
	adux1050->tot_stg = stg_num;

	if (pwr_mode == PWR_STAND_BY) {
		return 1;
	} else if (pwr_mode == PWR_FULL_POWER) {
		delay_in_ctoc = ZERO_VAL;
	} else if (pwr_mode == PWR_TIMED_CONV) {
		delay_in_ctoc = GET_TIMED_CONV_TIME(pwr_ctrl_reg);
	} else if (pwr_mode == PWR_AUTO_WAKE) {
		delay_in_ctoc = GET_TIMED_CONV_TIME(pwr_ctrl_reg);
		if (delay_in_ctoc < GET_AUTO_WAKE_TIME(pwr_ctrl_reg))
			delay_in_ctoc = GET_AUTO_WAKE_TIME(pwr_ctrl_reg);
	}
	avg = GET_AVG_CONV(cv_time_ctrl);
	avg = CALC_AVG_CONV(avg); /*AVG based multiplication factor*/
	osr = GET_OSR_CONV(cv_time_ctrl);
	osr = CALC_OSR_CONV(osr); /*OSR multipling factor*/
	phase = GET_CONV_TIME(cv_time_ctrl); /*Phase timing factor*/
	/* Calculate base time as a factor of phase */
	temp_base_time = ((phase + (phase / DECIMAL_BASE)) - 3) / 2;
	temp_base_time *= (avg * osr); /*Set the base_time*/
	for (; lp_cnt < stg_num; lp_cnt++) {
		ret = adux1050->read(adux1050->dev, GET_CONFIG_REG(lp_cnt),
				&stg_cfg, DEF_WR);
		if (ret < DEF_WR) {
			ADUX1050_ERR("%s: read GET_CONFIG_REG(lp_cnt) failed, ret = %d\n",
					__func__, ret);
			return ret;
		}
		if (IS_NOISE_MEASURE_EN(stg_cfg))
			base_time += (temp_base_time *
				      GET_NOISE_SAMPLE(cv_time_ctrl));
		else
			base_time += temp_base_time;
	}
	if (mul_flag == TWICE_CONV_DELAY_TIME) {
		base_time += base_time; /*Return twice the conv time*/
		base_time += delay_in_ctoc; /*Add the timed conv delay*/
	} else if (mul_flag == CONV_DELAY_TIME) {
		base_time += delay_in_ctoc;
	}
	ADUX1050_INFO("%s, Basetime(%d), delay_in_ctoc (%d)\n",
		 __func__, base_time, delay_in_ctoc);

	return base_time;
}

/**
* \fn int getstageinfo(struct adux1050_chip *adux1050)
* This function is used to get the current stage information.
@param adux1050 The chip structure of ADUX1050 driver
@return 0 on success.
*/
static int getstageinfo(struct adux1050_chip *adux1050)
{
	u16 temp_reg_val = 0;
	s16 ret;
	u8 stg_cnt = 0;
	u8 cin_cnt = 0;
	u8 temp_cin;
	adux1050->conn_stg_cnt = 0;

	/* Checking whether Conversion complete interrupt is enabled or not */
	ret = adux1050->read(adux1050->dev, INT_CTRL_REG, &temp_reg_val, DEF_WR);
	if (ret < DEF_WR) {
		ADUX1050_ERR("%s: read INT_CTRL_REG failed, ret = %d\n", __func__, ret);
		return ret;
	}
	adux1050->conv_enable = CHECK_CONV_EN(temp_reg_val);
	ADUX1050_INFO("%s - Checking conv_enable %d temp_reg_val %x\n",
			    __func__, adux1050->conv_enable, temp_reg_val);
	/* Checking whether High threshold interrupt is enabled or not */
	adux1050->high_thresh_enable = CHECK_THRESH_HIGH_EN(temp_reg_val);

	/* Checking whether Low threshold interrupt is enabled or not */
	adux1050->low_thresh_enable = CHECK_THRESH_LOW_EN(temp_reg_val);

	/* How many stages to measure CDC */
	ret = adux1050->read(adux1050->dev, CTRL_REG, &temp_reg_val, DEF_WR);
	if (ret < DEF_WR) {
		ADUX1050_ERR("%s: read CTRL_REG failed, ret = %d\n", __func__, ret);
		return ret;
	}
	adux1050->num_stages = GET_NUM_STG(temp_reg_val);
	/* Find whether stage is connected(either +ve or -ve) or not */
	for (stg_cnt = 0 ; stg_cnt < TOTAL_STG ; stg_cnt++) {
		if (stg_cnt < adux1050->num_stages) {
			ret = adux1050->read(adux1050->dev, GET_CONFIG_REG(stg_cnt),
				       &temp_reg_val, DEF_WR);
			if (ret < DEF_WR) {
				ADUX1050_ERR("%s: read GET_CONFIG_REG(stg_cnt) failed, ret = %d\n",
						__func__, ret);
				return ret;
			}
			adux1050->stg_info[stg_cnt].status = CIN_NOT_CONNECTED;
			for (cin_cnt = 0; cin_cnt < TOTAL_CIN ; cin_cnt++) {
				temp_cin = (temp_reg_val) & 3;
				if ((temp_cin == CIN_NEG_INPUT) ||
				    (temp_cin == CIN_POS_INPUT)) {
					adux1050->stg_info[stg_cnt].status =
							CIN_CONNECTED;
					adux1050->conn_stg_cnt++;
					ADUX1050_INFO("STG CONNECTED %d,tot=%d\n", stg_cnt, adux1050->conn_stg_cnt);
					break;
				}
				temp_reg_val = temp_reg_val >> 2;
			}
		} else {
			adux1050->stg_info[stg_cnt].status = CIN_NOT_CONNECTED;
			ADUX1050_WARN("%s - CDC not configured for STG_%d\n",
				__func__, stg_cnt);
		}
	}
	return 0;
}

/**
* \fn void update_calib_settings(struct adux1050_chip *adux1050, u16 total_stg,
				u16 *data, bool write_to_reg, u8 file_exist)
* This function updates the calibration output to local register array and
	registers of ADUX1050.
@param adux1050 The chip structure of ADUX1050 driver
@param total_stg Number of stages for which calibration settings to be updated
@param *data Pointer to buffer which contains the calib output
@param write_to_reg A Flag to specify whether to write to registers or not
@param file_exist A Flag to update the calib status
@return 0 on success.
*/
inline void update_calib_settings(struct adux1050_chip *adux1050, u16 total_stg,
				u16 *data, bool write_to_reg, u8 file_exist)
{
	u16 temp_baseline_ctrl = 0;
	u16 int_ctrl_reg = 0;
	u16 stg_cnt = 0;
	u16 stg_num = 0;
	u8 fp = 0;
	u16 cal_base_fail_flag = 0;
	u16 value = 0;
	u16 hys_reg = 0;
	s16 ret;


	/* Disable the interrupt	*/
	ret = adux1050->read(adux1050->dev, INT_CTRL_REG, &int_ctrl_reg, DEF_WR);
	if (ret < DEF_WR) {
		ADUX1050_ERR("%s: read INT_CTRL_REG failed, ret = %d\n",
				__func__, ret);
		return;
	}
	value = int_ctrl_reg | DISABLE_DEV_INT;
	ret = adux1050->write(adux1050->dev, INT_CTRL_REG, &value, DEF_WR);
	if (ret < DEF_WR) {
		ADUX1050_ERR("%s: write INT_CTRL_REG failed, line = %d, ret = %d\n",
				__func__, __LINE__, ret);
		return;
	}

	/* Disabling the Auto threhold & Force calib to
	update baseline registers */
	ret = adux1050->read(adux1050->dev, BASELINE_CTRL_REG,
		       &temp_baseline_ctrl, DEF_WR);
	if (ret < DEF_WR) {
		ADUX1050_ERR("%s: read BASELINE_CTRL_REG failed, ret = %d\n",
				__func__, ret);
		return;
	}
	temp_baseline_ctrl = temp_baseline_ctrl & ANTI_FORCE_CAL_MASK;
	if (temp_baseline_ctrl & AUTO_TH_MASK) {
		value = temp_baseline_ctrl;
		value = value & (~AUTO_TH_MASK);
		ret = adux1050->write(adux1050->dev, BASELINE_CTRL_REG,
				&value, DEF_WR);
		if (ret < DEF_WR) {
			ADUX1050_ERR("%s: write BASELINE_CTRL_REG failed, line = %d, ret = %d\n",
					__func__, __LINE__, ret);
			return;
		}
	}
	msleep(adux1050->slp_time_conv_complete);

	for (stg_cnt = 0; stg_cnt < total_stg; stg_cnt++) {
		stg_num = data[fp++];
		if ((stg_num >= STG_ZERO) && (stg_num <= STG_THREE)) {

			adux1050->pdata->cal_fact_base[stg_num] = data[fp++];
			adux1050->pdata->cal_offset[stg_num] = data[fp++];
			adux1050->pdata->digi_offset[stg_num] = data[fp++];
			adux1050->pdata->stg_cfg[stg_num] = data[fp++];

			adux1050->bs_reg[stg_num].wr_flag = ADUX1050_ENABLE;
			adux1050->bs_reg[stg_num].value =
				adux1050->pdata->cal_fact_base[stg_num];

			adux1050->reg[GET_OFFSET_REG(stg_num)].wr_flag =
							ADUX1050_ENABLE;
			adux1050->reg[GET_OFFSET_REG(stg_num)].value =
				adux1050->pdata->cal_offset[stg_num];

			adux1050->reg[GET_HYS_REG(stg_num)].wr_flag =
							ADUX1050_ENABLE;
			hys_reg =
			((adux1050->reg[GET_HYS_REG(stg_num)].value) &
			 HYS_BYTE_MASK) |
			 (adux1050->pdata->digi_offset[stg_num] << 8);
			adux1050->reg[GET_HYS_REG(stg_num)].value = hys_reg;

			adux1050->reg[GET_CONFIG_REG(stg_num)].wr_flag =
							ADUX1050_ENABLE;
			adux1050->reg[GET_CONFIG_REG(stg_num)].value =
				adux1050->pdata->stg_cfg[stg_num];
			cal_base_fail_flag |=
				adux1050->pdata->cal_fact_base[stg_num];

			if (write_to_reg == ADUX1050_ENABLE) {
				ret = adux1050->write(
				adux1050->dev, GET_BASE_LINE_REG(stg_cnt),
				&adux1050->pdata->cal_fact_base[stg_cnt],
				DEF_WR);
				if (ret < DEF_WR) {
					ADUX1050_ERR("%s: write GET_BASE_LINE_REG failed, ret = %d\n",
							__func__, ret);
					return;
				}

				ret = adux1050->write(
				adux1050->dev, GET_OFFSET_REG(stg_cnt),
				&adux1050->pdata->cal_offset[stg_cnt], DEF_WR);
				if (ret < DEF_WR) {
					ADUX1050_ERR("%s: write GET_OFFSET_REG(stg_cnt) failed, ret = %d\n",
							__func__, ret);
					return;
				}

				ret = adux1050->read(adux1050->dev,
					       GET_HYS_REG(stg_cnt),
					       &hys_reg, DEF_WR);
				if (ret < DEF_WR) {
					ADUX1050_ERR("%s: read  GET_HYS_REG(stg_cnt) failed, ret = %d\n",
							__func__, ret);
					return;
				}

				hys_reg = ((hys_reg & HYS_BYTE_MASK) |
				 (adux1050->pdata->digi_offset[stg_cnt] << 8));
				ret = adux1050->write(adux1050->dev,
						GET_HYS_REG(stg_cnt),
						&hys_reg, DEF_WR);
				if (ret < DEF_WR) {
					ADUX1050_ERR("%s: write  GET_HYS_REG(stg_cnt) failed, ret = %d\n",
							__func__, ret);
					return;
				}

				ret = adux1050->write(
				adux1050->dev, GET_CONFIG_REG(stg_cnt),
				&adux1050->pdata->stg_cfg[stg_cnt], DEF_WR);
				if (ret < DEF_WR) {
					ADUX1050_ERR("%s: write  GET_CONFIG_REG(stg_cnt) failed, ret = %d\n",
							__func__, ret);
					return;
				}
			}

		} else {
			ADUX1050_WARN( "%s Invalid Stg num in FILP or SYSFS input\n", __func__);
			break;
		}
	}

	ADUX1050_INFO("Calib status = %d\n", adux1050->dac_calib.cal_flags);
	/*Restoring the Auto threshold mode if enabled previously*/
	if (temp_baseline_ctrl & AUTO_TH_MASK) {
		ret = adux1050->write(adux1050->dev, BASELINE_CTRL_REG,
				&temp_baseline_ctrl, DEF_WR);
		if (ret < DEF_WR) {
			ADUX1050_ERR("%s: write BASELINE_CTRL_REG failed, line = %d, ret = %d\n",
					__func__, __LINE__, ret);
			return;
		}
	}
	/* Reenable the interrupt */
	ret = adux1050->write(adux1050->dev, INT_CTRL_REG, &int_ctrl_reg, DEF_WR);
	if (ret < DEF_WR) {
		ADUX1050_ERR("%s: write INT_CTRL_REG failed, line = %d, ret = %d\n",
				__func__, __LINE__, ret);
		return;
	}

	if (cal_base_fail_flag != 0) {
		adux1050->dac_calib.cal_flags = CAL_RET_SUCCESS;
	#ifdef CONFIG_EVAL
		/* Getting the stage info */
		getstageinfo(adux1050);
	#endif
	} else {
		if (file_exist)
			adux1050->dac_calib.cal_flags = CAL_RET_EXIST;
		else
			adux1050->dac_calib.cal_flags = CAL_RET_NONE;
	}
}

/**
* \fn int adux1050_store_register_values(struct adux1050_chip *adux1050)
* This is to retreive the register values from either device tree/local platform
data and store it in local array
@param  adux1050 The Device structure
@return Zero on success
*/
static int adux1050_store_register_values(struct adux1050_chip *adux1050)
{
	u32 lcnt = 0;
	u32 data_cnt = 0;
	const u32 *init_buffer = NULL;
	const __be32 *df_regs = NULL;
	u32 df_prop_length = 0;
#ifdef	CONFIG_OF
	u32 len;
	const __be32 *property = NULL;
#endif
	u8 of_reg_found = false;

#ifdef	CONFIG_OF
	/* Fetching data from Device tree */
	if (adux1050->dt_device_node) {
		df_regs = of_get_property(adux1050->dt_device_node,
					  "adi,adux1050_reg", &df_prop_length);
		/* Fetching required baseline value for STG 0 */
		property = of_get_property(adux1050->dt_device_node,
					   "adi,adux1050_stg0_base",
					   &len);
		if (property && len == sizeof(int)) {
			adux1050->pdata->req_stg0_base =
					be32_to_cpu(*property);

			ADUX1050_DEBUG("valid req_base on %s\n", adux1050->dt_device_node->full_name);
		} else {
			ADUX1050_WARN("Invalid req_base on %s\n", adux1050->dt_device_node->full_name);
		}
		/* Fetching required baseline value for STG 1 */
		property = of_get_property(adux1050->dt_device_node,
					   "adi,adux1050_stg1_base",
					   &len);
		if (property && len == sizeof(int)) {
			adux1050->pdata->req_stg1_base =
					be32_to_cpu(*property);

			ADUX1050_DEBUG("valid req_base1 on %s\n", adux1050->dt_device_node->full_name);
		} else {
			ADUX1050_WARN("Invalid req_base1 on %s\n", adux1050->dt_device_node->full_name);
		}
		/* Fetching required baseline value for STG 2 */
		property = of_get_property(adux1050->dt_device_node,
					   "adi,adux1050_stg2_base",
					   &len);
		if (property && len == sizeof(int)) {
			adux1050->pdata->req_stg2_base =
					be32_to_cpu(*property);

			ADUX1050_DEBUG("valid req_base2 on %s\n", adux1050->dt_device_node->full_name);
		} else {
			ADUX1050_WARN("Invalid req_base2 on %s\n", adux1050->dt_device_node->full_name);
		}
		/* Fetching required baseline value for STG 3 */
		property = of_get_property(adux1050->dt_device_node,
					   "adi,adux1050_stg3_base", &len);
		if (property && len == sizeof(int)) {
			adux1050->pdata->req_stg3_base =
					be32_to_cpu(*property);

			ADUX1050_DEBUG("valid req_base3 on %s\n", adux1050->dt_device_node->full_name);
		} else {
			ADUX1050_WARN("Invalid req_base3 on %s\n", adux1050->dt_device_node->full_name);
		}
	}
#endif
	if ((adux1050->pdata->req_stg0_base > MAX_CALIB_TARGET) ||
	    (adux1050->pdata->req_stg0_base < MIN_CALIB_TARGET))
		adux1050->pdata->req_stg0_base = HALF_SCALE_VAL;

	if ((adux1050->pdata->req_stg1_base > MAX_CALIB_TARGET) ||
	    (adux1050->pdata->req_stg1_base < MIN_CALIB_TARGET))
		adux1050->pdata->req_stg1_base = HALF_SCALE_VAL;

	if ((adux1050->pdata->req_stg2_base > MAX_CALIB_TARGET) ||
	    (adux1050->pdata->req_stg2_base < MIN_CALIB_TARGET))
		adux1050->pdata->req_stg2_base = HALF_SCALE_VAL;

	if ((adux1050->pdata->req_stg3_base > MAX_CALIB_TARGET) ||
	    (adux1050->pdata->req_stg3_base < MIN_CALIB_TARGET))
		adux1050->pdata->req_stg3_base = HALF_SCALE_VAL;

	/* Data from either DT or initial platform data */
	if ((!df_regs) || (df_prop_length % sizeof(u32))) {
		if (df_prop_length % sizeof(u32))
			ADUX1050_WARN("[ADUX1050]: Malformed prop regs, use local data.\n");
		init_buffer = adux1050->pdata->init_regs;
		data_cnt = sizeof(adux1050->pdata->init_regs)/sizeof(int);
	} else {
		ADUX1050_INFO("[ADUX1050]: Use dts regs\n");
		init_buffer = df_regs;
		data_cnt = df_prop_length / sizeof(u32);
		of_reg_found = true;
	}
	/* Setting enable for INT_CTRL register */
	adux1050->reg[INT_CTRL_REG].wr_flag = ADUX1050_ENABLE;

	for (lcnt = 0; lcnt < data_cnt; lcnt++) {
		u8 addr;
		u16 value;
		/* getting the address and the value to be written */
		if (likely(of_reg_found)) {
			addr = (u8)((be32_to_cpu(init_buffer[lcnt])
						 & ADDR_MASK) >> HEX_BASE);
			value = (u16)(be32_to_cpu(init_buffer[lcnt])
						 & DATA_MASK);
		} else {
			addr = (u8)((init_buffer[lcnt] & ADDR_MASK)
						>> HEX_BASE);
			value = (u16)(init_buffer[lcnt] & DATA_MASK);
		}

		ADUX1050_DEBUG("[ADUX1050]: addr = %#x, value = %#x\n", addr, value);

		/* Having a copy of device tree values in driver */
		if ((addr >= DEV_ID_REG) &&
		    (addr <= HIGHEST_WR_ACCESS)) {
			adux1050->reg[addr].wr_flag = ADUX1050_ENABLE;
			adux1050->reg[addr].value = value;
		}
		/* Copy of Baseline registers */
		if ((addr >= BASELINE_STG0_REG) &&
		    (addr <= BASELINE_STG3_REG)) {
			switch (addr) {
			case BASELINE_STG0_REG:
				adux1050->bs_reg[STG_ZERO].wr_flag =
							ADUX1050_ENABLE;
				adux1050->bs_reg[STG_ZERO].value = value;
				break;
			case BASELINE_STG1_REG:
				adux1050->bs_reg[STG_ONE].wr_flag =
							ADUX1050_ENABLE;
				adux1050->bs_reg[STG_ONE].value = value;
				break;
			case BASELINE_STG2_REG:
				adux1050->bs_reg[STG_TWO].wr_flag =
							ADUX1050_ENABLE;
				adux1050->bs_reg[STG_TWO].value = value;
				break;
			case BASELINE_STG3_REG:
				adux1050->bs_reg[STG_THREE].wr_flag =
							ADUX1050_ENABLE;
				adux1050->bs_reg[STG_THREE].value = value;
			}
		}
	}

	return 0;
}

/**
* \fn  int adux1050_hw_init(struct adux1050_chip *adux1050)
* To initialize the ADUX1050 device with register set defined in
	platform file or device tree
@param  adux1050 The Device structure
@return Zero on success
*/
static int adux1050_hw_init(struct adux1050_chip *adux1050)
{
	u32 lcnt = 0;
	u16 addr;
	u16 slp_time = 0;
	u16 temp_baseline_ctrl = 0;
	u16 pwr_ctrl_buff = 0;
	u16 temp_reg_value = 0;
	u16	value = 0;
	s16 ret;

	for (lcnt = 0; lcnt < (GLOBAL_REG_CNT + STG_CNF_CNT); lcnt++) {
		addr = lcnt;
		if (adux1050->reg[addr].wr_flag == ADUX1050_ENABLE) {
			value = adux1050->reg[addr].value;
			if (addr == BASELINE_CTRL_REG) {
				value = value & ANTI_FORCE_CAL_MASK;
				if (value & AUTO_TH_MASK) {
					temp_baseline_ctrl = value;
					value = value & (~AUTO_TH_MASK);
				}
			} else if (addr == CTRL_REG) {
				value = value & ~RESET_MASK;
				pwr_ctrl_buff = value;
				value = SET_PWR_MODE(value, PWR_STAND_BY);
			} else if (addr == INT_CTRL_REG) {
				if (adux1050->int_pol == ACTIVE_HIGH)
					adux1050->int_ctrl =
						(value | ACTIVE_HIGH);
				else
					adux1050->int_ctrl =
						(value & ~ACTIVE_HIGH);
				value = adux1050->int_ctrl | DISABLE_DEV_INT;
			}
			ADUX1050_DEBUG("Addr %x Val %x\n", addr, value);
			ret = adux1050->write(adux1050->dev, addr, &value, DEF_WR);
			if (ret < DEF_WR) {
				ADUX1050_ERR("%s: write addr = %#x, data = %#x failed, ret = %d\n",
						__func__, addr, value, ret);
				return ret;
			}
		}
	}
	/* Baseline registers update */
	if (adux1050->bs_reg[STG_ZERO].wr_flag == ADUX1050_ENABLE) {
		ret = adux1050->write(adux1050->dev, BASELINE_STG0_REG,
				&adux1050->bs_reg[STG_ZERO].value, DEF_WR);
		if (ret < DEF_WR) {
			ADUX1050_ERR("%s: write BASELINE_STG0_REG failed, ret = %d\n",
					__func__, ret);
			return ret;
		}
	}
	if (adux1050->bs_reg[STG_ONE].wr_flag == ADUX1050_ENABLE) {
		ret = adux1050->write(adux1050->dev, BASELINE_STG1_REG,
				&adux1050->bs_reg[STG_ONE].value, DEF_WR);
		if (ret < DEF_WR) {
			ADUX1050_ERR("%s: write BASELINE_STG1_REG failed, ret = %d\n",
					__func__, ret);
			return ret;
		}
	}
	if (adux1050->bs_reg[STG_TWO].wr_flag == ADUX1050_ENABLE) {
		ret = adux1050->write(adux1050->dev, BASELINE_STG2_REG,
				&adux1050->bs_reg[STG_TWO].value, DEF_WR);
		if (ret < DEF_WR) {
			ADUX1050_ERR("%s: write BASELINE_STG2_REG failed, ret = %d\n",
					__func__, ret);
			return ret;
		}
	}
	if (adux1050->bs_reg[STG_THREE].wr_flag == ADUX1050_ENABLE) {
		ret = adux1050->write(adux1050->dev, BASELINE_STG3_REG,
				&adux1050->bs_reg[STG_THREE].value, DEF_WR);
		if (ret < DEF_WR) {
			ADUX1050_ERR("%s: write BASELINE_STG3_REG failed, ret = %d\n",
					__func__, ret);
			return ret;
		}
	}

	/* Restoring the power mode given in configuration */
	if (pwr_ctrl_buff) {
		ret = adux1050->write(adux1050->dev, CTRL_REG,
				&pwr_ctrl_buff, DEF_WR);
		if (ret < DEF_WR) {
			ADUX1050_ERR("%s: write CTRL_REG failed, ret = %d\n",
					__func__, ret);
			return ret;
		}
		slp_time = get_conv_time(adux1050, CONV_TIME);
		msleep(slp_time);
		ADUX1050_DEBUG("Addr %x New Val %x\n", CTRL_REG, pwr_ctrl_buff);
	}
	/* Auto threshold enable  */
	if (temp_baseline_ctrl) {
		ret = adux1050->write(adux1050->dev, BASELINE_CTRL_REG,
				&temp_baseline_ctrl, DEF_WR);
		if (ret < DEF_WR) {
			ADUX1050_ERR("%s: write BASELINE_CTRL_REG failed, ret = %d\n",
					__func__, ret);
			return ret;
		}
		ADUX1050_DEBUG("Addr %x New Val %x\n", BASELINE_CTRL_REG, temp_baseline_ctrl);
	}
	/* Clearing the device interrupt STATUS register */
	ret = adux1050->read(adux1050->dev, INT_STATUS_REG, &temp_reg_value, DEF_WR);
	if (ret < DEF_WR) {
		ADUX1050_ERR("%s: read INT_STATUS_REG failed, ret = %d\n",
				__func__, ret);
		return ret;
	}
	adux1050->prev_low_status = GET_LOW_STATUS(temp_reg_value);
	adux1050->prev_high_status = GET_HIGH_STATUS(temp_reg_value);

	/* Enabling the device interrupt */
	ret = adux1050->write(adux1050->dev, INT_CTRL_REG,
			&adux1050->int_ctrl, DEF_WR);
	if (ret < DEF_WR) {
		ADUX1050_ERR("%s: write INT_CTRL_REG failed, ret = %d\n",
				__func__, ret);
		return ret;
	}
	ADUX1050_DEBUG("Addr %x New Val %x\n", INT_CTRL_REG, adux1050->int_ctrl);

	/* Getting the stage info */
	getstageinfo(adux1050);

	/* Storing the sleeping time required for this configuration */
	adux1050->slp_time_conv_complete = get_conv_time(adux1050,
						TWICE_CONV_DELAY_TIME);

	ADUX1050_INFO("hw init success\n");

	return 0;
}

/**
* \fn offset_write(struct adux1050_chip *adux1050, u16 stg_num, u16 data,
			u16 slp_time)
* Internal function used to write the offset of the Stages
		 with a predeterminded delay
@param adux1050 ADUX chip structure
@param stg_num The stage to which offset has to be written
@param data The Value to be written to the offset register
@param slp_time Sleep time to given after writing the offset register
@return write status is returned
 */
static int offset_write(struct adux1050_chip *adux1050, u16 stg_num, u16 data,
			u16 slp_time)
{
	s32 ret = 0;
	ret = adux1050->write(adux1050->dev, GET_OFFSET_REG(stg_num), &data,
				DEF_WR);
	if (ret < DEF_WR) {
		ADUX1050_ERR("%s: write GET_OFFSET_REG(stg_num) failed, ret = %d\n",
				__func__, ret);
		return ret;
	}
	if (likely(slp_time))
		msleep(slp_time);
	return ret;
}
/**
* \fn inline s16 get_calc_dac_offset(u16 offset, u16 stg_cfg_reg)
* To calculate effective DAC offset value from pos & neg dac offset
@param offset The current offset available.
@param stg_cfg_reg The current stage configuration.
@return the equivalant offset based on the swap bits.
*/
static inline s16 get_calc_dac_offset(u16 offset, u16 stg_cfg_reg)
{
	s16 cal_offset = 0;
	cal_offset = LD_POS_DAC_OFFSET(offset, stg_cfg_reg) +
		     LD_NEG_DAC_OFFSET(offset, stg_cfg_reg);
	return cal_offset;
}

/**
* \fn inline int set_calc_dac_offset(struct adux1050_chip *adux1050,
			      s16 cal_offset, u16 stg_num, u16 *stg_cfg_reg,
			      u16 *offset, u16 sleep_time)
* Helper function to calculate the DAC offset for a stage.
@param adux1050 Chip structure.
@param cal_offset Calculated equalized offset
@param stg_num Stage number
@param *stg_cfg_reg Stage configuration register value.
@param *offset Current offset set.
@param sleep_time Sleep time required for register result cdc.
@return Offset to be set to the register.
*/
static inline int set_calc_dac_offset(struct adux1050_chip *adux1050,
			      s16 cal_offset, u16 stg_num, u16 *stg_cfg_reg,
			      u16 *offset, u16 sleep_time)
{
	u16 err = 0;
	if ((-MAX_OFFSET > cal_offset) || (cal_offset > MAX_OFFSET)) {
		ADUX1050_ERR("%s, offset ERROR(%d)\n", __func__, cal_offset);
		return -EINVAL;
	} else {
		*offset = set_dac_offset(cal_offset);
		err = set_swap_state(adux1050, stg_num,
				     stg_cfg_reg, cal_offset);
		if (err < 0) {
			ADUX1050_ERR("%s, set swap state failed\n", __func__);
			return -EIO;
		}

		ADUX1050_DEBUG("%s, cal_off(%d)offset(%x) swap(%x)\n",
			__func__, cal_offset, *offset, *stg_cfg_reg);
	}
	return offset_write(adux1050, stg_num, *offset, sleep_time);
}

/**
* \fn int adux1050_offset_check(struct adux1050_chip *adux1050,
						u16 max_cnt, s16 dir_flag, u16 slp_time)
* ADUX1050 Calibration routine to check the offset
@param adux1050 The device structure to be calibrated
@param max_cnt	Maximum count
@param dir_flag	Direction flag
@param slp_time	Sleep time to be given after offset check
@return The number of configuration data generated
*/
static int adux1050_offset_check(struct adux1050_chip *adux1050, u16 max_cnt,
					s16 dir_flag, u16 slp_time)
{
	s16 cal_offset;
	u16 data;
	u16 power_ctrl;
	u16 cin_range;
	u16 lp_count;
	u16 stg_num;
	u16 *offset;
	u16 *stg_cfg_reg;
	s16 ret;

	ret = adux1050->read(adux1050->dev, CTRL_REG, &power_ctrl, DEF_WR);
	if (ret < DEF_WR) {
		ADUX1050_ERR("%s: read CTRL_REG failed, ret = %d\n",
				__func__, ret);
		return ret;
	}
	cin_range = GET_CIN_RANGE(power_ctrl);
	adux1050->tot_stg = adux1050->num_stages;
	for (lp_count = 0; lp_count < max_cnt ; lp_count++) {
		for (stg_num = 0; stg_num < adux1050->tot_stg; stg_num++) {
			if ((!CHECK_CAL_STATE(
			    adux1050->dac_calib.sat_comp_stat, stg_num)) ||
			    (adux1050->stg_info[stg_num].status ==
			    CIN_NOT_CONNECTED))
				continue;
			offset = &adux1050->cur_dac_offset[stg_num];
			stg_cfg_reg = &adux1050->cur_swap_state[stg_num];
			ret = adux1050->read(adux1050->dev, GET_RESULT_REG(stg_num),
				       &data, DEF_WR);
			if (ret < DEF_WR) {
				ADUX1050_ERR("%s: read GET_RESULT_REG(stg_num) failed, ret = %d\n",
						__func__, ret);
				return ret;
			}
			if ((data <= ZERO_SCALE_VALUE) ||
			    (data >= FULL_SCALE_VALUE)) {
				if (lp_count != 0)
					cal_offset = get_calc_dac_offset(
							*offset, *stg_cfg_reg);
				else
					cal_offset = 0;
				cal_offset +=
				(GET_DAC_STEP_SIZE(DAC_CODEOUT_SAT,
						   cin_range) * dir_flag);
				ADUX1050_DEBUG("stg_num[%d]Curr offset set (%d)\n",
					stg_num, cal_offset);
				set_calc_dac_offset(adux1050, cal_offset,
						    stg_num, stg_cfg_reg,
						    offset, 0);
			} else {
				CLR_CAL_STATUS(
				adux1050->dac_calib.sat_comp_stat, stg_num);
				ADUX1050_DEBUG("%s - SATUR state(%d)\n", __func__,
					adux1050->dac_calib.sat_comp_stat);
				if (!adux1050->dac_calib.sat_comp_stat)
					goto adux_offset_ok;
			}
			ADUX1050_INFO("SATURATION STATUS - %x\n",
					    adux1050->dac_calib.sat_comp_stat);
		}
		msleep(slp_time);
	}
	return -EIO;
adux_offset_ok:
	ADUX1050_INFO("%s, offset ok (%d)\n", __func__, data);
	return 0;
}

static inline void reset_stgcal_flag(struct adux1050_chip *adux1050, u16 *count)
{
	u16 lp_cnt = 0;
	for (; lp_cnt < adux1050->tot_stg; lp_cnt++) {
		if (adux1050->stg_info[lp_cnt].status == CIN_CONNECTED) {
			adux1050->dac_calib.stg_cal_stat |= (1 << lp_cnt);
			count[lp_cnt] = 1;
		}
	}

}

/**
* \fn do_dac_compensation(struct adux1050_chip *adux1050, u16 power_ctrl,
				u16 cin_range, u16 *slp_time)
* Non saturation CDC compensation routine.
@param adux1050 The device structure to be calibrated
@param power_ctrl The power control register read
@param cin_range  The current configuration's CIN range value
@param slp_time	  The sleep time to be given
*/
static int do_dac_compensation(struct adux1050_chip *adux1050, u16 power_ctrl,
				u16 cin_range, u16 *slp_time)
{
	u16 cdc_diff;
	u16 lp_count = 0;
	u16 stg_num = 0;
	u16 count[TOTAL_STG] = {0};
	s16 cal_offset = 0;
	u16 flr_cnt = 0;
	u16 data[TOTAL_STG] = {0};
	u16 hys_reg[TOTAL_STG] = {0};
	u16 offset = 0;
	u16 stg_cfg_reg = 0;
	s16 digi_offset = 0;
	u16 use_digi_offset = 0;
	s32 err = -EIO;
	u16 u16_div = GET_ARB_DAC_STEP_SIZE(cin_range);
	u16 dac_step = u16_div;
	u16 trgt[TOTAL_STG] = {0};
	u16 init_swap_state[TOTAL_STG];
	u16 init_dac_offset[TOTAL_STG];
	s16 ret;

	struct adux1050_platform_data *pdata = adux1050->pdata;
	*slp_time = get_conv_time(adux1050, TWICE_CONV_DELAY_TIME);
	adux1050->dac_calib.sat_comp_stat = ZERO_VAL;

	/*Writing initial values to zero*/
	for (lp_count = 0; lp_count < adux1050->tot_stg; lp_count++) {
		ret = adux1050->read(adux1050->dev, GET_OFFSET_REG(lp_count),
			       &offset, DEF_WR);
		if (ret < DEF_WR) {
			ADUX1050_ERR("%s: read GET_OFFSET_REG(lp_count) failed, ret = %d\n",
					__func__, ret);
			return ret;
		}
		ret = adux1050->read(adux1050->dev, GET_CONFIG_REG(lp_count),
			       &stg_cfg_reg, DEF_WR);
		if (ret < DEF_WR) {
			ADUX1050_ERR("%s: read GET_CONFIG_REG(lp_count) failed, ret = %d\n",
					__func__, ret);
			return ret;
		}
		init_swap_state[lp_count] = stg_cfg_reg;
		init_dac_offset[lp_count] = offset;
		/*Set initial step count to 1 WARNING: DO NOT set it to zero */
		count[lp_count] = 1;
		cal_offset = get_calc_dac_offset(offset, stg_cfg_reg);
		if (cal_offset != ZERO_VAL)
			cal_offset = 0;
		set_calc_dac_offset(adux1050, cal_offset, lp_count,
				    &stg_cfg_reg, &offset, 0);
		adux1050->cur_swap_state[lp_count] = stg_cfg_reg;
		adux1050->cur_dac_offset[lp_count] = offset;
		trgt[lp_count] = HALF_SCALE_VAL;
		/** Clear the Digital offset if set already*/
		ret = adux1050->read(adux1050->dev, GET_HYS_REG(lp_count),
			       &hys_reg[lp_count], DEF_WR);
		if (ret < DEF_WR) {
			ADUX1050_ERR("%s: read GET_HYS_REG(lp_count) failed, ret = %d\n",
					__func__, ret);
			return ret;
		}

		hys_reg[lp_count] &= HYS_BYTE_MASK;
		ret = adux1050->write(adux1050->dev, GET_HYS_REG(lp_count),
				&hys_reg[lp_count], DEF_WR);
		if (ret < DEF_WR) {
			ADUX1050_ERR("%s: write GET_HYS_REG(lp_count) failed, ret = %d\n",
					__func__, ret);
			return ret;
		}

		if (adux1050->stg_info[lp_count].status == CIN_CONNECTED) {
			adux1050->dac_calib.sat_comp_stat |= (1 << lp_count);
			adux1050->dac_calib.stg_cal_stat |= (1 << lp_count);
		}
	}
	msleep(*slp_time);
	u16_div = GET_ARB_DAC_STEP_SIZE(GET_CIN_RANGE(power_ctrl));
	dac_step = u16_div;
	/* Clear the device from saturation*/
	flr_cnt = (MAX_OFFSET / (GET_DAC_STEP_SIZE(DAC_CODEOUT_SAT,
					GET_CIN_RANGE(power_ctrl))));
	ADUX1050_DEBUG("%s - POSITIVE SAT ROUTINE\n", __func__);
	err = adux1050_offset_check(adux1050, flr_cnt, 1, *slp_time);
	if (err < 0) {
		ADUX1050_DEBUG("%s - NEGATIVE SAT ROUTINE\n", __func__);
		err = adux1050_offset_check(adux1050, flr_cnt,
					    MINUS_VAL, *slp_time);
		if (err < 0)
			return -EIO;
	}
	/* Set Stage cal status to uncalibrated initial value*/

	/* Make the stages to their corresponding target value*/
	for (lp_count = 0; lp_count < CALIB_LOOP_CNT; lp_count++) {
		u16 prv_data;
		for (stg_num = 0; stg_num < adux1050->tot_stg; stg_num++) {
			/*Skip the stage if already calibrated*/
			if ((!CHECK_CAL_STATE(
			    adux1050->dac_calib.stg_cal_stat, stg_num)) ||
			    (adux1050->stg_info[stg_num].status ==
			    CIN_NOT_CONNECTED))
				continue;
			prv_data = data[stg_num];
			ret = adux1050->read(adux1050->dev, GET_RESULT_REG(stg_num),
				       &data[stg_num], DEF_WR);
			if (ret < DEF_WR) {
				ADUX1050_ERR("%s: read GET_RESULT_REG(st_num) failed, ret = %d\n",
						__func__, ret);
				return ret;
			}
			ADUX1050_DEBUG("DATA %d,OFF %x,SWAP %x,STG %d\n",
				data[stg_num],
				adux1050->cur_dac_offset[stg_num],
				adux1050->cur_swap_state[stg_num],
				stg_num);
			/*Device not in saturation*/
			cdc_diff = abs(data[stg_num] - trgt[stg_num]);
			if (use_digi_offset && (cdc_diff < DIGI_OFFSET_SIZE)) {
				count[stg_num] = 0;
			} else {
				u16_div = abs(prv_data - data[stg_num]);
				u16_div = u16_div / count[stg_num];
				if ((u16_div < (dac_step/2)) ||
				    (u16_div > (dac_step + dac_step)))
					u16_div = dac_step;
				ADUX1050_DEBUG("%s,p_data(%d)",
					__func__, prv_data);
				flr_cnt = cdc_diff / u16_div;
				count[stg_num] = ((cdc_diff % u16_div) >
					 GET_60_PERCENT(u16_div)) ?
					 (flr_cnt + 1) : flr_cnt;
				ADUX1050_DEBUG("FLC(%d),CNT(%d),DIV(%d),DIF(%d)\n",
					flr_cnt, count[stg_num],
					u16_div, cdc_diff);
			}
			if (count[stg_num] != 0) {
				/* DAC step can be used to minimize the
				 * difference in current and required CDC*/
				cal_offset = get_calc_dac_offset(
					adux1050->cur_dac_offset[stg_num],
					adux1050->cur_swap_state[stg_num]);
				ADUX1050_DEBUG("%s,cal_off %d\n",
					__func__, cal_offset);
				if (data[stg_num] > trgt[stg_num])
					cal_offset += count[stg_num];
				else
					cal_offset -= count[stg_num];
				err = set_calc_dac_offset(adux1050, cal_offset,
				stg_num, &adux1050->cur_swap_state[stg_num],
				&adux1050->cur_dac_offset[stg_num], 0);
				if (err < ZERO_VAL) {
					ADUX1050_ERR("%s S_Off err\n",
						__func__);
					break;
				}
			} else {
				CLR_CAL_STATUS(
				adux1050->dac_calib.stg_cal_stat, stg_num);
				/*When all the stages are in halfscale change to
				  the original target and CIN_range*/
				if ((!adux1050->dac_calib.stg_cal_stat) &&
				    (!use_digi_offset)) {
					trgt[STG_ZERO] = pdata->req_stg0_base;
					trgt[STG_ONE] = pdata->req_stg1_base;
					trgt[STG_TWO] = pdata->req_stg2_base;
					trgt[STG_THREE] = pdata->req_stg3_base;
					ADUX1050_DEBUG("R TRGT %x,%x,%x,%x\n",
						trgt[STG_ZERO], trgt[STG_ONE],
						trgt[STG_TWO],
						trgt[STG_THREE]);
					power_ctrl = SET_CIN_RANGE(power_ctrl,
								   cin_range);
					ret = adux1050->write(adux1050->dev, CTRL_REG,
							&power_ctrl, DEF_WR);
					if (ret < DEF_WR) {
						ADUX1050_ERR("%s: write CTRL_REG failed, ret = %d\n",
								__func__, ret);
						return ret;
					}
					u16_div = GET_ARB_DAC_STEP_SIZE(
						    GET_CIN_RANGE(power_ctrl));
					dac_step = u16_div;
					reset_stgcal_flag(adux1050, count);

					use_digi_offset++;
					/* msleep(*slp_time);*/
					break;
				}
				/*Not in saturation and DAC minimal step size
				 * is higher than the required correction*/
				digi_offset = GET_DIGI_OFFSET(trgt[stg_num],
							      data[stg_num]);
				digi_offset = CLAMP_DIGI_OFFSET(digi_offset);

				adux1050->pdata->cal_offset[stg_num] =
					adux1050->cur_dac_offset[stg_num];
				adux1050->pdata->digi_offset[stg_num] =
					(u8)digi_offset;

				hys_reg[stg_num] =
				((hys_reg[stg_num] & HYS_BYTE_MASK) |
				 (adux1050->pdata->digi_offset[stg_num] << 8));
				ret = adux1050->write(adux1050->dev,
						GET_HYS_REG(stg_num), &hys_reg[stg_num], DEF_WR);
				if (ret < DEF_WR) {
					ADUX1050_ERR("%s: write GET_HYS_REG(stg_num) failed, ret = %d\n",
							__func__, ret);
					return ret;
				}
				ADUX1050_DEBUG("Hys value = %x\n", hys_reg[stg_num]);
				msleep(*slp_time);
				ret = adux1050->read(adux1050->dev,
					       GET_RESULT_REG(stg_num),
					       &data[stg_num], DEF_WR);
				if (ret < DEF_WR) {
					ADUX1050_ERR("%s: read GET_RESULT_REG(stg_num) failed, ret = %d\n",
							__func__, ret);
					return ret;
				}
				adux1050->pdata->cal_fact_base[stg_num] =
						data[stg_num];
				adux1050->pdata->stg_cfg[stg_num] =
					adux1050->cur_swap_state[stg_num];
				ADUX1050_DEBUG("bas(%d)off(%x)st_con(%x)dioff(%d)\n", data[stg_num], offset,
					 stg_cfg_reg, digi_offset);
				err = ZERO_VAL;
				ADUX1050_DEBUG("CAL STATUS - %x\n",
					adux1050->dac_calib.stg_cal_stat);
				if (!adux1050->dac_calib.stg_cal_stat)
					goto calib_success_break;
				else
					continue;
			}
		}	/*End of stage based loop*/
		msleep(*slp_time);
	}	/*End of for*/
	/*Failed to complete the compensation, return to the original values*/
	for (lp_count = 0; lp_count < adux1050->tot_stg; lp_count++) {
		ret = adux1050->write(adux1050->dev, GET_CONFIG_REG(lp_count),
				&init_swap_state[lp_count], DEF_WR);
		if (ret < DEF_WR) {
			ADUX1050_ERR("%s: write GET_CONFIG_REG(lp_count) failed, ret = %d\n",
					__func__, ret);
			return ret;
		}

		ret = adux1050->write(adux1050->dev, GET_OFFSET_REG(lp_count),
			       &init_dac_offset[lp_count], DEF_WR);
		if (ret < DEF_WR) {
			ADUX1050_ERR("%s: write GET_OFFSET_REG(lp_count) failed, ret = %d\n",
					__func__, ret);
			return ret;
		}

		adux1050->pdata->cal_fact_base[lp_count] = 0;
		adux1050->pdata->cal_offset[lp_count] = 0;
		adux1050->pdata->digi_offset[lp_count] = 0;
		adux1050->pdata->stg_cfg[lp_count] = 0;
		adux1050->dac_calib.cal_flags = CAL_RET_FAIL;
	}
	msleep(*slp_time);
	return err;
calib_success_break:
	adux1050->dac_calib.cal_flags = CAL_RET_SUCCESS;
	return CAL_RET_SUCCESS;
}

#ifdef CONFIG_USE_FILP
/**
* \fn int adux1050_open_calibration(struct adux1050_chip *adux1050,
				bool write_to_reg)
* This routine is used to get the calibration output from the file system and
  update the configuration to the device registers
@param adux1050 The Device Id structure
@param write_to_reg Flag to decide whether to update ADUX1050 registers
@return Returns the size of the read configuration or error number.
 */

static int adux1050_open_calibration(struct adux1050_chip *adux1050,
				bool write_to_reg)
{
	struct file *offset_filp = NULL;
	u16 st_file[FILP_PARAM_CNT * TOTAL_STG + 1] = {0};
	u16 tot_stg_cnt = 0;
	u16 fp = 0;
	s32 err = 0;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	offset_filp = filp_open(CAL_DATA_FILE_PATH, O_RDONLY, 0666);
	if (IS_ERR(offset_filp)) {
		ADUX1050_ERR("%s: no offset file\n", __func__);
		err = PTR_ERR(offset_filp);
		if (err != -ENOENT)
			ADUX1050_ERR("%s: Can't open calib file\n",
				__func__);
		set_fs(old_fs);
		adux1050->dac_calib.cal_flags = CAL_RET_NONE;
		return err;
	}


	err = offset_filp->f_op->read(offset_filp, (char *)&st_file,
		(sizeof(u16) * TOTAL_STG * FILP_PARAM_CNT + sizeof(u16)),
		&offset_filp->f_pos);

	if ((err <= 0) || (st_file[0] == 0) ||
	    (st_file[0] > (TOTAL_STG))) {
		ADUX1050_ERR("%s:Can't read, or the file is empty\n", __func__);
		err = -EIO;
		adux1050->dac_calib.cal_flags = CAL_RET_EXIST;
		goto flip_exit;
	}
	/* Modified st_file[0] contains only stage number */
	tot_stg_cnt = st_file[fp++];
	ADUX1050_DEBUG("Total_stg_cnt - %d, st_file[0] - %d\n", tot_stg_cnt, st_file[0]);
	update_calib_settings(adux1050, tot_stg_cnt,
			      &st_file[1], write_to_reg, ADUX1050_ENABLE);


flip_exit:
	/*filp_close(offset_filp, current->files);*/
	filp_close(offset_filp, NULL);
	set_fs(old_fs);

	return err;
}


/**
* \fn int save_calib_val_filp(struct adux1050_chip *adux1050)
* This function is used to save the the calibrated values in a file.
@param adux1050 adux1050_chip structure.
@return zero on success and Negaitive err values.
 */
static int save_calib_val_filp(struct adux1050_chip *adux1050)
{
	s32 err = 0;
	u16 st_file[TOTAL_STG * FILP_PARAM_CNT] = {0};
	struct file *offset_filp = NULL;
	mm_segment_t old_fs;
	u16 fp = 0;
	u16 lp_cnt = 0;

	fp++;
	for (lp_cnt = 0; lp_cnt < adux1050->tot_stg; lp_cnt++) {
		ADUX1050_DEBUG("LOOP %d ", lp_cnt);
		if (adux1050->stg_info[lp_cnt].status == CIN_CONNECTED) {
			st_file[fp++] = lp_cnt;
			ADUX1050_DEBUG("{ lp_cnt = %x \n", lp_cnt);
			st_file[fp++] = adux1050->pdata->cal_fact_base[lp_cnt];
			ADUX1050_DEBUG("cal_fact_base = %x \n", st_file[fp - 1]);
			st_file[fp++] = adux1050->pdata->cal_offset[lp_cnt];
			ADUX1050_DEBUG("cal_offset = %x \n", st_file[fp - 1]);
			st_file[fp++] = adux1050->pdata->digi_offset[lp_cnt];
			ADUX1050_DEBUG("digi_offset = %x \n", st_file[fp - 1]);
			st_file[fp++] = adux1050->pdata->stg_cfg[lp_cnt];
			ADUX1050_DEBUG("stg_cfg = %x }\n", st_file[fp - 1]);
		}
	}

	if ((adux1050->dac_calib.cal_flags == CAL_RET_FAIL) ||
	    (adux1050->dac_calib.cal_flags == CAL_RET_NONE))
		adux1050->dac_calib.cal_flags = CAL_RET_EXIST;

	if ((adux1050->dac_calib.action_flag != 0) &&
	    (adux1050->dac_calib.cal_flags == CAL_RET_SUCCESS))
		/*Modified - To stg number only */
		st_file[0] = adux1050->conn_stg_cnt;
	else
		st_file[0] = 0;

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	offset_filp = filp_open(CAL_DATA_FILE_PATH,
			O_CREAT | O_TRUNC | O_WRONLY | O_SYNC,
			0666);
	if (IS_ERR(offset_filp)) {
		ADUX1050_ERR("%s: Can't open file\n", __func__);
		set_fs(old_fs);
		err = PTR_ERR(offset_filp);
		adux1050->dac_calib.cal_flags = CAL_RET_NONE;
		goto err_file_open;
	}

	err = offset_filp->f_op->write(offset_filp, (u8 *)&st_file,
			(sizeof(u16) + (sizeof(u16) *
			adux1050->conn_stg_cnt * FILP_PARAM_CNT)),
			&offset_filp->f_pos);
	if (err != ((sizeof(u16) * FILP_PARAM_CNT * adux1050->conn_stg_cnt) +
		    sizeof(u16))) {
		ADUX1050_ERR("%s:Data write failed!!\n", __func__);
		adux1050->dac_calib.cal_flags = CAL_RET_NONE;
		err = -EIO;
	}

	/*filp_close(offset_filp, current->files);*/
	offset_filp->f_pos = 0;
	filp_close(offset_filp, NULL);


err_file_open:
	set_fs(old_fs);
	return 0;
}
#endif

#ifdef CONFIG_USE_FILP
/**
* a local function tests the valid data/adress in hexa decimal
*/
static bool is_valid_hexa(char *temp)
{
	if (!((((int)(*temp) >= ASCII_0) &&
	       ((int)(*temp) <= ASCII_9)) ||
	     (((int)(*temp) >= ASCII_A) &&
	      ((int)(*temp) <= ASCII_F)))) {
		return FALSE;
	} else {
		return TRUE;
	}
}

/**
* This routine is used to extract the configuration file for filp,
*If the loaded file is invalid then it skip the the data and come out
*of the loop and the register will not be effected
 */
static ssize_t adux1050_filp_config(struct adux1050_chip *adux1050)
{
	struct file *filp = NULL;
	int val;
	bool valid;
	s32 err = 0;
	u16 addr = 0;
	char *temp_data = NULL;
	char *temp_addr = NULL;
	char *temp_val = NULL;
	char *temp_buf_full = kmalloc(MAX_FILP_READ_SIZE, GFP_KERNEL);
	char *temp_base = temp_buf_full;
	mm_segment_t old_fs;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	if (!temp_buf_full) {
		set_fs(old_fs);
		return -EIO;
	}
	filp = filp_open(CONFIG_DATA_FILE_PATH, O_RDONLY, 0666);
	if (IS_ERR(filp)) {
		/*printk("%d\n", ERR_PTR(filp));*/
		ADUX1050_ERR("%s: no config file\n", __func__);
		err = PTR_ERR(filp);
		if (err != -ENOENT)
			ADUX1050_ERR("%s: Can't open file\n", __func__);
		kfree(temp_buf_full);
		set_fs(old_fs);
		return err;
	}

	err = filp->f_op->read(filp, temp_buf_full,
			       MAX_FILP_READ_SIZE,
			       &filp->f_pos);
	if (err <= 0) {
		ADUX1050_ERR("%s:Can't read Err %d\n",
			__func__, err);
		err = -EIO;
		goto err_filp_nodata;
	} else {
		do {
			temp_data = strsep(&temp_buf_full, LINE_BREAK);
			if (temp_data == NULL)
				break;
			strsep(&temp_data, TAB_BREAK);
			if (temp_data == NULL)
				break;
			valid = is_valid_hexa(temp_data);
			if (!valid) {
				ADUX1050_ERR("invalid data = %s\n", temp_data);
				goto err_filp_nodata;
			}
			temp_addr = strsep(&temp_data, TAB_BREAK);
			if (temp_addr != NULL) {
				valid = is_valid_hexa(temp_addr);
				if (!valid) {
					ADUX1050_ERR("invalid addr= %s\n", temp_addr);
					goto err_filp_nodata;
				}
				valid = is_valid_hexa(temp_data);
				if (!valid) {
					ADUX1050_ERR("invalid data = %s\n", temp_data);
					goto err_filp_nodata;
				}
				addr = 0;
				err = kstrtou16(temp_addr, HEX_BASE, &addr);
				if (err < 0)
					goto err_filp_nodata;
				if ((addr > DEV_ID_REG) &&
				    (addr <= GET_HYS_REG(STG_THREE))) {
					adux1050->reg[addr].wr_flag = ADUX1050_ENABLE;
					ADUX1050_DEBUG("Data %s\n", temp_data);
					temp_val = strsep(&temp_data, PRE_LINE_BREAK);
					ADUX1050_DEBUG( "aftr preline temp_val %s\n", temp_val);
					err = kstrtou16((const char *)temp_val, HEX_BASE,
						&adux1050->reg[addr].value);
					if (err < 0)
						goto err_filp_nodata;
					ADUX1050_DEBUG( "temp_reg[%x]= %x,err %d\n", addr,
						 adux1050->reg[addr].value, err);
				} else if ((addr >= BASELINE_STG0_REG) &&
					   (addr <= BASELINE_STG3_REG)) {
					val = addr-BASELINE_STG0_REG;
					adux1050->bs_reg[val].wr_flag = ADUX1050_ENABLE;
					temp_val = strsep(&temp_data, PRE_LINE_BREAK);
					ADUX1050_DEBUG( "temp_val %s\n", temp_val);
					err = kstrtou16(temp_val, HEX_BASE, &adux1050->bs_reg[val].value);
					if (err < 0)
						goto err_filp_nodata;
					ADUX1050_DEBUG( "bs_reg[%x]= %x , err = %d\n", val,
						 adux1050->bs_reg[val].value, err);
				} else {
					ADUX1050_INFO("Ignoring addr %x\n", addr);
				}
			} else {
				goto err_filp_nodata;
			}
		} while (*temp_buf_full != '0');
	}
	temp_buf_full = NULL;
	filp_close(filp, NULL);
	kfree(temp_base);
	set_fs(old_fs);
	return err;
err_filp_nodata:
	temp_buf_full = NULL;
	ADUX1050_ERR("No/Corrupt data\n");
	filp_close(filp, NULL);
	kfree(temp_base);
	set_fs(old_fs);
	return -EIO;
}
#endif

/**
* \fn inline int adux1050_disable(struct adux1050_chip *adux1050)
* Routine to set the power mode to standby in the ADUX1050 chip
@param adux1050 Chip structure to set the power mode to shutdown
@return Zero on success.
*/
static inline int adux1050_disable(struct adux1050_chip *adux1050)
{
	u16 data = 0;
	s16 ret;

	mutex_lock(&adux1050->mutex);
	ret = adux1050->read(adux1050->dev, CTRL_REG, &data, DEF_WR);
	if (ret < DEF_WR) {
		ADUX1050_ERR("%s: read CTRL_REG failed, ret = %d\n",
				__func__, ret);
		return ret;
	}
	if (GET_PWR_MODE(data) != PWR_STAND_BY) {
		data = SET_PWR_MODE(data, PWR_STAND_BY);
		ret = adux1050->write(adux1050->dev, CTRL_REG, &data, DEF_WR);
		if (ret < DEF_WR) {
			ADUX1050_ERR("%s: write CTRL_REG failed, ret = %d\n",
					__func__, ret);
			return ret;
		}
	}

	mutex_unlock(&adux1050->mutex);
	return 0;
}
/**
* \fn int adux1050_enable(struct adux1050_chip *adux1050)
* Routine to set the driver to enable state in the ADUX1050 chip
@param adux1050 Chip structure to set the power mode to shutdown
@return Zero on success.
*/
static int adux1050_enable(struct adux1050_chip *adux1050)
{
	s16 ret;

	mutex_lock(&adux1050->mutex);
#ifdef CONFIG_USE_FILP
#ifndef CONFIG_EVAL
	adux1050_open_calibration(adux1050, ADUX1050_DISABLE);
#endif
#endif
	if (adux1050_flag) {
		adux1050_hw_init(adux1050);
		ADUX1050_INFO("sensor open by upper layer\n");
	} else
		ADUX1050_INFO("sensor open by resume\n");

#ifdef CONFIG_EVAL
	if (adux1050->power_mode_flag == ADUX1050_ENABLE) {
		ret = adux1050->write(adux1050->dev, CTRL_REG,
				&adux1050->ctrl_reg, DEF_WR);
		if (ret < DEF_WR) {
			ADUX1050_ERR("%s: write CTRL_REG failed, ret = %d\n",
					__func__, ret);
			return ret;
		}
		adux1050->power_mode_flag = ADUX1050_DISABLE;
	}
#endif

	mutex_unlock(&adux1050->mutex);
	return 0;
}

/**
* \fn void adux1050_calibration(struct work_struct *cal_work)
* ADUX1050 Calibration routine
@param cal_work	Pointer to "calib_work", member of ADUX1050 chip structure
@return void
 */
static void adux1050_calibration(struct work_struct *cal_work)
{
	struct adux1050_chip *adux1050 =
		container_of(cal_work, struct adux1050_chip, calib_work);
	u8 slp_time_flag = 0;
	s32 err = 0;
	u16 data;
	u16 power_ctrl;
	u16 lp_cnt;
	u16 slp_time = 0;
	u16 s_t = 0;
	u16 cv_time_ctrl = 0;
	u16 cin_range = 0;
	u16 cur_power = 0;
	u32 start_time = jiffies;
	s16 ret;

	mutex_lock(&adux1050->mutex);
	ret = adux1050->read(adux1050->dev, INT_CTRL_REG,
		       &adux1050->dac_calib.enable_setting, DEF_WR);
	if (ret < DEF_WR) {
		ADUX1050_ERR("%s: read INT_CTRL_REG failed, line = %d, ret = %d\n",
				__func__, __LINE__, ret);
		return;
	}
	/*
	 * Disable interrupt and digital offset
	 */
	data = DISABLE_DEV_INT | adux1050->int_pol;
	ret = adux1050->write(adux1050->dev, INT_CTRL_REG, &data, DEF_WR);
	if (ret < DEF_WR) {
		ADUX1050_ERR("%s: write INT_CTRL_REG failed, line = %d, ret = %d\n",
				__func__, __LINE__, ret);
		return;
	}

	adux1050->dac_calib.cal_flags = CAL_RET_PENDING;
	if (adux1050->dac_calib.action_flag) {
		ADUX1050_DEBUG("\n\n%s CALIB STARTED\n\n", __func__);
		slp_time = get_conv_time(adux1050, CONV_DELAY_TIME);
		ret = adux1050->read(adux1050->dev, CONV_TIME_CTRL_REG,
			       &cv_time_ctrl, DEF_WR);
		if (ret < DEF_WR) {
			ADUX1050_ERR("%s: read CONV_TIME_CTRL_REG failed, line = %d, ret = %d\n",
					__func__, __LINE__, ret);
			return;
		}

		if (CHECK_MIN_TIME(cv_time_ctrl)) {
			data = SET_MIN_TIME(cv_time_ctrl);
			ret = adux1050->write(adux1050->dev, CONV_TIME_CTRL_REG,
					&data, DEF_WR);
			if (ret < DEF_WR) {
				ADUX1050_ERR("%s: write CONV_TIME_CTRL_REG failed, line = %d, ret = %d\n",
						__func__, __LINE__, ret);
				return;
			}
			slp_time_flag = 1;
			ADUX1050_DEBUG("%s Check Min time %x\n", __func__, data);
		}

		ret = adux1050->read(adux1050->dev, CTRL_REG, &power_ctrl, DEF_WR);
		if (ret < DEF_WR) {
			ADUX1050_ERR("%s: read CTRL_REG failed, line = %d, ret = %d\n",
					__func__, __LINE__, ret);
			return;
		}

		if (GET_PWR_MODE(power_ctrl) != PWR_FULL_POWER) {
			cur_power = SET_PWR_MODE(power_ctrl, PWR_FULL_POWER);
			ret = adux1050->write(adux1050->dev, CTRL_REG,
					&cur_power, DEF_WR);
			if (ret < DEF_WR) {
				ADUX1050_ERR("%s: write CTRL_REG failed, line = %d, ret = %d\n",
						__func__, __LINE__, ret);
				return;
			}

			slp_time_flag = slp_time_flag | 0x2;
			slp_time += get_conv_time(adux1050, CONV_TIME);
			ADUX1050_INFO("Mode change %d\n", slp_time);
		} else {
			cur_power = power_ctrl;
		}
		cin_range = GET_CIN_RANGE(cur_power);
		if (cin_range != PICO_5) {
			cur_power = SET_CIN_RANGE(cur_power, PICO_5);
			ret = adux1050->write(adux1050->dev, CTRL_REG,
					&cur_power, DEF_WR);
			if (ret < DEF_WR) {
				ADUX1050_ERR("%s: write CTRL_REG failed, line = %d, ret = %d\n",
						__func__, __LINE__, ret);
				return;
			}
		}
		if (slp_time_flag) {
			ADUX1050_DEBUG("Calib sleep %d\n", slp_time);
			msleep(slp_time);
		}
		err = do_dac_compensation(adux1050, cur_power,
					  cin_range, &s_t);
		ADUX1050_DEBUG("calibration return  %d\n", err);
	} else {
		adux1050->dac_calib.cal_flags = CAL_RET_NONE;
		for (lp_cnt = 0; lp_cnt < TOTAL_STG; lp_cnt++) {
			adux1050->pdata->cal_fact_base[lp_cnt] = 0;
			adux1050->pdata->cal_offset[lp_cnt] = 0;
			adux1050->pdata->digi_offset[lp_cnt] = 0;
			adux1050->pdata->stg_cfg[lp_cnt] = 0;
		}
	}

#ifdef CONFIG_USE_FILP
	err = save_calib_val_filp(adux1050);
	if (err)
		ADUX1050_INFO("calibration save failed [%d]\n", err);
#endif
	if (adux1050->dac_calib.action_flag) {
		/* If calibration succeed set baseline by using force calib*/
		if (adux1050->dac_calib.cal_flags == CAL_RET_SUCCESS)
			adux1050_force_cal(adux1050, s_t);
		/* Restore the prv conversion time settings */
		if (slp_time_flag & 1) {
			ret = adux1050->write(adux1050->dev, CONV_TIME_CTRL_REG,
					&cv_time_ctrl, DEF_WR);
			if (ret < DEF_WR) {
				ADUX1050_ERR("%s: write CONV_TIME_CTRL_REG, line = %d, ret = %d\n",
						__func__, __LINE__, ret);
				return;
			}
		}
		/* Restore the previous power Setting */
		if (slp_time_flag & 2) {
			ret = adux1050->write(adux1050->dev, CTRL_REG,
					&power_ctrl, DEF_WR);
			if (ret < DEF_WR) {
				ADUX1050_ERR("%s: write CTRL_REG, line = %d, ret = %d\n",
						__func__, __LINE__, ret);
				return;
			}
		}
	}
	data = adux1050->dac_calib.enable_setting | adux1050->int_pol;
	ret = adux1050->write(adux1050->dev, INT_CTRL_REG, &data, DEF_WR);
	if (ret < DEF_WR) {
		ADUX1050_ERR("%s: write INT_CTRL_REG, line = %d, ret = %d\n",
				__func__, __LINE__, ret);
		return;
	}
	ADUX1050_DEBUG("\n\nCALIB ENDS Status %d Time %d ms\n\n",
		 adux1050->dac_calib.cal_flags,
		 jiffies_to_msecs(jiffies - start_time));
	mutex_unlock(&adux1050->mutex);
	if (adux1050->dev_enable == ADUX1050_DISABLE)
		adux1050_disable(adux1050);

}

/*called by set_enable*/
static int __set_enable(struct adux1050_chip *adux1050, int val)
{
	s16 ret;

	ADUX1050_DEBUG("%s - prv_flag %d curr %d\n", __func__, adux1050->dev_enable, val);
	if (adux1050->dev_enable == val) {
		ADUX1050_INFO("%s - Device already in %s state\n",
			 __func__, val ? "enable":"disable");
		return 0;
	}
	if (val == ADUX1050_ENABLE) {
		adux1050_flag = true;
		adux1050->dev_enable = val;
#ifdef CONFIG_ADUX1050_POLL
		adux1050_enable(adux1050);
		wake_up_process(adux1050->polling_task);
#else
		ret = regulator_enable(adux1050->reg_vdd);
		if (ret) {
			ADUX1050_ERR("%s: could not enable vdd voltage\n", __func__);
			return ret;
		}

		adux1050_enable(adux1050);
		enable_irq(adux1050->irq);
		enable_irq_wake(adux1050->irq);

#endif
		ADUX1050_INFO("ADUX1050 is enabled\n");
	} else {
		adux1050->dev_enable = val;
#ifdef CONFIG_ADUX1050_POLL

#else
		disable_irq_wake(adux1050->irq);
		disable_irq(adux1050->irq);
#endif
		cancel_work_sync(&adux1050->work);
		cancel_work_sync(&adux1050->calib_work);
		adux1050_disable(adux1050);
		ret = regulator_disable(adux1050->reg_vdd);
		if (ret) {
			ADUX1050_ERR("%s: could not disable vdd voltage\n", __func__);
			return ret;
		}
		ADUX1050_INFO("ADUX1050 is Disabled\n");
	}

	return 0;
}

/*called by store_enable and sensor_class*/
static int set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct adux1050_chip *adux1050 = container_of(sensors_cdev,
			struct adux1050_chip, cdev);
	return __set_enable(adux1050, enable);
}
/**
* \fn static ssize_t store_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
* This Function is used to enable or to disable the device. The Sysfs attribute
is given as "enable", writing a '0' Disables the device.
While writing '1' , enables the device.
@param dev The Device Id structure(linux standard argument)
@param attr Standard Linux Device attributes to the ADUX1050.
@param buf The buffer which contains the data.
@param count The count of bytes to be transfered to the Device.
\note This is evoked upon an echo request in /sys/../<Device> region.
\note This also prints the results in the console for the user.
@return count of data written.
*/
static ssize_t store_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
//delete lines

	s32 err;
	u16 val;

	err = kstrtou16(buf, 0, &val);

	if (err < 0) {
		ADUX1050_ERR("%s,kstrtoint failed\n", __func__);
		return err;
	}
	if (val > 1) {
		ADUX1050_ERR("%s Invalid- Enable:1 Disable:0\n",
			 __func__);
//delete lines
		goto exit;
	}
	__set_enable(adux1050, val);
exit:
	return count;
}

/**
* \fn static ssize_t show_enable(struct device *dev,
			      struct device_attribute *attr, char *buf)
* This Function is used to show the status of the driver
 Status '1' signifies the device is ENABLED,
 while the status '0' signifies a DISABLED device.
@param dev The Device Id structure(linux standard argument)
@param attr standard Linux Device attributes to the ADUX1050.
@param buf The buffer to store the data to be written.
\note This is evoked upon an cat request in /sys/../<Device> region.
\note This also prints the results in the console for the user.
@return The count of data written.
*/
static ssize_t show_enable(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct adux1050_chip  *adux1050 = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", adux1050->dev_enable);
}

/**
* \fn static ssize_t show_dumpregs(struct device *dev,
			      struct device_attribute *attr, char *buf)
* This Function is used for dumping the registers value of the ADUX1050.
@param dev The Device Id structure(linux standard argument)
@param attr standard Linux Device attributes to the ADUX1050
@param buf The buffer to store the data to be written
@param count The count of bytes to be transfered to the Device
\note This is evoked upon an cat request in /sys/../<Device> region.
\note This also prints the results in the console for the user.
@return count of data written
*/
static ssize_t show_dumpregs(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	u16 u16temp[MAX_ADUX1050_WR_LEN];
	u32 u32_lpcnt = 0;
	u16 ret = 0;
	s16 err;

	mutex_lock(&adux1050->mutex);
	ADUX1050_INFO("Global control registers\n");
	err = adux1050->read(adux1050->dev, DEV_ID_REG, u16temp, GLOBAL_REG_CNT);
	if (err < DEF_WR) {
		ADUX1050_ERR("%s: read DEV_ID_REG failed, line = %d, ret = %d\n",
				__func__, __LINE__, err);
		return err;
	}

	for (; u32_lpcnt < GLOBAL_REG_CNT; u32_lpcnt++) {
		ADUX1050_INFO("Reg 0X%x val 0x%x\n", u32_lpcnt, u16temp[u32_lpcnt]);
		ret += sprintf(buf + ret, "%4x ", u16temp[u32_lpcnt]);
	}
	ADUX1050_INFO("Stage config registers\n");

	err = adux1050->read(adux1050->dev, CONFIG_STG0_REG, u16temp, STG_CNF_CNT);
	if (err < DEF_WR) {
		ADUX1050_ERR("%s: read CONFIG_STG0_REG failed, line = %d, ret = %d\n",
				__func__, __LINE__, err);
		return err;
	}

	for (u32_lpcnt = CONFIG_STG0_REG;
		u32_lpcnt <= GET_HYS_REG(STG_THREE) ; u32_lpcnt++) {
		ADUX1050_INFO("Reg 0X%x val 0x%x\n", u32_lpcnt, u16temp[u32_lpcnt-CONFIG_STG0_REG]);
		ret += sprintf(buf + ret, "%4x ",
				u16temp[u32_lpcnt-CONFIG_STG0_REG]);
	}
	ADUX1050_INFO("Result/Base/p2p registers\n");

	err = adux1050->read(adux1050->dev, INT_STATUS_REG, u16temp, STATUS_REG_CNT);
	if (err < DEF_WR) {
		ADUX1050_ERR("%s: read INT_STATUS_REG failed, line = %d, ret = %d\n",
				__func__, __LINE__, err);
		return err;
	}

	for (u32_lpcnt = INT_STATUS_REG;
	     u32_lpcnt <= PROX_STATUS_REG; u32_lpcnt++) {
		ADUX1050_INFO("Reg 0X%x val 0x%x\n",
			 u32_lpcnt, u16temp[u32_lpcnt-INT_STATUS_REG]);
		ret += sprintf(buf + ret, "%4x ",
			       u16temp[u32_lpcnt-INT_STATUS_REG]);
	}
	mutex_unlock(&adux1050->mutex);
	return ret;
}

static ssize_t show_prox_status(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	unsigned short status, ret;
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);

	ret = adux1050->read(adux1050->dev, INT_STATUS_REG, &status, DEF_WR);
	if (ret < DEF_WR) {
		ADUX1050_ERR("%s: read INT_STATUS_REG failed, line = %d, ret = %d\n",
				__func__, __LINE__, ret);
		return ret;
	}

	if (status & 0x1)
		status = 1;
	else if (status & 0x10)
		status = 2;
	else
		status = 0;

	return sprintf(buf, "%d\n", status);
}
/**
* \fn static ssize_t adux1050_name_show(struct device *dev,
struct device_attribute *attr, char *buf)
* This is used to display the device name of the chipset.
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
\note This is evoked upon an cat request in /sys/../<Device> region.
@return Returns the size of the output buffer with the on/off status
*/
static ssize_t adux1050_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", DEVICE_NAME);
}

/**
* \fn static ssize_t adux1050_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
* This is used to display the vendor name of the device.
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
\note This is evoked upon an cat request in /sys/../<Device> region.
@return Returns the size of the output buffer with the on/off status
*/
static ssize_t adux1050_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", VENDOR_NAME);
}

/**
* \fn static ssize_t adux1050_raw_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
* This is used to display the Raw CDC data of a stage
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
\note This is evoked upon an cat request in /sys/../<Device> region.
@return Returns the size of the raw data of all the stages
*/
static ssize_t adux1050_raw_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	u16 temp_cdc = 0;
	s16 ret = 0;
	s16 err;

	if (!adux1050->dev_enable) {
		ADUX1050_INFO("Device is not enabled,return\n");
		goto data_show_err;
	}
	mutex_lock(&adux1050->mutex);
	err = adux1050->read(adux1050->dev, GET_RESULT_REG(adux1050->stg_raw_cdc),
		       &temp_cdc, DEF_WR);
	if (err < DEF_WR) {
		ADUX1050_ERR("%s: read GET_RESULT_REG(adux1050->stg_raw_cdc) failed, line = %d, ret = %d\n",
				__func__, __LINE__, err);
		return err;
	}
	ADUX1050_DEBUG("%s, STG_NO - %d : raw_data - %x\n", __func__,
			    adux1050->stg_raw_cdc, temp_cdc);
	ret = snprintf(buf, PAGE_SIZE, "0x%04x ", temp_cdc);
	mutex_unlock(&adux1050->mutex);
data_show_err:
	return ret;
}

/**
* \fn adux1050_raw_data_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
* This is used to set the stage number to display CDC raw data.
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer which contains the data.
@param count The count of bytes to be transfered to the Device.
\note This is evoked upon an echo request in /sys/../<Device> region.
@return Returns the count of the raw data value of a single stage
*/
static ssize_t adux1050_raw_data_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	s32 err;
	u16 val;

	err = kstrtou16(buf, 0, &val);
	if (err) {
		ADUX1050_ERR("%s, kstrtoint failed\n", __func__);
		return err;
	}
	if (val < TOTAL_STG)
		adux1050->stg_raw_cdc = val;
	else
		ADUX1050_ERR("%s, Invalid input %d\n", __func__, val);

	return count;
}

/**
* \fn adux1050_send_event_show(struct device *dev,
	struct device_attribute *attr, char *buf)
* This is used to display the send event status of the driver.
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
\note This is evoked upon an cat request in /sys/../<Device> region.
@return Returns the size of the output buffer with the send event status
*/
static ssize_t adux1050_send_event_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", adux1050->send_event);
}

/**
* \fn adux1050_send_event_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
* This is used to set the send event flag in the driver
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
@param size The count of bytes to be transfered to the Buffer
\note This is evoked upon an write request in the /sys/.../<Device> region.
@return Returns the size of the output buffer
 */
static ssize_t adux1050_send_event_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	s32 val, err;

	err = kstrtoint(buf, 0, &val);
	if (err < 0) {
		ADUX1050_ERR("%s, kstrtoint failed\n", __func__);
		return err;
	}
	if ((val == ADUX1050_ENABLE) || (val == ADUX1050_DISABLE))
		adux1050->send_event = (unsigned char)val;
	else
		ADUX1050_ERR("%s - Invalid input %d\n", __func__, val);
	return count;
}

/**
This is used to show the threshold status of a stage
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
\note This is evoked upon an read request in the /sys/.../<Device> region.
@return Returns the size of the output buffer
*/
static ssize_t adux1050_threshold_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	u16 temp_base = 0;
	u16 temp_ht = 0;
	u16 temp_lt = 0;
	u32 htresult = 0;
	s32 ltresult = 0;
	s16 ret = 0;
	s16 err;

	if (!adux1050->dev_enable) {
		ADUX1050_INFO("Device is not enabled,return\n");
		goto err;
	}
	mutex_lock(&adux1050->mutex);
	err = adux1050->read(adux1050->dev,
		       GET_BASE_LINE_REG(adux1050->stg_threshold),
		       &temp_base, DEF_WR);
	if (err < DEF_WR) {
		ADUX1050_ERR("%s: read GET_BASE_LINE_REG failed, line = %d, ret = %d\n",
				__func__, __LINE__, err);
		return err;
	}
	err = adux1050->read(adux1050->dev, GET_HIGH_TH_REG(adux1050->stg_threshold),
		       &temp_ht, DEF_WR);
	if (err < DEF_WR) {
		ADUX1050_ERR("%s: read GET_HIGH_TH_REG failed, line = %d, ret = %d\n",
				__func__, __LINE__, err);
		return err;
	}
	err = adux1050->read(adux1050->dev, GET_LOW_TH_REG(adux1050->stg_threshold),
		       &temp_lt, DEF_WR);
	if (err < DEF_WR) {
		ADUX1050_ERR("%s: read GET_LOW_TH_REG failed, line = %d, ret = %d\n",
				__func__, __LINE__, err);
		return err;
	}

	mutex_unlock(&adux1050->mutex);
	htresult = temp_base + temp_ht;
	if (htresult >= FULL_SCALE_VALUE)
		htresult = FULL_SCALE_VALUE;

	ltresult = temp_base - temp_lt;
	if (ltresult <= 0)
		ltresult = 0;

	ADUX1050_DEBUG("%s, STG_NO - %d : BS: %x, HT - %x : LT - %x\n",
			    __func__, adux1050->stg_threshold,
			    temp_base, temp_ht, temp_lt);
	ret = snprintf(buf, PAGE_SIZE, "0x%04x 0x%04x ",
		       htresult, ltresult);

err:
	return ret;
}

/**
This is used to set the stage number to show the threhold details of that stage

@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
@param count The count of bytes to be transfered to the Buffer
\note This is evoked upon an read request in the /sys/.../<Device> region.
@return Returns the raw data value of a single stage
*/
static ssize_t adux1050_theshold_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	s32 err;
	u16 val;

	err = kstrtou16(buf, 0, &val);
	if (err) {
		ADUX1050_ERR("%s, kstrtoint failed\n", __func__);
		return err;
	}
	if (val < TOTAL_STG)
		adux1050->stg_threshold = val;
	else
		ADUX1050_ERR("%s, Invalid input %d\n", __func__, val);

	return count;
}

/**
This is used to check the DAC offset calibration work status from the driver
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
\note This is evoked upon an read request in the /sys/.../<Device> region.
@return Returns the size of the output buffer
*/
static ssize_t adux1050_calibration_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	s32 ret = 0;
	s32 stg_cnt = 0;
	struct adux1050_platform_data *lpdata = adux1050->pdata;

	if (adux1050->dac_calib.cal_flags == CAL_RET_SUCCESS) {
		ret = snprintf(buf + ret, PAGE_SIZE, "%1d %1d ",
			adux1050->dac_calib.cal_flags, adux1050->conn_stg_cnt);
		for (stg_cnt = 0; stg_cnt < adux1050->num_stages; stg_cnt++) {
			if (adux1050->stg_info[stg_cnt].status ==
			    CIN_CONNECTED) {
				/*The STATUS, Stage number, Target, Offset,
					SWAP_state, Digi_offset*/
				ADUX1050_DEBUG("%1d %1d 0x%04x 0x%04x 0x%04x 0x%04x ",
					 adux1050->dac_calib.cal_flags, stg_cnt,
					 lpdata->cal_fact_base[stg_cnt],
					 lpdata->cal_offset[stg_cnt],
					 lpdata->digi_offset[stg_cnt],
					 lpdata->stg_cfg[stg_cnt]);
				ret +=
				snprintf(buf + ret, PAGE_SIZE,
					 "%1d 0x%04x 0x%04x 0x%04x 0x%04x ",
					 (u16)stg_cnt,
					 (u16)lpdata->cal_fact_base[stg_cnt],
					 (u16)lpdata->cal_offset[stg_cnt],
					 (u16)lpdata->digi_offset[stg_cnt],
					 (u16)lpdata->stg_cfg[stg_cnt]);
			}
		}
		ret--;
	} else {
		ret = snprintf(buf, PAGE_SIZE, "%1d\n",
			       (u32)adux1050->dac_calib.cal_flags);
	}
	return ret;
}

/**
This is used to call the DAC offset calibration in the driver
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
@param size The count of bytes to be transfered to the Buffer
\note This is evoked upon an write request in the /sys/.../<Device> region.
@return Returns the size of the input buffer
 */
static ssize_t adux1050_calibration_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	s32 err;
	u32 val;

	err = kstrtoint(buf, 0, &val);

	if (err < 0) {
		ADUX1050_ERR("%s, kstrtoint failed\n", __func__);
		return err;
	}

	if ((val == ADUX1050_ENABLE) || (val == ADUX1050_DISABLE)) {
		if (adux1050->dev_enable != ADUX1050_ENABLE) {
			mutex_lock(&adux1050->mutex);
			adux1050_hw_init(adux1050);
			mutex_unlock(&adux1050->mutex);
			ADUX1050_INFO("%s, hw_init_done\n", __func__);
		}
		mutex_lock(&adux1050->mutex);
		adux1050->dac_calib.action_flag = (u8)val;
		adux1050->dac_calib.cal_flags = CAL_RET_NONE;
		ADUX1050_INFO("%s, Calling schedule_work\n", __func__);
		mutex_unlock(&adux1050->mutex);
		schedule_work(&adux1050->calib_work);
	} else {
		ADUX1050_ERR("%s, Invalid input %d\n",
			__func__, val);
	}
	return size;
}

/**
Command parsing function for echo/cat commands from command prompt.
This function is called when ever the User tries an echo / cat command
to the /../sysfs/<Device> especially during read/write registers.

@return void Returns Nothing
@see store_reg_read
 */
static int cmd_parsing(const char *buf, u16 *addr, u16 *cnt,
		       u16 *data, u16 data_limit)
{
	char **bp = (char **)&buf;
	u8 *token, minus, parsing_cnt = 0;
	u16 val;
	s32 ret;
	s32 pos;

	data_limit = data_limit + 2;
	while ((token = strsep(bp, SPACE_CHAR))) {
		pos = 0;
		minus = false;
		if ((char)token[pos] == MINUS_CHAR) {
			minus = true;
			pos++;
		}

		ret = kstrtou16(&token[pos], 0, (unsigned short *)&val);
		if (ret)
			return ret;
		if ((parsing_cnt == 0) & (val > HIGHEST_READ_REG))
			return -ERANGE;
		if (minus)
			val *= MINUS_VAL;

		switch (parsing_cnt) {
		case PARSE_ADDR:
			*addr = val;
			break;
		case PARSE_CNT:
			*cnt  = val;
			break;
		default:
		case PARSE_DATA:
			*data = val;
			data++;
			break;
		}
		parsing_cnt++;
		if (parsing_cnt > data_limit)
			return parsing_cnt;
	}
	return parsing_cnt;
}

/**
  This is used to update the calib output to the device from the application space
@return The Size of the Read data
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
@param size The count of bytes to be transfered to the Buffer
\note This is evoked upon an wr request in the /sys/.../<Device> region.
@return Returns the size of the data handled
*/
static ssize_t adux1050_update_calib_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	s32 err;
	u16 data[FILP_PARAM_CNT * TOTAL_STG];
	u16 status;
	u16 cnt;

	err = cmd_parsing(buf, &status, &cnt, data,
			  (FILP_PARAM_CNT * TOTAL_STG));
	if (err < 0) {
		ADUX1050_ERR("%s,kstrtos16 failed %x\n",
			__func__, err);
		return err;
	}

	mutex_lock(&adux1050->mutex);
	if (status == 1) {
#ifdef CONFIG_USE_FILP
#ifdef CONFIG_EVAL
		adux1050_open_calibration(adux1050, ADUX1050_ENABLE);
#endif
#else
		ADUX1050_ERR("%s , Invalid option\n", __func__);
#endif

	} else if ((status == CAL_RET_SUCCESS) && (cnt <= TOTAL_STG)) {
			if ((cnt * FILP_PARAM_CNT + sizeof(u16)) != err) {
				ADUX1050_ERR("%s,insuff data(%d)\n",
					__func__, err);
				goto err_data_cnt;
			}
			update_calib_settings(adux1050, cnt, data, ADUX1050_ENABLE, ZERO_VAL);
	} else {/*TODO: Case 3 for File Write*/
		ADUX1050_ERR("%s, Error for data val(%d)\n", __func__, err);
	}
err_data_cnt:
	mutex_unlock(&adux1050->mutex);
	return size;
}

/**
  This is used to show the DAC calibration routine baseline CDC value.
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
\note This is evoked upon an read request in the /sys/.../<Device> region.
@return Returns the Size of the output buffer
*/
static ssize_t calib_target_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	struct adux1050_platform_data *pdata = adux1050->pdata;
	return snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d\n", pdata->req_stg0_base,
			pdata->req_stg1_base, pdata->req_stg2_base,
			pdata->req_stg3_base);
}

/**
  This is used to set the DAC calibration offset baseline CDC value.
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
@param size The count of bytes to be transfered to the Buffer
\note This is evoked upon an write request in the /sys/.../<Device> region.
@return Returns the size of the data handled
*/
static ssize_t calib_target_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	u16 stg_num = 0;
	u16 val = 0;
	s32 err = 0;
	u16 dummy;

	err = cmd_parsing(buf, &stg_num, &val, &dummy, 0);
	if (err < 0) {
		ADUX1050_ERR("%s, kstrtos16 failed\n", __func__);
		return err;
	}
	if ((val <= MAX_CALIB_TARGET) && (val >= MIN_CALIB_TARGET))
		switch (stg_num) {
		case STG_ZERO:
			adux1050->pdata->req_stg0_base = val;
			break;
		case STG_ONE:
			adux1050->pdata->req_stg1_base = val;
			break;
		case STG_TWO:
			adux1050->pdata->req_stg2_base = val;
			break;
		case STG_THREE:
			adux1050->pdata->req_stg3_base = val;
			break;
		default:
		    ADUX1050_ERR("%s,Invalid stg no %d\n",
			     __func__, stg_num);
	} else {
		ADUX1050_ERR("[%d,%d] Limit exceeded(%d)\n",
			MAX_CALIB_TARGET, MIN_CALIB_TARGET, val);
	}

	return size;
}

#ifdef CONFIG_EVAL

/**
This is used to get register address whose data is to be read and
count of data to be read via sysfs
This function Reads the value at the Device's Register for the i2c client
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
@param count The count of bytes to be transfered to the Buffer
\note This called when the user requires to read the configuration
\note This is evoked upon an echo request in the /sys/.../<Device> region.
\note it hold the register address to be read.
@return The Size of the Read Register 0 if not read
@see cmd_parsing
*/

static ssize_t store_reg_read(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	s32 ret;
	u16 addr;
	u16 cnt = 0;
	u16 val = 0;
	u16 lp_cnt = 0;
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);

	mutex_lock(&adux1050->mutex);
	adux1050->stored_data_cnt = 0;

	ret = cmd_parsing(buf, &addr, &cnt, &val, 0);
	if (cnt == 0) {
		ADUX1050_ERR("[ADUX1050]: Invalid COM/ARG\n");
		goto error;
	} else if (ret == -ERANGE || (addr+cnt > HIGHEST_READ_REG+1)) {
		ADUX1050_ERR("[ADUX1050]: Values not in RANGE\n");
		goto error;
	} else if ((ret == -EINVAL) || (cnt > MAX_ADUX1050_WR_LEN)) {
		ADUX1050_ERR("[ADUX1050]: Invalid COMD/ARG\n");
		goto error;
	} else {
		val = 0;
	}
	memset(adux1050->stored_reg_data, 0, sizeof(adux1050->stored_reg_data));

	ret = adux1050->read(adux1050->dev, addr, adux1050->stored_reg_data, cnt);
	if (ret < DEF_WR) {
		ADUX1050_ERR("%s: read addr failed, line = %d, ret = %d\n",
				__func__, __LINE__, ret);
		return ret;
	}
	adux1050->stored_data_cnt = cnt;
	for (lp_cnt = 0; lp_cnt < adux1050->stored_data_cnt; lp_cnt++) {
		ADUX1050_DEBUG("Reg Read cmd:reg 0x%04x Data 0x%04x\n", addr + lp_cnt,
				    adux1050->stored_reg_data[lp_cnt]);
	}

error:
	mutex_unlock(&adux1050->mutex);
	return count;
}

/**
This is used to read the data of the register address via sysfs sent to reg_read
This functions Reads the value at the Device's Register for the given
client and Prints it in the output window
@param dev The Device ID structure(linux standard argument)
@param attr standard linux device attributes to the ADUX1050
@param buf The buffer to store the Read data
\note This is evoked upon an cat request in the /sys/.../<Device> region.
@return The Size of the read data, 0 if not read

@see dev_get_drvdata
@see store_reg_read
*/
static ssize_t show_reg_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	s32 val = 0, lp_cnt = 0;
	struct adux1050_chip  *adux1050 = dev_get_drvdata(dev);
	for (lp_cnt = 0; lp_cnt < adux1050->stored_data_cnt; lp_cnt++) {
		val += snprintf(buf + val, PAGE_SIZE, "0x%x ", adux1050->stored_reg_data[lp_cnt]);
	}
	return val;
}

/**
This is used to write data to a register through i2c.
This functions Writes the value of the buffer to the given client
provided the count value to write
@param dev The device ID structure(linux standard argument)
@param attr standard linux device attributes to the ADUX1050
@param count The number of bytes to write from the buffer
@param buf The buffer to store the Read data
\note This is used to store the register address to write the data.
\note This is evoked upon an echo request in the /sys/.../<Device> region.
\note This also prints the command received before writing the Registers.
@return The Size of the writen Data, 0 if not writen
*/

static ssize_t store_reg_write(struct device *dev,
	 struct device_attribute *attr, const char *buf, size_t count)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	u16 addr, cnt;
	u16 wr_data[MAX_ADUX1050_WR_LEN], loop_cnt;
	s32 ret;

	mutex_lock(&adux1050->mutex);
	ret = cmd_parsing(buf, &addr, &cnt, &wr_data[0], MAX_ADUX1050_WR_LEN);
	if (ret == -ERANGE || (ret == -EINVAL)) {
		ADUX1050_ERR("%s - Values not in RANGE\n", __func__);
		goto error;
	} else if ((addr >= BASELINE_STG0_REG &&
		    addr <= GET_BASE_LINE_REG(STG_THREE)) &&
		   (addr+cnt <= GET_BASE_LINE_REG(STG_THREE) + 1)) {
		ADUX1050_DEBUG("%s Baseline write register\n", __func__);
	} else if ((addr + cnt) > (HIGHEST_WR_ACCESS + 1)) {
		ADUX1050_ERR("%s - Addr reaches RD only regs\n", __func__);
		goto error;
	} else if (cnt > MAX_ADUX1050_WR_LEN) {
		ADUX1050_ERR("%s - Max write length\n", __func__);
		goto error;
	}
	ADUX1050_DEBUG("Register Write command :reg= 0x%x, size= %d\n",
			    addr, cnt);

	for (loop_cnt = 0; loop_cnt < cnt; loop_cnt++) {

		/* Storing local of the values before writing to registers */
		adux1050->reg[addr+loop_cnt].wr_flag = ADUX1050_ENABLE;
		adux1050->reg[addr+loop_cnt].value = wr_data[loop_cnt];

		/* Conditions to be checked
		   1.SW RESET, FORCE CALIB, AUTO_Threshold, power state ... */
		if ((addr + loop_cnt) == CTRL_REG) {
			/* Clearing SW Reset bit */
			if (CHK_SW_RESET_EN(wr_data[loop_cnt])) {
				ADUX1050_WARN("S/W reset not allowed");
				ADUX1050_WARN("- Use reset sysfs!!!\n");
				wr_data[loop_cnt] =
					CLR_SW_RESET_EN(wr_data[loop_cnt]);
			}
			/*Device is not allowed to wakeup
			  during drv disable state*/
			if (adux1050->dev_enable == ADUX1050_DISABLE) {
				if (GET_PWR_MODE(wr_data[loop_cnt]) !=
				    PWR_STAND_BY) {
					/* PWR saved, used in next enable*/
					adux1050->ctrl_reg = wr_data[loop_cnt];
					adux1050->power_mode_flag =
							ADUX1050_ENABLE;
					wr_data[loop_cnt] =
				SET_PWR_MODE(wr_data[loop_cnt], PWR_STAND_BY);
				}
			}
		} else if (addr + loop_cnt == BASELINE_CTRL_REG) {
			if (CHK_FORCE_CALIB_EN(wr_data[loop_cnt])) {
				ADUX1050_WARN("FORCE CAL not allowed-");
				ADUX1050_WARN("Use force calib sysfs\n");
				wr_data[loop_cnt] =
					CLR_FORCE_CALIB_EN(wr_data[loop_cnt]);
			}
		}
		ADUX1050_INFO("DATA = 0x%04X\n", wr_data[loop_cnt]);
	}
	ret = adux1050->write(adux1050->dev, addr, wr_data, cnt);
	if (ret < cnt) {
		ADUX1050_ERR("%s: write addr failed, line = %d, ret = %d\n",
				__func__, __LINE__, ret);
		return ret;
	}

error:
	mutex_unlock(&adux1050->mutex);
	return count;
}

/**
This is used to update the stage details upon every change in configuration
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
@param size The count of bytes to be transfered to the Buffer
\note This is evoked upon an write request in the /sys/.../<Device> region.
@return Returns the size of the data handled
*/
static ssize_t store_update_config(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	s32 err;
	u16 val;
	if (!adux1050->dev_enable) {
		ADUX1050_ERR("Device is not enabled, return\n");
		return count;
	}
	err = kstrtou16(buf, 0, &val);
	if (err < 0) {
		ADUX1050_ERR("%s, kstrtos16 failed\n", __func__);
		return err;
	}
	if (val == 1) {
			mutex_lock(&adux1050->mutex);
			getstageinfo(adux1050);
			mutex_unlock(&adux1050->mutex);
			msleep(adux1050->slp_time_conv_complete);
			adux1050->slp_time_conv_complete =
				get_conv_time(adux1050, TWICE_CONV_DELAY_TIME);
	} else {
		ADUX1050_ERR("Invalid Input %d\n", val);
	}

	return count;
}

/**
This is used to do Software reset of the device of ADUX1050
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
@param size The count of bytes to be transfered to the Buffer
\note This is evoked upon an write request in the /sys/.../<Device> region.
@return Returns the size of the data handled
*/
static ssize_t store_sw_reset(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	u16 ctrl_reg_val;
	s32 err;
	u16 val;
	s16 ret;
	err = kstrtou16(buf, 0, &val);

	if (err < 0) {
		ADUX1050_ERR("%s - kstrtos16 failed\n", __func__);
		return err;
	}
	if (val == 1) {
		mutex_lock(&adux1050->mutex);
		/* SW reset is enabled in ctrl register */
		ret = adux1050->read(adux1050->dev, CTRL_REG, &ctrl_reg_val, DEF_WR);
		if (ret < DEF_WR) {
			ADUX1050_ERR("%s: read CTRL_REG failed, line = %d, ret = %d\n",
					__func__, __LINE__, ret);
			return ret;
		}

		ctrl_reg_val = SET_SW_RESET_EN(ctrl_reg_val);
		ret = adux1050->write(adux1050->dev, CTRL_REG,
				&ctrl_reg_val, DEF_WR);
		if (ret < DEF_WR) {
			ADUX1050_ERR("%s: write CTRL_REG failed, line = %d, ret = %d\n",
					__func__, __LINE__, ret);
			return ret;
		}

		msleep(FORCE_CALIB_SLEEP_TIME);
		/*Device is put to disable mode */
		if (ADUX1050_ENABLE == adux1050->dev_enable) {
			adux1050->dev_enable = ADUX1050_DISABLE;
#ifndef CONFIG_ADUX1050_POLL
			disable_irq_wake(adux1050->irq);
			disable_irq(adux1050->irq);
#endif
		}
		mutex_unlock(&adux1050->mutex);
	} else {
		ADUX1050_ERR("%s - Invalid input %d\n", __func__, val);
	}

	return count;
}

/**
This is used to do factory calibration of the ADUX1050
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
@param size The count of bytes to be transfered to the Buffer
\note This is evoked upon an write request in the /sys/.../<Device> region.
@return Returns the size of the data handled
*/
static ssize_t store_force_calib(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	u16 baseline_ctrl_val;
	s32 err;
	u16 val;
	s16 ret;
	err = kstrtou16(buf, 0, &val);

	if (err < 0) {
		ADUX1050_ERR("%s, kstrtos16 failed\n", __func__);
		return err;
	}
	if (val == 1) {
		/* Force Calibration is enabled in Baseline ctrl register */
		/*TODO: Check AUTO-TH is enable or not */
		/*TODO: Can the Force calibratin be done
		   when the device is at DISABLED state?*/
		mutex_lock(&adux1050->mutex);
		ret = adux1050->read(adux1050->dev, BASELINE_CTRL_REG,
			       &baseline_ctrl_val, DEF_WR);
		if (ret < DEF_WR) {
			ADUX1050_ERR("%s: read BASELINE_CTRL_REG failed, line = %d, ret = %d\n",
					__func__, __LINE__, ret);
			return ret;
		}

		baseline_ctrl_val = SET_FORCE_CALIB_EN(baseline_ctrl_val);
		ret = adux1050->write(adux1050->dev, BASELINE_CTRL_REG,
				&baseline_ctrl_val, DEF_WR);
		if (ret < DEF_WR) {
			ADUX1050_ERR("%s: write BASELINE_CTRL_REG failed, line = %d, ret = %d\n",
					__func__, __LINE__, ret);
			return ret;
		}
		msleep(FORCE_CALIB_SLEEP_TIME);
		queue_work(adux1050->det_workqueue, &adux1050->work);
		mutex_unlock(&adux1050->mutex);
	} else {
		ADUX1050_ERR("%s,Value is invalid\n", __func__);
	}
	return count;
}

#endif
#ifdef CONFIG_USE_FILP
/**
This is used to perform filp based register configuration extraction
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
@param size The count of bytes to be transfered to the Buffer
\note This is evoked upon an write request in the /sys/.../<Device> region.
@return Returns the size of the data handled
*/
static ssize_t store_adux1050_filp_config(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u16 val = 0;
	s32 err = 0;
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	err = kstrtou16(buf, 0, &val);
	if (val == 1) {
		mutex_lock(&adux1050->mutex);
		err = adux1050_filp_config(adux1050);
		if (err == 0)
			ADUX1050_INFO("FILP Extraction complete\n");

		if ((adux1050->dev_enable == ADUX1050_ENABLE) && (err == 0)) {
			adux1050_hw_init(adux1050);
			ADUX1050_INFO("H/W has been initialised with new Eval file\n");
		}
		mutex_unlock(&adux1050->mutex);
	} else {
			ADUX1050_ERR("Enter a valid Filp_val\n");
	}
	return count;
}
#endif

#ifdef CONFIG_ADUX1050_POLL

/**
This is used to show the poll delay used by the driver
@param dev The Device ID structure(linux standard argument)
@param attr standard linux device attributes to the ADUX1050
@param buf The buffer to store the Read data
\note This is evoked upon an cat request in the /sys/.../<Device> region.
@return The Size of the read data, 0 if not read
*/
static ssize_t show_poll_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct adux1050_chip  *adux1050 = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", adux1050->poll_delay);
}

/**
This is used to set the poll delay for the adux1050 kthread which is running
@param dev The Device Id structure
@param attr The Device attributes to the ADUX1050
@param buf The buffer to store the Read data
@param size The count of bytes to be transfered to the Buffer
\note This is evoked upon an write request in the /sys/.../<Device> region.
@return Returns the size of the data handled
*/
static ssize_t store_poll_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct adux1050_chip *adux1050 = dev_get_drvdata(dev);
	s32 err;
	u16 val;
	err = kstrtou16(buf, 0, &val);

	if (err < 0) {
		ADUX1050_ERR("kstrou16 is not done\n");
		return err;
	}
	if (val <= MIN_POLL_DELAY)
		val = MIN_POLL_DELAY;
	adux1050->poll_delay = val;
	return count;
}
#endif

/*
The sysfs attributes used in the driver follows
*/
/*--------------------------------------------------------------*/
static DEVICE_ATTR(adux1050_enable, S_IRUGO | S_IWUSR ,
	show_enable, store_enable);
/*--------------------------------------------------------------*/
static struct device_attribute dev_attr_sensor_name =
	__ATTR(adux1050_device_name, S_IRUSR | S_IRGRP,
	       adux1050_name_show, NULL);
static struct device_attribute dev_attr_sensor_vendor =
	__ATTR(adux1050_vendor, S_IRUSR | S_IRGRP,
	       adux1050_vendor_show, NULL);
static struct device_attribute dev_attr_sensor_raw_data =
	__ATTR(adux1050_raw_data, S_IRUGO | S_IWUSR ,
	       adux1050_raw_data_show, adux1050_raw_data_store);
static struct device_attribute dev_attr_sensor_send_event =
	__ATTR(adux1050_send_event, S_IRUGO | S_IWUSR ,
	       adux1050_send_event_show, adux1050_send_event_store);
static struct device_attribute dev_attr_sensor_threshold =
	__ATTR(adux1050_threshold, S_IRUGO | S_IWUSR ,
	       adux1050_threshold_show, adux1050_theshold_store);
static struct device_attribute dev_attr_sensor_calibration =
	__ATTR(adux1050_calibration, S_IRUGO | S_IWUSR ,
	       adux1050_calibration_show, adux1050_calibration_store);
static struct device_attribute dev_attr_sensor_update_calib =
	__ATTR(adux1050_update_calib, S_IRUGO | S_IWUSR ,
	       NULL, adux1050_update_calib_store);
static struct device_attribute dev_attr_sensor_dump =
	__ATTR(adux1050_status, S_IRUSR | S_IRGRP,
	       show_dumpregs, NULL);
static struct device_attribute dev_attr_sensor_status =
	__ATTR(adux1050_prox_status, S_IRUSR | S_IRGRP,
	       show_prox_status, NULL);
static struct device_attribute dev_attr_sensor_calib_target =
	__ATTR(adux1050_calib_target, S_IRUGO | S_IWUSR ,
	       calib_target_show, calib_target_store);
#ifdef CONFIG_EVAL
static struct device_attribute dev_attr_sensor_reg_read =
	__ATTR(adux1050_reg_read, S_IRUGO | S_IWUSR ,
	       show_reg_read, store_reg_read);
static struct device_attribute dev_attr_sensor_reg_write =
	__ATTR(adux1050_reg_write, S_IRUGO | S_IWUSR ,
	       NULL, store_reg_write);
static struct device_attribute dev_attr_sensor_update_config =
	__ATTR(adux1050_update_config, S_IRUGO | S_IWUSR ,
	       NULL, store_update_config);
static struct device_attribute dev_attr_sensor_sw_reset =
	__ATTR(adux1050_sw_reset, S_IRUGO | S_IWUSR ,
	       NULL, store_sw_reset);
static struct device_attribute dev_attr_sensor_force_calib =
	__ATTR(adux1050_force_calib, S_IRUGO | S_IWUSR ,
	       NULL, store_force_calib);
#endif
#ifdef CONFIG_USE_FILP
static struct device_attribute dev_attr_sensor_use_filp =
	__ATTR(adux1050_filp, S_IRUGO | S_IWUSR ,
	       NULL, store_adux1050_filp_config);
#endif
#ifdef CONFIG_ADUX1050_POLL
static struct device_attribute dev_attr_sensor_poll =
	__ATTR(adux1050_poll_delay, S_IRUGO | S_IWUSR ,
	       show_poll_delay, store_poll_delay);
#endif


static struct attribute *adux1050_attrs[] = {
	&dev_attr_adux1050_enable.attr,
	&dev_attr_sensor_name.attr,
	&dev_attr_sensor_vendor.attr,
	&dev_attr_sensor_raw_data.attr,
	&dev_attr_sensor_send_event.attr,
	&dev_attr_sensor_threshold.attr,
	&dev_attr_sensor_calibration.attr,
	&dev_attr_sensor_update_calib.attr,
	&dev_attr_sensor_dump.attr,
	&dev_attr_sensor_status.attr,
	&dev_attr_sensor_calib_target.attr,
#ifdef CONFIG_EVAL
	&dev_attr_sensor_reg_read.attr,
	&dev_attr_sensor_reg_write.attr,
	&dev_attr_sensor_update_config.attr,
	&dev_attr_sensor_sw_reset.attr,
	&dev_attr_sensor_force_calib.attr,
#endif
#ifdef CONFIG_USE_FILP
	&dev_attr_sensor_use_filp.attr,
#endif
#ifdef CONFIG_ADUX1050_POLL
	&dev_attr_sensor_poll.attr,
#endif
	NULL,
};

static struct attribute_group adux1050_attr_group = {
	.name = NULL,
	.attrs = adux1050_attrs,
};

/**
This Routine reads the Device ID to confirm the existance
of the Device in the System.
@param  adux1050 The Device structure
@return 0 on Successful detection of the device,-ENODEV on err.
*/
static int adux1050_hw_detect(struct adux1050_chip *adux1050)
{
	u16 data;
	int ret;

	ret = adux1050->read(adux1050->dev, DEV_ID_REG, &data, DEF_WR);
	if (ret < DEF_WR) {
		ADUX1050_ERR("%s: read DEV_ID_REG failed, ret = %d\n", __func__, ret);
		return ret;
	}

	if (likely((data & ADUX1050_ID_MASK) == ADUX1050_GENERIC_ID)) {
		adux1050->product = ADUX1050_GENERIC_ID >> 8;
		adux1050->version = data & REV_ID_MASK >> 4;
		adux1050->metal_id = data & METAL_REV_ID_MASK;
		ADUX1050_INFO("Found ADUX1050, rev:%x met_id:%x\n",
			 adux1050->version, adux1050->metal_id);
		return 0;
	} else {
		ADUX1050_ERR("ADUX1050 Not Found,ID %4x\n", data);
		return -ENODEV;
	}
}

/**
Threaded IRQ Handler -- Assigns interrupt handling to work
@param handle The data of the ADUX1050 Device
@param irq The Interrupt Request queue to be assigned for the device.
@return IRQ_HANDLED
*/
#ifndef CONFIG_ADUX1050_POLL
static irqreturn_t adux1050_isr_thread(int irq, void *handle)
{
	struct adux1050_chip *adux1050 = handle;
/*	mutex_lock(&adux1050->mutex);
	if (!work_pending(&adux1050->work))
		schedule_work(&adux1050->work);
	else*/
		/*Cleared the interrupt for future intterupts to occur*/
	/*	adux1050->read(adux1050->dev, INT_STATUS_REG,
				&adux1050->int_status, DEF_WR);

	mutex_unlock(&adux1050->mutex);*/
	queue_work(adux1050->det_workqueue, &adux1050->work);
	return IRQ_HANDLED;
}
#endif
/**
conv_complete_cdc_fetch - Fetch the CDC for the connected stage after
receving the conversion sequence complete interrupt is asserted
@param	adux1050	The chip structure of ADUX1050 chip
@return void
*/
static inline void conv_complete_cdc_fetch(struct adux1050_chip *adux1050)
{
	u8 stg_cnt = 0;
	u16 result_cdc;
	s16 ret;
	unsigned int event_value = 0;

	for (stg_cnt = 0; stg_cnt < adux1050->num_stages; stg_cnt++) {
		/** Fetch the CDC only if that stage is connected */
		if (adux1050->stg_info[stg_cnt].status == CIN_CONNECTED) {
			ret = adux1050->read(adux1050->dev, GET_RESULT_REG(stg_cnt),
					&result_cdc, DEF_WR);
			if (ret < DEF_WR) {
				ADUX1050_ERR("%s: read GET_RESULT_REG failed, ret = %d\n",
						__func__, ret);
				return;
			}
			ADUX1050_DEBUG("STG_NO %x - CDC %x\n",
					stg_cnt, result_cdc);
			if (adux1050->send_event) {
				event_value = PACK_FOR_CDC_EVENT(result_cdc,
								 stg_cnt);
				input_event(adux1050->input, EV_MSC,
					    MSC_RAW, event_value);
				input_sync(adux1050->input);
			}
		}
	}
}

/**
This function is used to send ACTIVE event for a stage in ADUX1050
@param adux1050 The ADUX1050 chip structure pointer
@param stg_num	The stage nmmber for which the event to be sent
@param Threshold type	Indicates Low or High threshold
@return void
*/
static inline void indicate_active_state(struct adux1050_chip *adux1050,
					 int stg_num, int threshold_type)
{
	unsigned int event_value = 0;
	ADUX1050_DEBUG("%s\n", __func__);
	event_value = PACK_FOR_ACTIVE_STATE(stg_num, threshold_type);
	input_event(adux1050->input, EV_MSC, MSC_RAW, 0x81);
	input_sync(adux1050->input);

}

/**
This function is used to send IDLE event for a stage in ADUX1050
@param adux1050 The ADUX1050 chip structure pointer
@param stg_num	The stage nmmber for which the event to be sent
@param Threshold type	Indicates Low or High threshold
@return void
*/
static inline void indicate_idle_state(struct adux1050_chip *adux1050,
				       int stg_num, int threshold_type)
{
	unsigned int event_value = 0;
	ADUX1050_DEBUG("%s\n", __func__);
	event_value = PACK_FOR_IDLE_STATE(stg_num, threshold_type);
	input_event(adux1050->input, EV_MSC, MSC_RAW, 0x01);
	input_sync(adux1050->input);
}

/**
high_threshold_int_check - Identify which stage asserts the high threshold
interrupt.After identifying the stage, the state(ACTIVE,IDLE) of the stage
is sent as input event
@param	adux1050 The chip structure of ADUX1050 chip
@param	high_status_change Contains stage number which asserts INT.
@return void
*/
static inline void high_threshold_int_check(struct adux1050_chip *adux1050,
					    u16 high_status_change)
{
	u8 stg_cnt;
	u8 temp_ht_enable = adux1050->high_thresh_enable;

	for (stg_cnt = 0; stg_cnt < TOTAL_STG; stg_cnt++) {
		if (temp_ht_enable & 1) {
			if (high_status_change & 1) {
				if (adux1050->high_status & 1) {
					if (adux1050->send_event)
						indicate_active_state(adux1050,
								      stg_cnt,
								      TH_HIGH);
				} else {
					if (adux1050->send_event)
						indicate_idle_state(adux1050,
								    stg_cnt,
								    TH_HIGH);
				}
			}
		}
		temp_ht_enable = temp_ht_enable >> 1;
		high_status_change = high_status_change >> 1;
		adux1050->high_status = adux1050->high_status >> 1;
	}
}

/**
Low_threshold_int_check - Identify which stage asserts the low threshold
interrupt.After identifying the stage, the state(ACTIVE,IDLE) of the stage
is sent as input event
@param	adux1050 The chip structure of ADUX1050 chip
@param	low_status_change Contains stage number which asserts INT.
@return void
*/
static inline void low_threshold_int_check(struct adux1050_chip *adux1050,
					   u16 low_status_change)
{
	u8 stg_cnt;
	u8 temp_lt_enable = adux1050->low_thresh_enable;

	for (stg_cnt = 0; stg_cnt < TOTAL_STG; stg_cnt++) {
		if (temp_lt_enable & 1) {
			if (low_status_change & 1) {
				if (adux1050->low_status & 1) {
					ADUX1050_ERR("Low interrupt occurred, start force calibration\n");
					adux1050_force_cal(adux1050, adux1050->slp_time_conv_complete);
					indicate_idle_state(adux1050, stg_cnt, TH_HIGH);
				}
			}
			temp_lt_enable = temp_lt_enable >> 1;
			low_status_change = low_status_change >> 1;
			adux1050->low_status = adux1050->low_status >> 1;
		}
	}
}

/**
Work Handler -- Handles the interrupt status from ADUX1050
@param work The work structure for the ADUX1050 chip
@return void Nothing returned
*/
static void adux1050_isr_work_fn(struct work_struct *work)
{
	struct adux1050_chip *adux1050 =  container_of(work,
						   struct adux1050_chip, work);
	u16 high_status_change = 0;
	u16 low_status_change = 0;
	s16 ret;

	mutex_lock(&adux1050->mutex);
	ret = adux1050->read(adux1050->dev, INT_STATUS_REG,
		       &adux1050->int_status, DEF_WR);
	if (ret < DEF_WR) {
		ADUX1050_ERR("%s: read INT_STATUS_REG failed, ret = %d\n",
				__func__, ret);
		mutex_unlock(&adux1050->mutex);
		return;
	}
	ADUX1050_DEBUG("status_reg : %x int_status :%x DEF_WR:%x\n",
		INT_STATUS_REG, adux1050->int_status, DEF_WR);
	adux1050->low_status = GET_LOW_STATUS(adux1050->int_status);
	adux1050->high_status = GET_HIGH_STATUS(adux1050->int_status);
	adux1050->conv_status = GET_CONV_STATUS(adux1050->int_status);

	ADUX1050_DEBUG("HS:%x LS:%x CCS:%x\n", adux1050->high_status,
		adux1050->low_status, adux1050->conv_status);

	/* Handling High threshold interrupt */
	if (adux1050->high_thresh_enable) {
		high_status_change = ((adux1050->high_status) ^
				      (adux1050->prev_high_status));
		adux1050->prev_high_status = adux1050->high_status;
		if (high_status_change)
			high_threshold_int_check(adux1050, high_status_change);
	}

	/* Handling Low threshold interrupt */
	if (adux1050->low_thresh_enable) {
		low_status_change = ((adux1050->low_status) ^
				    (adux1050->prev_low_status));
		adux1050->prev_low_status = adux1050->low_status;
		if (low_status_change)
			low_threshold_int_check(adux1050, low_status_change);
	}
	/* Handling Conversion complete interrupt */
	if (adux1050->conv_enable && adux1050->conv_status)
		conv_complete_cdc_fetch(adux1050);

	mutex_unlock(&adux1050->mutex);
}

#ifdef CONFIG_ADUX1050_POLL
/**
adux1050_polling_thread_fn -- Run function of the ADUX1050 Kthread
@param void *chip_data - Hold the pointer of ADUX1050 chip structure
@return int - Returns Zero
*/

static int adux1050_polling_thread_fn(void *chip_data)
{
	struct adux1050_chip *adux1050 = chip_data;
	s16 ret;

	ADUX1050_INFO("Polling Thread Started\n");
	while (!kthread_should_stop()) {
		if ((adux1050->dev_enable == ADUX1050_ENABLE) ||
		    (adux1050->dac_calib.cal_flags == CAL_RET_PENDING)) {
			mutex_lock(&adux1050->mutex);
			if (!work_pending(&adux1050->work))
				schedule_work(&adux1050->work);
			else {
				/*Cleared the interrupt for future intterupts to occur*/
				ret = adux1050->read(adux1050->dev, INT_STATUS_REG,
						&adux1050->int_status, DEF_WR);
				if (ret < DEF_WR) {
					ADUX1050_ERR("%s: read INT_STATUS_REG failed, ret = %d\n",
							__func__, ret);
					return ret;
				}
			}
			mutex_unlock(&adux1050->mutex);
			if (likely(adux1050->poll_delay <= MSLEEP_MIN_TIME))
				usleep_range(adux1050->poll_delay * MS_TO_US,
					     adux1050->poll_delay * MS_TO_US);
			else
				msleep(adux1050->poll_delay);

		} else {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule();
		}
	}
	return 0;
}
#endif

/**
Device probe function  all initialization routines are handled here like
the ISR registration or the polling thead registeration,Work creation,
Input device registration, SYSFS attributes creation etc.
@param i2c_client the i2c structure of the adux1050 device/client.
@param i2c_device_id The i2c_device_id for the supported i2c device.
@return 0 on success,and On failure -ENOMEM, -EINVAL ,etc., will be returned
*/
static int adux1050_setup_input_device(struct adux1050_chip *adux1050)
{
	struct input_dev *input = NULL;
	int ret;

	/*
	 * Allocate and register adux1050 input device
	 */
	input = input_allocate_device();
	if (!input) {
		ADUX1050_ERR("%s: could not allocate input device\n", __func__);
		return -ENOMEM;
	}

	adux1050->input = input;
	input_set_drvdata(adux1050->input, adux1050);
	input->name = DEVICE_NAME;
	set_bit(EV_MSC, input->evbit);
	input_set_capability(input, EV_MSC, MSC_RAW);

	ret = input_register_device(input);
	if (ret) {
		ADUX1050_ERR("%s: could not input_register_device(input);\n",
				__func__);
		input_free_device(adux1050->input);
		return ret;
	}

	return 0;
}

/* sensor will work in poll mode if defined CONFIG_ADUX1050_POLL
 * otherwise interrupt mode
 */
static int adux1050_config_sensor_work_mode(struct adux1050_chip *adux1050,
		struct i2c_client *client)
{
	int ret;

	adux1050->det_workqueue =
		create_singlethread_workqueue("adux_det_workqueue");
	if (!adux1050->det_workqueue) {
		ADUX1050_ERR("%s : workqueue create Failed\n", __func__);
		return -ENOMEM;
	}

#ifdef CONFIG_ADUX1050_POLL
	/* Creation of Thread */
	adux1050->polling_task = kthread_create(adux1050_polling_thread_fn,
			(void *)adux1050, "adux1050_kthread");
	if (ERR_PTR(-ENOMEM) == adux1050->polling_task) {
		ADUX1050_ERR("%s : Thread Creation Failed\n", __func__);
		destroy_workqueue(adux1050->det_workqueue);
		return -ENOMEM;
	}
	if (adux1050->dev_enable == ADUX1050_ENABLE) {
		wake_up_process(adux1050->polling_task);
		ADUX1050_INFO("%s :wake up success\n", __func__);
	}
	adux1050->poll_delay = MIN_POLL_DELAY;
	adux1050->int_pol = ACTIVE_LOW;
#else
	adux1050->irq = client->irq;
	if (!adux1050->pdata->irq_flags)
		adux1050->pdata->irq_flags = IRQF_TRIGGER_FALLING;

	ret = request_threaded_irq(adux1050->irq, NULL, adux1050_isr_thread,
			IRQF_ONESHOT | adux1050->pdata->irq_flags,
			DEVICE_NAME, adux1050);

	if (ret) {
		ADUX1050_ERR("%s: irq %d Driver init Failed", __func__, adux1050->irq);
		destroy_workqueue(adux1050->det_workqueue);
		return ret;
	}

	ADUX1050_INFO("%s - request_threaded_irq %d success\n",
			__func__, adux1050->irq);
	disable_irq(adux1050->irq);

	/* if irq flag is IRQF_TRIGGER_RISING then
	 * interrupt output polarity is high
	 * otherwise low
	 * */
	if (adux1050->pdata->irq_flags & IRQF_TRIGGER_RISING)
		adux1050->int_pol = ACTIVE_HIGH;
	else
		adux1050->int_pol = ACTIVE_LOW;
#endif

	return 0;
}

static int adux1050_initialize(struct adux1050_chip *adux1050, struct i2c_client *client)
{
	struct device *dev;
	int ret;
#ifdef CONFIG_OF
	s32 df_prop_length = 0;
	const __be32 *irqf = NULL;
#endif

	dev = &client->dev;

	if (dev->platform_data == NULL) {
		adux1050->pdata = &local_plat_data;
		ADUX1050_WARN("%s: Platform Data Not Found\n", __func__);

#ifdef CONFIG_OF
		if (dev->of_node != NULL) {
			adux1050->dt_device_node = dev->of_node;
			ADUX1050_INFO("%s: DT node Found, \n", __func__);
			irqf = of_get_property(adux1050->dt_device_node,
				  "adi,adux1050_irq_flags", &df_prop_length);
			if (irqf && (df_prop_length == sizeof(int))) {
				adux1050->pdata->irq_flags =
						be32_to_cpu(*irqf);
				ADUX1050_INFO("%s Usg irq_flag from DT %d\n",
					__func__, adux1050->pdata->irq_flags);
			}

		} else {
			ADUX1050_INFO("%s - Usg local pltfm data\n", __func__);
		}
#endif
	} else {
		ADUX1050_INFO("%s - Pltfm Data Found\n", __func__);
		adux1050->pdata = dev->platform_data;
	}

	ret = adux1050_config_sensor_work_mode(adux1050, client);
	if (ret) {
		ADUX1050_ERR("%s: config work mode  failed\n", __func__);
		return ret;
	}

	/* parse the register data from dt first.
	 * if data not found then use local_plat_data.
	 */
	adux1050_store_register_values(adux1050);

	/*set the device enable state to ADUX1050_DISABLE*/
	adux1050->dev_enable = ADUX1050_DISABLE;

	/* set to 1 if we want to call input_event function.
	 * otherwise 0
	 * */
	adux1050->send_event = ADUX1050_ENABLE;
#ifdef CONFIG_EVAL
	adux1050->power_mode_flag = ADUX1050_DISABLE;
#endif

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
	set_hw_dev_flag(DEV_I2C_SAR_SENSOR);
#endif
	ret = app_info_set("Sar-Sensor", "ADI ADUX1050");
	if (ret < 0) {
		ADUX1050_ERR("%s %d:failed to add app_info\n", __func__, __LINE__);
	}
	return 0;
}

static int adux1050_power_config(struct adux1050_chip *adux1050)
{
	int ret;

	adux1050->reg_vio = regulator_get(adux1050->dev, "vio");
	if (IS_ERR(adux1050->reg_vio)) {
		ret = PTR_ERR(adux1050->reg_vio);
		ADUX1050_ERR("%s: regulator vio get failed\n", __func__);
		return ret;
	}

	adux1050->reg_vdd = regulator_get(adux1050->dev, "vdd");
	if (IS_ERR(adux1050->reg_vdd)) {
		ret = PTR_ERR(adux1050->reg_vdd);
		ADUX1050_ERR("%s: regulator vdd get failed\n", __func__);
		regulator_put(adux1050->reg_vio);
		return ret;
	}

	if (regulator_count_voltages(adux1050->reg_vdd) > 0) {
		ret = regulator_set_voltage(adux1050->reg_vdd, MAX_VDD, MAX_VDD);
		if (ret) {
			ADUX1050_ERR("%s: could not set vdd voltage\n", __func__);
			regulator_put(adux1050->reg_vdd);
			regulator_put(adux1050->reg_vio);
			return ret;
		}
	}

	return 0;
}

static int adux1050_power_on(struct adux1050_chip *adux1050)
{
	int ret;

	ret = regulator_enable(adux1050->reg_vdd);
	if (ret) {
		ADUX1050_ERR("%s: could not enable vdd voltage\n", __func__);
		return ret;
	}

	return 0;
}
static int adux1050_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	s32 ret = -EINVAL;
	struct adux1050_chip *adux1050 = NULL;

	ADUX1050_INFO("adux1050 probe come\n");
	if ((client == NULL) || (&client->dev == NULL)) {
		ADUX1050_ERR("%s: Client/client->dev doesn't exist\n", __func__);
		return ret;
	}
#ifndef CONFIG_ADUX1050_POLL
	if (client->irq <= 0) {
		ADUX1050_ERR("%s: IRQ not configured!\n", __func__);
		return ret;
	}
	ADUX1050_DEBUG("%s: client->irq = %d\n", __func__, client->irq);
#endif

	adux1050 = kzalloc(sizeof(*adux1050), GFP_KERNEL);
	if (!adux1050) {
		ADUX1050_ERR("%s: Memory alloc fail - Chip struct\n", __func__);
		ret = -ENOMEM;
		return ret;
	}

	adux1050->stg_info =
		kzalloc((sizeof(struct adux1050_stage_info) * TOTAL_STG),
			GFP_KERNEL);
	if (!adux1050->stg_info) {
		ADUX1050_ERR("%s: Memory Alloc fail - Stage info\n", __func__);
		goto err_kzalloc_stg;
	}

	mutex_init(&adux1050->mutex);

	adux1050->read = adux1050_i2c_read;
	adux1050->write = adux1050_i2c_write;
	adux1050->dev = &client->dev;

	/*config power vdd*/
	ret = adux1050_power_config(adux1050);
	if (ret) {
		ADUX1050_ERR("%s: config power failed\n", __func__);
		goto err_config_power;
	}

	/*power on*/
	ret = adux1050_power_on(adux1050);
	if (ret) {
		ADUX1050_ERR("%s: power on failed\n", __func__);
		goto err_power_on;
	}

	ADUX1050_INFO("power on success\n");
	/* check if the device is existing by reading device id of ADUX1050 */
	ret = adux1050_hw_detect(adux1050);
	if (ret) {
		ADUX1050_ERR("%s: sensor not found\n", __func__);
		goto err_detect_sensor;
	}

	ADUX1050_INFO("find adux1050 sensor\n");
	i2c_set_clientdata(client, adux1050);

	INIT_WORK(&adux1050->work, adux1050_isr_work_fn);
	INIT_WORK(&adux1050->calib_work, adux1050_calibration);

	/*set up input device*/
	ret = adux1050_setup_input_device(adux1050);
	if (ret) {
		ADUX1050_ERR("%s: set up input device failed\n", __func__);
		goto err_detect_sensor;
	}

	/*register sensor class*/
	sar_cdev.sensors_enable = set_enable;
	adux1050->cdev = sar_cdev;
	ret = sensors_classdev_register(adux1050->dev, &adux1050->cdev);
	if (ret) {
		ADUX1050_ERR("%s: could not register class device\n", __func__);
		goto err_input_register_device;
	}
	/*set up sys interface*/
	ret = sysfs_create_group(&adux1050->dev->kobj, &adux1050_attr_group);
	if (ret) {
		ADUX1050_ERR("%s: cound not register sensor sysfs\n", __func__);
		goto err_register_sensor_class;
	}

	/*set the adux1050 default state*/
	ret = adux1050_initialize(adux1050, client);
	if (ret) {
		ADUX1050_ERR("%s: cound not initialize sensor \n", __func__);
		goto err_sysfs_create_input;
	}

	ADUX1050_INFO("adux1050 sensor initialize success\n");
	ret = regulator_disable(adux1050->reg_vdd);
	if (ret) {
		ADUX1050_ERR("%s: could not disable vdd voltage\n", __func__);
		return ret;
	}
	ADUX1050_INFO("%s Completed\n", __func__);
	return 0;

err_sysfs_create_input:
	sysfs_remove_group(&adux1050->dev->kobj, &adux1050_attr_group);
err_register_sensor_class:
	sensors_classdev_unregister(&adux1050->cdev);
err_input_register_device:
	if (adux1050->input) {
		input_unregister_device(adux1050->input);
		/*input_unregister_device Already free device, dont use input_free_device again*/
	}
err_detect_sensor:
err_power_on:
	if (regulator_count_voltages(adux1050->reg_vdd) > 0) {
		regulator_set_voltage(adux1050->reg_vdd, MIN_VDD, MAX_VDD);
		regulator_disable(adux1050->reg_vdd);
	}
	regulator_put(adux1050->reg_vdd);
	regulator_put(adux1050->reg_vio);
err_config_power:
	kfree(adux1050->stg_info);
err_kzalloc_stg:
	kfree(adux1050);

	return ret;
}

/**
Removes the device.
This is used to remove the device or the I2C client from the system
@param client The client structure to be removed
@return 0 on success
*/
static int adux1050_i2c_remove(struct i2c_client *client)
{
	struct adux1050_chip *adux1050 = i2c_get_clientdata(client);
	u16 data = DISABLE_DEV_INT;
	s16 ret;

	ADUX1050_INFO("%s, Start\n", __func__);
	if (adux1050 != NULL) {
		if (adux1050->dev_enable == ADUX1050_ENABLE) {
			#ifdef CONFIG_ADUX1050_POLL
			#else
				disable_irq(adux1050->irq);
				disable_irq_wake(adux1050->irq);
			#endif
			ret = adux1050->write(adux1050->dev, INT_CTRL_REG,
					&data, DEF_WR);
			if (ret < DEF_WR)
				ADUX1050_ERR("%s: write INT_CTRL_REG failed, ret = %d\n", __func__, ret);

			data = data | RESET_MASK;
			/* Reset should be reviewed for nesesity */
			ret = adux1050->write(adux1050->dev, CTRL_REG,
					&data, DEF_WR);
			if (ret < DEF_WR)
				ADUX1050_ERR("%s: write CTRL_REG failed, ret = %d\n", __func__, ret);
		}
		sysfs_remove_group(&adux1050->dev->kobj,
				   &adux1050_attr_group);
	#ifdef CONFIG_ADUX1050_POLL
		kthread_stop(adux1050->polling_task);
	#else
		free_irq(adux1050->irq, adux1050);
	#endif
		cancel_work_sync(&adux1050->work);
		cancel_work_sync(&adux1050->calib_work);
		input_unregister_device(adux1050->input);
		/*input_unregister_device Already free device, dont use input_free_device again*/
		destroy_workqueue(adux1050->det_workqueue);
		kfree(adux1050->stg_info); /* BJ */
		kfree(adux1050);
		i2c_set_clientdata(client, NULL);
	}
	ADUX1050_INFO("%s, END\n", __func__);
	return 0;
}

	/*dont use shut down function*/
/**
Device suspend PM operation call back function
@dev Device structure for handling the power management.
@return Zero on success
*/
static int adux1050_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct adux1050_chip *adux1050 = i2c_get_clientdata(client);

	ADUX1050_INFO("%s,check (%d)\n", __func__, adux1050->dev_enable);
//delete lines


	return 0;
}

/**
Device resume PM operation call back function
@dev Device structure for handling the power management.
@return Zero on success
*/
static int adux1050_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct adux1050_chip *adux1050 = i2c_get_clientdata(client);

	ADUX1050_INFO("%s,check (%d)\n", __func__, adux1050->dev_enable);
//delete lines
#ifdef CONFIG_ADUX1050_POLL
		wake_up_process(adux1050->polling_task);
#endif
//delete lines

	return 0;
}

/**
Device ID table for the ADUX1050 driver
*/
static const struct of_device_id adux1050_of_id[] = {
	{.compatible = "adi,adux1050"},
	{}
};

/**
Device ID table for the ADUX1050 driver
*/
static const struct i2c_device_id adux1050_id[] = {
	{ "adux1050", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, adux1050_id);

/**
  The file Operation table for power
*/
static const struct dev_pm_ops adux1050_dev_pm_ops = {
	.suspend = adux1050_i2c_suspend,
	.resume = adux1050_i2c_resume,
};

/**
I2C driver structure containing the function callbacks,
driver name and ID tables
*/
struct i2c_driver adux1050_i2c_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.pm = &adux1050_dev_pm_ops,
#ifdef	CONFIG_OF
		.of_match_table = adux1050_of_id,
#endif
	},
	.probe    = adux1050_probe,
	/*dont use shut down function*/
	.remove   = adux1050_i2c_remove,
	.id_table = adux1050_id,
};

/**
This is an init function called during module insertion.
calls in turn i2c driver probe function
*/
static __init int adux1050_module_init(void)
{
	ADUX1050_INFO("%s,START\n", __func__);
	return i2c_add_driver(&adux1050_i2c_driver);
}
module_init(adux1050_module_init);

/**
This is an exit function called during module removal --
calls in turn i2c driver delete function
*/
static __exit void adux1050_module_exit(void)
{
	i2c_del_driver(&adux1050_i2c_driver);
}

module_exit(adux1050_module_exit);
MODULE_DESCRIPTION("Analog Devices ADUX1050 Driver");
MODULE_AUTHOR("Analog Devices");
MODULE_LICENSE("GPL");
