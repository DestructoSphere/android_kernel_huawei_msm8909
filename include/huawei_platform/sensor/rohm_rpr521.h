/*
 * This file is part of the RPR521 sensor driver.
 * Chip is combined proximity and ambient light sensor.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef __RPR521_H__
#define __RPR521_H__

/**
 * struct rpr521_platform_data - platform data for rpr521.c driver
 * @cf: chip factor data
 * @pddrive: IR-led driving current
 * @ppcount: number of IR pulses used for proximity estimation
 * @setup_resources: interrupt line setup call back function
 * @release_resources: interrupt line release call back function
 *
 * Proximity detection result depends heavily on correct ppcount, pdrive
 * and cover window.
 *
 */

/* POWER SUPPLY VOLTAGE RANGE */
#define RPR521_VDD_MIN_UV  2000000
#define RPR521_VDD_MAX_UV  3300000
#define RPR521_VIO_MIN_UV  1750000
#define RPR521_VIO_MAX_UV  1950000

/* Analog voltage @2.7 V */
#define AVDD_VTG_MIN_UV		3000000
#define AVDD_VTG_MAX_UV		3000000
#define AVDD_ACTIVE_LOAD_UA	15000

/* Digital voltage @1.8 V */
#define VDDIO_VTG_DIG_MIN_UV	1800000
#define VDDIO_VTG_DIG_MAX_UV	1800000
#define VDDIO_ACTIVE_LOAD_DIG_UA	10000

#define VDDIO_I2C_VTG_MIN_UV	1800000
#define VDDIO_I2C_VTG_MAX_UV	1800000
#define VDDIO_I2C_LOAD_UA		10000

/*the rpr521_platform_data structure needs to cite the definition of rpr521_data*/
struct rpr521_data;
struct rpr521_platform_data {

	u8	   pdrive;
	int    (*setup_resources)(void);
	int    (*release_resources)(void);

	int irq_num;
	int (*power)(unsigned char onoff);
	/*add the parameters to the init and exit function*/
	int (*init)(struct rpr521_data *data);
	void (*exit)(struct rpr521_data *data);
	int (*power_on)(bool,struct rpr521_data *data);
	/*remove to timer the rpr521_data structure*/
	unsigned int prox_threshold;
	unsigned int alsit;
	
	unsigned int pwave;
	unsigned int pwindow;
    
	bool i2c_pull_up;
	bool digital_pwr_regulator;

	unsigned int irq_gpio;
	u32 irq_gpio_flags;
	/* i2c gpio */
	unsigned int i2c_scl_gpio;
	unsigned int i2c_sda_gpio;
	unsigned int ps_gain;
};

#define RPR521_IOCTL_PS_ENABLE	1
#define RPR521_IOCTL_PS_GET_ENABLE	2
#define RPR521_IOCTL_PS_GET_PDATA	3	/* pdata */
#define RPR521_IOCTL_ALS_ENABLE	4
#define RPR521_IOCTL_ALS_GET_ENABLE	5
#define RPR521_IOCTL_ALS_GET_CH0DATA	6	/* ch0data */
#define RPR521_IOCTL_ALS_GET_CH1DATA	7	/* ch1data */
#define RPR521_IOCTL_ALS_DELAY	8

/* RPR400 REGSTER */
#define REG_SYSTEMCONTROL         (0x40)
#define REG_MODECONTROL           (0x41)
#define REG_ALSPSCONTROL          (0x42)
#define REG_PERSISTENCE           (0x43)
#define REG_PSDATA                (0x44)
#define REG_PSDATA_LSB            (0x44)
#define REG_PSDATA_MBS            (0x45)
#define REG_ALSDATA0              (0x46)
#define REG_ALSDATA0_LSB          (0x46)
#define REG_ALSDATA0_MBS          (0x47)
#define REG_ALSDATA1              (0x48)
#define REG_ALSDATA1_LSB          (0x48)
#define REG_ALSDATA1_MBS          (0x49)
#define REG_INTERRUPT             (0x4A)
#define REG_PSTH                  (0x4B)
#define REG_PSTH_LSB              (0x4B)
#define REG_PSTH_MBS              (0x4C)
#define REG_PSTL                  (0x4D)
#define REG_PSTL_LSB              (0x4D)
#define REG_PSTL_MBS              (0x4E)
#define REG_ALSDATA0TH            (0x4F)
#define REG_ALSDATA0TH_LSB        (0x4F)
#define REG_ALSDATA0TH_MBS        (0x50)
#define REG_ALSDATA0TL            (0x51)
#define REG_ALSDATA0TL_LSB        (0x51)
#define REG_ALSDATA0TL_MBS        (0x52)
#define REG_MANUFACT_ID           (0x92)

/* SETTINGS */
#define CUT_UNIT        (1) //(10)		//Andy 2012.6.6: it was (1000) previously. But I don't think the customer need als data with such accuracy.
#define CALC_ERROR        (0x80000000)
#define MASK_LONG         (0xFFFFFFFF)
#define MASK_CHAR         (0xFF)
#define BOTH_STANDBY	(0)
#define ALS100MS	(0x5)
#define PS100MS		(0x3)
#define BOTH100MS	(0x6)
#define PS10MS       (0x1)//grace modify in 2014.5.6
#define PS_EN         (1 << 6)//grace modify in 2014.5.6
#define ALS_EN         (1 << 7)//grace modify in 2014.5.6
#define LEDCURRENT_025MA    (0)
#define LEDCURRENT_050MA    (1)
#define LEDCURRENT_100MA    (2)
#define LEDCURRENT_200MA    (3)
#define ALSGAIN_X1X1        (0x0 << 2)
#define ALSGAIN_X1X2        (0x4 << 2)
#define ALSGAIN_X2X2        (0x5 << 2)
#define ALSGAIN_X64X64      (0xA << 2)
#define ALSGAIN_X128X64     (0xE << 2)
#define ALSGAIN_X128X128    (0xF << 2)
#define PSGAIN_X1           (0x0 << 4)
#define PSGAIN_X2           (0x1 << 4)
#define PSGAIN_X4           (0x2 << 4)
#define INFRARED_LOW        (0x0)  //grace modify in 2014.5.7
#define INFRARED_MID        (0x1)  //grace modify in 2014.5.7
#define INFRARED_HIGH       (0x3)  //grace modify in 2014.5.7
#define NORMAL_MODE         (0 << 4)
#define LOW_NOISE_MODE      (1 << 4)
#define PS_INT_MASK		(1 << 7)
#define ALS_INT_MASK	(1 << 6)
#define PS_THH_ONLY         (0 << 4)
#define PS_THH_BOTH_HYS     (1 << 4)
#define PS_THH_BOTH_OUTSIDE (2 << 4)
#define POLA_ACTIVEL        (0 << 3)
#define POLA_INACTIVEL      (1 << 3)
#define OUTPUT_ANYTIME      (0 << 2)
#define OUTPUT_LATCH        (1 << 2)
#define MODE_NONUSE         (0)
#define MODE_PROXIMITY      (1)
#define MODE_ILLUMINANCE    (2)
#define MODE_BOTH           (3)

/* RANGE */
#define REG_PSTH_MAX     (0xFFF)
#define RPR521_LUX_MAX            30000
#define RPR521_SUNLIGHT_CHODATA 28416
/* INIT PARAM */
#define PS_ALS_SET_MODE_CONTROL   (BOTH100MS)//(NORMAL_MODE)
#define PS_DOUBLE_PULSE       (1 << 5)
#define PS_ALS_SET_ALSPS_CONTROL  (LEDCURRENT_100MA | ALSGAIN_X2X2)	//Set high gain value to acquire high accuracy
#define PS_ALS_SET_INTR_PERSIST (PSGAIN_X1 | 0x1)
#define PS_ALS_SET_INTR           (PS_THH_BOTH_OUTSIDE| POLA_ACTIVEL)
//#define PS_ALS_SET_INTR           (PS_THH_BOTH_OUTSIDE| POLA_ACTIVEL | OUTPUT_LATCH | MODE_NONUSE) //grace modify
#define PS_ALS_SET_PS_TH          (30)	//Customer should change the threshold value according to their mechanical design and measured data
#define PS_ALS_SET_PS_TL          (10)	//Changed from (0x000)
#define PS_ALS_SET_ALS_TH         (2000) 	//Compare with ALS_DATA0. ALS_Data equals 0.192*ALS_DATA0 roughly. Usually not used.
#define PS_ALS_SET_ALS_TL         (0x0000)	//Usually not used.
#define PS_ALS_SET_MIN_DELAY_TIME (100)

#define RPR521_FAR_FLAG 1
#define RPR521_CLOSE_FLAG 0
#define PSGAIN1  1
#define PSGAIN2  2
#define PSGAIN4  4
typedef struct {
    long long      data;
    long long      data0;
    long long      data1;
    unsigned char  gain_data0;
    unsigned char  gain_data1;
    unsigned long  dev_unit;
    //unsigned char      als_time;
    unsigned short als_time; //grace modified in 2014.4.2
    unsigned short als_data0;
    unsigned short als_data1;
} CALC_DATA;

typedef struct {
    unsigned long positive;
    unsigned long decimal;
} CALC_ANS;
#endif
#define ARR_NUM  13
#define PRINT_LUX_TIME  30
