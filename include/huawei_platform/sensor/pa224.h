/*
 * This file is part of the APDS990x sensor driver.
 * Chip is combined proximity and ambient light sensor.
 *
 * Copyright (C) 2010 Nokia Corporation and/or its subsidiary(-ies).
 *
 * Contact: Samu Onkalo <samu.p.onkalo@nokia.com>
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

#ifndef __TXC_PA2240_H__
#define __TXC_PA2240_H__

/*pa12200001 als/ps sensor register map*/
#define TXC_PA2240_REG_CFG0 			0X00  	// ALS_GAIN(D5-4),PS_ON(D1) ALS_ON(D0)
#define TXC_PA2240_REG_CFG1 			0X01  	// LED_CURR(D5-4),PS_PRST(D3-2),ALS_PRST(D1-0)
#define TXC_PA2240_REG_CFG2 			0X02  	// PS_MODE(D7-6),CLEAR(D4),INT_SET(D3-2),PS_INT(D1),ALS_INT(D0)
#define TXC_PA2240_REG_CFG3			0X03  	// INT_TYPE(D6),PS_PERIOD(D5-3),ALS_PERIOD(D2-0)
#define TXC_PA2240_REG_ALS_TL_LSB		0X04  	// ALS Threshold Low LSB
#define TXC_PA2240_REG_ALS_TL_MSB		0X05  	// ALS Threshold Low MSB
#define TXC_PA2240_REG_ALS_TH_LSB		0X06  	// ALS Threshold high LSB
#define TXC_PA2240_REG_ALS_TH_MSB		0X07  	// ALS Threshold high MSB
#define TXC_PA2240_REG_PS_TL			0X08  	// PS Threshold Low
#define TXC_PA2240_ID_REG				0x09
#define TXC_PA2240_REG_PS_TH			0X0A  	// PS Threshold High
#define TXC_PA2240_REG_ALS_DATA_LSB	0X0B  	// ALS DATA
#define TXC_PA2240_REG_ALS_DATA_MSB	0X0C  	// ALS DATA
#define TXC_PA2240_REG_PS_DATA			0X0E  	// PS DATA
#define TXC_PA2240_REG_PS_OFFSET		0X10  	// TBD
#define TXC_PA2240_REG_PS_SET			0X11  	// 0x82
#define TXC_PA2240_REG_PS_FLCT		0x12      // 0x0C

#define TXC_PA2240_ALS_ACTIVE    		0x01
#define TXC_PA2240_PS_ACTIVE			0x02

#define TXC_PA2240_ALS_INT_ACTIVE    	0x01
#define TXC_PA2240_PS_INT_ACTIVE		0x02



#define PA2240_IRLED_CURR_12mA	0x3
#define PA2240_IRLED_CURR_25mA	0x2
#define PA2240_IRLED_CURR_50mA	0x1
#define PA2240_IRLED_CURR_100mA	0x0

/*Driver Parameters  */

/*pa224 als/ps Default*/  
#define PA2240_ALS_TH_HIGH  35000
#define PA2240_ALS_TH_LOW	 30000

#define PA2240_ALS_POLL_DELAY	100
#define PA2240_PS_POLL_DELAY		100	

#define PA2240_ALS_ENABLE_DELAY	120
#define PA2240_PS_ENABLE_DELAY	30

#define PA2240_PS_TH_HIGH	200
#define PA2240_PS_TH_LOW		50
#define PA2240_PS_TH_MIN		0	// Minimun value
#define PA2240_PS_TH_MAX		255     // 8 bit MAX

#define PA2240_PS_NEAR_DISTANCE	0       //Near distance 0 cm
#define PA2240_PS_FAR_DISTANCE	1       //Far distance 1 cm 
#define PA2240_PS_OFFSET_DEFAULT	10 	// for X-talk cannceling
#define PA2240_PS_OFFSET_MAX		200
#define PA2240_PS_OFFSET_MIN		0
#define PA2240_PS_OFFSET_EXTRA	5
#define PA22404_FAST_CAL			1
#define PA2240_FAST_CAL_ONCE		0

#define PA2240_ALS_GAIN			3 	// 0:125lux 1:1000lux 2:2000lux 3:10000lux 
#define PA2240_LED_CURR			4 	/* 0:150mA | 1:100mA | 2:50mA | 3:25mA | 4:15mA | 5:12mA | 6:10mA | 7:7mA*/
#define PA2240_PS_PRST			   2	// 0:1point 1:2points 2:4points 3:8points (for INT)
#define PA2240_ALS_PRST			0	// 0:1point 1:2points 2:4points 3:8points (for INT)
#define PA2240_PS_SET				1	// 0:ALS interrupt only 1:PS interrupt only 3:BOTH interrupt *no use now
#define PA2240_PS_MODE			   1	// 0:OFFSET 1:NORMAL
#define PA2240_INT_TYPE			0 	// 0:Window type 1:Hysteresis type for Auto Clear flag , if ALS use interrupt mode,should use windows mode 
#define PA2240_PS_PERIOD			1	/* 0:6.25 ms | 1:12.5 ms | 2:25 ms | 3:50 ms | 4:100 ms | 5:200 ms | 6:400 ms | 7:800 ms */
#define PA2240_ALS_PERIOD			0	// 0 ms 
#define ALS_PS_INT			0 	

#define ALS_AVG_ENABLE			0	

/* POWER SUPPLY VOLTAGE RANGE */
#define TXC_PA2240_VDD_MIN_UV  2000000
#define TXC_PA2240_VDD_MAX_UV  3300000
#define TXC_PA2240_VIO_MIN_UV  1750000
#define TXC_PA2240_VIO_MAX_UV  1950000

#define PA24_PS_ENABLE_DELAY	30
#define PA2240_CALIBRATION_SUCCESS 0
#define PA2240_CALIBRATION_I2C_ERROR  -4
#define PA2240_CALIBRATION_ERROR    -5
struct txc_pa2240_data;

struct txc_pa2240_platform_data {
	u8	   pdrive;
	unsigned int  ppcount;
	int    (*setup_resources)(void);
	int    (*release_resources)(void);

	int irq_num;
	int (*power)(unsigned char onoff);
	/*add the parameters to the init and exit function*/
	int (*init)(struct txc_pa2240_data *data);
	void (*exit)(struct txc_pa2240_data *data);
	int (*power_on)(bool,struct txc_pa2240_data *data);
	
	unsigned int pwave;
	unsigned int pwindow;
	unsigned int crosstalk;
	unsigned int defalt_crosstalk;

	unsigned int irq_gpio;
	u8 crosstalk_base;
	int flag;
};


#define TXC_PA2240_FAR_FLAG 1
#define TXC_PA2240_CLOSE_FLAG 0

#endif
