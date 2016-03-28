
/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
*
* File Name          : lis3dh_misc.h
* Authors            : MH - C&I BU - Application Team
*		     : Matteo Dameno (matteo.dameno@st.com)
*		     : Carmine Iascone (carmine.iascone@st.com)
*                    : Samuel Huo (samuel.huo@st.com)
* Version            : V 1.1.0
* Date               : 07/10/2012
*
********************************************************************************
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
*******************************************************************************/

#ifndef	__LIS3DH_H__
#define	__LIS3DH_H__


#define SAD0L			0x00
#define SAD0H			0x01
#define LIS3DH_ACC_I2C_SADROOT	0x0C
#define LIS3DH_ACC_I2C_SAD_L	((LIS3DH_ACC_I2C_SADROOT<<1)|SAD0L)
#define LIS3DH_ACC_I2C_SAD_H	((LIS3DH_ACC_I2C_SADROOT<<1)|SAD0H)
#define	LIS3DH_ACC_DEV_NAME	"lis3dh_acc"
#define ACCEL_INPUT_DEV_NAME	"accelerometer"

/************************************************/
/*	Accelerometer defines section		*/
/************************************************/

/* Accelerometer Sensor Full Scale */
#define	LIS3DH_ACC_FS_MASK		0x30
#define LIS3DH_ACC_G_2G			0x00
#define LIS3DH_ACC_G_4G			0x10
#define LIS3DH_ACC_G_8G			0x20
#define LIS3DH_ACC_G_16G		0x30


#ifdef	__KERNEL__
struct lis3dh_acc_platform_data {
	int init_interval;
	int min_interval;

	u8 g_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;
	bool enable_int;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

	/* set gpio_int[1,2] either to the choosen gpio pin number or to -EINVAL
	 * if leaved unconnected
	 */
	int gpio_int1;
	int gpio_int2;
	/* i2c gpio */
	unsigned int i2c_scl_gpio;
	unsigned int i2c_sda_gpio;
};

#ifdef CONFIG_HUAWEI_KERNEL
#include	<linux/sensors.h>

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



#define	FIFO_SRC_REG		0x2F	/*	FiFo source reg	*/
#define REG1_ODR_MASK			0XF0
#define REG1_ODR_SHIFT			4
#define FIFO_MODE_SHIFT			6
#define FIFO_MODE_MASK			0xC0
#define FIFO_WM_CFG_MASK		0x1F
#define FIFO_STATE_WATER_MARK		0x80
#define FIFO_STATE_FULL			0x40
#define FIFO_SRC_DATA_CNT_MASK		0x1F
#define FIFO_MAX_CNT			0x1F
#define FIFO_SRC_OVRN_MASK		0x40
#define REG5_FIFO_EN			0x40

/* interrupt configure REG BITS */
#define INT_GEN_I1_CLICK		0x80
#define INT_GEN_I1_GEN1			0x40
#define INT_GEN_I1_DRDY1		0x10
#define INT_GEN_I1_WTM			0x04
#define INT_GEN_I1_OVRN			0x02

#define IS_FIFO_FULL(status)		(status & FIFO_STATE_FULL)
#define IS_WATER_MARK_REACHED(status)	(status & FIFO_STATE_WATER_MARK)

/*soc irq set*/
#define CONFIG_IRQ_DRDY1	0x10
#define CONFIG_BLOCK_READ	0x80




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
struct lis3dh_acc_data;
#ifdef CONFIG_HUAWEI_DSM 
#define COUNT_EXCEPTION_TIME_2S     2000
#define EXCEPTION_PERCENTAGE 		80
#define EXCEPTION_BASE_TIMES		24
#define	LIS3DH_I2C_SCL	909
#define	LIS3DH_I2C_SDA	908
/* for example: base=1g, x_y_z_max = 3g*/
#define base_to_total_max(base) 		(3*base)
/* for example: base=1g, base_to_total_min = 0.6g*/
#define base_to_total_min(base) 		(3*base/5)
/* for example: base=1g, single_axis_max = 1.8g*/
#define base_to_single_axis_max(base) 	(9*base/5)


struct lis_gsensor_test_excep{
	int total_times;
	int exception_times;
	int i2c_scl_val;			/* when i2c transfer err, read the gpio value*/
	int i2c_sda_val;
	int vdd_mv;
	int vddio_mv;
	int i2c_err_num;			/*i2c transfer err number*/
	int excep_num;				/* the error number the dsm_client identified*/
	unsigned char reg_buf[6]; 	/* 0 - 6 : CTRL_REG1 ~ CTRL_REG6*/
	bool triger_timer_flag;		/*Allow to trigger the timer to judge exception or not */
	bool triger_count_flag;		/*Allow to trigger counting the exception and total times */
	int excep_base; 			/* exception base value to calculate x_y_z_max and single_axis_max */
	int x_y_z_max;				/*exception of	(x+y+z) max value*/
	int x_y_z_min;				/*exception of	(|x|+|y|+|z|) min value*/
	int single_axis_max;		/* single axis max value*/
	int cur_err_x;				/* error x,y,z value*/
	int cur_err_y;
	int cur_err_z;
	int xyz[3];					/*  current x,y,z for forc dump to read*/
	s16 pre_xyz[3];
	int same_times;
	int x_same_times;
	int y_same_times;
	int z_same_times;
	int error_times;
};

struct lis_gs_dsm_operation{
	int (*dump_i2c_status)(struct lis3dh_acc_data *acceld,int err);
	int (*judge_check_excep)(int *xyz, struct lis3dh_acc_data *acceld );
	void (*judge_same_value_excep)(s16 x,s16 y,s16 z,struct lis3dh_acc_data *acceld);
};


int lis_dsm_excep_init(struct lis3dh_acc_data  *acc);
void lis_dsm_excep_exit(struct lis3dh_acc_data  *acc);
#endif

struct sensor_regulator {
	struct regulator *vreg;
	const char *name;
	u32	min_uV;
	u32	max_uV;
};
struct lis3dh_acc_data {
	struct i2c_client *client;
	struct lis3dh_acc_platform_data *pdata;
	struct sensors_classdev cdev;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
	struct pinctrl_state *pin_sleep;

	struct mutex lock;
	struct delayed_work input_work;
	#ifdef CONFIG_HUAWEI_KERNEL
	struct delayed_work debug_work;
	bool	queued_debug_work_flag;
	bool print_xyz_flag;
	#endif

	struct input_dev *input_dev;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;
	int on_before_suspend;

	u8 sensitivity;

	u8 resume_state[RESUME_ENTRIES];
	
	/* batch mode is configured */
	bool use_batch;
	unsigned int delay_ms;
	unsigned int batch_mode;
	unsigned int fifo_timeout_ms;
	unsigned int flush_count;
	
	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;

	u8 reg_addr;

	struct mutex lock_i2c;							/* lock for i2c tranfer*/
	struct sensor_regulator lis3dh_acc_vreg[2];		/* the sensor's two regulators*/
	int (*i2c_read)(struct lis3dh_acc_data *acc,	/* i2c read callback func*/
					u8 *buf, int len);
	bool device_exist;

#ifdef CONFIG_HUAWEI_DSM
	/* some callback func to Calculate and report exception*/
	struct lis_gs_dsm_operation lis3dh_dsm_operation;
	struct timer_list gsensor_excep_timer;          /* timer for Calculate anomaly */
	struct lis_gsensor_test_excep lis_gs_exception; /* store exception data*/
	struct work_struct  excep_work;                 /* work for report exception*/
	struct dsm_client* gsensor_dclient;             /* the dsm_client */
	struct workqueue_struct* excep_workqueue;       /* workqueue to schedule exception work*/
	char* dsm_buf;          						/* buf to record error or exception */
#endif


};

#endif

#endif	/* __KERNEL__ */

#endif	/* __LIS3DH_H__ */



