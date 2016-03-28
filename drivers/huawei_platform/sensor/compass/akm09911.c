/*upload the qulacom patch to realize compass MMI test*/
/* drivers/misc/akm09911.c - akm09911 compass driver
 *
 * Copyright (C) 2007-2008 HTC Corporation.
 * Author: Hou-Kun Chen <houkun.chen@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*#define DEBUG*/
/*#define VERBOSE_DEBUG*/


#include <linux/delay.h>
#include <linux/device.h>
#include <linux/freezer.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/sensors.h>
#include <huawei_platform/sensor/akm09911.h>
#include <huawei_platform/sensor/hw_sensor_info.h>
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif
#ifdef CONFIG_HUAWEI_DSM
#include 	<linux/dsm_pub.h>
#endif
#include <misc/app_info.h>
#ifdef CONFIG_HUAWEI_KERNEL
#define AKM_DEBUG_IF			1
#else
#define AKM_DEBUG_IF			0
#endif
#define AKM_HAS_RESET			1
#define AKM_INPUT_DEVICE_NAME	"compass"
#define AKM_DRDY_TIMEOUT_MS		100
#define AKM_BASE_NUM			10

#define AKM_IS_MAG_DATA_ENABLED() (akm->enable_flag & (1 << MAG_DATA_FLAG))

/* POWER SUPPLY VOLTAGE RANGE */
#define AKM09911_VDD_MIN_UV	2000000
#define AKM09911_VDD_MAX_UV	3300000
#define AKM09911_VIO_MIN_UV	1750000
#define AKM09911_VIO_MAX_UV	1950000

#define STATUS_ERROR(st)		(((st)&0x08) != 0x0)

/* Save last device state for power down */
struct akm_sensor_state {
	bool power_on;
	uint8_t mode;
};

struct akm_compass_data {
	struct i2c_client	*i2c;
	struct input_dev	*input;
	struct device		*class_dev;
	struct class		*compass;
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pin_default;
	struct pinctrl_state	*pin_sleep;
	struct sensors_classdev	cdev;
	struct delayed_work	dwork;
	struct workqueue_struct	*work_queue;
	struct mutex		op_mutex;
	struct mutex		self_test_mutex;

	wait_queue_head_t	drdy_wq;
	wait_queue_head_t	open_wq;

	/* These two buffers are initialized at start up.
	   After that, the value is not changed */
	uint8_t sense_info[AKM_SENSOR_INFO_SIZE];
	uint8_t sense_conf[AKM_SENSOR_CONF_SIZE];

	struct	mutex sensor_mutex;
	uint8_t	sense_data[AKM_SENSOR_DATA_SIZE];
	struct mutex accel_mutex;
	int16_t accel_data[3];

	struct mutex	val_mutex;
	uint32_t		enable_flag;
	int64_t			delay[AKM_NUM_SENSORS];

	atomic_t	active;
	atomic_t	drdy;

	char	layout;
	int	irq;
	int	gpio_rstn;
	int	power_enabled;
	int	auto_report;
	int	use_hrtimer;

	/* The input event last time */
	int	last_x;
	int	last_y;
	int	last_z;

	/* dummy value to avoid sensor event get eaten */
	int	rep_cnt;
	bool	rst_not_use_gpio;
	struct regulator	*vdd;
	struct regulator	*vio;
	struct akm_sensor_state state;
	struct hrtimer	poll_timer;
	bool device_exist;
	/* i2c gpio */
	unsigned int i2c_scl_gpio;
	unsigned int i2c_sda_gpio;
};

static struct sensors_classdev sensors_cdev = {
	.name = "akm09911-mag",
	.vendor = "Asahi Kasei Microdevices Corporation",
	.version = 1,
	.handle = SENSORS_MAGNETIC_FIELD_HANDLE,
	.type = SENSOR_TYPE_MAGNETIC_FIELD,
	.max_range = "1228.8",
	.resolution = "0.6",
	.sensor_power = "0.35",
	.min_delay = 10000,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 10,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct akm_compass_data *s_akm;

static int akm_compass_power_set(struct akm_compass_data *data, bool on);
#ifdef CONFIG_HUAWEI_KERNEL
#define	SENSOR_I2C_SCL	 909
#define	SENSOR_I2C_SDA	908
#define I2C_RETRY_COUNT 3

static int akm_i2c_rxdata(struct i2c_client *i2c,uint8_t *rxData,int length);
static ssize_t akm_buf_print(char *buf, uint8_t *data, size_t num);

#ifdef CONFIG_HUAWEI_DSM
#define CLIENT_NAME_MS_AKM			"dsm_ms_akm09911"
static void akm_gpios_show(struct akm_compass_data *akm);
static void akm_regulators_show(struct akm_compass_data *akm);
static void akm_regs_show(struct device *dev, char *buf);


struct akm_excep_data{
	int vdd_status;
	int vdd_mv;
	int vio_status;
	int vio_mv;
	int rst_val;
	int scl_val;
	int sda_val;
	int i2c_err_no;
	int excep_err;
	char *reg_buf;
	bool data_ready;    //mark the data is ready or not
	struct mutex excep_lock;
	int same_val_times;
	int pre_xyz[3];
};

static struct akm_excep_data *akm09911_excep = NULL;

/* dsm client for m-sensor */
static struct dsm_dev dsm_ms_akm09911 = {
	.name = CLIENT_NAME_MS_AKM,	// dsm client name
	.fops = NULL,		// options
	.buff_size = DSM_SENSOR_BUF_MAX,		// buffer size
};
static struct dsm_client *akm09911_ms_dclient = NULL;


static int akm_dsm_excep_init(struct akm_compass_data *data)
{
	int ret = -1;

	akm09911_excep = kzalloc(sizeof(struct akm_excep_data),GFP_KERNEL);
	if(!akm09911_excep){
		pr_err("[akm_err] %s failed alloc mem for akm09911_excep.\n",__func__);
		ret = -ENOMEM;
		return ret;
	}

	akm09911_excep->reg_buf = kzalloc(DSM_SENSOR_BUF_COM, GFP_ATOMIC);
	if (akm09911_excep->reg_buf == NULL) {
		pr_err("%s, %d: kzalloc failed.\n", __FUNCTION__, __LINE__);
		ret = -EEXIST;
		goto err_free_excep_data;
	}

	akm09911_ms_dclient = dsm_register_client(&dsm_ms_akm09911);
	if (!akm09911_ms_dclient) {
		pr_err("[akm_err]register dms akm09911_ms_dclient failed!");
		ret = -ENOMEM;
		goto err_free_reg_buf;
	}

	akm09911_ms_dclient->driver_data = data;

	mutex_init(&akm09911_excep->excep_lock);

	return 0;
err_free_reg_buf:
	kfree(akm09911_excep->reg_buf);
	akm09911_excep->reg_buf = NULL;
err_free_excep_data:
	kfree(akm09911_excep);
	akm09911_excep = NULL;
	return ret;

}

static void akm_dsm_excep_exit(void)
{
	dsm_unregister_client(akm09911_ms_dclient, &dsm_ms_akm09911);

	kfree(akm09911_excep->reg_buf);
	akm09911_excep->reg_buf = NULL;

	kfree(akm09911_excep);
	akm09911_excep = NULL;

}

static ssize_t ms_dsm_record_i2c_err_info(struct akm_compass_data *data)
{
	struct akm_excep_data *excep = akm09911_excep;
	ssize_t size = 0;
	ssize_t total_size = 0;

	akm_regulators_show(data);
	akm_gpios_show(data);

	size = dsm_client_record(akm09911_ms_dclient,
				"vdd_status = %d, vdd_mv = %d, vio_status = %d, vio_mv = %d \n"
				"res_pin_val = %d, scl_pin = %d, sda_pin = %d, i2c_err_no =%d, excep_err = %d\n",
				excep->vdd_status,excep->vdd_mv,excep->vio_status,excep->vio_mv,
				excep->rst_val,excep->scl_val,excep->sda_val,excep->i2c_err_no,excep->excep_err);
	total_size += size;

	/* clear i2c transfer error number before next report*/
	akm09911_excep->i2c_err_no = 0;

	return total_size;
}

static void akm_read_data_regs(struct akm_compass_data *akm)
{
	int err;
	uint8_t buffer[AKM_SENSOR_DATA_SIZE];

	/* Read rest data */
	buffer[0] = AKM_REG_STATUS;
	err = akm_i2c_rxdata(akm->i2c, buffer, AKM_SENSOR_DATA_SIZE);
	if (err < 0) {
		dev_err(&akm->i2c->dev, "%s failed.", __func__);
		return ;
	}

	atomic_set(&akm->drdy, 0);
	/* print registers value with Fixed format to reg_buf*/
	memset(akm09911_excep->reg_buf,0,DSM_SENSOR_BUF_COM);
	akm_buf_print(akm09911_excep->reg_buf, buffer, AKM_SENSOR_DATA_SIZE);

}
static ssize_t ms_dsm_record_data_err_info(struct akm_compass_data *data,const char* extern_info)
{
	ssize_t size = 0;
	ssize_t total_size = 0;
	char *buf = akm09911_excep->reg_buf;

	mutex_lock(&akm09911_excep->excep_lock);
	/* get data registers value*/
	akm_read_data_regs(data);
	/* show kernel debug message*/
	akm_regs_show(&data->i2c->dev,buf);

	size = dsm_client_record(akm09911_ms_dclient,buf);

	mutex_unlock(&akm09911_excep->excep_lock);
	total_size += size;

	if (NULL != extern_info) {
		if(strlen(extern_info) < (DSM_SENSOR_BUF_COM - total_size)){
			size += dsm_client_record(akm09911_ms_dclient, extern_info);
			total_size += size;
		}else{
			pr_err("[akm_err]ms_dsm_record_data_err_info extern_info is too long!\n");
		}
	}

	size = ms_dsm_record_i2c_err_info(data);
	total_size += size;

	return total_size;
}


static ssize_t ms_dsm_record_same_data_err_info(struct akm_compass_data* data)
{
	int size = 0;
	size = ms_dsm_record_data_err_info(data,NULL);
	size += dsm_client_record(akm09911_ms_dclient,
							  "same val x = %d, y = %d , z = %d",
							  akm09911_excep->pre_xyz[0],
							  akm09911_excep->pre_xyz[1],
							  akm09911_excep->pre_xyz[2]
							 );

	return size;

}

/**
 * func - report err according to err type
 * NOTE:
 */
static ssize_t ms_report_dsm_err(int type, const char* extern_info)
{
	int used_size = 0;
	struct akm_compass_data *akm = akm09911_ms_dclient->driver_data;

	/* try to get permission to use the buffer */
	if (dsm_client_ocuppy(akm09911_ms_dclient)) {
		/* buffer is busy */
		pr_err("[akm_err]%s: buffer is busy!", __func__);
		return -EBUSY;
	}
	akm09911_excep->excep_err = type;
	/* m-sensor report err according to err type */
	switch (type) {
		case DSM_MS_I2C_ERROR:
			/* report i2c infomation */
			used_size = ms_dsm_record_i2c_err_info(akm);
			break;

		case DSM_MS_DATA_TIMES_NOTCHANGE_ERROR:
			ms_dsm_record_same_data_err_info(akm);
			break;

		case DSM_MS_SELF_TEST_ERROR:
		case DSM_MS_DATA_ERROR:
			/* report gsensor basic infomation */
			used_size = ms_dsm_record_data_err_info(akm,extern_info);
			break;

		default:
			pr_err("[akm_err]%s:unsupported dsm report type.\n",__func__);
			break;
	}

	/*if device is not probe successfully or client is null, don't notify dsm work func*/
	if(s_akm->device_exist != true || akm09911_ms_dclient == NULL){
			pr_err("[akm_err]device is not exist!\n");
			return -ENODEV;
	}
	dsm_client_notify(akm09911_ms_dclient, type);

	return used_size;
}

static void akm_report_i2c_err(int i2c_ret)

{
	akm09911_excep->i2c_err_no = i2c_ret;
	ms_report_dsm_err(DSM_MS_I2C_ERROR,NULL);

}


static void akm_dsm_copy_reg_buf(unsigned char* buffer)
{
	/* print registers value with Fixed format to reg_buf*/
	mutex_lock(&akm09911_excep->excep_lock);
	memset(akm09911_excep->reg_buf, 0, DSM_SENSOR_BUF_COM);
	akm_buf_print(akm09911_excep->reg_buf, buffer, AKM_SENSOR_DATA_SIZE);
	mutex_unlock(&akm09911_excep->excep_lock);

}


/*****************************************************************
Parameters    :  x y z  the source value which are readed from register

Description   :  if current x,y,z are the same as last time's, same_val_times++
			if same_val_times is serial added to 15, report error info to dsm
*****************************************************************/
static void ms_dsm_check_val_same_times(int x, int y, int z)
{
	int temp_xyz[3] = {x, y, z};

	if (akm09911_excep->pre_xyz[0] == x
		&&  akm09911_excep->pre_xyz[1] == y
		&&  akm09911_excep->pre_xyz[2] == z
		&&  akm09911_excep->data_ready) {
		akm09911_excep->same_val_times ++;
	} else {
		/* if last val don't equal this time, clear same_val_times*/
		akm09911_excep->same_val_times = 0;
	}

	if (akm09911_excep->same_val_times >= SENSOR_VAL_SAME_MAX_TIMES && akm09911_excep->data_ready) {
		akm09911_excep->same_val_times = 0;
		ms_report_dsm_err(DSM_MS_DATA_TIMES_NOTCHANGE_ERROR,NULL);
	}

	memcpy(akm09911_excep->pre_xyz, temp_xyz, sizeof(akm09911_excep->pre_xyz));

}
#endif
static int akm_i2c_rxdata(
	struct i2c_client *i2c,
	uint8_t *rxData,
	int length)
{
	int ret;
	int count = 0;

	struct i2c_msg msgs[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = i2c->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};
	uint8_t addr = rxData[0];

	ret = -1;
	while (ret != ARRAY_SIZE(msgs) && count < I2C_RETRY_COUNT) {
		ret = i2c_transfer(i2c->adapter, msgs, ARRAY_SIZE(msgs));
		count++;
	}

	if (ret < 0) {
		dev_err(&i2c->dev, "%s: transfer failed.", __func__);
		goto i2c_err;
	} else if (ret != ARRAY_SIZE(msgs)) {
		dev_err(&i2c->dev, "%s: transfer failed(size error).\n",
				__func__);
		ret = -ENXIO;
		goto i2c_err;
	}

	dev_vdbg(&i2c->dev, "RxData: len=%02x, addr=%02x, data=%02x",
		length, addr, rxData[0]);

	return 0;

i2c_err:

#ifdef CONFIG_HUAWEI_DSM
	akm_report_i2c_err(ret);
#endif
	return ret;

}


static int akm_i2c_txdata(
	struct i2c_client *i2c,
	uint8_t *txData,
	int length)
{
	int ret;
	int count = 0;

	struct i2c_msg msg[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	ret = -1;
	while (ret != ARRAY_SIZE(msg) && count < I2C_RETRY_COUNT) {
		ret = i2c_transfer(i2c->adapter, msg, ARRAY_SIZE(msg));
		count++;
	}

	if (ret < 0) {
		dev_err(&i2c->dev, "%s: transfer failed.", __func__);
		goto i2c_err;
	} else if (ret != ARRAY_SIZE(msg)) {
		dev_err(&i2c->dev, "%s: transfer failed(size error).",
				__func__);
		ret = -ENXIO;
		goto i2c_err;
	}

	dev_vdbg(&i2c->dev, "TxData: len=%02x, addr=%02x data=%02x",
		length, txData[0], txData[1]);

	return 0;

i2c_err:

#ifdef CONFIG_HUAWEI_DSM
	akm_report_i2c_err(ret);
#endif
	return ret;
}
#else

/***** I2C I/O function ***********************************************/
static int akm_i2c_rxdata(
	struct i2c_client *i2c,
	uint8_t *rxData,
	int length)
{
	int ret;
	struct i2c_msg msgs[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = i2c->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};
	uint8_t addr = rxData[0];
	ret = i2c_transfer(i2c->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: transfer failed.", __func__);
		return ret;
	} else if (ret != ARRAY_SIZE(msgs)) {
		dev_err(&i2c->dev, "%s: transfer failed(size error).\n",
				__func__);
		return -ENXIO;
	}

	dev_vdbg(&i2c->dev, "RxData: len=%02x, addr=%02x, data=%02x",
		length, addr, rxData[0]);

	return 0;
}

static int akm_i2c_txdata(
	struct i2c_client *i2c,
	uint8_t *txData,
	int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	ret = i2c_transfer(i2c->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: transfer failed.", __func__);
		return ret;
	} else if (ret != ARRAY_SIZE(msg)) {
		dev_err(&i2c->dev, "%s: transfer failed(size error).",
				__func__);
		return -ENXIO;
	}

	dev_vdbg(&i2c->dev, "TxData: len=%02x, addr=%02x data=%02x",
		length, txData[0], txData[1]);

	return 0;
}

#endif

/***** akm miscdevice functions *************************************/
static int AKECS_Set_CNTL(
	struct akm_compass_data *akm,
	uint8_t mode)
{
	uint8_t buffer[2];
	int err;

	/***** lock *****/
	mutex_lock(&akm->sensor_mutex);
	/* Set measure mode */
	buffer[0] = AKM_REG_MODE;
	buffer[1] = mode;
	err = akm_i2c_txdata(akm->i2c, buffer, 2);
	if (err < 0) {
		dev_err(&akm->i2c->dev,
				"%s: Can not set CNTL.", __func__);
	} else {
		dev_vdbg(&akm->i2c->dev,
				"Mode is set to (%d).", mode);
		atomic_set(&akm->drdy, 0);
		/* wait at least 100us after changing mode */
		udelay(100);
	}

	mutex_unlock(&akm->sensor_mutex);
	/***** unlock *****/

	return err;
}

static int AKECS_Set_PowerDown(
	struct akm_compass_data *akm)
{
	uint8_t buffer[2];
	int err;

	/***** lock *****/
	mutex_lock(&akm->sensor_mutex);

	/* Set powerdown mode */
	buffer[0] = AKM_REG_MODE;
	buffer[1] = AKM_MODE_POWERDOWN;
	err = akm_i2c_txdata(akm->i2c, buffer, 2);
	if (err < 0) {
		dev_err(&akm->i2c->dev,
			"%s: Can not set to powerdown mode.", __func__);
	} else {
		dev_dbg(&akm->i2c->dev, "Powerdown mode is set.");
		/* wait at least 100us after changing mode */
		udelay(100);
	}
	atomic_set(&akm->drdy, 0);

	mutex_unlock(&akm->sensor_mutex);
	/***** unlock *****/

	return err;
}

static int AKECS_Reset(
	struct akm_compass_data *akm,
	int hard)
{
	int err;

#if AKM_HAS_RESET
	uint8_t buffer[2];

	/***** lock *****/
	mutex_lock(&akm->sensor_mutex);

	if (hard != 0) {
		gpio_set_value(akm->gpio_rstn, 0);
		udelay(5);
		gpio_set_value(akm->gpio_rstn, 1);
		/* No error is returned */
		err = 0;
	} else {
		buffer[0] = AKM_REG_RESET;
		buffer[1] = AKM_RESET_DATA;
		err = akm_i2c_txdata(akm->i2c, buffer, 2);
		if (err < 0) {
			dev_err(&akm->i2c->dev,
				"%s: Can not set SRST bit.", __func__);
		} else {
			dev_dbg(&akm->i2c->dev, "Soft reset is done.");
		}
	}
	/* Device will be accessible 100 us after */
	udelay(100);
	atomic_set(&akm->drdy, 0);
	mutex_unlock(&akm->sensor_mutex);
	/***** unlock *****/

#else
	err = AKECS_Set_PowerDown(akm);
#endif

	return err;
}

static int AKECS_SetMode(
	struct akm_compass_data *akm,
	uint8_t mode)
{
	int err;

	switch (mode & 0x1F) {
	case AKM_MODE_SNG_MEASURE:
	case AKM_MODE_SELF_TEST:
	case AKM_MODE_FUSE_ACCESS:
	case AKM_MODE_CONTINUOUS_10HZ:
	case AKM_MODE_CONTINUOUS_20HZ:
	case AKM_MODE_CONTINUOUS_50HZ:
	case AKM_MODE_CONTINUOUS_100HZ:
		err = AKECS_Set_CNTL(akm, mode);
		break;
	case AKM_MODE_POWERDOWN:
		err = AKECS_Set_PowerDown(akm);
		break;
	default:
		dev_err(&akm->i2c->dev,
			"%s: Unknown mode(%d).", __func__, mode);
		return -EINVAL;
	}

	return err;
}

static void AKECS_SetYPR(
	struct akm_compass_data *akm,
	int *rbuf)
{
	uint32_t ready;
	dev_vdbg(&akm->i2c->dev, "%s: flag =0x%X", __func__, rbuf[0]);
	dev_vdbg(&akm->input->dev, "  Acc [LSB]   : %6d,%6d,%6d stat=%d",
		rbuf[1], rbuf[2], rbuf[3], rbuf[4]);
	dev_vdbg(&akm->input->dev, "  Geo [LSB]   : %6d,%6d,%6d stat=%d",
		rbuf[5], rbuf[6], rbuf[7], rbuf[8]);
	dev_vdbg(&akm->input->dev, "  Orientation : %6d,%6d,%6d",
		rbuf[9], rbuf[10], rbuf[11]);
	dev_vdbg(&akm->input->dev, "  Rotation V  : %6d,%6d,%6d,%6d",
		rbuf[12], rbuf[13], rbuf[14], rbuf[15]);

	/* No events are reported */
	if (!rbuf[0]) {
		dev_dbg(&akm->i2c->dev, "Don't waste a time.");
		return;
	}

	mutex_lock(&akm->val_mutex);
	ready = (akm->enable_flag & (uint32_t)rbuf[0]);
	mutex_unlock(&akm->val_mutex);

	/* Report acceleration sensor information */
	if (ready & ACC_DATA_READY) {
		input_report_abs(akm->input, ABS_X, rbuf[1]);
		input_report_abs(akm->input, ABS_Y, rbuf[2]);
		input_report_abs(akm->input, ABS_Z, rbuf[3]);
		input_report_abs(akm->input, ABS_RX, rbuf[4]);
	}
	/* Report magnetic vector information */
	if (ready & MAG_DATA_READY) {
		input_report_abs(akm->input, ABS_X, rbuf[5]);
		input_report_abs(akm->input, ABS_Y, rbuf[6]);
		input_report_abs(akm->input, ABS_Z, rbuf[7]);
		input_report_abs(akm->input, ABS_MISC, rbuf[8]);
	}
	/* Report fusion sensor information */
	if (ready & FUSION_DATA_READY) {
		/* Orientation */
		input_report_abs(akm->input, ABS_HAT0Y, rbuf[9]);
		input_report_abs(akm->input, ABS_HAT1X, rbuf[10]);
		input_report_abs(akm->input, ABS_HAT1Y, rbuf[11]);
		/* Rotation Vector */
		input_report_abs(akm->input, ABS_TILT_X, rbuf[12]);
		input_report_abs(akm->input, ABS_TILT_Y, rbuf[13]);
		input_report_abs(akm->input, ABS_TOOL_WIDTH, rbuf[14]);
		input_report_abs(akm->input, ABS_VOLUME, rbuf[15]);
	}

	input_sync(akm->input);
}

/* This function will block a process until the latest measurement
 * data is available.
 */
static int AKECS_GetData(
	struct akm_compass_data *akm,
	uint8_t *rbuf,
	int size)
{
	int err;

	/* Block! */
	err = wait_event_interruptible_timeout(
			akm->drdy_wq,
			atomic_read(&akm->drdy),
			msecs_to_jiffies(AKM_DRDY_TIMEOUT_MS));

	if (err < 0) {
		dev_err(&akm->i2c->dev,
			"%s: wait_event failed (%d).", __func__, err);
		return err;
	}
	if (!atomic_read(&akm->drdy)) {
		dev_err(&akm->i2c->dev,
			"%s: DRDY is not set.", __func__);
		return -ENODATA;
	}

	/***** lock *****/
	mutex_lock(&akm->sensor_mutex);

	memcpy(rbuf, akm->sense_data, size);
	atomic_set(&akm->drdy, 0);

	mutex_unlock(&akm->sensor_mutex);
	/***** unlock *****/

	return 0;
}
static int AKECS_GetData_Poll(
	struct akm_compass_data *akm,
	uint8_t *rbuf,
	int size,
	bool is_self_test)
{
	uint8_t buffer[AKM_SENSOR_DATA_SIZE];
	int err;

	/* Read status */
	buffer[0] = AKM_REG_STATUS;
	err = akm_i2c_rxdata(akm->i2c, buffer, 1);
	if (err < 0) {
		dev_err(&akm->i2c->dev, "%s read status failed.", __func__);
		return err;
	}
	/* Check ST bit */
	if (!(AKM_DRDY_IS_HIGH(buffer[0])))
	{
		akm09911_excep->data_ready = false;
		/*if compass is in self test mode, poll again; in other mode, get data to report*/
		if(is_self_test)
		{
			return -EAGAIN;
		}
	}
	else
	{
		akm09911_excep->data_ready = true;
	}

	/* Data is over run is */
	if (AKM_DOR_IS_HIGH(buffer[0]))
		dev_dbg(&akm->i2c->dev, "Data over run!\n");

	/* Read rest data */
	buffer[1] = AKM_REG_STATUS + 1;
	err = akm_i2c_rxdata(akm->i2c, &(buffer[1]), AKM_SENSOR_DATA_SIZE-1);
	if (err < 0) {
		dev_err(&akm->i2c->dev, "%s failed.", __func__);
		return err;
	}

#ifdef CONFIG_HUAWEI_DSM
	akm_dsm_copy_reg_buf(buffer);
#endif

	memcpy(rbuf, buffer, size);
	atomic_set(&akm->drdy, 0);

	return 0;
}

static int AKECS_GetOpenStatus(
	struct akm_compass_data *akm)
{
	return wait_event_interruptible(
			akm->open_wq, (atomic_read(&akm->active) > 0));
}

static int AKECS_GetCloseStatus(
	struct akm_compass_data *akm)
{
	return wait_event_interruptible(
			akm->open_wq, (atomic_read(&akm->active) <= 0));
}

static int AKECS_Open(struct inode *inode, struct file *file)
{
	file->private_data = s_akm;
	return nonseekable_open(inode, file);
}

static int AKECS_Release(struct inode *inode, struct file *file)
{
	return 0;
}

static long
AKECS_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct akm_compass_data *akm = file->private_data;

	/* NOTE: In this function the size of "char" should be 1-byte. */
	uint8_t i2c_buf[AKM_RWBUF_SIZE];		/* for READ/WRITE */
	uint8_t dat_buf[AKM_SENSOR_DATA_SIZE];/* for GET_DATA */
	int32_t ypr_buf[AKM_YPR_DATA_SIZE];		/* for SET_YPR */
	int64_t delay[AKM_NUM_SENSORS];	/* for GET_DELAY */
	int16_t acc_buf[3];	/* for GET_ACCEL */
	uint8_t mode;			/* for SET_MODE*/
	int status;			/* for OPEN/CLOSE_STATUS */
	int ret = 0;		/* Return value. */

	switch (cmd) {
	case ECS_IOCTL_READ:
	case ECS_IOCTL_WRITE:
		if (argp == NULL) {
			dev_err(&akm->i2c->dev, "invalid argument.");
			return -EINVAL;
		}
		if (copy_from_user(&i2c_buf, argp, sizeof(i2c_buf))) {
			dev_err(&akm->i2c->dev, "copy_from_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_SET_MODE:
		if (argp == NULL) {
			dev_err(&akm->i2c->dev, "invalid argument.");
			return -EINVAL;
		}
		if (copy_from_user(&mode, argp, sizeof(mode))) {
			dev_err(&akm->i2c->dev, "copy_from_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_SET_YPR:
		if (argp == NULL) {
			dev_err(&akm->i2c->dev, "invalid argument.");
			return -EINVAL;
		}
		if (copy_from_user(&ypr_buf, argp, sizeof(ypr_buf))) {
			dev_err(&akm->i2c->dev, "copy_from_user failed.");
			return -EFAULT;
		}
	case ECS_IOCTL_GET_INFO:
	case ECS_IOCTL_GET_CONF:
	case ECS_IOCTL_GET_DATA:
	case ECS_IOCTL_GET_OPEN_STATUS:
	case ECS_IOCTL_GET_CLOSE_STATUS:
	case ECS_IOCTL_GET_DELAY:
	case ECS_IOCTL_GET_LAYOUT:
	case ECS_IOCTL_GET_ACCEL:
		/* Check buffer pointer for writing a data later. */
		if (argp == NULL) {
			dev_err(&akm->i2c->dev, "invalid argument.");
			return -EINVAL;
		}
		break;
	default:
		break;
	}

	switch (cmd) {
	case ECS_IOCTL_READ:
		dev_vdbg(&akm->i2c->dev, "IOCTL_READ called.");
		if ((i2c_buf[0] < 1) || (i2c_buf[0] > (AKM_RWBUF_SIZE-1))) {
			dev_err(&akm->i2c->dev, "invalid argument.");
			return -EINVAL;
		}
		ret = akm_i2c_rxdata(akm->i2c, &i2c_buf[1], i2c_buf[0]);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_WRITE:
		dev_vdbg(&akm->i2c->dev, "IOCTL_WRITE called.");
		if ((i2c_buf[0] < 2) || (i2c_buf[0] > (AKM_RWBUF_SIZE-1))) {
			dev_err(&akm->i2c->dev, "invalid argument.");
			return -EINVAL;
		}
		ret = akm_i2c_txdata(akm->i2c, &i2c_buf[1], i2c_buf[0]);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_RESET:
		dev_vdbg(&akm->i2c->dev, "IOCTL_RESET called.");
		if(s_akm->rst_not_use_gpio)
		{
			ret = AKECS_Reset(s_akm, 0);
		}
		else
		{
			ret = AKECS_Reset(akm, akm->gpio_rstn);
		}
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_SET_MODE:
		dev_vdbg(&akm->i2c->dev, "IOCTL_SET_MODE called.");
		ret = AKECS_SetMode(akm, mode);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_SET_YPR:
		dev_vdbg(&akm->i2c->dev, "IOCTL_SET_YPR called.");
		AKECS_SetYPR(akm, ypr_buf);
		break;
	case ECS_IOCTL_GET_DATA:
		dev_vdbg(&akm->i2c->dev, "IOCTL_GET_DATA called.");
		if (akm->irq)
			ret = AKECS_GetData(akm, dat_buf, AKM_SENSOR_DATA_SIZE);
		else
			ret = AKECS_GetData_Poll(
					akm, dat_buf, AKM_SENSOR_DATA_SIZE, false);

		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_GET_OPEN_STATUS:
		dev_vdbg(&akm->i2c->dev, "IOCTL_GET_OPEN_STATUS called.");
		ret = AKECS_GetOpenStatus(akm);
		if (ret < 0) {
			dev_err(&akm->i2c->dev,
				"Get Open returns error (%d).", ret);
			return ret;
		}
		break;
	case ECS_IOCTL_GET_CLOSE_STATUS:
		dev_vdbg(&akm->i2c->dev, "IOCTL_GET_CLOSE_STATUS called.");
		ret = AKECS_GetCloseStatus(akm);
		if (ret < 0) {
			dev_err(&akm->i2c->dev,
				"Get Close returns error (%d).", ret);
			return ret;
		}
		break;
	case ECS_IOCTL_GET_DELAY:
		dev_vdbg(&akm->i2c->dev, "IOCTL_GET_DELAY called.");
		mutex_lock(&akm->val_mutex);
		delay[0] = ((akm->enable_flag & ACC_DATA_READY) ?
				akm->delay[0] : -1);
		delay[1] = ((akm->enable_flag & MAG_DATA_READY) ?
				akm->delay[1] : -1);
		delay[2] = ((akm->enable_flag & FUSION_DATA_READY) ?
				akm->delay[2] : -1);
		mutex_unlock(&akm->val_mutex);
		break;
	case ECS_IOCTL_GET_INFO:
		dev_vdbg(&akm->i2c->dev, "IOCTL_GET_INFO called.");
		break;
	case ECS_IOCTL_GET_CONF:
		dev_vdbg(&akm->i2c->dev, "IOCTL_GET_CONF called.");
		break;
	case ECS_IOCTL_GET_LAYOUT:
		dev_vdbg(&akm->i2c->dev, "IOCTL_GET_LAYOUT called.");
		break;
	case ECS_IOCTL_GET_ACCEL:
		dev_vdbg(&akm->i2c->dev, "IOCTL_GET_ACCEL called.");
		mutex_lock(&akm->accel_mutex);
		acc_buf[0] = akm->accel_data[0];
		acc_buf[1] = akm->accel_data[1];
		acc_buf[2] = akm->accel_data[2];
		mutex_unlock(&akm->accel_mutex);
		break;
	default:
		return -ENOTTY;
	}

	switch (cmd) {
	case ECS_IOCTL_READ:
		/* +1  is for the first byte */
		if (copy_to_user(argp, &i2c_buf, i2c_buf[0]+1)) {
			dev_err(&akm->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_INFO:
		if (copy_to_user(argp, &akm->sense_info,
					sizeof(akm->sense_info))) {
			dev_err(&akm->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_CONF:
		if (copy_to_user(argp, &akm->sense_conf,
					sizeof(akm->sense_conf))) {
			dev_err(&akm->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_DATA:
		if (copy_to_user(argp, &dat_buf, sizeof(dat_buf))) {
			dev_err(&akm->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_OPEN_STATUS:
	case ECS_IOCTL_GET_CLOSE_STATUS:
		status = atomic_read(&akm->active);
		if (copy_to_user(argp, &status, sizeof(status))) {
			dev_err(&akm->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_DELAY:
		if (copy_to_user(argp, &delay, sizeof(delay))) {
			dev_err(&akm->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_LAYOUT:
		if (copy_to_user(argp, &akm->layout, sizeof(akm->layout))) {
			dev_err(&akm->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_ACCEL:
		if (copy_to_user(argp, &acc_buf, sizeof(acc_buf))) {
			dev_err(&akm->i2c->dev, "copy_to_user failed.");
			return -EFAULT;
		}
		break;
	default:
		break;
	}

	return 0;
}

static const struct file_operations AKECS_fops = {
	.owner = THIS_MODULE,
	.open = AKECS_Open,
	.release = AKECS_Release,
	.unlocked_ioctl = AKECS_ioctl,
};

static struct miscdevice akm_compass_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = AKM_MISCDEV_NAME,
	.fops = &AKECS_fops,
};

/***** akm sysfs functions ******************************************/
static int create_device_attributes(
	struct device *dev,
	struct device_attribute *attrs)
{
	int i;
	int err = 0;

	for (i = 0 ; NULL != attrs[i].attr.name ; ++i) {
		err = device_create_file(dev, &attrs[i]);
		if (err)
			break;
	}

	if (err) {
		for (--i; i >= 0 ; --i)
			device_remove_file(dev, &attrs[i]);
	}

	return err;
}

static void remove_device_attributes(
	struct device *dev,
	struct device_attribute *attrs)
{
	int i;

	for (i = 0 ; NULL != attrs[i].attr.name ; ++i)
		device_remove_file(dev, &attrs[i]);
}

static int create_device_binary_attributes(
	struct kobject *kobj,
	struct bin_attribute *attrs)
{
	int i;
	int err = 0;

	err = 0;

	for (i = 0 ; NULL != attrs[i].attr.name ; ++i) {
		err = sysfs_create_bin_file(kobj, &attrs[i]);
		if (0 != err)
			break;
	}

	if (0 != err) {
		for (--i; i >= 0 ; --i)
			sysfs_remove_bin_file(kobj, &attrs[i]);
	}

	return err;
}

static void remove_device_binary_attributes(
	struct kobject *kobj,
	struct bin_attribute *attrs)
{
	int i;

	for (i = 0 ; NULL != attrs[i].attr.name ; ++i)
		sysfs_remove_bin_file(kobj, &attrs[i]);
}

/*********************************************************************
 *
 * SysFS attribute functions
 *
 * directory : /sys/class/compass/akmXXXX/
 * files :
 *  - enable_acc    [rw] [t] : enable flag for accelerometer
 *  - enable_mag    [rw] [t] : enable flag for magnetometer
 *  - enable_fusion [rw] [t] : enable flag for fusion sensor
 *  - delay_acc     [rw] [t] : delay in nanosecond for accelerometer
 *  - delay_mag     [rw] [t] : delay in nanosecond for magnetometer
 *  - delay_fusion  [rw] [t] : delay in nanosecond for fusion sensor
 *
 * debug :
 *  - mode       [w]  [t] : E-Compass mode
 *  - bdata      [r]  [t] : buffered raw data
 *  - asa        [r]  [t] : FUSEROM data
 *  - regs       [r]  [t] : read all registers
 *
 * [b] = binary format
 * [t] = text format
 */

/***** sysfs enable *************************************************/
static void akm_compass_sysfs_update_status(
	struct akm_compass_data *akm)
{
	uint32_t en;
	mutex_lock(&akm->val_mutex);
	en = akm->enable_flag;
	mutex_unlock(&akm->val_mutex);

	if (en == 0) {
		if (atomic_cmpxchg(&akm->active, 1, 0) == 1) {
			wake_up(&akm->open_wq);
			dev_dbg(akm->class_dev, "Deactivated");
		}
	} else {
		if (atomic_cmpxchg(&akm->active, 0, 1) == 0) {
			wake_up(&akm->open_wq);
			dev_dbg(akm->class_dev, "Activated");
		}
	}
	dev_dbg(&akm->i2c->dev,
		"Status updated: enable=0x%X, active=%d",
		en, atomic_read(&akm->active));
}

static inline uint8_t akm_select_frequency(int64_t delay_ns)
{
	if (delay_ns >= 100000000LL)
		return AKM_MODE_CONTINUOUS_10HZ;
	else if (delay_ns >= 50000000LL)
		return AKM_MODE_CONTINUOUS_20HZ;
	else if (delay_ns >= 20000000LL)
		return AKM_MODE_CONTINUOUS_50HZ;
	else
		return AKM_MODE_CONTINUOUS_100HZ;
}

static int akm_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	int ret = 0;
	struct akm_compass_data *akm = container_of(sensors_cdev,
			struct akm_compass_data, cdev);
	uint8_t mode;
	dev_err(&akm->i2c->dev,"start akm_enable_set!\n");

	mutex_lock(&akm->val_mutex);
	akm->enable_flag &= ~(1<<MAG_DATA_FLAG);
	akm->enable_flag |= ((uint32_t)(enable))<<MAG_DATA_FLAG;
	mutex_unlock(&akm->val_mutex);

	akm_compass_sysfs_update_status(akm);
	mutex_lock(&akm->op_mutex);
	if (enable) {
		ret = akm_compass_power_set(akm, true);
		if (ret) {
			dev_err(&akm->i2c->dev,
				"Fail to power on the device!\n");
			goto exit;
		}

		if (akm->auto_report) {
			mode = akm_select_frequency(akm->delay[MAG_DATA_FLAG]);
			mutex_lock(&akm->self_test_mutex);
			AKECS_SetMode(akm, mode);
			mutex_unlock(&akm->self_test_mutex);
			if (akm->use_hrtimer)
				hrtimer_start(&akm->poll_timer,
					ns_to_ktime(akm->delay[MAG_DATA_FLAG]),
					HRTIMER_MODE_REL);
			else
				queue_delayed_work(akm->work_queue, &akm->dwork,
					(unsigned long)nsecs_to_jiffies64(
						akm->delay[MAG_DATA_FLAG]));
		}
		dev_err(&akm->i2c->dev,"power on the device!\n");
	} else {
		if (akm->auto_report) {
			if (akm->use_hrtimer) {
				hrtimer_cancel(&akm->poll_timer);
				cancel_work_sync(&akm->dwork.work);
			} else {
				cancel_delayed_work_sync(&akm->dwork);
			}
			mutex_lock(&akm->self_test_mutex);
			AKECS_SetMode(akm, AKM_MODE_POWERDOWN);
			mutex_unlock(&akm->self_test_mutex);
		}
		ret = akm_compass_power_set(akm, false);
		if (ret) {
			dev_err(&akm->i2c->dev,
				"Fail to power off the device!\n");
			goto exit;
		}
		dev_err(&akm->i2c->dev,"power off the device!\n");
	}

exit:
	mutex_unlock(&akm->op_mutex);
	return ret;
}

static ssize_t akm_compass_sysfs_enable_show(
	struct akm_compass_data *akm, char *buf, int pos)
{
	int flag;

	mutex_lock(&akm->val_mutex);
	flag = ((akm->enable_flag >> pos) & 1);
	mutex_unlock(&akm->val_mutex);

	return scnprintf(buf, PAGE_SIZE, "%d\n", flag);
}

static ssize_t akm_compass_sysfs_enable_store(
	struct akm_compass_data *akm, char const *buf, size_t count, int pos)
{
	long en = 0;
	int ret = 0;

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	if (strict_strtol(buf, AKM_BASE_NUM, &en))
		return -EINVAL;

	en = en ? 1 : 0;

	mutex_lock(&akm->op_mutex);
	ret = akm_compass_power_set(akm, en);
	if (ret) {
		dev_err(&akm->i2c->dev,
			"Fail to configure device power!\n");
		goto exit;
	}
	mutex_lock(&akm->val_mutex);
	akm->enable_flag &= ~(1<<pos);
	akm->enable_flag |= ((uint32_t)(en))<<pos;
	mutex_unlock(&akm->val_mutex);

	akm_compass_sysfs_update_status(akm);

exit:
	mutex_unlock(&akm->op_mutex);

	return ret ? ret : count;
}

/***** Acceleration ***/
static ssize_t akm_enable_acc_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_enable_show(
		dev_get_drvdata(dev), buf, ACC_DATA_FLAG);
}
static ssize_t akm_enable_acc_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_enable_store(
		dev_get_drvdata(dev), buf, count, ACC_DATA_FLAG);
}

/***** Magnetic field ***/
static ssize_t akm_enable_mag_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_enable_show(
		dev_get_drvdata(dev), buf, MAG_DATA_FLAG);
}
static ssize_t akm_enable_mag_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_enable_store(
		dev_get_drvdata(dev), buf, count, MAG_DATA_FLAG);
}

/***** Fusion ***/
static ssize_t akm_enable_fusion_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_enable_show(
		dev_get_drvdata(dev), buf, FUSION_DATA_FLAG);
}
static ssize_t akm_enable_fusion_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_enable_store(
		dev_get_drvdata(dev), buf, count, FUSION_DATA_FLAG);
}

/***** sysfs delay **************************************************/
static int akm_poll_delay_set(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
	struct akm_compass_data *akm = container_of(sensors_cdev,
			struct akm_compass_data, cdev);
	uint8_t mode;
	int ret;
	dev_warn(&akm->i2c->dev, "akm_poll_delay_set:%dms\n", delay_msec);
	mutex_lock(&akm->val_mutex);
	mutex_lock(&akm->self_test_mutex);
	akm->delay[MAG_DATA_FLAG] = delay_msec * 1000000;
	mode = akm_select_frequency(akm->delay[MAG_DATA_FLAG]);
	/*before change compass mode, set to power down mode first*/
	ret = AKECS_SetMode(akm, AKM_MODE_POWERDOWN);
	if(ret < 0)
	{
		dev_err(&akm->i2c->dev, "Failed to set to power down mode\n");
	}
	ret = AKECS_SetMode(akm, mode);
	if (ret < 0)
	{
		dev_err(&akm->i2c->dev, "Failed to set to mode(%x)\n", mode);
	}
	mutex_unlock(&akm->self_test_mutex);
	mutex_unlock(&akm->val_mutex);
	return ret;
}

static ssize_t akm_compass_sysfs_delay_show(
	struct akm_compass_data *akm, char *buf, int pos)
{
	int64_t val;

	mutex_lock(&akm->val_mutex);
	val = akm->delay[pos];
	mutex_unlock(&akm->val_mutex);

	return scnprintf(buf, PAGE_SIZE, "%lld\n", val);
}

static ssize_t akm_compass_sysfs_delay_store(
	struct akm_compass_data *akm, char const *buf, size_t count, int pos)
{
	long long val = 0;

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	if (strict_strtoll(buf, AKM_BASE_NUM, &val))
		return -EINVAL;

	mutex_lock(&akm->val_mutex);
	akm->delay[pos] = val;
	mutex_unlock(&akm->val_mutex);

	return count;
}

/***** Accelerometer ***/
static ssize_t akm_delay_acc_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_delay_show(
		dev_get_drvdata(dev), buf, ACC_DATA_FLAG);
}
static ssize_t akm_delay_acc_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_delay_store(
		dev_get_drvdata(dev), buf, count, ACC_DATA_FLAG);
}

/***** Magnetic field ***/
static ssize_t akm_delay_mag_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_delay_show(
		dev_get_drvdata(dev), buf, MAG_DATA_FLAG);
}
static ssize_t akm_delay_mag_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_delay_store(
		dev_get_drvdata(dev), buf, count, MAG_DATA_FLAG);
}

/***** Fusion ***/
static ssize_t akm_delay_fusion_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_delay_show(
		dev_get_drvdata(dev), buf, FUSION_DATA_FLAG);
}
static ssize_t akm_delay_fusion_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_delay_store(
		dev_get_drvdata(dev), buf, count, FUSION_DATA_FLAG);
}

/***** accel (binary) ***/
static ssize_t akm_bin_accel_write(
	struct file *file,
	struct kobject *kobj,
	struct bin_attribute *attr,
		char *buf,
		loff_t pos,
		size_t size)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	int16_t *accel_data;

	if (size == 0)
		return 0;

	accel_data = (int16_t *)buf;

	mutex_lock(&akm->accel_mutex);
	akm->accel_data[0] = accel_data[0];
	akm->accel_data[1] = accel_data[1];
	akm->accel_data[2] = accel_data[2];
	mutex_unlock(&akm->accel_mutex);

	dev_vdbg(&akm->i2c->dev, "accel:%d,%d,%d\n",
			accel_data[0], accel_data[1], accel_data[2]);

	return size;
}


#if AKM_DEBUG_IF
static ssize_t akm_sysfs_mode_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	long mode = 0;

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	if (strict_strtol(buf, AKM_BASE_NUM, &mode))
		return -EINVAL;

	if (AKECS_SetMode(akm, (uint8_t)mode) < 0)
		return -EINVAL;

	return 1;
}

static ssize_t akm_buf_print(
	char *buf, uint8_t *data, size_t num)
{
	int sz, i;
	char *cur;
	size_t cur_len;

	cur = buf;
	cur_len = PAGE_SIZE;
	sz = snprintf(cur, cur_len, "(HEX):");
	if (sz < 0)
		return sz;
	cur += sz;
	cur_len -= sz;
	for (i = 0; i < num; i++) {
		sz = snprintf(cur, cur_len, "%02X,", *data);
		if (sz < 0)
			return sz;
		cur += sz;
		cur_len -= sz;
		data++;
	}
	sz = snprintf(cur, cur_len, "\n");
	if (sz < 0)
		return sz;
	cur += sz;

	return (ssize_t)(cur - buf);
}

static ssize_t akm_sysfs_bdata_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	uint8_t rbuf[AKM_SENSOR_DATA_SIZE];

	mutex_lock(&akm->sensor_mutex);
	memcpy(&rbuf, akm->sense_data, sizeof(rbuf));
	mutex_unlock(&akm->sensor_mutex);

	return akm_buf_print(buf, rbuf, AKM_SENSOR_DATA_SIZE);
}

static ssize_t akm_sysfs_asa_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	int err;
	uint8_t asa[3];

	err = AKECS_SetMode(akm, AKM_MODE_FUSE_ACCESS);
	if (err < 0)
		return err;

	asa[0] = AKM_FUSE_1ST_ADDR;
	err = akm_i2c_rxdata(akm->i2c, asa, 3);
	if (err < 0)
		return err;

	err = AKECS_SetMode(akm, AKM_MODE_POWERDOWN);
	if (err < 0)
		return err;

	return akm_buf_print(buf, asa, 3);
}

static ssize_t akm_sysfs_regs_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	/* The total number of registers depends on the device. */
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	int err;
	uint8_t regs[AKM_REGS_SIZE];

	/* This function does not lock mutex obj */
	regs[0] = AKM_REGS_1ST_ADDR;
	err = akm_i2c_rxdata(akm->i2c, regs, AKM_REGS_SIZE);
	if (err < 0)
		return err;

	return akm_buf_print(buf, regs, AKM_REGS_SIZE);
}
#endif
#ifdef CONFIG_HUAWEI_DSM
static ssize_t akm_show_test_dsm(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 100,
					"test data_err: echo 1 > dsm_excep\n"
					"test i2c_err: 	echo 2 > dsm_excep\n"
					"test xyz_NotChange: echo 3 > dsm_excep\n"
					"test selfTest:	echo 4 > dsm_excep\n"
				   );
}

/*
*	test data or i2c error interface for device monitor
*/
static ssize_t akm_sysfs_test_dsm(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	long mode;
	int ret;

	if (strict_strtol(buf, AKM_BASE_NUM, &mode))
			return -EINVAL;

	switch(mode){
		case 1:	/*test data error interface*/
			ret = ms_report_dsm_err(DSM_MS_DATA_ERROR,"just test data xyz0 err\n");
			break;
		case 2: /*test i2c error interface*/
			ret = ms_report_dsm_err(DSM_MS_I2C_ERROR,NULL);
			break;
		case 3:
			ret = ms_report_dsm_err(DSM_MS_DATA_TIMES_NOTCHANGE_ERROR,"just test data_same test\n");
			break;
		case 4:
			ret = ms_report_dsm_err(DSM_MS_SELF_TEST_ERROR,"just test selftest_err\n");
			break;
		default:
			break;
	}

	return ret;
}
#endif

static struct device_attribute akm_compass_attributes[] = {
	__ATTR(enable_acc, 0660, akm_enable_acc_show, akm_enable_acc_store),
	__ATTR(enable_mag, 0660, akm_enable_mag_show, akm_enable_mag_store),
	__ATTR(enable_fusion, 0660, akm_enable_fusion_show,
			akm_enable_fusion_store),
	__ATTR(delay_acc,  0660, akm_delay_acc_show,  akm_delay_acc_store),
	__ATTR(delay_mag,  0660, akm_delay_mag_show,  akm_delay_mag_store),
	__ATTR(delay_fusion, 0660, akm_delay_fusion_show,
			akm_delay_fusion_store),
#ifdef CONFIG_HUAWEI_DSM
	__ATTR(dsm_excep,0660, akm_show_test_dsm, akm_sysfs_test_dsm),
#endif

#if AKM_DEBUG_IF
	__ATTR(mode,  0220, NULL, akm_sysfs_mode_store),
	__ATTR(bdata, 0440, akm_sysfs_bdata_show, NULL),
	__ATTR(asa,   0440, akm_sysfs_asa_show, NULL),
	__ATTR(regs,  0440, akm_sysfs_regs_show, NULL),
#endif
	__ATTR_NULL,
};

#define __BIN_ATTR(name_, mode_, size_, private_, read_, write_) \
	{ \
		.attr    = { .name = __stringify(name_), .mode = mode_ }, \
		.size    = size_, \
		.private = private_, \
		.read    = read_, \
		.write   = write_, \
	}

#define __BIN_ATTR_NULL \
	{ \
		.attr   = { .name = NULL }, \
	}

static struct bin_attribute akm_compass_bin_attributes[] = {
	__BIN_ATTR(accel, 0220, 6, NULL,
				NULL, akm_bin_accel_write),
	__BIN_ATTR_NULL
};

static char const *const device_link_name = "i2c";
static dev_t const akm_compass_device_dev_t = MKDEV(MISC_MAJOR, 240);

static int create_sysfs_interfaces(struct akm_compass_data *akm)
{
	int err;

	if (NULL == akm)
		return -EINVAL;

	err = 0;

	akm->compass = class_create(THIS_MODULE, AKM_SYSCLS_NAME);
	if (IS_ERR(akm->compass)) {
		err = PTR_ERR(akm->compass);
		goto exit_class_create_failed;
	}

	akm->class_dev = device_create(
						akm->compass,
						NULL,
						akm_compass_device_dev_t,
						akm,
						AKM_SYSDEV_NAME);
	if (IS_ERR(akm->class_dev)) {
		err = PTR_ERR(akm->class_dev);
		goto exit_class_device_create_failed;
	}

	err = sysfs_create_link(
			&akm->class_dev->kobj,
			&akm->i2c->dev.kobj,
			device_link_name);
	if (0 > err)
		goto exit_sysfs_create_link_failed;

	err = create_device_attributes(
			akm->class_dev,
			akm_compass_attributes);
	if (0 > err)
		goto exit_device_attributes_create_failed;

	err = create_device_binary_attributes(
			&akm->class_dev->kobj,
			akm_compass_bin_attributes);
	if (0 > err)
		goto exit_device_binary_attributes_create_failed;

	return err;

exit_device_binary_attributes_create_failed:
	remove_device_attributes(akm->class_dev, akm_compass_attributes);
exit_device_attributes_create_failed:
	sysfs_remove_link(&akm->class_dev->kobj, device_link_name);
exit_sysfs_create_link_failed:
	device_destroy(akm->compass, akm_compass_device_dev_t);
exit_class_device_create_failed:
	akm->class_dev = NULL;
	class_destroy(akm->compass);
exit_class_create_failed:
	akm->compass = NULL;
	return err;
}

static void remove_sysfs_interfaces(struct akm_compass_data *akm)
{
	if (NULL == akm)
		return;

	if (NULL != akm->class_dev) {
		remove_device_binary_attributes(
			&akm->class_dev->kobj,
			akm_compass_bin_attributes);
		remove_device_attributes(
			akm->class_dev,
			akm_compass_attributes);
		sysfs_remove_link(
			&akm->class_dev->kobj,
			device_link_name);
		akm->class_dev = NULL;
	}
	if (NULL != akm->compass) {
		device_destroy(
			akm->compass,
			akm_compass_device_dev_t);
		class_destroy(akm->compass);
		akm->compass = NULL;
	}
}


/***** akm input device functions ***********************************/
static int akm_compass_input_init(
	struct input_dev **input)
{
	int err = 0;

	/* Declare input device */
	*input = input_allocate_device();
	if (!*input)
		return -ENOMEM;

	/* Setup input device */
	set_bit(EV_ABS, (*input)->evbit);
	/* Accelerometer (720 x 16G)*/
	input_set_abs_params(*input, ABS_X,
			-11520, 11520, 0, 0);
	input_set_abs_params(*input, ABS_Y,
			-11520, 11520, 0, 0);
	input_set_abs_params(*input, ABS_Z,
			-11520, 11520, 0, 0);
	input_set_abs_params(*input, ABS_RX,
			0, 3, 0, 0);
	/* Magnetic field (limited to 16bit) */
	input_set_abs_params(*input, ABS_RY,
			-32768, 32767, 0, 0);
	input_set_abs_params(*input, ABS_RZ,
			-32768, 32767, 0, 0);
	input_set_abs_params(*input, ABS_THROTTLE,
			-32768, 32767, 0, 0);
	input_set_abs_params(*input, ABS_RUDDER,
			0, 3, 0, 0);

	/* Orientation (degree in Q6 format) */
	/*  yaw[0,360) pitch[-180,180) roll[-90,90) */
	input_set_abs_params(*input, ABS_HAT0Y,
			0, 23040, 0, 0);
	input_set_abs_params(*input, ABS_HAT1X,
			-11520, 11520, 0, 0);
	input_set_abs_params(*input, ABS_HAT1Y,
			-5760, 5760, 0, 0);
	/* Rotation Vector [-1,+1] in Q14 format */
	input_set_abs_params(*input, ABS_TILT_X,
			-16384, 16384, 0, 0);
	input_set_abs_params(*input, ABS_TILT_Y,
			-16384, 16384, 0, 0);
	input_set_abs_params(*input, ABS_TOOL_WIDTH,
			-16384, 16384, 0, 0);
	input_set_abs_params(*input, ABS_VOLUME,
			-16384, 16384, 0, 0);

	/* Report the dummy value */
	input_set_abs_params(*input, ABS_MISC,
			INT_MIN, INT_MAX, 0, 0);

	/* Set name */
	(*input)->name = AKM_INPUT_DEVICE_NAME;

	/* Register */
	err = input_register_device(*input);
	if (err) {
		input_free_device(*input);
		return err;
	}

	return err;
}

/***** akm functions ************************************************/
static irqreturn_t akm_compass_irq(int irq, void *handle)
{
	struct akm_compass_data *akm = handle;
	uint8_t buffer[AKM_SENSOR_DATA_SIZE];
	int err;

	memset(buffer, 0, sizeof(buffer));

	/***** lock *****/
	mutex_lock(&akm->sensor_mutex);

	/* Read whole data */
	buffer[0] = AKM_REG_STATUS;
	err = akm_i2c_rxdata(akm->i2c, buffer, AKM_SENSOR_DATA_SIZE);
	if (err < 0) {
		dev_err(&akm->i2c->dev, "IRQ I2C error.");
		mutex_unlock(&akm->sensor_mutex);
		/***** unlock *****/

		return IRQ_HANDLED;
	}
	/* Check ST bit */
	if (!(AKM_DRDY_IS_HIGH(buffer[0])))
		goto work_func_none;

	memcpy(akm->sense_data, buffer, AKM_SENSOR_DATA_SIZE);

	mutex_unlock(&akm->sensor_mutex);
	/***** unlock *****/

	atomic_set(&akm->drdy, 1);
	wake_up(&akm->drdy_wq);

	dev_vdbg(&akm->i2c->dev, "IRQ handled.");
	return IRQ_HANDLED;

work_func_none:
	mutex_unlock(&akm->sensor_mutex);
	/***** unlock *****/

	dev_vdbg(&akm->i2c->dev, "IRQ not handled.");
	return IRQ_NONE;
}

static int akm_compass_suspend(struct device *dev)
{
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	int ret = 0;

	if (AKM_IS_MAG_DATA_ENABLED() && akm->auto_report) {
		if (akm->use_hrtimer)
			hrtimer_cancel(&akm->poll_timer);
		else
			cancel_delayed_work_sync(&akm->dwork);
	}

	ret = AKECS_SetMode(akm, AKM_MODE_POWERDOWN);
	if (ret)
		dev_warn(&akm->i2c->dev, "Failed to set to POWERDOWN mode.\n");

	akm->state.power_on = akm->power_enabled;
	if (akm->state.power_on)
		akm_compass_power_set(akm, false);

	ret = pinctrl_select_state(akm->pinctrl, akm->pin_sleep);
	if (ret)
		dev_err(dev, "Can't select pinctrl state\n");

	dev_dbg(&akm->i2c->dev, "suspended\n");

	return ret;
}

static int akm_compass_resume(struct device *dev)
{
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	int ret = 0;
	uint8_t mode;

	ret = pinctrl_select_state(akm->pinctrl, akm->pin_default);
	if (ret)
		dev_err(dev, "Can't select pinctrl state\n");

	if (akm->state.power_on) {
		ret = akm_compass_power_set(akm, true);
		if (ret) {
			dev_err(dev, "Sensor power resume fail!\n");
			goto exit;
		}

		if (AKM_IS_MAG_DATA_ENABLED() && akm->auto_report) {
			mode = akm_select_frequency(akm->delay[MAG_DATA_FLAG]);
			ret = AKECS_SetMode(akm, mode);
			if (ret < 0) {
				dev_err(&akm->i2c->dev, "Failed to set to mode(%d)\n",
						mode);
				goto exit;
			}
			if (akm->use_hrtimer)
				hrtimer_start(&akm->poll_timer,
					ns_to_ktime(akm->delay[MAG_DATA_FLAG]),
					HRTIMER_MODE_REL);
			else
				queue_delayed_work(akm->work_queue, &akm->dwork,
					(unsigned long)nsecs_to_jiffies64(
						akm->delay[MAG_DATA_FLAG]));
		}
	}

	dev_dbg(&akm->i2c->dev, "resumed\n");

exit:
	return ret;
}

static int akm09911_i2c_check_device(
	struct i2c_client *client)
{
	/* AK09911 specific function */
	struct akm_compass_data *akm = i2c_get_clientdata(client);
	int err;

	akm->sense_info[0] = AK09911_REG_WIA1;
	err = akm_i2c_rxdata(client, akm->sense_info, AKM_SENSOR_INFO_SIZE);
	if (err < 0){
		dev_err(&client->dev, "%s: akm_i2c_rxdata failed to read REG_WIA1.err:%d\n", __func__,err);
		return err;
	}

	/* Set FUSE access mode */
	err = AKECS_SetMode(akm, AK09911_MODE_FUSE_ACCESS);
	if (err < 0){
		dev_err(&client->dev, "%s: failed to set access mode.err:%d\n", __func__,err);
		return err;
	}

	akm->sense_conf[0] = AK09911_FUSE_ASAX;
	err = akm_i2c_rxdata(client, akm->sense_conf, AKM_SENSOR_CONF_SIZE);
	if (err < 0){
		dev_err(&client->dev, "%s: akm_i2c_rxdata failed to read FUSE_ASAX.err:%d\n", __func__,err);
		return err;
	}

	err = AKECS_SetMode(akm, AK09911_MODE_POWERDOWN);
	if (err < 0){
		dev_err(&client->dev, "%s: failed to set POWERDOWN mode.err:%d\n", __func__,err);
		return err;
	}

	/* Check read data */
	if ((akm->sense_info[0] != AK09911_WIA1_VALUE) ||
			(akm->sense_info[1] != AK09911_WIA2_VALUE)){
		dev_err(&client->dev,
			"%s: The device is not AKM Compass.", __func__);
		return -ENXIO;
	}

	return err;
}

static int akm_compass_power_set(struct akm_compass_data *data, bool on)
{
	int rc = 0;

	if (!on && data->power_enabled) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			dev_err(&data->i2c->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			goto err_vdd_disable;
		}

		rc = regulator_disable(data->vio);
		if (rc) {
			dev_err(&data->i2c->dev,
				"Regulator vio disable failed rc=%d\n", rc);
			goto err_vio_disable;
		}
		data->power_enabled = false;
		return rc;
	} else if (on && !data->power_enabled) {
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->i2c->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			goto err_vdd_enable;
		}

		rc = regulator_enable(data->vio);
		if (rc) {
			dev_err(&data->i2c->dev,
				"Regulator vio enable failed rc=%d\n", rc);
			goto err_vio_enable;
		}
		data->power_enabled = true;

		/*
		 * The max time for the power supply rise time is 50ms.
		 * Use 80ms to make sure it meets the requirements.
		 */
		msleep(80);
		return rc;
	} else {
		dev_warn(&data->i2c->dev,
				"Power on=%d. enabled=%d\n",
				on, data->power_enabled);
		return rc;
	}

err_vio_enable:
	regulator_disable(data->vio);
err_vdd_enable:
	return rc;

err_vio_disable:
	if (regulator_enable(data->vdd))
		dev_warn(&data->i2c->dev, "Regulator vdd enable failed\n");
err_vdd_disable:
	return rc;
}

static int akm_compass_power_init(struct akm_compass_data *data, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0,
				AKM09911_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0,
				AKM09911_VIO_MAX_UV);

		regulator_put(data->vio);

	} else {
		data->vdd = regulator_get(&data->i2c->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			dev_err(&data->i2c->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd,
				AKM09911_VDD_MIN_UV, AKM09911_VDD_MAX_UV);
			if (rc) {
				dev_err(&data->i2c->dev,
					"Regulator set failed vdd rc=%d\n",
					rc);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->i2c->dev, "vio");
		if (IS_ERR(data->vio)) {
			rc = PTR_ERR(data->vio);
			dev_err(&data->i2c->dev,
				"Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			rc = regulator_set_voltage(data->vio,
				AKM09911_VIO_MIN_UV, AKM09911_VIO_MAX_UV);
			if (rc) {
				dev_err(&data->i2c->dev,
				"Regulator set failed vio rc=%d\n", rc);
				goto reg_vio_put;
			}
		}
	}

	return 0;

reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, AKM09911_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}

#ifdef CONFIG_OF
static int akm_compass_parse_dt(struct device *dev,
				struct akm_compass_data *akm)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;

	rc = of_property_read_u32(np, "akm,layout", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read akm,layout\n");
		return rc;
	} else {
		akm->layout = temp_val;
	}

	akm->auto_report = of_property_read_bool(np, "akm,auto-report");
	akm->use_hrtimer = of_property_read_bool(np, "akm,use-hrtimer");
	if (of_property_read_bool(np, "akm,rst-not-use-gpio"))
	{
		s_akm->rst_not_use_gpio = true;
	}
	else
	{
		s_akm->rst_not_use_gpio = false;
	}
	dev_err(dev, "akm_compass_parse_dt,rst-not-use-gpio=%d\n", s_akm->rst_not_use_gpio);
	if(!akm->rst_not_use_gpio)
	{
		akm->gpio_rstn = of_get_named_gpio_flags(dev->of_node,
				"akm,gpio_rstn", 0, NULL);

		if (!gpio_is_valid(akm->gpio_rstn)) {
			dev_err(dev, "gpio reset pin %d is invalid.\n",
				akm->gpio_rstn);
			return -EINVAL;
		}
		dev_err(dev, "akm_compass_parse_dt,s_akm->gpio_rstn=%d\n", s_akm->gpio_rstn);
		rc = gpio_request(s_akm->gpio_rstn, "akm09911_reset");
		if (rc < 0) {
			gpio_free(s_akm->gpio_rstn);
			rc = gpio_request(s_akm->gpio_rstn, "akm09911_reset");
		}
		gpio_direction_output(s_akm->gpio_rstn, 1);
	}
	akm->i2c_scl_gpio = of_get_named_gpio_flags(np, "akm,i2c-scl-gpio", 0, NULL);
	if (!gpio_is_valid(akm->i2c_scl_gpio)) {
		dev_err(dev, "gpio i2c-scl pin %d is invalid\n", akm->i2c_scl_gpio);
		return -EINVAL;
	}

	akm->i2c_sda_gpio = of_get_named_gpio_flags(np, "akm,i2c-sda-gpio", 0, NULL);
	if (!gpio_is_valid(akm->i2c_sda_gpio)) {
		dev_err(dev, "gpio i2c-sda pin %d is invalid\n", akm->i2c_sda_gpio);
		return -EINVAL;
	}

	return 0;
}
#else
static int akm_compass_parse_dt(struct device *dev,
				struct akm_compass_data *akm)
{
	return -EINVAL;
}
#endif /* !CONFIG_OF */

static int akm_pinctrl_init(struct akm_compass_data *akm)
{
	struct i2c_client *client = akm->i2c;

	akm->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(akm->pinctrl)) {
		dev_err(&client->dev, "Failed to get pinctrl\n");
		return PTR_ERR(akm->pinctrl);
	}

	akm->pin_default = pinctrl_lookup_state(akm->pinctrl, "default");
	if (IS_ERR_OR_NULL(akm->pin_default)) {
		dev_err(&client->dev, "Failed to look up default state\n");
		return PTR_ERR(akm->pin_default);
	}

	akm->pin_sleep = pinctrl_lookup_state(akm->pinctrl, "sleep");
	if (IS_ERR_OR_NULL(akm->pin_sleep)) {
		dev_err(&client->dev, "Failed to look up sleep state\n");
		return PTR_ERR(akm->pin_sleep);
	}

	return 0;
}

#ifdef CONFIG_HUAWEI_DSM
/* Show akm magnetic sensor registers. */
static void akm_regs_show(struct device *dev, char *buf)
{
	dev_warn(dev, "Show registers:\n");
	dev_warn(dev, "%s", buf);

	return;
}


/* Show akm magnetic sensor regulators. */
static void akm_regulators_show(struct akm_compass_data *akm)
{
	if (akm->vdd == NULL) {
		return;
	}

	if (akm->vio == NULL) {
		return;
	}
	akm09911_excep->vdd_status 	= regulator_is_enabled(akm->vdd);
	akm09911_excep->vdd_mv 		= regulator_get_voltage(akm->vdd)/1000;
	akm09911_excep->vio_status 	= regulator_is_enabled(akm->vio);
	akm09911_excep->vio_mv 		= regulator_get_voltage(akm->vio)/1000;

	dev_warn(&akm->i2c->dev, "Show regulators:\n");
	dev_warn(&akm->i2c->dev, "vdd enable=%d\n",akm09911_excep->vdd_status );
	dev_warn(&akm->i2c->dev, "vdd voltage=%d(mV)\n",akm09911_excep->vdd_mv);

	dev_warn(&akm->i2c->dev, "vio enable=%d\n",akm09911_excep->vio_status);
	dev_warn(&akm->i2c->dev, "vio voltage=%d(mV)\n",akm09911_excep->vio_mv);


	return;
}

/* Show akm magnetic sensor gpios. */
static void akm_gpios_show(struct akm_compass_data *akm)
{
	if(!s_akm->rst_not_use_gpio)
	{
		akm09911_excep->rst_val = gpio_get_value(akm->gpio_rstn);
	}
	akm09911_excep->scl_val = gpio_get_value(akm->i2c_scl_gpio);
	akm09911_excep->sda_val = gpio_get_value(akm->i2c_sda_gpio);
	dev_warn(&akm->i2c->dev, "Show gpios:\n");
	dev_warn(&akm->i2c->dev, "gpio(reset): number=%d, value=%d\n",akm->gpio_rstn,akm09911_excep->rst_val);
	dev_warn(&akm->i2c->dev, "gpio(scl): number=%d, value=%d\n",SENSOR_I2C_SCL,akm09911_excep->scl_val);
	dev_warn(&akm->i2c->dev, "gpio(sda): number=%d, value=%d\n",SENSOR_I2C_SDA,akm09911_excep->sda_val);

	return;
}
#endif


static int akm_report_data(struct akm_compass_data *akm)
{
	uint8_t dat_buf[AKM_SENSOR_DATA_SIZE];/* for GET_DATA */
	int ret;
	int mag_x, mag_y, mag_z;
	int tmp;
	int count = 10;
	static unsigned long total_delay_count = 0;
#ifdef CONFIG_HUAWEI_DSM
	static unsigned long all_zero_count = 0;
#endif

	do {
		/* The typical time for single measurement is 7.2ms */
		ret = AKECS_GetData_Poll(akm, dat_buf, AKM_SENSOR_DATA_SIZE, false);
		if (ret == -EAGAIN) {
			int getdata_delay = 1000;
			usleep(getdata_delay);
			total_delay_count++;
			if ((total_delay_count & 0xFFF) == 0xFFF) {
                dev_warn(&akm->i2c->dev,
                "Waited %d us before polling again, count=%d, total_delay_count=%lu.\n",
                        getdata_delay, count, total_delay_count);
			}
		}
	} while ((ret == -EAGAIN) && (--count));

	if (!count) {
		dev_err(&akm->i2c->dev, "Timeout get valid data.\n");
		return -EIO;
	}

	if (STATUS_ERROR(dat_buf[8])) {
		dev_warn(&akm->i2c->dev, "Status error. Reset...\n");
		AKECS_Reset(akm, 0);
		return -EIO;
	}

	tmp = (int)((int16_t)(dat_buf[2]<<8)+((int16_t)dat_buf[1]));
	tmp = tmp * akm->sense_conf[0] / 128 + tmp;
	mag_x = tmp;

	tmp = (int)((int16_t)(dat_buf[4]<<8)+((int16_t)dat_buf[3]));
	tmp = tmp * akm->sense_conf[1] / 128 + tmp;
	mag_y = tmp;

	tmp = (int)((int16_t)(dat_buf[6]<<8)+((int16_t)dat_buf[5]));
	tmp = tmp * akm->sense_conf[2] / 128 + tmp;
	mag_z = tmp;

	dev_dbg(&akm->i2c->dev, "mag_x:%d mag_y:%d mag_z:%d\n",
			mag_x, mag_y, mag_z);
	dev_dbg(&akm->i2c->dev, "raw data: %d %d %d %d %d %d %d %d\n",
			dat_buf[0], dat_buf[1], dat_buf[2], dat_buf[3],
			dat_buf[4], dat_buf[5], dat_buf[6], dat_buf[7]);
	dev_dbg(&akm->i2c->dev, "asa: %d %d %d\n", akm->sense_conf[0],
			akm->sense_conf[1], akm->sense_conf[2]);

	switch (akm->layout) {
	case 0:
	case 1:
		/* Fall into the default direction */
		break;
	case 2:
		tmp = mag_x;
		mag_x = mag_y;
		mag_y = -tmp;
		break;
	case 3:
		mag_x = -mag_x;
		mag_y = -mag_y;
		break;
	case 4:
		tmp = mag_x;
		mag_x = -mag_y;
		mag_y = tmp;
		break;
	case 5:
		mag_x = -mag_x;
		mag_z = -mag_z;
		break;
	case 6:
		tmp = mag_x;
		mag_x = mag_y;
		mag_y = tmp;
		mag_z = -mag_z;
		break;
	case 7:
		mag_y = -mag_y;
		mag_z = -mag_z;
		break;
	case 8:
		tmp = mag_x;
		mag_x = -mag_y;
		mag_y = -tmp;
		mag_z = -mag_z;
		break;
	}

	if(mag_x == 0 && mag_y == 0 && mag_z == 0)
	{
		dev_err(&akm->i2c->dev, "Invalid data: x,y,z all is 0.\n");
#ifdef CONFIG_HUAWEI_DSM
		if(5 < all_zero_count)
		{
			all_zero_count = 0;
			ms_report_dsm_err(DSM_MS_DATA_ERROR,NULL);
		}
		else
		{
			all_zero_count++;
		}
#endif
	}
#ifdef CONFIG_HUAWEI_DSM
	else
	{
		all_zero_count = 0;
	}
#endif

	input_report_abs(akm->input, ABS_X, mag_x);
	input_report_abs(akm->input, ABS_Y, mag_y);
	input_report_abs(akm->input, ABS_Z, mag_z);

	/* avoid eaten by input subsystem framework */
	if ((mag_x == akm->last_x) && (mag_y == akm->last_y) &&
			(mag_z == akm->last_z))
		input_report_abs(akm->input, ABS_MISC, akm->rep_cnt++);

	akm->last_x = mag_x;
	akm->last_y = mag_y;
	akm->last_z = mag_z;

	input_sync(akm->input);

#ifdef CONFIG_HUAWEI_DSM
	ms_dsm_check_val_same_times(mag_x, mag_y, mag_z);
#endif
	if(sensorDT_mode)
	{
		compass_data_count++;
	}

	return 0;
}

static void akm_dev_poll(struct work_struct *work)
{
	struct akm_compass_data *akm;
	int ret;
	uint8_t mode;
	akm = container_of((struct delayed_work *)work,
			struct akm_compass_data,  dwork);
	mutex_lock(&akm->self_test_mutex);

	ret = akm_report_data(akm);
	if (ret < 0){
		dev_warn(&akm->i2c->dev, "Failed to report data\n");
		/*if error occur, set compass mode to used mode*/
		mode = akm_select_frequency(akm->delay[MAG_DATA_FLAG]);
		/*before change compass mode, set to power down mode first*/
		ret = AKECS_SetMode(akm, AKM_MODE_POWERDOWN);
		if(ret < 0)
		{
			dev_err(&akm->i2c->dev, "Failed to set to power down mode\n");
		}
		ret = AKECS_SetMode(akm, mode);
		if (ret < 0)
		{
			dev_err(&akm->i2c->dev, "Failed to set to mode(%x)\n", mode);
		}		
	}

	if (!akm->use_hrtimer)
	queue_delayed_work(akm->work_queue, &akm->dwork,
			(unsigned long)nsecs_to_jiffies64(akm->delay[MAG_DATA_FLAG]));
	mutex_unlock(&akm->self_test_mutex);
}
int
TEST_DATA(const char testno[],
		const char testname[],
		const int testdata,
		const int lolimit,
		const int hilimit,
		int * pf_total)
{
	int pf;                     //Pass;1, Fail;-1

	if ((testno == NULL) && (strncmp(testname, "START", 5) == 0)) {
		// Display header
		pr_info("--------------------------------------------------------------------\n");
		pr_info(" Test No. Test Name    Fail    Test Data    [      Low         High]\n");
		pr_info("--------------------------------------------------------------------\n");

		pf = 1;
	} else if ((testno == NULL) && (strncmp(testname, "END", 3) == 0)) {
		// Display result
		pr_info("--------------------------------------------------------------------\n");
		if (*pf_total == 1) {
			pr_info("Factory shipment test was passed.\n\n");
		} else {
			pr_info("Factory shipment test was failed.\n\n");
		}

		pf = 1;
	} else {
		if ((lolimit <= testdata) && (testdata <= hilimit)) {
			//Pass
			pf = 1;
		} else {
			//Fail
			pf = -1;
		}

		//display result
		pr_info(" %7s  %-10s      %c    %9d    [%9d    %9d]\n",
				 testno, testname, ((pf == 1) ? ('.') : ('F')), testdata,
				 lolimit, hilimit);
	}

	//Pass/Fail check
	if (*pf_total != 0) {
		if ((*pf_total == 1) && (pf == 1)) {
			*pf_total = 1;            //Pass
		} else {
			*pf_total = -1;           //Fail
		}
	}
	return pf;
}
static int akm_self_test(struct sensors_classdev *sensors_cdev)
{
	struct akm_compass_data *akm = container_of(sensors_cdev,
			struct akm_compass_data, cdev);
	int   pf_total;  //p/f flag for this subtest
	char    i2cData[16];
	int   hdata[3];
	int asax, asay, asaz;
	int count;
	int ret;
	pr_err("%s:%d : akm self test begin \n", __FUNCTION__, __LINE__);
	mutex_lock(&akm->self_test_mutex);

	/* Removed lines. */
	asax = akm->sense_conf[0];
	asay = akm->sense_conf[1];
	asaz = akm->sense_conf[2];

	//***********************************************
	//  Reset Test Result
	//***********************************************
	pf_total = 1;

	//***********************************************
	//  Step1
	//***********************************************
	ret = pinctrl_select_state(akm->pinctrl, akm->pin_default);
	if (ret)
		pr_err("Can't select pinctrl state\n");

	ret = akm_compass_power_set(akm, true);
	if (ret) {
		pr_err("Sensor power resume fail!\n");
		mutex_unlock(&akm->self_test_mutex);
#ifdef CONFIG_HUAWEI_DSM
		ms_report_dsm_err(DSM_MS_SELF_TEST_ERROR,"Sensor power resume fail!\n");
#endif
		return false;
	}

	// Reset device.
	if (AKECS_Reset(akm, 0) < 0) {
		dev_err(&akm->i2c->dev, "Reset failed.\n");
		mutex_unlock(&akm->self_test_mutex);
#ifdef CONFIG_HUAWEI_DSM
		ms_report_dsm_err(DSM_MS_SELF_TEST_ERROR,"Reset failed.\n");
#endif
		return false;
	}

	// TEST
	TEST_DATA(TLIMIT_NO_ASAX_09911, TLIMIT_TN_ASAX_09911, asax, TLIMIT_LO_ASAX_09911, TLIMIT_HI_ASAX_09911, &pf_total);
	TEST_DATA(TLIMIT_NO_ASAY_09911, TLIMIT_TN_ASAY_09911, asay, TLIMIT_LO_ASAY_09911, TLIMIT_HI_ASAY_09911, &pf_total);
	TEST_DATA(TLIMIT_NO_ASAZ_09911, TLIMIT_TN_ASAZ_09911, asaz, TLIMIT_LO_ASAZ_09911, TLIMIT_HI_ASAZ_09911, &pf_total);

	// Set to PowerDown mode
	if (AKECS_SetMode(akm, AK09911_MODE_POWERDOWN) < 0) {
		pr_info("%s:%d Error.\n", __FUNCTION__, __LINE__);
		mutex_unlock(&akm->self_test_mutex);
#ifdef CONFIG_HUAWEI_DSM
		ms_report_dsm_err(DSM_MS_SELF_TEST_ERROR,"AKECS_SetMode AK09911_MODE_POWERDOWN.\n");
#endif
		return false;
	}

	//***********************************************
	//  Step2
	//***********************************************

	// Set to SNG measurement pattern (Set CNTL register)
	if (AKECS_SetMode(akm, AK09911_MODE_SNG_MEASURE) < 0) {
		pr_info("%s:%d Error.\n", __FUNCTION__, __LINE__);
		mutex_unlock(&akm->self_test_mutex);
#ifdef CONFIG_HUAWEI_DSM
		ms_report_dsm_err(DSM_MS_SELF_TEST_ERROR,"AKECS_SetMode AK09911_MODE_SNG_MEASURE");
#endif
		return false;
	}

	// Wait for DRDY pin changes to HIGH.
	//usleep(AKM_MEASURE_TIME_US);
	// Get measurement data from AK09911
	// ST1 + (HXL + HXH) + (HYL + HYH) + (HZL + HZH) + TEMP + ST2
	// = 1 + (1 + 1) + (1 + 1) + (1 + 1) + 1 + 1 = 9yte
	//if (AKD_GetMagneticData(i2cData) != AKD_SUCCESS) {

	count = 10;

	do {
		/* The typical time for single measurement is 7.2ms */
		ret = AKECS_GetData_Poll(akm, i2cData, AKM_SENSOR_DATA_SIZE, true);
		if (ret == -EAGAIN)
			usleep_range(1000, 10000);
	} while ((ret == -EAGAIN) && (--count));

	if (!count) {
		pr_err("%s:%d :Timeout get valid data in SNG mode.\n",__FUNCTION__,__LINE__);
		mutex_unlock(&akm->self_test_mutex);
#ifdef CONFIG_HUAWEI_DSM
		ms_report_dsm_err(DSM_MS_SELF_TEST_ERROR,"Timeout get valid data.");
#endif
		return false;
	}

	//hdata[0] = (int)((((uint)(i2cData[2]))<<8)+(uint)(i2cData[1]));
	//hdata[1] = (int)((((uint)(i2cData[4]))<<8)+(uint)(i2cData[3]));
	//hdata[2] = (int)((((uint)(i2cData[6]))<<8)+(uint)(i2cData[5]));

	hdata[0] = (s16)(i2cData[1] | (i2cData[2] << 8));
	hdata[1] = (s16)(i2cData[3] | (i2cData[4] << 8));
	hdata[2] = (s16)(i2cData[5] | (i2cData[6] << 8));

	// TEST
	i2cData[0] &= 0x7F;
	TEST_DATA(TLIMIT_NO_SNG_ST1_09911,  TLIMIT_TN_SNG_ST1_09911,  (int)i2cData[0], TLIMIT_LO_SNG_ST1_09911,  TLIMIT_HI_SNG_ST1_09911,  &pf_total);

	// TEST
	TEST_DATA(TLIMIT_NO_SNG_HX_09911,   TLIMIT_TN_SNG_HX_09911,   hdata[0],          TLIMIT_LO_SNG_HX_09911,   TLIMIT_HI_SNG_HX_09911,   &pf_total);
	TEST_DATA(TLIMIT_NO_SNG_HY_09911,   TLIMIT_TN_SNG_HY_09911,   hdata[1],          TLIMIT_LO_SNG_HY_09911,   TLIMIT_HI_SNG_HY_09911,   &pf_total);
	TEST_DATA(TLIMIT_NO_SNG_HZ_09911,   TLIMIT_TN_SNG_HZ_09911,   hdata[2],          TLIMIT_LO_SNG_HZ_09911,   TLIMIT_HI_SNG_HZ_09911,   &pf_total);
	TEST_DATA(TLIMIT_NO_SNG_ST2_09911,  TLIMIT_TN_SNG_ST2_09911,  (int)i2cData[8], TLIMIT_LO_SNG_ST2_09911,  TLIMIT_HI_SNG_ST2_09911,  &pf_total);

	// Set to Self-test mode (Set CNTL register)
	if (AKECS_SetMode(akm, AK09911_MODE_SELF_TEST) < 0) {
		pr_info("%s:%d Error.\n", __FUNCTION__, __LINE__);
		mutex_unlock(&akm->self_test_mutex);
#ifdef CONFIG_HUAWEI_DSM
		ms_report_dsm_err(DSM_MS_SELF_TEST_ERROR,"AKECS_SetMode AK09911_MODE_SELF_TEST failed.\n");
#endif
		return false;
	}

	// Wait for DRDY pin changes to HIGH.
	//usleep(AKM_MEASURE_TIME_US);
	// Get measurement data from AK09911
	// ST1 + (HXL + HXH) + (HYL + HYH) + (HZL + HZH) + TEMP + ST2
	// = 1 + (1 + 1) + (1 + 1) + (1 + 1) + 1 + 1 = 9byte
	//if (AKD_GetMagneticData(i2cData) != AKD_SUCCESS) {

	count = 10;

	do {
		/* The typical time for single measurement is 7.2ms */
		ret = AKECS_GetData_Poll(akm, i2cData, AKM_SENSOR_DATA_SIZE, true);
		if (ret == -EAGAIN)
			usleep_range(1000, 10000);
	} while ((ret == -EAGAIN) && (--count));

	if (!count) {
		pr_err("%s:%d :Timeout get valid data in self test mode.\n",__FUNCTION__,__LINE__);
		mutex_unlock(&akm->self_test_mutex);
#ifdef CONFIG_HUAWEI_DSM
		ms_report_dsm_err(DSM_MS_SELF_TEST_ERROR,"Timeout get valid data");
#endif
		return false;
	}

	// TEST
	i2cData[0] &= 0x7F;
	TEST_DATA(TLIMIT_NO_SLF_ST1_09911, TLIMIT_TN_SLF_ST1_09911, (int)i2cData[0], TLIMIT_LO_SLF_ST1_09911, TLIMIT_HI_SLF_ST1_09911, &pf_total);

	//hdata[0] = (int)((((uint)(i2cData[2]))<<8)+(uint)(i2cData[1]));
	//hdata[1] = (int)((((uint)(i2cData[4]))<<8)+(uint)(i2cData[3]));
	//hdata[2] = (int)((((uint)(i2cData[6]))<<8)+(uint)(i2cData[5]));

	hdata[0] = (s16)(i2cData[1] | (i2cData[2] << 8));
	hdata[1] = (s16)(i2cData[3] | (i2cData[4] << 8));
	hdata[2] = (s16)(i2cData[5] | (i2cData[6] << 8));

	// TEST
	TEST_DATA(
			  TLIMIT_NO_SLF_RVHX_09911,
			  TLIMIT_TN_SLF_RVHX_09911,
			  (hdata[0])*(asax/128 + 1),
			  TLIMIT_LO_SLF_RVHX_09911,
			  TLIMIT_HI_SLF_RVHX_09911,
			  &pf_total
			  );

	TEST_DATA(
			  TLIMIT_NO_SLF_RVHY_09911,
			  TLIMIT_TN_SLF_RVHY_09911,
			  (hdata[1])*(asay/128 + 1),
			  TLIMIT_LO_SLF_RVHY_09911,
			  TLIMIT_HI_SLF_RVHY_09911,
			  &pf_total
			  );

	TEST_DATA(
			  TLIMIT_NO_SLF_RVHZ_09911,
			  TLIMIT_TN_SLF_RVHZ_09911,
			  (hdata[2])*(asaz/128 + 1),
			  TLIMIT_LO_SLF_RVHZ_09911,
			  TLIMIT_HI_SLF_RVHZ_09911,
			  &pf_total
			  );

	TEST_DATA(
			TLIMIT_NO_SLF_ST2_09911,
			TLIMIT_TN_SLF_ST2_09911,
			(int)i2cData[8],
			TLIMIT_LO_SLF_ST2_09911,
			TLIMIT_HI_SLF_ST2_09911,
			&pf_total
		 );
	pr_err("%s:%d : pf_total is : %d\n", __FUNCTION__,__LINE__, pf_total);
	if(pf_total <= 0)
	{
		pr_err("%s:%d :self test compass data error, Whether there is a strong magnetic field near?\n",
			__FUNCTION__,__LINE__);
	}

	if (AKM_IS_MAG_DATA_ENABLED() && akm->auto_report) {
		uint8_t mode;
		mode = akm_select_frequency(akm->delay[MAG_DATA_FLAG]);
		ret = AKECS_SetMode(akm, mode);
		if (ret < 0) {
			pr_err("Failed to restore to mode(%d)\n", mode);
		}
		pr_err("restore from self test mode to mode(%d)\n", mode);
	}else{
		akm_compass_power_set(akm, false);

		ret = pinctrl_select_state(akm->pinctrl, akm->pin_sleep);
		if (ret)
			pr_err("Can't select pinctrl state\n");
		
		pr_err("after compass self test, restore power down mode\n");
	}
	/* Removed lines. */
	mutex_unlock(&akm->self_test_mutex);
	return (pf_total > 0) ? true : false;
}

static enum hrtimer_restart akm_timer_func(struct hrtimer *timer)
{
	struct akm_compass_data *akm;

	akm = container_of(timer, struct akm_compass_data, poll_timer);

	queue_work(akm->work_queue, &akm->dwork.work);
	hrtimer_forward_now(&akm->poll_timer,
			ns_to_ktime(akm->delay[MAG_DATA_FLAG]));

	return HRTIMER_RESTART;
}

int akm_compass_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct akm09911_platform_data *pdata;
	int err = 0;
	int i;

	dev_err(&client->dev, "start probing.");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,
				"%s: check_functionality failed.", __func__);
		err = -ENODEV;
		goto exit0;
	}

	/* Allocate memory for driver data */
	s_akm = kzalloc(sizeof(struct akm_compass_data), GFP_KERNEL);
	if (!s_akm) {
		dev_err(&client->dev,
				"%s: memory allocation failed.", __func__);
		err = -ENOMEM;
		goto exit1;
	}

	/**** initialize variables in akm_compass_data *****/
	init_waitqueue_head(&s_akm->drdy_wq);
	init_waitqueue_head(&s_akm->open_wq);

	mutex_init(&s_akm->sensor_mutex);
	mutex_init(&s_akm->accel_mutex);
	mutex_init(&s_akm->val_mutex);
	mutex_init(&s_akm->op_mutex);
	mutex_init(&s_akm->self_test_mutex);

	atomic_set(&s_akm->active, 0);
	atomic_set(&s_akm->drdy, 0);

	s_akm->enable_flag = 0;
	s_akm->device_exist = false;

	/* Set to 1G in Android coordination, AKSC format */
	s_akm->accel_data[0] = 0;
	s_akm->accel_data[1] = 0;
	s_akm->accel_data[2] = 720;

	for (i = 0; i < AKM_NUM_SENSORS; i++)
		s_akm->delay[i] = -1;

	if (client->dev.of_node) {
		err = akm_compass_parse_dt(&client->dev, s_akm);
		if (err) {
			dev_err(&client->dev,
				"Unable to parse platfrom data err=%d\n", err);
			goto exit2;
		}
	} else {
		if (client->dev.platform_data) {
			/* Copy platform data to local. */
			pdata = client->dev.platform_data;
			s_akm->layout = pdata->layout;
			s_akm->gpio_rstn = pdata->gpio_RSTN;
		} else {
		/* Platform data is not available.
		   Layout and information should be set by each application. */
			s_akm->layout = 0;
			s_akm->gpio_rstn = 0;
			dev_warn(&client->dev, "%s: No platform data.",
				__func__);
		}
	}

	/***** I2C initialization *****/
	s_akm->i2c = client;
	/* set client data */
	i2c_set_clientdata(client, s_akm);

	/* initialize pinctrl */
	if (!akm_pinctrl_init(s_akm)) {
		err = pinctrl_select_state(s_akm->pinctrl, s_akm->pin_default);
		if (err) {
			dev_err(&client->dev, "Can't select pinctrl state\n");
			goto exit2;
		}
	}

#ifdef CONFIG_HUAWEI_DSM
	err = akm_dsm_excep_init(s_akm);
	if(err < 0)
		goto exit3;
#endif

	/* Pull up the reset pin */
	if(s_akm->rst_not_use_gpio)
	{
		AKECS_Reset(s_akm, 0);
	}
	else
	{
		AKECS_Reset(s_akm, 1);
	}

	/* check connection */
	err = akm_compass_power_init(s_akm, 1);
	if (err < 0)
		goto exit2;
	err = akm_compass_power_set(s_akm, 1);
	if (err < 0)
		goto exit3;

	err = akm09911_i2c_check_device(client);
	if (err < 0)
		goto exit4;

	/***** input *****/
	err = akm_compass_input_init(&s_akm->input);
	if (err) {
		dev_err(&client->dev,
			"%s: input_dev register failed", __func__);
		goto exit4;
	}
	input_set_drvdata(s_akm->input, s_akm);

	/***** IRQ setup *****/
	s_akm->irq = client->irq;

	dev_dbg(&client->dev, "%s: IRQ is #%d.",
			__func__, s_akm->irq);

	if (s_akm->irq) {
		err = request_threaded_irq(
				s_akm->irq,
				NULL,
				akm_compass_irq,
				IRQF_TRIGGER_HIGH|IRQF_ONESHOT,
				dev_name(&client->dev),
				s_akm);
		if (err < 0) {
			dev_err(&client->dev,
				"%s: request irq failed.", __func__);
			goto exit5;
		}
	} else if (s_akm->auto_report) {
		if (s_akm->use_hrtimer) {
			hrtimer_init(&s_akm->poll_timer, CLOCK_MONOTONIC,
					HRTIMER_MODE_REL);
			s_akm->poll_timer.function = akm_timer_func;
			s_akm->work_queue = alloc_workqueue("akm_poll_work",
				WQ_UNBOUND | WQ_MEM_RECLAIM | WQ_HIGHPRI, 1);
			INIT_WORK(&s_akm->dwork.work, akm_dev_poll);
		} else {
			s_akm->work_queue = alloc_workqueue("akm_poll_work",
					WQ_NON_REENTRANT, 0);
			INIT_DELAYED_WORK(&s_akm->dwork, akm_dev_poll);
		}
		s_akm->work_queue = alloc_workqueue("akm_poll_work", WQ_UNBOUND | WQ_MEM_RECLAIM | WQ_HIGHPRI, 1);
	}

	/***** misc *****/
	err = misc_register(&akm_compass_dev);
	if (err) {
		dev_err(&client->dev,
			"%s: akm_compass_dev register failed", __func__);
		goto exit6;
	}

	/***** sysfs *****/
	err = create_sysfs_interfaces(s_akm);
	if (0 > err) {
		dev_err(&client->dev,
			"%s: create sysfs failed.", __func__);
		goto exit7;
	}

	s_akm->cdev = sensors_cdev;
	s_akm->cdev.sensors_enable = akm_enable_set;
	s_akm->cdev.sensors_poll_delay = akm_poll_delay_set;
	s_akm->cdev.sensors_self_test = akm_self_test;

	s_akm->delay[MAG_DATA_FLAG] = sensors_cdev.delay_msec * 1000000;

	err = sensors_classdev_register(&s_akm->input->dev, &s_akm->cdev);

	if (err) {
		dev_err(&client->dev, "class device create failed: %d\n", err);
		goto exit8;
	}
	set_sensors_list(M_SENSOR);
	err = app_info_set("M-Sensor", "AKM09911");
	if (err)
	{
		dev_err(&client->dev, "%s, line %d:set compass AKM09911 app_info error\n", __func__, __LINE__);
	}
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
	set_hw_dev_flag(DEV_I2C_COMPASS);
#endif
	err = set_sensor_input(AKM, s_akm->input->dev.kobj.name);
	if (err) {
		dev_err(&client->dev, "%s set_sensor_input failed\n", __func__);
	}
	akm_compass_power_set(s_akm, false);

	dev_info(&client->dev, "akm compass successfully probed.");
	s_akm->device_exist = true;
	return 0;

exit8:
	remove_sysfs_interfaces(s_akm);
exit7:
	misc_deregister(&akm_compass_dev);
exit6:
	if (s_akm->irq)
		free_irq(s_akm->irq, s_akm);
exit5:
	input_unregister_device(s_akm->input);
exit4:
	akm_compass_power_set(s_akm, 0);
#ifdef CONFIG_HUAWEI_DSM
	akm_dsm_excep_exit();
#endif

exit3:
	akm_compass_power_init(s_akm, 0);
exit2:
	kfree(s_akm);
exit1:
exit0:
	return err;
}

static int akm_compass_remove(struct i2c_client *client)
{
	struct akm_compass_data *akm = i2c_get_clientdata(client);

	if (akm->auto_report) {
		if (akm->use_hrtimer) {
			hrtimer_cancel(&akm->poll_timer);
			cancel_work_sync(&akm->dwork.work);
		} else {
			cancel_delayed_work_sync(&akm->dwork);
		}
		destroy_workqueue(akm->work_queue);
	}

	if (akm_compass_power_set(akm, 0))
		dev_err(&client->dev, "power set failed.");
#ifdef CONFIG_HUAWEI_DSM
	akm_dsm_excep_exit();
#endif

	if (akm_compass_power_init(akm, 0))
		dev_err(&client->dev, "power deinit failed.");
	remove_sysfs_interfaces(akm);
	sensors_classdev_unregister(&akm->cdev);
	if (misc_deregister(&akm_compass_dev) < 0)
		dev_err(&client->dev, "misc deregister failed.");
	if (akm->irq)
		free_irq(akm->irq, akm);
	input_unregister_device(akm->input);
	kfree(akm);
	dev_info(&client->dev, "successfully removed.");
	return 0;
}

static const struct i2c_device_id akm_compass_id[] = {
	{AKM_I2C_NAME, 0 },
	{ }
};

static const struct dev_pm_ops akm_compass_pm_ops = {
	.suspend	= akm_compass_suspend,
	.resume		= akm_compass_resume,
};

static struct of_device_id akm09911_match_table[] = {
	{ .compatible = "ak,ak09911", },
	{ .compatible = "akm,akm09911", },
	{ },
};

static struct i2c_driver akm_compass_driver = {
	.probe		= akm_compass_probe,
	.remove		= akm_compass_remove,
	.id_table	= akm_compass_id,
	.driver = {
		.name	= AKM_I2C_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = akm09911_match_table,
		.pm		= &akm_compass_pm_ops,
	},
};

static int __init akm_compass_init(void)
{
	pr_info("AKM compass driver: initialize.");
	return i2c_add_driver(&akm_compass_driver);
}

static void __exit akm_compass_exit(void)
{
	pr_info("AKM compass driver: release.");
	i2c_del_driver(&akm_compass_driver);
}

module_init(akm_compass_init);
module_exit(akm_compass_exit);

MODULE_AUTHOR("viral wang <viral_wang@htc.com>");
MODULE_DESCRIPTION("AKM compass driver");
MODULE_LICENSE("GPL");

