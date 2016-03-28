#ifdef CONFIG_HUAWEI_DSM
/* drivers/input/misc/lis3dh_acc_monitor.c - Kionix accelerometer driver monitor interface
 *
 *	this file provides some interface to check (i2c transfer error/ data exception)
 *	and  report errors to device monitor
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
#include	<linux/dsm_pub.h>
#include	<huawei_platform/sensor/hw_sensor_info.h>
#include	<huawei_platform/sensor/lis3dh.h>

#define CLIENT_NAME_GS_LIS			"dsm_gs_lis3dh"
static struct dsm_client *lis3dh_gs_dclient = NULL;
static int lis_dump_gsensor_info (int type, void *buff, int size);
static ssize_t lis_gsensor_report_dsm_err(int type,bool auto_report_flag);

/* add gsensor dump_func */
static struct dsm_client_ops gs_ops={
	.poll_state = NULL,
	.dump_func = lis_dump_gsensor_info,
};

/* dsm client for gsensor */
static struct dsm_dev dsm_gs_lis_i2c = {
	.name 		= CLIENT_NAME_GS_LIS,	// dsm client name
	.fops 		= &gs_ops,				// options
	.buff_size 	= DSM_SENSOR_BUF_MAX,		// buffer size
};


/*
* check the current data is error or not
*/
static inline bool lis_exception_condition(int x, int y,int z,struct lis_gsensor_test_excep *excep )
{
	return (
		     /*condition 1: abs(x + y +z) > 3max */
		     ( abs(x + y +z) > excep->x_y_z_max)
			 //remove here for x, y, z all near zero may appear in the process of moving phone
			 /*condition 2:  abs(x) > max*/
			 || abs(x) > excep->single_axis_max
			 || abs(y) > excep->single_axis_max
			 || abs(z) > excep->single_axis_max
			) ? true : false;
}


/*
* if i2c transfer error, we check sda/scl value and regulator's value
*/
static int lis_dump_i2c_exception_status(struct lis3dh_acc_data *acceld, int ret)
{
	/* print pm status and i2c gpio status*/
	struct lis_gsensor_test_excep *excep = &acceld->lis_gs_exception;

	excep->i2c_err_num = ret;

	/* call report dsm error function with specified i2c error number*/
	lis_gsensor_report_dsm_err(DSM_GS_I2C_ERROR,true);

	pr_err("[lis3dh_err]i2c_scl_val=%d,i2c_sda_val=%d,excep_num=%d, vdd = %d, vddio=%d,i2c_errno=%d\n",
				excep->i2c_scl_val,excep->i2c_sda_val,excep->excep_num,
				excep->vdd_mv,excep->vddio_mv,excep->i2c_err_num);

	/*set the i2c_err_num 0 for next i2c exception detection*/
	excep->i2c_err_num = 0;

	return 0;
}


/**
 * func - lis_check_exception
 * @triger_timer_flag, if first data is error, triger_timer_flag = false, now counut exception data
 *		if timer is trigered, enter counting exceptions times mode in 2s,
 *		don't allow to trigger timer
 */
static int lis_check_exception(int *xyz, struct lis3dh_acc_data *acceld )
{
	struct lis_gsensor_test_excep *excep = &acceld->lis_gs_exception;
	struct timer_list *excep_timer = &acceld->gsensor_excep_timer;
	int x = xyz[0],y = xyz[1],z = xyz[2];
	bool condition = false;
	static int excep_times = 0;
	static int data_err_times = 0;
	/* store the current x,y,z value for forc dump to read current value*/
	memcpy(acceld->lis_gs_exception.xyz, xyz,3*sizeof(int));

	/* when an error occur, enter counting exceptions times mode, don't allow to trigger timer */
	if(excep->triger_timer_flag){
		condition = lis_exception_condition(x,y,z,excep);
		if(condition){
			excep->triger_count_flag = true;
			excep->triger_timer_flag = false;
			excep->exception_times = 1;
			excep->total_times = 1;
			excep->cur_err_x = x;
			excep->cur_err_y = y;
			excep->cur_err_z = z;
			/* start the timer and execute the func:exception_2s_timer_handler after 2s*/
			mod_timer(excep_timer , jiffies + msecs_to_jiffies(COUNT_EXCEPTION_TIME_2S));
			pr_err("[lis3dh_warn] the data is exception. start timer calc.\n");
		}
	}
	/*counting exceptions times until 2s  is over*/
	if(excep->triger_count_flag)
	{
		excep->total_times++;
		condition = lis_exception_condition(x,y,z,excep);
		if(condition){
			excep->exception_times ++;
		}
	}

	/* if data always exception, can't print so many log, phone maybe crash.*/
	if (condition) {
		if (0 == excep_times % 15 ) {
			pr_err("[lis3dh_warn] xyz_warn. x = %d, y = %d, z = %d",
				   x, y, z);
		} else if (excep_times >= 30) {
			excep_times = 0;
		}

		excep_times ++;
	}
/*
delete this because add new radar point include this feature.
the new point is at least one axis hold it's data more than twentity times the radar will report.
this delete point is only three axis hold it's data more than twentity times the radar will report.
*/
	if ( 0 == x && 0 == y && 0 == z) {
		pr_err("[lis3dh_warn]WARNING:%s at %s %d:lis3dh acceler data xyz = 0"
			   , __FILE__, __func__, __LINE__);
		if(10 < data_err_times++)
		{
			lis_gsensor_report_dsm_err(DSM_GS_XYZ_0_ERROR, false);
			data_err_times = 0;
		}
	}
	else
	{
		data_err_times = 0;
	}
	return 0;
}


/**
 * func - lis_exception_2s_timer_handler
 *	check the accelerator's data is error or not in 2 seconds,
 *  	if error, schedule  func: report_exception_work_handler to report status.
 *	after execute the func, clear all flag and count data.
 */
static void lis_exception_2s_timer_handler(unsigned long data)
{
	int exception_percentage;

	struct lis3dh_acc_data *acceld = (struct lis3dh_acc_data *)data;
	struct lis_gsensor_test_excep *excep = &acceld->lis_gs_exception;

	/*allow to triger timer again*/
	excep->triger_timer_flag = true;
	excep->triger_count_flag = false;

	exception_percentage = 100 *excep->exception_times / excep->total_times;
	if(exception_percentage >= EXCEPTION_PERCENTAGE
		&& excep->exception_times >= EXCEPTION_BASE_TIMES
		&&(atomic_read(&acceld->enabled) > 0)){
		pr_err("---------cut here-------\n"
			"[lis3dh_err]WARNING:at %s %d:lis3dh acceler data is exception %s()\n"
			,__FILE__,__LINE__,__func__);
		/* schedule func: report_exception_work_handler*/
		queue_work(acceld->excep_workqueue, &acceld->excep_work);
	}

	/*clear count times for next time*/
	excep->exception_times  = 0;
	excep->total_times = 0;
}

/**
 * func - lis_report_exception_work_handler
 *
 * 	report exception data to device monitor and kernel log.
 */
static void lis_report_exception_work_handler(struct work_struct *work)
{
	struct lis3dh_acc_data *acceld = container_of(work, struct lis3dh_acc_data, excep_work);
	struct lis_gsensor_test_excep *excep = &acceld->lis_gs_exception;

	lis_gsensor_report_dsm_err(DSM_GS_DATA_ERROR,true);

	pr_err("[lis3dh_err]CTRL1 = 0x%2x,CTRL2 = 0x%2x,CTRL3 = 0x%2x ,"
			"CTRL4 = 0x%2x,CTRL5 = 0x%2x,CTRL6 = 0x%2x \n"\
			"excep_x = %d,excep_y = %d,excep_z = %d\n"
			"i2c_scl_val=%d,i2c_sda_val=%d,vdd = %d, vddio=%d,i2c_errno=%d\n"
			,excep->reg_buf[0],excep->reg_buf[1],excep->reg_buf[2]
			,excep->reg_buf[3],excep->reg_buf[4],excep->reg_buf[5]
			,excep->cur_err_x,excep->cur_err_y,excep->cur_err_z
			,excep->i2c_scl_val,excep->i2c_sda_val
			,excep->vdd_mv,excep->vddio_mv,excep->i2c_err_num);

}

/**
 * func - lis_init_exception_timer
 *
 * 		init the exception data, such as timer, queue, flags and count data.
 *
 */
static int  lis_init_exception_timer(struct lis3dh_acc_data *acceld)
{
	struct lis_gsensor_test_excep *excep = &acceld->lis_gs_exception;
	struct timer_list *excep_timer = &acceld->gsensor_excep_timer;
	int range = 0;
	init_timer(excep_timer);
	excep_timer->data= (unsigned long)(acceld);  /*pointer the current platfrom data*/
	excep_timer->function = lis_exception_2s_timer_handler;

	/* init all data */
	excep->triger_timer_flag = true;
	excep->triger_count_flag = false;
	excep->exception_times  = 0;
	excep->total_times = 0;
	/*range means the full scale of g-sensor*/
	switch(acceld->pdata->g_range){
		case LIS3DH_ACC_G_2G:
			range = 2;
			break;
		//The data all move right 4 bit only can reach 4g
		case LIS3DH_ACC_G_4G:
		case LIS3DH_ACC_G_8G:
		case LIS3DH_ACC_G_16G:
			range = 4;
			break;
		default:
			range = 2;
			break;
	}
	excep->excep_base = 1024*range;
	/* Initialize x_y_z_max x_y_z_min  single_axis_max value form base(1g)*/
	excep->x_y_z_max   = base_to_total_max(excep->excep_base);
	excep->x_y_z_min   = base_to_total_min(excep->excep_base);
	excep->single_axis_max	    =  excep->excep_base;
	acceld->excep_workqueue = create_workqueue("Lis3dh check err Workqueue");

	INIT_WORK(&acceld->excep_work, lis_report_exception_work_handler);

	return 0;
}

/**
 * func - lis_exit_excep_timer
 *
 * 	no need to count exception, delete timer and cancle work.
 *
 */
static void lis_exit_excep_timer(struct lis3dh_acc_data *acceld)
{
	cancel_work_sync(&acceld->excep_work);
	del_timer_sync(&acceld->gsensor_excep_timer);
}


/**
 * func - lis_gsensor_dsm_init
 *
 * 	register dsm_client, and set acceld as dsm_client driver_data.
 *
 */
static int lis_gsensor_dsm_init(struct lis3dh_acc_data  *acceld)
{

	lis3dh_gs_dclient = dsm_register_client(&dsm_gs_lis_i2c);
	if (!lis3dh_gs_dclient) {
		pr_err("[lis3dh_err]register dsm lis3dh_gs_dclient failed!\n");
		return -ENOMEM;
	}

	lis3dh_gs_dclient->driver_data = acceld;
	acceld->gsensor_dclient = lis3dh_gs_dclient;

	return 0;
}

/**
 * func - lis_gsensor_read_i2c_err_info
 *
 *		if i2c transfer err, read i2c_sda/i2c_scl , vdd, vio value.
 *
 */
static void lis_gsensor_read_i2c_err_info(struct lis3dh_acc_data  *acceld)
{

	struct lis_gsensor_test_excep *excep = &acceld->lis_gs_exception;
	struct sensor_regulator *lis3dh_acc_vreg = acceld->lis3dh_acc_vreg;

	int num_reg = 2;
	int i,rc;
	int voltage_mv[2] = {0,0};

	mutex_lock(&acceld->lock_i2c);
	excep->i2c_scl_val = gpio_get_value(acceld->pdata->i2c_scl_gpio);
	excep->i2c_sda_val = gpio_get_value(acceld->pdata->i2c_sda_gpio);
	mutex_unlock(&acceld->lock_i2c);

	for(i = 0; i < num_reg; i++){
		if(NULL == lis3dh_acc_vreg[i].vreg){
			lis3dh_acc_vreg[i].vreg = regulator_get(&acceld->client->dev,
				lis3dh_acc_vreg[i].name);
		}

		if (IS_ERR(lis3dh_acc_vreg[i].vreg)) {
			rc = PTR_ERR(lis3dh_acc_vreg[i].vreg);
			pr_err("[lis3dh_err]%s:regulator get failed rc=%d\n",
							__func__, rc);
			regulator_put(lis3dh_acc_vreg[i].vreg);
			lis3dh_acc_vreg[i].vreg = NULL;
		}else{
			/* get regulator's status and value*/
			voltage_mv[i] = regulator_get_voltage(lis3dh_acc_vreg[i].vreg)/1000;
			if(voltage_mv[i] < 0){
				pr_err("[lis3dh_err]regulator_get_voltage %s %d failed\n",lis3dh_acc_vreg[i].name,i);
			}
		}
	}

	excep->vdd_mv = voltage_mv[0];
	excep->vddio_mv = voltage_mv[1];

}

/**
 * func - read some important registers' value,  and x,y,z value.
 *@auto_report_flag:
 *		auto_report_flag  = false, means userspace app force to read the value, if not err, repot the current x,y,z value
 *		auto_report_flag  = true, the data is exception, and report error value automatically.
 * NOTE:
 */
static void lis_gsensor_read_register_info(struct lis3dh_acc_data  *acceld,bool auto_report_flag)
{
	struct lis_gsensor_test_excep *excep = &acceld->lis_gs_exception;
	int err;
	unsigned char *reg_buf = excep->reg_buf;

	reg_buf[0] = CTRL_REG1;
	err = acceld->i2c_read(acceld, reg_buf, 6);
	if(err < 0){
		pr_err("[lis3dh_err]failed to read ACCEL_INT_REL regs value");
		return ;
	}

	/* if the device monitor auto read the current value, and no exception happened
	* else report the err value, use lock to protect avoid competition.
	*/
	/* delete here, otherwise will cause dead lock*/
	if(auto_report_flag == false && excep->triger_timer_flag == true){
		excep->cur_err_x = excep->xyz[0];
		excep->cur_err_y = excep->xyz[1];
		excep->cur_err_z = excep->xyz[2];
	}
	/* delete here, otherwise will cause dead lock*/

}


/**
 * func - after read sda/scl, vdd/vddio value, and report info to dsm server.
 * NOTE:
 */
static ssize_t lis_gsensor_dsm_record_i2c_err_info(struct lis3dh_acc_data  *acceld)
{
	struct lis_gsensor_test_excep *excep = &acceld->lis_gs_exception;

	ssize_t size = 0;
	ssize_t total_size = 0;

	/* read power and i2c scl/sda gpio value  and report them	*/
	lis_gsensor_read_i2c_err_info(acceld);

	memset(acceld->dsm_buf,0, DSM_SENSOR_BUF_COM);
	snprintf(acceld->dsm_buf, PAGE_SIZE,"i2c_scl_val=%d,i2c_sda_val=%d,vdd = %d, vddio=%d,i2c_errno=%d\n",
					excep->i2c_scl_val,excep->i2c_sda_val,
					excep->vdd_mv,excep->vddio_mv,excep->i2c_err_num);
	size = dsm_client_record(lis3dh_gs_dclient,acceld->dsm_buf);

	total_size += size;

	return total_size;
}

/**
 * func - report basic info.
 * 		after read power, i2c gpio status,x,y,z value, important registers value,
 *		and report info to dsm server.
 */
static ssize_t lis_gsensor_dsm_record_basic_err_info( struct lis3dh_acc_data  *acceld, bool auto_report_flag)
{
	struct lis_gsensor_test_excep *excep = &acceld->lis_gs_exception;

	ssize_t size = 0;
	ssize_t total_size = 0;

	/* read important registers' value and x,y,z value  and report them	*/
	lis_gsensor_read_register_info(acceld,auto_report_flag);

	memset(acceld->dsm_buf,0, PAGE_SIZE);
	snprintf(acceld->dsm_buf, PAGE_SIZE,"CTRL1 = 0x%2x,CTRL2 = 0x%2x ,"
				"CTRL3 = 0x%2x,CTRL4 = 0x%2x,CTRL4 = 0x%2x,CTRL5 = 0x%2x  \n"\
				"excep_x = %d,excep_y = %d,excep_z = %d\n"
				,excep->reg_buf[0],excep->reg_buf[1],excep->reg_buf[2]
				,excep->reg_buf[3],excep->reg_buf[4],excep->reg_buf[5]
				,excep->cur_err_x,excep->cur_err_y,excep->cur_err_z);
	size = dsm_client_record(lis3dh_gs_dclient,acceld->dsm_buf);

	total_size += size;

	/* read power and i2c scl/sda gpio value  and report them	*/
	size = lis_gsensor_dsm_record_i2c_err_info(acceld);

	total_size += size;

	return total_size;

}
void lis_dsm_check_val_same_times(s16 x, s16 y, s16 z,struct lis3dh_acc_data *acc)
{
	s16 temp_xyz[3] = {x, y, z};
	struct lis_gsensor_test_excep *excep = &acc->lis_gs_exception;

	//pr_err("%s:%d x=%d y=%d z=%d\n", __FUNCTION__,__LINE__,x,y,z);
	if(excep->pre_xyz[0] == x)
	{
		excep->x_same_times ++;
	}
	else
	{
		excep->x_same_times = 0;
	}

	if(excep->pre_xyz[1] == y)
	{
		excep->y_same_times ++;
	}
	else
	{
		excep->y_same_times = 0;
	}

	if(excep->pre_xyz[2] == z)
	{
		excep->z_same_times ++;
	}
	else
	{
		excep->z_same_times = 0;
	}
	
	if((excep->x_same_times >= SENSOR_VAL_SAME_MAX_TIMES) || (excep->y_same_times >= SENSOR_VAL_SAME_MAX_TIMES) || (excep->z_same_times >= SENSOR_VAL_SAME_MAX_TIMES))
	{
		if(excep->x_same_times >= SENSOR_VAL_SAME_MAX_TIMES)
		{
			excep->x_same_times = 0;
			pr_err("%s:%d x=%d\n", __FUNCTION__,__LINE__,x);
		}

		if(excep->y_same_times >= SENSOR_VAL_SAME_MAX_TIMES)
		{
			excep->y_same_times = 0;
			pr_err("%s:%d y=%d\n", __FUNCTION__,__LINE__,y);
		}

		if(excep->z_same_times >= SENSOR_VAL_SAME_MAX_TIMES)
		{
			excep->z_same_times = 0;
			pr_err("%s:%d z=%d\n", __FUNCTION__,__LINE__,z);
		}
		
		if(excep->error_times < 10)
		{
			excep->error_times++;
			lis_gsensor_report_dsm_err(DSM_GS_DATA_TIMES_NOTCHANGE_ERROR,false);
		}
	}
	memcpy(excep->pre_xyz, temp_xyz, sizeof(excep->pre_xyz));
}
/* gsensor report err according to err type */
static ssize_t lis_gsensor_report_dsm_err(int type,bool auto_report_flag)
{
	int used_size = 0;
	struct lis3dh_acc_data  *acceld = (struct lis3dh_acc_data *)lis3dh_gs_dclient->driver_data;

	/* try to get permission to use the buffer */
	if (dsm_client_ocuppy(lis3dh_gs_dclient)) {
		/* buffer is busy */
		pr_err("[lis3dh_err]%s: buffer is busy!\n", __func__);
		return -EBUSY;
	}
	acceld->lis_gs_exception.excep_num = type;
	/* gsensor report err according to err type */
	switch (type) {
		case DSM_GS_I2C_ERROR:
			/* report i2c infomation */
			used_size = lis_gsensor_dsm_record_i2c_err_info(acceld);
			break;
		case DSM_GS_XYZ_0_ERROR:
		case DSM_GS_DATA_TIMES_NOTCHANGE_ERROR:
		case DSM_GS_DATA_ERROR:
			/* report gsensor basic infomation */
			used_size = lis_gsensor_dsm_record_basic_err_info(acceld, auto_report_flag);
			break;

		default:
			break;
	}
	/*if device is not probe successfully or client is null, don't notify dsm work func*/
	if (false == acceld->device_exist || lis3dh_gs_dclient == NULL) {
		return -ENODEV;
	}
	dsm_client_notify(lis3dh_gs_dclient, type);

	return used_size;
}

/* force dump some gsensor infomation */
static int lis_dump_gsensor_info(int type, void *buff, int size)
{
	int used_size = 0;
	struct dsm_client *gs_client = lis3dh_gs_dclient;

	used_size = lis_gsensor_report_dsm_err(DSM_GS_DATA_ERROR,false);

	if( used_size > 0 )
	{
		pr_err("[lis3dh_err]%s: force dump gsensor error!\n",__func__);
		snprintf( buff, gs_client->used_size, lis3dh_gs_dclient->dump_buff );
	}

	return lis3dh_gs_dclient->used_size;
}


/**
 * func - register some call back func to check exception and report error.
 *
 *		Initialize the abnormal configuration, set check data error func, i2c transfer error dump func,
 *		and report func to device monitor server.
 */
static int register_lis3dh_dsm_operations(struct lis3dh_acc_data  *acceld)
{
	int ret;
	struct lis_gs_dsm_operation *pt_lis3dh_dsm_ops = &acceld->lis3dh_dsm_operation;

	ret = lis_gsensor_dsm_init(acceld);
	if(ret < 0){

		return ret;
	}

	pt_lis3dh_dsm_ops->dump_i2c_status		= lis_dump_i2c_exception_status;
	pt_lis3dh_dsm_ops->judge_check_excep	= lis_check_exception;
	pt_lis3dh_dsm_ops->judge_same_value_excep = lis_dsm_check_val_same_times;
	/*delete some line*/
	return 0;
}



static ssize_t attr_get_excep_base(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lis3dh_acc_data *acceld = i2c_get_clientdata(client);
	struct lis_gsensor_test_excep *excep = &acceld->lis_gs_exception;

	mutex_lock(&acceld->lock_i2c);
	excep->i2c_scl_val = gpio_get_value(acceld->pdata->i2c_scl_gpio);
	excep->i2c_sda_val = gpio_get_value(acceld->pdata->i2c_sda_gpio);
	mutex_unlock(&acceld->lock_i2c);

	return snprintf(buf, 200,"excep_base=%d,x_y_z_max=%d,x_y_z_min=%d,single_axis_max=%d\n"
			"scl = %d, sda = %d\n",
			excep->excep_base,excep->x_y_z_max,
			excep->x_y_z_min,excep->single_axis_max,
			excep->i2c_scl_val,excep->i2c_sda_val);
}

/*set exception check base interface*/
static ssize_t attr_set_excep_base(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lis3dh_acc_data *acceld = i2c_get_clientdata(client);
	struct lis_gsensor_test_excep *excep = &acceld->lis_gs_exception;

	unsigned long excep_base;

	if (kstrtoul(buf, 10, &excep_base))
		return -EINVAL;

	/*recalculate and refresh x_y_z_max  x_y_z_min single_axis_max value*/
	excep->excep_base 			= excep_base;
	excep->x_y_z_max   			= base_to_total_max(excep->excep_base);
	excep->x_y_z_min   			= base_to_total_min(excep->excep_base);
	excep->single_axis_max		= base_to_single_axis_max(excep->excep_base);

	return count;
}

static ssize_t attr_get_registers_value(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct lis3dh_acc_data *acceld = i2c_get_clientdata(client);
	unsigned char reg[6];

	reg[0] = CTRL_REG1;
	ret = acceld->i2c_read(acceld, reg, 6);
	if(ret < 0){
		pr_err("[lis3dh_err]%s,line %d: failed to read regs value",__func__,__LINE__);
		return ret;
	}

	return snprintf(buf,512,"ctl_reg[0x20~0x25]=0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x\n"
			,reg[0],reg[1],reg[2],reg[3],reg[4],reg[5]);
}



static struct device_attribute dbg_attributes[] = {
	__ATTR(excep_base, 0664, attr_get_excep_base,attr_set_excep_base),
	__ATTR(dump_regs, 0444, attr_get_registers_value,NULL),
};


int lis_dsm_excep_init(struct lis3dh_acc_data  *acc)
{
	int i = 0,ret = 0;

	ret = register_lis3dh_dsm_operations(acc);
	if(ret < 0)
		goto err_unregister_dsm;

	lis_init_exception_timer(acc);

	for (i = 0; i < ARRAY_SIZE(dbg_attributes); i++) {
		ret = device_create_file(&acc->client->dev, dbg_attributes + i);
		if (ret)
			goto error_device_create;
	}

	acc->dsm_buf = kzalloc(DSM_SENSOR_BUF_COM,GFP_KERNEL);
	if(!acc->dsm_buf){
		pr_err("[lis3dh_err]failed to alloc space for dsm_buf");
		ret = -ENOMEM;
		goto error_device_create;
	}
	return ret;

error_device_create:
	for (; i >= 0; i--)
		device_remove_file(&acc->client->dev, dbg_attributes + i);

	pr_err("[lis3dh_err]%s:Unable to create interface\n", __func__);
err_unregister_dsm:
	dsm_unregister_client(lis3dh_gs_dclient,&dsm_gs_lis_i2c);

	return ret;

}


void lis_dsm_excep_exit(struct lis3dh_acc_data  *acc)
{
	int i = 0;

	if ( NULL != acc->dsm_buf )
	{
		kfree(acc->dsm_buf);
		acc->dsm_buf = NULL;
	}

	for (i = 0; i < ARRAY_SIZE(dbg_attributes); i++) {
		device_remove_file(&acc->client->dev, dbg_attributes + i);
	}

	dsm_unregister_client(lis3dh_gs_dclient,&dsm_gs_lis_i2c);

	lis_exit_excep_timer(acc);

	pr_info("[lis3dh_info]unregister lis3dh dsm_client.\n");
}
#endif


