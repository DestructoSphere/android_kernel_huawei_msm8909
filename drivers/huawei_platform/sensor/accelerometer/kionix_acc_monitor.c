#ifdef CONFIG_HUAWEI_DSM
/* drivers/input/misc/kionix_acc_monitor.c - Kionix accelerometer driver monitor interface
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
#include	<huawei_platform/sensor/kionix_accel.h>
#include	<huawei_platform/sensor/hw_sensor_info.h>

#define CLIENT_NAME_GS_KX			"dsm_gs_kx023"

extern int kx023_debug_mask;
static struct dsm_client *kx023_gs_dclient = NULL;
static int dump_gsensor_info (int type, void *buff, int size);
static ssize_t gsensor_report_dsm_err(int type,bool auto_report_flag);
/* add gsensor dump_func */
static struct dsm_client_ops gs_ops={
	.poll_state = NULL,
	.dump_func = dump_gsensor_info,
};

/* dsm client for gsensor */
static struct dsm_dev dsm_gs_i2c = {
	.name = CLIENT_NAME_GS_KX,	// dsm client name
	.fops = &gs_ops,		// options
	.buff_size = DSM_SENSOR_BUF_MAX,		// buffer size
};

/**
 * func - register a new dsm_client , and set driver data to the client's private data.
  */
static int gsensor_dsm_init(struct kionix_accel_driver *acceld)
{

	kx023_gs_dclient = dsm_register_client(&dsm_gs_i2c);
	if (!kx023_gs_dclient) {
		KIONIX_ERR("register dms kx023_gs_dclient failed!");
		return -ENOMEM;
	}

	kx023_gs_dclient->driver_data = acceld;
	acceld->gsensor_dclient = kx023_gs_dclient;

	return 0;
}

/*
* check the current data is error or not
*/
static inline bool exception_condition(int x, int y,int z,struct gsensor_test_excep *excep)
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


/**
*	print main registers and xyz vaules
*/
static void dump_regs_xyz_values(struct work_struct *work)
{
	unsigned char buf[2];
	int err;
	struct kionix_accel_driver *acceld = container_of((struct delayed_work *)work, struct kionix_accel_driver, debug_work);
	acceld->queued_debug_work_flag = false;
	KIONIX_INFO("%s: acceld->poll_delay=%d, acceld->accel_enabled=%d, acceld->accel_enable_resume=%d",__func__,
	                          acceld->poll_delay,
	                          atomic_read(&acceld->accel_enabled),
	                          atomic_read(&acceld->accel_enable_resume));

	if(atomic_read(&acceld->accel_enabled) <= 0){
		KIONIX_DBG("accel disabled");
		return ;
	}
	err = acceld->i2c_read(acceld, ACCEL_INT_REL,buf, 2);
	if(err < 0){
		KIONIX_ERR("failed to read ACCEL_INT_REL regs value,err = %d",err);
		return ;
	}
	KIONIX_DBG("ACCEL_INT_REL = 0x%2x,ACCEL_CTRL_REG1 = 0x%2x",buf[0],buf[1]);

	if(buf[0] != 0 || buf[1] != 0xc0)
		KIONIX_ERR("register value err.");

	err = acceld->i2c_read(acceld, ACCEL_DATA_CTRL,buf, 2);
	if(err < 0){
		KIONIX_ERR("failed to read ACCEL_DATA_CTRL regs value,err = %d",err);
		return ;
	}
	KIONIX_DBG("ACCEL_DATA_CTRL = 0x%2x,ACCEL_INT_CTRL1 = 0x%2x",buf[0],buf[1]);

	KIONIX_INFO("%s,raw_x=%d", __func__,acceld->accel_data[acceld->axis_map_x]);
	KIONIX_INFO("%s,raw_y=%d", __func__,acceld->accel_data[acceld->axis_map_y]);
	KIONIX_INFO("%s,raw_z=%d", __func__, acceld->accel_data[acceld->axis_map_z]);

}


/*
* if i2c transfer error, we check sda/scl value and regulator's value
*/
static int kx_gs_dump_i2c_exception_status(struct kionix_accel_driver *acceld, int i2c_err)
{
	/* print pm status and i2c gpio status*/
	struct gsensor_test_excep *excep = &acceld->gsensor_test_exception;

	excep->i2c_err_num = i2c_err;

	/* call report dsm error function with specified error number*/
	gsensor_report_dsm_err(DSM_GS_I2C_ERROR,true);

	KIONIX_ERR("i2c_scl_val=%d,i2c_sda_val=%d,vdd = %d, vddio=%d,i2c_errno=%d",
				excep->i2c_scl_val,excep->i2c_sda_val,
				excep->vdd_mv,excep->vddio_mv,excep->i2c_err_num);

	excep->i2c_err_num = 0;

	return 0;
}


/**
 * func - kx_gs_check_exception
 * @triger_timer_flag, if first data is error, triger_timer_flag = false, now counut exception data
 *		if timer is trigered, enter counting exceptions times mode in 2s,
 *		don't allow to trigger timer
 */
static int kx_gs_check_exception(int *xyz, struct kionix_accel_driver *acceld )
{
	struct gsensor_test_excep *excep = &acceld->gsensor_test_exception;
	struct timer_list *excep_timer = &acceld->gsensor_excep_timer;
	bool condition = false;
	int x = xyz[0], y = xyz[1], z = xyz[2];
	static int excep_times = 0;
	static int data_err_times = 0;
	/* when an error occur, enter counting exceptions times mode, don't allow to trigger timer */
	if(excep->triger_timer_flag){
		condition = exception_condition(x,y,z,excep);
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
		}
	}
	/*counting exceptions times until 2s  is over*/
	if (excep->triger_count_flag) {
		excep->total_times++;
		condition = exception_condition(x, y, z, excep);
		if (condition) {
			excep->exception_times ++;
		}
	}

	/* if data always exception, can't print so many log, phone maybe crash.*/
	if (condition) {
		if (0 == excep_times % 15 ) {
			KIONIX_INFO(" xyz_warn. x = %d, y = %d, z = %d",
						x, y, z);
		} else if (excep_times >= 30) {
			excep_times = 0;
		}

		excep_times ++;
	}

/*
  * kx023 sensor is more stable when it is stationary, and vendor say
  * it's a feature of this sensor, so it can used to measure angle.
  *	so delete this check point.
  *
	if (excep->pre_xyz[0] == x
		&& excep->pre_xyz[1] == y
		&& excep->pre_xyz[2] == z) {
		excep->same_times ++;
	} else {
		excep->same_times = 0;
	}

	memcpy(excep->pre_xyz, xyz, 3*sizeof(int));
	if (excep->same_times >= SENSOR_VAL_SAME_MAX_TIMES) {
		excep->same_times = 0;
		gsensor_report_dsm_err(DSM_GS_DATA_TIMES_NOTCHANGE_ERROR, false);
	}
*/

	if ( 0 == x && 0 == y && 0 == z) {
		KIONIX_ERR("WARNING:%s at %s %d:kx023 acceler data xyz = 0"
				   , __FILE__, __func__, __LINE__);
		if(10 < data_err_times++)
		{
			gsensor_report_dsm_err(DSM_GS_XYZ_0_ERROR, false);
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
 * func - exception_2s_timer_handler
 *	check the accelerator's data is error or not in 2 seconds,
 *  	if error, schedule  func: report_exception_work_handler to report status.
 *	after execute the func, clear all flag and count data.
 */
static void exception_2s_timer_handler(unsigned long data)
{
	int exception_percentage;

	struct kionix_accel_driver *acceld = (struct kionix_accel_driver *)data;
	struct gsensor_test_excep *excep = &acceld->gsensor_test_exception;

	/*allow to triger timer again*/
	excep->triger_timer_flag = true;
	excep->triger_count_flag = false;

	exception_percentage = 100 *excep->exception_times / excep->total_times;
	if(exception_percentage >= EXCEPTION_PERCENTAGE
		&& excep->exception_times >= EXCEPTION_BASE_TIMES
		&&(atomic_read(&acceld->accel_enabled) > 0)){
		KIONIX_ERR("---------cut here-------\n"
			"WARNING:%s at %s %d:kx023 acceler data is exception"
			,__FILE__,__func__,__LINE__);
		/* schedule func: report_exception_work_handler*/
		queue_work(acceld->accel_workqueue, &acceld->excep_dwork);
	}

	/*clear count times for next time*/
	excep->exception_times  = 0;
	excep->total_times = 0;
}

/**
 * func - report_exception_work_handler
 *
 * 	report exception data to device monitor and kernel log.
 */
static void report_exception_work_handler(struct work_struct *work)
{
	struct kionix_accel_driver *acceld = container_of(work, struct kionix_accel_driver, excep_dwork);
	struct gsensor_test_excep *exception = &acceld->gsensor_test_exception;

	gsensor_report_dsm_err(DSM_GS_DATA_ERROR,true);

	KIONIX_ERR("ACCEL_INT_REL = 0x%2x,ACCEL_CTRL_REG1 = 0x%2x ,"\
				"ACCEL_DATA_CTRL = 0x%2x,ACCEL_INT_CTRL1 = 0x%2x \n"\
				"excep_x = %d,excep_y = %d,excep_z = %d\n"
				"i2c_scl_val=%d,i2c_sda_val=%d,excep_num = %d \nvdd = %d, vddio=%d,i2c_errno=%d"
			,exception->reg_buf[0],exception->reg_buf[1],exception->reg_buf[2],exception->reg_buf[3]
			,exception->cur_err_x,exception->cur_err_y,exception->cur_err_z
			,exception->i2c_scl_val,exception->i2c_sda_val,exception->excep_num
			,exception->vdd_mv,exception->vddio_mv,exception->i2c_err_num);
}

/**
 * func - kx_gs_init_exception_timer
 *
 * 		init the exception data, such as timer, queue, flags and count data.
 *
 */
static void  kx_gs_init_exception_timer(struct kionix_accel_driver *acceld)
{
	struct gsensor_test_excep *excep = &acceld->gsensor_test_exception;
	struct timer_list *excep_timer = &acceld->gsensor_excep_timer;
	int range = 0;
	init_timer(excep_timer);
	excep_timer->data= (unsigned long)(acceld);  /*pointer the current platfrom data*/
	excep_timer->function = exception_2s_timer_handler;

	/* init all data */
	excep->triger_timer_flag = true;
	excep->triger_count_flag = false;
	excep->exception_times  = 0;
	excep->total_times = 0;
	/*range means the full scale of g-sensor*/
	switch(acceld->accel_pdata->accel_g_range){
		case KIONIX_ACCEL_G_2G:
			range = 2;
			break;
		case KIONIX_ACCEL_G_4G:
			range = 4;
			break;
		case KIONIX_ACCEL_G_6G:
		case KIONIX_ACCEL_G_8G:
			range = 8;
			break;
		default:
			range = 2;
			break;
	}
	excep->excep_base = 1024*range;
	/* Initialize x_y_z_max x_y_z_min  single_axis_max value form base(1g)*/
	excep->x_y_z_max   		= base_to_total_max(excep->excep_base);
	excep->x_y_z_min   		= base_to_total_min(excep->excep_base);
	excep->single_axis_max	= excep->excep_base;
	INIT_WORK(&acceld->excep_dwork, report_exception_work_handler);
	/* use delay work to debug x,y,z value, this will make cpu efficiently than old method*/
	INIT_DELAYED_WORK(&acceld->debug_work, dump_regs_xyz_values);
	acceld->queued_debug_work_flag = false;

}

/**
 * func - kx_gs_exit_excep_timer
 *
 * 	no need to count exception, delete timer and cancle work.
 *
 */
static void kx_gs_exit_excep_timer(struct kionix_accel_driver *acceld)
{
	cancel_delayed_work_sync(&acceld->debug_work);
	cancel_work_sync(&acceld->excep_dwork);
	del_timer_sync(&acceld->gsensor_excep_timer);
}


/**
 * func - gsensor_read_i2c_err_info
 *
 *		if i2c transfer err, read i2c_sda/i2c_scl , vdd, vio value.
 *
 */
static void gsensor_read_i2c_err_info(struct kionix_accel_driver *acceld)
{

	struct gsensor_test_excep *exception = &acceld->gsensor_test_exception;
	struct sensor_regulator *kionix_acc_vreg = acceld->kionix_acc_vreg;

	int num_reg = 2;
	int i,rc;
	int voltage_mv[2] = {0,0};

	mutex_lock(&acceld->lock_i2c);
	exception->i2c_scl_val = gpio_get_value(acceld->accel_pdata->i2c_scl_gpio);
	exception->i2c_sda_val = gpio_get_value(acceld->accel_pdata->i2c_sda_gpio);
	mutex_unlock(&acceld->lock_i2c);

	for(i = 0; i < num_reg; i++){
		if(false == kionix_acc_vreg[i].got_regulator_flag){
			kionix_acc_vreg[i].vreg = regulator_get(&acceld->client->dev,
				kionix_acc_vreg[i].name);
			kionix_acc_vreg[i].got_regulator_flag = true;
		}

		if (IS_ERR(kionix_acc_vreg[i].vreg)) {
			rc = PTR_ERR(kionix_acc_vreg[i].vreg);
			KIONIX_ERR("%s:regulator get failed rc=%d\n",
							__func__, rc);
			regulator_put(kionix_acc_vreg[i].vreg);
			kionix_acc_vreg[i].vreg = NULL;
			kionix_acc_vreg[i].got_regulator_flag = false;
		}else{
			/* get regulator's status and value*/
			voltage_mv[i] = regulator_get_voltage(kionix_acc_vreg[i].vreg)/1000;
			if(voltage_mv[i] < 0){
				KIONIX_ERR("regulator_get_voltage %s %d failed",kionix_acc_vreg[i].name,i);
			}
		}
	}

	exception->vdd_mv = voltage_mv[0];
	exception->vddio_mv = voltage_mv[1];

}

/**
 * func - read some important registers' value,  and x,y,z value.
 *@auto_report_flag:
 *		auto_report_flag  = false, means userspace app force to read the value, if not err, repot the current x,y,z value
 *		auto_report_flag  = true, the data is exception, and report error value automatically.
 * NOTE:
 */
static void gsensor_read_register_info(struct kionix_accel_driver *acceld,bool auto_report_flag)
{
	struct gsensor_test_excep *exception = &acceld->gsensor_test_exception;
	int err;
	unsigned char *reg_buf = exception->reg_buf;

	err = acceld->i2c_read(acceld, ACCEL_INT_REL,reg_buf, 2);
	if(err < 0){
		KIONIX_DBG("failed to read ACCEL_INT_REL regs value");
		return ;
	}

	err = acceld->i2c_read(acceld, ACCEL_DATA_CTRL,&reg_buf[2], 2);
	if(err < 0){
		KIONIX_DBG("failed to read ACCEL_DATA_CTRL regs value");
		return	;
	}
	/* if the device monitor auto read the current value, and no exception happened
	* else report the err value.
	*/
	if(auto_report_flag == false && exception->triger_timer_flag == true){
		exception->cur_err_x = acceld->accel_data[acceld->axis_map_x];
		exception->cur_err_y = acceld->accel_data[acceld->axis_map_y];
		exception->cur_err_z = acceld->accel_data[acceld->axis_map_z];
	}

}


/**
 * func - after read sda/scl, vdd/vddio value, and report info to dsm server.
 * NOTE:
 */
static ssize_t gsensor_dsm_record_i2c_err_info(struct kionix_accel_driver *acceld)
{
	struct gsensor_test_excep *exception = &acceld->gsensor_test_exception;

	ssize_t size = 0;
	ssize_t total_size = 0;

	/* read power and i2c scl/sda gpio value  and report them	*/
	gsensor_read_i2c_err_info(acceld);

	size = dsm_client_record(kx023_gs_dclient,
					"i2c_scl_val=%d,i2c_sda_val=%d,excep_num = %d,vdd = %d, vddio=%d,i2c_errno=%d\n",
				exception->i2c_scl_val,exception->i2c_sda_val,exception->excep_num,
				exception->vdd_mv,exception->vddio_mv,exception->i2c_err_num);
	total_size += size;

	return total_size;
}

/**
 * func - gsensor_dsm_record_basic_err_info
 * 		after read power, i2c gpio status,x,y,z value, important registers value,
 *		and report info to dsm server.
 */
static ssize_t gsensor_dsm_record_basic_err_info( struct kionix_accel_driver *acceld, bool auto_report_flag)
{
	struct gsensor_test_excep *exception = &acceld->gsensor_test_exception;

	ssize_t size = 0;
	ssize_t total_size = 0;

	/* read important registers' value and x,y,z value  and report them	*/
	gsensor_read_register_info(acceld,auto_report_flag);

	memset(acceld->dsm_buf,0, DSM_SENSOR_BUF_COM);
	snprintf(acceld->dsm_buf,PAGE_SIZE,"ACCEL_INT_REL = 0x%2x,ACCEL_CTRL_REG1 = 0x%2x,"
				"ACCEL_DATA_CTRL = 0x%2x,ACCEL_INT_CTRL1 = 0x%2x \n"\
				"excep_x = %d,excep_y = %d,excep_z = %d\n"
				,exception->reg_buf[0],exception->reg_buf[1],exception->reg_buf[2],exception->reg_buf[3]
				,exception->cur_err_x,exception->cur_err_y,exception->cur_err_z);
	size =dsm_client_record(kx023_gs_dclient,acceld->dsm_buf);

	total_size += size;

	/* read power and i2c scl/sda gpio value  and report them	*/
	size = gsensor_dsm_record_i2c_err_info(acceld);

	total_size += size;

	return total_size;

}
void kionix_dsm_check_val_same_times(s16 x, s16 y, s16 z,struct kionix_accel_driver *acceld)
{
	s16 temp_xyz[3] = {x, y, z};
	struct gsensor_test_excep *excep = &acceld->gsensor_test_exception;

	//KIONIX_INFO("%s:%d x=%d y=%d z=%d\n", __FUNCTION__,__LINE__,x,y,z);
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
			KIONIX_INFO("%s:%d x=%d\n", __FUNCTION__,__LINE__,x);
		}

		if(excep->y_same_times >= SENSOR_VAL_SAME_MAX_TIMES)
		{
			excep->y_same_times = 0;
			KIONIX_INFO("%s:%d y=%d\n", __FUNCTION__,__LINE__,y);
		}

		if(excep->z_same_times >= SENSOR_VAL_SAME_MAX_TIMES)
		{
			excep->z_same_times = 0;
			KIONIX_INFO("%s:%d z=%d\n", __FUNCTION__,__LINE__,z);
		}
		
		if(excep->error_times < 10)
		{
			excep->error_times++;
			gsensor_report_dsm_err(DSM_GS_DATA_TIMES_NOTCHANGE_ERROR,false);
		}
	}
	
	memcpy(excep->pre_xyz, temp_xyz, sizeof(excep->pre_xyz));
}

/**
 * func - report err according to err type
 * NOTE:
 */
static ssize_t gsensor_report_dsm_err(int type,bool auto_report_flag)
{
	int used_size = 0;
	struct kionix_accel_driver *acceld = (struct kionix_accel_driver *)kx023_gs_dclient->driver_data;

	/* try to get permission to use the buffer */
	if(dsm_client_ocuppy(kx023_gs_dclient))
	{
		/* buffer is busy */
		KIONIX_ERR("%s: buffer is busy!", __func__);
		return -EBUSY;
	}
	acceld->gsensor_test_exception.excep_num = type;
	/* gsensor report err according to err type */
	switch(type)
	{
		case DSM_GS_I2C_ERROR:
			/* report i2c infomation */
			used_size = gsensor_dsm_record_i2c_err_info(acceld);
			break;

		case DSM_GS_XYZ_0_ERROR:
		case DSM_GS_DATA_TIMES_NOTCHANGE_ERROR:
		case DSM_GS_DATA_ERROR:
			/* report gsensor basic infomation */
			used_size = gsensor_dsm_record_basic_err_info(acceld,auto_report_flag);
			break;

		default:
			break;
	}
	/*if device is not probe successfully or client is null, don't notify dsm work func*/
 	if(false == acceld->device_exist || kx023_gs_dclient == NULL){
		return -ENODEV;
	}
	dsm_client_notify(kx023_gs_dclient, type);

	return used_size;
}

/**
 * func - force dump some gsensor infomation
  */
static int dump_gsensor_info (int type, void *buff, int size)
{
	int used_size = 0;
	struct dsm_client *gs_client = kx023_gs_dclient;

	used_size = gsensor_report_dsm_err(DSM_GS_DATA_ERROR,false);

	if( used_size > 0 )
	{
		KIONIX_DBG("%s: force dump tp error!",__func__);
		snprintf( buff, gs_client->used_size, kx023_gs_dclient->dump_buff );
	}

	return used_size;
}

/**
 * func - register dsm_client and set callback function
  */
int register_kx023_dsm_operations(struct kionix_accel_driver *acceld)
{
	int ret;
	struct kx_gs_dsm_operation *pt_kx023_dsm_ops = &acceld->kx023_dsm_operation;

	ret = gsensor_dsm_init(acceld);
	if(ret < 0){
		KIONIX_ERR("gsensor_dsm_init failed.");
		return ret;
	}
	kx_gs_init_exception_timer(acceld);

	pt_kx023_dsm_ops->dump_i2c_status	= kx_gs_dump_i2c_exception_status;
	pt_kx023_dsm_ops->check_exception	= kx_gs_check_exception;
	pt_kx023_dsm_ops->judge_same_value_excep = kionix_dsm_check_val_same_times;
	acceld->dsm_buf = kzalloc(DSM_SENSOR_BUF_COM,GFP_KERNEL);
	if(!acceld->dsm_buf){
		KIONIX_ERR("failed to alloc space for dsm_buf");
		return -ENOMEM;
	}

	return 0;
}


void unregister_kx023_dsm_operations(struct kionix_accel_driver *acceld)
{
	kfree(acceld->dsm_buf);
	acceld->dsm_buf = NULL;

	kx_gs_exit_excep_timer(acceld);

	dsm_unregister_client(kx023_gs_dclient,&dsm_gs_i2c);
	KIONIX_INFO("unregister kx023_dsm_operations.");
}

#endif



