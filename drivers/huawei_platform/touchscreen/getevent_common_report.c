/*
 *
 * The playback of input log.
 *
 *
 * This program is distributed in the driver that it will be playback the input-log of touch panel,
 * it create a independent input-subsystem.it will playback the input-log independent of the ic 
 * company.so it is independent.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/input/mt.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <huawei_platform/touchscreen/hw_tp_common.h>
#include "getevent_common_report.h"

extern struct kobject *virtual_key_kobject_ts;
static struct mutex     getevent_device_mutex;
static struct kobject  *getevent_common_kobject= NULL;
struct getevent_virtual_keys_data *getevent_virtual_pdata = NULL;
static char *key_names[KEY_NAME_LEN] = {"KEY_BACK","KEY_HOME","KEY_MENU"};

int getevent_common_log_mask = GETEVENT_ERR;
module_param_named(getevent_common_log_mask, getevent_common_log_mask, int, 0664);

/**
 * atoi - change the char to int
 *
 * @psz_buf: the name for the kobject
 *
 * This function change the char to int.
 *
 * If the kobject was not empty, int with sym will be returned.
 */
static unsigned int  atoi(char *psz_buf)
{
	unsigned int i=0,  sym=1;
	unsigned int val=0;

	while( psz_buf[i] != '\0')
	{
		if( ' ' != psz_buf[i] )
		{
			if( (psz_buf[i] >= '0') && (psz_buf[i] <= '9') )
			{
				val = val*16 + (psz_buf[i]-'0');
			}
			else if(psz_buf[i] >= 'A' && psz_buf[i] <= 'F')
			{
				val = val*16 + psz_buf[i]-'A'+10;
			}
			else if(psz_buf[i] >= 'a' && psz_buf[i] <= 'f')
			{
				val = val*16 + psz_buf[i]-'a'+10;
			}
			else
			{
				return sym*val;
			}
		}
		i++;
	}

	return sym*val;
}

/**
 * getevent_common_set_input_dev - allocate input device and set the input parameter 
 *
 * @getevent_dev: the struct of play back
 *
 * This function allocate input device and set the input parameter.
 *
 * If the input device is allocated and set successful, zero will be returned.
 */
static int getevent_common_set_input_dev(struct getevent_dev *getevent_dev)
{
	int rc =0;
	struct input_dev *input_dev = NULL;
	
	input_dev = input_allocate_device();
	if (!input_dev) {
		getevent_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		rc = -ENOMEM;
		goto exit_input_dev_alloc_failed;
	}
	
	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_PRESSURE, input_dev->absbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_SYN, input_dev->evbit);
	if(0 == strcasecmp(getevent_dev->agreement,"b"))
	{
		input_mt_init_slots(input_dev,10,0);
	}
	else
	{
		input_set_abs_params(input_dev, ABS_MT_TRACKING_ID,0, 255, 0, 0);
	}
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, getevent_dev->x_max -1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, getevent_dev->y_max -1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR,0, 255, 0, 0);
	
	input_dev->name = GETEVENT_COMMON_REPORT_NAME;
	rc = input_register_device(input_dev);
	if (rc) { 
		getevent_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		goto exit_input_register_device_failed;
	}
	getevent_dev->input_dev = input_dev;
	return 0;

	
exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
	return rc;
}

/**
 * getevent_get_size_of_file - get size of file and return file size
 *
 * @file_name: the file_name with absolute path
 *
 * This function use the file_path with absolute path,it will get the size of file which patch is /data/log/ and so on. 
 *
 * If the file_name is exist and not empty, file size  will be returned.if the file_name is not exist,then error will returned.
 */
static off_t getevent_get_size_of_file(char *file_name)
{
	struct file *pfile = NULL;
	struct inode *inode = NULL;
	unsigned long magic;
	off_t fsize = 0;

	if (NULL == pfile)
		pfile = filp_open(file_name, O_RDONLY, 0);
	if(IS_ERR(pfile))
	{
		getevent_log_dbg("%s:open %s file failed\n",__func__,file_name);
		return -EIO;
	}
		
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}

/**
 * getevent_read_file_data - get size of file and return file data
 *
 * @file_name: the file_name with absolute path
 * @file_buf:    the buf to storage the file data
 *
 * This function use the file_path with absolute path,it will get the file data and storage it to file_buf.
 *
 * If the file_name is exist and not empty, file data will be storaged to file_buf.if the file_name is not exist,then error will returned.
 */
static int getevent_read_file_data(char *file_name, char *file_buf)
{
	struct file *pfile = NULL;
	struct inode *inode = NULL;
	unsigned long magic;
	off_t fsize;
	loff_t pos;
	mm_segment_t old_fs;
	if(NULL == pfile)
		pfile = filp_open(file_name, O_RDONLY, 0);

	if(IS_ERR(pfile))
	{
		return -EIO;
	}
	
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, file_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);
	return 0;
}

/**
 * getevent_handle_of_file - get size of file and return file data
 *
 * @file_name: the file_name with absolute path
 * @out_pfile:   the buf to storage the file data
 * @out_size:   the size of file
 *
 * This function use the file_path with absolute path,it will alloc file_buf and handle file data.
 *
 * If it can alloc file data and file_name is exist or not empty, out_pfile will point to alloced buf and file_size will be storged to out_size.
 */
static int getevent_handle_of_file(char *file_name,char **out_pfile,u32 *out_size)
{
	off_t  inisize = 0;
	char *file_data = NULL;
	getevent_log_dbg("%s:line=%d\n",__func__,__LINE__);
	
	if(*out_pfile != NULL)
	{
		return -EIO;
	}
	
	inisize = getevent_get_size_of_file(file_name);
	if (inisize <= 0) 
	{
		return -EIO;
	}
	getevent_log_dbg("%s:%s size is =%ld\n",__func__,file_name,inisize);
	
	if(file_data == NULL)
		file_data = kzalloc(inisize, GFP_KERNEL);
	if(!file_data)
	{	
		getevent_log_dbg("%s:malloc file_data failed\n",__func__);
		return -ENOMEM;
	}
	
	/*read the file to filedata which is the memory_buf*/	
	if (getevent_read_file_data(file_name, file_data)) {        
		kfree(file_data);
		return -EIO;
	} 
	
	getevent_log_dbg("%s:read %s success\n",__func__,file_name);
	
	*out_pfile = file_data;
	*out_size = inisize;
	return 0;
}

/**
 * getevent_get_info_size - get size of file and return file data
 *
 * @getevent_dev: the struct of play back.
 *
 * This function get the line of file_data.
 *
 * If getevent_dev->file_data is exist then line of file_data will be returned .
 */
static size_t getevent_get_info_size(struct getevent_dev *getevent_dev)
{
	size_t rc = 0,parm_num = 0;
	for(rc = 0 ;rc < getevent_dev->file_size; rc++)
	{
		if(getevent_dev->file_data[rc] =='\n')
		{
			parm_num++;
		}
	}
#ifdef CONFIG_64BIT
	getevent_log_dbg("getevent_dev->para_num = %ld \n",parm_num);
#else
	getevent_log_dbg("getevent_dev->para_num = %d \n",parm_num);
#endif
	return parm_num;
}

/**
 * getevent_resolve_event_log - get size of file and return file data
 * @para_table: getevent info.
 * @getevent_dev: the struct of play back.
 *
 * This function malloc para_num data,then resolve the file_buf of input log by line.
 *
 * If resolved the file data by line success,getevent info(event,type,code,value,rate) will be stroged to line buf,
    else will return error.
 */
int getevent_resolve_event_log(void **para_table,struct getevent_dev *getevent_dev)
{
	u32 time0 = 0;
	u32 time1 = 0;
	char event[TP_GETEVENT_EVENT_LEN_MAX] = {0};
	char type[TP_GETEVENT_TYPE_LEN_MAX] = {0};
	char code[TP_GETEVENT_CODE_LEN_MAX] = {0};
	char value[TP_GETEVENT_VALUE_LEN_MAX] = {0};
	char rate[TP_GETEVENT_RATE_LEN_MAX] = {0};
	char buf[HW_TP_GETEVENT_LINE_MAX] = {0};
	char table1[TP_GETEVENT_TABLE_LEN_MAX] = {0};
	char table2[TP_GETEVENT_TABLE_LEN_MAX] = {0};
	struct getevent_info * tp_getevent_para = NULL;
	char *file_data_buf = NULL;
	char *file_data_temp = NULL;
	int tp_getevent_line_num=0;
	int ret=0;
	
	getevent_dev->para_num = getevent_get_info_size(getevent_dev);
	tp_getevent_para = kzalloc( (size_t)(sizeof(*tp_getevent_para)*getevent_dev->para_num), GFP_KERNEL);
	
	if(NULL ==  tp_getevent_para){
		ret = -ENOMEM;
		goto kalloc_err;
	}
	
	file_data_buf = getevent_dev->file_data;
	
	while(NULL != (file_data_buf = strstr(file_data_buf,"][")))
	{
		if(NULL !=(file_data_buf = strchr(file_data_buf,'[')))//find the time
		{
			memset(buf,0,sizeof(buf));
			memset(event,0,sizeof(event));
			memset(type,0,sizeof(type));
			memset(code,0,sizeof(code));
			memset(value,0,sizeof(value));
			memset(rate,0,sizeof(rate));
			if(NULL !=(file_data_temp = strchr(file_data_buf,'\n')))//find the line end
			{	
				memcpy(buf,file_data_buf,file_data_temp-file_data_buf);
			}
			else
			{
				break;
			}
			
			ret =sscanf(buf,"%s %u.%u %s %s %s %s %s %s\n",
			                       table1,&time0,&time1,table2,event,type,code,value,rate);
			if (ret > 0 && tp_getevent_line_num<getevent_dev->para_num)
			{
					tp_getevent_para[tp_getevent_line_num].time = time0;
					memcpy(tp_getevent_para[tp_getevent_line_num].event, event, sizeof(event));
					memcpy(tp_getevent_para[tp_getevent_line_num].type, type, sizeof(type));
					memcpy(tp_getevent_para[tp_getevent_line_num].code, code, sizeof(code));
					memcpy(tp_getevent_para[tp_getevent_line_num].value, value, sizeof(value));
					memcpy(tp_getevent_para[tp_getevent_line_num].rate, rate, sizeof(rate));
					tp_getevent_line_num++;
			}
			else
			{
				getevent_log_err("resolve error, ret = %d===,tp_getevent_line_num=%d \n",ret,tp_getevent_line_num);
				goto resolve_err;
			}
		}
		else 
		{
			getevent_log_err("strchr error\n");
			break;
		}
	}
	
	*para_table = tp_getevent_para;
	
	return 0;

resolve_err:
	kfree(tp_getevent_para);
kalloc_err:
	para_table = NULL;
	getevent_dev->para_num = 0 ;
	return ret;
}

/**
 * getevent_virtual_keys_show - show virtual key codes
 * @kobj: the kobject decirbe a struct.
 * @attr: the attribute of kobject.
 * @buf: the buf which can stroged key codes.
 *
 * This function set virtual key codes ,userdata can read it to get virtual key codes .
 *
 * If key_value is not empty,then key codes will be storged to buf.
 */
static ssize_t getevent_virtual_keys_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int index = 0,i = 0;
	struct getevent_virtual_keys_data *virtual_keys_pdata = NULL;
	
	virtual_keys_pdata = container_of(attr,struct getevent_virtual_keys_data, kobj_attr);

	for (i = 0; i < GETEVENT_VIRTUAL_KEY_SIZE; i += 5)
	{
		index += scnprintf(buf + index, PAGE_SIZE - index,
			"0x01:%d:%d:%d:%d:%d\n",
			virtual_keys_pdata->key_value[i], virtual_keys_pdata->key_value[i+1], 
			virtual_keys_pdata->key_value[i+2], virtual_keys_pdata->key_value[i+3], 
			virtual_keys_pdata->key_value[i+4]);
	}
	return index;
}

/**
 * getevent_init_virtual_key - init virtual key function
 * @inp_dev_name: the inp_dev_name with an absolute path.
 * @getevent_virtual_pdata: the struct of virtual key.
 *
 * This function open virtual key file and read key codes.
 *
 * If set the virtual key function syuccess,zero will be returned.
 */
static int getevent_init_virtual_key(const char *inp_dev_name, struct getevent_virtual_keys_data *getevent_virtual_pdata)
{
	int rc = 0;
	char *name = NULL;
	char *file_data_temp = NULL;
	char *file_data_buf = NULL;
	char buf[HW_TP_GETEVENT_LINE_MAX] = {0};
	
	getevent_log_dbg("%s:line = %d\n",__func__,__LINE__);
	
	if((virtual_key_kobject_ts == NULL) ||(NULL == getevent_virtual_pdata->file_data))
	{
		rc = -EINVAL;
		goto file_data_err;
	}
	
	file_data_buf = getevent_virtual_pdata->file_data;
	
	name = kzalloc(GETEVENT_VN_MAX_LENGTH, GFP_KERNEL);
	if (name == NULL) {
		getevent_log_dbg("%s:malloc failed\n",__func__);
		rc = -ENOMEM;
		goto fail_alloc_data;
	}
	snprintf(name, GETEVENT_VN_MAX_LENGTH, "virtualkeys.%s", inp_dev_name);
	if(NULL != (file_data_buf = strstr(file_data_buf,"[getevent_common,virtual_keys]")))
	{
		
		for(rc = 0 ;rc <KEY_NAME_LEN ;rc++ )
		{
			if(NULL != (file_data_buf = strstr(file_data_buf,key_names[rc])))
			{
				file_data_buf += strlen(key_names[rc]) + 1;
				if(NULL !=(file_data_temp = strchr(file_data_buf,'\n')))//find the line end
				{	
					memcpy(buf,file_data_buf,file_data_temp-file_data_buf);
				}
				else
				{
					goto find_the_section_err;
				}
				
				sscanf(buf, "%d %d %d %d %d",
				&getevent_virtual_pdata->key_value[5*rc],&getevent_virtual_pdata->key_value[5*rc+1],
				&getevent_virtual_pdata->key_value[5*rc+2],&getevent_virtual_pdata->key_value[5*rc+3],
				&getevent_virtual_pdata->key_value[5*rc+4]);
			}
			else
			{
				getevent_log_dbg("%s:virtual key file data format error\n",__func__);
				goto find_the_section_err;
			}
		}
	}
	else
	{
		getevent_log_dbg("%s:virtual key file data format error\n",__func__);
		goto find_the_section_err;
	}
	
	for(rc = 0;rc < GETEVENT_VIRTUAL_KEY_SIZE; rc++)
		getevent_log_dbg("key_value[%d] = %d\n",rc,getevent_virtual_pdata->key_value[rc]); 
	
	/* Initialize SysFs attribute */
	sysfs_attr_init(&getevent_virtual_pdata->kobj_attr.attr);
	getevent_virtual_pdata->kobj_attr.attr.name = name;
	getevent_virtual_pdata->kobj_attr.attr.mode = S_IRUGO;
	getevent_virtual_pdata->kobj_attr.show     = getevent_virtual_keys_show;
	rc = sysfs_create_file(virtual_key_kobject_ts, &getevent_virtual_pdata->kobj_attr.attr);
	if (rc)
	{
		getevent_log_dbg("%s:sysfs_create_file failed\n",__func__);
		rc = -EINVAL;
		goto fail_del_kobj;
	}
	 
	return 0;

fail_del_kobj:
	getevent_virtual_pdata->kobj_attr.attr.name = NULL;
find_the_section_err:
	kfree(name);
fail_alloc_data:
file_data_err:
	return rc;
}

/**
 * getevent_init_virtual_delay_work - init virtual key delay function
 * @work: the work of virtual key.
 *
 * This function of virtual delay work.
 *
 */
static void getevent_init_virtual_delay_work(struct work_struct *work)
{
	int rc = 0;
	struct getevent_virtual_keys_data *getevent_virtual_pdata = NULL;

	getevent_log_dbg("%s:line=%d\n",__func__,__LINE__);
	
	getevent_virtual_pdata = container_of(work,struct getevent_virtual_keys_data, delay_work.work);
	
	/*init virtual keys */
	rc = getevent_handle_of_file(HW_TP_GETEVENT_INI, &getevent_virtual_pdata->file_data, &getevent_virtual_pdata->file_size);
	if(rc)
	{
		goto getevent_handle_file_err;
	}
	rc = getevent_init_virtual_key(GETEVENT_COMMON_REPORT_NAME,getevent_virtual_pdata);
	if(rc)
	{
		goto init_virtual_key_fail;
	}
	getevent_log_dbg("%s:line=%d\n",__func__,__LINE__);
	return;
	
init_virtual_key_fail:
	kfree(getevent_virtual_pdata->file_data);
getevent_handle_file_err:
	kfree(getevent_virtual_pdata);	
	return;
}

/**
 * getevent_register_virtual_key_device - register virtual key device
 * @getevent_virtual_pdata: the struct of virtual key device.
 *
 * This function register virtual key device and start a work to delay time.
 *
 * If register the virtual key function success,zero will be returned.
 */
static void getevent_register_virtual_key_device(struct getevent_virtual_keys_data *getevent_virtual_pdata)
{
	getevent_log_dbg("%s:line=%d\n",__func__,__LINE__);
	
	if(getevent_virtual_pdata)
	{
		getevent_log_dbg("%s:getevent_virtual_pdata is null\n",__func__);
		goto getevent_virtual_pdata_fail;
	}
	
	getevent_virtual_pdata = kzalloc(sizeof(*getevent_virtual_pdata), GFP_KERNEL);
	if (getevent_virtual_pdata == NULL) {
		getevent_log_dbg("%s:malloc getevent_virtual_pdata is failed\n",__func__);
		goto kzalloc_virtual_fail;
	}
	
	INIT_DELAYED_WORK(&getevent_virtual_pdata->delay_work,getevent_init_virtual_delay_work);
	schedule_delayed_work(&getevent_virtual_pdata->delay_work, msecs_to_jiffies(3000));

	getevent_log_dbg("%s:line=%d\n",__func__,__LINE__);
getevent_virtual_pdata_fail:
kzalloc_virtual_fail:
	
	return;
}

/**
 * getevent_del_virtual_key - del virtual key device
 * @virtual_keys_pdata: the struct of virtual key device.
 *
 * This function del virtual key device.
 *
 */
static void getevent_del_virtual_key(struct getevent_virtual_keys_data *virtual_keys_pdata)
{
	kfree(virtual_keys_pdata->kobj_attr.attr.name);
	virtual_keys_pdata->kobj_attr.attr.name = NULL;
	virtual_keys_pdata->kobj_attr.show = NULL;
	kfree(virtual_keys_pdata->file_data);
	kfree(virtual_keys_pdata);
}

/**
 * getevent_comnon_replay_show - show use the play back section
 * @virtual_keys_pdata: the struct of virtual key device.
 *
 * This function show use the play back section.
 *
 */
static ssize_t getevent_comnon_replay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "please input:\necho rate=(0-5000) agreement=(a,b) x_max=(int) y_max=(int)> getevent_common_report\n");
}

/**
 * touch_abs_replay -report input abs events
 * @getevent_dev: the struct of play back.
 * @abs_code: report abs code.
 * @abs_value: report abs value.
 *
 * This function report input abs.
 *
 */
static int touch_abs_replay(struct getevent_dev *getevent_dev,char *abs_code,char *abs_value)
{
	unsigned int code = 0;
	unsigned int  value = 0;
			
	if(getevent_dev->input_dev == NULL)
	{
		return -EINVAL;
	}
	
	value = atoi(abs_value);
	
	if (value < 0) {
		return -EINVAL;
	}		
	if(!memcmp(abs_code,"ABS_MT_POSITION_X",strlen(abs_code)))
	{
		code = ABS_MT_POSITION_X;
	}
	else if(!memcmp(abs_code,"ABS_MT_POSITION_Y",strlen(abs_code)))
	{
		code = ABS_MT_POSITION_Y;
	}
	else if(!memcmp(abs_code,"ABS_MT_PRESSURE",strlen(abs_code)))
	{
		code = ABS_MT_PRESSURE;
	}
	else if(!memcmp(abs_code,"ABS_MT_TOUCH_MAJOR",strlen(abs_code)))
	{
		code = ABS_MT_TOUCH_MAJOR;
	}
	else if(!memcmp(abs_code,"ABS_MT_TOUCH_MINOR",strlen(abs_code)))
	{
		code = ABS_MT_TOUCH_MINOR;
	}
	else if(!memcmp(abs_code,"ABS_MT_WIDTH_MAJOR",strlen(abs_code)))
	{
		code = ABS_MT_WIDTH_MAJOR;
	}
	else if(!memcmp(abs_code,"ABS_MT_TRACKING_ID",strlen(abs_code)))
	{
		if(0 == strcasecmp(getevent_dev->agreement,"b"))
		{
			if(value == 0xFFFFFFFF)
				input_mt_report_slot_state(getevent_dev->input_dev,MT_TOOL_FINGER, false);
			else
				input_mt_report_slot_state(getevent_dev->input_dev,MT_TOOL_FINGER, true);
			return 0;
		}
		else
		{
			code = ABS_MT_TRACKING_ID;
		}
	}
	else if(!memcmp(abs_code,"ABS_MT_SLOT",strlen(abs_code)))
	{
		code = ABS_MT_SLOT;
	}
	else
	{
		getevent_log_err("%s:line %d,Unknow abs code = %s \n",__func__, __LINE__,abs_code);
		/*in case of prevent some unknow abs code report to system*/
		return 0;
	}
	
	input_report_abs(getevent_dev->input_dev,code, (int)value);
	
	return 0;
}

/**
 * getevent_report_function -handle input report  events
 * @i: the line number.
 * @getevent_dev: the struct of play back.
 *
 * This function handle input report  events.
 *
 */
int  getevent_report_function(unsigned int i,struct getevent_dev *getevent_dev)
{
	int ret = 0;
	struct getevent_info * event_para = getevent_dev->getevent_para;
	
	if (!memcmp(event_para[i].type,"EV_ABS",strlen(event_para[i].type)))
	{
		ret = touch_abs_replay(getevent_dev,event_para[i].code,event_para[i].value);
		if (ret < 0) 
		{
			getevent_log_err("%s:line %d,ret = %d,i = %d\n",__func__, __LINE__,ret,i);
			return -ENOANO;
		}		
	}
	else if (!memcmp(event_para[i].type,"EV_KEY",strlen(event_para[i].type)))
	{
		if(!memcmp(event_para[i].value,"DOWN",strlen(event_para[i].value)))
		{
			input_report_key(getevent_dev->input_dev,BTN_TOUCH, true);
		}
		else if(!memcmp(event_para[i].value,"UP",strlen(event_para[i].value)))
		{
			input_report_key(getevent_dev->input_dev,BTN_TOUCH, false);
		}
		else
		{
			getevent_log_err("%s:line %d,Unknow key value = %s\n",__func__, __LINE__,event_para[i].value);
			return -ENOANO;
		}
	}
	else if (!memcmp(event_para[i].type,"EV_SYN",strlen(event_para[i].type)))
	{
		if(!memcmp(event_para[i].code,"SYN_MT_REPORT",strlen(event_para[i].code)))
		{
			input_mt_sync(getevent_dev->input_dev);
		}
		else if(!memcmp(event_para[i].code,"SYN_REPORT",strlen(event_para[i].code)))
		{	
			input_sync(getevent_dev->input_dev);
		}
		else
		{
			getevent_log_err("%s:line %d,Unknow EV_SYN code =%s\n",__func__, __LINE__,event_para[i].code);
			return -ENOANO;
		}
	}
	usleep(getevent_dev->rate);

	return 0;
}

/**
 * touch_event_replay -while to report event replay
 * @getevent_para: the struct of getevent info.
 * @getevent_dev: the struct of play back.
 *
 * This function add a BTN_TOUCH at first in order to there is not have BTN_TOUCH of input log in first line,
 * then while to report event replay.
 */
static int touch_event_replay(struct getevent_info * getevent_para,struct getevent_dev *getevent_dev)
{
	unsigned int i = 0;
	
	getevent_log_dbg("%s:in!\n",__func__);
	if(!getevent_dev)
	{
		return -ENOANO;
	}
	
	getevent_log_dbg("%s:line %d,para_num = %d\n",__func__, __LINE__,getevent_dev->para_num);
	getevent_dev->getevent_para = getevent_para;
	
	/*add BTN_TOUCH if is not Down first*/
	/*del add BTN_TOUCH at first*/
	
	for(i = 0; i < getevent_dev->para_num; i++)
	{	
		if((i+1) == getevent_dev->para_num)//the last line
		{
			getevent_report_function(i,getevent_dev);
		}
		else if((getevent_para[i+1].time - getevent_para[i].time) < 2)//the time in 2 secs
		{
			getevent_report_function(i,getevent_dev);
		}
		else//over 2 sec,the next action
		{
			//kernel thresold wait 2 secs then report
			getevent_log_dbg("%s:wait to 2 secs to continue\n",__func__);
			getevent_report_function(i,getevent_dev);
			msleep(2000);
		}
	}
	/*del add BTN_TOUCH at end*/
	
	getevent_log_dbg("%s:out!\n",__func__);
	return 0;
}

/**
 * getevent_common_replay_store -to replay input event interface
 * @dev: the device.
 * @device_attribute: the attribute of device.
 * @buf: the attribute of device.
 * @device_attribute: the attribute of device.
 *
 * This function is the first interface which user to replay input event using sysfs.
 */
static ssize_t getevent_common_replay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	unsigned int rate=0,x_max=0,y_max=0;
	unsigned char agreement[20] = {0,0,0};
	struct getevent_info *getevent_para = NULL;
	struct getevent_dev *getevent_dev = NULL;
	
	mutex_lock(&getevent_device_mutex);
	
	getevent_log_dbg("%s: begin\n",__func__);
	
	if (sscanf(buf, "rate=%d agreement=%s x_max=%d y_max=%d", &rate,agreement,&x_max,&y_max) != GETEVENT_SIZEOF_SSCANF)
	{
		getevent_log_err("%s:getevent of sscanf err\n",__func__);	
		ret = -EINVAL;
		goto sscanf_err;
	}
	getevent_log_dbg("rate=%d agreement=%s x_max=%d y_max=%d",rate,agreement,x_max,y_max);
	
	/*malloc getevent_dev struct*/
	if(NULL == getevent_dev)
	{
		getevent_dev = kmalloc(sizeof(*getevent_dev), GFP_KERNEL);
		if(NULL == getevent_dev)
		{
			getevent_log_err("%s:kmalloc faild\n",__func__);
			ret = -ENOMEM;
			goto kmalloc_getevent_dev_err;
		}
		else 
		{
			getevent_log_dbg("%s:kmalloc success\n",__func__);
		}
	}
	
	/*init getevent_dev values*/
	memcpy(getevent_dev->agreement,agreement,strlen(agreement));
	getevent_dev->rate   = rate;
	getevent_dev->para_num = 0;
	getevent_dev->file_size = 0;
	getevent_dev->count = 0;
	getevent_dev->x_max = x_max;
	getevent_dev->y_max = y_max;
	getevent_dev->file_data = NULL;
	getevent_dev->input_dev = NULL;
	getevent_dev->getevent_para = NULL;
	
	/*alloc input_dev*/
	ret = getevent_common_set_input_dev(getevent_dev);
	if(ret)
	{	
		getevent_log_err("%s:error:getevent_common_set_input_dev fail\n",__func__);
		goto getevent_common_set_input_dev_err;
	}

	/*handle getevent file*/
	ret = getevent_handle_of_file(HW_TP_GETEVENT_PARAM,&getevent_dev->file_data,&getevent_dev->file_size);
	if(ret < 0)
	{
		getevent_log_err("%s:error:getevent_handle_of_file fail\n",__func__);
		ret = -EIO;
		goto getevent_handle_err;
	}
	
	/*resolve event log*/
	ret = getevent_resolve_event_log((void**)&getevent_para,getevent_dev);
	if(ret)
	{
		getevent_log_err("%s: Failed to getevent_resolve_event_log, ret=%d\n",__func__,ret);
		goto resolve_event_log_err;
	}
	
	getevent_log_dbg("%s: line = %d\n",__func__,__LINE__);
	if( !ret && (NULL != getevent_para))
	{
		getevent_log_dbg("%s: line %d, ret=%d\n",__func__,__LINE__,ret);
		ret = touch_event_replay(getevent_para,getevent_dev);
		if (ret) {
			getevent_log_err("%s: Failed to replay tp event, ret=%d\n",__func__,ret);
			goto touch_event_replay_err;
		} 
	}
		
	getevent_log_dbg("%s: end\n",__func__);
	
touch_event_replay_err:
	kfree(getevent_para);
resolve_event_log_err:
	kfree(getevent_dev->file_data);
getevent_handle_err:
	input_unregister_device(getevent_dev->input_dev);
	input_free_device(getevent_dev->input_dev);
getevent_common_set_input_dev_err:	
	kfree(getevent_dev);
kmalloc_getevent_dev_err:
sscanf_err:
	mutex_unlock(&getevent_device_mutex);
	//count = GETEVENT_SIZEOF_SSCANF;
	return count;
}

static DEVICE_ATTR(getevent_common_report, S_IRUGO|S_IWUSR, getevent_comnon_replay_show, getevent_common_replay_store);

static int __init init_getevent_sysfs(void)
{
	int rc = 0;
	getevent_log_dbg("%s:line=%d\n",__func__,__LINE__);
	
	if(getevent_common_kobject == NULL)
	{
		getevent_common_kobject = tp_get_touch_screen_obj();
		if (!getevent_common_kobject)
		{
			rc =  -EIO;
			goto getevent_kobject_err;
		}
	}
	rc = sysfs_create_file(getevent_common_kobject, &dev_attr_getevent_common_report.attr);
	if (rc)
	{
		getevent_log_dbg("%s:syscreat getevent_common_report failed\n",__func__);
		goto sys_creat_file_err;
	}
	
	getevent_register_virtual_key_device(getevent_virtual_pdata);
	
	mutex_init(&getevent_device_mutex);
	
	getevent_log_dbg("%s:line=%d\n",__func__,__LINE__);
	return 0;
	
sys_creat_file_err:
getevent_kobject_err:
	return rc;
}

static void __exit exit_getevent_sysfs(void)
{
	if(getevent_virtual_pdata)
	{
		getevent_del_virtual_key(getevent_virtual_pdata);
	}
	
	if(getevent_common_kobject)
	{
		sysfs_remove_file(getevent_common_kobject,&dev_attr_getevent_common_report.attr);
	}
	
	mutex_destroy(&getevent_device_mutex);
	return;
}

module_init(init_getevent_sysfs);
module_exit(exit_getevent_sysfs);
MODULE_AUTHOR("Weiqiangqiang, huawei.");
MODULE_DESCRIPTION("Getevent_common_report for huawei");
MODULE_LICENSE("GPL v1");
