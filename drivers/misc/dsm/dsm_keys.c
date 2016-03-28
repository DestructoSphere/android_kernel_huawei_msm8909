#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/dsm_pub.h>

#define DSM_KEYS_CLIENT "dsm_keys"
#define DSM_KEYS_BUF_SIZE 512

static struct platform_device *dsm_keys_pdev;
static struct dsm_client *dsm_keys_dclient = NULL;

#define PRESS_KEY_INTERVAL	(85)   //the minimum press interval
#define PRESS_HALL_INTERVAL (55)   //the minimum hall interval
#define STATISTIC_INTERVAL	(60) 	//the statistic interval for key event
#define MAX_PRESS_KEY_COUNT	(120)   //the default press count for a normal use

struct keys_paras{
	unsigned int power_key_cnt;
	unsigned int power_inv;
	unsigned int vol_up_cnt;
	unsigned int vol_up_inv;
	unsigned int vol_down_cnt;
	unsigned int vol_down_inv;
	unsigned int hall_irq_cnt;
	unsigned int hall_irq_inv;
	unsigned long power_key_last_pressed;
	unsigned long vol_up_last_pressed;
	unsigned long vol_down_last_pressed;
	unsigned long hall_irq_last;
};

struct dsm_keys_drv_data {
	struct timer_list dsm_key_timer; //used to reset the statistic variable
	struct keys_paras keys_data;
	struct mutex excep_mutex;
	struct work_struct excep_work;
	int type;
	unsigned long cur_jiffies;
};

struct dsm_keys_drv_data *dsm_key_data = NULL;


/* dsm client for keys */
static struct dsm_dev dsm_keys_device = {
	.name 		= CLIENT_DSM_KEY,				// dsm client name
	.fops 		= NULL,						// options
	.buff_size 	= DSM_KEYS_BUF_SIZE,		// buffer size
};



static void dsm_report_detailed_info(char *fmt, ...)
{
	char buf[DSM_KEYS_BUF_SIZE];
	va_list ap;

	memset(buf, 0, DSM_KEYS_BUF_SIZE);
	va_start(ap, fmt);
	vsnprintf(buf, (sizeof(buf)-1), fmt, ap);
	va_end(ap);

	dsm_client_record(dsm_keys_dclient,"%s", buf);
	printk("%s", buf);
}


static void dsm_keys_report(int errno,int type)
{
	struct keys_paras* keys_data = &dsm_key_data->keys_data;

	if (dsm_client_ocuppy(dsm_keys_dclient)) {
		printk("[dsm_key] ocuppy buffer failed. errno = %d\n", errno);
		return ;
	}

	switch (type) {
		case DSM_VOL_KEY:
			if (DSM_VOL_KEY_PRESS_TIMES_ERR == errno) {
				dsm_report_detailed_info("vol err,vol_up times = %d, vol_down times = %d\n",
										 keys_data->vol_up_cnt, keys_data->vol_down_cnt);
			} else if (DSM_VOL_KEY_PRESS_INTERVAL_ERR == errno) {
				dsm_report_detailed_info("vol err, up interval = %u, down interval = %u  ms, <%d (ms).\n",
										 keys_data->vol_up_inv, keys_data->vol_down_inv, PRESS_KEY_INTERVAL);
			} else {
				pr_err("%s@%d unsupport errno = %d\n", __func__, __LINE__, errno);
			}
			break;

		case DSM_POW_KEY:
			if (DSM_POWER_KEY_PRESS_INTERVAL_ERR == errno) {
				dsm_report_detailed_info("powerkey err, pressed interval = %u   < %d (ms).\n",
										 keys_data->power_inv, PRESS_KEY_INTERVAL);
			} else if (DSM_POWER_KEY_PRESS_TIMES_ERR == errno) {
				dsm_report_detailed_info("powerkey err, pressed times = %d \n",
										 keys_data->power_key_cnt);
			} else {
				pr_err("%s@%d unsupport errno = %d\n", __func__, __LINE__, errno);
			}
			break;

		case DSM_HALL_IRQ:
			if (DSM_HS_IRQ_INTERVAL_ERR == errno) {
				dsm_report_detailed_info("hall sensor err, irq interval = %u   < %d(ms).\n",
										 keys_data->hall_irq_inv, PRESS_KEY_INTERVAL);
			} else if (DSM_HS_IRQ_TIMES_ERR == errno) {
				dsm_report_detailed_info("hall sensor err, irq times = %d \n",
										 keys_data->hall_irq_cnt);
			} else {
				pr_err("%s@%d unsupport errno = %d\n", __func__, __LINE__, errno);
			}
			break;

		default:
			printk("%s unsupported error type = %d.\n", __func__, type);
			break;
	}

	dsm_client_notify(dsm_keys_dclient, errno);


}


/*****************************************************************
Parameters    :  data (timer private data )
Return        :
Description   :		every 60 seconds check if the key is pressed more than 120 times or not
*****************************************************************/
static void keytimes_timer_func(unsigned long data)
{

	struct dsm_keys_drv_data *ddata = (struct dsm_keys_drv_data *)data;
	struct keys_paras *keys_paras = &ddata->keys_data;

	if(keys_paras->power_key_cnt > MAX_PRESS_KEY_COUNT){
		dsm_keys_report(DSM_POWER_KEY_PRESS_TIMES_ERR, DSM_POW_KEY);
	}

	if(keys_paras->vol_up_cnt > MAX_PRESS_KEY_COUNT
		|| keys_paras->vol_down_cnt > MAX_PRESS_KEY_COUNT ){
		dsm_keys_report(DSM_VOL_KEY_PRESS_INTERVAL_ERR, DSM_VOL_KEY);
	}


	if(keys_paras->hall_irq_cnt > MAX_PRESS_KEY_COUNT){
		dsm_keys_report(DSM_HS_IRQ_TIMES_ERR, DSM_HALL_IRQ);
	}

	/* reset the statistic variable */
	keys_paras->power_key_cnt 	= 0;
	keys_paras->vol_up_cnt 		= 0;
	keys_paras->vol_down_cnt 	= 0;
	keys_paras->hall_irq_cnt	= 0;

	mod_timer(&ddata->dsm_key_timer, jiffies + STATISTIC_INTERVAL * HZ);

}

static int inline calc_interval(unsigned long* last_jiffies, unsigned long cur_jiffies)
{
	unsigned int tmp_inv = jiffies_to_msecs(abs(cur_jiffies - *last_jiffies));
	*last_jiffies = cur_jiffies;
	return tmp_inv;
}

static void dsm_keys_excep_work_func(struct work_struct *work)
{
	struct dsm_keys_drv_data *ddata = container_of(work, struct dsm_keys_drv_data, excep_work);
	struct keys_paras *pkeys_data = &ddata->keys_data;

	unsigned long tmp_jiffies = ddata->cur_jiffies;
	int type = ddata->type;
	unsigned long last_pressd_time;
	unsigned long min_inv = msecs_to_jiffies(PRESS_KEY_INTERVAL);
	unsigned long min_hall =msecs_to_jiffies(PRESS_HALL_INTERVAL);

	switch (type) {
		case DSM_VOL_UP_KEY:
			last_pressd_time = pkeys_data->vol_up_last_pressed;
			pkeys_data->vol_up_inv =
					calc_interval(&pkeys_data->vol_up_last_pressed, tmp_jiffies);
			if ((time_after(last_pressd_time + min_inv, tmp_jiffies))&&(last_pressd_time!=0)) {
				dsm_keys_report(DSM_VOL_KEY_PRESS_INTERVAL_ERR,DSM_VOL_KEY);
				printk("DSM_VOL_UP_KEY err! interval time = %d\n", jiffies_to_msecs(tmp_jiffies -last_pressd_time));
			}
			break;

		case DSM_VOL_DOWN_KEY:
			last_pressd_time = pkeys_data->vol_down_last_pressed;
			pkeys_data->vol_down_inv =
					calc_interval(&pkeys_data->vol_down_last_pressed, tmp_jiffies);
			if ((time_after(last_pressd_time + min_inv, tmp_jiffies))&&(last_pressd_time!=0)) {
				dsm_keys_report(DSM_VOL_KEY_PRESS_INTERVAL_ERR,DSM_VOL_KEY);
				printk("DSM_VOL_DOWN_KEY err! interval time = %d\n", jiffies_to_msecs(tmp_jiffies -last_pressd_time));
			}
			break;

		case DSM_POW_KEY:
			last_pressd_time = pkeys_data->power_key_last_pressed;
			pkeys_data->power_inv =
				calc_interval(&pkeys_data->power_key_last_pressed, tmp_jiffies);
			if ((time_after(last_pressd_time + min_inv, tmp_jiffies))&&(last_pressd_time!=0)) {
				dsm_keys_report(DSM_POWER_KEY_PRESS_INTERVAL_ERR,DSM_POW_KEY);
				printk("DSM_POW_KEY err! interval time = %d\n", jiffies_to_msecs(tmp_jiffies -last_pressd_time));
			}

			break;

		case DSM_HALL_IRQ:
			last_pressd_time = pkeys_data->hall_irq_last;
			pkeys_data->hall_irq_inv =
				calc_interval(&pkeys_data->hall_irq_last, tmp_jiffies);
			if ((time_after(last_pressd_time + min_hall, tmp_jiffies))&&(last_pressd_time!=0)) {
				dsm_keys_report(DSM_HS_IRQ_INTERVAL_ERR,DSM_HALL_IRQ);
				printk("DSM_HALL_IRQ err! interval time = %d\n", jiffies_to_msecs(tmp_jiffies -last_pressd_time));
			}


			break;
		default:
				printk("[dsm_key]%s unsupport type.\n", __func__);
			break;

		}


}


void dsm_key_pressed(int type)
{
	switch (type) {
		case DSM_VOL_UP_KEY:
			dsm_key_data->keys_data.vol_up_cnt ++;
			break;

		case DSM_VOL_DOWN_KEY:
			dsm_key_data->keys_data.vol_down_cnt ++;
			break;

		case DSM_POW_KEY:
			dsm_key_data->keys_data.power_key_cnt ++;
			break;

		case DSM_HALL_IRQ:
			dsm_key_data->keys_data.hall_irq_cnt++;
			break;

		default:
			printk("[dsm_keys]%s unsupport key type", __func__);
			return;
	}

	dsm_key_data->type = type;
	dsm_key_data->cur_jiffies = jiffies;
	schedule_work(&dsm_key_data->excep_work);

}
EXPORT_SYMBOL(dsm_key_pressed);



static int hw_dsm_keys_probe(struct platform_device *pdev)
{
	int ret = 0;

    dsm_keys_dclient = dsm_register_client(&dsm_keys_device);
    if (!dsm_keys_dclient) {
    	pr_err("[dsm_keys]register dsm_sensor_service_client failed!\n");
    	ret = -ENOMEM;
    	goto out;
    }

	dsm_key_data = kzalloc(sizeof(struct dsm_keys_drv_data), GFP_KERNEL);
	if(!dsm_key_data){
    	pr_err("[dsm_keys]alloc dsm_keys_drv_data failed!\n");
    	ret = -ENOMEM;
    	goto free_dsm_keys;
	}

	mutex_init(&dsm_key_data->excep_mutex);

	setup_timer(&dsm_key_data->dsm_key_timer, keytimes_timer_func, (unsigned long)dsm_key_data);

	platform_set_drvdata(pdev, dsm_key_data);

	INIT_WORK(&dsm_key_data->excep_work, dsm_keys_excep_work_func);

	printk("register dsm_keys successfully.\n");

	return 0;

free_dsm_keys:
	dsm_unregister_client(dsm_keys_dclient,&dsm_keys_device);
out:
	return ret;
}


static int hw_dsm_keys_remove(struct platform_device *pdev)
{
	dsm_unregister_client(dsm_keys_dclient,&dsm_keys_device);

	return 0;
}

static int dsm_keys_suspend(struct device *dev)
{
	struct dsm_keys_drv_data *ddata = dev_get_drvdata(dev);


	del_timer_sync(&ddata->dsm_key_timer);
	cancel_work_sync(&ddata->excep_work);

	return 0;
}

static int dsm_keys_resume(struct device *dev)
{
	struct dsm_keys_drv_data *ddata = dev_get_drvdata(dev);

	mod_timer(&ddata->dsm_key_timer, jiffies + STATISTIC_INTERVAL * HZ);

	return 0;
}

static SIMPLE_DEV_PM_OPS(dsm_keys_pm_ops, dsm_keys_suspend, dsm_keys_resume);

static struct platform_driver dsm_keys_driver = {
	.probe = hw_dsm_keys_probe,
	.remove = hw_dsm_keys_remove,
	.driver = {
		.name = DSM_KEYS_CLIENT,
		.pm = &dsm_keys_pm_ops,
		.owner	= THIS_MODULE,
	},
};


static int __init hw_dsm_keys_init(void)
{
	int err;
	err = platform_driver_register(&dsm_keys_driver);
	if (unlikely(err < 0)){
		goto out;
	}

	dsm_keys_pdev = platform_device_register_simple(DSM_KEYS_CLIENT, -1,
					     NULL, 0);
	if (IS_ERR(dsm_keys_pdev)) {
		platform_driver_unregister(&dsm_keys_driver);
		printk("load sensor_service_client driver failed.\n");
		err = PTR_ERR(dsm_keys_pdev);
		goto out;
	}

	return 0;

out:
	return err;
}


static void __exit hw_dsm_keys_exit(void)
{
	platform_device_unregister(dsm_keys_pdev);
	platform_driver_unregister(&dsm_keys_driver);
}


MODULE_DESCRIPTION("huawei dsm keys client driver");
MODULE_LICENSE("GPL");
module_init(hw_dsm_keys_init);
module_exit(hw_dsm_keys_exit);
