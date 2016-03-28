// Sine so many modification made, we just add DTS number in the begging
/*

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License version 2 as
   published by the Free Software Foundation.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

   This program is distributed in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
   or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
   for more details.


   Copyright (C) 2011-2012  Huawei Corporation
*/

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/notifier.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>

#include <linux/irq.h>
#include <linux/param.h>
#include <linux/termios.h>

#include <net/bluetooth/bluetooth.h>
#include <asm/uaccess.h>
#include <linux/of.h>
#include <asm/uaccess.h>
//#include <media/v4l2-dev.h>
//#include <media/radio-iris.h>
//#include <hsad/config_interface.h>

#ifndef KERNEL_HWFLOW
#define KERNEL_HWFLOW true
#endif

#undef FMDBG
#define FMDBG(fmt, args...) \
	do \
	{ \
		  if (KERNEL_HWFLOW) \
			pr_info("bt_fm_feature: " fmt, ##args); \
	} while (0)
#undef FMDERR
#define FMDERR(fmt, args...) pr_err("bt_fm_feature: " fmt, ##args)


#define BT_DEVICE_DBG
#ifndef BT_DEVICE_DBG
#define BT_DBG(fmt, arg...)
#endif

#define LOG_TAG "FeatureTransfer"

/*
 * Defines
 */
#define VERSION		"1.0"
#define PROC_DIR	"device_feature"
#define BT_FIRMWARE_UNKOWN	"Unkown"
#define BT_POWER_DEFAULT	"default"

struct proc_dir_entry *device_dir, *bt_dir, *fm_dir;
typedef enum
{
	FM_SINR_5 = 5,
	FM_SINR_6 = 6,
	FM_SINR_7 = 7,
	FM_SINR_8 = 8,
	FM_SINR_MAX = 0xff
}fm_sinr_value;

typedef enum 
{
	FM_SINR_SAMPLES_9 = 9,
	FM_SINR_SAMPLES_10 = 10,
	FM_SINR_SAMPLES_MAX = 0xff
}fm_sinr_samples;

static int g_sinr_threshold = FM_SINR_MAX;
static int g_sinr_samples = FM_SINR_SAMPLES_MAX;
struct bt_device
{
	char *chip_type;
	char *dev_name;
};

const struct bt_device bt_device_array[] =
{
	{ "BT_FM_BROADCOM_BCM4330", "1.2" },
	{ "BT_FM_BROADCOM_BCM4334", "1.2" },
	{ "BT_FM_QUALCOMM_WCN3660", "2.2" },
	{ "BT_FM_QUALCOMM_WCN3620", "2.2" },
	{ "BT_FM_QUALCOMM_WCN3680", "2.2" },
	{ "BT_FM_QUALCOMM_WCN3660B", "2.2" },
	{ "BT_FM_QUALCOMM_WCN3610", "2.2" },
	{ "BT_FM_UNKNOWN_DEVICE", "Unknown" }
};

const void *get_bt_fm_device_type(void)
{
	int chip_type_len;
	struct device_node *dp = NULL;
	dp = of_find_node_by_path("/huawei_bt_info");
	if (!of_device_is_available(dp))
	{
		FMDERR("device is not available!\n");
		return NULL;
	}
	return  of_get_property(dp,"bt,chiptype", &chip_type_len);
}
static char *get_bt_fm_device_name(void)
{  
	int i = 0;
	int arry_size = sizeof(bt_device_array)/sizeof(bt_device_array[0]);
	const char *bt_fm_chip_type = "BT_FM_UNKNOWN_DEVICE";

	/* get bt/fm chip type from the device feature configuration (.xml file) */
	bt_fm_chip_type = get_bt_fm_device_type();
	FMDBG("get_bt_fm_device_name bt_fm_chip_type:%s", bt_fm_chip_type);
	
	if (NULL == bt_fm_chip_type)
	{
		FMDERR("BT-FM, Get chip type fail.\n");
		return bt_device_array[arry_size - 1].dev_name;
	}
	/* lookup bt_device_model in bt_device_array[] */
	for (i = 0; i < arry_size; i++)
	{
		if (0 == strncmp(bt_fm_chip_type,bt_device_array[i].chip_type,strlen(bt_device_array[i].chip_type)))
		{
			break; 
		}
	}
	/* If we get invalid type name, return "Unknown".*/
	if ( i == arry_size)
	{
		FMDERR("BT-FM, Get chip type fail.\n");

		return bt_device_array[arry_size - 1].dev_name;
	}

	return bt_device_array[i].dev_name;
}


static int featuretransfer_remove(struct platform_device *pdev)
{
	FMDBG("devicefeature removed.");
	return 0;
}

static struct platform_driver featuretransfer_driver = {
	.remove = featuretransfer_remove,
	.driver = {
		.name = "featuretransfer",
		.owner = THIS_MODULE,
	},
};


EXPORT_SYMBOL(get_bt_fm_device_name);

/* modify printk to FMDBG or FMDERR */
static int chiptype_proc_show(struct seq_file *m, void *v)
{
	const char *bt_chiptype_name = NULL;

	bt_chiptype_name = get_bt_fm_device_name();
	FMDBG("%s enter chiptype:%s;\n", __func__, bt_chiptype_name);
	seq_printf(m,"%s",bt_chiptype_name);
	return 0;
}

static int chiptype_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, chiptype_proc_show, NULL);
}

static const struct file_operations chiptype_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= chiptype_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


const void *get_bt_fm_fw_ver(void)
{
	int bt_fm_fw_ver_len;
	struct device_node *dp = NULL;
	dp = of_find_node_by_path("/huawei_bt_info");
	if (!dp)
	{
		 FMDERR("device is not available!\n");
		 return NULL;
	}
	else
	{
		 FMDBG("%s:dp->name:%s,dp->full_name:%s;\n",__func__,dp->name,dp->full_name);
	}
	   
	return  of_get_property(dp,"bt,fw_ver", &bt_fm_fw_ver_len);
}

EXPORT_SYMBOL(get_bt_fm_fw_ver);

static int bt_fm_ver_proc_show(struct seq_file *m, void *v)
{
	const char *bt_fm_fw_ver = NULL;

	bt_fm_fw_ver = get_bt_fm_fw_ver();

	if ( NULL != bt_fm_fw_ver )
	{
		FMDBG("%s enter wifi_device_type:%s;\n", __func__,bt_fm_fw_ver);
		seq_printf(m,"%s",bt_fm_fw_ver);
	}
	else
	{
		FMDERR("%s get bt_fm_fw_ver failed;\n", __func__);
	}
	return 0;
}
static int bt_fm_ver_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, bt_fm_ver_proc_show, NULL);
}

static const struct file_operations bt_fm_ver_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= bt_fm_ver_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

const void *get_bt_power_level(void)
{
	int bt_power_level_len;
	struct device_node *dp = NULL;
	dp = of_find_node_by_path("/huawei_bt_info");
	if (!dp)
	{
		 FMDERR("device is not available!\n");
		 return NULL;
	}
	else
	{
		 FMDBG("%s:dp->name:%s,dp->full_name:%s;\n", __func__, dp->name, dp->full_name);
	}
	   
	return  of_get_property(dp,"bt,power_level", &bt_power_level_len);
}

EXPORT_SYMBOL(get_bt_power_level);

static int bt_power_level_proc_show(struct seq_file *m, void *v)
{
	const char *bt_power_level = NULL;

	bt_power_level = get_bt_power_level();

	if (NULL != bt_power_level)
	{
		FMDBG("%s enter bt_power_level:%s;\n", __func__, bt_power_level);
		seq_printf(m, "%s", bt_power_level);
	}
	else
	{
		FMDERR("%s get bt_power_level failed;\n", __func__);
	}
	return 0;
}
static int bt_power_level_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, bt_power_level_proc_show, NULL);
}

static const struct file_operations bt_power_level_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= bt_power_level_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

int get_fm_sinr_threshold_string(int *sinr)
{
	int sinr_threshold_len;
	struct device_node *dp = NULL;
	const char *psinr_threshold = NULL;
	int sinr_threshold;
	
	dp = of_find_node_by_path("/huawei_fm_info");
	if (!dp)
	{
		FMDERR("device is not available!\n");
		return -1;
	}
	
	psinr_threshold = of_get_property(dp,"fm,sinr_threshold", &sinr_threshold_len);
	if (NULL == psinr_threshold)
	{
		FMDERR("get sinr threshold value failed .\n");
		return -1;
	}
	sinr_threshold = simple_strtol(psinr_threshold,NULL,10);
	*sinr = sinr_threshold;
	return 0;
}

EXPORT_SYMBOL(get_fm_sinr_threshold_string);


static int fm_sinr_threshold_proc_show(struct seq_file *m, void *v)
{
	int sinr_threshold = FM_SINR_MAX;
	int ret ;
	
	FMDBG("fm_sinr_threshold_proc_show  enter\n");
	ret = get_fm_sinr_threshold_string(&sinr_threshold);
	if (-1 == ret)
	{
		// 7 is the default value
		FMDERR("Get FM Sinr Threshold failed and will use default value 7.\n");
		sinr_threshold = FM_SINR_7;
	}
	
	FMDBG("fm_sinr_threshold_proc_show g_sinr_threshold = %d .\n",g_sinr_threshold);	
	/* if we changed the sinr value, get the new value. used for debug */
	if (FM_SINR_MAX != g_sinr_threshold)
	{
		sinr_threshold = g_sinr_threshold;
	}

	FMDBG("fm_sinr_threshold_proc_show ,g_sinr_threshold = %d, sinr_threshold:%d\n", g_sinr_threshold, sinr_threshold);

	seq_printf(m, "%d", sinr_threshold);
	return 0;
}

static int fm_sinr_threshold_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fm_sinr_threshold_proc_show, NULL);
}

static ssize_t fm_sinr_threshold_write(struct file *filp, const char __user *userbuf,
						size_t count, loff_t *ppos)
{
	char sinr_threshold[10] = {0};

	FMDBG("fm_sinr_threshold_write.  count:%d\n", (int)count);

	if (NULL == userbuf )
	{
		FMDERR("[%s]: Invlid value.line:%d\n",__FUNCTION__,__LINE__);
		return -EFAULT;
	}

	if (10 < count)
	{
		FMDERR("fm_sinr_threshold_write count value too much!\n");
		return -EFAULT;
	}

	if (copy_from_user(sinr_threshold, userbuf, count))
		return -EFAULT;
 
	g_sinr_threshold = simple_strtol(sinr_threshold, NULL, 10);
	FMDBG("fm_sinr_threshold_write exit  g_sinr_threshold:%d\n", g_sinr_threshold);
	return count;
}

static const struct file_operations fm_sinr_threshold_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= fm_sinr_threshold_proc_open,
	.read	 = seq_read,
	.write	= fm_sinr_threshold_write,
	.llseek   = seq_lseek,
	.release  = single_release,
};


int get_fm_sinr_samples(int *sinr)
{
	int sinr_samples_len;
	struct device_node *dp = NULL;
	const char *psinr_samples = NULL;
	int sinr_samples;
	
	dp = of_find_node_by_path("/huawei_fm_info");
	if (!dp)
	{
		FMDERR("device is not available!\n");
		return -1;
	}
	
	psinr_samples = of_get_property(dp,"fm,sinr_samples", &sinr_samples_len);
	if (NULL == psinr_samples)
	{
		FMDERR("get sinr threshold value failed .\n");
		return -1;
	}
	sinr_samples = simple_strtol(psinr_samples,NULL,10);
	*sinr = sinr_samples;
	return 0;
}

EXPORT_SYMBOL(get_fm_sinr_samples);

static int fm_sinr_samples_proc_show(struct seq_file *m, void *v)
{
	unsigned int sinr_samples = FM_SINR_SAMPLES_MAX;
	int ret;
	
	FMDBG("fm_sinr_samples_proc_show enter.\n");
	ret = get_fm_sinr_samples(&sinr_samples);
	if (-1 == ret)
	{
		FMDERR("Get FM Sinr Samples failed and will use default value 10.\n");
		sinr_samples = FM_SINR_SAMPLES_10;
	}
	
	if (FM_SINR_SAMPLES_MAX != g_sinr_samples)
	{
		sinr_samples = g_sinr_samples;
	}
	
	FMDBG("fm_sinr_samples_proc_show ,g_sinr_samples = %d, sinr_samples:%d\n", g_sinr_samples, sinr_samples);

	seq_printf(m, "%d", sinr_samples);
	return 0;
}

static int fm_sinr_samples_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fm_sinr_samples_proc_show, NULL);
}

static ssize_t fm_sinr_samples_write(struct file *filp, const char __user *userbuf,
						size_t count, loff_t *ppos)
{
	char sinr_samples[10] = {0};

	FMDBG("fm_sinr_samples_write  enter.  count:%d\n", (int)count);


	if (NULL == userbuf)
	{
		FMDERR("[%s]: Invlid value.line:%d\n",__FUNCTION__,__LINE__);
		return -EFAULT;
	}

	if ( 10 < count)
	{
		FMDERR("count value too much.");
		return -EFAULT;
	}

	if (copy_from_user(sinr_samples, userbuf, count))
		return -EFAULT;

	g_sinr_samples = simple_strtol(sinr_samples, NULL, 10);
	FMDBG("fm_sinr_samples_write exit  g_sinr_samples:%d\n", g_sinr_samples);
	return count;
}


static const struct file_operations fm_sinr_samples_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= fm_sinr_samples_proc_open,
	.read		= seq_read,
	.write		= fm_sinr_samples_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/**
 * Initializes the module.
 * @return On success, 0. On error, -1, and <code>errno</code> is set
 * appropriately.
 */
static int __init featuretransfer_init(void)
{
	int retval = 0;
	struct proc_dir_entry *ent = NULL;

	FMDBG("BT DEVICE FEATURE VERSION: %s", VERSION);

	/* Driver Register */
	retval = platform_driver_register(&featuretransfer_driver);
	if (0 != retval)
	{
		FMDERR("[%s],featurntransfer driver register fail.",LOG_TAG);
		return retval;
	}

	/* create device_feature directory for bt chip info */
	device_dir = proc_mkdir("device_feature", NULL);
	if (NULL == device_dir)
	{
		FMDERR("Unable to create /proc/device_feature directory");
		return -ENOMEM;
	}

	/* create bt_feature for bluetooth feature */
	bt_dir = proc_mkdir("bt_feature", device_dir);
	if (NULL == bt_dir)
	{
		FMDERR("Unable to create /proc/%s directory", PROC_DIR);
		return -ENOMEM;
	}

	/* Creating read/write "chiptype" entry for bluetooth chip type*/
	ent = proc_create("chiptype", 0, bt_dir, &chiptype_proc_fops);
	if (NULL == ent) 
	{
		FMDERR("Unable to create /proc/%s/chiptype entry", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}

	/* Creating read/write "bt_fm_fw_ver" entry */
	ent = proc_create("bt_fm_fw_ver", 0, bt_dir, &bt_fm_ver_proc_fops);
	if (NULL == ent) 
	{
		FMDERR("Unable to create /proc/%s/bt_fm_fw_ver entry", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}


	/* Creating read/write "bt_power_level" entry */
	ent = proc_create("power_level", 0, bt_dir, &bt_power_level_proc_fops);
	if (NULL == ent)
	{
		FMDERR("Unable to create /proc/%s/power_level entry", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}

	/* create fm_feature for fm feature */
	fm_dir = proc_mkdir("fm_feature", device_dir);
	if (NULL == fm_dir)
	{
		FMDERR("Unable to create /proc/%s directory", PROC_DIR);
		return -ENOMEM;
	}

	/* Creating read/write "sinr" entry for bluetooth chip type*/
	ent = proc_create("sinr_threshold", 0, fm_dir, &fm_sinr_threshold_proc_fops);
	if (NULL == ent) 
	{
		FMDERR("Unable to create /proc/%s/sinr_threshold entry", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}


	/* Creating read/write "rssi" entry for bcm4330 fm*/
	ent = proc_create("sinr_samples", 0, fm_dir, &fm_sinr_samples_proc_fops);
	if (NULL == ent) 
	{
		FMDERR("Unable to create /proc/%s/sinr_samples entry", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}

	
	return 0;

fail:
	remove_proc_entry("chiptype", bt_dir);
	remove_proc_entry("bt_fm_fw_ver", bt_dir);
	remove_proc_entry("power_level", bt_dir);
	remove_proc_entry("sinr_threshold", fm_dir);
	remove_proc_entry("sinr_samples", fm_dir);
	remove_proc_entry("bt_feature", device_dir);
	remove_proc_entry("fm_feature", device_dir);
	remove_proc_entry("device_feature", 0);
	return retval;
}


/**
 * Cleans up the module.
 */
static void __exit featuretransfer_exit(void)
{
	platform_driver_unregister(&featuretransfer_driver);
	remove_proc_entry("chiptype", bt_dir);
	remove_proc_entry("bt_fm_fw_ver", bt_dir);
	remove_proc_entry("power_level", bt_dir);
	remove_proc_entry("sinr_threshold", fm_dir);
	remove_proc_entry("sinr_samples", fm_dir);
	remove_proc_entry("bt_feature", device_dir);
	remove_proc_entry("fm_feature", device_dir);
	remove_proc_entry("device_feature", 0);
}

module_init(featuretransfer_init);
module_exit(featuretransfer_exit);

MODULE_DESCRIPTION("BT DEVICE FEATURE VERSION: %s " VERSION);
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif
