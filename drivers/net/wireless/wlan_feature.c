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


   Copyright (C) 2011-2013  Huawei Corporation
*/
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <asm/uaccess.h>
#include <linux/wcnss_wlan.h>

#define WLAN_DEVICE_VERSION		"1.0"
#define PROC_DIR	"wlan_feature"

#define WIFI_TYPE_UNKNOWN "Unknown"
struct proc_dir_entry    *wlan_dir, *devtype_dir, *pubfd_dir;

static int wlan_featuretrans_remove(struct platform_device *pdev)
{
    printk("wlan devicefeature removed.");
    return 0;
}

static struct platform_driver wlan_featuretrans_driver = {
    .remove = wlan_featuretrans_remove,
    .driver = {
        .name = "wlanfeaturetrans",
        .owner = THIS_MODULE,
    },
};

/* --------------------------- Wifi Device Type --------------------------*/
struct wifi_device
{
    char *chip_type;
    char *dev_name;
};

/* wifi chip name defination.  */
const struct wifi_device wifi_device_array[] =
{
    { "WIFI_BROADCOM_4330", "1.2" },
    { "WIFI_BROADCOM_4330X", "1.3" },
    { "WIFI_QUALCOMM_WCN3660", "2.2" },
    { "WIFI_QUALCOMM_WCN3620", "2.3" },
    { "WIFI_QUALCOMM_WCN3660B", "2.4" },
    { "WIFI_QUALCOMM_WCN3680", "2.5" },
    { "WIFI_QUALCOMM_WCN3610", "2.6" },
    { "WIFI_TYPE_UNKNOWN", "Unknown" }
};

/**
 * Get wifi device type.
 * @param no.
 * @return On success, the number of bytes written. On error, -1, and
 * <code>errno</code> is set appropriately.
 */
char *get_wifi_device_type(void)
{
    int i = 0;
    int chip_type_len;
    int arry_size = sizeof(wifi_device_array)/sizeof(wifi_device_array[0]);
    const char *wifi_chip_type = WIFI_TYPE_UNKNOWN;
    struct device_node *dp = NULL;
    dp = of_find_node_by_path("/huawei_wifi_info");
    if(!dp)
    {
        printk("device is not available!\n");
        return wifi_device_array[arry_size - 1].dev_name;
    }

    /* get wifi chip type from the device feature configuration (.dtsi file) */
    wifi_chip_type = of_get_property(dp,"wifi,chiptype", &chip_type_len);
    if(NULL == wifi_chip_type)
    {
        printk("WIFI, Get chip type fail.\n");
        return wifi_device_array[arry_size - 1].dev_name;
    }

    /* lookup wifi_device_model in wifi_device_array[] */
    for(i = 0; i < arry_size; i++)
    {
        if(0 == strncmp(wifi_chip_type,wifi_device_array[i].chip_type,chip_type_len))
        {
            break;
        }
    }
    /* If we get invalid type name, return "Unknown".*/
    if( i == arry_size)
    {
         printk("WIFI, Get chip type fail.\n");
         return wifi_device_array[arry_size - 1].dev_name;
    }

    return wifi_device_array[i].dev_name;
}
EXPORT_SYMBOL(get_wifi_device_type);


static int devtype_proc_show(struct seq_file *m, void *v)
{
    const char *wifi_device_type = NULL;

    wifi_device_type = get_wifi_device_type();
    printk("%s enter wifi_device_type:%s;\n", __func__,wifi_device_type);
    seq_printf(m,"%s",wifi_device_type);

    return 0;
}

static int devtype_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, devtype_proc_show, NULL);
}

 static const struct file_operations devtype_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= devtype_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


 const void *get_hw_wifi_pubfile_id(void)
{
    int wifi_pubfile_id_len;
    struct device_node *dp = NULL;

    dp = of_find_node_by_path("/huawei_wifi_info");
    if(!dp)
    {
         printk("device is not available!\n");
         return NULL;
    }
    else
    {
         printk("%s:dp->name:%s,dp->full_name:%s;\n",__func__,dp->name,dp->full_name);
    }

    return  of_get_property(dp,"wifi,pubfile_id", &wifi_pubfile_id_len);
}
EXPORT_SYMBOL(get_hw_wifi_pubfile_id);

static int pubfd_proc_show(struct seq_file *m, void *v)
{
    const char *wifi_pubfile_id = NULL;

    wifi_pubfile_id = get_hw_wifi_pubfile_id();

    if( NULL != wifi_pubfile_id )
    {
        printk("%s enter wifi_device_type:%s;\n", __func__,wifi_pubfile_id);
    }
    else
    {
        printk("%s get pubfd failed;\n", __func__);
        return 0;
    }

    seq_printf(m,"%s",wifi_pubfile_id);

    return 0;
}

 static int pubfd_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, pubfd_proc_show, NULL);
}

 static const struct file_operations pubfd_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= pubfd_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


const void *get_wlan_fw_ver(void)
{
    int wlan_fw_ver_len;
    struct device_node *dp = NULL;
    dp = of_find_node_by_path("/huawei_wifi_info");
    if(!dp)
    {
         printk("device is not available!\n");
         return NULL;
    }
    else
    {
         printk("%s:dp->name:%s,dp->full_name:%s;\n",__func__,dp->name,dp->full_name);
    }

    return  of_get_property(dp,"wifi,fw_ver", &wlan_fw_ver_len);
}

 static int wlan_fw_ver_proc_show(struct seq_file *m, void *v)
{
    const char *wlan_fw_ver = NULL;

    wlan_fw_ver = get_wlan_fw_ver();

    if( NULL !=wlan_fw_ver )
    {
         printk("%s enter wifi_device_type:%s;\n", __func__,wlan_fw_ver);
    }
    else
    {
        printk("%s get wlan_fw_ver failed;\n", __func__);
        return 0;
    }

    seq_printf(m,"%s",wlan_fw_ver);

    return 0;
}

 static int wlan_fw_ver_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, wlan_fw_ver_proc_show, NULL);
}

 static const struct file_operations wlan_fw_ver_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= wlan_fw_ver_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static int __init wlanfeaturetrans_init(void)
{
    int retval = 0;
    struct proc_dir_entry *ent = NULL;

    printk("WIFI DEVICE FEATURE VERSION: %s", WLAN_DEVICE_VERSION);

    /* Driver Register */
    retval = platform_driver_register(&wlan_featuretrans_driver);
    if (0 != retval)
    {
        printk("[%s],featurntransfer driver register fail.", __func__);
        return retval;
    }

    /* create device_feature directory for wifi chip info */
    wlan_dir = proc_mkdir("wlan_feature", NULL);
    if (NULL == wlan_dir)
    {
        printk("Unable to create /proc/wlan_feature directory");
        retval =  -ENOMEM;
        goto error_dir;
    }

    /* Creating read/write "devtype" entry*/
    ent= proc_create("devtype", 0, wlan_dir, &devtype_proc_fops);
    if (!ent)
    {
        printk("Unable to create /proc/%s/devtype entry", PROC_DIR);
        retval =  -ENOMEM;
        goto error_devtype;
    }

    /* Creating read/write "pubfd" entry*/
    ent= proc_create("pubfd", 0, wlan_dir, &pubfd_proc_fops);
    if (!ent)
    {
        printk("Unable to create /proc/%s/pubfd entry", PROC_DIR);
        retval =  -ENOMEM;
        goto error_pubfd;
    }

    /* Creating read/write "wlan_fw_ver" entry for wifi chip type*/
    ent= proc_create("wlan_fw_ver", 0, wlan_dir, &wlan_fw_ver_proc_fops);
    if (!ent)
    {
        printk("Unable to create /proc/%s/wlan_fw_ver entry", PROC_DIR);
        retval =  -ENOMEM;
        goto error_wlan_fw_ver;
    }

    return 0;

error_wlan_fw_ver:
    remove_proc_entry("wlan_fw_ver", wlan_dir);
error_pubfd:
    remove_proc_entry("pubfd", wlan_dir);
error_devtype:
    remove_proc_entry("devtype", wlan_dir);
error_dir:
    remove_proc_entry("wlan_feature", NULL);
    platform_driver_unregister(&wlan_featuretrans_driver);

    return retval;
}


/**
 * Cleans up the module.
 */
static void __exit wlanfeaturetrans_exit(void)
{
    platform_driver_unregister(&wlan_featuretrans_driver);
    remove_proc_entry("devtype", wlan_dir);
    remove_proc_entry("pubfd", wlan_dir);
    remove_proc_entry("wlan_fw_ver", wlan_dir);
    remove_proc_entry("wlan_feature", 0);
}


module_init(wlanfeaturetrans_init);
module_exit(wlanfeaturetrans_exit);

MODULE_DESCRIPTION("WIFI DEVICE FEATURE VERSION: %s " WLAN_DEVICE_VERSION);
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif
