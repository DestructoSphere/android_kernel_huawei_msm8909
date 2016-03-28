/**
    @copyright: Huawei Technologies Co., Ltd. 2012-2013. All rights reserved.

    @file: srecorder_dev.c

    @brief:  Register a char device to dump log

    @version: 2.1.1 

    @author: Qi Dechun 00216641,    Yan Tongguang 00297150

    @date: 2015-03-13

    @history:
**/

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>
#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <asm/atomic.h>

#include "srecorder_dev.h"
#include "srecorder_misc.h"

#define SRECORDER_MISC_DEV_NAME "srecorder"

static ssize_t srecorder_read(struct file *file, char __user *buf, size_t count, loff_t *pos);
static long srecorder_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static int srecorder_open(struct inode *inode, struct file *file);
static int srecorder_release(struct inode *inode, struct file *file);

/*
* Kernel Interfaces
*/
static const struct file_operations s_srecorder_fops = 
{
    .owner          = THIS_MODULE,
    .open           = srecorder_open,
    .release        = srecorder_release,
    .read           = srecorder_read,
    .unlocked_ioctl = srecorder_ioctl,
};

static struct miscdevice s_srecorder_miscdev = 
{
    .minor      = MISC_DYNAMIC_MINOR,
    .name       = SRECORDER_MISC_DEV_NAME,
    .fops       = &s_srecorder_fops,
};

static DEFINE_SEMAPHORE(srecorder_dev_mutex);

static atomic_t srecorder_dev_blocker = ATOMIC_INIT(1);

/**
    @function: static int srecorder_open(struct inode *inode, struct file *file)
    @brief: open the SRecorder char device

    @param: inode
    @param: file

    @return: 0 - success; -1- failed

    @note: 
**/
static int srecorder_open(struct inode *inode, struct file *file)
{
    if (atomic_dec_and_test(&srecorder_dev_blocker) == 0) {
        atomic_inc(&srecorder_dev_blocker);
        return -EBUSY;
    }

    return generic_file_open(inode, file);
}

/**
    @function: static int srecorder_release(struct inode *inode, struct file *file)
    @brief: Close the SRecorder char device

    @param: inode
    @param: file

    @return: 0 - success; -1- failed

    @note: 
**/
static int srecorder_release(struct inode *inode, struct file *file)
{
    atomic_inc(&srecorder_dev_blocker);
    return 0;
}

/**
    @function: static long srecorder_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
    @brief: Ctrl the SRecorder char device
    @param: file
    @return: 0 - success; others - failed
    @note: 
**/
static long srecorder_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    unsigned flag = 0;
    unsigned long log_buf = 0;
    unsigned log_len = 0;
    log_category_e log_category;
    log_info_t log_info;

    switch (cmd)
    {
    case SRECORDER_IOC_ENABLE_CATEGORY_FLAG:
        {
            ret = __get_user(flag, (unsigned __user *)arg);
            srecorder_enable_log_category_flag(flag);
            break;
        }
    case SRECORDER_IOC_DISABLE_CATEGORY_FLAG:
        {
            ret = __get_user(flag, (unsigned __user *)arg);
            srecorder_disable_log_category_flag(flag);
            break;
        }
    case SRECORDER_IOC_ENABLE_TYPE_FLAG:
        {
            ret = __get_user(flag, (unsigned __user *)arg);
            srecorder_enable_log_type_flag(flag);
            break;
        }
    case SRECORDER_IOC_DISABLE_TYPE_FLAG:
        {
            ret = __get_user(flag, (unsigned __user *)arg);
            srecorder_disable_log_type_flag(flag);
            break;
        }
    case SRECORDER_IOC_GET_LOG_INFO:
        {
            ret = srecorder_get_first_log_info(&log_category, &log_buf, &log_len);
            if (ret == 0)
            {
                SRECORDER_INFO("srecorder_ioctl log_buf %lx log_len %x log_category %x\n", log_buf, log_len, (unsigned)log_category);
                log_info.category = log_category;
                log_info.len = log_len;
                if (copy_to_user((void*)arg, &log_info, sizeof(log_info_t)))
                {
                    SRECORDER_PRINTK("File: %s - Line: %d, copy data failed!\n", __FILE__, __LINE__);
                }
            }
            else
            {
                srecorder_wait_for_log();

                ret = srecorder_get_first_log_info(&log_category, &log_buf, &log_len);
                if (ret == 0)
                {
                    SRECORDER_INFO("srecorder_ioctl log_buf %lx log_len %x log_category %x\n", log_buf, log_len, (unsigned)log_category);
                    log_info.category = log_category;
                    log_info.len = log_len;
                    if (copy_to_user((void*)arg, &log_info, sizeof(log_info_t)))
                    {
                        SRECORDER_PRINTK("File: %s - Line: %d, copy data failed!\n", __FILE__, __LINE__);
                    }
                }
            }
            break;
        }
    default:
        {
            return -ENOTTY; 
        }
    }
    
    return ret;
}

/**
    @function: static ssize_t srecorder_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
    @brief: Read data from the SRecorder char device
    @param: pos
    @return: 0 - success; -1- failed
    @note: 
**/
static ssize_t srecorder_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
    size_t len = 0;
    unsigned long log_buf = 0;
    unsigned log_len = 0;
    log_category_e log_category;

    if (unlikely(NULL == buf))
    {
        SRECORDER_PRINTK("File: %s - Line: %d, Invalid parameter!\n", __FILE__, __LINE__);
        return -EINVAL;
    }
    
    down(&srecorder_dev_mutex);

    if(srecorder_get_first_log_info(&log_category, &log_buf, &log_len) != 0)
    {
        SRECORDER_PRINTK("File: %s - Line: %d, SRecorder's temp buf is NULL!\n", __FILE__, __LINE__);
        up(&srecorder_dev_mutex);
        return -EFAULT; /* Bad address */
    }
 
    len = MIN(count, log_len);
   
    if (copy_to_user((void*)buf, (void*)log_buf, len))
    {
        SRECORDER_PRINTK("File: %s - Line: %d, copy data failed!\n", __FILE__, __LINE__);
        up(&srecorder_dev_mutex);
        return -EFAULT;
    }

    srecorder_reset_first_log_info();

    up(&srecorder_dev_mutex);

    return len;
}

/**
    @function: int srecorder_init_dev(void)
    @brief: init this module
    @param: none
    @return: none
    @note: 
**/
int srecorder_init_dev(void)
{
    int ret = -1;
    
    ret = misc_register(&s_srecorder_miscdev);
    if (0 != ret)
    {
        SRECORDER_PRINTK("Cannot register miscdev on minor = %d (err = %d)\n", MISC_DYNAMIC_MINOR, ret);
    }
    else
    {
        SRECORDER_PRINTK("SRecorder registration done.\n ");
    }
    
    return ret;
}

/**
    @function: void srecorder_exit_dev(void)
    @brief: exit this module
    @param: none
    @return: none
    @note: 
**/
void srecorder_exit_dev(void)
{
    int rc = misc_deregister(&s_srecorder_miscdev);

    SRECORDER_PRINTK("SRecorder deregistration: %s!\n", (0 == rc) ? ("successfully") : ("failed!"));
}
