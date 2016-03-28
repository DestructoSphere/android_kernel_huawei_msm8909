#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/stat.h>
#include <linux/spinlock.h>
#include <linux/notifier.h>
#include <linux/kprobes.h>
#include <linux/reboot.h>
#include <linux/io.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/kernel.h>
#include <asm/uaccess.h>
#include <asm/barrier.h>
#include <linux/platform_device.h>
#include <linux/of_fdt.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/huawei_reset_detect.h>
#include <linux/of_address.h>

#define TRUE 1
#define FALSE 0


static void *reset_magic_addr = NULL;

void raw_writel(unsigned long value,void *p_addr)
{
    pr_info(RESET_DETECT_TAG "raw_writel p_addr=%p value=%08lx\n", p_addr, value);
    if(NULL == p_addr)
    {
        return;
    }
    __raw_writel(value,p_addr);
    return;
}

unsigned long raw_readl(void *p_addr)
{
    unsigned long return_value = 0;
    pr_info(RESET_DETECT_TAG "raw_readl p_addr=%p \n",p_addr);
    if(p_addr == NULL)
      return return_value;

	return_value = __raw_readl(p_addr);
    pr_info(RESET_DETECT_TAG "raw_readl value=%08lx \n",return_value);
    return return_value;
}

void set_reset_magic(int magic_number)
{   
    /* no need to check reset_magic_addr here, raw_writel will check */
    raw_writel(magic_number, reset_magic_addr);
    mb();
}

void clear_reset_magic()
{    
    /* no need to check reset_magic_addr here, raw_writel will check */
    raw_writel(0, reset_magic_addr);
    mb();
}


static int huawei_apanic_handler(struct notifier_block *this,
				  unsigned long event, void *ptr)
{
    int magic_number_test=0;
	pr_info(RESET_DETECT_TAG "huawei_apanic_handler enters \n");


#ifdef CONFIG_PREEMPT
	/* Ensure that cond_resched() won't try to preempt anybody */
	add_preempt_count(PREEMPT_ACTIVE);//增加软终端计数，避免该进程被调度
#endif
    magic_number_test= raw_readl(reset_magic_addr);
    if(magic_number_test!= LONG_PRESS_RESET_REASON_MAGIC_NUM)
    {
        set_reset_magic(RESET_MAGIC_APANIC);
    }
#ifdef CONFIG_PREEMPT
        sub_preempt_count(PREEMPT_ACTIVE);
#endif

	return NOTIFY_DONE;
}

static struct notifier_block huawei_apanic_event_nb = {
	.notifier_call  = huawei_apanic_handler,
	.priority = INT_MAX,
};

static void register_huawei_apanic_notifier(void)
{
    /* regitster the panic & reboot notifier */
    atomic_notifier_chain_register(&panic_notifier_list, &huawei_apanic_event_nb);
}


static int reset_magic_open(struct inode *inode, struct file *file)
{
	pr_debug(RESET_DETECT_TAG "%s enter\n", __func__);
	return 0;
}

static int reset_magic_release(struct inode *inode, struct file *file)
{
 	pr_debug(RESET_DETECT_TAG "%s enter\n", __func__); 
	return 0;
}


static ssize_t reset_magic_read(struct file *file, char __user *buf, size_t count,
			     loff_t *pos)
{
    unsigned magic_number = 0;

	if(count != RESET_MAGIC_NUM_LEN ){
		printk(RESET_DETECT_TAG "ERROR: %s magic number len must be 4\n", __func__);
		return -EINVAL;
	}

	magic_number = raw_readl(reset_magic_addr);

	pr_info(RESET_DETECT_TAG "%s hardware_reset_magic_number is %04x\n", __func__, magic_number);

	count = sizeof(magic_number);

	if(copy_to_user(buf, &magic_number, count))
	{
		printk(RESET_DETECT_TAG "ERROR: %s copy fail\n", __func__);
		return -EFAULT;
	}

	return count;
}


static ssize_t reset_magic_write(struct file *fp, const char __user *buf,
              size_t count, loff_t *pos)
{
	unsigned long magic_number = RESET_MAGIC_HW_RESET;

	if(count != RESET_MAGIC_NUM_LEN )
	{
		pr_err(RESET_DETECT_TAG "ERROR: %s magic number len must be 4\n", __func__);
		return -EINVAL;
	}

     if(copy_from_user(&magic_number, buf, RESET_MAGIC_NUM_LEN))
     {
         pr_err(RESET_DETECT_TAG "ERROR: %s copy fail\n", __func__);
         return -EINVAL;
     }

	raw_writel(magic_number, reset_magic_addr);
	pr_info(RESET_DETECT_TAG "%s written hardware reset magic number[%lx] to imem[%lx]\n", __func__, magic_number, (unsigned long)reset_magic_addr);

	return count;
}


static const struct file_operations reset_magic_fops = {
	.owner = THIS_MODULE,
	.open = reset_magic_open,
	.release = reset_magic_release,
	.read = reset_magic_read,
	.write = reset_magic_write,
};

static struct miscdevice reset_magic_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "reset_magic",
	.fops = &reset_magic_fops
};



static int __init reset_magic_address_get(void)
{
    struct device_node *np;

    np = of_find_compatible_node(NULL, NULL, "qcom,msm-imem-reset_magic_addr");
    if (!np)
    {
        pr_err(RESET_DETECT_TAG "unable to find DT imem reset_magic_addr node\n");
        return -1;
    }
    
    reset_magic_addr = of_iomap(np, 0);
    if (!reset_magic_addr)
    {
        pr_err(RESET_DETECT_TAG "unable to map imem reset_magic_addr offset\n");
        return -1;
    }

    pr_info(RESET_DETECT_TAG "reset_magic_addr=%p \n", reset_magic_addr);

    return 0;
}



static int __init huawei_reset_detect_init(void)
{
    int ret = 0;

    /* get reset magic address for dt and iomap it */
    ret = reset_magic_address_get();
    if(ret)
    {
        return -1;
    }
    
    /* regitster the panic & reboot notifier */
    register_huawei_apanic_notifier();


	misc_register(&reset_magic_miscdev);
    return 0;
}

static void __exit huawei_reset_detect_exit(void)
{
	misc_deregister(&reset_magic_miscdev);
}

module_init(huawei_reset_detect_init);
module_exit(huawei_reset_detect_exit);
MODULE_LICENSE(GPL);
