#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/printk.h>
#include <asm/uaccess.h>

//kzalloc use it.
#include <linux/slab.h>

#define EMERGDATA_NAME "emerg_data"

struct proc_dir_entry *emerg_data = NULL;
extern unsigned int get_datamount_flag(void);
extern void set_datamount_flag(int value);

static ssize_t proc_emergdata_read(struct file * file,char *data,size_t len,loff_t *off)
{
    /*
     * return the value of datamount_flag(definition in setup.c)
     */
	char buffer[32] = {0};

	if(*off > 0LL)
	{
	    return 0;
    }

	memset(buffer, 0x00, sizeof(buffer));
	snprintf(buffer, sizeof(buffer) - 1, "%d\n", get_datamount_flag());

	if(copy_to_user(data, buffer, strlen(buffer)))
	{
	    return -EFAULT;
    }

	*off += strlen(buffer);
	return strlen(buffer);

}

static ssize_t proc_emergdata_write(struct file *file, const char *buffer, size_t count, loff_t *off)
{
    long value = -1;
    int strtol_ret = -1;
    ssize_t ret = -EINVAL;
    char *tmp_buf = NULL;

    if ((tmp_buf = kzalloc(count, GFP_KERNEL)) == NULL)
        return -ENOMEM;
    if (copy_from_user(tmp_buf, buffer, count - 1)) { //should ignore character '\n'
        kfree(tmp_buf);
        return -EFAULT;
    }

    tmp_buf[count - 1] = '\0';

    strtol_ret = strict_strtol(tmp_buf, 10, &value);

    /*
     * call function set_datamount_flag conditions as follow
     * 1. strict_strtol return 0, AND,
     * 2. value equal 0
     */
    if (strtol_ret == 0) {
        if (value == 0) {
            set_datamount_flag(value);
            ret = count;
        }
    }
    kfree(tmp_buf);
    return ret;
}

static struct file_operations emgergdata_proc_ops = {
       .read = proc_emergdata_read,
       .write = proc_emergdata_write,
};

static int __init emergdata_proc_init(void) {
    emerg_data = proc_create(EMERGDATA_NAME, 0660, NULL, &emgergdata_proc_ops);
    return 0;
}

static void __exit emergdata_proc_exit(void) {
    remove_proc_entry(EMERGDATA_NAME, emerg_data);
}

module_init(emergdata_proc_init);
module_exit(emergdata_proc_exit);
