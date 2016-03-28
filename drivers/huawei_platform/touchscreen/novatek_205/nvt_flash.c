
/*************************
  Create Device Node (Proc Entry)
**************************/

#include "nvt.h"

extern void nvt_start_esd_timer(void);
extern void nvt_stop_esd_timer(void);

#define DEVICE_NAME "NVTflash"

extern struct nvt_ts_data *ts;

int nvt_flash_write(struct file *file, const char __user *buff, size_t count, loff_t *offp)
{
    struct i2c_msg msgs[2];    
    char *str;
    int ret=-1;
    int retries = 0;
    file->private_data = (uint8_t *)kmalloc(64, GFP_KERNEL);
    str = file->private_data;
    ret=copy_from_user(str, buff, count);

    msgs[0].flags = !I2C_M_RD;
    msgs[0].addr  = str[0];
    msgs[0].len   = str[1];
    msgs[0].buf   = &str[2];

    while(retries < 20){
        ret = i2c_transfer(ts->client->adapter, msgs, 1);
        if(ret == 1){
            break;
        }else{
            tp_log_err("%s line%d: write error %d\n", __func__,__LINE__,retries);
        }
        retries++;
    }
    return ret;
}

int nvt_flash_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
    struct i2c_msg msgs[2];     
    char *str;
    int ret = -1;
    int retries = 0;
    file->private_data = (uint8_t *)kmalloc(64, GFP_KERNEL);
    str = file->private_data;
    if(copy_from_user(str, buff, count)){
        return -EFAULT;
    }

    msgs[0].flags = !I2C_M_RD;
    msgs[0].addr  = str[0];
    msgs[0].len   = 1;
    msgs[0].buf   = &str[2];

    msgs[1].flags = I2C_M_RD;
    msgs[1].addr  = str[0];
    msgs[1].len   = str[1]-1;
    msgs[1].buf   = &str[3];

    while(retries < 20)
    {
        ret = i2c_transfer(ts->client->adapter, msgs, 2);
        if(ret == 2){
            break;
        }else{
            tp_log_err("%s line%d: read error %d\n",__func__,__LINE__, retries);
        }
        retries++;
    }
    ret=copy_to_user(buff, str, count);
    return ret;
}

int nvt_flash_open(struct inode *inode, struct file *file)
{
    struct nvt_flash_data *dev;

    dev = kmalloc(sizeof(struct nvt_flash_data), GFP_KERNEL);
    if(dev == NULL){
        return -ENOMEM;
    }
    rwlock_init(&dev->lock);
    file->private_data = dev;
    nvt_stop_esd_timer();
    return 0;
}

int nvt_flash_close(struct inode *inode, struct file *file)
{
    struct nvt_flash_data *dev = file->private_data;
    
    if(dev){
        kfree(dev);
    }
    nvt_start_esd_timer();
    return 0;   
}

struct file_operations nvt_flash_fops = {
    .owner = THIS_MODULE,
    .open = nvt_flash_open,
    .release = nvt_flash_close,
    .write = nvt_flash_write,
    .read = nvt_flash_read,
};

int nvt_proc_flash_init(void)
{        
    int ret=0;
    static struct proc_dir_entry *nvt_proc_entry;

    nvt_proc_entry = proc_create(DEVICE_NAME, 0644, NULL, &nvt_flash_fops);
    if(nvt_proc_entry == NULL){
        tp_log_err("%s line%d: proc_create entry failed!\n", __func__,__LINE__);
        ret = -ENOMEM;
        return ret ;
    }else{
        tp_log_info("%s line%d: proc_create entry succeed.\n",__func__,__LINE__);
    }
    
    return 0;
}
