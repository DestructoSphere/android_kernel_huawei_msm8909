
//modif esd check method so that TP can recover form disconnect...
#include <linux/init.h>
#include <linux/printk.h>
#ifdef CONFIG_HUAWEI_DSM
#include <linux/dsm_pub.h>
#endif/*CONFIG_HUAWEI_DSM*/
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>
#ifdef CONFIG_HUAWEI_DSM
#include <linux/dsm_pub.h>
#endif/*CONFIG_HUAWEI_DSM*/

#include "synaptics_dsx.h"
#include "synaptics_dsx_i2c.h"
#include "synaptics_dsx_esd.h"

static struct synaptics_esd synaptics_dsx_esd;
static struct synaptics_rmi4_data *g_rmi4_data = NULL;

synaptics_read synaptics_esd_read = NULL;
/*****************************************************************
Parameters    :  work
Return        :    
Description   :  check if is running
*****************************************************************/
static void synaptics_esd_work(struct work_struct *work)
{
    int i = 0;
    int ret = 0;
    unsigned char data = 0x00;

    synaptics_dsx_esd.esd_tirg_count++;
    tp_log_debug("%s %d:synaptics esd check is working\n", __func__, __LINE__);
    /* if irq is be handled, cancle esd check */
    ret = atomic_read(&(synaptics_dsx_esd.irq_status));
    if (ret != 0) 
    {
        tp_log_err("%s %d:synaptics ic is handle irq, count = %d.\n", 
                    __func__, __LINE__, ret);
        goto exit;
    }

    /* read 3 times, if success, ic is working, if all 3 times read 
        fail, ic is dead */
    for (i = 0; i < SYNAPTICS_ESD_RETRY_TIMES; i++) 
    {  
        ret = synaptics_esd_read(g_rmi4_data, 
                g_rmi4_data->f01_data_base_addr, &data, sizeof(data));
        tp_log_debug("%s#%d: data&0x80 = %d\n",__func__,__LINE__,(data&0x80));
        if (ret > 0 && !(data&0x80)) 
        {
            break;
        }
    }

    if (i == SYNAPTICS_ESD_RETRY_TIMES)
    {
        tp_log_err("%s %d:synaptics ic is dead\n", __func__, __LINE__);
#ifdef CONFIG_HUAWEI_DSM
        synp_tp_report_dsm_err(DSM_TP_ESD_ERROR_NO, ret);
#endif/*CONFIG_HUAWEI_DSM*/
        synaptics_dsx_hardware_reset(g_rmi4_data);
    }
    else if((synaptics_dsx_esd.esd_tirg_count % 10) == 0) {
        tp_log_info("%s#%d:synaptics ic is working...\n", __func__, __LINE__);
        tp_log_info("%s#%d: delay_time=%d, esd_count=%d\n",
            __func__,__LINE__,
            synaptics_dsx_esd.esd_delay_time,synaptics_dsx_esd.esd_tirg_count);
    }
exit:

    tp_log_debug("%s %d:synaptics data = %d\n", __func__, __LINE__, data);
    queue_delayed_work(synaptics_dsx_esd.esd_work_queue, 
        &synaptics_dsx_esd.esd_work, msecs_to_jiffies(synaptics_dsx_esd.esd_delay_time));
}


/*****************************************************************
Parameters    :  rmi4_data
                 read     
                 write    
Return        :  ok:0, error:-1
Description   :  init ESD parameter
*****************************************************************/
int synaptics_dsx_esd_init(struct synaptics_rmi4_data *rmi4_data, 
                            synaptics_read read, synaptics_write write)
{
    if (NULL == rmi4_data || NULL == read || NULL == write) 
    {
        tp_log_err("%s %d:input parameter is NULL\n", __func__, __LINE__);
        return -1;
    }

    tp_log_info("%s#%d:synaptics esd init\n", __func__, __LINE__);
    g_rmi4_data = rmi4_data;
    synaptics_esd_read = read;
    synaptics_dsx_esd.esd_delay_time = rmi4_data->board->esd_delay_time;
    
    INIT_DELAYED_WORK(&synaptics_dsx_esd.esd_work, synaptics_esd_work);
    
    synaptics_dsx_esd.esd_work_queue = 
                create_singlethread_workqueue("synaptics_esd_workqueue");
    if (!synaptics_dsx_esd.esd_work_queue)
    {
        tp_log_err("%s %d:synaptics esd workqueue alloc failed\n", __func__, __LINE__);
        return -1;
    }
    
    /* set esd check thread state */
    atomic_set(&(synaptics_dsx_esd.esd_check_status), ESD_CHECK_STOPED);
    atomic_set(&(synaptics_dsx_esd.irq_status), 0);
    return 0;
}

/*****************************************************************
Parameters    :  void
Return        :  success return work number,fail return 0; 
Description   :  
*****************************************************************/
int synaptics_dsx_esd_start(void) 
{
    int ret = 0;
    
    tp_log_info("%s %d:start synaptics esd check\n", __func__, __LINE__);    
    if (ESD_CHECK_STOPED == atomic_read(&synaptics_dsx_esd.esd_check_status))
    {
        ret = queue_delayed_work(synaptics_dsx_esd.esd_work_queue, 
            &synaptics_dsx_esd.esd_work, msecs_to_jiffies(synaptics_dsx_esd.esd_delay_time));
        if (!ret) {
            tp_log_err("%s %d:queue_delayed_work fail\n", __func__, __LINE__);
            return ret;
        }
        atomic_set(&(synaptics_dsx_esd.esd_check_status), ESD_CHECK_START);
        synaptics_dsx_esd.esd_tirg_count = 0;
    }
    else
    {
        tp_log_err("%s %d:synaptics esd check is not ready\n", __func__, __LINE__);
        ret = 0;
    }
    
    return ret;
}

/*****************************************************************
Parameters    :  void
Return        :    
Description   :  suspend esd check
*****************************************************************/
int synaptics_dsx_esd_stop(void)
{
    int ret = 0;
    
    tp_log_info("%s %d:stop synaptics esd check\n", __func__, __LINE__);
    if (ESD_CHECK_START == atomic_read(&synaptics_dsx_esd.esd_check_status))
    {
        ret = cancel_delayed_work(&synaptics_dsx_esd.esd_work);
        if (!ret) {
            tp_log_err("%s %d:stop synaptics esd fail.\n", __func__, __LINE__);
            return -1;
        }
        flush_delayed_work(&synaptics_dsx_esd.esd_work);
        atomic_set(&(synaptics_dsx_esd.esd_check_status), ESD_CHECK_STOPED);
    }
    else
    {
        tp_log_err("%s %d:synaptics esd check is not running\n", __func__, __LINE__);
    }
    
    return ret;
}

/*****************************************************************
Parameters    :  void
Return        :  void  
Description   :  set irq_handle flag, esd work will not read ic
*****************************************************************/
void synaptics_dsx_esd_suspend(void)
{
    int resume_count = atomic_read(&synaptics_dsx_esd.irq_status);
    atomic_set(&(synaptics_dsx_esd.irq_status), resume_count - 1);        
    tp_log_debug("%s %d:synaptics esd check suspend, count = %d\n", 
                __func__, __LINE__, resume_count - 1);
}

/*****************************************************************
Parameters    :  void
Return        :    
Description   :  set irq_handle flag, esd work will read ic
*****************************************************************/
void synaptics_dsx_esd_resume(void)
{
    int resume_count = atomic_read(&synaptics_dsx_esd.irq_status);
    atomic_set(&(synaptics_dsx_esd.irq_status), resume_count + 1);
    tp_log_debug("%s %d:synaptics esd check resume, count = %d\n", 
                __func__, __LINE__, resume_count + 1);
}

/*****************************************************************
Parameters    :  err_num
Return        :    
Description   :  save esd error to dsm
*****************************************************************/
#ifdef CONFIG_HUAWEI_DSM
ssize_t synaptics_dsm_record_esd_err_info( int err_num )
{

    ssize_t size = 0;
    struct dsm_client *tp_dclient = tp_dsm_get_client();

    /* esd err number */
    size =dsm_client_record(tp_dclient, "esd err number:%d\n", err_num);

    return size;
}
#endif/*CONFIG_HUAWEI_DSM*/

