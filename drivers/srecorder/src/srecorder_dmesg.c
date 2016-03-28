/**
    @copyright: Huawei Technologies Co., Ltd. 2012-2012. All rights reserved.
    
    @file: srecorder_dmesg.c
    
    @brief: 
    
    @version: 2.1.1 
    
    @author: Qi Dechun 00216641,    Yan Tongguang 00297150
    
    @date: 2015-03-13
    
    @history:
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/ctype.h>
#include <linux/highmem.h>

#include <asm/uaccess.h>
#include <asm/io.h>

#include "srecorder_dmesg.h"
#include "srecorder_symbols.h"
#include "srecorder_misc.h"

static int dmesg_flag = 0;

/**
    @function: void srecorder_enable_dmesg(void)
    @brief: 
    @return:
    @note: 
**/
void srecorder_enable_dmesg(void)
{
    dmesg_flag = 1;
}

/**
    @function: void srecorder_disable_dmesg(void)
    @brief: 
    @return:
    @note: 
**/
void srecorder_disable_dmesg(void)
{
    dmesg_flag = 0;
}

/**
    @function: int srecorder_dump_dmesg(void)
    @brief: 
    @return:
    @note: 
**/
int srecorder_dump_dmesg(void)
{
    unsigned log_end;
    unsigned log_buf_len;
    unsigned long log_buf;

    if (dmesg_flag == 0)
    {
        SRECORDER_PRINTK("The dump flag of dmesg isn't enabled\n");
        return -1;
    }

    srecorder_log_header_l_start(TYPE_DMESG);

    srecorder_dump_log_title(TYPE_DMESG);
    
    /* 
    * Theoretically, the log could move on after we do this, but
    * there's not a lot we can do about that. The new messages
    * will overwrite the start of what we dump. 
    */
    srecorder_get_printk_buf_info(&log_buf, &log_end, &log_buf_len);

    SRECORDER_SNPRINTF((char const*)log_buf + log_end, log_buf_len - log_end);

    SRECORDER_SNPRINTF((char const*)log_buf, log_end);

    srecorder_log_header_l_end();
    
    return 0;
}

/**
    @function: int srecorder_init_dmesg_log_info(unsigned long* p_log_buf_pa, unsigned long* p_log_buf_va, unsigned* p_log_len, unsigned* p_log_buf_len)
    @brief: 
    @return:
    @note: 
**/
int srecorder_init_dmesg_log_info(unsigned long* p_log_buf_pa, unsigned long* p_log_buf_va, unsigned* p_log_len, unsigned* p_log_buf_len)
{
    int log_end = 0;
    int log_buf_len = 0;
    unsigned long va = 0UL;

    srecorder_get_printk_buf_info(&va, &log_end, &log_buf_len);

    *p_log_buf_pa = __pa(va);
    *p_log_buf_va = (unsigned long)va;
    *p_log_len = log_buf_len - log_end;
    *p_log_buf_len = log_buf_len;

    return 0;
}
