/**
    @copyright: Huawei Technologies Co., Ltd. 2012-2012. All rights reserved.
    
    @file: srecorder_crash_time.c
    
    @brief: 
    
    @version: 2.1.1 
    
    @author: Qi Dechun 00216641,    Yan Tongguang 00297150
    
    @date: 2015-03-13
    
    @history:
*/

#include <linux/stddef.h>
#include <linux/timer.h>
#include <linux/timex.h>
#include <linux/rtc.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <asm/uaccess.h>

#include "srecorder_reason_time.h"
#include "srecorder_symbols.h"

#define LOG_BUF_MASK(log_buf_len) ((log_buf_len) - 1)

static int reason_time_flag = 0;

/**
    @function: void srecorder_enable_reason_time()
    @brief: 
    @return: 
    @note: 
**/
void srecorder_enable_reason_time(void)
{
    reason_time_flag = 1;
}

/**
    @function: void srecorder_disable_reason_time()
    @brief: 
    @return: 
    @note: 
**/
void srecorder_disable_reason_time(void)
{
    reason_time_flag = 0;
}

/**
    @function: int srecorder_dump_reason_time()
    @brief: 
    @return: 
    @note: 
**/
int srecorder_dump_reason_time(void)
{
    struct timeval tv;
    struct rtc_time tm;

    if (reason_time_flag == 0)
    {
        SRECORDER_PRINTK("The dump flag of crash reason and time isn't enabled\n");
        return -1;
    }

    memset(&tv, 0, sizeof(struct timeval));
    memset(&tm, 0, sizeof(struct rtc_time));

    srecorder_log_header_l_start(TYPE_REASON_TIME);

    srecorder_dump_log_title(TYPE_REASON_TIME);

    do_gettimeofday(&tv);
    tv.tv_sec -= sys_tz.tz_minuteswest * 60;
    rtc_time_to_tm(tv.tv_sec, &tm);

    SRECORDER_SNPRINTF("Crash reason: apanic Crash Time: %04d%02d%02d-%02d:%02d:%02d\n", 
        tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, 
        tm.tm_hour, tm.tm_min, tm.tm_sec);

    srecorder_log_header_l_end();
    
    return 0;
}
