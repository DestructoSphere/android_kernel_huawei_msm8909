/**
    @copyright: Huawei Technologies Co., Ltd. 2012-2012. All rights reserved.
    
    @file: srecorder_misc.h
    
    @brief: 
    
    @version: 2.1.1 
    
    @author: Qi Dechun 00216641,    Yan Tongguang 00297150
    
    @date: 2015-03-13
    
    @history:
*/

#ifndef SRECORDER_MISC_H
#define SRECORDER_MISC_H

#include <linux/spinlock.h>

#ifdef __cplusplus
extern "C" {
#endif

#define INVALID_KSYM_ADDR (0UL) 
#define DMESG_MAX_LENGTH (0x20000) /*128KB*/

#ifdef CONFIG_DEBUG_SRECORDER
#define DEBUG_SRECORDER (CONFIG_DEBUG_SRECORDER) 
#else
#define DEBUG_SRECORDER 1 
#endif

#define DEBUG_KERNEL_SYMBOLS 1
#define DEBUG_CRASH_TIME 1
#define DEBUG_SYS_INFO 1
#define DEBUG_DMESG 1
#define DEBUG_ALLCPU_STACK 1
#define DEBUG_ALLPS_INFO 1
#define DEBUG_CURRENT_PS_BACKTRACE 1
#define DEBUG_SLAB_INFO 1

#define K(x) ((x) << (PAGE_SHIFT - 10)) 
#define MAX(a, b) ((a) > (b)) ? (a) : (b)
#define MIN(a, b) ((a) > (b)) ? (b) : (a)
#define LOG_BUF_MASK(log_buf_len) ((log_buf_len) - 1)

/* log_buf_len must equal 2 ^ n ((n >= 0) && (n <= 32)) or the result may be wrong*/
#define LOG_OFFSET(len, log_buf_len) ((len) & LOG_BUF_MASK(log_buf_len))

typedef unsigned long srec_ul32;
typedef unsigned long long srec_ul64;
typedef srec_ul32 srec_ksym_addr_t;
typedef srec_ul32 srec_reserved_mem_t;

#ifdef __cplusplus
}
#endif
#endif /* SRECORDER_MISC_H */
