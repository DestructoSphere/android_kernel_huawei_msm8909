/**
    @copyright: Huawei Technologies Co., Ltd. 2012-2012. All rights reserved.
    
    @file: srecorder_current_ps_backtrace.h
    
    @brief: 
    
    @version: 2.1.1 
    
    @author: Qi Dechun 00216641,    Yan Tongguang 00297150
    
    @date: 2015-03-13
    
    @history:
*/

#ifndef SRECORDER_BACK_TRACE_H
#define SRECORDER_BACK_TRACE_H

#include <asm/stacktrace.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
    @function: void srecorder_enable_back_trace(void)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_enable_back_trace(void);

/**
    @function: void srecorder_disable_back_trace(void)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_disable_back_trace(void);

/**
    @function: int srecorder_dump_back_trace(void)
    @brief: 
    @return: 
    @note: 
**/
int srecorder_dump_back_trace(void);

/**
    @function: void srecorder_dump_kernel_back_trace(struct task_struct *ptask)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_dump_kernel_back_trace(struct task_struct *ptask);

/**
    @function: asmlinkage void srecorder_c_backtrace(unsigned long fp, int pmode)
    @brief: do backtrace for ARM disconfig CONFIG_ARM_UNWIND 
    @param: fp frame pointer
    @return: none
    @note: 
**/
asmlinkage void srecorder_c_backtrace(unsigned long fp, int pmode);

#ifdef __cplusplus
}
#endif
#endif /* SRECORDER_BACK_TRACE_H */
