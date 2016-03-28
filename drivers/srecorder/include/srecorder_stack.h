/**
    @copyright: Huawei Technologies Co., Ltd. 2012-2012. All rights reserved.
    
    @file: srecorder_stack.h
    
    @brief: 
    
    @version: 2.1.1 
    
    @author: Qi Dechun 00216641,    Yan Tongguang 00297150
    
    @date: 2015-03-13
    
    @history:
**/

#ifndef SRECORDER_STACK_H
#define SRECORDER_STACK_H

#include "srecorder_misc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
    @function: int srecorder_dump_stack()
    @brief: 
    @return: 
    @note: 
**/
int srecorder_dump_stack(void);

/**
    @function: void srecorder_enable_stack(void)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_enable_stack(void);

/**
    @function: void srecorder_disable_stack(void)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_disable_stack(void);

#ifdef __cplusplus
}
#endif
#endif /* SRECORDER_STACK_H */
