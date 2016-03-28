/**
    @copyright: Huawei Technologies Co., Ltd. 2012-2012. All rights reserved.
    
    @file: srecorder_sys_info.h
    
    @brief: 
    
    @version: 2.1.1 
    
    @author: Qi Dechun 00216641,    Yan Tongguang 00297150
    
    @date: 2015-03-13
    
    @history:
**/

#ifndef SRECORDER_SYS_INFO_H
#define SRECORDER_SYS_INFO_H

#include "srecorder_misc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
    @function: void srecorder_enable_sys_info(void)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_enable_sys_info(void);

/**
    @function: void srecorder_disable_sys_info(void)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_disable_sys_info(void);

/**
    @function: int srecorder_dump_sys_info(void)
    @brief: 
    @return: 
    @note: 
**/
int srecorder_dump_sys_info(void);

#ifdef __cplusplus
}
#endif
#endif /* SRECORDER_SYS_INFO_H */
