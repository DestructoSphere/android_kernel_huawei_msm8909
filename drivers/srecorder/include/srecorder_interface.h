/**
    @copyright: Huawei Technologies Co., Ltd. 2012-2012. All rights reserved.
    
    @file: srecorder_interface.h
    
    @brief: 
    
    @version: 2.1.1 
    
    @author: Qi Dechun 00216641,    Yan Tongguang 00297150
    
    @date: 2015-03-13
    
    @history:
*/

#ifndef SRECORDER_INTERFACE_H
#define SRECORDER_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "srecorder_log.h"

/**
    @function: void srecorder_enable_external_category(void)
    @brief: 
    @param: 
    @return: 
    @note:
**/
void srecorder_enable_external_category(void);

/**
    @function: void srecorder_disable_external_category(void)
    @brief: 
    @param: 
    @return: 
    @note:
**/
void srecorder_disable_external_category(void);

/**
    @function: int srecorder_register_external_log(unsigned id, void(*callback)(void*))
    @brief: 
    @return: 
    @note: 
**/
int srecorder_register_external_log(unsigned id, void(*callback)(void*));

/**
    @function: int srecorder_commit_external_log(log_registration_t* external, unsigned timeout)
    @brief: 
    @return: 
    @note: 
**/
int srecorder_commit_external_log(log_registration_t* external, unsigned timeout);

/**
    @function: int srecorder_unregister_external_log(unsigned id)
    @brief: 
    @return: 
    @note: 
**/
int srecorder_unregister_external_log(unsigned id);

/**
    @function: int srecorder_get_external_log_info(unsigned long* log_buf, unsigned* log_len)
    @brief: 
    @return: 
    @note: 
**/
int srecorder_get_external_log_info(unsigned long* log_buf, unsigned* log_len);

/**
    @function: void srecorder_reset_external_log_info(void)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_reset_external_log_info(void);

#ifdef __cplusplus
}
#endif
#endif /* SRECORDER_INTERFACE_H */
