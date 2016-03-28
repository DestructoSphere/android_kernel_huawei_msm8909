/**
    @copyright: Huawei Technologies Co., Ltd. 2012-2012. All rights reserved.
    
    @file: srecorder_dmesg.h
    
    @brief: 
    
    @version: 2.1.1 
    
    @author: Qi Dechun 00216641,    Yan Tongguang 00297150
    
    @date: 2015-03-13
    
    @history:
*/

#ifndef SRECORDER_DMESG_H
#define SRECORDER_DMESG_H

#include "srecorder_log.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
    @function: void srecorder_enable_dmesg(void)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_enable_dmesg(void);

/**
    @function: void srecorder_disable_dmesg(void)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_disable_dmesg(void);

/**
    @function: int srecorder_dump_dmesg(void)
    @brief: 
    @return: 
    @note: 
**/
int srecorder_dump_dmesg(void);

/**
    @function: void srecorder_dump_previous_dmesg(log_header_h_t* p_log_header_h, log_header_m_t* p_log_header_m_temp)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_dump_previous_dmesg(log_header_h_t* p_log_header_h, log_header_m_t* p_log_header_m_temp);

/**
    @function: int srecorder_init_dmesg_log_info(unsigned long* p_log_buf_pa, unsigned long* p_log_buf_va, unsigned* p_log_len, unsigned* p_log_buf_len)
    @brief: 
    @return: 
    @note: 
**/
int srecorder_init_dmesg_log_info(unsigned long* p_log_buf_pa, unsigned long* p_log_buf_va, unsigned* p_log_len, unsigned* p_log_buf_len);

/**
    @function: int srecorder_get_dmesg_log_info(unsigned long* log_buf, unsigned* log_len)
    @brief: 
    @return: 
    @note: 
**/
int srecorder_get_dmesg_log_info(unsigned long* log_buf, unsigned* log_len);

#ifdef __cplusplus
}
#endif
#endif /* SRECORDER_DMESG_H */

