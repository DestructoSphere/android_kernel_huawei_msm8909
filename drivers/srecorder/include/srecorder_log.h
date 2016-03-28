/**
    @copyright: Huawei Technologies Co., Ltd. 2012-2012. All rights reserved.
    
    @file: srecorder_log.h
    
    @brief: 
    
    @version: 2.1.1 
    
    @author: Qi Dechun 00216641,    Yan Tongguang 00297150
    
    @date: 2015-03-13
    
    @history:
**/

#ifndef SRECORDER_LOG_H
#define SRECORDER_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/srecorder.h>

#define SRECORDER_INFO(var...)
#define SRECORDER_PRINTK printk
#define SRECORDER_SNPRINTF srecorder_snprintf

/* log category */
typedef enum
{
    CATEGORY_MIN,
    CATEGORY_PREV = CATEGORY_MIN,
    CATEGORY_CURR, 
    CATEGORY_DMESG,
    CATEGORY_EXT,
    CATEGORY_MAX = CATEGORY_EXT
} log_category_e;

/**
    @function: void srecorder_ioremap_dts_memory(void)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
void srecorder_ioremap_dts_memory(void);

/**
    @function: void srecorder_iounremap_dts_memory(void)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
void srecorder_iounremap_dts_memory(void);

/**
    @function: void srecorder_init_log_headers()
    @brief: 
    @return: 
    @note: 
**/
void srecorder_init_log_headers(void);

/**
    @function: int srecorder_enable_log_category_flag(unsigned flag)
    @brief: 
    @return: 
    @note: 
**/
int srecorder_enable_log_category_flag(unsigned flag);

/**
    @function: int srecorder_disable_log_category_flag(unsigned flag)
    @brief: 
    @return: 
    @note: 
**/
int srecorder_disable_log_category_flag(unsigned flag);

/**
    @function: void srecorder_enable_log_category_flags(void)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_enable_log_category_flags(void);

/**
    @function: void srecorder_disable_log_category_flags(void)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_disable_log_category_flags(void);

/**
    @function: int srecorder_enable_log_type_flag(unsigned flag)
    @brief: 
    @return: 
    @note: 
**/
int srecorder_enable_log_type_flag(unsigned flag);

/**
    @function: int srecorder_disable_log_type_flag(unsigned flag)
    @brief: 
    @return: 
    @note: 
**/
int srecorder_disable_log_type_flag(unsigned flag);

/**
    @function: void srecorder_enable_log_type_flags(void)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_enable_log_type_flags(void);

/**
    @function: void srecorder_disable_log_type_flags(void)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_disable_log_type_flags(void);

/**
    @function: int srecorder_verify_log_header_h(log_header_h_t* log_header_h)
    @brief: 
    @return: 
    @note: 
**/
int srecorder_verify_log_header_h(log_header_h_t* log_header_h);

/**
    @function: int srecorder_verify_log_header_m(log_header_m_t* log_header_m, int log_len_check)
    @brief: 
    @return: 
    @note: 
**/
int srecorder_verify_log_header_m(log_header_m_t* log_header_m, int log_len_check);

/**
    @function: int srecorder_verify_log_header_l(log_header_l_t* log_header_l)
    @brief: 
    @return: 
    @note: 
**/
int srecorder_verify_log_header_l(log_header_l_t* log_header_l);

/**
    @function: void srecorder_log_header_m_start()
    @brief: 
    @return: 
    @note: 
**/
void srecorder_log_header_m_start(void);

/**
    @function: void srecorder_log_header_m_end()
    @brief: 
    @return: 
    @note: 
**/
void srecorder_log_header_m_end(void);

/**
    @function: void srecorder_log_header_l_start(log_type_e log_type)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_log_header_l_start(log_type_e log_type);

/**
    @function: void srecorder_log_header_l_end()
    @brief: 
    @return: 
    @note: 
**/
void srecorder_log_header_l_end(void);

/**
    @function: void srecorder_dump_log_title(log_type_e type)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_dump_log_title(log_type_e type);

/**
    @function: void srecorder_update_log_header_m(log_header_m_t* log_header_m)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_update_log_header_m(log_header_m_t* log_header_m);

/**
    @function: void srecorder_update_log_header_l(log_header_l_t* log_header_l)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_update_log_header_l(log_header_l_t* log_header_l);

/**
    @function: log_header_m_t* srecorder_get_log_header_m(addr_index_t idx)
    @brief: 
    @return: 
    @note: 
**/
log_header_m_t* srecorder_get_log_header_m(addr_index_t idx);

/**
    @function: int srecorder_snprintf(const char *fmt, ...)
    @brief: 
    @return: 
    @note: 
**/
int srecorder_snprintf(const char *fmt, ...);

/**
    @function: int srecorder_get_first_log_info(log_category_e* log_category, unsigned long* log_buf, unsigned* log_len)
    @brief: 
    @return: 
    @note: 
**/
int srecorder_get_first_log_info(log_category_e* log_category, unsigned long* log_buf, unsigned* log_len);

/**
    @function: int srecorder_reset_first_log_info(void)
    @brief: 
    @return: 
    @note: 
**/
int srecorder_reset_first_log_info(void);

/**
    @function: void srecorder_reset_previous_log_info(void)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_reset_previous_log_info(void);

/**
    @function: void srecorder_wait_for_log(void)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_wait_for_log(void);

/**
    @function: void srecorder_wake_for_log(void)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_wake_for_log(void);

/**
    @function: void srecorder_notify_framework(reset_type_e reset_type)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_notify_framework(reset_type_e reset_type);

#ifdef __cplusplus
}
#endif
#endif /* SRECORDER_LOG_H */
