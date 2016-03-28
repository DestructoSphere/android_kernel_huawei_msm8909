/**
    @copyright: Huawei Technologies Co., Ltd. 2012-2012. All rights reserved.
    
    @file: srecorder_current_ps_backtrace.h
    
    @brief: 
    
    @version: 2.1.1 
    
    @author: Qi Dechun 00216641,    Yan Tongguang 00297150
    
    @date: 2015-03-13
    
    @history:
**/

#ifndef SRECORDER_CRC32_H
#define SRECORDER_CRC32_H

/**
    @function: unsigned int srecorder_calculate_crc32(unsigned char const* data, unsigned long len)
    @brief: Use CRC32 to do data check
    @param: data buffer to be checked
    @param: len  buffer length
    @return: CRC32 value
    @note: 
**/
unsigned int srecorder_calculate_crc32(unsigned char const* data, unsigned long len);

#endif
