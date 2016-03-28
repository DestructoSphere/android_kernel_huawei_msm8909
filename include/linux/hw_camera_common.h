/*************************************************
Copyright . Huawei Technologies Co., Ltd. 1998-2014. All rights reserved.
File name:  hw_camera_common.c
Version:    Initial Draft
Date:       2014/07/26
Description: The camera common used macros and functions
Others: none
History:

*************************************************/


#ifndef HW_CAMERA_COMMON_H__
#define HW_CAMERA_COMMON_H__

#include<linux/printk.h>

#ifndef HW_CMR_LOGSWC
    #define HW_CMR_LOGSWC 1  //define the file log switch
#endif

#ifndef HW_CMR_LOG_TAG
    #define HW_CMR_LOG_TAG ""  //define the log tag
#endif

enum
{
    HW_CMR_LOG_DBG  = 1U<<3,
    HW_CMR_LOG_INFO = 1U<<2,
    HW_CMR_LOG_WARN = 1U<<1,
    HW_CMR_LOG_ERR  = 1U<<0
};

extern unsigned int hw_cmr_log_mask;    /*dynamic camera log mask */

//debug log
#ifndef CMR_LOGD
#define CMR_LOGD(...)\
    do{\
        if (KERNEL_HWFLOW && HW_CMR_LOGSWC && (hw_cmr_log_mask & HW_CMR_LOG_DBG))\
            printk(KERN_DEBUG"D/[HWCamera] "HW_CMR_LOG_TAG": " __VA_ARGS__);\
    }while(0)
#endif
//info log
#ifndef CMR_LOGI
#define CMR_LOGI(...)\
    do{\
        if (KERNEL_HWFLOW && HW_CMR_LOGSWC && (hw_cmr_log_mask & HW_CMR_LOG_INFO))\
            pr_info("I/[HWCamera] "HW_CMR_LOG_TAG": " __VA_ARGS__);\
    }while(0)
#endif
//warning log
#ifndef CMR_LOGW
#define CMR_LOGW(...)\
    do{\
        if (HW_CMR_LOGSWC && (hw_cmr_log_mask & HW_CMR_LOG_WARN))\
            pr_warn("W/[HWCamera] "HW_CMR_LOG_TAG": " __VA_ARGS__);\
    }while(0)
#endif
//error log
#ifndef CMR_LOGE
#define CMR_LOGE(...)\
    do{\
        if (HW_CMR_LOGSWC && (hw_cmr_log_mask & HW_CMR_LOG_ERR))\
            pr_err("E/[HWCamera] "HW_CMR_LOG_TAG": " __VA_ARGS__);\
    }while(0)
#endif


#endif  /*HW_CAMERA_COMMON_H__*/
