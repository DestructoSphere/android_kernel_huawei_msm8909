/*
 * hw_camera_log file
 */

#ifndef _HW_CAMERA_LOGE_H
#define _HW_CAMERA_LOGE_H

#define HW_CAMERA_LOG_E  1
#define HW_CAMERA_LOG_I  2
#define HW_CAMERA_LOG_D  3
#define HW_CAMERA_LOG_V  4

extern int g_hw_camera_log_mask ;
    
#ifndef hw_camera_log_error
#define hw_camera_log_error(fmt...)               \
do{                                     \
    if(g_hw_camera_log_mask >= HW_CAMERA_LOG_E )  \
    {                                   \
        printk(KERN_ERR "[hw_camera_log_error] " fmt); \
    }                                   \
                                        \
}while(0)
#endif

#ifndef hw_camera_log_info
#define hw_camera_log_info(fmt...)               \
do{                                     \
    if(g_hw_camera_log_mask >= HW_CAMERA_LOG_I )  \
    {                                   \
        printk(KERN_ERR "[hw_camera_log_info] " fmt); \
    }                                   \
                                        \
}while(0)
#endif

#ifndef hw_camera_log_debug
#define hw_camera_log_debug(fmt...)              \
do{                                     \
    if(g_hw_camera_log_mask >= HW_CAMERA_LOG_D)   \
    {                                   \
        printk(KERN_ERR "[hw_camera_log_debug] " fmt); \
    }                                   \
                                        \
}while(0)
#endif

#ifndef hw_camera_log_verbose
#define hw_camera_log_verbose(fmt...)              \
do{                                     \
    if(g_hw_camera_log_mask >= HW_CAMERA_LOG_V)   \
    {                                   \
        printk(KERN_ERR "[hw_camera_log_verbose] " fmt); \
    }                                   \
                                        \
}while(0)
#endif

#endif /*_HW_CAMERA_LOGE_H */
