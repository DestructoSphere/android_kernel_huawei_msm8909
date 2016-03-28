
#ifndef __LINUX_FOCALTECH_LOG_H__
#define __LINUX_FOCALTECH_LOG_H__

#include <linux/stringify.h>

/*debug */
#define TP_ERR  1
#define TP_INFO 2
#define TP_DBG  3
#define TP_VDBG  4
extern int focaltech_debug_mask;

#ifndef tp_log_err
#define tp_log_err(x...)                \
do{                                     \
    if( focaltech_debug_mask >= TP_ERR )   \
    {                                   \
        printk(KERN_ERR "[FT6X06] " x); \
    }                                   \
                                        \
}while(0)
#endif

#ifndef tp_log_info
#define tp_log_info(x...)               \
do{                                     \
    if( focaltech_debug_mask >= TP_INFO )  \
    {                                   \
        printk(KERN_ERR "[FT6X06] " x); \
    }                                   \
                                        \
}while(0)
#endif

#ifndef tp_log_debug
#define tp_log_debug(x...)              \
do{                                     \
    if( focaltech_debug_mask >= TP_DBG )   \
    {                                   \
        printk(KERN_ERR "[FT6X06] " x); \
    }                                   \
                                        \
}while(0)
#endif

#ifndef tp_log_vdebug
#define tp_log_vdebug(x...)              \
do{                                     \
    if( cyttsp_debug_mask >= TP_VDBG )   \
    {                                   \
        printk(KERN_ERR "[FT6X06] " x); \
    }                                   \
                                        \
}while(0)
#endif

#endif 

