#ifndef _HW_AUDIO_LOG_H
#define _HW_AUDIO_LOG_H

#include <linux/printk.h>

/* Current allowed minimum audio log level */
extern int audio_log_level;

/* Define audio log level */
#define AUDIO_LOG_LEVEL_VERBOSE 6
#define AUDIO_LOG_LEVEL_DEBUG   5
#define AUDIO_LOG_LEVEL_INFO    4
#define AUDIO_LOG_LEVEL_NOTICE  3
#define AUDIO_LOG_LEVEL_WARNING 2
#define AUDIO_LOG_LEVEL_ERROR   1
#define AUDIO_LOG_LEVEL_NONE    0

#ifdef CONFIG_HUAWEI_KERNEL

/* --------------------- printk wrap for HUAWEI BEGIN ----------------------- */
/* Verbose audio log */
#ifndef ad_logv
#define ad_logv(x...)                  \
do{                                    \
    if((KERNEL_HWFLOW)&&(audio_log_level >= AUDIO_LOG_LEVEL_VERBOSE)) \
    {                                       \
        pr_err("[AUDIO_V] " x);             \
    }                                       \
                                            \
}while(0)
#endif

/* Debug audio log */
#ifndef ad_logd
#define ad_logd(x...)                  \
do{                                    \
    if((KERNEL_HWFLOW)&&(audio_log_level >= AUDIO_LOG_LEVEL_DEBUG)) \
    {                                       \
        pr_err("[AUDIO_D] " x);             \
    }                                       \
                                            \
}while(0)
#endif

/* Info audio log */
#ifndef ad_logi
#define ad_logi(x...)                  \
do{                                    \
    if((KERNEL_HWFLOW)&&(audio_log_level >= AUDIO_LOG_LEVEL_INFO)) \
    {                                       \
        pr_err("[AUDIO_I] " x);             \
    }                                       \
                                            \
}while(0)
#endif

/* Notice audio log */
#ifndef ad_logn
#define ad_logn(x...)                  \
do{                                    \
    if(audio_log_level >= AUDIO_LOG_LEVEL_NOTICE) \
    {                                       \
        pr_err("[AUDIO_N] " x);             \
    }                                       \
                                            \
}while(0)
#endif

/* Warning audio log */
#ifndef ad_logw
#define ad_logw(x...)                  \
do{                                    \
    if(audio_log_level >= AUDIO_LOG_LEVEL_WARNING) \
    {                                       \
        pr_err("[AUDIO_W] " x);             \
    }                                       \
                                            \
}while(0)
#endif

/* Error audio log */
#ifndef ad_loge
#define ad_loge(x...)                  \
do{                                    \
    if(audio_log_level >= AUDIO_LOG_LEVEL_ERROR) \
    {                                       \
        pr_err("[AUDIO_E] " x);             \
    }                                       \
                                            \
}while(0)
#endif
/* --------------------- printk wrap for HUAWEI END ----------------------- */

/* ----------------- device log wrap for HUAWEI BEGIN --------------------- */
/* Verbose audio device log */
#ifndef ad_dev_logv
#define ad_dev_logv(x...)                   \
do{                                         \
    if((KERNEL_HWFLOW)&&(audio_log_level >= AUDIO_LOG_LEVEL_VERBOSE)) \
    {                                       \
        dev_err(x);                         \
    }                                       \
                                            \
}while(0)
#endif

/* Debug audio device log */
#ifndef ad_dev_logd
#define ad_dev_logd(x...)                   \
do{                                         \
    if((KERNEL_HWFLOW)&&(audio_log_level >= AUDIO_LOG_LEVEL_DEBUG)) \
    {                                       \
        dev_err(x);                         \
    }                                       \
                                            \
}while(0)
#endif

/* Info audio device log */
#ifndef ad_dev_logi
#define ad_dev_logi(x...)                   \
do{                                         \
    if((KERNEL_HWFLOW)&&(audio_log_level >= AUDIO_LOG_LEVEL_INFO)) \
    {                                       \
        dev_err(x);                         \
    }                                       \
                                            \
}while(0)
#endif

/* Notice audio device log */
#ifndef ad_dev_logn
#define ad_dev_logn(x...)                   \
do{                                         \
    if(audio_log_level >= AUDIO_LOG_LEVEL_NOTICE) \
    {                                       \
        dev_err(x);                         \
    }                                       \
                                            \
}while(0)
#endif

/* Warning audio device log */
#ifndef ad_dev_logw
#define ad_dev_logw(x...)                   \
do{                                         \
    if(audio_log_level >= AUDIO_LOG_LEVEL_WARNING) \
    {                                       \
        dev_err(x);                         \
    }                                       \
                                            \
}while(0)
#endif

/* Error audio device log */
#ifndef ad_dev_loge
#define ad_dev_loge(x...)                   \
do{                                         \
    if(audio_log_level >= AUDIO_LOG_LEVEL_ERROR) \
    {                                       \
        dev_err(x);                         \
    }                                       \
                                            \
}while(0)
#endif
/* ----------------- device log wrap for HUAWEI END --------------------- */

#else

/* ---------------- printk wrap for NON HUAWEI BEGIN -------------------- */
/* Verbose audio log */
#ifndef ad_logv
#define ad_logv(x...)                  \
do{                                    \
    pr_debug(x);                       \
}while(0)
#endif

/* Debug audio log */
#ifndef ad_logd
#define ad_logd(x...)                  \
do{                                    \
    pr_debug(x);                       \
}while(0)
#endif

/* Info audio log */
#ifndef ad_logi
#define ad_logi(x...)                  \
do{                                    \
    pr_debug(x);                       \
}while(0)
#endif

/* Notice audio log */
#ifndef ad_logn
#define ad_logn(x...)                  \
do{                                    \
    pr_debug(x);                       \
}while(0)
#endif

/* Warning audio log */
#ifndef ad_logw
#define ad_logw(x...)                  \
do{                                    \
    pr_warn(x);                        \
}while(0)
#endif

/* Error audio log */
#ifndef ad_loge
#define ad_loge(x...)                  \
do{                                    \
    pr_err(x);                         \
}while(0)
#endif
/* ---------------- printk wrap for NON HUAWEI END -------------------- */

/* --------------- device log wrap for NON HUAWEI BEGIN ----------------- */
/* Verbose audio device log */
#ifndef ad_dev_logv
#define ad_dev_logv(x...)                   \
do{                                         \
    dev_dbg(x);                             \
}while(0)
#endif

/* Debug audio device log */
#ifndef ad_dev_logd
#define ad_dev_logd(x...)                   \
do{                                         \
    dev_dbg(x);                             \
}while(0)
#endif

/* Info audio device log */
#ifndef ad_dev_logi
#define ad_dev_logi(x...)                   \
do{                                         \
    dev_dbg(x);                             \
}while(0)
#endif

/* Notice audio device log */
#ifndef ad_dev_logn
#define ad_dev_logn(x...)                   \
do{                                         \
    dev_dbg(x);                             \
}while(0)
#endif

/* Warning audio device log */
#ifndef ad_dev_logw
#define ad_dev_logw(x...)                   \
do{                                         \
    dev_warn(x);                            \
}while(0)
#endif

/* Error audio device log */
#ifndef ad_dev_loge
#define ad_dev_loge(x...)                   \
do{                                         \
    dev_err(x);                             \
}while(0)
#endif
/* --------------- device log wrap for NON HUAWEI END ----------------- */
#endif

#endif

