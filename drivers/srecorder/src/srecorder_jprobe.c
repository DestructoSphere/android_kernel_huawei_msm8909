/**
    @copyright: Huawei Technologies Co., Ltd. 2012-2012. All rights reserved.
    
    @file: srecorder_log.c
    
    @brief: 
    
    @version: 2.1.1 
    
    @author: Qi Dechun 00216641,    Yan Tongguang 00297150
    
    @date: 2015-03-13
    
    @history:
**/

#ifdef CONFIG_KPROBES

#include <linux/stddef.h>
#include <linux/kprobes.h>

#include "srecorder_jprobe.h"
#include "srecorder_log.h"

extern int register_jprobe(struct jprobe *p);
extern void unregister_jprobe(struct jprobe *p);

/**
    @function: static void srecorder_jkernel_restart(char *cmd)
    @brief: 
    @param: 
    @return: 
    @note:
**/
static void srecorder_jkernel_restart(char *cmd)
{
    srecorder_notify_framework(RESET_NORMAL);
    
    jprobe_return();
}

/**
    @function: static void srecorder_jemergency_restart(void)
    @brief: 
    @param: 
    @return: 
    @note:
**/
static void srecorder_jemergency_restart(void)
{
    srecorder_notify_framework(RESET_NORMAL);
    
    jprobe_return();
}

static struct jprobe s_srecorder_jkernel_restart = 
{
    .entry = srecorder_jkernel_restart,
    .kp = 
    {
        .symbol_name = "kernel_restart", 
    },
};

static struct jprobe s_srecorder_jemergency_restart = 
{
    .entry = srecorder_jemergency_restart,
    .kp = 
    {
        .symbol_name = "emergency_restart", 
    },
};

typedef struct
{
    struct jprobe *p;
    int (*register_func)(struct jprobe *p);
    void (*unregister_func)(struct jprobe *p);
} srecorder_jprobe_operations_t;

static srecorder_jprobe_operations_t s_srecorder_jprobe_operations[] = 
{
        {&s_srecorder_jkernel_restart, register_jprobe, unregister_jprobe},
        {&s_srecorder_jemergency_restart, register_jprobe, unregister_jprobe},
};

/**
    @function: void srecorder_resigter_jprobes(void)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
void srecorder_resigter_jprobes(void)
{
    int i = 0;
    int array_size = sizeof(s_srecorder_jprobe_operations) / sizeof(s_srecorder_jprobe_operations[0]);

    for (i = 0; i < array_size; i++)
    {
        if (NULL != s_srecorder_jprobe_operations[i].register_func)
        {
            s_srecorder_jprobe_operations[i].register_func(s_srecorder_jprobe_operations[i].p);
        }
    }
}

/**
    @function: void srecorder_unresigter_jprobes(void)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
void srecorder_unresigter_jprobes(void)
{
    int i = 0;
    int array_size = sizeof(s_srecorder_jprobe_operations) / sizeof(s_srecorder_jprobe_operations[0]);

    for (i = 0; i < array_size; i++)
    {
        if (NULL != s_srecorder_jprobe_operations[i].unregister_func)
        {
            s_srecorder_jprobe_operations[i].unregister_func(s_srecorder_jprobe_operations[i].p);
        }
    }
}
#endif
