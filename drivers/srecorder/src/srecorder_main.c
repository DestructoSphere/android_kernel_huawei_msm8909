/**
    @copyright: Huawei Technologies Co., Ltd. 2012-2012. All rights reserved.
    
    @file: srecorder_main.c
    
    @brief: 
    
    @version: 2.1.1 
    
    @author: Qi Dechun 00216641,    Yan Tongguang 00297150
    
    @date: 2015-03-13
    
    @history:
*/

#include <linux/module.h>

#include "srecorder_symbols.h"
#include "srecorder_dev.h"
#include "srecorder_log.h"
#include "srecorder_notifier.h"
#include "srecorder_jprobe.h"

static unsigned long params[3] = {0x0, 0x0, 0x0};
module_param_array(params, ulong, NULL, 0444);
MODULE_PARM_DESC(params, "SRecorder parameters");

/**
    @function: static int __init srecorder_init(void)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
static int __init srecorder_init(void)
{
    SRECORDER_PRINTK("Initialization %016lx %016lx.\n", params[0], params[1]);

    srecorder_init_symbols();

    srecorder_init_dev();

    srecorder_ioremap_dts_memory();

    srecorder_init_log_headers();

    srecorder_resigter_notifiers();

#ifdef CONFIG_KPROBES
    srecorder_resigter_jprobes();
#endif

    srecorder_enable_log_category_flags();

    srecorder_enable_log_type_flags();

    SRECORDER_PRINTK("Initialization done.\n");
    
    return 0;
}

/**
    @function: static void __exit srecorder_exit(void)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
static void __exit srecorder_exit(void)
{
    srecorder_iounremap_dts_memory();

    srecorder_unresigter_notifiers();

#ifdef CONFIG_KPROBES
    srecorder_unresigter_jprobes();
#endif
}

module_init(srecorder_init);
module_exit(srecorder_exit);
MODULE_LICENSE("GPL");
