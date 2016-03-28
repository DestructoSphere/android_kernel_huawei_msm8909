/**
    @copyright: Huawei Technologies Co., Ltd. 2012-2012. All rights reserved.
    
    @file: srecorder_symbols.h
    
    @brief: 
    
    @version: 2.1.1 
    
    @author: Qi Dechun 00216641,    Yan Tongguang 00297150
    
    @date: 2015-03-13
    
    @history:
**/

#ifndef SRECORDER_SYMBOLS_H
#define SRECORDER_SYMBOLS_H

#include <linux/version.h>

#include "srecorder_misc.h"
#include "srecorder_log.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CONFIG_DUMP_SYS_INFO)
/**
    @function: srec_ksym_addr_t srecorder_get_cpu_name(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_cpu_name(void);

/**
    @function: srec_ksym_addr_t srecorder_get_machine_name(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_machine_name(void);

/**
    @function: srec_ksym_addr_t srecorder_get_all_bdevs(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_all_bdevs(void);

/**
    @function: srec_ksym_addr_t srecorder_get_bdev_lock(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_bdev_lock(void);

#ifdef CONFIG_MMU
/**
    @function: srec_ksym_addr_t srecorder_get_bdev_lock(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_vmap_area_lock(void);
#endif

#ifdef CONFIG_SWAP
/**
    @function: srec_ksym_addr_t srecorder_get_nr_swapfiles(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_nr_swapfiles(void);

/**
    @function: srec_ksym_addr_t srecorder_get_swap_info(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_swap_info(void);

/**
    @function: srec_ksym_addr_t srecorder_get_swap_lock(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_swap_lock(void);
#endif
#endif

/**
    @function: srec_ksym_addr_t srecorder_get_register_jprobe(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_register_jprobe(void);

/**
    @function: srec_ksym_addr_t srecorder_get_unregister_jprobe(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_unregister_jprobe(void);

/**
    @function: srec_ksym_addr_t srecorder_get_jprobe_return(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_jprobe_return(void);


#if defined(CONFIG_DUMP_CURRENT_PS_BACKTRACE)
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36))
/**
    @function: srec_ksym_addr_t srecorder_get_vfsmount_lock_lock(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_vfsmount_lock_lock(void);
#else
/**
    @function: srec_ksym_addr_t srecorder_get_vfsmount_lock(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_vfsmount_lock(void);
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 38))
/**
    @function: srec_ksym_addr_t srecorder_get_dcache_lock(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_dcache_lock(void);
#else
/**
    @function: srec_ksym_addr_t srecorder_get_rename_lock(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_rename_lock(void);
#endif

/**
    @function: srec_ksym_addr_t srecorder_get_arch_vma_name(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_arch_vma_name(void);
#endif

#ifdef CONFIG_ARM_UNWIND
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0))
/**
    @function: srec_ksym_addr_t srecorder_get___origin_unwind_idx(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get___origin_unwind_idx(void);
#endif

/**
    @function: srec_ksym_addr_t srecorder_get_unwind_tables(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_unwind_tables(void);

/**
    @function: srec_ksym_addr_t srecorder_get_unwind_lock(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_unwind_lock(void);
#endif

/**
    @function: srec_ksym_addr_t srecorder_get_atomic_notifier_chain_register(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_atomic_notifier_chain_register(void);

/**
    @function: srec_ksym_addr_t srecorder_get_atomic_notifier_chain_unregister(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_atomic_notifier_chain_unregister(void);

#if defined(CONFIG_DUMP_SLAB_INFO)
#if defined(CONFIG_SLAB) || defined(CONFIG_SLUB)
/**
    @function: srec_ksym_addr_t srecorder_get_slab_caches(void)
    @brief: 
    @param: none
    @return: slab_caches
    @note: 
**/
srec_ksym_addr_t srecorder_get_slab_caches(void);

/**
    @function: srec_ksym_addr_t srecorder_get_slab_mutex(void)
    @brief: 
    @param: none
    @return: slab_mutex
    @note: 
**/
srec_ksym_addr_t srecorder_get_slab_mutex(void);
#endif
#endif

/**
    @function: int srecorder_init_symbols()
    @brief: 
    @param: pinit_params
    @return: 
    @note: 
**/
int srecorder_init_symbols(void);

#ifdef __cplusplus
}
#endif
#endif /* SRECORDER_SYMBOLS_H */
