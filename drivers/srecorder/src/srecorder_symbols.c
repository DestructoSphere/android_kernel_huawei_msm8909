/**
    @copyright: Huawei Technologies Co., Ltd. 2012-2012. All rights reserved.
    
    @file: srecorder_symbols.c
    
    @brief: 
    
    @version: 2.1.1 
    
    @author: Qi Dechun 00216641,    Yan Tongguang 00297150
    
    @date: 2015-03-13
    
    @history:
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/slab.h>
#include <linux/kallsyms.h>
#include <linux/version.h>

#include <asm/uaccess.h>
#include <asm/errno.h>

#include "srecorder_symbols.h"

#define KALLSYMS_LOOKUP_NAME_FUNC ("kallsyms_lookup_name")

typedef struct __syschk_sym_addr
{
    char *symbol_name;
    srec_ksym_addr_t *symbol_address;
}syschk_sym_addr;

typedef srec_ksym_addr_t (*kallsyms_lookup_name_func)(const char *name);

static kallsyms_lookup_name_func s_kallsyms_lookup_name = NULL;

#if defined(CONFIG_DUMP_SYS_INFO)
static srec_ksym_addr_t s_cpu_name = INVALID_KSYM_ADDR;
static srec_ksym_addr_t s_machine_name = INVALID_KSYM_ADDR;
static srec_ksym_addr_t s_all_bdevs = INVALID_KSYM_ADDR;
static srec_ksym_addr_t s_bdev_lock = INVALID_KSYM_ADDR;
#ifdef CONFIG_MMU
static srec_ksym_addr_t s_vmap_area_lock = INVALID_KSYM_ADDR;
#endif
#ifdef CONFIG_SWAP
static srec_ksym_addr_t s_nr_swapfiles = INVALID_KSYM_ADDR;
static srec_ksym_addr_t s_swap_info = INVALID_KSYM_ADDR;
static srec_ksym_addr_t s_swap_lock = INVALID_KSYM_ADDR;
#endif
#endif

#ifdef CONFIG_ARM_UNWIND
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0))
static srec_ksym_addr_t s___origin_unwind_idx = INVALID_KSYM_ADDR;
#endif
static srec_ksym_addr_t s_unwind_tables = INVALID_KSYM_ADDR;
static srec_ksym_addr_t s_unwind_lock = INVALID_KSYM_ADDR;
#endif

#if defined(CONFIG_DUMP_CURRENT_PS_BACKTRACE)
static srec_ksym_addr_t s_arch_vma_name = INVALID_KSYM_ADDR;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 38))
static srec_ksym_addr_t s_dcache_lock = INVALID_KSYM_ADDR;
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36))
static srec_ksym_addr_t s_vfsmount_lock_lock = INVALID_KSYM_ADDR;
#else
static srec_ksym_addr_t s_vfsmount_lock = INVALID_KSYM_ADDR;
#endif
#endif

#if defined(CONFIG_DUMP_SLAB_INFO)
#if defined(CONFIG_SLAB) || defined(CONFIG_SLUB)
static srec_ksym_addr_t s_slab_caches = INVALID_KSYM_ADDR;
static srec_ksym_addr_t s_slab_mutex = INVALID_KSYM_ADDR;
#endif
#endif

static syschk_sym_addr s_symbols_table[] = 
{      
#if defined(CONFIG_DUMP_SYS_INFO)
    {"cpu_name", &s_cpu_name}, 
    {"machine_name", &s_machine_name}, 
    {"all_bdevs", &s_all_bdevs}, 
    {"bdev_lock", &s_bdev_lock}, 
#ifdef CONFIG_MMU
    {"vmap_area_lock", &s_vmap_area_lock}, 
#endif
#ifdef CONFIG_SWAP
    {"nr_swapfiles", &s_nr_swapfiles}, 
    {"swap_info", &s_swap_info}, 
    {"swap_lock", &s_swap_lock}, 
#endif
#endif

#ifdef CONFIG_ARM_UNWIND
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0))
    {"__origin_unwind_idx", &s___origin_unwind_idx}, 
#endif
    {"unwind_tables", &s_unwind_tables}, 
    {"unwind_lock", &s_unwind_lock}, 
#endif

#if defined(CONFIG_DUMP_CURRENT_PS_BACKTRACE)
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36))
    {"vfsmount_lock_lock", &s_vfsmount_lock_lock}, 
#else
    {"vfsmount_lock", &s_vfsmount_lock}, 
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 38))
    {"dcache_lock", &s_dcache_lock}, 
#endif

    {"arch_vma_name", &s_arch_vma_name}, 
#endif

#if defined(CONFIG_DUMP_SLAB_INFO)
#if defined(CONFIG_SLAB) || defined(CONFIG_SLUB)
    {"slab_caches", &s_slab_caches}, 
    {"slab_mutex", &s_slab_mutex}, 
#endif
#endif
};

#if defined(CONFIG_DUMP_SYS_INFO) && !(defined(CONFIG_DEBUG_KERNEL) && defined(CONFIG_KALLSYMS_ALL))
extern unsigned long get_cpu_name(void);
extern unsigned long get_machine_name(void);
extern unsigned long get_all_bdevs(void);
extern unsigned long get_bdev_lock(void);
extern unsigned long get_vmap_area_lock(void);
extern unsigned long get_nr_swapfiles(void);
extern unsigned long get_swap_lock(void);
extern unsigned long get_swap_info(void);
#endif

#if defined(CONFIG_DUMP_SLAB_INFO) && !(defined(CONFIG_DEBUG_KERNEL) && defined(CONFIG_KALLSYMS_ALL))
#if defined(CONFIG_SLAB) || defined(CONFIG_SLUB)
extern unsigned long get_slab_caches(void);
extern unsigned long get_slab_mutex(void);
#endif
#endif

static int srecorder_get_other_symbols(void);

#ifndef CONFIG_KALLSYMS
static int srecorder_get_symbol_kallsyms_lookup_name(const char *line, int len);
#endif

#if defined(CONFIG_DUMP_SYS_INFO)
/**
    @function: srec_ksym_addr_t srecorder_get_cpu_name(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_cpu_name(void)
{
#if !(defined(CONFIG_DEBUG_KERNEL) && defined(CONFIG_KALLSYMS_ALL))
    s_cpu_name = get_cpu_name();
#endif
    
    return s_cpu_name;
}

/**
    @function: srec_ksym_addr_t srecorder_get_machine_name(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_machine_name(void)
{
#if !(defined(CONFIG_DEBUG_KERNEL) && defined(CONFIG_KALLSYMS_ALL))
    s_machine_name = get_machine_name();
#endif
    
    return s_machine_name;
}

/**
    @function: srec_ksym_addr_t srecorder_get_all_bdevs(void)
    @brief: 
    @param: none
    @return:
    @note: 
**/
srec_ksym_addr_t srecorder_get_all_bdevs(void)
{
#if !(defined(CONFIG_DEBUG_KERNEL) && defined(CONFIG_KALLSYMS_ALL))
    s_all_bdevs = get_all_bdevs();
#endif
    
    return s_all_bdevs;
}

/**
    @function: srec_ksym_addr_t srecorder_get_bdev_lock(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_bdev_lock(void)
{
#if !(defined(CONFIG_DEBUG_KERNEL) && defined(CONFIG_KALLSYMS_ALL))
    s_bdev_lock = get_bdev_lock();
#endif
    
    return s_bdev_lock;
}

#ifdef CONFIG_MMU
/**
    @function: srec_ksym_addr_t srecorder_get_vmap_area_lock(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_vmap_area_lock(void)
{
#if !(defined(CONFIG_DEBUG_KERNEL) && defined(CONFIG_KALLSYMS_ALL))
    s_vmap_area_lock = get_vmap_area_lock();
#endif
    
    return s_vmap_area_lock;
}
#endif

#ifdef CONFIG_SWAP
/**
    @function: srec_ksym_addr_t srecorder_get_nr_swapfiles(void)
    @brief: 

    @param: none
    
    @return: 

    @note: 
**/
srec_ksym_addr_t srecorder_get_nr_swapfiles(void)
{
#if !(defined(CONFIG_DEBUG_KERNEL) && defined(CONFIG_KALLSYMS_ALL))
    s_nr_swapfiles = get_nr_swapfiles();
#endif

    return s_nr_swapfiles;
}

/**
    @function: srec_ksym_addr_t srecorder_get_swap_info(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_swap_info(void)
{
#if !(defined(CONFIG_DEBUG_KERNEL) && defined(CONFIG_KALLSYMS_ALL))
    s_swap_info = get_swap_info();
#endif

    return s_swap_info;
}

/**
    @function: srec_ksym_addr_t srecorder_get_swap_lock(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_swap_lock(void)
{
#if !(defined(CONFIG_DEBUG_KERNEL) && defined(CONFIG_KALLSYMS_ALL))
    s_swap_lock = get_swap_lock();
#endif

    return s_swap_lock;
}
#endif
#endif

#if defined(CONFIG_DUMP_CURRENT_PS_BACKTRACE)
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36))
/**
    @function: srec_ksym_addr_t srecorder_get_vfsmount_lock_lock(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_vfsmount_lock_lock(void)
{
    return s_vfsmount_lock_lock;
}
#else
/**
    @function: srec_ksym_addr_t srecorder_get_vfsmount_lock(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_vfsmount_lock(void)
{
    return s_vfsmount_lock;
}
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 38))
/**
    @function: srec_ksym_addr_t srecorder_get_dcache_lock(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_dcache_lock(void)
{
    return s_dcache_lock;
}
#else
/**
    @function: srec_ksym_addr_t srecorder_get_rename_lock(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_rename_lock(void)
{
    return (srec_ksym_addr_t)&rename_lock;
}
#endif

/**
    @function: srec_ksym_addr_t srecorder_get_arch_vma_name(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_arch_vma_name(void)
{
    return s_arch_vma_name;
}
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
srec_ksym_addr_t srecorder_get___origin_unwind_idx(void)
{
    return s___origin_unwind_idx;
}
#endif

/**
    @function: srec_ksym_addr_t srecorder_get_unwind_tables(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_unwind_tables(void)
{
    return s_unwind_tables;
}

/**
    @function: srec_ksym_addr_t srecorder_get_unwind_lock(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
srec_ksym_addr_t srecorder_get_unwind_lock(void)
{
    return s_unwind_lock;
}
#endif

#if defined(CONFIG_DUMP_SLAB_INFO)
#if defined(CONFIG_SLAB) || defined(CONFIG_SLUB)
/**
    @function: srec_ksym_addr_t srecorder_get_slab_caches(void)
    @brief: 
    @param: none
    @return: slab_caches
    @note: 
**/
srec_ksym_addr_t srecorder_get_slab_caches(void)
{
#if !(defined(CONFIG_DEBUG_KERNEL) && defined(CONFIG_KALLSYMS_ALL))
    s_slab_caches = get_slab_caches();
#endif
    
    return s_slab_caches;
}

/**
    @function: srec_ksym_addr_t srecorder_get_slab_mutex(void)
    @brief: 
    @param: none
    @return: slab_mutex
    @note: 
**/
srec_ksym_addr_t srecorder_get_slab_mutex(void)
{
#if !(defined(CONFIG_DEBUG_KERNEL) && defined(CONFIG_KALLSYMS_ALL))
    s_slab_mutex = get_slab_mutex();
#endif
    
    return s_slab_mutex;
}
#endif
#endif

/**
    @function: static int srecorder_get_other_symbols(void)
    @brief: 
    @param: none
    @return: 
    @note: 
**/
static int srecorder_get_other_symbols(void)
{
    int i = 0;
    int symbols_table_size = sizeof(s_symbols_table) / sizeof(s_symbols_table[0]);  

    for (i = 0; i < symbols_table_size; i++)
    {
        if (unlikely(NULL == s_symbols_table[i].symbol_name || NULL == s_symbols_table[i].symbol_address))
        {
            continue;
        }
        
        *(s_symbols_table[i].symbol_address) = s_kallsyms_lookup_name(s_symbols_table[i].symbol_name);
        if (INVALID_KSYM_ADDR == *(s_symbols_table[i].symbol_address))
        {
            SRECORDER_PRINTK("Get %s failed.\n", s_symbols_table[i].symbol_name);
            continue;
        }
    }
    
    return 0;
}

#ifndef CONFIG_KALLSYMS
/**
    @function: static int srecorder_get_symbol_kallsyms_lookup_name(const char *line, int len)
    @brief: 
    @param: line
    @param: len
    @return: 
    @note: 
**/
static int srecorder_get_symbol_kallsyms_lookup_name(const char *line, int len)
{
    int ret = 0;
    srec_ksym_addr_t address;    
    char *psymbolname = NULL;
    char *pmodulename = NULL;
    char dummy;

    if (NULL == line)
    {
        SRECORDER_PRINTK("File [%s] line [%d] invalid param!\n", __FILE__, __LINE__);
        return -EINVAL;
    }

    psymbolname = kmalloc(NAME_MAX, GFP_KERNEL);
    if (NULL == psymbolname)
    {
        ret = -ENOMEM;
        goto err_out;
    }

    pmodulename = kmalloc(NAME_MAX, GFP_KERNEL);
    if (NULL == pmodulename)
    {        
        ret = -ENOMEM;
        goto err_out;
    }

    memset(psymbolname, 0, NAME_MAX);
    memset(pmodulename, 0, NAME_MAX);

    if (3 != sscanf(line, "%lx %c %s [%s]", &address, &dummy, psymbolname, pmodulename))
    {
        ret = -ENOENT;
        goto err_out;
    }

    if (strncmp(KALLSYMS_LOOKUP_NAME_FUNC, psymbolname, NAME_MAX) != 0)
    {
        ret = -ENOENT;
        goto err_out;
    }

    s_kallsyms_lookup_name =  (kallsyms_lookup_name_func)address;
    ret = 0;

err_out:
    if (NULL != psymbolname)
    {
        kfree(psymbolname);
        psymbolname = NULL;
    }

    if (NULL != pmodulename)
    {
        kfree(pmodulename);
        pmodulename = NULL;
    }

    return ret;
}
#endif

/**
    @function: int srecorder_init_symbols()
    @brief: 
    @param: pinit_params
    @return: 
    @note: 
**/
int srecorder_init_symbols(void)
{
    int ret;
    
#ifndef CONFIG_KALLSYMS
    int linelen = 0;
    int readlen = 0;
    int i = 0;
    int symbol_found = 0; 
    char *szbuff = NULL;
    char *szLine = NULL;
    struct file *file = NULL;
    mm_segment_t old_fs;

    szbuff = kmalloc(PAGE_SIZE, GFP_KERNEL);
    if (NULL == szbuff)
    {
        ret = -ENOMEM;
        goto err_out;
    }

    szLine = kmalloc(PAGE_SIZE, GFP_KERNEL);
    if (NULL == szLine)
    {
        ret = -ENOMEM;
        goto err_out;
    }

    file = filp_open("/proc/kallsyms", O_RDONLY, 0);
    if (IS_ERR(file))
    {
        ret = PTR_ERR(file);
        goto err_out;
    }

    if (!S_ISREG(file->f_path.dentry->d_inode->i_mode))
    {
        ret = -EACCES;
        goto err_out;
    }

    if (NULL == file->f_op->read)
    {
        ret = -EIO;
        goto err_out;
    }

    old_fs = get_fs();
    set_fs(get_ds());
    while ((readlen = file->f_op->read(file, (char*)szbuff, PAGE_SIZE - 1, &file->f_pos)) > 0
        && (0 == symbol_found))
    {
        for (i = 0; i < readlen; i++)
        {
            if (linelen >= PAGE_SIZE - 1)
            {
                linelen = 0;
            }

            szLine[linelen] = szbuff[i];
            if (szLine[linelen] == '\n' )
            {
                szLine[linelen + 1] = 0;
                ret = srecorder_get_symbol_kallsyms_lookup_name(szLine, linelen + 1);
                linelen = 0;

                if (0 == ret)
                {
                    symbol_found = 1;
                    break;
                }
                continue;
            }

            linelen++;
        }
    }

    if (NULL == (void *)s_kallsyms_lookup_name)
    {
        SRECORDER_PRINTK("can not find address for %s\n", KALLSYMS_LOOKUP_NAME_FUNC);
        ret = -EFAULT;
        goto err_out;
    }

    ret = srecorder_get_other_symbols();

    filp_close(file, NULL);
    file = NULL;
    set_fs(old_fs);

err_out:
    if (NULL != file)
    {
        filp_close(file, NULL);
    }

    if (NULL != szbuff)
    {
        kfree(szbuff);
    }

    if (NULL != szLine)
    {
        kfree(szLine);
    }
#else
    s_kallsyms_lookup_name = kallsyms_lookup_name;
#endif
    ret = srecorder_get_other_symbols();
    
    return ret;
}
