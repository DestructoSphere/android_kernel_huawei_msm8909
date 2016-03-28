/**
    @copyright: Huawei Technologies Co., Ltd. 2012-2012. All rights reserved.
    
    @file: srecorder_allps_info.c
    
    @brief: 
    
    @version: 2.1.1 
    
    @author: Qi Dechun 00216641,    Yan Tongguang 00297150
    
    @date: 2015-03-13
    
    @history:
**/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/fdtable.h>
#include <linux/version.h>

#include <asm/uaccess.h>

#include "srecorder_ps_info.h"
#include "srecorder_symbols.h"
#include "srecorder_log.h"


#define TOP_RSS_NUM 20

typedef struct srecorder_mem_stat
{
    struct task_struct *p;
    unsigned long total_vm;
    unsigned long task_rss_size;
} srecorder_mem_stat_t;

static const char *s_process_state[] = 
{
    "R (running)",        /*   0 */
    "S (sleeping)",       /*   1 */
    "D (disk sleep)",     /*   2 */
    "T (stopped)",        /*   4 */
    "t (tracing stop)",   /*   8 */
    "Z (zombie)",         /*  16 */
    "X (dead)",           /*  32 */
    "x (dead)",           /*  64 */
    "K (wakekill)",       /* 128 */
    "W (waking)",         /* 256 */
};

static void srecorder_write_allps_header(void);
static void srecorder_show_single_taskinfo(struct task_struct *ptask, unsigned int mask);
static void srecorder_show_all_taskinfo(unsigned int mask);
static int srecorder_count_open_files(struct fdtable *fdt);
static inline const char *srecorder_get_task_state(struct task_struct *ptsk);

static int ps_info_flag = 0;

/**
    @function: void srecorder_enable_ps_info(void)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_enable_ps_info(void)
{
    ps_info_flag = 1;
}

/**
    @function: void srecorder_disable_ps_info(void)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_disable_ps_info(void)
{
    ps_info_flag = 0;
}

/**
    @function:static void srecorder_show_all_taskinfo( unsigned int mask)
    @brief: 
    @param: mask 
    @return: none
    @note: 
**/
static void srecorder_show_all_taskinfo(unsigned int mask)
{
    struct task_struct *pcur = NULL;

    if (read_trylock(&tasklist_lock))
    {
        for_each_process(pcur)
        {
            if (spin_trylock(&(pcur->alloc_lock)))
            {
                srecorder_show_single_taskinfo( pcur, 0); 
                spin_unlock(&(pcur->alloc_lock));
            }
        }

        read_unlock(&tasklist_lock);
    }
}

/**
    @function: static int srecorder_count_open_files(struct fdtable *fdt)
    @brief: 
    @param: fdt  
    @return: none
    @note: 
**/
static int srecorder_count_open_files(struct fdtable *fdt)
{
    int size = 0;
    int i;

    if (NULL == fdt)
    {
        SRECORDER_PRINTK("File [%s] line [%d] invalid param!\n", __FILE__, __LINE__);
        return 0;
    }

    size = fdt->max_fds;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0))
    /* Find the last open fd */
    for (i = size / (8 * sizeof(long)); i > 0; ) /*8bit*/ 
    {
        if (fdt->open_fds->fds_bits[--i])
        {
            break;
        }
    }
    
    i = (i + 1) * 8 * sizeof(long); /*8bit*/ 
#else
    /* Find the last open fd */
    for (i = size / BITS_PER_LONG; i > 0; )
    {
        if (fdt->open_fds[--i])
        {
            break;
        }
    }
    
    i = (i + 1) * BITS_PER_LONG;
#endif
 
    return i;
}

/**
    @function: static inline const char *srecorder_get_task_state(struct task_struct *tsk)
    @brief: 
    @param: ptsk 
    @return: none
    @note: 
**/
static inline const char *srecorder_get_task_state(struct task_struct *ptsk)
{
    unsigned int state = (ptsk->state & TASK_REPORT) | ptsk->exit_state;
    const char * const *p = &s_process_state[0];

    while (0 != state) 
    {
        p++;
        state >>= 1;
    }
    
    return *p;
}

/**
    @function: static void srecorder_show_single_taskinfo( 
        struct task_struct *ptask, unsigned int mask)
    @brief: 
    @param: pbuf 
    @param: ptask 
    @param: mask 
    @return: none
    @note: 
**/
static void srecorder_show_single_taskinfo(struct task_struct *ptask, unsigned int mask)
{
    int open_files = 0;
    struct fdtable *old_fdt = NULL;

    if (NULL == ptask)
    {
        SRECORDER_PRINTK("File [%s] line [%d] invalid param!\n", __FILE__, __LINE__);
        return;
    }

    if (NULL != ptask->files)
    {
        if (spin_trylock(&ptask->files->file_lock))
        {
            old_fdt = files_fdtable(ptask->files);
            open_files = srecorder_count_open_files(old_fdt);
            spin_unlock(&ptask->files->file_lock);
        }
    }
    else
    {
        open_files = 0;
    }
    
    /*"pid ppid tgid cpuid state vmm rss files name"*/
    SRECORDER_SNPRINTF("%5d %6d %6d %d %s %8lu KB %8lu KB %d %s\n",  
        ptask->pid, 
        ptask->parent->pid, 
        ptask->tgid, 
        task_cpu(ptask), 
        srecorder_get_task_state(ptask),
        (NULL == ptask->mm) ? (0) : (K(ptask->mm->total_vm)), 
        (NULL == ptask->mm) ? (0) : (K(get_mm_rss(ptask->mm))),
        open_files, 
        ptask->comm);

}

/**
    @function: static void srecorder_write_allps_header()
    @brief: 
    @return: none
    @note: 
**/
static void srecorder_write_allps_header(void)
{
    SRECORDER_SNPRINTF("%s", "pid    ppid    tgid  cpuid  state        vmm         rss     files    name\n");
}

/**
    @function: int srecorder_dump_ps_info(void)
    @brief: 
    @return: 
    @note: 
**/
int srecorder_dump_ps_info(void)
{
    if (ps_info_flag == 0)
    {
        SRECORDER_PRINTK("The dump flag of ps info isn't enabled\n");
        return -1;
    }

    srecorder_log_header_l_start(TYPE_PS_INFO);

    srecorder_dump_log_title(TYPE_PS_INFO);

    srecorder_write_allps_header();
    srecorder_show_all_taskinfo( 0);

    srecorder_log_header_l_end();
    
    return 0;
}
