/**
    @copyright: Huawei Technologies Co., Ltd. 2012-2012. All rights reserved.
    
    @file: srecorder_sys_info.c
    
    @brief: 
    
    @version: 2.1.1 
    
    @author: Qi Dechun 00216641,    Yan Tongguang 00297150
    
    @date: 2015-03-13
    
    @history:
*/

#include <linux/stddef.h>
#include <linux/swap.h>
#include <linux/slab.h>
#include <linux/utsname.h>
#include <linux/blkdev.h>
#include <linux/jiffies.h>
#include <linux/hugetlb.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/vmalloc.h>
#include <linux/highmem.h>
#include <linux/version.h>
#include <linux/mman.h>

#include <asm/pgtable.h>
#include <asm/uaccess.h>

#include "srecorder_symbols.h"
#include "srecorder_sys_info.h"

typedef struct __vmalloc_info 
{
    unsigned long used;
    unsigned long largest_chunk;
} vmalloc_info;

#ifdef CONFIG_MMU
#define VMALLOC_TOTAL (VMALLOC_END - VMALLOC_START)
#else
#define VMALLOC_TOTAL 0UL
#define get_vmalloc_info(vmi)            \
do                        \
{                        \
    (vmi)->used = 0;            \
    (vmi)->largest_chunk = 0;        \
} while(0)
#endif

#ifdef CONFIG_SWAP
static void srecorder_si_swapinfo(struct sysinfo *val);
#endif

#ifdef CONFIG_MMU
static void srecorder_get_vmalloc_info(vmalloc_info *vmi);
#endif
static long srecorder_nr_blockdev_pages(void);
static void srecorder_si_meminfo(struct sysinfo *val);

static int sys_info_flag = 0;

/**
    @function: void srecorder_enable_sys_info(void)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_enable_sys_info(void)
{
    sys_info_flag = 1;
}

/**
    @function: void srecorder_disable_sys_info(void)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_disable_sys_info(void)
{
    sys_info_flag = 0;
}

#ifdef CONFIG_MMU
static struct vm_struct *vmlist;

/**
    @function: static void srecorder_get_vmalloc_info(vmalloc_info *vmi)
    @brief: 
    @return: 
    @note: 
**/
static void srecorder_get_vmalloc_info(vmalloc_info *vmi)
{
    struct vm_struct *vma;
    unsigned long free_area_size = 0;
    unsigned long prev_end = 0;

    if (unlikely(NULL == vmi))
    {
        return;
    }
    
    vmi->used = 0;
    if (NULL == vmlist) 
    {
        vmi->largest_chunk = VMALLOC_TOTAL;
    }
    else 
    {
        vmi->largest_chunk = 0;

        prev_end = VMALLOC_START;

        if (spin_trylock((spinlock_t *)srecorder_get_vmap_area_lock()))
        {
            for (vma = vmlist; NULL != vma; vma = vma->next)
            {
                unsigned long addr = (unsigned long)vma->addr;

                /*
            * Some archs keep another range for modules in vmlist
            */
                if (addr < VMALLOC_START)
                {
                    continue;
                }
                
                if (addr >= VMALLOC_END)
                {
                    break;
                }

                vmi->used += vma->size;

                free_area_size = addr - prev_end;
                if (vmi->largest_chunk < free_area_size)
                {
                    vmi->largest_chunk = free_area_size;
                }

                prev_end = vma->size + addr;
            }

            if (VMALLOC_END - prev_end > vmi->largest_chunk)
            {
                vmi->largest_chunk = VMALLOC_END - prev_end;
            }

            spin_unlock((spinlock_t *)srecorder_get_vmap_area_lock());
        }
    }
}
#endif

#ifdef CONFIG_SWAP
/**
    @function: static void srecorder_si_swapinfo(struct sysinfo *val)
    @brief: 
    @return: 
    @note: 
**/
static void srecorder_si_swapinfo(struct sysinfo *val)
{
    unsigned int type;
    unsigned long nr_to_be_unused = 0;
    unsigned int nr_swapfiles;
    struct swap_info_struct **swap_info = NULL;

    if (unlikely(NULL == val))
    {
        return;
    }

    nr_swapfiles = *(unsigned int *)srecorder_get_nr_swapfiles();
    swap_info = (struct swap_info_struct **)srecorder_get_swap_info();

    if (spin_trylock((spinlock_t *)srecorder_get_swap_lock()))
    {
        for (type = 0; type < nr_swapfiles; type++) 
        {
            struct swap_info_struct *si = swap_info[type];

            if ((si->flags & SWP_USED) && !(si->flags & SWP_WRITEOK))
            {
                nr_to_be_unused += si->inuse_pages;
            }
        }
        val->freeswap = atomic_long_read(&nr_swap_pages) + nr_to_be_unused;
        val->totalswap = total_swap_pages + nr_to_be_unused;
        spin_unlock((spinlock_t *)srecorder_get_swap_lock());
    }
}
#endif

/**
    @function: static long srecorder_nr_blockdev_pages(void)
    @brief: 
    @return: 
    @note: 
**/
static long srecorder_nr_blockdev_pages(void)
{
    struct block_device *bdev;
    long ret = 0;
    
    if (spin_trylock((spinlock_t *)srecorder_get_bdev_lock()))
    {
        list_for_each_entry(bdev, (struct list_head *)srecorder_get_all_bdevs(), bd_list)
        {
            ret += bdev->bd_inode->i_mapping->nrpages;
        }
        spin_unlock((spinlock_t *)srecorder_get_bdev_lock());
    }
    
    return ret;
}

/**
    @function: static void srecorder_si_meminfo(struct sysinfo *val)
    @brief: 
    @return: 
    @note: 
**/
static void srecorder_si_meminfo(struct sysinfo *val)
{
    val->totalram = totalram_pages;
    val->sharedram = 0;
    val->freeram = global_page_state(NR_FREE_PAGES);
    val->bufferram = srecorder_nr_blockdev_pages();
    val->totalhigh = totalhigh_pages;
    val->freehigh = nr_free_highpages();
    val->mem_unit = PAGE_SIZE;
}

/**
    @function: int srecorder_dump_sys_info()
    @brief: 
    @return: 
    @note: 
**/
int srecorder_dump_sys_info(void)
{
    struct sysinfo si;
    unsigned long committed;
    unsigned long allowed;
    vmalloc_info vmi;
    long cached;
    unsigned long pages[NR_LRU_LISTS];
    int lru;
    char *cpu_name = NULL;
    char *machine_name = NULL;

    if (sys_info_flag == 0)
    {
        SRECORDER_PRINTK("The dump flag of sys info isn't enabled\n");
        return -1;
    }

    if (unlikely(INVALID_KSYM_ADDR == srecorder_get_cpu_name()
        || INVALID_KSYM_ADDR == srecorder_get_machine_name()
#ifdef CONFIG_SWAP
        || INVALID_KSYM_ADDR == srecorder_get_nr_swapfiles()
        || INVALID_KSYM_ADDR == srecorder_get_swap_info()
        || INVALID_KSYM_ADDR == srecorder_get_swap_lock()
#endif
        || INVALID_KSYM_ADDR == srecorder_get_bdev_lock() 
        || INVALID_KSYM_ADDR == srecorder_get_all_bdevs()))
    {
        SRECORDER_PRINTK("File [%s] line [%d] invalid param or kernel symbol addr!\n", __FILE__, __LINE__);
        return -1;
    }

    srecorder_log_header_l_start(TYPE_SYS_INFO);

    srecorder_dump_log_title(TYPE_SYS_INFO);

    cpu_name = (char *)(*(srec_ksym_addr_t *)srecorder_get_cpu_name());
    machine_name = (char *)(*(srec_ksym_addr_t *)srecorder_get_machine_name());

    memset(&si, 0, sizeof(struct sysinfo));
    srecorder_si_meminfo(&si);
    
#ifdef CONFIG_SWAP
    srecorder_si_swapinfo(&si);
#else
    si_swapinfo(&si);
#endif

    committed = percpu_counter_read_positive(&vm_committed_as);
    allowed = ((totalram_pages - hugetlb_total_pages()) * sysctl_overcommit_ratio / 100) + total_swap_pages;
    cached = global_page_state(NR_FILE_PAGES) - total_swapcache_pages() - si.bufferram;
    if (cached < 0)
    {
        cached = 0;
    }

    memset(&vmi, 0, sizeof(vmalloc_info)); 
#ifdef CONFIG_MMU
    srecorder_get_vmalloc_info(&vmi);
#else
    get_vmalloc_info(&vmi);
#endif

    for (lru = LRU_BASE; lru < NR_LRU_LISTS; lru++)
    {
        pages[lru] = global_page_state(NR_LRU_BASE + lru);
    }

    SRECORDER_SNPRINTF("sysname: %s\nrelease: %s\nversion: %s\n"
        "Processor: %s\nHardware: %s\n"
        "jiffies: %llu\n\n"
        "MemTotal:       %8lu kB\n"
        "MemFree:        %8lu kB\n"
        "Buffers:        %8lu kB\n"
        "Cached:         %8lu kB\n"
        "SwapCached:     %8lu kB\n"
        "Active:         %8lu kB\n"
        "Inactive:       %8lu kB\n"
        "Active(anon):   %8lu kB\n"
        "Inactive(anon): %8lu kB\n"
        "Active(file):   %8lu kB\n"
        "Inactive(file): %8lu kB\n"
        "Unevictable:    %8lu kB\n"
        "Mlocked:        %8lu kB\n"
#ifdef CONFIG_HIGHMEM
        "HighTotal:      %8lu kB\n"
        "HighFree:       %8lu kB\n"
        "LowTotal:       %8lu kB\n"
        "LowFree:        %8lu kB\n"
#endif
#ifndef CONFIG_MMU
        "MmapCopy:       %8lu kB\n"
#endif
        "SwapTotal:      %8lu kB\n"
        "SwapFree:       %8lu kB\n"
        "Dirty:          %8lu kB\n"
        "Writeback:      %8lu kB\n"
        "AnonPages:      %8lu kB\n"
        "Mapped:         %8lu kB\n"
        "Shmem:          %8lu kB\n"
        "Slab:           %8lu kB\n"
        "SReclaimable:   %8lu kB\n"
        "SUnreclaim:     %8lu kB\n"
        "KernelStack:    %8lu kB\n"
        "PageTables:     %8lu kB\n"
#ifdef CONFIG_QUICKLIST
        "Quicklists:     %8lu kB\n"
#endif
        "NFS_Unstable:   %8lu kB\n"
        "Bounce:         %8lu kB\n"
        "WritebackTmp:   %8lu kB\n"
        "CommitLimit:    %8lu kB\n"
        "Committed_AS:   %8lu kB\n"
        "VmallocTotal:   %8lu kB\n"
        "VmallocUsed:    %8lu kB\n"
        "VmallocChunk:   %8lu kB\n"
#ifdef CONFIG_MEMORY_FAILURE
        "HardwareCorrupted: %5lu kB\n"
#endif
#ifdef CONFIG_TRANSPARENT_HUGEPAGE
        "AnonHugePages:  %8lu kB\n"
#endif
        ,
        init_uts_ns.name.sysname, init_uts_ns.name.release, init_uts_ns.name.version, 
        (NULL == cpu_name) ? "unknown" : cpu_name, (NULL == machine_name) ? "unknown" : machine_name, 
        jiffies_64 - INITIAL_JIFFIES, 
        K(si.totalram),
        K(si.freeram),
        K(si.bufferram),
        K(cached),
        K(total_swapcache_pages()),
        K(pages[LRU_ACTIVE_ANON] + pages[LRU_ACTIVE_FILE]),
        K(pages[LRU_INACTIVE_ANON] + pages[LRU_INACTIVE_FILE]),
        K(pages[LRU_ACTIVE_ANON]),
        K(pages[LRU_INACTIVE_ANON]),
        K(pages[LRU_ACTIVE_FILE]),
        K(pages[LRU_INACTIVE_FILE]),
        K(pages[LRU_UNEVICTABLE]),
        K(global_page_state(NR_MLOCK)),
#ifdef CONFIG_HIGHMEM
        K(si.totalhigh),
        K(si.freehigh),
        K(si.totalram - si.totalhigh),
        K(si.freeram - si.freehigh),
#endif
#ifndef CONFIG_MMU
        K((unsigned long) atomic_long_read(&mmap_pages_allocated)),
#endif
        K(si.totalswap),
        K(si.freeswap),
        K(global_page_state(NR_FILE_DIRTY)),
        K(global_page_state(NR_WRITEBACK)),
#ifdef CONFIG_TRANSPARENT_HUGEPAGE
        K(global_page_state(NR_ANON_PAGES) + global_page_state(NR_ANON_TRANSPARENT_HUGEPAGES) * HPAGE_PMD_NR),
#else
        K(global_page_state(NR_ANON_PAGES)),
#endif
        K(global_page_state(NR_FILE_MAPPED)),
        K(global_page_state(NR_SHMEM)),
        K(global_page_state(NR_SLAB_RECLAIMABLE) + global_page_state(NR_SLAB_UNRECLAIMABLE)),
        K(global_page_state(NR_SLAB_RECLAIMABLE)),
        K(global_page_state(NR_SLAB_UNRECLAIMABLE)),
        global_page_state(NR_KERNEL_STACK) * THREAD_SIZE / 1024,
        K(global_page_state(NR_PAGETABLE)),
#ifdef CONFIG_QUICKLIST
        K(quicklist_total_size()),
#endif
        K(global_page_state(NR_UNSTABLE_NFS)),
        K(global_page_state(NR_BOUNCE)),
        K(global_page_state(NR_WRITEBACK_TEMP)),
        K(allowed),
        K(committed),
        (unsigned long)VMALLOC_TOTAL >> 10,
        vmi.used >> 10,
        vmi.largest_chunk >> 10
#ifdef CONFIG_MEMORY_FAILURE
        , atomic_long_read(&mce_bad_pages) << (PAGE_SHIFT - 10)
#endif
#ifdef CONFIG_TRANSPARENT_HUGEPAGE
        , K(global_page_state(NR_ANON_TRANSPARENT_HUGEPAGES) * HPAGE_PMD_NR)
#endif
        );

    srecorder_log_header_l_end();
    
    return 0;
}
