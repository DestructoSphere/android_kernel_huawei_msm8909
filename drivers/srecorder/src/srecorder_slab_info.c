/**
    @copyright: Huawei Technologies Co., Ltd. 2012-2012. All rights reserved.
    
    @file: srecorder_slabinfo.c
    
    @brief: 
    
    @version: 2.1.1 
    
    @author: Qi Dechun 00216641,    Yan Tongguang 00297150
    
    @date: 2015-03-13
    
    @history:
*/

#include <linux/stddef.h>
#include <linux/seq_file.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/version.h>

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 0, 8))
#include <linux/swap.h> /* struct reclaim_state */
#include <linux/bit_spinlock.h>
#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/kmemtrace.h>
#include <linux/kmemcheck.h>
#include <linux/cpu.h>
#include <linux/cpuset.h>
#include <linux/mempolicy.h>
#include <linux/ctype.h>
#include <linux/debugobjects.h>
#include <linux/kallsyms.h>
#include <linux/memory.h>
#include <linux/math64.h>
#include <linux/fault-inject.h>
#endif

#include <asm/uaccess.h>

#include "srecorder_slab_info.h"
#include "srecorder_symbols.h"
#include "srecorder_log.h"

#define check_irq_on()    do { } while(0)

#ifdef CONFIG_SLUB
#define OO_SHIFT 16
#define OO_MASK ((1 << OO_SHIFT) - 1)
#endif

/*
 * The slab lists for all objects.
 */
struct kmem_cache_node {
    spinlock_t list_lock;

#ifdef CONFIG_SLAB
    struct list_head slabs_partial; /* partial list first, better asm code */
    struct list_head slabs_full;
    struct list_head slabs_free;
    unsigned long free_objects;
    unsigned int free_limit;
    unsigned int colour_next;   /* Per-node cache coloring */
    struct array_cache *shared; /* shared per node */
    struct array_cache **alien; /* on other nodes */
    unsigned long next_reap;    /* updated without locking */
    int free_touched;       /* updated without locking */
#endif

#ifdef CONFIG_SLUB
    unsigned long nr_partial;
    struct list_head partial;
#ifdef CONFIG_SLUB_DEBUG
    atomic_long_t nr_slabs;
    atomic_long_t total_objects;
    struct list_head full;
#endif
#endif
};

/*
* struct array_cache
*
* Purpose:
* - LIFO ordering, to hand out cache-warm objects from _alloc
* - reduce the number of linked list operations
* - reduce spinlock operations
*
* The limit is stored in the per-cpu structure to reduce the data cache
* footprint.
*
*/
struct array_cache 
{
    unsigned int avail;
    unsigned int limit;
    unsigned int batchcount;
    unsigned int touched;
    spinlock_t lock;
    
    /*
    * Must have this definition in here for the proper
    * alignment of array_cache. Also simplifies accessing
    * the entries.
    */
    void *entry[];    

};

typedef unsigned int kmem_bufctl_t;

/*
* struct slab_rcu
*
* slab_destroy on a SLAB_DESTROY_BY_RCU cache uses this structure to
* arrange for kmem_freepages to be called via RCU.  This is useful if
* we need to approach a kernel structure obliquely, from its address
* obtained without the usual locking.  We can lock the structure to
* stabilize it and check it's still at the given address, only if we
* can be sure that the memory has not been meanwhile reused for some
* other kind of object (which our subsystem's lock might corrupt).
*
* rcu_read_lock before reading the address, then rcu_read_unlock after
* taking the spinlock within the structure expected at that address.
*/
struct slab_rcu 
{
    struct rcu_head head;
    struct kmem_cache *cachep;
    void *addr;
};

/*
 * struct slab
 *
 * Manages the objs in a slab. Placed either at the beginning of mem allocated
 * for a slab, or allocated from an general cache.
 * Slabs are chained into three list: fully used, partial, fully free slabs.
 */
struct slab 
{
    union 
    {
        struct 
        {
            struct list_head list;
            unsigned long colouroff;
            void *s_mem;        /* including colour offset */
            unsigned int inuse;    /* num of objs active in slab */
            kmem_bufctl_t free;
            unsigned short nodeid;
        };
        
        struct slab_rcu __slab_cover_slab_rcu;
    };
};

/*
* The slab lists for all objects.
*/
struct kmem_list3 
{
    struct list_head slabs_partial;    /* partial list first, better asm code */
    struct list_head slabs_full;
    struct list_head slabs_free;
    unsigned long free_objects;
    unsigned int free_limit;
    unsigned int colour_next;    /* Per-node cache coloring */
    spinlock_t list_lock;
    struct array_cache *shared;    /* shared per node */
    struct array_cache **alien;    /* on other nodes */
    unsigned long next_reap;    /* updated without locking */
    int free_touched;        /* updated without locking */
};

static int srecorder_format_slabinfo_header(void);
static int srecorder_dump_slabinfo(void *p);
static void* srecorder_slabinfo_start(struct list_head *slab_caches, loff_t *pos);
static void* srecorder_slabinfo_next(void *p, struct list_head *slab_caches, loff_t *pos);

#ifdef CONFIG_SLUB
static unsigned long srecorder_count_partial(struct kmem_cache_node *n, int (*get_count)(struct page *));
static inline struct kmem_cache_node *srecorder_get_node(struct kmem_cache *s, int node);
static int srecorder_count_free(struct page *page);
static inline int srecorder_oo_order(struct kmem_cache_order_objects x);
static inline int srecorder_oo_objects(struct kmem_cache_order_objects x);
#endif

static int slab_info_flag = 0;

/**
    @function: void srecorder_enable_slab_info(void)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_enable_slab_info(void)
{
    slab_info_flag = 1;
}

/**
    @function: void srecorder_disable_slab_info(void)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_disable_slab_info(void)
{
    slab_info_flag = 0;
}

/**
    @function: static int srecorder_format_slabinfo_header()
    @brief: 
    @return: 
    @note: 
**/
static int srecorder_format_slabinfo_header(void)
{
    SRECORDER_SNPRINTF("%s\n", 
        "slabinfo - version: 2.1\n"
        "# name            <active_objs> <num_objs> <objsize> <objperslab> <pagesperslab>"
        " : tunables <limit> <batchcount> <sharedfactor>"
        " : slabdata <active_slabs> <num_slabs> <sharedavail>");
    
    return 0;
}

/**
    @function: static void* srecorder_slabinfo_start( 
                struct list_head *slab_caches, loff_t *pos)
    @brief: 
    @param: slab_caches 
    @param: pos 
    @return: 
    @note: 
**/
static void* srecorder_slabinfo_start(struct list_head *slab_caches, loff_t *pos)
{
    srecorder_format_slabinfo_header();
    
    return seq_list_start(slab_caches, *pos);
}

/**
    @function: static void* srecorder_slabinfo_next(void *p, struct list_head *slab_caches, loff_t *pos)
    @brief: 
    @param: p 
    @param: slab_caches 
    @param: pos 
    @return: 
    @note: 
**/
static void* srecorder_slabinfo_next(void *p, struct list_head *slab_caches, loff_t *pos)
{
    return seq_list_next(p, slab_caches, pos);
}


#ifdef CONFIG_SLUB
/**
    @function: static inline int srecorder_oo_order(struct kmem_cache_order_objects x)
    @brief: 
    @return: 
    @note: 
**/
static inline int srecorder_oo_order(struct kmem_cache_order_objects x)
{
    return x.x >> OO_SHIFT;
}

/**
    @function: static inline int srecorder_oo_objects(struct kmem_cache_order_objects x)
    @brief: 
    @return: 
    @note: 
**/
static inline int srecorder_oo_objects(struct kmem_cache_order_objects x)
{
    return x.x & OO_MASK;
}

/**
    @function: static int srecorder_count_free(struct page *page)
    @brief: 
    @return: 
    @note: 
**/
static int srecorder_count_free(struct page *page)
{
    if (unlikely(NULL == page))
    {
        return 0;
    }
    
    return page->objects - page->inuse;
}

/**
    @function: static unsigned long srecorder_count_partial(struct kmem_cache_node *n, int (*get_count)(struct page *))
    @brief: 
    @return: 
    @note: 
**/
static unsigned long srecorder_count_partial(struct kmem_cache_node *n, int (*get_count)(struct page *))
{
    unsigned long flags;
    unsigned long x = 0;
    struct page *page;
    
    if (unlikely(NULL == n || NULL == get_count))
    {
        return 0;
    }
    
    if (spin_trylock_irqsave(&n->list_lock, flags))
    {
        list_for_each_entry(page, &n->partial, lru)
        {
            x += get_count(page);
        }
        spin_unlock_irqrestore(&n->list_lock, flags);
    }
    
    return x;
}

/**
    @function: static inline struct kmem_cache_node *srecorder_get_node(struct kmem_cache *s, int node)
    @brief: 
    @return: 
    @note: 
**/
static inline struct kmem_cache_node *srecorder_get_node(struct kmem_cache *s, int node)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 37))
#ifdef CONFIG_NUMA
    return s->node[node];
#else
    return &s->local_node;
#endif
#else
    return s->node[node];
#endif
}
#endif


/**
    @function: static int srecorder_dump_slabinfo(void *p)
    @brief: 
    @param: p 
    @return: 
    @note: 
*/
static int srecorder_dump_slabinfo(void *p)
{
#ifdef CONFIG_SLAB
    struct kmem_cache *cachep = list_entry(p, struct kmem_cache, next);
    struct slab *slabp;
    unsigned long active_objs = 0;
    unsigned long num_objs = 0;
    unsigned long active_slabs = 0;
    unsigned long num_slabs = 0;
    unsigned long free_objects = 0;
    unsigned long shared_avail = 0;
    const char *name = NULL;
    char *error = NULL;
    int node;
    struct kmem_list3 *l3 = NULL;
    
    if (NULL == cachep)
    {
        SRECORDER_PRINTK("File [%s] line [%d] invalid param!\n", __FILE__, __LINE__);
        return -1;
    }

    for_each_online_node(node) 
    {
        l3 = cachep->nodelists[node];
        if (NULL == l3)
        {
            continue;
        }
        
        check_irq_on();
        if (spin_trylock_irq(&l3->list_lock))
        {
            list_for_each_entry(slabp, &l3->slabs_full, list) 
            {
                if (slabp->inuse != cachep->num && NULL == error)
                {
                    error = "slabs_full accounting error";
                }
                active_objs += cachep->num;
                active_slabs++;
            }
            
            list_for_each_entry(slabp, &l3->slabs_partial, list) 
            {
                if (slabp->inuse == cachep->num && NULL == error)
                {
                    error = "slabs_partial inuse accounting error";
                }
                
                if (!slabp->inuse && NULL == error)
                {
                    error = "slabs_partial/inuse accounting error";
                }
                active_objs += slabp->inuse;
                active_slabs++;
            }
            
            list_for_each_entry(slabp, &l3->slabs_free, list) 
            {
                if (slabp->inuse && NULL == error)
                {
                    error = "slabs_free/inuse accounting error";
                }
                num_slabs++;
            }
            
            free_objects += l3->free_objects;
            if (l3->shared)
            {
                shared_avail += l3->shared->avail;
            }

            spin_unlock_irq(&l3->list_lock);
        }
    }
    
    num_slabs += active_slabs;
    num_objs = num_slabs * cachep->num;
    if ((num_objs - active_objs) != free_objects && NULL == error)
    {
        error = "free_objects accounting error";
    }

    name = cachep->name;
    if (NULL != error)
    {
        SRECORDER_PRINTK("slab: cache %s error: %s\n", name, error);
    }

    SRECORDER_SNPRINTF("%-17s %6lu %6lu %6u %4u %4d"
        " : tunables %4u %4u %4u"
        " : slabdata %6lu %6lu %6lu\n", 
        name, active_objs, num_objs, cachep->buffer_size, cachep->num, (1 << cachep->gfporder), 
        cachep->limit, cachep->batchcount, cachep->shared, 
        active_slabs, num_slabs, shared_avail);
#elif defined(CONFIG_SLUB)
    unsigned long nr_partials = 0;
    
#ifdef CONFIG_SLUB_DEBUG
    unsigned long nr_slabs = 0;
#endif
    
    unsigned long nr_inuse = 0;
    unsigned long nr_objs = 0;
    unsigned long nr_free = 0;
    struct kmem_cache *s = NULL;
    int node;

    s = list_entry(p, struct kmem_cache, list);

    for_each_online_node(node)
    {
        struct kmem_cache_node *n = srecorder_get_node(s, node);

        if (NULL == n)
        {
            continue;
        }

        nr_partials += n->nr_partial;
#ifdef CONFIG_SLUB_DEBUG
        nr_slabs += atomic_long_read(&n->nr_slabs);
        nr_objs += atomic_long_read(&n->total_objects);
#endif
        nr_free += srecorder_count_partial(n, srecorder_count_free);
    }

    nr_inuse = (0 == nr_objs) ? (0UL) : (nr_objs - nr_free);
    SRECORDER_SNPRINTF("%-17s %6lu %6lu %6u %4u %4d"
        " : tunables %4u %4u %4u"
        " : slabdata %6lu %6lu %6lu\n", 
        s->name, 
        nr_inuse, 
#ifdef CONFIG_SLUB_DEBUG
        nr_objs, 
#else
        0UL, 
#endif
        s->size, srecorder_oo_objects(s->oo), (1 << srecorder_oo_order(s->oo)), 
        0UL, 0UL, 0UL, 
#ifdef CONFIG_SLUB_DEBUG
        nr_slabs, nr_slabs, 0UL
#else
        0UL, 0UL, 0UL
#endif
        );
#endif

    return 0;
}

/**
    @function: int srecorder_dump_slab_info()
    @brief: 
    @return: 
    @note: 
**/
int srecorder_dump_slab_info(void)
{
    int ret;
    loff_t pos = 0;
    void *ptr = NULL;
    
#if defined(CONFIG_SLAB) || defined(CONFIG_SLUB)
    struct mutex *slab_mutex = (struct mutex *)srecorder_get_slab_mutex();
    struct list_head *slab_caches = (struct list_head *)srecorder_get_slab_caches();
#endif

    if (slab_info_flag == 0)
    {
        SRECORDER_PRINTK("The dump flag of slab info isn't enabled\n");
        return -1;
    }

    if (
#if defined(CONFIG_SLAB) || defined(CONFIG_SLUB)
        NULL == slab_mutex || NULL == slab_caches 
#endif
        )
    {
        SRECORDER_PRINTK("File [%s] line [%d] invalid param or kernel symbol addr!\n", __FILE__, __LINE__);
        return -1;
    }

    srecorder_log_header_l_start(TYPE_SLAB_INFO);

    srecorder_dump_log_title(TYPE_SLAB_INFO);

#if defined(CONFIG_SLAB) || defined(CONFIG_SLUB)
    if (mutex_trylock(slab_mutex))
#endif
    {
#if defined(CONFIG_SLAB) || defined(CONFIG_SLUB)
        ptr = srecorder_slabinfo_start( slab_caches, &pos);
#endif
        while (1)
        {
            if (NULL == ptr)
            {
                break;
            }
            
            ret = srecorder_dump_slabinfo( ptr);
            if (ret < 0)
            {
                break;
            }
            
#if defined(CONFIG_SLAB) || defined(CONFIG_SLUB)
            ptr = srecorder_slabinfo_next(ptr, slab_caches, &pos);
#endif
        }
        
#if defined(CONFIG_SLAB) || defined(CONFIG_SLUB)
        mutex_unlock(slab_mutex);
#endif
    }
    
    srecorder_log_header_l_end();
    
    return 0;
}
