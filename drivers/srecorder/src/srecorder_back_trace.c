/**
    @copyright: Huawei Technologies Co., Ltd. 2012-2012. All rights reserved.
    
    @file: srecorder_current_ps_backtrace.c
    
    @brief: 
    
    @version: 2.1.1 
    
    @author: Qi Dechun 00216641,    Yan Tongguang 00297150
    
    @date: 2015-03-13
    
    @history:
**/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/fs_struct.h>
#include <linux/file.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/hugetlb.h>
#include <linux/dcache.h>
#include <linux/mount.h>
#include <linux/prefetch.h>
#include <linux/version.h>
#include <linux/ftrace.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38))
#include <linux/seqlock.h>
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0))
#include <linux/seq_file.h>
#include <linux/poll.h>
#endif

#ifdef CONFIG_ARM_UNWIND
#include <asm/stacktrace.h>
#include <asm/unwind.h>
#include <asm/thread_info.h>
#endif

#include <asm/pgalloc.h>
#include <asm/uaccess.h>
#include <asm/tlb.h>
#include <asm/tlbflush.h>
#include <asm/pgtable.h>
#include <asm/cacheflush.h>
#ifdef CONFIG_ARM
#include <asm/system.h>
#endif
#include <asm/traps.h>

#include "srecorder_back_trace.h"
#include "srecorder_symbols.h"
#include "srecorder_log.h"

#define SRECORDER_USER_STACK_DEPTH_MAX (64)
#define SRECORDER_ARM_USER_STACK_PRINT_MAX (10 * (PAGE_SIZE)) 

#ifdef CONFIG_ARM_UNWIND
/* Convert a prel31 symbol to an absolute address */
#define prel31_to_addr(ptr)                \
({                            \
    /* sign-extend to 32 bits */            \
    long offset = (((long)*(ptr)) << 1) >> 1;    \
    (unsigned long)(ptr) + offset;            \
})
#endif

#ifdef CONFIG_ARM_UNWIND
struct unwind_ctrl_block 
{
    unsigned long vrs[16];      /* virtual register set */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0))   
    const unsigned long *insn;  /* pointer to the current instructions word */
#else
    unsigned long *insn;        /* pointer to the current instructions word */
#endif
    int entries;                /* number of entries left to interpret */
    int byte;                   /* current byte number in the instructions word */
};

enum regs 
{
#ifdef CONFIG_THUMB2_KERNEL
    FP = 7,
#else
    FP = 11,
#endif
    SP = 13,
    LR = 14,
    PC = 15
};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0))
extern const struct unwind_idx __start_unwind_idx[];
extern const struct unwind_idx __stop_unwind_idx[];
#else
extern struct unwind_idx __start_unwind_idx[];
extern struct unwind_idx __stop_unwind_idx[];
#endif
#endif

typedef const char* (*arch_vma_name_func)(struct vm_area_struct *vma);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0))
struct mnt_namespace 
{
    atomic_t        count;
    struct mount *    root;
    struct list_head    list;
    wait_queue_head_t poll;
    int event;
};

struct mnt_pcp
{
    int mnt_count;
    int mnt_writers;
};

struct mount 
{
    struct list_head mnt_hash;
    struct mount *mnt_parent;
    struct dentry *mnt_mountpoint;
    struct vfsmount mnt;
#ifdef CONFIG_SMP
    struct mnt_pcp __percpu *mnt_pcp;
    atomic_t mnt_longterm;              /* how many of the refs are longterm */
#else
    int mnt_count;
    int mnt_writers;
#endif
    struct list_head mnt_mounts;        /* list of children, anchored here */
    struct list_head mnt_child;         /* and going through their mnt_child */
    struct list_head mnt_instance;      /* mount instance on sb->s_mounts */
    const char *mnt_devname;            /* Name of device e.g. /dev/dsk/hda1 */
    struct list_head mnt_list;
    struct list_head mnt_expire;        /* link in fs-specific expiry list */
    struct list_head mnt_share;         /* circular list of shared mounts */
    struct list_head mnt_slave_list;    /* list of slave mounts */
    struct list_head mnt_slave;         /* slave list entry */
    struct mount *mnt_master;           /* slave is on master->mnt_slave_list */
    struct mnt_namespace *mnt_ns;       /* containing namespace */
#ifdef CONFIG_FSNOTIFY
    struct hlist_head mnt_fsnotify_marks;
    __u32 mnt_fsnotify_mask;
#endif
    int mnt_id;                         /* mount identifier */
    int mnt_group_id;                   /* peer group identifier */
    int mnt_expiry_mark;                /* true if marked for expiry */
    int mnt_pinned;
    int mnt_ghosts;
};
#endif

#ifdef CONFIG_ARM_UNWIND
static void srecorder_unwind_back_trace(struct task_struct *ptask);
#ifdef CONFIG_ARM_UNWIND
static void srecorder_dump_back_trace_entry(unsigned long where, unsigned long from, unsigned long frame);
#else
void srecorder_dump_back_trace_entry(unsigned long where, unsigned long from, unsigned long frame);
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0))
const static struct unwind_idx *srecorder_search_index(unsigned long addr,
    const struct unwind_idx *start,
    const struct unwind_idx *origin,
    const struct unwind_idx *stop);
static const struct unwind_idx *srecorder_find_original_unwind_idx(const struct unwind_idx *start, const struct unwind_idx *stop);
#else
static struct unwind_idx *srecorder_search_index(unsigned long addr, struct unwind_idx *first, struct unwind_idx *last);
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0))
static const struct unwind_idx *srecorder_find_unwind_idx(unsigned long addr);
#else
static struct unwind_idx *srecorder_find_unwind_idx(unsigned long addr);
#endif
static int srecorder_execute_unwind_instruction(struct unwind_ctrl_block *ctrl);
static unsigned long srecorder_get_unwind_instruction(struct unwind_ctrl_block *ctrl);
#endif

static void srecorder_dump_memory(const char *str, unsigned long bottom, unsigned long top);

#ifdef CONFIG_ARM
static void srecorder_dump_user_back_trace(void);

static void srecorder_get_process_maps(struct task_struct *ptask);
static char *srecorder_d_path(const struct path *path, char *buf, int buflen);
static int srecorder_prepend(char **buffer, int *buflen, const char *str, int namelen);
static int srecorder_prepend_name(char **buffer, int *buflen, struct qstr *name);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 14))
static int srecorder_prepend_path(const struct path *path, const struct path *root, char **buffer, int *buflen);
#else
static int srecorder_prepend_path(const struct path *path, struct path *root, char **buffer, int *buflen);
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 14))
static int srecorder_path_with_deleted(const struct path *path, const struct path *root, char **buf, int *buflen);
#else
static int srecorder_path_with_deleted(const struct path *path, struct path *root, char **buf, int *buflen);
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36))
static char *srecorder_d_path_with_root(const struct path *path, struct path *root, char *buffer, int buflen);
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0))
static inline struct mount *srecorder_real_mount(struct vfsmount *mnt)
{
    return container_of(mnt, struct mount, mnt);
}

static inline int srecorder_mnt_has_parent(struct mount *mnt)
{
    return mnt != mnt->mnt_parent;
}
#endif
#endif

static int back_trace_flag = 0;

/**
    @function: void srecorder_enable_reason_time()
    @brief: 
    @return: 
    @note: 
**/
void srecorder_enable_back_trace(void)
{
    back_trace_flag = 1;
}

/**
    @function: void srecorder_disable_reason_time()
    @brief: 
    @return: 
    @note: 
**/
void srecorder_disable_back_trace(void)
{
    back_trace_flag = 0;
}

/**
    @function: static void srecorder_dump_memory(
        const char *str, unsigned long bottom, unsigned long top)
    @brief: Dump out the contents of some memory nicely...
    @param: 
    @return: none
    @note: 
*/
 static void srecorder_dump_memory(const char *str, unsigned long bottom, unsigned long top)
{
    unsigned long first;
    mm_segment_t fs;
    int i = 0;

    if (unlikely(NULL == str))
    {
        return;
    }
    
    /*
    * We need to switch to kernel mode so that we can use __get_user
    * to safely read from kernel space.  Note that we now dump the
    * code first, just in case the backtrace kills us.
    */
    fs = get_fs();
    set_fs(KERNEL_DS);

    SRECORDER_SNPRINTF("%s(0x%016lx to 0x%016lx)\n", str, bottom, top);
    
    for (first = bottom & (~31); first < top; first += 32)
    {
        unsigned long p;
        char str[sizeof(" 12345678") * 8 + 1];

        memset(str, ' ', sizeof(str));
        str[sizeof(str) - 1] = '\0';

        for (p = first, i = 0; i < 8 && p < top; i++, p += 4)
        {
            if (p >= bottom && p < top) 
            {
                unsigned int val;
                if (__get_user(val, (unsigned int *)p) == 0)
                {
                    sprintf(str + i * 9, " %08x", val);
                }
                else
                {
                    sprintf(str + i * 9, " ????????");
                }
            }
        }
        
        SRECORDER_SNPRINTF("%04lx:%s\n", first & 0xffff, str);
    }

    set_fs(fs);
}

#ifdef CONFIG_ARM
/**
    @function: static int srecorder_prepend(char **buffer, int *buflen, const char *str, int namelen)
    @brief: 
    @param: buffer 
    @return: 0 - success, -1 - failed
    @note: 
*/
static int srecorder_prepend(char **buffer, int *buflen, const char *str, int namelen)
{
    if (unlikely(NULL == buffer || NULL == buflen || NULL == str))
    {
        return -1;
    }
    
    *buflen -= namelen;
    if (*buflen < 0)
    {
        return -ENAMETOOLONG;
    }
    *buffer -= namelen;
    memcpy(*buffer, str, namelen);
    
    return 0;
}

/**
    @function: static int srecorder_prepend_name(char **buffer, int *buflen, struct qstr *name)
    @brief: 
    @param: buffer 
    @return: 0 - success, -1 - failed
    @note: 
*/
static int srecorder_prepend_name(char **buffer, int *buflen, struct qstr *name)
{
    if (unlikely(NULL == buffer || NULL == buflen || NULL == name))
    {
        return -1;
    }
    
    return srecorder_prepend(buffer, buflen, name->name, name->len);
}

/**
    @function: static int srecorder_prepend_path(const struct path *path, const struct path *root, char **buffer, int *buflen)
    @brief: Prepend path string to a buffer
    @param: path: the dentry/vfsmount to report
    @return: 0 - success, others - failed
    @note: 
*/
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 14))
static int srecorder_prepend_path(const struct path *path, const struct path *root, char **buffer, int *buflen)
#else
static int srecorder_prepend_path(const struct path *path, struct path *root, char **buffer, int *buflen)
#endif
{
    struct dentry *dentry = path->dentry;
    struct vfsmount *vfsmnt = path->mnt;
    bool slash = false;
    int error = 0;
    arch_spinlock_t *lock;
    
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0))
    struct mount *mnt = srecorder_real_mount(vfsmnt);
#endif

    if (unlikely(NULL == path || NULL == root || NULL == buffer || NULL == buflen 
        || INVALID_KSYM_ADDR == srecorder_get_vfsmount_lock_lock()))
    {
        return -1;
    }

    preempt_disable(); 
    lock = &__get_cpu_var(*(arch_spinlock_t *)srecorder_get_vfsmount_lock_lock()); 
    
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 3, 0))
    if (arch_spin_trylock(lock))
    {
        while (dentry != root->dentry || vfsmnt != root->mnt) 
        {
            struct dentry * parent;

            if (dentry == vfsmnt->mnt_root || IS_ROOT(dentry)) 
            {
                /* Global root? */
                if (vfsmnt->mnt_parent == vfsmnt) 
                {
                    goto global_root;
                }
                dentry = vfsmnt->mnt_mountpoint;
                vfsmnt = vfsmnt->mnt_parent;
                continue;
            }
            parent = dentry->d_parent;
            prefetch(parent);
            error = -1;
            
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38))
            if (spin_trylock(&dentry->d_lock))
            {
                error = srecorder_prepend_name(buffer, buflen, &dentry->d_name);
                spin_unlock(&dentry->d_lock);
            }
#else
            error = srecorder_prepend_name(buffer, buflen, &dentry->d_name);
#endif

            if (0 == error)
            {
                error = srecorder_prepend(buffer, buflen, "/", 1);
            }

            if (error)
            {
                break;
            }

            slash = true;
            dentry = parent;
        }

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 14))
        if (!error && !slash)
        {
            error = srecorder_prepend(buffer, buflen, "/", 1);
        }
#endif
    }

out:
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 0, 14))
        if (!error && !slash)
        {
            error = srecorder_prepend(buffer, buflen, "/", 1);
        }
#endif
    arch_spin_unlock(lock);
    preempt_enable();
    return error;

global_root:
    /*
    * Filesystems needing to implement special "root names"
    * should do so with ->d_dname()
    */
    if (IS_ROOT(dentry) && (dentry->d_name.len != 1 || dentry->d_name.name[0] != '/')) 
    {
        WARN(1, "Root dentry has weird name <%.*s>\n", (int) dentry->d_name.len, dentry->d_name.name);
    }

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 0, 14))
    root->mnt = vfsmnt;
    root->dentry = dentry; 
#else
    if (!slash)
    {
        error = srecorder_prepend(buffer, buflen, "/", 1);
    }

    if (!error)
    {
        error = vfsmnt->mnt_ns ? 1 : 2;
    }
#endif
    goto out;
#else
    if (arch_spin_trylock(lock))
    {
        while (dentry != root->dentry || vfsmnt != root->mnt)
        {
            struct dentry * parent;

            if (dentry == vfsmnt->mnt_root || IS_ROOT(dentry))
            {
                /* Global root? */
                if (!srecorder_mnt_has_parent(mnt))
                {
                    goto global_root;
                }
                dentry = mnt->mnt_mountpoint;
                mnt = mnt->mnt_parent;
                vfsmnt = &mnt->mnt;
                continue;
            }
            parent = dentry->d_parent;
            prefetch(parent);
            error = -1;
            if (spin_trylock(&dentry->d_lock))
            {
                error = srecorder_prepend_name(buffer, buflen, &dentry->d_name);
                spin_unlock(&dentry->d_lock);
            }
            if (!error)
            {
                error = srecorder_prepend(buffer, buflen, "/", 1);
            }

            if (error)
            {
                break;
            }

            slash = true;
            dentry = parent;
        }

        if (!error && !slash)
        {
            error = srecorder_prepend(buffer, buflen, "/", 1);
        }
    }
    
out:
    arch_spin_unlock(lock);
    preempt_enable();
    return error;

global_root:
    /*
    * Filesystems needing to implement special "root names"
    * should do so with ->d_dname()
    */
    if (IS_ROOT(dentry) && (dentry->d_name.len != 1 || dentry->d_name.name[0] != '/')) 
    {
        WARN(1, "Root dentry has weird name <%.*s>\n", (int) dentry->d_name.len, dentry->d_name.name);
    }

    if (!slash)
    {
        error = srecorder_prepend(buffer, buflen, "/", 1);
    }

    if (!error)
    {
        error = srecorder_real_mount(vfsmnt)->mnt_ns ? 1 : 2;
    }
    goto out;
#endif
}


/**
    @function: static int srecorder_path_with_deleted(const struct path *path, const struct path *root, 
        char **buf, int *buflen)
    @brief: return the path of a dentry but appends "(deleted)" for unlinked files.
    @param: path: path to report
    @return: none
    @note: 
*/
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 14))
static int srecorder_path_with_deleted(const struct path *path, const struct path *root, char **buf, int *buflen)
#else
static int srecorder_path_with_deleted(const struct path *path, struct path *root, char **buf, int *buflen)
#endif
{
    if (unlikely(NULL == path || NULL == root || NULL == buf || NULL == buflen))
    {
        return -1;
    }
    
    srecorder_prepend(buf, buflen, "\0", 1);
    
    if (d_unlinked(path->dentry))
    {
        int error = srecorder_prepend(buf, buflen, " (deleted)", 10);
        
        if (0 != error)
        {
            return error;
        }
    }

    return srecorder_prepend_path(path, root, buf, buflen);
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36))
/**
    * __d_path - return the path of a dentry
    * @path: the dentry/vfsmount to report
    * @root: root vfsmnt/dentry (may be modified by this function)
    * @buffer: buffer to return value in
    * @buflen: buffer length
    * Convert a dentry into an ASCII path name. If the entry has been deleted
    * the string " (deleted)" is appended. Note that this is ambiguous.
    * Returns a pointer into the buffer or an error code if the
    * path was too long.
    * "buflen" should be positive. Caller holds the dcache_lock.
    * If path is not reachable from the supplied root, then the value of
    * root is changed (without modifying refcounts).
*/
static char *srecorder_d_path_with_root(const struct path *path, struct path *root, char *buffer, int buflen)
{
    struct dentry *dentry = path->dentry;
    struct vfsmount *vfsmnt = path->mnt;
    char *end = buffer + buflen;
    char *retval;
    bool has_vfsmount_lock = false;
    
    if (unlikely(INVALID_KSYM_ADDR == srecorder_get_vfsmount_lock()))
    {
        return "";
    }

    has_vfsmount_lock = spin_trylock((spinlock_t *)srecorder_get_vfsmount_lock());
    if (has_vfsmount_lock)
    {
        srecorder_prepend(&end, &buflen, "\0", 1);
        if (d_unlinked(dentry) && (srecorder_prepend(&end, &buflen, " (deleted)", 10) != 0))
        {
            goto Elong;
        }

        if (buflen < 1)
        {
            goto Elong;
        }
        
        /* Get '/' right */
        retval = end-1;
        *retval = '/';

        for (;;) 
        {
            struct dentry * parent;

            if (dentry == root->dentry && vfsmnt == root->mnt)
            {
                break;
            }
            
            if (dentry == vfsmnt->mnt_root || IS_ROOT(dentry)) 
            {
                /* Global root? */
                if (vfsmnt->mnt_parent == vfsmnt) 
                {
                    goto global_root;
                }
                dentry = vfsmnt->mnt_mountpoint;
                vfsmnt = vfsmnt->mnt_parent;
                continue;
            }
            parent = dentry->d_parent;
            prefetch(parent);
            if ((srecorder_prepend_name(&end, &buflen, &dentry->d_name) != 0)
                || (srecorder_prepend(&end, &buflen, "/", 1) != 0))
            {
                goto Elong;
            }
            retval = end;
            dentry = parent;
        }
    }
out:
    if (has_vfsmount_lock)
    {
        spin_unlock((spinlock_t *)srecorder_get_vfsmount_lock());
    }
    return retval;

global_root:
    retval += 1;	/* hit the slash */
    if (srecorder_prepend_name(&retval, &buflen, &dentry->d_name) != 0)
    {
        goto Elong;
    }
    root->mnt = vfsmnt;
    root->dentry = dentry;
    goto out;

Elong:
    retval = ERR_PTR(-ENAMETOOLONG);
    goto out;
}
#endif

/**
    @function: static char *srecorder_d_path(const struct path *path, char *buf, int buflen)
    @brief: return the path of a dentry
    @param: path: path to report
    @return: none
    @note: 
*/
static char *srecorder_d_path(const struct path *path, char *buf, int buflen)
{
    char *res = buf + buflen;
    struct path root;
    int error;
    int have_lock = 0;

    if (unlikely(NULL == path || NULL == buf 
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 38))
        || INVALID_KSYM_ADDR == srecorder_get_dcache_lock())
#else
        || INVALID_KSYM_ADDR == srecorder_get_rename_lock())
#endif
        )
    {
        return "";
    }
    
    /*
     * We have various synthetic filesystems that never get mounted.  On
     * these filesystems dentries are never used for lookup purposes, and
     * thus don't need to be hashed.  They also don't need a name until a
     * user wants to identify the object in /proc/pid/fd/.  The little hack
     * below allows us to generate a name for these objects on demand:
     */
    if (path->dentry->d_op && path->dentry->d_op->d_dname)
    {
        return path->dentry->d_op->d_dname(path->dentry, buf, buflen);
    }

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36))
    if (spin_trylock(&(current->fs->lock)))
    {
        root = current->fs->root;
        have_lock = (NULL != root.dentry) && (spin_trylock(&(root.dentry->d_lock)));
        spin_unlock(&(current->fs->lock));
        if (0 != have_lock)
        {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 38))
            if (spin_trylock((spinlock_t *)srecorder_get_dcache_lock()))
            {
#else
            write_seqlock(&rename_lock);
#endif
                error = srecorder_path_with_deleted(path, &root, &res, &buflen);
                if (error < 0)
                {
                    res = ERR_PTR(error);
                }
                
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 38))
                spin_unlock((spinlock_t *)srecorder_get_dcache_lock());
            }
#else
            write_sequnlock(&rename_lock);
#endif
            spin_unlock(&(root.dentry->d_lock));
        }
    }
#else
    if (read_trylock(&(current->fs->lock)))
    {
        root = current->fs->root;
        read_unlock(&(current->fs->lock));
        if (spin_trylock((spinlock_t *)srecorder_get_dcache_lock()))
        {
            res = srecorder_d_path_with_root(path, &root, buf, buflen);
            spin_unlock((spinlock_t *)srecorder_get_dcache_lock());
        }
    }
#endif

    return res;
}

/**
    @function: static void srecorder_get_process_maps( struct task_struct *ptask)
    @brief: 
    @param: ptask 
    @return: none
    @note: 
*/
static void srecorder_get_process_maps(struct task_struct *ptask)
{
    struct mm_struct *mm = NULL;
    struct vm_area_struct *vma = NULL;

    if (unlikely(NULL == ptask))
    {
        return;
    }

    if (NULL == ptask->mm || ptask->flags & PF_KTHREAD) 
    {
        return;
    }

    mm = ptask->mm;
    vma = mm->mmap;
    while (NULL != vma)
    {
        unsigned long start;
        unsigned long end;
        
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 1))
        vm_flags_t flags = vma->vm_flags;
#else
        int flags = vma->vm_flags;
#endif

        /* We don't show the stack guard page in /proc/maps */
        start = vma->vm_start;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)) 
        if (stack_guard_page_start(vma, start))
        {
            start += PAGE_SIZE;
        }
#endif
        end = vma->vm_end;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39))
        if (stack_guard_page_end(vma, end))
        {
            end -= PAGE_SIZE;
        }
#endif
        if (flags & VM_EXEC)
        {
            struct file *file = vma->vm_file;

            if (NULL != file)
            {
                char buf[128] = {0}; 
                char *ppath = srecorder_d_path(&file->f_path, buf, 128);
                
                if (!IS_ERR(ppath))
                {
                    SRECORDER_SNPRINTF("$ %08lx-%08lx:%s\n", start, end, ppath);
                }
            }
            else
            {
                arch_vma_name_func arch_vma_name = (arch_vma_name_func)srecorder_get_arch_vma_name();
                const char *name = NULL;

                if (NULL != arch_vma_name)
                {
                    name = arch_vma_name(vma);
                }

                SRECORDER_SNPRINTF("$ %08lx-%08lx:%s\n", start, end, (NULL == name) ? ("anonymous") : (name));
            }
        }
        vma = vma->vm_next;
    }
}

/**
    @function: static void srecorder_dump_user_back_trace()
    @brief: 
    @param: 
    @return: none
    @note: 
*/
static void srecorder_dump_user_back_trace(void)
{
    struct stackframe frame;
    struct pt_regs * regs = NULL;
    unsigned long top; 
    unsigned long base;
    unsigned long i = 0;

    if (NULL == current)
    {
        SRECORDER_SNPRINTF("%s", "current: is NULL!\n");
        return;
    }
    
    regs = task_pt_regs(current);
    if (NULL == regs)
    {
        SRECORDER_SNPRINTF("%s", "current's registors is NULL!\n");
        return;
    }
    
    if ((NULL == current->mm)
        || (current->flags & PF_KTHREAD)
        || (NULL == (void *)regs->ARM_pc || NULL == (void *)regs->ARM_lr))
    {
        SRECORDER_SNPRINTF("pc: %p (%pF), lr: %p (%pF) %s\n", (void *)regs->ARM_pc, (void *)regs->ARM_pc, (void *)regs->ARM_lr, 
            (void *)regs->ARM_lr, "The current process is a kernel thread which has no user stack!");
        return;
    }
    
    frame.fp = regs->ARM_fp;
    frame.sp = regs->ARM_sp;
    frame.lr = regs->ARM_lr;
    frame.pc = regs->ARM_pc;
    SRECORDER_SNPRINTF("***************************** dump user stack begin *****************************\n"
        "* For %s mode, do user backtrace:\n* pc: %p (%pF), lr: %p (%pF), sp: %p, fp: %p\n",
        user_mode(regs) ? ("USER") : ("NON-USER"), 
        (void *)regs->ARM_pc, (void *)regs->ARM_pc, (void *)regs->ARM_lr, 
        (void *)regs->ARM_lr, (void *)regs->ARM_sp, (void *)regs->ARM_fp);

    top = frame.sp;
    top &= ~(sizeof(unsigned long) - 1); 
    base = current->mm->start_stack; 
    base &= ~(sizeof(unsigned long) - 1); 
 
    if (abs(base - top) > SRECORDER_ARM_USER_STACK_PRINT_MAX)
    {
#ifdef CONFIG_STACK_GROWSUP
        base = top - SRECORDER_ARM_USER_STACK_PRINT_MAX;
#else
        base = top + SRECORDER_ARM_USER_STACK_PRINT_MAX;
#endif
    }
    
    SRECORDER_SNPRINTF("* User stack: from 0x%08lx to 0x%08lx\n* User heap: from 0x%08lx to 0x%08lx\n"
        "* cmd arg: from 0x%08lx to 0x%08lx\n* env: from 0x%08lx to 0x%08lx\n", 
        current->mm->start_stack, frame.sp, 
        current->mm->start_brk, current->mm->brk, 
        current->mm->arg_start, current->mm->arg_end, 
        current->mm->env_start, current->mm->env_end);

    if (spin_trylock(&(current->alloc_lock)))
    {
        srecorder_get_process_maps(current);
        spin_unlock(&(current->alloc_lock));
    }
        
    i = 0;
        
#ifdef CONFIG_STACK_GROWSUP
    while (top >= base)
#else
    while (top <= base)
#endif
    {
        SRECORDER_SNPRINTF("%08lx ", *(unsigned long *)top);
#ifdef CONFIG_STACK_GROWSUP
        top -= 4; 
#else
        top += 4;
#endif
        i++;
        if (8 == i) 
        {
            SRECORDER_SNPRINTF("%s", "\n");
            i = 0;
        }
    }
 
    SRECORDER_SNPRINTF("%s", "\n****************************** dump user stack end ******************************\n");
}

#ifdef CONFIG_ARM_UNWIND
/**
    @function:static void srecorder_dump_back_trace_entry( 
            unsigned long where, unsigned long from, unsigned long frame)
    @brief:
    @param: 
    @return: none
    @note: 
*/
static void srecorder_dump_back_trace_entry(unsigned long where, unsigned long from, unsigned long frame)
{
#ifdef CONFIG_KALLSYMS
    SRECORDER_SNPRINTF("[<%08lx>] (%pS) from [<%08lx>] (%pS)\n", where, (void *)where, from, (void *)from);
#else
    SRECORDER_SNPRINTF("Function entered at [<%08lx>] from [<%08lx>]\n", where, from);
#endif
    
    if (in_exception_text(where))
    {
        srecorder_dump_memory("Exception stack", frame + 4, frame + 4 + sizeof(struct pt_regs));
    }
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0))
/*
 * Binary search in the unwind index. The entries are
 * guaranteed to be sorted in ascending order by the linker.
 *
 * start = first entry
 * origin = first entry with positive offset (or stop if there is no such entry)
 * stop - 1 = last entry
 */
const static struct unwind_idx *srecorder_search_index(unsigned long addr,
    const struct unwind_idx *start,
    const struct unwind_idx *origin,
    const struct unwind_idx *stop)
{
    unsigned long addr_prel31;

    if (unlikely(NULL == start || NULL == origin || NULL == stop))
    {
        return NULL;
    }

    /*
    * only search in the section with the matching sign. This way the
    * prel31 numbers can be compared as unsigned longs.
    */
    if (addr < (unsigned long)start)
    {
        /* negative offsets: [start; origin) */
        stop = origin;
    }
    else
    {
        /* positive offsets: [origin; stop) */
        start = origin;
    }
    
    /* prel31 for address relavive to start */
    addr_prel31 = (addr - (unsigned long)start) & 0x7fffffff;

    while (start < stop - 1)
    {
        const struct unwind_idx *mid = start + ((stop - start) >> 1);

        /*
        * As addr_prel31 is relative to start an offset is needed to
        * make it relative to mid.
        */
        if (addr_prel31 - ((unsigned long)mid - (unsigned long)start) < mid->addr_offset)
        {
            stop = mid;
        }
        else
        {
            /* keep addr_prel31 relative to start */
            addr_prel31 -= ((unsigned long)mid - (unsigned long)start);
            start = mid;
        }
    }

    if (likely(start->addr_offset <= addr_prel31))
    {
        return start;
    }
    else
    {
        return NULL;
    }
}

#else

/*
 * Binary search in the unwind index. The entries entries are
 * guaranteed to be sorted in ascending order by the linker.
 */
static struct unwind_idx *srecorder_search_index(unsigned long addr, struct unwind_idx *first, struct unwind_idx *last)
{
    if (unlikely(NULL == first || NULL == last))
    {
        return NULL;
    }
    
    if (addr < first->addr) 
    {
        return NULL;
    }
    else if (addr >= last->addr)
    {
        return last;
    }

    while (first < last - 1) 
    {
        struct unwind_idx *mid = first + ((last - first + 1) >> 1);

        if (addr < mid->addr)
        {
            last = mid;
        }
        else
        {
            first = mid;
        }
    }

    return first;
}
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0))
static const struct unwind_idx *srecorder_find_original_unwind_idx(const struct unwind_idx *start, const struct unwind_idx *stop)
{
    while (start < stop) 
    {
        const struct unwind_idx *mid = start + ((stop - start) >> 1);

        if (mid->addr_offset >= 0x40000000)
        {
            /* negative offset */
            start = mid + 1;
        }
        else
        {
            /* positive offset */
            stop = mid;
        }
    }
    
    return stop;
}
#endif


#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0))
static const struct unwind_idx *srecorder_find_unwind_idx(unsigned long addr)
#else
static struct unwind_idx *srecorder_find_unwind_idx(unsigned long addr)
#endif
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0))
    const struct unwind_idx *idx = NULL;
#else
    struct unwind_idx *idx = NULL;
#endif

    unsigned long flags;

    if (unlikely(INVALID_KSYM_ADDR == srecorder_get_unwind_lock()))
    {
        return NULL;
    }
    
    if (core_kernel_text(addr))
    {
        /* main unwind table */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0))
        const struct unwind_idx *srecoder_origin_unwind_idx = (struct unwind_idx *)
            (*(srec_ksym_addr_t *)srecorder_get___origin_unwind_idx());
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0))
        if (unlikely(NULL == srecoder_origin_unwind_idx))
        {
            srecoder_origin_unwind_idx = srecorder_find_original_unwind_idx(__start_unwind_idx, __stop_unwind_idx);
        }

        idx = srecorder_search_index(addr, __start_unwind_idx, srecoder_origin_unwind_idx, __stop_unwind_idx);
#else
        idx = srecorder_search_index(addr, __start_unwind_idx, __stop_unwind_idx - 1);
#endif
    }
    else 
    {
        /* module unwind tables */
        struct unwind_table *table;
        struct list_head *unwind_tables = (struct list_head *)srecorder_get_unwind_tables();

        if (unlikely(NULL == unwind_tables))
        {
            return NULL;
        }

        if (spin_trylock_irqsave((spinlock_t *)srecorder_get_unwind_lock(), flags))
        {
            list_for_each_entry(table, unwind_tables, list) 
            {
                if (addr >= table->begin_addr && addr < table->end_addr)
                {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0))
                    idx = srecorder_search_index(addr, (struct unwind_idx *)table->start, 
                        (struct unwind_idx *)table->origin, (struct unwind_idx *)table->stop);
#else
                    idx = srecorder_search_index(addr, table->start, table->stop - 1);
#endif

                    /* Move-to-front to exploit common traces */
                    list_move(&table->list, unwind_tables);
                    break;
                }
            }
            
            spin_unlock_irqrestore((spinlock_t *)srecorder_get_unwind_lock(), flags);
        }
    }

    return idx;
}

static unsigned long srecorder_get_unwind_instruction(struct unwind_ctrl_block *ctrl)
{
    unsigned long ret = 0;

    if (unlikely(NULL == ctrl))
    {
        return 0;
    }
    
    if (ctrl->entries <= 0) 
    {
        return 0;
    }

    ret = (*ctrl->insn >> (ctrl->byte * 8)) & 0xff;

    if (ctrl->byte == 0) 
    {
        ctrl->insn++;
        ctrl->entries--;
        ctrl->byte = 3;
    }
    else
    {
        ctrl->byte--;
    }

    return ret;
}

/*
 * Execute the current unwind instruction.
 */
static int srecorder_execute_unwind_instruction(struct unwind_ctrl_block *ctrl)
{
    unsigned long insn = srecorder_get_unwind_instruction(ctrl);

    if (unlikely(NULL == ctrl))
    {
        return -URC_FAILURE;
    }
    
    if ((insn & 0xc0) == 0x00)
    {
        ctrl->vrs[SP] += ((insn & 0x3f) << 2) + 4;
    }
    else if ((insn & 0xc0) == 0x40)
    {
        ctrl->vrs[SP] -= ((insn & 0x3f) << 2) + 4;
    }
    else if ((insn & 0xf0) == 0x80) 
    {
        unsigned long mask;
        unsigned long *vsp = (unsigned long *)ctrl->vrs[SP];
        int load_sp = 0; 
        int reg = 4;

        insn = (insn << 8) | srecorder_get_unwind_instruction(ctrl);
        mask = insn & 0x0fff;
        if (mask == 0) 
        {
            return -URC_FAILURE;
        }

        /* pop R4-R15 according to mask */
        load_sp = mask & (1 << (13 - 4));
        while (mask) 
        {
            if (mask & 1)
            {
                ctrl->vrs[reg] = *vsp++;
            }
            mask >>= 1;
            reg++;
        }
        
        if (!load_sp)
        {
            ctrl->vrs[SP] = (unsigned long)vsp;
        }
    }
    else if ((insn & 0xf0) == 0x90 && (insn & 0x0d) != 0x0d)
    {
        ctrl->vrs[SP] = ctrl->vrs[insn & 0x0f];
    }
    else if ((insn & 0xf0) == 0xa0) 
    {
        unsigned long *vsp = (unsigned long *)ctrl->vrs[SP];
        int reg;

        /* pop R4-R[4+bbb] */
        for (reg = 4; reg <= 4 + (insn & 7); reg++)
        {
            ctrl->vrs[reg] = *vsp++;
        }
        
        if (insn & 0x80)
        {
            ctrl->vrs[14] = *vsp++;
        }
        ctrl->vrs[SP] = (unsigned long)vsp;
    }
    else if (insn == 0xb0)
    {
        if (ctrl->vrs[PC] == 0)
        {
            ctrl->vrs[PC] = ctrl->vrs[LR];
        }
        
        /* no further processing */
        ctrl->entries = 0;
    } 
    else if (insn == 0xb1) 
    {
        unsigned long mask = srecorder_get_unwind_instruction(ctrl);
        unsigned long *vsp = (unsigned long *)ctrl->vrs[SP];
        int reg = 0;

        if (mask == 0 || mask & 0xf0) 
        {
            return -URC_FAILURE;
        }

        /* pop R0-R3 according to mask */
        while (mask) 
        {
            if (mask & 1)
            {
                ctrl->vrs[reg] = *vsp++;
            }
            mask >>= 1;
            reg++;
        }
        ctrl->vrs[SP] = (unsigned long)vsp;
    } 
    else if (insn == 0xb2) 
    {
        unsigned long uleb128 = srecorder_get_unwind_instruction(ctrl);

        ctrl->vrs[SP] += 0x204 + (uleb128 << 2);
    } 
    else 
    {
        return -URC_FAILURE;
    }

    return URC_OK;
}

/**
    @function: int srecorder_unwind_frame(struct stackframe *frame)
    @brief: 
    @param: frame 
    @return: 0 - successfully, others - failed
    @note: 
*/
int srecorder_unwind_frame(struct stackframe *frame)
{
    unsigned long high;
    unsigned long low;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0))
    const struct unwind_idx *idx;
#else
    struct unwind_idx *idx;
#endif

    struct unwind_ctrl_block ctrl;
    
    if (unlikely(NULL == frame))
    {
        return -1;
    }
    
    /* only go to a higher address on the stack */
    low = frame->sp;
    high = ALIGN(low, THREAD_SIZE);
    
    if (!kernel_text_address(frame->pc))
    {
        return -URC_FAILURE;
    }

    idx = srecorder_find_unwind_idx(frame->pc);
    if (NULL == idx) 
    {
        return -URC_FAILURE;
    }

    ctrl.vrs[FP] = frame->fp;
    ctrl.vrs[SP] = frame->sp;
    ctrl.vrs[LR] = frame->lr;
    ctrl.vrs[PC] = 0;

    if (idx->insn == 1)
    {
        /* can't unwind */
        return -URC_FAILURE;
    }
    else if ((idx->insn & 0x80000000) == 0)
    {
        /* prel31 to the unwind table */
        ctrl.insn = (unsigned long *)prel31_to_addr(&idx->insn);
    }
    else if ((idx->insn & 0xff000000) == 0x80000000)
    {
        /* only personality routine 0 supported in the index */
        ctrl.insn = &idx->insn;
    }
    else 
    {
        return -URC_FAILURE;
    }

    /* check the personality routine */
    if ((*ctrl.insn & 0xff000000) == 0x80000000) 
    {
        ctrl.byte = 2;
        ctrl.entries = 1;
    } 
    else if ((*ctrl.insn & 0xff000000) == 0x81000000)
    {
        ctrl.byte = 1;
        ctrl.entries = 1 + ((*ctrl.insn & 0x00ff0000) >> 16);
    }
    else 
    {
        return -URC_FAILURE;
    }

    while (ctrl.entries > 0)
    {
        int urc = srecorder_execute_unwind_instruction(&ctrl);
        if (urc < 0)
        {
            return urc;
        }
        
        if (ctrl.vrs[SP] < low || ctrl.vrs[SP] >= high)
        {
            return -URC_FAILURE;
        }
    }

    if (ctrl.vrs[PC] == 0)
    {
        ctrl.vrs[PC] = ctrl.vrs[LR];
    }

    /* check for infinite loop */
    if (frame->pc == ctrl.vrs[PC])
    {
        return -URC_FAILURE;
    }

    frame->fp = ctrl.vrs[FP];
    frame->sp = ctrl.vrs[SP];
    frame->lr = ctrl.vrs[LR];
    frame->pc = ctrl.vrs[PC];

    return URC_OK;
}

/**
    @function:static void srecorder_unwind_back_trace(struct task_struct *ptask)
    @brief: 
    @param: regs 
    @return: none
    @note: 
*/
static void srecorder_unwind_back_trace(struct task_struct *ptask)
{
    struct stackframe frame;
    register unsigned long current_sp asm ("sp");

    if (NULL == ptask)
    {
        ptask = current;
    }

    if (ptask == current) 
    {
        frame.fp = (unsigned long)__builtin_frame_address(0);
        frame.sp = current_sp;
        frame.lr = (unsigned long)__builtin_return_address(0);
        frame.pc = (unsigned long)srecorder_unwind_back_trace;
    } 
    else 
    {
        /* task blocked in __switch_to */
        frame.fp = thread_saved_fp(ptask);
        frame.sp = thread_saved_sp(ptask);
        
        /*
      * The function calling __switch_to cannot be a leaf function
      * so LR is recovered from the stack.
      */
        frame.lr = 0;
        frame.pc = thread_saved_pc(ptask);
    }

    while (1) 
    {
        int urc;
        unsigned long where = frame.pc;
        
        urc = srecorder_unwind_frame(&frame);
        if (urc < 0)
        {
            break;
        }
        
        srecorder_dump_back_trace_entry(where, frame.pc, frame.sp - 4);
    }
}
#else
/**
    @function:static void srecorder_dump_back_trace_entry( 
            unsigned long where, unsigned long from, unsigned long frame)
    @brief: 
    @param: where 
    @return: none
    @note: 
*/
void srecorder_dump_back_trace_entry(unsigned long where, unsigned long from, unsigned long frame)
{
#ifdef CONFIG_KALLSYMS
    SRECORDER_SNPRINTF("[<%08lx>] (%pS) from [<%08lx>] (%pS)\n", where, (void *)where, from, (void *)from);
#else
    SRECORDER_SNPRINTF("Function entered at [<%08lx>] from [<%08lx>]\n", where, from);
#endif
    
    if (in_exception_text(where))
    {
        srecorder_dump_memory("Exception stack", frame + 4, frame + 4 + sizeof(struct pt_regs));
    }
}
#endif

#ifndef CONFIG_ARM_UNWIND
extern void * high_memory;

/*
 * Stack pointers should always be within the kernels view of
 * physical memory.  If it is not there, then we can't dump
 * out any information relating to the stack.
 */
static int verify_stack(unsigned long sp)
{
	if (sp < PAGE_OFFSET ||
	    (sp > (unsigned long)high_memory && high_memory != NULL))
		return -EFAULT;

	return 0;
}
#endif

/**
    @function: void srecorder_dump_kernel_back_trace(struct task_struct *ptask)
    @brief: 
    @param: regs 
    @return: none
    @note: 
*/
void srecorder_dump_kernel_back_trace(struct task_struct *ptask)
{
#ifdef CONFIG_ARM_UNWIND
    srecorder_unwind_back_trace(ptask);
#else
    unsigned int fp;
    unsigned int mode;
    int ok = 1;

    if (NULL == ptask)
    {
        ptask = current;
    }

    if (ptask != current) 
    {
        fp = thread_saved_fp(ptask);
        mode = 0x10; /* MODE32_BIT */
    } 
    else 
    {
        asm("mov %0, fp" : "=r" (fp) : : "cc");
        mode = 0x10; /* MODE32_BIT */
    }

    if (0 == fp) 
    {
        SRECORDER_SNPRINTF("%s\n", "no frame pointer");
        ok = 0;
    } 
    else if (verify_stack(fp)) 
    {
        SRECORDER_SNPRINTF("invalid frame pointer 0x%08x\n", fp);
        ok = 0;
    }
    else if (fp < (unsigned long)end_of_stack(ptask))
    {
        SRECORDER_SNPRINTF("%s\n", "frame pointer underflow");
    }
    
    if (0 != ok)
    {
        srecorder_c_backtrace(fp, mode);
    }
#endif
}

#else

/*  FOR ARM64 */

/**
    @function: void srecorder_dump_backtrace_entry64(unsigned long where, unsigned long stack)
    @brief: 
    @param: regs 
    @return: none
    @note: 
*/
void srecorder_dump_backtrace_entry64(unsigned long where, unsigned long stack)
{
    SRECORDER_SNPRINTF("[<%p>] %pS\n", (void *)where, (void *)where);

    if (in_exception_text(where))
    {
        srecorder_dump_memory("Exception stack", stack, stack + sizeof(struct pt_regs));
    }
}

/**
    @function: void srecorder_dump_kernel_back_trace(struct task_struct *ptask)
    @brief: 
    @param: regs 
    @return: none
    @note: 
*/
void srecorder_dump_kernel_back_trace(struct task_struct *ptask)
{
    struct stackframe frame;
    const register unsigned long current_sp asm ("sp");

    if (NULL == ptask)
    {
        ptask = current;
    }

    if (ptask == current)
    {
        frame.fp = (unsigned long)__builtin_frame_address(0);
        frame.sp = current_sp;
        frame.pc = (unsigned long)srecorder_dump_kernel_back_trace;
    } 
    else
    {
        /*
        * task blocked in __switch_to
        */
        frame.fp = thread_saved_fp(ptask);
        frame.sp = thread_saved_sp(ptask);
        frame.pc = thread_saved_pc(ptask);
    }

    while (1)
    {
        unsigned long where = frame.pc;
        int ret;

        ret = unwind_frame(&frame);
        if (ret < 0)
        {
            break;
        }
        srecorder_dump_backtrace_entry64(where, frame.sp);
    }
}
#endif

/**
    @function: int srecorder_dump_current_ps_backtrace()
    @brief: 
    @param: 
    @return: 
    @note: 
*/
int srecorder_dump_back_trace(void)
{
    int is_user_thread = 0; 
    
    if (back_trace_flag == 0)
    {
        SRECORDER_PRINTK("The dump flag of backtrace isn't enabled\n");
        return -1;
    }

    srecorder_log_header_l_start(TYPE_BACK_TRACE);

    srecorder_dump_log_title(TYPE_BACK_TRACE);

    SRECORDER_SNPRINTF("current: pid=%d ppid=%d cpuid=%d name=%s\n", 
        current->pid, current->parent->pid, 
        task_cpu(current), current->comm);
    
    if (NULL != current->mm)
    {
        is_user_thread = 1;
    }

    srecorder_dump_kernel_back_trace(current);

#ifdef CONFIG_ARM
    if (0 != is_user_thread)
    {
        srecorder_dump_user_back_trace();
    }
#endif

    srecorder_log_header_l_end();
    
    return 0;
}
