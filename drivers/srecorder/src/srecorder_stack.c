/**
    @copyright: Huawei Technologies Co., Ltd. 2012-2012. All rights reserved.
    
    @file: srecorder_stack.c
    
    @brief: 
    
    @version: 2.1.1 
    
    @author: Qi Dechun 00216641,    Yan Tongguang 00297150
    
    @date: 2015-03-13
    
    @history:
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/utsname.h>
#include <linux/irq.h>
#include <linux/version.h>

#include "srecorder_stack.h"
#include "srecorder_symbols.h"
#include "srecorder_log.h"
#include "srecorder_back_trace.h"

static void srecorder_show_regs(struct pt_regs *regs, int all);
static void srecorder_show_data(unsigned long addr, int nbytes, const char *name);
static void srecorder_show_extra_register_data(struct pt_regs *regs, int nbytes);
static void __srecorder_show_regs(struct pt_regs *regs);

static int srecorder_trigger_all_cpu_backtrace(void);
static void srecorder_arch_trigger_all_cpu_backtrace(void);
static void srecorder_smp_send_all_cpu_backtrace(void);
static void srecorder_show_stack(struct task_struct *tsk, unsigned long *sp);

static const char *s_processor_modes[] = 
{
    "USER_26", "FIQ_26" , "IRQ_26" , "SVC_26" , "UK4_26" , "UK5_26" , "UK6_26" , "UK7_26" ,
    "UK8_26" , "UK9_26" , "UK10_26", "UK11_26", "UK12_26", "UK13_26", "UK14_26", "UK15_26",
    "USER_32", "FIQ_32" , "IRQ_32" , "SVC_32" , "UK4_32" , "UK5_32" , "UK6_32" , "ABT_32" ,
    "UK8_32" , "UK9_32" , "UK10_32", "UND_32" , "UK12_32", "UK13_32", "UK14_32", "SYS_32"
};

static const char *s_isa_modes[] =
{
    "ARM" , "Thumb" , "Jazelle", "ThumbEE"
};

static int stack_flag = 0;

/**
    @function: void srecorder_enable_stack(void)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_enable_stack(void)
{
    stack_flag = 1;
}

/**
    @function: void srecorder_disable_stack(void)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_disable_stack(void)
{
    stack_flag = 0;
}

/**
    @function: static void srecorder_show_data( 
                unsigned long addr, int nbytes, const char *name)
    @brief: dump a block of kernel memory from around the given address
    @param: addr 
    @param: nbytes 
    @param: name 
    @return: none
    @note: 
**/
static void srecorder_show_data(unsigned long addr, int nbytes, const char *name)
{
    int i;
    int j;
    int nlines;
    u32 *p = NULL;

    /*
   * don't attempt to dump non-kernel addresses or
   * values that are probably just small negative numbers
   */
    if (addr < PAGE_OFFSET || addr > -256UL)
    {
        SRECORDER_PRINTK("File [%s] line [%d] invalid param!\n", __FILE__, __LINE__);
        return;
    }

    SRECORDER_SNPRINTF("\n%s: %#lx:\n", name, addr);
    
    /*
   * round address down to a 32 bit boundary
   * and always dump a multiple of 32 bytes
   */
    p = (u32 *)(addr & ~(sizeof(u32) - 1));
    nbytes += (addr & (sizeof(u32) - 1));
    nlines = (nbytes + 31) / 32; 

    for (i = 0; i < nlines; i++) 
    {
        /*
      * just display low 16 bits of address to keep
      * each line of the dump < 80 characters
      */
        SRECORDER_SNPRINTF("%04lx ", (unsigned long)p & 0xffff);
        for (j = 0; j < 8; j++) 
        {
            u32 data;
            if (probe_kernel_address(p, data)) 
            {
                SRECORDER_SNPRINTF("%s", " ********");
            } 
            else 
            {
                SRECORDER_SNPRINTF(" %08x", data);
            }
            ++p;
        }
        
        SRECORDER_SNPRINTF("%s", "\n");
    }
}

/**
    @function: static void srecorder_show_extra_register_data( 
        struct pt_regs *regs, int nbytes)
    @brief: 
    @param: regs 
    @param: nbytes 
    @return: none
    @note: 
**/
static void srecorder_show_extra_register_data(struct pt_regs *regs, int nbytes)
{
    mm_segment_t fs;

    fs = get_fs();
    set_fs(KERNEL_DS);
    srecorder_show_data( regs->ARM_pc - nbytes, nbytes * 2, "PC");
    srecorder_show_data( regs->ARM_lr - nbytes, nbytes * 2, "LR");
    srecorder_show_data( regs->ARM_sp - nbytes, nbytes * 2, "SP");
    srecorder_show_data( regs->ARM_ip - nbytes, nbytes * 2, "IP");
    srecorder_show_data( regs->ARM_fp - nbytes, nbytes * 2, "FP");
    srecorder_show_data( regs->ARM_r0 - nbytes, nbytes * 2, "R0");
    srecorder_show_data( regs->ARM_r1 - nbytes, nbytes * 2, "R1");
    srecorder_show_data( regs->ARM_r2 - nbytes, nbytes * 2, "R2");
    srecorder_show_data( regs->ARM_r3 - nbytes, nbytes * 2, "R3");
    srecorder_show_data( regs->ARM_r4 - nbytes, nbytes * 2, "R4");
    srecorder_show_data( regs->ARM_r5 - nbytes, nbytes * 2, "R5");
    srecorder_show_data( regs->ARM_r6 - nbytes, nbytes * 2, "R6");
    srecorder_show_data( regs->ARM_r7 - nbytes, nbytes * 2, "R7");
    srecorder_show_data( regs->ARM_r8 - nbytes, nbytes * 2, "R8");
    srecorder_show_data( regs->ARM_r9 - nbytes, nbytes * 2, "R9");
    srecorder_show_data( regs->ARM_r10 - nbytes, nbytes * 2, "R10");
    set_fs(fs);
}

/**
    @function: static void __srecorder_show_regs( struct pt_regs *regs)
    @brief: 
    @param: regs 
    @return: none
    @note: 
**/
static void __srecorder_show_regs(struct pt_regs *regs)
{
    unsigned long flags = 0L;
    char buf[64];
    
    if (NULL == regs)
    {
        SRECORDER_PRINTK("File [%s] line [%d] invalid param!\n", __FILE__, __LINE__);
        return;
    }

    flags = regs->ARM_cpsr;
    buf[0] = flags & PSR_N_BIT ? 'N' : 'n';
    buf[1] = flags & PSR_Z_BIT ? 'Z' : 'z';
    buf[2] = flags & PSR_C_BIT ? 'C' : 'c';
    buf[3] = flags & PSR_V_BIT ? 'V' : 'v';
    buf[4] = '\0';
    SRECORDER_SNPRINTF("CPU: %d    %s  (%s %.*s)\n"
        "PC is at %08lx\n"
        "LR is at %08lx\n"
        "pc : [<%08lx>]    lr : [<%08lx>]    psr: %08lx\n"
        "sp : %08lx  ip : %08lx  fp : %08lx\n"
        "r10: %08lx  r9 : %08lx  r8 : %08lx\n"
        "r7 : %08lx  r6 : %08lx  r5 : %08lx  r4 : %08lx\n"
        "r3 : %08lx  r2 : %08lx  r1 : %08lx  r0 : %08lx\n"
        "Flags: %s  IRQs o%s  FIQs o%s  Mode %s  ISA %s  Segment %s\n",
        raw_smp_processor_id(), 
        print_tainted(),
        init_utsname()->release,
        (int)strcspn(init_utsname()->version, " "),
        init_utsname()->version, 
        instruction_pointer(regs),
        regs->ARM_lr, 
        regs->ARM_pc, 
        regs->ARM_lr, 
        regs->ARM_cpsr,
        regs->ARM_sp, 
        regs->ARM_ip, 
        regs->ARM_fp, 
        regs->ARM_r10, 
        regs->ARM_r9,
        regs->ARM_r8, 
        regs->ARM_r7, 
        regs->ARM_r6,
        regs->ARM_r5, 
        regs->ARM_r4, 
        regs->ARM_r3, 
        regs->ARM_r2,
        regs->ARM_r1, 
        regs->ARM_r0, 
        buf, 
        interrupts_enabled(regs) ? "n" : "ff",
        fast_interrupts_enabled(regs) ? "n" : "ff",
        s_processor_modes[processor_mode(regs)],
        s_isa_modes[isa_mode(regs)],
        (get_fs() == get_ds()) ? ("kernel") : ("user"));
    
#ifdef CONFIG_CPU_CP15
    {
        unsigned int ctrl;

        buf[0] = '\0';
#ifdef CONFIG_CPU_CP15_MMU
        {
            unsigned int transbase, dac;
            asm("mrc p15, 0, %0, c2, c0\n\t"
                "mrc p15, 0, %1, c3, c0\n"
                : "=r" (transbase), "=r" (dac));
            SRECORDER_SNPRINTF("  Table: %08x  DAC: %08x", transbase, dac);
        }
#endif
        asm("mrc p15, 0, %0, c1, c0\n" : "=r" (ctrl));
        
        SRECORDER_SNPRINTF("Control: %08x%s\n", ctrl, buf);
    }
#endif
    
    srecorder_show_extra_register_data( regs, 128); 
}

/**
    @function: static void srecorder_show_regs( struct pt_regs *regs, int all)
    @brief: 
    @param: regs 
    @param: all 
    @return: none
    @note: 
**/
static void srecorder_show_regs(struct pt_regs *regs, int all)
{
    if (NULL == regs)
    {
        SRECORDER_PRINTK("File [%s] line [%d] invalid param!\n", __FILE__, __LINE__);
        return;
    }
    
    SRECORDER_SNPRINTF("\nPid: %d, comm: %20s\n", task_pid_nr(current), current->comm);
    
    __srecorder_show_regs( regs);
    srecorder_dump_kernel_back_trace(NULL);
}

/**
    @function: static void srecorder_show_stack( 
                struct task_struct *tsk, unsigned long *sp)
    @brief: 
    @param: tsk 
    @param: sp 
    @return: none
    @note: 
**/
static void srecorder_show_stack(struct task_struct *tsk, unsigned long *sp)
{
    srecorder_dump_kernel_back_trace(tsk);
    barrier();
}

/**
    @function: static void srecorder_smp_send_all_cpu_backtrace()
    @brief: 
    @return: none
    @note: 
**/
static void srecorder_smp_send_all_cpu_backtrace(void)
{
    struct task_struct *g = NULL;
    struct task_struct *p = NULL;
    
    SRECORDER_SNPRINTF("%s", ">>>>> Do backtrace for all threads <<<<<\n");

    if (read_trylock(&tasklist_lock))
    {
        do_each_thread(g, p) 
        {
            /*
          * reset the NMI-timeout, listing all files on a slow
          * console might take a lot of time:
          */
            const char *stat_nam = TASK_STATE_TO_CHAR_STR;
            unsigned long free = 0;
            unsigned state = p->state ? __ffs(p->state) + 1 : 0;

            if ((TASK_RUNNING == p->state) || (TASK_UNINTERRUPTIBLE == (p->state & TASK_UNINTERRUPTIBLE)))
            {
                SRECORDER_SNPRINTF("\ntask:%s %c", p->comm, state < sizeof(stat_nam) - 1 ? stat_nam[state] : '?');
#if BITS_PER_LONG == 32
                if (state == TASK_RUNNING)
                {
                    SRECORDER_SNPRINTF("%s", " running ");
                }
                else
                {
                    SRECORDER_SNPRINTF(" PC:%08lx ", thread_saved_pc(p));
                }
#else
                if (state == TASK_RUNNING)
                {
                    SRECORDER_SNPRINTF("%s", " running task ");
                }
                else
                {
                    SRECORDER_SNPRINTF(" PC:%016lx ", thread_saved_pc(p));
                }
#endif
                
#ifdef CONFIG_DEBUG_STACK_USAGE
                free = stack_not_used(p);
#endif

                SRECORDER_SNPRINTF("stack:%lu pid:%d father:%d flags:0x%08lx\n", 
                    free, task_pid_nr(p), task_pid_nr(p->real_parent), (unsigned long)task_thread_info(p)->flags);
                
                srecorder_show_stack( p, NULL);
            }
        } while_each_thread(g, p);
        
        read_unlock(&tasklist_lock);
    }
}

/**
    @function: static void srecorder_arch_trigger_all_cpu_backtrace()
    @brief: 
    @return: none
    @note: 
**/
static void srecorder_arch_trigger_all_cpu_backtrace(void)
{
    srecorder_smp_send_all_cpu_backtrace();
}

/**
    @function: static int srecorder_trigger_all_cpu_backtrace()
    @brief: 
    @return: 
    @note: 
**/
static int srecorder_trigger_all_cpu_backtrace(void)
{
    srecorder_arch_trigger_all_cpu_backtrace();
    
    return 0;
}

/**
    @function: int srecorder_dump_stack()
    @brief: 
    @return: 
    @note: 
**/
int srecorder_dump_stack(void)
{
    if (stack_flag == 0)
    {
        SRECORDER_PRINTK("The dump flag of stack isn't enabled\n");
        return -1;
    }

    srecorder_log_header_l_start(TYPE_STACK);

    srecorder_dump_log_title(TYPE_STACK);

    if (0 > srecorder_trigger_all_cpu_backtrace())
    {
        struct pt_regs *regs = get_irq_regs();    
        
        if (NULL != regs) 
        {
            SRECORDER_SNPRINTF("CPU%d:\n", smp_processor_id());
            srecorder_show_regs( regs, 1);
        }
    }

    srecorder_log_header_l_end();
    
    return 0;
}
