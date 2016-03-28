/**
    @copyright: Huawei Technologies Co., Ltd. 2012-2012. All rights reserved.
    
    @file: srecorder_log.c
    
    @brief: 
    
    @version: 2.1.1 
    
    @author: Qi Dechun 00216641,    Yan Tongguang 00297150
    
    @date: 2015-03-13
    
    @history:
**/

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/highmem.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/of.h>

#include <asm/uaccess.h>

#include "srecorder_log.h"
#include "srecorder_crc32.h"
#include "srecorder_reason_time.h"
#include "srecorder_dmesg.h"
#include "srecorder_back_trace.h"
#include "srecorder_ps_info.h"
#include "srecorder_stack.h"
#include "srecorder_sys_info.h"
#include "srecorder_slab_info.h"
#include "srecorder_interface.h"

#ifndef CONFIG_SRECORDER_LOG_BUF_LEN
#define CONFIG_SRECORDER_LOG_BUF_LEN 0x200000
#endif

#define FLAG_REASON_TIME        0x0001
#define FLAG_SYS_INFO           0x0002
#define FLAG_STACK              0x0004
#define FLAG_PS_INFO            0x0008
#define FLAG_BACK_TRACE         0x0010
#define FLAG_SLAB_INFO          0x0020
#define FLAG_PREV               0x0040
#define FLAG_CURR               0x0080
#define FLAG_DMESG              0x0100
#define FLAG_EXT                0x0400

/* log title strings */
static char* s_log_titles[] = 
{
    TITLE_REASON_TIME,
    TITLE_SYS_INFO, 
    TITLE_STACK,
    TITLE_PS_INFO,
    TITLE_BACK_TRACE,
    TITLE_SLAB_INFO,
    TITLE_DMESG,
};

static DEFINE_SPINLOCK(srecorder_buffer_lock);
static DECLARE_COMPLETION(srecorder_log_completion);

static int log_dumped_flag = 0;

/* global variables for dts buffer addresses and size */
unsigned long srecorder_imem_va = 0;

/* global variables for dts buffer addresses and size */
unsigned long srecorder_dts_pa = 0;
unsigned long srecorder_dts_va = 0;
unsigned long srecorder_dts_size = 0;

static log_header_h_t log_header_h_src;

/* the array address of middle log header in dts buffer */
static log_header_m_t* log_header_m_src = NULL;

static log_header_l_t log_header_l_src;

static addr_info_t log_header_l_dst_addr;

typedef struct
{
    addr_index_t idx;
    log_category_e category;
    int (*init_log_info)(unsigned long* p_log_buf_pa, unsigned long* p_log_buf_va, unsigned* p_log_len, unsigned* p_log_buf_len);
    void (*exit_module)(void);
    int (*get_log_info)(unsigned long* p_log_buf_va, unsigned* p_log_len);
    void (*reset_log_info)(void);
} srecorder_buffer_operations_t;

typedef struct
{
    int (*init_module)(void);
    void (*exit_module)(void);
    int (*dump_sub_log)(void);
    void (*enable_sub_log)(void);
    void (*disable_sub_log)(void);
} srecorder_sublog_operations_t;

extern unsigned long cma_get_base_by_name(const char *name);
extern unsigned long cma_get_size_by_name(const char *name);

int srecorder_init_previous_log_info(unsigned long* p_log_buf_pa, unsigned long* p_log_buf_va, unsigned* p_log_len, unsigned* p_log_buf_len);
int srecorder_init_current_log_info(unsigned long* p_log_buf_pa, unsigned long* p_log_buf_va, unsigned* p_log_len, unsigned* p_log_buf_len);

int srecorder_get_previous_log_info(unsigned long* p_log_buf_va, unsigned* p_log_len);

static srecorder_buffer_operations_t s_srecorder_buffer_operations[] = 
{
    {IDX_PREV, CATEGORY_PREV, srecorder_init_previous_log_info, NULL, srecorder_get_previous_log_info, srecorder_reset_previous_log_info},
    /* no srecorder_get_current_log_info and srecorder_reset_current_log_info, cause it's only dumped during next booting */
    {IDX_CURR, CATEGORY_CURR, srecorder_init_current_log_info, NULL, NULL, NULL},
    /* no srecorder_get_dmesg_log_info and srecorder_reset_dmesg_log_info, cause it's only necessary for reset abnormal */
    {IDX_DMESG, CATEGORY_DMESG, srecorder_init_dmesg_log_info, NULL, NULL, NULL},
    /* no srecorder_init_external_log_info, cause it only can be registered after initialization */
    {IDX_EXT, CATEGORY_EXT, NULL, NULL, srecorder_get_external_log_info, srecorder_reset_external_log_info},
};

static srecorder_sublog_operations_t s_srecorder_sublog_operations[] = 
{
    {NULL, NULL, srecorder_dump_reason_time, srecorder_enable_reason_time, srecorder_disable_reason_time},
    {NULL, NULL, srecorder_dump_back_trace, srecorder_enable_back_trace, srecorder_disable_back_trace},
    {NULL, NULL, srecorder_dump_ps_info, srecorder_enable_ps_info, srecorder_disable_ps_info},
#ifdef CONFIG_ARM
    {NULL, NULL, srecorder_dump_stack, srecorder_enable_stack, srecorder_disable_stack},
#endif
    {NULL, NULL, srecorder_dump_sys_info, srecorder_enable_sys_info, srecorder_disable_sys_info},
    {NULL, NULL, srecorder_dump_slab_info, srecorder_enable_slab_info, srecorder_disable_slab_info},
};

/**
    @function: int srecorder_verify_log_header_h(log_header_h_t* p_log_header_h)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
int srecorder_verify_log_header_h(log_header_h_t* p_log_header_h)
{
    uint32_t crc32 = 0;

    if (unlikely(NULL == p_log_header_h))
    {
        return -1;
    }

    crc32 = srecorder_calculate_crc32((unsigned char const *)p_log_header_h, sizeof(log_header_h_t) - sizeof(p_log_header_h->crc32));
    if (crc32 != p_log_header_h->crc32)
    {
        return -1;
    }

    if (p_log_header_h->magic_num != SRECORDER_MAGIC_NUMBER)
    {
        return -1;
    }

    if (p_log_header_h->addr == 0)
    {
        return -1;
    }

    if (p_log_header_h->size == 0)
    {
        return -1;
    }

    return 0;
}

/**
    @function: int srecorder_verify_log_header_m(log_header_m_t* p_log_header_m, int log_len_check)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
int srecorder_verify_log_header_m(log_header_m_t* p_log_header_m, int log_len_check)
{
    uint32_t crc32 = 0;

    if (unlikely(NULL == p_log_header_m))
    {
        return -1;
    }

    crc32 = srecorder_calculate_crc32((unsigned char const *)p_log_header_m, sizeof(log_header_m_t) - sizeof(p_log_header_m->crc32) - sizeof(p_log_header_m->padding));
    if (crc32 != p_log_header_m->crc32)
    {
        return -1;
    }

    if (p_log_header_m->version != CONFIG_SRECORDER_VERSION)
    {
        return -1;
    }

    if (p_log_header_m->magic_num != SRECORDER_MAGIC_NUMBER)
    {
        return -1;
    }

    if (p_log_header_m->log_buf.pa == 0 && p_log_header_m->log_buf.va == 0)
    {
        return -1;
    }

    if (p_log_header_m->log_buf_len == 0)
    {
        return -1;
    }

    /* log_len will be zero when the log category is dmesg and ext */
    if (log_len_check == true && p_log_header_m->log_len == 0)
    {
        return -1;
    }

    if (p_log_header_m->log_buf_len < p_log_header_m->log_len)
    {
        return -1;
    }

    return 0;
}

/**
    @function: int srecorder_verify_log_header_l(log_header_l_t* p_log_header_l)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
int srecorder_verify_log_header_l(log_header_l_t* p_log_header_l)
{
    uint32_t crc32 = 0;

    if (unlikely(NULL == p_log_header_l))
    {
        return -1;
    }

    crc32 = srecorder_calculate_crc32((unsigned char const*)p_log_header_l, sizeof(log_header_l_t) - sizeof(p_log_header_l->crc32));
    if (crc32 != p_log_header_l->crc32)
    {
        return -1;
    }

    if (p_log_header_l->log_type < TYPE_MIN || p_log_header_l->log_type > TYPE_MAX)
    {
        return -1;
    }

    if (p_log_header_l->log_len == 0)
    {
        return -1;
    }

    return 0;
}

/**
    @function: void srecorder_update_log_header_h(log_header_h_t* p_log_header_h)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
void srecorder_update_log_header_h(log_header_h_t* p_log_header_h)
{
    if (unlikely(NULL == p_log_header_h))
    {
        return;
    }

    p_log_header_h->crc32 = srecorder_calculate_crc32((unsigned char const*)p_log_header_h, sizeof(log_header_h_t) - sizeof(p_log_header_h->crc32));

    return;
}

/**
    @function: void srecorder_update_log_header_m(log_header_m_t* p_log_header_m)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
void srecorder_update_log_header_m(log_header_m_t* p_log_header_m)
{
    if (unlikely(NULL == p_log_header_m))
    {
        return;
    }

    p_log_header_m->crc32 = srecorder_calculate_crc32((unsigned char const*)p_log_header_m, sizeof(log_header_m_t) - sizeof(p_log_header_m->crc32) - sizeof(p_log_header_m->padding));

    return;
}

/**
    @function: void srecorder_update_log_header_l(log_header_l_t* p_log_header_l)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
void srecorder_update_log_header_l(log_header_l_t* p_log_header_l)
{
    if (unlikely(NULL == p_log_header_l))
    {
        return;
    }

    p_log_header_l->crc32 = srecorder_calculate_crc32((unsigned char const*)p_log_header_l, sizeof(log_header_l_t) - sizeof(p_log_header_l->crc32));

    return;
}

/**
    @function: static void srecorder_set_reset_type(reset_type_e reset_type)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
static void srecorder_set_reset_type(reset_type_e reset_type)
{
    log_header_h_src.reset_flag = reset_type;
    srecorder_update_log_header_h(&log_header_h_src);
}

/**
    @function: static void srecorder_set_dumped_flag(int flag)
    @brief: 
    @return: 
    @note: 
**/
static void srecorder_set_dumped_flag(int flag)
{
    log_dumped_flag = flag;
}

/**
    @function: static int srecorder_get_dumped_flag(void)
    @brief: 
    @return: 
    @note: 
**/
static int srecorder_get_dumped_flag(void)
{
    return log_dumped_flag;
}

/**
    @function: int srecorder_ioremap_is_ready(void)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
int srecorder_ioremap_is_ready(void)
{
    /* ioremap is ok only when all vitual addresses are ok */
    return (srecorder_imem_va != 0 && srecorder_dts_pa != 0 && srecorder_dts_va != 0);
}

/**
    @function: int srecorder_dts_is_matched(void)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
int srecorder_dts_is_matched(void)
{
    log_header_h_t log_header_h;

    if (srecorder_ioremap_is_ready() == 0)
    {
        return 0;
    }

    /* retrieve the high header from imem */
    memcpy((void*)&log_header_h, (void*)srecorder_imem_va, sizeof(log_header_h_t));
    if (srecorder_verify_log_header_h(&log_header_h))
    {
        return 0;
    }

    /* return if the previous dts settings don't match the current ones */
    if (log_header_h.addr != srecorder_dts_pa || log_header_h.size > srecorder_dts_size)
    {
        return 0;
    }

    return 1;
}

 /**
    @function: void srecorder_ioremap_dts_memory(void)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
void srecorder_ioremap_dts_memory(void)
{
    /* the high header in imem */
#ifdef CONFIG_ARM
    srecorder_imem_va = (unsigned long)ioremap_nocache(CONFIG_SRECORDER_IMEM_HEADER_PHYS_ADDR, CONFIG_SRECORDER_IMEM_HEADER_PHYS_SIZE);
#else
    srecorder_imem_va = (unsigned long)ioremap_wc(CONFIG_SRECORDER_IMEM_HEADER_PHYS_ADDR, CONFIG_SRECORDER_IMEM_HEADER_PHYS_SIZE);
#endif
    if (srecorder_imem_va == 0)
    {
        SRECORDER_PRINTK("Remapping imem failed.\n");
        return;
    }

    /* get the phsical address and size of the dynamically allocated dts buffer */
    srecorder_dts_pa = cma_get_base_by_name("srecorder_mem"); 
    srecorder_dts_size = cma_get_size_by_name("srecorder_mem");
    if (srecorder_dts_pa == 0 || srecorder_dts_size < (2*CONFIG_SRECORDER_LOG_BUF_LEN))
    {
        SRECORDER_PRINTK("Wrong address or size of the dts buffer.\n");
        srecorder_dts_pa = 0;
        srecorder_dts_size = 0;
        iounmap((void*)srecorder_imem_va);
        return;
    }

    /* ioremap the dynamically allocated dts buffer */
#ifdef CONFIG_ARM
    srecorder_dts_va = (unsigned long)ioremap_nocache(srecorder_dts_pa, srecorder_dts_size);
#else
    srecorder_dts_va = (unsigned long)ioremap_wc(srecorder_dts_pa, srecorder_dts_size);
#endif
    if (srecorder_dts_va == 0)
    {
        SRECORDER_PRINTK("Remapping the dts buffer failed.\n");
        srecorder_dts_pa = 0;
        srecorder_dts_size = 0;
        iounmap((void*)srecorder_imem_va);
        return;
    }
}

/**
    @function: void srecorder_iounremap_dts_memory(void)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
void srecorder_iounremap_dts_memory(void)
{
    iounmap((void*)srecorder_imem_va);
    iounmap((void*)srecorder_dts_va);
}

/**
    @function: void srecorder_sync_log_header_h(void)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
void srecorder_sync_log_header_h(void)
{
    /* ioremap is ok only when all vitual addresses are ok */
    if (srecorder_ioremap_is_ready())
    {
        /* sync the log header with physical address */
        memcpy((void*)srecorder_imem_va, (void*)&log_header_h_src, sizeof(log_header_h_t));
    }
}

/**
    @function: void srecorder_connect_log_headers(void)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
void srecorder_connect_log_headers(void)
{
    int i;

    /* ioremap is ok only when all vitual addresses are ok */
    if (srecorder_ioremap_is_ready() == 0)
    {
        return;
    }

    /* initialize the high header */
    memset(&log_header_h_src, 0, sizeof(log_header_h_t));

    log_header_h_src.magic_num = SRECORDER_MAGIC_NUMBER;
    log_header_h_src.addr = srecorder_dts_pa;
    log_header_h_src.size = srecorder_dts_size;
    log_header_h_src.reset_flag = 0;
    srecorder_update_log_header_h(&log_header_h_src);

    log_header_m_src = (log_header_m_t *)srecorder_dts_va;

    /* initialize the middles headers */
    for (i=IDX_MIN; i<=IDX_MAX; i++)
    {
        /* log_header_m of previous log will be reset if the current buffer address/size don't match the previous address/size */
        if (i == IDX_PREV && srecorder_dts_is_matched())
        {
            continue;
        }

        memset(log_header_m_src + i, 0, sizeof(log_header_m_t));

        log_header_m_src[i].version = CONFIG_SRECORDER_VERSION;
        log_header_m_src[i].magic_num = SRECORDER_MAGIC_NUMBER;
        log_header_m_src[i].log_buf.pa = 0;
        log_header_m_src[i].log_buf.va = 0;
        log_header_m_src[i].log_len = 0;
        log_header_m_src[i].log_buf_len = 0;
        srecorder_update_log_header_m(log_header_m_src + i);
    }

    /* initialize the low header */
    memset(&log_header_l_src, 0, sizeof(log_header_l_t));

    srecorder_update_log_header_l(&log_header_l_src);
}

/**
    @function: void srecorder_connect_log_headers_with_buffers(void)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
void srecorder_connect_log_headers_with_buffers(void)
{
    int i;
    int idx;
    int array_size;
    int log_len;
    int log_buf_len;
    unsigned long pa;
    unsigned long va;

    if (log_header_m_src == NULL)
    {
        return;
    }

    /* after above cleaning, initial information is needed */
    array_size = sizeof(s_srecorder_buffer_operations) / sizeof(s_srecorder_buffer_operations[0]);
    for (i = 0; i < array_size; i++)
    {
        if (NULL != s_srecorder_buffer_operations[i].init_log_info)
        {
            idx = s_srecorder_buffer_operations[i].idx;
            if (idx < IDX_MIN || idx > IDX_MAX)
            {
                continue;
            }

            if (s_srecorder_buffer_operations[i].init_log_info(&pa, &va, &log_len, &log_buf_len) == 0)
            {
                /* most members of log_header_m of previous log are skipped, cause it has been initialized during booting */
                if (i == IDX_PREV)
                {
                    /* virtual addres is useful only if the current buffer address/size match the previous address/size */
                    if (srecorder_dts_is_matched())
                    {
                        log_header_m_src[idx].log_buf.va = va;
                        srecorder_update_log_header_m(log_header_m_src + i);
                    }
                }
                else
                {
                    log_header_m_src[idx].log_buf.pa = pa;
                    log_header_m_src[idx].log_buf.va = va;
                    log_header_m_src[idx].log_len = log_len;
                    log_header_m_src[idx].log_buf_len = log_buf_len;
                    srecorder_update_log_header_m(log_header_m_src + idx);
                }
            }
        }
    }
}

/**
    @function: void srecorder_init_log_headers(void)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
void srecorder_init_log_headers(void)
{
    spin_lock(&srecorder_buffer_lock);

    srecorder_connect_log_headers();

    srecorder_connect_log_headers_with_buffers();

    /* it will be set to RESET_NORMAL if the machine reboots normally */
    srecorder_set_reset_type(RESET_COLLAPSE);

    srecorder_sync_log_header_h();

    srecorder_set_dumped_flag(0);

    spin_unlock(&srecorder_buffer_lock);
}

/**
    @function: void srecorder_log_header_m_start(void)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
void srecorder_log_header_m_start(void)
{
    if (log_header_m_src == NULL)
    {
        return;
    }

    log_header_m_src[IDX_CURR].log_len = 0;
    srecorder_update_log_header_m(log_header_m_src + IDX_CURR);
}

/**
    @function: void srecorder_log_header_m_end(void)
    @brief: this function does nothing, and it exists to pair with srecorder_log_header_m_start 
    @param: reason
    @return: none
    @note:
**/
void srecorder_log_header_m_end(void)
{
    if (log_header_m_src == NULL)
    {
        return;
    }

    srecorder_update_log_header_m(log_header_m_src + IDX_CURR);
    return;
}

/**
    @function: void srecorder_log_header_l_start(log_type_e log_type)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
void srecorder_log_header_l_start(log_type_e log_type)
{
    if (log_header_m_src == NULL)
    {
        return;
    }

    log_header_l_dst_addr.pa = log_header_m_src[IDX_CURR].log_buf.pa + log_header_m_src[IDX_CURR].log_len;
    log_header_l_dst_addr.va = log_header_m_src[IDX_CURR].log_buf.va + log_header_m_src[IDX_CURR].log_len;

    log_header_l_src.log_type = log_type;
    log_header_l_src.log_len = 0;
    srecorder_update_log_header_l(&log_header_l_src);

    log_header_m_src[IDX_CURR].log_len += sizeof(log_header_l_t);
    srecorder_update_log_header_m(log_header_m_src + IDX_CURR);
}

/**
    @function: void srecorder_log_header_l_end(void)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
void srecorder_log_header_l_end(void)
{
    if (log_header_m_src == NULL)
    {
        return;
    }

    srecorder_update_log_header_l(&log_header_l_src);
    memcpy((void*)log_header_l_dst_addr.va, (const void*)&log_header_l_src, sizeof(log_header_l_t));
}

/**
    @function: int srecorder_snprintf(const char *fmt, ...)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
int srecorder_snprintf(const char *fmt, ...)
{
    int left_size = 0;
    int copy_size = 0;
    char* buf_pos = NULL;

    va_list args;

    if (log_header_m_src == NULL)
    {
        return 0;
    }

    if (srecorder_verify_log_header_h(&log_header_h_src))
    {
        return 0;
    }

    /* for the very first time the log len is zero */
    if (srecorder_verify_log_header_m(log_header_m_src + IDX_CURR, false))
    {
        return 0;
    }

    left_size = log_header_m_src[IDX_CURR].log_buf_len - log_header_m_src[IDX_CURR].log_len;
    if (left_size == 0)
    {
        return 0;
    }

    buf_pos = (char*)(log_header_m_src[IDX_CURR].log_buf.va + log_header_m_src[IDX_CURR].log_len);

    va_start(args, fmt);
    copy_size = vscnprintf(buf_pos, left_size, fmt, args);
    va_end(args);

    log_header_m_src[IDX_CURR].log_len += copy_size;
    srecorder_update_log_header_m(log_header_m_src + IDX_CURR);

    log_header_l_src.log_len += copy_size;
    srecorder_update_log_header_l(&log_header_l_src);

    return copy_size;
}

/**
    @function: void srecorder_dump_log_title(log_type_e type)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
void srecorder_dump_log_title(log_type_e type)
{
    if (type < TYPE_MIN || type > TYPE_MAX)
    {
        return;
    }

    SRECORDER_SNPRINTF("%s", s_log_titles[type]);

    return;
}

/**
    @function: static void srecorder_dump_common_log(void)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
static void srecorder_dump_common_log(void)
{
    int i = 0;
    int array_size = sizeof(s_srecorder_sublog_operations) / sizeof(s_srecorder_sublog_operations[0]);

    srecorder_log_header_m_start();

    for (i = 0; i < array_size; i++)
    {
        if (NULL != s_srecorder_sublog_operations[i].dump_sub_log)
        {
            s_srecorder_sublog_operations[i].dump_sub_log();
        }
    }

    srecorder_log_header_m_end();

    srecorder_set_reset_type(RESET_APANIC);

    srecorder_sync_log_header_h();
}

/**
    @function: int srecorder_enable_log_category_flag(unsigned flag)
    @brief: switch statement is kept for extension
    @param: 
    @return: 
    @note:
**/
int srecorder_enable_log_category_flag(unsigned flag)
{
    int ret = -1;

    if (spin_trylock(&srecorder_buffer_lock))
    {
        switch (flag)
        {
        case FLAG_EXT:
            //srecorder_enable_external_category();
            ret = 0;
            break;
        default:
            ret = -1;
        }

        spin_unlock(&srecorder_buffer_lock);
    }

    return ret;
}

/**
    @function: int srecorder_disable_log_category_flag(unsigned flag)
    @brief: switch statement is kept for extension
    @param: 
    @return: 
    @note:
**/
int srecorder_disable_log_category_flag(unsigned flag)
{
    int ret = -1;

    if (spin_trylock(&srecorder_buffer_lock))
    {
        switch (flag)
        {
        case FLAG_EXT:
            //srecorder_disable_external_category();
            ret = 0;
            break;
        default:
            ret = -1;
        }

        spin_unlock(&srecorder_buffer_lock);
    }

    return ret;
}

/**
    @function: void srecorder_enable_log_category_flags()
    @brief: this interface is kept for extension
    @param: 
    @return: 
    @note:
**/
void srecorder_enable_log_category_flags(void)
{
    /* now we only have one log category */
    //srecorder_enable_external_category();
}

/**
    @function: void srecorder_disable_category_flags()
    @brief: this interface is kept for extension
    @param: 
    @return: 
    @note:
**/
void srecorder_disable_log_category_flags(void)
{
    /* now we only have one log category */
    //srecorder_disable_external_category();
}

/**
    @function: int srecorder_enable_log_type_flag(unsigned flag)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
int srecorder_enable_log_type_flag(unsigned flag)
{
    int ret = -1;

    if (spin_trylock(&srecorder_buffer_lock))
    {
        switch (flag)
        {
        case FLAG_REASON_TIME:
            srecorder_enable_reason_time();
            ret = 0;
            break;
        case FLAG_SYS_INFO:
            srecorder_enable_sys_info();
            ret = 0;
            break;
        case FLAG_STACK:
#ifdef CONFIG_ARM
            srecorder_enable_stack();
#endif
            ret = 0;
            break;
        case FLAG_PS_INFO:
            srecorder_enable_ps_info();
            ret = 0;
            break;
        case FLAG_BACK_TRACE:
            srecorder_enable_back_trace();
            ret = 0;
            break;
        case FLAG_SLAB_INFO:
            srecorder_enable_slab_info();
            ret = 0;
            break;
        case FLAG_DMESG:
            srecorder_enable_dmesg();
            ret = 0;
            break;
        default:
            ret = -1;
        }

        spin_unlock(&srecorder_buffer_lock);
    }

    return ret;
}

/**
    @function: int srecorder_disable_log_type_flag(unsigned flag)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
int srecorder_disable_log_type_flag(unsigned flag)
{
    int ret = -1;

    if (spin_trylock(&srecorder_buffer_lock))
    {
        switch (flag)
        {
        case FLAG_REASON_TIME:
            srecorder_disable_reason_time();
            ret = 0;
            break;
        case FLAG_SYS_INFO:
            srecorder_disable_sys_info();
            ret = 0;
            break;
        case FLAG_STACK:
#ifdef CONFIG_ARM
            srecorder_disable_stack();
#endif
            ret = 0;
            break;
        case FLAG_PS_INFO:
            srecorder_disable_ps_info();
            ret = 0;
            break;
        case FLAG_BACK_TRACE:
            srecorder_disable_back_trace();
            ret = 0;
            break;
        case FLAG_SLAB_INFO:
            srecorder_disable_slab_info();
            ret = 0;
            break;
        case FLAG_DMESG:
            srecorder_disable_dmesg();
            ret = 0;
            break;
        default:
            ret = -1;
        }

        spin_unlock(&srecorder_buffer_lock);
    }

    return ret;
}

/**
    @function: void srecorder_enable_log_type_flags(void)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
void srecorder_enable_log_type_flags(void)
{
    srecorder_enable_reason_time();
    srecorder_enable_sys_info();
#ifdef CONFIG_ARM
    srecorder_enable_stack();
#endif
    srecorder_enable_ps_info();
    srecorder_enable_back_trace();
    srecorder_enable_slab_info();
    srecorder_enable_dmesg();
}

/**
    @function: void srecorder_disable_log_type_flags(void)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
void srecorder_disable_log_type_flags(void)
{
    srecorder_disable_reason_time();
    srecorder_disable_sys_info();
#ifdef CONFIG_ARM
    srecorder_disable_stack();
#endif
    srecorder_disable_ps_info();
    srecorder_disable_back_trace();
    srecorder_disable_slab_info();
    srecorder_disable_dmesg();
}

/**
    @function: int srecorder_get_first_log_info(log_category_e* log_category, unsigned long* log_buf, unsigned* log_len)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
int srecorder_get_first_log_info(log_category_e* log_category, unsigned long* log_buf, unsigned* log_len)
{
    int i = 0;
    int array_size = sizeof(s_srecorder_buffer_operations) / sizeof(s_srecorder_buffer_operations[0]);

    if (spin_trylock(&srecorder_buffer_lock))
    {
        for (i = 0; i < array_size; i++)
        {
            if (NULL != s_srecorder_buffer_operations[i].get_log_info)
            {
                if (s_srecorder_buffer_operations[i].get_log_info(log_buf, log_len) == 0)
                {
                    SRECORDER_INFO("Srecorder log info: %lx %x %x\n", *log_buf, *log_len, (unsigned)*log_category);
                    /* ugly trick */
                    *log_category = s_srecorder_buffer_operations[i].category;

                    spin_unlock(&srecorder_buffer_lock);
                    return 0;
                }
            }
        }

        spin_unlock(&srecorder_buffer_lock);
    }

    return -1;
}

/**
    @function: int srecorder_reset_first_log_info(void)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
int srecorder_reset_first_log_info(void)
{
    int i = 0;
    int log_len = 0;
    unsigned long log_buf = 0;
    int array_size = sizeof(s_srecorder_buffer_operations) / sizeof(s_srecorder_buffer_operations[0]);

    spin_lock(&srecorder_buffer_lock);

    for (i = 0; i < array_size; i++)
    {
        if (NULL != s_srecorder_buffer_operations[i].get_log_info)
        {
            if (s_srecorder_buffer_operations[i].get_log_info(&log_buf, &log_len) == 0)
            {
                if (NULL != s_srecorder_buffer_operations[i].reset_log_info)
                {
                    s_srecorder_buffer_operations[i].reset_log_info();

                    spin_unlock(&srecorder_buffer_lock);
                    return 0;
                }
            }
        }
    }

    spin_unlock(&srecorder_buffer_lock);
    return -1;
}

/**
    @function: int srecorder_get_previous_log_info(unsigned long* p_log_buf_va, unsigned* p_log_len)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
int srecorder_get_previous_log_info(unsigned long* p_log_buf_va, unsigned* p_log_len)
{
    if (log_header_m_src == NULL)
    {
        return -1;
    }

    if (srecorder_verify_log_header_m(log_header_m_src + IDX_PREV, true))
    {
        return -1;
    }

    *p_log_buf_va = log_header_m_src[IDX_PREV].log_buf.va;
    *p_log_len = log_header_m_src[IDX_PREV].log_len;

    return 0;
}

/**
    @function: void srecorder_reset_previous_log_info()
    @brief: this function will check and free the virtual momery allocated during booting
    @param: reason
    @return: none
    @note:
**/
void srecorder_reset_previous_log_info(void)
{
    if (log_header_m_src == NULL)
    {
        return;
    }

    log_header_m_src[IDX_PREV].log_buf.pa = 0;
    log_header_m_src[IDX_PREV].log_buf.va = 0;

    log_header_m_src[IDX_PREV].log_len = 0;
    log_header_m_src[IDX_PREV].log_buf_len = 0;

    srecorder_update_log_header_m(log_header_m_src + IDX_PREV);
}

/**
    @function: int srecorder_init_previous_log_info(unsigned long* p_log_buf_pa, unsigned long* p_log_buf_va, unsigned* p_log_len, unsigned* p_log_buf_len)
    @brief: all members except base.va has been initialized during booting
    @param: reason
    @return: none
    @note:
**/
int srecorder_init_previous_log_info(unsigned long* p_log_buf_pa, unsigned long* p_log_buf_va, unsigned* p_log_len, unsigned* p_log_buf_len)
{
    if (srecorder_ioremap_is_ready() == 0)
    {
        return -1;
    }

    *p_log_buf_va = srecorder_dts_va + CONFIG_SRECORDER_LOG_BUF_LEN;

    return 0;
}

/**
    @function: int srecorder_init_current_log_info(unsigned long* p_log_buf_pa, unsigned long* p_log_buf_va, unsigned* p_log_len, unsigned* p_log_buf_len)
    @brief: dump 
    @param: reason
    @return: none
    @note:
**/
int srecorder_init_current_log_info(unsigned long* p_log_buf_pa, unsigned long* p_log_buf_va, unsigned* p_log_len, unsigned* p_log_buf_len)
{
    if (srecorder_ioremap_is_ready() == 0)
    {
        return -1;
    }

    *p_log_buf_pa = srecorder_dts_pa + IDX_COUNT * sizeof(log_header_m_t);
    *p_log_buf_va = srecorder_dts_va + IDX_COUNT * sizeof(log_header_m_t);
    *p_log_len = 0;
    *p_log_buf_len = CONFIG_SRECORDER_LOG_BUF_LEN;

    return 0;
}

/**
    @function: void srecorder_dump_dmesg_info(void)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_dump_dmesg_info(void)
{
    int log_len = 0;
    int log_buf_len = 0;
    unsigned long pa = 0UL;
    unsigned long va = 0UL;

    if (log_header_m_src == NULL)
    {
        return;
    }

    srecorder_init_dmesg_log_info(&pa, &va, &log_len, &log_buf_len);

    log_header_m_src[IDX_DMESG].log_buf.pa = pa;
    log_header_m_src[IDX_DMESG].log_buf.va = va;
    log_header_m_src[IDX_DMESG].log_len = log_len;
    log_header_m_src[IDX_DMESG].log_buf_len = log_buf_len;

    srecorder_update_log_header_m(log_header_m_src + IDX_DMESG);
}

/**
    @function: void srecorder_wait_for_log(void)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_wait_for_log(void)
{
    //wait_for_completion_interruptible(&srecorder_log_completion);
}

/**
    @function: void srecorder_wake_for_log(void)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_wake_for_log(void)
{
    //complete(&srecorder_log_completion);
}

/**
    @function: void srecorder_notify_framework(reset_type_e reset_type)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_notify_framework(reset_type_e reset_type)
{
    if (spin_trylock(&srecorder_buffer_lock))
    {
        switch (reset_type)
        {
            case RESET_NORMAL:
                /* log data will be messed up if RESET_NORMAL is notified after RESET_APANIC by panic() */
                if (srecorder_get_dumped_flag() == 0)
                {
                    SRECORDER_INFO("Srecorder framework notification: RESET_NORMAL\n");
                    srecorder_set_reset_type(RESET_NORMAL);
                    srecorder_sync_log_header_h();
                }
                break;

            case RESET_APANIC:
                SRECORDER_INFO("Srecorder framework notification: RESET_APANIC\n");
                srecorder_dump_common_log();
                srecorder_dump_dmesg_info();
                srecorder_set_dumped_flag(1);
                break;
            case RESET_COLLAPSE:
                /* just for eliminating compiling warning */
                break;
        }

        spin_unlock(&srecorder_buffer_lock);
    }
}
