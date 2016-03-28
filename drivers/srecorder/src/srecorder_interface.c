/**
    @copyright: Huawei Technologies Co., Ltd. 2012-2012. All rights reserved.
    
    @file: srecorder_log.c
    
    @brief: 
    
    @version: 2.1.1 
    
    @author: Qi Dechun 00216641,    Yan Tongguang 00297150
    
    @date: 2015-03-13
    
    @history:
**/

#include <linux/slab.h>
#include <linux/semaphore.h>

#include "srecorder_interface.h"

typedef struct log_registration_entry
{
    void(*callback)(void*);
    log_registration_t  info;
    char* reason;
    struct log_registration_entry* next;
} log_registration_entry_t;

static log_registration_entry_t* p_log_registration_list = NULL;

/* spinlock between external threads to control that only one thread can modify the linked list */
static DEFINE_SPINLOCK(srecorder_registration_lock);

/* spinlock between internal and external threads to control that only one thread can modify the linked list */
static DEFINE_SPINLOCK(srecorder_list_lock);

static int external_flag = 0;

static unsigned found_id = 0;

/**
    @function: void srecorder_enable_external_category(void)
    @brief: 
    @param: 
    @return: 
    @note:
**/
void srecorder_enable_external_category(void)
{
    external_flag = 1;
}

/**
    @function: void srecorder_disable_external_category(void)
    @brief: 
    @param: 
    @return: 
    @note:
**/
void srecorder_disable_external_category(void)
{
    external_flag = 0;
}

/**
    @function: int srecorder_register_external_log(unsigned id, void(*callback)(void*))
    @brief: 
    @param: 
    @return: 
    @note:
**/
int srecorder_register_external_log(unsigned id, void(*callback)(void*))
{
    log_registration_entry_t* tmp_entry = NULL;
    log_registration_entry_t* log_entry = NULL;

    if (external_flag == 0)
    {
        SRECORDER_PRINTK("The dump flag of external log isn't enabled\n");
        return -1;
    }

    if (id == 0 || callback == NULL)
    {
        return -1;
    }

    /* allocate one entry first to safely use spinlock */
    log_entry = kzalloc(sizeof(log_registration_entry_t), GFP_KERNEL);
    if (log_entry == NULL)
    {
        return -1;
    }

    if (spin_trylock(&srecorder_registration_lock) == 0)
    {
        goto out;
    }

    if (spin_trylock(&srecorder_list_lock) == 0)
    {
        goto reg_out;
    }

    tmp_entry = p_log_registration_list;
    while (tmp_entry != NULL)
    {
        if (tmp_entry->info.log_id == id)
        {
            goto list_out;
        }

        tmp_entry = tmp_entry->next;
    }

    log_entry->info.log_id = id;

    log_entry->callback = callback;

    log_entry->next = p_log_registration_list;

    p_log_registration_list = log_entry;

    spin_unlock(&srecorder_list_lock);
    spin_unlock(&srecorder_registration_lock);
    return 0;

list_out:
    spin_unlock(&srecorder_list_lock);
reg_out:
    spin_unlock(&srecorder_registration_lock);
out:
    kfree(log_entry);
    return -1;
}
EXPORT_SYMBOL(srecorder_register_external_log);

/**
    @function: int srecorder_commit_external_log(log_registration_t* external, unsigned timeout)
    @brief: the log won't be committed if its id hasn't been registered
    @param: 
    @return: 
    @note:
**/
int srecorder_commit_external_log(log_registration_t* external, unsigned timeout)
{
    log_registration_entry_t* tmp_entry = NULL;

    if (external == NULL || external->log_buf == 0 || external->log_len == 0)
    {
        return -1;
    }

    if (p_log_registration_list == NULL)
    {
        return -1;
    }

    if (spin_trylock(&srecorder_registration_lock) == 0)
    {
        return -1;
    }

    if (spin_trylock(&srecorder_list_lock) == 0)
    {
        goto reg_out;
    }

    tmp_entry = p_log_registration_list;
    while (tmp_entry != NULL)
    {
        if (tmp_entry->info.log_id == external->log_id)
        {
            tmp_entry->info.log_buf = external->log_buf;
            tmp_entry->info.log_len = external->log_len;

            spin_unlock(&srecorder_list_lock);
            spin_unlock(&srecorder_registration_lock);

            srecorder_wake_for_log();

            return 0;
        }

        tmp_entry = tmp_entry->next;
    }

    /* return if nothing found in loop */
    spin_unlock(&srecorder_list_lock);
reg_out:
    spin_unlock(&srecorder_registration_lock);
    return -1;
}
EXPORT_SYMBOL(srecorder_commit_external_log);

/**
    @function: int srecorder_unregister_external_log(unsigned id)
    @brief: 
    @param: 
    @return: 
    @note:
**/
int srecorder_unregister_external_log(unsigned id)
{
    log_registration_entry_t* prev_entry = NULL;
    log_registration_entry_t* curr_entry = NULL;

    if (spin_trylock(&srecorder_registration_lock) == 0)
    {
        return -1;
    }

    if (spin_trylock(&srecorder_list_lock) == 0)
    {
        goto reg_out;
    }

    curr_entry = p_log_registration_list;
    while (curr_entry != NULL)
    {
        if (curr_entry->info.log_id == id)
        {
            if (curr_entry == p_log_registration_list)
            {
                p_log_registration_list =  curr_entry->next;
            }
            else
            {
                prev_entry->next = curr_entry->next;
            }

            spin_unlock(&srecorder_list_lock);
            spin_unlock(&srecorder_registration_lock);
            kfree(curr_entry);
            return 0;
        }

        prev_entry = curr_entry;
        curr_entry = curr_entry->next;
    }

    /* return if nothing found in loop */
    spin_unlock(&srecorder_list_lock);
reg_out:
    spin_unlock(&srecorder_registration_lock);
    return -1;
}
EXPORT_SYMBOL(srecorder_unregister_external_log);

/**
    @function: int srecorder_init_registration_interface(void)
    @brief: 
    @param: 
    @return: 
    @note:
**/
int srecorder_init_registration_interface(void)
{
    return 0;
}

/**
    @function: int srecorder_exit_registration_interface(void)
    @brief: 
    @param: 
    @return: 
    @note:
**/
int srecorder_exit_registration_interface(void)
{
    log_registration_entry_t* curr_entry = NULL;

    spin_lock(&srecorder_registration_lock);
    spin_lock(&srecorder_list_lock);

    curr_entry = p_log_registration_list;
    while (curr_entry != NULL)
    {
        p_log_registration_list = curr_entry->next;
        kfree(curr_entry);
        curr_entry = p_log_registration_list;
    }

    spin_unlock(&srecorder_list_lock);
    spin_unlock(&srecorder_registration_lock);
    return 0;
}

/**
    @function: int srecorder_get_external_log_info(unsigned long* log_buf, unsigned* log_len)
    @brief: srecorder_buffer_lock is already attained before this function is called
    @param: 
    @return: 
    @note:
**/
int srecorder_get_external_log_info(unsigned long* log_buf, unsigned* log_len)
{
    log_registration_entry_t* curr_entry = NULL;

    if (spin_trylock(&srecorder_registration_lock) == 0)
    {
        return -1;
    }

    if (spin_trylock(&srecorder_list_lock) == 0)
    {
        goto reg_out;
    }

    curr_entry = p_log_registration_list;
    while (curr_entry != NULL)
    {
        if (curr_entry->info.log_buf != 0 && curr_entry->info.log_len != 0)
        {
            *log_buf = curr_entry->info.log_buf;
            *log_len = curr_entry->info.log_len;

            /* save the last found id */
            found_id = curr_entry->info.log_id;

            spin_unlock(&srecorder_list_lock);
            spin_unlock(&srecorder_registration_lock);
            return 0;
        }

        curr_entry = curr_entry->next;
    }

    /* return if nothing found in loop */
    spin_unlock(&srecorder_list_lock);
reg_out:
    spin_unlock(&srecorder_registration_lock);
    return -1;
}

/**
    @function: void srecorder_reset_external_log_info(void)
    @brief: 
    @param: 
    @return: 
    @note:
**/
void srecorder_reset_external_log_info(void)
{
    log_registration_entry_t* prev_entry = NULL;
    log_registration_entry_t* curr_entry = NULL;

    spin_lock(&srecorder_registration_lock);
    spin_lock(&srecorder_list_lock);

    curr_entry = p_log_registration_list;
    while (curr_entry != NULL)
    {
        if (curr_entry->info.log_id == found_id)
        {
            if (curr_entry == p_log_registration_list)
            {
                p_log_registration_list =  curr_entry->next;
            }
            else
            {
                prev_entry->next = curr_entry->next;
            }

            /* reset the last found id */
            found_id = 0;

            spin_unlock(&srecorder_list_lock);
            spin_unlock(&srecorder_registration_lock);
            kfree(curr_entry);
            return;
        }

        prev_entry = curr_entry;
        curr_entry = curr_entry->next;
    }

    spin_unlock(&srecorder_list_lock);
    spin_unlock(&srecorder_registration_lock);
    return;
}
