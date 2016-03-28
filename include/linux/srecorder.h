

#ifndef SRECORDER_H
#define SRECORDER_H

#define SRECORDER_MAGIC_NUMBER (0x20122102)

#ifndef CONFIG_SRECORDER_VERSION
#define CONFIG_SRECORDER_M_VERSION 0x0000
#define CONFIG_SRECORDER_N_VERSION 0x0000
#define CONFIG_SRECORDER_R_VERSION 0x0000
#define CONFIG_SRECORDER_P_VERSION 0x0002
#define CONFIG_SRECORDER_VERSION (CONFIG_SRECORDER_M_VERSION | CONFIG_SRECORDER_N_VERSION | CONFIG_SRECORDER_R_VERSION | CONFIG_SRECORDER_P_VERSION)
#endif

#define REASON_TIME_COLLAPSE "Crash reason: powercollapse Crash time: Unknown\n"

#define TITLE_REASON_TIME           "===============crash reason time===============\n"
#define TITLE_SYS_INFO              "===================sys info====================\n"
#define TITLE_DMESG                 "=====================dmesg=====================\n"
#define TITLE_STACK                 "===============all cpu call stack==============\n"
#define TITLE_PS_INFO               "==================all ps info==================\n"
#define TITLE_BACK_TRACE            "===============current ps backtrace============\n"
#define TITLE_SLAB_INFO             "===================slabinfo====================\n"


/* reset type */
typedef enum
{
    RESET_NORMAL,
    RESET_APANIC,
    RESET_COLLAPSE
} reset_type_e;

/* log type */
typedef enum
{
    TYPE_MIN,
    TYPE_REASON_TIME = TYPE_MIN,
    TYPE_SYS_INFO, 
    TYPE_STACK,
    TYPE_PS_INFO,
    TYPE_BACK_TRACE,
    TYPE_SLAB_INFO,
    TYPE_DMESG,
    TYPE_MAX = TYPE_DMESG
} log_type_e;

/* log index */
typedef enum
{
    IDX_MIN,
    IDX_PREV = IDX_MIN,
    IDX_CURR,
    IDX_DMESG,
    IDX_EXT,
    IDX_MAX = IDX_EXT,
    IDX_COUNT
} addr_index_t;

typedef struct
{
    unsigned long pa;
    unsigned long va;
} addr_info_t;

typedef struct
{
    unsigned magic_num;
    unsigned reset_flag;
    unsigned long addr;
    unsigned size;
    unsigned crc32;
} log_header_h_t;

typedef struct
{
    unsigned version;
    unsigned magic_num;
    addr_info_t  log_buf;
    unsigned log_len;
    unsigned log_buf_len;
    unsigned crc32; /* the checksum of the header, not including log data */
    unsigned padding; /* alteration of __attribute__((__packed__)) */
}  log_header_m_t;

typedef struct
{
    log_type_e   log_type;
    unsigned log_len;
    unsigned crc32; /* the checksum of the header, not including log data */
} log_header_l_t;

typedef struct
{
    unsigned log_id;
    unsigned long log_buf;
    unsigned log_len;
} log_registration_t;

/**
    @function: void srecorder_retrieve_previous_log()
    @brief: 
    @return: 
    @note: 
**/
void srecorder_retrieve_previous_log(void);

/**
    @function: void recorder_reserve_special_mem)
    @brief: 
    @return: 
    @note: 
**/
void srecorder_reserve_special_mem(void);

/**
    @function: void srecorder_move_previous_log()
    @brief: 
    @return: 
    @note: 
**/
void srecorder_move_previous_log(void);

/**
    @function: void srecorder_move_previous_log()
    @brief: 
    @return: 
    @note: 
**/
void srecorder_get_printk_buf_info(unsigned long* p_log_buf, unsigned* p_log_end, unsigned* p_log_buf_len);

/**
    @function: int srecorder_enable_category_flag(unsigned flag)
    @brief: 
    @param: 
    @return: 
    @note:
**/
int srecorder_register_external_log(unsigned id, void(*callback)(void*));

/**
    @function: int srecorder_enable_category_flag(unsigned flag)
    @brief: the log won't be committed if its id hasn't been registered
    @param: 
    @return: 
    @note:
**/
int srecorder_commit_external_log(log_registration_t* external, unsigned timeout);

/**
    @function: int srecorder_enable_category_flag(unsigned flag)
    @brief: 
    @param: 
    @return: 
    @note:
**/
int srecorder_unregister_external_log(unsigned id);

#endif /* SRECORDER_H */
