#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/time.h>
#include <linux/spinlock.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/store_log.h>

extern ssize_t write_log_to_exception(const char* category, char level, const char* msg);
#define STORAGE_ID "storage"
#define LEVEL_A 'A'
#define LEVEL_B 'B'
#define LEVEL_C 'C'
#define LEVEL_D 'D'

unsigned int g_last_code = 0; /*last msg code*/
unsigned int g_code_number = 0; /*count of continuous same msg code*/
char g_last_msg[MSG_MAX_SIZE]; /*last msg*/
DEFINE_SPINLOCK(log_lock);

static void s2log(int code, const char*msg) {
    char level;
    if (!(MASK_DETAIL(code) & ~(STORAGE_ERROR_BASE|EXT4_MOUNT_ERROR_BASE|EXT4_RUNNING_ERROR_BASE)))
        level = (char)LEVEL_A;
    else if ((code >> GROUP_SHIFT) & (DEVICE_ACTION_BASE >> GROUP_SHIFT))
        level = (char)LEVEL_A;
    else
        level = (char)LEVEL_C;
    write_log_to_exception(STORAGE_ID, level, msg);
}

static long get_time(void) {
    struct timespec ctime;
    ctime = CURRENT_TIME;
    return ctime.tv_sec;
}

static void storage_log_internal(int code, const char* msg) {
    char message[MSG_MAX_SIZE];
    snprintf(message, MSG_MAX_SIZE-1, "%#lx %#x %s\n", get_time(), code, msg);
    s2log(code, message);
}

static void storage_log_abb(int code, int times) {
    char message[MSG_MAX_SIZE];
    snprintf(message, MSG_MAX_SIZE-1, "%#lx %#x --- %d\n", get_time(), code, times);
    s2log(code, message);
}

enum LOG_CONTENT {
    LOG_CONTENT_ORIG = 0,
    LOG_CONTENT_TIMES,
    LOG_CONTENT_LAST,
};

/**
 * storage_log() - log storage error message
 * @code: error number
 * @msg: error message
 */
void storage_log(int code, const char* msg) {
    int log_content = -1;
    int tmp_last_code=0, tmp_code_number=0;
    char tmp_last_msg[MSG_MAX_SIZE];

    spin_lock(&log_lock);
    if (code == g_last_code) {
        if (++g_code_number <= LOG_MAX_NUMBER)
            log_content = LOG_CONTENT_ORIG;
        else
            strncpy(g_last_msg, msg, sizeof(g_last_msg)-1);
    } else {
        if (g_code_number > LOG_MAX_NUMBER) {
            if (g_code_number > (LOG_MAX_NUMBER+1))
                log_content = LOG_CONTENT_TIMES;
            else
                log_content = LOG_CONTENT_LAST;
            tmp_last_code = g_last_code;
            tmp_code_number = g_code_number;
            strncpy(tmp_last_msg, g_last_msg, MSG_MAX_SIZE-1);
        } else
            log_content = LOG_CONTENT_ORIG;
        g_last_code = code;
        g_code_number = 1;
    }
    spin_unlock(&log_lock);

    switch (log_content) {
        case LOG_CONTENT_TIMES:
            storage_log_abb(tmp_last_code, tmp_code_number - (LOG_MAX_NUMBER+1));
        case LOG_CONTENT_LAST:
            storage_log_internal(tmp_last_code, tmp_last_msg);
        case LOG_CONTENT_ORIG:
            storage_log_internal(code, msg);
            break;
        default:
            break;
    }
}

#define LOG_PARTITION_DEV "mmcblk0p17"
#define LOG_PARTITION_START 0x94012
#define LOG_PARTITION_SIZE 0x20000
#define LOG_PARTITION_END (LOG_PARTITION_START + LOG_PARTITION_SIZE)

/**
 * is_log_partition_by_devname() - check device on log partiton or not
 * @devname: device name
 * return: true - is log partition, false - not log partiton
 */
bool is_log_partition_by_devname(const char* devname) {
    if (devname ==  NULL)
        return false;
    return strncmp(LOG_PARTITION_DEV, devname,
        sizeof(LOG_PARTITION_DEV)) == 0 ? true : false;
}

/**
 * is_log_partition_by_devname() - check device on log partiton or not
 * @devname: request start sector
 * return: true - is log partition, false - not log partiton
 */
bool is_log_partition_by_addr(sector_t sector) {
    return ((sector >= LOG_PARTITION_START)
        && (sector < LOG_PARTITION_END));
}
