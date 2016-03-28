#ifndef __GETEVENT_COMMON_REPORT_H__
#define __GETEVENT_COMMON_REPORT_H__
#include <linux/syscalls.h>
#include <linux/workqueue.h>

#define GETEVENT_ERR  1
#define GETEVENT_DBG  2

extern int getevent_common_log_mask;

#ifndef getevent_log_err
#define getevent_log_err(x...)                \
do{                                     \
    if( getevent_common_log_mask >= GETEVENT_ERR )   \
    {                                   \
        printk(KERN_ERR "[GETEVENT_ERR] " x); \
    }                                   \
                                        \
}while(0)
#endif

#ifndef getevent_log_dbg
#define getevent_log_dbg(x...)                \
do{                                     \
    if( getevent_common_log_mask >= GETEVENT_DBG )   \
    {                                   \
        printk(KERN_ERR "[GETEVENT_DBG] " x); \
    }                                   \
                                        \
}while(0)
#endif

#define HW_TP_GETEVENT_PARAM "/data/getevent.log"
#define HW_TP_GETEVENT_INI "/data/getevent-virtual-keys.ini"
#define HW_TP_GETEVENT_MAX_NUM 10240//point num
#define HW_TP_GETEVENT_LINE_MAX 100 //char per line
#define TP_GETEVENT_TYPE_LEN_MAX 10
#define TP_GETEVENT_CODE_LEN_MAX 20
#define TP_GETEVENT_VALUE_LEN_MAX 10
#define TP_GETEVENT_TIME_LEN_MAX 30
#define TP_GETEVENT_EVENT_LEN_MAX 30
#define TP_GETEVENT_RATE_LEN_MAX 30
#define TP_GETEVENT_TABLE_LEN_MAX 30
#define GETEVENT_VIRTUAL_KEY_SIZE 15
#define GETEVENT_VN_MAX_LENGTH 100
#define KEY_NAME_LEN 3
#define TYPE_B_PROTOCOL
#define CFG_MAX_TOUCH_POINTS	10
#define GETEVENT_SIZEOF_SSCANF 4
#define GETEVENT_REPORT_WAIT_NEXTACTION	2000
#define GETEVENT_COMMON_REPORT_NAME "getevent_common_report"

struct getevent_info{
    u32 time;//Time of getevent
    char event[TP_GETEVENT_EVENT_LEN_MAX];//event of getevent
    char type[TP_GETEVENT_TYPE_LEN_MAX];//EV_SYN,EV_KEY,EV_ABS
    char code[TP_GETEVENT_CODE_LEN_MAX];//ABS_MT_TRACKING_ID,ABS_MT_POSITION_X,SYN_REPORT...
    char value[TP_GETEVENT_VALUE_LEN_MAX];//value
    char rate[TP_GETEVENT_RATE_LEN_MAX];//rate
};
struct getevent_virtual_keys_data {
	struct kobj_attribute kobj_attr;
	struct delayed_work delay_work;
	char *file_data;
	u32 file_size;
	u32 key_value[GETEVENT_VIRTUAL_KEY_SIZE];
};
struct getevent_dev{
   struct input_dev *input_dev;
   struct delayed_work getevent_delay_report;
   struct completion getevent_comp;
   struct getevent_info *getevent_para;
   unsigned int x_max;
   unsigned int y_max;
   int rate;
   unsigned char agreement[20] ;
   u32 para_num;
   char *file_data;
   u32 file_size;
   u32 count;
};

#endif
