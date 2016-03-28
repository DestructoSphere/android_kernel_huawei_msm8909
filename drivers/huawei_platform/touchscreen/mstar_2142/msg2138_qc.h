#include <linux/dsm_pub.h>

#include <huawei_platform/touchscreen/hw_tp_common.h>

#ifndef _MSG2138_QC_H_
#define _MSG2138_QC_H_

#define __FIRMWARE_UPDATE__
#ifdef  __FIRMWARE_UPDATE__
#define __CHIP_MSG_2133A__
#ifndef __CHIP_MSG_2133A__
#define __CHIP_MSG_2133__
#endif

#endif

#ifndef CTP_PROXIMITY_FUN
#define CTP_PROXIMITY_FUN	0
#endif

#define N_BYTE_PER_TIME (8)
#define UPDATE_TIMES (1024/N_BYTE_PER_TIME)

#define UPDATE_INFO_BLK 0

#define FW_ADDR_MSG21XX   (0xC4>>1)
#define FW_ADDR_MSG21XX_TP   (0x4C>>1)

#define	CTP_ID_MSG21XX		1
#define	CTP_ID_MSG21XXA		2


#define u8 unsigned char
#define U8 unsigned char
#define u32 unsigned int
#define U32 unsigned int
#define s32 signed int
#define u16 ushort
#define U16 ushort
#define REPORT_PACKET_LENGTH  8
#define MSG21XX_INT_GPIO 1
#define MSG21XX_RESET_GPIO 0
#define FLAG_GPIO    122
#define ST_KEY_SEARCH 	217
#define MSG21XX_RETRY_COUNT 3
#define MS_TS_MSG20XX_X_MIN   0
#define MS_TS_MSG20XX_Y_MIN   0

#define MS_TS_MSG21XX_X_MAX   (480)
#define MS_TS_MSG21XX_Y_MAX   (854)
#define MS_I2C_VTG_MIN_UV	1800000
#define MS_I2C_VTG_MAX_UV	1800000
#define MS_VDD_VTG_MIN_UV   2850000
#define MS_VDD_VTG_MAX_UV   2850000
#define CFG_MAX_TOUCH_POINTS 	2

#define MSG21XX_ESD_TIMEOUT 3000

#define CTP_AUTHORITY 0777

/*delete tp debug mask to use common debug mask*/
#ifdef TP_LOG_NAME
#undef TP_LOG_NAME
#define TP_LOG_NAME "[MSTAR]"
#endif

#define MSTARALSPS_DEVICE_NAME "MStar-alsps-tpd-dev"
#define MSTARALSPS_INPUT_NAME  "proximity"
#define MSTARALSPS_IOCTL_MAGIC 0XDF
#define MSTARALSPS_IOCTL_PROX_ON _IOW(MSTARALSPS_IOCTL_MAGIC, 0,char *)
#define MSTARALSPS_IOCTL_PROX_OFF _IOW(MSTARALSPS_IOCTL_MAGIC, 1,char *)

#define PROXIMITY_TIMER_VAL   (300)
#define GTP_KEY_TAB	 {KEY_MENU, KEY_HOME, KEY_BACK, KEY_SEARCH}//KEY_HOMEPAGE

#define MAX_TOUCH_FINGER 2

#define FW_ADDR_MSG20XX_TP   	 (0x4C>>1) //device address of msg20xx    7Bit I2C Addr == 0x26;
#define FW_ADDR_MSG20XX      	 (0xC4>>1)  
#define FW_UPDATE_ADDR_MSG20XX   (0x92>>1)
#define DWIIC_MODE_ISP    0
#define DWIIC_MODE_DBBUS  1

#define I2C_FREQUENCY_50000 50000

#define MSG21XX_UPDATE_WAIT_TIMEOUT 2000

typedef struct
{
    u16 X;
    u16 Y;
} TouchPoint_t;

typedef struct
{
    u8 nTouchKeyMode;
    u8 nTouchKeyCode;
    u8 nFingerNum;
    TouchPoint_t Point[MAX_TOUCH_FINGER];
} TouchScreenInfo_t;
typedef enum {
    SWID_OFILM=0,  
    SWID_EELY,
    SWID_TRULY, 
    SWID_MUTTO,
    SWID_NULL,
} SWID_ENUM;


/*move this define from msg2138_qc.c*/
struct msg21xx_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	struct hrtimer timer;
	struct work_struct  work;

    struct timer_list esd_timer;
    struct work_struct timer_work;

	int (*power)(int on);
	int (*get_int_status)(void);
	void (*reset_ic)(void);
	unsigned irq_gpio;
    unsigned reset_gpio;
	u32 irq_flags;
    u32 reset_flags;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	struct delayed_work firmware_update_work; 
    
#if !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_FB)
	struct notifier_block fb_notif;
#endif
    u8  is_suspend;
	/* key code of back key */
	u32 key_value_back;
	/* key code of home key */
	u32 key_value_home;
	/* key code of menu key */
	u32 key_value_menu;

        char const *product_name;
        u16 vendor_id;
};


/*delete mstar_debug logs to use common debug logs*/

void msg2138_enable_irq(void);
void msg2138_disable_irq(void);

void msg21xx_start_esd_timer(struct msg21xx_ts_data * msg_data);
void msg21xx_stop_esd_timer(struct msg21xx_ts_data * msg_data);


size_t msg21xx_dsm_record_basic_info(struct msg21xx_ts_data *, struct dsm_client *);
void msg21xx_report_dsm_erro(struct msg21xx_ts_data *, struct dsm_client *, int, int);


#endif
