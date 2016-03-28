#include <linux/dsm_pub.h>

#ifndef __LINUX_FT6X06_TS_H__
#define __LINUX_FT6X06_TS_H__
#include <huawei_platform/touchscreen/hw_tp_common.h>
/* -- dirver configure -- */
#define CFG_MAX_TOUCH_POINTS	2
#ifdef TP_LOG_NAME
#undef TP_LOG_NAME
#define TP_LOG_NAME "[FOCALTECH]"
#endif
#define PRESS_MAX	0xFF
#define FT_PRESS		0x7F

#define FT6X06_I2C_NAME 	"ft6x06_i2c_adapter"

#define FT_MAX_ID	0x0F
#define FT_TOUCH_STEP	6
#define FT_TOUCH_X_H_POS		3
#define FT_TOUCH_X_L_POS		4
#define FT_TOUCH_Y_H_POS		5
#define FT_TOUCH_Y_L_POS		6
#define FT_TOUCH_EVENT_POS		3
#define FT_TOUCH_ID_POS			5

#define FT_RETURN_KEY_X_BASE   80
#define FT_RETURN_KEY_Y_BASE   900

#define FT_HOME_KEY_X_BASE   	240
#define FT_HOME_KEY_Y_BASE   	900

#define FT_MENU_KEY_X_BASE   	400
#define FT_MENU_KEY_Y_BASE   	900

#define HUAWEI_KEY_DIFF 		10
#define POINT_READ_BUF	(3 + FT_TOUCH_STEP * CFG_MAX_TOUCH_POINTS)

/*register address*/
#define FT6x06_REG_FW_VER		0xA6
#define FT6x06_REG_POINT_RATE	0x88
#define FT6x06_REG_THGROUP	0x80

#define FT6X06_REG_VENDOR_READ 0xA8
#ifdef   CONFIG_FOCALTECH_FT6X06_UPDATA_FW
#define FT6X06_UPDATE_WAIT_TIMEOUT	2000
#define FT6X06_UPDATE_COMPLITION_TIMEOUT  5000
#endif
#define FEI_VENDOR_ID  0x00
#define JUN_VENDOR_ID  0x05
#define LANSI_VENDOR_ID 0x06
#define LANSI_VENDOR_ID2 0x11
#define SHENGYUE_VENDOR_ID 0x07
#define FT6X06_FLAG_WAKE_LOCK  0x01
#define FT6X06_FLAG_WAKE_UNLOCK  0x02
#define FT6X06_VIRTUAL_KEY_ELEMENT_SIZE	5
#define FT6X06_MAX_VKEY_LEN	100
#define FT6X06_VIRTUAL_NAME_MAX_LENGTH	64

struct ft6x06_virtual_keys_data {
	struct kobj_attribute kobj_attr;
	u32  *data;
	int size;
};
struct ft6x06_core_data
{
	char const *name;
	char const *id;
	struct ft6x06_virtual_keys_data *virtual_keys_pdata;
};
/* The platform data for the Focaltech ft5x0x touchscreen driver */
struct ft6x06_platform_data {
	unsigned int x_max;
	unsigned int y_max;
	unsigned long irqflags;
	unsigned int irq_flags;
    unsigned int reset_flags;
	unsigned int irq;
	unsigned int reset;
	char const *power_pin_vdd;
	char const *power_pin_vbus;

        char const *product_name;
};
struct ts_event {
	u16 au16_x[CFG_MAX_TOUCH_POINTS];	/*x coordinate */
	u16 au16_y[CFG_MAX_TOUCH_POINTS];	/*y coordinate */
	u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];	/*touch event:0 -- down; 1-- contact; 2 -- contact */
	u8 au8_finger_id[CFG_MAX_TOUCH_POINTS];	/*touch ID */
	u16 pressure;
	u8 touch_point;
};

struct ft6x06_ts_data {
	struct device dev;
	unsigned int irq;
	unsigned int x_max;
	unsigned int y_max;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ts_event event;
	struct ft6x06_platform_data *pdata;
	struct ft6x06_core_data *ft6x06_core_pdata;
	unsigned char fw_ver_reg_value;
	/* vendor id of touch panel, this id was readed from flash */
	unsigned char vendor_id_value;

	/* vendor id of touch panel, this id was readed from firmware */
	unsigned char vendor_id_firmware;
	unsigned char is_suspended;
	#ifdef  CONFIG_FOCALTECH_FT6X06_UPDATA_FW

	struct delayed_work ft6x06_fw_config_delay;
	struct work_struct ft6x06_upgrade_with_i_work;
	#endif
	#if !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_FB)
	struct notifier_block fb_notif;
	#endif
	
    u8  suspend_resume_flag;
};
int ft6x06_i2c_Read(struct i2c_client *client, char *writebuf, int writelen,
		    char *readbuf, int readlen);
int ft6x06_i2c_Write(struct i2c_client *client, char *writebuf, int writelen);
void ft6x06_set_app_info_touchpanel(struct ft6x06_ts_data *ft6x06_ts);
#endif

size_t ft6x06_dsm_record_basic_info(struct ft6x06_ts_data *, struct dsm_client *);
void ft6x06_report_dsm_erro(struct ft6x06_ts_data *, struct dsm_client *, int, int);
