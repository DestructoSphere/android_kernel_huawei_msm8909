#ifndef __KERNEL_STORELOG__
#define __KERNEL_STORELOG__

#define GROUP_SHIFT 24
#define SUBGROUP_SHIFT 16
#define MASK_DETAIL(x) ((x) & ((~0)<<SUBGROUP_SHIFT))

/*
 * NOTICE: when insert other error-code,
 * SHOULD follow the existing rule.
 * eg: codeA & codeB MUST be equal to ZERO
 */
#define STORAGE_ERROR_BASE   (1 << GROUP_SHIFT)

#define EXT4_MOUNT_ERROR_BASE   (1 << SUBGROUP_SHIFT)
#define EXT4_MOUNT_READ_ERR     0x00000001
#define EXT4_MOUNT_WRITE_ERR    0x00000002
#define EXT4_MOUNT_CHECK_ERR    0x00000004

#define EXT4_RUNNING_ERROR_BASE     (2 << SUBGROUP_SHIFT)
#define EXT4_ERR                    0x00000001
#define EXT4_ERR_INODE              0x00000002
#define EXT4_ERR_FILE               0x00000004
#define EXT4_ERR_CAPS               0x0000008

#define MMC_ERROR_BASE              (4 << SUBGROUP_SHIFT)
#define MMC_ERROR_CMD               0x00000001
#define MMC_ERROR_DATA              0x00000002

#define DEVICE_ACTION_BASE  (2 << GROUP_SHIFT)
#define DEVICE_STARTUP      0x00000001
#define DEVICE_SHUTDOWN     0x00000002
#define DEVICE_RESTART      0x00000004

#define LOG_MAX_NUMBER 3
#define MSG_MAX_SIZE 200

#define CHARGE_ERROR_BASE	(8 << GROUP_SHIFT)
#define TMEPERATURE_ERR		(1 << SUBGROUP_SHIFT)
#define TMEPERATURE_OVERFLOW	(1)
#define TMEPERATURE_LIMIT		(2)
#define BATTERY_ERR		(2 << SUBGROUP_SHIFT)
#define RECHARGE_ERR	(3 << SUBGROUP_SHIFT)
#define CHARGER_ERR		(4 << SUBGROUP_SHIFT)
#define BOOST_ERR		(5 << SUBGROUP_SHIFT)

/**
 * MSG_WRAPPER() - wrapper storage error message, then log it
 * @no: error number
 * @fmt: message format
 */
#define MSG_WRAPPER(no, fmt, a...) \
	do { \
		char msg[MSG_MAX_SIZE]; \
		snprintf(msg, MSG_MAX_SIZE-1, fmt, ## a); \
		storage_log(no, msg); \
	}while(0)

/**
 * LOG_STORE_AND_UPLOAD() - wrapper storage error message, then log and tag it for uploading
 * @no: error number
 * @fmt: message format
 */
#define LOG_STORE_AND_UPLOAD(no, fmt, a...) \
	do { \
		char msg[MSG_MAX_SIZE]; \
		snprintf(msg, MSG_MAX_SIZE-1, "storage_log_upload: "fmt, ## a); \
		storage_log(no, msg); \
	}while(0)

/**
 * storage_log() - log storage error message
 * @code: error number
 * @msg: error message
 */
void storage_log(int code, const char* msg);

/**
 * is_log_partition_by_devname() - check device on log partiton or not
 * @devname: device name
 * return: true - is log partition, false - not log partiton
 */
bool is_log_partition_by_devname(const char* devname);

/**
 * is_log_partition_by_devname() - check device on log partiton or not
 * @devname: request start sector
 * return: true - is log partition, false - not log partiton
 */
bool is_log_partition_by_addr(sector_t sector);
#endif
