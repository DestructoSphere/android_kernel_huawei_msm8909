#ifdef CONFIG_HUAWEI_SDCARD_DSM
#ifndef LINUX_MMC_DSM_SDCARD_H
#define LINUX_MMC_DSM_SDCARD_H

#include <linux/dsm_pub.h>

#define DSM_REPORT_UEVENT_TRUE 		  1
#define DSM_REPORT_UEVENT_FALSE		  0	
#define SDCARD_MSG_MAX_SIZE 200

enum DSM_SDCARD_STATUS
{
	DSM_SDCARD_CMD8				= 0,
	DSM_SDCARD_CMD55			= 1,
	DSM_SDCARD_ACMD41			= 2,
	DSM_SDCARD_CMD2_R0 			= 3,
	DSM_SDCARD_CMD2_R1 			= 4,
	DSM_SDCARD_CMD2_R2 			= 5,
	DSM_SDCARD_CMD2_R3 			= 6,
	DSM_SDCARD_CMD3    			= 7,
	DSM_SDCARD_CMD9_R0			= 8,
	DSM_SDCARD_CMD9_R1			= 9,
	DSM_SDCARD_CMD9_R2			= 10,
	DSM_SDCARD_CMD9_R3			= 11,
	DSM_SDCARD_CMD7				= 12,
	DSM_SDCARD_REPORT_UEVENT 	= 13,
	DSM_SDCARD_CMD_MAX,
};

enum DSM_SDCARD_ERR
{
	DSM_SDCARD_CMD2_RESP_ERR 		= 20601,
	DSM_SDCARD_CMD3_RESP_ERR 		= 20602,
	DSM_SDCARD_CMD7_RESP_ERR 		= 20603,
	DSM_SDCARD_CMD8_RESP_ERR 		= 20604,
	DSM_SDCARD_CMD9_RESP_ERR 		= 20605,
	DSM_SDCARD_CMD55_RESP_ERR		= 20606,
	DSM_SDCARD_ACMD41_RESP_ERR		= 20607,
	DSM_SDCARD_NO_UEVENT_REPORT		= 20608,
	DMS_SDCARD_HARDWARE_TIMEOUT_ERR	= 20628,
	DMS_SDCARD_MMC_BLK_ABORT		= 20632,
};

struct dsm_sdcard_cmd_log
{
        char *log;
        u32  value;
};

extern struct dsm_client *sdcard_dclient;

extern char g_dsm_log_sum[1024];
extern struct dsm_sdcard_cmd_log dsm_sdcard_cmd_logs[];

extern char *dsm_sdcard_get_log(int cmd,int err);

#define DSM_SDCARD_LOG(error_num, fmt, a...) \
	do { \
		if(!dsm_client_ocuppy(sdcard_dclient)) { \
			dsm_client_record(sdcard_dclient, fmt, ## a); \
			dsm_client_notify(sdcard_dclient, error_num); } \
	}while(0)

#endif /* LINUX_MMC_DSM_SDCARD_H */
#endif /* CONFIG_HUAWEI_SDCARD_DSM */
