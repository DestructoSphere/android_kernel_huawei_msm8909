#ifndef __MSM_CAMERA_DSM_H__
#define __MSM_CAMERA_DSM_H__

#ifdef CONFIG_HUAWEI_DSM
#include <linux/dsm_pub.h>
#define MSM_CAMERA_DSM_BUFFER_SIZE 1024
extern int camera_is_closing;
extern int camera_is_in_probe;
extern char camera_dsm_log_buff[MSM_CAMERA_DSM_BUFFER_SIZE];
extern struct dsm_client *camera_dsm_client;

extern struct dsm_client* camera_dsm_get_client(void);
extern int camera_report_dsm_err( int type, int err_num , const char* str);
#endif

#endif
