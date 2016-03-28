#ifndef __MSM_CAMERA_VID_DSM_H__
#define __MSM_CAMERA_VID_DSM_H__

#ifdef CONFIG_HUAWEI_DSM
#include <linux/dsm_pub.h>
#define MSM_CAMERA_VID_DSM_BUFFER_SIZE 1024

extern struct dsm_client *camera_dsm_client;
extern char camera_vid_dsm_log_buff[MSM_CAMERA_VID_DSM_BUFFER_SIZE];

extern int camera_vid_report_dsm_err( int type, int err_num , const char* str);
extern void camera_vid_report_dsm_err_vidc(int type, int err_num , const char* str);
#endif
#endif
