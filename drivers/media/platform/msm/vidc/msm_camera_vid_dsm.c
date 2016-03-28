#include "msm_camera_vid_dsm.h"

/* buffer size for dsm camera_vid client */
#define CAMERA_VID_RADAR_BUF_MAX	4096

/*#define CONFIG_MSMB_CAMERA_DEBUG*/
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

char camera_vid_dsm_log_buff[MSM_CAMERA_VID_DSM_BUFFER_SIZE] = {0};

/* add camera_vid dump_func */
struct dsm_client_ops camera_vid_dsm_ops={
	.poll_state = NULL,
//	.dump_func = camera_vid_dsm_dump,
};

static char copy_buf[MSM_CAMERA_VID_DSM_BUFFER_SIZE];

ssize_t camera_vid_dsm_record_basic_info(struct dsm_client *pcamera_vid_client , int type, int err_num , const char* str)
{

	ssize_t size = 0;
	ssize_t len = 0;

	CDBG("%s: entry!\n", __func__);

	if (!str)
	{
		pr_err("%s str error\n",__func__);
		return -1;
	}
	/* Camera basic info */
	memset(copy_buf, 0, MSM_CAMERA_VID_DSM_BUFFER_SIZE);
	len = snprintf(copy_buf, MSM_CAMERA_VID_DSM_BUFFER_SIZE,
					"[Camera info] DSM Camera Error Type:%d, Error No:%d, %s\n",
					type,
					err_num,
					str);
	size = dsm_client_copy(pcamera_vid_client, copy_buf, len);

	return size;
}


int camera_vid_report_dsm_err( int type, int err_num, const char* str )
{
#ifdef CONFIG_HUAWEI_DSM
	CDBG("%s: entry! type:%d\n", __func__, type);

	/* try to get permission to use the buffer */
	if(dsm_client_ocuppy(camera_dsm_client))
	{
		/* buffer is busy */
		CDBG("%s: buffer is busy!\n", __func__);
		return -1;
	}

	/* report camera_vid basic infomation */
	camera_vid_dsm_record_basic_info(camera_dsm_client, type, err_num, str);

	dsm_client_notify(camera_dsm_client, type);

#endif
	return 0;

}
