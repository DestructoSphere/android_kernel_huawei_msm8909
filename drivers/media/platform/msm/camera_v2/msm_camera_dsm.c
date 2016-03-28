#include "msm_camera_dsm.h"

/* buffer size for dsm camera client */
#define CAMERA_RADAR_BUF_MAX	4096

/*#define CONFIG_MSMB_CAMERA_DEBUG*/
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

//extern struct msm_sensor_ctrl_t;

struct dsm_client *camera_dsm_client = NULL;
char camera_dsm_log_buff[MSM_CAMERA_DSM_BUFFER_SIZE] = {0};
int camera_is_closing = 0;
//struct msm_sensor_ctrl_t *ps_ctrl = NULL;

//int camera_dsm_dump (int type, void *buff, int size);

/* add camera dump_func */
struct dsm_client_ops camera_dsm_ops={
	.poll_state = NULL,
//	.dump_func = camera_dsm_dump,
};

/* dsm client for camera */
struct dsm_dev camera_dsm_dev = {
	.name = "dsm_camera",	// dsm client name
	.fops = &camera_dsm_ops,		// options
	.buff_size = CAMERA_RADAR_BUF_MAX,		// buffer size
};

struct dsm_client* camera_dsm_get_client(void)
{
	if (!camera_dsm_client) 
	{
		camera_dsm_client = dsm_register_client(&camera_dsm_dev);

		if ( NULL == camera_dsm_client)
		{
			pr_err("%s : camera register dsm client failed!!!\n", __func__);
		}
	}
	
	return camera_dsm_client;
}

static char copy_buf[MSM_CAMERA_DSM_BUFFER_SIZE];
ssize_t camera_dsm_record_basic_info(struct dsm_client *pcamera_client , int type, int err_num , const char* str)
{

	ssize_t size = 0;
	ssize_t len = 0;

	CDBG("%s: entry!\n", __func__);
	if(!str)
	{
		pr_err("%s str NULL\n",__func__);
		return -1;
	}

	/* Camera basic info */
	memset(copy_buf, 0, MSM_CAMERA_DSM_BUFFER_SIZE);
	len = snprintf(copy_buf, MSM_CAMERA_DSM_BUFFER_SIZE,
					"[Camera info] DSM Camera Error Type:%d, Error No:%d, %s\n",
					type,
					err_num,
					str);
	size = dsm_client_copy(pcamera_client, copy_buf, len);

	return size;
}


int camera_report_dsm_err( int type, int err_num, const char* str)
{
#ifdef CONFIG_HUAWEI_DSM
	CDBG("%s: entry! type:%d\n", __func__, type);

	/* try to get permission to use the buffer */
	if(dsm_client_ocuppy(camera_dsm_client))
	{
		/* buffer is busy */
		pr_err("%s: buffer is busy!\n", __func__);
		return -1;
	}

	/* report camera basic infomation */
	camera_dsm_record_basic_info(camera_dsm_client, type, err_num, str);

	dsm_client_notify(camera_dsm_client, type);

#endif
	return 0;

}

