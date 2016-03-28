

//#define HW_CMR_LOGSWC 0   //file log switch set 0 off,default is 1 on
#define HW_CMR_LOG_TAG "sensor_otp_common_if"

#include <linux/hw_camera_common.h>
#include "msm_sensor.h"
#include "sensor_otp_common_if.h"

#define OV8858_FOXCONN_RG_RATIO_TYPICAL 0x24B
#define OV8858_FOXCONN_BG_RATIO_TYPICAL 0x274

#define IMX219_LITEON_RG_RATIO_TYPICAL 0x23A
#define IMX219_LITEON_BG_RATIO_TYPICAL 0x251

#define IMX219_OFILM_RG_RATIO_TYPICAL 0x271
#define IMX219_OFILM_BG_RATIO_TYPICAL 0x2ba

/* has update the golden value(20150530) */
#define S5K4E1_FOXCONN_DC0301A_RG_RATIO_TYPICAL 0x315
#define S5K4E1_FOXCONN_DC0301A_BG_RATIO_TYPICAL 0x2ac

struct otp_function_t otp_function_lists []=
{
     {
		"imx219_liteon_3bab17",
		imx219_otp_func,
		IMX219_LITEON_RG_RATIO_TYPICAL,
		IMX219_LITEON_BG_RATIO_TYPICAL,
		false,
	},
     {
		"imx219_ofilm_ohw8a04",
		imx219_otp_func,
		IMX219_OFILM_RG_RATIO_TYPICAL,
		IMX219_OFILM_BG_RATIO_TYPICAL,
		false,
	},
	{
		"ov8858_foxconn_sc1102",
		ov8858_foxconn_otp_func,
		OV8858_FOXCONN_RG_RATIO_TYPICAL,
		OV8858_FOXCONN_BG_RATIO_TYPICAL,
		true,
	},
	{
		"s5k4e1_foxconn_dc0301a",
		s5k4e1_otp_func,
		S5K4E1_FOXCONN_DC0301A_RG_RATIO_TYPICAL,
		S5K4E1_FOXCONN_DC0301A_BG_RATIO_TYPICAL,
		false,
	},
};

/*************************************************
  Function    : is_exist_otp_function
  Description: Detect the otp we support
  Calls:
  Called By  : msm_sensor_config
  Input       : s_ctrl
  Output     : index
  Return      : true describe the otp we support
                false describe the otp we don't support

*************************************************/
bool is_exist_otp_function( struct msm_sensor_ctrl_t *s_ctrl, int32_t *index)
{
	int32_t i = 0;
	
	for (i=0; i<(sizeof(otp_function_lists)/sizeof(otp_function_lists[0])); ++i)
	{
		if (0 == strncmp(s_ctrl->sensordata->sensor_name, otp_function_lists[i].sensor_name, strlen(s_ctrl->sensordata->sensor_name)))
		{
			*index = i;
			CMR_LOGI("is_exist_otp_function success i = %d\n", i);
			return true;
		}
	}
	return false;
}

