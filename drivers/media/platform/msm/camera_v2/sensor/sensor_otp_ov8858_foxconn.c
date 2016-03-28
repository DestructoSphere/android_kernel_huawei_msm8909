/************************************************************
  Copyright (C), 1988-2014, Huawei Tech. Co., Ltd.
FileName: sensor_otp_ov8858_foxconn.c
Author:  yWX221546
Version :Initial Draft
Date: 2014/05/14
Description:    this file contion several functions to detect otp_ov5648 properties
Version:         Initial Draft
History:
History        :
1.Date        : 2014/06/12
Author        : yWX221546
Modification : Created function
 ***********************************************************/

//#define HW_CMR_LOGSWC 0   //file log switch set 0 off,default is 1 on
#define HW_CMR_LOG_TAG "sensor_otp_ov8858_foxconn"

#include <linux/hw_camera_common.h>
#include "msm_sensor.h"
#include "sensor_otp_common_if.h"

/* if support OTP or open OTP function, please define it */

#define GROUP_OTP_EMPTY 0  //group is empty
#define GROUP_OTP_INVALID 1  //group is invalid
#define GROUP_OTP_VALID 2  //group is vaild

//OV8858 Foxconn 0x400 = 1x gain
#define OV8858_OTP_ONE_GAIN 0x400
/*module info & awb & lens shading & vcm flag reg address from otp criterion doc.*/
#define OV8858_OTP_MODULE_INFO_FLAG_ADDR 0x7010
#define OV8858_OTP_AWB_FLAG_ADDR 0x7020
#define OV8858_OTP_LENS_FLAG_ADDR 0x7033
#define OV8858_OTP_VCM_FLAG_ADDR 0x717E
/*lens shading num is 110*/
#define OV8858_MAX_OTP_LENS_NUM 110

//the value used for vcm effect, maybe modified by others 
#define OV8858_OTP_VCM_OFFSET_VALUE            60

//OV8858 has three groups: [1,2,3]
typedef enum ov8858_groups_count{
	GROUP_1 = 1,
	GROUP_2,
	GROUP_3,
	GROUP_MAX
}enum_ov8858_gruops;
/*OV8858 foxconn have four types otp*/
typedef enum ov8858_otp_type_e{
	MODULE_INFO_OTP,
	AWB_OTP,
	LENS_OTP,
	VCM_OTP
}enum_ov8858_otp_type;

typedef struct ov8858_otp_reg_addr {
	uint16_t start_address;
	uint16_t end_address;
}ST_ED_REG_ADDR;

//OTP info struct
typedef struct ov8858_otp_info {
	uint16_t module_integrator_id;
	uint16_t lens_id;
	uint16_t production_year;
	uint16_t production_month;
	uint16_t production_day;
	uint16_t rg_ratio;
	uint16_t bg_ratio;
	uint16_t gb_gr_ratio;
	//uint16_t light_rg;
	//uint16_t light_bg;
	uint16_t VCM_start;
	uint16_t VCM_end;
	uint8_t lenc[OV8858_MAX_OTP_LENS_NUM];
}st_ov8858_otp_info;

//Golden sensor typical ratio
static int RG_Ratio_Typical = 0x253;
static int BG_Ratio_Typical = 0x27F;

#define OV8858_MMI_OTP_VCM_FLAG                    ( 1 << 0 )
#define OV8858_MMI_OTP_AWB_FLAG                    ( 1 << 1 )
#define OV8858_MMI_OTP_MODULE_INFO_FLAG            ( 1 << 2 )
#define OV8858_MMI_OTP_LSC_FLAG                    ( 1 << 3 )

static ST_ED_REG_ADDR ov8858_module_info_otp_read_addr[GROUP_3] = {
	{0x7011,0x7015},
	{0x7016,0x701A},
	{0x701B,0x701F},
};
static ST_ED_REG_ADDR ov8858_awb_otp_read_addr[GROUP_3] = {
	{0x7021,0x7026},
	{0x7027,0x702C},
	{0x702D,0x7032},
};
static ST_ED_REG_ADDR ov8858_lens_otp_read_addr[GROUP_3] = {
	{0x7034,0x70A1},
	{0x70A2,0x71AF},
	{0x7110,0x717D},
};
static ST_ED_REG_ADDR ov8858_vcm_otp_read_addr[GROUP_3] = {
	{0x717F,0x7182},
	{0x7183,0x7186},
	{0x7187,0x718A},
};

//OTP info
static st_ov8858_otp_info g_ov8858_otp = {0};

/****************************************************************************
 * FunctionName: ov8858_otp_write_i2c;
 * Description : write otp info via i2c;
 ***************************************************************************/
int32_t ov8858_otp_write_i2c(struct msm_sensor_ctrl_t *s_ctrl, int32_t addr, uint16_t data)
{
	int32_t rc = 0;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,
			addr,
			data,
			MSM_CAMERA_I2C_BYTE_DATA);

	if ( rc < 0 )
	{
		CMR_LOGE("%s fail, rc = %d! addr = 0x%x, data = 0x%x\n", __func__, rc, addr, data);
	}

	return rc;
}

/****************************************************************************
 * FunctionName: ov8858_otp_read_i2c;
 * Description : read otp info via i2c;
 ***************************************************************************/
int32_t ov8858_otp_read_i2c(struct msm_sensor_ctrl_t *s_ctrl,uint32_t addr, uint16_t *data)
{
	int32_t rc = 0;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,
			addr,
			data,
			MSM_CAMERA_I2C_BYTE_DATA);
	if ( rc < 0 )
	{
		CMR_LOGE("%s fail, rc = %d! addr = 0x%x, data = 0x%x\n", __func__, rc, addr, *data);
	}

	return rc;
}
/********************************************************************************************
* To avoid OTP memory access timing conflict,before doing OTP read/write,register 0x5002[3] 
* must be set to ¡°0¡±. After OTP memory access,set register 0x5002[3] back to ¡°1¡±.
********************************************************************************************/
int32_t ov8858_otp_enable_DPC(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	uint16_t addr = 0x5002;
	uint16_t data = 0;
	
	rc = ov8858_otp_read_i2c(s_ctrl, addr, &data);

	pr_info("%s change 0x%x value from 0x%x to 0x%x\n", __func__,addr, data,  data | 0x08);

	/* set 0x5002[3] to 1 */
	rc = ov8858_otp_write_i2c(s_ctrl, addr, data | 0x08);

	return rc;
}

int32_t ov8858_otp_disable_DPC(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	uint16_t addr = 0x5002;
	uint16_t data = 0;
	
	rc = ov8858_otp_read_i2c(s_ctrl, addr, &data);

	pr_info("%s change 0x%x value from 0x%x to 0x%x\n",__func__, addr, data,  data & 0xF7);

	/* set 0x5002[3] to 0 */
	rc = ov8858_otp_write_i2c(s_ctrl, addr, data & 0xF7);
	
	return rc;
}

/****************************************************************************
// index: index of otp group will be checked: (1, 2, 3)
// check_address: otp flag address
// return:
// 0, group index is empty
// 1, group index has invalid data
// 2, group index has valid data
 ***************************************************************************/
int32_t check_otp_group_index(struct msm_sensor_ctrl_t *s_ctrl, int index, uint16_t check_address)
{
	int32_t rc = 0;
	uint16_t data = 0;
	if(!s_ctrl || (index < GROUP_1 || index >= GROUP_MAX))
	{
		CMR_LOGE("%s: index = %d error! \n",__func__,index);
		return -1;
	}

	if(check_address != OV8858_OTP_MODULE_INFO_FLAG_ADDR && check_address != OV8858_OTP_AWB_FLAG_ADDR
			&& check_address != OV8858_OTP_LENS_FLAG_ADDR && check_address != OV8858_OTP_VCM_FLAG_ADDR)
	{
		CMR_LOGE("%s: check_address = 0x%x  error! \n",__func__,check_address);
		return -1;
	}

	// clear otp data buffer
	ov8858_otp_write_i2c(s_ctrl,check_address, 0x00);

	// enter otp read mode
	ov8858_otp_write_i2c(s_ctrl,0x3d84, 0xc0); // program disable, manual mode

	//partial mode OTP write start address
	ov8858_otp_write_i2c(s_ctrl,0x3d88, (check_address>>8));
	ov8858_otp_write_i2c(s_ctrl,0x3d89, (check_address & 0xff));

	// partial mode OTP write end address
	ov8858_otp_write_i2c(s_ctrl,0x3d8A, (check_address>>8));
	ov8858_otp_write_i2c(s_ctrl,0x3d8B, (check_address & 0xff));

	// open otp read function
	ov8858_otp_write_i2c(s_ctrl,0x3d81, 0x01); // read otp

	//delay 20ms
	msleep(20);

	//read otp data
	rc = ov8858_otp_read_i2c(s_ctrl,check_address, &data);
	if(rc < 0)
	{
		data = 0;
		CMR_LOGE("%s:%d read otp data fail\n",__func__,__LINE__);
	}
	else
	{
        	pr_info("%s:%d data = %d\n",__func__,__LINE__, data);
		//select group
		switch(index)
		{
			case GROUP_1:
				data = (data>>6) & 0x03;
				break;

			case GROUP_2:
				data = (data>>4) & 0x03;
				break;

			case GROUP_3:
				data = (data>>2) & 0x03;
				break;

			default:
				data = 0;
				CMR_LOGE("%s:%d read otp data fail\n",__func__,__LINE__);
				break;
		}
	}

	// close otp read function
	ov8858_otp_write_i2c(s_ctrl,0x3d81, 0x00);

	// clear otp data buffer
	ov8858_otp_write_i2c(s_ctrl,check_address, 0x00);

	if (data == 0x00)
	{
		rc = GROUP_OTP_EMPTY;
	}
	else if (data & 0x02)
	{
		rc = GROUP_OTP_INVALID;
	}
	else
	{
		/*find otp data success*/
		rc = GROUP_OTP_VALID;
	}

	return rc;
}

/****************************************************************************
 * FunctionName: ov8858_read_group_address_index;
 * Description : find group index which real store otp data and convert the group index to address array index.
 ***************************************************************************/
int32_t ov8858_read_group_address_index(struct msm_sensor_ctrl_t *s_ctrl, uint16_t check_addr)
{
	int32_t ret = 0;
	int32_t index = 0;

	//get vaild gruop index
	for(index = 1; index < GROUP_MAX; index++)
	{
		ret = check_otp_group_index(s_ctrl,index,check_addr);
		if(ret == GROUP_OTP_VALID)
		{
			break;
		}
	}

	if(index >= GROUP_MAX)
	{
		CMR_LOGE("%s: check module info gruop index fail\n",__func__);
		return -1;
	}

	return (index-1);
}

/****************************************************************************
 * FunctionName: ov8858_read_group_data;
 * Description : read the specific otp data from group1 or group2 or group3. and fill the otp data to g_ov8858_otp
 ***************************************************************************/
int32_t ov8858_read_group_data(struct msm_sensor_ctrl_t *s_ctrl, enum_ov8858_otp_type otp_type)
{
	int32_t i = 0;
	int32_t rc = 0;
	int32_t index = 0;
	uint16_t msb_data = 0;
	uint16_t lsb_data = 0;
	uint16_t lens_temp = 0;
	uint16_t check_addr = 0;
	uint16_t start_address = 0;
	uint16_t end_address = 0;
	ST_ED_REG_ADDR *pcur_addr_array = NULL;

	switch(otp_type)
	{
		case MODULE_INFO_OTP:
			check_addr = OV8858_OTP_MODULE_INFO_FLAG_ADDR;
			pcur_addr_array = ov8858_module_info_otp_read_addr;
			break;

		case AWB_OTP:
			check_addr = OV8858_OTP_AWB_FLAG_ADDR;
			pcur_addr_array = ov8858_awb_otp_read_addr;
			break;

		case LENS_OTP:
			check_addr = OV8858_OTP_LENS_FLAG_ADDR;
			pcur_addr_array = ov8858_lens_otp_read_addr;
			break;

		case VCM_OTP:
			check_addr = OV8858_OTP_VCM_FLAG_ADDR;
			pcur_addr_array = ov8858_vcm_otp_read_addr;
			break;

		default:
			CMR_LOGE("%s: otp type error \n",__func__);
			return -1;
	}

	index = ov8858_read_group_address_index(s_ctrl,check_addr);
	if(index < 0 || index >= GROUP_3)
	{
		CMR_LOGE("%s: otp_type=%d fail index = %d\n",__func__,otp_type,index);
		return -1;
	}

	CMR_LOGD("%s: otp_type=%d index:%d \n", __func__,otp_type,index);

	start_address = pcur_addr_array[index].start_address;
	end_address = pcur_addr_array[index].end_address;

	//clear otp data buffer
	for(i = start_address; i <= end_address; i++)
	{
		ov8858_otp_write_i2c(s_ctrl,i, 0x00);
	}

	// enter otp read mode
	ov8858_otp_write_i2c(s_ctrl,0x3d84, 0xc0); // program disable, manual mode

	//partial mode OTP write start address
	ov8858_otp_write_i2c(s_ctrl,0x3d88, (start_address>>8));
	ov8858_otp_write_i2c(s_ctrl,0x3d89, (start_address & 0xff));

	// partial mode OTP write end address
	ov8858_otp_write_i2c(s_ctrl,0x3d8A, (end_address>>8));
	ov8858_otp_write_i2c(s_ctrl,0x3d8B, (end_address & 0xff));

	// enable otp read function
	ov8858_otp_write_i2c(s_ctrl,0x3d81, 0x01); // read otp

	//delay 20ms
	msleep(20);

	if(otp_type == MODULE_INFO_OTP)
	{
		ov8858_otp_read_i2c(s_ctrl,start_address, &g_ov8858_otp.production_year);
		ov8858_otp_read_i2c(s_ctrl,start_address+1, &g_ov8858_otp.production_month);
		ov8858_otp_read_i2c(s_ctrl,start_address+2, &g_ov8858_otp.production_day);
		ov8858_otp_read_i2c(s_ctrl,start_address+3, &g_ov8858_otp.lens_id);
		ov8858_otp_read_i2c(s_ctrl,start_address+4, &g_ov8858_otp.module_integrator_id);
	}
	else if(otp_type == AWB_OTP)
	{
		ov8858_otp_read_i2c(s_ctrl,start_address, &msb_data); //rg msb
		ov8858_otp_read_i2c(s_ctrl,start_address + 1, &lsb_data); //rg lsb
		g_ov8858_otp.rg_ratio = ((msb_data & 0xFF) << 8) | (lsb_data & 0xFF);

		ov8858_otp_read_i2c(s_ctrl,start_address + 2, &msb_data); //bg msb
		ov8858_otp_read_i2c(s_ctrl,start_address + 3, &lsb_data); //bg lsb
		g_ov8858_otp.bg_ratio = ((msb_data & 0xFF) << 8) | (lsb_data & 0xFF);

		ov8858_otp_read_i2c(s_ctrl,start_address + 4, &msb_data); //gb_gr msb
		ov8858_otp_read_i2c(s_ctrl,start_address + 5, &lsb_data); //gb_gr lsb
		g_ov8858_otp.gb_gr_ratio= ((msb_data & 0xFF) << 8) | (lsb_data & 0xFF);
	}
	else if(otp_type == LENS_OTP)
	{
		for(i=0;i<OV8858_MAX_OTP_LENS_NUM;i++) {
			ov8858_otp_read_i2c(s_ctrl,start_address+i, &lens_temp);
			g_ov8858_otp.lenc[i] = (uint8_t)(lens_temp & 0xFF);
		}
	}
	else if(otp_type == VCM_OTP)
	{
		s_ctrl->afc_otp_info.starting_dac  = 0;
		s_ctrl->afc_otp_info.infinity_dac  = 0;
		s_ctrl->afc_otp_info.macro_dac     = 0;
		ov8858_otp_read_i2c(s_ctrl,start_address, &msb_data); //vcm start code msb
		ov8858_otp_read_i2c(s_ctrl,start_address + 1, &lsb_data); //vcm start code lsb
		g_ov8858_otp.VCM_start = ((msb_data & 0xFF) << 2) | ((lsb_data>>6) & 0x03);

		ov8858_otp_read_i2c(s_ctrl,start_address + 2, &msb_data); //vcm max current msb
		ov8858_otp_read_i2c(s_ctrl,start_address + 3, &lsb_data); //vcm max curren lsb
		g_ov8858_otp.VCM_end = ((msb_data & 0xFF) << 2) | ((lsb_data>>6) & 0x03);

		if ((0 == g_ov8858_otp.VCM_start) || (0 == g_ov8858_otp.VCM_end) || (g_ov8858_otp.VCM_end <= g_ov8858_otp.VCM_start))
		{
		      CMR_LOGE("g_ov8858_otp.VCM_start = 0x%x\n", g_ov8858_otp.VCM_start);
		      CMR_LOGE("g_ov8858_otp.VCM_end = 0x%x\n", g_ov8858_otp.VCM_end);
			  return -1;
		}
		
		if (g_ov8858_otp.VCM_start <= OV8858_OTP_VCM_OFFSET_VALUE)
		{
			CMR_LOGE("%s, otp_ptr->VCM_start = 0x%x\n", __func__,g_ov8858_otp.VCM_start);
			g_ov8858_otp.VCM_start = 0;
		}
		else
		{
			g_ov8858_otp.VCM_start -= OV8858_OTP_VCM_OFFSET_VALUE;
		}

		g_ov8858_otp.VCM_end += OV8858_OTP_VCM_OFFSET_VALUE;
		
		s_ctrl->afc_otp_info.starting_dac = g_ov8858_otp.VCM_start;
		s_ctrl->afc_otp_info.infinity_dac = g_ov8858_otp.VCM_start;
		s_ctrl->afc_otp_info.macro_dac = g_ov8858_otp.VCM_end;
	}
	else
	{
		CMR_LOGE("%s: otp type error! \n",__func__);
		rc = -1;
	}
	// close otp read function
	ov8858_otp_write_i2c(s_ctrl,0x3d81, 0x00);

	//clear otp data buffer
	for(i = start_address; i <= end_address; i++)
	{
		ov8858_otp_write_i2c(s_ctrl,i, 0x00);
	}

	return rc;
}

/****************************************************************************
 * FunctionName: ov8858_read_otp_data;
 * Description : read all otp data info;
 * return value:
 * 0 means read otp info succeed.
 * -1 means read otp info failed, should not write otp.
 ***************************************************************************/
int32_t ov8858_read_otp_data(struct msm_sensor_ctrl_t *s_ctrl, uint16_t *hw_otp_check)
{
	int index = 0;
	int ret = 0;
	uint16_t  ov8858_mmi_otp_flag = 0x0F;

	if (!s_ctrl)
	{
		CMR_LOGE("%s:%d fail\n",__func__,__LINE__);
		return -1;
	}

	if (!hw_otp_check)
	{
		CMR_LOGE("%s:%d fail\n",__func__,__LINE__);
		return -1;
	}

	*hw_otp_check = 0x0F;
 
	ov8858_otp_disable_DPC(s_ctrl);
	//read awb otp data
	ret = ov8858_read_group_data(s_ctrl,AWB_OTP);
	if(ret < 0)
	{
		CMR_LOGE("%s: read module info otp data fail\n",__func__);
		goto OTP_ERROR;
	}
      ov8858_mmi_otp_flag &= ~OV8858_MMI_OTP_AWB_FLAG;

	//read vcm otp data
	ret = ov8858_read_group_data(s_ctrl,VCM_OTP);
	if(ret < 0)
	{
		CMR_LOGE("%s: read module info otp data fail\n",__func__);
		goto OTP_ERROR;
	}
      ov8858_mmi_otp_flag &= ~OV8858_MMI_OTP_VCM_FLAG;
	  
       //read module info otp data
	ret = ov8858_read_group_data(s_ctrl,MODULE_INFO_OTP);
	if(ret < 0)
	{
		CMR_LOGE("%s: read module info otp data fail\n",__func__);
		goto OTP_ERROR;
	}
       ov8858_mmi_otp_flag &= ~OV8858_MMI_OTP_MODULE_INFO_FLAG;
	   
	//read lens otp data
	ret = ov8858_read_group_data(s_ctrl,LENS_OTP);
	if(ret < 0)
	{
		CMR_LOGE("%s: read module info otp data fail\n",__func__);
		goto OTP_ERROR;
	}
	 ov8858_mmi_otp_flag &= ~OV8858_MMI_OTP_LSC_FLAG;

	ov8858_otp_enable_DPC(s_ctrl);
	CMR_LOGD("%s: year=%d month=%d day=%d product=%d moduleid=%d rg=%d bg=%d gb_gr=%d vcm_start=%d vcm_end=%d \n", __func__,
			g_ov8858_otp.production_year,g_ov8858_otp.production_month,g_ov8858_otp.production_day,g_ov8858_otp.lens_id,g_ov8858_otp.module_integrator_id,
			g_ov8858_otp.rg_ratio,g_ov8858_otp.bg_ratio,g_ov8858_otp.gb_gr_ratio,g_ov8858_otp.VCM_start,g_ov8858_otp.VCM_end);

	//debug info test
	CMR_LOGD("%s: ---------lens shading otp data start----------\n",__func__);
	for(index = 0; index <OV8858_MAX_OTP_LENS_NUM; index++)
	{
		CMR_LOGD("[%d]=0x%02x \n",index,g_ov8858_otp.lenc[index]);
	}
	CMR_LOGD("%s: ---------lens shading otp data start----------\n",__func__);

OTP_ERROR:
	*hw_otp_check = ov8858_mmi_otp_flag;
	return ret;
}

/****************************************************************************
 * FunctionName: ov8858_update_awb_gain;
 * Description : write R_gain,G_gain,B_gain to otp;
 * 0x400 =1x Gain
 * 0 means write WB info succeed.
 * -1 means write WB info failed.
 ***************************************************************************/
int32_t ov8858_update_awb_otp(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint16_t R_gain=0, G_gain=0, B_gain=0, G_gain_R=0, G_gain_B=0;
	if(g_ov8858_otp.rg_ratio == 0 || g_ov8858_otp.bg_ratio == 0)
	{
		CMR_LOGE("%s: rg_ratio=%d bg_ratio=%d fail\n",__func__,g_ov8858_otp.rg_ratio,g_ov8858_otp.bg_ratio);
		return -1;
	}
	//calculate G gain
	//0x400 = 1x gain
	if(g_ov8858_otp.bg_ratio < BG_Ratio_Typical) {
		if (g_ov8858_otp.rg_ratio< RG_Ratio_Typical) {
			G_gain = 0x400;
			B_gain = 0x400 * BG_Ratio_Typical / g_ov8858_otp.bg_ratio;
			R_gain = 0x400 * RG_Ratio_Typical / g_ov8858_otp.rg_ratio;
		}
		else {
			R_gain = 0x400;
			G_gain = 0x400 * g_ov8858_otp.rg_ratio / RG_Ratio_Typical;
			B_gain = G_gain * BG_Ratio_Typical /g_ov8858_otp.bg_ratio;
		}
	}
	else {
		if (g_ov8858_otp.rg_ratio < RG_Ratio_Typical) {
			B_gain = 0x400;
			G_gain = 0x400 * g_ov8858_otp.bg_ratio / BG_Ratio_Typical;
			R_gain = G_gain * RG_Ratio_Typical / g_ov8858_otp.rg_ratio;
		}
		else {
			G_gain_B = 0x400 * g_ov8858_otp.bg_ratio / BG_Ratio_Typical;
			G_gain_R = 0x400 * g_ov8858_otp.rg_ratio / RG_Ratio_Typical;
			if(G_gain_B > G_gain_R ) {
				B_gain = 0x400;
				G_gain = G_gain_B;
				R_gain = G_gain * RG_Ratio_Typical /g_ov8858_otp.rg_ratio;
			}
			else {
				R_gain = 0x400;
				G_gain = G_gain_R;
				B_gain = G_gain * BG_Ratio_Typical / g_ov8858_otp.bg_ratio;
			}
		}
	}

	if (R_gain>0x400) {
		ov8858_otp_write_i2c(s_ctrl,0x5032, R_gain>>8);
		ov8858_otp_write_i2c(s_ctrl,0x5033, R_gain & 0x00ff);
	}
	if (G_gain>0x400) {
		ov8858_otp_write_i2c(s_ctrl,0x5034, G_gain>>8);
		ov8858_otp_write_i2c(s_ctrl,0x5035, G_gain & 0x00ff);
	}
	if (B_gain>0x400) {
		ov8858_otp_write_i2c(s_ctrl,0x5036, B_gain>>8);
		ov8858_otp_write_i2c(s_ctrl,0x5037, B_gain & 0x00ff);
	}

	CMR_LOGD("%s: R_gain=%d, G_gain=%d, B_gain=%d \n",__func__,R_gain,G_gain,B_gain);
	return 0;
}

/****************************************************************************
 * FunctionName: ov8858_update_lenc_otp;
 * Description : write 110 lens shading otp data to sensor;
 ***************************************************************************/
int32_t ov8858_update_lenc_otp(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t i = 0;
	uint16_t temp = 0;

	ov8858_otp_read_i2c(s_ctrl,0x5000, &temp);
	temp = 0x80 | temp;
	ov8858_otp_write_i2c(s_ctrl, 0x5000, temp);

	for(i=0;i<OV8858_MAX_OTP_LENS_NUM;i++) {
		ov8858_otp_write_i2c(s_ctrl, 0x5800 + i, g_ov8858_otp.lenc[i]);
	}

	return 0;
}

/****************************************************************************
 * FunctionName: ov8858_foxconn_get_otp_info;
 * Description : get otp info from sensor;
 ***************************************************************************/
int32_t ov8858_foxconn_get_otp_info(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t ov8858_mmi_otp_flag = 0x0F;
	CMR_LOGD("Get ov8858 foxconn OTP info Enter\n");
	//set sensor mode:Active
	ov8858_otp_write_i2c(s_ctrl, 0x100, 0x1);

	rc = ov8858_read_otp_data(s_ctrl, &ov8858_mmi_otp_flag);
	s_ctrl->hw_otp_check_flag.mmi_otp_check_flag =  ov8858_mmi_otp_flag;
	CMR_LOGI("%s ov8858_mmi_otp_flag = 0x%x\n",__func__, s_ctrl->hw_otp_check_flag.mmi_otp_check_flag);
	usleep_range(5000, 6000);

	//set sensor mode:Standby
	ov8858_otp_write_i2c(s_ctrl, 0x100, 0x0);

	CMR_LOGD("Get ov8858 foxconn OTP info Exit\n");
	return rc;
}

/****************************************************************************
 * FunctionName: ov8858_foxconn_set_otp_info;
 * Description : set otp data to sensor;
 * call this function after OV8858 initialization
 ***************************************************************************/
int ov8858_foxconn_otp_func(struct msm_sensor_ctrl_t *s_ctrl,int index)
{
	int32_t rc = 0;
	static int i_read_otp = 0;

	if(otp_function_lists[index].rg_ratio_typical)
	{
		RG_Ratio_Typical = otp_function_lists[index].rg_ratio_typical;
	}

	if(otp_function_lists[index].bg_ratio_typical)
	{
		BG_Ratio_Typical = otp_function_lists[index].bg_ratio_typical;
	}
	CMR_LOGD("%s, rg_ratio_typical=%04x,bg_ratio_typical=%04x\n", __func__,RG_Ratio_Typical,BG_Ratio_Typical );


	//Get otp info on the first time
	if ( 0 == i_read_otp )
	{
		rc = ov8858_foxconn_get_otp_info(s_ctrl);

		if ( rc < 0 )
		{
			CMR_LOGE("%s:%d otp read failed.\n", __func__, __LINE__);
			return -1;
		}
		else
		{
			i_read_otp = 1;
		}
	}

	CMR_LOGI("Set ov8858 foxconn OTP info Enter\n");

	ov8858_otp_write_i2c(s_ctrl, 0x100, 0x1);

	ov8858_update_awb_otp(s_ctrl);

	ov8858_update_lenc_otp(s_ctrl);

	usleep_range(5000, 6000);

	ov8858_otp_write_i2c(s_ctrl, 0x100, 0x0);

	CMR_LOGI("Set ov8858 foxconn OTP info Exit\n");
	
	return rc;
}

