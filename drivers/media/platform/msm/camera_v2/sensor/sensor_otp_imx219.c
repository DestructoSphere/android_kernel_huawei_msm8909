/************************************************************
  Copyright (C), 2014-2015, Huawei Tech. Co., Ltd.
FileName: sensor_otp_imx219.c
Author:  jwx206032
Version :Initial Draft
Date: 2015/02/27
Description:    this file contion several functions to detect otp_imx219 properties
Version:         Initial Draft
History:
History        :
1.Date        : 2015/02/27
Author        : jwx206032
Modification : Created function
 ***********************************************************/

//#define HW_CMR_LOGSWC 0   //file log switch set 0 off,default is 1 on
#define HW_CMR_LOG_TAG "sensor_otp_imx219"

#include <linux/hw_camera_common.h>
#include "msm_cci.h"
#include "msm_sensor.h"
#include "sensor_otp_common_if.h"

static  uint32_t rg_ratio_typical = 0x248;   //the average of 4 Golden samples' RG ratio
static  uint32_t bg_ratio_typical = 0x257;   //the average of 4 Golden samples' BG ratio

//#define IMX219_OTP_READ_TIME_PRINT
#define OTP_SLAVE_ADDR1       0xa0
#define OTP_SLAVE_ADDR2       0xa2
#define IMX219_SLAVE_ADDR     0x20

#define OTP_ID_REG 		      0x00
#define OTP_AWB_REG 	      0x05
#define OTP_LSC_1_REG 	      0x0b
#define OTP_LSC_2_REG	      0x00
#define OTP_VCM_REG 	      0x23
#define OTP_CHECKSUM_REG      0x27

#define IMX219_OTP_ID_READ				(1 << 0)
#define IMX219_OTP_VCM_READ				(1 << 1)
#define IMX219_OTP_LSC_READ				(1 << 2)
#define IMX219_OTP_AWB_READ  			(1 << 3)
#define IMX219_OTP_CHECKSUM_READ        (1 << 4)
#define IMX219_OTP_CHECKSUM_ERR			(1 << 5)
#define IMX219_OTP_FAIL_FLAG			(1 << 6)

#define IMX219_OTP_LSC_SIZE             280
#define IMX219_OTP_LSC_QUARTER_SIZE     70

#define OTP_INSURANCE_OFFSET 0x30

#define IMX219_MMI_OTP_VCM_FLAG          (1 << 0)
#define IMX219_MMI_OTP_AWB_FLAG          (1 << 1)
#define IMX219_MMI_OTP_MODULE_INFO_FLAG  (1 << 2)
#define IMX219_MMI_OTP_LSC_FLAG          (1 << 3)

//the value used for vcm effect, maybe modified by others
#define IMX219_OTP_VCM_OFFSET_VALUE       80
typedef enum {
	SUNNY_MODULE_VENDOR_ID = 1,
	FOXCONN_MODULE_VENDOR_ID,
	LITEON_MODULE_VENDOR_ID,
	SEMCO_MODULE_VENDOR_ID,
	BYD_MODULE_VENDOR_ID,
	OFILM_MODULE_VENDOR_ID
}camera_module_vendor_id;

static uint16_t imx219_vcm_start = 0;
static uint16_t imx219_vcm_end   = 0;
static uint32_t OTPSUMVAL        = 0;
static uint8_t  imx219_otp_lsc_param[IMX219_OTP_LSC_SIZE] ;
static uint8_t imx219_otp_flag = 0;
static bool imx219_get_otp_from_sensor(struct msm_sensor_ctrl_t *s_ctrl);
static bool imx219_otp_set_lsc(struct msm_sensor_ctrl_t *s_ctrl);

/****************************************************************************
* FunctionName: imx219_cci_i2c_write;
* Description : i2c write interface;
***************************************************************************/
static int32_t imx219_cci_i2c_write(struct msm_sensor_ctrl_t *s_ctrl,uint32_t addr, u16 data)
{
	int32_t rc = -EFAULT;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		s_ctrl->sensor_i2c_client, addr, data, MSM_CAMERA_I2C_BYTE_DATA);

	if (rc < 0)
	{
		CMR_LOGE("%s fail, rc = %d! addr = 0x%x, data = 0x%x\n", __func__, rc, addr, data);
	}

	return rc;
}

/****************************************************************************
* FunctionName: imx219_cci_i2c_read;
* Description : i2c read interface;
***************************************************************************/
static int32_t imx219_cci_i2c_read(struct msm_sensor_ctrl_t *s_ctrl, uint32_t addr, uint16_t *data)
{
	int32_t rc = -EFAULT;
	uint16_t temp_data = 0;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			addr,
			&temp_data, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0)
	{
		CMR_LOGE("%s fail, rc = %d! addr = 0x%x, data = 0x%x\n", __func__, rc, addr, temp_data);
	}
	*data = temp_data;

	return rc;
}

/*
 **************************************************************************
 * FunctionName: imx219_module_i2c_opt_interface;
 * Description : the i2c api to use imx219 module;
 * Input         : NULL;
 * Output       : NA;
 * ReturnValue :NONEl ;
 * Other         : NA;
 **************************************************************************
*/
static int imx219_module_i2c_opt_interface(struct msm_sensor_ctrl_t * s_ctrl,uint16_t i2c_addr,enum msm_camera_i2c_reg_addr_type addr_type)
{
	int rc = 0;
	CMR_LOGD("%s enter! i2c_addr = 0x%x.\n",__func__,i2c_addr);

	s_ctrl->sensor_i2c_client->addr_type = addr_type;

	if (s_ctrl->sensor_i2c_client->client)
	{
	    s_ctrl->sensor_i2c_client->client->addr = i2c_addr;
	}
	else if (s_ctrl->sensor_i2c_client->cci_client)
	{
		s_ctrl->sensor_i2c_client->cci_client->sid = (i2c_addr >> 1);
	}
	else
	{
		CMR_LOGE("%s,error: no i2c/cci client found.\n", __func__);
		rc = -EFAULT;
	}

	return rc;
}

/****************************************************************************
* FunctionName: imx219_read_otp;
* Description : i2c api used to read information from eeprom.
* Input       : NA;
* Output      : imx219_otp;
* ReturnValue : NONEl;
* Other       : NA;
***************************************************************************/
static bool imx219_read_otp(struct msm_sensor_ctrl_t *s_ctrl,uint16_t i2c_addr,uint16_t reg,uint8_t *buf,uint16_t count)
{
	uint16_t i = 0;
	int ret = 0;
	uint16_t val = 0;

	ret = imx219_module_i2c_opt_interface(s_ctrl,i2c_addr, MSM_CAMERA_I2C_BYTE_ADDR);

	if (ret < 0)
	{
		return false;
	}

	for (i=0; i<count; i++)
	{
		ret =imx219_cci_i2c_read(s_ctrl,(reg+i) ,&val);
		if (ret < 0)
		{
			CMR_LOGE("%s fail to read otp with error code %d, i2c_addr=0x%x reg_addr=0x%x\n", __func__,ret,i2c_addr,reg+i);
			return false;
		}
		buf[i] = (val&0xff);
		OTPSUMVAL += buf[i];
	}

	return true;
}

static bool imx219_otp_read_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint8_t buf[5] = {0};
	uint8_t vendor_id = 0;

	CMR_LOGD("enter %s", __func__);

	imx219_read_otp(s_ctrl,OTP_SLAVE_ADDR1,OTP_ID_REG,buf,5);

	CMR_LOGD("module info year 20%02d month %d day %d, SNO. 0x%x  vendor id&version 0x%x\n", buf[0],buf[1],buf[2],buf[3],buf[4]);

	vendor_id = buf[4]>>4;
	if (vendor_id == LITEON_MODULE_VENDOR_ID && buf[3] == 0x98) { //Liteon 0x03 & huaweiModuleCode is 23060152(152 = 0x98)
		imx219_otp_flag |= IMX219_OTP_ID_READ;
		return true;
	} else if (vendor_id == OFILM_MODULE_VENDOR_ID && buf[3] == 0x98) {
		imx219_otp_flag |= IMX219_OTP_ID_READ;
		return true;
	} else {
		CMR_LOGE("%s OTP data is worng for with wrong vender id!!!\n",__func__);
		return false;
	}
}

static bool imx219_otp_read_awb(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint8_t  buf[6] = {0};
	uint16_t awbRG,awbBG,awbRB;

	imx219_read_otp(s_ctrl,OTP_SLAVE_ADDR1,OTP_AWB_REG,buf,6);

	CMR_LOGD("%s OTP data are Rg_high=%x, Rg_low=%x, Bg_high=%x, Bg_low=%x, gbgr_high=%x, gbgr_low=%x!!!\n", __func__, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

	awbRG = buf[0];
	awbRG <<= 8;
	awbRG += buf[1];
	awbBG = buf[2];
	awbBG <<= 8;
	awbBG += buf[3];
	awbRB = buf[4];
	awbRB <<= 8;
	awbRB += buf[5];
	CMR_LOGI("%s OTP data are awbRG=%x, awbBG=%x, awbRB=%x!!!\n",
		__func__, awbRG, awbBG, awbRB);

	if(0 == awbRG || 0 ==awbBG || 0 == awbRB){//if awb value read is error for zero, abnormal branch deal
		CMR_LOGE("%s OTP data is worng!!!\n",__func__);
		return false;
	}

	imx219_otp_flag |= IMX219_OTP_AWB_READ;

	s_ctrl->awb_otp_info.RG = awbRG;
	s_ctrl->awb_otp_info.BG = awbBG;
	s_ctrl->awb_otp_info.typical_RG = rg_ratio_typical;
	s_ctrl->awb_otp_info.typical_BG = bg_ratio_typical;

	return true;
}

static bool imx219_otp_read_lsc(struct msm_sensor_ctrl_t *s_ctrl)
{
	CMR_LOGD("enter %s", __func__);

	memset(imx219_otp_lsc_param, 0, IMX219_OTP_LSC_SIZE);
	//LSC 0xa0:0b--0xff & 0xa2:00--0x22  total = 280
	imx219_read_otp(s_ctrl,OTP_SLAVE_ADDR1,OTP_LSC_1_REG,imx219_otp_lsc_param,0xff-0x0b+1);
	imx219_read_otp(s_ctrl,OTP_SLAVE_ADDR2,OTP_LSC_2_REG,&imx219_otp_lsc_param[0xff-0x0b+1],0x22+1);

	CMR_LOGI("%s LCS[0]= %x,LSC[244] = %x  LSC[245]=%x,LSC[279]=%d\n",__func__,
		imx219_otp_lsc_param[0],imx219_otp_lsc_param[244],imx219_otp_lsc_param[245],imx219_otp_lsc_param[279]);
	imx219_otp_flag |= IMX219_OTP_LSC_READ;

	return true;
}

/****************************************************************************
* FunctionName: imx219_otp_set_lsc;
* Description : Set lens shading parameters to sensor registers.; cost time is 0.0341s on sunny module
* Input       : NA;
* Output      : NA;
* ReturnValue : bool;
* Other       : NA;
***************************************************************************/
static bool imx219_otp_set_lsc(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint8_t *pval = NULL;
	int i = 0;

	CMR_LOGD("enter %s\n", __func__);

    //Access Code for address over 0x3000
	imx219_cci_i2c_write(s_ctrl,0x30EB, 0x05);
	imx219_cci_i2c_write(s_ctrl,0x30EB, 0x0C);
	imx219_cci_i2c_write(s_ctrl,0x300A, 0xFF);
	imx219_cci_i2c_write(s_ctrl,0x300B, 0xFF);
	imx219_cci_i2c_write(s_ctrl,0x30EB, 0x05);
    imx219_cci_i2c_write(s_ctrl,0x30EB, 0x09);

	/* Lens shading parameters are burned OK,Write lens shading parameters to sensor registers. */
	pval = imx219_otp_lsc_param;
	for (i=0; i<IMX219_OTP_LSC_QUARTER_SIZE; i++,pval++) {//target reg:0xD200 ~ 0xD245
		imx219_cci_i2c_write(s_ctrl,0xd200+i, *pval);
		CMR_LOGD("LSC[%d] = 0x%x \n",i,*pval);
	}

	for (i=0; i<IMX219_OTP_LSC_QUARTER_SIZE; i++,pval++) {//target reg:0xD248 ~ 0xD28D
		imx219_cci_i2c_write(s_ctrl,0xd248+i, *pval);
		CMR_LOGD("LSC[%d] = 0x%x \n",70+i,*pval);
	}

	for (i=0; i<IMX219_OTP_LSC_QUARTER_SIZE; i++,pval++) {//target reg:0xD290 ~ 0xD2D5
		imx219_cci_i2c_write(s_ctrl,0xd290+i, *pval);
		CMR_LOGD("LSC[%d] = 0x%x \n",140+i,*pval);
	}

	for (i=0; i<IMX219_OTP_LSC_QUARTER_SIZE; i++,pval++) {//target reg:0xD2D8 ~  0xD31D
		imx219_cci_i2c_write(s_ctrl,0xd2d8+i, *pval);
		CMR_LOGD("LSC[%d] = 0x%x \n",210+i,*pval);
	}

	//Open OTP lsc mode
	imx219_cci_i2c_write(s_ctrl,0x0190, 0x01); //LSC enable A
	imx219_cci_i2c_write(s_ctrl,0x0192, 0x00); //LSC table 0
	imx219_cci_i2c_write(s_ctrl,0x0191, 0x00); //LCS color mode:4 color R/Gr/Gb/B
	imx219_cci_i2c_write(s_ctrl,0x0193, 0x00); //LSC tuning disable
	imx219_cci_i2c_write(s_ctrl,0x01a4, 0x03); //Knot point format A:u4.8

	CMR_LOGD("%s,set OTP LSC to sensor OK.\n", __func__);

	return true;
}

/****************************************************************************
* FunctionName: imx219_otp_read_vcm;
* Description : Get AF motor parameters from OTP.;
* Input       : NA;
* Output      : NA;
* ReturnValue : bool;
* Other       : NA;
***************************************************************************/
static bool imx219_otp_read_vcm(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint8_t buf[4] = {0};
    uint16_t start_code,end_code;

	CMR_LOGD("enter %s", __func__);

	s_ctrl->afc_otp_info.starting_dac  = 0;
	s_ctrl->afc_otp_info.infinity_dac  = 0;
	s_ctrl->afc_otp_info.macro_dac     = 0;

	if((imx219_otp_flag & IMX219_OTP_VCM_READ) == IMX219_OTP_VCM_READ) { //OTP data is read all ready
		CMR_LOGD("%s OTP VCM data is read all ready!!!\n",__func__);
		return true;
	}

	imx219_read_otp(s_ctrl,OTP_SLAVE_ADDR2,OTP_VCM_REG,buf,4);

	start_code = buf[0];
	start_code <<= 8;
	start_code += buf[1];
	end_code = buf[2];
	end_code <<= 8;
	end_code += buf[3];

	if((start_code != end_code) &&(end_code>start_code) && start_code != 0 && end_code != 0) {
		imx219_otp_flag |= IMX219_OTP_VCM_READ;//this is no need to use, just use imx219_vcm_start/end 0 value to achieve
		imx219_vcm_start = start_code;
		imx219_vcm_end = end_code;
		CMR_LOGI("%s imx219_vcm_start= 0x%x, imx219_vcm_end = 0x%x \n",__func__, imx219_vcm_start,imx219_vcm_end);

	} else {//Abnormal branch deal
		imx219_vcm_start = 0;
		imx219_vcm_end = 0;
		CMR_LOGE("%s VCM OTP data is worng! imx219_vcm_start= 0x%x, imx219_vcm_end = 0x%x \n",__func__, imx219_vcm_start,imx219_vcm_end);
		return false;
	}

	if (imx219_vcm_start <= IMX219_OTP_VCM_OFFSET_VALUE)
	{
		CMR_LOGE("%s, imx219_vcm_start = 0x%x\n", __func__,imx219_vcm_start);
		imx219_vcm_start = 0;
	}
	else
	{
		imx219_vcm_start -= IMX219_OTP_VCM_OFFSET_VALUE;
	}

	imx219_vcm_end += IMX219_OTP_VCM_OFFSET_VALUE;

	s_ctrl->afc_otp_info.starting_dac = imx219_vcm_start;
	s_ctrl->afc_otp_info.infinity_dac = imx219_vcm_start;
	s_ctrl->afc_otp_info.macro_dac = imx219_vcm_end;

	return true;
}

static bool imx219_get_otp_from_sensor(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint8_t sum = 0;
    uint8_t otpCheckSumVal = 0;
	bool retVal = false;
	uint8_t tmp_mmi_otp_flag = 0x0F;

#ifdef IMX219_OTP_READ_TIME_PRINT
	uint16_t otpReadUsed = 0;
	struct timeval otpReadBegin, otpReadEnd;
#endif

	CMR_LOGI("%s enters!\n",__func__);
	if((imx219_otp_flag & IMX219_OTP_FAIL_FLAG) == IMX219_OTP_FAIL_FLAG) {//Just check OTP once whether success or not
		CMR_LOGE("%s OTP data is worng!\n", __func__);
		return false;
	}else if((imx219_otp_flag & IMX219_OTP_CHECKSUM_READ) == IMX219_OTP_CHECKSUM_READ){//branch for read success, no need read more
		CMR_LOGI("%s OTP has been read success already!\n",__func__);
		return true;
	}

#ifdef IMX219_OTP_READ_TIME_PRINT
	memset(&otpReadBegin,0,sizeof(otpReadBegin));
	memset(&otpReadEnd,0,sizeof(otpReadEnd));
	do_gettimeofday(&otpReadBegin);
#endif

	retVal = imx219_otp_read_id(s_ctrl);
	if(false == retVal){
		CMR_LOGE("%s imx219_otp_read_id(s_ctrl) failed!\n",__func__);
		goto OTP_FAIL;
	}
     tmp_mmi_otp_flag &= ~IMX219_MMI_OTP_MODULE_INFO_FLAG;

	retVal = imx219_otp_read_awb(s_ctrl);
	if(false == retVal){
		CMR_LOGE("%s imx219_otp_read_awb(s_ctrl) failed!\n",__func__);
		goto OTP_FAIL;
	}
	 tmp_mmi_otp_flag &= ~IMX219_MMI_OTP_AWB_FLAG;

	retVal = imx219_otp_read_vcm(s_ctrl);
	if(false == retVal){
		CMR_LOGE("%s imx219_otp_read_vcm(s_ctrl) failed!\n",__func__);
		goto OTP_FAIL;
	}
	tmp_mmi_otp_flag &= ~IMX219_MMI_OTP_VCM_FLAG;

	imx219_otp_read_lsc(s_ctrl);
    tmp_mmi_otp_flag &= ~IMX219_MMI_OTP_LSC_FLAG;

	imx219_read_otp(s_ctrl,OTP_SLAVE_ADDR2,OTP_CHECKSUM_REG,&otpCheckSumVal,1);

	sum = (OTPSUMVAL - otpCheckSumVal)%0xff + 1;

#ifdef IMX219_OTP_READ_TIME_PRINT
	do_gettimeofday(&otpReadEnd);
	otpReadUsed = (otpReadEnd.tv_sec - otpReadBegin.tv_sec) * 1000 + (otpReadEnd.tv_usec - otpReadBegin.tv_usec) / 1000;
	CMR_LOGI("%s used time is %d ms\n", __func__,otpReadUsed);
#endif

	if(otpCheckSumVal == sum){
		imx219_otp_flag |= IMX219_OTP_CHECKSUM_READ;
		CMR_LOGE("%s success, OTPSUMVAL: %d, otpCheckSumVal: %d ,sum:%d, imx219_otp_flag=0x%x\n", __func__, OTPSUMVAL, otpCheckSumVal,sum,imx219_otp_flag);
	    s_ctrl->hw_otp_check_flag.mmi_otp_check_flag  = tmp_mmi_otp_flag;
		CMR_LOGI("%s imx219_mmi_otp_flag = 0x%x\n",__func__, s_ctrl->hw_otp_check_flag.mmi_otp_check_flag);
		return true;
	} else {
		imx219_otp_flag |= IMX219_OTP_CHECKSUM_ERR;
		CMR_LOGE("%s fail, OTPSUMVAL: %d, otpCheckSumVal: %d ,sum:%d \n", __func__, OTPSUMVAL, otpCheckSumVal,sum);
	}

OTP_FAIL:
	imx219_otp_flag |= IMX219_OTP_FAIL_FLAG;
	CMR_LOGI("%s imx219_otp_flag=0x%x \n", __func__,imx219_otp_flag);
	s_ctrl->hw_otp_check_flag.mmi_otp_check_flag  = tmp_mmi_otp_flag;
	CMR_LOGI("%s imx219_mmi_otp_flag = 0x%x\n",__func__, s_ctrl->hw_otp_check_flag.mmi_otp_check_flag);
	return false;
}

static void imx219_set_otp_to_sensor(struct msm_sensor_ctrl_t *s_ctrl)
{
	if((imx219_otp_flag & IMX219_OTP_LSC_READ) == IMX219_OTP_LSC_READ && (imx219_otp_flag & IMX219_OTP_FAIL_FLAG) != IMX219_OTP_FAIL_FLAG){
		imx219_otp_set_lsc(s_ctrl);
	}
}

/******************************************************************************
Function   :  imx219_otp_func
Description:  read the otp info
******************************************************************************/
int imx219_otp_func(struct msm_sensor_ctrl_t * s_ctrl, int index)
{
	 int rc = 0;

	 CMR_LOGD("%s enters!\n",__func__);

	if (otp_function_lists[index].rg_ratio_typical)
	{
		rg_ratio_typical = otp_function_lists[index].rg_ratio_typical;
	}

	if (otp_function_lists[index].bg_ratio_typical)
	{
		bg_ratio_typical = otp_function_lists[index].bg_ratio_typical;
	}

	/* i2c opt in imx219 eeprom module */
	rc = imx219_get_otp_from_sensor(s_ctrl);

    /* i2c opt in imx219 sensor module */
	imx219_module_i2c_opt_interface(s_ctrl,IMX219_SLAVE_ADDR,MSM_CAMERA_I2C_WORD_ADDR);

	if (false == rc)
	{
		return -1;
	}

	imx219_set_otp_to_sensor(s_ctrl);

	CMR_LOGI("%s,IMX219 OTP read and set OK.\n", __func__);
	return rc;
}