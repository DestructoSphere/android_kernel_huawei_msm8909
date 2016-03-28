

//#define HW_CMR_LOGSWC 0   //file log switch set 0 off,default is 1 on
#define HW_CMR_LOG_TAG "sensor_otp_s5k4e1"

#include <linux/hw_camera_common.h>
#include "msm_sensor.h"
#include "sensor_otp_common_if.h"


struct  s5k4e1_otp_struct {
    uint32_t product_year;//product year
    uint32_t product_month;//product month
    uint32_t product_date;//product date
    uint32_t camera_id;//hw camera id
    uint32_t supplier_version_id;//supplier version code
    uint32_t version;//camera version
    uint32_t wb_rg;// r/g ratio
    uint32_t wb_bg;// b/g ratio
};

typedef enum tags5k4e1_otp_location
{
    S5K4E1_OTP_BEGIN = 0,
    S5K4E1_OTP_310D_LSB,//reg 0x310d lsb
    S5K4E1_OTP_310E_MSB,//reg 0x310e msb
    S5K4E1_OTP_310E_LSB,//reg 0x310e lsb
    S5K4E1_OTP_310F_MSB,//reg 0x310f msb
    S5K4E1_OTP_310F_LSB,//reg 0x310f lsb
    S5K4E1_OTP_MAX,//no otp
}s5k4e1_otp_location;

#define S5K4E1_LSB  0x0f
#define S5K4E1_MSB  0xf0

//otp Module info start layer
#define S5K4E1_LAYER_START      18
//otp Module info end layer
#define S5K4E1_LAYER_END        27
//awb otp start layer
#define S5K4E1_LAYER_AWB_START  28
//awb otp end layer
#define S5K4E1_LAYER_AWB_END    35

//number of otp info layers
#define S5K4E1_LAYER_NUMS   (S5K4E1_LAYER_AWB_END - S5K4E1_LAYER_START +1)

//AWB otp exist flag
#define S5K4E1_OTP_AWB          1
#define S5K4E1_OTP_ALL          (S5K4E1_OTP_AWB)

#define S5K4E1_MMI_OTP_AWB_FLAG          (1 << 1)

//R/G and B/G value of Golden Samples.
static  uint32_t rg_ratio_typical = 0x312;
static  uint32_t bg_ratio_typical = 0x2DE;

//Final r/g/b gain
static uint32_t gr_gain = 0;
static uint32_t gg_gain = 0;
static uint32_t gb_gain = 0;

//OTP read indications
static uint32_t s5k4e1_otp_read_flag = 0 ;


/****************************************************************************
* FunctionName: s5k4e1_cci_i2c_write;
* Description : i2c write interface;
***************************************************************************/
static int32_t s5k4e1_cci_i2c_write(struct msm_sensor_ctrl_t *s_ctrl,uint32_t addr, uint16_t data)
{
	int32_t rc = -EFAULT;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		s_ctrl->sensor_i2c_client, addr, data, MSM_CAMERA_I2C_BYTE_DATA);

	if(rc < 0)
	{
		CMR_LOGE("%s fail, rc = %d! addr = 0x%x, data = 0x%x\n", __func__, rc, addr, data);
	}

	return rc;
}

/****************************************************************************
* FunctionName: s5k4e1_cci_i2c_read;
* Description : i2c read interface;
***************************************************************************/
static int32_t s5k4e1_cci_i2c_read(struct msm_sensor_ctrl_t *s_ctrl, uint32_t addr, uint16_t *data)
{
	int32_t rc = -EFAULT;
	uint16_t temp_data = 0;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			addr,
			&temp_data, MSM_CAMERA_I2C_BYTE_DATA);
	if(rc < 0)
	{
		CMR_LOGE("%s fail, rc = %d! addr = 0x%x, data = 0x%x\n", __func__, rc, addr, temp_data);
	}
	*data = temp_data;
	return rc;
}
/******************************************************************************
Function   :   s5k4e1_otp_debug
Description:   Only for debug use
******************************************************************************/
void s5k4e1_otp_debug(struct s5k4e1_otp_struct  otp_ptr)
{
	CMR_LOGD("%s,otp_ptr.product_year:0x%x",__func__,otp_ptr.product_year);
	CMR_LOGD("%s,otp_ptr.product_month:0x%x",__func__,otp_ptr.product_month);
	CMR_LOGD("%s,otp_ptr.product_date:0x%x",__func__,otp_ptr.product_date);
	CMR_LOGD("%s,otp_ptr.camera_id:0x%x",__func__,otp_ptr.camera_id);
	CMR_LOGD("%s,otp_ptr.supplier_version_id:0x%x",__func__,otp_ptr.supplier_version_id);
	CMR_LOGD("%s,otp_ptr.version:0x%x\n",__func__,otp_ptr.version);
	CMR_LOGD("%s,otp_ptr.wb_rg:0x%x",__func__,otp_ptr.wb_rg);
	CMR_LOGD("%s,otp_ptr.wb_bg:0x%x",__func__,otp_ptr.wb_bg);
	return;
}
/******************************************************************************
Function   :    s5k4e1_update_awb_gain
Description:    Update the RGB AWB gain to sensor
******************************************************************************/
void s5k4e1_update_awb_gain(struct msm_sensor_ctrl_t *s_ctrl,
                                             uint32_t r_gain,
                                             uint32_t g_gain,
                                             uint32_t b_gain)
{
	CMR_LOGD("%s, R_gain:0x%x,G_gain:0x%x,B_gain:0x%x\n",__func__, r_gain, g_gain, b_gain);
	if (r_gain >= 0x100) {
		s5k4e1_cci_i2c_write(s_ctrl, 0x0210, r_gain >> 8);
		s5k4e1_cci_i2c_write(s_ctrl, 0x0211, r_gain & 0x00ff);
	}
	//G gain include Gr gain and Gb gain
	if (g_gain >= 0x100) {
		s5k4e1_cci_i2c_write(s_ctrl, 0x020e, g_gain >> 8);
		s5k4e1_cci_i2c_write(s_ctrl, 0x020f, g_gain & 0x00ff);

		s5k4e1_cci_i2c_write(s_ctrl, 0x0214, g_gain >> 8);
		s5k4e1_cci_i2c_write(s_ctrl, 0x0215, g_gain & 0x00ff);
	}
	if (b_gain >= 0x100) {
		s5k4e1_cci_i2c_write(s_ctrl, 0x0212, b_gain >> 8);
		s5k4e1_cci_i2c_write(s_ctrl, 0x0213, b_gain & 0x00ff);
	}
	return;
}

/******************************************************************************
Function:       s5k4e1_read_otp_regs
Description:    According the locaion, read the otp mem
******************************************************************************/
void s5k4e1_read_otp_regs(struct msm_sensor_ctrl_t * s_ctrl,
                                          struct s5k4e1_otp_struct *current_otp,
                                          uint32_t reg,
                                          int isMsb)
{
	int layer =0;
	int i = 0;
	uint16_t reg_values[S5K4E1_LAYER_NUMS] = {0};
	int flag_true = 0;
	uint8_t tmp_mmi_otp_flag = S5K4E1_MMI_OTP_AWB_FLAG;
	CMR_LOGD("%s reg:0x%x, isMsb:%d ", __func__, reg, isMsb);

	for (layer = S5K4E1_LAYER_START; layer <= S5K4E1_LAYER_AWB_END; layer++)
	{
		i = layer - S5K4E1_LAYER_START;

		//set layer
		s5k4e1_cci_i2c_write(s_ctrl, 0x310c, layer);

		//read specific reg value
		s5k4e1_cci_i2c_read(s_ctrl, reg, &reg_values[i]);

		if (flag_true == isMsb)//MSB
		{
			reg_values[i] =  (reg_values[i] & S5K4E1_MSB) >> 4;
		}
		else//LSB
		{
			reg_values[i] =  reg_values[i] & S5K4E1_LSB;
		}

		CMR_LOGD("%s reg_values[%d]:0x%x", __func__, i, reg_values[i]);
	}

	//copy the readed data into OTP struct according to otp map
	current_otp->product_year = (reg_values[0] <<4) + reg_values[1];
	current_otp->product_month = (reg_values[2] <<4) + reg_values[3];
	current_otp->product_date = (reg_values[4] <<4) + reg_values[5];
	current_otp->camera_id = (reg_values[6] <<4) + reg_values[7];
	current_otp->supplier_version_id = reg_values[8];
	current_otp->version = reg_values[9] ;
	current_otp->wb_rg = (reg_values[10] <<12) + (reg_values[11] <<8)
						 + (reg_values[12] <<4) + reg_values[13];
	current_otp->wb_bg = (reg_values[14] <<12) + (reg_values[15] <<8)
						 + (reg_values[16] <<4) + reg_values[17];
	  
	//check if AWB value is valid
	if ((0 !=  current_otp->wb_rg) && ( 0 !=  current_otp->wb_bg))
	{
		//read successful!
		s5k4e1_otp_read_flag |= S5K4E1_OTP_AWB;
		CMR_LOGD("%s read awb success!\n", __func__);
		tmp_mmi_otp_flag &= ~S5K4E1_MMI_OTP_AWB_FLAG;
	}
	s_ctrl->hw_otp_check_flag.mmi_otp_check_flag  = tmp_mmi_otp_flag;
	return;
}

/******************************************************************************
Function   :    s5k4e1_read_awb_otp
Description:    According the locaion, read the otp mem
******************************************************************************/
static int s5k4e1_read_awb_otp(struct msm_sensor_ctrl_t * s_ctrl,
                                                s5k4e1_otp_location location,
                                                struct s5k4e1_otp_struct *current_otp)
{
	int rc = 0;
	int flag_flase = -1;
	int flag_true = 0;
	//check invalid location
	if (location >= S5K4E1_OTP_MAX)
	{
		CMR_LOGE("%s Error! Invalid OTP location:%d \n", __func__, location);
		rc = -1;
		return rc;
	}

	CMR_LOGD("%s location:%d \n", __func__, location);
	//read otp regs according to the locations
	switch (location)
	{
		case S5K4E1_OTP_310D_LSB:
			s5k4e1_read_otp_regs(s_ctrl, current_otp, 0x310d, flag_flase);
			break;
		case S5K4E1_OTP_310E_MSB:
			s5k4e1_read_otp_regs(s_ctrl, current_otp, 0x310e, flag_true);
			break;
		case S5K4E1_OTP_310E_LSB:
			s5k4e1_read_otp_regs(s_ctrl, current_otp, 0x310e, flag_flase);
			break;
		case S5K4E1_OTP_310F_MSB:
			s5k4e1_read_otp_regs(s_ctrl, current_otp, 0x310f, flag_true);
			break;
		case S5K4E1_OTP_310F_LSB:
			s5k4e1_read_otp_regs(s_ctrl, current_otp, 0x310f, flag_flase);
			break;
		default:
			rc = -1;
			break;
	}

	return rc;
}

/******************************************************************************
Function   :   s5k4e1_check_awb_otp_location
Description:   find which location otp lays in
******************************************************************************/
s5k4e1_otp_location s5k4e1_check_awb_otp_location(struct msm_sensor_ctrl_t * s_ctrl)
{
	uint16_t reg310d = 0;//reg 0x310d value
	uint16_t reg310e = 0;//reg 0x310e value
	uint16_t reg310f = 0;//reg 0x310f value

	uint16_t layer_data = S5K4E1_LAYER_START;

	//use layer 18 to find which location OTP data lays in
	s5k4e1_cci_i2c_write(s_ctrl, 0x310c, layer_data);

	s5k4e1_cci_i2c_read(s_ctrl, 0x310d, &reg310d);
	s5k4e1_cci_i2c_read(s_ctrl, 0x310e, &reg310e);
	s5k4e1_cci_i2c_read(s_ctrl, 0x310f, &reg310f);
	CMR_LOGD("%s read layer %d: reg310d:0x%x, reg310e:0x%x, reg310f:0x%x \n", __func__, layer_data, reg310d, reg310e, reg310f);
	//find which reg postion otp lays in, from right to left
	if (reg310d & S5K4E1_LSB)
	{
		return S5K4E1_OTP_310D_LSB;
	}
	else if (reg310e & S5K4E1_MSB)
	{
		return S5K4E1_OTP_310E_MSB;
	}
	else if (reg310e & S5K4E1_LSB)
	{
		return S5K4E1_OTP_310E_LSB;
	}
	else if (reg310f & S5K4E1_MSB)
	{
		return S5K4E1_OTP_310F_MSB;
	}
	else if (reg310f & S5K4E1_LSB)
	{
		return S5K4E1_OTP_310F_LSB;
	}
	else
	{
		CMR_LOGE("%s Error! No data in OTP! \n", __func__);
	}

	return S5K4E1_OTP_MAX;
}



/**************************************************************************
* FunctionName: s5k4e1_start_otp_read;
* Description : OTP start read process;
***************************************************************************/
static int s5k4e1_start_otp_read (struct msm_sensor_ctrl_t *s_ctrl)
{
	//OTP start read process
	s5k4e1_cci_i2c_write(s_ctrl, 0x30F9, 0x0E);
	s5k4e1_cci_i2c_write(s_ctrl, 0x30FA, 0x0A);
	s5k4e1_cci_i2c_write(s_ctrl, 0x30FB, 0x71);
	s5k4e1_cci_i2c_write(s_ctrl, 0x30FB, 0x70);

	//must delay
	msleep(5);

	return 0;
}

/**************************************************************************
* FunctionName: s5k4e1_update_otp;
* Description : read calculate otp info;
***************************************************************************/
static int s5k4e1_update_otp (struct msm_sensor_ctrl_t *s_ctrl)
{
	struct s5k4e1_otp_struct current_otp;
	uint32_t r_gain = 0;
	uint32_t g_gain = 0;
	uint32_t b_gain = 0;
	uint32_t g_gain_r = 0;
	uint32_t g_gain_b = 0;
	uint32_t rg = 0;
	uint32_t bg = 0;

	s5k4e1_otp_location awb_otp_Location = S5K4E1_OTP_MAX;
	memset(&current_otp, 0, sizeof(struct s5k4e1_otp_struct));

	/*read*/
	//send read commnad to IC
	s5k4e1_start_otp_read(s_ctrl);

	//check AWB OTP location
	awb_otp_Location = s5k4e1_check_awb_otp_location(s_ctrl);
	if (0 != s5k4e1_read_awb_otp(s_ctrl, awb_otp_Location, &current_otp))
	{
		CMR_LOGE("%s Error!no data in OTP!\n", __func__);
		return -1;
	}

	//debug info
	s5k4e1_otp_debug(current_otp);

	if (0 ==(s5k4e1_otp_read_flag & S5K4E1_OTP_AWB))
	{
		CMR_LOGE("%s Error!no AWB data in OTP!\n", __func__);
		return -1;
	}

	//validate the AWB data gain
	if ((0 == current_otp.wb_rg) || (0 ==current_otp.wb_bg))
	{
		CMR_LOGE("%s Error!Zero RG or BG data! RG:0x%x, BG:0x%x!\n", __func__, current_otp.wb_rg, current_otp.wb_bg);
		return -1;
	}
	/*calculate*/
	rg = current_otp.wb_rg;
	bg = current_otp.wb_bg;

	//calculate RGB gain
	//0x100 = 1x gain
	if(bg < bg_ratio_typical) {
		if (rg< rg_ratio_typical)
		{
			// current_otp.bg_ratio < BG_Ratio_typical &&
			// current_otp.rg_ratio < RG_Ratio_typical
			g_gain = 0x100;
			b_gain = (0x100 * bg_ratio_typical) / bg;
			r_gain = (0x100 * rg_ratio_typical) / rg;
		}
		else
		{
			// current_otp.bg_ratio < BG_Ratio_typical &&
			// current_otp.rg_ratio >= RG_Ratio_typical
			r_gain = 0x100;
			g_gain = (0x100 * rg) / rg_ratio_typical;
			b_gain = (g_gain * bg_ratio_typical) / bg;
		}
	}
	else {
		if (rg < rg_ratio_typical)
		{
			// current_otp.bg_ratio >= BG_Ratio_typical &&
			// current_otp.rg_ratio < RG_Ratio_typical
			b_gain = 0x100;
			g_gain = (0x100 * bg) / bg_ratio_typical;
			r_gain = (g_gain * rg_ratio_typical) / rg;
		}
		else
		{
			// current_otp.bg_ratio >= BG_Ratio_typical &&
			// current_otp.rg_ratio >= RG_Ratio_typical
			g_gain_b = (0x100 * bg) / bg_ratio_typical;
			g_gain_r = (0x100 * rg) / rg_ratio_typical;

			if(g_gain_b > g_gain_r )
			{
				b_gain = 0x100;
				g_gain = g_gain_b;
				r_gain = (g_gain * rg_ratio_typical) / rg;
			}
			else
			{
				r_gain = 0x100;
				g_gain = g_gain_r;
				b_gain = (g_gain * bg_ratio_typical) / bg;
			}
		}
	}

	CMR_LOGD("%s,: R_gain:0x%x,G_gain:0x%x,B_gain:0x%x\n", __func__, r_gain, g_gain, b_gain);
	gr_gain = r_gain;
	gg_gain = g_gain;
	gb_gain = b_gain;

	return 0;
}

/******************************************************************************
Function   :    s5k4e1_set_otp_info
Description:    check whether we need
******************************************************************************/
int s5k4e1_otp_func(struct msm_sensor_ctrl_t *s_ctrl, int index)
{

	if(otp_function_lists[index].rg_ratio_typical)
	{
		rg_ratio_typical = otp_function_lists[index].rg_ratio_typical;
	}

	if(otp_function_lists[index].bg_ratio_typical)
	{
		bg_ratio_typical = otp_function_lists[index].bg_ratio_typical;
	}
	CMR_LOGD("%s, rg_ratio_typical=%04x,bg_ratio_typical=%04x\n", __func__,rg_ratio_typical,bg_ratio_typical);


	if (0 == (s5k4e1_otp_read_flag & S5K4E1_OTP_AWB))
	{
		if(0 != s5k4e1_update_otp(s_ctrl))
		{
			//todo need to check whether stop the sensor
			CMR_LOGE("%s : No OTP info! return error!\n", __func__);
			return -1;
		}
	}
	else
	{
		CMR_LOGD("%s :AWB OTP has already been read out!\n",__func__);
	}
	/*write*/
	s5k4e1_update_awb_gain(s_ctrl, gr_gain, gg_gain, gb_gain);

	return 0;
}
