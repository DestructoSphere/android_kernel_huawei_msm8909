/*
 *drivers/input/touchscreen/ft5x06_ex_fun.c
 *
 *FocalTech ft6x06 expand function for debug.
 *
 *Copyright (c) 2010  Focal tech Ltd.
 *
 *This software is licensed under the terms of the GNU General Public
 *License version 2, as published by the Free Software Foundation, and
 *may be copied, distributed, and modified under those terms.
 *
 *This program is distributed in the hope that it will be useful,
 *but WITHOUT ANY WARRANTY; without even the implied warranty of
 *MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *GNU General Public License for more details.
 *
 *Note:the error code of EIO is the general error in this file.
 */

#include <linux/mount.h>
#include <linux/netdevice.h>
#include <linux/proc_fs.h>
#ifdef CONFIG_APP_INFO
#include <misc/app_info.h> 
#endif
#include "ft6x06_ex_fun.h"
#include "ft6x06_ts.h"
#include "focaltech_log.h"

extern struct dsm_client *tp_dclient;
extern struct ft6x06_ts_data * g_ft6x06_ts;

#include "scap_test_lib.h"
extern struct i2c_client *i2c_client;
struct i2c_client *g_focalclient = NULL;
static int g_focalscaptested = 0;
static struct mutex g_device_mutex;

static unsigned int fw_jun_len = 0;
static unsigned char * CTPM_FW_JUN = NULL;
static unsigned char CTPM_FW_JUN_Y550[] = {
	#include "focaltech_Jun_0X85_Y550.i"
};
static unsigned char CTPM_FW_JUN_ULC02[] = {
    #include "focaltech_Jun_0X85_ULC02.i"
};
static unsigned int fw_lansi_len = 0;
static unsigned char * CTPM_FW_LANSI = NULL;
static unsigned char CTPM_FW_LANSI_Y550[] = {
	#include "ft6x06_lansi_firmware.i"
};

struct Upgrade_Info {
	u16 delay_aa;		/*delay of write FT_UPGRADE_AA */
	u16 delay_55;		/*delay of write FT_UPGRADE_55 */
	u8 upgrade_id_1;	/*upgrade id 1 */
	u8 upgrade_id_2;	/*upgrade id 2 */
	u16 delay_readid;	/*delay of read id */
	u16 delay_earse_flash; /*delay of earse flash*/
};


int fts_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			  u32 dw_lenth);




int ft6x06_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};
	buf[0] = regaddr;
	buf[1] = regvalue;

	return ft6x06_i2c_Write(client, buf, sizeof(buf));
}


int ft6x06_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{
	return ft6x06_i2c_Read(client, &regaddr, 1, regvalue, 1);
}


int fts_ctpm_auto_clb(struct i2c_client *client)
{
	unsigned char uc_temp = 0x00;
	unsigned char i = 0;

	/*start auto CLB */
	msleep(200);

	ft6x06_write_reg(client, 0, FTS_FACTORYMODE_VALUE);
	/*make sure already enter factory mode */
	msleep(100);
	/*write command to start calibration */
	ft6x06_write_reg(client, 2, 0x4);
	msleep(300);
	for (i = 0; i < 100; i++) {
		ft6x06_read_reg(client, 0, &uc_temp);
		/*return to normal mode, calibration finish */
		if (0x0 == ((uc_temp & 0x70) >> 4))
			break;
	}

	msleep(200);
	/*calibration OK */
	msleep(300);
	ft6x06_write_reg(client, 0, FTS_FACTORYMODE_VALUE);	/*goto factory mode for store */
	msleep(100);	/*make sure already enter factory mode */
	ft6x06_write_reg(client, 2, 0x5);	/*store CLB result */
	msleep(300);
	ft6x06_write_reg(client, 0, FTS_WORKMODE_VALUE);	/*return to normal mode */
	msleep(300);

	/*store CLB result OK */
	return 0;
}

void ft6x06_upgrade_with_i_handler(struct ft6x06_ts_data *ft6x06_ts)
{
    tp_log_info("%s,%d\n",__func__,__LINE__);
    
    if(ft6x06_ts->vendor_id_value == JUN_VENDOR_ID) 
    fts_ctpm_fw_upgrade_with_i_file(ft6x06_ts->client,CTPM_FW_JUN,fw_jun_len);
    else if(ft6x06_ts->vendor_id_value == LANSI_VENDOR_ID)
        fts_ctpm_fw_upgrade_with_i_file(ft6x06_ts->client,CTPM_FW_LANSI,fw_lansi_len);
    else
        tp_log_info("Error vendor id, id = %d\n", ft6x06_ts->vendor_id_value);
    
    tp_log_debug("complete for ft6x06_updata_completion \n");
}


/*
upgrade with *.i file
*/

int  fts_ctpm_fw_upgrade_with_i_file(struct i2c_client *client,unsigned char *CTPM_FW,u16 fw_len)
{
	u8 *pbt_buf = NULL;
	int i_ret;
	tp_log_info("CTPM_FW NUM = %d\n",fw_len);
	/*judge the fw that will be upgraded
	* if illegal, then stop upgrade and return.
	*/
	if (fw_len < 8 || fw_len > 32 * 1024) {
		tp_log_err("%s:FW length error\n", __func__);
		return -EIO;
	}

	/*FW upgrade */
	pbt_buf = CTPM_FW;
	/*call the upgrade function */
	i_ret = fts_ctpm_fw_upgrade(client, pbt_buf, fw_len);

    if (i_ret != 0)
    {
        tp_log_err("%s %d: firmware upgrade failed!\n", __func__, __LINE__);
        ft6x06_report_dsm_erro(g_ft6x06_ts, tp_dclient, DSM_TP_FW_ERROR_NO, 0);
    }
	
	return i_ret;
}

u8 fts_ctpm_get_i_file_ver(unsigned  char CTPM_FW[],u16 ui_sz)
{
	if (ui_sz > 2)
		return CTPM_FW[0x10a];

	return 0x00;	/*default value */
}

/*update project setting
*only update these settings for COB project, or for some special case
*/
int fts_ctpm_update_project_setting(struct i2c_client *client)
{
	u8 uc_i2c_addr;	/*I2C slave address (7 bit address)*/
	u8 uc_io_voltage;	/*IO Voltage 0---3.3v;	1----1.8v*/
	u8 uc_panel_factory_id;	/*TP panel factory ID*/
	u8 buf[FTS_SETTING_BUF_LEN];
	u8 reg_val[2] = {0};
	u8 auc_i2c_write_buf[10] = {0};
	u8 packet_buf[FTS_SETTING_BUF_LEN + 6];
	u32 i = 0;
	int i_ret;
	int rc = 0;

	uc_i2c_addr = client->addr;
	uc_io_voltage = 0x0;
	uc_panel_factory_id = 0x5a;


	/*Step 1:Reset  CTPM
	*write 0xaa to register 0xfc
	*/
	ft6x06_write_reg(client, 0xfc, 0xaa);
	msleep(50);

	/*write 0x55 to register 0xfc */
	ft6x06_write_reg(client, 0xfc, 0x55);
	msleep(30);

	/*********Step 2:Enter upgrade mode *****/
	auc_i2c_write_buf[0] = 0x55;
	auc_i2c_write_buf[1] = 0xaa;
	do {
		i++;
		i_ret = ft6x06_i2c_Write(client, auc_i2c_write_buf, 2);
		msleep(5);
	} while (i_ret <= 0 && i < 5);


	/*********Step 3:check READ-ID***********************/
	auc_i2c_write_buf[0] = 0x90;
	auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;

	rc = ft6x06_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);
	if(rc < 0)
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
	
	if (reg_val[0] == 0x79 && reg_val[1] == 0x3)
	{
		tp_log_debug("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n", reg_val[0], reg_val[1]);
	}
	else
	{
		tp_log_err("%s:Check check READ-ID fail\n", __func__);
		return -EIO;
	}

	auc_i2c_write_buf[0] = 0xcd;
	rc = ft6x06_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if(rc < 0)
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
	dev_dbg(&client->dev, "bootloader version = 0x%x\n", reg_val[0]);

	/*--------- read current project setting  ---------- */
	/*set read start address */
	buf[0] = 0x3;
	buf[1] = 0x0;
	buf[2] = 0x78;
	buf[3] = 0x0;

	rc = ft6x06_i2c_Read(client, buf, 4, buf, FTS_SETTING_BUF_LEN);
	if(rc < 0)
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
	tp_log_debug("[FTS] old setting: uc_i2c_addr = 0x%x,\
			uc_io_voltage = %d, uc_panel_factory_id = 0x%x\n",
			buf[0], buf[2], buf[4]);

	 /*--------- Step 4:erase project setting --------------*/
	auc_i2c_write_buf[0] = 0x63;
	ft6x06_i2c_Write(client, auc_i2c_write_buf, 1);
	msleep(100);

	/*----------  Set new settings ---------------*/
	buf[0] = uc_i2c_addr;
	buf[1] = ~uc_i2c_addr;
	buf[2] = uc_io_voltage;
	buf[3] = ~uc_io_voltage;
	buf[4] = uc_panel_factory_id;
	buf[5] = ~uc_panel_factory_id;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;
	packet_buf[2] = 0x78;
	packet_buf[3] = 0x0;
	packet_buf[4] = 0;
	packet_buf[5] = FTS_SETTING_BUF_LEN;

	for (i = 0; i < FTS_SETTING_BUF_LEN; i++)
		packet_buf[6 + i] = buf[i];

	ft6x06_i2c_Write(client, packet_buf, FTS_SETTING_BUF_LEN + 6);
	msleep(100);

	/********* reset the new FW***********************/
	auc_i2c_write_buf[0] = 0x07;
	ft6x06_i2c_Write(client, auc_i2c_write_buf, 1);

	msleep(200);
	return 0;
}
void  fts_ctpm_auto_upgrade(struct work_struct *ft6x06_fw_config_delay)
{
	u8 uc_host_fm_ver = 0;
	struct ft6x06_ts_data *ft6x06_ts = container_of(ft6x06_fw_config_delay,
	struct ft6x06_ts_data, ft6x06_fw_config_delay.work);

	int rc = 0;
    /*read FW version before upgrade FW*/
	unsigned char uc_reg_addr = FT6x06_REG_FW_VER;
	unsigned char vendor_id_addr = FT6X06_REG_VENDOR_READ;

    if(strncmp(ft6x06_ts->pdata->product_name,PHONE_NAME_Y550, sizeof(PHONE_NAME_Y550)) == 0)
    {
        CTPM_FW_JUN = CTPM_FW_JUN_Y550;
        fw_jun_len = sizeof(CTPM_FW_JUN_Y550);
        CTPM_FW_LANSI = CTPM_FW_LANSI_Y550;
        fw_lansi_len = sizeof(CTPM_FW_LANSI_Y550);
        tp_log_info("%s %d: select fw for Y550, fw_jun_len = %d, fw_lansi_len = %d\n",
                            __func__, __LINE__, fw_jun_len, fw_lansi_len);
    }
    else if(strncmp(ft6x06_ts->pdata->product_name, PHONE_NAME_ULC02, sizeof(PHONE_NAME_ULC02)) == 0)
    {
        CTPM_FW_JUN = CTPM_FW_JUN_ULC02;
        fw_jun_len = sizeof(CTPM_FW_JUN_ULC02);
        CTPM_FW_LANSI = CTPM_FW_LANSI_Y550;      //ULC02 do not use LANSI
        fw_lansi_len = sizeof(CTPM_FW_LANSI_Y550);
        tp_log_info("%s %d: select fw for ULC02, fw_jun_len = %d, fw_lansi_len = %d\n",
                            __func__, __LINE__, fw_jun_len, fw_lansi_len);
    }
    else
    {
        tp_log_err("%s %d: unkown product_name: %s\n", __func__, __LINE__, ft6x06_ts->pdata->product_name);
        return;
    }
 
    tp_log_err("in %s\n",__func__);
	/*read FW version from ic*/
	rc = ft6x06_i2c_Read(ft6x06_ts->client, &uc_reg_addr, 1, &ft6x06_ts->fw_ver_reg_value, 1);
	if(rc < 0)
	{
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
	}
	
	/* read Vendor ID from firmware */
	rc = ft6x06_i2c_Read(ft6x06_ts->client, &vendor_id_addr, 1, &ft6x06_ts->vendor_id_firmware, 1);
	if(rc < 0)
	{
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
	}
	else 
	{
		tp_log_info("ft6x06_ts->vendor_id_value = %d\n",ft6x06_ts->vendor_id_firmware);
	}
	/* read vendor id from flash */
	rc = ft6x36_get_factory_id(i2c_client, &ft6x06_ts->vendor_id_value);
	if (rc < 0) 
	{
		tp_log_err("%s:Get vendor id from flash fail, stop update, rc = %d\n", __func__, rc);    		    
		goto finish_update;
	}
	else 
	{
		tp_log_info("%s:Flash Vendor ID is: %d\n", __func__, ft6x06_ts->vendor_id_value);
	}

    /* load firmware data */
    if(ft6x06_ts->vendor_id_value == JUN_VENDOR_ID)
    {
        uc_host_fm_ver = fts_ctpm_get_i_file_ver(CTPM_FW_JUN,fw_jun_len);
        tp_log_info(" get vendor of junda\n");
    } 
    else if (ft6x06_ts->vendor_id_value == LANSI_VENDOR_ID) 
    {
        uc_host_fm_ver = fts_ctpm_get_i_file_ver(CTPM_FW_LANSI,fw_lansi_len);
        tp_log_info(" get vendor of lansi\n");
    } 
    else
    {
        tp_log_err(" error vendor id, id = %d\n", ft6x06_ts->vendor_id_value);
		goto finish_update;
    }
	
	tp_log_info("old version = 0x%x,new version =  0x%x\n",ft6x06_ts->fw_ver_reg_value,uc_host_fm_ver);
	//if vendor id in firmware not match vendor id in flash, update firmware
	if ( ft6x06_ts->vendor_id_value != ft6x06_ts->vendor_id_firmware || 
		ft6x06_ts->fw_ver_reg_value < uc_host_fm_ver ||		//the firmware in host flash is new, need upgrade
		ft6x06_ts->fw_ver_reg_value == FT6x06_REG_FW_VER 	//the firmware in touch panel maybe corrupted
	    ) 
	{
	    tp_log_info("starting ft6x06_upgrade_tp\n");
		ft6x06_ts->is_suspended = FT6X06_FLAG_WAKE_LOCK;
		
		ft6x06_upgrade_with_i_handler(ft6x06_ts);

		tp_log_info("ending ft6x06_upgrade_tp successful\n");

		//update firmware version
        msleep(2000);   //ensure that right fw_version can be read
		rc = ft6x06_i2c_Read(ft6x06_ts->client, &uc_reg_addr, 1, &ft6x06_ts->fw_ver_reg_value, 1);
		if(rc < 0)
			tp_log_err("error:%s,line=%d,update app_info fail,rc=%d\n", __func__, __LINE__,rc);
		else 
			tp_log_err("update app_info success, ft6x06_ts->uc_reg_value = %d\n",ft6x06_ts->fw_ver_reg_value);
	}
	else
	{
		tp_log_info("old version >= new version,can not upgrade\n");
	}

finish_update:
	ft6x06_set_app_info_touchpanel(ft6x06_ts);
	ft6x06_ts->is_suspended = FT6X06_FLAG_WAKE_UNLOCK;
	tp_log_debug("FT6X06_FLAG_WAKE_UNLOCK\n");
}
void delay_qt_ms(unsigned long  w_ms)
{
	unsigned long i;
	unsigned long j;

	for (i = 0; i < w_ms; i++)
	{
		for (j = 0; j < 1000; j++)
		{
			 udelay(1);
		}
	}
}

int fts_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf,
			  u32 dw_lenth)
{
	u8 reg_val[2] = {0};
	u32 i = 0;
	u32 packet_number;
	u32 j=0;
	u32 temp;
	u32 lenght;
	int rc = 0;
	u32 fw_length;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret;

   	tp_log_debug("%s %d:fts_ctpm_fw_upgrade\n", __func__, __LINE__);
	if(pbt_buf[0] != 0x02)
	{
		tp_log_err("[FTS] FW first byte is not 0x02. so it is invalid \n");
		return -1;
	}

	if(dw_lenth > 0x11f)
	{
       	tp_log_debug("%s %d:dw_lenth > 0x11f\n", __func__, __LINE__);
		fw_length = ((u32)pbt_buf[0x100]<<8) + pbt_buf[0x101];
		if(dw_lenth < fw_length)
		{
			tp_log_err("[FTS] Fw length is invalid \n");
			return -1;
		}
	}
	else
	{
        
		tp_log_err("[FTS] Fw length is invalid \n");
		return -1;
	}
	//DBG("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n", reg_val[0], reg_val[1]);
	
	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		/*********Step 1:Reset  CTPM *****/
		/*write 0xaa to register 0xbc */
		
		ft6x06_write_reg(client, 0xbc, FT_UPGRADE_AA);
		msleep(FT6X06_UPGRADE_AA_DELAY);

		/*write 0x55 to register 0xbc */
		ft6x06_write_reg(client, 0xbc, FT_UPGRADE_55);
	
		msleep(FT6X06_UPGRADE_55_DELAY+20);

		/*********Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = FT_UPGRADE_55;
		auc_i2c_write_buf[1] = FT_UPGRADE_AA;
		do {
			j++;
			i_ret = ft6x06_i2c_Write(client, auc_i2c_write_buf, 2);
			msleep(5);
		} while (i_ret <= 0 && j< 5);


		/*********Step 3:check READ-ID***********************/
		msleep(FT6X06_UPGRADE_READID_DELAY);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =
			0x00;
		reg_val[0] = 0x00;
		reg_val[1] = 0x00;
		rc = ft6x06_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);
		if(rc < 0)
			tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);

		if (reg_val[0] == FT6X06_UPGRADE_ID_1 && reg_val[1] == FT6X06_UPGRADE_ID_2) {
			tp_log_debug("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n", reg_val[0], reg_val[1]);
			break;
		} else {
			tp_log_err("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",	reg_val[0], reg_val[1]);
		}
	}
	tp_log_debug("%s %d\n",__FUNCTION__,__LINE__);
	if (i >= FTS_UPGRADE_LOOP)
	{
		tp_log_err("%s:Can not get into update model\n", __func__);
		return -EIO;
	}

	auc_i2c_write_buf[0] = 0x90;
	auc_i2c_write_buf[1] = 0x00;
	auc_i2c_write_buf[2] = 0x00;
	auc_i2c_write_buf[3] = 0x00;
	auc_i2c_write_buf[4] = 0x00;
	ft6x06_i2c_Write(client, auc_i2c_write_buf, 5);

	/*Step 4:erase app and panel paramenter area*/
	tp_log_debug("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = 0x61;
	ft6x06_i2c_Write(client, auc_i2c_write_buf, 1);	/*erase app area */
	msleep(FT6X06_UPGRADE_EARSE_DELAY);

	for(i = 0;i < 200;i++)
	{
		auc_i2c_write_buf[0] = 0x6a;
		auc_i2c_write_buf[1] = 0x00;
		auc_i2c_write_buf[2] = 0x00;
		auc_i2c_write_buf[3] = 0x00;
		reg_val[0] = 0x00;
		reg_val[1] = 0x00;
		ft6x06_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);
		if(0xb0 == reg_val[0] && 0x02 == reg_val[1])
		{
			tp_log_err("[FTS] erase app finished \n");
			break;
		}
		msleep(50);
	}

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	tp_log_debug("Step 5:write firmware(FW) to ctpm flash\n");

	dw_lenth = fw_length;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;

	for (j = 0; j < packet_number; j++) {
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;

		for (i = 0; i < FTS_PACKET_LENGTH; i++) {
			packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		
		ft6x06_i2c_Write(client, packet_buf, FTS_PACKET_LENGTH + 6);
		
		for(i = 0;i < 30;i++)
		{
			auc_i2c_write_buf[0] = 0x6a;
			auc_i2c_write_buf[1] = 0x00;
			auc_i2c_write_buf[2] = 0x00;
			auc_i2c_write_buf[3] = 0x00;
			reg_val[0] = 0x00;
			reg_val[1] = 0x00;
			ft6x06_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);
			if(0xb0 == (reg_val[0] & 0xf0) && (0x03 + (j % 0x0ffd)) == (((reg_val[0] & 0x0f) << 8) |reg_val[1]))
			{
				tp_log_err("[FTS] write a block data finished \n");
				break;
			}
			msleep(1);
		}
		
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}

		ft6x06_i2c_Write(client, packet_buf, temp + 6);

		for(i = 0;i < 30;i++)
		{
			auc_i2c_write_buf[0] = 0x6a;
			auc_i2c_write_buf[1] = 0x00;
			auc_i2c_write_buf[2] = 0x00;
			auc_i2c_write_buf[3] = 0x00;
			reg_val[0] = 0x00;
			reg_val[1] = 0x00;
			ft6x06_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);
			if(0xb0 == (reg_val[0] & 0xf0) && (0x03 + (j % 0x0ffd)) == (((reg_val[0] & 0x0f) << 8) |reg_val[1]))
			{
				/* delete it, too many log will be print */
				break;
			}
			msleep(1);
		}
		//msleep(20);
	}


	/*********Step 6: read out checksum***********************/
	/*send the opration head */
	tp_log_debug("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = 0xcc;
	rc = ft6x06_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if(rc < 0)
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
	if (reg_val[0] != bt_ecc) {
		tp_log_err("[FTS]--ecc error! FW=%02x bt_ecc=%02x\n", reg_val[0], bt_ecc);
		return -EIO;
	}

	/*********Step 7: reset the new FW***********************/
	tp_log_debug("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	ft6x06_i2c_Write(client, auc_i2c_write_buf, 1);
	msleep(300);	/*make sure CTP startup normally */

	return 0;
}

/*sysfs debug*/

/*
*get firmware size

@firmware_name:firmware name
*note:the firmware default path is sdcard.
	if you want to change the dir, please modify by yourself.
*/
static int ft6x06_GetFirmwareSize(char *firmware_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];
	memset(filepath, 0, sizeof(filepath));

        snprintf(filepath, sizeof(filepath), "/sdcard/%s", firmware_name);

	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);

	if (IS_ERR(pfile)) {
		tp_log_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}



/*
*read firmware buf for .bin file.

@firmware_name: fireware name
@firmware_buf: data buf of fireware

note:the firmware default path is sdcard.
	if you want to change the dir, please modify by yourself.
*/
static int ft6x06_ReadFirmware(char *firmware_name,
			       unsigned char *firmware_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));

        snprintf(filepath, sizeof(filepath), "/sdcard/%s", firmware_name);

        if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		tp_log_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, firmware_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);

	return 0;
}



/*
upgrade with *.bin file
*/

int fts_ctpm_fw_upgrade_with_app_file(struct i2c_client *client,
				       char *firmware_name)
{
	u8 *pbt_buf = NULL;
	int i_ret;
	int fwsize = ft6x06_GetFirmwareSize(firmware_name);

	if (fwsize <= 0) {
		tp_log_err("%s ERROR:Get firmware size failed\n", __func__);
		return -EIO;
	}

	if (fwsize < 8 || fwsize > 32 * 1024) {
		tp_log_err("%s:FW length error\n", __func__);
		return -EIO;
	}

	/*=========FW upgrade========================*/
	pbt_buf = kmalloc(fwsize + 1, GFP_ATOMIC);

	if (ft6x06_ReadFirmware(firmware_name, pbt_buf)) {
		tp_log_err("%s() - ERROR: request_firmware failed\n", __func__);
		kfree(pbt_buf);
		return -EIO;
	}
	
	/*call the upgrade function */
	i_ret = fts_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	if (i_ret != 0)
		tp_log_err("%s() - ERROR:[FTS] upgrade failed..\n", __func__);
	else
		fts_ctpm_auto_clb(client);
	kfree(pbt_buf);

	return i_ret;
}

#define FT5X0X_INI_FILEPATH "/system/etc/"
static int ft5x0x_GetInISize(char *config_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];
	memset(filepath, 0, sizeof(filepath));
    
        snprintf(filepath, sizeof(filepath), "%s%s", FT5X0X_INI_FILEPATH, config_name);

	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);

	if (IS_ERR(pfile)) {
		tp_log_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}

static int ft5x0x_ReadInIData(char *config_name,
			      char *config_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));

	snprintf(filepath, sizeof(filepath),  "%s%s", FT5X0X_INI_FILEPATH, config_name);

        if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		tp_log_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, config_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);

	return 0;
}
static int ft5x0x_get_testparam_from_ini(char *config_name)
{
	char *filedata = NULL;

	int inisize = ft5x0x_GetInISize(config_name);

	pr_info("inisize = %d \n ", inisize);
	if (inisize <= 0) {
		tp_log_err("%s ERROR:Get firmware size failed\n", __func__);
		return -EIO;
	}

	filedata = kmalloc(inisize + 1, GFP_ATOMIC);
		
	if (ft5x0x_ReadInIData(config_name, filedata)) {        
		tp_log_err("%s() - ERROR: request_firmware failed\n", __func__);
		kfree(filedata);
		return -EIO;
	} else {
		tp_log_info("ft5x0x_ReadInIData successful\n");
	}

	SetParamData(filedata);
	return 0;
}


//#endif
int focal_i2c_Read(unsigned char *writebuf,
		    int writelen, unsigned char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = g_focalclient->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = g_focalclient->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(g_focalclient->adapter, msgs, 2);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = g_focalclient->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(g_focalclient->adapter, msgs, 1);
	}

        if(ret <= 0)
        {
            tp_log_err("%s %d: i2c_transfer return: %d\n", __func__, __LINE__,ret);

            ft6x06_report_dsm_erro(g_ft6x06_ts, tp_dclient, DSM_TP_I2C_RW_ERROR_NO, ret);

            return -1;
        }

        return ret;
}
/*write data by i2c*/
int focal_i2c_Write(unsigned char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = g_focalclient->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(g_focalclient->adapter, msg, 1);

        if(ret <= 0)
        {
            tp_log_err("%s %d: i2c_transfer return: %d\n", __func__, __LINE__,ret);

            ft6x06_report_dsm_erro(g_ft6x06_ts, tp_dclient, DSM_TP_I2C_RW_ERROR_NO, ret);

            return -1;
        }
        
	return ret;
}
//#endif
static ssize_t ft5x0x_ftsscaptest_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t ret_count = 0;
	char cfgname[128];
	unsigned char tp_factory_id = 0xFF;
	memset(cfgname, 0, sizeof(cfgname));
    
	g_focalclient = i2c_client;
	mutex_lock(&g_device_mutex);

	Init_I2C_Write_Func(focal_i2c_Write);
	Init_I2C_Read_Func(focal_i2c_Read);

	//»ñÈ¡ÆÁ³§ID
	if(ft6x06_read_reg(i2c_client, 0xA8, &tp_factory_id) < 0) {
        
        /*--coverity--sprintf-->snprintf---*/
        ret_count = snprintf(buf, PAGE_SIZE, "[Focal] Get TP factory id failure\n");
		goto FOCAL_SCAP_TEST_ERR;
	} else {
        snprintf(cfgname, sizeof(cfgname), "focal_scap_test_0x%02x_%s.ini", 
                                   tp_factory_id, g_ft6x06_ts->pdata->product_name);
        tp_log_info("%s %d: cfgname: %s\n", __func__, __LINE__, cfgname);
	}
	
	if(ft5x0x_get_testparam_from_ini(cfgname) <0)
		ret_count = snprintf(buf, PAGE_SIZE, "get testparam from ini failure\n");
	else {
		g_focalscaptested = 1;
		if(true == StartTestTP())
			ret_count = snprintf(buf, PAGE_SIZE, "PASS\n");
		else
			ret_count = snprintf(buf, PAGE_SIZE, "FAIL\n");

		FreeTestParamData();
	}
FOCAL_SCAP_TEST_ERR:	
	mutex_unlock(&g_device_mutex);

	return ret_count;
}

unsigned int ft6x36_get_factory_id(struct i2c_client * client, unsigned char *pProjectCode) 
{
	u8 reg_val[2] = {0};
	u32 i = 0;
	u32 j=0;
	u32 temp;
	u8 packet_buf[4];
	u8 auc_i2c_write_buf[10];
	int i_ret;
	int rc =0;
	
	for (i = 0; i < FTS_UPGRADE_LOOP; i++) 
	{
		auc_i2c_write_buf[0] = 0xbc;
		auc_i2c_write_buf[1] = FT_UPGRADE_AA;
		ft6x06_i2c_Write(client, auc_i2c_write_buf, 2);

		msleep(FT6X06_UPGRADE_AA_DELAY);
		auc_i2c_write_buf[0] = 0xbc;
		auc_i2c_write_buf[1] = FT_UPGRADE_55;
		ft6x06_i2c_Write(client, auc_i2c_write_buf, 2);

		msleep(FT6X06_UPGRADE_55_DELAY+20);   
		auc_i2c_write_buf[0] = FT_UPGRADE_55;
		auc_i2c_write_buf[1] = FT_UPGRADE_AA;

		do{
			j++;
			i_ret = ft6x06_i2c_Write(client, auc_i2c_write_buf, 2);
		msleep(5);
		}while(i_ret <= 0 && j<5);

		msleep(FT6X06_UPGRADE_READID_DELAY);
		auc_i2c_write_buf[0] = 0x90; 
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;
		rc = ft6x06_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);
		if(rc < 0)
			tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
		if (reg_val[0] == FT6X06_UPGRADE_ID_1 
			&& reg_val[1] == FT6X06_UPGRADE_ID_2)
		{
			tp_log_debug("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
			break;
		}
		else
		{
			tp_log_err("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
			continue;
		}
	}
	if (i >= FTS_UPGRADE_LOOP)
	{
		tp_log_err("Can not change model to update\n");
		return -EIO;
	}

	auc_i2c_write_buf[0] = 0xcd;
	rc = ft6x06_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if(rc < 0)
	{
		tp_log_err("error:%s,line=%d,rc=%d\n", __func__, __LINE__,rc);
	}

	tp_log_debug("bootloader version:%d\n", reg_val[0]);
	packet_buf[0] = 0x03;
	packet_buf[1] = 0x00;
	temp = FT_VENDOR_ID_REG;
	packet_buf[2] = (u8)(temp>>8);
	packet_buf[3] = (u8)temp;
	ft6x06_i2c_Read(client, packet_buf, sizeof(packet_buf), pProjectCode, 1);

	auc_i2c_write_buf[0] = 0x07;
	ft6x06_i2c_Write(client, auc_i2c_write_buf, 1);
	msleep(300);  /*make sure CTP startup normally*/
	tp_log_info("Panel_Vendor_ID = 0x%x \n", pProjectCode[0]);
	return 0;
}


static ssize_t ft5x0x_ftsscaptest_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	/*place holder for future use*/
	return -EPERM;
}

static ssize_t ft5x0x_ftsscapsample_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t ret_count = 0;
	char* databuf=NULL;
	unsigned int databuf_len = 2048;
	tp_log_info("enter %s %d\n", __func__, __LINE__);
	databuf = kmalloc(databuf_len, GFP_ATOMIC);
	if (NULL == databuf) 
	{

            /*--coverity--spintf-->snprintf--*/
            ret_count = snprintf(buf, PAGE_SIZE, "Can not malloc memory.\n");

            return ret_count;
	}

	memset(databuf, 0, databuf_len);
	mutex_lock(&g_device_mutex);

	if(1 == g_focalscaptested) {
		ret_count = focal_save_scap_sample(databuf, 2048);
		//tp_log_err("[TP]length = %d\n", ret_count);
		memcpy(buf, databuf, ret_count);
	} else {

            /*--coverity--spintf-->snprintf--*/
            ret_count = snprintf(buf, PAGE_SIZE, "please tested at first.\n");
                
	}
	
	mutex_unlock(&g_device_mutex);
	kfree(databuf);
	tp_log_info("exit %s %d\n", __func__, __LINE__);
	return ret_count;
}


static ssize_t ft5x0x_ftsscapsample_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	/*place holder for future use*/
	return -EPERM;
}
static DEVICE_ATTR(scap_test, S_IRUGO|S_IWUSR, ft5x0x_ftsscaptest_show, ft5x0x_ftsscaptest_store);
static DEVICE_ATTR(scap_test_sample, S_IRUGO|S_IWUSR, ft5x0x_ftsscapsample_show, ft5x0x_ftsscapsample_store);

static struct attribute *ft6x06_attributes[] = {
	&dev_attr_scap_test.attr,
	&dev_attr_scap_test_sample.attr,
	NULL
};

static struct attribute_group ft6x06_attribute_group = {
	.attrs = ft6x06_attributes
};

/*create sysfs for debug*/
static struct kobject *touch_screen_kobject_ts = NULL;
int ft6x06_create_sscap_sysfs(struct i2c_client *client)
{
	int err;
	tp_log_info(" in ft6x06_create_sysfs\n");
    if( NULL == touch_screen_kobject_ts )
	{
		touch_screen_kobject_ts = kobject_create_and_add("touch_screen", NULL);
		if (!touch_screen_kobject_ts)
		{
			err=-EIO;
			return err;
		}
	}
	
	err = sysfs_create_group(touch_screen_kobject_ts, &ft6x06_attribute_group);
	if (0 != err) {
		tp_log_err("%s() - ERROR: sysfs_create_group() failed.\n", __func__);
		sysfs_remove_group(&client->dev.kobj, &ft6x06_attribute_group);
		return -EIO;
	} else {
		mutex_init(&g_device_mutex);
		tp_log_err("ft6x06:%s() - sysfs_create_group() succeeded.\n", __func__);
	}
	tp_log_info(" out ft6x06_create_sysfs\n");
	return err;
}

void ft6x06_release_sscap_sysfs(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &ft6x06_attribute_group);
	mutex_destroy(&g_device_mutex);
}
