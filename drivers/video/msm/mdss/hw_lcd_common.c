/* add cmi lcd driver */
/* Copyright (c) 2009, Code HUAWEI. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>

#include<linux/init.h>
#include<linux/module.h>

#include <linux/hw_lcd_common.h>
#include <mdss_dsi.h>
#include <linux/of.h>
#ifdef CONFIG_HUAWEI_DSM
#include "mdss_panel.h"
#include <linux/sched.h>
struct dsm_client *lcd_dclient = NULL;
#endif

#ifdef CONFIG_DEBUG_FS
/* whether print log or not */
#define LCD_MIPI_DEBUG_ON    (1)

static struct mdss_dsi_ctrl_pdata *g_lcd_dbg_dsi_ctrl_pdata = NULL;    // global mdss_dsi_ctrl_pdata

/* set global ctrl_pdata pointer */
void lcd_dbg_set_dsi_ctrl_pdata(struct mdss_dsi_ctrl_pdata *ctrl)
{
	static char already_set = 0;

	/* judge if already set or not*/
	if (already_set)
	{
		LCD_LOG_ERR("%s: already set\n", __func__);
	}
	else
	{
		g_lcd_dbg_dsi_ctrl_pdata = ctrl;
		already_set = 1;
	}

	return;
}

/* get global ctrl_pdata pointer */
struct mdss_dsi_ctrl_pdata *lcd_dbg_get_dsi_ctrl_pdata(void)
{
	return g_lcd_dbg_dsi_ctrl_pdata;
}

/* check whether mipi input is legal or not */
/* return: 0 - success, negative - fail */
static int is_mipi_input_legal(int op_type,int ic_reg, int cmd_type, int param_num,char *buf)
{
	int ret = 0;
	if( (op_type != OPER_READ) && (op_type != OPER_WRITE))
	{
		ret = -1;
	}
	/* ic_reg must in [0x00, 0xff] */
	if (!((unsigned int)ic_reg >= 0 && (unsigned int)ic_reg <= 0xff))
	{
		ret = -1;
	}

	/* cmd_type must be 0x01 or 0x04 */
	if ((cmd_type != MIPI_DCS_COMMAND) &&(cmd_type != MIPI_GEN_COMMAND))
	{
		ret = -1;
	}
	/* param_num must larger or equal 0 */
	if (param_num < 0)
	{
		ret = -1;
	}

	if(NULL == buf)
	{
		ret = -1;
	}
	return ret;
}


/**********************************************************************************
*function:process read and write commands  for lcd reg debug
*op_type:	read or write
*reg:		lcd register
*cmd_type:	DCS or GEN
*param_num:	the count of prameters to tranfer
*param_buf:	prameters
*read_value:  value from lcd panel
*delay_ms:	delay time
*return: 0 - success, negative - fail
**********************************************************************************/
int lcd_dbg_mipi_prcess_ic_reg(int op_type,int reg, int cmd_type, int param_num, char *param_buf,int *read_value, int delay_ms)
{
	static struct dcs_cmd_req cmdreq;
	static struct dsi_cmd_desc dsi_cmd;                  // dsi cmd struct
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
#if LCD_MIPI_DEBUG_ON
	int i = 0;
#endif

	/* check if input legal */
	if (is_mipi_input_legal(op_type,reg, cmd_type, param_num,param_buf))
	{
		LCD_LOG_ERR("%s, input illegal.\n", __func__);
		return -1;
	}

	ctrl = lcd_dbg_get_dsi_ctrl_pdata();
	/* translate cmd_type from huawei to qcom's format */
	switch (param_num)
	{
		case 0:
		{
			if(OPER_READ == op_type)
			{
				/* DCS MODE */
				if (MIPI_DCS_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_DCS_READ;
				}
				/* GEN MODE */
				else if (MIPI_GEN_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_GEN_READ;
				}
			}
			else
			{
				/* DCS MODE */
				if (MIPI_DCS_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_DCS_WRITE;
				}
				/* GEN MODE */
				else if (MIPI_GEN_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_GEN_WRITE1;
				}
			}
			break;
		}

		case 1:
		{
			if(OPER_READ == op_type)
			{
				/* DCS MODE */
				if (MIPI_DCS_COMMAND == cmd_type)
				{
					LCD_LOG_ERR("%s ,not support this kind of dcs read! \n",__func__);
					return -1;
				}
				/* GEN MODE */
				else if (MIPI_GEN_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_GEN_READ1;
				}
			}
			else
			{
				/* DCS MODE */
				if (MIPI_DCS_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_DCS_WRITE1;
				}
				/* GEN MODE */
				else if (MIPI_GEN_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_GEN_WRITE2;
				}
			}
			break;
		}

		default:
		{
			if(OPER_READ == op_type)
			{
				/* DCS MODE */
				if (MIPI_DCS_COMMAND == cmd_type)
				{
					LCD_LOG_ERR("%s ,not support this kind of dcs read! \n",__func__);
					return -1;
				}
				/* GEN MODE */
				else if (MIPI_GEN_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_GEN_READ2;
				}
			}
			else
			{
				/* DCS MODE */
				if (MIPI_DCS_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_DCS_LWRITE;
				}
				/* GEN MODE */
				else if (MIPI_GEN_COMMAND == cmd_type)
				{
					cmd_type = DTYPE_GEN_LWRITE;
				}
			}
			break;
		}
	}

	/* insert reg into param_buf's beginning */
	memmove(param_buf + 1, param_buf, param_num);
	param_buf[0] = reg;
	param_num++;

	/* debug begin */
#if LCD_MIPI_DEBUG_ON
	LCD_LOG_INFO("%s,op_type=%d, reg=0x%02x, cmd_type=0x%02x, param_num=0x%02x, delay_ms=0x%02x\n", __func__, op_type,reg, cmd_type, param_num, delay_ms);
	LCD_LOG_INFO("%s, print param_buf begin\n", __func__);
	for (i = 0; i < param_num; i++)
	{
		LCD_LOG_INFO("0x%02x ", param_buf[i]);
	}
	LCD_LOG_INFO("%s, print param_buf end\n", __func__);
#endif
	/* debug end */

	dsi_cmd.dchdr.dtype = cmd_type;
	dsi_cmd.dchdr.last =1;
	dsi_cmd.dchdr.vc = 0;
	dsi_cmd.dchdr.dlen = param_num;
	dsi_cmd.payload = param_buf;
	memset(&cmdreq, 0, sizeof(cmdreq));
	switch(op_type)
	{
		case OPER_READ:
			dsi_cmd.dchdr.ack = 1;
			dsi_cmd.dchdr.wait = 5;//5 ms
			cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
			cmdreq.rbuf = (char*)read_value;
			break;
		case OPER_WRITE:
			dsi_cmd.dchdr.ack = 0;
			dsi_cmd.dchdr.wait = delay_ms;//5 ms
			cmdreq.flags = CMD_REQ_COMMIT;
			cmdreq.rbuf = NULL;
			break;
	}
	cmdreq.cmds = &dsi_cmd;
	cmdreq.cmds_cnt = 1;
	if (ctrl->long_read_flag)
	{
		cmdreq.rlen = 3;
	}else
	{
		cmdreq.rlen = 1;
	}

	cmdreq.cb = NULL; /* call back */
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
	if( op_type ==OPER_READ )
	{
		LCD_LOG_INFO("%s, read value is 0x%02x\n", __func__,cmdreq.rbuf[0]);
	}

	return 0;
}
#endif
#ifdef CONFIG_HUAWEI_DSM
int mdss_record_dsm_err(u32 *dsi_status)
{
	if( NULL == lcd_dclient )
	{
		LCD_LOG_ERR("%s: there is no lcd_dclient!\n", __func__);
		return -1;
	}

	/* try to get permission to use the buffer */
	if(dsm_client_ocuppy(lcd_dclient))
	{
		/* buffer is busy */
		LCD_LOG_ERR("%s: buffer is busy!\n", __func__);
		return -1;
	}

	LCD_LOG_INFO("%s: entry!\n", __func__);
	if (dsi_status[0])
		dsm_client_record(lcd_dclient, "DSI_ACK_ERR_STATUS is wrong ,err number :%x\n", dsi_status[0]);

	if (dsi_status[1] & 0x0111)
		dsm_client_record(lcd_dclient, "DSI_TIMEOUT_STATUS is wrong ,err number :%x\n", dsi_status[1]);

	if (dsi_status[2] & 0x011111)
		dsm_client_record(lcd_dclient, "DSI_DLN0_PHY_ERR is wrong ,err number :%x\n", dsi_status[2]);
	//Disable check reg 00c because the register can not show dsi status accurately
	if (dsi_status[3] & 0xcccc4489)
		return 0;
	if (dsi_status[4] & 0x80000000)
		dsm_client_record(lcd_dclient, "DSI_STATUS is wrong ,err number :%x\n", dsi_status[4]);

	dsm_client_notify(lcd_dclient, DSM_LCD_MDSS_DSI_ISR_ERROR_NO);

	return 0;
}
/* remove APR web LCD report log information  */
int lcd_report_dsm_err(int type, int err_value,int add_value)
{


	if ((DSM_LCD_MIPI_ERROR_NO == type && 0x51 == add_value)|| DSM_LCD_MDSS_DSI_ISR_ERROR_NO == type)
	{
		return 0;
	}

	LCD_LOG_INFO("%s: entry! type:%d\n", __func__, type);


	if( NULL == lcd_dclient )
	{
		LCD_LOG_ERR("%s: there is not lcd_dclient!\n", __func__);
		return -1;
	}

	/* try to get permission to use the buffer */
	if(dsm_client_ocuppy(lcd_dclient))
	{
		/* buffer is busy */
		LCD_LOG_ERR("%s: buffer is busy!\n", __func__);
		return -1;
	}

	/* lcd report err according to err type */
	switch(type)
	{
		case DSM_LCD_STATUS_ERROR_NO:
			dsm_client_record(lcd_dclient, "lcd register %x status wrong, value :%x\n",add_value, err_value);
			break;
		case DSM_LCD_MIPI_ERROR_NO:
			dsm_client_record(lcd_dclient, "mipi transmit register %x time out ,err number :%x\n", add_value, err_value );
			break;
		/* add for lcd esd */
		case DSM_LCD_ESD_STATUS_ERROR_NO:
			dsm_client_record(lcd_dclient, "LCD STATUS ERROR %x read data = %x\n", add_value, err_value );
			break;
		case DSM_LCD_ESD_REBOOT_ERROR_NO:
			dsm_client_record(lcd_dclient, "LCD RESET register %x read data =%x\n", add_value, err_value );
			break;
		case DSM_LCD_MDSS_IOMMU_ERROR_NO:
			dsm_client_record(lcd_dclient, "mdss iommu attach/detach or map memory fail (%d)\n", err_value);
			break;
		case DSM_LCD_MDSS_PIPE_ERROR_NO:
			dsm_client_record(lcd_dclient, "mdss pipe status error (%d)\n",err_value);
			break;
		case DSM_LCD_MDSS_PINGPONG_ERROR_NO:
			dsm_client_record(lcd_dclient, "mdss wait pingpong time out (%d)\n",err_value);
			break;
		case DSM_LCD_MDSS_BIAS_ERROR_NO:
			dsm_client_record(lcd_dclient, "get vsp/vsn(%d) register fail (%d) \n", add_value, err_value);
			break;
		case DSM_LCD_MDSS_ROTATOR_ERROR_NO:
			dsm_client_record(lcd_dclient, "mdss rotator queue fail (%d) \n",err_value);
			break;
		case DSM_LCD_MDSS_FENCE_ERROR_NO:
			dsm_client_record(lcd_dclient, "mdss sync_fence_wait fail (%d) \n", err_value);
			break;
		case DSM_LCD_MDSS_CMD_STOP_ERROR_NO:
			dsm_client_record(lcd_dclient, "mdss stop cmd time out (%d) \n", err_value);
			break;
		case DSM_LCD_MDSS_VIDEO_DISPLAY_ERROR_NO:
			dsm_client_record(lcd_dclient, "mdss commit without wait! ctl=%d", err_value);
			break;
		case DSM_LCD_MDSS_MDP_CLK_ERROR_NO:
			dsm_client_record(lcd_dclient, "mdss mdp clk can't be turned off\n", err_value);
			break;
		case DSM_LCD_MDSS_MDP_BUSY_ERROR_NO:
			dsm_client_record(lcd_dclient, "mdss mdp dma tx time out (%d)\n", err_value);
			break;
		default:
			break;
	}

	dsm_client_notify(lcd_dclient, type);

	return 0;
}

/* remove APR web LCD report log information  */

/*
*
*bit 0  do unblank
*bit 1  lcd on
*bit 2  set frame
*bit 3  set backlgiht
*/
/*if did the operation the bit will be set to 1 or the bit is 0*/
void lcd_dcm_pwr_status_handler(unsigned long data)
{
	if(lcd_pwr_status.lcd_dcm_pwr_status != LCD_PWR_STAT_GOOD)
	{
		show_state_filter(TASK_UNINTERRUPTIBLE);
		dsm_client_record(lcd_dclient, "lcd power status wrong, value :%x\n",lcd_pwr_status.lcd_dcm_pwr_status);
		dsm_client_record(lcd_dclient, "lcd power status :bit 0  do unblank\n");
		dsm_client_record(lcd_dclient, "lcd power status :bit 1  lcd on\n");
		dsm_client_record(lcd_dclient, "lcd power status :bit 2  set frame\n");
		dsm_client_record(lcd_dclient, "lcd power status :bit 3  set backlgiht\n");
		dsm_client_record(lcd_dclient, "lcd power status :if did the operation the bit will be set to 1 or the bit is 0\n");
		dsm_client_record(lcd_dclient,"unblank at [%d-%d-%d]%d:%d:%d:%d\n",lcd_pwr_status.tm_unblank.tm_year + 1900,lcd_pwr_status.tm_unblank.tm_mon+1,
						lcd_pwr_status.tm_unblank.tm_mday,lcd_pwr_status.tm_unblank.tm_hour,lcd_pwr_status.tm_unblank.tm_min,lcd_pwr_status.tm_unblank.tm_sec,lcd_pwr_status.tvl_unblank.tv_usec%1000);
		dsm_client_record(lcd_dclient,"lcd on at [%d-%d-%d]%d:%d:%d:%d\n",lcd_pwr_status.tm_lcd_on.tm_year + 1900,lcd_pwr_status.tm_lcd_on.tm_mon+1,
						lcd_pwr_status.tm_lcd_on.tm_mday,lcd_pwr_status.tm_lcd_on.tm_hour,lcd_pwr_status.tm_lcd_on.tm_min,lcd_pwr_status.tm_lcd_on.tm_sec,lcd_pwr_status.tvl_lcd_on.tv_usec%1000);
		dsm_client_record(lcd_dclient,"set frame at [%d-%d-%d]%d:%d:%d:%d\n",lcd_pwr_status.tm_set_frame.tm_year + 1900,lcd_pwr_status.tm_set_frame.tm_mon+1,
						lcd_pwr_status.tm_set_frame.tm_mday,lcd_pwr_status.tm_set_frame.tm_hour,lcd_pwr_status.tm_set_frame.tm_min,lcd_pwr_status.tm_set_frame.tm_sec,lcd_pwr_status.tvl_set_frame.tv_usec%1000);
		dsm_client_record(lcd_dclient,"set backlight at [%d-%d-%d]%d:%d:%d:%d\n",lcd_pwr_status.tm_backlight.tm_year + 1900,lcd_pwr_status.tm_backlight.tm_mon+1,
						lcd_pwr_status.tm_backlight.tm_mday,lcd_pwr_status.tm_backlight.tm_hour,lcd_pwr_status.tm_backlight.tm_min,lcd_pwr_status.tm_backlight.tm_sec,lcd_pwr_status.tvl_backlight.tv_usec%1000);
		dsm_client_notify(lcd_dclient, DSM_LCD_POWER_STATUS_ERROR_NO);
	}
}

/* remove APR web LCD report log information  */
void mdp_underrun_dsm_report(unsigned long num,unsigned long underrun_cnt)
{
	/* try to get permission to use the buffer */
	if(dsm_client_ocuppy(lcd_dclient))
	{
		/* buffer is busy */
		LCD_LOG_ERR("%s: buffer is busy!\n", __func__);
		return;
	}
	dsm_client_record(lcd_dclient, "Lcd underrun detected for ctl=%d,count=%d\n",num,underrun_cnt);
	dsm_client_notify(lcd_dclient, DSM_LCD_MDSS_UNDERRUN_ERROR_NO);
}
#endif

