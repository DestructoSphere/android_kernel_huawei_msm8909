/* Copyright (c) 2009-2013, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/debugfs.h>
#include <linux/ctype.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/unistd.h>
#include <linux/hw_lcd_common.h>

/* define macro */
#define DCS_STR ("dcs_")
#define GEN_STR ("gen_")

#define READ_STR ("read_")
#define WRITE_STR ("write_")

#define MAX_PARAM_NUM (25)

#define LCD_DEBUG_BUF	(1024)
#define LCD_PARAM_BUF	(256)

/* internal buffer */
static char lcd_debug_buf[LCD_DEBUG_BUF];
static char lcd_param_buf[LCD_PARAM_BUF];

static int g_ic_mipi_reg = 0;    // read register
static int g_ic_mipi_value = 0;  // read value
atomic_t mipi_path_status = ATOMIC_INIT(1);
/*
 * reg_dbg
 *
 */
/* open function */
static int lcd_reg_dbg_mipi_open(struct inode *inode, struct file *file)
{
	/* non-seekable */
	file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
	return 0;
}

/* release function */
static int lcd_reg_dbg_mipi_release(struct inode *inode, struct file *file)
{
	return 0;
}

/* read function */
/* show usage or print last read result */
static ssize_t lcd_reg_dbg_mipi_read(struct file *file,	char __user *buff,size_t count,loff_t *ppos)
{
	int len = 0;
	int ret_len = 0;
	char *cur = lcd_debug_buf;
	int buf_len = sizeof(lcd_debug_buf);
	if (*ppos)
		return 0;	
	/* show usage */
	if (!g_ic_mipi_reg )
	{
		len = snprintf(cur, buf_len, "Usage: echo \"<[Dcs|Gen]>_<[write|read]>_1A_<[0..25]>P(reg, param0..paramN)_delayms\" > reg_dbg_mipi\n");
		buf_len -= len;
		cur += len;

		len = snprintf(cur, buf_len, "Read or write lcd ic register using mipi interface.\n");
		buf_len -= len;
		cur += len;

		len = snprintf(cur, buf_len, "\teg. echo \"Dcs_read_1A_0P(0x52)_0\" > reg_dbg_mipi\n");
		buf_len -= len;
		cur += len;

		len = snprintf(cur, buf_len, "\teg. echo \"Dcs_write_1A_1P(0x51,0x00)_0\" > reg_dbg_mipi\n");
		buf_len -= len;
		cur += len;

		len = snprintf(cur, buf_len, "\teg. echo \"Gen_write_1A_3P(0xFF,0x80,0x09,0x01)_0\" > reg_dbg_mipi\n");
		buf_len -= len;
		cur += len;

		len = snprintf(cur, buf_len, "\teg. echo \"Dcs_write_1A_0P(0x28)_0x14\" > reg_dbg_mipi\n");
		buf_len -= len;
		cur += len;

		len = snprintf(cur, buf_len, "\tFor double bytes command ,to write offset first then the other byte like following\n");
		buf_len -= len;
		cur += len;

		len = snprintf(cur, buf_len, "\tto read 0xC680\n");
		buf_len -= len;
		cur += len;

		len = snprintf(cur, buf_len, "\teg. echo \"Dcs_write_1A_1P(0x00,0x80)_0\" > reg_dbg_mipi\n");
		buf_len -= len;
		cur += len;

		len = snprintf(cur, buf_len, "\teg. echo \"Dcs_read_1A_0P(0xC6)_0\" > reg_dbg_mipi\n");
		buf_len -= len;
		cur += len;

		len = snprintf(cur, buf_len, "\tthen cat reg_dbg_mipi.\n");
		buf_len -= len;
		cur += len;

		ret_len = sizeof(lcd_debug_buf) - buf_len;
		LCD_LOG_INFO("%s, show usage, ret_len = %d\n", __func__, ret_len);    // debug
	}
	/* show last read result */
	else
	{
		if(!atomic_read(&mipi_path_status))
		{
			LCD_LOG_ERR("%s, the panel has been closed, please open it firstly.\n", __func__);
			ret_len = snprintf(lcd_debug_buf, sizeof(lcd_debug_buf), "0x%02x = -1\n", g_ic_mipi_reg);
		}
		else
		{
			ret_len = snprintf(lcd_debug_buf, sizeof(lcd_debug_buf), "0x%02x = 0x%02x\n", g_ic_mipi_reg, g_ic_mipi_value);
		}
	}

	/* some error happen */
	if (ret_len < 0)
		return 0;

	/* copy to user */
	if (copy_to_user(buff, lcd_debug_buf, ret_len))
		return -EFAULT;

	*ppos += ret_len;	// increase offset
	return ret_len;
}

/* convert string to lower case */
/* return: 0 - success, negative - fail */
static int str_to_lower(char *str)
{
	char *tmp = str;

	/* check param */
	if (NULL == tmp)
	{
		return -1;
	}

	while (*tmp != '\0')
	{
		*tmp = tolower(*tmp);
		tmp++;
	}

	return 0;
}

/* check if string start with sub string */
/* return: 0 - success, negative - fail */
static int str_start_with(char *str, char *sub)
{
	/* check param */
	if (NULL == str || NULL == sub)
	{
		return -1;
	}

	return (0 == strncmp(str, sub, strlen(sub)) ? 0: -1);
}

/* get reg, param and delay */
/* return: 0 - success, negative - fail */
static int lcd_dbg_mipi_get_reg_param_delay(char *buf, int param_num, int *reg, char *param_buf, int *delay_ms)
{
	char *cur = buf;
	char *temp = NULL;
	char *write_pos = param_buf;
	int cnt = 0;
	int param = 0;
	int i = 0;

	/* get reg */
	/* input format like: 1a_1p(0x51,0x00)_0 or 1a_0p(0x11)_0x78 */
	temp = strchr(cur, '(');
	if (NULL == temp)
	{
		goto err_handle;
	}
	cur = temp;
	cnt = sscanf(cur, "(%x", reg);
	if (cnt != 1)
	{
		goto err_handle;
	}
	else
	{
		/* when param_num != 0, get ',' position */
		if (param_num != 0)
		{
			temp = strchr(cur, ',');
			if (NULL == temp)
			{
				goto err_handle;
			}
			cur = temp;
		}
	}

	/* loop to get param */
	for (i = 0; i < param_num; i++)
	{
		/* get a param */
		cnt = sscanf(cur, ",%x", &param);
		if (cnt != 1)
		{
			goto err_handle;
		}
		else
		{
			/* save param to param_buf */
			*write_pos = param;
			write_pos++;

			/* if current is not the last param */
			if (i != param_num - 1)
			{
				cur++;  // ignore ',' which already handled
				temp = strchr(cur, ',');
				if (NULL == temp)
				{
					goto err_handle;
				}
				cur = temp;  // move to next param
			}
		}
	}

	/* get delay */
	temp = strchr(cur, ')');
	if (NULL == temp)
	{
		goto err_handle;
	}
	cur = temp;

	cnt = sscanf(cur, ")_%x,", delay_ms);
	if (cnt != 1)
	{
		goto err_handle;
	}

	return 0;

err_handle:
	LCD_LOG_ERR("%s, input illegal\n", __func__);
	return -1;
}

/* write function */
/* handle read or write ic command */
static ssize_t lcd_reg_dbg_mipi_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
	char *cur = lcd_debug_buf;
	int ret = 0;
	int cmd_type = 0;
	int op_type = 0;
	int cnt = 0;
	int reg_num = 0;
	int param_num = 0;
	int reg = 0;
	int delay_ms = 0;
	if(!atomic_read(&mipi_path_status))
	{
		LCD_LOG_ERR("%s, the panel is closed, please open it firstly.\n", __func__);
		return -EFAULT;
	}
	if (count >= sizeof(lcd_debug_buf))
	{
		LCD_LOG_ERR(" %s,input overflow \n",__func__);
		return -EFAULT;
	}

	if (copy_from_user(lcd_debug_buf, buff, count))
		return -EFAULT;

	lcd_debug_buf[count] = 0;	/* end of string */

	/* convert to lower case */
	if (0 != str_to_lower(cur))
	{
		goto err_handle;
	}

	/* get cmd type */
	/* input format like: dcs_write_1a_1p(0x51,0x00)_0 or dcs_write_1a_0p(0x11)_0x78 */
	if (0 == str_start_with(cur, DCS_STR))
	{
		cmd_type = MIPI_DCS_COMMAND;
		cur += strlen(DCS_STR);  // move cur behind
	}
	else if (0 == str_start_with(cur, GEN_STR))
	{
		cmd_type = MIPI_GEN_COMMAND;
		cur += strlen(GEN_STR);  // move cur behind
	}
	else
	{
		LCD_LOG_ERR("%s, cmd type not support!\n", __func__);  // not support
		goto err_handle;
	}
	//pr_info("========%s, cmd_type = %d.\n", __func__, cmd_type);

	/* get op type */
	/* input format like: write_1a_1p(0x51,0x00)_0 or write_1a_0p(0x11)_0x78 */
	if (0 == str_start_with(cur, READ_STR))
	{
		op_type = OPER_READ;
		cur += strlen(READ_STR);  // move cur behind
	}
	else if (0 == str_start_with(cur, WRITE_STR))
	{
		op_type = OPER_WRITE;
		cur += strlen(WRITE_STR);  // move cur behind
	}
	else
	{
		LCD_LOG_ERR("%s, op type not support!\n", __func__);  // not support
		goto err_handle;
	}
	//pr_info("========%s, op_type = %d.\n", __func__, op_type);

	/* get reg_num, param_num */
	/* input format like: 1a_1p(0x51,0x00)_0 or 1a_0p(0x11)_0x78 */
	cnt = sscanf(cur,"%da_%dp(", &reg_num, &param_num);
	if (cnt != 2)
	{
		LCD_LOG_ERR("%s, get reg_num, param_num fail!\n", __func__);
		goto err_handle;
	}
	else
	{
		/* check input */
		if (reg_num != 1 || param_num < 0 || param_num > MAX_PARAM_NUM)
		{
			LCD_LOG_ERR("%s, input illegal, reg_num = %d, param_num = %d\n", __func__, reg_num, param_num);  // not support
			goto err_handle;
		}
	}

	/* get reg, param and delay */
	/* input format like: 1a_1p(0x51,0x00)_0 or 1a_0p(0x11)_0x78 */
	ret = lcd_dbg_mipi_get_reg_param_delay(cur, param_num, &reg, lcd_param_buf, &delay_ms);
	if (ret)
	{
		LCD_LOG_ERR("%s, lcd_dbg_mipi_get_reg_param_delay fail!\n", __func__);
		goto err_handle;
	}

	/* handle read or write ic command */
	switch (op_type)
	{
		/* read ic */
		case OPER_READ:
		{
			ret = lcd_dbg_mipi_prcess_ic_reg(OPER_READ,reg, cmd_type, param_num, lcd_param_buf, &g_ic_mipi_value, 0);
			if (0 == ret)
			{
				g_ic_mipi_reg = reg;    // save read reg to global
				g_ic_mipi_value=(g_ic_mipi_value&0xff); //mask hight byte; only need 1 byte
			}
			else
			{
				LCD_LOG_ERR("%s, read ic reg fail\n", __func__);  // read fail
				goto err_handle;
			}
			break;
		}

		/* write ic */
		case OPER_WRITE:
		{
			ret = lcd_dbg_mipi_prcess_ic_reg(OPER_WRITE,reg, cmd_type, param_num, lcd_param_buf, 0, delay_ms);
			if (0 == ret)
			{
				LCD_LOG_INFO("%s, write reg: 0x%02x success\n", __func__, reg);
			}
			else
			{
				LCD_LOG_ERR("%s, write ic reg fail\n", __func__);  // write fail
				goto err_handle;
			}
			break;
		}

		/* error */
		default:
		{
			LCD_LOG_ERR("%s, op type not support!\n", __func__);
			ret = -1;
			break;
		}
	}

	/* finish */
	if (ret)
	{
		LCD_LOG_ERR("%s, fail\n", __func__);
		goto err_handle;
	}
	else
	{
		return count;
	}
err_handle:
	return -EFAULT;
}

/* fops of lcd_reg_dbg_mipi */
static const struct file_operations lcd_reg_dbg_mipi_fops = {
	.open = lcd_reg_dbg_mipi_open,
	.release = lcd_reg_dbg_mipi_release,
	.read = lcd_reg_dbg_mipi_read,
	.write = lcd_reg_dbg_mipi_write,
};


/*
 * debugfs
 *
 */
/* init lcd debugfs interface */
int lcd_debugfs_init(void)
{
	static char already_init = 0;  // internal flag
	struct dentry *dent = NULL;

	/* judge if already init */
	if (already_init)
	{
		LCD_LOG_ERR("%s(%d): already init\n", __func__, __LINE__);
		return 0;
	}

	/* create dir */
	dent = debugfs_create_dir("lcd-dbg", NULL);
	if (IS_ERR_OR_NULL(dent)) {
		LCD_LOG_ERR("%s(%d): debugfs_create_dir fail, error %ld\n",
			__func__, __LINE__, PTR_ERR(dent));
		dent = NULL;
		goto err_create_dir;
	}

	/* create reg_dbg_mipi node */
	if (NULL == debugfs_create_file("reg_dbg_mipi", 0644, dent, 0, &lcd_reg_dbg_mipi_fops)) {
		LCD_LOG_ERR("%s(%d): debugfs_create_file: reg_dbg fail\n",
			__func__, __LINE__);
		goto err_create_mipi;
	}

	already_init = 1;  // set flag
	return 0;

err_create_mipi:
	if (dent)
	{
		debugfs_remove_recursive(dent);
		dent = NULL;
	}
err_create_dir:
	return -1;
}
