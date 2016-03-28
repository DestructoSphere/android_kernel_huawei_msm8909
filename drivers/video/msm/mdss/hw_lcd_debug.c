/* kernel\drivers\video\msm\mdss\lcd_hw_debug.c
 * this file is used by the driver team to change the 
 * LCD init parameters by putting a config file in the mobile,
 * this function can make the LCD parameter debug easier.
 * 
 * Copyright (C) 2010 HUAWEI Technology Co., ltd.
 * 
 * Date: 2013/12/6
 * By vaibhav
 * 
 */

#include "hw_lcd_debug.h"
#include <asm/uaccess.h>
#include <linux/hw_lcd_common.h>

#define IS_VALID_CHAR(_ch) ((_ch >= '0' && _ch <= '9')?1:\
	(_ch >= 'a' && _ch <= 'f')?1:(_ch >= 'A' && _ch <= 'F'))?1:0

#define HEX_BASE ((char)16)

static char hex_char_to_value(char ch){
	switch (ch){
	case 'a' ... 'f':
		ch = 10 + (ch - 'a');
		break;
	case 'A' ... 'F':
		ch = 10 + (ch - 'A');
		break;
	case '0' ... '9':
		ch = ch - '0';
		break;
	}
	return ch;
}

static int fget_dtsi_style(unsigned char * buf, int max_num, int fd,off_t *fd_seek)
{
	int cur_seek=*fd_seek;
	unsigned char ch = '\0';
	unsigned char last_char = 'Z';
	int j =0;
	
	sys_lseek(fd, (off_t)0,0);

	while (j < max_num){
		if ((unsigned)sys_read(fd, &ch, 1) != 1) {
			LCD_LOG_INFO("\n%s: its end of file %d : len = %d\n", __func__, __LINE__, j);
			return j;
		}
		else
		{
			if (!IS_VALID_CHAR(ch)){
				last_char = 'Z';
				cur_seek++;
				sys_lseek(fd, (off_t)cur_seek,0);
				continue;
			}

			if (last_char != 'Z'){
				/*two char value is possible like F0, so make it a single char*/
				--j;
				buf[j] = (buf[j] * HEX_BASE) + hex_char_to_value(ch);
				last_char = 'Z';
			}else{
				buf[j]= hex_char_to_value(ch);
				last_char = buf[j];
			}

			j++;
			cur_seek++;
			sys_lseek(fd, (off_t)cur_seek,0);
		}		
	}

	if (j >= max_num){
		LCD_LOG_ERR("%s:Buffer is not enough", __func__);
		j *= -1;
	}

	*fd_seek=cur_seek;
	return j;
}

static bool lcd_resolve_dtsi_config_file(int fd, void **para_table,uint32_t *para_num)
{
	off_t fd_seek=0;
	int num = 0;
	unsigned char *lcd_config_table = NULL;
	lcd_config_table = kzalloc(HW_LCD_CONFIG_TABLE_MAX_NUM, 0);

	if(NULL ==  lcd_config_table){
		goto kalloc_err;
	}

	sys_lseek(fd, (off_t)0, 0);

	num = fget_dtsi_style(lcd_config_table, HW_LCD_CONFIG_TABLE_MAX_NUM, fd, &fd_seek);
	if (num <= 0){
		LCD_LOG_INFO("%s read failed with error return:%d", __func__, num);
		goto debug_init_read_fail;
	}

	*para_num = num;
	*para_table = lcd_config_table;
	return TRUE;

debug_init_read_fail:
	kfree(lcd_config_table);
	lcd_config_table = NULL;

kalloc_err:
	para_table = NULL;
	*para_num = 0;
	return FALSE;
}

bool lcd_debug_malloc_dtsi_para(void **para_table, uint32_t *para_num)
{
	int ret = 0 ;
	int fd = 0 ;
	void * table_tmp = NULL;
	int num_tmp =0 ;
	mm_segment_t fs;

	if(NULL==para_table){
		return FALSE;
	}

	fs = get_fs();     /* save previous value */
	set_fs (get_ds()); /* use kernel limit */

	fd = sys_open((const char __force *) HW_LCD_INIT_TEST_PARAM, O_RDONLY, 0);
	if (fd < 0) 
	{
		LCD_LOG_INFO("%s: %s file doesn't exsit\n", __func__, HW_LCD_INIT_TEST_PARAM);
		set_fs(fs);
		return FALSE;
	}

	LCD_LOG_INFO( "%s: Config file %s opened. \n", __func__, HW_LCD_INIT_TEST_PARAM);

	//resolve the config file
	ret = lcd_resolve_dtsi_config_file(fd, &table_tmp,&num_tmp);
	sys_close(fd);
	set_fs(fs);

	*para_table = table_tmp;
	*para_num = (uint32_t)num_tmp;

	if (FALSE == ret){
		LCD_LOG_ERR("%s failed to read the init code into memory\n",__func__);
		return FALSE;
	}
	*para_table = table_tmp;

	LCD_LOG_INFO("%s init code is copied into memory\n",__func__);
	return TRUE;
}

static void print_cmds(struct dsi_cmd_desc *cmds, int cnt)
{
	int i = 0;
	int j = 0;

	for (i = 0; i < cnt; ++i){
		printk("%02x ", cmds->dchdr.dtype);
		printk("%02x ", cmds->dchdr.last);
		printk("%02x ", cmds->dchdr.vc);
		printk("%02x ", cmds->dchdr.ack);
		printk("%02x ", cmds->dchdr.wait);
		printk("%04x ", cmds->dchdr.dlen);
		for (j = 0; j < cmds->dchdr.dlen; ++j){
			printk("%02x ", cmds->payload[j]);
		}

		printk("\n");
		cmds++;
	}
}

int hw_parse_dsi_cmds(struct dsi_panel_cmds *pcmds)
{
	int blen = 0, len = 0;
	char *buf = NULL, *bp = NULL;
	struct dsi_ctrl_hdr *dchdr;
	int i = 0, cnt = 0;

	memset(pcmds, sizeof(struct dsi_panel_cmds), 0);

	if (!lcd_debug_malloc_dtsi_para((void **)&buf, &blen)) {
		LCD_LOG_ERR("%s: failed\n", __func__);
		return -ENOMEM;
	}

	/* scan dcs commands */
	bp = buf;
	len = blen;
	cnt = 0;
	while (len > sizeof(*dchdr)) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		dchdr->dlen = ntohs(dchdr->dlen);
		if (dchdr->dlen > len) {
			LCD_LOG_ERR("%s: dtsi cmd=%x error, len=%d",
				__func__, dchdr->dtype, dchdr->dlen);
			return -ENOMEM;
		}
		bp += sizeof(*dchdr);
		len -= sizeof(*dchdr);
		bp += dchdr->dlen;
		len -= dchdr->dlen;
		cnt++;
	}

	if (len != 0) {
		LCD_LOG_ERR("%s: dcs_cmd=%x len=%d error!",
				__func__, buf[0], blen);
		kfree(buf);
		return -ENOMEM;
	}

	pcmds->cmds = kzalloc(cnt * sizeof(struct dsi_cmd_desc),
						GFP_KERNEL);
	if (!pcmds->cmds){
		kfree(buf);
		return -ENOMEM;
	}

	pcmds->cmd_cnt = cnt;
	pcmds->buf = buf;
	pcmds->blen = blen;

	bp = buf;
	len = blen;
	for (i = 0; i < cnt; i++) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		len -= sizeof(*dchdr);
		bp += sizeof(*dchdr);
		pcmds->cmds[i].dchdr = *dchdr;
		pcmds->cmds[i].payload = bp;
		bp += dchdr->dlen;
		len -= dchdr->dlen;
	}

	print_cmds(pcmds->cmds, pcmds->cmd_cnt);

	LCD_LOG_INFO("%s: dcs_cmd=%x len=%d, cmd_cnt=%d\n", __func__,
		pcmds->buf[0], pcmds->blen, pcmds->cmd_cnt);

	return 0;
}

bool hw_free_dsi_cmds(struct dsi_panel_cmds *pcmds)
{

	if (pcmds->buf)
		kfree(pcmds->buf);

	if (pcmds->cmds)
		kfree(pcmds->cmds);
	LCD_LOG_INFO("The new LCD config region has been freed\n");
	return TRUE;
}
