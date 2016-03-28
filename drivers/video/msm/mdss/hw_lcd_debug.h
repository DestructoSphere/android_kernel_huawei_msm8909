
/* kernel\drivers\video\msm\lcd_hw_debug.h
 * this file is used by the driver team to change the 
 * LCD init parameters by putting a config file in the mobile,
 * this function can make the LCD parameter debug easier.
 * 
 * Copyright (C) 2010 HUAWEI Technology Co., ltd.
 * 
 * Date: 2010/12/10
 * By genghua
 * 
 */

#ifndef __HW_LCD_DEBUG__
#define __HW_LCD_DEBUG__
#include <linux/syscalls.h>
#include <linux/slab.h>
#include "mdss_dsi.h"

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define HW_LCD_INIT_TEST_PARAM "/data/hw_lcd_init_param.txt"
#define HW_LCD_CONFIG_TABLE_MAX_NUM 2*PAGE_SIZE

int hw_parse_dsi_cmds(struct dsi_panel_cmds *pcmds);
bool hw_free_dsi_cmds(struct dsi_panel_cmds *pcmds);
#endif 

