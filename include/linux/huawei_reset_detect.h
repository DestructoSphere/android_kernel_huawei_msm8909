#ifndef __HUAWEI_RESET_DETECT_H__
#define __HUAWEI_RESET_DETECT_H__


#include <linux/types.h>
#include <asm/sizes.h>
#include <linux/of.h>

#define RESET_DETECT_TAG "[reset_detect]"


#define RESET_MAGIC_NUM_LEN      4


/* reset magic number */
#define RESET_MAGIC_APANIC       0x504E4943    /* 'P' 'N' 'I' 'C' */
#define RESET_MAGIC_WDT_BARK     0x5742524B    /* 'W' 'B' 'R' 'K' */
#define RESET_MAGIC_THERMAL      0x54484D4C    /* 'T' 'H' 'M' 'L' */
#define RESET_MAGIC_HW_RESET     0xCACADEAD
#define LONG_PRESS_RESET_REASON_MAGIC_NUM   0X4C505353  /* LPSS */

void set_reset_magic(int magic_number);
void clear_reset_magic(void);
#endif
