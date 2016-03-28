/* yuanxiaofeng add for sandisk emmc 4.5 firmware update begin */
/*
 *  swrm.h
 *
 *  Copyright ? 2013 SanDisk Corp
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This program was created by SanDisk Corp.
 * However, this program include ioctl.h  header files
 * which can be obtained via https://www.kernel.org/
 * However this code was created by Copyright ? 2013 SanDisk Corp.
 * The swrm.h file is obtained under the GPL v2.0 license that is
 * available via http://www.gnu.org/licenses/,
 * or http:www.opensource.org/licenses/gpl-2.0.php
 */

#ifndef LINUX_MMC_SWRM_H_
#define LINUX_MMC_SWRM_H_
#include <linux/mmc/ioctl.h>

enum swrm_opt {
	SWRM_INIT_DEV,
	SWRM_WRITE,
	SWRM_WRITE_NO_STATUS,
	SWRM_WRITE_ACCUMELATED_DATA,
	SWRM_READ,
	SWRM_ACCUMULATE_DATA,
	SWRM_ALLOCAT_SIZE,
	SWRM_FREE_DATA_BUF,
	SWRM_CLAIM,
	SWRM_RELEASE,
};

#define MMC_SWRM_OP 301 /* send data to SWRM module */

/*
 * FFU module callback methods
 */

struct swrm_ops {
	int (*swrm_start_req)(struct mmc_card *card, u8 *bufferr, u32 opt_mode,
		u32 length);
};

typedef int (*swrm_driver_callback)(struct swrm_ops *swrm_ops);
extern int swrm_install_mmc_io_callback(struct swrm_ops *swrm_op);
extern void swrm_uninstall_mmc_io_callback(void);
extern int mmc_blk_dispatch_swrm(struct mmc_card *card, void *idata);

#endif /* LINUX_MMC_SWRM_H_ */
/* yuanxiaofeng add for sandisk emmc 4.5 firmware update begin */
