/* yuanxiaofeng add for sandisk emmc 4.5 firmware update begin */
/*
 *  swrm_int.c
 *
 *  Copyright ? 2013 SanDisk Corp
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This program was created by SanDisk Corp.
 * However, this program includes module.h, init.h, slab.h and card.h header
 * files which can be obtained via https://www.kernel.org/
 * However, this code was created by Copyright ? 2013 SanDisk Corp. 2013
 * The swrm_int.c file is obtained under the GPL v2.0 license that is
 * available via http://www.gnu.org/licenses/,
 * or http:www.opensource.org/licenses/gpl-2.0.php
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/mmc/card.h>
#include <linux/mmc/swrm.h>
#include <asm/uaccess.h>

/* This struct was taking from /drver/mmc/card/block.c */
struct mmc_blk_ioc_data {
	struct mmc_ioc_cmd ic;
	unsigned char *buf;
	u64 buf_bytes;
};

struct swrm_ioc {
	u8 *data;
	u32 size;
	u32 mode;
};

static struct swrm_ops *current_swrm_driver_callback = NULL;

int swrm_install_mmc_io_callback(struct swrm_ops *callback)
{
	if (callback == NULL ||
		callback->swrm_start_req == NULL) {
		pr_err("%s: not all functions defined\n", __FUNCTION__);
		return -EINVAL;
	}

	current_swrm_driver_callback = callback;
	return 0;
}

void swrm_uninstall_mmc_io_callback(void)
{
	current_swrm_driver_callback = NULL;
}

EXPORT_SYMBOL(swrm_install_mmc_io_callback);
EXPORT_SYMBOL(swrm_uninstall_mmc_io_callback);

int mmc_blk_dispatch_swrm(struct mmc_card *card, void *data)
{
	int rc = 0;
	u32 length;
	struct swrm_ioc *ioc_opt;
	struct mmc_blk_ioc_data *swrm_idata = (struct mmc_blk_ioc_data*)data;

	if (current_swrm_driver_callback == NULL)
		return -EINVAL;

	ioc_opt = kzalloc(sizeof(*ioc_opt), GFP_KERNEL);
	if (!ioc_opt) {
		printk(KERN_ERR "mmc_blk_dispatch_swrm kzalloc ioc_opt\n");
		rc =  -ENOMEM;
		goto exit;
	}

	ioc_opt->mode = ((struct swrm_ioc*)(swrm_idata->buf))->mode;
	ioc_opt->size = ((struct swrm_ioc*)(swrm_idata->buf))->size;
	length = ioc_opt->size * swrm_idata->ic.blksz;

	if (ioc_opt->mode == SWRM_ACCUMULATE_DATA ||
		ioc_opt->mode == SWRM_WRITE_NO_STATUS ||
		ioc_opt->mode == SWRM_WRITE ||
		ioc_opt->mode == SWRM_READ)
	{
		ioc_opt->data = kzalloc(length, GFP_KERNEL);
		if (!ioc_opt->data) {
			rc =  -ENOMEM;
			goto exit;
		}

		if (copy_from_user(ioc_opt->data,
			((struct swrm_ioc*)(swrm_idata->buf))->data, length)) {
			printk(KERN_ERR "mmc_blk_dispatch_swrm copy fail\n");
			rc = -EFAULT;
			goto exit;
		}
	}

	/* call module callback function */
	rc = current_swrm_driver_callback->swrm_start_req(card, ioc_opt->data,
		ioc_opt->mode, length);
	if (rc != 0)
		goto exit;

	if (!swrm_idata->ic.write_flag){
		if (copy_to_user(((struct swrm_ioc*)(swrm_idata->buf))->data,
			ioc_opt->data, length)) {
				rc = -EFAULT;
		}
	}
exit:
	kfree(ioc_opt->data);
	return rc;
}

EXPORT_SYMBOL(mmc_blk_dispatch_swrm);
/* yuanxiaofeng add for sandisk emmc 4.5 firmware update begin */
