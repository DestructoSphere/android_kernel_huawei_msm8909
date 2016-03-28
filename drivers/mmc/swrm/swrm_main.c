
/*
 *  swrm_main.c
 *
 *  Copyright © 2013 SanDisk Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This program was created by SanDisk Corp.
 * However, this program includes mmc.h, init.h,
 * slab.h, module.h, moduleparam.h,card.h, mutex.h and mmc.h header files
 * which can be obtained via https://www.kernel.org/
 * However this code was created by Copyright © 2013 SanDisk Corp.
 * These header files swrm_main.c file is obtained under the GPL v2.0 license
 * that is available via http://www.gnu.org/licenses/,
 * or http://www.opensource.org/licenses/gpl-2.0.php
*/

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include <linux/mutex.h>
#include <linux/mmc/swrm.h>

#include "swrm_utilities.h"
#include "swrm_mmc_api.h"

typedef struct {
	u8 *buf;
	u32 offset;
	u32 length;
} swrm_data;

static DEFINE_MUTEX(swrm_lock);

static swrm_data swrm_dat;

/*
 * Allocate buffer of data that will store the fW data according to size
 * that the user application passed */
static int swrm_allocate_data(u32 size)
{
	int rc = SWRM_ERR_SUCCESS;

	if (size <= 0)
		return SWRM_ERR_INVALID;

	mutex_lock(&swrm_lock);
	if (swrm_dat.buf) {
		rc =  SWRM_ERR_INVALID;
		goto exit;
	}

	swrm_dat.buf = kmalloc(size, GFP_KERNEL);
	if (swrm_dat.buf == NULL) {
		rc = SWRM_ERR_MMC_ALLOCATION;
		goto exit;
	}
	swrm_dat.length = size;
	swrm_dat.offset = 0;

exit:

	mutex_unlock(&swrm_lock);
	return rc;
}
/* store data to the allocated buffer */
static int swrm_accumulate_data(u8 *data_buffer, u32 data_size)
{
	mutex_lock(&swrm_lock);

	if (!swrm_dat.buf)
		return SWRM_ERR_INVALID;

	if ((swrm_dat.offset + data_size) > swrm_dat.length)
		return SWRM_ERR_INVALID;

	memcpy(&swrm_dat.buf[swrm_dat.offset], data_buffer, data_size);
	swrm_dat.offset += data_size;
	mutex_unlock(&swrm_lock);

	return SWRM_ERR_SUCCESS;
}

/* swrm callback function - called from the block driver */
static int swrm_driver_callback_imp(struct mmc_card *card,
	u8 *buffer, u32 opt_mode, u32 length)
{
	int rc = SWRM_ERR_SUCCESS;

	switch (opt_mode) {
	case SWRM_INIT_DEV:
		rc = swrm_mmc_power_cycle(card);
		break;
	case SWRM_WRITE:
		rc = swrm_mmc_write(card, buffer, COMMAND_BASE, length,
			WAIT_FOR_REQ);
		break;
	case SWRM_WRITE_NO_STATUS:
		rc = swrm_mmc_write(card, buffer, COMMAND_BASE, length, 0);
		break;
	case SWRM_WRITE_ACCUMELATED_DATA:
		mutex_lock(&swrm_lock);
		if (swrm_dat.buf) {
			rc = swrm_mmc_write(card, swrm_dat.buf, COMMAND_BASE,
				length, WAIT_FOR_REQ);
			kfree(swrm_dat.buf);
			swrm_dat.buf = NULL;
			swrm_dat.offset = 0;
			swrm_dat.length = 0;
		}
		else {
			rc = SWRM_ERR_INVALID;
		}

		mutex_unlock(&swrm_lock);
		break;
	case SWRM_READ:
		rc = swrm_mmc_read(card, COMMAND_BASE, buffer, length);
		break;
	case SWRM_ACCUMULATE_DATA:
		rc = swrm_accumulate_data(buffer, length);
		break;
	case SWRM_ALLOCAT_SIZE:
		rc = swrm_allocate_data(length);
		break;
	case SWRM_FREE_DATA_BUF:
		mutex_lock(&swrm_lock);
		kfree(swrm_dat.buf);
		swrm_dat.buf = NULL;
		swrm_dat.offset = 0;
		swrm_dat.length = 0;
		mutex_unlock(&swrm_lock);
		break;
	case SWRM_CLAIM:
		mmc_claim_host(card->host);
		break;
	case SWRM_RELEASE:
		mmc_release_host(card->host);
		break;
	default:
		rc = SWRM_ERR_INVALID;
		break;
	}

	printk(KERN_DEBUG "SWRM swrm_driver_callback_imp status %d", rc);

	return rc;
}

static struct swrm_ops swrm_ops = {
	.swrm_start_req = swrm_driver_callback_imp
};

static int __init swrm_module(void)
{
	printk(KERN_DEBUG "SWRM moudle loaded %s", SWRM_VERSION);
	return swrm_install_mmc_io_callback(&swrm_ops);
}

static void __exit swrm_exit(void)
{
	swrm_uninstall_mmc_io_callback();
	kfree(swrm_dat.buf);
	printk(KERN_DEBUG "SWRM exit %s", SWRM_VERSION);
}

module_init(swrm_module);
module_exit(swrm_exit);

MODULE_DESCRIPTION("Multimedia Card (MMC) host SWRM");
MODULE_AUTHOR("Avi Shchislowski");
MODULE_LICENSE("GPL");

