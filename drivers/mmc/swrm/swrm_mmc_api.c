/*
 * *  swrm_mmc_api.c
 *
 *  Copyright 2007-2008 Pierre Ossman
 *
 *  Modified by SanDisk Corp., Copyright © 2013 SanDisk Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This program includes bug.h, card.h, host.h, mmc.h, scatterlist.h,
 * slab.h header files
 * The original, unmodified version of this program – the swrm_mmc_apich
 * file – is obtained under the GPL v2.0 license that is available via
 * http://www.gnu.org/licenses/,
 * or http://www.opensource.org/licenses/gpl-2.0.php
*/

#include <linux/bug.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/swap.h>
#include "swrm_utilities.h"
#include "swrm_mmc_api.h"

/**
 * struct mmc_swrm_pages - pages allocated by 'alloc_pages()'.
 * @page: first page in the allocation
 * @order: order of the number of pages allocated
 */
struct mmc_swrm_pages {
	struct page *page;
	unsigned int order;
};

/**
 * struct mmc_swrm_mem - allocated memory.
 * @arr: array of allocations
 * @cnt: number of allocations
 */
struct mmc_swrm_mem {
	struct mmc_swrm_pages *arr;
	unsigned int cnt;
};

struct mmc_swrm_area {
	unsigned long max_sz;
	unsigned int dev_addr;
	unsigned int max_tfr;
	unsigned int max_segs;
	unsigned int max_seg_sz;
	unsigned int blocks;
	unsigned int sg_len;
	struct mmc_swrm_mem *mem;
	struct scatterlist *sg;
};

static void swrm_mmc_prepare_mrq(struct mmc_card *card,
	struct mmc_request *mrq, struct scatterlist *sg, unsigned int sg_len,
	unsigned int lba, unsigned int blocks, unsigned int blksz, int write)
{
	BUG_ON(!mrq || !mrq->cmd || !mrq->data || !mrq->stop);

	if (blocks > 1) {
		mrq->cmd->opcode = write ?
			MMC_WRITE_MULTIPLE_BLOCK : MMC_READ_MULTIPLE_BLOCK;
	} else {
		mrq->cmd->opcode = write ? MMC_WRITE_BLOCK :
			MMC_READ_SINGLE_BLOCK;
	}

	mrq->cmd->arg = lba;
	if (!mmc_card_blockaddr(card))
		mrq->cmd->arg <<= 9;
		printk(KERN_DEBUG "swrm_mmc_prepare_mrq opcode: %d\n",
			mrq->cmd->opcode);
	mrq->cmd->flags = MMC_RSP_R1 | MMC_CMD_ADTC;

	if (blocks == 1) {
		mrq->stop = NULL;
	} else {
		mrq->stop->opcode = MMC_STOP_TRANSMISSION;
		mrq->stop->arg = 0;
		mrq->stop->flags = MMC_RSP_R1B | MMC_CMD_AC;
	}

	mrq->data->blksz = blksz;
	mrq->data->blocks = blocks;
	mrq->data->flags = write ? MMC_DATA_WRITE : MMC_DATA_READ;
	mrq->data->sg = sg;
	mrq->data->sg_len = sg_len;

	mmc_set_data_timeout(mrq->data, card);
}

/*
 * Checks that a normal transfer didn't have any errors
 */
static int swrm_mmc_check_result(struct mmc_request *mrq)
{
	BUG_ON(!mrq || !mrq->cmd || !mrq->data);

	if (mrq->cmd->error != 0)
		return SWRM_ERR_INVALID;

	if (mrq->data->error != 0)
		return SWRM_ERR_INVALID;

	if (mrq->stop != NULL && mrq->stop->error != 0)
		return SWRM_ERR_INVALID;

	if (mrq->data->bytes_xfered != mrq->data->blocks * mrq->data->blksz)
		return SWRM_ERR_INVALID;

	return SWRM_ERR_SUCCESS;
}

static int swrm_mmc_busy(struct mmc_command *cmd)
{
	return !(cmd->resp[0] & R1_READY_FOR_DATA) ||
		(R1_CURRENT_STATE(cmd->resp[0]) == R1_STATE_PRG);
}

static int swrm_mmc_wait_busy(struct mmc_card *card)
{
	int ret, busy = 0;
	struct mmc_command cmd = {0};

	do {
		memset(&cmd, 0, sizeof(struct mmc_command));
		cmd.opcode = MMC_SEND_STATUS;
		cmd.arg = card->rca << 16;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;
		ret = mmc_wait_for_cmd(card->host, &cmd, 0);
		if (ret)
			break;

		if (!busy && swrm_mmc_busy(&cmd)) {
			busy = 1;
			if (card->host->caps & MMC_CAP_WAIT_WHILE_BUSY) {
				printk(KERN_DEBUG "%s: Warning: Host did not \
					wait for busy state to end.\n",
					mmc_hostname(card->host));
			}
		}

	} while (swrm_mmc_busy(&cmd));

	return ret;
}

/*
 * transfer with certain parameters
 */
static int swrm_mmc_simple_transfer(struct mmc_card *card,
	struct scatterlist *sg, unsigned int sg_len, unsigned int lba,
	unsigned int blocks, unsigned int blksz, int write ,
	int wait_for_request)
{
	struct mmc_request mrq = {0};
	struct mmc_command cmd = {0};
	struct mmc_command stop = {0};
	struct mmc_data data = {0};

	mrq.cmd = &cmd;
	mrq.data = &data;
	mrq.stop = &stop;
	swrm_mmc_prepare_mrq(card, &mrq, sg, sg_len, lba, blocks, blksz,
		write);
	mmc_wait_for_req(card->host, &mrq);

	/* if wait_for_request == 0 we will not send CMD 13 */
	if (wait_for_request == 1)
		swrm_mmc_wait_busy(card);

	return swrm_mmc_check_result(&mrq);
}

/*
 * Map memory into a scatterlist.
 */
static int mmc_swrm_map_sg(struct mmc_swrm_mem *mem, unsigned long size,
	struct scatterlist *sglist, unsigned int max_segs, unsigned int max_seg_sz,
	unsigned int *sg_len, int min_sg_len)
{
	struct scatterlist *sg = NULL;
	unsigned int i;
	unsigned long sz = size;

	sg_init_table(sglist, max_segs);
	if (min_sg_len > max_segs)
		min_sg_len = max_segs;

	*sg_len = 0;
	do {
		for (i = 0; i < mem->cnt; i++) {
			unsigned long len = PAGE_SIZE << mem->arr[i].order;
			if (min_sg_len && (size / min_sg_len < len))
				len = ALIGN(size / min_sg_len, 512);
			if (len > sz)
				len = sz;
			if (len > max_seg_sz)
				len = max_seg_sz;
			if (sg)
				sg = sg_next(sg);
			else
				sg = sglist;
			if (!sg)
				return -EINVAL;
			sg_set_page(sg, mem->arr[i].page, len, 0);
			sz -= len;
			*sg_len += 1;
			if (!sz)
				break;
		}
	} while (sz);

	if (sz)
		return -EINVAL;

	if (sg)
		sg_mark_end(sg);

	return 0;
}

static void mmc_swrm_free_mem(struct mmc_swrm_mem *mem)
{
	if (!mem)
		return;
	while (mem->cnt--)
		__free_pages(mem->arr[mem->cnt].page,
			     mem->arr[mem->cnt].order);
	kfree(mem->arr);
}

/*
 * Cleanup struct mmc_swrm_area.
 */
static int mmc_swrm_area_cleanup(struct mmc_swrm_area *area)
{
	struct mmc_swrm_area *t = area;

	kfree(t->sg);
	mmc_swrm_free_mem(t->mem);

	return 0;
}

/*
 * Allocate a lot of memory, preferably max_sz but at least min_sz.  In case
 * there isn't much memory do not exceed 1/16th total lowmem pages.  Also do
 * not exceed a maximum number of segments and try not to make segments much
 * bigger than maximum segment size.
 */
static struct mmc_swrm_mem *mmc_swrm_alloc_mem(unsigned long min_sz,
	unsigned long max_sz, unsigned int max_segs, unsigned int max_seg_sz)
{
	unsigned long max_page_cnt = DIV_ROUND_UP(max_sz, PAGE_SIZE);
	unsigned long min_page_cnt = DIV_ROUND_UP(min_sz, PAGE_SIZE);
	unsigned long max_seg_page_cnt = DIV_ROUND_UP(max_seg_sz, PAGE_SIZE);
	unsigned long page_cnt = 0;
	unsigned long limit = nr_free_buffer_pages() >> 4;
	struct mmc_swrm_mem *mem;

	if (max_page_cnt > limit)
		max_page_cnt = limit;
	if (min_page_cnt > max_page_cnt)
		min_page_cnt = max_page_cnt;

	if (max_seg_page_cnt > max_page_cnt)
		max_seg_page_cnt = max_page_cnt;

	if (max_segs > max_page_cnt)
		max_segs = max_page_cnt;

	mem = kzalloc(sizeof(struct mmc_swrm_mem), GFP_KERNEL);
	if (!mem)
		return NULL;

	mem->arr = kzalloc(sizeof(struct mmc_swrm_pages) * max_segs, GFP_KERNEL);
	if (!mem->arr)
		goto out_free;

	while (max_page_cnt) {
		struct page *page;
		unsigned int order;
		gfp_t flags = GFP_KERNEL | GFP_DMA | __GFP_NOWARN |
			__GFP_NORETRY;

		order = get_order(max_seg_page_cnt << PAGE_SHIFT);
		while (1) {
			page = alloc_pages(flags, order);
			if (page || !order)
				break;
			order -= 1;
		}
		if (!page) {
			if (page_cnt < min_page_cnt)
				goto out_free;
			break;
		}
		mem->arr[mem->cnt].page = page;
		mem->arr[mem->cnt].order = order;
		mem->cnt += 1;
		if (max_page_cnt <= (1UL << order))
			break;
		max_page_cnt -= 1UL << order;
		page_cnt += 1UL << order;
		if (mem->cnt >= max_segs) {
			if (page_cnt < min_page_cnt)
				goto out_free;
			break;
		}
	}

	return mem;

out_free:
	mmc_swrm_free_mem(mem);
	return NULL;
}

/*
 * Initialize an area for data transfers.
 * Copy the data to the allocated pages.
 */
static int mmc_swrm_area_init(struct mmc_swrm_area *area, struct mmc_card *card,
		u8 *data, unsigned int size)
{
	struct mmc_swrm_area *t = area;
	int ret, i, length;


	/* Make the test area size about 4MiB */
	t->max_sz = size;

	t->max_segs = card->host->max_segs;
	t->max_seg_sz = card->host->max_seg_size;
	t->max_seg_sz -= t->max_seg_sz % 512;
	t->max_tfr = t->max_sz;

	if (t->max_tfr >> 9 > card->host->max_blk_count)
		t->max_tfr = card->host->max_blk_count << 9;
	if (t->max_tfr > card->host->max_req_size)
		t->max_tfr = card->host->max_req_size;
	if (t->max_tfr / t->max_seg_sz > t->max_segs)
		t->max_tfr = t->max_segs * t->max_seg_sz;

	/*
	 * Try to allocate enough memory for a max. sized transfer.  Less is OK
	 * because the same memory can be mapped into the scatterlist more than
	 * once.  Also, take into account the limits imposed on scatterlist
	 * segments by the host driver.
	 */
	t->mem = mmc_swrm_alloc_mem(0, t->max_tfr, t->max_segs,
				    t->max_seg_sz);
	if (!t->mem)
		return -ENOMEM;

	/* copy data to page */
	length = 0;
	for ( i = 0; i < t->mem->cnt; i++)
	{
		 memcpy(page_address(t->mem->arr[i].page), data + length,
			min(size - length, t->max_seg_sz));
		length +=  t->max_seg_sz;
	}

	t->sg = kmalloc(sizeof(struct scatterlist) * t->max_segs, GFP_KERNEL);
	if (!t->sg) {
		ret = -ENOMEM;
		goto out_free;
	}

	ret = mmc_swrm_map_sg(t->mem, size, t->sg,t->max_segs,t->max_seg_sz,
		&t->sg_len, 0);

	if (ret != 0)
		goto out_free;

	return 0;

out_free:
	mmc_swrm_area_cleanup(t);
	return ret;
}

/*
 * Initilaze the MMC device
 */
int swrm_mmc_power_cycle(struct mmc_card *card)
{
	int rc;

	rc = mmc_power_save_host(card->host);
	if (rc != 0)
		return rc;
	return mmc_power_restore_host(card->host);
}

int swrm_mmc_read(struct mmc_card *card, unsigned int lba, u8 *dst,
        int size)
{
	int rc;
    struct mmc_swrm_area mem;

	if (!dst) {
		printk(KERN_ERR "swrm dst is NULL\n");
		return SWRM_ERR_INVALID;
	}

	rc =  mmc_swrm_area_init(&mem, card, dst, size);
	if (rc != 0)
		goto exit;

	rc = swrm_mmc_simple_transfer(card, mem.sg, mem.sg_len, lba,
		size / CARD_BLOCK_SIZE, CARD_BLOCK_SIZE, 0, 1);

	if (rc == 0)
		memcpy(dst, page_address(mem.mem->arr->page), size);

exit:
	mmc_swrm_area_cleanup(&mem);
	return rc;
}

int swrm_mmc_write(struct mmc_card *card, u8 *src, unsigned long lba,
        int size, int wait_for_request)
{
	int rc;
	struct mmc_swrm_area mem;

	if (!src) {
		printk(KERN_ERR "swrm src is NULL\n");
		return SWRM_ERR_INVALID;
	}

	rc =  mmc_swrm_area_init(&mem, card, src, size);
	if (rc != 0)
		goto exit;

	rc = swrm_mmc_simple_transfer(card, mem.sg, mem.sg_len, lba,
		size / CARD_BLOCK_SIZE, CARD_BLOCK_SIZE, 1, wait_for_request);

exit:
	mmc_swrm_area_cleanup(&mem);
	return rc;
}
