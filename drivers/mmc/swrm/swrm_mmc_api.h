/*
 *
 *  swrm_mmc_api.h
 *
 * Copyright © 2013 SanDisk Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This program was created by SanDisk Corp
 * The swrm_mmc_api.h file is obtained under the GPL v2.0 license that is
 * available via http://www.gnu.org/licenses/,
 * or http://www.opensource.org/licenses/gpl-2.0.php
*/

#if !defined(_SWRM_MMC_API_H_)
#define _SWRM_MMC_API_H_

#define CARD_BLOCK_SIZE 512

/*
 * Initialize the device
 *
 * command sequence:ard_send_if_cond
 * CMD0 - GO IDLE STATE
 */
int swrm_mmc_power_cycle(struct mmc_card *card);

/*
 * reads from the device
 *
 * command sequence multiple read:
 * CMD16 - SET BLOCKLEN to 512 bytes (if necessary)
 * CMD18 - READ MULTIPLE BLOCK
 * CMD12 - STOP TRANSMISSION
 */
int swrm_mmc_read(struct mmc_card *card, unsigned int lba, u8 *dst, int size);

/*
 * writes to the device
 *
 * command sequence multiple write:
 * CMD16 - SET BLOCKLEN to 512 bytes (if necessary)
 * CMD25 - READ MULTIPLE BLOCK
 * CMD12 - STOP TRANSMISSION
 */
int swrm_mmc_write(struct mmc_card *card, u8 *src, unsigned long lba,
	int size, int wait_for_request);

#endif /* _SWRM_MMC_API_H_ */

