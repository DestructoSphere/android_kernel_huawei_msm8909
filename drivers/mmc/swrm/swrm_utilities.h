/*
 *
 *  swrm_mmc_api.h
 *
 *  Copyright © 2013 SanDisk Corp
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

#ifndef _SWRM_UTILITIES_H_
#define _SWRM_UTILITIES_H_

#define SWRM_VERSION "0.2"

#define CARD_BLOCK_SIZE 512
#define COMMAND_BASE 0x2000
#define WAIT_FOR_REQ 0x1

enum swrm_error {
	SWRM_ERR_SUCCESS,
	SWRM_ERR_INIT,
	SWRM_ERR_READ,
	SWRM_ERR_WRITE,
	SWRM_ERR_INVALID,
	SWRM_ERR_MMC_ALLOCATION
};

#endif /* SWRM_UTILITIES_H_ */
