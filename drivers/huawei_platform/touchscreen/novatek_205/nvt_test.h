/* drivers/input/touchscreen/novatek/test.h
 *
 * Copyright (C) 2010 - 2014 Novatek, Inc.
 * 
 * History:
 * V1.0 : First Released
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
 
#ifndef     _LINUX_NVT_TOUCH_TEST_H
#define _LINUX_NVT_TOUCH_TEST_H

/********************Changeble Parms*******************/
#include "nvt_samples/nvt_sample_SCALE_CMI.h"

//Open Test Parameters
int IPOSTIVE_TOLERANCE = 320;
int ITOLERANCE_S = 320;

//---Sensor Mapping---
char * AIN = NULL;
//---FPC Golden Sample---
int * FPC_CM = NULL;
//---Module Golden Sample---
long int * Mutual_AVG = NULL;
void init_sample_test_parms(struct nvt_ts_data * );
/********************Changeble Parms*******************/


typedef struct{
    unsigned short tx;
    unsigned short rx;
    signed short flag;
}mapping_t;
extern int update_firmware(u8 *fw_data);

#endif /* _LINUX_NVT_TOUCH_H */
