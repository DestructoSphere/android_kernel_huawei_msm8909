/*
 *
 * Copyright (C) 2013 HUAWEI, Inc.
 *File Name: kernel/drivers/misc/hw_sensor_info.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __HW_SENSOR_INFO_H__
#define __HW_SENSOR_INFO_H__

#define SENSORS_ACCELERATION_HANDLE             0
#define SENSORS_MAGNETIC_FIELD_HANDLE           1
#define SENSORS_ORIENTATION_HANDLE              2
#define SENSORS_LIGHT_HANDLE                    3
#define SENSORS_PROXIMITY_HANDLE                4
#define SENSORS_GYROSCOPE_HANDLE                5
#define SENSORS_PRESSURE_HANDLE                 6
#define SENSORS_HALL_HANDLE                     7
#define SENSORS_PROX_HANDLE			8

#define SENSOR_TYPE_CAP_PROX            	(10005)
#define SENSOR_VAL_SAME_MAX_TIMES 			(20)

const char *get_sensor_info_of_product_name(void);

#endif
