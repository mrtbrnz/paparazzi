/*
 * Copyright (C) Murat BRONZ
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/autonomous_soaring/autonomous_soaring.h"
 * @author Murat BRONZ
 * Soaring related stuff
 */

#include "std.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_geodetic_int.h"


#ifndef AUTONOMOUS_SOARING_H
#define AUTONOMOUS_SOARING_H

// extern float fault_right;
// extern float fault_left;
// extern float fault_offset_left;
// extern float fault_offset_right;
extern void soaring_init(void);
// extern void fault_Set_Right(float _v);
// extern void fault_Set_Left(float _v);
// extern void fault_Set_Offset_Right(float _v);
// extern void fault_Set_Offset_Left(float _v);


extern void soaring_parse_AEROPROBE(uint8_t *buf);

extern void soaring_status_report(void);
// extern void fault_periodic();
// extern void fault_datalink_callback();

#endif

