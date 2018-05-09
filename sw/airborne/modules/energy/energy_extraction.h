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
 * @file "modules/energy/energy_extraction.h"
 * @author Murat BRONZ
 * Briefly estimates the in-plane wind gradients and defines a longitudinal control via pitch angle.
 */

#include "mcu_periph/sys_time.h"

#ifndef ENERGY_EXTRACTION_H
#define ENERGY_EXTRACTION_H

struct Gust_states {
  float Vx;          ///< GPS, Horizontal ground speed norm
  float Vz;          ///< GPS, Vertical speed
  float wx;          ///< in-plane horizontal wind component
  float wz;          ///< vertical wind component
  float va;          ///< airspeed
  float gama;        ///< flight path angle
  float aoa;         ///< angle of attack
  float theta;       ///< theta pitch angle
  float theta_cmd;   ///< desired optimum pitch angle calculated from wind gradients
  uint32_t dt;       ///< time step
};

struct Gust_gains {
	float p_wx;
	float d_wx;
	float p_wz;
	float d_wz;
  float wx_sign;
  float wz_sign;
  int sample_nr;
};

extern struct Gust_gains gust_gains;

// Access from guidance loop
extern struct Gust_states gust_states;

extern void gust_init(void);
extern void gust_periodic(void);
// extern void gust_event();
// extern void gust_datalink_callback();


// paramenters settings handler
extern void energy_extraction_Set_WX_Sign(float _v);
extern void energy_extraction_Set_WZ_Sign(float _v);

extern void energy_extraction_Set_WX_P(float _v);
extern void energy_extraction_Set_WX_D(float _v);
extern void energy_extraction_Set_WZ_P(float _v);
extern void energy_extraction_Set_WZ_D(float _v);

extern void energy_extraction_Set_Sampling(int _v);
#endif

