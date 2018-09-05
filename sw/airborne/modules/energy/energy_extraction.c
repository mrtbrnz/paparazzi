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
 * @file "modules/energy/energy_extraction.c"
 * @author Murat BRONZ
 * Briefly estimates the in-plane wind gradients and defines a longitudinal control via pitch angle.
 */

#include "modules/energy/energy_extraction.h"

// State interface for rotation compensation
#include "state.h"

// Time and uint32_t 
#include "mcu_periph/sys_time.h"

//#include "generated/modules.h"

// Math functions
#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_float.h"

// Messages
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#endif

#if FLIGHTRECORDER_SDLOG
#include "subsystems/datalink/telemetry.h"
#include "modules/loggers/pprzlog_tp.h"
#include "modules/loggers/sdlog_chibios.h"
#endif

// local variables
static uint32_t last_periodic_time;     // last periodic time
static float wx_old;
static float wz_old;
static int sampling;

/**
 * Default parameters
 */
#ifndef WX_P
#define WX_P 0.f           // 
#endif
#ifndef WX_D
#define WX_D 0.f           // 
#endif
#ifndef WZ_P
#define WZ_P 0.f           // 
#endif
#ifndef WZ_D
#define WZ_D 0.f           // 
#endif
#ifndef FLIGHT_CORIDOR
#define FLIGHT_CORIDOR 20.f// 
#endif

struct Gust_states gust_states;
struct Gust_gains gust_gains;

static void gust_msg_send(struct transport_tx *trans, struct link_device *dev) {
  pprz_msg_send_GUST(trans, dev, AC_ID,
      &gust_states.wx,
      &gust_states.wz,
      &gust_states.va,
      &gust_states.gama,
      &gust_states.aoa,
      &gust_states.theta_cmd);
}



void gust_init(void) {
	memset(&gust_states, 0, sizeof(struct Gust_states));
	last_periodic_time = 0;
	wx_old = 0.0;
	wz_old = 0.0;

	gust_gains.p_wx = WX_P;
	gust_gains.d_wx = WX_D;
	gust_gains.p_wz = WZ_P;
	gust_gains.d_wz = WZ_D;

	gust_gains.wx_sign = 1;
	gust_gains.wz_sign = -1;
	gust_gains.throttle_control = 0;
	gust_gains.sample_nr = 1;
	gust_gains.flight_coridor = FLIGHT_CORIDOR;

	#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GUST, gust_msg_send);
	#endif
}

static inline void gust_run_step(){
	float va = gust_states.va;
	gust_states.gama = gust_states.theta - gust_states.aoa;
	gust_states.wx = gust_gains.wx_sign*(va*cosf(gust_states.gama) - gust_states.Vx);
	gust_states.wz = gust_gains.wz_sign*(va*sinf(gust_states.gama) - gust_states.Vz);

	// Calculate the derivatives
	float d_wx = (gust_states.wx - wx_old) / gust_states.dt;
	float d_wz = (gust_states.wz - wz_old) / gust_states.dt;
	wx_old = gust_states.wx;
	wz_old = gust_states.wz;
	if (va > 5.0f){
		gust_states.theta_cmd = gust_gains.p_wx*gust_states.wx + gust_gains.d_wx*d_wx + gust_gains.p_wz*gust_states.wz + gust_gains.d_wz*d_wz; // stateGetBodyRates_f()->q // just to try for now...
	} else {
		gust_states.theta_cmd = 0.0;
	}
}
	

#include "subsystems/imu.h"
/*----------------periodic function-----------------*/
/*  Get Data from State and place into struct       */
/*--------------------------------------------------*/
void gust_periodic(void) {

// Get state variables...
  gust_states.va  = stateGetAirspeed_f();           // m/s
  gust_states.aoa = stateGetAngleOfAttack_f();      // rad.
  gust_states.theta = stateGetNedToBodyEulers_f()->theta; // pitch angle in rad.
  gust_states.Vx = stateGetHorizontalSpeedNorm_f(); // Ground speedNorm, used as in-plane forward direction groundspeed
  gust_states.Vz = stateGetSpeedNed_f()->z;         // Vertical speed  m/s

  if (last_periodic_time == 0) {
    gust_states.dt = GUST_PERIODIC_PERIOD;
    last_periodic_time = get_sys_time_msec();
  } else {
    gust_states.dt = (get_sys_time_msec() - last_periodic_time); //   / 1000.f;
    last_periodic_time = get_sys_time_msec();
  }
  if (sampling > gust_gains.sample_nr){
	gust_run_step();
	sampling = 0;	
  }
  sampling += 1;
}





void energy_extraction_Set_WX_Sign(float _v)
{
  gust_gains.wx_sign = _v;
}

void energy_extraction_Set_WZ_Sign(float _v)
{
  gust_gains.wz_sign = _v;
}

void energy_extraction_Set_WX_P(float _v)
{
  gust_gains.p_wx = _v;
}

void energy_extraction_Set_WX_D(float _v)
{
  gust_gains.d_wx = _v;
}

void energy_extraction_Set_WZ_P(float _v)
{
  gust_gains.p_wz = _v;
}

void energy_extraction_Set_WZ_D(float _v)
{
  gust_gains.d_wz = _v;
}

void energy_extraction_Set_Sampling(int _v)
{
  gust_gains.sample_nr = _v;
}

void energy_extraction_Set_Throttle_Control(int _v)
{
  gust_gains.throttle_control = _v;
}

void energy_extraction_Set_Flight_Coridor(float _v)
{
  gust_gains.flight_coridor = _v;
}
// float gain = (float)fabs( (double) (cosf(phi) * cosf(theta)));


// void gust_event() {}
// void gust_datalink_callback() {}




