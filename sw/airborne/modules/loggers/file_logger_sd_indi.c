/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file modules/loggers/file_logger.c
 *  @brief File logger for Linux based autopilots
 */

#include "file_logger_sd_indi.h"

#include "modules/loggers/sdlog_chibios.h"

#include "modules/fault/fault.h"

#include "subsystems/imu.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "subsystems/commands.h"
#include "subsystems/actuators.h"
#include "subsystems/actuators/motor_mixing.h"
#include "state.h"

static uint32_t counter;

/** Start the file logger and open a new file */
void file_logger_sd_indi_start(void)
{
  counter = 0;

  if (pprzLogFile != -1) {
    sdLogWriteLog(pprzLogFile,
      "counter,gyro_p,gyro_q,gyro_r,ax,ay,az,act0,act1,act2,act3,COMMAND_THROTTLE,COMMAND_ROLL,COMMAND_PITCH,COMMAND_YAW\n"
      //"counter,gyro_p,gyro_q,gyro_r,COMMAND_THRUST,COMMAND_ROLL,COMMAND_PITCH,COMMAND_YAW,accel_z\n"
    );
  }
}



/** Log the values to a csv file */
void file_logger_sd_indi_periodic(void)
{
  if (pprzLogFile == -1) {
    return;
  }
  // Check sw/airborne/math/pprz_algebra_int.h for the definition of integer conversions
  //struct FloatRates *rates = stateGetBodyRates_f();
  struct Int32Rates *rates = stateGetBodyRates_i();
  struct Int32Vect3 *accel_body = stateGetAccelBody_i();
  struct NedCoor_i *pos_ned = stateGetPositionNed_i();
  //float accelz = ACCEL_FLOAT_OF_BFP(accel_body->z);
  // int32_t psi = stateGetNedToBodyEulers_i()->psi;
  struct Int32Quat *quat = stateGetNedToBodyQuat_i();
  // struct FloatQuat *quat = stateGetNedToBodyQuat_f();
  // struct Int32Quat   stab_att_sp_quat;


  sdLogWriteLog(pprzLogFile,
      //"%lu,%.5f,%.5f,%.5f,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n",
      // "%lu, %ld,%ld,%ld, %ld,%ld,%ld, %ld,%ld,%ld, %d,%d,%d,%d,%d,%d, %ld,%ld,%ld,%ld \n",
      // "%lu, %ld,%ld,%ld, %ld,%ld,%ld, %ld,%ld,%ld, %d,%d,%d,%d,%d,%d \n", for RoBust
      // "%lu, %ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%d,%d,%d,%d\n", //for Quadrotor
    // "%lu, %ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%d,%d,%d,%d,%ld,%ld,%ld,%ld\n", //for Quadrotor with quaternions
    "%lu, %ld,%ld,%ld, %ld,%ld,%ld, %ld,%ld,%ld, %d,%d,%d,%d,%d,%d, %d, %f,%f,%f,%f,%f,%f, %ld,%ld,%ld,%ld, %ld,%ld,%ld,%ld, \n", //for Hexa with GEAR and quaternions as setpoints
      //"%lu,%.5f,%.5f,%.5f,%ld,%ld,%ld,%d,%d,%d\n",
      counter,
      rates->p,
      rates->q,
      rates->r,
      accel_body->x,
      accel_body->y,
      accel_body->z,
      pos_ned->x,
      pos_ned->y,
      pos_ned->z,
      actuators_pprz[0],
      actuators_pprz[1],
      actuators_pprz[2],
      actuators_pprz[3],
      actuators_pprz[4],
      actuators_pprz[5],
      // motor_mixing.commands[0], these are for the motor mixing module
      // motor_mixing.commands[1],
      // motor_mixing.commands[2],
      // motor_mixing.commands[3],
      //commands[COMMAND_THROTTLE],
      //commands[COMMAND_ROLL],
      //commands[COMMAND_PITCH]
      commands[COMMAND_GEAR],
      upper_front_effectiveness,
      upper_right_effectiveness,
      upper_left_effectiveness,
      bottom_front_effectiveness,
      bottom_right_effectiveness,
      bottom_left_effectiveness,
      // stabilization_cmd[COMMAND_THRUST],
      // stabilization_cmd[COMMAND_ROLL],
      // stabilization_cmd[COMMAND_PITCH],
      // stabilization_cmd[COMMAND_YAW]
      // int32_t stateGetNedToBodyEulers_i()->theta,
      //accelz
      quat->qi,
      quat->qx,
      quat->qy,
      quat->qz,
      stab_att_sp_quat.qi,
      stab_att_sp_quat.qx,
      stab_att_sp_quat.qy,
      stab_att_sp_quat.qz
      );

  counter++;
}
