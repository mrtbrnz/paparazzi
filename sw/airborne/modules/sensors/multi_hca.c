/*
 * Copyright (C) 2017 Murat Bronz, Gautier Hattenberger (ENAC)
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

#include "sensors/multi_hca.h"
#include "mcu_periph/i2c.h"
#include "subsystems/abi.h"
#include <math.h>

//Messages
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

#if FLIGHTRECORDER_SDLOG
#include "subsystems/datalink/telemetry.h"
#include "modules/loggers/pprzlog_tp.h"
#include "modules/loggers/sdlog_chibios.h"
#endif

#define BARO_HCA_MAX_OUT 27852 //dec
#define BARO_HCA_MIN_OUT 1638 //dec

// FIXME
#ifndef BARO_HCA_SCALE
#define BARO_HCA_SCALE 1.0
#endif

// FIXME
#ifndef BARO_HCA_PRESSURE_OFFSET
#define BARO_HCA_PRESSURE_OFFSET 101325.0
#endif

#ifndef MULTI_HCA_I2C_DEV
#define MULTI_HCA_I2C_DEV i2c0
#endif

#define SENSOR_SYNC_SEND  1

struct hca_sensor hca_sensors[HCA_NB_SENSORS];

struct i2c_transaction multi_hca_i2c_trans[HCA_NB_SENSORS];

#ifndef HCA_SENSORS_ADDR
#define HCA_SENSORS_ADDR { 0x70<<1, 0x71<<1, 0x72<<1, 0x73<<1 }
#endif
uint8_t multi_hca_addr[] = HCA_SENSORS_ADDR;


void multi_hca_init(void)
{
  uint8_t i = 0;

  for (i = 0; i < HCA_NB_SENSORS; i++) {
    hca_sensors[i].valid = false;
    hca_sensors[i].raw = 0;
    hca_sensors[i].scaled = 0.f;
    multi_hca_i2c_trans[i].status = I2CTransDone;
  }
}


void multi_hca_read_periodic(void)
{
  uint8_t i = 0;

  for (i = 0; i < HCA_NB_SENSORS; i++) {
    if (multi_hca_i2c_trans[i].status == I2CTransDone) {
      i2c_receive(&MULTI_HCA_I2C_DEV, &multi_hca_i2c_trans[i], multi_hca_addr[i], 2);
    }
  }
}


void multi_hca_read_event(void)
{
  uint8_t i = 0;
  uint16_t pBaroRaw = 0;
  static uint8_t new_data = 0;

  for (i = 0; i < HCA_NB_SENSORS; i++) {
    if (multi_hca_i2c_trans[i].status == I2CTransSuccess) {
      pBaroRaw = 0;
      pBaroRaw = ((uint16_t)multi_hca_i2c_trans[i].buf[0] << 8) | multi_hca_i2c_trans[i].buf[1];

      if (pBaroRaw == 0) {
        hca_sensors[i].valid = false;
      } else {
        hca_sensors[i].valid = true;
	new_data++;
      }

      if (hca_sensors[i].valid) {
        //Cut RAW Min and Max
        if (pBaroRaw < BARO_HCA_MIN_OUT) {
          pBaroRaw = BARO_HCA_MIN_OUT;
        }
        if (pBaroRaw > BARO_HCA_MAX_OUT) {
          pBaroRaw = BARO_HCA_MAX_OUT;
        }
      }
      hca_sensors[i].raw = pBaroRaw;
      //FIXME apply scale
      hca_sensors[i].scaled = (float) pBaroRaw;

      multi_hca_i2c_trans[i].status = I2CTransDone;
    } else if (multi_hca_i2c_trans[0].status == I2CTransFailed) {
      multi_hca_i2c_trans[0].status = I2CTransDone;
    }
  }

#if SENSOR_SYNC_SEND || defined FLIGHTRECORDER_SDLOG
  if (new_data >= HCA_NB_SENSORS) {
    float tab[HCA_NB_SENSORS];
    for (i = 0; i < HCA_NB_SENSORS; i++) {
      tab[i] = hca_sensors[i].scaled;
    }
#if SENSOR_SYNC_SEND
    DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, HCA_NB_SENSORS, tab);
#endif

#if FLIGHTRECORDER_SDLOG
    if (flightRecorderLogFile != -1) {
      DOWNLINK_SEND_PAYLOAD_FLOAT(pprzlog_tp, flightrecorder_sdlog, HCA_NB_SENSORS, tab);
    }
#endif

    new_data = 0;
  }
#endif
}

