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

uint8_t multi_hca_add[4]={0x70, 0x71, 0x72, 0x73};

#define MULTI_HCA_ADDR 0x70
#define BARO_HCA_MAX_PRESSURE 1100 // mBar
#define BARO_HCA_MIN_PRESSURE 800 // mBar
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

// Global variables
uint16_t pBaroRaw[4];
bool multi_hca_valid;
float multi_hca_p;


struct i2c_transaction multi_hca_i2c_trans[4];

void multi_hca_init(void)
{
  uin8_t i=0;

  for(i=0;i<4;i++) {
    pBaroRaw[i] = 0;
    multi_hca_valid[i] = true;
    multi_hca_i2c_trans[i].status = I2CTransDone;
  }
}


void multi_hca_read_periodic(void)
{
  uin8_t i=0;

  for(i=0;i<4;i++) {
    if (multi_hca_i2c_trans[i].status == I2CTransDone) {
      i2c_receive(&MULTI_HCA_I2C_DEV, &multi_hca_i2c_trans[i], multi_hca_add[i], 2);
    }
  }
}


void multi_hca_read_event(void)
{
  uin8_t i=0;
  uint16_t pBaroRaw = 0;

  for(i=0;i<4;i++) {

    pBaroRaw = 0;
    pBaroRaw = ((uint16_t)multi_hca_i2c_trans[i].buf[0] << 8) | multi_hca_i2c_trans[i].buf[1];
  
    if (pBaroRaw == 0) {
      multi_hca_valid = false;
    } else {
      multi_hca_valid = true;
    }
  
    if (multi_hca_valid) {
      //Cut RAW Min and Max
      if (pBaroRaw < BARO_HCA_MIN_OUT) {
        pBaroRaw = BARO_HCA_MIN_OUT;
      }
      if (pBaroRaw > BARO_HCA_MAX_OUT) {
        pBaroRaw = BARO_HCA_MAX_OUT;
      }
    }
    multi_hca_i2c_trans[i].status = I2CTransDone;
    pBaroRaw[i]=pBaroRaw;
    
    uint16_t foo = 0;
    float bar = 0;
  #ifdef SENSOR_SYNC_SEND
    DOWNLINK_SEND_BARO_ETS(DefaultChannel, DefaultDevice, &pBaroRaw[i], &foo, &bar)
  #else
    RunOnceEvery(10, DOWNLINK_SEND_BARO_ETS(DefaultChannel, DefaultDevice, &pBaroRaw[i], &foo, &bar));
  #endif
  } 
}















