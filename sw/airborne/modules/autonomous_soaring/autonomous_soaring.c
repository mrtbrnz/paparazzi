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
 * @file "modules/autonomous_soaring/autonomous_soaring.c"
 * @author Murat BRONZ
 * Soaring related stuff
 */

#include "modules/autonomous_soaring/autonomous_soaring.h"
#include "subsystems/datalink/telemetry.h"

#if FLIGHTRECORDER_SDLOG
#include "modules/loggers/pprzlog_tp.h"
#include "modules/loggers/sdlog_chibios.h"
#endif

// #include "modules/datalink/extra_pprz_dl.h"
// #include "subsystems/datalink/telemetry.h"
#include <string.h>
#include "generated/airframe.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/downlink.h"

int16_t smartprobe_velocity;
int16_t smartprobe_a_attack;
int16_t smartprobe_a_sideslip;

static void soaring_downlink(struct transport_tx *trans, struct link_device *dev){
  pprz_msg_send_SOARING_TELEMETRY(trans, dev, AC_ID, &smartprobe_velocity, &smartprobe_a_attack, &smartprobe_a_sideslip);
}

void soaring_init(void) {
  smartprobe_velocity = 0;
  smartprobe_a_attack = 0;
  smartprobe_a_sideslip = 2;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SOARING_TELEMETRY, soaring_downlink);
#endif
}

void soaring_parse_AEROPROBE(uint8_t *buf)
{
  smartprobe_velocity  =  DL_AEROPROBE_velocity(buf);
  smartprobe_a_attack  =  DL_AEROPROBE_a_attack(buf);
  smartprobe_a_sideslip  =  DL_AEROPROBE_a_sideslip(buf);
}

void soaring_status_report(void)
{
  soaring_downlink(&(DefaultChannel).trans_tx, &(DefaultDevice).device);
}

// void fault_periodic() {}
// void fault_datalink_callback() {}


