/*
 * Copyright (C) 2012 Murat Bronz, Gautier Hattenberger (ENAC)
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

#ifndef MULTI_HCA_H
#define MULTI_HCA_H

#include "std.h"

#ifndef HCA_NB_SENSORS
#define HCA_NB_SENSORS	4
#endif

struct hca_sensor {
  bool valid;
  float scaled;
  uint16_t raw;
};

extern struct hca_sensor hca_sensors[HCA_NB_SENSORS];

extern void multi_hca_init(void);
extern void multi_hca_read_periodic(void);
extern void multi_hca_read_event(void);

#endif

