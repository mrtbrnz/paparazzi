/*
 * Copyright (C) 2015 Murat Bronz
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/nav/nav_zigzag.c
 *
 * ZIGZAG Navigation routines.
 *
 * Run initialize function when the plane is on the bungee, the bungee is
 * fully extended and you are ready to launch the plane.
 * After initialized, the plane will follow a line drawn by the position
 * of the plane on initialization and the  position of the bungee (given in
 * the arguments).
 * Once the plane crosses the throttle line, which is perpendicular to the line
 * the plane is following, and intersects the position of the bungee (plus or
 * minus a fixed distance (BUNGEE_TAKEOFF_DISTANCE in airframe file) from
 * the bungee just in case the bungee doesn't release exactly above the bungee)
 * the prop will come on.
 * The plane will then continue to follow the line until it has reached a
 * specific height (defined in as BUNGEE_TAKEOFF_HEIGHT in airframe file) above
 * the bungee waypoint and airspeed (defined as BUNGEE_TAKEOFF_AIRSPEED in the
 * airframe file). The airspeed limit is only used if USE_AIRSPEED flag is
 * defined or set to true (and assuming the airspeed is then available).
 * It is also possible to specify the pitch angle (BUNGEE_TAKEOFF_PITCH) and
 * the throttle (BUNGEE_TAKEOFF_THROTTLE, between 0 and 1).
 *
 * @verbatim
 * <section name="BUNGEE" prefix="BUNGEE_TAKEOFF_">
 *   <define name="HEIGHT" value="30" unit="m"/>
 *   <define name="AIRSPEED" value="15" unit="m/s"/>
 *   <define name="DISTANCE" value="10" unit="m"/>
 *   <define name="MIN_SPEED" value="5" unit="m/s"/>
 *   <define name="PITCH" value="15." unit="deg"/>
 *   <define name="THROTTLE" value="1.0"/>
 * </section>
 * @endverbatim
 *
 */

#include "modules/nav/nav_zigzag.h"

#include "state.h"
#include "paparazzi.h"
#include "firmwares/fixedwing/nav.h"
#include "firmwares/fixedwing/autopilot.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "math/pprz_algebra_float.h"




#ifndef ZIGZAG_LEG_DISTANCE
#define ZIGZAG_LEG_DISTANCE 100.0
#endif
#ifndef ZIGZAG_BETA_ANGLE
#define ZIGZAG_BETA_ANGLE RadOfDeg(45.)
#endif

float beta;
float zigzag_distance;
uint8_t zigzag_count;

enum zigzag_status { P01, P12, P21 };

static enum zigzag_status zigzag_status;


bool_t nav_zigzag_init(void)
{
  zigzag_status = P01;
  zigzag_count = 0;
  return FALSE; //since called only once...
}

/** Navigation along a line with zigzags defined with beta angle and distance.
               2       2
              / \     / \
---START-0   /   \   /   \   /-FINISH--------
          \ /     \ /     \ /
           1       1       1
*/

bool_t nav_zigzag_run(uint8_t p1, uint8_t p2)
{
  /* Beta angle given in degrees, convert it to rad */
  //beta = beta * M_PI / 180. ;
  beta = ZIGZAG_BETA_ANGLE;

  //zigzag_distance = fabs(zigzag_distance); /* Always positive */
  zigzag_distance = fabs(ZIGZAG_LEG_DISTANCE); /* Always positive */

  float alt = waypoints[p1].a;
  waypoints[p2].a = alt;

  float p1_p2_x = waypoints[p2].x - waypoints[p1].x;
  float p1_p2_y = waypoints[p2].y - waypoints[p1].y;
  float d = sqrtf(p1_p2_x * p1_p2_x + p1_p2_y * p1_p2_y);
  d = Max(d, 1.); /* To prevent a division by zero */
  
  /* Target point initialisation */
  struct point target = {
    waypoints[p1].x,
    waypoints[p1].y,
    alt
  };

  struct point target_old = {
    waypoints[p1].x,
    waypoints[p1].y,
    alt
  };
  
  /* Unit vector from p1 to p2 */
  float u_x = p1_p2_x / d;
  float u_y = p1_p2_y / d;
  
  /* Calculate the new positive unit vector   */
  float u_x2 = u_x * cosf(beta) - u_y * sinf(beta);
  float u_y2 = u_x * sinf(beta) + u_y * cosf(beta);
  
  /* Calculate the new negative unit vector   */
  float u_x3 = u_x * cosf(-beta) - u_y * sinf(-beta);
  float u_y3 = u_x * sinf(-beta) + u_y * cosf(-beta);

  /* Update target point calculation */
  target.x = target.x + 1 * zigzag_distance * u_x2;
  target.y = target.y + 1 * zigzag_distance * u_y2;
       

//  nav_route_xy(target_old.x, target_old.y, target.x, target.y);
  int i;
  switch (zigzag_status) {
    case P01 :
      nav_route_xy(target_old.x, target_old.y, target.x, target.y);
      if (nav_approaching_xy(target.x, target.y, target_old.x, target_old.y, CARROT)) {
        zigzag_status = P12;
        zigzag_count++;
        nav_init_stage();
      }
      return TRUE;
      
    case P12 :
      for (i=1; i<zigzag_count; i++){
       target_old.x = target.x;
       target_old.y = target.y;
	if (i % 2){
	  target.x = target.x + 2 * zigzag_distance * u_x3;
	  target.y = target.y + 2 * zigzag_distance * u_y3;
	  }
	else {
	  target.x = target.x + 2 * zigzag_distance * u_x2;
	  target.y = target.y + 2 * zigzag_distance * u_y2;  
	  }
      }
      nav_route_xy(target_old.x, target_old.y, target.x, target.y);
      if (nav_approaching_xy(target.x, target.y, target_old.x, target_old.y, CARROT)) {
        zigzag_status = P21;
	zigzag_count++;
        nav_init_stage();
      }
      return TRUE;

    case P21 :
      for (i=1; i<zigzag_count; i++){
       target_old.x = target.x;
       target_old.y = target.y;
	if (i % 2){
	  target.x = target.x + 2 * zigzag_distance * u_x3;
	  target.y = target.y + 2 * zigzag_distance * u_y3;
	  }
	else {
	  target.x = target.x + 2 * zigzag_distance * u_x2;
	  target.y = target.y + 2 * zigzag_distance * u_y2;  
	  }
      }
      nav_route_xy(target_old.x, target_old.y, target.x, target.y);
      if (nav_approaching_xy(target.x, target.y, target_old.x, target_old.y, CARROT)) {
        zigzag_status = P12;
	zigzag_count++;
        nav_init_stage();
      }
      return TRUE;

    default:/* Should not occur !!! Doing nothing */
      return TRUE;
  } /* switch */
  return TRUE;  
}