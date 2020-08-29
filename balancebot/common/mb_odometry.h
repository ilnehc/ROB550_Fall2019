/*******************************************************************************
* mb_odometry.h
*
* 
*******************************************************************************/
#ifndef MB_ODOMETRY_H
#define MB_ODOMETRY_H

#include "mb_defs.h"
#include "mb_structs.h"
#include <math.h>

//state variables
float prev_x; // previously known x position (m)
float prev_y; // previously known y position (m)
float prev_gamma; // previously known heading (rad)
int prev_Rw_enc; // previous value of right wheel encoder
int prev_Lw_enc; // previous value of left wheel encoder
float prev_gyro_heading; // previous value of gyroscope heading
float prev_phi; //previously known wheel angle traveled
float gamma_offset; //offset for gamma  (for initialization and slip correction)

void mb_odometry_init(mb_odometry_t* mb_odometry, float x, float y, float gamma);
void mb_odometry_update(mb_odometry_t* mb_odometry, mb_state_t* mb_state);
float mb_clamp_radians(float angle);
int mb_in_range(float num, float min, float max);

#endif