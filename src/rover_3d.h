/*
 * rover_3d.h
 *
 *  Created on: Dec 26, 2013
 *      Author: petera
 */

#ifndef ROVER_3D_H_
#define ROVER_3D_H_

#include "system.h"
#include "gfx_bitmap.h"

typedef struct {
  s16_t pan;
  s16_t tilt;
  s16_t radar;
  s16_t wheel_left;
  s16_t wheel_right;

  s16_t anim_d_radar;
  s16_t anim_d_wheel_left;
  s16_t anim_d_wheel_right;
} rover_angles;

void ROVER_init(void);
void ROVER_paint(gcontext *ctx);
void ROVER_view(s16_t x, s16_t y, s16_t z, s16_t ax, s16_t ay, s16_t az, bool direct);
rover_angles *ROVER_angle_config(void);

#endif /* ROVER_3D_H_ */
