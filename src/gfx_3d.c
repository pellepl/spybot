/*
 * gfx_3d.c
 *
 *  Created on: Dec 26, 2013
 *      Author: petera
 */

#include "gfx_3d.h"

void GFX_project_3d(s16_t *vec, s16_t magx, s16_t magy, s16_t offsx, s16_t offsy) {
  vec[0] = (((s32_t)magx*(vec[0] << 16) / (vec[2]+64)) >> 16) + offsx;
  vec[1] = (((s32_t)magy*((vec[1]+16) << 16) / (vec[2]+64)) >> 16) + offsy;
}

void GFX_rotate_x_3d(s16_t *v, s32_t cos, s32_t sin) {
  //s32_t x = 1   * v[0]  +  0   * v[1]   +  0   * v[2];
  s32_t y = 0   * v[0]  +  cos * v[1]   -  sin * v[2];
  s32_t z = 0   * v[0]  +  sin * v[1]   +  cos * v[2];
  //v[0] = (x >> 15);
  v[1] = (y >> 15);
  v[2] = (z >> 15);
}

void GFX_rotate_y_3d(s16_t *v, s32_t cos, s32_t sin) {
  s32_t x = cos * v[0]  +  0   * v[1]   -  sin   * v[2];
  //s32_t y = 0   * v[0]  +  1   * v[1]   +  0    * v[2];
  s32_t z = sin * v[0]  +  0   * v[1]   +  cos * v[2];
  v[0] = (x >> 15);
  //v[1] = (y >> 15);
  v[2] = (z >> 15);
}

void GFX_rotate_z_3d(s16_t *v, s32_t cos, s32_t sin) {
  s32_t x = cos * v[0]  -  sin * v[1]   +  0   * v[2];
  s32_t y = sin * v[0]  +  cos * v[1]   +  0   * v[2];
  //s32_t z = 0   * v[0]  +  0   * v[1]   +  1   * v[2];
  v[0] = (x >> 15);
  v[1] = (y >> 15);
  //v[2] = (z >> 15);
}

void GFX_translate_3d(s16_t *v, s32_t dx, s32_t dy, s32_t dz) {
  v[0] += dx;
  v[1] += dy;
  v[2] += dz;
}
