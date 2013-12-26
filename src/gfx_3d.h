/*
 * gfx_3d.h
 *
 *  Created on: Dec 26, 2013
 *      Author: petera
 */

#ifndef GFX_3D_H_
#define GFX_3D_H_

#include "system.h"

void GFX_project_3d(s16_t *vec, s16_t magx, s16_t magy, s16_t offsx, s16_t offsy);
void GFX_rotate_x_3d(s16_t *v, s32_t cos_q15, s32_t sin_q15);
void GFX_rotate_y_3d(s16_t *v, s32_t cos_q15, s32_t sin_q15);
void GFX_rotate_z_3d(s16_t *v, s32_t cos_q15, s32_t sin_q15);
void GFX_translate_3d(s16_t *v, s32_t dx, s32_t dy, s32_t dz);

#endif /* GFX_3D_H_ */
