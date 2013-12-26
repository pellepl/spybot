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

void ROVER_paint(gcontext *ctx);
void ROVER_view(s16_t x, s16_t y, s16_t z, s16_t ax, s16_t ay, s16_t az);

#endif /* ROVER_3D_H_ */
