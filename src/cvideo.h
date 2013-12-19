/*
 * cvideo.h
 *
 *  Created on: Nov 30, 2013
 *      Author: petera
 */

#ifndef CVIDEO_H_
#define CVIDEO_H_

#include "system.h"
#include "gfx_bitmap.h"

void CVIDEO_dump(void);
void CVIDEO_init(void);
void CVIDEO_set_v_offset(u16_t hoffset);
void CVIDEO_set_v_scroll(s16_t speed);
void CVIDEO_set_effect(u8_t effect);
void CVIDEO_init_gcontext(gcontext *gctx);

#endif /* CVIDEO_H_ */
