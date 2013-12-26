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

typedef void (*vbl_cb)(void);

void CVIDEO_init(vbl_cb fn);
void CVIDEO_set_v_offset(u16_t hoffset);
void CVIDEO_set_v_scroll(s16_t speed);
void CVIDEO_set_effect(u8_t effect);
void CVIDEO_init_gcontext(gcontext *gctx);
void CVIDEO_gram_double_buffer(bool enable);
void CVIDEO_gram_switch(void);

#endif /* CVIDEO_H_ */
