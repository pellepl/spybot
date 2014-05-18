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

typedef enum {
  INPUT_CAMERA = 0,
  INPUT_GENERATED
} video_input_t;

void CVIDEO_init(vbl_cb fn);
void CVIDEO_set_v_offset(u16_t hoffset);
void CVIDEO_set_v_scroll(s16_t speed);
void CVIDEO_set_effect(u8_t effect);
void CVIDEO_init_gcontext(gcontext *gctx);
void CVIDEO_gram_double_buffer(bool enable);
void CVIDEO_gram_switch(void);
void CVIDEO_set_input(video_input_t input);
video_input_t CVIDEO_get_input(void);

#endif /* CVIDEO_H_ */
