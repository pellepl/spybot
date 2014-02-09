/*
 * gfx_bitmap.h
 *
 *  Created on: Dec 2, 2013
 *      Author: petera
 */

#ifndef GFX_BITMAP_H_
#define GFX_BITMAP_H_

#include "system.h"

typedef struct {
  u16_t width;
  u16_t height;
  u16_t hstride;
  u8_t *gram;
} gcontext;

typedef enum {
  COL_RESET = 0,
  COL_SET,
  COL_MIX,
  COL_OVER
} gcolor;

void GFX_put_pixel(gcontext *ctx, s16_t x, s16_t y, gcolor col);
void GFX_fill(gcontext *ctx, s16_t x, s16_t y, u16_t width, u16_t height, gcolor col);
void GFX_printn(gcontext *ctx, const char *s, int len, u8_t cx, u8_t cy, gcolor col);
void GFX_printn_big(gcontext *ctx, const char *str, int len, u8_t cx, u8_t cy, gcolor c);
//void GFX_draw_string(gcontext *ctx, s8_t *s, s16_t x, s16_t y, gcolor col);
void GFX_draw_line(gcontext *ctx, s16_t x1, s16_t y1, s16_t x2, s16_t y2, gcolor col);
void GFX_rect(gcontext *ctx, s16_t x, s16_t y, s16_t w, s16_t h, gcolor col);
void GFX_draw_horizontal_line(gcontext *ctx, s16_t x1, s16_t x2, s16_t y, gcolor col);
void GFX_draw_vertical_line(gcontext *ctx, s16_t x, s16_t y1, s16_t y2, gcolor col);
void GFX_draw_image_q(gcontext *ctx, const u8_t const *img_data, u8_t cx, u8_t y, u16_t w, u16_t h);
void GFX_draw_image(gcontext *ctx, const u8_t *img,
    s16_t x, s16_t y,
    s16_t w, s16_t h,
    s16_t img_offs_x, s16_t img_offs_y, u16_t img_stride);

#endif /* GFX_BITMAP_H_ */
