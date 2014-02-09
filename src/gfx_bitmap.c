/*
 * gfx_bitmap.c
 *
 *  Created on: Dec 2, 2013
 *      Author: petera
 */

#include "gfx_bitmap.h"

extern unsigned char font_spybotfont_png[][8];

static const u8_t const snippet[8] = {
    0b10000000,
    0b11000000,
    0b11100000,
    0b11110000,
    0b11111000,
    0b11111100,
    0b11111110,
    0b11111111,
};
static const u8_t const snippet_rev[8] = {
    0b00000001,
    0b00000011,
    0b00000111,
    0b00001111,
    0b00011111,
    0b00111111,
    0b01111111,
    0b11111111,
};

static const u8_t const morton_nibble[16] = {
    0b00000000,
    0b00000011,
    0b00001100,
    0b00001111,
    0b00110000,
    0b00110011,
    0b00111100,
    0b00111111,
    0b11000000,
    0b11000011,
    0b11001100,
    0b11001111,
    0b11110000,
    0b11110011,
    0b11111100,
    0b11111111
};

#define INSIDE 0
#define LEFT   1
#define RIGHT  2
#define BOTTOM 4
#define TOP    8

static inline void _gfx_draw_horizontal_line(gcontext *ctx, s16_t x, s16_t w, s16_t y, gcolor c) {
  if (w == 0) return;
  if ((x & 0xfff8) == ((x+w) & 0xfff8)) {
    u8_t g = snippet[w-1];
    u8_t *ga = ctx->gram + (x/8) + (y*ctx->hstride);
    g >>= (x & 7);
    if (c == COL_RESET) {
      *ga &= ~g;
    } else if (c == COL_MIX) {
      *ga ^= g;
    } else {
      *ga |= g;
    }

    return;
  }
  // left unaligned part
  if ((x&7) != 0) {
    u8_t left = 0xff >> (x & 7);
    u8_t *ga = ctx->gram + (x/8) + (y*ctx->hstride);
    if (c == COL_RESET) {
      // reset
      *ga &= ~left;
    } else if (c == COL_MIX) {
      *ga ^= left;
    } else {
      // set
      *ga |= left;
    }

    w -= 8 - (x & 7);
    x += 8 - (x & 7);
  }

  if (w == 0) return;

  // middle line
  {
    u16_t tw = (w & 0xfff8) / 8;
    if (tw > 0) {
      u8_t *ga = ctx->gram + (x/8) + (y*ctx->hstride);
      w -= tw * 8;
      x += tw * 8;
      if (c == COL_RESET) {
        while (tw--) {
          *ga++ = 0x00;
        }
      } else if (c == COL_MIX) {
        while (tw--) {
          *ga++ ^= 0xff;
        }
      } else {
        while (tw--) {
          *ga++ = 0xff;
        }
      }
    }
  }

  // right unaligned part
  if (w) {
    u8_t right = 0xff << (8 - w);
    u8_t *ga = ctx->gram + (x/8) + (y*ctx->hstride);
    if (c == COL_RESET) {
      // reset
      *ga &= ~right;
    } else if (c == COL_MIX) {
      *ga ^= right;
    } else {
      *ga |= right;
    }
  }

}

static inline void _gfx_draw_vertical_line(gcontext *ctx, s16_t y, s16_t h, s16_t x, gcolor c) {
  if (h == 0) return;
  u8_t *ga = ctx->gram + (x/8) + (y*ctx->hstride);
  u8_t g = 1<<(7-(x&7));
  if (c == COL_RESET) {
    g = ~g;
    while (h > 0) {
      switch (h) {
      default:
        *ga &= g;
        ga += ctx->hstride;
        //no break
      case 3:
        *ga &= g;
        ga += ctx->hstride;
        //no break
      case 2:
        *ga &= g;
        ga += ctx->hstride;
        //no break
      case 1:
        *ga &= g;
        ga += ctx->hstride;
        //no break
      }
      if (h >= 4) {
        h -= 4;
      } else {
        h -= (h & 3);
      }
    }
  } else if (c == COL_MIX) {
    while (h > 0) {
      switch (h) {
      default:
        *ga ^= g;
        ga += ctx->hstride;
        //no break
      case 3:
        *ga ^= g;
        ga += ctx->hstride;
        //no break
      case 2:
        *ga ^= g;
        ga += ctx->hstride;
        //no break
      case 1:
        *ga ^= g;
        ga += ctx->hstride;
        //no break
      }
      if (h >= 4) {
        h -= 4;
      } else {
        h -= (h & 3);
      }
    }
  } else {
    while (h > 0) {
      switch (h) {
      default:
        *ga |= g;
        ga += ctx->hstride;
        //no break
      case 3:
        *ga |= g;
        ga += ctx->hstride;
        //no break
      case 2:
        *ga |= g;
        ga += ctx->hstride;
        //no break
      case 1:
        *ga |= g;
        ga += ctx->hstride;
        //no break
      }
      if (h >= 4) {
        h -= 4;
      } else {
        h -= (h & 3);
      }
    }
  }
}

static u8_t gfx_clip_code(gcontext *ctx, s32_t x, s32_t y) {
  u8_t code;

  code = INSIDE;
  if (x < 0) {
    code |= LEFT;
  } else if (x >= ctx->width) {
    code |= RIGHT;
  }
  if (y < 0) {
    code |= BOTTOM;
  } else if (y >= ctx->height) {
    code |= TOP;
  }

  return code;
}

#define LINE_ACC    14
#define LINE_ROUND  ((1<<(LINE_ACC-1))-1)
void GFX_draw_line(gcontext *ctx, s16_t xq1, s16_t yq1, s16_t xq2, s16_t yq2, gcolor col) {
  s32_t x1 = xq1;
  s32_t y1 = yq1;
  s32_t x2 = xq2;
  s32_t y2 = yq2;
  s32_t dx = x2 - x1;
  s32_t dy = y2 - y1;
  s32_t fkx = ((dx << LINE_ACC) + LINE_ROUND) / (dy == 0 ? 1 : dy);
  s32_t fky = ((dy << LINE_ACC) + LINE_ROUND) / (dx == 0 ? 1 : dx);

  { // cohen sutherland clipping
    u8_t clip1 = gfx_clip_code(ctx, x1, y1);
    u8_t clip2 = gfx_clip_code(ctx, x2, y2);
    bool accept = FALSE;
    while (1) {
      if (!(clip1 | clip2)) {
        accept = TRUE;
        break;
      } else if (clip1 & clip2) {
        break;
      } else {
        s32_t x = 0, y = 0;
        u8_t clip = clip1 ? clip1 : clip2;
        if (clip & TOP) {
          x = x1 + (((ctx->height-1 - y1) * fkx) >> LINE_ACC);
          y = ctx->height-1;
        } else if (clip & BOTTOM) {
          x = x1 + (((0 - y1) * fkx) >> LINE_ACC);
          y = 0;
        } else if (clip & RIGHT) {
          y = y1 + (((ctx->width-1 - x1) * fky) >> LINE_ACC);
          x = ctx->width-1;
        } else if (clip & LEFT) {
          y = y1 + (((0 - x1) * fky) >> LINE_ACC);
          x = 0;
        }
        if (clip == clip1) {
          x1 = x;
          y1 = y;
          clip1 = gfx_clip_code(ctx, x1, y1);
        } else {
          x2 = x;
          y2 = y;
          clip2 = gfx_clip_code(ctx, x2, y2);
        }
      }
    }
    if (!accept) return;
  }

  if (dx == 0) {
    _gfx_draw_vertical_line(ctx, MIN(y1,y2), MAX(y1,y2)-MIN(y1,y2), x1, col);
    return;
  }
  if (dy == 0) {
    _gfx_draw_horizontal_line(ctx, MIN(x1,x2), MAX(x1,x2)-MIN(x1,x2), y1, col);
    return;
  }

  dx = ABS(dx);
  dy = ABS(dy);
  fkx = ABS(fkx);
  fky = ABS(fky);

  if (dx >= dy) {
    // non-steep
    u32_t fx1, fx2;
    s32_t ys, yd;
    if (x1 < x2) {
      fx1 = (x1 << LINE_ACC);
      fx2 = (x2 << LINE_ACC);
      ys = y1;
      yd = y2 > y1 ? 1 : -1;
    } else {
      fx1 = (x2 << LINE_ACC);
      fx2 = (x1 << LINE_ACC);
      ys = y2;
      yd = y1 > y2 ? 1 : -1;
    }
    //fx1 += LINE_ROUND;
    //fx2 += LINE_ROUND;
    u32_t ox = fx1>>LINE_ACC;
    while (fx1 <= fx2) {
      fx1 += fkx;
      if (fx1 > fx2 || fkx == 0) {
        fx1 = fx2;
        _gfx_draw_horizontal_line(ctx, ox, ((fx1+LINE_ROUND) >> LINE_ACC) - ox, ys, col);
        return;
      }
      _gfx_draw_horizontal_line(ctx, ox, ((fx1+LINE_ROUND) >> LINE_ACC) - ox, ys, col);
      ys += yd;
      ox = (fx1+LINE_ROUND) >> LINE_ACC;
    }
  } else {
    // steep
    u32_t fy1, fy2;
    s32_t xs, xd;
    if (y1 < y2) {
      fy1 = (y1 << LINE_ACC);
      fy2 = (y2 << LINE_ACC);
      xs = x1;
      xd = x2 > x1 ? 1 : -1;
    } else {
      fy1 = (y2 << LINE_ACC);
      fy2 = (y1 << LINE_ACC);
      xs = x2;
      xd = x1 > x2 ? 1 : -1;
    }
    //fy1 += LINE_ROUND;
    //fy2 += LINE_ROUND;
    u32_t oy = fy1>>LINE_ACC;
    while (fy1 <= fy2) {
      fy1 += fky;
      if (fy1 > fy2 || fky == 0) {
        fy1 = fy2;
        _gfx_draw_vertical_line(ctx, oy, ((fy1+LINE_ROUND) >> LINE_ACC) - oy, xs, col);
        return;
      }
      _gfx_draw_vertical_line(ctx, oy, ((fy1+LINE_ROUND) >> LINE_ACC) - oy, xs, col);
      xs += xd;
      oy = (fy1+LINE_ROUND) >> LINE_ACC;
    }
  }
}

void GFX_rect(gcontext *ctx, s16_t x, s16_t y, s16_t w, s16_t h, gcolor col) {
  GFX_draw_horizontal_line(ctx, x, x+w, y, col);
  GFX_draw_horizontal_line(ctx, x, x+w, y+h, col);
  GFX_draw_vertical_line(ctx, x, y, y+h, col);
  GFX_draw_vertical_line(ctx, x+w, y, y+h+1, col);
}

void GFX_printn(gcontext *ctx, const char *str, int len, u8_t cx, u8_t cy, gcolor c) {
  u8_t ch;
  u8_t cw = ctx->width/8;
  if (cy*8+8 > ctx->height) {
    return;
  }

  if (len) len++;

  while (cx < cw && (ch = *str++) != 0 && (len == 0 || --len > 0)) {
    if (cx >= 0) {
      u8_t y;
      u8_t *ga = ctx->gram + cx + (cy*8*ctx->hstride);
      switch (c) {
      case COL_SET:
        for (y = 0; y < 8; y++) {
          u8_t g = font_spybotfont_png[ch][y];
          *ga |= g;
          ga += ctx->hstride;
        }
        break;
      case COL_RESET:
        for (y = 0; y < 8; y++) {
          u8_t g = font_spybotfont_png[ch][y];
          *ga &= ~g;
          ga += ctx->hstride;
        }
        break;
      case COL_MIX:
        for (y = 0; y < 8; y++) {
          u8_t g = font_spybotfont_png[ch][y];
          *ga ^= g;
          ga += ctx->hstride;
        }
        break;
      case COL_OVER:
        for (y = 0; y < 8; y++) {
          u8_t g = font_spybotfont_png[ch][y];
          *ga = g;
          ga += ctx->hstride;
        }
        break;
      }
    } else {
      str++;
    }
    cx++;
  }
}

void GFX_printn_big(gcontext *ctx, const char *str, int len, u8_t cx, u8_t cy, gcolor c) {
  u8_t ch;
  u8_t cw = ctx->width/8;
  if (cy*8+16 > ctx->height) {
    return;
  }

  if (len) len++;

  while (cx < cw-1 && (ch = *str++) != 0 && (len == 0 || --len > 0)) {
    if (cx >= 0) {
      u8_t y;
      u8_t *ga = ctx->gram + cx + (cy*8*ctx->hstride);
      switch (c) {
      case COL_SET:
        for (y = 0; y < 8; y++) {
          u8_t g = font_spybotfont_png[ch][y];
          u8_t g1 = morton_nibble[(g&0xf0)>>4];
          u8_t g2 = morton_nibble[g&0x0f];
          *ga++ |= g1;
          *ga |= g2;
          ga += ctx->hstride-1;
          *ga++ |= g1;
          *ga |= g2;
          ga += ctx->hstride-1;
        }
        break;
      case COL_RESET:
        for (y = 0; y < 8; y++) {
          u8_t g = font_spybotfont_png[ch][y];
          u8_t g1 = morton_nibble[(g&0xf0)>>4];
          u8_t g2 = morton_nibble[g&0x0f];
          *ga++ &= ~g1;
          *ga &= ~g2;
          ga += ctx->hstride-1;
          *ga++ &= ~g1;
          *ga &= ~g2;
          ga += ctx->hstride-1;
        }
        break;
      case COL_MIX:
        for (y = 0; y < 8; y++) {
          u8_t g = font_spybotfont_png[ch][y];
          u8_t g1 = morton_nibble[(g&0xf0)>>4];
          u8_t g2 = morton_nibble[g&0x0f];
          *ga++ ^= g1;
          *ga ^= g2;
          ga += ctx->hstride-1;
          *ga++ ^= g1;
          *ga ^= g2;
          ga += ctx->hstride-1;
        }
        break;
      case COL_OVER:
        for (y = 0; y < 8; y++) {
          u8_t g = font_spybotfont_png[ch][y];
          u8_t g1 = morton_nibble[(g&0xf0)>>4];
          u8_t g2 = morton_nibble[g&0x0f];
          *ga++ = g1;
          *ga = g2;
          ga += ctx->hstride-1;
          *ga++ = g1;
          *ga = g2;
          ga += ctx->hstride-1;
        }
        break;
      }
    } else {
      str++;
    }
    cx += 2;
  }
}

void GFX_put_pixel(gcontext *ctx, s16_t x, s16_t y, gcolor c) {
  if (x < 0 || x >= ctx->width || y < 0 || y >= ctx->height)
    return;
  switch (c) {
  case COL_RESET:
    ctx->gram[y*ctx->hstride + x/8] &= ~(1<<(7-(x&7)));
    break;
  case COL_MIX:
    ctx->gram[y*ctx->hstride + x/8] ^= (1<<(7-(x&7)));
    break;
  default:
    ctx->gram[y*ctx->hstride + x/8] |= (1<<(7-(x&7)));
    break;
  }
}


void GFX_fill(gcontext *ctx, s16_t x, s16_t y, u16_t w, u16_t h, gcolor c) {
  if (y > ctx->height || x > ctx->width) return;
  if (x < 0) {
    w += x;
    x = 0;
  }
  if (y < 0) {
    h += y;
    y = 0;
  }
  if (x + w >= ctx->width) {
    w = ctx->width - x - 1;
  }
  if (y + h > ctx->height) {
    h = ctx->height - y;
  }

  if (h == 0 || w == 0) return;

  if ((x & 0xfff8) == ((x+w) & 0xfff8)) {
    u8_t g = snippet[w-1];
    u8_t *ga = ctx->gram + (x/8) + (y*ctx->hstride);
    g >>= (x & 7);
    u16_t th = 0;
    if (c == COL_RESET) {
      while (th < h) {
        *ga &= ~g;
        th++;
        ga += ctx->hstride;
      }
    } else {
      while (th < h) {
        *ga |= g;
        th++;
        ga += ctx->hstride;
      }
    }

    return;
  }


  // left unaligned part
  if ((x&7) != 0) {
    u8_t left = 0xff >> (x & 7);
    u8_t *ga = ctx->gram + (x/8) + (y*ctx->hstride);
    u16_t th = 0;
    if (c == COL_RESET) {
      // reset
      left = ~left;
      while (th < h) {
        *ga &= left;
        th++;
        ga += ctx->hstride;
      }
    } else {
      // set
      while (th < h) {
        *ga |= left;
        th++;
        ga += ctx->hstride;
      }
    }

    w -= 8 - (x & 7);
    x += 8 - (x & 7);
  }

  if (w == 0) return;

  // middle blob
  {
    u16_t tw = (w & 0xfff8) / 8;
    if (tw > 0) {
      u8_t *ga = ctx->gram + (x/8) + (y*ctx->hstride);
      u16_t th = 0;
      u8_t tc = c == COL_RESET ? 0x00 : 0xff;

      while (th < h) {
        memset(ga, tc, tw);
        th++;
        ga += ctx->hstride;
      }

      w -= tw * 8;
      x += tw * 8;
    }
  }

  // right unaligned part
  if (w) {
    u8_t right = 0xff << (8 - w);
    u8_t *ga = ctx->gram + (x/8) + (y*ctx->hstride);
    u16_t th = 0;
    if (c == COL_RESET) {
      // reset
      right = ~right;
      while (th < h) {
        *ga &= right;
        th++;
        ga += ctx->hstride;
      }
    } else {
      // set
      while (th < h) {
        *ga |= right;
        th++;
        ga += ctx->hstride;
      }
    }
  }
}

void GFX_draw_horizontal_line(gcontext *ctx, s16_t x1, s16_t x2, s16_t y, gcolor c) {
  s16_t x = MIN(x1, x2);
  u16_t w = ABS(x2 - x1);
  if (x < 0) {
    w += x;
    x = 0;
  }
  if (y >= ctx->height || y < 0) {
    return;
  }
  if (x + w >= ctx->width) {
    w = ctx->width - x - 1;
  }

  _gfx_draw_horizontal_line(ctx, x, w, y, c);
}

void GFX_draw_vertical_line(gcontext *ctx, s16_t x, s16_t y1, s16_t y2, gcolor col) {
  s16_t y = MIN(y1, y2);
  u16_t h = ABS(y2 - y1);
  if (y < 0) {
    h += y;
    y = 0;
  }
  if (x >= ctx->width || x < 0) {
    return;
  }
  if (y + h >= ctx->height) {
    h = ctx->height - y - 1;
  }

  _gfx_draw_vertical_line(ctx, y, h, x, col);
}

void GFX_draw_image_q(gcontext *ctx, const u8_t const *img_data, u8_t cx, u8_t y, u16_t w, u16_t h) {
  while (h--) {
    u16_t xx;
    u8_t *ga = ctx->gram + (cx) + (y*ctx->hstride);
    for (xx = 0; xx < w; xx++) {
      *ga++ = *img_data++;
    }
    y++;
  }
}

void GFX_draw_image(gcontext *ctx, const u8_t *img,
    s16_t x, s16_t y,
    s16_t w, s16_t h,
    s16_t img_offs_x, s16_t img_offs_y, u16_t img_stride)
{
  u8_t *g = ctx->gram;
  g += y*ctx->hstride + (x/8);
  u8_t gfx_offs_p = (x & 7);
  u16_t img_offs_b = img_offs_x/8;
  u8_t img_offs_p = img_offs_x & 7;
  ///u8_t d_offs_p = (8 + (img_offs_p - gfx_offs_p)) & 7;
  u8_t d_offs_p;
  d_offs_p = (8 + (gfx_offs_p - img_offs_p)) & 7;
  if (gfx_offs_p >= img_offs_p) {
    img_offs_b = (img_offs_b + img_stride - 1) % img_stride;
  }
  while (h--) {
    u8_t img_d;
    s16_t px = 0;
    u16_t bx = 0;
    while (px < w) {
      if (d_offs_p == 0) {
        img_d = img[(bx+img_offs_b) % img_stride];
      } else {
        img_d =
            ((img[(bx+img_offs_b) % img_stride]) >> (d_offs_p)) |
            ((img[(bx+img_offs_b+img_stride-1) % img_stride]) << (8-d_offs_p));
      }
      if (px == 0 && gfx_offs_p > 0) {
        img_d &= snippet_rev[7-gfx_offs_p];
        px += gfx_offs_p;
      } else if (px + 8 > w) {
        img_d &= snippet[gfx_offs_p];
        px += 8;
      } else {
        px += 8;
      }
      g[bx] |= img_d;
      bx++;
    }
    g += ctx->hstride;
    img += img_stride;
  }

}


