/*
 * gfx_bitmap.c
 *
 *  Created on: Dec 2, 2013
 *      Author: petera
 */

#include "gfx_bitmap.h"

extern unsigned char font_spybotfont_png[][8];

static u8_t snippet[8] = {
    0b10000000,
    0b11000000,
    0b11100000,
    0b11110000,
    0b11111000,
    0b11111100,
    0b11111110,
    0b11111111,
};

static inline void _gfx_draw_horizontal_line(gcontext *ctx, s16_t x, s16_t w, s16_t y, gcolor c) {
  if (w == 0) return;
  if ((x & 0xfff8) == ((x+w) & 0xfff8)) {
    u8_t g = snippet[w-1];
    u8_t *ga = ctx->gram + (x/8) + (y*ctx->hscan);
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
    u8_t *ga = ctx->gram + (x/8) + (y*ctx->hscan);
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
      u8_t *ga = ctx->gram + (x/8) + (y*ctx->hscan);
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
    u8_t *ga = ctx->gram + (x/8) + (y*ctx->hscan);
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
  u8_t *ga = ctx->gram + (x/8) + (y*ctx->hscan);
  u8_t g = 1<<(7-(x&7));
  if (c == COL_RESET) {
    g = ~g;
    while (h > 0) {
      switch (h) {
      default:
        *ga &= g;
        ga += ctx->hscan;
        //no break
      case 3:
        *ga &= g;
        ga += ctx->hscan;
        //no break
      case 2:
        *ga &= g;
        ga += ctx->hscan;
        //no break
      case 1:
        *ga &= g;
        ga += ctx->hscan;
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
        ga += ctx->hscan;
        //no break
      case 3:
        *ga ^= g;
        ga += ctx->hscan;
        //no break
      case 2:
        *ga ^= g;
        ga += ctx->hscan;
        //no break
      case 1:
        *ga ^= g;
        ga += ctx->hscan;
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
        ga += ctx->hscan;
        //no break
      case 3:
        *ga |= g;
        ga += ctx->hscan;
        //no break
      case 2:
        *ga |= g;
        ga += ctx->hscan;
        //no break
      case 1:
        *ga |= g;
        ga += ctx->hscan;
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

void GFX_draw_line(gcontext *ctx, s16_t x1, s16_t y1, s16_t x2, s16_t y2, gcolor col) {
  // todo: clipping
  u16_t dx = ABS(x2 - x1);
  u16_t dy = ABS(y2 - y1);
  if (dx >= dy) {
    // non-steep
    u32_t fk = (dx << 8) / (1+dy);
    u32_t fx1, fx2;
    s16_t ys, yd;
    if (x1 < x2) {
      fx1 = (x1 << 8);
      fx2 = (x2 << 8);
      ys = y1;
      yd = y2 > y1 ? 1 : -1;
    } else {
      fx1 = (x2 << 8);
      fx2 = (x1 << 8);
      ys = y2;
      yd = y1 > y2 ? 1 : -1;
    }
    fx1 += 0x7f;
    fx2 += 0x7f;
    u32_t ox = fx1>>8;
    while (fx1 < fx2) {
      fx1 += fk;
      if (fx1 > fx2) fx1 = fx2;
      _gfx_draw_horizontal_line(ctx, ox, (fx1 >> 8) - ox, ys, col);
      ys += yd;
      ox = fx1 >> 8;
    }
  } else {
    // steep
    u32_t fk = (dy << 8) / (1+dx);
    u32_t fy1, fy2;
    s16_t xs, xd;
    if (y1 < y2) {
      fy1 = (y1 << 8);
      fy2 = (y2 << 8);
      xs = x1;
      xd = x2 > x1 ? 1 : -1;
    } else {
      fy1 = (y2 << 8);
      fy2 = (y1 << 8);
      xs = x2;
      xd = x1 > x2 ? 1 : -1;
    }
    fy1 += 0x7f;
    fy2 += 0x7f;
    u32_t oy = fy1>>8;
    while (fy1 < fy2) {
      fy1 += fk;
      if (fy1 > fy2) fy1 = fy2;
      _gfx_draw_vertical_line(ctx, oy, (fy1 >> 8) - oy, xs, col);
      xs += xd;
      oy = fy1 >> 8;
    }
  }
}

void GFX_rect(gcontext *ctx, s16_t x, s16_t y, s16_t w, s16_t h, gcolor col) {
  GFX_draw_horizontal_line(ctx, x, x+w, y, col);
  GFX_draw_horizontal_line(ctx, x, x+w, y+h, col);
  GFX_draw_vertical_line(ctx, x, y, y+h, col);
  GFX_draw_vertical_line(ctx, x+w, y, y+h+1, col);
}

void GFX_print(gcontext *ctx, char *str, u8_t cx, u8_t cy, gcolor c) {
  u8_t ch;
  u8_t cw = ctx->width/8;
  if (cy*8+8 > ctx->height) {
    return;
  }

  while (cx < cw && (ch = *str++) != 0) {
    if (cx >= 0) {
      u8_t y;
      u8_t *ga = ctx->gram + cx + (cy*8*ctx->hscan);
      switch (c) {
      case COL_SET:
        for (y = 0; y < 8; y++) {
          u8_t g = font_spybotfont_png[ch][y];
          if (g) {
            *ga |= g;
          }
          ga += ctx->hscan;
        }
        break;
      case COL_RESET:
        for (y = 0; y < 8; y++) {
          u8_t g = font_spybotfont_png[ch][y];
          if (g) {
            *ga &= ~g;
          }
          ga += ctx->hscan;
        }
        break;
      case COL_MIX:
        for (y = 0; y < 8; y++) {
          u8_t g = font_spybotfont_png[ch][y];
          if (g) {
            *ga ^= g;
          }
          ga += ctx->hscan;
        }
        break;
      case COL_OVER:
        for (y = 0; y < 8; y++) {
          u8_t g = font_spybotfont_png[ch][y];
          *ga = g;
          ga += ctx->hscan;
        }
        break;
      }
    } else {
      str++;
    }
    cx++;
  }
}

void GFX_put_pixel(gcontext *ctx, s16_t x, s16_t y, gcolor c) {
  if (x < 0 || x >= ctx->width || y < 0 || y >= ctx->height)
    return;
  switch (c) {
  case COL_RESET:
    ctx->gram[y*ctx->hscan + x/8] &= ~(1<<(7-(x&7)));
    break;
  case COL_MIX:
    ctx->gram[y*ctx->hscan + x/8] ^= (1<<(7-(x&7)));
    break;
  default:
    ctx->gram[y*ctx->hscan + x/8] |= (1<<(7-(x&7)));
    break;
  }
}


void GFX_fill(gcontext *ctx, s16_t x, s16_t y, u16_t w, u16_t h, gcolor c) {
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
  if (y + h >= ctx->height) {
    h = ctx->height - y - 1;
  }

  if (h == 0 || w == 0) return;

  if ((x & 0xfff8) == ((x+w) & 0xfff8)) {
    u8_t g = snippet[w-1];
    u8_t *ga = ctx->gram + (x/8) + (y*ctx->hscan);
    g >>= (x & 7);
    u16_t th = 0;
    if (c == COL_RESET) {
      while (th < h) {
        *ga &= ~g;
        th++;
        ga += ctx->hscan;
      }
    } else {
      while (th < h) {
        *ga |= g;
        th++;
        ga += ctx->hscan;
      }
    }

    return;
  }


  // left unaligned part
  if ((x&7) != 0) {
    u8_t left = 0xff >> (x & 7);
    u8_t *ga = ctx->gram + (x/8) + (y*ctx->hscan);
    u16_t th = 0;
    if (c == COL_RESET) {
      // reset
      left = ~left;
      while (th < h) {
        *ga &= left;
        th++;
        ga += ctx->hscan;
      }
    } else {
      // set
      while (th < h) {
        *ga |= left;
        th++;
        ga += ctx->hscan;
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
      u8_t *ga = ctx->gram + (x/8) + (y*ctx->hscan);
      u16_t th = 0;
      u8_t tc = c == COL_RESET ? 0x00 : 0xff;

      while (th < h) {
        memset(ga, tc, tw);
        th++;
        ga += ctx->hscan;
      }

      w -= tw * 8;
      x += tw * 8;
    }
  }

  // right unaligned part
  if (w) {
    u8_t right = 0xff << (8 - w);
    u8_t *ga = ctx->gram + (x/8) + (y*ctx->hscan);
    u16_t th = 0;
    if (c == COL_RESET) {
      // reset
      right = ~right;
      while (th < h) {
        *ga &= right;
        th++;
        ga += ctx->hscan;
      }
    } else {
      // set
      while (th < h) {
        *ga |= right;
        th++;
        ga += ctx->hscan;
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

void GFX_draw_image(gcontext *ctx, const u8_t const *img_data, u8_t cx, u8_t y, u16_t w, u16_t h) {
  while (h--) {
    u16_t xx;
    u8_t *ga = ctx->gram + (cx) + (y*ctx->hscan);
    for (xx = 0; xx < w; xx++) {
      *ga++ = *img_data++;
    }
    y++;
  }
}


