/*
 * hud_dbg.c
 *
 *  Created on: Dec 27, 2013
 *      Author: petera
 */

#include "hud.h"
#include "gfx_bitmap.h"
#include "cvideo.h"
#include "miniutils.h"

static struct {
  u8_t dbg_cy;
  u8_t dbg_cx;
  bool cursor;
} dhud;


void hud_paint_dbg(gcontext *ctx, bool init) {
  if (init) {
    dhud.dbg_cx = 0;
    dhud.dbg_cy = 0;
    dhud.cursor = TRUE;
    CVIDEO_set_v_offset(((dhud.dbg_cy+1) * 8));
  }
  if (dhud.cursor) {
    if ((SYS_get_time_ms() / 500) & 1) {
      GFX_printn(ctx, " ", 1, dhud.dbg_cx, dhud.dbg_cy, COL_OVER);
    } else {
      GFX_printn(ctx, "\037", 1, dhud.dbg_cx, dhud.dbg_cy, COL_OVER);
    }
  }
}


void HUD_dbg_print(gcontext *ctx, char *str) {
  dhud.cursor = TRUE;
  int len = strlen(str);
  while (len > 0) {
    const char *nl = strchr(str, '\n');
    int rem = (ctx->width / 8) - dhud.dbg_cx;
    int rlen = MIN(rem, len);
    if (nl) {
      rlen = MIN(nl - str, rlen);
    }
    if (dhud.dbg_cx == 0) {
      GFX_fill(ctx, 0, dhud.dbg_cy * 8, ctx->width, 8, COL_RESET);
    }
    GFX_printn(ctx, str, rlen, dhud.dbg_cx, dhud.dbg_cy, COL_OVER);
    if (nl) {
      rlen++;
    }
    str += rlen;
    len -= rlen;
    dhud.dbg_cx += rlen;
    if (dhud.dbg_cx >= ctx->width/8 || nl) {
      dhud.dbg_cx = 0;
      dhud.dbg_cy++;
      CVIDEO_set_v_offset(((dhud.dbg_cy+1) * 8));
      if (dhud.dbg_cy >= (ctx->height/8)) {
        dhud.dbg_cy = 0;
      }
      GFX_fill(ctx, 0, dhud.dbg_cy * 8, ctx->width, 8, COL_RESET);
    }
  }
}

void HUD_dbg_print_same_line(gcontext *ctx, char *str) {
  dhud.cursor = FALSE;
  int len = strlen(str);
  GFX_fill(ctx, 0, dhud.dbg_cy * 8, ctx->width, 8, COL_RESET);
  GFX_printn(ctx, str, MIN(ctx->width / 8, len), 0, dhud.dbg_cy, COL_OVER);
  dhud.dbg_cx = 0;
}
