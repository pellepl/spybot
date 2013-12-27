/*
 * hud.c
 *
 *  Created on: Dec 10, 2013
 *      Author: petera
 */

#include "hud.h"
#include "gfx_bitmap.h"
#include "gfx_3d.h"
#include "trig_q.h"
#include "miniutils.h"
#include "cvideo.h"
#include "rover_3d.h"
#include "cvideo.h"

static struct {
  gcontext *ctx;
  hud_state state;
  hud_state cstate;
  lsm303_dev *lsm;
} hud;

void HUD_state(hud_state state) {
  if (hud.state != state) {
    if (state == HUD_MAIN  || state == HUD_CONFIG) {
      CVIDEO_gram_double_buffer(TRUE);
    } else {
      CVIDEO_gram_double_buffer(FALSE);
    }
    CVIDEO_set_v_offset(0);
    GFX_fill(hud.ctx, 0, 0, hud.ctx->width, hud.ctx->height, COL_RESET);
    hud.state = state;
    CVIDEO_set_effect(79);

    HUD_paint();
  }
}

void HUD_init(gcontext *ctx, lsm303_dev *lsm_dev) {
  memset(&hud, 0, sizeof(hud));
  hud.ctx = ctx;
  hud.lsm = lsm_dev;
  ROVER_init();
}

void HUD_paint(void) {
  if (hud.state == HUD_MAIN) {
    GFX_fill(hud.ctx, 0, 0, hud.ctx->width, hud.ctx->height, COL_RESET);
    hud_paint_main(hud.ctx, hud.lsm, hud.state == hud.cstate);
  } else if (hud.state == HUD_CONFIG) {
    GFX_fill(hud.ctx, 0, 0, hud.ctx->width, hud.ctx->height, COL_RESET);
    hud_paint_config(hud.ctx, hud.state == hud.cstate);
  } else if (hud.state == HUD_DBG) {
    hud_paint_dbg(hud.ctx, hud.state == hud.cstate);
  }
  CVIDEO_gram_switch();
  hud.cstate = hud.state;
}

void HUD_vbl(void) {
}
