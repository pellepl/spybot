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
  time last_vbl_check;
  u32_t vbl_count;
} hud;

void HUD_state(hud_state state) {
  if (hud.state != state) {
    if (state == HUD_MAIN  || state == HUD_CONFIG || state == HUD_TEST) {
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

void HUD_init(gcontext *ctx) {
  memset(&hud, 0, sizeof(hud));
  hud.ctx = ctx;
  ROVER_init();
}

void HUD_paint(void) {
  time t = SYS_get_time_ms();
  bool init = hud.state != hud.cstate;

  // check we get vbl from cvideo
  if (hud.last_vbl_check == 0) {
    hud.last_vbl_check = t;
  } else {
    if (t - hud.last_vbl_check >= 1000) {
      // check each ~sec
      hud.last_vbl_check = t;
      if (hud.vbl_count < 25) {
        // must at least get 25 vbls per second
        // else switch to generated input
        CVIDEO_set_input(INPUT_GENERATED);
        DBG(D_APP, D_WARN, "lost camera input, switch to generated\n");
      }
      hud.vbl_count = 0;
    }
  }

  if (hud.state == HUD_MAIN) {
    GFX_fill(hud.ctx, 0, 0, hud.ctx->width, hud.ctx->height, COL_RESET);
    hud_paint_main(hud.ctx, init);
  } else if (hud.state == HUD_CONFIG) {
    GFX_fill(hud.ctx, 0, 0, hud.ctx->width, hud.ctx->height, COL_RESET);
    hud_paint_config(hud.ctx, init);
  } else if (hud.state == HUD_DBG) {
    hud_paint_dbg(hud.ctx, init);
  } else if (hud.state == HUD_TEST) {
    GFX_fill(hud.ctx, 0, 0, hud.ctx->width, hud.ctx->height, COL_RESET);
    ROVER_paint(hud.ctx);
  }
  CVIDEO_gram_switch();
  hud.cstate = hud.state;
}

hud_state HUD_get_state(void) {
  return hud.state;
}

void HUD_vbl(void) {
  hud.vbl_count++;
}
