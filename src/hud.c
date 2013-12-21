/*
 * hud.c
 *
 *  Created on: Dec 10, 2013
 *      Author: petera
 */

#include "hud.h"
#include "gfx_bitmap.h"
#include "trig_q.h"
#include "miniutils.h"
#include "cvideo.h"

static struct {
  gcontext *ctx;
  hud_state state;
  hud_state cstate;
  u8_t dbg_cy;
  u8_t dbg_cx;
  s32_t heading_ang;
  s32_t xplane_cos;
  s32_t xplane_sin;
  s32_t yplane_cos;
  s32_t yplane_sin;
  s16_t vecs[8][3];
} hud;

const s16_t const base_plane_vecs[8][3] = {
    { 15,  0,-16}, //0
    {-16,  0,-16}, //1
    {-16,  0, 15}, //2
    { 15,  0, 15}, //3
    {-16,  0, 0},  //4
    { 15,  0, 0},  //5
    {  0,  0,-16}, //6
    {  0,  0, 15}, //7
};


static void project_3d(s16_t *vec, s16_t magx, s16_t magy, s16_t offsx, s16_t offsy) {
  vec[0] = (((s32_t)magx*(vec[0] << 16) / (vec[2]+64)) >> 16) + offsx;
  vec[1] = (((s32_t)magy*((vec[1]+16) << 16) / (vec[2]+64)) >> 16) + offsy;
}

static void rotate_x_3d(s16_t *v, s32_t cos, s32_t sin) {
  //s32_t x = 1   * v[0]  +  0   * v[1]   +  0   * v[2];
  s32_t y = 0   * v[0]  +  cos * v[1]   -  sin * v[2];
  s32_t z = 0   * v[0]  +  sin * v[1]   +  cos * v[2];
  //v[0] = (x >> 15);
  v[1] = (y >> 15);
  v[2] = (z >> 15);
}


static void rotate_z_3d(s16_t *v, s32_t cos, s32_t sin) {
  s32_t x = cos * v[0]  -  sin * v[1]   +  0   * v[2];
  s32_t y = sin * v[0]  +  cos * v[1]   +  0   * v[2];
  //s32_t z = 0   * v[0]  +  0   * v[1]   +  1   * v[2];
  v[0] = (x >> 15);
  v[1] = (y >> 15);
  //v[2] = (z >> 15);
}

static void vecs_draw(gcontext *ctx, gcolor col) {
//  GFX_draw_line(ctx, hud.vecs[0][0], hud.vecs[0][1], hud.vecs[1][0], hud.vecs[1][1], col);
//  GFX_draw_line(ctx, hud.vecs[1][0], hud.vecs[1][1], hud.vecs[2][0], hud.vecs[2][1], col);
//  GFX_draw_line(ctx, hud.vecs[2][0], hud.vecs[2][1], hud.vecs[3][0], hud.vecs[3][1], col);
//  GFX_draw_line(ctx, hud.vecs[3][0], hud.vecs[3][1], hud.vecs[0][0], hud.vecs[0][1], col);
//  GFX_draw_line(ctx, hud.vecs[4][0], hud.vecs[4][1], hud.vecs[5][0], hud.vecs[5][1], col);
//  GFX_draw_line(ctx, hud.vecs[6][0], hud.vecs[6][1], hud.vecs[7][0], hud.vecs[7][1], col);

  GFX_draw_line(ctx, hud.vecs[2][0], hud.vecs[2][1], hud.vecs[4][0], hud.vecs[4][1], col);
  GFX_draw_line(ctx, hud.vecs[3][0], hud.vecs[3][1], hud.vecs[5][0], hud.vecs[5][1], col);
  GFX_draw_line(ctx, hud.vecs[4][0], hud.vecs[4][1], hud.vecs[6][0], hud.vecs[6][1], col);
  GFX_draw_line(ctx, hud.vecs[5][0], hud.vecs[5][1], hud.vecs[6][0], hud.vecs[6][1], col);
  GFX_draw_line(ctx, hud.vecs[2][0], hud.vecs[2][1], hud.vecs[3][0], hud.vecs[3][1], col);
  GFX_draw_line(ctx, hud.vecs[4][0], hud.vecs[4][1], hud.vecs[5][0], hud.vecs[5][1], col);
  GFX_draw_line(ctx, hud.vecs[6][0], hud.vecs[6][1], hud.vecs[7][0], hud.vecs[7][1], col);
}

static void vecs_clear_reset(gcontext *ctx) {
  vecs_draw(ctx, COL_RESET);
  memcpy(hud.vecs, base_plane_vecs, sizeof(base_plane_vecs));
}

static void hud_paint_dbg(void) {
  if (hud.cstate != hud.state) {
    CVIDEO_set_v_offset(((hud.dbg_cy+1) * 8));
  }
  if ((SYS_get_time_ms() / 500) & 1) {
    GFX_printn(hud.ctx, " ", 1, hud.dbg_cx, hud.dbg_cy, COL_OVER);
  } else {
    GFX_printn(hud.ctx, "\037", 1, hud.dbg_cx, hud.dbg_cy, COL_OVER);
  }
}

static void hud_paint_main(gcontext *ctx, lsm303_dev *lsm_dev) {
  if (hud.cstate != hud.state) {
    char txt[28];
    memset(txt,0,28);
    GFX_fill(ctx, 0, 0, ctx->width, ctx->height, COL_RESET);
    sprint(txt, "%s build:%i", APP_NAME, SYS_build_number());
    GFX_printn(ctx, txt,0,  0, 16, COL_OVER);
  }

  s16_t x = ctx->width/2;
  s16_t y = 110;
  s32_t dx = (cos_table(hud.heading_ang)*17) >> 15;
  s32_t dy = (sin_table(hud.heading_ang)*14) >> 15;
  GFX_draw_line(ctx, x, y, x+dx, y+dy, COL_RESET);

  u16_t heading_raw = lsm_get_heading(lsm_dev);
  s16_t *acc = lsm_get_acc_reading(lsm_dev);
  hud.heading_ang = (heading_raw >> (16-9)) - (PI_TRIG_T / 4);

  dx = (cos_table(hud.heading_ang)*17) >> 15;
  dy = (sin_table(hud.heading_ang)*14) >> 15;
  GFX_draw_line(ctx, x, y, x+dx, y+dy, COL_SET);
  char txt[16];
  memset(txt, 0, 16);
  sprint(txt, "DIR:%4i%c", (heading_raw * 360) >> 16, 186);
  GFX_printn(ctx, txt, 0, 28 - strlen(txt), 12, COL_OVER);

  // accelerometer
  s16_t ax,ay,az;
  ax = acc[0];
  ay = acc[1];
  az = acc[2];
  s32_t ax2 = ax*ax;
  s32_t ay2 = ay*ay;
  s32_t az2 = az*az;
  s32_t ad = _sqrt(ax2+ay2+az2);
  s32_t axn = (ax<<15)/ad;
  s32_t ayn = (ay<<15)/ad;
  s32_t azn = (az<<15)/ad;

  {
    memset(txt, 0, 16);
    sprint(txt, "ACC:%4i ", ad>>(15-14));
    GFX_printn(ctx, txt, 0, 28 - strlen(txt), 13, COL_OVER);

    s32_t axzn = _sqrt(axn*axn + azn*azn);
    s32_t ayzn = _sqrt(ayn*ayn + azn*azn);
    hud.xplane_cos = (azn<<15) / axzn;
    hud.xplane_sin = (axn<<15) / axzn;
    hud.yplane_cos = (azn<<15) / ayzn;
    hud.yplane_sin = (ayn<<15) / ayzn;
  }

  // accelerometer plane
  {
    vecs_clear_reset(ctx);

#define _H 32
#define _W 40
#define _X 17
#define _Y 99

    int i;
    for (i = 0; i < 8; i++) {
      rotate_x_3d(&hud.vecs[i][0], -hud.xplane_cos, hud.xplane_sin);
      rotate_z_3d(&hud.vecs[i][0], hud.yplane_cos, -hud.yplane_sin);
      project_3d(&hud.vecs[i][0], _W, _H, _X, _Y);
    }
    vecs_draw(ctx, COL_SET);

    // accelerometer bars
#undef _W
#undef _H
#undef _X
#undef _Y
#define _W 32
#define _X 0
#define _Y 120
      GFX_rect(ctx, _X, _Y, _W, 2, COL_SET);
      GFX_draw_horizontal_line(ctx, _X+1, _X+1+_W-1, _Y+1, COL_RESET);
      GFX_draw_horizontal_line(ctx,
          _X+(_W/2),
          _X+(_W/2) -
          ( ( (1+(_W-2)/2)*ayn )>>15 ),
          _Y+1, COL_SET);
#undef _W
#undef _X
#undef _Y

#define _H 24
#define _X 32
#define _Y 96
      GFX_rect(ctx, _X, _Y, 2, _H, COL_SET);
      GFX_draw_vertical_line(ctx, _X+1, _Y+1, _Y+1+_H-1, COL_RESET);
      GFX_draw_vertical_line(ctx,
          _X+1,
          _Y+(_H/2),
          _Y+(_H/2) +
          ( ( (1+(_H-2)/2)*axn )>>15 ),
          COL_SET);

  }

  if ((SYS_get_time_ms() / 500) & 1) {
    GFX_printn(ctx, " ", 0, 27, 16, COL_OVER);
  } else {
    GFX_printn(ctx, "\001",0, 27, 16, COL_OVER);
  }
}

void HUD_state(hud_state state) {
  if (hud.state != state) {
    CVIDEO_set_v_offset(0);
    GFX_fill(hud.ctx, 0, 0, hud.ctx->width, hud.ctx->height, COL_RESET);
    hud.state = state;
    hud.dbg_cx = 0;
    hud.dbg_cy = 0;
    CVIDEO_set_effect(79);
  }
}

void HUD_paint(lsm303_dev *lsm_dev) {
  if (hud.state == HUD_MAIN) {
    hud_paint_main(hud.ctx, lsm_dev);
  } else if (hud.state == HUD_DBG) {
    hud_paint_dbg();
  }
  hud.cstate = hud.state;
}

void HUD_init(gcontext *ctx) {
  memset(&hud, 0, sizeof(hud));
  hud.ctx = ctx;
}

void HUD_dbg_print(char *str) {
  if (hud.state != HUD_DBG) {
    HUD_state(HUD_DBG);
  }

  int len = strlen(str);
  while (len > 0) {
    const char *nl = strchr(str, '\n');
    int rem = (hud.ctx->width / 8) - hud.dbg_cx;
    int rlen = MIN(rem, len);
    if (nl) {
      rlen = MIN(nl - str, rlen);
    }
    if (hud.dbg_cx == 0) {
      GFX_fill(hud.ctx, 0, hud.dbg_cy * 8, hud.ctx->width, 8, COL_RESET);
    }
    GFX_printn(hud.ctx, str, rlen, hud.dbg_cx, hud.dbg_cy, COL_OVER);
    if (nl) {
      rlen++;
    }
    str += rlen;
    len -= rlen;
    hud.dbg_cx += rlen;
    if (hud.dbg_cx >= hud.ctx->width/8 || nl) {
      hud.dbg_cx = 0;
      hud.dbg_cy++;
      CVIDEO_set_v_offset(((hud.dbg_cy+1) * 8));
      if (hud.dbg_cy >= 17) {
        hud.dbg_cy = 0;
      }
      GFX_fill(hud.ctx, 0, hud.dbg_cy * 8, hud.ctx->width, 8, COL_RESET);
    }
  }
}
