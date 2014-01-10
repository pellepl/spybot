/*
 * hud_main.c
 *
 *  Created on: Dec 27, 2013
 *      Author: petera
 */

#include "hud.h"
#include "gfx_bitmap.h"
#include "gfx_3d.h"
#include "trig_q.h"
#include "miniutils.h"
#include "comm_radio.h"
#include "app.h"

static struct {
  s32_t heading_ang;
  s32_t xplane_cos;
  s32_t xplane_sin;
  s32_t yplane_cos;
  s32_t yplane_sin;
  s16_t plane_pts[8][3];
} mhud;

const s16_t const base_plane_pts[8][3] = {
    { 15,  0,-16}, //0
    {-16,  0,-16}, //1
    {-16,  0, 15}, //2
    { 15,  0, 15}, //3
    {-16,  0, 0},  //4
    { 15,  0, 0},  //5
    {  0,  0,-16}, //6
    {  0,  0, 15}, //7
};

static void hud_plane_draw(gcontext *ctx, gcolor col) {
  GFX_draw_line(ctx, mhud.plane_pts[2][0], mhud.plane_pts[2][1], mhud.plane_pts[4][0], mhud.plane_pts[4][1], col);
  GFX_draw_line(ctx, mhud.plane_pts[3][0], mhud.plane_pts[3][1], mhud.plane_pts[5][0], mhud.plane_pts[5][1], col);
  GFX_draw_line(ctx, mhud.plane_pts[4][0], mhud.plane_pts[4][1], mhud.plane_pts[6][0], mhud.plane_pts[6][1], col);
  GFX_draw_line(ctx, mhud.plane_pts[5][0], mhud.plane_pts[5][1], mhud.plane_pts[6][0], mhud.plane_pts[6][1], col);
  GFX_draw_line(ctx, mhud.plane_pts[2][0], mhud.plane_pts[2][1], mhud.plane_pts[3][0], mhud.plane_pts[3][1], col);
  GFX_draw_line(ctx, mhud.plane_pts[4][0], mhud.plane_pts[4][1], mhud.plane_pts[5][0], mhud.plane_pts[5][1], col);
  GFX_draw_line(ctx, mhud.plane_pts[6][0], mhud.plane_pts[6][1], mhud.plane_pts[7][0], mhud.plane_pts[7][1], col);
}

static void hud_plane_reset(gcontext *ctx) {
  memcpy(mhud.plane_pts, base_plane_pts, sizeof(base_plane_pts));
}

void hud_paint_main(gcontext *ctx, bool init) {
  char txt[28];
  memset(txt,0,28);
  sprint(txt, "%s %i", APP_NAME, SYS_build_number());
  GFX_printn(ctx, txt, 0,  0, 16, COL_OVER);

  {
    u8_t h, m, s;
    u16_t ms;
    SYS_get_time(NULL, &h, &m, &s, &ms);
    sprint(txt, "%02i:%02i:%02i.%i ", h,m,s,ms/100);
    GFX_printn(ctx, txt, 0,  17, 16, COL_OVER);
  }

  time now = SYS_get_time_ms();
  if ((now / 500) & 1) {
    GFX_printn(ctx, " ", 0, 27, 16, COL_OVER);
  } else {
    GFX_printn(ctx, "\001",0, 27, 16, COL_OVER);
  }

  // upper status bar
  static time last_comm_check;
  static u8_t last_squal;
  if (now - last_comm_check >= 500) {
    last_comm_check = now;
    last_squal = COMRAD_stats();
  }

  if (last_squal > 192) {
    GFX_printn(ctx, "\010\011", 0, 0, 0, COL_OVER);
  } else if (last_squal > 128) {
    GFX_printn(ctx, "\006\007", 0, 0, 0, COL_OVER);
  } else if (last_squal > 64) {
    GFX_printn(ctx, "\004\005", 0, 0, 0, COL_OVER);
  } else {
    GFX_printn(ctx, "\002\003", 0, 0, 0, COL_OVER);
  }

  if ((APP_pair_status() == 1 && ((now>>8)&1)) || APP_pair_status() == 2) {
    GFX_printn(ctx, "\022", 0, 3, 0, COL_OVER);
  }


  if (APP_pair_status() != 2) {
    return;
  }

  // heading
  s16_t x = ctx->width/2;
  s16_t y = 110;

  u8_t heading_raw = APP_remote_get_heading();
  s8_t *acc = APP_remote_get_acc();

  mhud.heading_ang = (heading_raw <<1) - (PI_TRIG_T / 4);

  s16_t dx = (cos_table(mhud.heading_ang)*17) >> 15;
  s16_t dy = (sin_table(mhud.heading_ang)*14) >> 15;
  GFX_draw_line(ctx, x, y, x+dx, y+dy, COL_SET);

  memset(txt, 0, 16);
  sprint(txt, "DIR:%4i%c", (heading_raw * 360) >> 16, 186);
  GFX_printn(ctx, txt, 0, 28 - strlen(txt), 12, COL_OVER);

  // accelerometer
  s8_t ax,ay,az;
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

  memset(txt, 0, 16);
  sprint(txt, "ACC:%4i ", ad>>(15-14));
  GFX_printn(ctx, txt, 0, 28 - strlen(txt), 13, COL_OVER);

  s32_t axzn = _sqrt(axn*axn + azn*azn);
  s32_t ayzn = _sqrt(ayn*ayn + azn*azn);
  s32_t xplane_cos = (azn<<15) / axzn;
  s32_t xplane_sin = (axn<<15) / axzn;
  s32_t yplane_cos = (azn<<15) / ayzn;
  s32_t yplane_sin = (ayn<<15) / ayzn;

  // accelerometer plane
  {
    hud_plane_reset(ctx);

#define _H 32
#define _W 40
#define _X 17
#define _Y 99

    int i;
    for (i = 0; i < 8; i++) {
      GFX_rotate_x_3d(&mhud.plane_pts[i][0], -xplane_cos, xplane_sin);
      GFX_rotate_z_3d(&mhud.plane_pts[i][0], yplane_cos, -yplane_sin);
      GFX_project_3d(&mhud.plane_pts[i][0], _W, _H, _X, _Y);
    }
    hud_plane_draw(ctx, COL_SET);

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
}

