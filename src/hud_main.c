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
#include "adc.h"
#include "cvideo.h"

extern unsigned const char const img_compass_bmp[2560];

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

#ifdef ACC_GRAPH
static u8_t acc_buf[64];
static u8_t acc_buf_ix = 0;
#endif

static u8_t audio_buf[64];

static video_input_t last_input = INPUT_GENERATED;
static time video_loss_msg_t = 0;

#include "radar_vecs.c"

static void hud_graph_draw(gcontext *ctx, u8_t *data, u8_t start_ix, u8_t len, u8_t height, u8_t x, u8_t y) {
  GFX_draw_vertical_line(ctx, x,y,y+height+2, COL_SET);
  GFX_draw_horizontal_line(ctx, x,x+len+2,y+height+2, COL_SET);
  int i;
  u8_t spl = data[start_ix];
  for (i = 1; i < len; i++) {
    u8_t new_spl;
    if (start_ix + i >= len) {
      new_spl = data[start_ix + i - len];
    } else {
      new_spl = data[start_ix + i];
    }
    GFX_draw_line(ctx, x+i, y+height-spl,
                       x+1+i, y+height-new_spl,
                       COL_SET);
    spl = new_spl;
  }
}

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
  const u8_t max_ychar = (ctx->height/8)-1;
  char txt[28];
  memset(txt,0,28);
  time now = SYS_get_time_ms();
  bool blink = (now/500) & 1;
  bool show_init = now < 4000;

  // info status line
  if (show_init) {
    sprint(txt, "%s %i", APP_NAME, SYS_build_number());
    GFX_printn(ctx, txt, 0,  0, 0, COL_OVER);
  }

  {
    u8_t h, m, s;
    u16_t ms;
    SYS_get_time(NULL, &h, &m, &s, &ms);
    sprint(txt, "%02i:%02i:%02i.%i", h,m,s,ms/100);
    GFX_printn(ctx, txt, 0, 18, 0, COL_OVER);
  }

  // upper status bar
  if (!show_init) {
    u8_t lqual = COMRAD_get_link_qual();

    if (lqual >= COMM_RADIO_LQUAL_PERFECT) {
      GFX_printn(ctx, "\010\011", 0, 0, 0, COL_OVER);
    } else if (lqual >= COMM_RADIO_LQUAL_GOOD) {
      GFX_printn(ctx, "\006\007", 0, 0, 0, COL_OVER);
    } else if (lqual >= COMM_RADIO_LQUAL_WEAK) {
      GFX_printn(ctx, "\004\005", 0, 0, 0, COL_OVER);
    } else {
      GFX_printn(ctx, "\002\003", 0, 0, 0, COL_OVER);
    }

    u32_t last_batt_1000 = APP_get_last_batt();
    if (last_batt_1000 > 3900*2)
      GFX_printn(ctx, "\013", 0, 3, 0, COL_OVER);
    else if (last_batt_1000 > 3775*2)
      GFX_printn(ctx, "\014", 0, 3, 0, COL_OVER);
    else if (last_batt_1000 > 3700*2)
      GFX_printn(ctx, "\015", 0, 3, 0, COL_OVER);
    else if (last_batt_1000 > 3600*2)
      GFX_printn(ctx, "\016", 0, 3, 0, COL_OVER);
    else if (last_batt_1000 == 0)
      GFX_printn(ctx, "\012", 0, 3, 0, COL_OVER);
    else {
      if (((now>>8)&1))
        GFX_printn(ctx, "\016", 0, 3, 0, COL_OVER);
    }


    if ((APP_pair_status() == PAIRING_STAGE_TWO && ((now>>8)&1)) || APP_pair_status() == PAIRING_OK) {
      GFX_printn(ctx, "\022", 0, 5, 0, COL_OVER);
    }
  }

  if (APP_pair_status() != PAIRING_OK) {
    return;
  }

  // paired state

  // joystick control, remote settings/state
  if (!show_init) {
    GFX_printn(ctx, APP_get_joystick_control() == APP_JOYSTICK_CONTROL_MOTOR ? "\020" : "\021",
        0, 7, 0, COL_OVER);
    GFX_printn(ctx, APP_remote_get()->light_ir ? "\033" : "\032", 0, 9, 0, COL_OVER);
    GFX_printn(ctx, APP_remote_get()->light_white ? "\035" : "\034", 0, 11, 0, COL_OVER);
    u8_t remote_batt = APP_remote_get()->batt;
    u32_t v_1000 = (remote_batt << 4) * 7400 / 0xab0; // TODO might need modification for remote
    if (v_1000 > 3900*2)
      GFX_printn(ctx, "\013", 0, 13, 0, COL_OVER);
    else if (v_1000 > 3775*2)
      GFX_printn(ctx, "\014", 0, 13, 0, COL_OVER);
    else if (v_1000 > 3700*2)
      GFX_printn(ctx, "\015", 0, 13, 0, COL_OVER);
    else if (v_1000 > 3600*2)
      GFX_printn(ctx, "\016", 0, 13, 0, COL_OVER);
    else if (v_1000 == 0)
      GFX_printn(ctx, "\012", 0, 13, 0, COL_OVER);
    else {
      if (((now>>8)&1))
        GFX_printn(ctx, "\016", 0, 13, 0, COL_OVER);
    }
    //GFX_printn(ctx, "\013", 0, 13, 0, COL_OVER);
  }

  // audio
  ADC_sample_sound(NULL, audio_buf, sizeof(audio_buf));
  hud_graph_draw(ctx, audio_buf, 0, sizeof(audio_buf), 32, 157, ctx->height-32-4);

  // heading
#define _X 14

  s16_t x = ctx->width/2 + _X;
  s16_t y = ctx->height - 4 - 17;

  u8_t heading_raw = APP_remote_get_heading();
  heading_raw = 128 - heading_raw;
  static u8_t comp_heading = 0;
  u8_t d_head = ABS(comp_heading - heading_raw);
  if (d_head > 4) {
    s8_t comp = ((s8_t)(heading_raw - comp_heading))/2;
    if (ABS(comp) > 16) {
      if (comp > 0) {
        comp_heading += 16;
      } else {
        comp_heading -= 16;
      }
    } else {
      comp_heading += comp;
    }
  }
  s8_t *acc = APP_remote_get_acc();

  mhud.heading_ang = (comp_heading << 1) - (PI_TRIG_T / 4);
  s16_t dxp = (cos_table(mhud.heading_ang)*16) >> 15;
  s16_t dyp = (sin_table(mhud.heading_ang)*8) >> 15;
  s16_t dxb = (cos_table(mhud.heading_ang + PI_TRIG_T / 4)*4) >> 15;
  s16_t dyb = (sin_table(mhud.heading_ang + PI_TRIG_T / 4)*3) >> 15;
  GFX_draw_line(ctx, x+dxp, y+dyp, x+dxb, y+dyb, COL_SET);
  GFX_draw_line(ctx, x+dxp, y+dyp, x-dxb, y-dyb, COL_SET);
  GFX_draw_line(ctx, x+dxb, y+dyb, x+dxp/4, y+dyp/4, COL_SET);
  GFX_draw_line(ctx, x-dxb, y-dyb, x+dxp/4, y+dyp/4, COL_SET);

  GFX_draw_image(ctx, img_compass_bmp, ctx->width/2 - 20 + _X, ctx->height - 8 - 4, 40, 10, (comp_heading-12)&0xff, 0, 256/8);
  GFX_rect(ctx, ctx->width/2 - 20 - 1 + _X, ctx->height - 8 - 4 -1, 41, 11, COL_SET);


  // accelerometer
  s8_t ax,ay,az;
  static s16_t lp_az = 64<<8;
  ax = acc[0];
  ay = acc[1];
  az = acc[2];
  lp_az = 15*(lp_az)/16 + 1*(az << 8)/16;
  s32_t ax2 = ax*ax;
  s32_t ay2 = ay*ay;
  s32_t az2 = az*az;
  s32_t ad = _sqrt(ax2+ay2+az2);
  s32_t axn = (ax<<15)/ad;
  s32_t ayn = (ay<<15)/ad;
  s32_t azn = (az<<15)/ad;

#ifdef ACC_GRAPH
  acc_buf[acc_buf_ix++] = ad>>2;
  if (acc_buf_ix >= sizeof(acc_buf)) {
    acc_buf_ix = 0;
  }
  hud_graph_draw(ctx, acc_buf, (acc_buf_ix + sizeof(acc_buf)) % sizeof(acc_buf),
      sizeof(acc_buf), 32, 150, 90);
#endif

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
#undef _X
#define _X 17
#define _Y (ctx->height - 8)

    int i;
    for (i = 0; i < 8; i++) {
      GFX_rotate_x_3d(&mhud.plane_pts[i][0], -xplane_cos, xplane_sin);
      GFX_rotate_z_3d(&mhud.plane_pts[i][0], yplane_cos, -yplane_sin);
      GFX_project_3d(&mhud.plane_pts[i][0], _W, _H, _X, _Y-_H/2);
    }

    if (blink) {
      if (lp_az < 0) {
        GFX_printn(ctx, "ROLL", 4, 0, max_ychar - 1, COL_OVER);
      } else if (lp_az < (46<<8)) {
        GFX_printn(ctx, "TILT", 4, 0, max_ychar - 1, COL_OVER);
      } else {
        hud_plane_draw(ctx, COL_SET);
      }
    } else {
      hud_plane_draw(ctx, COL_SET);
    }

    // accelerometer bars
#undef _W
#undef _H
#undef _X
#undef _Y
#define _W 32
#define _X 0
#define _Y (ctx->height - 4)
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
#define _Y (ctx->height - 4 - _H)
      GFX_rect(ctx, _X, _Y, 2, _H, COL_SET);
      GFX_draw_vertical_line(ctx, _X+1, _Y+1, _Y+1+_H-1, COL_RESET);
      GFX_draw_vertical_line(ctx,
          _X+1,
          _Y+(_H/2),
          _Y+(_H/2) +
          ( ( (1+(_H-2)/2)*axn )>>15 ),
          COL_SET);

      // radar
#undef _X
#undef _Y
#define _X 70
#define _Y (ctx->height - 22)
      {
        u32_t i;
        radar_vec_t v = radar_vecs[0];
        GFX_draw_line(ctx, v.x1+_X-1, v.y1+_Y-1, v.x2+_X-1, v.y2+_Y+1, COL_SET);
        v = radar_vecs[CONFIG_RADAR_ANGLES-1];
        GFX_draw_line(ctx, v.x1+_X+1, v.y1+_Y-1, v.x2+_X+1, v.y2+_Y+1, COL_SET);
        u8_t *radar_vals = (u8_t *)APP_get_radar_values();
        for (i = 0; i < CONFIG_RADAR_ANGLES; i++) {
          v = radar_vecs[i];
          u8_t radar_val = 255-radar_vals[i];
          GFX_put_pixel(ctx, v.x1+_X, v.y1+_Y-1, COL_SET);
          GFX_put_pixel(ctx, v.x2+_X, v.y2+_Y+1, COL_SET);
          if (radar_val > 0)
          {
            s16_t x2 = v.x1 + (radar_val * v.dx >> 8);
            s16_t y2 = v.y1 + (radar_val * v.dy >> 8);
            GFX_draw_line(ctx, v.x1+_X, v.y1+_Y+1, x2+_X, y2+_Y+1, COL_SET);
            GFX_draw_line(ctx, v.x1+_X+1, v.y1+_Y+1, x2+_X, y2+_Y+1, COL_SET);
          }
        }
      }

  }

  // crosshair
#define _L  16
  {
    GFX_draw_horizontal_line(ctx, ctx->width/2-_L, ctx->width/2+_L, ctx->height/2, COL_MIX);
    GFX_draw_vertical_line(ctx, ctx->width/2, ctx->height/2-_L, ctx->height/2+_L, COL_MIX);

    s16_t d = (-APP_remote_get()->tilt * _L) >> 7;
    GFX_draw_horizontal_line(ctx, ctx->width/2-4, ctx->width/2+5, ctx->height/2+d, COL_MIX);
    d = (APP_remote_get()->pan * _L) >> 7;
    GFX_draw_vertical_line(ctx, ctx->width/2+d, ctx->height/2-4, ctx->height/2+5, COL_MIX);
  }

  // video input info
  if (CVIDEO_get_input() == INPUT_GENERATED) {
    if (last_input != INPUT_GENERATED) {
      video_loss_msg_t = now;
    }
    if (now - video_loss_msg_t < 4000) {
      GFX_printn_big(ctx, "VIDEO LOST", 10, 4, 3, COL_OVER);
    }
  }

  last_input = CVIDEO_get_input();
}

