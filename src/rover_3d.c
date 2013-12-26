/*
 * rover_3d.c
 *
 *  Created on: Dec 26, 2013
 *      Author: petera
 */

#include "rover_3d.h"
#include "gfx_bitmap.h"
#include "gfx_3d.h"
#include "trig_q.h"

#define ROVER_BASE_POINTS     27
#define WHEEL_BASE_POINTS     8
#define RADAR_BASE_POINTS     4
#define CAMERA_BASE_POINTS    4
#define LENS_BASE_POINTS      4

#define ROVER_POINTS          (ROVER_BASE_POINTS*2)
#define WHEEL_POINTS          (WHEEL_BASE_POINTS*2)
#define RADAR_POINTS          (RADAR_BASE_POINTS)
#define CAMERA_POINTS         (CAMERA_BASE_POINTS*2)
#define LENS_POINTS           (LENS_BASE_POINTS)

#define ALL_POINTS            (ROVER_POINTS + WHEEL_POINTS*4 + RADAR_POINTS + CAMERA_POINTS + LENS_POINTS)

#define ROVER_IX              0
#define WHEEL1_IX             ROVER_IX + ROVER_POINTS
#define WHEEL2_IX             WHEEL1_IX + WHEEL_POINTS
#define WHEEL3_IX             WHEEL2_IX + WHEEL_POINTS
#define WHEEL4_IX             WHEEL3_IX + WHEEL_POINTS
#define RADAR_IX              WHEEL4_IX + WHEEL_POINTS
#define CAMERA_IX             RADAR_IX + RADAR_POINTS
#define LENS_IX               CAMERA_IX + CAMERA_POINTS
#define END_IX                LENS_IX + LENS_POINTS


static s16_t transform[ALL_POINTS][3];

static s16_t cam_x = 0;
static s16_t cam_y = 0;
static s16_t cam_z = 150;
static s16_t cam_ax = -PI_TRIG_T/4;
static s16_t cam_ay = 0;
static s16_t cam_az = 0;

static s16_t cam_cx = 0;
static s16_t cam_cy = -100;
static s16_t cam_cz = 90;
static s16_t cam_cax = 0;
static s16_t cam_cay = 0;
static s16_t cam_caz = 0;

const s16_t const base_rover_pts[ROVER_POINTS][3] = {
    {0-120,0,0},
    {100/5-120,295/5,0},
    {200/5-120,295/5,0},
    {255/5-120,350/5,0},
    {450/5-120,350/5,0},
    {505/5-120,295/5,0},
    {770/5-120,295/5,0},
    {825/5-120,350/5,0},
    {1015/5-120,350/5,0},
    {1070/5-120,295/5,0},
    {1170/5-120,295/5,0},
    {1200/5-120,265/5,0},
    {1200/5-120,85/5,0},
    {1170/5-120,55/5,0},
    {1170/5-120,-55/5,0},
    {1200/5-120,-85/5,0},
    {1200/5-120,-265/5,0},
    {1170/5-120,-295/5,0},
    {1070/5-120,-295/5,0},
    {1015/5-120,-350/5,0},
    {825/5-120,-350/5,0},
    {770/5-120,-295/5,0},
    {505/5-120,-295/5,0},
    {450/5-120,-350/5,0},
    {255/5-120,-350/5,0},
    {200/5-120,-295/5,0},
    {100/5-120,-295/5,0},
};

const s16_t const base_wheel_pts[WHEEL_BASE_POINTS][3] = {
    {200/5,0,80/5},
    {200/5,0,-80/5},
    {80/5,0,-200/5},
    {-80/5,0,-200/5},
    {-200/5,0,-80/5},
    {-200/5,0,80/5},
    {-80/5,0,200/5},
    {80/5,0,200/5},
};

const s16_t const base_radar_pts[RADAR_BASE_POINTS][3] = {
    {0,-225/5,100/5},
    {0,225/5,100/5},
    {0,225/5,-100/5},
    {0,-225/5,-100/5},
};

const s16_t const base_camera_pts[CAMERA_BASE_POINTS][3] = {
    {0,-100/5,100/5},
    {0,100/5,100/5},
    {0,100/5,-100/5},
    {0,-100/5,-100/5},
};

const s16_t const lens_camera_pts[LENS_BASE_POINTS][3] = {
    {0,0/5,60/5},
    {0,60/5,0/5},
    {0,0/5,-60/5},
    {0,-60/5,0/5},
};

static void rover_draw_transform(gcontext *ctx,  s16_t (*t)[3], gcolor col) {
  int i;

  // paint rover base
  for (i = ROVER_IX; i < ROVER_IX + ROVER_BASE_POINTS-1; i++) {
    GFX_draw_line(ctx,
        t[i][0], t[i][1],
        t[i + 1][0], t[i + 1][1], col);
    GFX_draw_line(ctx,
        t[i + ROVER_BASE_POINTS][0], t[i + ROVER_BASE_POINTS][1],
        t[i + ROVER_BASE_POINTS + 1][0], t[i + ROVER_BASE_POINTS + 1][1], col);
  }
  GFX_draw_line(ctx,
      t[ROVER_IX + ROVER_BASE_POINTS - 1][0], t[ROVER_IX + ROVER_BASE_POINTS - 1][1],
      t[ROVER_IX][0], t[ROVER_IX][1], col);
  GFX_draw_line(ctx,
      t[ROVER_IX + ROVER_BASE_POINTS*2 - 1][0], t[ROVER_IX + ROVER_BASE_POINTS*2 - 1][1],
      t[ROVER_IX + ROVER_BASE_POINTS][0], t[ROVER_IX + ROVER_BASE_POINTS][1], col);
  for (i = ROVER_IX+1; i < ROVER_IX + ROVER_BASE_POINTS; i++) {
    GFX_draw_line(ctx,
        t[i][0], t[i][1],
        t[i + ROVER_BASE_POINTS][0], t[i + ROVER_BASE_POINTS][1], col);
  }

  // paint wheels
  int j;
  for (j = 0; j < WHEEL_POINTS * 4; j += WHEEL_POINTS) {
    for (i = WHEEL1_IX+j; i < WHEEL1_IX + WHEEL_BASE_POINTS-1+j; i++) {
      GFX_draw_line(ctx,
          t[i][0], t[i][1],
          t[i + 1][0], t[i + 1][1], col);
      GFX_draw_line(ctx,
          t[i + WHEEL_BASE_POINTS][0], t[i + WHEEL_BASE_POINTS][1],
          t[i + WHEEL_BASE_POINTS + 1][0], t[i + WHEEL_BASE_POINTS + 1][1], col);
      GFX_draw_line(ctx,
          t[i][0], t[i][1],
          t[i + WHEEL_BASE_POINTS][0], t[i + WHEEL_BASE_POINTS][1], col);
    }
    GFX_draw_line(ctx,
        t[WHEEL1_IX + WHEEL_BASE_POINTS - 1+j][0], t[WHEEL1_IX + WHEEL_BASE_POINTS - 1+j][1],
        t[WHEEL1_IX+j][0], t[WHEEL1_IX+j][1], col);
    GFX_draw_line(ctx,
        t[WHEEL1_IX + WHEEL_BASE_POINTS*2 - 1+j][0], t[WHEEL1_IX + WHEEL_BASE_POINTS*2 - 1+j][1],
        t[WHEEL1_IX + WHEEL_BASE_POINTS+j][0], t[WHEEL1_IX + WHEEL_BASE_POINTS+j][1], col);
    GFX_draw_line(ctx,
        t[WHEEL1_IX + WHEEL_BASE_POINTS - 1+j][0], t[WHEEL1_IX + WHEEL_BASE_POINTS - 1+j][1],
        t[WHEEL1_IX + WHEEL_BASE_POINTS*2-1+j][0], t[WHEEL1_IX + WHEEL_BASE_POINTS*2-1+j][1], col);
  }

  // paint radar base
  for (i = RADAR_IX; i < RADAR_IX + RADAR_BASE_POINTS-1; i++) {
    GFX_draw_line(ctx,
        t[i][0], t[i][1],
        t[i + 1][0], t[i + 1][1], col);
  }
  GFX_draw_line(ctx,
      t[RADAR_IX + RADAR_BASE_POINTS - 1][0], t[RADAR_IX + RADAR_BASE_POINTS - 1][1],
      t[RADAR_IX][0], t[RADAR_IX][1], col);

  // paint camera base
  for (i = CAMERA_IX; i < CAMERA_IX + CAMERA_BASE_POINTS-1; i++) {
    GFX_draw_line(ctx,
        t[i][0], t[i][1],
        t[i + 1][0], t[i + 1][1], col);
    GFX_draw_line(ctx,
        t[i + CAMERA_BASE_POINTS][0], t[i + CAMERA_BASE_POINTS][1],
        t[i + CAMERA_BASE_POINTS + 1][0], t[i + CAMERA_BASE_POINTS + 1][1], col);
    GFX_draw_line(ctx,
        t[i][0], t[i][1],
        t[i + CAMERA_BASE_POINTS][0], t[i + CAMERA_BASE_POINTS][1], col);
  }
  GFX_draw_line(ctx,
      t[CAMERA_IX + CAMERA_BASE_POINTS - 1][0], t[CAMERA_IX + CAMERA_BASE_POINTS - 1][1],
      t[CAMERA_IX][0], t[CAMERA_IX][1], col);
  GFX_draw_line(ctx,
      t[CAMERA_IX + CAMERA_BASE_POINTS*2 - 1][0], t[CAMERA_IX + CAMERA_BASE_POINTS*2 - 1][1],
      t[CAMERA_IX + CAMERA_BASE_POINTS][0], t[CAMERA_IX + CAMERA_BASE_POINTS][1], col);
  GFX_draw_line(ctx,
      t[CAMERA_IX + CAMERA_BASE_POINTS - 1][0], t[CAMERA_IX + CAMERA_BASE_POINTS - 1][1],
      t[CAMERA_IX + CAMERA_BASE_POINTS*2 - 1][0], t[CAMERA_IX + CAMERA_BASE_POINTS*2 - 1][1],
      col);

  // paint lens base
  for (i = LENS_IX; i < LENS_IX + LENS_POINTS-1; i++) {
    GFX_draw_line(ctx,
        t[i][0], t[i][1],
        t[i + 1][0], t[i + 1][1], col);
  }
  GFX_draw_line(ctx,
      t[LENS_IX + LENS_POINTS - 1][0], t[LENS_IX + LENS_POINTS - 1][1],
      t[LENS_IX][0], t[LENS_IX][1], col);
}

static void rover_reset_transform(gcontext *ctx, s16_t (*t)[3]) {
  int i;
  s16_t cos, sin;
  s32_t ang = -SYS_get_time_ms();
  cos = cos_table(ang/8);
  sin = sin_table(ang/8);
  // camera base build
  memcpy(&t[CAMERA_IX], base_camera_pts, sizeof(base_camera_pts));
  memcpy(&t[CAMERA_IX+CAMERA_BASE_POINTS], base_camera_pts, sizeof(base_camera_pts));
  for (i = CAMERA_IX; i < CAMERA_IX+CAMERA_BASE_POINTS; i++) {
    GFX_translate_3d(&t[i][0], -20, 0, 0);
    GFX_translate_3d(&t[i+CAMERA_BASE_POINTS][0], 20, 0, 0);
    GFX_translate_3d(&t[i][0], -100, 0, -20);
    GFX_translate_3d(&t[i+CAMERA_BASE_POINTS][0], -100, 0, -20);
  }
  // lens base build
  memcpy(&t[LENS_IX], lens_camera_pts, sizeof(lens_camera_pts));
  for (i = LENS_IX; i < LENS_IX+LENS_POINTS; i++) {
    GFX_translate_3d(&t[i][0], -20, 0, 0);
    GFX_translate_3d(&t[i][0], -100, 0, -20);
  }
  // radar base build
  memcpy(&t[RADAR_IX], base_radar_pts, sizeof(base_radar_pts));
  for (i = RADAR_IX; i < RADAR_IX+RADAR_POINTS; i++) {
    GFX_rotate_z_3d(&t[i][0], cos, sin);
    GFX_translate_3d(&t[i][0], 20, 0, -50);
  }
  // rover base build
  memcpy(&t[ROVER_IX], base_rover_pts, sizeof(base_rover_pts));
  memcpy(&t[ROVER_IX+ROVER_BASE_POINTS], base_rover_pts, sizeof(base_rover_pts));
  for (i = ROVER_IX+ROVER_BASE_POINTS; i < ROVER_IX+ROVER_BASE_POINTS*2; i++) {
    GFX_translate_3d(&t[i][0], 0, 0, 40);
  }
  // wheel base build
  cos = cos_table(ang/7);
  sin = sin_table(ang/7);

  memcpy(&t[WHEEL1_IX], base_wheel_pts, sizeof(base_wheel_pts));
  memcpy(&t[WHEEL1_IX+WHEEL_BASE_POINTS], base_wheel_pts, sizeof(base_wheel_pts));
  for (i = WHEEL1_IX+WHEEL_BASE_POINTS; i < WHEEL1_IX+WHEEL_BASE_POINTS*2; i++) {
    GFX_translate_3d(&t[i][0],0, -20, 0);
  }
  for (i = WHEEL1_IX; i < WHEEL1_IX+WHEEL_POINTS; i++) {
    GFX_rotate_y_3d(&t[i][0], cos, sin);
    GFX_translate_3d(&t[i][0], (255+450)/2/5, -(350/5+8),  35);
    memcpy(&t[i+WHEEL_POINTS][0], &t[i][0], sizeof(s16_t)*3);
    GFX_translate_3d(&t[i+WHEEL_POINTS][0], -(1015+825)/2/5 + (255+450)/2/5, 0,0);
  }

  memcpy(&t[WHEEL3_IX], base_wheel_pts, sizeof(base_wheel_pts));
  memcpy(&t[WHEEL3_IX+WHEEL_BASE_POINTS], base_wheel_pts, sizeof(base_wheel_pts));
  for (i = WHEEL3_IX+WHEEL_BASE_POINTS; i < WHEEL3_IX+WHEEL_BASE_POINTS*2; i++) {
    GFX_translate_3d(&t[i][0],0, 20, 0);
  }
  for (i = WHEEL3_IX; i < WHEEL3_IX+WHEEL_POINTS; i++) {
    GFX_rotate_y_3d(&t[i][0], cos, sin);
    GFX_translate_3d(&t[i][0], (255+450)/2/5, (350/5+8), 35);
    memcpy(&t[i+WHEEL_POINTS][0], &t[i][0], sizeof(s16_t)*3);
    GFX_translate_3d(&t[i+WHEEL_POINTS][0], -(1015+825)/2/5 + (255+450)/2/5, 0,0);
  }

}

void ROVER_paint(gcontext *ctx) {
  rover_reset_transform(ctx, transform);
  int i;
  s16_t cosx = cos_table(cam_cax);
  s16_t sinx = sin_table(cam_cax);
  s16_t cosy = cos_table(cam_cay);
  s16_t siny = sin_table(cam_cay);
  s16_t cosz = cos_table(cam_caz);
  s16_t sinz = sin_table(cam_caz);
  for (i = 0; i < ALL_POINTS; i++) {
    GFX_rotate_x_3d(&transform[i][0], cosx, sinx);
    GFX_rotate_y_3d(&transform[i][0], cosy, siny);
    GFX_rotate_z_3d(&transform[i][0], cosz, sinz);
    GFX_translate_3d(&transform[i][0], cam_cx, cam_cy, cam_cz);
    GFX_project_3d(&transform[i][0], 70, (80*70)/100, ctx->width/2, ctx->height/2-12);
  }
  rover_draw_transform(ctx, transform, COL_SET);

  s16_t cam_dx = cam_x-cam_cx;
  s16_t cam_dy = cam_y-cam_cy;
  s16_t cam_dz = cam_z-cam_cz;
  s16_t cam_dax = cam_ax-cam_cax;
  s16_t cam_day = cam_ay-cam_cay;
  s16_t cam_daz = cam_az-cam_caz;

  if (ABS(cam_dx) < 8) cam_cx += SIGN(cam_dx);
  else cam_cx += (cam_dx>>2);
  if (ABS(cam_dy) < 8) cam_cy += SIGN(cam_dy);
  else cam_cy += (cam_dy>>2);
  if (ABS(cam_dz) < 8) cam_cz += SIGN(cam_dz);
  else cam_cz += (cam_dz>>2);
  if (ABS(cam_dax) < 8) cam_cax += SIGN(cam_dax);
  else cam_cax += (cam_dax>>2);
  if (ABS(cam_day) < 8) cam_cay += SIGN(cam_day);
  else cam_cay += (cam_day>>2);
  if (ABS(cam_daz) < 8) cam_caz += SIGN(cam_daz);
  else cam_caz += (cam_daz>>2);
}

void ROVER_view(s16_t x, s16_t y, s16_t z, s16_t ax, s16_t ay, s16_t az) {
  cam_x = x;
  cam_y = y;
  cam_z = z;
  cam_ax = ax;
  cam_ay = ay;
  cam_az = az;
}
