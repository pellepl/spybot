/*
 * hud_conf.c
 *
 *  Created on: Dec 27, 2013
 *      Author: petera
 */

#include "hud.h"
#include "rover_3d.h"
#include "trig_q.h"
#include "miniutils.h"

static const char const *(MAIN_MENU[]) =  {
  " MAIN",
  "MOTOR",
  "RADAR",
  "CAMERA",
  "SPEED",
  "EXIT",
  NULL
};

static struct {
  const char **cur_menu;
  u8_t menu_sel_ix;
} conf;

//////////////////////////////////////////////////////

// paint a setting bar, setting may range from -128 to 127
static void hud_conf_paint_setting_bar(gcontext *ctx, bool ver_else_hor, s16_t x, s16_t y, s16_t w, s16_t h,
        s8_t setting) {
  GFX_fill(ctx, x, y, w, h, COL_RESET);
  GFX_rect(ctx, x, y, w, h, COL_SET);
  u8_t dtick = ((ver_else_hor ? h : w) - 4) / 10;
  int i;
  for (i = 0; i < 10; i++) {
    if (ver_else_hor) {
      GFX_draw_horizontal_line(ctx, x+1, x+w-1, y + dtick*i, COL_SET);
    } else {
      GFX_draw_vertical_line(ctx, x + dtick*i, y+1, y+h-1, COL_SET);
    }
  }

  if (ver_else_hor) {
    if (setting < 0) {
      GFX_fill(ctx, x+2, y+h/2, w-4, (((h/2)-2) * -setting)/128, COL_MIX);
    } else if (setting > 0) {
      s16_t sy = y+2+(((h/2)-2) * setting)/128;
      GFX_fill(ctx, x+2, sy, w-4, y+h/2 - sy, COL_MIX);
    }
  } else {
    if (setting > 0) {
      GFX_fill(ctx, x+w/2, y+2, (((w/2)-4) * -setting)/128, h-4, COL_MIX);
    } else if (setting < 0) {
      s16_t sx = x+2+(((w/2)-2) * setting)/128;
      GFX_fill(ctx, sx, y+2, x+w/2 - sx, h-4, COL_MIX);
    }
  }
}

static void hud_conf_paint_menu(gcontext *ctx) {
  char txt[14];
  memset(txt, 0, sizeof(txt));
  const char **menu_items = conf.cur_menu;
  sprint(txt, "%s%s", "CONFIG", menu_items[0] ? menu_items[0] : "");
  int title_len = strlen(txt);
  GFX_printn_big(ctx, txt, 0, (14-title_len)/2, 0, COL_SET);

  u8_t menu_ix = 0;
  u8_t y = 4;
  bool do_fill = ((SYS_get_time_ms() >> 9) & 1);
  while (menu_items[menu_ix+1]) {
    bool select = do_fill && menu_ix == conf.menu_sel_ix;
    if (select) {
      GFX_fill(ctx, 8, y*8, ctx->width-8, 8, COL_SET);
    }
    GFX_printn(ctx, menu_items[menu_ix+1], 0, 1, y, select ? COL_RESET : COL_SET);
    y += 2;
  }
}


////////////////////////////////////////////////////// MOTOR CONFIG


////////////////////////////////////////////////////// RADAR CONFIG


////////////////////////////////////////////////////// CAMERA CONFIG


////////////////////////////////////////////////////// SPEED CONFIG


////////////////////////////////////////////////////// SAVE


//////////////////////////////////////////////////////


void hud_paint_config(gcontext *ctx, bool init) {
  if (init) {
    memset(&conf, 0, sizeof(conf));
    conf.cur_menu = MAIN_MENU;
    rover_angles *ra = ROVER_angle_config();
    ra->anim_d_radar = PI_TRIG_T/17;
    ra->anim_d_wheel_left = -PI_TRIG_T/7;
    ra->anim_d_wheel_right = -PI_TRIG_T/7;
    ROVER_view(0,100,350, 0,0,0, TRUE);
    ROVER_view(0,0,150, 0,0,-PI_TRIG_T/2, FALSE);
  }
  ROVER_paint(ctx);
  hud_conf_paint_menu(ctx);
}
