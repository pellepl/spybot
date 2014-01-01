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

typedef enum {
   CONF_MAIN = 0,
   CONF_MOTOR,
   CONF_RADAR,
   CONF_CAMERA,
   CONF_SPEED,
   CONF_EXIT
} conf_state;

static const char * const(MAIN_MENU[]) =  {
  " MAIN",
  "MOTOR",
  "RADAR",
  "CAMERA",
  "SPEED",
  "EXIT",
  NULL
};

static struct {
  conf_state state;
  const char * const *cur_menu;
  u8_t menu_sel_ix;

  s8_t motor;
} conf;

//////////////////////////////////////////////////////

// paint a setting bar, setting may range from -128 to 127
static void hud_conf_paint_setting_bar(gcontext *ctx, bool ver_else_hor, s16_t x, s16_t y, s16_t w, s16_t h,
        s8_t setting) {
  GFX_fill(ctx, x, y, w, h, COL_RESET);
  GFX_rect(ctx, x, y, w, h, COL_SET);

  if (ver_else_hor) {
    if (setting < 0) {
      GFX_fill(ctx, x+3, y+h/2, w-5, (((h/2)-2) * (128+setting))/128, COL_SET);
    } else if (setting > 0) {
      s16_t sy = y+2+(((h/2)-2) * setting)/128;
      GFX_fill(ctx, x+3, sy, w-5, y+h/2 - sy, COL_SET);
    }
  } else {
    if (setting > 0) {
      GFX_fill(ctx, x+w/2, y+3, (((w/2)-4) * setting)/128, h-5, COL_SET);
    } else if (setting < 0) {
      s16_t sx = x+2+(((w/2)-2) * (128+setting))/128;
      GFX_fill(ctx, sx, y+3, x+w/2 - sx, h-5, COL_SET);
    }
  }
  u32_t dtick = (((ver_else_hor ? h : w) - 4) << 8) / 10;
  int i;
  for (i = 1; i < 10; i++) {
    if (ver_else_hor) {
      GFX_draw_horizontal_line(ctx, x+1, x+w, y + 2 + ((dtick*i)>>8), COL_MIX);
    } else {
      GFX_draw_vertical_line(ctx, x + 2 + ((dtick*i)>>8), y+1, y+h, COL_MIX);
    }
  }
}

static void hud_conf_paint_title(gcontext *ctx, const char *title) {
  char txt[14];
  memset(txt, 0, sizeof(txt));
  sprint(txt, "%s%s", "CONFIG", title ? title : "");
  int title_len = strlen(txt);
  GFX_printn_big(ctx, txt, 0, (28-title_len*2)/2, 0, COL_SET);

}

static void hud_conf_paint_menu(gcontext *ctx) {
  if (conf.cur_menu == NULL) {
    return;
  }
  const char * const *menu_items = conf.cur_menu;

  hud_conf_paint_title(ctx, menu_items[0]);

  u8_t menu_ix = 0;
  u8_t y = 4;
  bool do_fill = ((SYS_get_time_ms() >> 9) & 1);
  while (menu_items[menu_ix+1]) {
    bool select = do_fill && menu_ix == conf.menu_sel_ix;
    if (select) {
      int item_len = strlen(menu_items[menu_ix+1]);
      GFX_fill(ctx, 8, y*8, item_len*8, 8, COL_SET);
    }
    GFX_printn(ctx, menu_items[menu_ix+1], 0, 1, y, select ? COL_RESET : COL_SET);
    y += 2;
    menu_ix++;
  }
}


////////////////////////////////////////////////////// MAIN CONFIG

static void hud_conf_paint_main(gcontext *ctx) {
  ROVER_view(0,0,200, 50,70,-200, FALSE);
  hud_conf_paint_menu(ctx);
}

static void hud_conf_input_main(input_type in) {
  if (in == UP) {
    if (conf.menu_sel_ix == 0) {
      conf.menu_sel_ix = 4;
    } else {
      conf.menu_sel_ix--;
    }
  } else if (in == DOWN) {
    if (conf.menu_sel_ix == 4) {
      conf.menu_sel_ix = 0;
    } else {
      conf.menu_sel_ix++;
    }
  } else if (in == PRESS) {
    conf.state = conf.menu_sel_ix+1;
  }
}

////////////////////////////////////////////////////// MOTOR CONFIG

static void hud_conf_paint_motor(gcontext *ctx) {
  ROVER_view(0,90,70, 0,0, PI_TRIG_T/4, FALSE);
  hud_conf_paint_title(ctx, " MOTOR");
  hud_conf_paint_setting_bar(ctx, FALSE, 10, 24, ctx->width-10*2, 16, conf.motor);
  rover_angles *ra = ROVER_angle_config();
  ra->anim_d_wheel_left = -10 + ((9*conf.motor)/128);
  ra->anim_d_wheel_right = -10 - ((9*conf.motor)/128);
}

static void hud_conf_input_motor(input_type in) {
  if (in == UP) {
  } else if (in == DOWN) {
  } else if (in == LEFT) {
    conf.motor -=10; // TODO
  } else if (in == RIGHT) {
    conf.motor +=10; // TODO
  } else if (in == PRESS) {
    conf.state = CONF_MAIN;
  }
}


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
    ra->anim_d_radar = 7;
    ra->anim_d_wheel_left = -10;
    ra->anim_d_wheel_right = -10;
    ROVER_view(0,100,400, 0,0,0, TRUE);
  }
  ROVER_paint(ctx);
  switch (conf.state) {
  case CONF_MAIN:
    hud_conf_paint_main(ctx);
    break;
  case CONF_MOTOR:
    hud_conf_paint_motor(ctx);
    break;
  }
}

void HUD_input(input_type in) {
  switch (conf.state) {
  case CONF_MAIN:
    hud_conf_input_main(in);
    break;
  case CONF_MOTOR:
    hud_conf_input_motor(in);
    break;
  }
}
