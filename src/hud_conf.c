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
   CONF_STEER,
   CONF_RADAR,
   CONF_CAMERA,
   CONF_SPEED,
   CONF_EXIT
} conf_state;

static const char * const(MAIN_MENU[]) =  {
  " MAIN",
  "STEERING",
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
  time last_selection;
  s16_t fx_x;
  s16_t fx_y;
  s16_t fx_z;
  s8_t dx;
  s8_t dy;
  s8_t dz;

  s8_t steer;
  s8_t radar;
  s8_t pan;
  s8_t tilt;
} conf;

//////////////////////////////////////////////////////

// paint a setting bar, setting may range from -128 to 127
static void hud_conf_paint_setting_bar(gcontext *ctx, bool ver_else_hor, s16_t x, s16_t y, s16_t w, s16_t h,
        s8_t setting) {
  GFX_fill(ctx, x, y, w, h, COL_RESET);
  GFX_rect(ctx, x, y, w, h, COL_SET);

  if (ver_else_hor) {
    if (setting < 0) {
      GFX_fill(ctx, x+3, y+h/2, w-5, (((h/2)-2) * (-setting))/128, COL_SET);
    } else if (setting > 0) {
      s16_t sy = y+2+(((h/2)-2) * (128-setting))/128;
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
  bool do_fill = (((SYS_get_time_ms() - conf.last_selection) >> 9) & 1) == 0;
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

#define FX_X_LIM  40
#define FX_Y_LIM  40
#define FX_Z_LIM  40
#define FX_BITS   3

static void hud_conf_paint_main(gcontext *ctx) {
  int view = (SYS_get_time_ms() / 4096) & 3;
  switch (view) {
  case 0:
    ROVER_view(0+(conf.fx_x>>FX_BITS),0+(conf.fx_y>>FX_BITS),200+(conf.fx_z>>FX_BITS), 50,70,-200, FALSE);
    break;
  case 1:
    ROVER_view(0+(conf.fx_x>>FX_BITS),0+(conf.fx_y>>FX_BITS),200+(conf.fx_z>>FX_BITS), -128, 0, 0, FALSE);
    break;
  case 2:
    ROVER_view(0+(conf.fx_x>>FX_BITS),50+(conf.fx_y>>FX_BITS),200+(conf.fx_z>>FX_BITS), 0, 0, 128, FALSE);
    break;
  case 3:
    ROVER_view(0+(conf.fx_x>>FX_BITS),50+(conf.fx_y>>FX_BITS),200+(conf.fx_z>>FX_BITS), -128, 128, 0, FALSE);
    break;
  }
  rover_angles *ra = ROVER_angle_config();
  ra->anim_d_radar = 4;
  ra->anim_d_wheel_left = -10;
  ra->anim_d_wheel_right = -10;
  ra->tilt = 0;
  ra->pan = 0;
  conf.fx_x += conf.dx;
  conf.fx_y += conf.dy;
  conf.fx_z += conf.dz;
  if (conf.fx_x < -FX_X_LIM<<FX_BITS) {
    conf.dx = 1 + (rand_next() & 7);
    conf.fx_x = -FX_X_LIM<<FX_BITS;
  } else  if (conf.fx_x > FX_X_LIM<<FX_BITS) {
    conf.dx = -(1 + (rand_next() & 7));
    conf.fx_x = FX_X_LIM<<FX_BITS;
  }
  if (conf.fx_y < -FX_Y_LIM<<FX_BITS) {
    conf.dy = 1 + (rand_next() & 7);
    conf.fx_y = -FX_Y_LIM<<FX_BITS;
  } else  if (conf.fx_y > FX_Y_LIM<<FX_BITS) {
    conf.dy = -(1 + (rand_next() & 7));
    conf.fx_y = FX_Y_LIM<<FX_BITS;
  }
  if (conf.fx_z < -FX_Z_LIM<<FX_BITS) {
    conf.dz = 1 + (rand_next() & 7);
    conf.fx_z = -FX_Z_LIM<<FX_BITS;
  } else  if (conf.fx_z > FX_Z_LIM<<FX_BITS) {
    conf.dz = -(1 + (rand_next() & 7));
    conf.fx_z = FX_Z_LIM<<FX_BITS;
  }


  hud_conf_paint_menu(ctx);
}

static void hud_conf_input_main(input_type in, bool change) {
  if (!change) {
    return;
  }
  if (in & IN_UP) {
    if (conf.menu_sel_ix == 0) {
      conf.menu_sel_ix = 4;
    } else {
      conf.menu_sel_ix--;
    }
    conf.last_selection = SYS_get_time_ms();
  } else if (in & IN_DOWN) {
    if (conf.menu_sel_ix == 4) {
      conf.menu_sel_ix = 0;
    } else {
      conf.menu_sel_ix++;
    }
    conf.last_selection = SYS_get_time_ms();
  }
  if (in & IN_PRESS) {
    conf.state = conf.menu_sel_ix+1;
  }
}

////////////////////////////////////////////////////// STEERING CONFIG

static void hud_conf_paint_steer(gcontext *ctx) {
  ROVER_view(0,90,70, 0,0, PI_TRIG_T/4, FALSE);
  hud_conf_paint_title(ctx, " STEER");
  hud_conf_paint_setting_bar(ctx, FALSE, 10, 24, ctx->width-10*2, 16, conf.steer);
  rover_angles *ra = ROVER_angle_config();
  ra->anim_d_wheel_left = -10 + ((9*conf.steer)/128);
  ra->anim_d_wheel_right = -10 - ((9*conf.steer)/128);
}

static void hud_conf_input_steer(input_type in, bool change) {
  if (in & IN_UP) {
  } else if (in & IN_DOWN) {
  }
  if (in & IN_LEFT) {
    conf.steer -=1; // TODO
  } else if (in & IN_RIGHT) {
    conf.steer +=1; // TODO
  }
  if ((in & IN_PRESS) && change) {
    conf.state = CONF_MAIN;
  }
}


////////////////////////////////////////////////////// RADAR CONFIG

static void hud_conf_paint_radar(gcontext *ctx) {
  ROVER_view(0,30,70, 0,-PI_TRIG/32, PI_TRIG_T/4, FALSE);
  hud_conf_paint_title(ctx, " RADAR");
  hud_conf_paint_setting_bar(ctx, FALSE, 10, 24, ctx->width-10*2, 16, conf.radar);
  rover_angles *ra = ROVER_angle_config();
  ra->anim_d_wheel_left = 0;
  ra->anim_d_wheel_right = 0;
  ra->anim_d_radar = 0;
  ra->radar = (PI_TRIG_T/4 * (conf.radar)) / 256;
}

static void hud_conf_input_radar(input_type in, bool change) {
  if (in & IN_UP) {
  } else if (in & IN_DOWN) {
  }
  if (in & IN_LEFT) {
    conf.radar -=1; // TODO
  } else if (in & IN_RIGHT) {
    conf.radar +=1; // TODO
  }
  if ((in & IN_PRESS) && change) {
    conf.state = CONF_MAIN;
  }
}


////////////////////////////////////////////////////// CAMERA CONFIG
//hud_view 0 20 120 0 128 -128

static void hud_conf_paint_camera(gcontext *ctx) {
  ROVER_view(0,20,120, 0,PI_TRIG/5, -PI_TRIG_T/4, FALSE);
  hud_conf_paint_title(ctx, " CAMERA");
  hud_conf_paint_setting_bar(ctx, FALSE, 30, 24, ctx->width-30*2, 16, conf.pan);
  hud_conf_paint_setting_bar(ctx, TRUE, 10, 44, 16, ctx->height-44-10, conf.tilt);
  rover_angles *ra = ROVER_angle_config();
  ra->anim_d_wheel_left = 0;
  ra->anim_d_wheel_right = 0;
  ra->anim_d_radar = 0;
  ra->radar = 0;
  ra->pan = (PI_TRIG_T/4 * -(conf.pan)) / 256;
  ra->tilt = (PI_TRIG_T/4 * (conf.tilt)) / 256;
}

static void hud_conf_input_camera(input_type in, bool change) {
  if (in & IN_DOWN) {
    conf.tilt -=1; // TODO
  } else if (in & IN_UP) {
    conf.tilt +=1; // TODO
  }
  if (in & IN_LEFT) {
    conf.pan -=1; // TODO
  } else if (in & IN_RIGHT) {
    conf.pan +=1; // TODO
  }
  if ((in & IN_PRESS) && change) {
    conf.state = CONF_MAIN;
  }
}
////////////////////////////////////////////////////// SPEED CONFIG


////////////////////////////////////////////////////// SAVE


//////////////////////////////////////////////////////


void hud_paint_config(gcontext *ctx, bool init) {
  if (init) {
    memset(&conf, 0, sizeof(conf));
    conf.cur_menu = MAIN_MENU;
    rover_angles *ra = ROVER_angle_config();
    ra->anim_d_radar = 4;
    ra->anim_d_wheel_left = -10;
    ra->anim_d_wheel_right = -10;
    ROVER_view(0,100,400, 0,0,0, TRUE);
    conf.dx = 2;
    conf.dy = 4;
    conf.dz = -5;
  }

  ROVER_paint(ctx);

  switch (conf.state) {
  case CONF_MAIN:
    hud_conf_paint_main(ctx);
    break;
  case CONF_STEER:
    hud_conf_paint_steer(ctx);
    break;
  case CONF_RADAR:
    hud_conf_paint_radar(ctx);
    break;
  case CONF_CAMERA:
    hud_conf_paint_camera(ctx);
    break;
  }
}

void HUD_input(input_type in, bool change) {
  switch (conf.state) {
  case CONF_MAIN:
    hud_conf_input_main(in, change);
    break;
  case CONF_STEER:
    hud_conf_input_steer(in, change);
    break;
  case CONF_RADAR:
    hud_conf_input_radar(in, change);
    break;
  case CONF_CAMERA:
    hud_conf_input_camera(in, change);
    break;
  }
}