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
#include "configuration.h"
#include "app.h"

typedef enum {
   CONF_MAIN = 0,
   CONF_STEER,
   CONF_RADAR,
   CONF_CAMERA,
   CONF_GENERAL,
   CONF_EXIT
} conf_state;

static const char * const(MAIN_MENU[]) =  {
  "CONFIG MAIN",
  "STEERING",
  "RADAR",
  "CAMERA",
  "GENERAL",
  "EXIT",
  NULL
};

static const char * const(EXIT_MENU[]) =  {
  "EXIT CONFIG",
  "CANCEL",
  "SAVE & EXIT",
  "DISCARD & EXIT",
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
  s16_t fx_alpha;
  s16_t fx_beta;
  s8_t dx;
  s8_t dy;
  s8_t dz;
  bool dirty;

  configuration_main_t remote;
} conf;

//////////////////////////////////////////////////////

static void hud_conf_set_current(void) {
  const configuration_t *cfg = APP_cfg_get();
  memcpy(&conf.remote, &cfg->main, sizeof(configuration_main_t));
}

static void hud_conf_apply(void) {

}

static void hud_conf_discard(void) {
  APP_remote_load_config();
}


//////////////////////////////////////////////////////


// paint a setting bar, setting may range from -128 to 127
static void hud_conf_paint_setting_bar(gcontext *ctx, bool ver_else_hor, s16_t x, s16_t y, s16_t w, s16_t h,
        s8_t setting) {
  GFX_fill(ctx, x, y, w, h, COL_RESET);
  if (ver_else_hor) {
    GFX_rect(ctx, x, y, w, h-1, COL_SET);
  } else {
    GFX_rect(ctx, x, y, w-1, h, COL_SET);
  }

  if (ver_else_hor) {
    if (setting < 0) {
      GFX_fill(ctx, x+3, y+h/2+1, w-5, (((h/2)-2) * (-setting))/128, COL_SET);
    } else if (setting > 0) {
      s16_t sy = y+2+(((h/2)-2) * (128-setting))/128;
      GFX_fill(ctx, x+3, sy+1, w-5, y+h/2 - sy, COL_SET);
    }
  } else {
    if (setting > 0) {
      GFX_fill(ctx, x+w/2+1, y+3, (((w/2)-4) * setting)/128, h-5, COL_SET);
    } else if (setting < 0) {
      s16_t sx = x+2+(((w/2)-2) * (128+setting))/128;
      GFX_fill(ctx, sx+1, y+3, x+w/2 - sx, h-5, COL_SET);
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
  sprint(txt, "%s", title ? title : "");
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

static void hud_conf_set_menu(const char * const *menu) {
  conf.menu_sel_ix = 0;
  conf.cur_menu = menu;
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
    if (conf.state == CONF_EXIT) {
      hud_conf_set_menu(EXIT_MENU);
    }
  }
}

////////////////////////////////////////////////////// STEERING CONFIG

static void hud_conf_paint_steer(gcontext *ctx) {
  ROVER_view(0,90,70, 0,0, PI_TRIG_T/4, FALSE);
  hud_conf_paint_title(ctx, "SETUP STEER");
  hud_conf_paint_setting_bar(ctx, FALSE, 10, 24, ctx->width-10*2, 16, conf.remote.steer_adjust);
  rover_angles *ra = ROVER_angle_config();
  ra->anim_d_wheel_left = -10 + ((9*conf.remote.steer_adjust)/128);
  ra->anim_d_wheel_right = -10 - ((9*conf.remote.steer_adjust)/128);
}

static void hud_conf_input_steer(input_type in, bool change) {
  if (in & IN_UP) {
  } else if (in & IN_DOWN) {
  }
  if (in & IN_LEFT) {
    conf.dirty = TRUE;
    conf.remote.steer_adjust -=1; // TODO
  } else if (in & IN_RIGHT) {
    conf.dirty = TRUE;
    conf.remote.steer_adjust +=1; // TODO
  }
  if ((in & IN_PRESS) && change) {
    conf.state = CONF_MAIN;
  }
}


////////////////////////////////////////////////////// RADAR CONFIG

static void hud_conf_paint_radar(gcontext *ctx) {
  ROVER_view(0,30,70, 0,-PI_TRIG/32, PI_TRIG_T/4, FALSE);
  hud_conf_paint_title(ctx, "SETUP RADAR");
  hud_conf_paint_setting_bar(ctx, FALSE, 10, 24, ctx->width-10*2, 16, conf.remote.radar_adjust);
  rover_angles *ra = ROVER_angle_config();
  ra->anim_d_wheel_left = 0;
  ra->anim_d_wheel_right = 0;
  ra->anim_d_radar = 0;
  ra->radar = (PI_TRIG_T/4 * (conf.remote.radar_adjust)) / 256;
}

static void hud_conf_input_radar(input_type in, bool change) {
  if (in & IN_UP) {
  } else if (in & IN_DOWN) {
  }
  if (in & IN_LEFT) {
    conf.dirty = TRUE;
    conf.remote.radar_adjust -=1; // TODO
  } else if (in & IN_RIGHT) {
    conf.dirty = TRUE;
    conf.remote.radar_adjust +=1; // TODO
  }
  if ((in & IN_PRESS) && change) {
    conf.state = CONF_MAIN;
  }
}


////////////////////////////////////////////////////// CAMERA CONFIG
//hud_view 0 20 120 0 128 -128

static void hud_conf_paint_camera(gcontext *ctx) {
  ROVER_view(0,20,120, 0,PI_TRIG/5, -PI_TRIG_T/4, FALSE);
  hud_conf_paint_title(ctx, "SETUP CAMERA");
  hud_conf_paint_setting_bar(ctx, FALSE, 30, 24, ctx->width-30*2, 16, conf.remote.cam_pan_adjust);
  hud_conf_paint_setting_bar(ctx, TRUE, 10, 44, 16, ctx->height-44-10, conf.remote.cam_tilt_adjust);
  rover_angles *ra = ROVER_angle_config();
  ra->anim_d_wheel_left = 0;
  ra->anim_d_wheel_right = 0;
  ra->anim_d_radar = 0;
  ra->radar = 0;
  ra->pan = (PI_TRIG_T/4 * -(conf.remote.cam_pan_adjust)) / 256;
  ra->tilt = (PI_TRIG_T/4 * (conf.remote.cam_tilt_adjust)) / 256;
}

static void hud_conf_input_camera(input_type in, bool change) {
  if (in & IN_DOWN) {
    conf.dirty = TRUE;
    conf.remote.cam_tilt_adjust -=1; // TODO
  } else if (in & IN_UP) {
    conf.dirty = TRUE;
    conf.remote.cam_tilt_adjust +=1; // TODO
  }
  if (in & IN_LEFT) {
    conf.dirty = TRUE;
    conf.remote.cam_pan_adjust -=1; // TODO
  } else if (in & IN_RIGHT) {
    conf.dirty = TRUE;
    conf.remote.cam_pan_adjust +=1; // TODO
  }
  if ((in & IN_PRESS) && change) {
    conf.state = CONF_MAIN;
  }
}
////////////////////////////////////////////////////// GENERAL CONFIG

// TODO

////////////////////////////////////////////////////// EXIT

static void hud_conf_paint_exit(gcontext *ctx) {
  if (!conf.dirty) {
    HUD_state(HUD_MAIN);
    return;
  }

  ROVER_view(160,210,250, 0, conf.fx_alpha, conf.fx_beta, FALSE);

  conf.fx_alpha += PI_TRIG_T / 5/ 64;
  conf.fx_beta += PI_TRIG_T / 3 / 64;
  if (conf.fx_alpha > PI_TRIG_T) {
    conf.fx_alpha -= 2*PI_TRIG_T;
  }
  if (conf.fx_beta > PI_TRIG_T) {
    conf.fx_beta -= 2*PI_TRIG_T;
  }
  hud_conf_paint_menu(ctx);
}

static void hud_conf_input_exit(input_type in, bool change) {
  if (!conf.dirty) {
    return;
  }
  if (!change) {
    return;
  }
  if (in & IN_UP) {
    if (conf.menu_sel_ix == 0) {
      conf.menu_sel_ix = 2;
    } else {
      conf.menu_sel_ix--;
    }
    conf.last_selection = SYS_get_time_ms();
  } else if (in & IN_DOWN) {
    if (conf.menu_sel_ix == 2) {
      conf.menu_sel_ix = 0;
    } else {
      conf.menu_sel_ix++;
    }
    conf.last_selection = SYS_get_time_ms();
  }
  if (in & IN_PRESS) {
    switch (conf.menu_sel_ix) {
    case 0:
      hud_conf_set_menu(MAIN_MENU);
      conf.state = CONF_MAIN;
      break;
    case 1:
      hud_conf_apply();
      HUD_state(HUD_MAIN);
      break;
    case 2:
      hud_conf_discard();
      HUD_state(HUD_MAIN);
      break;
    default:
      HUD_state(HUD_MAIN);
      break;
    }
  }
}

//////////////////////////////////////////////////////


void hud_paint_config(gcontext *ctx, bool init) {
  if (init) {
    memset(&conf, 0, sizeof(conf));
    conf.dirty = FALSE;
    hud_conf_set_menu(MAIN_MENU);
    rover_angles *ra = ROVER_angle_config();
    ra->anim_d_radar = 4;
    ra->anim_d_wheel_left = -10;
    ra->anim_d_wheel_right = -10;
    ROVER_view(0,100,400, 0,0,0, TRUE);
    conf.dx = 2;
    conf.dy = 4;
    conf.dz = -5;

    hud_conf_set_current();
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
  case CONF_GENERAL:
  case CONF_EXIT:
    hud_conf_paint_exit(ctx);
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
  case CONF_GENERAL:
  case CONF_EXIT:
    hud_conf_input_exit(in, change);
    break;
  }
}
