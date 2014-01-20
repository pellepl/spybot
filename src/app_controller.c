/*
 * app_controller.c
 *
 *  Created on: Jan 16, 2014
 *      Author: petera
 */

#include "app_controller.h"

#include "taskq.h"
#include "comm_radio.h"
#include "spybot_protocol.h"
#include "miniutils.h"

#ifdef CONFIG_ADC
#include "adc.h"
#endif

#ifdef CONFIG_SPYBOT_VIDEO
#include "cvideo.h"
#include "gfx_bitmap.h"
#include "hud.h"
#endif

#ifdef CONFIG_SPYBOT_INPUT
#include "input.h"
#endif


#define PAIRING_CTRL_SEND_BEACON    PAIRING_STAGE_ONE
#define PAIRING_CTRL_AWAIT_ECHO     PAIRING_STAGE_TWO

// control ram
static app_common *common;
static app_remote *remote;
static configuration_t *app_cfg;

static u32_t app_ctrl_cfg_update = 0;
static u32_t app_ctrl_cfg_update_sent = 0;
static u32_t app_ctrl_remote_req = 0;
#ifdef CONFIG_SPYBOT_VIDEO
gcontext gctx;
static task_timer ui_timer;
static task *ui_task = NULL;
#endif
#ifdef CONFIG_SPYBOT_JOYSTICK
static u16_t joystick_v = 0x800;
static u16_t joystick_h = 0x800;
#endif
static bool app_joy_ctrl;

static void app_control_clr_remote_req(u32_t req) {
  app_ctrl_remote_req &= ~req;
}

static void app_control_set_remote_req(u32_t req) {
  app_ctrl_remote_req |= req;
}


void app_control_set_paired(bool paired) {
  if (paired) {
    app_control_set_remote_req(APP_CTRL_REMOTE_REQ_GET_CONFIG);
  }
}

#ifdef CONFIG_SPYBOT_JOYSTICK
static void app_control_adc_cb(u16_t ch1, u16_t ch2) {
  joystick_v = ch1;
  joystick_h = ch2;
  if (APP_cfg_get_val(CFG_COMMON) & CFG_COMMON_JOY_H_INVERT) {
    joystick_h = 0xfff-joystick_h;
  }
  if (APP_cfg_get_val(CFG_COMMON) & CFG_COMMON_JOY_V_INVERT) {
    joystick_v = 0xfff-joystick_v;
  }
}
#endif

bool APP_get_joystick_control(void) {
  return app_joy_ctrl;
}
void APP_set_joystick_control(bool c) {
  app_joy_ctrl = c;
}

static void app_ctrl_update_camera_ctrl(s8_t h, s8_t v) {
#define CAM_DELTA 4
  if (h < -110) {
    if (remote->pan > -128+CAM_DELTA) {
      remote->pan -= CAM_DELTA;
    }
  } else if (h > 110) {
    if (remote->pan < 127-CAM_DELTA) {
      remote->pan += CAM_DELTA;
    }
  }
  if (v < -110) {
    if (remote->tilt > -128+CAM_DELTA) {
      remote->tilt -= CAM_DELTA;
    }
  } else if (v > 110) {
    if (remote->tilt < 127-CAM_DELTA) {
      remote->tilt += CAM_DELTA;
    }
  }
}


// control ui

#ifdef CONFIG_SPYBOT_JOYSTICK
void APP_get_joystick_reading(s8_t *hori, s8_t *vert) {
  s8_t h = (u16_t)(joystick_h >> 4) - 128;
  s8_t v = (u16_t)(joystick_v >> 4) - 128;
  if (hori) *hori = h;
  if (vert) *vert= v;
}
#endif

#ifdef CONFIG_SPYBOT_VIDEO
static void app_control_ui_task(u32_t a, void *b) {
  APP_remote_set_motor_ctrl(0, 0);
#ifdef CONFIG_SPYBOT_JOYSTICK
  ADC_sample_joystick(app_control_adc_cb);
#endif // CONFIG_SPYBOT_JOYSTICK
  HUD_paint();
#ifdef CONFIG_SPYBOT_JOYSTICK
  INPUT_read(joystick_v, joystick_h);
  if (HUD_get_state() == HUD_MAIN) {
    s8_t hori, vert;
    APP_get_joystick_reading(&hori, &vert);
    if (APP_get_joystick_control() == APP_JOYSTICK_CONTROL_MOTOR) {
      APP_remote_set_motor_ctrl(hori, vert);
    } else {
      APP_remote_set_motor_ctrl(0,0);
      app_ctrl_update_camera_ctrl(hori, vert);
    }

  }
#endif // CONFIG_SPYBOT_JOYSTICK
}
#endif // CONFIG_SPYBOT_VIDEO

// control common

static void app_control_handle_rx(comm_arg *rx, u16_t len, u8_t *data, bool already_received) {
  //u8_t reply[REPLY_MAX_LEN];
  //u8_t reply_ix = 0;
  switch (data[0]) {
  case CMD_RADAR_REPORT:
    // todo
    break;
  case CMD_ALERT:
    // todo
    break;
  case CMD_CHANNEL_CHANGE:
    // todo
    break;
  default:
    DBG(D_APP, D_WARN, "rover: unknown message %02x\n", data[0]);
    COMRAD_reply(REPLY_DENY, 1);
    APP_set_paired_state(FALSE);
    break;
  }
}

static void app_control_handle_ack(u8_t cmd, comm_arg *rx, u16_t len, u8_t *data) {
  u8_t ix = 1;
  switch (cmd) {
  case CMD_CONTROL: {
    u8_t sr = data[ix++];
    if (sr & SPYBOT_SR_ACC) {
      remote->lsm_acc[0] = data[ix++];
      remote->lsm_acc[1] = data[ix++];
      remote->lsm_acc[2] = data[ix++];
    }
    if (sr & SPYBOT_SR_HEADING) {
      remote->lsm_heading = data[ix++];
    }

    break;
  }
  case CMD_SET_CONFIG: {
    if (data[1] == 0) {
      DBG(D_APP, D_WARN, "rover denied setting config\n");
    } else {
      app_ctrl_cfg_update &= ~app_ctrl_cfg_update_sent;
      DBG(D_APP, D_INFO, "rover set config\n");
    }
    app_control_clr_remote_req(APP_CTRL_REMOTE_REQ_SET_CONFIG);
    break;
  }
  case CMD_STORE_CONFIG: {
    if (data[1] == 0) {
      DBG(D_APP, D_WARN, "rover denied storing config\n");
    } else {
      DBG(D_APP, D_INFO, "rover stored config\n");
    }
    app_control_clr_remote_req(APP_CTRL_REMOTE_REQ_STORE_CONFIG);
    break;
  }
  case CMD_LOAD_CONFIG: {
    DBG(D_APP, D_INFO, "rover loaded config\n");
    app_control_clr_remote_req(APP_CTRL_REMOTE_REQ_LOAD_CONFIG);
    break;
  }
  case CMD_GET_CONFIG: {
    if (data[1]) {
      DBG(D_APP, D_INFO, "got rover config\n");
      APP_cfg_set(CFG_STEER_ADJUST, (s8_t)data[2]);
      APP_cfg_set(CFG_RADAR_ADJUST, (s8_t)data[3]);
      APP_cfg_set(CFG_CAM_PAN_ADJUST, (s8_t)data[4]);
      APP_cfg_set(CFG_CAM_TILT_ADJUST, (s8_t)data[5]);
      APP_cfg_set(CFG_COMMON, (u8_t)data[6]);
      app_control_clr_remote_req(APP_CTRL_REMOTE_REQ_GET_CONFIG);
    } else {
      // rover has no config yet
      DBG(D_APP, D_INFO, "rover has no config yet\n");
    }
    break;
  }
  case CMD_LSM_MAG_EXTREMES: {
    // todo
    break;
  }
  case CMD_LSM_ACC_EXTREMES: {
    // todo
    break;
  }
  }
}

static void app_control_dispatch_config(void) {
  app_ctrl_cfg_update_sent = 0;
  u8_t msg[REPLY_MAX_LEN];
  u8_t msg_ix = 0;
  u8_t cfg_bit_ix = 0;
  msg[msg_ix++] = CMD_SET_CONFIG;
  while (cfg_bit_ix < 32 && msg_ix < REPLY_MAX_LEN-2) {
    if (app_ctrl_cfg_update & (1<<cfg_bit_ix)) {
      s16_t val = APP_cfg_get_val(cfg_bit_ix);
      msg[msg_ix++] = cfg_bit_ix;
      msg[msg_ix++] = (val >> 8) & 0xff;
      msg[msg_ix++] = (val & 0xff);
      app_ctrl_cfg_update_sent |= (1<<cfg_bit_ix);
    }
    cfg_bit_ix++;
  }
  msg[msg_ix++] = CFG_STOP;
  APP_tx(msg, msg_ix);
}


static void app_control_tick(void) {
  u8_t msg_divider = (app_ctrl_remote_req & APP_REMOTE_REQ_URGENT) ? 1 : 7;
  if ((app_ctrl_remote_req == 0 || (common->tick_count & msg_divider) != 0)) {
    s8_t left_motor = remote->motor_ctrl[0];
    s8_t right_motor = remote->motor_ctrl[1];
    s8_t pan = remote->pan;
    s8_t tilt  = remote->tilt;
    s8_t radar  = remote->radar;
    u8_t act = 0;
    u8_t sr = SPYBOT_SR_ACC | SPYBOT_SR_HEADING;
    u8_t msg[] = {
        CMD_CONTROL, left_motor, right_motor, pan, tilt, radar, act, sr
    };
    APP_tx(msg, sizeof(msg));
  } else {

    // order is of importance

    if (app_ctrl_remote_req & APP_CTRL_REMOTE_REQ_SET_CONFIG) {
      app_control_dispatch_config();
      DBG(D_APP, D_INFO, "update config, mask 0b%032b\n", app_ctrl_cfg_update_sent);
    } else if (app_ctrl_remote_req & APP_CTRL_REMOTE_REQ_STORE_CONFIG) {
      u8_t msg[] = {
          CMD_STORE_CONFIG
      };
      APP_tx(msg, sizeof(msg));
      DBG(D_APP, D_INFO, "asking to save config\n");
    } else if (app_ctrl_remote_req & APP_CTRL_REMOTE_REQ_LOAD_CONFIG) {
      u8_t msg[] = {
          CMD_LOAD_CONFIG
      };
      APP_tx(msg, sizeof(msg));
      DBG(D_APP, D_INFO, "asking to load config\n");
    } else if (app_ctrl_remote_req & APP_CTRL_REMOTE_REQ_GET_CONFIG) {
      u8_t msg[] = {
          CMD_GET_CONFIG
      };
      APP_tx(msg, sizeof(msg));
      DBG(D_APP, D_INFO, "asking for config\n");
    }

    app_ctrl_remote_req &= ~APP_REMOTE_REQ_URGENT;
  }
}

static void app_control_setup(app_common *com, app_remote *rem, configuration_t *cnf) {
  common = com;
  remote = rem;
  app_cfg = cnf;

  // setup display
#ifdef CONFIG_SPLASH
  extern unsigned const char const img_modbla_car_bmp[14400/8];
#endif

#ifdef CONFIG_SPYBOT_VIDEO
  CVIDEO_init(HUD_vbl);
  CVIDEO_init_gcontext(&gctx);
  CVIDEO_set_effect(79);
#endif

  #ifdef CONFIG_SPLASH
  GFX_draw_image(&gctx, img_modbla_car_bmp, 7, 8, 120/8, 120);
#endif

  // setup ui & input
#ifdef CONFIG_SPYBOT_CONTROL
  INPUT_init();
#endif

#ifdef CONFIG_SPYBOT_VIDEO
  HUD_init(&gctx);
  HUD_state(HUD_MAIN);
  ui_task = TASK_create(app_control_ui_task, TASK_STATIC);
  TASK_start_timer(ui_task, &ui_timer, 0, 0, 0, 50, "ui_upd");
#endif

  APP_remote_set_camera_ctrl(0, 0);
  APP_remote_set_radar_ctrl(RADAR_CTRL_NORMAL);

  app_joy_ctrl = APP_JOYSTICK_CONTROL_MOTOR;
}

static app_impl ctrl_impl;

void APP_control_init(void) {
  ctrl_impl.app_impl_setup = app_control_setup;
  ctrl_impl.app_impl_handle_rx = app_control_handle_rx;
  ctrl_impl.app_impl_handle_ack = app_control_handle_ack;
  ctrl_impl.app_impl_tick = app_control_tick;
  ctrl_impl.app_impl_set_paired = app_control_set_paired;
  ctrl_impl.app_impl_set_remote_req = app_control_set_remote_req;
  ctrl_impl.app_impl_clr_remote_req = app_control_clr_remote_req;
  APP_impl_set(&ctrl_impl);
}


void APP_remote_load_config(void) {
  app_control_set_remote_req(
      APP_CTRL_REMOTE_REQ_LOAD_CONFIG  |
      APP_CTRL_REMOTE_REQ_GET_CONFIG);
}

void APP_remote_store_config(void) {
  app_control_set_remote_req(
      APP_CTRL_REMOTE_REQ_STORE_CONFIG |
      APP_CTRL_REMOTE_REQ_GET_CONFIG);
}

void APP_remote_update_config(spybot_cfg cfg, s16_t val, bool urgent) {
  APP_cfg_set(cfg, val);
  app_control_set_remote_req(APP_CTRL_REMOTE_REQ_SET_CONFIG);
  app_ctrl_cfg_update |= (1<<cfg);
  if (urgent) {
    app_control_set_remote_req(APP_REMOTE_REQ_URGENT);
  }
}
