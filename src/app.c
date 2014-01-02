/*
 * app.c
 *
 *  Created on: Jan 2, 2014
 *      Author: petera
 */

#include "cvideo.h"
#include "gfx_bitmap.h"
#include "hud.h"
#include "lsm303_driver.h"
#include "app.h"
#include "i2c_driver.h"
#include "taskq.h"
#include "comm_radio.h"
#include "spybot_protocol.h"
#include "miniutils.h"

static struct {
  u16_t tx_seqno;
  bool comrad_busy;
} common;

// control ram
gcontext gctx;
static task_timer hud_timer;
static task *hud_task = NULL;
static task_timer comm_timer;
static task *comm_task = NULL;

// rover ram
lsm303_dev lsm_dev;

// rover funcs
static void app_rover_lsm_cb(lsm303_dev *dev, int res) {
  if (res != I2C_OK) {
    //print("lsm error %i\n", res);
    I2C_config(_I2C_BUS(0), 100000);
  }
}

// control funcs
static void app_control_hud_task(u32_t a, void *b) {
  HUD_paint();
  lsm_read_both(&lsm_dev); // TODO for test
}

static void app_rover_init(void) {
  I2C_config(_I2C_BUS(0), 100000);

  lsm_open(&lsm_dev, _I2C_BUS(0), FALSE, app_rover_lsm_cb);
  lsm_config_default(&lsm_dev);
  lsm_set_lowpass(&lsm_dev, 50, 50);
}

static void app_control_comm_task(u32_t a, void *p) {
  if (common.comrad_busy) {
    return;
  }
  u8_t ctrl[] = {
      CMD_CONTROL, 0, 0, 0, 0xff
  };
  int res = COMRAD_send(ctrl, sizeof(ctrl), TRUE);
  if (res >= R_COMM_OK) {
    common.tx_seqno = res;
    common.comrad_busy = TRUE;
  }
}

static void app_control_init(void) {
#ifdef CONFIG_SPLASH
  extern unsigned const char const img_modbla_car_bmp[14400/8];
#endif
  CVIDEO_init(HUD_vbl);
  CVIDEO_init_gcontext(&gctx);
  CVIDEO_set_effect(79);
#ifdef CONFIG_SPLASH
  GFX_draw_image(&gctx, img_modbla_car_bmp, 7, 8, 120/8, 120);
#endif

  HUD_init(&gctx, &lsm_dev);
  HUD_state(HUD_MAIN);

  hud_task = TASK_create(app_control_hud_task, TASK_STATIC);
  TASK_start_timer(hud_task, &hud_timer, 0, 0, 0, 50, "hud_upd");

  comm_task = TASK_create(app_control_comm_task, TASK_STATIC);
  TASK_start_timer(comm_task, &comm_timer, 0, 0, 100, 110, "comm");
}

void APP_init(void) {
  // common
  memset(&common, 0, sizeof(common));
  COMRAD_init();

  // TODO for test
  app_rover_init();
#ifndef SECONDARY
  app_control_init();
#endif
}

void APP_comrad_rx(comm_arg *rx, u16_t len, u8_t *data, bool already_received) {

}

void APP_comrad_ack(comm_arg *rx, u16_t seq_no, u16_t len, u8_t *data) {
  if (seq_no == common.tx_seqno) {
    common.comrad_busy = FALSE;
  }
}

void APP_comrad_err(u16_t seq_no, int err) {
  if (seq_no == common.tx_seqno) {
    common.comrad_busy = FALSE;
  }
}
