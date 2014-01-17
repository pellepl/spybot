/*
 * app_rover.c
 *
 *  Created on: Jan 16, 2014
 *      Author: petera
 */

#include "app_rover.h"

#include "taskq.h"
#include "comm_radio.h"
#include "spybot_protocol.h"
#include "miniutils.h"

#ifdef CONFIG_I2C
#include "lsm303_driver.h"
#include "m24m01_driver.h"
#include "i2c_driver.h"
#include "trig_q.h"
#endif

#ifdef CONFIG_SPYBOT_MOTOR
#include "motor.h"
#endif

#ifdef CONFIG_SPYBOT_SERVO
#include "servo.h"
#endif

#ifdef CONFIG_M24M01
#include "configuration_ee.h"
#endif

#define PAIRING_ROVER_AWAIT_BEACON  PAIRING_STAGE_ONE
#define PAIRING_ROVER_SEND_ECHO     PAIRING_STAGE_TWO

static app_common *common;
static app_remote *remote;
static configuration_t *app_cfg;

static time last_ctrl = 0;

static u32_t app_rover_remote_req = 0;
#ifdef CONFIG_I2C
task_mutex i2c_mutex = TASK_MUTEX_INIT;

#ifdef CONFIG_SPYBOT_LSM
lsm303_dev lsm_dev;
static task_timer lsm_timer;
static task *lsm_task = NULL;
static bool reading_lsm = FALSE;
s16_t mag_extremes[3][2]; // x,y,z : min,max
s16_t acc_extremes[3][2]; // x,y,z : min,max
#endif
#ifdef CONFIG_M24M01
m24m01_dev eeprom_dev;
#endif
#endif // CONFIG_I2C

static task_timer mech_timer;
static task *mech_task = NULL;

static void app_rover_clr_remote_req(u32_t req) {
  app_rover_remote_req &= ~req;
}

static void app_rover_set_remote_req(u32_t req) {
  app_rover_remote_req |= req;
}

void app_rover_set_paired(bool paired) {
  last_ctrl = 0;
  if (!paired) {
    MOTOR_go(0);
  }
}

#ifdef CONFIG_I2C

// rover eeprom config

static void app_rover_cfg_cb(cfg_state state, cfg_ee_state ee_state, int res) {
  DBG(D_APP, D_DEBUG,"CFG EE CB state:%i eestate:%i res:%i\n", state, ee_state, res);
  if (state == LOAD_CFG && res == CFG_OK) {
    int res = CFG_get_config(app_cfg);
    ASSERT(res == CFG_OK);
  }
}

// rover lsm
#ifdef CONFIG_SPYBOT_LSM
static void app_rover_lsm_cb_task(u32_t ares, void *adev) {
  lsm303_dev *dev = (lsm303_dev *)dev;
  int res = (int)ares;
  if (res == I2C_ERR_LSM303_BAD_READ) {
    DBG(D_APP, D_WARN, "lsm bad read\n");
  } else  if (res != I2C_OK) {
    DBG(D_APP, D_WARN, "lsm err %i\n", res);
    I2C_config(_I2C_BUS(0), 100000);
  }
  TASK_mutex_unlock(&i2c_mutex);
  reading_lsm = FALSE;

  u16_t heading_raw = lsm_get_heading(&lsm_dev);
  s16_t *acc = lsm_get_acc_reading(&lsm_dev);
  s16_t *mag = lsm_get_mag_reading(&lsm_dev);
  remote->lsm_heading = heading_raw >> 8;
  remote->lsm_acc[0] = acc[0] >> 4;
  remote->lsm_acc[1] = acc[1] >> 4;
  remote->lsm_acc[2] = acc[2] >> 4;
  int i;
  for (i = 0; i < 3; i++) {
    acc_extremes[i][0] = MIN(acc_extremes[i][0], acc[i]);
    acc_extremes[i][1] = MAX(acc_extremes[i][1], acc[i]);
    mag_extremes[i][0] = MIN(mag_extremes[i][0], mag[i]);
    mag_extremes[i][1] = MAX(mag_extremes[i][1], mag[i]);
  }
}

static void app_rover_lsm_cb_irq(lsm303_dev *dev, int res) {
  task *t = TASK_create(app_rover_lsm_cb_task, 0);
  ASSERT(t);
  TASK_run(t, res, dev);
}

static void app_rover_lsm_task(u32_t a, void *b) {
  if (reading_lsm) {
    return;
  }
  if (!TASK_mutex_lock(&i2c_mutex)) {
    return;
  }

  reading_lsm = TRUE;
  lsm_read_both(&lsm_dev);
}
#endif // CONFIG_SPYBOT_LSM
#endif // CONFIG_I2C

static void app_rover_mech_task(u32_t a, void *b) {
#ifdef CONFIG_SPYBOT_MOTOR
  MOTOR_update();
#endif // CONFIG_SPYBOT_MOTOR
#ifdef CONFIG_SPYBOT_SERVO
  SERVO_update();
#endif // CONFIG_SPYBOT_SERVO
}

static void app_rover_handle_rx(comm_arg *rx, u16_t len, u8_t *data, bool already_received) {
  last_ctrl = SYS_get_time_ms();
  u8_t reply[REPLY_MAX_LEN];
  u8_t reply_ix = 0;
  reply[reply_ix++] = ACK_OK;
  switch (data[0]) {

  case CMD_CONTROL: {
    u8_t act = (u8_t)data[6];
    u8_t sr = (u8_t)data[7];

    reply[reply_ix++] = sr;
    remote->motor_ctrl[0] = (s8_t)data[1];
    remote->motor_ctrl[1] = (s8_t)data[2];

    if (already_received) {

    }
    remote->pan = (s8_t)data[3];
    remote->tilt = (s8_t)data[4];
    remote->radar = (s8_t)data[5];

    // action_mask
    // todo
    (void)act;

    // status_mask
    if (sr & SPYBOT_SR_ACC) {
      reply[reply_ix++] = remote->lsm_acc[0];
      reply[reply_ix++] = remote->lsm_acc[1];
      reply[reply_ix++] = remote->lsm_acc[2];
    }
    if (sr & SPYBOT_SR_HEADING) {
      reply[reply_ix++] = remote->lsm_heading;
    }
    if (sr & SPYBOT_SR_TEMP) {
      // todo
    }
    if (sr & SPYBOT_SR_BATT) {
      // todo
    }
    if (sr & SPYBOT_SR_RADAR) {
      // todo
    }
    COMRAD_reply(reply, reply_ix);

#ifdef CONFIG_SPYBOT_MOTOR
    MOTOR_control(remote->motor_ctrl[0], remote->motor_ctrl[1]);
#endif
#ifdef CONFIG_SPYBOT_SERVO
    SERVO_control(SERVO_CAM_PAN, remote->pan);
    SERVO_control(SERVO_CAM_TILT, remote->tilt);
    SERVO_control_radar(remote->radar);
#endif

    break;
  }


  case CMD_SET_CONFIG: {
    int data_ix = 1;
    while (data_ix < COMM_LNK_MAX_DATA && data[data_ix] != CFG_STOP) {
      spybot_cfg cfg_type = data[data_ix];
      s16_t cfg_val = (data[data_ix + 1] << 8) | (data[data_ix + 2]);
      APP_cfg_set(cfg_type, cfg_val);
      data_ix += 3;
      DBG(D_APP, D_DEBUG, "rover remote update cfg %i = %i\n", cfg_type, cfg_val);
    }
    reply[reply_ix++] = 1; // ok
    COMRAD_reply(reply, reply_ix);
    break;
  }


  case CMD_STORE_CONFIG:
#ifdef CONFIG_M24M01
    CFG_set_config(app_cfg);
    CFG_store_config();
    reply[reply_ix++] = 1; // ok
    COMRAD_reply(reply, reply_ix);
    DBG(D_APP, D_DEBUG, "rover stored config\n");
#else
    COMRAD_reply(REPLY_DENY, 1);
#endif
    break;


  case CMD_LOAD_CONFIG:
#ifdef CONFIG_M24M01
    CFG_load_config();
    COMRAD_reply(reply, reply_ix);
    DBG(D_APP, D_DEBUG, "rover loaded config\n");
#else
    COMRAD_reply(REPLY_DENY, 1);
#endif
    break;


  case CMD_GET_CONFIG: {
#ifdef CONFIG_M24M01
    configuration_t cfg;
    int res = CFG_get_config(&cfg);
    if (res == CFG_OK) {
      reply[reply_ix++] = 1; // ok, have config
      reply[reply_ix++] = app_cfg->main.steer_adjust;
      reply[reply_ix++] = app_cfg->main.radar_adjust;
      reply[reply_ix++] = app_cfg->main.cam_pan_adjust;
      reply[reply_ix++] = app_cfg->main.cam_tilt_adjust;
      reply[reply_ix++] = app_cfg->main.common;
      DBG(D_APP, D_DEBUG, "rover returned config\n");
    } else {
      reply[reply_ix++] = 0; // nok, no config yet - try later
      DBG(D_APP, D_DEBUG, "rover no config to return\n");
    }
#else
    reply[reply_ix++] = 0; // nok, no config ever
#endif

    COMRAD_reply(reply, reply_ix);
    break;
  }


  case CMD_LSM_MAG_EXTREMES:
    // todo
    break;


  case CMD_LSM_ACC_EXTREMES:
    // todo
    break;


  default:
    DBG(D_APP, D_WARN, "rover: unknown message %02x\n", data[0]);
    COMRAD_reply(REPLY_DENY, 1);
    APP_set_paired_state(FALSE);
    break;
  }
}

static void app_rover_handle_ack(u8_t cmd, comm_arg *rx, u16_t len, u8_t *data) {
  switch (cmd) {
  case CMD_CHANNEL_CHANGE:
    // todo
    break;
  }
}

static void app_rover_tick(void) {
  if (APP_pair_status() == PAIRING_OK) {
    if (last_ctrl == 0) {
      last_ctrl = SYS_get_time_ms();
    } else {
      if (SYS_get_time_ms() - last_ctrl > 2000) {
        APP_set_paired_state(FALSE);
      }
    }
  }
}

static void app_rover_setup(app_common *com, app_remote *rem, configuration_t *cnf) {
  common = com;
  remote = rem;
  app_cfg = cnf;

#ifdef CONFIG_I2C
  I2C_config(_I2C_BUS(0), 100000);

  lsm_open(&lsm_dev, _I2C_BUS(0), FALSE, app_rover_lsm_cb_irq);
  // lock mutex for lsm config
  if (!TASK_mutex_lock(&i2c_mutex)) {
    ASSERT(FALSE); // should never happen, during init
  }
  int res = lsm_config_default(&lsm_dev);
  ASSERT(res == I2C_OK);
  lsm_set_lowpass(&lsm_dev, 50, 50);

  lsm_task = TASK_create(app_rover_lsm_task, TASK_STATIC);
  TASK_start_timer(lsm_task, &lsm_timer, 0, 0, 500, 100, "lsm_read");

  CFG_init(&eeprom_dev, app_rover_cfg_cb);
  CFG_load_config();

#endif // CONFIG_I2C
#ifdef CONFIG_SPYBOT_MOTOR
  MOTOR_init();
#endif
#ifdef CONFIG_SPYBOT_SERVO
  SERVO_init();
#endif
  mech_task = TASK_create(app_rover_mech_task, TASK_STATIC);
  TASK_start_timer(mech_task, &mech_timer, 0, 0, 0, 20, "mech");

#ifdef CONFIG_SPYBOT_LSM
int i;
for (i = 0; i < 3; i++) {
  acc_extremes[i][0] = S16_MAX;
  acc_extremes[i][1] = S16_MIN;
  mag_extremes[i][0] = S16_MAX;
  mag_extremes[i][1] = S16_MIN;
}
#endif
}

static app_impl rover_impl;

void APP_rover_init(void) {
  rover_impl.app_impl_setup = app_rover_setup;
  rover_impl.app_impl_handle_rx = app_rover_handle_rx;
  rover_impl.app_impl_handle_ack = app_rover_handle_ack;
  rover_impl.app_impl_tick = app_rover_tick;
  rover_impl.app_impl_set_remote_req = app_rover_set_remote_req;
  rover_impl.app_impl_clr_remote_req = app_rover_clr_remote_req;
  rover_impl.app_impl_set_paired = app_rover_set_paired;

  APP_impl_set(&rover_impl);
}


#ifdef CONFIG_SPYBOT_LSM

void APP_get_acc_extremes(s16_t x[3][2], bool reset) {
  memcpy(&x, acc_extremes, sizeof(s16)*3*2);
  if (reset) {
    int i;
    for (i = 0; i < 3; i++) {
      acc_extremes[i][0] = S16_MAX;
      acc_extremes[i][1] = S16_MIN;
    }
  }
}

void APP_get_mag_extremes(s16_t x[3][2], bool reset) {
  memcpy(&x, mag_extremes, sizeof(s16)*3*2);
  if (reset) {
    int i;
    for (i = 0; i < 3; i++) {
      mag_extremes[i][0] = S16_MAX;
      mag_extremes[i][1] = S16_MIN;
    }
  }
}

#endif
