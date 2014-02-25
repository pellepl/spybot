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

#ifdef CONFIG_SPYBOT_HCSR
#include "range_sens_hcsr04_driver.h"
#endif

#define PAIRING_ROVER_AWAIT_BEACON  PAIRING_STAGE_ONE
#define PAIRING_ROVER_SEND_ECHO     PAIRING_STAGE_TWO

typedef u32_t radar_bm_t;
#define RADAR_BITMAP_ENTRY_SIZE   (sizeof(radar_bm_t)*8)
#define RADAR_BITMAP_ENTRIES      (CONFIG_RADAR_ANGLES / RADAR_BITMAP_ENTRY_SIZE)
#define RADAR_ANG_TO_ENTRY(a)     ((a)/RADAR_BITMAP_ENTRY_SIZE)
#define RADAR_ANG_TO_ENTRY_BIT(a) (1<<((a) & (RADAR_BITMAP_ENTRY_SIZE-1)))


// current common app state of rover
static app_common *common;
// data constantly updated to controller
static app_remote *remote;
// configuration that is currently used by rover
static configuration_t *app_cfg;

static time last_ctrl = 0;

// bitfield indicating which pending remote requests there are from
// rover to controller (being any of APP_CTRL_REMOTE_REQ_*)
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

#ifdef CONFIG_SPYBOT_HCSR
static task_timer radar_timer;
static task *radar_task = NULL;
#endif

static struct {
  // radar values per angle
  s8_t vals[CONFIG_RADAR_ANGLES];
  // bitmap containing radar angles which are changed and unsent
  radar_bm_t dirty_map[RADAR_BITMAP_ENTRIES];
  // bitmap containing radar angles which are changed and while sending
  radar_bm_t changed_sending_map[RADAR_BITMAP_ENTRIES];
  // bitmap containing radar angles being sent
  radar_bm_t dirty_sent_map[RADAR_BITMAP_ENTRIES];
} radar;

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
    // loaded stored configuration successfully
    // start using it by copying loaded config to volatile static config
    int res = CFG_EE_get_config(app_cfg);
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

#ifdef CONFIG_SPYBOT_HCSR
static s16_t blip_angle = 0;
static s16_t blip_dir = 0;
static s16_t old_angle = 0;
static s16_t old_dir = 0;
static void app_rover_radar_cb(u32_t echo_ticks) {
  u8_t radar_log_dist;
#if 1
  if (echo_ticks == (u32_t)(~0) ||
      echo_ticks >= (1<<CONFIG_SPYBOT_RADAR_SENSITIVITY)) {
    radar_log_dist = 255;
  } else {
    radar_log_dist = echo_ticks >> (CONFIG_SPYBOT_RADAR_SENSITIVITY-8);
  }
#else
  // todo for test only
  radar_log_dist = (SYS_get_time_ms() >> 7) & 0xff;
#endif

  if (old_dir == 0) {
    return;
  }

  u8_t start_log_ang, end_log_ang;
  if (old_dir != blip_dir) {
    if (blip_dir > 0) {
      start_log_ang = 0;
      end_log_ang = MAX(blip_angle, old_angle);
    } else {
      start_log_ang = MIN(blip_angle, old_angle);
      end_log_ang = CONFIG_RADAR_ANGLES - 1;
    }
  } else {
    start_log_ang = MIN(blip_angle, old_angle);
    end_log_ang = MAX(blip_angle, old_angle);
  }
  s16_t a;
  for (a = start_log_ang; a < end_log_ang; a++) {
    APP_report_radar_value(a, radar_log_dist);
  }
}

static void app_rover_radar_task(u32_t a, void *b) {
  old_angle = blip_angle;
  old_dir = blip_dir;
  SERVO_get_radar_position(&blip_angle, &blip_dir);
  blip_angle = ((blip_angle + 0x8000) * CONFIG_RADAR_ANGLES) / 0x10000;
  RANGE_SENS_trigger();
}
#endif // CONFIG_SPYBOT_HCSR


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
    // general rover control
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
      app_rover_set_remote_req(APP_ROVER_REMOTE_REQ_RADAR_REPORT | APP_REMOTE_REQ_URGENT);
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
    // sets volatile static configuration on rover side
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
    // stores current volatile static configuration
#ifdef CONFIG_M24M01
    CFG_EE_set_config(app_cfg);
    CFG_EE_store_config();
    reply[reply_ix++] = 1; // ok
    COMRAD_reply(reply, reply_ix);
    DBG(D_APP, D_DEBUG, "rover stored config\n");
#else
    COMRAD_reply(REPLY_DENY, 1);
#endif
    break;


  case CMD_LOAD_CONFIG:
    // loads latest stored configuration (and will start using it by
    // copying it to volatile static configuration)
#ifdef CONFIG_M24M01
    CFG_EE_load_config();
    COMRAD_reply(reply, reply_ix);
    DBG(D_APP, D_DEBUG, "rover loaded config\n");
#else
    COMRAD_reply(REPLY_DENY, 1);
#endif
    break;


  case CMD_GET_CONFIG: {
#ifdef CONFIG_M24M01
    configuration_t cfg;
    int res = CFG_EE_get_config(&cfg);
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
    APP_handle_unknown_msg(rx, len, data, already_received);
    break;
  }
}

static void app_rover_handle_ack(u8_t cmd, comm_arg *rx, u16_t len, u8_t *data) {
  switch (cmd) {
  case CMD_SET_RADIO_CONFIG:
    // todo
    break;
  case CMD_RADAR_REPORT:
  {
    app_rover_clr_remote_req(APP_ROVER_REMOTE_REQ_RADAR_REPORT);
    // clear dirty bits for all sent bits
    // move changed when sending bits to dirty bits
    // clear changed when sending bits
    u32_t a;
    for (a = 0; a < CONFIG_RADAR_ANGLES; a += RADAR_BITMAP_ENTRY_SIZE) {
      radar.dirty_map[RADAR_ANG_TO_ENTRY(a)] =
          (radar.dirty_map[RADAR_ANG_TO_ENTRY(a)] & ~radar.dirty_sent_map[RADAR_ANG_TO_ENTRY(a)])
          | radar.changed_sending_map[RADAR_ANG_TO_ENTRY(a)];
      radar.changed_sending_map[RADAR_ANG_TO_ENTRY(a)] = 0;
    }
    break;
  }
  }
}

static void app_rover_dispatch_radar_report(void) {
  memset(radar.dirty_sent_map, 0, sizeof(radar.dirty_sent_map));
  u8_t msg[REPLY_MAX_LEN];
  u8_t msg_ix = 0;
  msg[msg_ix++] = CMD_RADAR_REPORT;
  u32_t a = 0;
  // find first dirty angle entry
  while (a < CONFIG_RADAR_ANGLES && radar.dirty_map[RADAR_ANG_TO_ENTRY(a)] == 0) {
    a += RADAR_BITMAP_ENTRY_SIZE;
  }
  if (a >= CONFIG_RADAR_ANGLES) {
    // nothing to update, don't send anything - cancel request
    app_rover_clr_remote_req(APP_ROVER_REMOTE_REQ_RADAR_REPORT);
    return;
  }
  // find first dirty angle
  while (a < CONFIG_RADAR_ANGLES &&
      ( radar.dirty_map[RADAR_ANG_TO_ENTRY(a)] & RADAR_ANG_TO_ENTRY_BIT(a)) == 0 ) {
    a++;
  }
  msg[msg_ix++] = a;  // start angle
  u8_t nbr_angs = 0;   // number of angles
  u8_t msg_nbr_angs_ix = msg_ix++; // store len index for later

  while (msg_ix < REPLY_MAX_LEN-1 &&
      a < CONFIG_RADAR_ANGLES &&
      ( radar.dirty_map[RADAR_ANG_TO_ENTRY(a)] & RADAR_ANG_TO_ENTRY_BIT(a) ) != 0) {
    // mark which radar values we're sending
    radar.dirty_sent_map[RADAR_ANG_TO_ENTRY(a)] |= RADAR_ANG_TO_ENTRY_BIT(a);
    msg[msg_ix++] = radar.vals[a];
    a++;
    nbr_angs++;
  }

  // note how many radar values we're sending
  msg[msg_nbr_angs_ix] = nbr_angs;

  APP_tx(msg, msg_ix);
}

static void app_rover_tick(void) {
  if (APP_pair_status() != PAIRING_OK) {
    return;
  }
  if (APP_pair_status() == PAIRING_OK) {
    if (last_ctrl == 0) {
      last_ctrl = SYS_get_time_ms();
    } else {
      if (SYS_get_time_ms() - last_ctrl > 2000) {
        // no message from controller in a while, consider us unpaired
        DBG(D_APP, D_WARN, "UNPAIR due to controller silence\n");
        APP_set_paired_state(FALSE);
        return;
      }
    }
  }

  if (app_rover_remote_req & APP_ROVER_REMOTE_REQ_RADAR_REPORT) {
    // dispatch radar report
    app_rover_dispatch_radar_report();
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

  CFG_EE_init(&eeprom_dev, app_rover_cfg_cb);
  CFG_EE_load_config();

#endif // CONFIG_I2C
#ifdef CONFIG_SPYBOT_MOTOR
  MOTOR_init();
#endif
#ifdef CONFIG_SPYBOT_SERVO
  SERVO_init();
#endif
  mech_task = TASK_create(app_rover_mech_task, TASK_STATIC);
  TASK_start_timer(mech_task, &mech_timer, 0, 0, 0, 20, "mech");
#ifdef CONFIG_SPYBOT_HCSR
  RANGE_SENS_init(app_rover_radar_cb);
  radar_task = TASK_create(app_rover_radar_task, TASK_STATIC);
  TASK_start_timer(radar_task, &radar_timer, 0, 0, 100, 65, "radar");
#endif // CONFIG_SPYBOT_HCSR

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

#ifdef CONFIG_SPYBOT_HCSR

void APP_report_radar_value(u8_t a, s8_t value) {
  if (a >= CONFIG_RADAR_ANGLES) return;
  radar.vals[a] = value;
  radar.dirty_map[RADAR_ANG_TO_ENTRY(a)] |= RADAR_ANG_TO_ENTRY_BIT(a);
  if ((radar.dirty_sent_map[RADAR_ANG_TO_ENTRY(a)] & RADAR_ANG_TO_ENTRY_BIT(a)) != 0) {
    radar.changed_sending_map[RADAR_ANG_TO_ENTRY(a)] |= RADAR_ANG_TO_ENTRY_BIT(a);
  }
}

#endif

s8_t *APP_get_radar_values(void) {
  return radar.vals;
}
