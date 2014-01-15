/*
 * app.c
 *
 *  Created on: Jan 2, 2014
 *      Author: petera
 */

#include "app.h"
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

#ifdef CONFIG_SPYBOT_MOTOR
#include "motor.h"
#endif

#ifdef CONFIG_M24M01
#include "configuration_ee.h"
#endif

#define PAIRING_CTRL_SEND_BEACON    0
#define PAIRING_CTRL_AWAIT_ECHO     1
#define PAIRING_ROVER_AWAIT_BEACON  0
#define PAIRING_ROVER_SEND_ECHO     1
#define PAIRING_OK                  2

#define ACK_OK                      0x01
#define ACK_DENY                    0x00

#ifdef CONFIG_SPYBOT_ROVER
#define BEACON_RECURRENCE           409
#define COMM_RECURRENCE             113
#else
#define BEACON_RECURRENCE           401
#define COMM_RECURRENCE             97
#endif

#define APP_CTRL_REMOTE_REQ_SET_CONFIG    (1<<0)
#define APP_CTRL_REMOTE_REQ_STORE_CONFIG  (1<<1)
#define APP_CTRL_REMOTE_REQ_LOAD_CONFIG   (1<<2)
#define APP_CTRL_REMOTE_REQ_GET_CONFIG    (1<<3)

static struct {
  u16_t tx_seqno;
  u8_t tx_cmd;
  bool comrad_busy;
  u8_t pair_state;
  u8_t pair_wait_cnt;
  task_timer tick_timer;
  task *tick_task;
  u32_t tick_count;
} common;

static u8_t const REPLY_OK[] = {ACK_OK};
static u8_t const REPLY_DENY[] = {ACK_DENY};


// control ram
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

// rover ram
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

#ifdef CONFIG_SPYBOT_MOTOR
static task_timer motor_timer;
static task *motor_task = NULL;
#endif

// ram updated over RF
static struct {
  s8_t lsm_heading;
  s8_t lsm_acc[3];
  s8_t motor_ctrl[2]; // steer vector : [0] hori, [1] vert
  s8_t pan;
  s8_t tilt;
  s8_t radar;

} remote;

configuration_t app_cfg;

// helpers

static int app_tx(u8_t *data, u16_t len) {
  if (common.comrad_busy) return R_COMM_PHY_TRY_LATER;
  int res = COMRAD_send(data, len, TRUE);
  if (res >= R_COMM_OK) {
    common.tx_seqno = res;
    common.tx_cmd = data[0];
    common.comrad_busy = TRUE;
  }
  return res;
}

static void app_set_paired_state(bool paired) {
  if (!paired) {
#ifdef CONFIG_SPYBOT_APP_MASTER
    common.pair_state = PAIRING_CTRL_SEND_BEACON;
#endif
#ifdef CONFIG_SPYBOT_APP_CLIENT
    common.pair_state = PAIRING_ROVER_AWAIT_BEACON;
#endif
#ifdef CONFIG_SPYBOT_MOTOR
    MOTOR_go(0);
#endif
  } else {
    common.pair_state = PAIRING_OK;
#ifdef CONFIG_SPYBOT_APP_MASTER
    app_ctrl_remote_req |= (APP_CTRL_REMOTE_REQ_GET_CONFIG);
#endif

  }
  COMRAD_report_paired(paired);
}


///////////////////
// rover funcs
///////////////////


#ifdef CONFIG_I2C

// rover eeprom config

static void app_rover_cfg_cb(cfg_state state, cfg_ee_state ee_state, int res) {
  // todo
  DBG(D_APP, D_DEBUG,"CFG EE CB state:%i eestate:%i res:%i\n", state, ee_state, res);
  if (state == LOAD_CFG && res == CFG_OK) {
    int res = CFG_get_config(&app_cfg);
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
  remote.lsm_heading = heading_raw >> 8;
  remote.lsm_acc[0] = acc[0] >> 4;
  remote.lsm_acc[1] = acc[1] >> 4;
  remote.lsm_acc[2] = acc[2] >> 4;
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

#ifdef CONFIG_SPYBOT_MOTOR
static void app_rover_motor_task(u32_t a, void *b) {
  MOTOR_update();
}
#endif // CONFIG_SPYBOT_MOTOR

// rover common

static void app_rover_init(void) {
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
  motor_task = TASK_create(app_rover_motor_task, TASK_STATIC);
  TASK_start_timer(motor_task, &motor_timer, 0, 0, 0, 20, "motor");
#endif
}

#define REPLY_MAX_LEN (COMM_LNK_MAX_DATA-COMM_H_SIZE-1)

static void app_rover_handle_rx(comm_arg *rx, u16_t len, u8_t *data, bool already_received) {
  u8_t reply[REPLY_MAX_LEN];
  u8_t reply_ix = 0;
  reply[reply_ix++] = ACK_OK;
  switch (data[0]) {


  case CMD_CONTROL: {
    u8_t act = (u8_t)data[6];
    u8_t sr = (u8_t)data[7];

    reply[reply_ix++] = sr;
    remote.motor_ctrl[0] = (s8_t)data[1];
    remote.motor_ctrl[1] = (s8_t)data[2];

    if (already_received) {

    }
    remote.pan = (s8_t)data[3];
    remote.tilt = (s8_t)data[4];
    remote.radar = (s8_t)data[5];

    // action_mask
    // todo
    (void)act;

    // status_mask
    if (sr & SPYBOT_SR_ACC) {
      reply[reply_ix++] = remote.lsm_acc[0];
      reply[reply_ix++] = remote.lsm_acc[1];
      reply[reply_ix++] = remote.lsm_acc[2];
    }
    if (sr & SPYBOT_SR_HEADING) {
      reply[reply_ix++] = remote.lsm_heading;
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
    MOTOR_control_vector(remote.motor_ctrl[0], remote.motor_ctrl[1]);
#endif
#ifdef CONFIG_SPYBOT_SERVO
    // todo
    //SERVO_control(SERVO_CAM_PAN, remote.pan);
    //SERVO_control(SERVO_CAM_TILT, remote.tilt);
    //SERVO_control(SERVO_CAM_RADAR, remote.radar);
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
    CFG_set_config(&app_cfg);
    CFG_store_config();
    reply[reply_ix++] = 1; // ok
    COMRAD_reply(reply, reply_ix);
    DBG(D_APP, D_DEBUG, "rover stored config\n");
#else
    COMRAD_reply(REPLY_DENY, 1);
#endif
    // todo
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
      reply[reply_ix++] = app_cfg.main.steer_adjust;
      reply[reply_ix++] = app_cfg.main.radar_adjust;
      reply[reply_ix++] = app_cfg.main.cam_pan_adjust;
      reply[reply_ix++] = app_cfg.main.cam_tilt_adjust;
      reply[reply_ix++] = app_cfg.main.common;
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
    app_set_paired_state(FALSE);
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
  // rover
  if (common.pair_state == PAIRING_ROVER_SEND_ECHO) {
    u32_t v = SPYBOT_VERSION;
    u32_t b = SYS_build_number();
    u8_t beacon_echo[] =
    { CMD_PAIRING_ECHO ,
        (v >> 24) & 0xff, (v >> 16) & 0xff, (v >> 8) & 0xff, (v & 0xff),
        (b >> 24), (b >> 16), (b >> 8), (b & 0xff),
    };
    app_tx(beacon_echo, sizeof(beacon_echo));
  } else if (common.pair_state == PAIRING_OK) {
    // todo
  }
}

///////////////////
// control funcs
///////////////////


// control joystick

#ifdef CONFIG_SPYBOT_JOYSTICK
static void app_control_adc_cb(u16_t ch1, u16_t ch2) {
  joystick_v = ch1;
  joystick_h = ch2;
}
#endif

// control ui

#ifdef CONFIG_SPYBOT_VIDEO
static void app_control_ui_task(u32_t a, void *b) {
  APP_remote_set_motor_ctrl(0, 0);
#ifdef CONFIG_SPYBOT_JOYSTICK
  ADC_sample(app_control_adc_cb);
#endif // CONFIG_SPYBOT_JOYSTICK
  HUD_paint();
#ifdef CONFIG_SPYBOT_JOYSTICK
  INPUT_read(joystick_v, joystick_h);
  if (HUD_get_state() == HUD_MAIN) {
    APP_remote_set_motor_ctrl((u16_t)(joystick_h >> 4) - 128, (u16_t)(joystick_v >> 4) - 128);
  }
#endif // CONFIG_SPYBOT_JOYSTICK
}
#endif // CONFIG_SPYBOT_VIDEO

// control common

static void app_control_init(void) {
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
}

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
    app_set_paired_state(FALSE);
    break;
  }
}

static void app_control_handle_ack(u8_t cmd, comm_arg *rx, u16_t len, u8_t *data) {
  u8_t ix = 1;
  switch (cmd) {
  case CMD_CONTROL: {
    u8_t sr = data[ix++];
    if (sr & SPYBOT_SR_ACC) {
      remote.lsm_acc[0] = data[ix++];
      remote.lsm_acc[1] = data[ix++];
      remote.lsm_acc[2] = data[ix++];
    }
    if (sr & SPYBOT_SR_HEADING) {
      remote.lsm_heading = data[ix++];
    }

    break;
  }
  case CMD_SET_CONFIG: {
    if (data[1] == 0) {
      DBG(D_APP, D_WARN, "rover denied setting config\n");
    } else {
      DBG(D_APP, D_INFO, "rover set config\n");
    }
    app_ctrl_remote_req &= ~(APP_CTRL_REMOTE_REQ_SET_CONFIG);
    break;
  }
  case CMD_STORE_CONFIG: {
    if (data[1] == 0) {
      DBG(D_APP, D_WARN, "rover denied storing config\n");
    } else {
      DBG(D_APP, D_INFO, "rover stored config\n");
    }
    app_ctrl_remote_req &= ~(APP_CTRL_REMOTE_REQ_STORE_CONFIG);
    break;
  }
  case CMD_LOAD_CONFIG: {
    DBG(D_APP, D_INFO, "rover loaded config\n");
    app_ctrl_remote_req &= ~(APP_CTRL_REMOTE_REQ_LOAD_CONFIG);
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
      app_ctrl_remote_req &= ~(APP_CTRL_REMOTE_REQ_GET_CONFIG);
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

static void app_control_tick(void) {
  if (common.pair_state == PAIRING_CTRL_SEND_BEACON) {
    TASK_set_timer_recurrence(&common.tick_timer, BEACON_RECURRENCE);

    u32_t v = SPYBOT_VERSION;
    u32_t b = SYS_build_number();

    u8_t beacon[] = { CMD_PAIRING_BEACON,
        (v >> 24) & 0xff, (v >> 16) & 0xff, (v >> 8) & 0xff, (v & 0xff),
        (b >> 24), (b >> 16), (b >> 8), (b & 0xff),
    };
    int res = app_tx(beacon, sizeof(beacon));
    if (res >= R_COMM_OK) {
      DBG(D_APP, D_DEBUG, "ctrl sent beacon: seq %04x\n", res);
    } else {
      DBG(D_APP, D_WARN, "ctrl failed sending beacon %i\n", res);
    }
  } else if (common.pair_state == PAIRING_CTRL_AWAIT_ECHO) {
    common.pair_wait_cnt++;
    if (common.pair_wait_cnt > 3) {
      DBG(D_APP, D_DEBUG, "ctrl echo wait timed out\n");
      app_set_paired_state(FALSE);
    }
  } else if (common.pair_state == PAIRING_OK) {

    if (app_ctrl_remote_req == 0 || (common.tick_count & 0x7) != 0) {
      s8_t left_motor = remote.motor_ctrl[0];
      s8_t right_motor = remote.motor_ctrl[1];
      s8_t pan = remote.pan;
      s8_t tilt  = remote.tilt;
      s8_t radar  = remote.radar;
      u8_t act = 0;
      u8_t sr = SPYBOT_SR_ACC | SPYBOT_SR_HEADING;
      u8_t msg[] = {
          CMD_CONTROL, left_motor, right_motor, pan, tilt, radar, act, sr
      };
      app_tx(msg, sizeof(msg));
    } else {
      // order is of importance

      if (app_ctrl_remote_req & APP_CTRL_REMOTE_REQ_SET_CONFIG) {
        // todo
      } else if (app_ctrl_remote_req & APP_CTRL_REMOTE_REQ_STORE_CONFIG) {
        u8_t msg[] = {
            CMD_STORE_CONFIG
        };
        app_tx(msg, sizeof(msg));
        DBG(D_APP, D_INFO, "asking to save config\n");
      } else if (app_ctrl_remote_req & APP_CTRL_REMOTE_REQ_LOAD_CONFIG) {
        u8_t msg[] = {
            CMD_LOAD_CONFIG
        };
        app_tx(msg, sizeof(msg));
        DBG(D_APP, D_INFO, "asking to load config\n");
      } else if (app_ctrl_remote_req & APP_CTRL_REMOTE_REQ_GET_CONFIG) {
        u8_t msg[] = {
            CMD_GET_CONFIG
        };
        app_tx(msg, sizeof(msg));
        DBG(D_APP, D_INFO, "asking for config\n");
      }
    }
  }

}


///////////////////
// common funcs
///////////////////


static void app_tick_task(u32_t a, void *p) {
  common.tick_count++;
#ifdef CONFIG_SPYBOT_APP_CLIENT
  app_rover_tick();
#endif

#ifdef CONFIG_SPYBOT_APP_MASTER
  app_control_tick();
#endif
}


void APP_tx_dbg(const char *s) {
  u8_t msg[COMM_RADIO_LNK_MAX_DATA];
  memset(msg, 0, sizeof(msg));
  msg[0] = CMD_DBG;
  strncpy((char *)&msg[1], s, 24);
  app_tx(msg, 24);
}


void APP_comrad_rx(comm_arg *rx, u16_t len, u8_t *data, bool already_received) {
  if (common.pair_state == PAIRING_OK) {
    do {
#ifdef CONFIG_SPYBOT_APP_CLIENT
      if (data[0] == CMD_PAIRING_BEACON) {
        app_set_paired_state(FALSE);
        break; // handle new pairing beacon while being paired
      }
#endif
      if (data[0] == CMD_DBG) {
        print("RDBG: %s\n", &data[1]);
        COMRAD_reply(REPLY_OK, 1);
      } else {
#ifdef CONFIG_SPYBOT_APP_CLIENT
        app_rover_handle_rx(rx, len, data, already_received);
#endif
#ifdef CONFIG_SPYBOT_APP_SERVER
        app_control_handle_rx(rx, len, data, already_received);
#endif
      }
      return;
    } while (0);
  }

  // pairing

  DBG(D_APP, D_DEBUG, "app: pairing message %02x\n", data[0]);

#ifdef CONFIG_SPYBOT_APP_CLIENT
  if (common.pair_state == PAIRING_ROVER_AWAIT_BEACON) {
    if (data[0] == CMD_PAIRING_BEACON) {
      DBG(D_APP, D_DEBUG, "rover got pairing beacon\n");
      common.pair_state = PAIRING_ROVER_SEND_ECHO;
      COMRAD_reply(REPLY_OK, 1);
      TASK_set_timer_recurrence(&common.tick_timer, BEACON_RECURRENCE);
      u32_t ctrl_version =
          (data[1]<<24) | (data[2]<<16) | (data[3]<<8) | (data[4]);
      u32_t ctrl_build =
          (data[5]<<24) | (data[6]<<16) | (data[7]<<8) | (data[8]);
      COMRAD_reply(REPLY_OK, 1);
      DBG(D_APP, D_INFO, "ctrl version %08x, build %i\n", ctrl_version, ctrl_build);

    } else {
      // rover awaits pairing beacon cmd only
      DBG(D_APP, D_DEBUG, "rover awaits only pairing beacon\n");
      COMRAD_reply(REPLY_DENY, 1);
    }
  } else {
    // rover awaits no message in this state, but we allow beacon anyway
    if (data[0] != CMD_PAIRING_BEACON) {
      DBG(D_APP, D_DEBUG, "rover awaits no message\n");
      COMRAD_reply(REPLY_DENY, 1);
    } else {
      COMRAD_reply(REPLY_OK, 1);
    }
  }
#endif //CONFIG_SPYBOT_APP_CLIENT


#ifdef CONFIG_SPYBOT_APP_MASTER
  if (common.pair_state == PAIRING_CTRL_AWAIT_ECHO) {
    // controller awaits pairing echo cmd only
    if (data[0] == CMD_PAIRING_ECHO) {
      DBG(D_APP, D_DEBUG, "ctrl got beacon echo, pairing ok\n");
      app_set_paired_state(TRUE);
      TASK_set_timer_recurrence(&common.tick_timer, COMM_RECURRENCE);
      u32_t rover_version =
          (data[1]<<24) | (data[2]<<16) | (data[3]<<8) | (data[4]);
      u32_t rover_build =
          (data[5]<<24) | (data[6]<<16) | (data[7]<<8) | (data[8]);
      COMRAD_reply(REPLY_OK, 1);
      DBG(D_APP, D_INFO, "rover version %08x, build %i\n", rover_version, rover_build);
    } else {
      DBG(D_APP, D_DEBUG, "ctrl awaits beacon echo only\n");
      COMRAD_reply(REPLY_DENY, 1);
    }
  } else {
    // controller awaits no message in this state
    DBG(D_APP, D_DEBUG, "ctrl awaits no message\n");
    COMRAD_reply(REPLY_DENY, 1);
  }
#endif // CONFIG_SPYBOT_APP_MASTER
}

void APP_comrad_ack(comm_arg *rx, u16_t seq_no, u16_t len, u8_t *data) {
  if (seq_no == common.tx_seqno) {
    common.comrad_busy = FALSE;
  } else {
    DBG(D_APP, D_WARN, "got ack on unsent pkt, seq_no:%04x\n", seq_no);
    return;
  }

  // bad ack or denied ack
  if (len == 0 || data[0] != ACK_OK) {
    DBG(D_APP, D_DEBUG, "app got denial on ack\n");
    app_set_paired_state(FALSE);
    TASK_set_timer_recurrence(&common.tick_timer, BEACON_RECURRENCE);
    return;
  }

#ifdef CONFIG_SPYBOT_APP_CLIENT
  if (common.pair_state == PAIRING_ROVER_SEND_ECHO) {
    DBG(D_APP, D_DEBUG, "rover got ack on beacon echo, pairing ok\n");
    app_set_paired_state(TRUE);
    TASK_set_timer_recurrence(&common.tick_timer, COMM_RECURRENCE);
  } else if (common.pair_state == PAIRING_OK) {
    app_rover_handle_ack(common.tx_cmd, rx, len, data);
  }
#endif
#ifdef CONFIG_SPYBOT_APP_MASTER
  if (common.pair_state == PAIRING_CTRL_SEND_BEACON) {
    DBG(D_APP, D_DEBUG, "ctrl got ack on pairing beacon, awaiting echo\n");
    common.pair_state = PAIRING_CTRL_AWAIT_ECHO;
    common.pair_wait_cnt = 0;
  } else if (common.pair_state == PAIRING_OK) {
    app_control_handle_ack(common.tx_cmd, rx, len, data);
  }
#endif
}

void APP_comrad_err(u16_t seq_no, int err) {
  DBG(D_APP, D_WARN, "comrad err: seq:%04x (%04x) %i\n", seq_no, common.tx_seqno, err);
  if (seq_no == common.tx_seqno) {
    common.comrad_busy = FALSE;
    app_set_paired_state(FALSE);
  }
}

u8_t APP_pair_status(void) {
  return common.pair_state;
}


void APP_init(void) {
  SYS_dbg_mask_enable(D_APP);
  // common
  memset(&common, 0, sizeof(common));
  memset(&remote, 0, sizeof(remote));
  COMRAD_init();

  #ifdef CONFIG_SPYBOT_LSM
  int i;
  for (i = 0; i < 3; i++) {
    acc_extremes[i][0] = S16_MAX;
    acc_extremes[i][1] = S16_MIN;
    mag_extremes[i][0] = S16_MAX;
    mag_extremes[i][1] = S16_MIN;
  }
#endif

#ifdef CONFIG_SPYBOT_TEST
  app_rover_init();
  app_control_init();
#endif
#ifdef CONFIG_SPYBOT_CONTROLLER
  app_control_init();
#endif
#ifdef CONFIG_SPYBOT_ROVER
  app_rover_init();
#endif

  common.tick_task = TASK_create(app_tick_task, TASK_STATIC);
  TASK_start_timer(common.tick_task, &common.tick_timer, 0, 0, 50, 1000, "apptick");
  //print("priogroup to stop at critical context:%i\n",
  //  NVIC_EncodePriority(8 - __NVIC_PRIO_BITS, 1, 0));
}

u8_t APP_remote_get_heading(void) {
  return remote.lsm_heading;
}
s8_t *APP_remote_get_acc(void) {
  return &remote.lsm_acc[0];
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

void APP_remote_set_motor_ctrl(s8_t horizontal, s8_t vertical) {
  remote.motor_ctrl[0] = horizontal;
  remote.motor_ctrl[1] = vertical;
}

void APP_remote_load_config(void) {
  app_ctrl_remote_req |=
      APP_CTRL_REMOTE_REQ_LOAD_CONFIG  |
      APP_CTRL_REMOTE_REQ_GET_CONFIG;
}

const configuration_t *APP_cfg_get(void) {
  return &app_cfg;
}

void APP_cfg_set(spybot_cfg c, s16_t val) {
  switch (c) {
  case CFG_STEER_ADJUST:
    app_cfg.main.steer_adjust = val;
  break;
  case CFG_RADAR_ADJUST:
    app_cfg.main.radar_adjust = val;
  break;
  case CFG_CAM_PAN_ADJUST:
    app_cfg.main.cam_pan_adjust = val;
  break;
  case CFG_CAM_TILT_ADJUST:
    app_cfg.main.cam_tilt_adjust = val;
  break;
  case CFG_COMMON:
    app_cfg.main.common = val;
  break;
  case CFG_RADIO_CHANNEL:
    app_cfg.radio.radio_channel = val;
  break;
  case CFG_LSM_MAG_X_MIN:
    app_cfg.magneto.mag_x_min = val;
  break;
  case CFG_LSM_MAG_X_MAX:
    app_cfg.magneto.mag_x_max = val;
  break;
  case CFG_LSM_MAG_Y_MIN:
    app_cfg.magneto.mag_y_min = val;
  break;
  case CFG_LSM_MAG_Y_MAX:
    app_cfg.magneto.mag_y_max = val;
  break;
  case CFG_LSM_MAG_Z_MIN:
    app_cfg.magneto.mag_z_min = val;
  break;
  case CFG_LSM_MAG_Z_MAX:
    app_cfg.magneto.mag_z_max = val;
  break;
  case CFG_LSM_ACC_X_MIN:
    app_cfg.accel.acc_x_min = val;
  break;
  case CFG_LSM_ACC_X_MAX:
    app_cfg.accel.acc_x_max = val;
  break;
  case CFG_LSM_ACC_Y_MIN:
    app_cfg.accel.acc_y_min = val;
  break;
  case CFG_LSM_ACC_Y_MAX:
    app_cfg.accel.acc_y_max = val;
  break;
  case CFG_LSM_ACC_Z_MIN:
    app_cfg.accel.acc_z_min = val;
  break;
  case CFG_LSM_ACC_Z_MAX:
    app_cfg.accel.acc_z_max = val;
  break;
  default:
    ASSERT(FALSE);
    break;
  }
}
