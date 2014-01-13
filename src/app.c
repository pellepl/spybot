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

#ifdef CONFIG_SPYBOT_LSM
#include "lsm303_driver.h"
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

#define PAIRING_CTRL_SEND_BEACON    0
#define PAIRING_CTRL_AWAIT_ECHO     1
#define PAIRING_ROVER_AWAIT_BEACON  0
#define PAIRING_ROVER_SEND_ECHO     1
#define PAIRING_OK                  2

#define ACK_OK                      0x01
#define ACK_DENY                    0x00

#define BEACON_RECURRENCE           400
#define COMM_RECURRENCE             107

static struct {
  u16_t tx_seqno;
  u8_t tx_cmd;
  bool comrad_busy;
  u8_t pair_state;
  u8_t pair_wait_cnt;
} common;

static u8_t const REPLY_OK[] = {ACK_OK};
static u8_t const REPLY_DENY[] = {ACK_DENY};


static task_timer comm_timer;
static task *comm_task = NULL;

// control ram
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
#ifdef CONFIG_SPYBOT_LSM
task_mutex i2c_mutex = TASK_MUTEX_INIT;
lsm303_dev lsm_dev;
static task_timer lsm_timer;
static task *lsm_task = NULL;
static bool reading_lsm = FALSE;
#endif

#ifdef CONFIG_SPYBOT_MOTOR
static task_timer motor_timer;
static task *motor_task = NULL;
#endif

// ram updated over RF
static struct {
  s8_t lsm_heading;
  s8_t lsm_acc[3];
  s8_t motor_ctrl[2]; // steer vector : [0] hori, [1] vert
} remote;


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
  }
  COMRAD_report_paired(paired);
}

// rover funcs

#ifdef CONFIG_SPYBOT_LSM
static void app_rover_lsm_cb_task(u32_t ares, void *adev) {
  lsm303_dev *dev = (lsm303_dev *)dev;
  int res = (int)ares;
  if (res == I2C_ERR_LSM303_BAD_READ) {
    print("lsm bad read\n");
  } else  if (res != I2C_OK) {
    //print("lsm error %i\n", res);
    I2C_config(_I2C_BUS(0), 100000);
  }
  TASK_mutex_unlock(&i2c_mutex);
  reading_lsm = FALSE;

  u16_t heading_raw = lsm_get_heading(&lsm_dev);
  //print("heading:%04x\n", heading_raw);
  s16_t *acc = lsm_get_acc_reading(&lsm_dev);
  remote.lsm_heading = heading_raw >> 8;
  remote.lsm_acc[0] = acc[0] >> 4;
  remote.lsm_acc[1] = acc[1] >> 4;
  remote.lsm_acc[2] = acc[2] >> 4;
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

#ifdef CONFIG_SPYBOT_MOTOR
static void app_rover_motor_task(u32_t a, void *b) {
  MOTOR_update();
}

#endif

static void app_rover_init(void) {
#ifdef CONFIG_SPYBOT_LSM
  I2C_config(_I2C_BUS(0), 100000);

  if (!TASK_mutex_lock(&i2c_mutex)) {
    ASSERT(FALSE); // should never happen, during init
  }

  lsm_open(&lsm_dev, _I2C_BUS(0), FALSE, app_rover_lsm_cb_irq);
  lsm_config_default(&lsm_dev);
  lsm_set_lowpass(&lsm_dev, 50, 50);

  lsm_task = TASK_create(app_rover_lsm_task, TASK_STATIC);
  TASK_start_timer(lsm_task, &lsm_timer, 0, 0, 0, 100, "lsm_read");
#endif // CONFIG_SPYBOT_LSM
#ifdef CONFIG_SPYBOT_MOTOR
  MOTOR_init();
  motor_task = TASK_create(app_rover_motor_task, TASK_STATIC);
  TASK_start_timer(motor_task, &motor_timer, 0, 0, 0, 20, "motor");
#endif
}

#define REPLY_MAX_LEN (COMM_LNK_MAX_DATA-COMM_H_SIZE-1)

void app_rover_handle_rx(comm_arg *rx, u16_t len, u8_t *data, bool already_received) {
  u8_t reply[REPLY_MAX_LEN];
  u8_t reply_ix = 0;
  switch (data[0]) {
  case CMD_CONTROL:
    reply[reply_ix++] = ACK_OK;
    reply[reply_ix++] = data[4];
    remote.motor_ctrl[0] = (s8_t)data[1];
    remote.motor_ctrl[1] = (s8_t)data[2];

    if (already_received) {

    }
    // action_mask
    if (data[3]) {

    }
    // status_mask
    if (data[4] & SPYBOT_SR_ACC) {
      //DBG(D_APP, D_DEBUG, "ctrl wants acc data\n");
      reply[reply_ix++] = remote.lsm_acc[0];
      reply[reply_ix++] = remote.lsm_acc[1];
      reply[reply_ix++] = remote.lsm_acc[2];
    }
    if (data[4] & SPYBOT_SR_HEADING) {
      //DBG(D_APP, D_DEBUG, "ctrl wants heading data\n");
      reply[reply_ix++] = remote.lsm_heading;
    }
    if (data[4] & SPYBOT_SR_TEMP) {
      DBG(D_APP, D_DEBUG, "ctrl wants temp data\n");
    }
    if (data[4] & SPYBOT_SR_BATT) {
      DBG(D_APP, D_DEBUG, "ctrl wants batt data\n");
    }
    if (data[4] & SPYBOT_SR_RADAR) {
      DBG(D_APP, D_DEBUG, "ctrl wants radar data\n");
    }
    COMRAD_reply(reply, reply_ix);

#ifdef CONFIG_SPYBOT_MOTOR
    MOTOR_control_vector(remote.motor_ctrl[0], remote.motor_ctrl[1]);
#endif

    break;
  case CMD_SET_CONFIG:
    break;
  case CMD_STORE_CONFIG:
    break;
  case CMD_LOAD_CONFIG:
    break;
  case CMD_LSM_MAG_EXTREMES:
    break;
  case CMD_LSM_ACC_EXTREMES:
    break;
  default:
    DBG(D_APP, D_WARN, "rover: unknown message %02x\n", data[0]);
    COMRAD_reply(REPLY_DENY, 1);
    app_set_paired_state(FALSE);
    break;
  }
}

void app_rover_handle_ack(u8_t cmd, comm_arg *rx, u16_t len, u8_t *data) {

}

// control funcs

#ifdef CONFIG_SPYBOT_JOYSTICK
static void app_control_adc_cb(u16_t ch1, u16_t ch2) {
  joystick_v = ch1;
  joystick_h = ch2;
}
#endif

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
  INPUT_init();

#ifdef CONFIG_SPYBOT_VIDEO
  HUD_init(&gctx);
  HUD_state(HUD_MAIN);
  ui_task = TASK_create(app_control_ui_task, TASK_STATIC);
  TASK_start_timer(ui_task, &ui_timer, 0, 0, 0, 50, "ui_upd");
#endif
}

void app_control_handle_rx(comm_arg *rx, u16_t len, u8_t *data, bool already_received) {

}

void app_control_handle_ack(u8_t cmd, comm_arg *rx, u16_t len, u8_t *data) {
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
  }
}

// common funcs

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

static void app_comm_task(u32_t a, void *p) {
#ifdef CONFIG_SPYBOT_APP_CLIENT
  // rover
  if (common.pair_state == PAIRING_ROVER_SEND_ECHO) {
    u8_t beacon_echo[] = { CMD_PAIRING_ECHO };
    app_tx(beacon_echo, sizeof(beacon_echo));
  } else if (common.pair_state == PAIRING_OK) {
    // todo
  }
#endif

#ifdef CONFIG_SPYBOT_APP_MASTER
  // control
  if (common.pair_state == PAIRING_CTRL_SEND_BEACON) {
    TASK_set_timer_recurrence(&comm_timer, BEACON_RECURRENCE);
    u8_t beacon[] = { CMD_PAIRING_BEACON };
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
    u8_t sr = SPYBOT_SR_ACC | SPYBOT_SR_HEADING;
    s8_t left_motor = remote.motor_ctrl[0];
    s8_t right_motor = remote.motor_ctrl[1];
    u8_t ctrl[] = {
        CMD_CONTROL, left_motor, right_motor, 0, sr
    };
    app_tx(ctrl, sizeof(ctrl));
  }
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
      TASK_set_timer_recurrence(&comm_timer, BEACON_RECURRENCE);
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
#endif


#ifdef CONFIG_SPYBOT_APP_MASTER
  if (common.pair_state == PAIRING_CTRL_AWAIT_ECHO) {
    // controller awaits pairing echo cmd only
    if (data[0] == CMD_PAIRING_ECHO) {
      DBG(D_APP, D_DEBUG, "ctrl got beacon echo, pairing ok\n");
      app_set_paired_state(TRUE);
      TASK_set_timer_recurrence(&comm_timer, COMM_RECURRENCE);
      COMRAD_reply(REPLY_OK, 1);
    } else {
      DBG(D_APP, D_DEBUG, "ctrl awaits beacon echo only\n");
      COMRAD_reply(REPLY_DENY, 1);
    }
  } else {
    // controller awaits no message in this state
    DBG(D_APP, D_DEBUG, "ctrl awaits no message\n");
    COMRAD_reply(REPLY_DENY, 1);
  }
#endif
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
    TASK_set_timer_recurrence(&comm_timer, BEACON_RECURRENCE);
    return;
  }

#ifdef CONFIG_SPYBOT_APP_CLIENT
  if (common.pair_state == PAIRING_ROVER_SEND_ECHO) {
    DBG(D_APP, D_DEBUG, "rover got ack on beacon echo, pairing ok\n");
    app_set_paired_state(TRUE);
    TASK_set_timer_recurrence(&comm_timer, COMM_RECURRENCE);
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
  print("APP comrad err: seq:%04x (%04x) %i\n", seq_no, common.tx_seqno, err);
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
  comm_task = TASK_create(app_comm_task, TASK_STATIC);
  TASK_start_timer(comm_task, &comm_timer, 0, 0, 1000, 1000, "comm");
  //print("priogroup to stop at critical context:%i\n",
  //  NVIC_EncodePriority(8 - __NVIC_PRIO_BITS, 1, 0));
}

u8_t APP_remote_get_heading(void) {
  return remote.lsm_heading;
}
s8_t *APP_remote_get_acc(void) {
  return &remote.lsm_acc[0];
}

void APP_remote_set_motor_ctrl(s8_t horizontal, s8_t vertical) {
  remote.motor_ctrl[0] = horizontal;
  remote.motor_ctrl[1] = vertical;
}
