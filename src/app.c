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
#include "adc.h"

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
  bool comrad_busy;
  u8_t pair_state;
  u8_t pair_wait_cnt;
} common;

static u8_t const REPLY_OK[] = {ACK_OK};
static u8_t const REPLY_DENY[] = {ACK_DENY};


static task_timer comm_timer;
static task *comm_task = NULL;

// control ram
gcontext gctx;
static task_timer ui_timer;
static task *ui_task = NULL;
static u16_t joystick_v = 0x800;
static u16_t joystick_h = 0x800;

// rover ram
task_mutex i2c_mutex = TASK_MUTEX_INIT;
lsm303_dev lsm_dev;
static task_timer lsm_timer;
static task *lsm_task = NULL;
static bool reading_lsm = FALSE;

// rover funcs

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


static void app_rover_init(void) {
  I2C_config(_I2C_BUS(0), 100000);

  if (!TASK_mutex_lock(&i2c_mutex)) {
    ASSERT(FALSE); // should never happen, during init
  }

  lsm_open(&lsm_dev, _I2C_BUS(0), FALSE, app_rover_lsm_cb_irq);
  lsm_config_default(&lsm_dev);
  lsm_set_lowpass(&lsm_dev, 50, 50);

  lsm_task = TASK_create(app_rover_lsm_task, TASK_STATIC);
  TASK_start_timer(lsm_task, &lsm_timer, 0, 0, 0, 100, "lsm_read");
}


// control funcs

static void app_ctrl_adc_cb(u16_t ch1, u16_t ch2) {
  joystick_v = ch1;
  joystick_h = ch2;
}

static void app_control_ui_task(u32_t a, void *b) {
  ADC_sample(app_ctrl_adc_cb);
  HUD_paint();
  INPUT_read(joystick_v, joystick_h);
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

  INPUT_init();

  HUD_init(&gctx, &lsm_dev);
  HUD_state(HUD_MAIN);

  ui_task = TASK_create(app_control_ui_task, TASK_STATIC);
  TASK_start_timer(ui_task, &ui_timer, 0, 0, 0, 50, "ui_upd");
}

// common funcs

static void app_tx(u8_t *data, u16_t len) {
  if (common.comrad_busy) return;
  int res = COMRAD_send(data, len, TRUE);
  if (res >= R_COMM_OK) {
    common.tx_seqno = res;
    common.comrad_busy = TRUE;
  }
}



static void app_comm_task(u32_t a, void *p) {
#ifdef SECONDARY
  // control
  if (common.pair_state == PAIRING_CTRL_SEND_BEACON) {
    TASK_set_timer_recurrence(&comm_timer, BEACON_RECURRENCE);
    u8_t beacon[] = { CMD_PAIRING_BEACON };
    app_tx(beacon, sizeof(beacon));
  } else if (common.pair_state == PAIRING_CTRL_AWAIT_ECHO) {
    common.pair_wait_cnt++;
    if (common.pair_wait_cnt > 1) {
      DBG(D_APP, D_DEBUG, "ctrl echo wait timed out\n");
      common.pair_state = PAIRING_CTRL_SEND_BEACON;
    }
  }
  else if (common.pair_state == PAIRING_OK) {
    u8_t ctrl[] = {
        CMD_CONTROL, 0, 0, 0, 0xff
    };
    app_tx(ctrl, sizeof(ctrl));
  }

#else
  // rover
  if (common.pair_state == PAIRING_ROVER_SEND_ECHO) {
    u8_t beacon_echo[] = { CMD_PAIRING_ECHO };
    app_tx(beacon_echo, sizeof(beacon_echo));
  } else if (common.pair_state == PAIRING_OK) {
    // todo
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
  //DBG(D_APP, D_DEBUG, "app: message %02x\n", data[0]);
#ifndef SECONDARY
  if (data[0] == CMD_PAIRING_BEACON) {
    DBG(D_APP, D_DEBUG, "rover got pairing beacon\n");
    common.pair_state = PAIRING_ROVER_SEND_ECHO;
    COMRAD_reply(REPLY_OK, 1);
    TASK_set_timer_recurrence(&comm_timer, BEACON_RECURRENCE);
  } else
#endif
  if (common.pair_state == PAIRING_OK) {
    // TODO handle
    if (data[0] == CMD_DBG) {
      print("RDBG: %s\n", &data[1]);
    }
    COMRAD_reply(REPLY_OK, 1);
    return;
  }

#ifdef SECONDARY
  if (common.pair_state == PAIRING_CTRL_AWAIT_ECHO) {
    // controller awaits pairing echo cmd only
    if (data[0] == CMD_PAIRING_ECHO) {
      DBG(D_APP, D_DEBUG, "ctrl got beacon echo, pairing ok\n");
      common.pair_state = PAIRING_OK;
      COMRAD_report_paired(TRUE);
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
#else

  if (common.pair_state == PAIRING_ROVER_AWAIT_BEACON) {
    // rover awaits pairing beacon cmd only
    if (data[0] != CMD_PAIRING_BEACON) {
      DBG(D_APP, D_DEBUG, "rover awaits only pairing beacon\n");
      COMRAD_reply(REPLY_DENY, 1);
    }
  } else {
    // rover awaits no message in this state
    DBG(D_APP, D_DEBUG, "rover awaits no message\n");
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

  if (len == 0 || data[0] != ACK_OK) {
#ifdef SECONDARY
    DBG(D_APP, D_DEBUG, "ctrl got denial on ack\n");
    common.pair_state = PAIRING_CTRL_SEND_BEACON;
#else
    DBG(D_APP, D_DEBUG, "rover got denial on ack\n");
    common.pair_state = PAIRING_ROVER_AWAIT_BEACON;
#endif
    COMRAD_report_paired(FALSE);
    TASK_set_timer_recurrence(&comm_timer, BEACON_RECURRENCE);
  }

#ifdef SECONDARY
  if (common.pair_state == PAIRING_CTRL_SEND_BEACON) {
    DBG(D_APP, D_DEBUG, "ctrl got ack on pairing beacon, awaiting echo\n");
    common.pair_state = PAIRING_CTRL_AWAIT_ECHO;
    common.pair_wait_cnt = 0;
  }
#else
  if (common.pair_state == PAIRING_ROVER_SEND_ECHO) {
    DBG(D_APP, D_DEBUG, "rover got ack on beacon echo, pairing ok\n");
    common.pair_state = PAIRING_OK;
    COMRAD_report_paired(TRUE);
    TASK_set_timer_recurrence(&comm_timer, COMM_RECURRENCE);
  }
#endif
}

void APP_comrad_err(u16_t seq_no, int err) {
  if (seq_no == common.tx_seqno) {
    common.comrad_busy = FALSE;
  }
}

u8_t APP_pair_status(void) {
  return common.pair_state;
}


void APP_init(void) {
  SYS_dbg_mask_enable(D_APP);
  // common
  memset(&common, 0, sizeof(common));
  COMRAD_init();

  // TODO for test
  app_rover_init();
#ifndef SECONDARY
  app_control_init();
#endif
  comm_task = TASK_create(app_comm_task, TASK_STATIC);
  TASK_start_timer(comm_task, &comm_timer, 0, 0, 1000, 1000, "comm");
}
