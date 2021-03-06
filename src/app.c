/*
 * app.c
 *
 * Takes care of radio pairing and dispatches events further down
 * to specific instance (rover or controller)
 *
 *  Created on: Jan 2, 2014
 *      Author: petera
 */

#include "app.h"
#include "taskq.h"
#include "comm_radio.h"
#include "miniutils.h"
#include "led.h"


#ifdef CONFIG_SPYBOT_CONTROLLER
#include "app_controller.h"
#endif
#ifdef CONFIG_SPYBOT_ROVER
#include "app_rover.h"
#endif
#ifdef CONFIG_ADC
#include "adc.h"
#endif
#ifdef CONFIG_I2C
#include "stmpe811_impl.h"
#endif
#include "gpio.h"

static app_common common;
static app_remote remote;
static configuration_t app_cfg;

static u8_t batt_meas_state;
static u16_t batt_meas_sum;
static time batt_meas_start;
static time last_batt_meas = 0;
static bool batt_measured = FALSE;
static u32_t last_batt_time;
static u16_t last_batt_raw_time;

u8_t const REPLY_OK[] = {ACK_OK};
u8_t const REPLY_DENY[] = {ACK_DENY};
u8_t const REPLY_DBG[] = {ACK_DBG};

#ifdef CONFIG_I2C
task_mutex i2c_mutex = TASK_MUTEX_INIT;
#endif


// helpers

int APP_tx(u8_t *data, u16_t len) {
  if (!COMRAD_is_initialized()) return R_COMM_PHY_TRY_LATER;
  if (common.comrad_busy) return R_COMM_PHY_TRY_LATER;
  int res = COMRAD_send(data, len, TRUE);
  if (res >= R_COMM_OK) {
    common.tx_seqno = res;
    common.tx_cmd = data[0];
    common.comrad_busy = TRUE;
  }
  return res;
}

void APP_set_paired_state(bool paired) {
  common.err_count = 0;
  if (!paired) {
    common.pair_state = PAIRING_STAGE_ONE;
  } else {
    common.pair_state = PAIRING_OK;
  }
  common.impl->app_impl_set_paired(paired);
  COMRAD_report_paired(paired);
}

static void app_tick_task(u32_t a, void *p) {
  if (common.pair_state == PAIRING_STAGE_ONE) {
    LED_set(LED_MAIN, (common.tick_count & 3) == 0);
  } else if (common.pair_state == PAIRING_STAGE_TWO) {
    LED_set(LED_MAIN, (common.tick_count & 1) == 0);
  } else {
    LED_disable(LED_MAIN);
  }
  common.tick_count++;
#ifdef CONFIG_SPYBOT_APP_MASTER
  if (common.pair_state == PAIRING_STAGE_ONE) {
    TASK_set_timer_recurrence(&common.tick_timer, BEACON_RECURRENCE);

    u32_t v = SPYBOT_VERSION;
    u32_t b = SYS_build_number();

    u8_t beacon[] = { CMD_PAIRING_BEACON,
        (v >> 24) & 0xff, (v >> 16) & 0xff, (v >> 8) & 0xff, (v & 0xff),
        (b >> 24), (b >> 16), (b >> 8), (b & 0xff),
    };
    int res = APP_tx(beacon, sizeof(beacon));
    if (res >= R_COMM_OK) {
      DBG(D_APP, D_DEBUG, "ctrl sent beacon: seq %04x\n", res);
    } else {
      DBG(D_APP, D_WARN, "ctrl failed sending beacon %i\n", res);
    }
  } else if (common.pair_state == PAIRING_STAGE_TWO) {
    common.pair_wait_cnt++;
    if (common.pair_wait_cnt > 3) {
      DBG(D_APP, D_DEBUG, "UNPAIR ctrl echo wait timed out\n");
      APP_set_paired_state(FALSE);
    }
  }
#endif

#ifdef CONFIG_SPYBOT_APP_CLIENT
  if (common.pair_state == PAIRING_STAGE_TWO) {
    u32_t v = SPYBOT_VERSION;
    u32_t b = SYS_build_number();
    u8_t beacon_echo[] =
    { CMD_PAIRING_ECHO ,
        (v >> 24) & 0xff, (v >> 16) & 0xff, (v >> 8) & 0xff, (v & 0xff),
        (b >> 24), (b >> 16), (b >> 8), (b & 0xff),
    };
    APP_tx(beacon_echo, sizeof(beacon_echo));
  }
#endif

  if (common.pair_state == PAIRING_OK) {
    common.impl->app_impl_tick();
  }
}


void APP_tx_dbg(const char *s) {
  u8_t msg[COMM_RADIO_LNK_MAX_DATA];
  memset(msg, 0, sizeof(msg));
  msg[0] = CMD_DBG;
  strncpy((char *)&msg[1], s, 24);
  APP_tx(msg, 24);
}


void APP_comrad_rx(comm_arg *rx, u16_t len, u8_t *data, bool already_received) {
  if (common.pair_state == PAIRING_OK) {
    do {
#ifdef CONFIG_SPYBOT_APP_CLIENT
      if (data[0] == CMD_PAIRING_BEACON) {
        APP_set_paired_state(FALSE);
        break; // handle new pairing beacon while being paired
      }
#endif
      if (data[0] == CMD_DBG) {
        print("RDBG: %s\n", &data[1]);
        COMRAD_reply(REPLY_OK, 1);
      } else {
        common.impl->app_impl_handle_rx(rx, len, data, already_received);
      }
      return;
    } while (0);
  }

  // pairing

  DBG(D_APP, D_DEBUG, "app: pairing message %02x\n", data[0]);

#ifdef CONFIG_SPYBOT_APP_CLIENT
  if (common.pair_state == PAIRING_STAGE_ONE) {
    if (data[0] == CMD_PAIRING_BEACON) {
      DBG(D_APP, D_DEBUG, "rover got pairing beacon\n");
      common.pair_state = PAIRING_STAGE_TWO;
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
  if (common.pair_state == PAIRING_STAGE_TWO) {
    // controller awaits pairing echo cmd only
    if (data[0] == CMD_PAIRING_ECHO) {
      DBG(D_APP, D_DEBUG, "ctrl got beacon echo, pairing ok\n");
      APP_set_paired_state(TRUE);
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

  common.err_count = 0;

  // ack check
  if (len > 0 && data[0] == ACK_DBG) {
    DBG(D_APP, D_DEBUG, "Got dbg ack on seq %03x\n", seq_no);
    return;
  } else  if (len == 0 || data[0] == ACK_DENY) {
    DBG(D_APP, D_DEBUG, "UNPAIR app got denial on ack\n");
    APP_set_paired_state(FALSE);
    TASK_set_timer_recurrence(&common.tick_timer, BEACON_RECURRENCE);
    return;
  } else if (data[0] != ACK_OK) {
    DBG(D_APP, D_DEBUG, "Unknown ack on seq %03x\n", seq_no);
    // bad ack, ignore
    return;
  }

  // pairing
#ifdef CONFIG_SPYBOT_APP_CLIENT
  if (common.pair_state == PAIRING_STAGE_TWO) {
    DBG(D_APP, D_DEBUG, "rover got ack on beacon echo, pairing ok\n");
    APP_set_paired_state(TRUE);
    TASK_set_timer_recurrence(&common.tick_timer, COMM_RECURRENCE);
    return;
  }
#endif
#ifdef CONFIG_SPYBOT_APP_MASTER
  if (common.pair_state == PAIRING_STAGE_ONE) {
    DBG(D_APP, D_DEBUG, "ctrl got ack on pairing beacon, awaiting echo\n");
    common.pair_state = PAIRING_STAGE_TWO;
    common.pair_wait_cnt = 0;
    return;
  }
#endif

  // dispatch
  if (common.pair_state == PAIRING_OK) {
    common.impl->app_impl_handle_ack(common.tx_cmd, rx, len, data);
  }
}

void APP_comrad_err(u16_t seq_no, int err) {
  DBG(D_APP, D_WARN, "comrad err: seq:%04x (%04x) %i\n", seq_no, common.tx_seqno, err);
  if (seq_no == common.tx_seqno) {
    common.comrad_busy = FALSE;
    common.err_count++; // count erroneous txed messages
    if (APP_pair_status() == PAIRING_OK && common.err_count > COMM_MAX_TX_ERR) {
      // reached max limit of failed tx
      DBG(D_APP, D_WARN, "UNPAIR due to tx err\n");
      APP_set_paired_state(FALSE);
    }
  }
}

void APP_handle_unknown_msg(comm_arg *rx, u16_t len, u8_t *data, bool already_received) {
  IF_DBG(D_APP, D_INFO) {
    print("unknown message received 0x%02x\n", data[0]);
    print("  s:%i d:%i seq:%03x len:%i flags:%08b\n", rx->src, rx->dst, rx->seqno, len, rx->flags);
    printbuf(IOSTD, data, len);
  }
  if (!already_received) common.err_count++; // count errorenous rxed messages
  if (APP_pair_status() == PAIRING_OK && common.err_count > COMM_MAX_TX_ERR) {
    DBG(D_APP, D_WARN, "UNPAIR due to unknown message 0x%02x\n", data[0]);
    COMRAD_reply(REPLY_DENY, 1);
    APP_set_paired_state(FALSE);
  } else {
    COMRAD_reply(REPLY_DBG, 1);
  }
}

u8_t APP_pair_status(void) {
  return common.pair_state;
}


// battery measuring

static void stmpe_batt_meas_adc_cb(u16_t adc_val) {
  if (batt_meas_state >= 3) {
    batt_meas_state++;
    batt_meas_sum += adc_val;
    if (batt_meas_state > 6) {
      batt_meas_sum /= 4;
      u32_t v_1000 = batt_meas_sum * 7400 / 0xab0;
      last_batt_time = v_1000;
      last_batt_raw_time = batt_meas_sum;
      DBG(D_APP, D_INFO, "BATT: %i.%03i V\n", v_1000 / 1000, v_1000 % 1000);
      batt_meas_state = 0;
      gpio_disable(BAT_LOAD_PORT, BAT_LOAD_PIN);
      STMPE_req_gpio_set(0, STMPE_GPIO_VBAT_EN);
#ifdef CONFIG_SPYBOT_ROVER
      remote.batt = (batt_meas_sum >> 4) & 0xff;
#endif
    } else {
      STMPE_req_read_adc(STMPE_ADC_VBAT, stmpe_batt_meas_adc_cb);
    }
  }
}

void APP_measure_batt_poll(void) {
  if (batt_meas_state == 0) {
    time now = SYS_get_time_ms();
    if ((!batt_measured && now > 750) || (now - last_batt_meas) >= 45000) {
      batt_measured = TRUE;
      last_batt_meas = now;
      batt_meas_state = 1;
      batt_meas_sum = 0;
    } else {
      return;
    }
  }

  if (batt_meas_state == 1) batt_meas_start = SYS_get_time_ms();
  if (SYS_get_time_ms() - batt_meas_start > 100) {
    // this is taking too long, abort
    batt_meas_state = 0;
    gpio_disable(BAT_LOAD_PORT, BAT_LOAD_PIN);
    STMPE_req_gpio_set(0, STMPE_GPIO_VBAT_EN);
    DBG(D_APP, D_WARN, "battery measuring aborted due to timeout\n");
    return;
  }

  switch (batt_meas_state) {
  case 1:
    gpio_enable(BAT_LOAD_PORT, BAT_LOAD_PIN);
    STMPE_req_gpio_set(STMPE_GPIO_VBAT_EN, 0);
    batt_meas_state = 2;
    break;

  case 2:
    if (SYS_get_time_ms() - batt_meas_start < 5) {
      // allow 5 ms to load capacitors
      return;
    }
    batt_meas_state = 3;
    STMPE_req_read_adc(STMPE_ADC_VBAT, stmpe_batt_meas_adc_cb);
    break;
    break;
  default:
    // handled in stmpe adc callback
    break;
  }
}

// common

void APP_init(void) {
  //SYS_dbg_level(D_WARN);
  //SYS_dbg_mask_enable(D_ANY); // todo remove
  // common
  memset(&common, 0, sizeof(common));
  memset(&remote, 0, sizeof(remote));
  batt_meas_state = 0;
  batt_meas_sum = 0;
  batt_meas_start = 0;
  last_batt_time = 0;
  LED_init();

#ifdef CONFIG_SPYBOT_CONTROLLER
  APP_control_init();
#endif

#ifdef CONFIG_SPYBOT_ROVER
  APP_rover_init();
#endif

  if (!APP_get_safe_mode()) {
    common.impl->app_impl_setup(&common, &remote, &app_cfg);

    common.tick_task = TASK_create(app_tick_task, TASK_STATIC);
    TASK_start_timer(common.tick_task, &common.tick_timer, 0, 0, 50, 1000, "apptick");
    //print("priogroup to stop at critical context:%i\n",
    //  NVIC_EncodePriority(8 - __NVIC_PRIO_BITS, 1, 0));
  } else {
    print("\n*** SAFE MODE ***\n");
  }

#ifdef CONFIG_ADC
  u16_t vref;
  (void)ADC_sample_vref_sync(&vref);
  // vref corresponds to 1.32-1.41-1.50V
  u32_t voltage_m_1000 =
      (0xfff
      * 1410) // 1.41 * 1000
      / vref;
  print("\nchip voltage: %i.%03iV\n", (voltage_m_1000 / 1000), (voltage_m_1000 % 1000));
#endif
}

void APP_impl_set(app_impl *impl) {
  common.impl = impl;
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

void APP_remote_set_camera_ctrl(s8_t horizontal, s8_t vertical) {
  remote.pan = horizontal;
  remote.tilt = vertical;
}

void APP_remote_set_radar_ctrl(s8_t radar) {
  remote.radar = radar;
}

const app_remote *APP_remote_get(void) {
  return &remote;
}


const configuration_t *APP_cfg_get(void) {
  return &app_cfg;
}

s16_t APP_cfg_get_val(spybot_cfg c) {
  switch (c) {
  case CFG_STEER_ADJUST:
    return (s16_t) app_cfg.main.steer_adjust ;
  break;
  case CFG_RADAR_ADJUST:
    return (s16_t) app_cfg.main.radar_adjust ;
  break;
  case CFG_CAM_PAN_ADJUST:
    return (s16_t) app_cfg.main.cam_pan_adjust ;
  break;
  case CFG_CAM_TILT_ADJUST:
    return (s16_t) app_cfg.main.cam_tilt_adjust ;
  break;
  case CFG_CONTROL:
    return (s16_t) app_cfg.main.control ;
  break;
  case CFG_RADIO_CHANNEL:
    return (s16_t) app_cfg.radio.radio_channel ;
  break;
  case CFG_RADIO_PA:
    return (s16_t) app_cfg.radio.pa_scheme;
  break;
  case CFG_LSM_MAG_X_MIN:
    return (s16_t) app_cfg.magneto.mag_x_min ;
  break;
  case CFG_LSM_MAG_X_MAX:
    return (s16_t) app_cfg.magneto.mag_x_max ;
  break;
  case CFG_LSM_MAG_Y_MIN:
    return (s16_t) app_cfg.magneto.mag_y_min ;
  break;
  case CFG_LSM_MAG_Y_MAX:
    return (s16_t) app_cfg.magneto.mag_y_max ;
  break;
  case CFG_LSM_MAG_Z_MIN:
    return (s16_t) app_cfg.magneto.mag_z_min ;
  break;
  case CFG_LSM_MAG_Z_MAX:
    return (s16_t) app_cfg.magneto.mag_z_max ;
  break;
  case CFG_LSM_ACC_X_MIN:
    return (s16_t) app_cfg.accel.acc_x_min ;
  break;
  case CFG_LSM_ACC_X_MAX:
    return (s16_t) app_cfg.accel.acc_x_max ;
  break;
  case CFG_LSM_ACC_Y_MIN:
    return (s16_t) app_cfg.accel.acc_y_min ;
  break;
  case CFG_LSM_ACC_Y_MAX:
    return (s16_t) app_cfg.accel.acc_y_max ;
  break;
  case CFG_LSM_ACC_Z_MIN:
    return (s16_t) app_cfg.accel.acc_z_min ;
  break;
  case CFG_LSM_ACC_Z_MAX:
    return (s16_t) app_cfg.accel.acc_z_max ;
  break;
  default:
    ASSERT(FALSE);
    break;
  }
  return (s16_t)-1;
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
  case CFG_CONTROL:
    app_cfg.main.control = val;
  break;
  case CFG_RADIO_CHANNEL:
    app_cfg.radio.radio_channel = val;
  break;
  case CFG_RADIO_PA:
    app_cfg.radio.pa_scheme = val;
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

u32_t APP_get_last_batt(void) {
  return last_batt_time;
}

void APP_force_batt_reading(void) {
  batt_measured = FALSE;
}

DBG_ATTRIBUTE static bool __safe_mode;

void APP_set_safe_mode(bool enable) {
  __safe_mode = enable;
}

bool APP_get_safe_mode(void) {
  return __safe_mode;
}
