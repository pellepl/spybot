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

#ifdef CONFIG_I2C
#include "stmpe811_impl.h"
#endif

// for diagnosis
#include "gpio.h"
#include "led.h"
#include "nrf905_impl.h"

#define PAIRING_CTRL_SEND_BEACON    PAIRING_STAGE_ONE
#define PAIRING_CTRL_AWAIT_ECHO     PAIRING_STAGE_TWO

// current common app state of controller
static app_common *common;
// data constantly updated from rover
static app_remote *remote;
// controller's view of rover's current configuration
static configuration_t *app_cfg;

// bitfield indicating which configurations are altered on controller's side
// and should be sent to rover for update
static u32_t app_ctrl_cfg_dirty = 0;
// bitfield indicating which dirty configurations have been sent to rover
// (i.e. which dirty flags can be cleared upon message update-config ack)
static u32_t app_ctrl_cfg_dirty_sent = 0;
// bitfield indicating which pending remote requests there are from
// controller to rover (being any of APP_CTRL_REMOTE_REQ_*)
static u32_t app_ctrl_remote_req = 0;

#ifdef CONFIG_SPYBOT_VIDEO
gcontext gctx;
static task_timer ui_timer;
static task *ui_task = NULL;
static bool diagnostics = FALSE;
#endif
#ifdef CONFIG_SPYBOT_JOYSTICK
static u16_t joystick_v = 0x800;
static u16_t joystick_h = 0x800;
#endif
// whether joystick is controlling motors (TRUE) or camera (FALSE)
static bool app_joy_ctrl;

static task_timer diag_timer;
static char _diag_msg[32];

static struct {
  s8_t vals[CONFIG_RADAR_ANGLES];
} remote_radar;


static void app_control_start(void);

static void dprint(const char *f, ...) {
  va_list arg_p;
  memset(_diag_msg, 0, sizeof(_diag_msg));

  va_start(arg_p, f);
  vsprint(_diag_msg, f, arg_p);
  va_end(arg_p);
  HUD_dbg_print(&gctx, _diag_msg);
  print("--DIAG-- %s", _diag_msg);
}

static void dprint_sl(const char *f, ...) {
  va_list arg_p;
  memset(_diag_msg, 0, sizeof(_diag_msg));

  va_start(arg_p, f);
  vsprint(_diag_msg, f, arg_p);
  va_end(arg_p);
  HUD_dbg_print_same_line(&gctx, _diag_msg);
}

static void app_control_clr_remote_req(u32_t req) {
  app_ctrl_remote_req &= ~req;
}

static void app_control_set_remote_req(u32_t req) {
  app_ctrl_remote_req |= req;
}


void app_control_set_paired(bool paired) {
  if (paired) {
    app_control_set_remote_req(APP_CTRL_REMOTE_REQ_GET_CONFIG);
    if (!diagnostics) {
      CVIDEO_set_input(INPUT_CAMERA);
    } else {
      dprint("!CONTROLLER PAIRED\n");
    }
  }
}

#ifdef CONFIG_SPYBOT_JOYSTICK
static void app_control_adc_cb(u16_t ch1, u16_t ch2) {
  joystick_v = ch1;
  joystick_h = ch2;
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
  APP_remote_set_motor_ctrl(0, 0); // reset until someone says otherwise
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

// diagnostics ui

typedef enum {
  DIAG_SYS = 0,
  DIAG_CPU_VOLTAGE,
  DIAG_BATT_VOLTAGE1,
  DIAG_BATT_VOLTAGE2,
  DIAG_BATT_VOLTAGE3,
  DIAG_RADIO_ENABLE,
  DIAG_TX_CARRIER,
  DIAG_RX,
  DIAG_RADIO_CLOSE,
  DIAG_LEDS,
  DIAG_COMRAD_START,
  DIAG_COMRAD_SETTINGS,
  DIAG_JOYSTICK1,
  DIAG_JOYSTICK2,
  DIAG_AUDIO,
  DIAG_COMRAD_TX,
  DIAG_REMOTE_SETTINGS,
} diag_phase;

static void app_control_diag_task(u32_t phase, void *b) {
  static u32_t joy_read = 0;
  static u32_t last_joy_read = 0;
  static u16_t old_h=0, old_v=0;
  static u16_t loop;
  static u8_t audio_buf;
  static u8_t audio_hi, audio_lo;

  switch (phase) {

  case DIAG_SYS: {
    HUD_state(HUD_DBG);
    dprint(" DIAGNOSTICS\n");
    dprint(" -----------\n");
    dprint(" BUILD  %8i\n", SYS_build_number());
    dprint(" CPU    %8i Hz\n", SYS_CPU_FREQ);
    dprint(" TIMER  %8i Hz\n", SYS_MAIN_TIMER_FREQ);

    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    dprint(" HCLK   %8i Hz\n", clocks.HCLK_Frequency);
    dprint(" PCLK1  %8i Hz\n", clocks.PCLK1_Frequency);
    dprint(" PCLK2  %8i Hz\n", clocks.PCLK2_Frequency);
    dprint(" SYSCLK %8i Hz\n", clocks.SYSCLK_Frequency);
    dprint(" ADCCLK %8i Hz\n", clocks.ADCCLK_Frequency);
    dprint(" \n");

    task *t = TASK_create(app_control_diag_task, 0);
    ASSERT(t);
    TASK_start_timer(t, &diag_timer, phase+1, 0, 1500, 0, "diag");

    break;
  }

  case DIAG_CPU_VOLTAGE: {
    dprint(" CHECK CPU VOLTAGE..\n");
    u16_t vref;
    (void)ADC_sample_vref_sync(&vref);
    // vref corresponds to 1.32-1.41-1.50V
    u32_t voltage_m_1000 =
        (0xfff
        * 1410) // 1.41 * 1000
        / vref;
    dprint("   CPU VOLTAGE %i.%03iV\n", (voltage_m_1000 / 1000), (voltage_m_1000 % 1000));
    if (voltage_m_1000 < 3300) {
      dprint("!WARNING: VOLTAGE LOW\n");
    }
    dprint(" ENABLE LOAD\n");
    gpio_enable(BAT_LOAD_PORT, BAT_LOAD_PIN);
    SYS_hardsleep_ms(300);
    dprint(" CHECK CPU VOLTAGE..\n");
    (void)ADC_sample_vref_sync(&vref);
    // vref corresponds to 1.32-1.41-1.50V
    gpio_disable(BAT_LOAD_PORT, BAT_LOAD_PIN);
    voltage_m_1000 =
        (0xfff
        * 1410) // 1.41 * 1000
        / vref;
    dprint("   CPU VOLTAGE %i.%03iV\n", (voltage_m_1000 / 1000), (voltage_m_1000 % 1000));
    if (voltage_m_1000 < 3300) {
      dprint("!WARNING: VOLTAGE LOW\n");
    }
    dprint(" DISABLE LOAD\n");

    task *t = TASK_create(app_control_diag_task, 0);
    ASSERT(t);
    TASK_start_timer(t, &diag_timer, phase+1, 0, 1000, 0, "diag");
    break;
  }

  case DIAG_BATT_VOLTAGE1: {
    dprint(" CHECK BATT VOLTAGE..\n");
    dprint(" ENABLE LOAD\n");
    gpio_enable(BAT_LOAD_PORT, BAT_LOAD_PIN);
    STMPE_req_gpio_set(STMPE_GPIO_VBAT_EN, 0);
    task *t = TASK_create(app_control_diag_task, 0);
    ASSERT(t);
    TASK_start_timer(t, &diag_timer, phase+1, 0, 200, 0, "diag");
    break;
  }
  case DIAG_BATT_VOLTAGE2: {
    STMPE_req_read_adc(STMPE_ADC_VBAT);
    task *t = TASK_create(app_control_diag_task, 0);
    ASSERT(t);
    TASK_start_timer(t, &diag_timer, phase+1, 0, 300, 0, "diag");
    break;
  }
  case DIAG_BATT_VOLTAGE3: {
    u32_t val = STMPE_adc_value();
    u32_t v_1000 = val * 7300 / 0xab0;
    dprint("   BATT VOLTAGE %i.%03i V\n", v_1000 / 1000, v_1000 % 1000);
    if (v_1000 == 0) {
      dprint("!WARNING: STMPE811 BROKEN\n");
    } else if (v_1000 < 5250) {
      dprint("!WARNING: BATTERY LOW\n");
    }
    dprint(" DISABLE LOAD\n");
    gpio_disable(BAT_LOAD_PORT, BAT_LOAD_PIN);
    STMPE_req_gpio_set(0, STMPE_GPIO_VBAT_EN);

    task *t = TASK_create(app_control_diag_task, 0);
    ASSERT(t);
    TASK_start_timer(t, &diag_timer, phase+1, 0, 500, 0, "diag");
    break;
  }

  case DIAG_RADIO_ENABLE: {
    dprint(" RADIO ENABLE\n");
    NRF905_IMPL_init(NULL, NULL, NULL, NULL);
    nrf905_config conf;
    conf.auto_retransmit = NRF905_CFG_AUTO_RETRAN_OFF;
    conf.channel_freq = 0;
    conf.crc_en = NRF905_CFG_CRC_ON;
    conf.crc_mode = NRF905_CFG_CRC_MODE_16BIT;
    conf.crystal_frequency = NRF905_CFG_XTAL_FREQ_16MHZ;
    conf.hfreq_pll = NRF905_CFG_HFREQ_433;
    conf.out_clk_enable = NRF905_CFG_OUT_CLK_OFF;
    conf.out_clk_freq = NRF905_CFG_OUT_CLK_FREQ_1MHZ;
    conf.pa_pwr= NRF905_CFG_PA_PWR_10;
  #ifdef SECONDARY
    conf.rx_address = 0x9ce3aed2;
  #else
    conf.rx_address = 0x631c512d;
  #endif
    conf.rx_address_field_width = NRF905_CFG_ADDRESS_FIELD_WIDTH_4;
    conf.rx_payload_width = COMM_LNK_MAX_DATA;
    conf.rx_reduced_power = NRF905_CFG_RX_REDUCED_POWER_OFF;
    conf.tx_address_field_width = NRF905_CFG_ADDRESS_FIELD_WIDTH_4;
    conf.tx_payload_width = COMM_LNK_MAX_DATA;

    int res = NRF905_IMPL_conf(&conf, TRUE);
    if (res != NRF905_OK) {
      dprint("!ERR: RADIO CFG %i\n", res);
    } else {
      dprint(" RADIO CONFIG OK\n", res);
    }

    task *t = TASK_create(app_control_diag_task, 0);
    ASSERT(t);
    TASK_start_timer(t, &diag_timer, phase+1, 0, 250, 0, "diag");
    loop = 0;
    break;
  }

  case DIAG_TX_CARRIER: {
    if (loop == 0) dprint(" RADIO TX CARRIER\n");
    u32_t time_to_next = 3000;
    u16_t channel_freq = (loop/2)*128;

    if ((loop & 1) == 0) {
      int res = NRF905_IMPL_conf_channel(channel_freq, TRUE);
      if (res != NRF905_OK) {
        dprint("!ERR: RADIO SET CH %i\n", res);
      } else {
        u32_t ch100 = (42240 + channel_freq*10)*(1 + NRF905_CFG_HFREQ_433);
        dprint(" FREQ %i.%i MHz", ch100/100, ch100%100);
      }
      time_to_next = 100;
    } else {
      int res = NRF905_IMPL_carrier();
      if (res != NRF905_OK) {
        dprint(" \n");
        dprint("!ERR: RADIO SET CARRIER %i\n", res);
      } else {
        dprint(" CARRIER\n");
      }
    }

    loop++;
    task *t = TASK_create(app_control_diag_task, 0);
    ASSERT(t);
    if (loop < 8) {
      TASK_start_timer(t, &diag_timer, phase, 0, time_to_next, 0, "diag");
    } else {
      loop = 0;
      TASK_start_timer(t, &diag_timer, phase+1, 0, time_to_next, 0, "diag");
    }
    break;
  }

  case DIAG_RX: {
    if (loop == 0) {
      dprint(" RADIO RX\n");
      int res = NRF905_IMPL_rx();
      if (res != NRF905_OK) {
        dprint("!ERR: RADIO SET RX %i\n", res);
      }
    } else {
      dprint_sl("   CARRIER DETECTS:%i", NRF905_IMPL_carrier_count());
    }
    loop++;

    task *t = TASK_create(app_control_diag_task, 0);
    ASSERT(t);
    if (loop < 60) {
      TASK_start_timer(t, &diag_timer, phase, 0, 100, 0, "diag");
    } else {
      dprint("   CARRIER DETECTS:%i\n", NRF905_IMPL_carrier_count());
      loop = 0;
      TASK_start_timer(t, &diag_timer, phase+1, 0, 100, 0, "diag");
    }
    break;
  }

  case DIAG_RADIO_CLOSE: {
    dprint(" RADIO CLOSE..\n");
    NRF905_IMPL_close();
    task *t = TASK_create(app_control_diag_task, 0);
    ASSERT(t);
    TASK_start_timer(t, &diag_timer, phase+1, 0, 300, 0, "diag");
    break;
  }

  case DIAG_LEDS: {
    if (loop == 0) {
      dprint(" BLINK MAIN LED..\n");
    }
    loop++;
    LED_set(LED_MAIN, (SYS_get_time_ms() % 200 < 100));
    task *t = TASK_create(app_control_diag_task, 0);
    ASSERT(t);
    if (loop < 50) {
      TASK_start_timer(t, &diag_timer, phase, 0, 50, 0, "diag");
    } else {
      TASK_start_timer(t, &diag_timer, phase+1, 0, 50, 0, "diag");
    }
    break;
  }

  case DIAG_COMRAD_START: {
    dprint(" COMM STACK STARTUP..\n");
    COMRAD_init();
    task *t = TASK_create(app_control_diag_task, 0);
    ASSERT(t);
    TASK_start_timer(t, &diag_timer, phase+1, 0, 1000, 0, "diag");
    break;
  }

  case DIAG_COMRAD_SETTINGS: {
    NRF905_IMPL_read_conf();
    dprint(" COMM RADIO CONFIG\n");
    dprint("   CRYSTAL %i MHz\n",
        (int[5]){4,8,12,16,20}[NRF905_IMPL_get_config()->crystal_frequency]);
    dprint("   PLL     %s MHz\n",
        NRF905_IMPL_get_config()->hfreq_pll == NRF905_CFG_HFREQ_433 ? "433":"868,915");
    dprint("   CHANNEL %i\n", NRF905_IMPL_get_config()->channel_freq);

    u32_t ch100 = (42240 + NRF905_IMPL_get_config()->channel_freq*10)*(1 + NRF905_IMPL_get_config()->hfreq_pll);
    dprint("   CH FREQ %i.%i MHz\n", ch100/100, ch100%100);
    dprint("   PA PWR  %+i dBm\n",
        (int[4]){-10,-2,6,10}[NRF905_IMPL_get_config()->pa_pwr]);
    dprint("   CRC     %s\n",
        NRF905_IMPL_get_config()->crc_en == NRF905_CFG_CRC_OFF ? "NONE" :
            (NRF905_IMPL_get_config()->crc_mode ? "16bit" : "8bit")
            );
    task *t = TASK_create(app_control_diag_task, 0);
    ASSERT(t);
    TASK_start_timer(t, &diag_timer, phase+1, 0, 1500, 0, "diag");
    break;
  }

  case DIAG_JOYSTICK1: {
    dprint(" READ RAW JOYSTICK..\n");

    task *t = TASK_create(app_control_diag_task, 0);
    ASSERT(t);
    TASK_start_timer(t, &diag_timer, phase+1, 0, 200, 0, "diag");
    break;
  }
  case DIAG_JOYSTICK2: {
    task *t = TASK_create(app_control_diag_task, 0);
    ASSERT(t);
    if (joy_read < 500 &&
        ((ABS(old_h-joystick_h)>>3) != 0 || (ABS(old_v - joystick_v)>>3) != 0))  {
      last_joy_read = joy_read;
      old_h = joystick_h;
      old_v = joystick_v;
      dprint_sl("   h:%04x v:%04x", joystick_h, joystick_v);
      TASK_start_timer(t, &diag_timer, phase, 0, 100, 0, "diag");
    } else {
      TASK_start_timer(t, &diag_timer,
          ((joy_read-last_joy_read < 20) && joy_read < 500) ? phase : (phase+1), 0, 100, 0, "diag");
      loop = 0;
    }
    joy_read++;
    break;
  }

  case DIAG_AUDIO: {
    if (loop == 0) {
      dprint(" READ RAW AUDIO..\n");
      audio_hi = 0;
      audio_lo = 0xff;
    }
    loop++;
    ADC_sample_sound(NULL, &audio_buf, 1);
    char level[17];
    memset(level, '=', sizeof(level));
    level[audio_buf>>1] = 0;
    dprint_sl("  %3i %s", audio_buf, level);
    if (loop > 1) {
      audio_hi = MAX(audio_hi, audio_buf);
      audio_lo = MIN(audio_lo, audio_buf);
    }

    task *t = TASK_create(app_control_diag_task, 0);
    ASSERT(t);
    if (loop < 500) {
      TASK_start_timer(t, &diag_timer, phase, 0, 20, 0, "diag");
    } else {
      TASK_start_timer(t, &diag_timer, phase+1, 0, 100, 0, "diag");
      dprint("   lo:%2i  hi:%2i\n", audio_lo, audio_hi);
      loop = 0;
    }
    break;
  }

  case DIAG_COMRAD_TX: {
    if (loop == 0) dprint(" COMRAD TX DBG\n");
    loop++;
    task *t = TASK_create(app_control_diag_task, 0);
    ASSERT(t);
    if (loop <= 5) {
      char msg[20];
      memset(msg, 0, sizeof(msg));
      sprint(msg, "DIAGNOSIS TX #%i", loop);
      APP_tx_dbg(msg);
      dprint_sl("   SENDING MSG #%i", loop);
      TASK_start_timer(t, &diag_timer, phase, 0, 300, 0, "diag");
    } else {
      TASK_start_timer(t, &diag_timer, phase+1, 0, 100, 0, "diag");
    }
    break;

  }

  case DIAG_REMOTE_SETTINGS: {
    if (APP_pair_status() == PAIRING_OK) {
      dprint(" REMOTE CONFIG\n");
      dprint("   PAN    %i\n", APP_cfg_get()->main.cam_pan_adjust);
      dprint("   TILT   %i\n", APP_cfg_get()->main.cam_tilt_adjust);
      dprint("   STEER  %i\n", APP_cfg_get()->main.steer_adjust);
      dprint("   RADAR  %i\n", APP_cfg_get()->main.radar_adjust);
      dprint("   COMMON %08b\n", APP_cfg_get()->main.common);
    } else {
      dprint(" NOT PAIRED, NO CONFIG\n");
    }
    task *t = TASK_create(app_control_diag_task, 0);
    ASSERT(t);
    TASK_start_timer(t, &diag_timer, phase+1, 0, 1000, 0, "diag");
    loop = 0;
    break;
  }

  default: {
    if (!gpio_get(JOY_BUTT_PORT, JOY_BUTT_PIN))
    {
      if (APP_pair_status() == PAIRING_OK) {
        CVIDEO_set_input(INPUT_CAMERA);
      }
      app_control_start();
      diagnostics = FALSE;
    } else {
      if (loop == 0) dprint(" DIAGNOSIS FINISHED\n");

      task *t = TASK_create(app_control_diag_task, 0);
      ASSERT(t);
      loop++;
      TASK_start_timer(t, &diag_timer, -1, 0, 333, 0, "diag");
      u32_t ti = loop % 3;
      if (ti == 0)
        dprint_sl(" LEAVE DIAG BY PRESS..");
      else if (ti == 2)
        dprint_sl(" LEAVE DIAG BY PRESS.");
      else
        dprint_sl(" LEAVE DIAG BY PRESS");


    }
    break;
  }
  }
}

static void app_control_start_diagnostics(void) {
  diagnostics = TRUE;
  CVIDEO_set_input(INPUT_NONE);
  SYS_hardsleep_ms(500);
  CVIDEO_set_input(INPUT_GENERATED);
  task *t = TASK_create(app_control_diag_task, 0);
  ASSERT(t);
  TASK_run(t, 0, NULL);
}

// control common

static void app_control_start(void) {
  if (!diagnostics) COMRAD_init();
  HUD_state(HUD_MAIN);
}

static void app_control_handle_rx(comm_arg *rx, u16_t len, u8_t *data, bool already_received) {
  u8_t reply[REPLY_MAX_LEN];
  u8_t reply_ix = 0;
  reply[reply_ix++] = ACK_OK;

  switch (data[0]) {
  case CMD_RADAR_REPORT: {
    // todo handle different, diff buffer
    u8_t a = data[1];
    u8_t len = data[2];
    u8_t i = 0;
    while (len--) {
      remote_radar.vals[a++] = data[3 + i];
      i++;
    }
    reply[reply_ix++] = 1; // ok
    COMRAD_reply(reply, reply_ix);
    break;
  }
  case CMD_ALERT:
    // todo
    break;
  case CMD_SET_RADIO_CONFIG:
    print("radio ch:%02x\n", data[1]);
    print("radio pa:%02x\n", data[2]);
    reply[reply_ix++] = 1; // ok
    COMRAD_reply(reply, reply_ix);
    break;
  default:
    APP_handle_unknown_msg(rx, len, data, already_received);
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
      app_ctrl_cfg_dirty &= ~app_ctrl_cfg_dirty_sent;
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

// send over updated configuration to rover
static void app_control_dispatch_config(void) {
  app_ctrl_cfg_dirty_sent = 0;
  u8_t msg[REPLY_MAX_LEN];
  u8_t msg_ix = 0;
  u8_t cfg_bit_ix = 0;
  msg[msg_ix++] = CMD_SET_CONFIG;
  while (cfg_bit_ix < 32 && msg_ix < REPLY_MAX_LEN-2) {
    if (app_ctrl_cfg_dirty & (1<<cfg_bit_ix)) {
      s16_t val = APP_cfg_get_val(cfg_bit_ix);
      msg[msg_ix++] = cfg_bit_ix;
      msg[msg_ix++] = (val >> 8) & 0xff;
      msg[msg_ix++] = (val & 0xff);
      app_ctrl_cfg_dirty_sent |= (1<<cfg_bit_ix);
    }
    cfg_bit_ix++;
  }
  msg[msg_ix++] = CFG_STOP;
  APP_tx(msg, msg_ix);
}

static bool await_radar_report = FALSE;

static void app_control_tick(void) {
  u8_t msg_divider = (app_ctrl_remote_req & APP_REMOTE_REQ_URGENT) ? 1 : 7;
  // primitive TDM

  if (await_radar_report) {
    // leave room for rxing radar report
    await_radar_report = FALSE;
    return;
  }

  if ((app_ctrl_remote_req == 0 || (common->tick_count & msg_divider) != 0)) {
    // plain request
    s8_t left_motor = remote->motor_ctrl[0];
    s8_t right_motor = remote->motor_ctrl[1];
    s8_t pan = remote->pan;
    s8_t tilt  = remote->tilt;
    s8_t radar  = remote->radar;
    u8_t act = 0;
    u8_t sr = SPYBOT_SR_ACC | SPYBOT_SR_HEADING;
    sr |= (common->tick_count % 6) ? 0 : SPYBOT_SR_RADAR;
    u8_t msg[] = {
        CMD_CONTROL, left_motor, right_motor, pan, tilt, radar, act, sr
    };
    await_radar_report = sr & SPYBOT_SR_RADAR;
    APP_tx(msg, sizeof(msg));
  } else {
    // order is of importance

    if (app_ctrl_remote_req & APP_CTRL_REMOTE_REQ_SET_CONFIG) {
      // sends updated configurations to rover
      app_control_dispatch_config();
      DBG(D_APP, D_INFO, "update config, mask 0b%032b\n", app_ctrl_cfg_dirty_sent);
    } else if (app_ctrl_remote_req & APP_CTRL_REMOTE_REQ_STORE_CONFIG) {
      // request to store volatile static config on rover side
      u8_t msg[] = {
          CMD_STORE_CONFIG
      };
      APP_tx(msg, sizeof(msg));
      DBG(D_APP, D_INFO, "asking to save config\n");
    } else if (app_ctrl_remote_req & APP_CTRL_REMOTE_REQ_LOAD_CONFIG) {
      // request to load latest stored configuration on rover side
      // and use that as volatile static config
      u8_t msg[] = {
          CMD_LOAD_CONFIG
      };
      APP_tx(msg, sizeof(msg));
      DBG(D_APP, D_INFO, "asking to load config\n");
    } else if (app_ctrl_remote_req & APP_CTRL_REMOTE_REQ_GET_CONFIG) {
      // request to get current volatile static config from rover
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
#ifdef CONFIG_SPYBOT_VIDEO
  CVIDEO_init(HUD_vbl);
  CVIDEO_init_gcontext(&gctx);
  CVIDEO_set_effect(79);
#endif

  // setup ui & input
#ifdef CONFIG_SPYBOT_CONTROLLER
  INPUT_init();
#endif

#ifdef CONFIG_I2C
  STMPE_init();
#endif

#ifdef CONFIG_SPYBOT_VIDEO
  CVIDEO_set_input(INPUT_GENERATED);
  HUD_init(&gctx);
  ui_task = TASK_create(app_control_ui_task, TASK_STATIC);
  TASK_start_timer(ui_task, &ui_timer, 0, 0, 0, 35, "ui_upd");

  if (!gpio_get(JOY_BUTT_PORT, JOY_BUTT_PIN)) {
    app_control_start_diagnostics();
  } else {
    app_control_start();
  }
#endif

  APP_remote_set_camera_ctrl(0, 0);
  APP_remote_set_radar_ctrl(RADAR_CTRL_NORMAL);

  memset(remote_radar.vals, 0xff, CONFIG_RADAR_ANGLES);

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
  // will first load stored config at rover and
  // then send config over to controller
  app_control_set_remote_req(
      APP_CTRL_REMOTE_REQ_LOAD_CONFIG  |
      APP_CTRL_REMOTE_REQ_GET_CONFIG);
}

void APP_remote_store_config(void) {
  // will first store current config at rover and
  // then send config over to controller
  app_control_set_remote_req(
      APP_CTRL_REMOTE_REQ_STORE_CONFIG |
      APP_CTRL_REMOTE_REQ_GET_CONFIG);
}

void APP_remote_update_config(spybot_cfg cfg, s16_t val, bool urgent) {
  // will eventually set same configuration value at rover
  APP_cfg_set(cfg, val);
  app_control_set_remote_req(APP_CTRL_REMOTE_REQ_SET_CONFIG);
  app_ctrl_cfg_dirty |= (1<<cfg);
  if (urgent) {
    app_control_set_remote_req(APP_REMOTE_REQ_URGENT);
  }
}

s8_t *APP_get_radar_values(void) {
  return remote_radar.vals;
}
