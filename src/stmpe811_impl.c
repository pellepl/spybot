/*
 * stmpe811_impl.c
 *
 *  Created on: May 21, 2014
 *      Author: petera
 */

#include "stmpe811_impl.h"
#include "app.h"
#include "taskq.h"
#include "gpio.h"

#define STMPE_REQ_GPIO    (1<<0)
#define STMPE_REQ_ADC     (1<<1)
#define STMPE_REQ_TEMP    (1<<2)

static struct {
  stmpe811_handler handler;
  volatile u32_t req_mask;
  volatile bool irq_task_en;
  task *task_irq;
  u8_t gpio_set;
  u8_t gpio_reset;
  u8_t adc_chan;
  u16_t val_adc;
  u16_t val_temp;
} stmpe;

static volatile bool init = FALSE;
static bool pre_state;

static void stmpe_task_config(u32_t a, void *b) {
  if (!TASK_mutex_lock(&i2c_mutex)) {
    print("stmpe config mutex locked\n");
    return;
  }
  print("stmpe config mutex acquired\n");
  int res = stmpe811_handler_setup(&stmpe.handler,
      STMPE_BLOCK_TEMP | STMPE_BLOCK_GPIO | STMPE_BLOCK_ADC,
      STMPE_GPIO_VBAT_EN |
      STMPE_GPIO2 | STMPE_GPIO3 | STMPE_GPIO4 |  STMPE_GPIO5 | STMPE_GPIO6 | STMPE_GPIO7, // analog / digital
      STMPE_GPIO_VBAT_EN, // direction
      0, // default output
      TRUE, // interrupt
      STMPE_INT_POLARITY_ACTIVE_LOW_FALLING, STMPE_INT_TYPE_LEVEL, // interrupt config
      STMPE_INT_GPIO | STMPE_INT_ADC | STMPE_INT_TEMP_SENS, // interrupt enable
      STMPE_GPIO2 | STMPE_GPIO3 | STMPE_GPIO4 |  STMPE_GPIO5 | STMPE_GPIO6 | STMPE_GPIO7, // gpio interrupt mask
      STMPE_GPIO_ADC_VBAT, // adc interrupt mask
      STMPE_GPIO2 | STMPE_GPIO3 | STMPE_GPIO4 | STMPE_GPIO5 | STMPE_GPIO6 | STMPE_GPIO7, // detect rising
      STMPE_GPIO2 | STMPE_GPIO3 | STMPE_GPIO4 | STMPE_GPIO5 | STMPE_GPIO6 | STMPE_GPIO7, // detect falling
      STMPE_ADC_CLK_3_25MHZ,
      STMPE_ADC_TIM_80,
      STMPE_ADC_RES_12B,
      STMPE_ADC_REF_INT,
      TRUE, // temp enable
      STMPE_TEMP_MODE_ONCE,
      FALSE,
      STMPE_TEMP_THRES_OVER,
      0);
  if (res != I2C_OK) {
    print("stmpe config ERR mutex unlocked\n");
    TASK_mutex_unlock(&i2c_mutex);
  }
}

static void stmpe_task_gpio(u32_t a, void *b) {
  if (!TASK_mutex_lock(&i2c_mutex)) {
    print("stmpe gpio mutex locked\n");
    return;
  }
  print("stmpe gpio mutex acquired\n");
  int res = stmpe811_handler_gpio_define(&stmpe.handler, stmpe.gpio_set, stmpe.gpio_reset);
  if (res != I2C_OK) {
    print("stmpe gpio ERR mutex unlocked\n");
    TASK_mutex_unlock(&i2c_mutex);
  }
}

static void stmpe_task_adc(u32_t a, void *b) {
  if (!TASK_mutex_lock(&i2c_mutex)) {
    print("stmpe adc mutex locked\n");
    return;
  }
  print("stmpe adc mutex acquired\n");
  int res = stmpe811_handler_adc_read(&stmpe.handler, stmpe.adc_chan);
  if (res != I2C_OK) {
    print("stmpe adc ERR mutex unlocked\n");
    TASK_mutex_unlock(&i2c_mutex);
  }
}

static void stmpe_task_temp(u32_t a, void *b) {
  if (!TASK_mutex_lock(&i2c_mutex)) {
    print("stmpe temp mutex locked\n");
    return;
  }
  print("stmpe temp mutex acquired\n");
  int res = stmpe811_handler_temp_read(&stmpe.handler, TRUE);
  if (res != I2C_OK) {
    print("stmpe temp ERR mutex unlocked\n");
  TASK_mutex_unlock(&i2c_mutex);
  }
}

static void stmpe_exe_req(void) {
  u32_t req_mask = stmpe.req_mask;
  if (req_mask) {
    if (req_mask & STMPE_REQ_GPIO) {
      print("stmpe req EXE gpio\n");
      task *t = TASK_create(stmpe_task_gpio, 0);
      ASSERT(t);
      TASK_run(t, 0, NULL);
      return;
    }
    if (req_mask & STMPE_REQ_ADC) {
      print("stmpe req EXE adc\n");
      task *t = TASK_create(stmpe_task_adc, 0);
      ASSERT(t);
      TASK_run(t, 0, NULL);
      return;
    }
    if (req_mask & STMPE_REQ_TEMP) {
      print("stmpe req EXE temp\n");
      task *t = TASK_create(stmpe_task_temp, 0);
      ASSERT(t);
      TASK_run(t, 0, NULL);
      return;
    }
  }
}

static void stmpe_task_irq(u32_t a, void *b) {
  if (!TASK_mutex_lock(&i2c_mutex)) {
    print("stmpe irq mutex locked\n");
    return;
  }
  print("stmpe irq mutex acquired\n");
  stmpe811_handler_interrupt_cb(&stmpe.handler);
  stmpe.irq_task_en = FALSE;
}

static void stmpe_irq(gpio_pin pin) {
  if (!stmpe.irq_task_en) {
    stmpe.irq_task_en = TRUE;
    TASK_run(stmpe.task_irq, 0, NULL);
  }
}

static void stmpe_gpio_cb(u8_t mask, u8_t gpio) {
  print("stmpe gpio irq mask:%08b gpio:%08b\n", mask, gpio);
}

static void stmpe_adc_cb(u8_t adc, u16_t val) {
  print("stmpe adc irq adc:%08b val:%04x\n", adc, val);
  stmpe.val_adc = val;
}

static void stmpe_temp_cb(u16_t val) {
  // assume 3.3V (Vio * val) / 7.51
  print("stmpe temp irq temp:%04x (%i celsius)\n", val, ((s32_t)val * 330) / 751);
  stmpe.val_temp = val;
}

static void stmpe_err_cb(stmpe811_handler_state state, int err) {
  if (err) DBG(D_APP, D_WARN, "stmpe res state:%i res:%i\n", state, err);
  print("stmpe res state:%i res:%i\n", state, err);
  enter_critical();
  print("stmpe finished mutex unlocked\n");
  TASK_mutex_unlock(&i2c_mutex);
  if (err) {
    stmpe.req_mask = 0;
  }
  if (state == STMPE811_HDL_STATE_GPIO_DEFINE_CLR || state == STMPE811_HDL_STATE_GPIO_DEFINE_SET) {
    print("stmpe req CLR gpio\n");
    stmpe.req_mask &= ~STMPE_REQ_GPIO;
    stmpe.gpio_set = 0;
    stmpe.gpio_reset = 0;
  }
  else if (state == STMPE811_HDL_STATE_ADC_READ) {
    print("stmpe req CLR adc\n");
    stmpe.req_mask &= ~STMPE_REQ_ADC;
    stmpe.adc_chan = 0;
  }
  else if (state == STMPE811_HDL_STATE_TEMP_READ || state == STMPE811_HDL_STATE_TEMP_RESULT) {
    print("stmpe req CLR temp\n");
    stmpe.req_mask &= ~STMPE_REQ_TEMP;
  }
  stmpe_exe_req();
  exit_critical();
}

void STMPE_timer(void) {
  if (!init) return;
  bool state = gpio_get(PORTC, PIN13) != 0;
  //if (state != pre_state) print("\npc13:%i\n", state);
  if (pre_state && !state) {
    stmpe_irq(PIN13);
  }
  pre_state = state;
}

void STMPE_init(void) {
  memset(&stmpe, 0x00, sizeof(stmpe));
  stmpe811_handler_open(&stmpe.handler, _I2C_BUS(0), stmpe_gpio_cb, stmpe_adc_cb, stmpe_temp_cb, stmpe_err_cb);
  gpio_config(PORTC, PIN13, CLK_50MHZ, IN, AF0, OPENDRAIN, NOPULL);
  // cannot do this, will interfere with high prio hsync signal on controller
  //gpio_interrupt_config(PORTC, PIN13, stmpe_irq, FLANK_DOWN);
  //gpio_interrupt_mask_enable(PORTC, PIN13, TRUE);
  // instead, poll in timer
  pre_state = gpio_get(PORTC, PIN13) != 0;
  stmpe.task_irq = TASK_create(stmpe_task_irq, TASK_STATIC);
  ASSERT(stmpe.task_irq);

  init = TRUE;
  task *t = TASK_create(stmpe_task_config, 0);
  ASSERT(t);
  TASK_run(t, 0, NULL);
}

void STMPE_req_gpio_set(u8_t set, u8_t reset) {
  enter_critical();
  bool do_req = stmpe.req_mask == 0;
  stmpe.gpio_set |= set;
  stmpe.gpio_reset |= reset;
  stmpe.req_mask |= STMPE_REQ_GPIO;
  if (do_req) {
    stmpe_exe_req();
  }
  exit_critical();
}

void STMPE_req_read_adc(u8_t adc) {
  enter_critical();
  bool do_req = stmpe.req_mask == 0;
  stmpe.adc_chan = adc;
  stmpe.req_mask |= STMPE_REQ_ADC;
  if (do_req) {
    stmpe_exe_req();
  }
  exit_critical();
}

void STMPE_req_read_temp(void) {
  enter_critical();
  bool do_req = stmpe.req_mask == 0;
  stmpe.req_mask |= STMPE_REQ_TEMP;
  if (do_req) {
    stmpe_exe_req();
  }
  exit_critical();
}

u16_t STMPE_adc_value(void) {
  return stmpe.val_adc;
}

u16_t STMPE_temp_value(void) {
  return stmpe.val_temp;
}

