/*
 * range_sens_hcsr04_driver.c
 *
 * Trigger HC-SR04 range sensor by a positive pulse on TRIG, 10us duration.
 * ECHO will give positive pulse corresponding to sensed distance:
 *   range = high level time * velocity (340M/S) / 2
 *
 * Time to wait for echo is 60ms max, after we assume that sensor points to
 * free space.
 *
 * ECHO is connected to 2 capture/compare (CC) channels on a timer. One CC
 * is used for sensing positive flank, and the other CC for sensing negative
 * flank. Timer wraps are also accounted for.
 *
 *  Created on: Nov 15, 2013
 *      Author: petera
 */

#include "range_sens_hcsr04_driver.h"
#include "taskq.h"
#include "gpio.h"

#if HCSR04_PULS_TIM == 1
#define _HCSR04_TIMER     TIM1
#elif HCSR04_PULS_TIM == 2
#define _HCSR04_TIMER     TIM2
#elif HCSR04_PULS_TIM == 3
#define _HCSR04_TIMER     TIM3
#elif HCSR04_PULS_TIM == 4
#define _HCSR04_TIMER     TIM4
#else
#error unknown timer
#endif

#if HCSR04_PULS_TIMCH_H == 1
#define _HCSR04_CHAN_H      TIM_Channel_1
#define _HCSR04_CC_CH_H     TIM_IT_CC1
#elif HCSR04_PULS_TIMCH_H == 2
#define _HCSR04_CHAN_H      TIM_Channel_2
#define _HCSR04_CC_CH_H     TIM_IT_CC2
#elif HCSR04_PULS_TIMCH_H == 3
#define _HCSR04_CHAN_H      TIM_Channel_3
#define _HCSR04_CC_CH_H     TIM_IT_CC3
#elif HCSR04_PULS_TIMCH_H == 4
#define _HCSR04_CHAN_H      TIM_Channel_4
#define _HCSR04_CC_CH_H     TIM_IT_CC4
#else
#error unknown channel
#endif

#if HCSR04_PULS_TIMCH_L == 1
#define _HCSR04_CHAN_L      TIM_Channel_1
#define _HCSR04_CC_CH_L     TIM_IT_CC1
#elif HCSR04_PULS_TIMCH_L == 2
#define _HCSR04_CHAN_L      TIM_Channel_2
#define _HCSR04_CC_CH_L     TIM_IT_CC2
#elif HCSR04_PULS_TIMCH_L == 3
#define _HCSR04_CHAN_L      TIM_Channel_3
#define _HCSR04_CC_CH_L     TIM_IT_CC3
#elif HCSR04_PULS_TIMCH_L == 4
#define _HCSR04_CHAN_L      TIM_Channel_4
#define _HCSR04_CC_CH_L     TIM_IT_CC4
#else
#error unknown channel
#endif


static struct _sta {
  range_sens_cb_fn cb_fn;
  volatile bool busy;
  task_timer timer;
  task *task;
  bool cc_high;
  u32_t wraps;
}  hcsr;

static void range_sens_disable(void) {
  TIM_Cmd(_HCSR04_TIMER, DISABLE);
  TIM_ITConfig(_HCSR04_TIMER, _HCSR04_CC_CH_L | _HCSR04_CC_CH_H | TIM_IT_Update, DISABLE);
  TIM_ClearITPendingBit(_HCSR04_TIMER, _HCSR04_CC_CH_L | _HCSR04_CC_CH_H | TIM_IT_Update);
  hcsr.cc_high = FALSE;
}

static void range_sens_enable(void) {
  hcsr.cc_high = FALSE;
  TIM_ClearITPendingBit(_HCSR04_TIMER, _HCSR04_CC_CH_L | _HCSR04_CC_CH_H | TIM_IT_Update);
  TIM_ITConfig(_HCSR04_TIMER, _HCSR04_CC_CH_L | _HCSR04_CC_CH_H | TIM_IT_Update, ENABLE);
  TIM_Cmd(_HCSR04_TIMER, ENABLE);
}

static void range_sens_trig_tmo(u32_t arg, void *argp) {
  enter_critical();
  range_sens_disable();
  exit_critical();

  ///print("hscr: tmo\n");
  hcsr.busy = FALSE;
  if (hcsr.cb_fn) {
    hcsr.cb_fn((u32_t)~0);
  }
}

static void range_sens_trig_echo(u32_t delta, void *argp) {
  ///print("hscr: %08x\n", delta);
  hcsr.cb_fn(delta);
}

static void range_sens_irq(void) {
  // got high flank
  if(TIM_GetITStatus(_HCSR04_TIMER, _HCSR04_CC_CH_H) == SET) {
    TIM_ClearITPendingBit(_HCSR04_TIMER, _HCSR04_CC_CH_H);
    hcsr.cc_high = TRUE;
    hcsr.wraps = 0;
  }
  // got timer counter wrap interrupt
  if(TIM_GetITStatus(_HCSR04_TIMER, TIM_IT_Update) == SET) {
    TIM_ClearITPendingBit(_HCSR04_TIMER, TIM_IT_Update);
    if (hcsr.cc_high) {
      hcsr.wraps++;
    }
  }
  // got low flank
  if(TIM_GetITStatus(_HCSR04_TIMER, _HCSR04_CC_CH_L) == SET) {
    TIM_ClearITPendingBit(_HCSR04_TIMER, _HCSR04_CC_CH_L);
    if (hcsr.cc_high) {
      u32_t cch =
#if HCSR04_PULS_TIMCH_H == 1
      TIM_GetCapture1(_HCSR04_TIMER);
#elif HCSR04_PULS_TIMCH_H == 2
      TIM_GetCapture2(_HCSR04_TIMER);
#elif HCSR04_PULS_TIMCH_H == 3
      TIM_GetCapture3(_HCSR04_TIMER);
#elif HCSR04_PULS_TIMCH_H == 4
      TIM_GetCapture4(_HCSR04_TIMER);
#endif
    u32_t ccl =
#if HCSR04_PULS_TIMCH_L == 1
      TIM_GetCapture1(_HCSR04_TIMER);
#elif HCSR04_PULS_TIMCH_L == 2
      TIM_GetCapture2(_HCSR04_TIMER);
#elif HCSR04_PULS_TIMCH_L == 3
      TIM_GetCapture3(_HCSR04_TIMER);
#elif HCSR04_PULS_TIMCH_L == 4
      TIM_GetCapture4(_HCSR04_TIMER);
#endif
      range_sens_disable();
      TASK_stop_timer(&hcsr.timer);
      u32_t delta = ccl-cch+hcsr.wraps*0x10000;
      if (hcsr.busy && hcsr.cb_fn) {
        hcsr.busy = FALSE;
        // call back in task context
        task *t = TASK_create(range_sens_trig_echo, 0);
        ASSERT(t);
        TASK_run(t, delta, 0);
      }
    }
  }
}

void RANGE_SENS_init(range_sens_cb_fn cb_fn) {
  memset(&hcsr, 0, sizeof(struct _sta));

  hcsr.cb_fn = cb_fn;

#if HCSR04_PULS_TIM == 1
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
#elif HCSR04_PULS_TIM == 2
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
#elif HCSR04_PULS_TIM == 3
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
#elif HCSR04_PULS_TIM == 4
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
#endif

  gpio_config(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, CLK_50MHZ, OUT, AF0, PUSHPULL, NOPULL);
  gpio_config(HCSR04_PULS_PORT, HCSR04_PULS_PIN_H, CLK_50MHZ, IN, AF0, OPENDRAIN, NOPULL);
  gpio_config(HCSR04_PULS_PORT, HCSR04_PULS_PIN_L, CLK_50MHZ, IN, AF0, OPENDRAIN, NOPULL);
  gpio_disable(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN);

  TIM_ICInitTypeDef TIM_ICInitStructure;
  TIM_ICInitStructure.TIM_Channel = _HCSR04_CHAN_H;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(_HCSR04_TIMER, &TIM_ICInitStructure);

  TIM_ICInitStructure.TIM_Channel = _HCSR04_CHAN_L;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
  TIM_ICInit(_HCSR04_TIMER, &TIM_ICInitStructure);

  hcsr.task = TASK_create(range_sens_trig_tmo, TASK_STATIC);
  ASSERT(hcsr.task != NULL);
}

s32_t RANGE_SENS_trigger(void) {
  if (hcsr.busy) {
    return ERR_RANGE_SENS_BUSY;
  }

  hcsr.busy = TRUE;

  // start timeout timer
  TASK_start_timer(hcsr.task, &hcsr.timer, 0, 0, 60, 0, "hcsr04");

  // enable flank sense and timer
  range_sens_enable();

  // give trigger pulse
  gpio_enable(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN);
  SYS_hardsleep_us(10); // todo busyloop on timer value instead
  gpio_disable(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN);

  return RANGE_SENS_OK;
}

#if HCSR04_PULS_TIM == 1
void TIM1_IRQHandler(void) {
  TRACE_IRQ_ENTER(TIM1_IRQn);
  range_sens_irq();
  TRACE_IRQ_EXIT(TIM1_IRQn);
}
#elif HCSR04_PULS_TIM == 2
void TIM2_IRQHandler(void) {
  TRACE_IRQ_ENTER(TIM2_IRQn);
  range_sens_irq();
  TRACE_IRQ_EXIT(TIM2_IRQn);
}
#elif HCSR04_PULS_TIM == 3
void TIM3_IRQHandler(void) {
  TRACE_IRQ_ENTER(TIM3_IRQn);
  range_sens_irq();
  TRACE_IRQ_EXIT(TIM3_IRQn);
}
#elif HCSR04_PULS_TIM == 4
void TIM4_IRQHandler(void) {
  TRACE_IRQ_ENTER(TIM4_IRQn);
  range_sens_irq();
  TRACE_IRQ_EXIT(TIM4_IRQn);
}
#endif
