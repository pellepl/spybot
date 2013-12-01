/*
 * cvideo.c
 *
 *  Created on: Nov 30, 2013
 *      Author: petera
 */

#include "cvideo.h"
#include "gpio.h"
#include "miniutils.h"

#define CLK_MHZ           72
#define NS_TO_TICKS(x)    ((x)*CLK_MHZ/1000)

#define TIM_ENABLE        TIM1->CR1 |= TIM_CR1_CEN
#define TIM_DISABLE       TIM1->CR1 &= (u16_t)(~((u16_t)TIM_CR1_CEN))
#define TIM_NEXT(x)       TIM1->ARR = (x)-1 /*TIM_SetAutoreload(TIM1, (x)-1)*/
#define TIM_NEXT_NS(x)    TIM_NEXT(NS_TO_TICKS((x)))

static struct {
  u8_t vsync_initial;
  volatile bool in_sync;
  volatile u16_t cur_hsync;

  u16_t hline_state;
  u16_t x;
  u16_t y;
  s16_t dx;
  s16_t dy;
  u16_t dbg_last_hsyncs;
} vid;

static void cvideo_vsync_irq(gpio_pin pin) {
  vid.dbg_last_hsyncs = vid.cur_hsync;

  if (vid.vsync_initial < 4) {
    vid.vsync_initial++;
    return;
  } else if (vid.vsync_initial == 4){
    vid.vsync_initial = 5;
    return;
  }

  bool nsync = vid.cur_hsync == 320;
  if (nsync != vid.in_sync) {
    if (nsync) {
      TIM1->SR = (u16_t)~TIM_IT_Update;
      NVIC_EnableIRQ(TIM1_UP_IRQn);
    } else {
      GPIO_set(GPIOB, 0, GPIO_Pin_15);
      NVIC_DisableIRQ(TIM1_UP_IRQn);
      TIM_DISABLE;
      TIM1->SR = (u16_t)~TIM_IT_Update;

      vid.in_sync = nsync;
      vid.cur_hsync = 0;
      vid.vsync_initial = 0;

      return;
    }
  }

  vid.in_sync = nsync;
  vid.cur_hsync = 0;

  vid.x += vid.dx;
  vid.y += vid.dy;
  if (vid.x > 1760) {
    vid.dx = -9;
  } else if (vid.x < 10) {
    vid.dx = 9;
  }
  if (vid.y > 220) {
    vid.dy = -1;
  } else if (vid.y < 10) {
    vid.dy = 1;
  }
}

static void cvideo_hsync_irq(gpio_pin pin) {
  vid.cur_hsync++;
  if (!vid.in_sync) return;
  GPIO_set(GPIOB, 0, GPIO_Pin_15);
  if (vid.in_sync &&
      vid.cur_hsync >= 30+vid.y && vid.cur_hsync < 45+vid.y) {
    vid.hline_state = 0;
    TIM_NEXT((200+vid.x)/2);
    TIM_ENABLE;
  } else {
    TIM_DISABLE;
  }
}

static void cvideo_trig_irq(void) {
  if (!vid.in_sync) {
    GPIO_set(GPIOB, 0, GPIO_Pin_15);
    NVIC_DisableIRQ(TIM1_UP_IRQn);
    TIM_DISABLE;
    TIM1->SR = (u16_t)~TIM_IT_Update;
    return;
  }
  if (vid.hline_state == 0) {
    GPIO_set(GPIOB, GPIO_Pin_15, 0);
    vid.hline_state = 1;
    TIM_NEXT(144/2);
  } else if (vid.hline_state == 1) {
    GPIO_set(GPIOB, 0, GPIO_Pin_15);
    vid.hline_state = 2;
    TIM_DISABLE;
  } else {
    vid.in_sync = FALSE;
    GPIO_set(GPIOB, 0, GPIO_Pin_15);
    TIM_DISABLE;
  }
}

void CVIDEO_init(void) {
  memset(&vid, 0, sizeof(vid));

  gpio_interrupt_config(PORTB, PIN8, cvideo_vsync_irq, FLANK_UP);
  gpio_interrupt_config(PORTB, PIN14, cvideo_hsync_irq, FLANK_UP);
  gpio_interrupt_mask_enable(PORTB, PIN14, TRUE);
  gpio_interrupt_mask_enable(PORTB, PIN8, TRUE);

  TIM1->SR = (u16_t)~TIM_IT_Update;
  NVIC_DisableIRQ(TIM1_UP_IRQn);
  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

}

void CVIDEO_dump(void) {
  gpio_interrupt_mask_disable(PORTB, PIN8);
  gpio_interrupt_mask_disable(PORTB, PIN14);


  gpio_interrupt_mask_enable(PORTB, PIN8, TRUE);
  gpio_interrupt_mask_enable(PORTB, PIN14, TRUE);
}

void TIM1_UP_IRQHandler(void)
{
  //TRACE_IRQ_ENTER(TIM1_UP_IRQn);
  if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
    //TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    TIM1->SR = (u16_t)~TIM_IT_Update;
    cvideo_trig_irq();
  }
  //TRACE_IRQ_EXIT(TIM1_UP_IRQn);
}
