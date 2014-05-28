/*
 * cvideo.c
 *
 *  Created on: Nov 30, 2013
 *      Author: petera
 */

#include "cvideo.h"
#include "gpio.h"
#include "miniutils.h"

//
// China Surveillance Cam V1
//  Black:  GND
//  Red:    5V
//  Yellow: 8V
//

#define CVID_DISABLE_EFFECTS

#define CVID_HALF_VERTICAL
//#define CVID_DISPLAY_SYNCED_ONLY

#ifdef NTSC
#define CVID_MIN_HSCANLINE      18
#define CVID_HSCANLINES         227
#else
#define CVID_MIN_HSCANLINE      23
#define CVID_HSCANLINES         275
#endif

#define CVID_MAX_HSCANLINE      (CVID_HSCANLINES+CVID_MIN_HSCANLINE)
#define CVID_HSCANLINE_BYTE_LEN 29

#define CVID_GRAM_WIDTH         (CVID_HSCANLINE_BYTE_LEN * 8)-4
#ifdef CVID_HALF_VERTICAL
#define CVID_GRAM_HEIGHT        ((CVID_HSCANLINES - 1)/2)
#else
#define CVID_GRAM_HEIGHT        (CVID_HSCANLINES - 1)
#endif

#ifdef CVID_HALF_VERTICAL
static u8_t cvideo_gram_1[(CVID_HSCANLINES/2 + 1)*CVID_HSCANLINE_BYTE_LEN];
static u8_t cvideo_gram_2[(CVID_HSCANLINES/2 + 1)*CVID_HSCANLINE_BYTE_LEN];
#else
static u8_t cvideo_gram_1[CVID_HSCANLINES*CVID_HSCANLINE_BYTE_LEN];
static u8_t cvideo_gram_2[CVID_HSCANLINES*CVID_HSCANLINE_BYTE_LEN];
#endif

static struct {
  vbl_cb vbl_cb_fn;
  u8_t vsync_initial;
  volatile bool in_sync;
  volatile u16_t cur_hsync;
  u16_t dbg_last_hsyncs;
  volatile s16_t v_offset;
  volatile s16_t v_scroll;
#ifndef CVID_DISABLE_EFFECTS
  volatile s16_t effect_stage;
  u8_t effect;
#endif
  gcontext *gctx;
  u8_t *cur_gram;
  volatile bool gram_switch;
  volatile bool gram_dblbuf;
  video_input_t input;
} vid;

static void cvideo_disable_output(void) {
  gpio_config(PORTB, PIN15, CLK_50MHZ, IN, AF0, OPENDRAIN, NOPULL);
}

static void cvideo_vsync_irq(gpio_pin pin) {
  vid.dbg_last_hsyncs = vid.cur_hsync;

#ifdef CVID_DISPLAY_SYNCED_ONLY
  if (vid.vsync_initial < 2) {
    vid.vsync_initial++;
    return;
  } else if (vid.vsync_initial == 2){
    vid.vsync_initial = 3;
    return;
  }
#else
  bool nsync = TRUE;
  vid.cur_hsync = 0;
  vid.vsync_initial = 0;
#endif

  if (vid.v_scroll != 0) {
    vid.v_offset += vid.v_scroll;
  }
#ifndef CVID_DISABLE_EFFECTS
  if (vid.effect) {
    vid.effect_stage += 8;
  }
#endif
#ifdef CVID_DISPLAY_SYNCED_ONLY
  bool nsync = vid.cur_hsync == 320;
  if (nsync != vid.in_sync) {
    if (nsync) {
      gpio_config(PORTB, PIN15, CLK_50MHZ, AF, AF0, PUSHPULL, NOPULL);
    } else {
      cvideo_disable_output();

      vid.in_sync = nsync;
      vid.cur_hsync = 0;
      vid.vsync_initial = 0;

      return;
    }
  }
#endif
  vid.in_sync = nsync;
  vid.cur_hsync = 0;

  // just in case, e.g. if getting a video signal with less scanlines than defined
  if (vid.gram_switch && vid.gram_dblbuf) {
    vid.gram_switch = FALSE;
    vid.cur_gram = vid.cur_gram == cvideo_gram_1 ? cvideo_gram_2 : cvideo_gram_1;
    if (vid.gctx) {
      vid.gctx->gram = vid.cur_gram;
    }
  }
}

static void cvideo_hsync_irq(gpio_pin)  __attribute__(( optimize(3) ));
static void cvideo_hsync_irq(gpio_pin pin) {
  vid.cur_hsync++;
#ifdef CVID_DISPLAY_SYNCED_ONLY
  if (!vid.in_sync) return;
  if (vid.in_sync)
#endif
  {
    if (vid.cur_hsync >= CVID_MIN_HSCANLINE && vid.cur_hsync < CVID_MAX_HSCANLINE) {
      DMA1_Channel5->CCR &= (u16_t)(~DMA_CCR1_EN);
      DMA1_Channel5->CNDTR = CVID_HSCANLINE_BYTE_LEN;
      u32_t scanline = vid.cur_hsync - CVID_MIN_HSCANLINE;
#ifndef CVID_DISABLE_EFFECTS
      s16_t sc = scanline;
      if (vid.effect > 64) {
        sc = (sc & 0xffe0) + (32 - (sc & 0x1f));
      } else if (vid.effect > 32) {
        sc = (sc & 0xfff0) + (16 - (sc & 0x0f));
      }
      s16_t fx_d1 = sc;
      s16_t fx_d2 = fx_d1+(CVID_HSCANLINES/4);
      s16_t fx_u1 = CVID_HSCANLINES - sc;
      s16_t fx_u2 = fx_u1+(CVID_HSCANLINES/4);
      if (vid.effect) {
        switch (vid.effect&0x1f) {
        case 0:
          break;
        case 1:
          if (fx_d1 > vid.effect_stage) return;
          break;
        case 2:
          if (fx_u1 < vid.effect_stage) return;
          break;
        case 3:
          if (fx_d1 < vid.effect_stage) return;
          break;
        case 4:
          if (fx_u1 > vid.effect_stage) return;
          break;

        case 5:
          if (fx_d1 > vid.effect_stage) return;
          if ((fx_d2 > vid.effect_stage) && (scanline & 1)) return;
          break;
        case 6:
          if (fx_u2 < vid.effect_stage) return;
          if ((fx_u1 < vid.effect_stage) && (scanline & 1)) return;
          break;
        case 7:
          if (fx_d2 < vid.effect_stage) return;
          if ((fx_d1 < vid.effect_stage) && (scanline & 1)) return;
          break;
        case 8:
          if (fx_u1 > vid.effect_stage) return;
          if ((fx_u2 > vid.effect_stage) && (scanline & 1)) return;
          break;

        case 9:
          if (fx_d1 > vid.effect_stage) return;
          if (fx_u1 > vid.effect_stage) return;
          break;
        case 10:
          if (fx_d1 < vid.effect_stage) return;
          if (fx_u1 < vid.effect_stage) return;
          break;
        case 11:
          if ((fx_d1 > vid.effect_stage) & (fx_u1 > vid.effect_stage)) return;
          break;
        case 12:
          if ((fx_d1 < vid.effect_stage) & (fx_u1 < vid.effect_stage)) return;
          break;

        case 13:
          if (fx_d1 > vid.effect_stage) return;
          if (fx_u1 > vid.effect_stage) return;
          if ((fx_d2 > vid.effect_stage) & (scanline & 1)) return;
          if ((fx_u2 > vid.effect_stage) & (scanline & 1)) return;
          break;
        case 14:
          if (fx_d2 < vid.effect_stage) return;
          if (fx_u2 < vid.effect_stage) return;
          if ((fx_d1 < vid.effect_stage) & (scanline & 1)) return;
          if ((fx_u1 < vid.effect_stage) & (scanline & 1)) return;
          break;
        case 15:
          if ((fx_d1 > vid.effect_stage) & (fx_u1 > vid.effect_stage)) return;
          if ((fx_d2 > vid.effect_stage) & (fx_u2 > vid.effect_stage) & (scanline & 1)) return;
          break;
        case 16:
          if ((fx_d2 < vid.effect_stage) & (fx_u2 < vid.effect_stage)) return;
          if ((fx_d1 < vid.effect_stage) & (fx_u1 < vid.effect_stage) & (scanline & 1)) return;
          break;

        default:
          return;
        }
      }
      if (vid.effect) {
        if (vid.effect_stage > CVID_HSCANLINES*2) {
            vid.effect = 0;
        };
      }
#endif

#ifdef CVID_HALF_VERTICAL
      scanline /= 2;
#endif
      scanline = (scanline + vid.v_offset) % CVID_GRAM_HEIGHT;

      u8_t *gram;
      if (vid.gram_dblbuf) {
        gram = vid.cur_gram == cvideo_gram_1 ? cvideo_gram_2 : cvideo_gram_1;
      } else {
        gram = vid.cur_gram;
      }
      u8_t *gram_scanline = &(gram)[scanline * CVID_HSCANLINE_BYTE_LEN] ;
      DMA1_Channel5->CMAR = (u32_t)(gram_scanline);
      DMA1_Channel5->CCR |= DMA_CCR1_EN;
      SPI2->CR1 |= 0x0040;
    } else if (vid.cur_hsync == CVID_MAX_HSCANLINE) {
      if (vid.gram_switch && vid.gram_dblbuf) {
        vid.gram_switch = FALSE;
        vid.cur_gram = vid.cur_gram == cvideo_gram_1 ? cvideo_gram_2 : cvideo_gram_1;
        if (vid.gctx) {
          vid.gctx->gram = vid.cur_gram;
        }
      }
      if (vid.vbl_cb_fn) vid.vbl_cb_fn();
    }
  }
}

void CVIDEO_set_v_offset(u16_t offset) {
  vid.v_offset = offset;
}

void CVIDEO_set_v_scroll(s16_t speed) {
  vid.v_scroll = speed;
}

void CVIDEO_set_effect(u8_t effect) {
#ifndef CVID_DISABLE_EFFECTS
  vid.effect_stage = 0;
  vid.effect = effect;
#endif
}


void CVIDEO_init(vbl_cb vbl_cb_fn) {
  memset(&vid, 0, sizeof(vid));
  memset(&cvideo_gram_1[0], 0, sizeof(cvideo_gram_1));
  memset(&cvideo_gram_2[0], 0, sizeof(cvideo_gram_2));
  cvideo_disable_output();
  vid.vbl_cb_fn = vbl_cb_fn;
  vid.cur_gram = cvideo_gram_1;

  gpio_interrupt_config(PORTB, PIN8, cvideo_vsync_irq, FLANK_UP);
  gpio_interrupt_config(PORTB, PIN14, cvideo_hsync_irq, FLANK_UP);
  gpio_interrupt_mask_enable(PORTB, PIN14, TRUE);
  gpio_interrupt_mask_enable(PORTB, PIN8, TRUE);

  gpio_config(PORTB, PIN15, CLK_50MHZ, AF, AF0, PUSHPULL, NOPULL);
}

void CVIDEO_init_gcontext(gcontext *gctx) {
  gctx->width = (CVID_HSCANLINE_BYTE_LEN-1)*8;
  gctx->hstride= CVID_HSCANLINE_BYTE_LEN;
  gctx->height = CVID_GRAM_HEIGHT;
  gctx->gram = vid.cur_gram;
  vid.gctx = gctx;
}

void CVIDEO_gram_double_buffer(bool enable) {
  vid.gram_dblbuf = enable;
}

void CVIDEO_gram_switch(void) {
  vid.gram_switch = TRUE;
}

video_input_t CVIDEO_get_input(void) {
  return vid.input;
}

void CVIDEO_set_input(video_input_t input) {
  vid.input = input;
  if (input == INPUT_CAMERA || input == INPUT_NONE) {
    TIM_DeInit(TIM1);
    TIM_Cmd(TIM1, DISABLE);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);
    NVIC_DisableIRQ(TIM1_UP_IRQn);
    TIM_CtrlPWMOutputs(TIM1, DISABLE);

    if (input == INPUT_CAMERA) {
      if (VID_SEL_PINVAL_GEN) {
        gpio_disable(VID_SEL_PORT, VID_SEL_PIN);
      } else {
        gpio_enable(VID_SEL_PORT, VID_SEL_PIN);
      }
    } else {
      if (VID_SEL_PINVAL_GEN) {
        gpio_enable(VID_SEL_PORT, VID_SEL_PIN);
      } else {
        gpio_disable(VID_SEL_PORT, VID_SEL_PIN);
      }
    }

  } else {
    // INPUT_GENERATED

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    TIM_Cmd(TIM1, DISABLE);
    NVIC_DisableIRQ(TIM1_UP_IRQn);
    //TIM_DeInit(TIM1);

    if (VID_SEL_PINVAL_GEN) {
      gpio_enable(VID_SEL_PORT, VID_SEL_PIN);
    } else {
      gpio_disable(VID_SEL_PORT, VID_SEL_PIN);
    }

    TIM_TimeBaseStructure.TIM_Period = 4608;//SYS_CPU_FREQ / 15625;
    TIM_TimeBaseStructure.TIM_Prescaler = 1-1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 4608/2;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM1, DISABLE);

    TIM_ClearITPendingBit(TIM1,
        TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 |
        TIM_IT_COM | TIM_IT_Trigger | TIM_IT_Break);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    NVIC_EnableIRQ(TIM1_UP_IRQn);

    TIM_SetCounter(TIM1, 0);
    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
  }
}

void TIM1_UP_IRQHandler(void) {
  static int linecount = 0;
  TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
  linecount++;

#define US2_35 169
#define US4_7  338
#define USFULL 4608

  if (linecount == 3 || linecount == 310) {
    TIM_SetCompare1(TIM1, US2_35);
  }

  if (linecount == 5) {
    TIM_SetCompare1(TIM1, US4_7);
  }

  if (linecount == 313) {
    TIM_SetCompare1(TIM1, USFULL/2-US2_35);
    linecount = 0;
  }
}
