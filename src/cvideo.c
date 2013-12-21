/*
 * cvideo.c
 *
 *  Created on: Nov 30, 2013
 *      Author: petera
 */

#include "cvideo.h"
#include "gpio.h"
#include "miniutils.h"

#define CVID_HALF_VERTICAL

#define CVID_MIN_HSCANLINE      21
#define CVID_MAX_HSCANLINE      (CVID_HSCANLINES+CVID_MIN_HSCANLINE)
#define CVID_HSCANLINES         275
#define CVID_HSCANLINE_BYTE_LEN 29

#define CVID_GRAM_WIDTH         (CVID_HSCANLINE_BYTE_LEN * 8)-4
#ifdef CVID_HALF_VERTICAL
#define CVID_GRAM_HEIGHT        ((CVID_HSCANLINES - 1)/2)
#else
#define CVID_GRAM_HEIGHT        (CVID_HSCANLINES - 1)
#endif

#ifdef CVID_HALF_VERTICAL
static u8_t cvideo_gram[(CVID_HSCANLINES/2 + 1)*CVID_HSCANLINE_BYTE_LEN];
#else
static u8_t cvideo_gram[CVID_HSCANLINES*CVID_HSCANLINE_BYTE_LEN];
#endif

static struct {
  u8_t vsync_initial;
  volatile bool in_sync;
  volatile u16_t cur_hsync;
  u16_t dbg_last_hsyncs;
  volatile s16_t v_offset;
  volatile s16_t v_scroll;
  volatile s16_t effect_stage;
  u8_t effect;
} vid;

static void cvideo_disable_output(void) {
  gpio_config(PORTB, PIN15, CLK_50MHZ, IN, AF0, OPENDRAIN, NOPULL);
}

static void cvideo_vsync_irq(gpio_pin pin) {
  vid.dbg_last_hsyncs = vid.cur_hsync;

  if (vid.vsync_initial < 2) {
    vid.vsync_initial++;
    return;
  } else if (vid.vsync_initial == 2){
    vid.vsync_initial = 3;
    return;
  }

  if (vid.v_scroll != 0) {
    vid.v_offset += vid.v_scroll;
  }
  if (vid.effect) {
    vid.effect_stage += 8;
  }

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

  vid.in_sync = nsync;
  vid.cur_hsync = 0;
}

static void cvideo_hsync_irq(gpio_pin)  __attribute__(( optimize(3) ));
static void cvideo_hsync_irq(gpio_pin pin) {
  vid.cur_hsync++;
  if (!vid.in_sync) return;
  if (vid.in_sync &&
      vid.cur_hsync >= CVID_MIN_HSCANLINE && vid.cur_hsync < CVID_MAX_HSCANLINE) {
    DMA1_Channel5->CCR &= (u16_t)(~DMA_CCR1_EN);
    DMA1_Channel5->CNDTR = CVID_HSCANLINE_BYTE_LEN;
    u32_t scanline = vid.cur_hsync - CVID_MIN_HSCANLINE;
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

#ifdef CVID_HALF_VERTICAL
    scanline /= 2;
#endif
    scanline = (scanline + vid.v_offset) % CVID_GRAM_HEIGHT;

    u8_t *gram_scanline = &cvideo_gram[scanline * CVID_HSCANLINE_BYTE_LEN] ;
    DMA1_Channel5->CMAR = (u32_t)(gram_scanline);
    DMA1_Channel5->CCR |= DMA_CCR1_EN;
    SPI2->CR1 |= 0x0040;
  }
}

void CVIDEO_set_v_offset(u16_t offset) {
  vid.v_offset = offset;
}

void CVIDEO_set_v_scroll(s16_t speed) {
  vid.v_scroll = speed;
}

void CVIDEO_set_effect(u8_t effect) {
  vid.effect_stage = 0;
  vid.effect = effect;
}


void CVIDEO_init(void) {
  memset(&vid, 0, sizeof(vid));
  memset(&cvideo_gram[0], 0, sizeof(cvideo_gram));
  cvideo_disable_output();

  gpio_interrupt_config(PORTB, PIN8, cvideo_vsync_irq, FLANK_UP);
  gpio_interrupt_config(PORTB, PIN14, cvideo_hsync_irq, FLANK_UP);
  gpio_interrupt_mask_enable(PORTB, PIN14, TRUE);
  gpio_interrupt_mask_enable(PORTB, PIN8, TRUE);

  gpio_config(PORTB, PIN15, CLK_50MHZ, AF, AF0, PUSHPULL, NOPULL);
}

void CVIDEO_init_gcontext(gcontext *gctx) {
  gctx->width = (CVID_HSCANLINE_BYTE_LEN-1)*8;
  gctx->hscan= CVID_HSCANLINE_BYTE_LEN;
  gctx->height = CVID_GRAM_HEIGHT;
  gctx->gram = cvideo_gram;
}

void CVIDEO_dump(void) {
  gpio_interrupt_mask_disable(PORTB, PIN8);
  gpio_interrupt_mask_disable(PORTB, PIN14);

  // hmm

  gpio_interrupt_mask_enable(PORTB, PIN8, TRUE);
  gpio_interrupt_mask_enable(PORTB, PIN14, TRUE);
}
