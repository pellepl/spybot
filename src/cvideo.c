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

static void cvideo_hsync_irq(gpio_pin pin) {
  vid.cur_hsync++;
  if (!vid.in_sync) return;
  if (vid.in_sync &&
      vid.cur_hsync >= CVID_MIN_HSCANLINE && vid.cur_hsync < CVID_MAX_HSCANLINE) {
    DMA1_Channel5->CCR &= (u16_t)(~DMA_CCR1_EN);
    DMA1_Channel5->CNDTR = CVID_HSCANLINE_BYTE_LEN;
#ifdef CVID_HALF_VERTICAL
    DMA1_Channel5->CMAR = (u32_t)(&cvideo_gram[((vid.cur_hsync - CVID_MIN_HSCANLINE)/2) * CVID_HSCANLINE_BYTE_LEN]);
#else
    DMA1_Channel5->CMAR = (u32_t)(&cvideo_gram[(vid.cur_hsync - CVID_MIN_HSCANLINE) * CVID_HSCANLINE_BYTE_LEN]);
#endif
    DMA1_Channel5->CCR |= DMA_CCR1_EN;
    SPI2->CR1 |= 0x0040;
  }
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
