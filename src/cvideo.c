/*
 * cvideo.c
 *
 *  Created on: Nov 30, 2013
 *      Author: petera
 */

#include "cvideo.h"
#include "gpio.h"
#include "miniutils.h"

#define CVID_MIN_HSCANLINE      21
#define CVID_MAX_HSCANLINE      (CVID_HSCANLINES+CVID_MIN_HSCANLINE)
#define CVID_HSCANLINES         275
#define CVID_HSCANLINE_BYTE_LEN 29

#define CVID_GRAM_WIDTH         (CVID_HSCANLINE_BYTE_LEN * 8)-4
#define CVID_GRAM_HEIGHT        (CVID_HSCANLINES - 1)

static u8_t cvideo_gram[(CVID_MAX_HSCANLINE - CVID_MIN_HSCANLINE)*CVID_HSCANLINE_BYTE_LEN];

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
    DMA1_Channel5->CMAR = (u32_t)(&cvideo_gram[(vid.cur_hsync - CVID_MIN_HSCANLINE) * CVID_HSCANLINE_BYTE_LEN]);
    DMA1_Channel5->CCR |= DMA_CCR1_EN;
    SPI2->CR1 |= 0x0040;
  }
}

static void cvideo_put_pixel(int x, int y, bool on) {
  if (on) {
    cvideo_gram[y*CVID_HSCANLINE_BYTE_LEN + x/8] |= (1<<(x&7));
  } else {
    cvideo_gram[y*CVID_HSCANLINE_BYTE_LEN + x/8] &= ~(1<<(x&7));
  }
}

void CVIDEO_init(void) {
  memset(&vid, 0, sizeof(vid));
  memset(&cvideo_gram[0], 0, sizeof(cvideo_gram));

  int i;
  for (i = 5; i < CVID_GRAM_WIDTH-5; i++) {
    cvideo_put_pixel(i, 0, TRUE);
    cvideo_put_pixel(i, CVID_GRAM_HEIGHT, TRUE);
  }
  for (i = 0; i < CVID_GRAM_HEIGHT; i++) {
    cvideo_put_pixel(5, i, TRUE);
    cvideo_put_pixel(CVID_GRAM_WIDTH-5, i, TRUE);
  }

  cvideo_disable_output();

  gpio_interrupt_config(PORTB, PIN8, cvideo_vsync_irq, FLANK_UP);
  gpio_interrupt_config(PORTB, PIN14, cvideo_hsync_irq, FLANK_UP);
  gpio_interrupt_mask_enable(PORTB, PIN14, TRUE);
  gpio_interrupt_mask_enable(PORTB, PIN8, TRUE);

  gpio_config(PORTB, PIN15, CLK_50MHZ, AF, AF0, PUSHPULL, NOPULL);
}

void CVIDEO_dump(void) {
  gpio_interrupt_mask_disable(PORTB, PIN8);
  gpio_interrupt_mask_disable(PORTB, PIN14);

  // hmm

  gpio_interrupt_mask_enable(PORTB, PIN8, TRUE);
  gpio_interrupt_mask_enable(PORTB, PIN14, TRUE);
}
