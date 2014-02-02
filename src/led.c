/*
 * led.c
 *
 *  Created on: Jan 27, 2014
 *      Author: petera
 */

#include "led.h"
#include "gpio.h"

typedef struct {
  gpio_port port;
  gpio_pin pin;
  bool state;
} led_pin;

static const led_pin led_pinmap[_LEDS] = {
    {.port = PORTB, .pin = PIN6, .state = FALSE},
#ifdef CONFIG_SPYBOT_XLEDS
    {.port = PORTB, .pin = PIN12, .state = FALSE},
    {.port = PORTB, .pin = PIN13, .state = FALSE},
#endif
};

void LED_init(void) {
  int i;
  for (i = 0; i < _LEDS; i++) {
    gpio_config(led_pinmap[i].port, led_pinmap[i].pin, CLK_2MHZ, OUT, AF0, OPENDRAIN, NOPULL);
    LED_set(i, led_pinmap[i].state);
  }
}

void LED_set(led l, bool on) {
  if (l >= _LEDS) return;
  if (on) {
    gpio_disable(led_pinmap[l].port, led_pinmap[l].pin);
  } else {
    gpio_enable(led_pinmap[l].port, led_pinmap[l].pin);
  }
}

void LED_enable(led l) {
  if (l >= _LEDS) return;
  gpio_disable(led_pinmap[l].port, led_pinmap[l].pin);
}

void LED_disable(led l) {
  if (l >= _LEDS) return;
  gpio_enable(led_pinmap[l].port, led_pinmap[l].pin);
}

