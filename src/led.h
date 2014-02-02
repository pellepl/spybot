/*
 * led.h
 *
 *  Created on: Jan 27, 2014
 *      Author: petera
 */

#ifndef LED_H_
#define LED_H_

#include "system.h"

typedef enum {
  LED_MAIN = 0,
#ifdef CONFIG_SPYBOT_XLEDS
  LED_X_PRIM,
  LED_X_SEC,
#endif
  _LEDS
} led;

void LED_init(void);
void LED_enable(led l);
void LED_disable(led l);
void LED_set(led l, bool on);

#endif /* LED_H_ */
