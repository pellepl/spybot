/*
 * input.c
 *
 *  Created on: Jan 8, 2014
 *      Author: petera
 */

#include "input.h"
#include "hud.h"
#include "gpio.h"

#define ADC_LEVEL_HIGH    0xe00
#define ADC_LEVEL_LOW     0x100

static input_type cur_input = IN_IDLE;
static time press_time;

static void input_main(input_type in, bool change) {
  // check joystick long-press
  if (in & IN_PRESS) {
    if (change) {
      press_time = SYS_get_time_ms();
    } else {
      if (SYS_get_time_ms()-press_time > 3000) {
        HUD_state(HUD_CONFIG);
      }
    }
  }
}

void INPUT_init(void) {
  // joystick button
  gpio_config_in(PORTB, PIN7, CLK_2MHZ);
  // TODO
  // button a
//  gpio_config_in(PORTB, PIN9, CLK_2MHZ);
  // switch a
//  gpio_config_in(PORTB, PIN10, CLK_2MHZ);
  // switch b
//  gpio_config_in(PORTB, PIN11, CLK_2MHZ);

}

void INPUT_read(u16_t joystick_v, u16_t joystick_h) {
  input_type new_input = IN_IDLE;
  if (!gpio_get(PORTB, PIN7)) {
    new_input |= IN_PRESS;
  }
  if (joystick_v > ADC_LEVEL_HIGH) {
    new_input |= IN_UP;
  } else if (joystick_v < ADC_LEVEL_LOW) {
    new_input |= IN_DOWN;
  }
  if (joystick_h > ADC_LEVEL_HIGH) {
    new_input |= IN_RIGHT;
  } else if (joystick_h < ADC_LEVEL_LOW) {
    new_input |= IN_LEFT;
  }

  bool change = (cur_input ^ new_input) != 0;

  switch (HUD_get_state()) {
  case HUD_MAIN:
    input_main(new_input, change);
    break;
  case HUD_CONFIG:
    HUD_input(new_input, change);
    break;
  default:
    break;
  }

  cur_input = new_input;
}