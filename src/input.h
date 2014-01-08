/*
 * input.h
 *
 *  Created on: Dec 27, 2013
 *      Author: petera
 */

#ifndef INPUT_H_
#define INPUT_H_

#include "system.h"

typedef enum {
  IN_IDLE = 0,
  IN_UP = 1,
  IN_DOWN = 2,
  IN_LEFT = 4,
  IN_RIGHT = 8,
  IN_PRESS = 16
} input_type;

void INPUT_init(void);
void INPUT_read(u16_t joystick_v, u16_t joystick_h);

#endif /* INPUT_H_ */
