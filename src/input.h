/*
 * input.h
 *
 *  Created on: Dec 27, 2013
 *      Author: petera
 */

#ifndef INPUT_H_
#define INPUT_H_

typedef enum {
  IDLE = 0,
  UP = 1,
  DOWN = 2,
  LEFT = 4,
  RIGHT = 8,
  PRESS = 16
} input_type;

#endif /* INPUT_H_ */
