/*
 * servo.h
 *
 *  Created on: Jan 15, 2014
 *      Author: petera
 */

#ifndef SERVO_H_
#define SERVO_H_

#include "system.h"

typedef enum {
  SERVO_CAM_PAN = 0,
  SERVO_CAM_TILT,
  SERVO_RADAR,
  _SERVO_CNT
} servo_out;

void SERVO_init(void);

void SERVO_set(servo_out servo, s16_t pwm);

void SERVO_control(servo_out servo, s8_t value);

void SERVO_update(void);


#endif /* SERVO_H_ */
