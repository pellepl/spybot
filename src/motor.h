/*
 * motor.h
 *
 *  Created on: Jan 10, 2014
 *      Author: petera
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "system.h"

void MOTOR_init(void);

void MOTOR_go(s8_t x);

void MOTOR_control(s16_t hori, s16_t veri);

void MOTOR_update(void);


#endif /* MOTOR_H_ */
