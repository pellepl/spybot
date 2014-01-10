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

void MOTOR_control(s8_t left, s8_t right);

void MOTOR_control_vector(s8_t hori, s8_t veri);

void MOTOR_update(void);


#endif /* MOTOR_H_ */
