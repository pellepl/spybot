/*
 * range_sens_hcsr04.h
 *
 *  Created on: Nov 15, 2013
 *      Author: petera
 */

#ifndef RANGE_SENS_HCSR04_DRIVER_H_
#define RANGE_SENS_HCSR04_DRIVER_H_

#include "system.h"

#define RANGE_SENS_OK         0
#define ERR_RANGE_SENS_BUSY   -30000

#ifndef HCSR04_TRIG_PORT
#define HCSR04_TRIG_PORT    PORTA
#endif
#ifndef HCSR04_TRIG_PIN
#define HCSR04_TRIG_PIN     PIN4
#endif
#ifndef HCSR04_PULS_PORT
#define HCSR04_PULS_PORT    PORTB
#endif
#ifndef HCSR04_PULS_PIN_H
#define HCSR04_PULS_PIN_H   PIN0
#endif
#ifndef HCSR04_PULS_PIN_L
#define HCSR04_PULS_PIN_L   PIN1
#endif
#ifndef HCSR04_PULS_TIM
#define HCSR04_PULS_TIM     3
#endif
#ifndef HCSR04_PULS_TIMCH_H
#define HCSR04_PULS_TIMCH_H 3
#endif
#ifndef HCSR04_PULS_TIMCH_L
#define HCSR04_PULS_TIMCH_L 4
#endif

typedef void (*range_sens_cb_fn)(u32_t echo_cpu_ticks);

#define RANGE_SENS_CPU_TICKS_TO_MM(x) \
  ( (x)*343200 / (2*SYS_CPU_FREQ) )

void RANGE_SENS_init(range_sens_cb_fn cb_fn);
s32_t RANGE_SENS_trigger(void);

#endif /* RANGE_SENS_HCSR04_DRIVER_H_ */
