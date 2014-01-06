/*
 * adc.h
 *
 *  Created on: May 8, 2013
 *      Author: petera
 */

#ifndef ADC_H_
#define ADC_H_

#include "system.h"

void ADC_init();
u32_t ADC_sample(int ch);
void ADC_irq(void);

#endif /* ADC_H_ */
