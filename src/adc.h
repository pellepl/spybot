/*
 * adc.h
 *
 *  Created on: May 8, 2013
 *      Author: petera
 */

#ifndef ADC_H_
#define ADC_H_

#define ADC_OK        0
#define ADC_ERR_BUSY  -6000

#include "system.h"

typedef void (*adc_cb)(u16_t ch1, u16_t ch2);

void ADC_init(void);
s32_t ADC_sample_vref_sync(u16_t *vref);
s32_t ADC_sample_joystick(adc_cb cb);
s32_t ADC_sample_sound(adc_cb cb, u8_t *buf, u32_t len);
void ADC_irq(void);

#endif /* ADC_H_ */
