/*
 * stmpe811_impl.h
 *
 *  Created on: May 21, 2014
 *      Author: petera
 */

#ifndef STMPE811_IMPL_H_
#define STMPE811_IMPL_H_

#include "system.h"
#include "stmpe811_handler.h"

typedef void (* adc_cb_f)(u16_t adc_val);

void STMPE_init(void);
void STMPE_req_gpio_set(u8_t set, u8_t reset);
void STMPE_req_read_adc(u8_t adc, adc_cb_f fn);
void STMPE_req_read_temp(void);
void STMPE_req_read_int_sta(void);
u16_t STMPE_adc_value(void);
u16_t STMPE_temp_value(void);

#endif /* STMPE811_IMPL_H_ */

