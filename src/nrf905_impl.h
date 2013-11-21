/*
 * nrf905_impl.h
 *
 *  Created on: Nov 18, 2013
 *      Author: petera
 */

#ifndef NRF905_IMPL_H_
#define NRF905_IMPL_H_

#include "nrf905_driver.h"

void NRF905_IMPL_read_conf(void);
void NRF905_IMPL_init(void);
void NRF905_IMPL_set_conf(nrf905_config *c);
void NRF905_IMPL_set_addr(void);
void NRF905_IMPL_rx(void);
void NRF905_IMPL_tx(void);

#endif /* NRF905_IMPL_H_ */
