/*
 * nrf905_impl.h
 *
 *  Created on: Nov 18, 2013
 *      Author: petera
 */

#ifndef NRF905_IMPL_H_
#define NRF905_IMPL_H_

#include "nrf905_driver.h"

// called upon reception of a radio packet
typedef void (*nrf905_rx)(u8_t *data, u8_t len);
// called upon transmission of a radio packet
typedef void (*nrf905_tx)(int res);
// called upon error
typedef void (*nrf905_err)(nrf905_state state, int res);

void NRF905_IMPL_init(nrf905_rx rx_cb, nrf905_tx tx_cb, nrf905_err err_cb);
void NRF905_IMPL_read_conf(void);
int NRF905_IMPL_set_conf(nrf905_config *c);
int NRF905_IMPL_set_addr(u8_t *addr);
int NRF905_IMPL_rx(void);
int NRF905_IMPL_tx(u8_t *data, u8_t len);
int NRF905_IMPL_carrier(void);
bool NRF905_IMPL_lbt_check_rts(u32_t ms);
void NRF905_IMPL_return_to_rx_after_tx(bool set);

#endif /* NRF905_IMPL_H_ */
