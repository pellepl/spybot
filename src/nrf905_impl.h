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
// called upon radio configuration
typedef void (*nrf905_cfg)(int res);
// called upon error
typedef void (*nrf905_err)(nrf905_state state, int res);

void NRF905_IMPL_init(nrf905_rx rx_cb, nrf905_tx tx_cb, nrf905_cfg cfg_cb, nrf905_err err_cb);
void NRF905_IMPL_read_conf(void);
int NRF905_IMPL_conf(nrf905_config *c, bool force);
int NRF905_IMPL_conf_tx_addr(u8_t *addr, bool force);
int NRF905_IMPL_conf_channel(u16_t channel_freq, bool force);
int NRF905_IMPL_conf_pa(nrf905_cfg_pa_pwr pa, bool force);
int NRF905_IMPL_rx(void);
int NRF905_IMPL_tx(u8_t *data, u8_t len);
int NRF905_IMPL_carrier(void);
bool NRF905_IMPL_lbt_check_rts(u32_t ms);

#endif /* NRF905_IMPL_H_ */
