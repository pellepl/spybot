/*
 * app.h
 *
 *  Created on: Jan 2, 2014
 *      Author: petera
 */

#ifndef APP_H_
#define APP_H_

#include "system.h"
#include "comm.h"

void APP_init(void);
void APP_tx_dbg(const char *s);
u8_t APP_pair_status(void);

void APP_comrad_rx(comm_arg *rx, u16_t len, u8_t *data, bool already_received);
void APP_comrad_ack(comm_arg *rx, u16_t seq_no, u16_t len, u8_t *data);
void APP_comrad_err(u16_t seq_no, int err);

#endif /* APP_H_ */
