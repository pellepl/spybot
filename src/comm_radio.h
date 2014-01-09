/*
 * comm_radio.h
 *
 *  Created on: Dec 13, 2013
 *      Author: petera
 */

#ifndef COMM_RADIO_H_
#define COMM_RADIO_H_

#include "system.h"
#include "comm.h"

#define COMM_RADIO_LNK_MAX_DATA     COMM_LNK_MAX_DATA
#define COMM_RADIO_POOLED_PACKETS   4
#define COMM_RADIO_LBT_MS           5

void COMRAD_init(void);
int COMRAD_send(const u8_t *data, u16_t len, bool ack);
int COMRAD_reply(const u8_t *data, u16_t len);
u8_t COMRAD_stats(void);
void COMRAD_report_paired(bool paired);

#endif /* COMM_RADIO_H_ */
