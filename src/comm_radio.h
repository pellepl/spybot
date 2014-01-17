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
#define COMRAD_RADIO_ADJUST_TICK_D  1000
#define COMRAD_RADIO_ADJUST_TICKS   5

#define COMM_RADIO_LQUAL_PERFECT    250
#define COMM_RADIO_LQUAL_GOOD       210
#define COMM_RADIO_LQUAL_WEAK       160

void COMRAD_init(void);
int COMRAD_send(const u8_t *data, u16_t len, bool ack);
int COMRAD_reply(const u8_t *data, u16_t len);
void COMRAD_report_paired(bool paired);
u8_t COMRAD_get_link_qual(void);
void COMRAD_dbg_pkt_drop_rate(u8_t percentage);

#endif /* COMM_RADIO_H_ */
