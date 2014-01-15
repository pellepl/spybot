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
#include "configuration.h"

#define SPYBOT_VERSION        0x00010000

#ifdef CONFIG_I2C
extern task_mutex i2c_mutex;
#endif

void APP_init(void);
void APP_tx_dbg(const char *s);
u8_t APP_pair_status(void);

void APP_comrad_rx(comm_arg *rx, u16_t len, u8_t *data, bool already_received);
void APP_comrad_ack(comm_arg *rx, u16_t seq_no, u16_t len, u8_t *data);
void APP_comrad_err(u16_t seq_no, int err);

#ifdef CONFIG_SPYBOT_LSM
void APP_get_acc_extremes(s16 x[3][2], bool reset);
void APP_get_mag_extremes(s16 x[3][2], bool reset);
#endif

s8_t *APP_remote_get_acc(void);
u8_t APP_remote_get_heading(void);

void APP_remote_set_motor_ctrl(s8_t horizontal, s8_t vertical);
void APP_remote_load_config(void);

const configuration_t *APP_cfg_get(void);
void APP_cfg_set(spybot_cfg c, s16_t val);


#endif /* APP_H_ */
