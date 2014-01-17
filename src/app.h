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
#include "spybot_protocol.h"

#define SPYBOT_VERSION        0x00010000

#define APP_CTRL_REMOTE_REQ_SET_CONFIG    (1<<0)
#define APP_CTRL_REMOTE_REQ_STORE_CONFIG  (1<<1)
#define APP_CTRL_REMOTE_REQ_LOAD_CONFIG   (1<<2)
#define APP_CTRL_REMOTE_REQ_GET_CONFIG    (1<<3)

#define APP_REMOTE_REQ_URGENT             (1<<31)

#define PAIRING_STAGE_ONE           0
#define PAIRING_STAGE_TWO           1
#define PAIRING_OK                  2

#ifdef CONFIG_SPYBOT_ROVER
#define BEACON_RECURRENCE           409
#define COMM_RECURRENCE             113
#define COMM_OTHER_RECURRENCE       97
#else
#define BEACON_RECURRENCE           401
#define COMM_RECURRENCE             97
#define COMM_OTHER_RECURRENCE       113
#endif


#define RADAR_CTRL_STILL            0
#define RADAR_CTRL_NORMAL           3

struct app_impl_s;

typedef struct {
  u16_t tx_seqno;
  u8_t tx_cmd;
  bool comrad_busy;
  u8_t pair_state;
  u8_t pair_wait_cnt;
  task_timer tick_timer;
  task *tick_task;
  u32_t tick_count;
  struct app_impl_s *impl;
} app_common;


// ram updated over RF
typedef struct {
  s8_t lsm_heading;
  s8_t lsm_acc[3];
  s8_t motor_ctrl[2]; // steer vector : [0] hori, [1] vert
  s8_t pan;
  s8_t tilt;
  s8_t radar;

} app_remote;

typedef struct app_impl_s {
  void (*app_impl_setup)(app_common *common, app_remote *remote, configuration_t *app_conf);
  void (*app_impl_handle_rx)(comm_arg *rx, u16_t len, u8_t *data, bool already_received);
  void (*app_impl_handle_ack)(u8_t cmd, comm_arg *rx, u16_t len, u8_t *data);
  void (*app_impl_tick)(void);
  void (*app_impl_set_paired)(bool paired);
  void (*app_impl_clr_remote_req)(u32_t req);
  void (*app_impl_set_remote_req)(u32_t req);
} app_impl;


#ifdef CONFIG_I2C
extern task_mutex i2c_mutex;
#endif

#define APP_JOYSTICK_CONTROL_MOTOR    TRUE
#define APP_JOYSTICK_CONTROL_CAMERA   FALSE

#define REPLY_MAX_LEN (COMM_LNK_MAX_DATA-COMM_H_SIZE-1)

extern u8_t const REPLY_OK[];
extern u8_t const REPLY_DENY[];

void APP_init(void);
void APP_impl_set(app_impl *impl);
int APP_tx(u8_t *data, u16_t len);
void APP_tx_dbg(const char *s);
u8_t APP_pair_status(void);
void APP_set_paired_state(bool paired);

void APP_comrad_rx(comm_arg *rx, u16_t len, u8_t *data, bool already_received);
void APP_comrad_ack(comm_arg *rx, u16_t seq_no, u16_t len, u8_t *data);
void APP_comrad_err(u16_t seq_no, int err);

bool APP_get_joystick_control(void);
void APP_set_joystick_control(bool c);
void APP_get_joystick_reading(s8_t *hori, s8_t *vert);

s8_t *APP_remote_get_acc(void);
u8_t APP_remote_get_heading(void);
void APP_remote_set_radar_ctrl(s8_t radar);
void APP_remote_set_motor_ctrl(s8_t horizontal, s8_t vertical);
void APP_remote_set_camera_ctrl(s8_t horizontal, s8_t vertical);
#ifdef CONFIG_SPYBOT_MASTER
void APP_remote_load_config(void);
void APP_remote_store_config(void);
void APP_remote_update_config(spybot_cfg cfg, s16_t val, bool urgent);
#endif

const configuration_t *APP_cfg_get(void);
void APP_cfg_set(spybot_cfg c, s16_t val);
s16_t APP_cfg_get_val(spybot_cfg c);


#ifdef CONFIG_SPYBOT_LSM
void APP_get_acc_extremes(s16 x[3][2], bool reset);
void APP_get_mag_extremes(s16 x[3][2], bool reset);
#endif




#endif /* APP_H_ */
