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
#ifdef CONFIG_I2C
#include "stmpe811_handler.h"
#endif

#define SPYBOT_VERSION        0x00010000

// controller tell rover about updated configuration
#define APP_CTRL_REMOTE_REQ_SET_CONFIG    (1<<0)
// controller tell rover to store (updated) configuration
#define APP_CTRL_REMOTE_REQ_STORE_CONFIG  (1<<1)
// controller tell rover to load last stored configuration
#define APP_CTRL_REMOTE_REQ_LOAD_CONFIG   (1<<2)
// controller tell rover to send back current configuration
#define APP_CTRL_REMOTE_REQ_GET_CONFIG    (1<<3)

// controller wants radar data
#define APP_ROVER_REMOTE_REQ_RADAR_REPORT (1<<16)

// indicates that current remote requests are urgent and should be
// sent as soon as possible
#define APP_REMOTE_REQ_URGENT             (1<<31)

#define PAIRING_STAGE_ONE           0
#define PAIRING_STAGE_TWO           1
#define PAIRING_OK                  2

#ifdef CONFIG_SPYBOT_ROVER
#define BEACON_RECURRENCE           109
#define COMM_RECURRENCE             113
#define COMM_OTHER_RECURRENCE       97
#define COMM_MAX_TX_ERR             8
#else
#define BEACON_RECURRENCE           401
#define COMM_RECURRENCE             97
#define COMM_OTHER_RECURRENCE       113
#define COMM_MAX_TX_ERR             15
#endif

// radar not scanning
#define RADAR_CTRL_STILL            0
// radar scanning with normal speed
#define RADAR_CTRL_NORMAL           3

struct app_impl_s;

// common stuff for both rover and controller
typedef struct {
  u16_t tx_seqno;
  u8_t tx_cmd;
  bool comrad_busy;
  u8_t err_count;
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
  s8_t temp;
  u8_t batt;
  bool light_ir;
  bool light_white;
  bool beep;

} app_remote;

// app implementation functions
typedef struct app_impl_s {
  // specific app setup
  void (*app_impl_setup)(app_common *common, app_remote *remote, configuration_t *app_conf);
  // how to handle received msgs
  void (*app_impl_handle_rx)(comm_arg *rx, u16_t len, u8_t *data, bool already_received);
  // how to handle acks
  void (*app_impl_handle_ack)(u8_t cmd, comm_arg *rx, u16_t len, u8_t *data);
  // app ticker
  void (*app_impl_tick)(void);
  // actions to take when getting/losing pairing
  void (*app_impl_set_paired)(bool paired);
  // indicates that a request to send msg to other side is executed and finished
  void (*app_impl_clr_remote_req)(u32_t req);
  // request to send msg to other side (where req can be any of APP_CTRL_REMOTE_REQ_*)
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

// initializes application
void APP_init(void);
// sets application implementation (as rover or controller)
void APP_impl_set(app_impl *impl);
// transmits a message to other side
int APP_tx(u8_t *data, u16_t len);
// transmits a debug message to other side
void APP_tx_dbg(const char *s);
// returns pair status
u8_t APP_pair_status(void);
// sets whether paired or not
void APP_set_paired_state(bool paired);

void APP_handle_unknown_msg(comm_arg *rx, u16_t len, u8_t *data, bool already_received);

// called from comm_radio stack on msg
void APP_comrad_rx(comm_arg *rx, u16_t len, u8_t *data, bool already_received);
// called from comm_radio stack on ack
void APP_comrad_ack(comm_arg *rx, u16_t seq_no, u16_t len, u8_t *data);
// called from comm_radio stack on err
void APP_comrad_err(u16_t seq_no, int err);

bool APP_get_joystick_control(void);
void APP_set_joystick_control(bool c);
void APP_get_joystick_reading(s8_t *hori, s8_t *vert);

// returns remotely shared acceleration vector
s8_t *APP_remote_get_acc(void);
// returns remotely shared heading
u8_t APP_remote_get_heading(void);
// sets remotely shared radar speed
void APP_remote_set_radar_ctrl(s8_t radar);
// sets remotely shared motor control vector
void APP_remote_set_motor_ctrl(s8_t horizontal, s8_t vertical);
// sets remotely shared camera control  vector
void APP_remote_set_camera_ctrl(s8_t horizontal, s8_t vertical);
// returns remotely shared state
const app_remote *APP_remote_get(void);
#ifdef CONFIG_SPYBOT_APP_MASTER
void APP_remote_load_config(void);
void APP_remote_store_config(void);
void APP_remote_update_config(spybot_cfg cfg, s16_t val, bool urgent);
#endif

// returns current volatile static local configuration
const configuration_t *APP_cfg_get(void);
// sets a value in current volatile static local configuration
void APP_cfg_set(spybot_cfg c, s16_t val);
// returns a value from current volatile static local configuration
s16_t APP_cfg_get_val(spybot_cfg c);

void APP_measure_batt_poll(void);

u32_t APP_get_last_batt(void);

void APP_force_batt_reading(void);


#ifdef CONFIG_SPYBOT_LSM
void APP_get_acc_extremes(s16 x[3][2], bool reset);
void APP_get_mag_extremes(s16 x[3][2], bool reset);
#endif


#ifdef CONFIG_SPYBOT_HCSR
void APP_report_radar_value(u8_t angle, s8_t value);
#endif

s8_t *APP_get_radar_values(void);

#endif /* APP_H_ */
