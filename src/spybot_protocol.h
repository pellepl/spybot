/*
 * spybot_protocol.h
 *
 *  Created on: Dec 19, 2013
 *      Author: petera
 */

#ifndef SPYBOT_PROTOCOL_H_
#define SPYBOT_PROTOCOL_H_

typedef enum {
  // [rover <-> ctrl]
  // arg: null term string
  // ack: [0x00=deny, 0x01=accept]
  CMD_DBG = 0,

  // [rover <-- ctrl]
  // arg: none
  // ack: [0x00=deny, 0x01=accept]
  CMD_PAIRING_BEACON,

  // [rover --> ctrl]
  // arg: none
  // ack: [0x00=deny, 0x01=accept]
  CMD_PAIRING_ECHO,

  // [rover <-- ctrl]
  // arg: s8 left, s8 right, u8_t action_mask, u8 status_mask
  //      u8 action_mask: bit 0 - lights
  //                      bit 1 - beep
  //      u8 status_mask: bit 0 - acc
  //                      bit 1 - heading
  //                      bit 2 - temp
  //                      bit 3 - batt
  //                      bit 7 - req_radar
  // ack: [0x00=deny, 0x01=accept]
  //      acc:     s8 acc_x, s8 acc_y, s8 acc_z
  //      heading: s8 heading
  //      temp:    s8 temperature
  //      batt:    u8 battery
  CMD_CONTROL,

  // [rover --> ctrl]
  // arg: s8 angle_start, s8 len, [s8 value]*len
  //      rest of arg is filled with status_mask results from requesting CMD_CONTROL
  // ack: [0x00=deny, 0x01=accept]
  CMD_RADAR_REPORT,

  // [rover --> ctrl]
  // arg: none
  // ack: [0x00=deny, 0x01=accept]
  CMD_ALERT,

  // [rover <-- ctrl]
  // arg: [u8 cfg, s16 val]* until CFG_STOP
  // ack: [0x00=deny, 0x01=accept]
  //      s8 result
  CMD_SET_CONFIG,

  // [rover <-- ctrl]
  // arg: none
  // ack: [0x00=deny, 0x01=accept]
  //      s8 result
  CMD_STORE_CONFIG,

  // [rover <-- ctrl]
  // arg: none
  // ack: [0x00=deny, 0x01=accept]
  CMD_LOAD_CONFIG,

  // [rover <-- ctrl]
  // arg: u8 reset [1 = reset mag extremes, 0 = keep mag extremes]
  // ack: [0x00=deny, 0x01=accept]
  //      s16 lsm_mag_x_min, s16 lsm_mag_x_max,
  //      s16 lsm_mag_y_min, s16 lsm_mag_y_max,
  //      s16 lsm_mag_z_min, s16 lsm_mag_z_max
  CMD_LSM_MAG_EXTREMES,

  // [rover <-- ctrl]
  // arg: u8 reset [1 = reset acc extremes, 0 = keep acc extremes]
  // ack: [0x00=deny, 0x01=accept]
  //      s16 lsm_acc_x_min, s16 lsm_acc_x_max,
  //      s16 lsm_acc_y_min, s16 lsm_acc_y_max,
  //      s16 lsm_acc_z_min, s16 lsm_acc_z_max
  CMD_LSM_ACC_EXTREMES,

  // [rover --> ctrl]
  // arg: u8 channel
  // ack: [0x00=deny, 0x01=accept]
  //      [0x00=deny channel change, 0x01=accept channel change]
  CMD_CHANNEL_CHANGE,

} spybot_cmd;

typedef enum {
  CFG_STOP = 0,
  CFG_STEER_ADJUST,
  CFG_RADAR_ADJUST,
  CFG_CAM_PAN_ADJUST,
  CFG_CAM_TILT_ADJUST,
  CFG_SPEED_ADJUST,
  CFG_RADIO_CHANNEL,
  CFG_LSM_MAG_X_MIN,
  CFG_LSM_MAG_X_MAX,
  CFG_LSM_MAG_Y_MIN,
  CFG_LSM_MAG_Y_MAX,
  CFG_LSM_MAG_Z_MIN,
  CFG_LSM_MAG_Z_MAX,
  CFG_LSM_ACC_X_MIN,
  CFG_LSM_ACC_X_MAX,
  CFG_LSM_ACC_Y_MIN,
  CFG_LSM_ACC_Y_MAX,
  CFG_LSM_ACC_Z_MIN,
  CFG_LSM_ACC_Z_MAX,
} spybot_cfg;


#endif /* SPYBOT_PROTOCOL_H_ */
