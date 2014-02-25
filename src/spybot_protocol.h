/*
 * spybot_protocol.h
 *
 *  Created on: Dec 19, 2013
 *      Author: petera
 */

#ifndef SPYBOT_PROTOCOL_H_
#define SPYBOT_PROTOCOL_H_

#define SPYBOT_ACT_LIGHTS   (1<<0)
#define SPYBOT_ACT_BEEP     (1<<1)

#define SPYBOT_SR_ACC       (1<<0)
#define SPYBOT_SR_HEADING   (1<<1)
#define SPYBOT_SR_TEMP      (1<<2)
#define SPYBOT_SR_BATT      (1<<3)
#define SPYBOT_SR_RADAR     (1<<7)

#define ACK_OK                      0x01
#define ACK_DBG                     0xee
#define ACK_DENY                    0x00

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
  // arg: s8 left, s8 right, s8 pan, s8 tilt, s8 radar, u8 action_mask, u8 status_mask
  //      u8 action_mask: bit 0 - lights
  //                      bit 1 - beep
  //      u8 status_mask: bit 0 - acc
  //                      bit 1 - heading
  //                      bit 2 - temp
  //                      bit 3 - batt
  //                      bit 7 - req_radar
  // ack: [0x00=deny, 0x01=accept]
  //      sr:      u8 sent sr
  //      acc:     s8 acc_x, s8 acc_y, s8 acc_z
  //      heading: s8 heading
  //      temp:    s8 temperature
  //      batt:    u8 battery
  CMD_CONTROL,

  // [rover --> ctrl]
  // arg: s8 angle_start, s8 len, [s8 value]*len
  // ack: [0x00=deny, 0x01=accept]
  CMD_RADAR_REPORT,

  // [rover --> ctrl]
  // arg: none
  // ack: [0x00=deny, 0x01=accept]
  CMD_ALERT,

  // [rover <-- ctrl]
  // arg: [u8 cfg, s16 val]* until CFG_STOP
  // ack: [0x00=deny, 0x01=accept]
  //      s8 result [0x01=ok]
  CMD_SET_CONFIG,

  // [rover <-- ctrl]
  // arg: none
  // ack: [0x00=deny, 0x01=accept]
  //      s8 result [0x01=ok]
  CMD_STORE_CONFIG,

  // [rover <-- ctrl]
  // arg: none
  // ack: [0x00=deny, 0x01=accept]
  CMD_LOAD_CONFIG,

  // [rover <-- ctrl]
  // arg: none
  // ack: [0x00=deny, 0x01=accept]
  //      [0x00=nocfg, 0x01=ok]
  //      s8 steer adjust
  //      s8 radar adjust
  //      s8 cam_pan adjust
  //      s8 cam_tilt adjust
  //      u8 common config
  CMD_GET_CONFIG,

  // [rover --> ctrl]
  // arg: u8 channel [0xff == no change]
  // arg: u8 pa power scheme [0xff == no change]
  // ack: [0x00=deny, 0x01=accept]
  //      [0x00=deny radio config, 0x01=accept radio config]
  CMD_SET_RADIO_CONFIG,

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


} spybot_cmd;


#endif /* SPYBOT_PROTOCOL_H_ */
