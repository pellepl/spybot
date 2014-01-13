/*
 * configuration.h
 *
 *  Created on: Jan 11, 2014
 *      Author: petera
 */

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include "system.h"


#define CFG_COMMON_LEFT_INVERT    (1<<0)
#define CFG_COMMON_RIGHT_INVERT   (1<<1)
#define CFG_COMMON_PAN_INVERT     (1<<2)
#define CFG_COMMON_TILT_INVERT    (1<<3)

typedef enum {
  CFG_STEER_ADJUST = 0,
  CFG_RADAR_ADJUST,
  CFG_CAM_PAN_ADJUST,
  CFG_CAM_TILT_ADJUST,
  CFG_COMMON,

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

  CFG_STOP = 0xff,
} spybot_cfg;

typedef struct __attribute__ (( packed )) {
  s8_t steer_adjust;
  s8_t radar_adjust;
  s8_t cam_pan_adjust;
  s8_t cam_tilt_adjust;
  u8_t common;
} configuration_main_t;

typedef struct __attribute__ (( packed )) {
  u8_t radio_channel;
} configuration_radio_t;

typedef struct __attribute__ (( packed )) {
  s16_t mag_x_min;
  s16_t mag_x_max;
  s16_t mag_y_min;
  s16_t mag_y_max;
  s16_t mag_z_min;
  s16_t mag_z_max;
} configuration_mag_t;

typedef struct __attribute__ (( packed )) {
  s16_t acc_x_min;
  s16_t acc_x_max;
  s16_t acc_y_min;
  s16_t acc_y_max;
  s16_t acc_z_min;
  s16_t acc_z_max;
} configuration_acc_t;

typedef struct __attribute__ (( packed )) {
  configuration_main_t main;
  configuration_radio_t radio;
  configuration_mag_t magneto;
  configuration_acc_t accel;

} configuration_t;

#endif /* CONFIGURATION_H_ */