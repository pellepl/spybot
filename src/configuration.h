/*
 * configuration.h
 *
 *  Created on: Jan 11, 2014
 *      Author: petera
 */

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include "system.h"

#define CONFIGURATION_VERSION     0x00020001

#define CFG_CONTROL_LEFT_INVERT    (1<<0)
#define CFG_CONTROL_RIGHT_INVERT   (1<<1)
#define CFG_CONTROL_PAN_INVERT     (1<<2)
#define CFG_CONTROL_TILT_INVERT    (1<<3)
#define CFG_CONTROL_JOY_H_INVERT   (1<<4)
#define CFG_CONTROL_JOY_V_INVERT   (1<<5)

typedef enum {
  CFG_STEER_ADJUST = 0,
  CFG_RADAR_ADJUST,
  CFG_CAM_PAN_ADJUST,
  CFG_CAM_TILT_ADJUST,
  CFG_CONTROL,

  CFG_RADIO_CHANNEL,
  CFG_RADIO_PA,

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

  _CFG_END,

  CFG_STOP = 0xff,
} spybot_cfg;

typedef struct __attribute__ (( packed )) {
  s8_t steer_adjust;
  s8_t radar_adjust;
  s8_t cam_pan_adjust;
  s8_t cam_tilt_adjust;
  u8_t control;
} configuration_main_t;

#define CFG_RADIO_PA_SCHEME_STATIC_10     0
#define CFG_RADIO_PA_SCHEME_STATIC_6      1
#define CFG_RADIO_PA_SCHEME_STATIC_m2     2
#define CFG_RADIO_PA_SCHEME_STATIC_m10    3
#define CFG_RADIO_PA_SCHEME_DYNAMIC_FAST  4
#define CFG_RADIO_PA_SCHEME_DYNAMIC_SLOW  5

typedef struct __attribute__ (( packed )) {
  u8_t radio_channel;
  u8_t pa_scheme;
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
