/*
 * servo.c
 *
 *  Created on: Jan 15, 2014
 *      Author: petera
 */

#include "servo.h"
#include "gpio.h"
#include "app.h"

typedef struct {
  gpio_port port;
  gpio_pin pin;
  u8_t oc_channel;
} servo_pinmap;

const static servo_pinmap hw_map[_SERVO_CNT] = {
    {.port = PORTB, .pin = PIN8, .oc_channel = 3}, // pan
    {.port = PORTB, .pin = PIN9, .oc_channel = 4}, // tilt
    {.port = PORTB, .pin = PIN7, .oc_channel = 2}, // radar
};

static s16_t servo_cur_pwm[_SERVO_CNT];
static s16_t servo_set_pwm[_SERVO_CNT];
static s16_t radar_pos = 0;
static s16_t radar_speed = 512;
static s16_t radar_dir = 1;

#define SERVO_TIM   TIM4

#define SERVO_MIN   (2*65536/(2*20))    // 1 ms
#define SERVO_MID   (3*65536/(2*20))    // 1.5 ms
#define SERVO_MAX   (4*65536/(2*20))    // 2 ms

static TIM_OCInitTypeDef oc_conf;

static void servo_set_channel_duty(s16_t duty, u8_t channel) {
  u32_t short_percent = duty + 32768;
  u32_t pulse = ((SERVO_MAX - SERVO_MIN) * short_percent) / 65536 + SERVO_MIN;
  oc_conf.TIM_Pulse = pulse -1;
  switch(channel) {
  case 1:
    TIM_OC1Init(SERVO_TIM, &oc_conf);
    TIM_OC1PreloadConfig(SERVO_TIM, TIM_OCPreload_Enable);
    break;
  case 2:
    TIM_OC2Init(SERVO_TIM, &oc_conf);
    TIM_OC2PreloadConfig(SERVO_TIM, TIM_OCPreload_Enable);
    break;
  case 3:
    TIM_OC3Init(SERVO_TIM, &oc_conf);
    TIM_OC3PreloadConfig(SERVO_TIM, TIM_OCPreload_Enable);
    break;
  case 4:
    TIM_OC4Init(SERVO_TIM, &oc_conf);
    TIM_OC4PreloadConfig(SERVO_TIM, TIM_OCPreload_Enable);
    break;
  default:
    ASSERT(FALSE);
    break;
  }
}

static void servo_set(servo_out servo, s16_t pwm) {
  ASSERT(servo < _SERVO_CNT);
  servo_cur_pwm[servo] = pwm;
  servo_set_channel_duty(pwm, hw_map[servo].oc_channel);
}

static s16_t servo_get_adjust(servo_out servo) {
  s16_t adj = 0;
  switch (servo) {
    case SERVO_CAM_PAN:
      adj = APP_cfg_get_val(CFG_CAM_PAN_ADJUST);
      break;
    case SERVO_CAM_TILT:
      adj = APP_cfg_get_val(CFG_CAM_TILT_ADJUST);
      break;
    case SERVO_RADAR:
      adj = APP_cfg_get_val(CFG_RADAR_ADJUST);
      break;
    default:
      break;
  }
  return adj;
}

void SERVO_control(servo_out servo, s8_t value) {
  ASSERT(servo < _SERVO_CNT);
  s32_t pwm_val;
  s16_t adj = servo_get_adjust(servo);

  pwm_val = value << 8;
  pwm_val = 6*pwm_val / 7;
  pwm_val += 1*(adj<<8) / 7;

  if (servo == SERVO_CAM_PAN && (APP_cfg_get_val(CFG_CONTROL) & CFG_CONTROL_PAN_INVERT)) {
    pwm_val = -pwm_val;
  } else if (servo == SERVO_CAM_TILT && (APP_cfg_get_val(CFG_CONTROL) & CFG_CONTROL_TILT_INVERT)) {
    pwm_val = -pwm_val;
  }

  servo_set_pwm[servo] = pwm_val;
}

void SERVO_control_radar(s8_t value) {
  if (value == RADAR_CTRL_STILL) {
    radar_speed = 0;
    radar_pos = 0;
  } else {
    radar_speed = value << 7;
  }
}

void SERVO_update(void) {
#define MAX_D   512
  int i;

  if (radar_dir < 0) {
    if (radar_pos <= S16_MIN + radar_speed) {
      radar_pos = S16_MIN;
      radar_dir = 1;
    }
  } else if (radar_dir > 0) {
    if (radar_pos >= S16_MAX - radar_speed) {
      radar_pos = S16_MAX;
      radar_dir = -1;
    }
  }
  radar_pos += radar_dir * radar_speed;
  SERVO_control(SERVO_RADAR, radar_pos >> 8);

  for (i = 0; i < _SERVO_CNT; i++) {
    s16_t d = servo_set_pwm[i] - servo_cur_pwm[i];
    if (ABS(d) > MAX_D * 20) {
      servo_cur_pwm[i] = servo_set_pwm[0];
      servo_set(i, servo_cur_pwm[i]);
    } else {
      if (d < 0) {
        if (d <= -MAX_D) {
          servo_set(i, servo_cur_pwm[i] - MAX_D);
        } else if (d > -MAX_D) {
          servo_set(i, servo_cur_pwm[i] - 1);
        }
      } else if (d > 0) {
        if (d >= MAX_D) {
          servo_set(i, servo_cur_pwm[i] + MAX_D);
        } else if (d < MAX_D) {
          servo_set(i, servo_cur_pwm[i] + 1);
        }
      }
    }
  }
}

void SERVO_get_radar_position(s16_t *pos, s16 *dir) {
  if (pos) *pos = radar_pos;
  if (dir) *dir= radar_dir;
}

void SERVO_init(void) {
  int i;

  // 0xffff => 20ms = 1/50
  // 1/(50*65535) = 1/(72000000/x) => x = 72000000/(50*65535) => 22
  // 72000000/22 = 3272727
  // 3272727 / 65535 = 49.94
  // 3277 ~= 1ms
  // 4915 ~= 1.5ms
  // 6554 ~= 2ms

  const int prescaler = SYS_CPU_FREQ / (50*65536);

  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Period = 0xffff;
  TIM_TimeBaseStructure.TIM_Prescaler = prescaler-1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(SERVO_TIM, &TIM_TimeBaseStructure);

  oc_conf.TIM_OCMode = TIM_OCMode_PWM1;
  oc_conf.TIM_OutputState = TIM_OutputState_Enable;
  oc_conf.TIM_OCPolarity = TIM_OCPolarity_High;

  for (i = 0; i < _SERVO_CNT; i++) {
    servo_cur_pwm[i] = 0;
    servo_set_pwm[i] = 0;
    servo_set_channel_duty(0, hw_map[i].oc_channel);
  }

  radar_pos = 0;
  radar_dir = 1;
  radar_speed = 512;

  TIM_ARRPreloadConfig(TIM4, ENABLE);

  TIM_Cmd(TIM4, ENABLE);
  TIM_CtrlPWMOutputs(TIM4, ENABLE);

  for (i = 0; i < _SERVO_CNT; i++) {
    gpio_config(hw_map[i].port, hw_map[i].pin, CLK_50MHZ, AF, AF0, PUSHPULL, NOPULL);
  }
}


