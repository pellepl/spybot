/*
 * servo.c
 *
 *  Created on: Jan 15, 2014
 *      Author: petera
 */

#include "servo.h"
#include "gpio.h"

typedef struct {
  gpio_port port;
  gpio_pin pin;
  u8_t oc_channel;
} servo_pinmap;

const static servo_pinmap hw_map[_SERVO_CNT] = {
    {.port = PORTB, .pin = PIN9, .oc_channel = 4}, // pan
    {.port = PORTB, .pin = PIN8, .oc_channel = 3}, // tilt
    {.port = PORTB, .pin = PIN7, .oc_channel = 2}, // radar
};

s16_t servo_cur_pwm[_SERVO_CNT];
s16_t servo_set_pwm[_SERVO_CNT];

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

  TIM_ARRPreloadConfig(TIM4, ENABLE);

  TIM_Cmd(TIM4, ENABLE);
  TIM_CtrlPWMOutputs(TIM4, ENABLE);

  for (i = 0; i < _SERVO_CNT; i++) {
    gpio_config(hw_map[i].port, hw_map[i].pin, CLK_50MHZ, AF, AF0, PUSHPULL, NOPULL);
  }
}

void SERVO_set(servo_out servo, s16_t pwm) {
  ASSERT(servo < _SERVO_CNT);
  servo_cur_pwm[servo] = pwm;
  servo_set_channel_duty(pwm, hw_map[servo].oc_channel);
}

void SERVO_control(servo_out servo, s8_t value) {
  ASSERT(servo < _SERVO_CNT);
  servo_set_pwm[servo] = value << 8;
}

void SERVO_update(void) {
#define MAX_D   512
  int i;
  for (i = 0; i < _SERVO_CNT; i++) {
    s16_t d = servo_set_pwm[i] - servo_cur_pwm[i];
    if (ABS(d) > MAX_D * 20) {
      servo_cur_pwm[i] = servo_set_pwm[0];
    } else {
      if (d < 0) {
        if (d <= -MAX_D) {
          SERVO_set(i, servo_cur_pwm[i] - MAX_D);
        } else if (d > -MAX_D) {
          SERVO_set(i, servo_cur_pwm[i] - 1);
        }
      } else if (d > 0) {
        if (d >= MAX_D) {
          SERVO_set(i, servo_cur_pwm[i] + MAX_D);
        } else if (d < MAX_D) {
          SERVO_set(i, servo_cur_pwm[i] + 1);
        }
      }
    }
  }

}
