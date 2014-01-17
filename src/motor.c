/*
 * motor.c
 *
 *  Created on: Jan 10, 2014
 *      Author: petera
 */
/*
PB12               GPIO_OUT          MOTOR_AIN1
PB13               GPIO_OUT          MOTOR_AIN2
PB14               GPIO_OUT          MOTOR_BIN1
PB15               GPIO_OUT          MOTOR_BIN2
PA8                TIM1_CH1          MOTOR_PWMA
PA9                TIM1_CH2          MOTOR_PWMB

IN1 IN2 PWM OUT1 OUT2 Mode
H   H   -    L   L     short brake
L   H   H    L   H     CCW
        L    L   L     short brake
H   L   H    H   L     CW
        L    L   L     short brake
L   L   H    hz  hz    stop



L/R   LEFT          MID              RIGHT

UP     5/ 9   7/ 9   9/ 9     9/ 7    9/ 5

      -2/ 9   1/ 7   5/ 5     7/ 1    9/-2

MID   -9/ 9  -5/ 5   0/ 0     5/-5    9/-9

      -7/ 0  -5/-1  -5/-5    -1/-5    0/-7

DOWN  -5/-9  -7/-9  -9/-9    -9/-7   -9/-5



*/

#include "motor.h"
#include "gpio.h"
#include "app.h"
#include "configuration.h"

#define MOTORA_PORT     PORTB
#define MOTORB_PORT     PORTB
#define MOTORAIN1_PIN   PIN12
#define MOTORAIN2_PIN   PIN13
#define MOTORBIN1_PIN   PIN14
#define MOTORBIN2_PIN   PIN15
#define MOTOR_PWM_TIM   TIM1
#define set_motora_pwm_duty_cycle(x) MOTOR_PWM_TIM->CCR1 = (x)
#define set_motorb_pwm_duty_cycle(x) MOTOR_PWM_TIM->CCR2 = (x)

#define MOTOR_PERIOD    (SYS_CPU_FREQ / CONFIG_MOTOR_PWM_FREQ)

#define MOTOR_DELTA     25

const static s8_t ctrl_map[3][3][2] = {
    {
        {  63, 127}, { 127, 127}, { 127,  63},
    },
    {
        {-64, 63}, {   0,   0}, { 63,-64},
    },
    {
        { -64,-128}, {-128,-128}, {-128, -64},
    }
};

static struct {
  s8_t current[2];
  s8_t target[2];
} motor;

static void motor_stop(void) {
  memset(&motor, 0, sizeof(motor));
  gpio_disable(MOTORA_PORT, MOTORAIN1_PIN);
  gpio_disable(MOTORA_PORT, MOTORAIN2_PIN);
  gpio_disable(MOTORB_PORT, MOTORBIN1_PIN);
  gpio_disable(MOTORB_PORT, MOTORBIN2_PIN);
  set_motora_pwm_duty_cycle(MOTOR_PERIOD);
  set_motorb_pwm_duty_cycle(MOTOR_PERIOD);
}

static void motor_interpolate(
    const s8_t o00[2], const s8_t o01[2],
    const s8_t o10[2], const s8_t o11[2],
    u8_t x, u8_t y, s8_t res[2]) {
/*
 *   (1 / (128-0)*(128-0)) * (
 *    o00[Z] * (128-x)*(128-y) +
 *    o10[Z] * (x - 0)*(128-y) +
 *    o01[Z] * (128-x)*(y - 0) +
 *    o11[Z] * (x - 0)*(y - 0)
 *   )
 */
  s32_t o00f = (127-x) * (127-y);
  s32_t o10f = (x - 0) * (127-y);
  s32_t o01f = (127-x) * (y - 0);
  s32_t o11f = (x - 0) * (y - 0);

  s32_t res0 = o00[0] * o00f + o10[0] * o10f + o01[0] * o01f + o11[0] * o11f;
  s32_t res1 = o00[1] * o00f + o10[1] * o10f + o01[1] * o01f + o11[1] * o11f;
  res[0] = res0 / (128*128);
  res[1] = res1 / (128*128);
}

static void motor_translate(s8_t h, s8_t v, s8_t motor[2]) {
  const s8_t *o00, *o01, *o10, *o11;
  if (h < 0) {
    if (v < 0) {
      o00 = ctrl_map[0][0]; o01 = ctrl_map[0][1];
      o10 = ctrl_map[1][0]; o11 = ctrl_map[1][1];
      v = (128+v);
    } else {
      o00 = ctrl_map[1][0]; o01 = ctrl_map[1][1];
      o10 = ctrl_map[2][0]; o11 = ctrl_map[2][1];
    }
    h = (128+h);
  } else {
    if (v < 0) {
      o00 = ctrl_map[0][1]; o01 = ctrl_map[0][2];
      o10 = ctrl_map[1][1]; o11 = ctrl_map[1][2];
      v = (128+v);
    } else {
      o00 = ctrl_map[1][1]; o01 = ctrl_map[1][2];
      o10 = ctrl_map[2][1]; o11 = ctrl_map[2][2];
    }
  }
  motor_interpolate(o00, o10, o01, o11, h, v, motor);
}

static s8_t motor_lin_squash(s8_t v, u8_t threshold) {
  if ((v < 0 && v > -threshold) || (v >= 0 && v < threshold)) {
    return 0;
  }
  u32_t vp = ABS(v);
  vp = ((vp - threshold) * 128) / (128-threshold);
  return v < 0 ? -vp : vp;
}

static void motor_set(s8_t left, s8_t right) {
  if (left == 0) {
    gpio_disable(MOTORA_PORT, MOTORAIN1_PIN);
    gpio_disable(MOTORA_PORT, MOTORAIN2_PIN);
    set_motora_pwm_duty_cycle(MOTOR_PERIOD);
  } else {
    if (left > 0) {
      gpio_enable(MOTORA_PORT, MOTORAIN1_PIN);
      gpio_disable(MOTORA_PORT, MOTORAIN2_PIN);
    } else {
      gpio_disable(MOTORA_PORT, MOTORAIN1_PIN);
      gpio_enable(MOTORA_PORT, MOTORAIN2_PIN);
      left = -left;
    }
    set_motora_pwm_duty_cycle((MOTOR_PERIOD*left)/128);
  }
  if (right == 0) {
    gpio_disable(MOTORB_PORT, MOTORBIN1_PIN);
    gpio_disable(MOTORB_PORT, MOTORBIN2_PIN);
    set_motorb_pwm_duty_cycle(MOTOR_PERIOD);
  } else {
    if (right > 0) {
      gpio_enable(MOTORB_PORT, MOTORBIN1_PIN);
      gpio_disable(MOTORB_PORT, MOTORBIN2_PIN);
    } else {
      gpio_disable(MOTORB_PORT, MOTORBIN1_PIN);
      gpio_enable(MOTORB_PORT, MOTORBIN2_PIN);
      right = -right;
    }
    set_motorb_pwm_duty_cycle((MOTOR_PERIOD*right)/128);
  }
}



void MOTOR_init(void) {
  motor_stop();
  memset(&motor, 0, sizeof(motor));
}

void MOTOR_go(s8_t x) {
  if (x == 0) {
    motor_stop();
    return;
  } else if (x > 0) {
    gpio_enable(MOTORA_PORT, MOTORAIN1_PIN);
    gpio_disable(MOTORA_PORT, MOTORAIN2_PIN);
    gpio_enable(MOTORB_PORT, MOTORBIN1_PIN);
    gpio_disable(MOTORB_PORT, MOTORBIN2_PIN);
  } else if (x < 0) {
    gpio_disable(MOTORA_PORT, MOTORAIN1_PIN);
    gpio_enable(MOTORA_PORT, MOTORAIN2_PIN);
    gpio_disable(MOTORB_PORT, MOTORBIN1_PIN);
    gpio_enable(MOTORB_PORT, MOTORBIN2_PIN);
    x = -x;
  }
  set_motora_pwm_duty_cycle((MOTOR_PERIOD*x)/128);
  set_motorb_pwm_duty_cycle((MOTOR_PERIOD*x)/128);
}

void MOTOR_control(s8_t hori, s8_t veri) {
  s8_t motor_ctrl[2];
  hori = motor_lin_squash(hori, 10);
  veri = motor_lin_squash(veri, 10);
  motor_translate(hori, veri, motor_ctrl);

  s16_t adj = APP_cfg_get_val(CFG_STEER_ADJUST);
  s8_t left = motor_ctrl[0];
  s8_t right = motor_ctrl[1];

  if (adj < 0) {
    adj = -adj;
    adj >>= 2;
    adj = 128-adj;
    right = (s8_t)((s32_t)(adj*right)/128);
  } else if (adj > 0) {
    adj >>= 2;
    adj = 128-adj;
    left = (s8_t)((s32_t)(adj*right)/128);
  }

  if (APP_cfg_get_val(CFG_COMMON) & CFG_COMMON_LEFT_INVERT) {
    left = -left;
  }
  if (APP_cfg_get_val(CFG_COMMON) & CFG_COMMON_RIGHT_INVERT) {
    right = -right;
  }

  motor.target[0] = left;
  motor.target[1] = right;

}

void MOTOR_update(void) {
  s16_t motor_d[2];
  motor_d[0] = motor.target[0] - motor.current[0];
  motor_d[1] = motor.target[1] - motor.current[1];
  int i;
  for (i = 0; i < 2; i++) {
    if (motor_d[i] == 0) {
      continue;
    } else if (ABS(motor_d[i]) < MOTOR_DELTA) {
      motor.current[i] = motor.target[i];
    } else {
      if (motor_d[i] < 0) {
        motor.current[i] -= MOTOR_DELTA;
      } else {
        motor.current[i] += MOTOR_DELTA;
      }
    }
  }

  motor_set(motor.current[0], motor.current[1]);
}

