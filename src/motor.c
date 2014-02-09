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

#define MOTOR_CONTROL_VECTOR_SQUASH       10

#define MOTOR_PWM_CYCLE_STRETCH           20

const static s16_t ctrl_map[3][3][2] = {
    {
        { -64,-128}, {-128,-128}, {-128, -64},
    },
    {
        { -64,  64}, {   0,   0}, {  64, -64},
    },
    {
        {  64, 128}, { 128, 128}, { 128,  64},
    }
};

static struct {
  s16_t actual[2];
  s16_t desired[2];
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
    const s16_t o00[2], const s16_t o01[2],
    const s16_t o10[2], const s16_t o11[2],
    u16_t x, u16_t y, s16_t res[2]) {
/*
 *   (1 / (128-0)*(128-0)) * (
 *    o00[Z] * (128-x)*(128-y) +
 *    o10[Z] * (x - 0)*(128-y) +
 *    o01[Z] * (128-x)*(y - 0) +
 *    o11[Z] * (x - 0)*(y - 0)
 *   )
 */
  s32_t o00f = (128-x) * (128-y);
  s32_t o10f = (x - 0) * (128-y);
  s32_t o01f = (128-x) * (y - 0);
  s32_t o11f = (x - 0) * (y - 0);

  s32_t res0 = o00[0] * o00f + o10[0] * o10f + o01[0] * o01f + o11[0] * o11f;
  s32_t res1 = o00[1] * o00f + o10[1] * o10f + o01[1] * o01f + o11[1] * o11f;
  res[0] = res0 / (128*128);
  res[1] = res1 / (128*128);
}

static void motor_translate(s16_t h, s16_t v, s16_t motor[2]) {
  const s16_t *o00, *o01, *o10, *o11;
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

/*                1
 *                |        -
 *                |       -
 *                |      -
 *   -1  ---======0======--- +1
 *         -      |
 *        -       |
 *       -        |
 *                -1
 *          |---thres---|
 */
static s16_t motor_lin_squash(s16_t v, u16_t threshold) {
  if ((v < 0 && v > -threshold) || (v >= 0 && v < threshold)) {
    return 0;
  }
  u32_t vp = ABS(v);
  vp = ((vp - threshold) * 128) / (128-threshold);
  return v < 0 ? -vp : vp;
}

/*                1
 *                |     ----
 *                |-----        -
 *                |             |
 *   -1  ---------0--------- +1 thres
 *                |             |
 *           -----|             -
 *       ----     |
 *                -1
 *
 */
static s16_t motor_lin_stretch(s16_t v, u16_t threshold) {
  if (v == 0) return 0;
  u32_t vp = ABS(v);
  vp = ((vp * (128-threshold)) / 128) + threshold;
  return v < 0 ? -vp : vp;
}

static void motor_set(s16_t left, s16_t right) {
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

//#define DBG_MOTOR

#ifdef DBG_MOTOR
static bool upd_report;
#endif

void MOTOR_control(s16_t ohori, s16_t overi) {
  s16_t motor_ctrl[2];

  if (APP_cfg_get_val(CFG_COMMON) & CFG_COMMON_JOY_H_INVERT) {
    ohori = -ohori;
  }
  if (APP_cfg_get_val(CFG_COMMON) & CFG_COMMON_JOY_V_INVERT) {
    overi = -overi;
  }

  // squash control vector around origin
  s16_t hori = motor_lin_squash(ohori, MOTOR_CONTROL_VECTOR_SQUASH);
  s16_t veri = motor_lin_squash(overi, MOTOR_CONTROL_VECTOR_SQUASH);
#ifdef DBG_MOTOR
  print("MOTOR CTRL[(%+03i/%+03i) -> %+03i/%+03i]",
        ohori, overi, hori, veri);
#endif
  motor_translate(hori, veri, motor_ctrl);

  s16_t adj = APP_cfg_get_val(CFG_STEER_ADJUST);
  s16_t left = motor_ctrl[0];
  s16_t right = motor_ctrl[1];

#ifdef DBG_MOTOR
  print("  TRANS[%+03i/%+03i]",
      left, right);
#endif

  if (adj < 0) {
    adj = -adj;
    adj >>= 2;
    adj = 128-adj;
    right = (s16_t)((s32_t)(adj*right)/128);
  } else if (adj > 0) {
    adj >>= 2;
    adj = 128-adj;
    left = (s16_t)((s32_t)(adj*left)/128);
  }

  if (APP_cfg_get_val(CFG_COMMON) & CFG_COMMON_LEFT_INVERT) {
    left = -left;
  }
  if (APP_cfg_get_val(CFG_COMMON) & CFG_COMMON_RIGHT_INVERT) {
    right = -right;
  }

#ifdef DBG_MOTOR
  print("  ADJ[%+03i/%+03i]\n",
      left, right);
  upd_report = TRUE;
#endif

  motor.desired[0] = left;
  motor.desired[1] = right;

}

void MOTOR_update(void) {
  s16_t motor_d[2];
  motor_d[0] = motor.desired[0] - motor.actual[0];
  motor_d[1] = motor.desired[1] - motor.actual[1];
  int i;
  for (i = 0; i < 2; i++) {
    if (motor_d[i] == 0) {
      continue;
    } else if (ABS(motor_d[i]) < MOTOR_DELTA) {
      motor.actual[i] = motor.desired[i];
    } else {
      if (motor_d[i] < 0) {
        motor.actual[i] -= MOTOR_DELTA;
      } else {
        motor.actual[i] += MOTOR_DELTA;
      }
    }
  }

  // stretch motor pwm cycle from origo, too low duty cycles will not make motors run
  s16_t left_stretch = motor_lin_stretch(motor.actual[0], MOTOR_PWM_CYCLE_STRETCH);
  s16_t right_stretch = motor_lin_stretch(motor.actual[1], MOTOR_PWM_CYCLE_STRETCH);
#ifdef DBG_MOTOR
  if (upd_report) {
    print("MOTOR  SET[(%+03i/%+03i) -> %+03i/%+03i]\n",
        motor.desired[0], motor.desired[1], left_stretch, right_stretch);
    upd_report = FALSE;
  }
#endif

  motor_set(left_stretch, right_stretch);
}
