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
*/

#include "motor.h"
#include "gpio.h"

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

static void motor_stop(void) {
  gpio_disable(PORTB, PIN12);
  gpio_disable(PORTB, PIN13);
  gpio_disable(PORTB, PIN14);
  gpio_disable(PORTB, PIN15);
  set_motora_pwm_duty_cycle(MOTOR_PERIOD);
  set_motorb_pwm_duty_cycle(MOTOR_PERIOD);
}

void MOTOR_init(void) {
  motor_stop();
}

void MOTOR_go(s8_t x) {
  if (x == 0) {
    motor_stop();
    return;
  } else if (x > 0) {
    gpio_enable(PORTB, PIN12);
    gpio_disable(PORTB, PIN13);
    gpio_enable(PORTB, PIN14);
    gpio_disable(PORTB, PIN15);
    x = x+1;
  } else if (x < 0) {
    gpio_disable(PORTB, PIN12);
    gpio_enable(PORTB, PIN13);
    gpio_disable(PORTB, PIN14);
    gpio_enable(PORTB, PIN15);
    x = -x;
  }
  set_motora_pwm_duty_cycle((MOTOR_PERIOD*x)/128);
  set_motorb_pwm_duty_cycle((MOTOR_PERIOD*x)/128);
}
