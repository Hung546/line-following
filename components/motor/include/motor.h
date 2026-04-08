#ifndef MOTOR_H
#define MOTOR_H

#include "driver/gpio.h"
#include "driver/ledc.h"

#define PWM_TIMER LEDC_TIMER_0
#define PWM_MODE LEDC_HIGH_SPEED_MODE
#define PWM_RES LEDC_TIMER_8_BIT // 8-bit (0–255)
#define PWM_FREQUENCY 5000
#define PWM_DUTY 128

#define MOTOR_MAX_SPEED 255
#define MOTOR_MIN_SPEED -255

typedef struct {
  gpio_num_t in1;
  gpio_num_t in2;
  gpio_num_t pwm_pin;

  ledc_channel_t channel;
  ledc_timer_t timer;
  ledc_mode_t speed_mode;
  int duty;
} motor_t;

// API
void motor_init(motor_t *motor, int in1, int in2, int pwm_pin, int channel);
void motor_set_speed(motor_t *motor, int speed);
void motor_stop(motor_t *motor);

#endif
