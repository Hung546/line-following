#include "motor.h"
#include <stdio.h>

static bool is_timer_config = false;
void motor_init(motor_t *motor, int in1, int in2, int pwm_pin, int channel) {
  // set cac gia tri mac dinh
  motor->duty = PWM_DUTY;
  motor->speed_mode = PWM_MODE;
  motor->timer = PWM_TIMER;
  // GPIO direction
  motor->in1 = in1;
  motor->in2 = in2;

  gpio_set_direction(motor->in1, GPIO_MODE_OUTPUT);
  gpio_set_direction(motor->in2, GPIO_MODE_OUTPUT);

  // PWM timer config
  if (is_timer_config == false) {
    ledc_timer_config_t timer_conf = {.speed_mode = LEDC_HIGH_SPEED_MODE,
                                      .timer_num = LEDC_TIMER_0,
                                      .duty_resolution = PWM_RES,
                                      .freq_hz = PWM_FREQUENCY,
                                      .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&timer_conf);
    is_timer_config = true;
  }

  // PWM channel config
  motor->pwm_pin = pwm_pin;
  motor->channel = channel;
  ledc_channel_config_t ch_conf = {.gpio_num = motor->pwm_pin,
                                   .speed_mode = motor->speed_mode,
                                   .channel = motor->channel,
                                   .timer_sel = motor->timer,
                                   .duty = 0,
                                   .hpoint = 0};
  ledc_channel_config(&ch_conf);
}

void motor_set_speed(motor_t *motor, int speed) {
  if (speed > MOTOR_MAX_SPEED) {
    speed = MOTOR_MAX_SPEED;
  }
  if (speed < MOTOR_MIN_SPEED) {
    speed = MOTOR_MIN_SPEED;
  }
  if (speed > 0) {
    gpio_set_level(motor->in1, 1);
    gpio_set_level(motor->in2, 0);
  } else if (speed < 0) {
    gpio_set_level(motor->in1, 0);
    gpio_set_level(motor->in2, 1);
    speed = -speed;
  } else {
    gpio_set_level(motor->in1, 0);
    gpio_set_level(motor->in2, 0);
  }

  int duty = speed;

  ledc_set_duty(motor->speed_mode, motor->channel, duty);
  ledc_update_duty(motor->speed_mode, motor->channel);
}

void motor_stop(motor_t *motor) {
  gpio_set_level(motor->in1, 0);
  gpio_set_level(motor->in2, 0);

  ledc_set_duty(motor->speed_mode, motor->channel, 0);
  ledc_update_duty(motor->speed_mode, motor->channel);
}
