#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lineanalog.h"
#include "motor.h"
#include <stdio.h>

#define BASE_SPEED_FAST 180 // Tốc độ khi đi thẳng
#define BASE_SPEED_SLOW 100 // Tốc độ khi vào cua gắt
#define KP_STRAIGHT 0.04f   // Kp khi đường thẳng
#define KP_CURVE 0.08f      // Kp khi cua gắt
#define KD_VAL 0.6f         // Kd dùng chung (hoặc tách ra nếu cần)

line_sensor_t sensor;
motor_t motor_l, motor_r;

void app_main(void) {
  // Motor Trái: IN1=GPIO5, IN2=GPIO17, PWM=GPIO18
  motor_init(&motor_l, 5, 17, 18, LEDC_CHANNEL_0);
  // Motor Phải: IN1=GPIO16, IN2=GPIO4, PWM=GPIO19
  motor_init(&motor_r, 16, 4, 19, LEDC_CHANNEL_1);

  // 2. Cấu hình Cảm biến & Gain Scheduling
  line_sensor_config_t line_cfg = {.pins =
                                       {
                                           {ADC_UNIT_2, ADC_CHANNEL_9}, // S1
                                           {ADC_UNIT_1, ADC_CHANNEL_7}, // S2
                                           {ADC_UNIT_2, ADC_CHANNEL_8}, // S3
                                           {ADC_UNIT_1, ADC_CHANNEL_6}, // S4
                                           {ADC_UNIT_1, ADC_CHANNEL_5}, // S5
                                           {ADC_UNIT_1, ADC_CHANNEL_3}, // S6
                                           {ADC_UNIT_1, ADC_CHANNEL_4}, // S7
                                           {ADC_UNIT_1, ADC_CHANNEL_0}, // S8
                                       },
                                   .atten = ADC_ATTEN_DB_12,
                                   .ema_alpha = 0.2f, // Lọc mượt trung bình
                                   .gs = {.kp_min = KP_STRAIGHT,
                                          .kp_max = KP_CURVE,
                                          .kd_min = KD_VAL,
                                          .kd_max = KD_VAL * 1.5f,
                                          .speed_min = BASE_SPEED_SLOW,
                                          .speed_max = BASE_SPEED_FAST}};
  line_sensor_init(&sensor, &line_cfg);

  // 3. Calib cảm biến (Cho xe tự xoay 3 giây trước khi chạy)
  motor_set_speed(&motor_l, 100);
  motor_set_speed(&motor_r, -100);
  line_sensor_calibrate(&sensor, 5000);
  motor_stop(&motor_l);
  motor_stop(&motor_r);
  vTaskDelay(pdMS_TO_TICKS(1000));

  // 4. Vòng lặp PID chính
  float last_error = 0;
  int mapped[8], contrast, active, base_speed;
  float kp, kd;

  while (1) {
    // Lấy Error và thông số Gain Scheduling tự động
    float error = line_sensor_get_error(&sensor, mapped, &contrast, &active,
                                        &kp, &kd, &base_speed);

    // Tính toán PD
    float p_term = error * kp;
    float d_term = (error - last_error) * kd;
    float pid_output = p_term + d_term;

    // Differential Drive (Bù trừ tốc độ)
    int speed_l = base_speed + (int)pid_output;
    int speed_r = base_speed - (int)pid_output;

    // Xuất lệnh cho motor
    motor_set_speed(&motor_l, speed_l);
    motor_set_speed(&motor_r, speed_r);

    last_error = error;

    // In log để debug khi cần
    // printf("E:%.1f | P:%.1f | Kp:%.3f | Spd:%d\n", error, pid_output, kp,
    // base_speed);

    vTaskDelay(pdMS_TO_TICKS(10)); // Chu kỳ 100Hz
  }
}
