#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lineanalog.h"
#include "motor.h"
#include <math.h>
#include <stdio.h>

// SPEED
#define BASE_SPEED_FAST 110
#define BASE_SPEED_CURVE 70
#define SPEED_PIVOT_TURN 30
// KP AND KD
#define KP_STRAIGHT 0.03f
#define KP_MAX_WHIP 0.13f
#define KD 1.0f
#define KD_FILTER 0.4f

line_sensor_t sensor;
motor_t motor_l, motor_r;

void app_main(void) {
  // INIT MOTOR AND ANALOG SENSOR BEGIN
  motor_init(&motor_l, 5, 17, 18, LEDC_CHANNEL_0);
  motor_init(&motor_r, 16, 4, 19, LEDC_CHANNEL_1);

  line_sensor_config_t line_cfg = {.pins = {{ADC_UNIT_2, ADC_CHANNEL_9},
                                            {ADC_UNIT_1, ADC_CHANNEL_7},
                                            {ADC_UNIT_2, ADC_CHANNEL_8},
                                            {ADC_UNIT_1, ADC_CHANNEL_6},
                                            {ADC_UNIT_1, ADC_CHANNEL_5},
                                            {ADC_UNIT_1, ADC_CHANNEL_3},
                                            {ADC_UNIT_1, ADC_CHANNEL_4},
                                            {ADC_UNIT_1, ADC_CHANNEL_0}},
                                   .atten = ADC_ATTEN_DB_12,
                                   .ema_alpha = 0.25f,
                                   .gs = {.kp_min = KP_STRAIGHT,
                                          .kp_max = 0.045f,
                                          .kd_min = KD,
                                          .kd_max = KD * 2.2f,
                                          .speed_min = BASE_SPEED_CURVE,
                                          .speed_max = BASE_SPEED_FAST}};

  line_sensor_init(&sensor, &line_cfg);
  // INIT END

  // Cho sensor đọc line trong vòng x ms
  line_sensor_calibrate(&sensor, 5000);

  float last_error = 0;
  float filtered_d = 0;
  int mapped[8], contrast, active, base_speed;
  float kp, kd;

  while (1) {
    float raw_error = line_sensor_get_error(&sensor, mapped, &contrast, &active,
                                            &kp, &kd, &base_speed);

    float final_error = raw_error;
    int current_base_speed = base_speed;
    float current_kp = kp;

    // Xử lý vạch đen song song cong và nét đứt
    if (contrast < 400 || active == 0) {
      final_error = last_error;
      // Xử lý vạch đen song song cong
      if (fabsf(last_error) > 1500.0f) {
        current_base_speed = SPEED_PIVOT_TURN;
        current_kp = KP_MAX_WHIP;
        // Xử lý nét đứt
      } else {
        current_base_speed = BASE_SPEED_FAST - 10; // Phanh nhẹ hơn ở vạch đứt
      }
    }
    // Xử lý đường zigzag và đoạn cong ở cuối
    else {
      float err_abs = fabsf(final_error);
      // Nếu ads(error) mà > 3000 tức là vạch đang ở rìa của mắt
      if (err_abs > 3000.0f) {
        current_base_speed = SPEED_PIVOT_TURN; // Cho xe đi chậm lại
        current_kp = KP_MAX_WHIP; // Tăng KP để nó có thể bẻ lái mạnh hơn
      } else if (err_abs > 1800.0f) {
        current_base_speed = BASE_SPEED_CURVE;
      }
    }

    float p_term = final_error * current_kp;

    float raw_d = (final_error - last_error) * kd;
    // filter cho D bằng EMA
    filtered_d = (filtered_d * (1.0f - KD_FILTER)) + (raw_d * KD_FILTER);

    float pid_output = p_term + filtered_d;

    // Giới hạn output để chắc chắn không làm motor quá tải
    if (pid_output > 200)
      pid_output = 200;
    if (pid_output < -200)
      pid_output = -200;

    // Điều khiển tốc độ motor
    motor_set_speed(&motor_l, current_base_speed + (int)pid_output);
    motor_set_speed(&motor_r, current_base_speed - (int)pid_output);

    last_error =
        final_error; // Cập nhật lại cái last error để khi mất line thì dùng
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
