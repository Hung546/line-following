#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lineanalog.h"
#include "motor.h"
#include <math.h>
#include <stdbool.h>

// Thông số cơ bản
#define BASE_SPEED 100
#define BASE_KP 0.005f
#define BASE_KD 1.0f

line_sensor_t sensor;
motor_t motor_l, motor_r;

// Các trạng thái
typedef enum {
  STATE_STRAIGHT,     // Thẳng
  STATE_SLIGHT_CURVE, // Cong nhẹ
  STATE_CORNER,       // Góc 90 độ, Zigzag
  STATE_U_SHAPE,      // Cua gắt
  STATE_SHARP_PEAK,   // Đỉnh nhọn }
  STATE_LOST          // Nét đứt
} robot_state_t;

// Hàm lấy trạng thái dựa trên dữ liệu lấy từ hàm get_error
robot_state_t get_robot_state(float error, int active, int contrast,
                              float last_error) {
  float err_abs = fabsf(error);

  // 1. Mất Line
  if (contrast < 400 || active == 0)
    return STATE_LOST;

  // 2. Đỉnh Nhọn }
  if (active >= 6 && fabsf(last_error) > 1500.0f) {
    return STATE_SHARP_PEAK;
  }

  // 3. Các trường hợp còn lại
  if (err_abs >= 1950.0f)
    return STATE_U_SHAPE;
  if (err_abs >= 1650.0f || (active >= 5 && err_abs > 1500.0f))
    return STATE_CORNER;
  if (err_abs >= 1200.0f)
    return STATE_SLIGHT_CURVE;

  // 4. Đường thẳng
  return STATE_STRAIGHT;
}

// Hàm gán kp,kd,speed,error sau khi đã có trạng thái của xe
void set_parameters(robot_state_t state, float *target_speed, float *target_kp,
                    float *target_kd, float raw_error, float last_error,
                    float *final_error) {

  // Mặc định error là error thô đọc từ sensor
  *final_error = raw_error;

  switch (state) {
  case STATE_STRAIGHT:
    // P = 0.01, D = 1.0
    *target_speed = BASE_SPEED;
    *target_kp = BASE_KP + 0.005f;
    *target_kd = BASE_KD;
    break;

  case STATE_SLIGHT_CURVE:
    // P = 0.02, D = 1.5
    *target_speed = BASE_SPEED;
    *target_kp = BASE_KP + 0.015f;
    *target_kd = BASE_KD + 0.5f;
    break;

  case STATE_CORNER:
    // P = 0.045, D = 3.5
    *target_speed = BASE_SPEED - 50;
    *target_kp = BASE_KP + 0.04f;
    *target_kd = BASE_KD + 2.5f;
    break;

  case STATE_U_SHAPE:
    // P = 0.045, D = 3.5
    *target_speed = BASE_SPEED - 60;
    *target_kp = BASE_KP + 0.04f;
    *target_kd = BASE_KD + 2.5f;
    break;

  case STATE_SHARP_PEAK:
    // P = 0.10, D = 1.0
    *target_speed = BASE_SPEED - 140;
    *target_kp = BASE_KP + 0.095f;
    *target_kd = BASE_KD;

    *final_error = (last_error > 0) ? 3500.0f : -3500.0f;
    break;

  case STATE_LOST:
    *final_error = last_error;
    // P = 0.035, D = 2.0
    *target_speed = BASE_SPEED;
    *target_kp = BASE_KP + 0.030f;
    *target_kd = BASE_KD + 1.0f;
    break;
  }
}

// CHƯƠNG TRÌNH CHÍNH
void app_main(void) {
  // Khởi tạo Motor
  motor_init(&motor_l, 5, 17, 18, LEDC_CHANNEL_0);
  motor_init(&motor_r, 16, 4, 19, LEDC_CHANNEL_1);

  // Khởi tạo Sensor dò line
  line_sensor_config_t line_cfg = {.pins = {{ADC_UNIT_2, ADC_CHANNEL_9},
                                            {ADC_UNIT_1, ADC_CHANNEL_7},
                                            {ADC_UNIT_2, ADC_CHANNEL_8},
                                            {ADC_UNIT_1, ADC_CHANNEL_6},
                                            {ADC_UNIT_1, ADC_CHANNEL_5},
                                            {ADC_UNIT_1, ADC_CHANNEL_3},
                                            {ADC_UNIT_1, ADC_CHANNEL_4},
                                            {ADC_UNIT_1, ADC_CHANNEL_0}},
                                   .atten = ADC_ATTEN_DB_12,
                                   .ema_alpha = 0.25f};

  line_sensor_init(&sensor, &line_cfg);
  // Bước hiệu chỉnh tính hiệu
  line_sensor_calibrate(&sensor, 5000);

  float last_error = 0;
  int mapped[8], contrast, active;

  while (1) {
    // 1. Lấy error đo được tại thời điểm đó
    float raw_error =
        line_sensor_get_error(&sensor, mapped, &contrast, &active);

    // 2. Lấy trạng thái của xe
    robot_state_t current_state =
        get_robot_state(raw_error, active, contrast, last_error);

    float target_speed, target_kp, target_kd, final_error;

    // 3. Gán speed,kp,kd sau khi đã xác định được trạng thái của xe
    set_parameters(current_state, &target_speed, &target_kp, &target_kd,
                   raw_error, last_error, &final_error);

    // 4. Tính PID
    float p_term = final_error * target_kp;
    float d_term = (final_error - last_error) * target_kd;

    float pid_output = p_term + d_term;

    // Giới hạn PWM bảo vệ Motor
    if (pid_output > 180)
      pid_output = 180;
    if (pid_output < -180)
      pid_output = -180;

    // 5. Xuất xung PWM cho Motor
    motor_set_speed(&motor_l, (int)target_speed + (int)pid_output);
    motor_set_speed(&motor_r, (int)target_speed - (int)pid_output);

    last_error = final_error;
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}
