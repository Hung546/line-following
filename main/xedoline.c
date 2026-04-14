#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lineanalog.h"
#include "motor.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

line_sensor_t sensor;
motor_t motor_l, motor_r;

// Tập hợp Trạng thái FSM Thuần (Đã băm nhỏ địa hình và khôi phục Đỉnh nhọn)
typedef enum {
  STATE_STRAIGHT,     // Thẳng tắp (Error < 1200)
  STATE_SLIGHT_CURVE, // Cong nhẹ (Error 1200 - 1650)
  STATE_CORNER,       // Góc 90 độ, Zigzag (Error 1650 - 1950)
  STATE_U_SHAPE,      // Cua tay áo (Error > 1950)
  STATE_SHARP_PEAK,   // Đỉnh nhọn } (Active >= 6)
  STATE_LOST          // Nét đứt (Mù)
} robot_state_t;

// ==========================================
// HÀM 1: FSM NHẬN DIỆN ĐỊA HÌNH TỪ SỐ ĐO THỰC TẾ
// ==========================================
robot_state_t get_robot_state(float error, int active, int contrast,
                              float last_error) {
  float err_abs = fabsf(error);

  // 1. Mất Line
  if (contrast < 400 || active == 0)
    return STATE_LOST;

  // 2. Đỉnh Nhọn } (Ưu tiên tát vỡ "cú lừa trọng tâm")
  if (active >= 6 && fabsf(last_error) > 1500.0f) {
    return STATE_SHARP_PEAK;
  }

  // 3. Phân lô bán nền theo thông số chuẩn xác đã test
  if (err_abs >= 1950.0f)
    return STATE_U_SHAPE;
  if (err_abs >= 1650.0f || (active >= 5 && err_abs > 1500.0f))
    return STATE_CORNER;
  if (err_abs >= 1200.0f)
    return STATE_SLIGHT_CURVE;

  // 4. Đường thẳng
  return STATE_STRAIGHT;
}

// ==========================================
// HÀM 2: GÁN THÔNG SỐ (GIỮ NGUYÊN BỘ TUNING CỦA BẠN)
// ==========================================
// Thêm tham số last_error vào khai báo hàm
void set_parameters(robot_state_t state, float *target_speed, float *target_kp,
                    float *target_kd, float last_error) {
  switch (state) {
  case STATE_STRAIGHT:
    *target_speed = 100;
    *target_kp = 0.01f;
    *target_kd = 1.0f;
    break;

  case STATE_SLIGHT_CURVE:
    *target_speed = 100;
    *target_kp = 0.02f;
    *target_kd = 1.5f;
    break;

  case STATE_CORNER:
    *target_speed = 50;
    *target_kp = 0.045f;
    *target_kd = 3.5f;
    break;

  case STATE_U_SHAPE:
    *target_speed = 40;
    *target_kp = 0.045f;
    *target_kd = 3.5f;
    break;

  case STATE_SHARP_PEAK:
    // Phanh kịch sàn, lùi lại để triệt tiêu đà tiến
    *target_speed = -40;
    *target_kp = 0.10f;
    *target_kd = 1.0f;
    break;

  case STATE_LOST:
    // ====================================================
    // BỘ NHỚ CHỐNG VĂNG: Kiểm tra xem trước khi mù, xe đang làm gì?
    // ====================================================
    if (fabsf(last_error) >= 3000.0f) {
      // Đang ôm cua cực gắt (hoặc bị ép 3500 ở đỉnh nhọn) mà tự nhiên mù
      // -> CHẮC CHẮN BỊ VĂNG TRỚN! Giữ nguyên lực phanh và quất đuôi.
      *target_speed = -20; // Tiếp tục ghì lùi
      *target_kp = 0.10f;  // Ép vô lăng mạnh để ngoạm lại line
      *target_kd = 1.0f;
    } else {
      // Đang đi thẳng/cong nhẹ mà mù -> ĐÂY LÀ NÉT ĐỨT! Phóng qua.
      *target_speed = 100;
      *target_kp = 0.035f;
      *target_kd = 2.0f;
    }
    break;
  }
}

// ==========================================
// CHƯƠNG TRÌNH CHÍNH
// ==========================================
void app_main(void) {
  motor_init(&motor_l, 5, 17, 18, LEDC_CHANNEL_0);
  motor_init(&motor_r, 16, 4, 19, LEDC_CHANNEL_1);

  // Cấu hình Sensor (Chỉ lấy Error, BỎ QUA Gain Scheduling)
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
                                   .gs = {.kp_min = 0,
                                          .kp_max = 0,
                                          .kd_min = 0,
                                          .kd_max = 0,
                                          .speed_min = 0,
                                          .speed_max = 0}};
  line_sensor_init(&sensor, &line_cfg);
  line_sensor_calibrate(&sensor, 5000);

  float last_error = 0, filtered_d = 0;
  int mapped[8], contrast, active;

  // Bỏ các biến _raw của GS đi cho nhẹ đầu
  float unused_kp, unused_kd;
  int unused_speed;

  while (1) {
    // 1. Lấy dữ liệu thô
    float raw_error =
        line_sensor_get_error(&sensor, mapped, &contrast, &active, &unused_kp,
                              &unused_kd, &unused_speed);

    // 2. FSM Quyết định trạng thái (Có truyền thêm last_error)
    robot_state_t current_state =
        get_robot_state(raw_error, active, contrast, last_error);

    float target_speed, target_kp, target_kd;

    // GỌI HÀM MỚI: Truyền last_error vào đây
    set_parameters(current_state, &target_speed, &target_kp, &target_kd,
                   last_error);

    float final_error = raw_error;
    if (current_state == STATE_SHARP_PEAK) {
      // Ép lỗi kịch trần để vắt đuôi
      final_error = (last_error > 0) ? 3500.0f : -3500.0f;
    } else if (current_state == STATE_LOST) {
      // Dùng trí nhớ khi bị mù
      final_error = last_error;
    }

    // 5. TÍNH TOÁN PID
    float p_term = final_error * target_kp;
    float raw_d = (final_error - last_error) * target_kd;
    filtered_d = (filtered_d * 0.6f) + (raw_d * 0.4f);

    float pid_output = p_term + filtered_d;

    // Giới hạn PWM bảo vệ phần cứng
    if (pid_output > 180)
      pid_output = 180;
    if (pid_output < -180)
      pid_output = -180;

    // 6. XUẤT XUNG MOTOR
    motor_set_speed(&motor_l, (int)target_speed + (int)pid_output);
    motor_set_speed(&motor_r, (int)target_speed - (int)pid_output);

    last_error = final_error;
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}
