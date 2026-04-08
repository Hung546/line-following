#ifndef LINE_ANALOG_H
#define LINE_ANALOG_H

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

// 1. Cấu trúc lưu thông số Gain Scheduling (GS)
typedef struct {
  float kp_min, kp_max;     // Hệ số P cho đường thẳng và cua gắt
  float kd_min, kd_max;     // Hệ số D cho đường thẳng và cua gắt
  int speed_min, speed_max; // Tốc độ nền tối thiểu và tối đa
} gs_params_t;

// 2. Định nghĩa chân pin cho từng mắt cảm biến
typedef struct {
  adc_unit_t unit;
  adc_channel_t channel;
} line_pin_t;

// 3. Cấu trúc cấu hình tổng thể (Config)
typedef struct {
  line_pin_t pins[8]; // 8 chân ADC
  adc_atten_t atten;  // Độ suy giảm (thường là 12dB cho 3.3V)
  float ema_alpha;    // Hệ số lọc mượt (0.0 - 1.0)
  gs_params_t gs;     // Gói thông số Gain Scheduling
} line_sensor_config_t;

// 4. Đối tượng Sensor chính
typedef struct {
  adc_oneshot_unit_handle_t adc1_h;
  adc_oneshot_unit_handle_t adc2_h;
  adc_cali_handle_t cali1_h;
  adc_cali_handle_t cali2_h;
  line_sensor_config_t config; // Lưu lại config để dùng khi tính toán
  int min_volt[8];             // Giá trị Volt nhỏ nhất (Đen) sau Calib
  int max_volt[8];             // Giá trị Volt lớn nhất (Trắng) sau Calib
  float last_filtered_error;   // Lưu lỗi cũ cho bộ lọc EMA và vi phân
} line_sensor_t;

// --- API HÀM HỆ THỐNG ---

// Khởi tạo ADC và các handle
esp_err_t line_sensor_init(line_sensor_t *sensor, line_sensor_config_t *config);

// Chế độ xoay tại chỗ để lấy ngưỡng trắng/đen cho từng mắt
void line_sensor_calibrate(line_sensor_t *sensor, uint32_t duration_ms);

// Hàm quan trọng nhất: Vừa tính Error (CWA+EMA), vừa tính luôn bộ hệ số GS thời
// điểm đó
float line_sensor_get_error(line_sensor_t *sensor, int out_mapped[8],
                            int *out_contrast, int *out_active, float *out_kp,
                            float *out_kd, int *out_speed);

#endif
