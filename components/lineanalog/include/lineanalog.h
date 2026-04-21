#ifndef LINE_ANALOG_H
#define LINE_ANALOG_H

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_oneshot.h"
// 1. Định nghĩa chân pin cho từng mắt cảm biến
typedef struct {
  adc_unit_t unit;
  adc_channel_t channel;
} line_pin_t;

// 2. Cấu trúc cấu hình tổng thể (Config)
typedef struct {
  line_pin_t pins[8]; // 8 chân ADC
  adc_atten_t atten;  // Độ suy giảm (thường là 12dB cho 3.3V)
  float ema_alpha;    // Hệ số của filter (0.0 - 1.0)
} line_sensor_config_t;

// 3. Đối tượng Sensor
typedef struct {
  adc_oneshot_unit_handle_t adc1_h;
  adc_oneshot_unit_handle_t adc2_h;
  adc_cali_handle_t cali1_h;
  adc_cali_handle_t cali2_h;
  line_sensor_config_t config;
  int min_volt[8];           // Giá trị Volt nhỏ nhất (Đen) sau Calib
  int max_volt[8];           // Giá trị Volt lớn nhất (Trắng) sau Calib
  float last_filtered_error; // Lưu lỗi cũ cho bộ lọc EMA
} line_sensor_t;

// Khởi tạo ADC và các handler
esp_err_t line_sensor_init(line_sensor_t *sensor, line_sensor_config_t *config);

// Hàm hiệu chỉnh
void line_sensor_calibrate(line_sensor_t *sensor, uint32_t duration_ms);

// Hàm lấy error đọc được từ sensor
float line_sensor_get_error(line_sensor_t *sensor, int out_mapped[8],
                            int *out_contrast, int *out_active);

#endif
