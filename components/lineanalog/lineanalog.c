#include "lineanalog.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

// --- TRÌNH HELPER NỘI BỘ ---

static bool adc_cali_init_internal(adc_unit_t unit, adc_atten_t atten,
                                   adc_cali_handle_t *out_handle) {
  adc_cali_line_fitting_config_t cali_config = {
      .unit_id = unit,
      .atten = atten,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  return (adc_cali_create_scheme_line_fitting(&cali_config, out_handle) ==
          ESP_OK);
}

static int find_otsu_threshold(int mapped[]) {
  int sorted[8];
  memcpy(sorted, mapped, sizeof(int) * 8);
  // Bubble sort cho 8 phần tử là quá nhanh rồi
  for (int i = 0; i < 7; i++) {
    for (int j = 0; j < 7 - i; j++) {
      if (sorted[j] > sorted[j + 1]) {
        int temp = sorted[j];
        sorted[j] = sorted[j + 1];
        sorted[j + 1] = temp;
      }
    }
  }
  float max_var = 0;
  int threshold = 500;
  for (int i = 1; i < 8; i++) {
    float w0 = (float)i / 8.0f, w1 = 1.0f - w0;
    float mu0 = 0, mu1 = 0;
    for (int j = 0; j < i; j++)
      mu0 += sorted[j];
    for (int j = i; j < 8; j++)
      mu1 += sorted[j];
    mu0 /= i;
    mu1 /= (8 - i);
    float var = w0 * w1 * (mu0 - mu1) * (mu0 - mu1);
    if (var > max_var) {
      max_var = var;
      threshold = (sorted[i - 1] + sorted[i]) / 2;
    }
  }
  return threshold;
}

// --- API CHÍNH ---

esp_err_t line_sensor_init(line_sensor_t *sensor,
                           line_sensor_config_t *config) {
  memset(sensor, 0, sizeof(line_sensor_t));
  memcpy(&sensor->config, config, sizeof(line_sensor_config_t));

  // Khởi tạo ADC Units (Cẩn thận ADC2 nếu dùng Bluetooth/Wifi)
  adc_oneshot_unit_init_cfg_t cfg1 = {.unit_id = ADC_UNIT_1},
                              cfg2 = {.unit_id = ADC_UNIT_2};
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&cfg1, &sensor->adc1_h));
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&cfg2, &sensor->adc2_h));

  adc_oneshot_chan_cfg_t chan_cfg = {.bitwidth = ADC_BITWIDTH_12,
                                     .atten = config->atten};
  adc_cali_init_internal(ADC_UNIT_1, config->atten, &sensor->cali1_h);
  adc_cali_init_internal(ADC_UNIT_2, config->atten, &sensor->cali2_h);

  for (int i = 0; i < 8; i++) {
    adc_oneshot_unit_handle_t h =
        (config->pins[i].unit == ADC_UNIT_1) ? sensor->adc1_h : sensor->adc2_h;
    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(h, config->pins[i].channel, &chan_cfg));
  }
  return ESP_OK;
}

void line_sensor_calibrate(line_sensor_t *sensor, uint32_t duration_ms) {
  for (int i = 0; i < 8; i++) {
    sensor->min_volt[i] = 4095; // Max ADC 12-bit
    sensor->max_volt[i] = 0;
  }
  uint32_t start = xTaskGetTickCount();
  while (xTaskGetTickCount() - start < pdMS_TO_TICKS(duration_ms)) {
    for (int i = 0; i < 8; i++) {
      int raw, mv;
      adc_oneshot_unit_handle_t h = (sensor->config.pins[i].unit == ADC_UNIT_1)
                                        ? sensor->adc1_h
                                        : sensor->adc2_h;
      adc_cali_handle_t ch = (sensor->config.pins[i].unit == ADC_UNIT_1)
                                 ? sensor->cali1_h
                                 : sensor->cali2_h;

      adc_oneshot_read(h, sensor->config.pins[i].channel, &raw);
      adc_cali_raw_to_voltage(ch, raw, &mv);

      if (mv < sensor->min_volt[i])
        sensor->min_volt[i] = mv;
      if (mv > sensor->max_volt[i])
        sensor->max_volt[i] = mv;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

float line_sensor_get_error(line_sensor_t *sensor, int out_mapped[8],
                            int *out_contrast, int *out_active, float *out_kp,
                            float *out_kd, int *out_speed) {
  int mapped[8], max_m = 0, min_m = 1000;
  const int weights[8] = {-4000, -3000, -2000, -1000, 1000, 2000, 3000, 4000};

  // 1. Đọc ADC và Chuẩn hóa (Normalization)
  for (int i = 0; i < 8; i++) {
    int raw, mv;
    adc_oneshot_unit_handle_t h = (sensor->config.pins[i].unit == ADC_UNIT_1)
                                      ? sensor->adc1_h
                                      : sensor->adc2_h;
    adc_cali_handle_t ch = (sensor->config.pins[i].unit == ADC_UNIT_1)
                               ? sensor->cali1_h
                               : sensor->cali2_h;

    adc_oneshot_read(h, sensor->config.pins[i].channel, &raw);
    adc_cali_raw_to_voltage(ch, raw, &mv);

    int range = sensor->max_volt[i] - sensor->min_volt[i];
    if (range < 100)
      range = 100; // Tránh chia cho 0 hoặc nhiễu quá thấp

    mapped[i] = (mv - sensor->min_volt[i]) * 1000 / range;
    if (mapped[i] < 0)
      mapped[i] = 0;
    if (mapped[i] > 1000)
      mapped[i] = 1000;

    if (mapped[i] > max_m)
      max_m = mapped[i];
    if (mapped[i] < min_m)
      min_m = mapped[i];
    out_mapped[i] = mapped[i];
  }

  // 2. Kiểm tra tương phản (Xử lý vạch đứt - Dashed lines)
  *out_contrast = max_m - min_m;
  if (*out_contrast < 500) {
    // Mất line: giữ Error cũ, dùng bộ số "an toàn" để lướt bằng quán tính
    *out_active = 0;
    *out_kp = sensor->config.gs.kp_min;
    *out_kd = sensor->config.gs.kd_min;
    *out_speed = sensor->config.gs.speed_min;
    return sensor->last_filtered_error;
  }

  // 3. Otsu + CWA (Tìm trọng tâm line)
  int thres = find_otsu_threshold(mapped);
  double sum_w = 0, total_i = 0;
  *out_active = 0;
  for (int i = 0; i < 8; i++) {
    if (mapped[i] < thres) { // Đang thấy vạch đen
      float intensity = 1000.0f - (float)mapped[i];
      sum_w += (double)weights[i] * intensity;
      total_i += intensity;
      (*out_active)++;
    }
  }

  float raw_error = (total_i > 0) ? (float)(sum_w / total_i) : 0;

  // 4. EMA Filter (Dập tắt nhiễu nhấp nhô)
  sensor->last_filtered_error =
      (sensor->config.ema_alpha * raw_error) +
      ((1.0f - sensor->config.ema_alpha) * sensor->last_filtered_error);

  // 5. GAIN SCHEDULING (Tính toán bộ thông số PID thích nghi)
  float ratio = fabsf(sensor->last_filtered_error) / 4000.0f;
  if (ratio > 1.0f)
    ratio = 1.0f;

  // Nội suy tuyến tính hệ số P và D
  *out_kp = sensor->config.gs.kp_min +
            (sensor->config.gs.kp_max - sensor->config.gs.kp_min) * ratio;
  *out_kd = sensor->config.gs.kd_min +
            (sensor->config.gs.kd_max - sensor->config.gs.kd_min) * ratio;

  // Giảm tốc độ nền khi lệch nhiều (Vào cua chậm, ra thẳng nhanh)
  *out_speed =
      (int)(sensor->config.gs.speed_max -
            (sensor->config.gs.speed_max - sensor->config.gs.speed_min) *
                ratio);

  return sensor->last_filtered_error;
}
