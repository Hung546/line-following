#include "lineanalog.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

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

esp_err_t line_sensor_init(line_sensor_t *sensor,
                           line_sensor_config_t *config) {
  memset(sensor, 0, sizeof(line_sensor_t));
  memcpy(&sensor->config, config, sizeof(line_sensor_config_t));

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
    sensor->min_volt[i] = 4095;
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
                            int *out_contrast, int *out_active) {
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
      range = 100; // Tránh chia cho 0

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
    // Mất line: giữ Error cũ
    *out_active = 0;
    return sensor->last_filtered_error;
  }

  // 3. Tự cập nhật threshold + CWA (Tìm trọng tâm line)
  int thres = (max_m + min_m) / 2;

  float sum_w = 0, total_i = 0;
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

  // 4. EMA Filter
  sensor->last_filtered_error =
      (sensor->config.ema_alpha * raw_error) +
      ((1.0f - sensor->config.ema_alpha) * sensor->last_filtered_error);

  return sensor->last_filtered_error;
}
