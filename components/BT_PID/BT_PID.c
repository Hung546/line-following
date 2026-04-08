#include "BT_PID.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"
#include "esp_spp_api.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SPP_SERVER_NAME "SPP_SERVER"
#define DEVICE_NAME "xedoline"

static const char *TAG = "BT_PID";

// Khởi tạo giá trị mặc định an toàn để nếu Flash trống xe vẫn chạy được
static robot_params_t current_params = {.kp_min = 0.8f,
                                        .kp_max = 2.0f,
                                        .kd_min = 0.1f,
                                        .kd_max = 1.0f,
                                        .speed_min = 80,
                                        .speed_max = 180,
                                        .ema_alpha = 0.2f};

// --- QUẢN LÝ BỘ NHỚ FLASH ---

static void save_params_to_flash(robot_params_t *params) {
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err == ESP_OK) {
    // Lưu toàn bộ "cục" struct vào Flash
    nvs_set_blob(my_handle, "gs_params", params, sizeof(robot_params_t));
    nvs_commit(my_handle);
    nvs_close(my_handle);
    ESP_LOGI(TAG, "Da ghi Flash các tham số GS mới.");
  }
}

static void load_params_from_flash() {
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
  if (err == ESP_OK) {
    size_t required_size = sizeof(robot_params_t);
    // Kiểm tra xem dữ liệu trong Flash có khớp kích thước struct không
    err = nvs_get_blob(my_handle, "gs_params", &current_params, &required_size);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "Dữ liệu Flash cũ không khớp, dùng giá trị mặc định.");
    }
    nvs_close(my_handle);
  }
}

// --- XỬ LÝ DỮ LIỆU BLUETOOTH ---

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  switch (event) {
  case ESP_SPP_INIT_EVT:
    esp_bt_gap_set_device_name(DEVICE_NAME);
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);
    break;

  case ESP_SPP_DATA_IND_EVT: {
    char buf[128];
    int len = param->data_ind.len < (sizeof(buf) - 1) ? param->data_ind.len
                                                      : (sizeof(buf) - 1);
    memcpy(buf, param->data_ind.data, len);
    buf[len] = '\0';

    ESP_LOGI(TAG, "Data nhận: %s", buf);
    bool updated = false;
    char *ptr;

    // PM=Kp Max, Pm=Kp Min, DM=Kd Max, Dm=Kd Min, SM=Spd Max, Sm=Spd Min,
    // AL=Alpha
    if ((ptr = strstr(buf, "PM=")) != NULL) {
      sscanf(ptr, "PM=%f", &current_params.kp_max);
      updated = true;
    }
    if ((ptr = strstr(buf, "Pm=")) != NULL) {
      sscanf(ptr, "Pm=%f", &current_params.kp_min);
      updated = true;
    }
    if ((ptr = strstr(buf, "DM=")) != NULL) {
      sscanf(ptr, "DM=%f", &current_params.kd_max);
      updated = true;
    }
    if ((ptr = strstr(buf, "Dm=")) != NULL) {
      sscanf(ptr, "Dm=%f", &current_params.kd_min);
      updated = true;
    }
    if ((ptr = strstr(buf, "SM=")) != NULL) {
      sscanf(ptr, "SM=%d", &current_params.speed_max);
      updated = true;
    }
    if ((ptr = strstr(buf, "Sm=")) != NULL) {
      sscanf(ptr, "Sm=%d", &current_params.speed_min);
      updated = true;
    }
    if ((ptr = strstr(buf, "AL=")) != NULL) {
      sscanf(ptr, "AL=%f", &current_params.ema_alpha);
      updated = true;
    }

    if (updated) {
      save_params_to_flash(&current_params);
      ESP_LOGI(TAG, "Update OK! PM=%.2f, Pm=%.2f, SM=%d", current_params.kp_max,
               current_params.kp_min, current_params.speed_max);
    }
    break;
  }
  case ESP_SPP_SRV_OPEN_EVT:
    ESP_LOGI(TAG, "Smartphone đã kết nối!");
    break;
  default:
    break;
  }
}

// --- HÀM PUBLIC ---

void bt_pid_init(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  load_params_from_flash();

  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
  ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BTDM));
  ESP_ERROR_CHECK(esp_bluedroid_init());
  ESP_ERROR_CHECK(esp_bluedroid_enable());

  esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
  esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
  esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

  ESP_ERROR_CHECK(esp_spp_register_callback(esp_spp_cb));
  esp_spp_cfg_t bt_spp_cfg = {
      .mode = ESP_SPP_MODE_CB, .enable_l2cap_ertm = true, .tx_buffer_size = 0};
  ESP_ERROR_CHECK(esp_spp_enhanced_init(&bt_spp_cfg));

  ESP_LOGI(TAG, "Bluetooth Gain Scheduling Ready!");
}

robot_params_t get_current_params(void) { return current_params; }
