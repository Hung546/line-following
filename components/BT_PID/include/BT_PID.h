#ifndef BT_PID_H
#define BT_PID_H

#include <stdint.h>

// Struct mới chứa toàn bộ linh hồn của Gain Scheduling
typedef struct {
  float kp_min, kp_max;
  float kd_min, kd_max;
  int speed_min, speed_max;
  float ema_alpha; // Thêm cái này để tune độ mượt từ xa luôn
} robot_params_t;

void bt_pid_init(void);
robot_params_t get_current_params(void);

#endif
