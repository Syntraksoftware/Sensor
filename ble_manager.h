#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <Arduino.h>

// BLE命令回调类型：收到WRITE特征数据时触发
typedef void (*ble_command_callback_t)(const char* cmd);

// 初始化BLE服务，device_name如 "Syntrak-IMU-1"
void ble_init(const char* device_name, uint32_t sensor_id);

// 是否有客户端连接
bool ble_is_connected();

// 发送原始传感器数据（9轴）
void ble_send_raw(const float accel[3], const float gyro[3], const float mag[3]);

// 发送校准后传感器数据
void ble_send_calibrated(const float accel[3], const float gyro[3], const float mag[3]);

// 发送四元数 (w,x,y,z) + 静止状态 (1=静止, 0=运动)
// 格式: "w,x,y,z,stationary"
void ble_send_quaternion(float w, float x, float y, float z, bool stationary);

// 注册命令回调（RESET, CANID=xxx, 校准数据字符串）
void ble_set_command_callback(ble_command_callback_t cb);

#endif // BLE_MANAGER_H
