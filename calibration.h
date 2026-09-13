#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>

// 校准参数：加速度计/磁力计的零偏 + 3x3变换矩阵
struct CalibrationParams {
    float accel_bias[3];
    float accel_transform[3][3];
    float mag_bias[3];
    float mag_transform[3][3];
    bool valid;
};

// 初始化（加载NVS）
void calibration_init();

// 从NVS加载校准参数，返回是否有效
bool calibration_load();

// 保存当前校准参数到NVS
void calibration_save();

// 恢复默认（单位矩阵，零偏为0）
void calibration_reset_default();

// 应用加速度计校准：cal = A * (raw - bias)
void calibration_apply_accel(const float raw[3], float cal[3]);

// 应用磁力计校准
void calibration_apply_mag(const float raw[3], float cal[3]);

// 获取当前校准参数指针
const CalibrationParams* calibration_get();

// 从BLE字符串解析校准参数（24个float，逗号分隔）
// 顺序: accel_bias[3], accel_transform[9], mag_bias[3], mag_transform[9]
bool calibration_parse_from_string(const char* str);

#endif // CALIBRATION_H
