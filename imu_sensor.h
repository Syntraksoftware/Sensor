#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <Arduino.h>

// 初始化 ICM-20948（I2C, 400kHz），重试3次
// 返回 true=成功, false=失败（进入降级模式，IMU数据不可用）
bool imu_init();

// IMU是否初始化成功
bool imu_is_ready();

// 读取一帧传感器数据，同时输出校准后数据和原始数据
// 只读一次 I2C，保证 raw 和 cal 是同一时刻的数据
// cal_*: 校准后数据 (accel=g, gyro=deg/s, mag=uT)
// raw_*: 原始未校准数据
// 返回 true=读取成功, false=I2C通信失败（应跳过此帧）
bool imu_read(float cal_accel[3], float cal_gyro[3], float cal_mag[3],
              float raw_accel[3], float raw_gyro[3], float raw_mag[3]);

// 读取原始未校准数据（独立调用，用于调试）
// 返回 true=成功, false=失败
bool imu_read_raw(float accel[3], float gyro[3], float mag[3]);

#endif // IMU_SENSOR_H
