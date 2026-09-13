#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <Arduino.h>

// 四元数 (w, x, y, z)，单位四元数
struct Quaternion {
    float w, x, y, z;
};

// 欧拉角 (roll, pitch, yaw)，单位度
struct EulerAngle {
    float roll, pitch, yaw;
};

// 初始化姿态解算（Fusion AHRS + Bias）
void attitude_init();

// 更新一帧姿态
// gyro: deg/s, accel: g, mag: uT, dt: 秒
void attitude_update(const float gyro[3], const float accel[3],
                     const float mag[3], float dt);

// 复位姿态（重新初始航向，清零积分）
void attitude_reset();

// 获取当前四元数
Quaternion attitude_get_quaternion();

// 获取当前欧拉角
EulerAngle attitude_get_euler();

#endif // ATTITUDE_H
