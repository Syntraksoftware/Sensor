/**
 * motion_detector.cpp - 系统动静判断实现
 *
 * 滞回机制：
 *   - 陀螺仪幅度 < 阈值 持续 CONFIRM_FRAMES 帧 → 判定静止
 *   - 陀螺仪幅度 >= 阈值 持续 RELEASE_FRAMES 帧 → 判定运动
 *   - 滞回避免在阈值附近频繁跳变
 */
#include "motion_detector.h"
#include "config.h"
#include <math.h>

// 内部状态
static bool  is_stationary = false;
static float current_gyro_mag = 0.0f;
static uint32_t stationary_counter = 0;
static uint32_t moving_counter = 0;

void motion_detector_init(void) {
    is_stationary = false;
    current_gyro_mag = 0.0f;
    stationary_counter = 0;
    moving_counter = 0;
}

void motion_detector_update(const float gyro[3]) {
    // 计算陀螺仪幅度 (deg/s)
    float gx = gyro[0], gy = gyro[1], gz = gyro[2];
    current_gyro_mag = sqrtf(gx * gx + gy * gy + gz * gz);

    if (current_gyro_mag < STATIONARY_GYRO_THRESHOLD) {
        // 可能静止
        stationary_counter++;
        moving_counter = 0;
        if (stationary_counter >= STATIONARY_CONFIRM_FRAMES) {
            if (!is_stationary) {
                is_stationary = true;
            }
        }
    } else {
        // 运动中
        moving_counter++;
        stationary_counter = 0;
        if (moving_counter >= STATIONARY_RELEASE_FRAMES) {
            if (is_stationary) {
                is_stationary = false;
            }
        }
    }
}

bool motion_detector_is_stationary(void) {
    return is_stationary;
}

float motion_detector_get_gyro_mag(void) {
    return current_gyro_mag;
}
