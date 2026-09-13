/**
 * motion_detector.h - 系统动静判断
 *
 * 原理：
 *   静止时陀螺仪角速度应接近0（零偏已被FusionBias在线估计）。
 *   如果陀螺仪幅度持续低于阈值，判定为静止；
 *   超过阈值持续一定帧数，判定为运动。
 *
 * 用途：
 *   - TPOS校准时判断某一姿态是否保持稳定
 *   - CAN status字节上报静止状态
 *   - BLE上位机显示动静状态
 */
#ifndef MOTION_DETECTOR_H
#define MOTION_DETECTOR_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 初始化动静检测器（重置计数器和状态）
void motion_detector_init(void);

// 更新一帧陀螺仪数据（单位：deg/s），内部计算幅度并更新状态
void motion_detector_update(const float gyro[3]);

// 当前是否静止
bool motion_detector_is_stationary(void);

// 获取当前陀螺仪幅度（deg/s），用于调试显示
float motion_detector_get_gyro_mag(void);

#ifdef __cplusplus
}
#endif

#endif // MOTION_DETECTOR_H
