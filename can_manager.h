#ifndef CAN_MANAGER_H
#define CAN_MANAGER_H

#include <Arduino.h>
#include "config.h"

// 初始化 CAN 驱动，sensor_id 决定发送时间片
void can_init(uint32_t sensor_id);

// 主循环调用：处理时间片发送 + 接收命令 + 时间同步
void can_tick();

// 主动发送一帧四元数（通常由时间片自动调用，也可手动调用）
// 传入完整四元数 w,x,y,z；qw符号编码到status字节bit1
// status_extra: 额外的状态位（会与qw符号位合并）
// response: 命令响应字节
void can_send_quaternion(float qw, float qx, float qy, float qz,
                         uint8_t status_extra, uint8_t response);

// 是否已收到时间同步（同步后才开始按时间片发送）
bool can_is_synced();

// 获取最近收到的命令字节（0表示无新命令）
uint8_t can_get_last_command();

// 获取并清除命令（调用后命令被清零）
uint8_t can_pop_command();

// 修改传感器ID（同时改变发送时间片），存NVS
void can_set_sensor_id(uint32_t id);

uint32_t can_get_sensor_id();

// 触发一次本地时间同步（测试用，模拟收到同步帧）
void can_trigger_sync();

// CAN发送失败计数（总线忙/仲裁失败等）
uint32_t can_get_tx_fail_count();
void can_reset_tx_fail_count();

// CAN总线是否处于bus-off状态
bool can_is_bus_off();

#endif // CAN_MANAGER_H
