#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include "./src/SyntrakImuVersion.h"

// ===== 构建类型 =====
// 启用 _PRODUCT_BUILD = 产品版（无串口、无BLE、产品板引脚、LED）
// 注释掉          = 开发板（串口+BLE、开发板引脚）
#define _PRODUCT_BUILD  1

// ===== 功能开关 =====
#define _ENABLE_CAN       1   // CAN始终启用
// #define _ENABLE_9AXIS    1   // 9轴融合（默认关闭=6轴，yaw不漂移但易受铁磁干扰）

#ifndef _PRODUCT_BUILD
    // 开发板：启用BLE和串口调试
    #define _ENABLE_BLE       1
    #define _DEBUG_PRINT      1
#endif  
// 产品版：_ENABLE_BLE 和 _DEBUG_PRINT 不定义，自动关闭


// ===== 引脚配置 =====
#ifdef _PRODUCT_BUILD
    // ===== 产品板引脚 =====
    #define IMU_SDA_PIN      13
    #define IMU_SCL_PIN      12
    #define CAN_TX_PIN       GPIO_NUM_34
    #define CAN_RX_PIN       GPIO_NUM_33
    #define LED_PIN          10
    #define LED_ON_LEVEL     0     // 0=点亮, 1=熄灭
#else
    // ===== 开发板引脚 =====
    #define IMU_SDA_PIN      6
    #define IMU_SCL_PIN      7
    #define CAN_TX_PIN       GPIO_NUM_4
    #define CAN_RX_PIN       GPIO_NUM_5
    // 开发板无LED（不定义LED_PIN）
#endif

#define IMU_I2C_ADDR     0x68
#define IMU_SAMPLE_RATE  100.0f
#define IMU_SAMPLE_PERIOD (1.0f / IMU_SAMPLE_RATE)

// BLE 通知频率（仅开发板用）
#define BLE_NOTIFY_RATE      30.0f
#define BLE_NOTIFY_INTERVAL_US ((uint32_t)(1000000.0f / BLE_NOTIFY_RATE))  // ~33333us

// ===== 动静判断阈值 =====
#define STATIONARY_GYRO_THRESHOLD   8.0f   // 陀螺仪幅度阈值 (deg/s)，低于此值认为可能静止（TPOS时13个IMU全部静止，允许人手抖动）
#define STATIONARY_CONFIRM_FRAMES   40     // 持续多少帧确认静止 (40帧=0.4s@100Hz)
#define STATIONARY_RELEASE_FRAMES   5      // 持续多少帧确认运动 (5帧=0.05s@100Hz)

// ===== CAN 配置 =====
#define CAN_BAUDRATE     500000
#define CAN_CMD_ID       Syntrak_TYPE_MAINCONTROLLER   // 0xC0 主控命令ID

// CAN 发送频率 60Hz + 20时间片分配
#define CAN_SEND_RATE       60.0f
#define CAN_PERIOD_US        ((uint32_t)(1000000.0f / CAN_SEND_RATE))  // 16666us
#define TIME_SLOT_COUNT      20
#define TIME_SLOT_US         (CAN_PERIOD_US / TIME_SLOT_COUNT)           // 833us
#define MASTER_TIME_SLOT     0    // 主控在时间片0发送命令/同步

// 默认传感器ID（可通过BLE CANID命令修改，存NVS）
#define DEFAULT_SENSOR_ID    IMU_ID_ForeArm_Left   // 12

// ===== BLE UUID（仅开发板用）=====
#define BLE_SERVICE_UUID     "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHAR_RAW_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_CHAR_CAL_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define BLE_CHAR_QUAT_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26aa"
#define BLE_CHAR_WRITE_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26ab"
#define BLE_CHAR_INFO_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26ac"

// ===== CAN 命令字节定义 =====
#define CAN_CMD_SYNC         0x01   // 时间同步命令
#define CAN_CMD_RESET        0x02   // 复位姿态
#define CAN_CMD_SHUTDOWN     0x03   // 关机

// ===== CAN 状态字节位定义 (byte6) =====
#define CAN_STATUS_RUNNING    0x01  // bit0: 运行状态
#define CAN_STATUS_QW_NEG     0x02  // bit1: qw符号 (0=正, 1=负)
#define CAN_STATUS_CAL_VALID  0x04  // bit2: 校准参数有效
#define CAN_STATUS_BLE_CONN   0x08  // bit3: BLE已连接
#define CAN_STATUS_STATIONARY 0x10  // bit4: 静止状态 (1=静止, 0=运动)

// ===== 设备版本信息 =====
#ifndef SYNTRAK_DEVICE_SN
#define SYNTRAK_DEVICE_SN        "SN-IMU-001"
#endif
#ifndef SYNTRAK_HARDWARE_VERSION
#define SYNTRAK_HARDWARE_VERSION "HW1.3"
#endif
#ifndef SYNTRAK_SOFTWARE_VERSION
#define SYNTRAK_SOFTWARE_VERSION "FW2.1"
#endif

#endif // CONFIG_H
