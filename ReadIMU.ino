/**
 * Syntrak IMU Sensor - ESP32-S3 + ICM-20948
 * 模块化重构版
 *
 * 架构:
 *   config.h        - 全局配置（引脚/频率/时间片/UUID/编译选项）
 *   calibration.*   - 校准参数管理（NVS存储/加载/应用）
 *   imu_sensor.*    - ICM-20948 驱动（读取+校准）
 *   attitude.*      - Fusion AHRS 姿态解算（四元数输出）
 *   can_manager.*   - CAN 通信（60Hz时间片发送 + 时间同步 + 命令接收）
 *   ble_manager.*   - BLE 服务（数据通知 + 命令回调）
 *
 * 编译选项 (在 config.h 中开关):
 *   _ENABLE_CAN     - 启用CAN通信
 *   _ENABLE_BLE     - 启用BLE通信
 *   _DEBUG_PRINT    - 启用串口调试输出
 *
 * CAN 时间片机制:
 *   60Hz周期 = 16667us, 分20片 = 每片833us
 *   主控在时间片0发送同步/命令
 *   传感器ID=N 在第N个时间片发送四元数
 *   所有传感器收到同步帧后对齐周期起点
 *
 * CAN 帧格式 (8字节):
 *   byte0-1: qx (int16, q*32767)
 *   byte2-3: qy
 *   byte4-5: qz
 *   byte6:   status (bit0=运行, bit1=qw符号, bit2=校准有效, bit3=BLE连接, bit4=静止)
 *   byte7:   response
 */

#include <Arduino.h>
#include "config.h"
#include "calibration.h"
#include "imu_sensor.h"
#include "attitude.h"
#include "motion_detector.h"
#ifdef _ENABLE_CAN
#include "can_manager.h"
#endif
#ifdef _ENABLE_BLE
#include "ble_manager.h"
#endif

// 调试输出宏
#ifdef _DEBUG_PRINT
  #define DBG_PRINT(...)   Serial.printf(__VA_ARGS__)
  #define DBG_PRINTLN(x)   Serial.println(x)
#else
  #define DBG_PRINT(...)
  #define DBG_PRINTLN(x)
#endif

// ============ 全局状态 ============
static uint32_t last_sample_micros = 0;
static uint32_t last_ble_micros = 0;

// ============ BLE命令处理 ============
#ifdef _ENABLE_BLE
static void on_ble_command(const char* cmd) {
    String s = cmd;
    DBG_PRINT("[MAIN] BLE命令: %s\n", cmd);

    // 复位姿态 + CAN ID恢复默认
    if (s.equalsIgnoreCase("RESET")) {
        Serial.println("[MAIN] 执行 RESET");
        attitude_reset();
        #ifdef _ENABLE_CAN
        can_set_sensor_id(DEFAULT_SENSOR_ID);
        #endif
        return;
    }

    // 设置CAN ID
    #ifdef _ENABLE_CAN
    if (s.startsWith("CANID=")) {
        uint32_t newId = s.substring(6).toInt();
        can_set_sensor_id(newId);
        return;
    }
    #endif

    // 校准数据（24个float逗号分隔）
    calibration_parse_from_string(cmd);
}
#endif

// ============ setup ============
void setup() {
    // 降低CPU频率以减少功耗（100Hz采样+60Hz CAN不需要240MHz）
    setCpuFrequencyMhz(160);

#ifdef _DEBUG_PRINT
    Serial.begin(115200);
    delay(200);
    Serial.println("\n===== Syntrak IMU Sensor 启动 =====");
    Serial.printf("固件版本: %s\n", SYNTRAK_SOFTWARE_VERSION);
#ifdef _PRODUCT_BUILD
    Serial.println("构建类型: 产品版");
#else
    Serial.println("构建类型: 开发板");
#endif
#ifdef _ENABLE_CAN
    Serial.printf("CAN: %ubps, %uHz发送, %u时间片(每片%uus)\n",
                  CAN_BAUDRATE, (uint32_t)CAN_SEND_RATE,
                  TIME_SLOT_COUNT, TIME_SLOT_US);
#else
    Serial.println("CAN: 禁用");
#endif
#ifdef _ENABLE_BLE
    Serial.println("BLE: 启用");
#else
    Serial.println("BLE: 禁用");
#endif
#endif

    // LED开机亮1秒后熄灭（产品板）
#ifdef LED_PIN
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_ON_LEVEL);
    delay(1000);
    digitalWrite(LED_PIN, !LED_ON_LEVEL);
#endif

    // 1. 校准参数
    calibration_init();
    calibration_load();

    // 2. IMU（重试3次，失败进入降级模式）
    if (!imu_init()) {
        Serial.println("[MAIN] 警告：IMU初始化失败，进入降级模式（姿态数据为0）");
    }

    // 3. 姿态解算
    attitude_init();

    // 4. 动静检测器
    motion_detector_init();

    // 5. CAN（加载NVS中的sensor_id）
#ifdef _ENABLE_CAN
    can_init(DEFAULT_SENSOR_ID);
#endif

    // 6. BLE
#ifdef _ENABLE_BLE
    String bleName = "Syntrak-IMU-" + String(
#ifdef _ENABLE_CAN
        can_get_sensor_id()
#else
        DEFAULT_SENSOR_ID
#endif
    );
    ble_init(bleName.c_str(),
#ifdef _ENABLE_CAN
        can_get_sensor_id()
#else
        DEFAULT_SENSOR_ID
#endif
    );
    ble_set_command_callback(on_ble_command);
#endif

    Serial.println("===== 初始化完成 =====");
#ifdef _ENABLE_CAN
    Serial.println("等待CAN时间同步...");
#endif
}

// ============ loop ============
void loop() {
    // ---- 1. IMU采样（100Hz）----
    uint32_t now = micros();
    float dt = (float)(now - last_sample_micros) / 1000000.0f;

    if (dt >= IMU_SAMPLE_PERIOD) {
        last_sample_micros = now;
        // 限制最大dt（防止长时间阻塞后跳变）
        if (dt > IMU_SAMPLE_PERIOD * 2.0f) dt = IMU_SAMPLE_PERIOD;

        // 读取一帧传感器数据（同时获取校准后和原始数据，只读一次I2C）
        float accel[3], gyro[3], mag[3];
        float raw_a[3], raw_g[3], raw_m[3];
        if (imu_read(accel, gyro, mag, raw_a, raw_g, raw_m)) {
            // 姿态解算
            attitude_update(gyro, accel, mag, dt);
            Quaternion q = attitude_get_quaternion();

            // 动静检测（基于校准后陀螺仪幅度）
            motion_detector_update(gyro);
            bool is_stationary = motion_detector_is_stationary();

            // BLE数据通知（30Hz节流，避免100Hz×3特征=300条/秒导致丢包）
#ifdef _ENABLE_BLE
            if (ble_is_connected()) {
                uint32_t now_ble = micros();
                if (now_ble - last_ble_micros >= BLE_NOTIFY_INTERVAL_US) {
                    last_ble_micros = now_ble;
                    ble_send_raw(raw_a, raw_g, raw_m);
                    ble_send_calibrated(accel, gyro, mag);
                    ble_send_quaternion(q.w, q.x, q.y, q.z, is_stationary);
                }
            }
#endif

            // 构造状态字节（额外位：校准有效 + BLE连接 + 静止状态）
            uint8_t status_extra = 0x00;
            if (calibration_get()->valid) status_extra |= CAN_STATUS_CAL_VALID;
#ifdef _ENABLE_BLE
            if (ble_is_connected()) status_extra |= CAN_STATUS_BLE_CONN;
#endif
            if (is_stationary) status_extra |= CAN_STATUS_STATIONARY;

            // 更新CAN发送缓存（实际发送由时间片触发）
#ifdef _ENABLE_CAN
            uint8_t resp = can_pop_command();
            can_send_quaternion(q.w, q.x, q.y, q.z, status_extra, resp);

            // 处理CAN命令（复位等）
            if (resp == CAN_CMD_RESET) {
                attitude_reset();
            }
#endif
        }
        // I2C读取失败时跳过此帧，但继续执行can_tick()（不影响时间片调度）
    }

    // ---- 2. CAN tick 已由 esp_timer 每1ms驱动（can_manager.cpp），主循环不再调用 ----
    // 这样发送时机不受主循环 IMU 处理耗时影响，时间片不会被跳过
#ifdef _ENABLE_CAN
    // can_tick();  // 已移至 esp_timer 回调
#endif

    // 让出CPU，降低功耗和发热（对100Hz采样和60Hz CAN无影响）
    vTaskDelay(1);
}
