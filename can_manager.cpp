#include "can_manager.h"
#include <driver/twai.h>
#include <Preferences.h>

// 调试输出宏（默认关闭，需要时在config.h定义_CAN_DEBUG）
#ifdef _CAN_DEBUG
  #define DBG_CAN_PRINT(...) Serial.printf(__VA_ARGS__)
#else
  #define DBG_CAN_PRINT(...)
#endif

static const char* NVS_NS = "imu_cal";
static uint32_t sensor_id = DEFAULT_SENSOR_ID;

// 时间片同步状态
static bool     synced = false;
static uint32_t sync_base_us = 0;       // 同步基准时刻
static uint32_t last_slot_index = 0xFF;  // 上一次处理的时间片序号
static bool     slot_sent = false;        // 当前周期是否已发送

// 命令接收
static volatile uint8_t last_command = 0;

// 待发送的四元数缓存（时间片到了就发最新的）
static float    pending_qw = 1, pending_qx = 0, pending_qy = 0, pending_qz = 0;
static uint8_t  pending_status_extra = 0x00;
static uint8_t  pending_response = 0;
static bool     has_pending_data = false;

// 统计
static uint32_t tx_fail_count = 0;
static bool     bus_off_state = false;

// ============ 内部函数 ============

static void load_sensor_id() {
    Preferences p;
    p.begin(NVS_NS, true);
    sensor_id = p.getUInt("can_id", DEFAULT_SENSOR_ID);
    p.end();
    Serial.printf("[CAN] 传感器ID: %lu (时间片 %lu)\n", sensor_id, sensor_id);
}

static void save_sensor_id() {
    Preferences p;
    p.begin(NVS_NS, false);
    p.putUInt("can_id", sensor_id);
    p.end();
}

// 安装并启动TWAI驱动（Bus-Off恢复时复用）
static bool can_driver_start() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    // 只接收主控命令ID
    f_config.acceptance_code = (CAN_CMD_ID << 21) & 0x1FFFFFFF;
    f_config.acceptance_mask = 0x1FFFFFFF;

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("[CAN] 驱动安装失败");
        return false;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("[CAN] 启动失败");
        twai_driver_uninstall();
        return false;
    }
    return true;
}

// 停止并卸载TWAI驱动（Bus-Off恢复前调用）
static void can_driver_stop() {
    twai_stop();
    twai_driver_uninstall();
}

// 检查并处理Bus-Off状态
static void can_check_bus_off() {
    twai_status_info_t status;
    if (twai_get_status_info(&status) != ESP_OK) return;

    if (status.state == TWAI_STATE_BUS_OFF) {
        if (!bus_off_state) {
            Serial.println("[CAN] 总线进入 Bus-Off，尝试恢复...");
            bus_off_state = true;
        }
        // 恢复：停止→卸载→重装→启动
        can_driver_stop();
        if (can_driver_start()) {
            Serial.println("[CAN] Bus-Off 恢复成功");
            bus_off_state = false;
            synced = false;  // 重新等待同步
        } else {
            Serial.println("[CAN] Bus-Off 恢复失败，下一周期重试");
        }
    } else {
        bus_off_state = false;
    }
}

// 处理收到的CAN命令（同步基准已在can_receive里处理，这里只处理命令本身）
static void handle_command(const twai_message_t* msg) {
    if (msg->data_length_code < 1) return;
    uint8_t cmd = msg->data[0];
    last_command = cmd;

    switch (cmd) {
        case CAN_CMD_SYNC:
            // 纯同步帧，无额外操作（同步基准已在can_receive处理）
            break;

        case CAN_CMD_RESET:
            // 复位姿态（由主循环通过can_pop_command处理）
            Serial.println("[CAN] 收到复位命令");
            break;

        default:
            Serial.printf("[CAN] 收到命令: 0x%02X\n", cmd);
            break;
    }
}

// 检查并接收CAN消息
// 任何来自主控(ID=0xC0)的帧都作为时间同步基准
static void can_receive() {
    twai_message_t msg;
    while (twai_receive(&msg, pdMS_TO_TICKS(0)) == ESP_OK) {
        if (msg.identifier == CAN_CMD_ID) {
            // 收到主控帧 → 立即重置同步基准（不管帧里是什么命令）
            sync_base_us = micros();
            if (!synced) {
                synced = true;
                Serial.println("[CAN] 首次收到主控帧，时间同步已对齐");
            }
            slot_sent = false;
            last_slot_index = 0xFF;
            // 处理具体命令
            handle_command(&msg);
        }
    }
}

// 计算当前在周期中的时间片序号 (0 ~ TIME_SLOT_COUNT-1)
static uint32_t current_time_slot() {
    uint32_t now = micros();
    uint32_t elapsed = now - sync_base_us;
    uint32_t cycle_pos = elapsed % CAN_PERIOD_US;
    return cycle_pos / TIME_SLOT_US;
}

// 发送缓存的四元数
static void send_pending() {
    if (!has_pending_data) return;

    int16_t qx_int = (int16_t)(pending_qx * 32767.0f);
    int16_t qy_int = (int16_t)(pending_qy * 32767.0f);
    int16_t qz_int = (int16_t)(pending_qz * 32767.0f);

    // 状态字节：
    //   bit0 = 运行状态 (每周期同步后恒为1)
    //   bit1 = qw符号
    //   bit2 = 校准有效
    //   bit3 = BLE连接
    //   bit4 = 静止状态
    //   bit5-7 = 保留
    uint8_t status_byte = CAN_STATUS_RUNNING;
    if (pending_qw < 0.0f) status_byte |= CAN_STATUS_QW_NEG;
    status_byte |= pending_status_extra;

    uint8_t data[8];
    data[0] = (uint8_t)(qx_int >> 8);
    data[1] = (uint8_t)(qx_int & 0xFF);
    data[2] = (uint8_t)(qy_int >> 8);
    data[3] = (uint8_t)(qy_int & 0xFF);
    data[4] = (uint8_t)(qz_int >> 8);
    data[5] = (uint8_t)(qz_int & 0xFF);
    data[6] = status_byte;
    data[7] = pending_response;

    twai_message_t msg;
    msg.identifier = sensor_id;
    msg.extd = 0;
    msg.data_length_code = 8;
    msg.flags = 0;
    memcpy(msg.data, data, 8);

    // 非阻塞发送（超时0），失败就丢这一帧，下一周期再发
    // 避免在833μs时间片里等待导致跨片
    if (twai_transmit(&msg, 0) != ESP_OK) {
        tx_fail_count++;
        DBG_CAN_PRINT("[CAN] 发送失败(总线忙), 累计失败=%lu\n", tx_fail_count);
    }
    pending_response = 0;  // 响应只发一次
}

// ============ 公开接口 ============

void can_init(uint32_t id) {
    load_sensor_id();
    if (id != DEFAULT_SENSOR_ID) {
        sensor_id = id;
        save_sensor_id();
    }

    if (can_driver_start()) {
        Serial.printf("[CAN] 初始化成功 500kbps, ID=%lu, 时间片=%lu/%u\n",
                      sensor_id, sensor_id, TIME_SLOT_COUNT);
    }
}

void can_tick() {
    // 0. Bus-Off检测与恢复
    can_check_bus_off();
    if (bus_off_state) return;  // 恢复中，不处理发送

    // 1. 接收主控帧（任何主控帧都自动重置同步基准）
    can_receive();

    // 2. 未收到主控帧前不发送（等待同步）
    if (!synced) return;

    // 3. 时间片调度
    uint32_t slot = current_time_slot();

    // 检测到新周期开始（slot回绕）
    if (slot < last_slot_index) {
        slot_sent = false;
    }
    last_slot_index = slot;

    // 到达本传感器的时间片，且本周期未发送过
    if (slot == sensor_id && !slot_sent) {
        send_pending();
        slot_sent = true;
    }
}

void can_send_quaternion(float qw, float qx, float qy, float qz,
                         uint8_t status_extra, uint8_t response) {
    // 只更新缓存，实际发送由can_tick在对应时间片触发
    pending_qw = qw;
    pending_qx = qx;
    pending_qy = qy;
    pending_qz = qz;
    pending_status_extra = status_extra;
    if (response != 0) pending_response = response;
    has_pending_data = true;
}

bool can_is_synced() {
    return synced;
}

uint8_t can_get_last_command() {
    return last_command;
}

uint8_t can_pop_command() {
    uint8_t cmd = last_command;
    last_command = 0;
    return cmd;
}

void can_set_sensor_id(uint32_t id) {
    // 限制ID范围：1 ~ TIME_SLOT_COUNT-1（0是主控）
    if (id == 0 || id >= TIME_SLOT_COUNT) {
        Serial.printf("[CAN] ID %lu 超出范围 (1~%u)，忽略\n", id, TIME_SLOT_COUNT - 1);
        return;
    }
    sensor_id = id;
    save_sensor_id();
    Serial.printf("[CAN] ID更新为 %lu (时间片 %lu)\n", sensor_id, sensor_id);
}

uint32_t can_get_sensor_id() {
    return sensor_id;
}

void can_trigger_sync() {
    sync_base_us = micros();
    synced = true;
    slot_sent = false;
    last_slot_index = 0xFF;
    Serial.println("[CAN] 本地触发时间同步");
}

uint32_t can_get_tx_fail_count() {
    return tx_fail_count;
}

void can_reset_tx_fail_count() {
    tx_fail_count = 0;
}

bool can_is_bus_off() {
    return bus_off_state;
}
