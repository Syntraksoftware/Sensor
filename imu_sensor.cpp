#include "imu_sensor.h"
#include "config.h"
#include "calibration.h"
#include <Wire.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>

static Adafruit_ICM20948 icm;
static bool imu_ready = false;

#define IMU_INIT_RETRIES  3     // 初始化重试次数
#define IMU_INIT_DELAY_MS 500   // 每次重试间隔

bool imu_init() {
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    Wire.setClock(400000);

    for (int attempt = 1; attempt <= IMU_INIT_RETRIES; attempt++) {
        Serial.printf("[IMU] 初始化尝试 %d/%d...\n", attempt, IMU_INIT_RETRIES);
        if (icm.begin_I2C(IMU_I2C_ADDR)) {
            icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
            icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
            icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);
            imu_ready = true;
            Serial.println("[IMU] ICM-20948 初始化成功 (±16G, ±2000DPS, 100Hz mag)");
            return true;
        }
        delay(IMU_INIT_DELAY_MS);
    }

    imu_ready = false;
    Serial.println("[IMU] ICM-20948 初始化失败！进入降级模式（IMU数据不可用）");
    return false;
}

bool imu_is_ready() {
    return imu_ready;
}

bool imu_read_raw(float accel[3], float gyro[3], float mag[3]) {
    if (!imu_ready) return false;

    sensors_event_t a, g, m;
    if (!icm.getEvent(&a, &g, &m)) {
        return false;
    }
    accel[0] = a.acceleration.x / 9.80665f;
    accel[1] = a.acceleration.y / 9.80665f;
    accel[2] = a.acceleration.z / 9.80665f;
    gyro[0] = g.gyro.x * 57.2958f;   // rad/s -> deg/s
    gyro[1] = g.gyro.y * 57.2958f;
    gyro[2] = g.gyro.z * 57.2958f;
    mag[0] = m.magnetic.x;
    mag[1] = m.magnetic.y;
    mag[2] = m.magnetic.z;
    return true;
}

bool imu_read(float cal_accel[3], float cal_gyro[3], float cal_mag[3],
              float raw_accel[3], float raw_gyro[3], float raw_mag[3]) {
    if (!imu_ready) return false;

    // 只读一次 I2C，保证 raw 和 cal 是同一时刻的数据
    sensors_event_t a, g, m;
    if (!icm.getEvent(&a, &g, &m)) {
        return false;
    }

    // 原始数据（单位转换）
    raw_accel[0] = a.acceleration.x / 9.80665f;
    raw_accel[1] = a.acceleration.y / 9.80665f;
    raw_accel[2] = a.acceleration.z / 9.80665f;
    raw_gyro[0] = g.gyro.x * 57.2958f;   // rad/s -> deg/s
    raw_gyro[1] = g.gyro.y * 57.2958f;
    raw_gyro[2] = g.gyro.z * 57.2958f;
    raw_mag[0] = m.magnetic.x;
    raw_mag[1] = m.magnetic.y;
    raw_mag[2] = m.magnetic.z;

    // 陀螺仪直接透传（校准结构里没有gyro bias，由FusionBias在线估计）
    cal_gyro[0] = raw_gyro[0];
    cal_gyro[1] = raw_gyro[1];
    cal_gyro[2] = raw_gyro[2];

    // 加速度计和磁力计应用校准
    calibration_apply_accel(raw_accel, cal_accel);
    calibration_apply_mag(raw_mag, cal_mag);
    return true;
}
