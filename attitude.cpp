#include "attitude.h"
#include "config.h"
#include "Fusion.h"

static FusionAhrs ahrs;
static FusionBias bias;
static bool initialised = false;

// 最近一帧传感器数据（reset时用于初始姿态对准）
static float last_accel[3] = {0.0f, 0.0f, 1.0f};
static float last_mag[3]   = {1.0f, 0.0f, 0.0f};

// 配置参数（与原代码一致）
static void apply_settings() {
    const FusionAhrsSettings settings = {
        .convention = FusionConventionNwu,
        .gain = 1.5f,
        .gyroscopeRange = 2000.0f,
        .accelerationRejection = 10.0f,
        .magneticRejection = 2.0f,
        .recoveryTriggerPeriod = (unsigned int)(5.0f * IMU_SAMPLE_RATE),
    };
    FusionAhrsSetSettings(&ahrs, &settings);
}

void attitude_init() {
    FusionAhrsInitialise(&ahrs);
    apply_settings();
    FusionAhrsSetHeading(&ahrs, 0.0f);

    FusionBiasInitialise(&bias);
    FusionBiasSettings biasSettings = fusionBiasDefaultSettings;
    biasSettings.sampleRate = IMU_SAMPLE_RATE;
    FusionBiasSetSettings(&bias, &biasSettings);

    initialised = true;
#ifdef _ENABLE_9AXIS
    Serial.println("[ATT] Fusion AHRS 初始化完成 (NWU, 9-axis)");
#else
    Serial.println("[ATT] Fusion AHRS 初始化完成 (NWU, 6-axis)");
#endif
}

void attitude_update(const float gyro[3], const float accel[3],
                     const float mag[3], float dt) {
    if (!initialised) return;

    // 保存最近一帧数据（reset时用于初始姿态对准）
    last_accel[0] = accel[0]; last_accel[1] = accel[1]; last_accel[2] = accel[2];
    last_mag[0] = mag[0];     last_mag[1] = mag[1];     last_mag[2] = mag[2];

    // 陀螺仪零偏自适应
    FusionVector gyro_vec = {gyro[0], gyro[1], gyro[2]};
    gyro_vec = FusionBiasUpdate(&bias, gyro_vec);

    FusionVector accel_vec = {accel[0], accel[1], accel[2]};
    FusionVector mag_vec = {mag[0], mag[1], mag[2]};

    FusionAhrsSetSamplePeriod(&ahrs, dt);

#ifdef _ENABLE_9AXIS
    // 9轴融合（带磁力计，yaw不漂移，但易受铁磁干扰）
    FusionAhrsUpdate(&ahrs, gyro_vec, accel_vec, mag_vec);
#else
    // 6轴融合（无磁力计，yaw会缓慢漂移，但抗干扰）
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyro_vec, accel_vec);
#endif
}

void attitude_reset() {
    // 用最近一帧加速度+磁力计做初始姿态对准
    // roll/pitch 来自加速度计（静止时加速度=重力方向）
    // yaw 来自磁力计（倾斜补偿后计算航向）

    float ax = last_accel[0], ay = last_accel[1], az = last_accel[2];

    // roll/pitch（弧度）
    float roll_rad  = atan2f(ay, az);
    float pitch_rad = atan2f(-ax, sqrtf(ay * ay + az * az));

    // 倾斜补偿磁力计
    float cos_p = cosf(pitch_rad), sin_p = sinf(pitch_rad);
    float cos_r = cosf(roll_rad),  sin_r = sinf(roll_rad);
    float mx = last_mag[0] * cos_p + last_mag[1] * sin_r * sin_p - last_mag[2] * cos_r * sin_p;
    float my = last_mag[1] * cos_r + last_mag[2] * sin_r;
    float yaw_rad = atan2f(-my, mx);

    // 转成度（仅用于日志打印）
    float roll_deg  = roll_rad  * 57.2958f;
    float pitch_deg = pitch_rad * 57.2958f;
    float yaw_deg   = yaw_rad   * 57.2958f;

    // 重置AHRS并设置初始四元数（手动从ZYX欧拉角构造）
    FusionAhrsRestart(&ahrs);
    apply_settings();

    // ZYX欧拉角 -> 四元数（用弧度计算）
    float cr = cosf(roll_rad * 0.5f),  sr = sinf(roll_rad * 0.5f);
    float cp = cosf(pitch_rad * 0.5f), sp = sinf(pitch_rad * 0.5f);
    float cy = cosf(yaw_rad * 0.5f),   sy = sinf(yaw_rad * 0.5f);
    FusionQuaternion q = {
        .array = {
            cr * cp * cy + sr * sp * sy,  // w
            sr * cp * cy - cr * sp * sy,  // x
            cr * sp * cy + sr * cp * sy,  // y
            cr * cp * sy - sr * sp * cy   // z
        }
    };
    FusionAhrsSetQuaternion(&ahrs, q);

    // 重置陀螺仪零偏估计
    FusionBiasInitialise(&bias);
    FusionBiasSettings biasSettings = fusionBiasDefaultSettings;
    biasSettings.sampleRate = IMU_SAMPLE_RATE;
    FusionBiasSetSettings(&bias, &biasSettings);

    Serial.printf("[ATT] 姿态复位(初始对准): roll=%.1f pitch=%.1f yaw=%.1f deg\n",
                  roll_deg, pitch_deg, yaw_deg);
}

Quaternion attitude_get_quaternion() {
    FusionQuaternion q = FusionAhrsGetQuaternion(&ahrs);
    Quaternion result;
    result.w = q.array[0];
    result.x = q.array[1];
    result.y = q.array[2];
    result.z = q.array[3];
    return result;
}

EulerAngle attitude_get_euler() {
    FusionQuaternion q = FusionAhrsGetQuaternion(&ahrs);
    FusionEuler e = FusionQuaternionToEuler(q);
    EulerAngle result;
    result.roll = e.angle.roll;
    result.pitch = e.angle.pitch;
    result.yaw = e.angle.yaw;
    return result;
}
