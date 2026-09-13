#include "calibration.h"
#include <Preferences.h>

static Preferences prefs;
static const char* NVS_NAMESPACE = "imu_cal";

static CalibrationParams calib = {
    .accel_bias = {0, 0, 0},
    .accel_transform = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
    .mag_bias = {0, 0, 0},
    .mag_transform = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
    .valid = false
};

void calibration_init() {
    // 空操作，Preferences在load/save时按需begin
}

bool calibration_load() {
    prefs.begin(NVS_NAMESPACE, true);
    if (!prefs.isKey("valid") || !prefs.getBool("valid", false)) {
        calibration_reset_default();
        prefs.end();
        Serial.println("[CAL] 使用默认校准参数");
        return false;
    }
    calib.valid = true;
    for (int i = 0; i < 3; i++)
        calib.accel_bias[i] = prefs.getFloat(("acc_bias_" + String(i)).c_str(), 0.0f);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            calib.accel_transform[i][j] = prefs.getFloat(
                ("acc_t_" + String(i) + "_" + String(j)).c_str(), (i == j) ? 1.0f : 0.0f);
    for (int i = 0; i < 3; i++)
        calib.mag_bias[i] = prefs.getFloat(("mag_bias_" + String(i)).c_str(), 0.0f);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            calib.mag_transform[i][j] = prefs.getFloat(
                ("mag_t_" + String(i) + "_" + String(j)).c_str(), (i == j) ? 1.0f : 0.0f);
    prefs.end();
    Serial.println("[CAL] 校准参数加载成功");
    return true;
}

void calibration_save() {
    prefs.begin(NVS_NAMESPACE, false);
    for (int i = 0; i < 3; i++)
        prefs.putFloat(("acc_bias_" + String(i)).c_str(), calib.accel_bias[i]);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            prefs.putFloat(("acc_t_" + String(i) + "_" + String(j)).c_str(), calib.accel_transform[i][j]);
    for (int i = 0; i < 3; i++)
        prefs.putFloat(("mag_bias_" + String(i)).c_str(), calib.mag_bias[i]);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            prefs.putFloat(("mag_t_" + String(i) + "_" + String(j)).c_str(), calib.mag_transform[i][j]);
    prefs.putBool("valid", calib.valid);
    prefs.end();
    Serial.println("[CAL] 校准参数已保存");
}

void calibration_reset_default() {
    for (int i = 0; i < 3; i++) {
        calib.accel_bias[i] = 0;
        calib.mag_bias[i] = 0;
        for (int j = 0; j < 3; j++) {
            calib.accel_transform[i][j] = (i == j) ? 1.0f : 0.0f;
            calib.mag_transform[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    calib.valid = false;
}

void calibration_apply_accel(const float raw[3], float cal[3]) {
    float tmp[3] = {raw[0] - calib.accel_bias[0],
                     raw[1] - calib.accel_bias[1],
                     raw[2] - calib.accel_bias[2]};
    cal[0] = calib.accel_transform[0][0]*tmp[0] + calib.accel_transform[0][1]*tmp[1] + calib.accel_transform[0][2]*tmp[2];
    cal[1] = calib.accel_transform[1][0]*tmp[0] + calib.accel_transform[1][1]*tmp[1] + calib.accel_transform[1][2]*tmp[2];
    cal[2] = calib.accel_transform[2][0]*tmp[0] + calib.accel_transform[2][1]*tmp[1] + calib.accel_transform[2][2]*tmp[2];
}

void calibration_apply_mag(const float raw[3], float cal[3]) {
    float tmp[3] = {raw[0] - calib.mag_bias[0],
                     raw[1] - calib.mag_bias[1],
                     raw[2] - calib.mag_bias[2]};
    cal[0] = calib.mag_transform[0][0]*tmp[0] + calib.mag_transform[0][1]*tmp[1] + calib.mag_transform[0][2]*tmp[2];
    cal[1] = calib.mag_transform[1][0]*tmp[0] + calib.mag_transform[1][1]*tmp[1] + calib.mag_transform[1][2]*tmp[2];
    cal[2] = calib.mag_transform[2][0]*tmp[0] + calib.mag_transform[2][1]*tmp[1] + calib.mag_transform[2][2]*tmp[2];
}

const CalibrationParams* calibration_get() {
    return &calib;
}

bool calibration_parse_from_string(const char* str) {
    float params[24];
    int idx = 0;
    char buf[512];
    strncpy(buf, str, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    char* p = strtok(buf, ",");
    while (p != NULL && idx < 24) {
        params[idx++] = atof(p);
        p = strtok(NULL, ",");
    }
    if (idx != 24) {
        Serial.printf("[CAL] 解析失败，需要24个参数，实际%d个\n", idx);
        return false;
    }
    int k = 0;
    for (int i = 0; i < 3; i++) calib.accel_bias[i] = params[k++];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            calib.accel_transform[i][j] = params[k++];
    for (int i = 0; i < 3; i++) calib.mag_bias[i] = params[k++];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            calib.mag_transform[i][j] = params[k++];
    calib.valid = true;
    calibration_save();
    Serial.println("[CAL] 校准参数已更新并保存");
    return true;
}
