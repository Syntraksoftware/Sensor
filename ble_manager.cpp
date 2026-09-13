#include "ble_manager.h"
#include "config.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

static BLECharacteristic *pCharInfo, *pCharRaw, *pCharCal, *pCharQuat, *pCharWrite;
static bool deviceConnected = false;
static ble_command_callback_t command_cb = NULL;

// ============ BLE回调 ============

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        uint16_t connId = pServer->getConnId();
        pServer->updatePeerMTU(connId, 300);
        Serial.printf("[BLE] 已连接, MTU=%d\n", pServer->getPeerMTU(connId));
    }
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        BLEDevice::startAdvertising();
        Serial.println("[BLE] 已断开，重新广播");
    }
};

class WriteCallback : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String value = pCharacteristic->getValue();
        if (value.length() == 0) return;
        Serial.printf("[BLE] 收到命令: %s\n", value.c_str());
        if (command_cb) {
            command_cb(value.c_str());
        }
    }
};

// ============ 公开接口 ============

void ble_init(const char* device_name, uint32_t sensor_id) {
    BLEDevice::init(device_name);
    BLEServer *pServer = BLEDevice::createServer();
    BLEDevice::setMTU(300);
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(BLE_SERVICE_UUID);

    pCharRaw = pService->createCharacteristic(BLE_CHAR_RAW_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    pCharRaw->addDescriptor(new BLE2902());

    pCharCal = pService->createCharacteristic(BLE_CHAR_CAL_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    pCharCal->addDescriptor(new BLE2902());

    pCharQuat = pService->createCharacteristic(BLE_CHAR_QUAT_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    pCharQuat->addDescriptor(new BLE2902());

    pCharWrite = pService->createCharacteristic(BLE_CHAR_WRITE_UUID, BLECharacteristic::PROPERTY_WRITE);
    pCharWrite->setCallbacks(new WriteCallback());

    pCharInfo = pService->createCharacteristic(BLE_CHAR_INFO_UUID, BLECharacteristic::PROPERTY_READ);

    // 设备信息 JSON
    String infoJson = "{";
    infoJson += "\"serial\":\"" + String(SYNTRAK_DEVICE_SN) + "\",";
    infoJson += "\"IMU id\":\"" + String(sensor_id) + "\",";
    infoJson += "\"hardware\":\"" + String(SYNTRAK_HARDWARE_VERSION) + "\",";
    infoJson += "\"software\":\"" + String(SYNTRAK_SOFTWARE_VERSION) + "\"";
    infoJson += "}";
    pCharInfo->setValue(infoJson.c_str());

    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();

    Serial.printf("[BLE] 初始化完成，设备名: %s\n", device_name);
}

bool ble_is_connected() {
    return deviceConnected;
}

void ble_send_raw(const float accel[3], const float gyro[3], const float mag[3]) {
    if (!deviceConnected) return;
    char buf[200];
    snprintf(buf, sizeof(buf), "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
             accel[0], accel[1], accel[2],
             gyro[0], gyro[1], gyro[2],
             mag[0], mag[1], mag[2]);
    pCharRaw->setValue(buf);
    pCharRaw->notify();
}

void ble_send_calibrated(const float accel[3], const float gyro[3], const float mag[3]) {
    if (!deviceConnected) return;
    char buf[200];
    snprintf(buf, sizeof(buf), "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
             accel[0], accel[1], accel[2],
             gyro[0], gyro[1], gyro[2],
             mag[0], mag[1], mag[2]);
    pCharCal->setValue(buf);
    pCharCal->notify();
}

void ble_send_quaternion(float w, float x, float y, float z, bool stationary) {
    if (!deviceConnected) return;
    char buf[100];
    snprintf(buf, sizeof(buf), "%.4f,%.4f,%.4f,%.4f,%d",
             w, x, y, z, stationary ? 1 : 0);
    pCharQuat->setValue(buf);
    pCharQuat->notify();
}

void ble_set_command_callback(ble_command_callback_t cb) {
    command_cb = cb;
}
