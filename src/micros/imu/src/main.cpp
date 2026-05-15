#include <Arduino.h>
#include <Arduino_LSM9DS1.h>
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
    Serial.begin(115200);
    delay(2000);

    Wire.begin();
    Wire.setClock(400000);

    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("ERR:MPU6050");
    } else {
        Serial.println("MPU6050 OK");
        mpu.setDLPFMode(MPU6050_DLPF_BW_42);
    }

    if (!IMU.begin()) {
        Serial.println("ERR:LSM9DS1");
    } else {
        Serial.println("LSM9DS1 OK");
    }

    Serial.println("READY");
}

void loop() {
    // ── LSM9DS1 (onboard) ────────────────────────────────────────────────────
    float lax = 0, lay = 0, laz = 0;
    float lgx = 0, lgy = 0, lgz = 0;

    if (IMU.accelerationAvailable()) IMU.readAcceleration(lax, lay, laz);
    if (IMU.gyroscopeAvailable())    IMU.readGyroscope(lgx, lgy, lgz);

    // ── MPU-6050 (external) ──────────────────────────────────────────────────
    int16_t axR, ayR, azR, gxR, gyR, gzR;
    mpu.getMotion6(&axR, &ayR, &azR, &gxR, &gyR, &gzR);

    float max_ = axR / 16384.0f;
    float may  = ayR / 16384.0f;
    float maz  = azR / 16384.0f;
    float mgx  = gxR / 131.0f;
    float mgy  = gyR / 131.0f;
    float mgz  = gzR / 131.0f;

    // ── Remap MPU-6050 axes to match LSM9DS1 orientation ─────────────────────
    float max_r =  may;   // MPU Y  → LSM X
    float may_r = -maz;   // MPU -Z → LSM Y
    float maz_r =  max_;  // MPU X  → LSM Z
    float mgx_r =  mgy;
    float mgy_r = -mgz;
    float mgz_r =  mgx;

    // Format: "D lax lay laz lgx lgy lgz max_r may_r maz_r mgx_r mgy_r mgz_r\n"
    Serial.print("D ");
    Serial.print(lax,   4); Serial.print(' ');
    Serial.print(lay,   4); Serial.print(' ');
    Serial.print(laz,   4); Serial.print(' ');
    Serial.print(lgx,   4); Serial.print(' ');
    Serial.print(lgy,   4); Serial.print(' ');
    Serial.print(lgz,   4); Serial.print(' ');
    Serial.print(max_r, 4); Serial.print(' ');
    Serial.print(may_r, 4); Serial.print(' ');
    Serial.print(maz_r, 4); Serial.print(' ');
    Serial.print(mgx_r, 4); Serial.print(' ');
    Serial.print(mgy_r, 4); Serial.print(' ');
    Serial.println(mgz_r, 4);

    delay(10); // 100 Hz
}