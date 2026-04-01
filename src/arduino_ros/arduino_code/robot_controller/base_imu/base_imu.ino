#include <ArduinoJson.h>
#include <Arduino_LSM9DS1.h>

unsigned long lastIMUTime = 0;
const long imuInterval = 50;

void setup(){

  Serial.begin(115200);
  // Remove while(!Serial) if you want it to run without a PC attached
  if (!IMU.begin()) { /* blink LED or something */ }
}

void loop(){
  unsigned long currentTime = millis();
  if(currentTime - lastIMUTime >= imuInterval){
    lastIMUTime = currentTime;
    sendIMUData();
  }
}

void sendIMUData(){
  float ax, ay, az, gx, gy, gz;
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    StaticJsonDocument<200> imuDoc;
    imuDoc["ax"] = ax;
    imuDoc["ay"] = ay;
    imuDoc["az"] = az;
    imuDoc["gx"] = gx;
    imuDoc["gy"] = gy;
    imuDoc["gz"] = gz;

    serializeJson(imuDoc, Serial);
    Serial.println(); 
  }
}
