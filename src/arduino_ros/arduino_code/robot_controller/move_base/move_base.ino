#include <ArduinoJson.h>
#include <Arduino_LSM9DS1.h>

const int in1 = 10; const int in2 = 9;
const int in3 = 8; const int in4 = 7;
const int ena = 12; const int enb = 11;

unsigned long lastIMUTime = 0;
const long imuInterval = 50;

void setup(){
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  pinMode(ena, OUTPUT); pinMode(enb, OUTPUT);

  Serial.begin(9600);
  // Remove while(!Serial) if you want it to run without a PC attached
  if (!IMU.begin()) { /* blink LED or something */ }
}

void loop(){
  // NON-BLOCKING SERIAL READ
  if (Serial.available() > 0) {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, Serial); // Read directly from Serial
  
    if (error == DeserializationError::Ok) {
      float lx = doc["linear_x"];
      float az = doc["angular_z"];
      driveRobot(lx, az);
    } 
  }

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

void driveRobot(float lx, float az) {
  if (abs(lx) < 0.01 && abs(az) < 0.01) {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
    digitalWrite(in3, LOW); digitalWrite(in4, LOW);
    analogWrite(ena, 0); analogWrite(enb, 0);
    return;
  }

  float linear_pwm  = (lx / 0.22) * 200.0;
  float angular_pwm = (az / 0.22) * 200.0;
  float leftSpeed  = linear_pwm + angular_pwm;
  float rightSpeed = linear_pwm - angular_pwm;

  leftSpeed  = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  digitalWrite(in1, leftSpeed >= 0 ? HIGH : LOW);
  digitalWrite(in2, leftSpeed >= 0 ? LOW : HIGH);
  digitalWrite(in3, rightSpeed >= 0 ? HIGH : LOW);
  digitalWrite(in4, rightSpeed >= 0 ? LOW : HIGH);

  analogWrite(ena, abs(leftSpeed));
  analogWrite(enb, abs(rightSpeed));
}
