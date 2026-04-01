#include <ArduinoJson.h>

const int in1 = 10; const int in2 = 9;
const int in3 = 8; const int in4 = 7;
const int ena = 12; const int enb = 11;

void setup(){
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  pinMode(ena, OUTPUT); pinMode(enb, OUTPUT);

  Serial.begin(9600);
}

void loop(){
  if (Serial.available() > 0) {
      // Read the JSON string
      String input = Serial.readStringUntil('\n');
  
      // Create a buffer for the JSON object
      StaticJsonDocument<200> doc;
      DeserializationError error = deserializeJson(doc, input);
  
      if (!error) {
        // ACCESS INDIVIDUALLY HERE
        float lx = doc["linear_x"];
        float az = doc["angular_z"];

        driveRobot(lx, az);
      } 
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
