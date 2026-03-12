#include <ArduinoJson.h>

// Motors Pins
const int in1 = 10;
const int in2 = 9;
const int in3 = 8;
const int in4 = 7;

const int ena = 12;
const int enb = 11;

void setup(){
  // Motors Pins Configuration
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // ROS TEST
  pinMode(LED_BUILTIN, OUTPUT);
  Serial1.begin(9600);
}

void loop(){
    if (Serial1.available() > 0) {
      // Read the JSON string
      String input = Serial1.readStringUntil('\n');
  
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

  // Stop Condition
  if (abs(lx) < 0.01 && abs(az) < 0.01) {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
    digitalWrite(in3, LOW); digitalWrite(in4, LOW);
    analogWrite(ena, 0);
    analogWrite(enb, 0);
    return;
  }

  // Map velocities to PWM while keeping sign
  float linear_pwm  = (lx / 0.22) * 200.0;
  float angular_pwm = (az / 0.22) * 200.0;

  // Differential drive
  float leftSpeed  = linear_pwm + angular_pwm;
  float rightSpeed = linear_pwm - angular_pwm;

  // Limit speeds
  leftSpeed  = constrain(leftSpeed,  -200, 200);
  rightSpeed = constrain(rightSpeed, -200, 200);

  // Left motor direction
  if (leftSpeed >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }

  // Right motor direction
  if (rightSpeed >= 0) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }

  // Send PWM
  analogWrite(ena, abs(leftSpeed));
  analogWrite(enb, abs(rightSpeed));
}
