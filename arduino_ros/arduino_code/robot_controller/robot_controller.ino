// Motors Pins
const int in1 = 7;
const int in2 = 6;
const int in3 = 5;
const int in4 = 4;

const int ena = 9;
const int enb = 2;

void setup(){
  // Motors Pins Configuration
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // ROS TEST
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
}

void loop(){
  if(Serial.available()){
    char command = Serial.read();

    if(command == '1'){
      digitalWrite(LED_BUILTIN, HIGH);
    } else if(command == '0') {
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}

void driveRobot(){
  // For adjusting speeds
  analogWrite(ena, 125);
  analogWrite(enb, 125);

  // For Rotating the motors
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}
