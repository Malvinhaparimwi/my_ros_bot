// Motors Pins
const int in1 = 7;
const int in2 = 8;
const int in3 = 11;
const int in4 = 10;

const int ena = 9;
const int enb = 12;

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
      driveRobot();
    } else if(command == '0') {
      stopRobot();
    }
  }
}

void driveRobot(){
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  analogWrite(ena,200);
  
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
  analogWrite(enb,200);
}

void stopRobot(){
  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
  
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);
}
