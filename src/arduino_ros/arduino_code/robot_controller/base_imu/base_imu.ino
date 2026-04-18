const int soundPin = 18; // Pin for ESP32

const int ledPin = 13;     /

void setup() {
  Serial.begin(115200);
  pinMode(soundPin, INPUT);
  pinMode(ledPin, OUTPUT);
  Serial.println("3-Pin Sound Sensor Test Active...");
}

void loop() {
  // Most 3-pin sensors go LOW when they hear sound
  int sensorState = digitalRead(soundPin);

  if (sensorState == LOW) { 
    digitalWrite(ledPin, HIGH);
    Serial.println("Sound Detected!");
    delay(200); // Small pause to see the LED flash
  } else {
    digitalWrite(ledPin, LOW);
  }
}
