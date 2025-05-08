#define SERVO_PIN 10  // Define the pin connected to the servo

void setup() {
  pinMode(SERVO_PIN, OUTPUT);
}

void loop() {
  // Sweep from 0° to 180°
  for (int angle = 0; angle <= 180; angle+=8) {
    int pulseWidth = map(angle, 0, 180, 500, 2500);  // Convert angle to pulse width
    moveServo(pulseWidth);
    delay(20);  // Delay between position updates
  }

  // Sweep back from 180° to 0°
  for (int angle = 180; angle >= 0; angle-=8) {
    int pulseWidth = map(angle, 0, 180, 500, 2500);
    moveServo(pulseWidth);
    delay(20);
  }
}

void moveServo(int pulseWidth) {
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(pulseWidth);  // Hold HIGH for pulse duration
  digitalWrite(SERVO_PIN, LOW);
  delay(20 - pulseWidth / 1000);  // Maintain ~20ms period
}