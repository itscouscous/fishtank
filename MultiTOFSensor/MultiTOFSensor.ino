#include "MultiTOFSensor.h"

// Define pins for ESP8266/ESP32
#define SDA_PIN 5  // D1 on NodeMCU
#define SCL_PIN 4  // D2 on NodeMCU

// Define shutdown pins
#define SHT_LEFT_SENSOR 7
#define SHT_RIGHT_SENSOR 6
#define SHT_FRONT_SENSOR 8

// Create sensor object with default pins and addresses
MultiTOFSensor tofSensors(
  SHT_LEFT_SENSOR,
  SHT_RIGHT_SENSOR,
  SHT_FRONT_SENSOR
);

void setup() {
  Serial.begin(115200);
  Serial.println("Starting TOF sensors example");
  
  // Initialize sensors with custom I2C pins
  if (tofSensors.begin(SDA_PIN, SCL_PIN)) {
    Serial.println("Sensors initialized successfully");
  } else {
    Serial.println("Failed to initialize sensors");
    while (1) delay(10);
  }
}

void loop() {
  // Read all sensors
  tofSensors.readSensors();
  
  // Print readings
  tofSensors.printReadings();
  
  // Alternatively, get individual readings
  int leftDistance = tofSensors.getLeftDistance();
  int rightDistance = tofSensors.getRightDistance();
  int frontDistance = tofSensors.getFrontDistance();
  
  // Example of using individual readings for obstacle avoidance
  if (frontDistance > 0 && frontDistance < 200) {
    Serial.println("Obstacle detected in front!");
  }
  
  delay(100);
}