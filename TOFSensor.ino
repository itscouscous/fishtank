#include "TOFSensor.h"

// Define pins
#define SHT_FRONT_SENSOR 12
#define SHT_LEFT_SENSOR 11
#define SHT_RIGHT_SENSOR 13
#define SDA_PIN 8
#define SCL_PIN 9

// Define addresses
#define LEFT_SENSOR_ADDRESS 0x30
#define RIGHT_SENSOR_ADDRESS 0x31
#define FRONT_SENSOR_ADDRESS 0x32

// Create TOFSensors object
TOFSensors sensors(
  SHT_FRONT_SENSOR, SHT_LEFT_SENSOR, SHT_RIGHT_SENSOR,
  SDA_PIN, SCL_PIN,
  FRONT_SENSOR_ADDRESS, LEFT_SENSOR_ADDRESS, RIGHT_SENSOR_ADDRESS
);

void setup() {
  Serial.begin(115200);
  Serial.println("Starting TOF Sensors Example");
  
  // Initialize sensors
  if (!sensors.begin()) {
    Serial.println("Failed to initialize sensors!");
    while (1) delay(10);
  }
  
  Serial.println("Sensors initialized successfully");
}

void loop() {
  // Read all sensors
  if (sensors.readSensors()) {
    // Print the distances
    sensors.printDistances();
    
    // You can also access individual sensor readings
    int16_t frontDist = sensors.getFrontDistance();
    int16_t leftDist = sensors.getLeftDistance();
    int16_t rightDist = sensors.getRightDistance();
    
    // Do something with the distance values
    // For example, obstacle avoidance logic could go here
    
    // Add your custom code here
    
  } else {
    Serial.println("Error reading sensors");
  }
  
  delay(1000);
}