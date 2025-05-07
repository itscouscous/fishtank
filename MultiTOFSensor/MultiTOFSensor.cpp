#include "MultiTOFSensor.h"

// Constructor implementation
MultiTOFSensor::MultiTOFSensor(
  int shutdownPinLeft,
  int shutdownPinRight,
  int shutdownPinFront,
  uint8_t leftAddress,
  uint8_t rightAddress,
  uint8_t frontAddress
) {
  _shutdownPinLeft = shutdownPinLeft;
  _shutdownPinRight = shutdownPinRight;
  _shutdownPinFront = shutdownPinFront;
  _leftAddress = leftAddress;
  _rightAddress = rightAddress;
  _frontAddress = frontAddress;
  _sensorsInitialized = false;
  _leftOutOfRange = true;
  _rightOutOfRange = true;
  _frontOutOfRange = true;
  _frontDistance = -1;
}

// Initialize the sensors with custom I2C pins if provided
bool MultiTOFSensor::begin(int sdaPin, int sclPin, uint32_t frequency) {
  // Configure shutdown pins as outputs
  pinMode(_shutdownPinLeft, OUTPUT);
  pinMode(_shutdownPinRight, OUTPUT);
  pinMode(_shutdownPinFront, OUTPUT);
  
  // Initially set all sensors to shutdown state
  digitalWrite(_shutdownPinLeft, LOW);
  digitalWrite(_shutdownPinRight, LOW);
  digitalWrite(_shutdownPinFront, LOW);
  
  // Initialize I2C with custom pins if provided
  if (sdaPin >= 0 && sclPin >= 0) {
    Wire.begin(sdaPin, sclPin, frequency);
  } else {
    Wire.begin();
    // Set I2C frequency if supported by platform
    #if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32)
    Wire.setClock(frequency);
    #endif
  }
  
  // Set unique I2C addresses for each sensor
  if (!setI2CAddresses()) {
    Serial.println(F("Failed to initialize sensors"));
    return false;
  }
  
  // Configure front sensor (VL53L1X)
  if (!_frontLox.startRanging()) {
    Serial.print(F("Couldn't start ranging for front sensor: "));
    Serial.println(_frontLox.vl_status);
    return false;
  }
  
  // Set timing budget for front sensor
  _frontLox.setTimingBudget(50); // Valid values: 15, 20, 33, 50, 100, 200, 500ms
  
  _sensorsInitialized = true;
  return true;
}

// Set unique I2C addresses for each sensor
bool MultiTOFSensor::setI2CAddresses() {
  // Reset all sensors
  digitalWrite(_shutdownPinLeft, LOW);
  digitalWrite(_shutdownPinRight, LOW);
  digitalWrite(_shutdownPinFront, LOW);
  delay(10);
  
  // Enable all sensors
  digitalWrite(_shutdownPinLeft, HIGH);
  digitalWrite(_shutdownPinRight, HIGH);
  digitalWrite(_shutdownPinFront, HIGH);
  delay(10);
  
  // Initialize left sensor
  digitalWrite(_shutdownPinLeft, HIGH);
  digitalWrite(_shutdownPinRight, LOW);
  digitalWrite(_shutdownPinFront, LOW);
  delay(10);
  
  if (!_leftLox.begin(_leftAddress, &Wire)) {
    Serial.println(F("Failed to initialize left sensor"));
    return false;
  }
  
  // Initialize right sensor
  digitalWrite(_shutdownPinRight, HIGH);
  digitalWrite(_shutdownPinLeft, HIGH);
  digitalWrite(_shutdownPinFront, LOW);
  delay(10);
  
  if (!_rightLox.begin(_rightAddress, &Wire)) {
    Serial.println(F("Failed to initialize right sensor"));
    return false;
  }
  
  // Initialize front sensor
  digitalWrite(_shutdownPinFront, HIGH);
  digitalWrite(_shutdownPinLeft, HIGH);
  digitalWrite(_shutdownPinRight, HIGH);
  delay(10);
  
  if (!_frontLox.begin(_frontAddress, &Wire)) {
    Serial.println(F("Failed to initialize front sensor"));
    return false;
  }
  
  return true;
}

// Read all sensors
void MultiTOFSensor::readSensors() {
  if (!_sensorsInitialized) {
    return;
  }
  
  // Read left sensor
  _leftLox.rangingTest(&_leftMeasure, false);
  _leftOutOfRange = (_leftMeasure.RangeStatus == 4);
  
  // Read right sensor
  _rightLox.rangingTest(&_rightMeasure, false);
  _rightOutOfRange = (_rightMeasure.RangeStatus == 4);
  
  // Read front sensor
  _frontDistance = _frontLox.distance();
  _frontOutOfRange = (_frontDistance == -1);
  
  // Clear interrupt for front sensor for next reading
  _frontLox.clearInterrupt();
}

// Get distance from left sensor
int MultiTOFSensor::getLeftDistance() {
  return _leftOutOfRange ? -1 : _leftMeasure.RangeMilliMeter;
}

// Get distance from right sensor
int MultiTOFSensor::getRightDistance() {
  return _rightOutOfRange ? -1 : _rightMeasure.RangeMilliMeter;
}

// Get distance from front sensor
int MultiTOFSensor::getFrontDistance() {
  return _frontDistance;
}

// Print readings to Serial
void MultiTOFSensor::printReadings() {
  Serial.print("Left Sensor: ");
  if (!_leftOutOfRange) {
    Serial.print(_leftMeasure.RangeMilliMeter);
    Serial.print(" mm");
  } else {
    Serial.print("Out of range");
  }
  
  Serial.print(" | Right Sensor: ");
  if (!_rightOutOfRange) {
    Serial.print(_rightMeasure.RangeMilliMeter);
    Serial.print(" mm");
  } else {
    Serial.print("Out of range");
  }
  
  Serial.print(" | Front Sensor: ");
  if (!_frontOutOfRange) {
    Serial.print(_frontDistance);
    Serial.print(" mm");
  } else {
    Serial.print("Out of range");
  }
  
  Serial.println();
}

// Check if sensors are out of range
bool MultiTOFSensor::isLeftOutOfRange() {
  return _leftOutOfRange;
}

bool MultiTOFSensor::isRightOutOfRange() {
  return _rightOutOfRange;
}

bool MultiTOFSensor::isFrontOutOfRange() {
  return _frontOutOfRange;
}