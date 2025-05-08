#include "TOFSensor.h"

TOFSensor::TOFSensor(uint8_t front_pin, uint8_t left_pin, uint8_t right_pin,
                     uint8_t sda_pin, uint8_t scl_pin,
                     uint8_t front_addr, uint8_t left_addr, uint8_t right_addr) {
  _sht_front_sensor = front_pin;
  _sht_left_sensor = left_pin;
  _sht_right_sensor = right_pin;
  _sda_pin = sda_pin;
  _scl_pin = scl_pin;
  _front_address = front_addr;
  _left_address = left_addr;
  _right_address = right_addr;
  
  _front_distance = -1;
  _left_distance = -1;
  _right_distance = -1;
}

bool TOFSensor::begin() {
  // Initialize I2C with custom pins
  Wire.begin(_sda_pin, _scl_pin, 400000);
  
  // Initialize sensor pins
  pinMode(_sht_front_sensor, OUTPUT);
  pinMode(_sht_left_sensor, OUTPUT);
  pinMode(_sht_right_sensor, OUTPUT);
  
  // All sensors off initially
  digitalWrite(_sht_front_sensor, LOW);
  digitalWrite(_sht_left_sensor, LOW);
  digitalWrite(_sht_right_sensor, LOW);
  delay(10);
  
  // Setup I2C addresses for each sensor
  if (!setI2CAddresses()) {
    return false;
  }
  
  // Start ranging for front sensor
  if (!_front_lox.startRanging()) {
    Serial.print(F("Couldn't start ranging for front sensor: "));
    Serial.println(_front_lox.vl_status);
    return false;
  }
  
  // Start ranging for left sensor
  if (!_left_lox.startRanging()) {
    Serial.print(F("Couldn't start ranging for left sensor: "));
    Serial.println(_left_lox.vl_status);
    return false;
  }
  
  // Set timing budget for sensors
  _front_lox.setTimingBudget(50);
  _left_lox.setTimingBudget(50);
  
  return true;
}

bool TOFSensor::setI2CAddresses() {
  // All reset
  digitalWrite(_sht_left_sensor, LOW);    
  digitalWrite(_sht_right_sensor, LOW);
  digitalWrite(_sht_front_sensor, LOW);
  delay(10);
  
  // All unreset
  digitalWrite(_sht_left_sensor, HIGH);
  digitalWrite(_sht_right_sensor, HIGH);
  digitalWrite(_sht_front_sensor, HIGH);
  delay(10);

  // Left sensor setup
  digitalWrite(_sht_left_sensor, HIGH);
  digitalWrite(_sht_right_sensor, LOW);
  digitalWrite(_sht_front_sensor, LOW);
  delay(10);
  
  if (!_left_lox.begin(_left_address, &Wire)) {
    Serial.println(F("Failed to boot left VL53L1X"));
    return false;
  }
  delay(10);
  
  // Right sensor setup
  digitalWrite(_sht_right_sensor, HIGH);
  digitalWrite(_sht_front_sensor, LOW);
  delay(10);
  
  if (!_right_lox.begin(_right_address, &Wire)) {
    Serial.println(F("Failed to boot right VL53L0X"));
    return false;
  }
  delay(10);
  
  // Front sensor setup
  digitalWrite(_sht_front_sensor, HIGH);
  delay(10);
  
  if (!_front_lox.begin(_front_address, &Wire)) {
    Serial.println(F("Failed to boot front VL53L1X"));
    return false;
  }
  delay(10);
  
  return true;
}

bool TOFSensor::readSensors() {
  // Read right sensor (VL53L0X)
  _right_lox.rangingTest(&_right_measure, false);
  if (_right_measure.RangeStatus != 4) {
    _right_distance = _right_measure.RangeMilliMeter;
  } else {
    _right_distance = -1; // Out of range
  }
  
  // Read left sensor (VL53L1X)
  _left_distance = _left_lox.distance();
  if (_left_distance == -1) {
    Serial.print(F("Couldn't get left distance: "));
    Serial.println(_left_lox.vl_status);
    return false;
  }
  _left_lox.clearInterrupt();
  
  // Read front sensor (VL53L1X)
  _front_distance = _front_lox.distance();
  if (_front_distance == -1) {
    Serial.print(F("Couldn't get front distance: "));
    Serial.println(_front_lox.vl_status);
    return false;
  }
  _front_lox.clearInterrupt();
  
  return true;
}

int16_t TOFSensor::getFrontDistance() {
  return _front_distance;
}

int16_t TOFSensor::getLeftDistance() {
  return _left_distance;
}

int16_t TOFSensor::getRightDistance() {
  return _right_distance;
}

void TOFSensor::printDistances() {
  Serial.print(F("Left Sensor: "));
  if (_left_distance != -1) {
    Serial.print(_left_distance);
  } else {
    Serial.print("Out of range");
  }
  
  Serial.print(F(" | Right Sensor: "));
  if (_right_distance != -1) {
    Serial.print(_right_distance);
  } else {
    Serial.print("Out of range");
  }
  
  Serial.print(F(" | Front Sensor: "));
  if (_front_distance != -1) {
    Serial.print(_front_distance);
  } else {
    Serial.print("Out of range");
  }
  
  Serial.println();
}