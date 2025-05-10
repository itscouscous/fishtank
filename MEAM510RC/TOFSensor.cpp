#include "TOFSensor.h"
#define INFINITE 8000

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
  Wire.begin(_sda_pin, _scl_pin, 40000);
  
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

  // Set timing budget for sensors
  _front_lox.setTimingBudget(20);
  _left_lox.setTimingBudget(20);

  // 20 ms ≈ 50 Hz continuous mode:
  _right_lox.setMeasurementTimingBudgetMicroSeconds(20000);  
  if (!_right_lox.startRangeContinuous(20)) {
    Serial.println("Couldn't start continuous on right sensor");
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
  // --- RIGHT (VL53L0X) ---
  uint16_t r = _right_lox.readRangeResult();
  _right_distance = (r != 0xFFFF) ? r : INFINITE;

  // --- LEFT (VL53L1X) ---
  int32_t d = _left_lox.distance();
  _left_distance = (d > 0) ? d : INFINITE;
  _left_lox.clearInterrupt();

  // --- FRONT (VL53L1X) ---
  d = _front_lox.distance();
  _front_distance = (d > 0) ? d : INFINITE;
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
    _right_distance = INFINITE; 
  }
  
  Serial.print(F(" | Front Sensor: "));
  if (_front_distance != -1) {
    Serial.print(_front_distance);
  } else {
    Serial.print("Out of range");
  }
  
  Serial.println();
}

// Function to receive data from slave device
uint8_t TOFSensor::receive_I2C_byte() {
  // Request data from slave
  uint8_t bytesReceived = Wire.requestFrom(I2C_SLAVE_ADDR, 1);
  uint8_t byteIn = 0;

  if (bytesReceived > 0) {
    Serial.print("Received from slave: ");
    while (Wire.available()) {
      byteIn = Wire.read();
      Serial.printf("0x%02X ", byteIn);
      Serial.printf(", %d HP", byteIn);
    }
    Serial.println();
  } else return 0;
  return byteIn;
}

void TOFSensor::send_I2C_byte(uint8_t data) {
  // Send data to slave
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write(data);  // Send data
  uint8_t error = Wire.endTransmission();

  if (error == 0) {
    Serial.println("Data sent successfully");
  } else {
    Serial.printf("Error sending data: %d\n", error);
  }
}