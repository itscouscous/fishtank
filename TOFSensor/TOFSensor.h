#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include "Adafruit_VL53L1X.h"

class TOFSensor {
  public:
    // Constructor
    TOFSensor(uint8_t front_pin, uint8_t left_pin, uint8_t right_pin, 
               uint8_t sda_pin, uint8_t scl_pin,
               uint8_t front_addr, uint8_t left_addr, uint8_t right_addr);
    
    // Initialization
    bool begin();
    
    // Read sensors
    bool readSensors();
    
    // Get distance values
    int16_t getFrontDistance();
    int16_t getLeftDistance();
    int16_t getRightDistance();
    
    // Debug output
    void printDistances();
    
  private:
    // Sensor pins
    uint8_t _sht_front_sensor;
    uint8_t _sht_left_sensor;
    uint8_t _sht_right_sensor;
    uint8_t _sda_pin;
    uint8_t _scl_pin;
    
    // Sensor addresses
    uint8_t _front_address;
    uint8_t _left_address;
    uint8_t _right_address;
    
    // Sensor objects
    Adafruit_VL53L1X _front_lox;
    Adafruit_VL53L1X _left_lox;
    Adafruit_VL53L0X _right_lox;
    
    // Distance measurements
    int16_t _front_distance;
    int16_t _left_distance;
    int16_t _right_distance;
    
    // Measurement data structure for VL53L0X
    VL53L0X_RangingMeasurementData_t _right_measure;
    
    // Private methods
    bool setI2CAddresses();
};

#endif
