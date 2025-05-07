#ifndef MULTI_TOF_SENSOR_H
#define MULTI_TOF_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include "Adafruit_VL53L1X.h"

class MultiTOFSensor {
  public:
    // Constructor
    MultiTOFSensor(
      int shutdownPinLeft = 7,
      int shutdownPinRight = 6,
      int shutdownPinFront = 8,
      uint8_t leftAddress = 0x30,
      uint8_t rightAddress = 0x31,
      uint8_t frontAddress = 0x32
    );
    
    // Initialize with custom I2C pins (optional)
    bool begin(int sdaPin = -1, int sclPin = -1, uint32_t frequency = 400000);
    
    // Read sensors and get distances
    void readSensors();
    
    // Get individual sensor readings
    int getLeftDistance();
    int getRightDistance();
    int getFrontDistance();
    
    // Print readings to Serial
    void printReadings();
    
    // Check if sensors are out of range
    bool isLeftOutOfRange();
    bool isRightOutOfRange();
    bool isFrontOutOfRange();

  private:
    // Set I2C addresses for multiple sensors
    bool setI2CAddresses();
    
    // Pins and addresses
    int _shutdownPinLeft;
    int _shutdownPinRight;
    int _shutdownPinFront;
    uint8_t _leftAddress;
    uint8_t _rightAddress;
    uint8_t _frontAddress;
    
    // Sensor objects
    Adafruit_VL53L0X _leftLox;
    Adafruit_VL53L0X _rightLox;
    Adafruit_VL53L1X _frontLox;
    
    // Measurement data
    VL53L0X_RangingMeasurementData_t _leftMeasure;
    VL53L0X_RangingMeasurementData_t _rightMeasure;
    int16_t _frontDistance;
    
    // Status flags
    bool _sensorsInitialized;
    bool _leftOutOfRange;
    bool _rightOutOfRange;
    bool _frontOutOfRange;
};

#endif