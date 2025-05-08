#include "Adafruit_VL53L0X.h"
#include "Adafruit_VL53L1X.h"

#define I2C_SLAVE_ADDR 0x28
// address we will assign if dual sensor is present
#define LEFT_SENSOR_ADDRESS 0x30
#define RIGHT_SENSOR_ADDRESS 0x31
#define FRONT_SENSOR_ADDRESS 0x32

// set the pins to shutdown
#define SHT_FRONT_SENSOR 12
#define SHT_LEFT_SENSOR 11
#define SHT_RIGHT_SENSOR 13

#define SDA_PIN 8  // GPIO5 (D1 on NodeMCU)
#define SCL_PIN 9  // GPIO4 (D2 on NodeMCU)

// objects for the sensors
Adafruit_VL53L1X left_lox = Adafruit_VL53L1X();
Adafruit_VL53L0X right_lox = Adafruit_VL53L0X();
Adafruit_VL53L1X front_lox = Adafruit_VL53L1X();

// this holds the measurement for VL53L0X
VL53L0X_RangingMeasurementData_t right_measure;

//---------tophat code--------
void send_I2C_byte(uint8_t data) {
  // Send data to slave
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write(data);  // Send some test data
  uint8_t error = Wire.endTransmission();

  if (error == 0) {
    Serial.println("Data sent successfully");
  } else {
    Serial.printf("Error sending data: %d\n", error);
  }
}

uint8_t receive_I2C_byte() {  // data should have space declared from caller
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
  } else {
    Serial.println("No data received from slave");
    return 0;
  }
  return byteIn;  // return number of bytes read
}

//----------tof code----------------

void set_i2C_address() {
  // all reset
  digitalWrite(SHT_LEFT_SENSOR, LOW);    
  digitalWrite(SHT_RIGHT_SENSOR, LOW);
  digitalWrite(SHT_FRONT_SENSOR, LOW);
  delay(10);
  
  // Activate and configure sensors one by one
  
  // Left sensor first
  digitalWrite(SHT_LEFT_SENSOR, HIGH);
  delay(10);
  if(!left_lox.begin(LEFT_SENSOR_ADDRESS, &Wire)) {
    Serial.println(F("Failed to boot left VL53L1X"));
    while(1); // Critical failure
  }
  
  // Right sensor second
  digitalWrite(SHT_RIGHT_SENSOR, HIGH);
  delay(10);
  if(!right_lox.begin(RIGHT_SENSOR_ADDRESS, &Wire)) {
    Serial.println(F("Failed to boot right VL53L0X"));
    while(1); // Critical failure
  }
  
  // Front sensor third
  digitalWrite(SHT_FRONT_SENSOR, HIGH);
  delay(10);
  if(!front_lox.begin(FRONT_SENSOR_ADDRESS, &Wire)) {
    Serial.println(F("Failed to boot front VL53L1X"));
    while(1); // Critical failure
  }
}

void read_sensors() {
  // Read left sensor (VL53L1X)
  int16_t left_distance = left_lox.distance();
  if (left_distance == -1) {
    Serial.print(F("Left sensor error: "));
    Serial.println(left_lox.vl_status);
  } else {
    Serial.print(F("Left Sensor: "));
    Serial.print(left_distance);
    Serial.print(F(" mm, "));
  }
  left_lox.clearInterrupt();
  
  // Read right sensor (VL53L0X)
  right_lox.rangingTest(&right_measure, false);
  Serial.print("Right Sensor: ");
  if(right_measure.RangeStatus != 4) {
    Serial.print(right_measure.RangeMilliMeter);
    Serial.print(F(" mm, "));
  } else {
    Serial.print("Out of range, ");
  }
  
  // Read front sensor (VL53L1X)
  int16_t front_distance = front_lox.distance();
  if (front_distance == -1) {
    Serial.print(F("Front sensor error: "));
    Serial.println(front_lox.vl_status);
  } else {
    Serial.print(F("Front Sensor: "));
    Serial.print(front_distance);
    Serial.print(F(" mm"));
  }
  front_lox.clearInterrupt();
  
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting program");

  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN, 400000);

  // Setup shutdown pins
  pinMode(SHT_LEFT_SENSOR, OUTPUT);
  pinMode(SHT_RIGHT_SENSOR, OUTPUT);
  pinMode(SHT_FRONT_SENSOR, OUTPUT);

  // All sensors in shutdown
  digitalWrite(SHT_LEFT_SENSOR, LOW);
  digitalWrite(SHT_RIGHT_SENSOR, LOW);
  digitalWrite(SHT_FRONT_SENSOR, LOW);
  delay(10);

  Serial.println(F("Initializing sensors..."));
  set_i2C_address();
  Serial.println("I2C addresses assigned");
  
  // Configure VL53L1X sensors timing budget
  if (!left_lox.startRanging()) {
    Serial.print(F("Couldn't start left sensor ranging: "));
    Serial.println(left_lox.vl_status);
  } else {
    left_lox.setTimingBudget(50);
    Serial.println(F("Left sensor ranging started"));
  }
  
  if (!front_lox.startRanging()) {
    Serial.print(F("Couldn't start front sensor ranging: "));
    Serial.println(front_lox.vl_status);
  } else {
    front_lox.setTimingBudget(50);
    Serial.println(F("Front sensor ranging started"));
  }
  
  Serial.println(F("Setup complete"));
}

void loop() {
  read_sensors();
  
  // Send data to slave device
  uint8_t num_packet = 3;
  send_I2C_byte(num_packet);
  uint8_t response = receive_I2C_byte();
  
  delay(1000);
}
