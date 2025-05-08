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

// objects for the vl53l0x
Adafruit_VL53L1X left_lox = Adafruit_VL53L1X();
Adafruit_VL53L0X right_lox = Adafruit_VL53L0X();
Adafruit_VL53L1X front_lox = Adafruit_VL53L1X();


// this holds the measurement
//VL53L0X_RangingMeasurementData_t left_measure;
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
      // if HP == 0, stop while(HP == 0){ receive bytes? }
    }
    Serial.println();
  } else return 0;
  return byteIn;  // return number of bytes read
}

//----------tof code----------------

void set_i2C_address() {
  // all reset
  digitalWrite(SHT_LEFT_SENSOR, LOW);    
  digitalWrite(SHT_RIGHT_SENSOR, LOW);
  digitalWrite(SHT_FRONT_SENSOR, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LEFT_SENSOR, HIGH);
  digitalWrite(SHT_RIGHT_SENSOR, HIGH);
  digitalWrite(SHT_FRONT_SENSOR, HIGH);
  delay(10);

  // left sensor
  digitalWrite(SHT_LEFT_SENSOR, HIGH);
  digitalWrite(SHT_RIGHT_SENSOR, LOW);
  digitalWrite(SHT_FRONT_SENSOR, LOW);
  
  if(!left_lox.begin(LEFT_SENSOR_ADDRESS, &Wire)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }

  delay(10);
  
  // right sensor
  digitalWrite(SHT_RIGHT_SENSOR, HIGH);
  digitalWrite(SHT_FRONT_SENSOR, LOW);
  digitalWrite(SHT_LEFT_SENSOR, HIGH);

  delay(10);

  if(!right_lox.begin(RIGHT_SENSOR_ADDRESS, &Wire)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }

  // front sensor
  digitalWrite(SHT_FRONT_SENSOR, HIGH);
  digitalWrite(SHT_RIGHT_SENSOR, HIGH);
  digitalWrite(SHT_LEFT_SENSOR, HIGH);

  delay(10);

  if(!front_lox.begin(FRONT_SENSOR_ADDRESS, &Wire)) {
    Serial.println(F("Failed to boot thrid VL53L0X"));
    while(1);
  }
  delay(10);

}

void read_sensors() {
  
  //left_lox.rangingTest(&left_measure, false); // pass in 'true' to get debug data printout!
  right_lox.rangingTest(&right_measure, false); // pass in 'true' to get debug data printout!

  // print sensor one reading
  // Serial.print("Left Sensor: ");
  // if(left_measure.RangeStatus != 4) {     // if not out of range
  //   Serial.print(left_measure.RangeMilliMeter);
  // } else {
  //   Serial.print("Out of range");
  // }
  
  // Serial.print(F(" "));
  int16_t Ldistance = left_lox.distance();
  if (Ldistance == -1) {
    // something went wrong!
    Serial.print(F("Couldn't get distance: "));
    Serial.println(left_lox.vl_status);
    return;
  }
  Serial.print(F("left Sensor: "));
  Serial.print(Ldistance);

  // data is read out, time for another reading!
  left_lox.clearInterrupt();



  // print sensor two reading
  Serial.print("Right Sensor: ");
  if(right_measure.RangeStatus != 4) {
    Serial.print(right_measure.RangeMilliMeter);
  } else {
    Serial.print("Out of range");
  }

  //sensor three reading

  // new measurement for the taking!
  int16_t distance;
  distance = front_lox.distance();
  if (distance == -1) {
    // something went wrong!
    Serial.print(F("Couldn't get distance: "));
    Serial.println(front_lox.vl_status);
    return;
  }
  Serial.print(F("Front Sensor: "));
  Serial.print(distance);

  // data is read out, time for another reading!
  front_lox.clearInterrupt();

  
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Serial.println("starting program");

  Wire.begin(SDA_PIN, SCL_PIN, 400000);  // Init I2C with custom pins





  pinMode(SHT_LEFT_SENSOR, OUTPUT);
  pinMode(SHT_RIGHT_SENSOR, OUTPUT);
  pinMode(SHT_FRONT_SENSOR, OUTPUT);


  digitalWrite(SHT_LEFT_SENSOR, LOW);
  digitalWrite(SHT_RIGHT_SENSOR, LOW);
  digitalWrite(SHT_FRONT_SENSOR, LOW);

  Serial.println(F("Starting..."));
  set_i2C_address();
  Serial.println("done with I2C address");
  


  Serial.print(F("Sensor ID: 0x"));
  Serial.println(front_lox.sensorID(), HEX);

  if (! front_lox.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(front_lox.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("Ranging started"));

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  front_lox.setTimingBudget(50);
  Serial.print(F("Timing budget (ms): "));
  Serial.println(front_lox.getTimingBudget());

  //-------left-------
  if (! left_lox.startRanging()) {
  Serial.print(F("Couldn't start ranging: "));
  Serial.println(left_lox.vl_status);
  while (1)       delay(10);
  }
  Serial.println(F("Ranging started"));

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  left_lox.setTimingBudget(50);
  Serial.print(F("Timing budget (ms): "));
  Serial.println(left_lox.getTimingBudget());

  //Wire.begin(I2C_SLAVE_ADDR); // becomes slave at 0x28
  // Wire.onReceive(receive_I2C_byte());
  // Wire.onRequest(send_I2C_byte(4));

}

void loop() {
  //unsigned long millisLast = millis(); 
  read_sensors();
  //2hz
  // if(millis() % 500 == 0) {
    uint8_t num_packet = 3;
    send_I2C_byte(num_packet);
    receive_I2C_byte();
  // }
  
  delay(1000);
}
