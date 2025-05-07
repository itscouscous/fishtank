#include "Adafruit_VL53L0X.h"
#include "Adafruit_VL53L1X.h"

// address we will assign if dual sensor is present
#define LEFT_SENSOR_ADDRESS 0x30
#define RIGHT_SENSOR_ADDRESS 0x31
#define FRONT_SENSOR_ADDRESS 0x32

// set the pins to shutdown
#define SHT_FRONT_SENSOR 8
#define SHT_LEFT_SENSOR 7
#define SHT_RIGHT_SENSOR 6

#define SDA_PIN 5  // GPIO5 (D1 on NodeMCU)
#define SCL_PIN 4  // GPIO4 (D2 on NodeMCU)

// objects for the vl53l0x
Adafruit_VL53L0X left_lox = Adafruit_VL53L0X();
Adafruit_VL53L0X right_lox = Adafruit_VL53L0X();
Adafruit_VL53L1X front_lox = Adafruit_VL53L1X();


// this holds the measurement
VL53L0X_RangingMeasurementData_t left_measure;
VL53L0X_RangingMeasurementData_t right_measure;

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
  
  left_lox.rangingTest(&left_measure, false); // pass in 'true' to get debug data printout!
  right_lox.rangingTest(&right_measure, false); // pass in 'true' to get debug data printout!

  // print sensor one reading
  Serial.print("Left Sensor: ");
  if(left_measure.RangeStatus != 4) {     // if not out of range
    Serial.print(left_measure.RangeMilliMeter);
  } else {
    Serial.print("Out of range");
  }
  
  Serial.print(F(" "));

  // print sensor two reading
  Serial.print("RIght Sensor: ");
  if(right_measure.RangeStatus != 4) {
    Serial.print(right_measure.RangeMilliMeter);
  } else {
    Serial.print("Out of range");
  }

  //sensor three reading
  //if (front_lox.dataReady()) {
    // new measurement for the taking!
    int16_t distance;
    distance = front_lox.distance();
    if (distance == -1) {
      // something went wrong!
      Serial.print(F("Couldn't get distance: "));
      Serial.println(front_lox.vl_status);
      return;
    }
    Serial.print(F("front distance: "));
    Serial.print(distance);
    //Serial.println(" mm");

    // data is read out, time for another reading!
    front_lox.clearInterrupt();
  // }
  // else {
  //   Serial.println("Front not ready");
  // }
  
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




}

void loop() {
   
  read_sensors();
  delay(100);
}
