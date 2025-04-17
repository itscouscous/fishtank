/*
 * sends text udp packet to TargetIP
 *  using station mode (needs router) and static IP
 */
#include <WiFi.h>
#include <WiFiUdp.h>


const char* ssid     = "ESP32-Controller"; 
const char* password = "esp32pass";  

IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

#define WHEEL1_PWM 0
#define WHEEL1_DIR1 7
#define WHEEL1_DIR2 6
//y
#define WHEEL2_PWM 1
#define WHEEL2_DIR1 8
#define WHEEL2_DIR2 10

#define ENCODER_A 35
// #define ENCODER_B 5
// #define MOTOR_PWM_PIN 0  // Output pin for PWM using LEDC
// #define PWM_FREQ 30     // 1 kHz
// #define PWM_RES_BITS 12     // 10-bit resolution: 0–1023
int target_RPM_L = 0;     // Desired RPM

const int PPR = 3; // Pulses per revolution from A channel

// PID constants
float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.0;

// PID variables
volatile int lastDirection = 0; // +1 = CW, -1 = CCW
volatile unsigned long lastRotationTime = 0;
volatile int rpm = 0;
volatile int pulseCount = 0;

float lastError = 0;
float integral = 0;
unsigned long lastPIDUpdateTime = 0;
unsigned long lastISRUpdateTime = 0;
float duty = 0;


// variables for PWM init
const int freq = 50;      // Frequency in Hz
const int resolution = 12; // 12-bit resolution (0-4095)



WiFiUDP UDPServer;


uint8_t udpBuffer[100];

volatile uint16_t x = 0; 
volatile uint16_t y = 0; 

volatile int duty_y;
volatile int duty_x;

void IRAM_ATTR handleEncoderA() {
  //pulseCount++;

  // Direction
  //lastDirection = digitalRead(ENCODER_B) ? 1 : -1;

  unsigned long now = micros();
  unsigned long dt = now - lastRotationTime;
  lastRotationTime = now;

  if (dt > 0) {
    rpm = (3.0 * 1000000.0) / dt; //20 ticks per revolution
  }
  else {
    rpm = 0.0; 
  }
  lastISRUpdateTime = now;
  
}

int abs(int num) {
  if(num < 0) {
    return -num;
  }
  return num;
}

void handleUDPServer() {
  uint8_t packetBuffer[100];
  int i, cb = UDPServer.parsePacket();
  if (cb) {
    UDPServer.read(packetBuffer, 100);
    i = packetBuffer[0] + (packetBuffer[1] << 8); // puts 2 bytes into int
    uint32_t Duty;
    if(i >= 32768) {
      y = i - 32768;
    }
    else {
      x = i; 
    }
    makeCarGo(x, y); 
  }
}

void makeCarGo(int x, int y){
  //make potentiometer values centered around 0, and scale them up by 2 so that each half is 0~4064, which is what the Dutycycle range is
  duty_x = (x - 2048)/10; 
  duty_y = (y - 2048)/10;


  if(duty_x > 5){
    digitalWrite(WHEEL2_DIR2, HIGH);
    digitalWrite(WHEEL2_DIR1, LOW);
  }else if(duty_x < -5){
    duty_x = - duty_x;
    digitalWrite(WHEEL2_DIR1, HIGH);
    digitalWrite(WHEEL2_DIR2, LOW);
  }else{
    duty_x = 0; 
    digitalWrite(WHEEL2_DIR2, LOW);
    digitalWrite(WHEEL2_DIR1, LOW);
  }

  if(duty_y > 5){
    digitalWrite(WHEEL1_DIR2, HIGH);
    digitalWrite(WHEEL1_DIR1, LOW);
  }else if(duty_y < -5){
    duty_y = -duty_y;
    digitalWrite(WHEEL1_DIR1, HIGH);
    digitalWrite(WHEEL1_DIR2, LOW);
  }else{
    duty_y = 0; 
    digitalWrite(WHEEL1_DIR2, LOW);
    digitalWrite(WHEEL1_DIR1, LOW);
  }

  // Serial.print("\n target_RPM_L:");
  // Serial.println(target_RPM_L);
  
  target_RPM_L = duty_y;
  set_target_RPM_L(target_RPM_L);


  // ledcWrite(WHEEL1_PWM, duty_y);
  // ledcWrite(WHEEL2_PWM, duty_x);
 
}

void set_target_RPM_L(int target_RPM_L) {
  volatile unsigned long now = micros();
  volatile unsigned long resetInterval = 2000000;

  if( (now-lastISRUpdateTime) >= resetInterval && (lastISRUpdateTime - now) >= resetInterval){
    rpm = 0.0;
    Serial.print("now ");
    Serial.print(now);
    Serial.print(" | lastISRUpdateTime ");
    Serial.println(lastISRUpdateTime);

    lastISRUpdateTime = now;

  }  

  static unsigned long lastPrint = 0;

  if (micros() - lastPrint >= 100000) {
    int currentRPM = rpm;
    // forward
    if(target_RPM_L > 0) {
      digitalWrite(WHEEL1_DIR1, HIGH);
      digitalWrite(WHEEL1_DIR2, LOW);
    }
    else {
      digitalWrite(WHEEL1_DIR1, LOW);
      digitalWrite(WHEEL1_DIR2, HIGH);
      currentRPM = -rpm; 
    }

    
    //int dir = lastDirection;

    // PID calculation
    int16_t error = abs(target_RPM_L) - abs(currentRPM);
    error = abs(error);
    if (abs(error) < 1) {
      error = 0; 
    }
    Serial.print("error ");
    Serial.print(error);

    unsigned long now = micros();
    int deltaT = (now - lastPIDUpdateTime);
    integral += error * deltaT;
    float derivative = (error - lastError);
    float output = Kp * error + Ki * integral + Kd * derivative;
    duty += output;
    if (duty > 4095)
    {
      duty = 4095;
    }
    else if (duty < 0)
    {
      duty = 0;
    }


    lastError = error;
    lastPIDUpdateTime = now;



    ledcWrite(WHEEL1_PWM, duty);


    // Debug print
    // Serial.print(" | output ");
    // Serial.print(output);
    Serial.print("RPM: ");
    Serial.print(currentRPM);
    // Serial.print(" | PWM: ");
    // Serial.println(duty);
    Serial.print(",");
    Serial.print("target_RPM_L: ");
    Serial.println(target_RPM_L);

    lastPrint = now;
  }

}


void setup() {
  Serial.begin(115200);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ssid, password);  // Start AP


  
  UDPServer.begin(2808);  // 2808 arbitrary UDP port#    

  ledcAttach(WHEEL1_PWM, freq, resolution);
  pinMode(WHEEL1_DIR1, OUTPUT);
  pinMode(WHEEL1_DIR2, OUTPUT);


  ledcAttach(WHEEL2_PWM, freq, resolution);
  pinMode(WHEEL2_DIR1, OUTPUT);
  pinMode(WHEEL2_DIR2, OUTPUT);

  pinMode(ENCODER_A, INPUT);
  //pinMode(ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), handleEncoderA, RISING);

  // Set up LEDC PWM
  // ledcAttach(MOTOR_PWM_PIN, PWM_FREQ, PWM_RES_BITS);

  // pinMode(WHEEL1DIR1, OUTPUT);
  // pinMode(WHEEL1DIR2, OUTPUT);
}

void loop() {
  handleUDPServer();
  delay(10);

  
}
