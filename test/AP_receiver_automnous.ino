/*
 * sends text udp packet to TargetIP
 *  using station mode (needs router) and static IP
 */

//////// WIFI init ////////

#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid     = "ESP32-Controller"; 
const char* password = "esp32pass";  

IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

WiFiUDP UDPServer;

uint8_t udpBuffer[100];

//////// PINS & VARS init ////////

// x
#define WHEEL1_PWM 0
#define WHEEL1_DIR1 7
#define WHEEL1_DIR2 6

// y
#define WHEEL2_PWM 1
#define WHEEL2_DIR1 8
#define WHEEL2_DIR2 10

//Encoders
#define ENCODER_L 35
#define ENCODER_R 3
// #define MOTOR_PWM_PIN 0  // Output pin for PWM using LEDC
// #define PWM_FREQ 30     // 1 kHz
// #define PWM_RES_BITS 12     // 10-bit resolution: 0–1023
int target_RPM_L = 0;     // Desired RPM
int target_RPM_R = 0;     // Desired RPM


// PID constants
float Kp_L = 6.0;
float Ki_L = 10.0;
float Kd_L = 0.0;

float Kp_R = 3.0;
float Ki_R = 1.0;
float Kd_R = 0.0;


// PID variables
volatile int lastDirectionL = 0; // +1 = CW, -1 = CCW
volatile int lastDirectionR = 0; // +1 = CW, -1 = CCW

volatile unsigned long lastRotationTimeL = 0;
volatile unsigned long lastRotationTimeR = 0;

volatile int rpm_L = 0;
volatile int rpm_R = 0;


float lastError = 0;
float integral = 0;
unsigned long lastPIDUpdateTimeL = 0;
unsigned long lastPIDUpdateTimeR = 0;

unsigned long lastISRUpdateTimeL = 0;
unsigned long lastISRUpdateTimeR = 0;

float dutyL = 0;
float dutyR = 0;


// variables for PWM init
const int freq = 50;      // Frequency in Hz
const int resolution = 12; // 12-bit resolution (0-4095)

volatile uint16_t x = 0; //received wheel1 value from joystick
volatile uint16_t y = 0; //received wheel2 value from joystick

volatile int duty_y;
volatile int duty_x;

static unsigned long lastPrintL = 0;
static unsigned long lastPrintR = 0;

//ISR to calculate the RPM
void IRAM_ATTR handleEncoderL() {
  unsigned long now = micros();
  unsigned long dt = now - lastRotationTimeL;
  lastRotationTimeL = now;
  
  if (dt > 0) {
    rpm_L = (3.0 * 1000000.0) / dt; //20 ticks per revolution
  }
  else {
    rpm_L = 0.0; 
  }
  lastISRUpdateTimeL = now;
  
}

void IRAM_ATTR handleEncoderR() {
  unsigned long now = micros();
  unsigned long dt = now - lastRotationTimeR;
  lastRotationTimeR = now;

  if (dt > 0) {
    rpm_R = (3.0 * 1000000.0) / dt; //20 ticks per revolution
  }
  else {
    rpm_R = 0.0; 
  }
  lastISRUpdateTimeR = now;
  
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

  //using controller input to control target RPM
  target_RPM_L = duty_y;
  target_RPM_R = duty_x;
 
}

void set_target_RPM_L(int target_RPM_L) {
  volatile unsigned long now = micros();
  volatile unsigned long resetInterval = 2000000;
  // checks if the wheels hasn't moved in more than 2 seconds, if so sets the rpm to be 0
  if( (now-lastISRUpdateTimeL) >= resetInterval && (lastISRUpdateTimeL - now) >= resetInterval){
    rpm_L = 0.0;
    Serial.print("now ");
    Serial.print(now);
    Serial.print(" | lastISRUpdateTime ");
    Serial.println(lastISRUpdateTimeL);

    lastISRUpdateTimeL = now;

  }  

  if (micros() - lastPrintL >= 100000) {
    int currentRPM = rpm_L;
    // forward
    if(target_RPM_L > 0) {
      digitalWrite(WHEEL1_DIR1, HIGH);
      digitalWrite(WHEEL1_DIR2, LOW);
    }
    else {
      digitalWrite(WHEEL1_DIR1, LOW);
      digitalWrite(WHEEL1_DIR2, HIGH);
      currentRPM = -rpm_L; 
    }

    // PID calculation
    int16_t error = abs(target_RPM_L) - abs(currentRPM);
    error = abs(error);
    if (abs(error) < 1) {
      error = 0; 
    }
    Serial.print("error ");
    Serial.print(error);

    unsigned long now = micros();
    int deltaT = (now - lastPIDUpdateTimeL);
    integral += error * deltaT;
    float derivative = (error - lastError);
    float output = Kp_L * error + Ki_L * integral + Kd_L * derivative;
    dutyL += output;
    // caps the actual duty cycle to be within reasonable ranges
    if (dutyL > 4095)
    {
      dutyL = 4095;
    }
    else if (dutyL < 0)
    {
      dutyL = 0;
    }

    lastError = error;
    lastPIDUpdateTimeL = now;

    ledcWrite(WHEEL1_PWM, dutyL);

    Serial.print("RPM: ");
    Serial.print(currentRPM);
    Serial.print(",");
    Serial.print("target_RPM_L: ");
    Serial.println(target_RPM_L);

    lastPrintL = now;
  }

}

void set_target_RPM_R(int target_RPM_R) {
  volatile unsigned long now = micros();
  volatile unsigned long resetInterval = 2000000;
  //checks if the wheel hasn't moved in more than 2seconds, if so, sets the rpm to be 0
  if( (now-lastISRUpdateTimeR) >= resetInterval && (lastISRUpdateTimeR - now) >= resetInterval){
    rpm_R = 0.0;
    Serial.print("now ");
    Serial.print(now);
    Serial.print(" | lastISRUpdateTime ");
    Serial.println(lastISRUpdateTimeR);

    lastISRUpdateTimeR = now;

  }  

  //recalculates the PID look every 100ms
  if (micros() - lastPrintR >= 100000) {
    int currentRPM = rpm_R;
    // forward
    if(target_RPM_R > 0) {
      digitalWrite(WHEEL2_DIR1, HIGH);
      digitalWrite(WHEEL2_DIR2, LOW);
    }
    else {
      digitalWrite(WHEEL2_DIR1, LOW);
      digitalWrite(WHEEL2_DIR2, HIGH);
      currentRPM = -rpm_R; 
    }


    // PID calculation
    int16_t error = abs(target_RPM_R) - abs(currentRPM);
    error = abs(error);
    if (abs(error) < 1) {
      error = 0; 
    }
    Serial.print("error ");
    Serial.print(error);
    unsigned long now = micros();
    int deltaT = (now - lastPIDUpdateTimeR);
    integral += error * deltaT;
    float derivative = (error - lastError);
    float output = Kp_R * error + Ki_R * integral + Kd_R * derivative;
    dutyR += output;
    
    //hard caps the max and min duty values to be within acceptable ranges
    if (dutyR > 4095)
    {
      dutyR = 4095;
    }
    else if (dutyR < 0)
    {
      dutyR = 0;
    }


    lastError = error;
    lastPIDUpdateTimeR = now;
    //actually spins the motor
    ledcWrite(WHEEL2_PWM, dutyR);
    
    Serial.print("RPM: ");
    Serial.print(currentRPM);
    Serial.print(",");
    Serial.print("target_RPM_R: ");
    Serial.println(target_RPM_R);

    lastPrintR = now;
  }

}


void setup() {
  //sets up wifi
  Serial.begin(115200);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ssid, password);  // Start AP

  UDPServer.begin(2808);  // 2808 arbitrary UDP port#    

  // WHEEL1 init
  ledcAttach(WHEEL1_PWM, freq, resolution);
  pinMode(WHEEL1_DIR1, OUTPUT);
  pinMode(WHEEL1_DIR2, OUTPUT);

  // WHEEL2 init
  ledcAttach(WHEEL2_PWM, freq, resolution);
  pinMode(WHEEL2_DIR1, OUTPUT);
  pinMode(WHEEL2_DIR2, OUTPUT);

  pinMode(ENCODER_L, INPUT);
  pinMode(ENCODER_R, INPUT);
  //attatches the ISR to detect rising edges for both controllers
  attachInterrupt(digitalPinToInterrupt(ENCODER_L), handleEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R), handleEncoderR, RISING);
  Serial.println("testing serial setup");


}

void loop() {
  Serial.println("testing serial print");
  //handleUDPServer();
  //sets target RPM for both wheels
  set_target_RPM_L(60);
  set_target_RPM_L(60);
  delay(10);

  
}

