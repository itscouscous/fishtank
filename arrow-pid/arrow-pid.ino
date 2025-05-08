/*
 * Uses wifi station mode to 
 *  take x (trajectory) & y (direction) values using arrow keys
 *  to make car go
 */



//////// WIFI init ////////
 
#include "html510.h"
#include "body.h" // html and javascript (content defined in separate file)

HTML510Server h(80);

const char* ssid = "ArrowController";
const char* password = "";



//////// ARROW KEY var init ////////

int xAxis = 0;  // -1: left, 0: center, 1: right
int yAxis = 0;  // -1: down, 0: center, 1: up

int packetCount = 0; // track # of packets sent



//////// WHEELS pin & var init ////////

#include <DualEncoder.h>

#define ENCL_A 4
#define ENCL_B 5
#define ENCR_A 1
#define ENCR_B 2

#define L_EN 6
#define R_EN 42

#define L_Forward 7
#define L_Backward 15
#define R_Forward 41
#define R_Backward 40

// variables for PWM init
const int freq = 10000;      // Frequency in Hz
const int resolution = 10; // 12-bit resolution (0-4095)

int16_t target_RPM_L = 50;  // Desired target RPM
int16_t target_RPM_R = 50;

int32_t lastEncoderRead = 0;
int32_t LastPID = 0;
int32_t LastCountR = 0;
int32_t LastCountL = 0;
int16_t R_RPM = 0;
int16_t L_RPM = 0;

int pwmOutput = 0;

float duty_L = 0.0;
float duty_R = 0.0;

DualEncoder encoders(ENCR_A, ENCR_B, ENCL_A, ENCL_B);  // AR, BR, AL, BL



//////// ARROW KEY Functions ////////

void arrow_Setup(){

  Serial.println("Starting Arrow Key Controller");
  
  // WiFi setup
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, ""); // no password
  Serial.println(ssid);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
  
  // Initialize web server
  h.begin();
  h.attachHandler("/ ", handleRoot); // default page
  h.attachHandler("/cmd?val=", handleCommand); // arrow key command
  
}

void handleRoot() {
  h.sendhtml(body);
}

void handleCommand() {
  String command = h.getText();
  
  // Parse x,y values from the command string (format: "x:-1,y:0")
  int xIndex = command.indexOf("x:");
  int yIndex = command.indexOf("y:");
  int commaIndex = command.indexOf(",");
  
  if (xIndex >= 0 && yIndex >= 0 && commaIndex >= 0) {
    // Extract x value
    String xString = command.substring(xIndex + 2, commaIndex);
    // Extract y value
    String yString = command.substring(yIndex + 2);
    
    // Convert to integers
    int newXAxis = xString.toInt();
    int newYAxis = yString.toInt();
    
    // Only update if values are valid (-1, 0, or 1)
    if (newXAxis >= -1 && newXAxis <= 1 && newYAxis >= -1 && newYAxis <= 1) {
      xAxis = newXAxis;
      yAxis = newYAxis;
      
      // Print to serial for debugging
      Serial.print("L/R: ");
      Serial.print(xAxis);
      Serial.print(" F/B: ");
      Serial.println(yAxis);
      
      packetCount++; // increment # of packets sent
      Serial.println(packetCount);
    }
  }
  
  // Send response back to client
  String response = "Received: x:" + String(xAxis) + ",y:" + String(yAxis);
  h.sendplain(response);
}



//////// WHEEL Functions ////////

void wheel_Setup(){

  Serial.begin(115200);
  encoders.begin();

  // Wheel L
  ledcAttach(L_Forward, freq, resolution);
  ledcAttach(L_Backward, freq, resolution);
  pinMode(L_EN, OUTPUT);
  digitalWrite(L_EN, HIGH);
  
  // Wheel R
  ledcAttach(R_Forward, freq, resolution);
  ledcAttach(R_Backward, freq, resolution);
  pinMode(R_EN, OUTPUT);
  digitalWrite(R_EN, HIGH);

}

struct myPID {
  float Kp, Ki, Kd;
  float integral = 0;
  float lastError = 0;

  myPID(float p, float i, float d) : Kp(p), Ki(i), Kd(d) {}
};

myPID LmotorPID(2, 0.0, 0.06);
myPID RmotorPID(2, 0.0, 0.06);

float calc_PID(myPID &pid, int32_t target, int32_t measured) {
  float error = target - measured;
  pid.integral += error * 0.04;
  float derivative = (error - pid.lastError) / 0.04;
  pid.lastError = error;

  float PID_output = (pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative);

  return PID_output;
}

void Sample_RPM(){
  lastEncoderRead = micros();
  R_RPM = (encoders.getCountR() - LastCountR)*100*60/30/64;
  L_RPM = (encoders.getCountL() - LastCountL)*100*60/30/64;
  LastCountR = encoders.getCountR();
  LastCountL = encoders.getCountL();
}


void set_PWM(char dir, int16_t RPM) {
  // dir is Left or Right wheel
  // RPM can be negative


  if (dir == 'L'){
    if (RPM > 0){
      ledcWrite(L_Backward, 0);
      ledcWrite(L_Forward, RPM);
    }
    else{
      ledcWrite(L_Forward, 0);
      ledcWrite(L_Backward, -RPM);
    }
  }else if (dir == 'R'){
    if (RPM > 0){
      ledcWrite(R_Forward, RPM);
      ledcWrite(R_Backward, 0);
    }
    else{
      ledcWrite(R_Forward, 0);
      ledcWrite(R_Backward, -RPM);
    }
  }
}

void makeWheelsGo(){

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    target_RPM_L = input.toInt();  // Convert typed input to int
    target_RPM_R = input.toInt();
  }

  //Encoder read at 100Hz
  if (micros() - lastEncoderRead > 10000) Sample_RPM();


  //PID runs at 25Hz
  if (micros() - LastPID > 40000){
    LastPID = micros();
    duty_L += calc_PID(LmotorPID, target_RPM_L, L_RPM);
    duty_R += calc_PID(RmotorPID, target_RPM_R, R_RPM);
    
    //Clamp
    if (duty_L >= 1024) duty_L = 1024;
    else if (duty_L <= -1024) duty_L = -1024;

    if (duty_R >= 1024) duty_R = 1024;
    else if (duty_R <= -1024) duty_R = -1024;

    set_PWM('L', (int16_t)duty_L);
    set_PWM('R', (int16_t)duty_R);
    
  }

  // Serial.print(350);
  // Serial.print(",");
  // Serial.print(-350);
  // Serial.print(",");
  // Serial.print(target_RPM);
  // Serial.print(",");
  // Serial.print(pwmOutput);
  // Serial.print(",");
  // Serial.print(L_RPM);
  // Serial.print(",");
  // Serial.println(R_RPM);

}

void makeCarGo(){
  // MAKE CAR GO based on xAxis & yAxis values

  // xAxis, yAxis
  // x: -1 left, 0 neutral, 1 right
  // y: -1 backwards, 0 stop, 1 forwards

  // set_PWM Function
  // dir: 'L' for left wheel, 'R' for right wheel
  // RPM: +int for forwards, -int for backwards

  if (yAxis = 1){
    target_RPM_L = 50;
    target_RPM_R = 50;
    makeWheelsGo();
  } else if (yAxis = -1){
    target_RPM_L = 50;
    target_RPM_R = 50;
    makeWheelsGo();
  } else if (yAxis = 0){
    target_RPM_L = 0;
    target_RPM_R = 0;
    makeWheelsGo();
  }

  if (xAxis = 1){
    target_RPM_R = target_RPM_R-50;
    makeWheelsGo();
  } else if (xAxis = -1){
    target_RPM_L = target_RPM_L-50;
    makeWheelsGo();
  } else if (xAxis = 0){
    makeWheelsGo();
    //???? hELP
  }



}



//////// SETUP ////////

void setup() {
  Serial.begin(115200);
  
  //////// WHEEL SETUP ////////
  wheel_Setup();

  //////// ARROW KEY SETUP ////////
  arrow_Setup();

}



//////// LOOP ////////

void loop() {
  h.serve();
  makeWheelGo();
  delay(10);
}