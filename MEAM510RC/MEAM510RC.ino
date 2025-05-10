#include <DualEncoder.h>
#include "TOFSensor.h"

#include "html510.h"
#include "body.h" // html and javascript (content defined in separate file)

HTML510Server h(80);


// Define pins
#define SHT_FRONT_SENSOR 12
#define SHT_LEFT_SENSOR 11
#define SHT_RIGHT_SENSOR 13
#define SDA_PIN 8
#define SCL_PIN 9

#define Servo_Pin 10

// Define addresses
#define LEFT_SENSOR_ADDRESS 0x30
#define RIGHT_SENSOR_ADDRESS 0x31
#define FRONT_SENSOR_ADDRESS 0x32


#define ENCL_A 2
#define ENCL_B 1
#define ENCR_A 4
#define ENCR_B 5

#define R_EN 6
#define L_EN 42

#define R_Forward 15
#define R_Backward 7
#define L_Forward 41
#define L_Backward 40

TOFSensor sensors(
  SHT_FRONT_SENSOR, SHT_LEFT_SENSOR, SHT_RIGHT_SENSOR,
  SDA_PIN, SCL_PIN,
  FRONT_SENSOR_ADDRESS, LEFT_SENSOR_ADDRESS, RIGHT_SENSOR_ADDRESS
);


const char* ssid = "ArrowController";
const char* password = "mother";

int xAxis = 0;  // -1: left, 0: center, 1: right
int yAxis = 0;  // -1: down, 0: center, 1: up


// New global variables for task selection and arm control
int task = 1;       // Default task: 1 = Remote control
int activateArm = 0; // Default arm state: 0 = deactivated

// variables for PWM init
const int freq = 10000;      // Frequency in Hz
const int resolution = 10; // 12-bit resolution (0-4095)

int16_t target_RPM = 0;  // Desired target RPM
int16_t Rtarget_RPM = 0;  // Desired target RPM
int16_t Ltarget_RPM = 0;  // Desired target RPM

int32_t lastEncoderRead = 0;
int32_t LastPID = 0;
int32_t LastCountR = 0;
int32_t LastCountL = 0;
int16_t R_RPM = 0;
int16_t L_RPM = 0;

int16_t TargetDistnace = 150;


int pwmOutput = 0;

float duty_L = 0.0;
float duty_R = 0.0;
float turn = 0.0;

int healthCounter = 100; 

DualEncoder encoders(ENCR_A, ENCR_B, ENCL_A, ENCL_B);  // AR, BR, AL, BL



struct myPID {
  float Kp, Ki, Kd;
  float integral = 0;
  float lastError = 0;

  myPID(float p, float i, float d) : Kp(p), Ki(i), Kd(d) {}
};

myPID LmotorPID(1, 0.0, 0.01);
myPID RmotorPID(1, 0.0, 0.01);
myPID WallFollowingPID(1, 0.0, 0.1);

float calc_PID(myPID &pid, int32_t target, int32_t measured, int32_t time) {
  float error = target - measured;
  pid.integral += error * (1000000.0/time);
  float derivative = ((error - pid.lastError) / (1000000.0/time));
  pid.lastError = error;

  float PID_output = (pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative);

  return PID_output;
}

void Sample_RPM(int32_t time){
  lastEncoderRead = micros();
  R_RPM = (encoders.getCountR() - LastCountR)*(1000000.0/time)*60/30/64;
  L_RPM = (encoders.getCountL() - LastCountL)*(1000000.0/time)*60/30/64;
  LastCountR = encoders.getCountR();
  LastCountL = encoders.getCountL();
}


void set_PWM(char dir, int16_t RPM) {

  if (dir == 'L'){
    if (RPM > 0)
    {
      ledcWrite(L_Forward, RPM);
      ledcWrite(L_Backward, 0);
    }
    else
    {
      ledcWrite(L_Forward, 0);
      ledcWrite(L_Backward, -RPM);
    }
  }else if (dir == 'R'){
    if (RPM > 0)
    {
      ledcWrite(R_Forward, RPM);
      ledcWrite(R_Backward, 0);
    }
    else
    {
      ledcWrite(R_Forward, 0);
      ledcWrite(R_Backward, -RPM);
    }
  }
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
      //need to decrement health here
      sensors.send_I2C_byte(1);
      sensors.receive_I2C_byte();
      healthCounter--; 
      Serial.print("health: ");
      Serial.println(healthCounter);
      if(healthCounter <= 0) {
        Serial.println("------------------------I died-----------------------");
      }
    }
  }
  
  // Send response back to client
  String response = "Received: x:" + String(xAxis) + ",y:" + String(yAxis);
  h.sendplain(response);
}


// New handler for task selection
void handleTask() {
  // Get the task value from the request
  int newTask = h.getVal();
  
  // Validate task value (1-5)
  if (newTask >= 1 && newTask <= 5) {
    task = newTask;
    
    // Print task selection to serial
    Serial.print("Task changed to: ");
    switch(task) {
      case 1:
        Serial.println("Remote Control");
        break;
      case 2:
        Serial.println("Wall Following");
        break;
      case 3:
        Serial.println("Attack Lower");
        break;
      case 4:
        Serial.println("Attack Upper");
        break;
      case 5:
        Serial.println("Attack Base");
        break;
      case 6:
        Serial.println("Attack All");
        break;
    }
  }
  
  // Send response back to client
  String response = "Task set to: " + String(task);
  h.sendplain(response);
}

// New handler for arm activation
void handleArm() {
  // Get the arm state from the request (0 or 1)
  int newArmState = h.getVal();
  
  // Validate arm state
  if (newArmState == 0 || newArmState == 1) {
    activateArm = newArmState;
    
    // Print arm state to serial
    Serial.print("Arm state changed to: ");
    Serial.println(activateArm ? "Activated" : "Deactivated");
  }
  
  // Send response back to client
  String response = "Arm state set to: " + String(activateArm);
  h.sendplain(response);
}


void setupWifi() {
  //Serial.begin(115200);
  Serial.println("Starting Arrow Key Controller");

    // 1) define your new AP network parameters:
  IPAddress local_IP(192, 168, 5, 69);      // <-- the IP you want your ESP to have
  IPAddress gateway(192, 168, 5, 69);       // <-- usually same as local_IP
  IPAddress subnet(255, 255, 255, 0);      // <-- your subnet mask
  
  // 3) apply your custom network config _before_ calling softAP()
  if (!WiFi.softAPConfig(local_IP, gateway, subnet)) {
    Serial.println("AP Config Failed!");
  }

  // 4) start the open AP (no password)
  WiFi.softAP(ssid);

  WiFi.softAP(ssid, ""); // no password
  Serial.println(ssid);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
  
  // Initialize web server
  h.begin();
  h.attachHandler("/ ", handleRoot); // default page
  h.attachHandler("/cmd?val=", handleCommand); // arrow key command
  h.attachHandler("/task?val=", handleTask); // new task selection handler
  h.attachHandler("/arm?val=", handleArm); // new arm control handler
  
  Serial.println("Server ready!");
}


void setup() {
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

  setupWifi();

  if (!sensors.begin()) {
    Serial.println("Failed to initialize sensors!");
    while (1) delay(10);
  }
  
  Serial.println("Sensors initialized successfully");

  ledcAttach(10, 50, 13);

}

void RC(){

  if (activateArm == 1)
  {
    ledcWrite(10, 1000);
  }
  else
  {
    ledcWrite(10, 350);
  }

  //Encoder read at 100Hz
  if (micros() - lastEncoderRead > 10000) Sample_RPM(micros() - lastEncoderRead);


  //PID runs at 25Hz
  if (micros() - LastPID > 40000){
    int32_t DeltaPID = micros() - LastPID;
    LastPID = micros();

    target_RPM = yAxis * 100;

    Ltarget_RPM = target_RPM + (xAxis * 50);
    Rtarget_RPM = target_RPM - (xAxis * 50);


    duty_L += calc_PID(LmotorPID, Ltarget_RPM, L_RPM, DeltaPID);
    duty_R += calc_PID(RmotorPID, Rtarget_RPM, R_RPM, DeltaPID);
    //Clamp
    if (duty_L >= 1024) duty_L = 1024;
    else if (duty_L <= -1024) duty_L = -1024;

    if (duty_R >= 1024) duty_R = 1024;
    else if (duty_R <= -1024) duty_R = -1024;

    set_PWM('L', (int16_t)duty_L);
    set_PWM('R', (int16_t)duty_R);
  }
}

void WallFollow()
{
  
  target_RPM = 50;
  sensors.readSensors();
  Serial.println(sensors.getFrontDistance());
  //Encoder read at 100Hz
  if (micros() - lastEncoderRead > 10000) Sample_RPM(micros() - lastEncoderRead);


  //PID runs at 25Hz
  if (micros() - LastPID > 40000){
    int32_t Delta = micros() - LastPID;
    LastPID = micros();

    turn += calc_PID(WallFollowingPID, TargetDistnace, sensors.getLeftDistance(), Delta);
    if (turn >= 20) turn = 20;
    else if (turn <= -20) turn = -20;
      Serial.print(" left distance: "); 
      Serial.print(sensors.getLeftDistance());

      Serial.print(" front distance: "); 
      Serial.print(sensors.getFrontDistance());

      Serial.print(" right distance: "); 
      Serial.print(sensors.getRightDistance());
     if (sensors.getFrontDistance() <= 300)
     {
       //wall in front, turn left)
       Serial.print("Hit wall!");
      Serial.print(sensors.getFrontDistance()); 
       turn = 50;
     }

    Ltarget_RPM = target_RPM + turn;
    Rtarget_RPM = target_RPM - turn;

    Serial.print("turn: "); 
    Serial.print(turn); 
    Serial.print(" Ltarget_RPM: "); 
    Serial.print(Ltarget_RPM);
    Serial.print(" Rtarget_RPM: ");
    Serial.print(Rtarget_RPM); 


    duty_L += calc_PID(LmotorPID, Ltarget_RPM, L_RPM, Delta);
    duty_R += calc_PID(RmotorPID, Rtarget_RPM, R_RPM, Delta);

    Serial.print(" duty_L: ");
    Serial.print(duty_L); 
    Serial.print(" duty_R: ");
    Serial.println(duty_R); 
    //Clamp
    if (duty_L >= 1024) duty_L = 1024;
    else if (duty_L <= -1024) duty_L = -1024;

    if (duty_R >= 1024) duty_R = 1024;
    else if (duty_R <= -1024) duty_R = -1024;

    set_PWM('L', (int16_t)duty_L);
    set_PWM('R', (int16_t)duty_R);
  }
}


void loop() {

  h.serve();

  if (task == 1) RC();
  else if (task ==2) WallFollow();

}