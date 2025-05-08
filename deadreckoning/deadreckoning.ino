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

int16_t target_RPM = 0;  // Desired target RPM

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


//-------dead reckoning--------
#define wayThreshold 10.0
// waypoints
#define wayN 4
#define wayXvals 0.0, 63.5, 0.0, 63.5
#define wayYvals 0.0, 63.5, 63.5, 0 
////////////////////
//
// heading gain (heading is in radians in this code)
#define headingK 50.0
// wheel geometry
#define wheelRadius 3.45
#define wheelBase 14.05
// math constant
#define PI 3.14159265
//12 ticks on the magnetic disk and 120:1 gear ration = 12*120 = 1440
#define TICKS_PER_ROTATION 1440

// define data structure for robot pose
struct robotPose{
  double X;
  double Y;
  double Q;
};

// define data structure for navigation error
struct wayError{
  double errD; // distance to goal
  double errQ; // heading to goal
};

//
// waypoint management (track goal waypoint)
//
struct wayError getWayE(struct robotPose currentP) {
  static int kGoal = 0; // kGoal = index of target waypoint
  static double dist2GoLast = wayThreshold + 1.0; // initialize to distance larger than threshold
  double wayX[] = {wayXvals}; // wayXvals, wayYvals defined at top
  double wayY[] = {wayYvals};
  //
  // compute distance to current goal
  //
  double dist2Go = sqrt(pow(currentP.X - wayX[kGoal],2) + pow(currentP.Y - wayY[kGoal],2));
  //
  // if we are closer than threshold but further away than last time, go to next goal
  //
  if ( (dist2GoLast < wayThreshold) && (dist2Go > dist2GoLast) ) {
    kGoal = (kGoal + 1) % wayN;
    dist2GoLast = sqrt(pow(currentP.X - wayX[kGoal],2) + pow(currentP.Y - wayY[kGoal],2));
  }
  else { // if farther than threshold or getting closer, keep curent goal
    dist2GoLast = dist2Go;
  }
  //
  // find heading to point at the waypoint
  //
  double desiredQ = atan2(wayY[kGoal]-currentP.Y,wayX[kGoal]-currentP.X);


  //
  // return distance error and heading error
  //
  struct wayError E;
  E.errD = dist2GoLast;
  E.errQ = desiredQ - currentP.Q;

  if(E.errQ > PI){
    E.errQ -= 2*PI;
  }
  if(E.errQ < -PI){
    E.errQ += 2*PI;
  }

  return E;
}


//
// dead reckoning
//
struct robotPose getPose(float deltaT) {

// static variables to integrate encoders to get X, Y, Theta
// "hat" --> best estimate of values
  static double Xhat=0.0;
  static double Yhat=0.0;
  static double Qhat=0.0;
///////////////////////////////////////////////////////////////////
// STUDENT CODE GOES HERE
float countLeft = encoders.getCountL();
float countRight = encoders.getCountR(); 
double vLeft = PI*2*wheelRadius*(countLeft/TICKS_PER_ROTATION)/deltaT;
double vRight = PI*2*wheelRadius*(countRight/TICKS_PER_ROTATION)/deltaT;


double vAvg = (vLeft + vRight)/2;

//xhat += deltaT*(xDot)
Xhat += deltaT*(vAvg*cos(Qhat));
Yhat += deltaT*(vAvg*sin(Qhat));
Qhat += deltaT*((vRight - vLeft)/wheelBase);

///////////////////////////////////////////////////////////////////



struct robotPose measuredP;
measuredP.X = Xhat;
measuredP.Y = Yhat;
measuredP.Q = Qhat;
return measuredP;
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
}

void loop() {

  // if (Serial.available() > 0) {
  //   String input = Serial.readStringUntil('\n');
  //   target_RPM = input.toInt();  // Convert typed input to int
  // }

  target_RPM = 100; 
  static unsigned long millisLast = millis();
  float deltaT = 0.05;
  while ((millis() - millisLast) < 1000*deltaT) {}
  millisLast = millis();

  // get current pose (do dead reckoning)
  // robotPose struct has X, Y, and Theta (heading)
  robotPose P = getPose(deltaT);
  Serial.print("(");
  Serial.print(P.X);
  Serial.print(",");
  Serial.print(P.Y);
  Serial.print(",");
  Serial.print(P.Q);
  Serial.print(")");


  // get error to next waypoint
  // wayError struct has distance to waypoint and unwrapped heading error
  struct wayError wayE = getWayE(P);
  int pwmDel = constrain((int) headingK*wayE.errQ,-100,100);



  //---------------pid----------------
  //Encoder read at 100Hz
  if (micros() - lastEncoderRead > 10000) Sample_RPM();


  //PID runs at 25Hz
  if (micros() - LastPID > 40000){
    LastPID = micros();
    duty_L += calc_PID(LmotorPID, target_RPM - pwmDel, L_RPM);
    duty_R += calc_PID(RmotorPID, target_RPM + pwmDel, R_RPM);
    //Clamp
    if (duty_L >= 1024) duty_L = 1024;
    else if (duty_L <= -1024) duty_L = -1024;

    if (duty_R >= 1024) duty_R = 1024;
    else if (duty_R <= -1024) duty_R = -1024;

    set_PWM('L', (int16_t)duty_L);
    set_PWM('R', (int16_t)duty_R);
    
  }

  Serial.print(350);
  Serial.print(",");
  Serial.print(-350);
  Serial.print(",");
  Serial.print(target_RPM);
  Serial.print(",");
  Serial.print(pwmOutput);
  Serial.print(",");
  Serial.print(L_RPM);
  Serial.print(",");
  Serial.println(R_RPM);
}