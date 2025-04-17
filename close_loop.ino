#define ENCODER_A 35
// #define ENCODER_B 5
#define MOTOR_PWM_PIN 0  // Output pin for PWM using LEDC
#define PWM_FREQ 30     // 1 kHz
#define PWM_RES_BITS 12     // 10-bit resolution: 0–1023
#define TARGET_RPM 100      // Desired RPM

#define WHEEL1DIR1 6
#define WHEEL1DIR2 7
// #define WHEEL2DIR1 8
// #define WHEEL2DIR2 10

#include <math.h>

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

void setup() {
  Serial.begin(115200);

  pinMode(ENCODER_A, INPUT_PULLUP);
  //pinMode(ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), handleEncoderA, RISING);


  // Set up LEDC PWM
  ledcAttach(MOTOR_PWM_PIN, PWM_FREQ, PWM_RES_BITS);

  pinMode(WHEEL1DIR1, OUTPUT);
  pinMode(WHEEL1DIR2, OUTPUT);

}


void loop() {
  
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
    //int dir = lastDirection;

    // PID calculation
    int16_t error = TARGET_RPM - currentRPM;
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

    if(TARGET_RPM > 0) {
      digitalWrite(WHEEL1DIR1, HIGH);
      digitalWrite(WHEEL1DIR2, LOW);
    }
    else {
      digitalWrite(WHEEL1DIR1, LOW);
      digitalWrite(WHEEL1DIR2, HIGH);
    }

    ledcWrite(MOTOR_PWM_PIN, duty);


    // Debug print
    Serial.print(output);
    Serial.print(" RPM: ");
    Serial.print(currentRPM, 1);
    Serial.print(" | PWM: ");
    Serial.println(duty);

    lastPrint = now;
  }
  
}

