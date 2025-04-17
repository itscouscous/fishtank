#define ENCODER_A 3
#define ENCODER_B 5
#define MOTOR_PWM_PIN 7  // Output pin for PWM using LEDC
#define PWM_FREQ 10000     // 1 kHz
#define PWM_RES_BITS 10     // 10-bit resolution: 0–1023
#define TARGET_RPM 1000      // Desired RPM
const int PPR = 3; // Pulses per revolution from A channel


// PID constants
float Kp = 0.01;
float Ki = 0.0;
float Kd = 0.1;

// PID variables
volatile int lastDirection = 0; // +1 = CW, -1 = CCW
volatile unsigned long lastRotationTime = 0;
volatile int rpm = 0;
volatile int pulseCount = 0;

float lastError = 0;
float integral = 0;
unsigned long lastPIDUpdateTime = 0;
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

  Serial.println("interrupt!"); 


  // Only measure every full rotation
  // if (pulseCount >= PPR) {
  //   unsigned long now = micros();
  //   unsigned long dt = now - lastRotationTime;
  //   lastRotationTime = now;
  //   pulseCount = 0;

  //   if (dt > 0) {
  //     rpm = (60.0 * 1000000.0) / dt;
  //   }
  // }
}

void setup() {
  Serial.begin(115200);

  pinMode(ENCODER_A, INPUT_PULLUP);
  //pinMode(ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), handleEncoderA, RISING);


  // Set up LEDC PWM
  //ledcAttach(MOTOR_PWM_PIN, PWM_FREQ, PWM_RES_BITS);
}


void loop() {
  Serial.print("RPM: ");
  Serial.println(rpm);
  /*
  static unsigned long lastPrint = 0;

  if (micros() - lastPrint >= 10000) {

    int currentRPM = rpm;
    int dir = lastDirection;

    // PID calculation
    int16_t error = TARGET_RPM - currentRPM;
    unsigned long now = micros();
    int deltaT = (now - lastPIDUpdateTime);
    integral += error * deltaT;
    float derivative = (error - lastError);
    float output = Kp * error + Ki * integral + Kd * derivative;
    duty += output;
    if (duty > 1023)
    {
      duty = 1023;
    }
    else if (duty < 0)
    {
      duty = 0;
    }


    lastError = error;
    lastPIDUpdateTime = now;

    ledcWrite(MOTOR_PWM_PIN, duty);


    // Debug print
    Serial.print(output);
    Serial.print("RPM: ");
    Serial.print(currentRPM, 1);
    Serial.print(" | PWM: ");
    Serial.println(duty);

    lastPrint = now;
  }
  */
}

