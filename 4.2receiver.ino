/*
 * sends text udp packet to TargetIP
 *  using station mode (needs router) and static IP
 */
#include <WiFi.h>
#include <WiFiUdp.h>
// GM
const char* ssid     = "TP-Link_8A8C";
const char* password = "12488674";
// Detkin
// const char* ssid     = "esesyno";
// const char* password = "eniacpenn";

#define WHEEL1_PWM 0
#define WHEEL1_DIR1 7
#define WHEEL1_DIR2 6
//y
#define WHEEL2_PWM 1
#define WHEEL2_DIR1 10
#define WHEEL2_DIR2 8

// variables for PWM init
const int freq = 30;      // Frequency in Hz
const int resolution = 12; // 12-bit resolution (0-4095)

// deadzone calibration
const int deadzone_x = 2460;
const int range_x = 100;
const int dzx_max = deadzone_x + range_x;
const int dzx_min = deadzone_x - range_x;

// const int deadzone_x = 3836;
// const int range_x = 50;
// const int dzx_max = deadzone_x + range_x;
// const int dzx_min = deadzone_x - range_x;

const int deadzone_y = 2405;
const int range_y = 100;
const int dzy_max = deadzone_y + range_y;
const int dzy_min = deadzone_y - range_y;

WiFiUDP UDPServer;

IPAddress target(192, 168, 1, 129); 
IPAddress myIP(192, 168, 1, 124);

uint8_t udpBuffer[100];

volatile uint16_t x = 0; 
volatile uint16_t y = 0; 

volatile int duty_y;
volatile int duty_x;

void handleUDPServer() {
  uint8_t packetBuffer[100];
  int i, cb = UDPServer.parsePacket();
  if (cb) {
    UDPServer.read(packetBuffer, 100);
    i = packetBuffer[0] + (packetBuffer[1] << 8); // puts 2 bytes into int
    uint32_t Duty;
    if(i >= 32767) {
      y = i - 32767;
    }
    else {
      x = i; 
    }
    Serial.print("x: ");
    Serial.print(x);

    Serial.print(" y: "); // prints the number (note no need to convert to asii)
    Serial.println(y);

    makeCarGo(x, y); 

  }
}

void makeCarGo(int x, int y){
  //Changes pwm duty cycle based on parsed x,y values
    //if x is within deadzone, dutycycle = 0
    //if x is between dzx_max~4095, dutycycle map 0:4095 and dzx_max:4095 and DIRECTION 1
    //if x is between 0~dzx_max, dutycycle map 0:4095 and dzx_max~0 (MAKE SURE ITS INVERSED) and DIRECTION 2



  if(y <= dzy_max && y >= dzy_min){
    duty_y = 0; 
    Serial.print("\n in duty y deadzone");
  }else if(y > dzy_max){
    duty_y = map(y, dzy_max, 4095, 0, 4095);    
    digitalWrite(WHEEL2_DIR1, LOW);
    digitalWrite(WHEEL2_DIR2, HIGH);
  }else if(y < dzy_min){
    duty_y = map(y, dzy_min, 0, 0, 4095);    
    digitalWrite(WHEEL2_DIR2, LOW);
    digitalWrite(WHEEL2_DIR1, HIGH);
    
  }


  if(x <= dzx_max && x >= dzx_min){
    duty_x = 0; 
    Serial.print("\n in duty x deadzone");
  }else if(x > dzx_max){
    duty_x = map(x, dzx_max, 4095, 0, 4095);  
    digitalWrite(WHEEL1_DIR1, LOW);
    digitalWrite(WHEEL1_DIR2, HIGH);
  }else if(x < dzx_min){
    duty_x = map(x, dzx_min, 0, 0, 4095);
    digitalWrite(WHEEL1_DIR2, LOW);
    digitalWrite(WHEEL1_DIR1, HIGH);
  }

  Serial.print("\n duty_x:");
  Serial.println(duty_x);
  Serial.print("\n duty_y:");
  Serial.println(duty_y);


  ledcWrite(WHEEL1_PWM, duty_x);
  ledcWrite(WHEEL2_PWM, duty_y);
 
}




void setup() {
  Serial.begin(115200);


  WiFi.begin(ssid, password);

  WiFi.config( myIP,    // device IP address
      IPAddress(192, 168, 1, 1), // gateway (not used)
      IPAddress(255, 255, 255, 0)); // netmask
  
  UDPServer.begin(2808);  // 2808 arbitrary UDP port#    
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);    Serial.print(".");
  }
}

void loop() {
  handleUDPServer();
  delay(10);
}
