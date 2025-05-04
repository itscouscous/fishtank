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

// variables for PWM init
const int freq = 10000;      // Frequency in Hz
const int resolution = 12; // 12-bit resolution (0-4095)

WiFiUDP UDPServer;


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
  duty_x = 2*(x - 2048); 
  duty_y = 2*(y - 2048);


  if(duty_x > 50){
    digitalWrite(WHEEL2_DIR2, HIGH);
    digitalWrite(WHEEL2_DIR1, LOW);
  }else if(duty_x < -50){
    duty_x = - duty_x;
    digitalWrite(WHEEL2_DIR1, HIGH);
    digitalWrite(WHEEL2_DIR2, LOW);
  }else{
    duty_x = 0; 
    digitalWrite(WHEEL2_DIR2, LOW);
    digitalWrite(WHEEL2_DIR1, LOW);
  }

  if(duty_y > 50){
    digitalWrite(WHEEL1_DIR2, HIGH);
    digitalWrite(WHEEL1_DIR1, LOW);
  }else if(duty_y < -50){
    duty_y = -duty_y;
    digitalWrite(WHEEL1_DIR1, HIGH);
    digitalWrite(WHEEL1_DIR2, LOW);
  }else{
    duty_y = 0; 
    digitalWrite(WHEEL1_DIR2, LOW);
    digitalWrite(WHEEL1_DIR1, LOW);
  }

  Serial.print("\n Right:");
  Serial.println(duty_y);
  Serial.print("\n Left:");
  Serial.println(duty_x);


  ledcWrite(WHEEL1_PWM, duty_y);
  ledcWrite(WHEEL2_PWM, duty_x);
 
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

  digitalWrite(WHEEL1_DIR1, HIGH);
  digitalWrite(WHEEL1_DIR2, LOW);

}

void loop() {
  //handleUDPServer();
  ledcWrite(WHEEL1_PWM, 2048);
  
  delay(10);
}
