/*
 * sends text udp packet to TargetIP
 *  using station mode (needs router) and static IP
 */
#include <WiFi.h>
#include <WiFiUdp.h>

#define PWM 1
#define ForwardPin 10
#define ReversePin 8
#define resolution 12 // 12-bit resolution


const char* ssid     = "TP-Link_8A8C";
const char* password = "12488674";
// const char* ssid     = "esesyno"; // Detkiin
// const char* password = "eniacpenn";

WiFiUDP UDPServer;

IPAddress target(192, 168, 1, 129); 
IPAddress myIP(192, 168, 1, 124);

uint8_t udpBuffer[100];



void handleUDPServer() {
  uint8_t packetBuffer[100];
  int i, cb = UDPServer.parsePacket();
  if (cb) {
    UDPServer.read(packetBuffer, 100);
    i = packetBuffer[0] + (packetBuffer[1] << 8); // puts 2 bytes into int
    uint32_t Duty;
    Serial.println(i); // prints the number (note no need to convert to asii)
  }
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
