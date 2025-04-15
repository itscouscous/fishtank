/*
 * sends text udp packet to TargetIP
 *  using station mode (needs router) and static IP
 */
#include <WiFi.h>
#include <WiFiUdp.h>

const int potPinX = 4;     // Potentiometer
const int potPinY = 1;     // Potentiometer



const char* ssid     = "TP-Link_8A8C";
const char* password = "12488674";
// const char* ssid     = "esesyno"; // Detkiin
// const char* password = "eniacpenn";

WiFiUDP UDPServer;

IPAddress target(192, 168, 1, 124); 
IPAddress myIP(192, 168, 1, 129);

uint8_t udpBuffer[100];

void fncUdpSend(short int i) // send 2 byte int i
{
  udpBuffer[0] = i & 0xff; // send 1st (LSB) byte of i
  udpBuffer[1] = i>>8; // send 2nd (MSB) byte of i
  UDPServer.beginPacket(target, 2808);
  UDPServer.write(udpBuffer, 2); // send 2 bytes in udpBuffer
  UDPServer.endPacket();
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
  fncUdpSend((uint16_t) analogRead(potPinX));
  fncUdpSend(((uint16_t) analogRead(potPinY)) * -1);
  delay(10);
}
