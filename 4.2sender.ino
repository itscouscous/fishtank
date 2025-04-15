/*
Demo of slider for MEAM5100 2025
Jessica Weakly
(C)2024 all rights reserved
*/

/////////// ESPNOW init ///////////
// #include <esp_now.h>
// #include "html510.h"
// #include <esp_wifi.h>  //for setting mac
// #include "body.h"      // html and javascript
// HTML510Server h(80);
// const char* ssid     = "sliderDemo2";
// const char* password = "";

/////////// UDP init ///////////
#include <WiFi.h>
#include <WiFiUdp.h>
// GM
const char* ssid     = "TP-Link_8A8C";
const char* password = "12488674";
// Detkin
// const char* ssid     = "esesyno"; 
// const char* password = "eniacpenn";

WiFiUDP UDPServer;

//Use same code for both ESP32s, just swap the IP addresses
IPAddress myIPaddress(192, 168, 1, 109); // change to your IP
IPAddress target(192, 168, 1, 109); // change to IP you are sending to

uint8_t udpBuffer[100]; //buffer can hold up to 100 characters

/////////// VAR & PINS init ///////////

#define INPUTX 4 //input pin for pot x
#define INPUTY 1 //input pin for pot y

/////////// FUNCTIONS ///////////

void fncUdpSend(){
  // send what ever you want upto buffer size                      
  UDPServer.beginPacket(target, 2808);  // send to UDPport 2808
  UDPServer.printf("%s",udpBuffer);
  UDPServer.endPacket();
  //Serial.println(udpBuffer);
}

void setup(){
  Serial.begin(115200);

  WiFi.begin(ssid, password);

  WiFi.config( myIPaddress,        // Device IP address
      IPAddress(192, 168, 1, 1),   // gateway (not important for 5100)
      IPAddress(255, 255, 255, 0)); // net mask 
  
  UDPServer.begin(2808);  // 2808 arbitrary UDP port# need to use same one   
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);    Serial.print(".");
  }

  Serial.println("WiFi connected");
  Serial.print("Using static IP "); 
  Serial.println(myIPaddress);
  Serial.printf("WiFi connected to %s", ssid);
  Serial.print(" to "); Serial.println(target);  

  // init thumstick pins as input
  pinMode(INPUTX, INPUT);
  pinMode(INPUTY, INPUT);
}

void loop() {
  int val_x = analogRead(INPUTX); //take in pot value x
  int val_y = analogRead(INPUTY); //take in pot value y

  
  Serial.printf ("%d,%d\n", val_x, val_y);
  sprintf((char*)udpBuffer, "%d,%d", val_x, val_y); //set pot value to char and print to udpbuffer

  fncUdpSend();//send buffer
  delay(100);

  // delay(10);
}