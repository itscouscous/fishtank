/*
Demo of slider for MEAM5100 2025
Jessica Weakly
(C)2024 all rights reserved
*/

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

char udpBuffer[100]; //buffer can hold up to 100 characters

/////////// VAR & PINS init ///////////

// #define WHEEL1_PWM 1
// #define WHEEL1_DIR1 10
// #define WHEEL1_DIR2 8

// #define WHEEL2_PWM 0
// #define WHEEL2_DIR1 7
// #define WHEEL2_DIR2 6

//x
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

/////////// FUNCTIONS ///////////

void handleUDPServer() {
  uint8_t packetBuffer[100]; //send int up to 100 characters long
  int cb = UDPServer.parsePacket();
  if (cb) { //if there is anything in the packet run this code

    // int len = UDPServer.read(packetBuffer, 100);
    // packetBuffer[len]=0;
    // Serial.printf ("%s\n",packetBuffer);
    // int please = atoi((char*)packetBuffer); //convert contents of packet buffer into an int
    // if(please < 25){
    //   ledcWrite(outputguy, 0);
    // }else{ //use Ledc to modulate LED brightness based on potentiometer value
    //   ledcWrite(outputguy, please);
    // }

    int len = UDPServer.read(udpBuffer, sizeof(udpBuffer) - 1);
    if (len > 0) {
      udpBuffer[len] = '\0';  // Null-terminate the received string
    }

    Serial.printf("Received UDP Packet: %s\n", udpBuffer);

    // Parse the received "x,y" data
    int x, y;
    if (sscanf(udpBuffer, "%d,%d", &x, &y) == 2) {

      Serial.printf("Parsed X: %d, Y: %d\n", x, y);
      makeCarGo(x, y);

    } else {
      Serial.println("Failed to parse UDP data!");
    }
  }
}

void makeCarGo(int x, int y){
  //Changes pwm duty cycle based on parsed x,y values
    //if x is within deadzone, dutycycle = 0
    //if x is between dzx_max~4095, dutycycle map 0:4095 and dzx_max:4095 and DIRECTION 1
    //if x is between 0~dzx_max, dutycycle map 0:4095 and dzx_max~0 (MAKE SURE ITS INVERSED) and DIRECTION 2

  int duty_y;
  int duty_x;

  if(y <= dzy_max && y >= dzy_min){
    ledcWrite(WHEEL2_PWM, 0);
    Serial.print("\n WHEEL2 duty: 0");
  }else if(y > dzy_max){
    duty_y = map(y, dzy_max, 4095, 0, 4095);
    // ledcWrite(WHEEL2_PWM, duty_y);
    
    digitalWrite(WHEEL2_DIR1, LOW);
    digitalWrite(WHEEL2_DIR2, HIGH);
    
    Serial.print("\n WHEEL2 dir1, duty_y:");
    Serial.println(duty_y);

  }else if(y < dzy_min){
    duty_y = map(y, dzy_min, 0, 0, 4095);
    // ledcWrite(WHEEL2_PWM, duty_y);
    
    digitalWrite(WHEEL2_DIR2, LOW);
    digitalWrite(WHEEL2_DIR1, HIGH);

    Serial.print("\n WHEEL2 dir2, duty_y:");
    Serial.println(duty_y);
    
  }


  if(x <= dzx_max && x >= dzx_min){
    ledcWrite(WHEEL1_PWM, 0);
    Serial.print("\n WHEEL1 duty: 0");
  }else if(x > dzx_max){
    duty_x = map(x, dzx_max, 4095, 0, 4095);
    // ledcWrite(WHEEL1_PWM, duty_x);
    
    digitalWrite(WHEEL1_DIR1, LOW);
    digitalWrite(WHEEL1_DIR2, HIGH);
    
    Serial.print("\n WHEEL1 dir1, duty_x:");
    Serial.println(duty_x);

  }else if(x < dzx_min){
    duty_x = map(x, dzx_min, 0, 0, 4095);
    // ledcWrite(WHEEL1_PWM, duty_x);
    
    digitalWrite(WHEEL1_DIR2, LOW);
    digitalWrite(WHEEL1_DIR1, HIGH);

    Serial.print("\n WHEEL1 dir2, duty_x:");
    Serial.println(duty_x);
    
  }

  ledcWrite(WHEEL1_PWM, duty_x);
  ledcWrite(WHEEL2_PWM, duty_y);

  
  // if(y <= dzy_max && y >= dzy_min){
  //   ledcWrite(WHEEL2_PWM, 0);
  //   Serial.print("\n WHEEL2 duty: 0");
  // }else if(y > dzy_max){
  //   int duty_y = map(y, dzy_max, 4095, 0, 4095);
  //   ledcWrite(WHEEL2_PWM, duty_y);
    
  //   digitalWrite(WHEEL2_DIR1, LOW);
  //   digitalWrite(WHEEL2_DIR2, HIGH);
    
  //   Serial.print("\n WHEEL2 dir1, duty_y:");
  //   Serial.println(duty_y);

  // }else if(y < dzy_min){
  //   int duty_y = map(y, dzy_min, 0, 0, 4095);
  //   ledcWrite(WHEEL2_PWM, duty_y);
    
  //   digitalWrite(WHEEL2_DIR2, LOW);
  //   digitalWrite(WHEEL2_DIR1, HIGH);

  //   Serial.print("\n WHEEL2 dir2, duty_y:");
  //   Serial.println(duty_y);
    
  // }

    //   int duty_forward = map(y, dzy_max, 4095, 0, 4095);
    //   int duty_turn = map(y, dzy_max, 4095, 0, 4095);

  // if(duty_forward >= deadzone_ming && duty_forward <= deadzone_max) {
    //duty_forward = 0; 
  //}
    // if(duty_turn >= deadzone_ming && duty_turn <= deadzone_max) {
    //duty_turn = 0; 
  //}

    //     ledcWrite(WHEEL1_PWM, duty_forward - duty_turn);
  //     ledcWrite(WHEEL2_PWM, duty_forward + duty_turn);

  
}

void setup() {

  Serial.begin(115200);
  analogReadResolution(12);

  //WIFI Setup
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

  // WHEEL 1 init
  ledcAttach(WHEEL1_PWM, freq, resolution);
  ledcWrite(WHEEL1_PWM, 4095);
  pinMode(WHEEL1_DIR1,OUTPUT);
  pinMode(WHEEL1_DIR2,OUTPUT);

  // WHEEL 2 init
  ledcAttach(WHEEL2_PWM, freq, resolution);
  ledcWrite(WHEEL2_PWM, 4095);
  pinMode(WHEEL2_DIR1,OUTPUT);
  pinMode(WHEEL2_DIR2,OUTPUT);

}

void loop() {
//   Serial.println("loop1");
  // h.serve();
  // delay(10);

  handleUDPServer(); //handle incoming packets
  delay(10);

}

