//Libraries
#include <WiFi.h>
#include <esp_now.h> //library for bidirectional wireless communication between two ESP32
#include <esp_wifi.h> //needed to adjust the MAC address
#include <Wire.h>

//Pin definitions
#define vsense 2
#define joyx 3
#define joyy 4 
#define button 5 
#define led 20

//calibration data
bool motordirection = false; //change this variable to false to invert the motor spin direction 
int motorint = 10; //sets the delay time between motor speed changes (in ms). Higher values result in 
//better traction but slower responses. Values between 5 and 10 are recommended.
int speedlimit = 127; //this variable sets the maximum speed of the car (values between 0 and 255)
int leftlimit = 358; //steering limit calculation: leftlimit = x/20ms*2^12 (default: 205 (x = 1ms))
int rightlimit = 258; //steering limit calculation: rightlimit = x/20ms*2^12 (default: 410 (x = 2ms))
//RC servos run at 50 Hz (20 ms intervals), with the center at 1.5 ms and a range of +-0.5 ms around the center
//starting values should be 205 to 410 (left to right or right to left, depends on the servo)
//adjust these values to a smaller or larger interval depending on the actual steering angles
//individually decrease/increase one of the two values to trim the center of the steering

//MAC addresses
uint8_t CarMAC[] = {0xAA, 0xA0, 0xBE, 0xEF, 0xFE, 0xED}; //we will set this MAC address for the car
uint8_t RemoteMAC[] = {0xB0, 0xAD, 0xBE, 0xEF, 0xFE, 0xEF}; //we will set this MAC address for the remote control
uint8_t broadMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; //using this MAC address as a receiver for sending 
//packages means broadcasting (ignoring if the data has been received)

//global variables
int speed = 0;
int oldspeed = 0;
int steer = 0;
int rctimer = 0;
int battimer = 0;
float batvolt = 0;
int batlow = 0;
int speedcenter = 0;
int steercenter = 0;
int analogspeed = 0;
int analogsteer = 0;

//ESP-NOW setup
/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-two-way-communication-esp32/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/

// Structure to send data; must match the receiver structure of the car!
typedef struct send_message {
  int speed;
  int steer;
  bool horn;
  int motorint;
  int indicator;
  int brake;
} send_message;

// Create a structured object
send_message outgoing;

esp_now_peer_info_t peerInfo; //make sure that the ESP-NOW lines are in the exact same order as in this sketch. Otherwise, there may be malfunctions.

void setup() {
  // put your setup code here, to run once:

  for (int i = 0; i<10; i++){
    speed = speed + analogRead(joyx);
    steer = steer + analogRead(joyy);

    speedcenter = speed/10;
    steercenter = steer/10;
  }

  pinMode(led,OUTPUT);
  digitalWrite(led,HIGH);
  pinMode(button,INPUT_PULLUP);

  // Activate ESP-NOW
  WiFi.mode(WIFI_STA); //Set device as a Wi-Fi Station
  esp_wifi_set_mac(WIFI_IF_STA, RemoteMAC); //Overwrite hardware MAC with this new MAC address
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR); //the long range WiFI protocol should theoretically improve the max. distance of the wireless connection (not verified)
  esp_now_init(); // Initialize ESP-NOW

  esp_wifi_set_promiscuous(true); //set WiFi channel
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);     
  peerInfo.channel = 1;  
  peerInfo.encrypt = false;  

  memcpy(peerInfo.peer_addr, broadMAC, 6); //data is sent in broadcast mode: sent to all listening devices
  esp_now_add_peer(&peerInfo); // Add peer   

  outgoing.motorint = motorint;

  rctimer = millis();
  battimer = millis();

  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:

  readXaxis();
  readYaxis();
  checkBattery();

  if(digitalRead(button)){
    outgoing.horn = false;
  }else{
    outgoing.horn = true;
  }

  if (millis()>rctimer+49){ //send a new data package every 50 ms (20 times per second)
    rctimer = millis(); 
    esp_now_send(broadMAC, (uint8_t *) &outgoing, sizeof(outgoing));
    SerialOutput();
  }

}

void readXaxis(){
  oldspeed = speed;
  speed = analogRead(joyx);
  analogspeed = speed;
  if (speed>speedcenter-200&&speed<speedcenter+200){ //sets +-5% from the center value of the joystick as zero
    speed = speedcenter;
  }
  if (speed<speedcenter-1750){
    speed = speedcenter-1750;
  }else if(speed>speedcenter + 1750){
    speed = speedcenter + 1750;
  }
  if(speed>speedcenter){
    speed = map(speed,speedcenter+100,speedcenter+1750,speedcenter,speedcenter+1750);
  }else if(speed<speedcenter){
    speed = map(speed,speedcenter-100,speedcenter-1750,speedcenter,speedcenter-1750);
  }

  if (motordirection){
    speed = map(speed,speedcenter-1750,speedcenter+1750,-speedlimit,speedlimit);
  }else{
    speed = map(speed,speedcenter-1750,speedcenter+1750,speedlimit,-speedlimit);
  }
  outgoing.speed = speed;
  if (motordirection){
    if (speed<=0){
      outgoing.brake = 1;
    }else{
      outgoing.brake = 0;
    }
  }else{
    if (speed>=0){
      outgoing.brake = 1;
    }else{
      outgoing.brake = 0;
    }  
  }
}

void readYaxis(){
  steer = analogRead(joyy);
  analogsteer = steer;
  if (steer>steercenter-150&&steer<steercenter+150){ //sets +-5% from the center value of the joystick as zero
    steer = steercenter;
  }
  if (steer<steercenter-1750){
    steer = steercenter-1750;
  }else if(steer>steercenter + 1750){
    steer = steercenter + 1750;
  }
  if(steer>steercenter){
    steer = map(steer,steercenter+150,steercenter+1750,steercenter,steercenter+1750);
  }else if(steer<steercenter){
    steer = map(steer,steercenter-150,steercenter-1750,steercenter,steercenter-1750);
  }

  steer = map(steer,steercenter-1750,steercenter+1750,leftlimit,rightlimit);
  outgoing.steer = steer;  
  if (leftlimit>rightlimit){
    if(steer>(leftlimit+rightlimit)/2+5){
      outgoing.indicator = 1;
    }else if(steer<(leftlimit+rightlimit)/2-5){
      outgoing.indicator = 2;
    }else{
      outgoing.indicator = 0;
    }
  }else{
    if(steer<(leftlimit+rightlimit)/2-5){
      outgoing.indicator = 1;
    }else if(steer>(leftlimit+rightlimit)/2+5){
      outgoing.indicator = 2;
    }else{
      outgoing.indicator = 0;
    }    
  }
}

void checkBattery(){
  if (batlow<500){ //low battery warning circuit
    batvolt = analogReadMilliVolts(vsense);
    batvolt = (batvolt/1000)*(33+33)/33; //voltage divider: 33kOhm + 33kOhm equals measured voltage times 2
    if (batvolt<3.4){
      batlow = batlow + 1;
    }else if(batlow>0){
      batlow = batlow - 1;
    }
  }else{
    if(millis()>battimer+500){
      battimer = millis();
      digitalWrite(led,LOW);
    }else if(millis()>battimer+250){
      digitalWrite(led,HIGH);
    }
  }
}

void SerialOutput(){
  Serial.print("Speedcenter: ");
  Serial.println(speedcenter);
  Serial.print("Steercenter: ");
  Serial.println(steercenter);
  Serial.print("Battery voltage: ");
  Serial.println(batvolt,2);
  Serial.print("JoyX: ");
  Serial.println(analogspeed);
  Serial.print("JoyY: ");
  Serial.println(analogsteer);
  Serial.print("Steer output: ");
  Serial.println(outgoing.steer);
  Serial.print("Button: ");
  Serial.println(outgoing.horn);
  Serial.println("--------");
}
