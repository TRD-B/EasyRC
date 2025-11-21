//Libraries
#include <WiFi.h>
#include <esp_now.h> //library for bidirectional wireless communication between two ESP32
#include <esp_wifi.h> //needed to adjust the MAC address
#include <Wire.h>
#include <driver/ledc.h>

//Pin definitions
#define buzzer 3
#define vsense 4
#define led1 5
#define led2 6
#define led3 7 
#define pwm 10 
#define dir 20
#define servo 21

#define servoChannel 0
#define pwmChannel 1
#define ledChannel 2

//MAC addresses
uint8_t CarMAC[] = {0xAA, 0xA0, 0xBE, 0xEF, 0xFE, 0xED}; //we will set this MAC address for the car
uint8_t RemoteMAC[] = {0xB0, 0xAD, 0xBE, 0xEF, 0xFE, 0xEF}; //we will set this MAC address for the remote control
uint8_t broadMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; //using this MAC address as a receiver for sending packages means broadcasting (ignoring if the data has been received)

//global variables
int motorspeed = 0;
int motortimer = 0;
bool pwmconfig = 0;
int battimer = 0;
float batvolt = 0;
int batlow = 0;
int rctimer = 0;
int counter = 0;
bool afk = 0;
int serialtimer = 0;
int afktimer = 0;
int safetyloop = 0;
int indicatortimer = 0;
int ledstatus = 0;
int indicator = 0;
//ESP-NOW setup

//Structure to receive data; must match the sender structure of the RC!
typedef struct receive_message {
  int speed;
  int steer;
  bool horn;
  int motorint;
  int indicator;
  int brake;
} receive_message;

receive_message incoming;

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  bool sendercheck = true;
  for (int i = 0; i< 6; i++){ //When using ESP-NOW in broadcast mode, this code snippet ensures that only paired devices communicate with each other (verifying the sender's MAC address)
    if (mac[i] != RemoteMAC[i]){
      sendercheck = false;
      break;
    }
  }
  if (sendercheck){//&&len==13){ //only process the data if it is from the RC and if the packet has the correct length (data from the car to logger has a different length!)
    memcpy(&incoming, incomingData, sizeof(incoming));   
    rctimer = millis();  
  } 
}
esp_now_peer_info_t peerInfo; //make sure that the ESP-NOW lines are in the exact same order as in this sketch. Otherwise, there may be malfunctions.


void setup() {
  // put your setup code here, to run once:

Serial.begin(115200);

pinMode(buzzer, OUTPUT);
digitalWrite(buzzer, LOW);
pinMode(led1, OUTPUT);
digitalWrite(led1, HIGH);
pinMode(led2, OUTPUT);
digitalWrite(led2, LOW);
pinMode(led3, OUTPUT);
digitalWrite(led3, LOW);
pinMode(pwm, OUTPUT);
digitalWrite(pwm, LOW);
pinMode(dir, OUTPUT);
digitalWrite(dir, HIGH);
pinMode(servo, OUTPUT);
digitalWrite(servo, LOW);

ledc_timer_config_t timer_0;
  timer_0.speed_mode = LEDC_LOW_SPEED_MODE;
  timer_0.timer_num = LEDC_TIMER_0;
  timer_0.duty_resolution = LEDC_TIMER_12_BIT; // Servo: 12 Bit
  timer_0.freq_hz = 50;
  timer_0.clk_cfg = LEDC_AUTO_CLK;
ledc_timer_config(&timer_0);

ledc_channel_config_t ch0;
  ch0.gpio_num = servo;
  ch0.speed_mode = LEDC_LOW_SPEED_MODE;
  ch0.channel = LEDC_CHANNEL_0;
  ch0.intr_type = LEDC_INTR_DISABLE;
  ch0.timer_sel = LEDC_TIMER_0;
  ch0.duty = 0;
  ch0.hpoint = 0;
ledc_channel_config(&ch0);

  // Activate ESP-NOW
  WiFi.mode(WIFI_STA); //Set device as a Wi-Fi Station
  esp_wifi_set_mac(WIFI_IF_STA, CarMAC); //Overwrite hardware MAC with this new MAC address
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);

  esp_now_init(); // Initialize ESP-NOW

  // Register peer
  memcpy(peerInfo.peer_addr, broadMAC, 6);
  esp_now_add_peer(&peerInfo); // Add peer 

  esp_wifi_set_promiscuous(true); //set WiFi channel
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);     
  peerInfo.channel = 1;  
  peerInfo.encrypt = false;

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  digitalWrite(led2,HIGH);
  digitalWrite(led3,HIGH);
  setPWMchannel(buzzer,250); //startup buzzer sequence
  Buzzer(true);
  delay(100);
  Buzzer(false);
  delay(400);
  Buzzer(true);
  delay(100);
  Buzzer(false);
  delay(400);
  setPWMchannel(buzzer,350);
  Buzzer(true);
  delay(500);
  Buzzer(false);
  delay(1000);
  setPWMchannel(buzzer,400);
  digitalWrite(led2,LOW);
  digitalWrite(led3,LOW);

  while(rctimer==0&&batlow<500){ //indicate "waiting for connection" with beeping every 2 s
    checkBattery();
    delay(10);
    counter = counter + 1;
    if(counter>199){
      Buzzer(false);
      digitalWrite(led2,LOW);
      digitalWrite(led3,LOW);
      counter = 0;
    }else if(counter>179){
      Buzzer(true);
      digitalWrite(led2,HIGH);   
      digitalWrite(led3,HIGH);    
    }
  }

  counter = 0;

  if(rctimer>0){ //connection established sequence
    digitalWrite(led2, HIGH);
    digitalWrite(led3, HIGH);
    setPWMchannel(buzzer,350);
    Buzzer(true);
    delay(400);
    Buzzer(false);
    delay(100);
    setPWMchannel(buzzer,500);
    Buzzer(true);
    delay(200);
    Buzzer(false);
    digitalWrite(led2, LOW);
    digitalWrite(led3, LOW);
  }

  serialtimer = millis();
  afktimer = millis();
  motortimer = millis();
  indicatortimer = millis();

}

void loop() {
  checkBattery();
  checkUser();

  if(millis()>rctimer+999||batlow>9999||afk==1){ //stop motor and beep if an error occurs
    safetyloop = 1;
    ServoControl(0);
    motorspeed = setMotorSpeed(0, motorspeed);
    if(motorspeed!=0){
      MotorControl(motorspeed);
    }else{
      if(pwmconfig==0){
        setPWMchannel(buzzer,400);
      }
      if (afk==1){
        if(millis()>battimer+3000){
          battimer = millis();
          digitalWrite(led2,LOW);
          digitalWrite(led3,LOW);
          Buzzer(false);
        }else if(millis()>battimer+2000){
          digitalWrite(led2,HIGH);
          digitalWrite(led3,HIGH);
          Buzzer(true);
        }      
      }else if(batlow>9999){
        if(millis()>battimer+3000){
          battimer = millis();
          digitalWrite(led2,LOW);
          digitalWrite(led3,LOW);
          Buzzer(false);
        }else if(millis()>battimer+2500){
          digitalWrite(led2,HIGH);
          digitalWrite(led3,HIGH);
          Buzzer(true);
        }           
      }else{
        if(millis()>battimer+3000){
          battimer = millis();
          digitalWrite(led2,LOW);
          digitalWrite(led3,LOW);
          Buzzer(false);
        }else if(millis()>battimer+1500){
          digitalWrite(led2,HIGH);
          digitalWrite(led3,HIGH);
          Buzzer(true);
        }   
      }

    } 
    if(batlow>2499){
      Serial.println("Battery low!");
    }else if(millis()>rctimer+999){
      Serial.println("No connection");
    }else if(afk==1){
      Serial.println("No user input!");
    }

  }else{
    ServoControl(incoming.steer);
    motorspeed = setMotorSpeed(incoming.speed, motorspeed);  
    if (incoming.brake==1){
      digitalWrite(led1,HIGH);
    }else if(ledstatus<2){
      digitalWrite(led1,LOW);
      ledstatus=ledstatus+1;
    }else{
      digitalWrite(led1,HIGH);
      ledstatus=0;
    }
    if(indicator!=incoming.indicator){
      indicatortimer = millis()-501;
      digitalWrite(led2,LOW);
      digitalWrite(led3,LOW);      
    }
    if (millis()>indicatortimer + 1000){
      indicatortimer = millis();
      digitalWrite(led2,LOW);
      digitalWrite(led3,LOW);
    }else if(millis()>indicatortimer + 500){
      if(incoming.indicator==1){
        digitalWrite(led2,HIGH);
        digitalWrite(led3,LOW);
      }else if(incoming.indicator==2){
        digitalWrite(led3,HIGH);
        digitalWrite(led2,LOW);
      }
    }
    indicator = incoming.indicator;
    safetyloop = 0;
   }

  if (safetyloop==0){
    if(motorspeed==0){
      if (incoming.horn==1&&pwmconfig==0){ //switch PWM from motor to buzzer if the horn is requested at standstill
        setPWMchannel(buzzer,400);
      }else{
        if (incoming.horn){
          Buzzer(true);
        }else{
          Buzzer(false);
        }
      }
    }else{
      if(pwmconfig==1){
        setPWMchannel(pwm,20000);
      }else{
        MotorControl(motorspeed);
        //MotorControl(incoming.speed);
      }
    }
  }


  if(millis()>serialtimer+1000){
    Serial.print("Speed: ");
    Serial.println(incoming.speed);
    Serial.print("Motor speed: ");
    Serial.println(motorspeed);
    Serial.print("Steering: ");
    Serial.println(incoming.steer);
    Serial.print("Horn: ");
    Serial.println(incoming.horn);
    Serial.print("Battery voltage: ");
    Serial.println(batvolt);
    Serial.println("----------");
    serialtimer = millis();
  }

}

void checkUser(){
  if(incoming.speed==0&&incoming.horn==0){ //if neither the horn or the motor is operated for approx. 1 min (assuming a loop time of 5 ms), the user is probably away from keyboard - triggers a warning
    if(millis()>afktimer+60000){
      afk=1;
    }
  }else{
    afktimer = millis();
    afk = 0;
  }
}

void checkBattery(){
  if (batlow<10000){ //low battery warning circuit
    batvolt = analogReadMilliVolts(vsense);
    batvolt = (batvolt/1000)*(100+22)/22; //voltage divider: 100kOhm + 22kOhm
    if (batvolt<10.2){
      batlow = batlow + 1;
    }else if(batlow>0){
      batlow = batlow - 1;
    }
  }
}

int setMotorSpeed(int newval, int oldval){
  int outputval = 0;
  if(millis()>motortimer+incoming.motorint){

    motortimer = millis();
    
    if (oldval<10&&oldval>-10){ //select motor spin direction: only change at speeds close to 0
      if (newval<0){
        digitalWrite(dir, LOW);
      }else{
        digitalWrite(dir, HIGH);
      }
    }

    if(newval>oldval){
      if (newval>oldval+10){
        outputval = oldval + 5;
      }else if (newval>oldval+5){
        outputval = oldval + 2;
      }else if (newval>oldval){
        outputval = oldval + 1;
      }
    }else if(newval<oldval){
      if (newval<oldval-10){
        outputval = oldval - 5;
      }else if (newval<oldval-5){
        outputval = oldval - 2;
      }else if (newval<oldval){
        outputval = oldval - 1;
      }      
    }else{
      outputval = oldval;
    }  
  }else{
    outputval = oldval;
  }

  return outputval;
}

void setPWMchannel(int pinNumber, int frequency){

  ledc_timer_config_t timer_1;
    timer_1.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_1.timer_num = LEDC_TIMER_1;
    timer_1.duty_resolution = LEDC_TIMER_8_BIT; // Servo: 12 Bit
    timer_1.freq_hz = frequency;
    timer_1.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&timer_1);

  ledc_channel_config_t ch1;
    ch1.gpio_num = pinNumber;
    ch1.speed_mode = LEDC_LOW_SPEED_MODE;
    ch1.channel = LEDC_CHANNEL_1;
    ch1.intr_type = LEDC_INTR_DISABLE;
    ch1.timer_sel = LEDC_TIMER_1;
    ch1.duty = 0;
    ch1.hpoint = 0;
  ledc_channel_config(&ch1); 

  if (pinNumber==pwm){ //reset pin usage of the other PWM channel
    pinMode(buzzer,OUTPUT);
    digitalWrite(buzzer,LOW);
    pwmconfig=0;
    Serial.println("Now switching to drivng!");
  }else{
    pinMode(pwm,OUTPUT);
    digitalWrite(pwm,LOW);
    pwmconfig=1;
    Serial.println("Now switching to honking!");    
  }
  delay(10);

}

void ServoControl(int DutyCycle){
   ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0,DutyCycle);
   ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0);   
}

void MotorControl(int DutyCycle){
  if(pwmconfig==0){
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1,abs(DutyCycle));
    ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_1);
  }
}

void Buzzer(bool on){
  if(pwmconfig==1){ //prevent motor pwm outputs upon buzzer calling by checking the pwm channel configuration
    if(on){
      ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1,127);
      ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_1); 
    }else{
      ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1,0);
      ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_1); 
    }
  }
}

float readVoltage(){
  float val = 0;
  val = analogReadMilliVolts(vsense);
  val = (val/1000)*(100+22)/22;
  return val;
}
