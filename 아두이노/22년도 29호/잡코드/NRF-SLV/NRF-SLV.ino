#include <Arduino.h>
#include <TM1637Display.h>
#include <TimerOne.h>
#include <SPI.h>
#include "RF24.h"
#include "SoftReset.h"

RF24 radio(9, 10); //CE, SS
uint8_t address[6] = "04171";

#define coolantsensorDivider 4700   
const float steinconstA = 0.001092768344245138;       
const float steinconstB = 0.000181450723833218;      
const float steinconstC = 0.000000222855858126706000;   

const byte CLK1 = 3;   // define CLK pin (any digital pin)
const byte DIO1 = 4;   // define DIO pin (any digital pin)
int interruptPin = 2;
const float Diameter = 325.0;  //mm단위
const uint8_t SEG_DONE[] = {
  SEG_B | SEG_C | SEG_D | SEG_E | SEG_G,           // d
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,   // O
  SEG_C | SEG_E | SEG_G,                           // n
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G            // E
};
const uint8_t SEG_ERR1[] = {
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G,            // E
  SEG_E | SEG_G,                                    // R
  SEG_E | SEG_G,                                    // R
  SEG_B | SEG_C                                     // 1
};
const uint8_t SEG_ERR2[] = {
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G,            // E
  SEG_E | SEG_G,                                    // R
  SEG_E | SEG_G,                                    // R
  SEG_A | SEG_B | SEG_D | SEG_E | SEG_G             // 2
};
const uint8_t SEG_ERR3[] = {
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G,            // E
  SEG_E | SEG_G,                                    // R
  SEG_E | SEG_G,                                    // R
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_G             // 3
};
const uint8_t SEG_RECD[] = {
  SEG_E | SEG_G,                                    // R
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G,            // E
  SEG_A | SEG_D | SEG_E | SEG_F,                    // C
  SEG_B | SEG_C | SEG_D | SEG_E | SEG_G             // d
};
TM1637Display display(CLK1, DIO1);// define dispaly object
uint8_t data[] = { 0x0, 0x0, 0x0, 0x0 };  // all segments clear
int rpm=0;
float rev=0;
unsigned long oldtime=0;
unsigned long times;
int lin_vel=0;

void setup(){
  display.setBrightness(0x0f);
  display.showNumberDec(9999);
  delay(1000);
  Serial.begin(115200);
  radio.begin(); //아두이노-RF모듈간 통신라인
  radio.setPALevel(RF24_PA_MAX); 
  radio.openWritingPipe(address);
  radio.openReadingPipe(0, address);
  radio.stopListening(); //기본 :  송신모드

  while(1){
    if(radio.begin()){
      int i1;
      radio.startListening();  //수신모드 master 보드에서 실행하는 신호를 보내야지 진행가능
      radio.read(&i1,sizeof(i1));
      //Serial.println(i1);
      delay(10);
        if(i1 == 257){
          Serial.println("Slv started");
          delay(500);
          break;
        }
        if (i1 == 10280){ 
          soft_restart();
        }
     }
  }

  for(int i=0;i<30;i++){
    if (radio.begin()){                        //-------- 통신이 잘 시작되면 1111 출력
      display.showNumberDec(1111);
      int c1 = 1;
      radio.stopListening();
      radio.write(&c1,sizeof(c1));
      //Serial.println(c1);
      delay(10);
    }
    else{
      display.setSegments(SEG_ERR1);
      radio.stopListening();
      int c2 = 2;
      radio.write(&c2,sizeof(c2));
      //Serial.println(c2);
      delay(10);
    }
  }
  for(int i=0;i<30;i++){
    if(analogRead(A0)<1000&&analogRead(A1)<1000&&analogRead(A2)<1000){ //센서값 잘 들어오면 2222 출력
      display.showNumberDec(2222);
      int c3 = 3;
      radio.stopListening();
      radio.write(&c3,sizeof(c3));
      //Serial.println(c3);
      delay(10);
    }
    else{
      display.setSegments(SEG_ERR2);
      int c4 = 4;
      radio.stopListening();
      radio.write(&c4,sizeof(c4));
      //Serial.println(c4);
      delay(10);
    }
  }
  for(int i=0;i<30;i++){
    if(digitalRead(interruptPin)==LOW||digitalRead(interruptPin)==HIGH){ //디지털에서 신호 양호시 3333 출력
      display.showNumberDec(3333);
      int c5= 5;
      radio.stopListening();
      radio.write(&c5,sizeof(c5));
      //Serial.println(c5);
      delay(10);
    }
    else{
      display.setSegments(SEG_ERR3);
      int c6 = 6;
      radio.stopListening();
      radio.write(&c6,sizeof(c6));
      //Serial.println(c6);
      delay(10);
    }
  }

  while(true){
    if(radio.begin()){
      int i2;
      radio.startListening();  //수신모드 master 보드에서 실행하는 신호를 보내야지 진행가능
      radio.read(&i2,sizeof(i2));
      //Serial.println(i2);
      delay(10);
        if(i2 == 2570){
          Serial.println("Start recording");
          display.setSegments(SEG_RECD);
          delay(500);
          break;
        }
        if (i2 == 10280){ 
          soft_restart();
        }
     }
  }
attachInterrupt(digitalPinToInterrupt(interruptPin),isr,FALLING);
}


void loop() {
  delay(300);
  detachInterrupt(digitalPinToInterrupt(interruptPin));
  times = micros()-oldtime;
  rpm=(rev/times)*60000000;  // 60초 곱하기
  //Serial.println(times);
  oldtime = micros();
  rev=0;
  lin_vel = (Diameter) * rpm * PI * 60/1000000;   //KM/H
  int lin_vel_16 = lin_vel;
  display.showNumberDec(lin_vel_16);
  attachInterrupt(digitalPinToInterrupt(interruptPin),isr,FALLING);

  float sensor_value1 = analogRead(A0),sensor_value2= analogRead(A1),sensor_value3 = analogRead(A2);
  sensor_value1 = (coolantsensorDivider*sensor_value1)/(1023-sensor_value1);
  sensor_value2 = (coolantsensorDivider*sensor_value2)/(1023-sensor_value2);
  sensor_value3 = (coolantsensorDivider*sensor_value3)/(1023-sensor_value3);     
  
  float sen1,sen2,sen3;                         

  sen1 = log(sensor_value1);                   
  sen1 = pow(sen1,3);                 
  sen1 *= steinconstC;                     
  sen1 += (steinconstB*(log(sensor_value1)));    
  sen1 += steinconstA;                     
  sen1 = 1.0/sen1;                  
  sen1 -= 273.15;   

  sen2 = log(sensor_value2);                   
  sen2 = pow(sen2,3);                 
  sen2 *= steinconstC;                     
  sen2 += (steinconstB*(log(sensor_value2)));    
  sen2 += steinconstA;                     
  sen2 = 1.0/sen2;                  
  sen2 -= 273.15; 

  sen3 = log(sensor_value3);                   
  sen3 = pow(sen3,3);                 
  sen3 *= steinconstC;                     
  sen3 += (steinconstB*(log(sensor_value3)));    
  sen3 += steinconstA;                     
  sen3 = 1.0/sen3;                  
  sen3 -= 273.15;
  float sen11=sen1,sen22=sen2,sen33=sen3;
  
  float sensor_data[4];
  sensor_data[0]=sen11; sensor_data[1]=sen22; sensor_data[2]=sen33; sensor_data[3]=lin_vel_16;
  radio.stopListening(); //송신모드
  radio.write(sensor_data,sizeof(sensor_data));

  int i3;
  radio.startListening();  //수신모드 master 보드에서 실행하는 신호를 보내야지 진행가능
  radio.read(&i3,sizeof(i3));
  if (i3 == 40){ 
    soft_restart();
  }
}

void isr(){
  rev++;
}
