#include <Arduino.h>
#include <TM1637Display.h>
#include <TimerOne.h>
#include <SPI.h>
#include <SD.h>

const int chipSelect = 4;
#define coolantsensorDivider 4700   
const float steinconstA = 0.001092768344245138;       
const float steinconstB = 0.000181450723833218;      
const float steinconstC = 0.000000222855858126706000;   

const byte CLK1 = 3;   // define CLK pin (any digital pin)
const byte DIO1 = 4;   // define DIO pin (any digital pin)
int interruptPin = 2;
const float Diameter = 100.0;  // mm 단위로
const uint8_t SEG_DONE[] = {
  SEG_B | SEG_C | SEG_D | SEG_E | SEG_G,           // d
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,   // O
  SEG_C | SEG_E | SEG_G,                           // n
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G            // E
};
const uint8_t SEG_ERR[] = {
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G,            // E
  SEG_E | SEG_G, // R
  SEG_E | SEG_G, // R
};
 
TM1637Display display1(CLK1, DIO1);// define dispaly 1 object
uint8_t data[] = { 0x0, 0x0, 0x0, 0x0 };  // all segments clear
int rpm;
float rev=0;
unsigned long oldtime=0;
unsigned long times;
int lin_vel=0;

void setup() {

  display1.setBrightness(0x0f);// set brightness of dispaly 1
  Serial.begin(9600);
  display1.setSegments(SEG_DONE);
}

void isr(){
  rev++;
}

void vel(){
  lin_vel = (Diameter) * rpm * PI * 60/1000000;   //KM/H
  int lin_vel_16 = lin_vel;
  Serial.println(lin_vel);
  display1.showNumberDec(lin_vel_16);
}
 
void loop() {
  attachInterrupt(digitalPinToInterrupt(interruptPin),isr,FALLING);
  delay(500);
  detachInterrupt(digitalPinToInterrupt(interruptPin));
  times = micros()-oldtime;
  rpm=(rev/times)*60000000;  // 60초 곱하기
  //Serial.println(rpm);
  oldtime=micros();
  rev=0;
  vel();
  
  
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
  float time=millis()/1000.0;
//  Serial.print(time);
//  Serial.print(",");Serial.print(sen1);
//  Serial.print(",");Serial.print(sen2);
//  Serial.print(",");Serial.println(sen3);
}
