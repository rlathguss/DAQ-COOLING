#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#define coolantsensorDivider1 71400                    //defines the resistor value that is in series in the voltage divider
#define coolantsensorDivider2 71900                    //defines the resistor value that is in series in the voltage divider
#define coolantsensorPin1 A0                          //defines the analog pin of the input voltage from the voltage divider
#define coolantsensorPin2 A1
#define NUMSAMPLES 3                          //defines the number of samples to be taken for a smooth average  
const float steinconstA = 0.00103425656083732;        //steinhart equation constant A, determined from wikipedia equations
const float steinconstB = 0.000191846988665616;       //steinhart equation constant B, determined from wikipedia equations
const float steinconstC = 0.000000180267870856939;    //steinhart equation constant C, determined from wikipedia equations
void tempsensor();




void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(200);
  tempsensor();
}



////////////////////////////////////////////CoolantTemp function/////////////////////////////////////////
void tempsensor(){  
  float average1 = 0;
  float average2 = 0;
  for (uint8_t i=0; i<NUMSAMPLES; i++) {                      
    average1 += analogRead(coolantsensorPin1);
    average2 += analogRead(coolantsensorPin2);    
    //Serial.print("average1 = ");
    //Serial.print(average1);Serial.println(" *v");    
    delay(10);
  }
  //average1 = average1*3.3/4.9;  
  average1 /= NUMSAMPLES;  
  average1 = (coolantsensorDivider1*average1)/(1023-average1);  
  //average2 = average2*3.3/4.9;  
  average2 /= NUMSAMPLES;  
  average2 = (coolantsensorDivider2*average2)/(1023-average2);  
  float steinhart1;                              
  float steinhart2;
  steinhart1 = log(average1);                      //lnR
  steinhart1 = pow(steinhart1,3);                  //(lnR)^3
  steinhart1 *= steinconstC;                       //C*((lnR)^3)
  steinhart1 += (steinconstB*(log(average1)));     //B*(lnR) + C*((lnR)^3)
  steinhart1 += steinconstA;                       //Complete equation, 1/T=A+BlnR+C(lnR)^3
  steinhart1 = 1.0/steinhart1;                     //Inverse to isolate for T
  steinhart1 -= 273.15; 

  steinhart2 = log(average2);                      //lnR
  steinhart2 = pow(steinhart2,3);                  //(lnR)^3
  steinhart2 *= steinconstC;                       //C*((lnR)^3)
  steinhart2 += (steinconstB*(log(average2)));     //B*(lnR) + C*((lnR)^3)
  steinhart2 += steinconstA;                       //Complete equation, 1/T=A+BlnR+C(lnR)^3
  steinhart2 = 1.0/steinhart2;                     //Inverse to isolate for T
  steinhart2 -= 273.15; 
  Serial.print("Coolant Temperature = ");
  Serial.print(steinhart1);Serial.println(" *C");
  Serial.print(" / ");Serial.print(steinhart2);Serial.println(" *C");              
  //CoolantTemp1 = steinhart1 ;
  //CoolantTemp2 = steinhart2 ;
  
  return; 
}