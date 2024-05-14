#include <SPI.h>
#include <SD.h>
#include <TimerOne.h>

int j=0;
const int chipSelect = 4;
float time;

#define coolantsensorDivider 4700   
             
const float steinconstA = 0.001092768344245138;       
const float steinconstB = 0.000181450723833218;      
const float steinconstC = 0.000000222855858126706000;   

void setup() {
    Serial.begin(9600);
    Serial.print("Initializing SD card...");

    if (!SD.begin(chipSelect)) {
       Serial.println("Card failed, or not present");
       while (1);
    }
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    dataFile.println("");dataFile.close();
    Serial.println("card initialized.");
    Serial.println("sneor1,sneor2,sneor3"); // legend
}


void loop() {

  float sensor_value1 = analogRead(A0),sensor_value2= analogRead(A1),sensor_value3 = analogRead(A2);
  delay(10);

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
   
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
    if (dataFile){
        time = millis()/1000.0;
        Serial.print(time);
        Serial.print(",");Serial.print(sen1);
        Serial.print(",");Serial.print(sen2);
        Serial.print(",");Serial.println(sen3);
        dataFile.print(String(time));
        dataFile.print(" ");dataFile.print(String(sen1));
        dataFile.print(" ");dataFile.print(String(sen2));
        dataFile.print(" ");dataFile.println(String(sen3));
        dataFile.close();
    }
    else{
     Serial.println("error opening datalog.txt");
     while(1);
    }
}
