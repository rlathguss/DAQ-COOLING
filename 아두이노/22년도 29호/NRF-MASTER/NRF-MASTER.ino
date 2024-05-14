#include <SPI.h>
#include "RF24.h"
#include <ArduinoJson.h>
RF24 radio(9, 10); //CE, SS
uint8_t address[6] = "41715"; 
bool is_datapack4 = false;
bool is_datapack8 = false;


void setup() {
  Serial.begin(115200);
  radio.begin(); 
  radio.setPALevel(RF24_PA_MAX); 
  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address);
  radio.startListening(); 


/////////////////////////////////////////Slv Start 0000////////////////////////////////////////
//  String p1;
//  Serial.println("Start slv?");
//  while(1){
//    p1 = Serial.readString();
//    if(p1.indexOf("yes") != -1){
//      for(int i=0;i<30;i++){
//        radio.stopListening(); 
//        int i1 = 1;
//        radio.write(&i1,sizeof(i1));
//        Serial.println(i1);
//        delay(10);
//      }
//      break;
//    }
//    if(p1.indexOf("reset") != -1){
//    Serial.println("Restart the slv board ");
//      for(int i=0;i<30;i++){
//        int i3 = 40;
//        radio.stopListening();  
//        radio.write(&i3,sizeof(i3));
//        delay(10);
//      }
//    soft_restart();
//    }
//  }
//
///////////////////////////////////////////Radio check 1111//////////////////////////////////////
//  while(1){
//    if(radio.begin()){
//      int c1;
//      radio.startListening();
//      radio.read(&c1,sizeof(c1));
//      Serial.println(c1);
//      delay(10);
//        if (c1==257){ 
//          Serial.println("radio_connected");  
//          delay(500);              
//          break;
//        }
//    }
//    else{   
//      Serial.println("ERR_Radio");            
//      delay(500);
//      break;
//    }
//  }
//  
/////////////////////////////////////analogpin input check 2222//////////////////////////////////
//  while(1){
//    if(radio.begin()){
//      int c2;
//      radio.startListening();
//      radio.read(&c2,sizeof(c2));
//      Serial.println(c2);
//      delay(10);
//        if (c2==771){   
//          Serial.println("Analogsensor_connected"); 
//          delay(500);
//          break;
//        }
//        if (c2==1028){   
//          Serial.println("ERR_Analogsensor");            
//          delay(500);
//          break;
//        }
//    }
//  }
//
///////////////////////////////////////////Can Trans check 3333//////////////////////////////////
//  while(1){
//    if(radio.begin()){
//      int c3;
//      radio.startListening();
//      radio.read(&c3,sizeof(c3));
//      Serial.println(c3);  
//      delay(10);
//        if (c3==1285){   
//         Serial.println("CanTrans_connected");
//         delay(500);
//         break;
//        }
//        if (c3==1542){   
//          Serial.println("ERR_Digitalsensor");            
//          delay(500);
//          break;
//        }
//    }
//  }
//
///////////////////////////////////////////Record OR Reset///////////////////////////////////////
//  String p2;
//  Serial.println("Start recording? ");
//  while(1){
//    p2 = Serial.readString();
//    if(p2.indexOf("yes") != -1){
//      for(int i=0;i<30;i++){
//        radio.stopListening(); //송신모드
//        int i2 = 10;
//        radio.write(&i2,sizeof(i2));
//        Serial.println(i2);
//      }
//      Serial.println("Senor1, Senor2, Senor3, RPM, Lin_vel, Motor_temp, Heatsink_temp, Motor_torque, Torque_demand, Motor_vol, Motor_curr, Batt_vol, Batt_curr, Throttle_input_vol"); // legend
//      break;
//    }
//
//    if(p1.indexOf("reset") != -1){
//      Serial.println("Restart the slv board ");
//      for(int i=0;i<30;i++){
//        int i3 = 40;
//        radio.stopListening();  
//        radio.write(&i3,sizeof(i3));
//        delay(10);
//      }
//    soft_restart();
//    }
//  }
//Serial.println("Senor1, Senor2, Senor3, RPM, Lin_vel, Motor_temp, Heatsink_temp, Motor_torque, Torque_demand, Motor_vol, Motor_curr, Batt_vol, Batt_curr, Throttle_input_vol"); // legend
}



void loop() {
  
////////////////////////////////////////Datapack RF Trans//////////////////////////////////////
  bool is_timeout = false;
  unsigned long t = millis();
  while(true){
    if(radio.available()){break;
      if(millis() - t > 3000){
      is_timeout = true;
      break;
      }
    }
  } 
  
  if(is_timeout){
    Serial.println("통신시간 초과!");
  }
  else{
    float datapack[8],datalog[14];
    radio.read(datapack,sizeof(datapack));
    if (datapack[0]==4){
      datalog[0]=datapack[1];
      datalog[1]=datapack[2];
      datalog[2]=datapack[3];
      datalog[3]=datapack[4];
      datalog[4]=datapack[5];
      datalog[5]=datapack[6];
      datalog[6]=datapack[7];
      is_datapack4 = true;
    }
    if (datapack[0]==8){
      datalog[7]=datapack[1];
      datalog[8]=datapack[2];
      datalog[9]=datapack[3];
      datalog[10]=datapack[4];
      datalog[11]=datapack[5];
      datalog[12]=datapack[6];
      datalog[13]=datapack[7];
      is_datapack8 = true;
    }
    if(is_datapack4&&is_datapack8){
//      Serial.print(datalog[0]);Serial.print("  ");Serial.print(datalog[1]);Serial.print("  ");
//      Serial.print(datalog[2]);Serial.print("  ");Serial.print(datalog[3]);Serial.print("  ");
//      Serial.print(datalog[4]);Serial.print("  ");Serial.print(datalog[5]);Serial.print("  ");
//      Serial.print(datalog[6]);Serial.print("  ");Serial.print(datalog[7]);Serial.print("  ");
//      Serial.print(datalog[8]);Serial.print("  ");Serial.print(datalog[9]);Serial.print("  ");
//      Serial.print(datalog[10]);Serial.print("  ");Serial.print(datalog[11]);Serial.print("  ");
//      Serial.print(datalog[12]);Serial.print("  ");Serial.println(datalog[13]);

        String output;
        StaticJsonDocument<128> doc;

        doc["Sensor1"] = datalog[0];
        doc["Sensor2"] = datalog[1];
        doc["Sensor3"] = datalog[2];
        doc["RPM"] = datalog[3];
        doc["Lin_vel"] = datalog[4];
        doc["Motor_temp"] = datalog[5];
        doc["Heatsink_temp"] = datalog[6];
        doc["Motor_torque"] = datalog[7];
        doc["Torque_demand"] = datalog[8];
        doc["Motor_vol"] = datalog[9];
        doc["Motor_curr"] = datalog[10];
        doc["Batt_vol"] = datalog[11];
        doc["Batt_curr"] = datalog[12];
        doc["Throttle_input_vol"] = datalog[13];
        
        serializeJson(doc, output);
        Serial.println(output);
        is_datapack4 = false; is_datapack8 = false;
    }
  }
  delay(10);
}

//Serial.println("Senor1, Senor2, Senor3, RPM, Lin_vel, Motor_temp, Heatsink_temp, Motor_torque, Torque_demand, Motor_vol, Motor_curr, Batt_vol, Batt_curr, Throttle_input_vol");
//{"Sensor1":123,"Sensor2":123,"Sensor3":123,"RPM":123,"Lin_vel":123,"Motor_temp":123,"Heatsink_temp":123,"Motor_torque":123,"Torque_demand":123,"Motor_vol":123,"Motor_curr":123,"Batt_vol":123,"Batt_curr":123,"Throttle_input_vol":123,}
