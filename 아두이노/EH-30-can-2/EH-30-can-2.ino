#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <SPI.h>
#include <mcp2515.h>
#include "RF24.h"
#include "Wire.h"
#include <MPU6050_light.h>
#include <TM1637Display.h>

const byte CLK = 2;                                   // define CLK pin (any digital pin)
const byte DIO = 4;                                   // define DIO pin (any digital pin)
TM1637Display display(CLK, DIO);                      // define dispaly object
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

#define coolantsensorDivider 10000                    //defines the resistor value that is in series in the voltage divider
#define coolantsensorPin1 A1                          //defines the analog pin of the input voltage from the voltage divider
#define coolantsensorPin2 A2
#define NUMSAMPLES 3                                  //defines the number of samples to be taken for a smooth average  
const float steinconstA = 0.0012176747872224;         //steinhart equation constant A, determined from wikipedia equations
const float steinconstB = 0.000166354853312481;       //steinhart equation constant B, determined from wikipedia equations
const float steinconstC = 0.000000250091367737019;    //steinhart equation constant C, determined from wikipedia equations
void tempsensor();

MPU6050 mpu(Wire);
long timer = 0;
RF24 radio(9, 8); //CE, CS
uint8_t address[6] = "41715";

#define rx 0x201
#define tx 0x181
MCP2515 mcp2515(10);   //cs핀 번호 설정
byte recv[8];
byte id;
void mcp2515_send(unsigned int sid, byte data[], byte dlc);
byte mcp2515_receive();


int16_t SpeedRpmMax = 5000, CurrentDevice = 2000, Current200pc = 1070, I_Max_PK = 424.2;   //0xCE, 0xC6, 0xD9, 0xC4
int16_t SpeedActual, RPM, Motor_Temp, PowerStage_Temp, LPM,                                //0x30, 0x49, 0x4A
        M_Out, CurrentActual, DCvoltage, ACvoltage,                                        //0xA0, 0x5F, 0xEB, 0x8A 
        Xaccle, Yaccle, Zaccle, Xangle, Yangle, 
        Rad, CoolantTemp1, CoolantTemp2;
uint16_t Status1, Status2, Err1, Err2;                                                     //0X40, 0X8F
float LPM1, Xaccle1, Yaccle1, Zaccle1;


bool bSpeedActual = false;
bool bMotor_Temp = false;
bool bPowerStage_Temp = false;
bool bM_Out = false;
bool bCurrentActual = false;
bool bDCvoltage = false;
bool bACvoltage = false;
bool bStatus1 = false;
bool bStatus2 = false;
bool bErr1 = false;
bool bErr2 = false;

unsigned long sft;                                  //start function time
bool bsft = true;                                   //can 통신 실패시 재요청 코드 초기 설정은 켜짐
bool bRad = true;                                   //Rad 속도 측정 함수 처음엔 자동 켜짐


volatile int flow_frequency;                        // Measures flow sensor pulses
unsigned char flowsensor = 3;                       // Sensor Input
unsigned long currentTime1;
unsigned long cloopTime1;


unsigned long currentTime2;
unsigned long cloopTime2;
const int aena = 5;
const int adir1 = 6;
const int adir2 = 7;


void setup() {
  Serial.begin(115200);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS);
  mcp2515.setNormalMode();
  Serial.println("Mcp2515 setup success");

  radio.begin();                                           //아두이노-RF모듈간 통신라인
  radio.setPALevel(RF24_PA_MAX);
  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address);
  radio.stopListening();                                   //기본 : 송신모드
  Serial.println("NRF24 setup success");

  Wire.begin();                                            // 가속도 센서 부팅
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  if (status == 0){
    display.setSegments(SEG_DONE);
  }
  delay(1000);
  mpu.calcOffsets(true,true);                              // gyro and accelero
  Serial.println("Done!\n");


  pinMode(flowsensor, INPUT);
  digitalWrite(flowsensor, HIGH);
  attachInterrupt(digitalPinToInterrupt(3), flow, RISING); // Setup Interrupt
  currentTime1 = millis();
  cloopTime1 = currentTime1;
  sft = currentTime1;
  Serial.println("Flow sensor ready");

  pinMode(aena, OUTPUT);                                  //l298n 모터 a 활성화
  pinMode(adir1, OUTPUT);                                 //l298n 모터 a 활성화 
  pinMode(adir2, OUTPUT);                                 //l298n 모터 a 활성화 
  currentTime2 = millis();
  cloopTime2 = currentTime2;
  analogWrite(aena, 0);                                   // 팬 정지 설정
  digitalWrite(adir1, 1);                                 // 디지털 6번 전진
  digitalWrite(adir2, 0);                                 // 디지털 7번 전진
  Serial.println("L298N to radiator connect / fan speed = 0 ");
  Rad = 0;

  byte data[3] = { 0x3D,0x30,0x96 };
  mcp2515_send(rx, data, 3); //Request SpeedActual 150ms
  delay(10);
  data[1] = 0x49;
  mcp2515_send(rx, data, 3); //Request Motor_Temp 150ms
  delay(10);
  data[1] = 0x4A;
  mcp2515_send(rx, data, 3); //Request PowerStage_Temp 150ms
  delay(10);
  data[1] = 0xA0;
  mcp2515_send(rx, data, 3); //Request M_out 150ms
  delay(10);
  data[1] = 0x5F;
  mcp2515_send(rx, data, 3); //Request CurrentActual 150ms
  delay(10);
  data[1] = 0xEB;
  mcp2515_send(rx, data, 3); //Request DCvoltage 150ms
  delay(10);
  data[1] = 0x8A;
  mcp2515_send(rx, data, 3); //Request ACvoltage 150ms
  delay(10);
  data[1] = 0x40;
  mcp2515_send(rx, data, 3); //Request Status 150ms
  delay(10);
  data[1] = 0x8F;
  mcp2515_send(rx, data, 3); //Request Err 150ms
  Serial.println("Request data transfer success");
}


void loop() {
  mpu.update();
  radio.startListening();
  if (radio.available()){
    int16_t rrf[1];
    radio.read(rrf,sizeof(rrf));
    if (rrf[0] == 9){
      //Serial.println(rrf[0]);
      bRad = false;
      //Serial.println("명령 전달 양호 팬 속도 최대 설정9999999999999999999999999999999"); 
    }
    else if (rrf[0] == 4){
      //Serial.println(rrf[0]);
      bRad = true;
      //Serial.println("팬 속도 자동 제어 설정444444444444444444444444444444444444"); 
    }
  }

  tempsensor();
  accsen();

  currentTime1 = millis();                       // Every 0.2 second, calculate and print litres/min
  if (currentTime1 >= (cloopTime1 + 200)) {
    cloopTime1 = currentTime1;                   // Updates cloopTime
    LPM1 = (5.0f * flow_frequency / 11.0f);      // Pulse frequency (Hz) = 11Q, Q is flow rate in L/min.
    flow_frequency = 0;
    LPM = LPM1 * 10;                               
    Serial.print(LPM1);                          // Print litres/min
    Serial.println(" L/Min");
  }
  
  for (;;) {
    id = mcp2515_receive();
    mpu.update();
    if (id == 1) { 
      if (bsft == true && currentTime1>=(sft +10000)){             // 아두이노 부팅이후 캔 통신 데이터 10초 이후에도 없는 경우 재요청 코드
        byte data[3] = { 0x3D,0x30,0x96 };
        mcp2515_send(rx, data, 3); //Request SpeedActual 150ms
        delay(10);
        data[1] = 0x49;
        mcp2515_send(rx, data, 3); //Request Motor_Temp 150ms
        delay(10);
        data[1] = 0x4A;
        mcp2515_send(rx, data, 3); //Request PowerStage_Temp 150ms
        delay(10);
        data[1] = 0xA0;
        mcp2515_send(rx, data, 3); //Request M_out 150ms
        delay(10);
        data[1] = 0x5F;
        mcp2515_send(rx, data, 3); //Request CurrentActual 150ms
        delay(10);
        data[1] = 0xEB;
        mcp2515_send(rx, data, 3); //Request DCvoltage 150ms  
        delay(10);
        data[1] = 0x8A;
        mcp2515_send(rx, data, 3); //Request ACvoltage 150ms
        delay(10);
        data[1] = 0x40;
        mcp2515_send(rx, data, 3); //Request Status 150ms
        delay(10);
        data[1] = 0x8F;
        mcp2515_send(rx, data, 3); //Request Err 150ms
      }
      display.setSegments(SEG_ERR1);
      mpu.update();
      break;
    }
    else if (id == 0x30) {                                          // SpeedActual
      SpeedActual  = ((int16_t)recv[2] << 8) | recv[1];
      bSpeedActual = true;  
      RPM = SpeedRpmMax * ( SpeedActual / 32767);
      display.showNumberDec(RPM);      
    }
    else if (id == 0x49) {                                          // Motor_Temp
      Motor_Temp = ((int16_t)recv[2] << 8) | recv[1];
      bMotor_Temp = true;
    }
    else if (id == 0x4A) {                                          // PowerStage_Temp
      PowerStage_Temp = ((int16_t)recv[2] << 8) | recv[1];
      bPowerStage_Temp = true;
    }
    else if (id == 0xA0) {                                          //M_Out
      M_Out = ((int16_t)recv[2] << 8) | recv[1];
      bM_Out = true;
    }
    else if (id == 0x5F) {                                          // CurrentActual
      CurrentActual = ((int16_t)recv[2] << 8) | recv[1];       
      bCurrentActual = true;
    } 
    else if (id == 0xEB) {                                          // DCvoltage
      DCvoltage = ((int16_t)recv[2] << 8) | recv[1];       
      bDCvoltage = true;
    } 
    else if (id == 0x8A) {                                          // ACvoltage  4바이트 송신에서 low면 4번 3번 high면 2번 1번
      ACvoltage = ((int16_t)recv[4] << 8) | recv[3];       
      bACvoltage = true;
    }
    else if (id == 0x40) {                                          // Status    
      Status1 = ((uint16_t)recv[2] << 8) | recv[1];
      Status2 = ((uint16_t)recv[4]<< 8) | recv[3];
      bStatus1 = true;
      bStatus2 = true;
    }
    else if (id == 0x8F) {                                          // Err   
      Err1 = ((uint16_t)recv[2] << 8) | recv[1];
      Err2 = ((uint16_t)recv[4]<< 8) | recv[3];
      bErr1 = true;
      bErr2 = true;
    }
    bsft = false; 
    if (bSpeedActual == true && bMotor_Temp == true && bPowerStage_Temp == true && bM_Out == true && bCurrentActual == true && bDCvoltage == true && bACvoltage == true && bStatus1 == true  && bStatus2 == true && bErr1 == true && bErr2 == true){
      bSpeedActual = false;
      bMotor_Temp = false;
      bPowerStage_Temp= false;
      bM_Out = false;
      bCurrentActual = false;
      bDCvoltage = false;
      bACvoltage = false;
      bStatus1 = false;
      bStatus2 = false;
      bErr1 = false;
      bErr2 = false;
      break;
    }
  }

  currentTime2 = millis();                         // Every second
  if (currentTime2 >= (cloopTime2 + 1000) && bRad == true) {
    cloopTime2 = currentTime2;
    if(id == 1){
      analogWrite(aena, 255);                       //디지털 5번 핀이 a 모터 속도 조절 핀  0~255
      digitalWrite(adir1, 1);
      digitalWrite(adir2, 0);
      //Serial.println("can 통신 실패 팬속도 최대"); 
      Rad = 1; 
    }
    else if (Motor_Temp >= 11646 || PowerStage_Temp >= 20250) {  //40도
      analogWrite(aena, 255);
      digitalWrite(adir1, 1);
      digitalWrite(adir2, 0);
      //Serial.println("온도 40 이상 팬속도 최대");
      Rad = 255; 
    }
    else if (Motor_Temp >= 11364 || PowerStage_Temp >= 19733) {  //35도
      analogWrite(aena, 200);
      digitalWrite(adir1, 1);
      digitalWrite(adir2, 0);
      //Serial.println("온도 35 이상 팬속도 200");
      Rad = 200; 
    }
    else if (Motor_Temp >= 11080 || PowerStage_Temp >= 19247) {  //30도
      analogWrite(aena, 150);
      digitalWrite(adir1, 1);
      digitalWrite(adir2, 0);
      //Serial.println("온도 30 이상 팬속도 150");
      Rad = 150; 
    }
    else if (Motor_Temp >= 10795 || PowerStage_Temp >= 18797) {  //25도
      analogWrite(aena, 100);
      digitalWrite(adir1, 1);
      digitalWrite(adir2, 0);                
      //Serial.println("온도 25 이상 팬속도 100");        
      Rad = 100; 
    }
    else if (Motor_Temp >= 10510 || PowerStage_Temp >= 18387) {  //20도
      analogWrite(aena, 50); 
      digitalWrite(adir1, 1);
      digitalWrite(adir2, 0);
      //Serial.println("온도 20 이상 팬속도 50");                       
      Rad = 50; 
    }
    else{
      digitalWrite(aena, 0); 
      digitalWrite(adir1, 1);
      digitalWrite(adir2, 0); 
      //Serial.println("팬 off");
      Rad = 0; 
    }
  }
  else if (bRad == false){
    analogWrite(aena, 255);
    digitalWrite(adir1, 1);
    digitalWrite(adir2, 0);
    Rad = 9; 
  }  
  

  //Datapack RF Trans
  int16_t datapack1[16];
  datapack1[0] = 11,
  datapack1[1] = RPM, datapack1[2] = Motor_Temp, datapack1[3] = PowerStage_Temp, datapack1[4] = LPM, 
  datapack1[5] = M_Out, datapack1[6] = CurrentActual, datapack1[7] = DCvoltage, datapack1[8] = ACvoltage,
  datapack1[9] = Xaccle, datapack1[10] = Yaccle, datapack1[11] = Zaccle, datapack1[12] = Xangle,
  datapack1[13] = Yangle, datapack1[14] = CoolantTemp1 , datapack1[15] = CoolantTemp2;

  radio.stopListening(); //송신모드
  radio.write(datapack1, sizeof(datapack1));
  
  uint16_t datapack2[16];
  datapack2[0] = 22,
  datapack2[1] = Rad, datapack2[2] = Status1, datapack2[3] = Status2, datapack2[4] = Err1, datapack2[5] = Err2;

  radio.stopListening(); //송신모드
  radio.write(datapack2, sizeof(datapack2));
}

  

//////////////////////////////////////////////////////send/////////////////////////////////////////
void mcp2515_send(unsigned int sid, byte data[], byte dlc) {
  struct can_frame canMsg;
  canMsg.can_id = sid;                    //슬레이브의 ID
  canMsg.can_dlc = dlc;
  for (int i = 0; i < dlc; i++) {
    canMsg.data[i] = data[i];
  }
  mcp2515.sendMessage(&canMsg);
  // Serial.println("[모터 컨트롤러에 보낸 메시지]");
  // Serial.println(sid, HEX);
  // for (int i = 0; i < canMsg.can_dlc; i++) {
  //   Serial.print(canMsg.data[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println();
  return;
}

///////////////////////////////////////////////////receive/////////////////////////////////////////
byte mcp2515_receive() {
  struct can_frame canMsg;
  unsigned long t = millis();
  byte ret;
  while (1) {
    if (millis() - t > 100) {
      ret = (byte)1;
      //Serial.println(ret);
      break;
    }
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK && canMsg.can_id == tx) {
      int dlc = canMsg.can_dlc;
      //Serial.println("[모터 컨트롤러에서 받은 메시지]");
      //Serial.println(canMsg.data[0], HEX);
      for (int i = 0; i < dlc; i++) {
        recv[i] = canMsg.data[i];
        //Serial.print(recv[i], HEX);
        //Serial.print(" ");
      }
      //Serial.println();
      ret = canMsg.data[0];
      break;
    }
  }
  return ret;
}

////////////////////////////////////////////가속도 센서/////////////////////////////////////////
void accsen(){
  mpu.update();
  //Lv_Temp1 = mpu.getTemp();
  //Lv_Temp = Lv_Temp1 * 100; 
  //Serial.print(F("TEMPERATURE: "));Serial.println(Lv_Temp1);
  Xaccle1 = mpu.getAccX();
  Xaccle= Xaccle1 * 10;
  //Serial.print(F("ACCELERO  X: "));Serial.print(Xaccle1);
  Yaccle1 = mpu.getAccY();
  Yaccle = Yaccle1 * 10;
  //Serial.print("\tY: ");Serial.print(Yaccle1);
  Zaccle1 = mpu.getAccZ();
  Zaccle = Zaccle1 * 10;
  //Serial.print("\tZ: ");Serial.println(Zaccle1);
  Xangle = mpu.getAngleX();
  //Serial.print(F("ANGLE     X: "));Serial.print(Xangle);
  Yangle = mpu.getAngleY();
  //Serial.print("\tY: ");Serial.println(Yangle);
  //Serial.println(F("=====================================================\n"));
  timer = millis();
  return;
}

////////////////////////////////////////////CoolantTemp function/////////////////////////////////////////
void tempsensor(){  
  float average1 = 0;
  float average2 = 0;
  for (uint8_t i=0; i<NUMSAMPLES; i++) {                      
    average1 += analogRead(coolantsensorPin1);
    average2 += analogRead(coolantsensorPin2);        
    delay(10);
  }
  average1 /= NUMSAMPLES;  
  average1 = (coolantsensorDivider*average1)/(1023-average1);  
  average2 /= NUMSAMPLES;  
  average2 = (coolantsensorDivider*average2)/(1023-average2);  
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
  //Serial.print("Coolant Temperature = ");
  //Serial.print(steinhart1);Serial.print(" *C");Serial.print(" / ");Serial.print(steinhart2);Serial.println(" *C");              
  CoolantTemp1 = steinhart1 * 10;
  CoolantTemp2 = steinhart2 * 10;
  return; 
}

////////////////////////////////////////////Interrupt function/////////////////////////////////////////
void flow(){
  flow_frequency ++;
  return;
}
