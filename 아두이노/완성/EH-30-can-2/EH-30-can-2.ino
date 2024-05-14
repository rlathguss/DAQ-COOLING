#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <SPI.h>
#include <mcp2515.h>  
#include <RF24.h>   
#include <Wire.h>    
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TM1637Display.h>

const byte CLK = 3;                                   // define CLK pin (any digital pin)s
const byte DIO = 4;                                   // define DIO pin (any digital pin)
TM1637Display display(CLK, DIO);                      // define dispaly object

void accsen(sensors_event_t* event);
void accsen1(sensors_event_t* event);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
sensors_event_t orientationData, linearAccelData;

#define coolantsensorDivider1 71500                    //defines the resistor value that is in series in the voltage divider
#define coolantsensorDivider2 71300                    //defines the resistor value that is in series in the voltage divider
#define coolantsensorPin1 A0                          //defines the analog pin of the input voltage from the voltage divider
#define coolantsensorPin2 A1
#define NUMSAMPLES 50                                 //defines the number of samples to be taken for a smooth average  
const float steinconstA = 0.00103425656083732;        //steinhart equation constant A
const float steinconstB = 0.000191846988665616;       //steinhart equation constant B
const float steinconstC = 0.000000180267870856939;    //steinhart equation constant C
void tempsensor();

float mt(uint16_t x);
float pst(uint16_t x);

void Radiator_Fan_Control();
HardwareTimer pwmtimer3(TIM3);
const int REN = 7;           //정회전 활성화 핀
const int LEN = 6;           //역회전 활성화 핀
//PB4 RPWM PIN D5

RF24 radio(9, 8);             //CE, CS
uint8_t address[6] = "41715";
int16_t rrf[3] = { 0, };

#define rx 0x201
#define tx 0x181
MCP2515 mcp2515(10);   //cs핀 번호 설정
byte recv[8];
byte id;
void Clear();
void Request();
void mcp2515_send(unsigned int sid, byte data[], byte dlc);
byte mcp2515_receive();

//int16_t SpeedRpmMax = 5000, CurrentDevice = 2000, Current200pc = 1070, I_Max_PK = 424.2;  //0xCE, 0xC6, 0xD9, 0xC4
int16_t SpeedActual, Motor_Temp, PowerStage_Temp,                                           //0x30, 0x49, 0x4A
        Lq_actual, CurrentActual, DCvoltage, ACvoltage,                                     //0xA0, 0x5F, 0xEB, 0x8A 
        Roll, Pitch, Rad, roll = 0, pitch =0;
uint16_t Status1, Status2, ERROR1, ERROR2; //LOGIC;                                         //0X40 0X8F
float Torque_Out, CoolantTemp1, CoolantTemp2, LPM, LT_accele, LN_accele,  
      fACcurrent, fACpower1, fACpower2, fDCvoltage, fACvoltage, Cp = 0, Hp = 0;
int RPM;

bool bSpeedActual = false;
bool bMotor_Temp = false;
bool bPowerStage_Temp = false;
bool bLq_actual = false;
bool bCurrentActual = false;
bool bDCvoltage = false;
bool bACvoltage = false;
bool bStatus = false;
bool bERROR = false;
//bool bLOGIC = false;

unsigned long sft = 0, SAFE = 0;                    //start function time
bool bsft = true;                                   //can 통신 실패시 재요청 코드 초기 설정은 켜짐
bool bRad = true;                                   //Rad 속도 측정 함수 처음엔 자동 켜짐
bool bhcRad = false;                                // 기본은 온도기반 설정

volatile int flow_frequency;                        // Measures flow sensor pulses
unsigned char flowsensor = 2;                       // Sensor Input
unsigned long Endtime = 0, interval;

typedef struct vector {int x;int y;}vector2;
typedef struct pp {vector2* points;int size;} polygon;
void Eff(int a, int b);
int M_efficiency;
int isInside(vector2 B, const polygon* p);
 
vector2 point1 = { 3455.77, 115.556 };          // effi 96
vector2 point2 = { 2265.77, 128.592 };
vector2 point3 = { 1497.17, 124.234 };
vector2 point4 = { 1468.89, 115.476 };
vector2 point5 = { 1824.47, 85.689 };
vector2 point6 = { 2672.51, 78.311 };
vector2 point7 = { 3478.86, 97.27 };
vector2 points1[] = { point1, point2, point3, point4, point5, point6, point7};
polygon poly1 = { points1, 7 };


vector2 point8 = { 3971.93, 152.207 };        // effi 95
vector2 point9 = { 2641.38, 167.508 };
vector2 point10 = { 1183.75, 153.269 };
vector2 point11 = { 1178.73, 135.362 };
vector2 point12 = { 1468.47, 92.816 };
vector2 point13 = { 2175.53, 64.676 };
vector2 point14 = { 3605.18, 86.24 };
vector2 point15 = { 4275.86, 117.33 };
vector2 points2[] = {point8, point9, point10, point11, point12, point13, point14 , point15 };
polygon poly2 = { points2, 8 };



vector2 point16 = { 4989.61, 195.546 };     // effi 94
vector2 point17 = { 1929.69, 198.939 };
vector2 point18 = { 1080.89, 165.383 };
vector2 point19 = { 952.72, 77.363 };
vector2 point20 = { 1880.16, 56.786 };
vector2 point21 = { 3735.95, 63.147 };
vector2 point22 = { 4491.1, 100.041 };
vector2 point23 = { 4979.32, 146.94 };
vector2 points3[] = { point16, point17, point18, point19, point20, point21, point22, point23 };
polygon poly3 = { points3, 8 };



vector2 point24 = { 4994.9, 228.073 };      // effi 90-94
vector2 point25 = { 1419.5, 230.268 };
vector2 point26 = { 738.57, 152.765 };
vector2 point27 = { 620.32, 93.981 };
vector2 point28 = { 947.6, 54.34 };
vector2 point29 = { 2212.58, 41.265 };
vector2 point30 = { 3637.19, 43.826 };
vector2 point31 = { 4467.22, 76.296 };
vector2 point32 = { 5011.67, 122.436 };
vector2 points4[] = {point24, point25, point26, point27, point28, point29, point30, point31, point32 };
polygon poly4 = { points4, 9 };



vector2 point33 = { 4990.46, 241.233 };     // effi 86-90
vector2 point34 = { 1147.92, 241.737 };
vector2 point35 = { 415.37, 160.241 };
vector2 point36 = { 244.98, 69.685 };
vector2 point37 = { 487.97, 33.011 };
vector2 point38 = { 1635.8, 19.996 };
vector2 point39 = { 3711.85, 26.975 };
vector2 point40 = { 4499.72, 60.198 };
vector2 point41 = { 5000.00, 105.262 };
vector2 points5[] = {point33, point34, point35, point36, point37, point38, point39, point40, point41 };
polygon poly5 = { points5, 9 };




void setup() {
  Serial.begin(115200);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS);
  mcp2515.setNormalMode();
  Serial.println("Mcp2515 setup success");
  display.setBrightness(0x0f);
  display.showNumberDec(0000);
  delay(300);

  radio.begin();                                           //아두이노-RF모듈간 통신라인
  radio.setPALevel(RF24_PA_MAX);
  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address);
  radio.stopListening();                                   //기본 : 송신모드
  Serial.println("NRF24 setup success");
  display.showNumberDec(1111);
  delay(300);

  Serial.println("Orientation Sensor Test"); Serial.println(""); // 가속도 센서 부팅
  if (bno.begin()){
    Serial.print("BNO055 connection success!!");
    display.showNumberDec(2222);
    delay(300);
  }
  else{
    Serial.print("No BNO055 detected ... Check your wiring or I2C ADDR!");
    display.showNumberDec(8888);
    delay(300);
  }

  pinMode(flowsensor, INPUT);
  digitalWrite(flowsensor, HIGH);
  attachInterrupt(digitalPinToInterrupt(2), flow, RISING); // Setup Interrupt
  sft = millis();
  
  Serial.println("Flow sensor ready");

  pinMode(REN, OUTPUT);                             //BTS7960 모터 우회전 활성화
  pinMode(LEN, OUTPUT);                             //BTS7960 모터 좌회전 활성화
  pinMode(PB4, OUTPUT);                             //BTS7960 모터 우회전 속도조절 활성화

  analogWriteFrequency(20000);
  digitalWrite(REN, LOW);                           // 팬 정지 설정
  digitalWrite(LEN, LOW);
  Serial.println("BTS7960 active & Set fan speed = 0 ");
  Rad = 0;

  Clear();
  delay(10);
  Request();
  display.showNumberDec(3333);
  delay(300);
}


void loop() {
  rrf[3] = { 0, };                        //Master 명령 수신 전 배열 초기화
  radio.startListening();
  if (radio.available()){
    radio.read(rrf,sizeof(rrf));
    if (rrf[0] == 8){
      if(rrf[1] == 1){
        bRad = true;
        bhcRad = true;
        //Serial.println("Hp / Cp 자동제어");
      }
      else if(rrf[1] == 2){
        bRad = true;
        bhcRad = false;
        //Serial.println("Mt, Pt 자동제어");
      }
    }
    else if (rrf[0] == 9){
      bRad = false;
      //Serial.println("라디 수동제어 255");  
    }
    else if (rrf[0] == 7){
      radio.stopListening();
      for (int i = 0; i<4; i++){
        radio.write(rrf, sizeof(rrf));
        //Serial.println("보드 초기화 응답");
      }
      display.showNumberDec(7777); 
      delay(1000);
      HAL_NVIC_SystemReset();     
    }
    else if (rrf[0] == 6){
      Request();
      radio.stopListening();
      for (int i = 0; i<4; i++){
        radio.write(rrf, sizeof(rrf));
        //Serial.println("CAN 재요청 활성화 응답");
      }
    }
    else if (rrf[0] == 5){
      Clear();
      radio.stopListening();
      for (int i = 0; i<4; i++){
        radio.write(rrf, sizeof(rrf));
        //Serial.println("CAN CLEAR  응답");
      }
    }
    else if (rrf[0] == 4){
      Cp = rrf[1] / 10.0f; 
      Hp = rrf[2] / 10.0f; 
          
    }
    else if (rrf[0] == 3){
      SAFE = millis();          // Master로 수신 받을 시 SAFE 업데이트
    }
    else if (rrf[0] == 2){
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      accsen1(&orientationData);
    }
  }


  //캔통신
  for (;;) {
    id = mcp2515_receive();
    if (id == 1) { 
      display.showNumberDec(4444);
      if (millis() > sft && bsft == true){             // 아두이노 부팅이후 캔 통신 데이터 5초 이후에도 없는 경우 자동 재요청 코드
        sft = millis() + 5000;
        Request();
      }
      break;
    }
    else if (id == 0x30) {                                          // SpeedActual
      SpeedActual  = ((int16_t)recv[2] << 8) | recv[1];
      bSpeedActual = true;
      RPM = SpeedActual * 6500 / 32767  ;                           //SpeedActual * SpeedRpmMax / 32767.0f;
      display.showNumberDec(RPM);        
    }
    else if (id == 0x49) {                                          // Motor_Temp
      Motor_Temp = ((int16_t)recv[2] << 8) | recv[1];
      Motor_Temp = mt(Motor_Temp);
      bMotor_Temp = true;
    }
    else if (id == 0x4A) {                                          // PowerStage_Temp
      PowerStage_Temp = ((int16_t)recv[2] << 8) | recv[1];
      PowerStage_Temp = pst(PowerStage_Temp);
      bPowerStage_Temp = true;
    }
    else if (id == 0x27) {                                          // Lq_actual
      Lq_actual = ((int16_t)recv[2] << 8) | recv[1];
      Torque_Out = ((float)Lq_actual) * 0.373832f  * 0.61f ;
      bLq_actual = true;
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
      ACvoltage = ((int16_t)recv[2] << 8) | recv[1];       
      bACvoltage = true;
    }
    else if (id == 0x40) {                                          // Status    
      Status1 = ((uint16_t)recv[2] << 8) | recv[1];
      Status2 = ((uint16_t)recv[4]<< 8) | recv[3];
      bStatus = true;
    }
    else if (id == 0x8F) {                                          // ERROR    
      ERROR1 = ((uint16_t)recv[2] << 8) | recv[1];
      ERROR2 = ((uint16_t)recv[4]<< 8) | recv[3];
      bERROR = true;
    }
    bsft = false; 
    if (bSpeedActual == true && bMotor_Temp == true && bPowerStage_Temp == true && bLq_actual == true && bCurrentActual == true && bDCvoltage == true && bACvoltage == true && bStatus == true  && bERROR == true ){
      bSpeedActual = false;
      bMotor_Temp = false;
      bPowerStage_Temp= false;
      bLq_actual = false;
      bCurrentActual = false;
      bDCvoltage = false;
      bACvoltage = false;
      bStatus = false;
      bERROR = false;
      break;
    }
  }

  fACcurrent = ((float)CurrentActual) * 0.373832f ;                          //Arms CurrentDevice / Current200pc 
  fDCvoltage = ((float)DCvoltage) / 31.5848f - 27;  
  fACvoltage = fDCvoltage / 1.4142f * ((float)ACvoltage) / 4096.0f * 0.92f;           //Vrms
  fACpower2 = fACcurrent * fACvoltage * 1.732f / 1000.0f;                    //kW
  Eff(RPM, Torque_Out);
  interval = millis() - Endtime;
  Endtime = millis();    
  //Serial.print("interval : ");Serial.println(interval);
  Cp = Cp + (abs(CoolantTemp1 - CoolantTemp2) * 6.0f / 1000.0f * 998.0f / 60.0f * 4.186f ) * interval / 1000.0f;              // kW 단위 
  Hp = Hp + (abs(fACpower2 / 0.92f * 0.08f + fACpower2 * (1 - 0.01 * M_efficiency))) * interval / 1000.0f;                   // kW 단위
  //Serial.print("M_efficiency : ");Serial.println(M_efficiency);
  Radiator_Fan_Control();
  Flow_sensor();
  tempsensor();
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  accsen(&linearAccelData);
  accsen(&orientationData);
  Auto_Reset();
  

  //Datapack RF Trans
  int16_t datapack1[16];
  datapack1[0] = 11,
  datapack1[1] = RPM, datapack1[2] = 10 * Motor_Temp, datapack1[3] = 10 * PowerStage_Temp, datapack1[4] = 10 *LPM, datapack1[5] = Rad,
  datapack1[6] = 10 * Torque_Out, datapack1[7] = CurrentActual, datapack1[8] = DCvoltage, datapack1[9] = ACvoltage,
  datapack1[10] = 10 * LN_accele, datapack1[11] = 10 * LT_accele, datapack1[12] = Roll, datapack1[13] = Pitch,
  datapack1[14] =  CoolantTemp1, datapack1[15] = CoolantTemp2;
  radio.stopListening(); //송신모드
  radio.write(datapack1, sizeof(datapack1));
  delay(5);

  uint16_t datapack2[16];
  datapack2[0] = 22,
  datapack2[1] = Status1, datapack2[2] = Status2, datapack2[3] = ERROR1, datapack2[4] = ERROR2,
  datapack2[5] = 10 * Cp, datapack2[6] = 10 * Hp;
  radio.stopListening(); //송신모드
  radio.write(datapack2, sizeof(datapack2));
  
  //Serial.println("------------------------------------------------------------------------------------------- ");
}


////////////////////////////////////////////CP HP 라디에이터 팬 속도 판단/////////////////////////////////////////
void Radiator_Fan_Control(){               
  if (bRad == true && bhcRad == true ) { //Hp / Cp 자동제어 
    if(id == 1){
        digitalWrite(REN, 1);
        digitalWrite(LEN, 1);
        analogWrite(PB4, 255);
        //Serial.println("can 통신 실패 팬속도 최대"); 
        Rad = 44; 
    }
    else if ( Hp/Cp >= 1 ) {  //과열 상황
      digitalWrite(REN, 1);
      digitalWrite(LEN, 1);
      analogWrite(PB4, 255);
      //Serial.println(" 팬속도 최대");
      Rad = 10; 
    }
    else if (Hp/Cp >= 0.8) {  
      digitalWrite(REN, 1);
      digitalWrite(LEN, 1);
      analogWrite(PB4, 200);
      //Serial.println("팬속도 200");
      Rad = 8; 
    }
    else if (Hp/Cp >= 0.6) { 
      digitalWrite(REN, 1);
      digitalWrite(LEN, 1);
      analogWrite(PB4, 150);
      //Serial.println("팬속도 150");
      Rad = 7; 
    }
    else if (Hp/Cp >= 0.5) { 
      digitalWrite(REN, 1);
      digitalWrite(LEN, 1);
      analogWrite(PB4, 100);                
      //Serial.println("팬속도 100");        
      Rad = 6; 
    }
    else if (Hp/Cp < 0.5) {  
      digitalWrite(REN, 0);
      digitalWrite(LEN, 0);
      analogWrite(PB4, 0);
      //Serial.println("팬 정지");                       
      Rad = 5; 
    }
  }
  else if (bRad == true && bhcRad == false) { //Mt, Pt 자동제어 
    if(id == 1){
        digitalWrite(REN, 1);
        digitalWrite(LEN, 1);
        analogWrite(PB4, 255);
        //Serial.println("can 통신 실패 팬속도 최대"); 
        Rad = 44; 
    }
    else if (Motor_Temp >= 50 || PowerStage_Temp >= 50) {  //50도
      digitalWrite(REN, 1);
      digitalWrite(LEN, 1);
      analogWrite(PB4, 255);
      //Serial.println("온도 50 이상 팬속도 최대");
      Rad = 255; 
    }
    else if (Motor_Temp >= 45 || PowerStage_Temp >= 45) {  //45도
      digitalWrite(REN, 1);
      digitalWrite(LEN, 1);
      analogWrite(PB4, 200);
      //Serial.println("온도 45 이상 팬속도 200");
      Rad = 200; 
    }
    else if (Motor_Temp >= 40 || PowerStage_Temp >= 40) {  //40도
      digitalWrite(REN, 1);
      digitalWrite(LEN, 1);
      analogWrite(PB4, 150);
      //Serial.println("온도 40 이상 팬속도 150");
      Rad = 150; 
    }
    else if (Motor_Temp >= 30 || PowerStage_Temp >= 30) {  //30도
      digitalWrite(REN, 1);
      digitalWrite(LEN, 1);
      analogWrite(PB4, 100);                
      //Serial.println("온도 30 이상 팬속도 100");        
      Rad = 100; 
    }
    else if (Motor_Temp < 30 || PowerStage_Temp < 30) {  //30도
      digitalWrite(REN, 0);
      digitalWrite(LEN, 0);
      analogWrite(PB4, 0);
      //Serial.println("온도 30도 이하 팬 정지");                       
      Rad = 50; 
    }
    else{
      digitalWrite(REN, 1);
      digitalWrite(LEN, 1);
      analogWrite(PB4, 255);
      //Serial.println("혹시 몰");
      Rad = 250; 
    }
  }
  else if (bRad == false){
    if (rrf[1] == 9 ){
      digitalWrite(REN, 1);
      digitalWrite(LEN, 1);
      analogWrite(PB4, 255);
      Rad = 9; 
    }
    else if(rrf[1] == 1 ){
      digitalWrite(REN, 0);
      digitalWrite(LEN, 0);
      analogWrite(PB4, 0);
      Rad = 1; 
    }
    else if(rrf[1] == 2 ){
      digitalWrite(REN, 1);
      digitalWrite(LEN, 1);
      analogWrite(PB4, 200);
      Rad = 2; 
    }
    else if(rrf[1] == 3 ){
      digitalWrite(REN, 1);
      digitalWrite(LEN, 1);
      analogWrite(PB4, 150);
      Rad = 3; 
    }
    else if(rrf[1] == 4 ){
      digitalWrite(REN, 1);
      digitalWrite(LEN, 1);
      analogWrite(PB4, 100);
      Rad = 4; 
    }
  }  
  return;
}

///////////////////////////////////////////////Clear/////////////////////////////////////////
void Clear() {
  byte data1[3] = { 0x3D,0x30,0xFF };
  mcp2515_send(rx, data1, 3); // Clear SpeedActual 150ms
  delay(10);
  data1[1] = 0x49;
  mcp2515_send(rx, data1, 3); //Clear Motor_Temp 150ms
  delay(10);
  data1[1] = 0x4A;
  mcp2515_send(rx, data1, 3); //Clear PowerStage_Temp 150ms
  delay(10);
  data1[1] = 0x27;
  mcp2515_send(rx, data1, 3); //Clear Lq_actual 150ms
  delay(10);
  data1[1] = 0x26;
  mcp2515_send(rx, data1, 3); //Clear Lq_cmd 100ms
  delay(10);
  data1[1] = 0xD5;
  mcp2515_send(rx, data1, 3); //Clear Pedal 100ms
  delay(10);
  data1[1] = 0x5F;
  mcp2515_send(rx, data1, 3); //Clear CurrentActual 150ms
  // delay(10);
  // data1[1] = 0x4B;
  // mcp2515_send(rx, data1, 3); //Clear Air_Temp 150ms
  // delay(10);
  // data1[1] = 0xD9;
  // mcp2515_send(rx, data1, 3); //Clear Current200pc 150ms
  // delay(10);
  // data1[1] = 0xC6;
  // mcp2515_send(rx, data1, 3); //Clear CurrentDevice 150ms
  // delay(10);
  // data1[1] = 0xC4;
  // mcp2515_send(rx, data1, 3); //Clear I_Max_PK 150ms
  delay(10);
  data1[1] = 0xEB;
  mcp2515_send(rx, data1, 3); //Clear DCvoltage 150ms
  delay(10);
  data1[1] = 0x8A;
  mcp2515_send(rx, data1, 3); //Clear ACvoltage 150ms
  delay(10);
  data1[1] = 0x40;
  mcp2515_send(rx, data1, 3); //Clear Status 150ms
  delay(10);
  data1[1] = 0x8F;
  mcp2515_send(rx, data1, 3); //Clear ERROR 150ms
  //delay(1);
  //data1[1] = 0xD8;
  //mcp2515_send(rx, data1, 3); //Clear LOGIC 150ms
  //Serial.println("Clear can line"); 
  return; 
}

//////////////////////////////////////////////Request/////////////////////////////////////////
void Request() {
  byte data[3] = { 0x3D,0x30,0x64 };
  mcp2515_send(rx, data, 3); //Request SpeedActual 100ms
  delay(10);
  data[1] = 0x49;
  mcp2515_send(rx, data, 3); //Request Motor_Temp 100ms
  delay(10);
  data[1] = 0x4A;
  mcp2515_send(rx, data, 3); //Request PowerStage_Temp 100ms
  delay(10);
  data[1] = 0x40;
  mcp2515_send(rx, data, 3); //Request Status 100ms
  delay(10);
  data[1] = 0x8F;
  mcp2515_send(rx, data, 3); //Request ERROR 100ms
  delay(10);
  data[1] = 0x27;
  mcp2515_send(rx, data, 3); //Request Lq_actual 100ms
  delay(10);
  data[1] = 0x5F;
  mcp2515_send(rx, data, 3); //Request CurrentActual 100ms
  delay(10);
  data[1] = 0xEB;
  mcp2515_send(rx, data, 3); //Request DCvoltage 100ms
  delay(10);
  data[1] = 0x8A;
  mcp2515_send(rx, data, 3); //Request ACvoltage 100ms
  //delay(5);
  //data[1] = 0xD8;
  //mcp2515_send(rx, data, 3); //Request LOGIC 150ms
  //Serial.println("Request data transfer success ");
  return; 
}

///////////////////////////////////////////////send/////////////////////////////////////////
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
  // Serial.print(canMsg.data[i], HEX);
  // Serial.print(" ");
  // }
  // Serial.println();
  return;
}

//////////////////////////////////////////////receive/////////////////////////////////////////
byte mcp2515_receive() {
  struct can_frame canMsg;
  unsigned long t = millis();
  byte ret;
  while (1) {
    if (millis() - t > 150) {
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

//////////////////////////////////////////////모터 효율 판단 함수/////////////////////////////////////////
void Eff(int a, int b){
  vector2 testPoint = { a, b };
  int isInsidePolygon1 = isInside(testPoint, &poly1);       // effi 96 d
  int isInsidePolygon2 = isInside(testPoint, &poly2);       // effi 95-96 d
  int isInsidePolygon3 = isInside(testPoint, &poly3);       // effi 94-95 d
  int isInsidePolygon4 = isInside(testPoint, &poly4);       // effi 90-94 d
  int isInsidePolygon5 = isInside(testPoint, &poly5);       // effi 86-90 d


  if (isInsidePolygon1){
    M_efficiency = 96;
    //Serial.println("96%");
  }
  else if(isInsidePolygon2){
    M_efficiency = 95;
    //Serial.println("95%");
  }
  else if(isInsidePolygon3){
    M_efficiency = 94;
    //Serial.println("94%");
  }
  else if(isInsidePolygon4){
    M_efficiency = 90;
    //Serial.println("94%");
  }
  else if(isInsidePolygon5){
    M_efficiency = 86;
    //Serial.println("86%"");
  }
  else{
    M_efficiency = 80;
    //Serial.println("80%"");
  }
  return ;
}

////////////////////////////////////////////Poly 판단/////////////////////////////////////////
int isInside(vector2 B, const polygon* p) {
    int crosses = 0;
    for (int i = 0; i < p->size; i++) {
        int j = (i + 1) % p->size;
        if ((p->points[i].y > B.y) != (p->points[j].y > B.y)) {
            double atX = (p->points[j].x - p->points[i].x) * (B.y - p->points[i].y) / (p->points[j].y - p->points[i].y) + p->points[i].x;
            if (B.x < atX)
                crosses++;
        }
    }
    //Serial.println(crosses);
    return crosses % 2 > 0;
}

////////////////////////////////////////////Auto_Reset/////////////////////////////////////////
void Auto_Reset(){
  if (millis() >= (SAFE + 30000)){              // Master 보드로 30초 동안 수신 받은게 없으면 자동 초기화 코드 실행
    display.showNumberDec(7474); 
    delay(200);
    HAL_NVIC_SystemReset();
  }
  return;
}

////////////////////////////////////////////모터 온도 계산함수/////////////////////////////////////////
float mt(uint16_t x){
  float mtemp;
  float p1 =  -1.387e-16 ;
  float p2 =   3.614e-11 ; 
  float p3 =  -1.009e-06  ;
  float p4 =     0.02741 ;
  float p5 =      -196.9 ;
  mtemp = p1*pow(x,4) + p2*pow(x,3) + p3*pow(x,2) + p4*x + p5;
  
  return mtemp;
}

////////////////////////////////////////////컨트롤러 온도 계산함수/////////////////////////////////////////
float pst(int16_t x){
  float pstemp;
  float  p1 =  -2.8e-15;
  float  p2 =  3.375e-10;
  float  p3 =  -1.426e-05;
  float  p4 =  0.2651;
  float  p5 =  -1810;
  pstemp = p1*pow(x,4) + p2*pow(x,3) + p3*pow(x,2) + p4*x + p5;
  return pstemp;
}

////////////////////////////////////////////가속도 센서/////////////////////////////////////////
void accsen(sensors_event_t* event){
  
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    Roll = (-1 * (event->orientation.z) - roll);
    //Serial.print(F("Roll  : "));Serial.print(-1 *Roll);
    Pitch = (-1 * (event->orientation.y) - pitch);
    //Serial.print("\tPitch : ");Serial.println(-1 *Pitch);
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    LT_accele = -1 * (event->acceleration.x);
    //Serial.print(F("LT_accele : "));Serial.print(-1 * LT_accele1);
    LN_accele = -1 * (event->acceleration.z);
    //Serial.print("\tLN_accele : ");Serial.println(-1 * LN_accele1);
    //Serial.println(F("=====================================================\n"));
  }
  else {
  //Serial.print("Unk:");
  }
  return;
}

////////////////////////////////////////////가속도 센서 offset/////////////////////////////////////////
void accsen1(sensors_event_t* event){
  roll = -1 * (event->orientation.z);
  pitch = -1 * (event->orientation.y);
  
  return;
}

//////////////////////////////////////////CoolantTemp function/////////////////////////////////////////
void tempsensor(){  
  float average1 = 0;
  float average2 = 0;
  for (uint8_t i=0; i<NUMSAMPLES; i++) {                      
    average1 += analogRead(coolantsensorPin1);
    average2 += analogRead(coolantsensorPin2);       
  }
   
  average1 /= NUMSAMPLES;  
  average1 = (coolantsensorDivider1*average1)/(1023-average1); 
  average2 /= NUMSAMPLES;  
  average2 = (coolantsensorDivider2*average2)/(1023-average2);  

  CoolantTemp1 = log(average1);                      //lnR
  CoolantTemp1 = pow(CoolantTemp1,3);                  //(lnR)^3
  CoolantTemp1 *= steinconstC;                       //C*((lnR)^3)
  CoolantTemp1 += (steinconstB*(log(average1)));     //B*(lnR) + C*((lnR)^3)
  CoolantTemp1 += steinconstA;                       //Complete equation, 1/T=A+BlnR+C(lnR)^3
  CoolantTemp1 = 1.0/CoolantTemp1;                     //Inverse to isolate for T
  CoolantTemp1 -= 273.15; 

  CoolantTemp2 = log(average2);                      //lnR
  CoolantTemp2 = pow(CoolantTemp2,3);                  //(lnR)^3
  CoolantTemp2 *= steinconstC;                       //C*((lnR)^3)
  CoolantTemp2 += (steinconstB*(log(average2)));     //B*(lnR) + C*((lnR)^3)
  CoolantTemp2 += steinconstA;                       //Complete equation, 1/T=A+BlnR+C(lnR)^3
  CoolantTemp2 = 1.0/CoolantTemp2;                     //Inverse to isolate for T
  CoolantTemp2 -= 273.15; 
  //Serial.print("Coolant Temperature = ");
  //Serial.print(CoolantTemp1);Serial.print(" *C");Serial.print(" / ");Serial.print(CoolantTemp2);Serial.println(" *C");              

  return; 
}

/////////////////////////////////////////////유량센서/////////////////////////////////////////
void Flow_sensor(){    

  LPM = ( flow_frequency / 11.0f / interval * 1000);        // Pulse frequency (Hz) = 11Q, Q is flow rate in L/min.
  flow_frequency = 0;                              
  //Serial.print(LPM);                                      // Print litres/min
  //Serial.println(" L/Min");

  return;
}

//////////////////////////////////////////Interrupt function/////////////////////////////////////////
void flow(){
  flow_frequency ++;
  return;
}
