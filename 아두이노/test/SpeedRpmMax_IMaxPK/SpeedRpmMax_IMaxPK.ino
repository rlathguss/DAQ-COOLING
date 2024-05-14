#include <Arduino.h>
#include <TM1637Display.h>
#include <SPI.h>
#include <mcp2515.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

const byte CLK = 3;                                   // define CLK pin (any digital pin)s
const byte DIO = 4;                                   // define DIO pin (any digital pin)
TM1637Display display(CLK, DIO);                      // define dispaly object

#define rx 0x201
#define tx 0x181
MCP2515 mcp2515(10);   //cs핀 번호 설정
byte recv[8];
byte id; 
void mcp2515_send(unsigned int sid, byte data[], byte dlc);
byte mcp2515_receive();
double pst(int16_t x);
double at(int16_t x);

const int REN = 7;           //정회전 활성화 핀
const int LEN = 6;           //역회전 활성화 핀
HardwareTimer pwmtimer3(TIM3);

int16_t SpeedRpmMax=0;                                                            //0xCE,
int16_t CurrentDevice, Current200pc, I_Max_PK;                                            //0xC6, 0xD9, 0xC4
int16_t SpeedActual, Motor_Temp, PowerStage_Temp, Air_Temp,                               //0x30, 0x49, 0x4A, 0x4B
        M_Out, CurrentActual, Rad, DCvoltage, ACvoltage,                                  //0xA0, 0x5F, 0xEB, 0x8A
        Xaccle, Yaccle, Zaccle, Xangle, Yangle, 
        CoolantTemp1, CoolantTemp2,
        Status1, Status2, LOGIC;
int Status;
float LPM1, Xaccle1, Yaccle1, Zaccle1, Xangle1, Yangle1, RPM;


bool bSpeedActual = false;
bool bMotor_Temp = false;
bool bPowerStage_Temp = false;
bool bAir_Temp = false;
bool bM_Out = false;
bool bCurrentActual = false;
bool bSpeedRpmMax = false;
bool bCurrent200pc = false;
bool bCurrentDevice = false;
bool bI_Max_PK = false;  
bool bDCvoltage = false;
bool bACvoltage = false;
bool bStatus = false;
bool bLOGIC = false;

unsigned long currentTime1;
unsigned long currentTime2;
unsigned long sft;
bool bsft = false;

void setup() {
  display.setBrightness(0x0f);
  display.showNumberDec(0000);
  analogWriteFrequency(20000);
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);
  pinMode(PB4, OUTPUT);
  //pinMode(LPWM, OUTPUT);
  digitalWrite(REN, LOW);
  digitalWrite(LEN, LOW);

  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);
  analogWrite(PB4, 255);

  Serial.begin(115200);
  currentTime1 = millis();
  sft = currentTime1;
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS);
  mcp2515.setNormalMode();
  Serial.println("Mcp2515 setup success");


  //Serial.print("Current200pc : ");Serial.println(Current200pc);
  //Serial.print("CurrentDevice : ");Serial.println(CurrentDevice);
  //Serial.print("PowerStage_Temp : ");Serial.print(PowerStage_Temp);Serial.print(" / ");Serial.println(pst(PowerStage_Temp));
  //Serial.print("Air_Temp : ");Serial.print(Air_Temp);Serial.print(" / ");Serial.println(at(Air_Temp));
  //Serial.print("I_Max_PK : ");Serial.println(I_Max_PK);


  byte data1[3] = { 0x3D,0x30,0xFF };
  mcp2515_send(rx, data1, 3); //Request SpeedActual 150ms
  delay(10);
  data1[1] = 0x49;
  mcp2515_send(rx, data1, 3); //Request Motor_Temp 150ms
  delay(10);
  data1[1] = 0x4A;
  mcp2515_send(rx, data1, 3); //Request PowerStage_Temp 150ms
  delay(10);
  data1[1] = 0x4B;
  mcp2515_send(rx, data1, 3); //Request Air_Temp 150ms
  delay(10);
  data1[1] = 0xA0;
  mcp2515_send(rx, data1, 3); //Request M_out 150ms
  delay(10);
  data1[1] = 0x5F;
  mcp2515_send(rx, data1, 3); //Request CurrentActual 150ms
  delay(10);
  data1[1] = 0xCE;
  mcp2515_send(rx, data1, 3); //Request SpeedRpmMax 150ms
  delay(10);
  data1[1] = 0xD9;
  mcp2515_send(rx, data1, 3); //Request Current200pc 150ms
  delay(10);
  data1[1] = 0xC6;
  mcp2515_send(rx, data1, 3); //Request CurrentDevice 150ms
  delay(10);
  data1[1] = 0xC4;
  mcp2515_send(rx, data1, 3); //Request I_Max_PK 150ms
  delay(10);
  data1[1] = 0xEB;
  mcp2515_send(rx, data1, 3); //Request DCvoltage 150ms
  delay(10);
  data1[1] = 0x8A;
  mcp2515_send(rx, data1, 3); //Request ACvoltage 150ms
  delay(10);
  data1[1] = 0x40;
  mcp2515_send(rx, data1, 3); //Request Status 150ms
  delay(10);
  data1[1] = 0x8F;
  mcp2515_send(rx, data1, 3); //Request ERROR 150ms
  delay(10);
  data1[1] = 0xD8;
  mcp2515_send(rx, data1, 3); //Request LOGIC 150ms
  Serial.println("Clear can line");

  byte data[3] = { 0x3D,0x30,0x96 };
  //mcp2515_send(rx, data, 3); //Request SpeedActual 150ms
  //delay(10);
  //data[1] = 0x49;
  //mcp2515_send(rx, data, 3); //Request Motor_Temp 150ms
  //delay(10);
  //data[1] = 0x4A;
  //mcp2515_send(rx, data, 3); //Request PowerStage_Temp 150ms
  //delay(10);
  //data[1] = 0x4B;
  //mcp2515_send(rx, data, 3); //Request Air_Temp 150ms
  //delay(10);
  data[1] = 0xA0;
  mcp2515_send(rx, data, 3); //Request M_out 150ms
  delay(10);
  //data[1] = 0x5F;
  //mcp2515_send(rx, data, 3); //Request CurrentActual 150ms
  //delay(10);
  //data[1] = 0xCE;
  //mcp2515_send(rx, data, 3); //Request SpeedRpmMax 150ms
  //delay(10);
  //data[1] = 0xD9;
  //mcp2515_send(rx, data, 3); //Request Current200pc 150ms
  //delay(10);
  //data[1] = 0xC6;
  //mcp2515_send(rx, data, 3); //Request CurrentDevice 150ms
  //delay(10);
  //data[1] = 0xC4;
  //mcp2515_send(rx, data, 3); //Request I_Max_PK 150ms
  //delay(10);
  //data[1] = 0xEB;
  //mcp2515_send(rx, data, 3); //Request DCvoltage 150ms
  //delay(10);
  //data[1] = 0x8A;
  //mcp2515_send(rx, data, 3); //Request ACvoltage 150ms
  //delay(10);
  //data[1] = 0x40;
  //mcp2515_send(rx, data, 3); //Request Status 150ms
  delay(10);
  data[1] = 0xD8;
  mcp2515_send(rx, data, 3); //Request LOGIC 150ms
  Serial.println("Request data transfer success ");
}

void loop() {
  currentTime1 = millis();  
  for (;;) {
    id = mcp2515_receive();
    if (id == 1) {
      display.showNumberDec(4444); 
      if (bsft == false && currentTime1>=(sft +10000)){
        byte data[3] = { 0x3D,0x30,0x96 };
        //mcp2515_send(rx, data, 3); //Request SpeedActual 150ms
        //delay(10);
        //data[1] = 0x49;
        //mcp2515_send(rx, data, 3); //Request Motor_Temp 150ms
        //delay(10);
        //data[1] = 0x4A;
        //mcp2515_send(rx, data, 3); //Request PowerStage_Temp 150ms
        //delay(10);
        //data[1] = 0x4B;
        //mcp2515_send(rx, data, 3); //Request Air_Temp 150ms
        //delay(10);
        data[1] = 0xA0;
        mcp2515_send(rx, data, 3); //Request M_out 150ms
        delay(10);
        //data[1] = 0x5F;
        //mcp2515_send(rx, data, 3); //Request CurrentActual 150ms
        //delay(10);
        //data[1] = 0xCE;
        //mcp2515_send(rx, data, 3); //Request SpeedRpmMax 150ms
        //delay(10);
        //data[1] = 0xD9;
        //mcp2515_send(rx, data, 3); //Request Current200pc 150ms
        //delay(10);
        //data[1] = 0xC6;
        //mcp2515_send(rx, data, 3); //Request CurrentDevice 150ms
        //delay(10);
        //data[1] = 0xC4;
        //mcp2515_send(rx, data, 3); //Request I_Max_PK 150ms
        //delay(10);
        //data[1] = 0xEB;
        //mcp2515_send(rx, data, 3); //Request DCvoltage 150ms
        //delay(10);
        //data[1] = 0x8A;
        //mcp2515_send(rx, data, 3); //Request ACvoltage 150ms
        //delay(10);
        //data[1] = 0x40;
        //mcp2515_send(rx, data, 3); //Request Status 150ms
        //delay(10);
        //data[1] = 0x8F;
        //mcp2515_send(rx, data, 3); //Request ERROR 150ms
        //delay(10);
        data[1] = 0xD8;
        mcp2515_send(rx, data, 3); //Request LOGIC 150ms
      }
      Serial.println("컨트롤러 오프라인! / 팬속도 255 ");
      break;
    }
    else if (id == 0x30) {                                          // SpeedActual
      SpeedActual  = ((int16_t)recv[2] << 8) | recv[1];
      bSpeedActual = true;        
    }
    else if (id == 0x49) {                                          // Motor_Temp
      Motor_Temp = ((int16_t)recv[2] << 8) | recv[1];
      bMotor_Temp = true;
    }
    else if (id == 0x4A) {                                          // PowerStage_Temp
      PowerStage_Temp = ((int16_t)recv[2] << 8) | recv[1];
      bPowerStage_Temp = true;
    }
    else if (id == 0x4B) {                                          //Air_Temp
      Air_Temp = ((int16_t)recv[2] << 8) | recv[1];
      bAir_Temp = true;
    }
    else if (id == 0xA0) {                                          //M_Out
      M_Out = ((int16_t)recv[2] << 8) | recv[1];
      bM_Out = true;
    }
    else if (id == 0x5F) {                                          // CurrentActual
      CurrentActual = ((int16_t)recv[2] << 8) | recv[1];       
      bCurrentActual = true;
    }
    else if (id == 0xCE) {                                          // SpeedRpmMax;
      SpeedRpmMax = ((int16_t)recv[2] << 8) | recv[1];     
      bSpeedRpmMax = true;
    }
    else if (id == 0xD9) {                                          // Current200pc
      Current200pc = ((int16_t)recv[2] << 8) | recv[1];       
      bCurrent200pc = true;
    }
    else if (id == 0xC6) {                                          // CurrentDevice
      CurrentDevice = ((int16_t)recv[2] << 8) | recv[1];       
      bCurrentDevice = true;
    }
    else if (id == 0xC4) {                                          // I_Max_PK 424.2 max
      I_Max_PK = ((int16_t)recv[2] << 8) | recv[1]; 
      I_Max_PK = I_Max_PK / 16383.5 * 424.2;
      bI_Max_PK = true;
    }
    else if (id == 0xEB) {                                          // DCvoltage
      DCvoltage = ((int16_t)recv[2] << 8) | recv[1];       
      bDCvoltage = true;
    } 
    else if (id == 0x8A) {                                          // ACvoltage
      ACvoltage = ((int16_t)recv[2] << 8) | recv[1];       
      bACvoltage = true;
    } 
    else if (id == 0x40) {                                          // Status    
      Status1 = ((uint16_t)recv[2] << 8) | recv[1];
      Status2 = ((uint16_t)recv[4]<< 8) | recv[3];
      Status = ((int)Status2 << 16) | Status1;
      //Serial.println(Status, HEX);
      bStatus = true;
    }
    else if (id == 0x8F) {                                          // ERROR    
      ERROR1 = ((uint16_t)recv[2] << 8) | recv[1];
      ERROR2 = ((uint16_t)recv[4]<< 8) | recv[3];
      bERROR1 = true;
      bERROR2 = true;
    }
    else if (id == 0xD8) {                                          // LOGIC  
      LOGIC = ((uint16_t)recv[2] << 8) | recv[1];
      bLOGIC = true;
    }
    bsft = true;

    if ( bM_Out == true && bLOGIC == true){
      //bSpeedActual == true && bMotor_Temp == true && bPowerStage_Temp == true && bAir_Temp == true && bM_Out == true && bCurrentActual == true &&  bSpeedRpmMax == true && bCurrent200pc == true && bCurrentDevice == true && bI_Max_PK == true && bDCvoltage == true && bACvoltage == true && bStatus == true && bLOGIC == true
      bSpeedActual = false;
      bMotor_Temp = false;
      bPowerStage_Temp= false;
      bAir_Temp = false;
      bM_Out = false;
      bCurrentActual = false;
      bSpeedRpmMax = false;
      bCurrent200pc = false;
      bCurrentDevice = false;
      bI_Max_PK = false;  
      bDCvoltage = false;
      bACvoltage = false;
      bStatus = false;
      bERROR1 = false;
      bERROR2 = false;
      bLOGIC = false;
      break;
    }
  }
  //RPM = SpeedActual * 5000.0f / 32767.0f ; 
  //float DCvoltage1 = DCvoltage / 31.5848f - 27;
  //display.showNumberDec(ACvoltage);
  //currentTime2= millis(); 
  Serial.println("-------------------------");//Serial.print((currentTime2-currentTime1));Serial.print("-------------------------");
  //Serial.print("SpeedActual : ");Serial.println(SpeedActual);
  //Serial.print("SpeedRpmMax : ");Serial.println(SpeedRpmMax);
  //Serial.print("RPM : ");Serial.println(RPM);
  Serial.print("M_Out : ");Serial.println(M_Out);
  //Serial.print("Current : ");Serial.println(Current);
  
  //Serial.print("Current200pc : ");Serial.println(Current200pc);
  //Serial.print("CurrentDevice : ");Serial.println(CurrentDevice);
  //Serial.print("I_Max_PK : ");Serial.println(I_Max_PK);
  //Serial.print("DCvoltage : ");Serial.println(DCvoltage1);
  //Serial.print("ACvoltage : ");Serial.println(ACvoltage);
  //Serial.print("PowerStage_Temp : ");Serial.print(PowerStage_Temp);Serial.print(" / ");Serial.println(pst(PowerStage_Temp));
  //Serial.print("Air_Temp : ");Serial.print(Air_Temp);Serial.print(" / ");Serial.println(at(Air_Temp));
  
}
  


/////////////////////////////////////////////////////send/////////////////////////////////////////
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
    if (millis() - t > 1000) {
      ret = (byte)1;
      Serial.println(ret);
      break;
    }
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK && canMsg.can_id == tx) {
      int dlc = canMsg.can_dlc;
      Serial.println("[모터 컨트롤러에서 받은 메시지]");
      Serial.println(canMsg.data[0], HEX);
      for (int i = 0; i < dlc; i++) {
        recv[i] = canMsg.data[i];
        Serial.print(recv[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      ret = canMsg.data[0];
      break;
    }
  }
  return ret;
}


////////////////////////////////////////////컨트롤러 온도 계산함수/////////////////////////////////////////
double pst(int16_t x){
  double pstemp;
  float  p1 =  -2.8e-15;
  float  p2 =  3.375e-10;
  float  p3 =  -1.426e-05;
  float  p4 =  0.2651;
  float  p5 =  -1810;
  pstemp = p1*pow(x,4) + p2*pow(x,3) + p3*pow(x,2) + p4*x + p5;
  return pstemp;
}


//////////////////////////////////////////// 외기 온도 계산함수/////////////////////////////////////////
double at(int16_t x){
  double atemp;
  float p1 =   3.804e-15 ;
  float p2 =   -1.24e-10 ; 
  float p3 =    1.71e-06  ;
  float p4 =    0.006557 ;
  float p5 =        -158 ;
  atemp = p1*pow(x,4) + p2*pow(x,3) + p3*pow(x,2) + p4*x + p5;
  return atemp;
}

