#include <Arduino.h>
#include <SPI.h>
#include "RF24.h"
#include <ArduinoJson.h>
#include <math.h>

//Lin_vel = (Diameter) * RPM * PI * 60/1000000/2.8f;      //KM/H  기어비 2.8
RF24 radio(9, 10); //CE, SS
uint8_t address[6] = "41715";
const float Diameter = 525.0;  //mm단위
int16_t SpeedRpmMax = 5000, CurrentDevice = 2000, Current200pc = 1070, I_Max_PK = 424.2;   //0xCE, 0xC6, 0xD9, (0xC4 A단위)
int  RPM, Xangle, Yangle, Rad ;
uint16_t Status1, Status2, Err1, Err2; 
float Motor_Temp, PowerStage_Temp, LPM, 
      Torque_Out, I_des, ACcurrent, ACvoltage, ACpower1, ACpower2, DCcurret, DCvoltage, DCpower,
      Xaccle, Yaccle, Zaccle, CoolantTemp1, CoolantTemp2 ;

bool bRad = true;
bool d1 = false;
bool d2 = false;
double mt(uint16_t x);
double pst(uint16_t x);
unsigned long Status, Err;
void cStatus(unsigned long x);
void cErr(unsigned long Y);
byte binary[33] = {0};
byte binary1[33] = {0};


void setup() {
  Serial.begin(115200);
  radio.begin(); 
  radio.setPALevel(RF24_PA_MAX); 
  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address);
  radio.startListening(); 
}


void loop() {
  radio.startListening();
  while (1){
    if(radio.available()){
      int datapack[16];
      radio.read(datapack,sizeof(datapack));
      if (datapack[0] == 11){
        RPM = datapack[1];
        Motor_Temp = mt(datapack[2]);
        Motor_Temp = floor(Motor_Temp * 10)/10;    
        PowerStage_Temp = pst(datapack[3]);
        PowerStage_Temp= floor(PowerStage_Temp * 10)/10;
        LPM = datapack[4]/10.0f;

        Torque_Out = datapack[5] * I_Max_PK * 0.61f / 32767.0f / 1.4142f;
        Torque_Out = floor(Torque_Out * 10)/10;
        //I_des = Torque_Out / 0.61f;                                              //Arms
        //I_des = floor(I_des * 10)/10;
        ACpower1 = RPM * Torque_Out / 9550.0f;                                   //kW
        ACpower1 = floor(ACpower1 * 10)/10;
        ACcurrent = datapack[6] * CurrentDevice / Current200pc / 5.0f ;            //Arms
        ACcurrent = floor(ACcurrent * 10)/10;
        DCvoltage = datapack[7] / 31.5848f;                                        //V
        DCvoltage = floor(DCvoltage * 10)/10;
        ACvoltage = DCvoltage / 1.4142f * datapack[8] / 4096.0f * 0.92f;           //Vrms
        ACvoltage = floor(ACvoltage * 10)/10;
        //ACpower2 = ACcurrent * ACvoltage * 1.732f;                               //kW
        //ACpower2 = floor(ACpower2 * 10)/10;
        //DCpower = ACpower2 / 0.92f;                                              //kW
        //DCpower = floor(DCpower * 10)/10;
        //DCcurrent = DCpower / DCvoltage;                                         //A
        //DCcurrent = floor(DCcurrent * 10)/10;

        Xaccle = datapack[9]/10.0f;
        Yaccle = datapack[10]/10.0f;
        Zaccle = datapack[11]/10.0f;
        Xangle = datapack[12];
        Yangle = datapack[13];

        CoolantTemp1 = datapack[14]/10.0f;
        CoolantTemp2 = datapack[15]/10.0f;
        d1 = true; 
      // Serial.println("11111111111111111받음");
      }
      else if (datapack[0] == 22){
        Rad = datapack[1];
        Status1 = datapack[2];
        Status2 = datapack[3];
        Status = ((unsigned long)Status2 << 16) | Status1;
        cStatus(Status);

        Err1 = datapack[4];
        Err2 = datapack[5];
        Err = ((unsigned long)Err2 << 16) | Err1;
        cErr(Err);
        d2 = true;
        //Serial.println("22222222222222222받음");
      }
    }
    if(d1 == true && d2 == true ){
      d1 = false;
      d2 = false;
      break;
    }
  }

  String output;
  StaticJsonDocument<512> doc;

  doc["RPM"] = RPM;
  doc["MT"] = Motor_Temp;
  doc["PT"] = PowerStage_Temp;
  doc["LPM"] = LPM;

  doc["T"] = Torque_Out;
  //doc["I"] = I_des;  
  //doc["ACp1"] = ACpower1;
  doc["ACc"] = ACcurrent;
  doc["DCv"] = DCvoltage;
  doc["ACv"] = ACvoltage;
  //doc["ACp2"] = ACpower2;
  //doc["DCp"] = DCpower;
  //doc["DCc"] = DCcurrent;

  doc["Xac"] = Xaccle;
  doc["Yac"] = Yaccle;
  doc["Zac"] = Zaccle;
  doc["Xan"] = Xangle;
  doc["Yan"] = Yangle;
  doc["CT1"] = CoolantTemp1;
  doc["CT2"] = CoolantTemp2;

  doc["Rad"] = Rad;
  //doc["S"] = Status;
  doc["Ena"] = binary[1];
  doc["NcR0"] = binary[2];
  doc["L1"] = binary[3];
  doc["L2"] = binary[4];
  doc["OK"] = binary[5];
  doc["Icns"] = binary[6];
  doc["TNlim"] = binary[7];
  doc["PN"] = binary[8];
  doc["NI"] = binary[9];
  doc["N0"] = binary[10];
  doc["Rsw"] = binary[11];
  doc["Cal0"] = binary[12]; 
  doc["Cal"] = binary[13];
  doc["Tol"] = binary[14];
  doc["Rdy"] = binary[15];
  doc["Brk0"] = binary[16];
  doc["SM"] = binary[17];
  doc["Nc"] = binary[18];
  doc["Nc1"] = binary[19];
  doc["Nc2"] = binary[20];
  doc["Dig"] = binary[21];
  doc["Iuse"] = binary[22];
  doc["IrdN"] = binary[23];
  doc["TI"] = binary[24];
  doc["TIR"] = binary[25];
  doc["HZ"] = binary[26];
  doc["TM"] = binary[27];
  doc["Ana"] = binary[28];
  doc["Iwcns"] = binary[29];
  doc["RFE"] = binary[30];
  doc["Hnd"] = binary[32];

  //doc["E"] = Err;
  //doc["E0"] = binary1[1];
  doc["E1"] = binary1[2];                      //powerfault
  doc["E2"] = binary1[3];                      //RFE
  //doc["E3"] = binary1[4];
  doc["E4"] = binary1[5];                      //FEEDBACK
  //doc["E5"] = binary1[6];                      //POWERVOLTAGE
  //doc["E6"] = binary1[7];                      //MOTORTEMP
  //doc["E7"] = binary1[8];                      //DEVICETEMP
  //doc["E8"] = binary1[9];                      //OVERVOLTAGE
  doc["E9"] = binary1[10];                     //I_PEAK
  // doc["EA"] = binary1[11];
  // doc["EB"] = binary1[12]; 
  // doc["EE"] = binary1[13];
  // doc["EF"] = binary1[14];

  //doc["e0"] = binary1[17];                     //Device detection inconsistent
  //doc["e1"] = binary1[18];                     //RUN signal disturbed, EMI
  //doc["e2"] = binary1[19];                     //RFE input inactive (without RUN input active)
  //doc["e5"] = binary1[22];                     //Power voltage too low or missing
  doc["e6"] = binary1[23];                     //Motor temperature > (I-red-TM or 93% of M-Temp)
  doc["e7"] = binary1[24];                     //Device temperature >87% of limit
  //doc["e8"] = binary1[25];                     //Limit of existing voltage output reached
  doc["e9"] = binary1[26];                     //Overcurrent 200%
  //doc["ea"] = binary1[27];                     //Resolution rage of the speed measurement exceeded
  //doc["ef"] = binary1[32];                     //Ballast circuit > 87% overloaded

  serializeJson(doc, output);
  Serial.println(output); 
  
  if(Serial.available()){
    char c = Serial.read();
    int16_t trf[1];
    if(c == '9'){
      trf[0] = 9;
      radio.stopListening();
      for (int i = 0; i<4; i++){
        radio.write(trf, sizeof(trf));
        //Serial.println("9999999999999999999999999999999999");
      }
    }
    else if(c == '4'){
      trf[0] = 4;
      radio.stopListening();
      for (int i = 0; i<4; i++){
        radio.write(trf, sizeof(trf));
        //Serial.println("4444444444444444444444444444");
      }
    }
  }
}


////////////////////////////////////////////Status 변수 할당 함수 /////////////////////////////////////////
void cStatus(unsigned long x){
  int position = 1;
  binary[33] = {0};
  while (1){
    binary[position] = x % 2;    // 2로 나누었을 때 나머지를 배열에 저장
    x = x / 2;                   // 2로 나눈 몫을 저장
    position++;                  // 자릿수 변경
    if (x == 0){                 // 몫이 0이 되면 반복을 끝냄
      break;
    }
  }

  // 배열의 요소를 0-31 출력
  // for (int i = 1; i <= 32; i++){ 
  //  Serial.print(binary[i]);
  // }
  // Serial.println(" ");
  // 배열의 요소를 역순으로 출력
  // for (int i = 32 - 1; i >= 0; i--){ 
  //  Serial.print(binary[i]);
  // }
  // Serial.println(" ");
   return ;
}


////////////////////////////////////////////Err 변수 할당 함수 /////////////////////////////////////////
void cErr(unsigned long y){
  int position1 = 1;
  binary1[33] = {0};
  while (1){
    binary1[position1] = y % 2;    // 2로 나누었을 때 나머지를 배열에 저장
    y = y / 2;                   // 2로 나눈 몫을 저장
    position1++;                  // 자릿수 변경
    if (y == 0){                 // 몫이 0이 되면 반복을 끝냄
      break;
    }
  }

  //배열의 요소를 역순으로 출력
  // for (int i = position - 1; i >= 0; i--){ 
  //  Serial.print(binary[i]);
  // }
  // Serial.println(" ");
   return ;
}


////////////////////////////////////////////모터 온도 계산함수/////////////////////////////////////////
double mt(uint16_t x){
  double mtemp;
  float p1 =  -1.387e-16 ;
  float p2 =   3.614e-11 ; 
  float p3 =  -1.009e-06  ;
  float p4 =     0.02741 ;
  float p5 =      -196.9 ;
  mtemp = p1*pow(x,4) + p2*pow(x,3) + p3*pow(x,2) + p4*x + p5;
  
  return mtemp;
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
//double at(int16_t x){
//   double atemp;
//   float p1 =   3.804e-15 ;
//   float p2 =   -1.24e-10 ; 
//   float p3 =    1.71e-06  ;
//   float p4 =    0.006557 ;
//   float p5 =        -158 ;
//   atemp = p1*pow(x,4) + p2*pow(x,3) + p3*pow(x,2) + p4*x + p5;
//   return atemp;
//}




