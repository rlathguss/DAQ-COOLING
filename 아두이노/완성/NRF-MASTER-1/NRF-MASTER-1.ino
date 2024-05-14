#include <Arduino.h>
#include <SPI.h>
#include "RF24.h"
#include <math.h>
RF24 radio(9, 10); //CE, SS
uint8_t address[6] = "41715";

unsigned long Status, ERROR;
void parser(unsigned long x);
byte binary[32] = { 0, };
unsigned long currentTime1;
unsigned long STET, SAFE;

// uint16_t CurrentDevice = 2000, Current200pc = 1070, I_Max_PK = 424.2; //0xCE, 0xC6, 0xD9, (0xC4 A단위)
uint16_t Status1, Status2, ERROR1, ERROR2, Cp, Hp;
int RPM, Motor_Temp, PowerStage_Temp, LPM, Roll, Pitch, Rad, LT_accele, LN_accele, CoolantTemp1, CoolantTemp2,
    Torque_Out, iACcurrent, iACvoltage, iACpower1, iACpower2, iDCcurrent, iDCvoltage, iDCpower, SOC;
float ACcurrent, ACvoltage, ACpower1, ACpower2, DCcurrent, DCvoltage, DCpower;

float pSOC(float x);

void setup() {
  Serial.begin(115200);
  radio.begin(); 
  radio.setPALevel(RF24_PA_MAX); 
  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address);
  radio.startListening(); 
  currentTime1 = millis();
  STET = currentTime1;;
}


void loop() {
  radio.startListening();
  currentTime1 = millis();
  int i_;
  char tmp[32];
  if(radio.available()){
    int datapack[16] = { 0 ,};
    radio.read(datapack, sizeof(datapack));
    if (datapack[0] == 7){
      Serial.println("{\"TQ1\":1,\"Rad\":77}");
      //Rad = 77;
      //Serial.println("보드 초기화 응답");
      }
    else if (datapack[0] == 6){
      Serial.println("{\"TQ1\":1,\"Rad\":66}");
      //Rad = 66;
      //Serial.println("CAN 재요청 활성화 응답");
    }
    else if (datapack[0] == 5){
      Serial.println("{\"TQ1\":1,\"Rad\":55}");
      //Rad = 55;
      //Serial.println("CAN CLEAR   응답");
    }
    else if (datapack[0] == 11){
      RPM = datapack[1];
      Motor_Temp = datapack[2];
      PowerStage_Temp = datapack[3];
      LPM = datapack[4];
      Rad = datapack[5];
      

      Torque_Out = datapack[6];                                                        //NM 10 곱해진 값
      ACpower1 = ((float)RPM) * ((float)Torque_Out) / 10 / 9550.0f;                                           //kW
      iACpower1 = 100 * ACpower1;

      ACcurrent = ((float)datapack[7]) * 0.373832f;                                   //Arms CurrentDevice / Current200pc
      iACcurrent = 10 * ACcurrent;
      DCvoltage = (((float)datapack[8]) / 31.5848f - 27.0f);                           //V
      SOC = pSOC(DCvoltage);
      iDCvoltage = 10 * DCvoltage;
      ACvoltage = DCvoltage / 1.4142f * ((float)datapack[9]) / 4096.0f * 0.92f;        //Vrms
      iACvoltage = 10 * ACvoltage;
      ACpower2 = ACcurrent * ACvoltage * 1.732f / 1000.0f;                             //kW
      iACpower2 = 100 * ACpower2;
      DCpower = ACpower2 / 0.92f;                                                      //kW
      iDCpower = 100 * DCpower;   
      DCcurrent = DCpower / DCvoltage * 1000.0f;                                     //A
      iDCcurrent = 10 * DCcurrent;
      LN_accele = datapack[10];
      LT_accele = datapack[11];
      Roll = datapack[12];
      Pitch = datapack[13];

      CoolantTemp1 = datapack[14];
      CoolantTemp2 = datapack[15];

      int len;
      char* key_11[] = {
        "RPM", "MT", "PT", "LPM",
        "Rad", "T", "ACp1",
        "ACc", "DCv", "ACv", "ACp2",
        "DCp", "DCc", "LN_accele", "LT_accele",
        "Roll", "Pitch", "CT1", "CT2", "SOC" };

      int* value_11[] = {
        &RPM, &Motor_Temp, &PowerStage_Temp, &LPM,
        &Rad, &Torque_Out, &iACpower1,
        &iACcurrent, &iDCvoltage, &iACvoltage, &iACpower2,
        &iDCpower, &iDCcurrent, &LN_accele, &LT_accele,
        &Roll, &Pitch, &CoolantTemp1, &CoolantTemp2, &SOC };

      memset(tmp, 0x0, sizeof(tmp));
      snprintf(tmp, sizeof(tmp), "{\"TQ1\":1,");
      Serial.print(tmp);

      for (i_ = 0; i_ < sizeof(key_11) / sizeof(int) - 1; i_++){
        memset(tmp, 0x0, sizeof(tmp));
        snprintf(tmp, sizeof(tmp), "\"%s\":", key_11[i_]);
        Serial.print(tmp);
        Serial.print(*(value_11[i_]));
        Serial.print(",");
      }
      memset(tmp, 0x0, sizeof(tmp));
      snprintf(tmp, sizeof(tmp), "\"%s\":", key_11[i_], *(value_11[i_]));
      Serial.print(tmp);
      Serial.print(*(value_11[i_]));
      Serial.print("}\n");
    }
    else if (datapack[0] == 22 && currentTime1 >= (STET + 500)){
      int len;
      char output_char[250] = { 0 };
      char* key_22[] = {
        "Ena", "NcR0", "L1", "L2",
        "OK", "Icns", "TNlim", "PN",
        "NI", "N0", "Rsw", "Cal0",
        "Cal", "Tol", "Rdy", "Brk0",
        "SM", "Nc", "Nc1", "Nc2",
        "Dig", "Iuse", "IrdN", "TI",
        "TIR", "HZ", "TM", "Ana",
        "Iwcns", "RFE"};

      int index_22[] = {
        0, 1, 2, 3,
        4, 5, 6, 7,
        8, 9, 10, 11,
        12, 13, 14, 15,
        16, 17, 18, 19,
        20, 21, 22, 23,
        24, 25, 26, 27,
        28, 29};

      // Cp = datapack[5];
      // Hp = datapack[6];

      Status1 = datapack[1];
      Status2 = datapack[2];
      Status = ((unsigned long)Status2 << 16) | Status1;
      memset(binary, 0x0, sizeof(binary));
      parser(Status);

      memset(tmp, 0x0, sizeof(tmp));
      snprintf(tmp, sizeof(tmp), "{\"TQ2\":1,\"S\":%lu,", Status);
      Serial.print(tmp);

      for (i_ = 0; i_ < sizeof(key_22)/sizeof(int) - 1; i_++){
        memset(tmp, 0x0, sizeof(tmp));
        snprintf(tmp, 32, "\"%s\":", key_22[i_]);
        Serial.print(tmp);
        Serial.print(binary[index_22[i_]]);
        Serial.print(",");
      }
      memset(tmp, 0x0, sizeof(tmp));
      snprintf(tmp, 32, "\"%s\":", key_22[i_]);
      Serial.print(tmp);
      Serial.print(binary[index_22[i_]]);
      Serial.print("}\n");

      char* key_33[] = {
        "E0","E1","E2","E3",
        "E4","E5","E6","E7",
        "E8","E9","EA","EB",
        "EE","EF","W0","W1",
        "W2","W6","W7","W8",
        "W9","WA","WF" };

      int index_33[] = {
        0, 1, 2, 3,
        4, 5, 6, 7,
        8, 9, 10, 11,
        14, 15, 16, 17,
        18, 22, 23, 24,
        25, 26, 31 };

      ERROR1 = datapack[3];
      ERROR2 = datapack[4];
      ERROR = ((unsigned long)ERROR2 << 16) | ERROR1;
      memset(binary, 0x0, sizeof(binary));
      parser(ERROR);

      memset(tmp, 0x0, sizeof(tmp));
      snprintf(tmp, sizeof(tmp), "{\"TQ3\":1,");      
      //snprintf(tmp, sizeof(tmp), "{\"TQ3\":1,\"Cp\":%d,\"Hp\":%d,\"LPM\":%d,", Cp, Hp, LPM);      
      Serial.print(tmp);

      for (i_ = 0; i_ < sizeof(key_33) / sizeof(int) - 1; i_++){
        memset(tmp, 0x0, sizeof(tmp));
        snprintf(tmp, sizeof(tmp), "\"%s\":", key_33[i_], binary[index_33[i_]]);
        Serial.print(tmp);
        Serial.print(binary[index_33[i_]]);
        Serial.print(",");
      }
      memset(tmp, 0x0, sizeof(tmp));
      snprintf(tmp, 32, "\"%s\":", key_33[i_], binary[index_33[i_]]);
      Serial.print(tmp);
      Serial.print(binary[index_33[i_]]);
      Serial.print("}\n");

      STET = millis();
    }
  }
  if(currentTime1 >= (SAFE + 5000)){
    int16_t trf[1];
    trf[0] = 3;
    radio.stopListening();
    for (int i = 0; i<4; i++){
      radio.write(trf, sizeof(trf));
      //Serial.println("주기적 통신 확인용 전송");
    }
    SAFE = millis(); 
  }  
  seral_read_RF_write();
}





//////////////////////////////////////////시리얼 입력 후 slave에 명령 전달 /////////////////////////////////////////
void seral_read_RF_write(){
  if(Serial.available()){
    char c = Serial.read();
    int16_t trf[3];
    if (c == '2'){
      trf[0] = 2;
      radio.stopListening();
      for (int i = 0; i<4; i++){
        radio.write(trf, sizeof(trf));
        //Serial.println("가속도 센서 offset");
      }
    }
    else if (c == '4'){
      trf[0] = 4;
      trf[1] = Cp;            // 10 곱해진 값
      trf[2] = Hp;            // 10 곱해진 값
      radio.stopListening();
      for (int i = 0; i<4; i++){
        radio.write(trf, sizeof(trf));
        //Serial.println("Hp, Cp 백업");
      }
    }
    else if (c == '5'){
      trf[0] = 5;
      radio.stopListening();
      for (int i = 0; i<4; i++){
        radio.write(trf, sizeof(trf));
        //Serial.println("Clear can line");
      }
    }
    else if (c == '6'){
      trf[0] = 6;
      radio.stopListening();
      for (int i = 0; i<4; i++){
        radio.write(trf, sizeof(trf));
        //Serial.println("can 자동요청 코드 재활성화");
      }
    }
    else if (c == '7'){
      trf[0] = 7;
      radio.stopListening();
      for (int i = 0; i<4; i++){
        radio.write(trf, sizeof(trf));
        //Serial.println(" stm32 reset");
      }
    }
    else if(c == '8'){
      trf[0] = 8;
      trf[1] = 1;
      radio.stopListening();
      for (int i = 0; i<4; i++){
        radio.write(trf, sizeof(trf));
        //Serial.println("Hp / Cp 자동제어");
      }
    }
    else if (c == 'e'){
      trf[0] = 8;
      trf[1] = 2;
      radio.stopListening();
      for (int i = 0; i<4; i++){
        radio.write(trf, sizeof(trf));
        //Serial.println("Mt, Pt 자동제어");
      }
    }
    else if (c == '9'){
      trf[0] = 9;
      trf[1] = 9;
      radio.stopListening();
      for (int i = 0; i<4; i++){
        radio.write(trf, sizeof(trf));
        //Serial.println("팬 속도 수동제어 255");
      }
    }
    else if (c == 'a'){
      trf[0] = 9;
      trf[1] = 1;
      radio.stopListening();
      for (int i = 0; i<4; i++){
        radio.write(trf, sizeof(trf));
        //Serial.println("팬 속도 수동제어 0");
      }
    }
    else if (c == 'b'){
      trf[0] = 9;
      trf[1] = 2;
      radio.stopListening();
      for (int i = 0; i<4; i++){
        radio.write(trf, sizeof(trf));
        //Serial.println("팬 속도 수동제어 200");
      }
    }
    else if (c == 'c'){
      trf[0] = 9;
      trf[1] = 3;
      radio.stopListening();
      for (int i = 0; i<4; i++){
        radio.write(trf, sizeof(trf));
        //Serial.println("팬 속도 수동제어 150");
      }
    }
    else if (c == 'd'){
      trf[0] = 9;
      trf[1] = 4;
      radio.stopListening();
      for (int i = 0; i<4; i++){
        radio.write(trf, sizeof(trf));
        //Serial.println("팬 속도 수동제어 100");
      }
    }
  }
  return;
}

////////////////////////////////////////////SOC 계산함수/////////////////////////////////////////
float pSOC(float x){
  x = x / 70; 
  float y;
  if ( x <= 3.4 ){
    y = 12.5 * x -37.5;
  }
  else if( x <= 3.56 ){
    y = 31.25 * x - 101.25;
  } 
  else if( x <= 3.61 ){
    y = 200 * x - 702;
  } 
  else if( x <= 4.03 ){
    y = 142.8571 * x - 495.7143;
  } 
  else if( x <= 4.17 ){
    y = 107.1429 * x - 351.7857;
  } 
  else{
    y = 4444;
  }


  if(y <=0 ){
    y = 0;
  }
  return y;
}

////////////////////////////////////////////변수 할당 함수 /////////////////////////////////////////
void parser(unsigned long x){
  int position = 0;
  binary[32] = { 0, };
  while (1){
    binary[position] = x % 2;    // 2로 나누었을 때 나머지를 배열에 저장
    x = x / 2;                   // 2로 나눈 몫을 저장
    position++;                  // 자릿수 변경
    if (x == 0){                 // 몫이 0이 되면 반복을 끝냄
      break;
    }
  }
  // 배열의 요소 출력
  // for (int i = 0; i <= 31; i++){ 
  //  Serial.print(binary2[i]);
  // }
  // Serial.println(" ");
   return ;
}
