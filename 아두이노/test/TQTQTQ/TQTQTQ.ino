#include <Arduino.h>
#include <EEPROM.h>
#include <TM1637Display.h>

const byte CLK = 3;                                   // define CLK pin (any digital pin)s
const byte DIO = 4;                                   // define DIO pin (any digital pin)
TM1637Display display(CLK, DIO);                      // define dispaly object

float defaultValue = 3.14; // 기본 값 설정
float* valueAddress = (float*)0x0802 0000; // 내부 Flash 메모리 주소 (예시)


void setup() {
  Serial.begin(115200);
  display.setBrightness(0x0f);
  display.showNumberDec(0000);
  //Serial.println(value);

  // 저장된 데이터를 읽어옴
  float storedValue = *valueAddress;

  // 저장된 값이 기본 값과 다르면 기본 값으로 설정하고 Flash 메모리에 저장
  if (storedValue != defaultValue) {
      *valueAddress = defaultValue;
  }
}

void loop() {
  // 이후에는 Flash 메모리에서 읽어온 값을 사용하면 됨
  float value;
  Serial.println(value);
  display.showNumberDec(1111);
  delay(5000);
  value = *valueAddress;
  Serial.println(value);
  delay(5000);
  display.showNumberDec(7474); 
  delay(5000);
  HAL_NVIC_SystemReset();

  // value 변수에 저장된 값 사용
}