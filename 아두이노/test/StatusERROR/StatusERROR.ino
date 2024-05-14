unsigned long x = 42091345;


#include <Arduino.h>
byte binary[32] = { 0, };

void setup() {
  Serial.begin(115200);

}

void loop() {


 cStatus(x);
 delay(5000);
}



void cStatus(unsigned long x){
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

  //배열의 요소를 역순으로 출력
  for (int i = 0; i <= 31; i++){ 
   Serial.print(binary[i]);
  }
  Serial.println(" ");
   return ;
}
