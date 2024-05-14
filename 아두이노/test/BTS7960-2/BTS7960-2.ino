const int REN = 7;           //정회전 활성화 핀
const int LEN = 6;           //역회전 활성화 핀
HardwareTimer pwmtimer3(TIM3);


void setup(){
  Serial.begin(115200);
  analogWriteFrequency(20000);
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);
  pinMode(PB4, OUTPUT);
  digitalWrite(REN, LOW);
  digitalWrite(LEN, LOW);
  //Serial.println("DC motor test");
}

void loop(){
  int rad;
  if(Serial.available()) {
    String data = Serial.readString();
    //Serial.println(data);
    rad = data.toInt();
    digitalWrite(REN, HIGH);
    digitalWrite(LEN, HIGH);
    analogWrite(PB4, rad);
  }
}

