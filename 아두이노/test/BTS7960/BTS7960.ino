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
  Serial.println("DC motor test");
}

void loop(){
  if(Serial.available() > 0) {
    String rad = Serial.readStringUntil('\n');
    if (rad == "0"){
      digitalWrite(REN, LOW);
      digitalWrite(LEN, LOW);
      analogWrite(PB4, 0);
      Serial.println(rad);
    }
    else if (rad == "50"){
      digitalWrite(REN, HIGH);
      digitalWrite(LEN, HIGH);
      analogWrite(PB4, 50);
      Serial.println(rad);
    }
    else if (rad == "100"){
      digitalWrite(REN, HIGH);
      digitalWrite(LEN, HIGH);
      analogWrite(PB4, 100);
      Serial.println(rad);
    }
    else if (rad == "170"){
      digitalWrite(REN, HIGH);
      digitalWrite(LEN, HIGH);
      analogWrite(PB4, 170);
      Serial.println(rad);
    }
    else if (rad == "200"){
      digitalWrite(REN, HIGH);
      digitalWrite(LEN, HIGH);
      analogWrite(PB4, 200);
      Serial.println(rad);
    }
    else if (rad == "255"){
      digitalWrite(REN, HIGH);
      digitalWrite(LEN, HIGH);
      analogWrite(PB4, 255);
      Serial.println(rad);
    }
    else{
      analogWrite(PB4, 0);
      digitalWrite(REN, LOW);
      digitalWrite(LEN, LOW);
      Serial.println(rad);

    }
  }
}
