const int aena = 5;
const int adir1 = 6;
const int adir2 = 7;


void setup(){
  Serial.begin(9600);
  pinMode(aena, OUTPUT);
  pinMode(adir1, OUTPUT);
  pinMode(adir2, OUTPUT);

  Serial.println("DC motor test");
}

void loop(){
  if(Serial.available() > 0) {
    String rad = Serial.readStringUntil('\n');
    if (rad == "0"){
      analogWrite(aena, 0);
      Serial.println(rad);
    }
    else if (rad == "50"){
      analogWrite(aena, 50);
      digitalWrite(adir1, 1);
      digitalWrite(adir2, 0);
      Serial.println(rad);
    }
    else if (rad == "100"){
      analogWrite(aena, 100);
      digitalWrite(adir1, 1);
      digitalWrite(adir2, 0);
      Serial.println(rad);
    }
    else if (rad == "150"){
      analogWrite(aena, 150);
      digitalWrite(adir1, 1);
      digitalWrite(adir2, 0);      
      Serial.println(rad);
    }
    else if (rad == "200"){
      analogWrite(aena, 200);
      digitalWrite(adir1, 1);
      digitalWrite(adir2, 0);      
      Serial.println(rad);
    }
    else if (rad == "255"){
      analogWrite(aena, 255);
      digitalWrite(adir1, 1);
      digitalWrite(adir2, 0);      
      Serial.println(rad);
    }
    else{
      analogWrite(aena, 255);
      Serial.println(rad);
      digitalWrite(adir1, 1);
      digitalWrite(adir2, 0);      
    }
  }
}
