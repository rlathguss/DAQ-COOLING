#include <Arduino.h>
#include <TimerOne.h>
int interruptPin = 2;
int rpm;
float rev ;
int rev1;
unsigned long currentTime1;
unsigned long cloopTime1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(interruptPin),isr, RISING);
  currentTime1 = millis();
  cloopTime1 = currentTime1;
}

void loop() {
  currentTime1 = millis();                         // Every 0.4 second, calculate and print litres/min
  if (currentTime1 >= (cloopTime1 + 500)) {
    cloopTime1 = currentTime1;                     // Updates cloopTime
    rpm = (rev1 * 120 );             // Pulse frequency (Hz) = 11Q, Q is flow rate in L/min.
    rev = 0;
    Serial.println(rpm);
  }
}

void isr(){
  rev++;
  rev1 = rev /4;
  //Serial.println(rev1);
}