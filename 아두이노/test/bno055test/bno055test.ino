#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
uint16_t BNO055_SAMPLERATE_DELAY_MS = 300;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
int16_t LT_accele, LN_accele, Roll, Pitch, roll = 0, pitch =0;
float LT_accele1, LN_accele1;
void accsen(sensors_event_t* event);
void accsen1(sensors_event_t* event);



void setup() {
 Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

}

void loop() {
  sensors_event_t orientationData, linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  accsen(&linearAccelData);
  accsen(&orientationData);
  delay(BNO055_SAMPLERATE_DELAY_MS);

  if(Serial.available()){
    char c = Serial.read();
    if (c == '2'){
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      accsen1(&orientationData);
    
    
    }


  }

}



////////////////////////////////////////////가속도 센서/////////////////////////////////////////
void accsen(sensors_event_t* event){
  
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    Roll = -1 * (event->orientation.z);
    Serial.print(F("Roll  : "));Serial.print(Roll - roll);
    Pitch = -1 * (event->orientation.y);
    Serial.print("\tPitch : ");Serial.println(Pitch - pitch);
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    LT_accele = -1 * (event->acceleration.x);
    //Serial.print(F("LT_accele : "))Serial.println(-LT_accele1);
    LN_accele = -1 * (event->acceleration.z);
    LN_accele = LN_accele1 * 10;
    //Serial.print("\tLN_accele : ");Serial.println(LN_accele1);
    //Zaccle1 = event->acceleration.z;
    //Zaccle = Zaccle1 * 10;
    //Serial.print("\tZ: ");Serial.println(Zaccle1);
    Serial.println(F("=====================================================\n"));
  }
  else {
  Serial.print("Unk:");
  }
  return;
}

////////////////////////////////////////////가속도 센서 offset/////////////////////////////////////////
void accsen1(sensors_event_t* event){
  roll = -1 * (event->orientation.z);
  pitch = -1 * (event->orientation.y);
  
  return;
}