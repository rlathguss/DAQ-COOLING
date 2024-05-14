volatile int flow_frequency; // Measures flow sensor pulses
float l_min; // Calculated litres/min
unsigned char flowsensor = 2; // Sensor Input
unsigned long currentTime;
unsigned long cloopTime;


void setup() {
  pinMode(flowsensor, INPUT);
  digitalWrite(flowsensor, HIGH); // Optional Internal Pull-Up
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(2), flow, RISING); // Setup Interrupt
  currentTime = millis();
  cloopTime = currentTime;
  Serial.println("Flow sensor ready");
}

void loop() {
 currentTime = millis();
  if(currentTime >= (cloopTime + 400)){         // Every second, calculate and print litres/hour
    cloopTime = currentTime;                    // Updates cloopTime
    l_min = (2.5 *flow_frequency / 11);         // Pulse frequency (Hz) = 11Q, Q is flow rate in L/min.
    flow_frequency = 0;                         // Reset Counter
    Serial.print(l_min);                        // Print litres/min
    Serial.println(" L/Min");
  }
}



void flow () // Interrupt function
{
  flow_frequency++;
}