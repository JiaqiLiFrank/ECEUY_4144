#include <Arduino.h>

// PD0, INT0, Detecting Transition Down

uint8_t FLAG = 0;

// WHat written in the function will run once the INT0 falls from high to low
ISR(INT0_vect){
  FLAG = 1;
}

void setup(){
  // Falling Edgeing Setup
  EICRA |= (1<<1);
  // Arm INT0
  EIMSK |= (1<<0);
  Serial.begin(9600);
}

void loop(){
  if(FLAG == 1){
    Serial1.println("YES");
  }
}