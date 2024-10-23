#include <Arduino.h>

/* 
  Sound sense:
    PF1, ADC1
  Light Sense:
    PF0, ADC0
*/
void setup() {
  Serial.begin(9600);
  ADMUX = (1<<REFS0) | (1<<MUX2) | (1<<MUX1) | (1<<MUX0);
  ADCSRA = (1<<ADEN) | (1<<ADATE);
  ADCSRB = (1<<ADHSM);
  DIDR0 |= (1<<ADC7D);
  ADCSRA |= (1<<ADSC);
}

void loop() {
  Serial.println(ADCW);
  delay(10);
}