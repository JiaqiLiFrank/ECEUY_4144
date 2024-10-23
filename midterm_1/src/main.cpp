#include <Arduino.h>


/*
  Bay Lock Valve - PD0
  Ocean Lock Valve - PD1
  Bay Lock - PD2
  Ocean Lock - PD3
  BL Status - PD4
  OL Status - PD5
  Transition - PD6

  Bay Level - PC1, ADC0
  Transition Level - PC2, ADC1
  Ocean Level - PC3, ADC2
*/

void setup() {
  // Set up GPIO
  // Set up DDR
  DDRD |= (1<<0);
  DDRD |= (1<<1);
  DDRD |= (1<<2);
  DDRD |= (1<<3);
  DDRD &= ~(1<<4);
  DDRD &= ~(1<<5);
  DDRD &= ~(1<<6);

  // Set up PORT
  PORTD &= ~(1<<PORTD0); // Bay Lock Valve
  PORTD &= ~(1<<PORTD1); // Ocean Lock Valve
  PORTD &= ~(1<<PORTD2); // Bay Lock
  PORTD &= ~(1<<PORTD3); // Ocean Lock

  // Set up the ADC
  ADMUX = (1<<REFS0); // Setting the Vcc as the reference
  ADCSRA = (1<<ADEN) | (1<<ADATE) | (1<<ADPS1) | (1<<ADPS0); // ADC Enable; Auto Trigger Enable; Prescaler 8. 
  // ADCSRB at Free Running Mode.
}

uint8_t beginTransitBayToOcean(){
  // Either lock is open or a boat is already in transition
  if ((PIND & (1<<PIND4)) || (PIND & (1<<PIND5)) || (PIND & (1<<PIND6))) {
    return 0; 
  }

  // Open the Bay Lock Valve
  PORTD |= (1<<PORTD0);

  while(true) {
  // Set up ADC0 - Bay Level
  DIDR0 = (1<<ADC0D);
  uint16_t bay_lv = ADCW;

  // Set up ADC1 - Transition Level
  ADMUX |= (1<<MUX0);
  DIDR0 = (1<<ADC1D);
  uint16_t transition_lv = ADCW;

  if (bay_lv == transition_lv){
      // Open the bay lock if bay level = transition level
      PORTD |= (1<<PORTD2);
      break;
    }
  }

  // If the transition is occupied, jump out of the loop and close the lock
  while(true){
    if(PIND & (1<<PIND6)){
      // Close the bay lock
      PORTD &= ~(1<<PORTD2);
      break;
    }else{
      continue;
    }
  }

  // Check if the bay lock is closed. If closed, open the ocean valve
  if(!(PIND & (1<<PIND2))){
    // Open Ocean Valve
    PORTD |= (1<<PORTD1);
  }

  while(true) {
  // Set up ADC1 - Transition Level
  ADMUX |= (1<<MUX0);
  DIDR0 = (1<<ADC1D);
  uint16_t transition_lv = ADCW;

  // Set up ADC2 - Ocean Level
  ADMUX |= (1<<MUX1);
  DIDR0 = (1<<ADC2D);
  uint16_t ocean_lv = ADCW;

  if (ocean_lv == transition_lv){
      // Open the ocean lock if ocean level = transition level
      PORTD |= (1<<PORTD3);
      break;
    }
  }

  // If the transition is not occupied anymore, jump out of the loop and close the lock
  while(true){
    if(!(PIND & (1<<PIND6))){
      // Close the ocean lock
      PORTD &= ~(1<<PORTD3);
      break;
    }else{
      continue;
    }
  }

  return 1;

}

uint8_t beginTransitOceanToBay(){
  // Either lock is open or a boat is already in transition
  if ((PIND & (1<<PIND4)) || (PIND & (1<<PIND5)) || (PIND & (1<<PIND6))) {
    return 0; 
  }

  // Open the Ocean Lock Valve
  PORTD |= (1<<PORTD1);

  while(true) {
  // Set up ADC2 - Ocean Level
  ADMUX |= (1<<MUX1);
  DIDR0 = (1<<ADC2D);
  uint16_t ocean_lv = ADCW;

  // Set up ADC1 - Transition Level
  ADMUX |= (1<<MUX0);
  DIDR0 = (1<<ADC1D);
  uint16_t transition_lv = ADCW;

  if (ocean_lv == transition_lv){
      // Open the ocean lock if ocean level = transition level
      PORTD |= (1<<PORTD3);
      break;
    }
  }

  // If the transition is occupied, jump out of the loop and close the lock
  while(true){
    if(PIND & (1<<PIND6)){
      // Close the ocean lock
      PORTD &= ~(1<<PORTD3);
      break;
    }else{
      continue;
    }
  }

  // Check if the ocean lock is closed. If closed, open the bay valve
  if(!(PIND & (1<<PIND5))){
    // Open Bay Valve
    PORTD |= (1<<PORTD0);
  }

  while(true) {
  // Set up ADC1 - Transition Level
  ADMUX |= (1<<MUX0);
  DIDR0 = (1<<ADC1D);
  uint16_t transition_lv = ADCW;

  // Set up ADC0 - Bay Level
  DIDR0 = (1<<ADC0D);
  uint16_t bay_lv = ADCW;

  if (bay_lv == transition_lv){
      // Open the bay lock if bay level = transition level
      PORTD |= (1<<PORTD2);
      break;
    }
  }

  // If the transition is not occupied anymore, jump out of the loop and close the lock
  while(true){
    if(!(PIND & (1<<PIND6))){
      // Close the Bay lock
      PORTD &= ~(1<<PORTD2);
      break;
    }else{
      continue;
    }
  }

  return 1;

}

void loop() {
  beginTransitBayToOcean();
  beginTransitBayToOcean();
}