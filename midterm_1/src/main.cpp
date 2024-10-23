#include <Arduino.h>

void setup() {
  // Set up GPIO
  DDRD |= (1<<0); //  Bay Lock Valve    - PD0 - Output
  DDRD |= (1<<1); //  Ocean Lock Valve  - PD1 - Output
  DDRD |= (1<<2); //  Bay Lock          - PD2 - Output
  DDRD |= (1<<3); //  Ocean Lock        - PD3 - Output
  DDRD &= ~(1<<4); // Bay Lock Status   - PD4 - Input
  DDRD &= ~(1<<5); // Ocean Lock Status - PD5 - Input
  DDRD &= ~(1<<6); // Transition Status - PD6 - Input

  PORTD &= ~(1<<PORTD0); // Bay Lock Valve
  PORTD &= ~(1<<PORTD1); // Ocean Lock Valve
  PORTD &= ~(1<<PORTD2); // Bay Lock
  PORTD &= ~(1<<PORTD3); // Ocean Lock

  // Set up the ADC
  ADMUX = (1<<REFS0); // Setting the Vcc as the reference
  ADCSRA = (1<<ADEN) | (1<<ADATE) | (1<<ADPS1) | (1<<ADPS0); // ADC Enable; Auto Trigger Enable; Prescaler 8
  // ADCSRB at Free Running Mode, thus 0000
}

/*
  Continusly checking if the levels of the bay and the transition waterway are the same.
  OUTPUT: 1 - same; 0 - not same.
*/
bool checkBayTransition(){
  while(true){
    // Set up ADC0 - Bay Level
    DIDR0 = (1<<ADC0D);
    uint16_t bay_lv = ADCW; // Bay water level

    // Set up ADC1 - Transition Level
    ADMUX |= (1<<MUX0);
    DIDR0 = (1<<ADC1D);
    uint16_t transition_lv = ADCW; // Transition waterway water lavel

    // Check if bay water level equals to transition waterway level. If same, return true; else, continue the loop to check. 
    if (bay_lv == transition_lv){
      return true;
    }else{
      continue;
    }
  }

  return false;
}

/*
  Continusly checking if the levels of the the transition waterway and ocean are the same.
  OUTPUT: 1 - same; 0 - not same.
*/
bool checkTransitionOcean(){
  while(true) {
  // Set up ADC1 - Transition Level
  ADMUX |= (1<<MUX0);
  DIDR0 = (1<<ADC1D);
  uint16_t transition_lv = ADCW; // Transition waterway water lavel

  // Set up ADC2 - Ocean Level
  ADMUX |= (1<<MUX1);
  DIDR0 = (1<<ADC2D);
  uint16_t ocean_lv = ADCW; // Ocean water lavel

  // Check if ocean water level equals to transition waterway level. If same, return true; else, continue the loop to check. 
  if (ocean_lv == transition_lv){
      return true;
    }else{
      continue;
    }
  }

  return false;
}

/*
  Continusly checking if the transition waterway is occupied or not.
  OUTPUT: 1 - occupied; 0 - not occupied. 
*/
bool isTransitionOccupied(){
  // If the transition is occupied, return true; else, continue the loop to check. 
  while(true){
    if(PIND & (1<<PIND6)){
      return true;
    }else{
      continue;
    }
  }
  return false;
}


/*
  Moving water from the Bay area, through the transition waterway, and finally reach the ocean area. 
*/
uint8_t beginTransitBayToOcean(){
  // Either lock is open or a boat is already in transition
  if ((PIND & (1<<PIND4)) || (PIND & (1<<PIND5)) || (PIND & (1<<PIND6))) {
    return 0; 
  }

  PORTD |= (1<<PORTD0); // Open the bay lock valve

  if(checkBayTransition){
    PORTD |= (1<<PORTD2); // Open the Bay lock
  }

  if(isTransitionOccupied){
    PORTD &= ~(1<<PORTD2); // Close the bay lock
  }

  // Check if the bay lock is closed. If closed, open the ocean valve
  if(!(PIND & (1<<PIND2))){
    PORTD |= (1<<PORTD1); // Open ocean valve
  }

  if(checkTransitionOcean){
    PORTD |= (1<<PORTD3); // Open the ocean lock
  }

  if(!isTransitionOccupied){
    PORTD &= ~(1<<PORTD3); // Close the ocean lock
  }

  return 1;

}

uint8_t beginTransitOceanToBay(){
  // Either lock is open or a boat is already in transition
  if ((PIND & (1<<PIND4)) || (PIND & (1<<PIND5)) || (PIND & (1<<PIND6))) {
    return 0; 
  }

  PORTD |= (1<<PORTD1); // Open the Ocean Lock valve

  if(checkTransitionOcean){
    PORTD |= (1<<PORTD3); // Open the ocean lock
  }

  if(isTransitionOccupied){
    PORTD &= ~(1<<PORTD3); // Close the ocean lock
  }

  // Check if the ocean lock is closed. If closed, open the bay valve
  if(!(PIND & (1<<PIND5))){
    PORTD |= (1<<PORTD0); // Open Bay valve
  }

  if(checkBayTransition){
    PORTD |= (1<<PORTD2); // Open Bay lock
  }

  if(!isTransitionOccupied){
    PORTD &= ~(1<<PORTD2); // Close Bay lock
  }

  return 1;

}

void loop() {
  beginTransitBayToOcean();
  beginTransitBayToOcean();
}