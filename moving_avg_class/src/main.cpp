#include <Arduino.h>

// Use the moving average method to clean up the messy signal from ADC. 
// Simulating an ECG, which received electro signals from our body. 
// INFO: Digital D9, PB5, ADC12

void setup() {
  pinMode(A9, INPUT_PULLUP); // If no signal, resistor is connected to the voltage input
  Serial.begin(115200);
}

#define BUFFER_SIZE 7               // Set up the moving average contant as 7
uint16_t buffer[BUFFER_SIZE] = {0}; // Buffer to store the values
uint16_t currentAvg;                // Current average value

/*
  * Moving average method
  * 1. Shift the buffer to the left
  * 2. Add the new value to the buffer
  * 3. Calculate the average value
*/
uint16_t moving_average(){
  for(int i = 0; i < BUFFER_SIZE - 1; i++){
    buffer[i] = buffer[i + 1]; // Shift the buffer to the left
    currentAvg += buffer[i];   // Sum up the values
  }
  buffer[BUFFER_SIZE - 1] = analogRead(A9); // Add the new value to the buffer
  currentAvg += buffer[BUFFER_SIZE - 1];    // Sum up the values
  currentAvg /= BUFFER_SIZE;                // Calculate the average value
}

void loop() {
  Serial.println(analogRead(A9));
  delay(500);
}