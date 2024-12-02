#include <Arduino.h>
extern "C" uint16_t addAssem(uint16_t a, uint16_t b); // declare the assembly function
extern "C" uint8_t findLetter(char *str, char letter); // declare the assembly function

void setup() {
  uint16_t addResult = addAssem(5,8); // call the assembly function for test
  Serial.begin(9600);
  while(!Serial); // wait for Serial Monitor
  Serial.println(addResult);

  char str[] = "Hello World!";
  char letter = 'x';
  uint8_t letterIndex = findLetter(str, letter);
  Serial.println(letterIndex);
}

void loop() {
  // Expected output: 13
  // What if reach an overflow? It will automatically neglect the overflowed bits (in binary)
}
