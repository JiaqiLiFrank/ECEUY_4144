#include <Arduino.h>
#include <Wire.h>

/* 
 * Address for the sensor to communicate using I2C
 * Address is 0x57 since the microcontroller is using 7-bit addressing, thus a right shift of 1 is needed
 * Source: https://github.com/oxullo/Arduino-MAX30100/issues/10
 */
#define MAX30100_ADDRESS       0x57

// Register addresses for the MAX30100
#define INTERRUPT_STATUS       0x00
#define INTERRUPT_ENABLE       0x01
#define FIFO_DATA              0x05
#define MODE_CONFIG            0x06
#define SPO2_CONFIG            0x07

// Interrupt status flag for SPO2 data ready
#define SPO2_RDY (1<<4)

// Mode configuration for SPO2
// MODE[2:0] 011 -> 0x03
#define MODE_SPO2 0x03

/*
 * Write a value to a register
 * @param reg: Register to write to
 * @param value: Value to write to the register
*/
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MAX30100_ADDRESS);
  Wire.write(reg);          // Register to write to 
  Wire.write(value);        // Value to write to the register
  Wire.endTransmission();   // End the transmission
}

/*
 * Read a value from a register
 * @param reg: Register to read from
 * @param *tempData: Pointer to store the value read from the register
 * @param length: Number of bytes to read from the register
*/
void readFIFO(uint8_t fifoReg, uint8_t *tempData) {
  Wire.beginTransmission(MAX30100_ADDRESS);
  Wire.write(fifoReg);    // FIFO register
  Wire.endTransmission();
  Wire.requestFrom(MAX30100_ADDRESS, 4);

  // read() can only go byte by byte, loop to read all bytes
  for (int i = 0; i < 4; i++) {
    tempData[i] = Wire.read();
  }
}

/*
 * Get the IR and RED values from the FIFO. Get two 8-bit values and combine them to get a 16-bit value.
 * @param ir: Pointer to store the IR value
 * @param red: Pointer to store the RED value
*/
void GetData(uint16_t &ir, uint16_t &red) {
  uint8_t data[4] = {0};
  readFIFO(FIFO_DATA, data);

  ir = (data[0] << 8) | data[1]; // Higher byte first
  red = (data[2] << 8) | data[3];
}

// Initialize the MAX30100
void MAX30100_Init(){
  writeRegister(INTERRUPT_ENABLE, SPO2_RDY);  // Enable interrupt for SPO2_RDY
  writeRegister(MODE_CONFIG, MODE_SPO2);      // Set the mode to SPO2
  writeRegister(INTERRUPT_STATUS, 0x00);      // Clear interrupt status
}

//Initialize the LED on the board, port PC7
void LED_Init(){
  DDRC |= (1 << PC7);
  PORTC &= ~(1 << PC7);
}

//Blink the LED on the board
void ledBlink() {
  PORTC |= (1<<PC7); // Turn on the LED
  delay(200);
  PORTC &= ~(1<<PC7); // Turn off the LED
}

// Initialize INT2 external interrupt
void INT2_Init(){
  DDRD &= ~(1 << PD2);                    // Set INT2 as input
  EICRA |= (1 << ISC21) | (1 << ISC20);   // Rising edge of INT2
  EIMSK |= (1 << INT2);                   // Enable INT2
}

void printLED(uint16_t ir, uint16_t red){
  // Print out the IR and RED values
  Serial.print("IR: ");
  Serial.print(ir);
  Serial.print(", RED: ");
  Serial.println(red);
}

/*
 * ISR handler for INT2
 * ISR is triggered when the MAX30100 has data ready
 * Reads data using the FIFO helper function and print
*/
ISR(INT2_vect) {
  uint16_t ir, red;
  GetData(ir, red);

  ledBlink(); // Blink LED to show receiving data
  writeRegister(INTERRUPT_STATUS, 0x00); // Clear interrupt status
  printLED(ir, red); // Print out the result
}

void setup() {
  Wire.begin();         // Begin I2C communication
  LED_Init();           // Initialize the on-board LED
  INT2_Init();          // Initialize INT2
  Serial.begin(115200); // Begin Serial communication
  MAX30100_Init();      // Initialize the MAX30100
}

void loop() {
  // Nothing Needed Here
}
