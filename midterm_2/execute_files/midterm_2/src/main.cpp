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
#define SPO2_RDY (1 << 4)

// Mode configuration for SPO2
// MODE[2:0] 011
#define MODE_SPO2 0x03

// LED values
uint16_t irValue = 0;
uint16_t redValue = 0;

/*
 * Write a value to a register
 * @param reg: Register to write to
 * @param value: Value to write to the register
*/
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MAX30100_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

/*
 * Read a value from a register
 * @param reg: Register to read from
 * @param *tempData: Pointer to store the value read from the register
 * @param length: Number of bytes to read from the register
*/
void readFIFO(uint8_t startReg, uint8_t *tempData, uint8_t length) {
  Wire.beginTransmission(MAX30100_ADDRESS);
  Wire.write(startReg);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX30100_ADDRESS, length);
  for (uint8_t i = 0; i < length; i++) {
    tempData[i] = Wire.read();
  }
}

void GetData(uint16_t &ir, uint16_t &red) {
  uint8_t data[4];
  readFIFO(FIFO_DATA, data, 4);

  ir  = (uint16_t)(data[0] << 8) | data[1];
  red = (uint16_t)(data[2] << 8) | data[3];
}

ISR(INT2_vect) {
  uint16_t ir, red;
  GetData(ir, red);
  irValue = ir;
  redValue = red;

  PORTB ^= (1 << 7);

  writeRegister(INTERRUPT_STATUS, 0x00);
}

void setup() {
  Wire.begin();

  DDRB |= (1 << 7);
  PORTB &= ~(1 << 7);

  Serial.begin(115200);
  while(!Serial);
  Serial.println("Initializing MAX30100...");

  pinMode(2, INPUT);

  EICRB &= ~((1 << ISC21) | (1 << ISC20)); 

  EICRB |=  ((1 << ISC21) | (1 << ISC20)); 
  EIMSK |= (1 << INT2);

  writeRegister(INTERRUPT_ENABLE, SPO2_RDY);

  writeRegister(MODE_CONFIG, MODE_SPO2);

  writeRegister(INTERRUPT_STATUS, 0x00);  // Clear interrupt status

  Serial.println("Setup Complete. Waiting for data...");
}

void loop() {

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.print("IR: ");
    Serial.print(irValue);
    Serial.print(", RED: ");
    Serial.println(redValue);
  }
}
