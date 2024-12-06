#include <Arduino.h>
#include <Wire.h>

/*
 * Address for the sensor to communicate using I2C
 * Address is 0x57 since the microcontroller is using 7-bit addressing, thus a right shift of 1 is needed
 * Source: https://github.com/oxullo/Arduino-MAX30100/issues/10
 */
#define MAX30100_ADDRESS 0x57

// Register addresses for the MAX30100
#define INTERRUPT_STATUS 0x00
#define INTERRUPT_ENABLE 0x01
#define FIFO_WR_PTR 0x02
#define FIFO_RD_PTR 0x04
#define FIFO_DATA 0x05
#define MODE_CONFIG 0x06
#define SPO2_CONFIG 0x07

// Interrupt status flag for SPO2 data ready
#define SPO2_RDY (1 << 4)

// Mode configuration for SPO2
// MODE[2:0] 011 -> 0x03
#define MODE_SPO2 0x03

// Function prototypes
void writeRegister(uint8_t reg, uint8_t value);
uint8_t readRegister(uint8_t reg);
void readFIFO(uint8_t fifoReg, uint8_t *tempData);
void GetData(uint16_t &ir, uint16_t &red);
void MAX30100_Init();
void INT2_Init();
void LED_Init();
void ledBlink();

void setup()
{
  Wire.begin();         // Begin I2C communication
  LED_Init();           // Initialize the on-board LED
  INT2_Init();          // Initialize INT2
  Serial.begin(115200); // Begin Serial communication
  MAX30100_Init();      // Initialize the MAX30100
}

void loop()
{
  // Nothing Needed Here
}

/*
 * Write a value to a register
 * @param reg: Register to write to
 * @param value: Value to write to the register
 */
void writeRegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(MAX30100_ADDRESS);
  Wire.write(reg);        // Register to write to
  Wire.write(value);      // Value to write to the register
  Wire.endTransmission(); // End the transmission
}

/*
 * Read a value from a register
 * @param reg: Register to read from
 * @return: Value read from the register
 */
uint8_t readRegister(uint8_t reg)
{
  Wire.beginTransmission(MAX30100_ADDRESS);
  Wire.write(reg); // Register to read from
  Wire.endTransmission();
  Wire.requestFrom(MAX30100_ADDRESS, 1);
  return Wire.read();
}

/*
 * Read the FIFO register and store the data into an array buffer
 * @param fifoReg: FIFO register to read from
 * @param tempData: Buffer to store the data
 */
void readFIFO(uint8_t fifoReg, uint8_t *tempData)
{
  // Get FIFO_WR_PTR and FIFO_RD_PTR
  uint8_t writePtr = readRegister(FIFO_WR_PTR);
  uint8_t readPtr = readRegister(FIFO_RD_PTR);

  // Calculate number of available samples, taking wrap-around into account
  uint8_t numAvailableSamples;
  if (writePtr >= readPtr)
  {
    numAvailableSamples = writePtr - readPtr;
  }
  else
  {
    numAvailableSamples = (writePtr + 16) - readPtr;
  }

  // Read samples from FIFO
  for (int i = 0; i < numAvailableSamples; i++)
  {
    Wire.beginTransmission(MAX30100_ADDRESS);
    Wire.write(fifoReg);
    Wire.endTransmission(false); // Send repeated start
    Wire.requestFrom(MAX30100_ADDRESS, 4);

    for (int i = 0; i < 4; i++)
    {
      tempData[i] = Wire.read();
    }
  }
}

/*
 * Get the IR and RED values from the FIFO. Get two 8-bit values and combine them to get a 16-bit value.
 * @param ir: Pointer to store the IR value
 * @param red: Pointer to store the RED value
 */
void GetData(uint16_t &ir, uint16_t &red)
{
  uint8_t data[4] = {0};
  readFIFO(FIFO_DATA, data);

  ir = (data[0] << 8);
  ir |= data[1]; // Higher byte first
  red = (data[2] << 8);
  red |= data[3];
}

// Initialize the MAX30100
void MAX30100_Init()
{
  writeRegister(INTERRUPT_ENABLE, SPO2_RDY); // Enable interrupt for SPO2_RDY
  writeRegister(MODE_CONFIG, MODE_SPO2);     // Set the mode to SPO2
  writeRegister(INTERRUPT_STATUS, 0x00);     // Clear interrupt status
}

// Initialize the LED on the board, port PC7
void LED_Init()
{
  DDRC |= (1 << PC7);
  PORTC &= ~(1 << PC7);
}

// Blink the LED on the board
void ledBlink()
{
  PORTC |= (1 << PC7); // Turn on the LED
  delay(200);
  PORTC &= ~(1 << PC7); // Turn off the LED
}

// Initialize INT2 external interrupt
void INT2_Init()
{
  DDRD &= ~(1 << PD2);                  // Set INT2 as input
  EICRA |= (1 << ISC21) | (1 << ISC20); // Rising edge of INT2
  EIMSK |= (1 << INT2);                 // Enable INT2
}

void printLED(uint16_t ir, uint16_t red)
{
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
ISR(INT2_vect)
{
  uint16_t ir, red;
  GetData(ir, red);

  ledBlink();                            // Blink LED to show receiving data
  writeRegister(INTERRUPT_STATUS, 0x00); // Clear interrupt status
  printLED(ir, red);                     // Print out the result
}