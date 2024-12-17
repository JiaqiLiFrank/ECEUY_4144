/* Introduction to Embedded System and Design, 
 * Fall 2024 Final Project: Embedded Sentry
 * Group Members: Ariel Wang， Frank Li, Owen Wang

 * This project is supposed to simulate a security system that records users' gestures as passwords. 
 * We read the data from the accelerometer integrated in the Adafruit Circuit Playground Classic board, 
 * calculate to store them into Single Magnitude Vector (SMV) arrays, 
 * and compare the SMV arrays to verify the user's identity using the correlation formula. 
 * If the passqword is correct, then the NeoPixel strip will light up in green, 
 * otherwise it will light up in red. 
*/


#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>

// SPI CS pin based on the schematic
#define CS_PIN 8

// Control regs based on the LISH3DH datasheet
#define CTRL_REG1   0x20
#define OUT_X_L     0x28
#define OUT_X_H     0x29
#define OUT_Y_L     0x2A
#define OUT_Y_H     0x2B
#define OUT_Z_L     0x2C
#define OUT_Z_H     0x2D

#define KEY_SIZE 500            // Key size for the SMV

#define PIN 17                  // Data pin connected to the NeoPixel
#define NUMPIXELS 10            // Number of NeoPixels

#define RECORDING_TIME 3000     // 3 seconds for recording

// Fucntion prototypes
void PIXEL_Init();
void pixelFullGreen();
void pixelFullRed();
void TIMER_Init();
void pixelRecording(int currentPixel);
void SPI_Init();
void ACC_Init();
uint8_t ACC_Read(uint8_t reg);
void readAccelerometer(int16_t &x, int16_t &y, int16_t &z);
void recordKey(uint16_t SMV[]);
void timingReset();
bool verifySMV(uint16_t SMV1[], uint16_t SMV2[]);
void loop();

// Global variables
Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);  // Setting up the NeoPixel strip
bool recordingDone = false;                                     // Flag to check if recording is done
uint16_t rightSMV[KEY_SIZE], checkSMV[KEY_SIZE];                // SMV arrays for right and check keys

// Initialize the NeoPixel strip
void PIXEL_Init()
{
    strip.begin();
    strip.setBrightness(100);                   // Set brightness to approximately 40%
    for (int i = 0; i < 2; i++)
    {
        strip.fill(strip.Color(255, 255, 255)); // Set to white color to show the strip is working
        strip.show();
        delay(500);
        strip.clear();
        strip.show();
        delay(500);
    }
}

// Set the NeoPixel strip to full green color
void pixelFullGreen()
{
    strip.fill(strip.Color(0, 255, 0)); // Set green color
    strip.show();
}

// Set the NeoPixel strip to full red color
void pixelFullRed()
{
    strip.fill(strip.Color(255, 0, 0)); // Set red color
    strip.show();
}

// Initialize the timer for recording
void TIMER_Init()
{
    DDRC |= (1 << PC7);
    TCCR0A = (1 << WGM01);                         // CTC mode
    TCCR0B = (1 << CS02) | (1 << CS00);            // Prescaler 1024
    OCR0A = ((RECORDING_TIME / KEY_SIZE) / 0.128); // Compare match every 10ms

    TIMSK0 |= (1 << OCIE0A);                       // Enable compare match interrupt
    sei();                                         // Enable global interrupts
}


uint16_t compare_count = 0; // Counter for the compare match interrupt
// ISR for the timer interrupt
ISR(TIMER0_COMPA_vect)
{
    compare_count++;

    uint16_t compare_top = OCR0A;
    uint16_t compare_counter_limit = RECORDING_TIME / (compare_top * 0.128);    // Calculate the compare counter limit

    // Check if the compare counter limit is reached
    if (compare_count >= compare_counter_limit)
    {
        recordingDone = true;
    }
}

/*
 * Function to show the pixel recording
 * @param currentPixel: The current pixel to show
*/
void pixelRecording(int currentPixel)
{
    strip.setPixelColor(currentPixel, strip.Color(currentPixel * 25, currentPixel * 20, 255));
    strip.show();
    if (recordingDone)
    {
        strip.fill(strip.Color(0, 0, 0)); // Set black color (off)
        strip.show();
    }
}

// Initialize the SPI communication
void SPI_Init()
{
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);          // Ensure CS is high (inactive) initially
    SPI.begin();
    SPI.setDataMode(SPI_MODE0);          // LIS3DH operates in SPI Mode 0
    SPI.setClockDivider(SPI_CLOCK_DIV4); // Adjust clock as needed
    SPI.setBitOrder(MSBFIRST);           // LIS3DH uses MSB first
}

// Initialize the accelerometer
void ACC_Init()
{
    digitalWrite(CS_PIN, LOW);          // Enable SPI communication
    SPI.transfer(CTRL_REG1 & 0x7F);     // Write command (clear MSB)
    SPI.transfer(0x97 | CTRL_REG1);     // 0x97 = 10010111: X, Y, Z enable, 1.344 kHz, normal mode
    digitalWrite(CS_PIN, HIGH);         // Disable SPI communication
}

/*
 * Function to read the accelerometer registers
 * @param reg: The register to read
 * @return: The value read from the register
*/
uint8_t ACC_Read(uint8_t reg)
{
    digitalWrite(CS_PIN, LOW);          // Enable SPI communication
    SPI.transfer(0x80 | reg);           // Read command (set MSB)
    uint8_t value = SPI.transfer(0x00); // Dummy byte to clock data in
    digitalWrite(CS_PIN, HIGH);         // Disable SPI communication
    return value;
}

/*
 * Function to read the accelerometer values
 * @param x: The x-axis value
 * @param y: The y-axis value
 * @param z: The z-axis value
*/
void readAccelerometer(int16_t &x, int16_t &y, int16_t &z)
{
    uint8_t xl = ACC_Read(OUT_X_L);
    uint8_t xh = ACC_Read(OUT_X_H);
    x = (int16_t)((xh << 8) | xl);

    uint8_t yl = ACC_Read(OUT_Y_L);
    uint8_t yh = ACC_Read(OUT_Y_H);
    y = (int16_t)((yh << 8) | yl);

    uint8_t zl = ACC_Read(OUT_Z_L);
    uint8_t zh = ACC_Read(OUT_Z_H);
    z = (int16_t)((zh << 8) | zl);
}

// Setup function
void setup()
{
    Serial.begin(115200);
    SPI_Init();
    ACC_Init();
    PIXEL_Init();
    TIMER_Init();
    DDRD &= ~(1 << PD4);    // Left Button - Record Right Key
    DDRF &= ~(1 << PF6);    // Right Button - Record Key to Check
}

/*
 * Function to record the key
 * @param SMV: The SMV array to store the key
*/
void recordKey(uint16_t SMV[])
{
    int16_t x, y, z;
    strip.clear();
    strip.show();

    // Setting up the moving average values for the accelerometer
    const uint8_t BUFFER_SIZE = 20;
    uint32_t sum = 0;
    uint16_t tempValues[BUFFER_SIZE];
    uint8_t index = 0;
    uint8_t count = 0;

    for (uint8_t i = 0; i < BUFFER_SIZE; i++)
    {
        tempValues[i] = 0;
    }

    /*
     * Record the key by calculating the SMV values
     * The SMV values are calculated by taking the moving average of the accelerometer values
    */
    while (compare_count < KEY_SIZE - 1)
    {
        uint8_t currentPixel = (compare_count * NUMPIXELS) / KEY_SIZE;
        pixelRecording(currentPixel);

        readAccelerometer(x, y, z);
        uint16_t rawValue = (uint16_t)sqrtf((float)x * x + (float)y * y + (float)z * z);

        // Moving to the next value
        sum -= tempValues[index];
        tempValues[index] = rawValue;
        sum += rawValue;

        index = (index + 1) % BUFFER_SIZE;
        if (count < BUFFER_SIZE)
        {
            count++;
        }

        uint16_t smoothedValue;
        smoothedValue = sum / BUFFER_SIZE;

        // Print out the SMV value for debugging
        SMV[compare_count] = smoothedValue;
        Serial.print(">SMV: ");
        Serial.println(SMV[compare_count]);
    }

    strip.clear();
    strip.show();
}

/*
 * Function to reset the timing for recording
*/
void timingReset()
{
    TCNT0 = 0;             // Reset hardware timer counter
    compare_count = 0;     // Reset software counter
    recordingDone = false; // Ensure start fresh
}

/*
 * Function to verify the SMV values by calculating the correlation between two arrays of data.
 * @param SMV1: The SMV array to verify
 * @param SMV2: The SMV array to compare with
 * @return: True if the SMV values are verified, false otherwise
 * Correlation formula = (Σ((x - mean(x)) * (y - mean(y))) / sqrt(Σ(x - mean(x))^2 * Σ(y - mean(y))^2)
*/
bool verifySMV(uint16_t SMV1[], uint16_t SMV2[])
{
    float mean1 = 0, mean2 = 0;
    for (int i = 0; i < KEY_SIZE; i++)
    {
        mean1 += SMV1[i];
        mean2 += SMV2[i];
    }
    mean1 /= KEY_SIZE;
    mean2 /= KEY_SIZE;

    float numerator = 0;
    float denominator1 = 0;
    float denominator2 = 0;

    for (int i = 0; i < KEY_SIZE; i++)
    {
        float diff1 = SMV1[i] - mean1;
        float diff2 = SMV2[i] - mean2;
        numerator += diff1 * diff2;
        denominator1 += diff1 * diff1;
        denominator2 += diff2 * diff2;
    }

    float correlation = numerator / sqrt(denominator1 * denominator2);
    Serial.print("> Correlation: ");
    Serial.println(correlation);

    // Set a threshold for correlation
    if (correlation > 0.80)
    {
        pixelFullGreen();
        return true;
    }
    else
    {
        pixelFullRed();
        return false;
    }
}

// Main loop
void loop()
{
    if (PIND & (1 << 4))
    {
        sei();
        delay(500);             // Debounce delay
        timingReset();          // Reset the timing for recording
        recordKey(rightSMV);    // Record the right key
    }
    if (PINF & (1 << 6))
    {
        sei();
        delay(500);             // Debounce delay
        timingReset();          // Reset the timing for recording
        recordKey(checkSMV);    // Record the check key
        cli();
        verifySMV(rightSMV, checkSMV);
    }
}