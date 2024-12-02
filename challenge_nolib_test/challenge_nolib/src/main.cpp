#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>

// Define SPI pins (PlatformIO default uses hardware SPI pins)
#define CS_PIN 8  // Chip Select

// LIS3DH Register Definitions
#define CTRL_REG1 0x20
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D

#define KEY_SIZE 500
#define BUFFER_SIZE 10
uint16_t buffer[BUFFER_SIZE] = {0}; // Buffer to store the values
uint16_t currentAvg;                // Current average value

#define PIN 17         // Data pin connected to the NeoPixel
#define NUMPIXELS 10  // Number of NeoPixels

Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
void PIXEL_Init(){
    strip.begin();
    strip.setBrightness(5); // Set brightness to 50%
    strip.show();
}

bool done = false;
void pixelReady(){
    while(!done){
        for(int i = 0; i < 2; i ++){
            strip.fill(strip.Color(255,255,255)); // Set white
            strip.show();
            delay(500);
            strip.clear();
            strip.show();
            delay(200);
        }
        done = true;
    }
    strip.clear();
    strip.show();
}

void SPI_Init() {
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);             // Ensure CS is high (inactive) initially
    SPI.begin();
    SPI.setDataMode(SPI_MODE0);             // LIS3DH operates in SPI Mode 0
    SPI.setClockDivider(SPI_CLOCK_DIV4);    // Adjust clock as needed
    SPI.setBitOrder(MSBFIRST);              // LIS3DH uses MSB first
}

void ACC_Init() {
    digitalWrite(CS_PIN, LOW);              // Enable SPI communication
    SPI.transfer(CTRL_REG1 & 0x7F);         // Write command (clear MSB)
    SPI.transfer(0x77 | CTRL_REG1);         // 0x77 = 01110111: X, Y, Z enable, 100Hz
    digitalWrite(CS_PIN, HIGH);             // Disable SPI communication
}

uint8_t ACC_Read(uint8_t reg) {
    digitalWrite(CS_PIN, LOW);              // Enable SPI communication
    SPI.transfer(0x80 | reg);               // Read command (set MSB)
    uint8_t value = SPI.transfer(0x00);     // Dummy byte to clock data in
    digitalWrite(CS_PIN, HIGH);             // Disable SPI communication
    return value;
}

void readAccelerometer(uint16_t &x, uint16_t &y, uint16_t &z) {
    uint8_t xl = ACC_Read(OUT_X_L);
    uint8_t xh = ACC_Read(OUT_X_H);
    x = (uint16_t)((xh << 8) | xl);

    uint8_t yl = ACC_Read(OUT_Y_L);
    uint8_t yh = ACC_Read(OUT_Y_H);
    y = (uint16_t)((yh << 8) | yl);

    uint8_t zl = ACC_Read(OUT_Z_L);
    uint8_t zh = ACC_Read(OUT_Z_H);
    z = (uint16_t)((zh << 8) | zl);
    
}


void setup() {
    // Serial.begin(115200);
    SPI_Init();
    ACC_Init();
    PIXEL_Init();
    DDRD &= ~(1<<PD4); // Left Button - Record Right Key
    DDRF &= ~(1<<PF6); // Right Button - Record Key to Check
}


uint16_t rightSMV[KEY_SIZE], checkSMV[KEY_SIZE];

void recordKey(uint16_t SMV[]) {
    uint16_t x, y, z;
    int count = 0;

    for (int i = 0; i < KEY_SIZE; i++) {

        for(int j = 0; j < BUFFER_SIZE - 1; j++){
            buffer[i] = buffer[i + 1]; // Shift the buffer to the left
            currentAvg += buffer[i];   // Sum up the values
        }
        // Read accelerometer data
        readAccelerometer(x, y, z);
        buffer[BUFFER_SIZE - 1] = (uint16_t)(sqrtf((float)x * x + 
                                                   (float)y * y + 
                                                   (float)z * z)); // Add the new value to the buffer
        currentAvg += buffer[BUFFER_SIZE - 1];    // Sum up the values
        currentAvg /= BUFFER_SIZE;                // Calculate the average value

        SMV[i] = currentAvg;

        float progress = (float)(i + 1) / KEY_SIZE;
        int numPixelsToLight = progress * strip.numPixels();
        strip.clear(); 
        for (int j = 0; j < numPixelsToLight; j++) {
            strip.setPixelColor(j, strip.Color(0, 0, 255)); // Set blue color
        }
        strip.show();

        delay(5);
        Serial.print(">SMV: ");
        Serial.println(SMV[i]);
    }
}

void pixelFullGreen(){
    strip.fill(strip.Color(0, 255, 0)); // Set green color
    strip.show();
    delay(1000);
    strip.clear();
    strip.show();
}

void pixelFullRed(){
    strip.fill(strip.Color(255, 0, 0)); // Set red color
    strip.show();
    delay(1000);
    strip.clear();
    strip.show();
}


bool verifySMV(uint16_t SMV1[], uint16_t SMV2[]) {
    float mean1 = 0, mean2 = 0;
    for (int i = 0; i < KEY_SIZE; i++) {
        mean1 += SMV1[i];
        mean2 += SMV2[i];
    }
    mean1 /= KEY_SIZE;
    mean2 /= KEY_SIZE;

    float numerator = 0;
    float denominator1 = 0;
    float denominator2 = 0;

    for (int i = 0; i < KEY_SIZE; i++) {
        float diff1 = SMV1[i] - mean1;
        float diff2 = SMV2[i] - mean2;
        numerator += diff1 * diff2;
        denominator1 += diff1 * diff1;
        denominator2 += diff2 * diff2;
    }

    float correlation = numerator / sqrt(denominator1 * denominator2);
    // Serial.print("Correlation: ");
    // Serial.println(correlation);

    // Set a threshold for correlation
    if (correlation > 0.75) {
        return true;
    } else {
        return false;
    }
}

void loop() {
    pixelReady();
    if(PIND & (1<<4)){
        recordKey(rightSMV);
        pixelFullGreen();
    }
    if(PINF & (1<<6)){
        recordKey(checkSMV);
        // pixelFullRed();
        if(verifySMV(rightSMV, checkSMV)){
            pixelFullGreen();
        } else {
            pixelFullRed();
        }
    }
}