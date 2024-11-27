#include <Arduino.h>
#include <SPI.h>

// Define SPI pins (PlatformIO default uses hardware SPI pins)
#define CS_PIN 8  // Chip Select (adjust as per your wiring)

// LIS3DH Register Definitions
#define CTRL_REG1 0x20
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D

#define KEY_SIZE 280

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

void readAccelerometer(int16_t &x, int16_t &y, int16_t &z) {
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


void setup() {
    Serial.begin(115200);
    SPI_Init();
    ACC_Init();
    DDRD &= ~(1<<PD4); // Left Button - Record Right Key
    DDRF &= ~(1<<PF6); // Right Button - Record Key to Check
}

float rightSMV[KEY_SIZE], checkSMV[KEY_SIZE];

void recordKey(float SMV[]){
    int16_t x, y, z;
    Serial.println("Recording Start");
    // unsigned long startTime = millis();
    for(int i = 0; i < KEY_SIZE; i ++){
        readAccelerometer(x, y, z);
        SMV[i] = sqrtf((float)x * x + (float)y * y + (float)z * z);
        delay(10);
        Serial.println(SMV[i]);
        }
    Serial.println("Recording Finish");
}

bool verifySMV(float SMV1[], float SMV2[]) {
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
    Serial.print("Correlation: ");
    Serial.println(correlation);

    // Set a threshold for correlation (e.g., 0.8 for 80% similarity)
    if (correlation > 0.75) {
        return true;
    } else {
        return false;
    }
}

void loop() {
    if(PIND & (1<<4)){
        recordKey(rightSMV);
    }
    if(PINF & (1<<6)){
        recordKey(checkSMV);
        if(verifySMV(rightSMV, checkSMV)){
            Serial.println("SUCCESS");
        }else{
            Serial.println("FAIL");
        }
        Serial.println("-------------------");
    }
}