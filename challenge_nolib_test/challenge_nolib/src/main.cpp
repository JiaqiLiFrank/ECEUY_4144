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

#define KEY_SIZE 200

#define PIN 17         // Data pin connected to the NeoPixel
#define NUMPIXELS 10  // Number of NeoPixels

#define RECORDING_TIME 3000

// Prototypes
void PIXEL_Init();
void pixelReady();
void pixelFullGreen();
void pixelFullRed();
void TIMER_Init();
void pixelRecording(int currentPixel);
void SPI_Init();
void ACC_Init();
uint8_t ACC_Read(uint8_t reg);
void readAccelerometer(int16_t &x, int16_t &y, int16_t &z);
void recordKey(uint16_t SMV[], int addr);
bool verifySMV(uint16_t SMV1[], uint16_t SMV2[]);
void pixelRecording(int currentPixel);


Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
bool recordingDone = false;
uint16_t rightSMV[KEY_SIZE], checkSMV[KEY_SIZE];


void PIXEL_Init(){
    strip.begin();
    strip.setBrightness(5); // Set brightness to 50%
    for(int i = 0; i < 2; i++){
        strip.fill(strip.Color(255, 255, 255)); // Set white color
        strip.show();
        delay(500);
        strip.clear();
        strip.show();
        delay(500);
    }
}

void pixelFullGreen(){
    strip.fill(strip.Color(0, 255, 0)); // Set green color
    strip.show();
}

void pixelFullRed(){
    strip.fill(strip.Color(255, 0, 0)); // Set red color
    strip.show();
}

void TIMER_Init(){
    DDRC |= (1 << PC7);
    TCCR0A = (1 << WGM01);                                  // CTC mode
    TCCR0B = (1 << CS02) | (1 << CS00);                     // Prescaler 1024
    OCR0A = ((RECORDING_TIME / KEY_SIZE)/0.128);            // Compare match every 10ms

    TIMSK0 |= (1 << OCIE0A);
    sei();
}

uint16_t compare_count = 0;
ISR(TIMER0_COMPA_vect) {
    compare_count++;

    uint16_t compare_top = OCR0A;
    uint16_t compare_counter_limit = RECORDING_TIME / (compare_top * 0.128);

    if (compare_count >= compare_counter_limit) {
        recordingDone = true;
    }
}

void pixelRecording(int currentPixel){
    strip.setPixelColor(currentPixel, strip.Color(currentPixel*25,currentPixel*20,255));
    strip.show();
    if(recordingDone){
        strip.fill(strip.Color(0, 0, 0)); // Set black color
        strip.show();
    }
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
    SPI.transfer(0x97 | CTRL_REG1);         // 0x77 = 01110111: X, Y, Z enable, 100Hz
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
    // Serial.begin(115200);
    SPI_Init();
    ACC_Init();
    PIXEL_Init();
    TIMER_Init();
    DDRD &= ~(1<<PD4); // Left Button - Record Right Key
    DDRF &= ~(1<<PF6); // Right Button - Record Key to Check
}

void recordKey(uint16_t SMV[]) {
    int16_t x, y, z;
    strip.clear();
    strip.show();
    // Capture KEY_SIZE samples as timer increments
    while (compare_count < KEY_SIZE-1) {
        uint8_t currentPixel = (compare_count * NUMPIXELS) / KEY_SIZE;
        pixelRecording(currentPixel);
        readAccelerometer(x, y, z);
        SMV[compare_count] = (uint16_t)sqrtf((float)x * x + (float)y * y + (float)z * z);
        Serial.print(">SMV: ");
        Serial.println(SMV[compare_count]);
    }
    strip.clear();
    strip.show();
}

void timingReset(){
    // Reset the timer and related counters
    TCNT0 = 0;            // Reset hardware timer counter
    compare_count = 0;    // Reset software counter
    recordingDone = false;// Ensure we start fresh
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
    Serial.print("Correlation: ");
    Serial.println(correlation);

    // Set a threshold for correlation
    if (correlation > 0.75) {
        pixelFullGreen();
        return true;
    } else {
        pixelFullRed();
        return false;
    }
}

void loop() {
    if(PIND & (1<<4)){
        timingReset();
        recordKey(rightSMV);
    }
    if(PINF & (1<<6)){
        timingReset();
        recordKey(checkSMV);
        cli();
        verifySMV(rightSMV, checkSMV);
    }
}