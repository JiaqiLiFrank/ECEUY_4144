#include <Arduino.h>
void setup(){
    Serial.begin(115200);
}

void loop(){
    int16_t x = 30000;
    uint16_t y = x;
    Serial.println(y);
    delay(1000);
}