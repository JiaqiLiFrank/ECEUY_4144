#include <Adafruit_NeoPixel.h>

#define PIN 17         // Data pin connected to the NeoPixel
#define NUMPIXELS 10  // Number of NeoPixels

Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void setup() {
    strip.begin();
    strip.setBrightness(5); // Set brightness to 50%
    strip.fill(strip.Color(255, 0, 0)); // Fill with red color
    strip.show();
}

bool done = false;
void pixeltest(){
    while(!done){
        for (int i = 0; i < strip.numPixels(); i++) {
            strip.setPixelColor(i, strip.Color(0, 255, 0)); // Set green
            strip.show();
            delay(100);
        }
        done = true;
        strip.clear();
        strip.show();
    }
}


void loop() {
pixeltest();
}
