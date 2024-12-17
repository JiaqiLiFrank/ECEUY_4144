#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

/*
 * Total 3000 milliseconds
 * 8-bit timer - max 256 ticks
 * processor clock 8MHz
 * prescaler 1024
 * period = 1024 / 8MHz = 0.128 ms/tick
 * 
 * ticks per compare match = OCR0A + 1
 * interrupts = 3000ms / (OCR0A + 1) * 0.128ms
*/

uint16_t compare_count = 0;

void setup() {
    // Configure PC7 as output
    DDRC |= (1 << PC7);

    // Configure Timer0
    TCCR0A = (1 << WGM01);          // CTC mode (WGM01=1, WGM00=0)
    TCCR0B = (1 << CS02) | (1 << CS00); // Prescaler 1024 (CS02=1, CS01=0, CS00=1)

    OCR0A = 249;        // Compare match every 32 ms (250 * 0.128 ms)

    // Enable Timer0 compare match interrupt
    TIMSK0 |= (1 << OCIE0A);

    // Enable global interrupts
    sei();
}

bool done = false;
// Timer0 Compare Match Interrupt Service Routine (ISR)
ISR(TIMER0_COMPA_vect) {
    compare_count++;

    // Each compare match occurs at 32 ms
    // 3 seconds = 3000 ms / 32 ms = ~93.75 compare matches
    if (compare_count >= 94) {
        // PORTC ^= (1 << PORTC7); // Toggle LED on PC7
        compare_count = 0;      // Reset compare count
        done = true;
    }
}

Adafruit_NeoPixel strip = Adafruit_NeoPixel(10, 17, NEO_GRB + NEO_KHZ800);
void loop() {
  strip.begin();
  strip.setBrightness(5);
  // strip.fill(strip.Color(255,255,255));
  strip.show();
  while(!done){
    strip.setPixelColor(compare_count/9, strip.Color(255,0,0));
    strip.show();
  }
  strip.clear();
  strip.show();
}
