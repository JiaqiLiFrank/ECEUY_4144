#include <Adafruit_CircuitPlayground.h>

// Frequencies for notes in Hz
#define C 261
#define D 294
#define E 329
#define F 349
#define REST 0

void playMusic() {
  // Define the melody sequence
  int melody[] = {E, D, C, D, E, F,   // 321 23 432
                  E, D, REST, C, D, E, F,   // 321 23 432
                  E, D, REST, C, D, E, F,   // 321 23 432
                  E, D, REST, C, D, E, F,   // 321 23 432
                  C, D, E, C, D, E, F, REST,    // 12332432
                  D, D, C, C, F, D, REST,      // 2211432
                  D, C, D, E, REST};           // 2123
  
  int noteDuration = 300; // 500ms per note

  for (int i = 0; i < sizeof(melody) / sizeof(melody[0]); i++) {
    if (melody[i] == REST) {
      delay(noteDuration); // Rest for the duration of the note
    } else {
      CircuitPlayground.playTone(melody[i], noteDuration);
    }
  }
}

void setup() {
  CircuitPlayground.begin();
  playMusic();
}

void loop() {
  // Do nothing
}
