#include <Arduino.h>

// Define the button pin as PF6 (ADC6)
#define BUTTON_PIN PF6 

volatile bool buttonPressed = false; // Flag to detect button press

void setup() {
    // Configure PF6 as an input
    DDRF &= ~(1 << BUTTON_PIN); // Set PF6 as input
    PORTF |= (1 << BUTTON_PIN); // Enable pull-up resistor on PF6

    // Configure external interrupt for PF6
    PCICR |= (1 << PCIE0);       // Enable pin change interrupt for PCINT[7:0]
    PCMSK0 |= (1 << PCINT6);     // Enable interrupt for PF6 (PCINT6)

    // Initialize serial communication
    Serial.begin(9600);

    // Enable global interrupts
    sei();
}

// Pin Change Interrupt Service Routine
ISR(PCINT0_vect) {
    // Check if PF6 is LOW (button pressed)
    if (!(PINF & (1 << BUTTON_PIN))) {
        buttonPressed = true; // Set the flag
    }
}

void loop() {
    if (buttonPressed) {
        // Print message only once
        Serial.println("Button pressed!");

        // Clear the flag to ensure message is printed only once
        buttonPressed = false;

        // Wait for the button to be released (debounce handling)
        while (!(PINF & (1 << BUTTON_PIN))) {
            // Do nothing until the button is released
        }
    }
}
