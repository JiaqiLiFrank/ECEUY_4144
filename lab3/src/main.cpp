#include <Arduino.h>

void setup() {
    UCSR1A = 0x00; // UCSRA set as default. 
    UCSR1B = 0x00; // UCSRB set as default.
    UCSR1C = 0x00; // UCSRC set as default.

    UCSR1B |= (1<<RXEN1) | // Enable receiver
              (1<<TXEN1); // Enable transmitter
    UCSR1C |= (1<<UCSZ10) |
              (1<<UCSZ11); // 8-bit data
    UBRR1 = 51; // UBRR1 = (fosc / (16 * Baud Rate)) - 1
                //       = (8MHz / (16 * 9600)) - 1 = 51.08

    DDRF &= ~(1<<PF6); // Right button PF6 as input
    DDRC |= (1<<PC7); // Set PC7 as output
    PORTC &= ~(1<<PC7); // Set LED to off

}


void loop() {
    // Team A
    if(PINF & (1<<PF6)){
        PORTC |= (1<<PC7); // Set LED to on
        UDR1 = 0x31; // Send 1
    } else {
        PORTC &= ~(1<<PC7); // Set LED to off
        UDR1 = 0x30; // Send 0
    }
}
