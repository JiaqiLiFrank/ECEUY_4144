#include <Arduino.h>

void setup() {
    UCSR1A = 0x00; // UCSRA set as default. 
    UCSR1B = 0x00; // UCSRB set as default.
    UCSR1C = 0x00; // UCSRC set as default.

    UCSR1B |= (1<<RXEN1) | // Enable receiver
              (1<<TXEN1) | // Enable transmitter
              (1<<RXCIE1)| // Enable RX Interrupt
              (1<<TXCIE1); // Enable TX Interrupt
    UCSR1C |= (1<<UCSZ10) |
              (1<<UCSZ11); // 8-bit data
    UBRR1 = 51; // UBRR1 = (fosc / (16 * Baud Rate)) - 1
                //       = (8MHz / (16 * 9600)) - 1 = 51.08

    DDRF &= ~(1<<PF6); // Right button PF6 as input
    DDRC |= (1<<PC7); // Set PC7 as output
    PORTC &= ~(1<<PC7); // Set LED to off
}

ISR(USART1_RX_vect){
    uint16_t receivedData = UDR1; // Read received data
    UDR1 = receivedData; // Send received data
}

void loop() {
    // Team A
    if(PINF & (1<<PF6)){
        UDR1 = 0x31; // Send 1
    } else {
        UDR1 = 0x30; // Send 0
    }
    // if(UDR1 == 0x31){
    //     PORTC |= (1<<PC7); // Set LED to on
    // } else {
    //     PORTC &= ~(1<<PC7); // Set LED to off
    // }

    // // Team B
    // if(UDR1 == 0x31){
    //     PORTC |= (1<<PC7); // Set LED to on
    //     UDR1 = 0x31; // Send 1
    // } else {
    //     PORTC &= ~(1<<PC7); // Set LED to off
    //     UDR1 = 0x30; // Send 0
    // }
}
