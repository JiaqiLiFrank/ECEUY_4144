#include <Arduino.h>

void setup(){
  /*UCSR1A is set as default. 
    Normal transmission speed, 
    disable the multi-processor communication mode */
  UCSR1B |= (1<<RXCIE1) | // Turn on RX Complete Interrupt
            (1<<UDRIE1) | // Turn on the data register empty interrupt
            (1<<RXEN1);   // Turn on the receiver
  UCSR1B &= ~(1<<TXEN1);  // Turn off the Transmitter
  UCSR1B &= ~(1<<TXCIE1);  // Turn off TX Complete Interrupt
  UCSR1C |= (1<<UCSZ11) | // Character Size to 8-bit
            (1<<UCSZ10);
  UBRR1 = 51;             // UBRR1 = (fosc / (16*Baud Rate)) - 1
                          //       = (8MHz/(16*9600))-1 = 51.08
}

void loop(){

}