#include <Arduino.h>

char receivedByte = 0;

void USART_Init() {
  /* UCSR1A is set as default. 
     Normal transmission speed, 
     disable the multi-processor communication mode */
  UCSR1A = 0x00;

  UCSR1B = 0x00;            // Reset the UCSR1B
  UCSR1B |= (1 << RXCIE1) | // Enable RX Complete Interrupt
            (1 << RXEN1)  | // Enable Receiver
            (1 << TXEN1);   // Enable Transmitter

  UCSR1C = 0x00;            // Reset the UCSR1C
  UCSR1C |= (1 << UCSZ11) | // Set Character Size to 8-bit
            (1 << UCSZ10);
            
  UBRR1 = 51;               // UBRR1 = (fosc / (16 * Baud Rate)) - 1
                            //       = (8MHz / (16 * 9600)) - 1 = 51.08
}

ISR(USART1_RX_vect) {
  receivedByte = UDR1; // Read the received byte
}

void TransmitString(const char* str, uint8_t length) {
  for (uint8_t i = 0; i < length; i++) {
    while (!(UCSR1A & (1 << UDRE1))) {
    }
    UDR1 = str[i]; // Send the next byte
  }
}

char GetNextReceivedByte(){
  char byte;
  cli();               // Disable global interrupts
  byte = receivedByte; // Read the received byte
  receivedByte = 0;    // Clear the received byte after reading
  sei();               // Re-enable global interrupts
  return byte;
}

void setup() {
  USART_Init(); // Initialize USART1
  sei();        // Enable global interrupts
}

void loop() {
  char currentByte = GetNextReceivedByte();
  if (currentByte != 0) {
    switch (currentByte) {
      case '1':
        TransmitString("One\n", 4);
        break;
      case '2':
        TransmitString("Two\n", 4);
        break;
      default:
        TransmitString("Default\n", 8);
        break;
    }
  }
}
