// #include <Arduino.h>

// char receivedByte = 0;

// void USART_Init() {
//   /* UCSR0A is set as default. 
//      Normal transmission speed, 
//      disable the multi-processor communication mode */
//   UCSR1A = 0x00;            // Reset the UCSR0A

//   UCSR1B = 0x00;            // Reset the UCSR0B
//   UCSR1B |= (1 << RXCIE1) | // Enable RX Complete Interrupt
//             (1 << RXEN1)  | // Enable Receiver
//             (1 << TXEN1);   // Enable Transmitter

//   UCSR1C = 0x00;            // Reset the UCSR0C
//   UCSR1C |= (1 << UCSZ11) | // Set Character Size to 8-bit
//             (1 << UCSZ10);
            
//   UBRR1 = 51;               // UBRR0 = (fosc / (16 * Baud Rate)) - 1
//                             //       = (8MHz / (16 * 9600)) - 1 = 51.08
// }

// ISR(USART1_RX_vect) {
//   receivedByte = UDR1;      // Read the received byte
  
// }

// void TransmitString(const char* str, uint8_t length) {
//   // Transmit byte by byte
//   for (uint8_t i = 0; i < length; i++) {
//     while (!(UCSR1A & (1 << UDRE1))) {
//       // Wait for the transmit buffer to be empty
//     }
//     UDR1 = str[i]; // Transmit the byte
//   }
// }

// char GetNextReceivedByte(){
//   char byte;
//   cli();               // Disable global interrupts
//   byte = receivedByte; // Read the received byte
//   receivedByte = 0;    // Clear the received byte after reading
//   sei();               // Enable global interrupts
//   return byte;
// }

// void setup() {
//   USART_Init(); // Initialize USART0
//   sei();        // Enable global interrupts
// }

// void loop() {
//   char currentByte = GetNextReceivedByte();
//   // Check if a byte is received
//   if (currentByte != 0) {
//     switch (currentByte) {
//       case '1':
//         TransmitString("One\n", 4);
//         break;
//       case '2':
//         TransmitString("Two\n", 4);
//         break;
//       default:
//         TransmitString("Default\n", 8);
//         break;
//     }
//   }
// }

#include <Arduino.h>

#define BAUD_RATE 9600

volatile uint8_t rx_buffer;
volatile uint8_t rx_flag = 0;

void USART1_Init(void) {
    // Calculate UBRR1 value based on F_CPU
    uint16_t ubrr_value = (F_CPU / (16UL * BAUD_RATE)) - 1;

    // Set baud rate
    UBRR1H = (uint8_t)(ubrr_value >> 8);
    UBRR1L = (uint8_t)(ubrr_value & 0xFF);

    // Set USART Control and Status Register A
    UCSR1A = 0x00;

    // Set USART Control and Status Register B (Transmitter Enabled)
    UCSR1B = (1 << RXCIE1) | (1 << RXEN1) | (1 << TXEN1);

    // Set USART Control and Status Register C
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
}

void setup() {
    cli(); // Disable global interrupts
    USART1_Init(); // Initialize USART1
    sei(); // Enable global interrupts
}

uint8_t GetNextReceivedByte() {
    while (!rx_flag); // Wait until a byte is received
    rx_flag = 0;
    return rx_buffer;
}

void TransmitString(const char* str, uint8_t length) {
    for (uint8_t i = 0; i < length; i++) {
        while (!(UCSR1A & (1 << UDRE1))); // Wait until buffer is empty
        UDR1 = str[i]; // Send the character
    }
}

void loop() {
    switch (GetNextReceivedByte()) {
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

// ISR for USART1 Receive Complete
ISR(USART1_RX_vect) {
    rx_buffer = UDR1; // Read the received byte
    rx_flag = 1;
}
