#include <Arduino.h>

/*
 * Function: UART1_Init
 * --------------------
 * Initializes UART1 with a given baud rate.
 *
 * baud: Desired baud rate for UART communication (e.g., 9600).
 */
void UART1_Init(unsigned int baud) {
    // Calculate UBRR (USART Baud Rate Register) value
    // Formula: UBRR = (F_CPU / (16 * baud rate)) - 1
    unsigned int ubrr_value = (F_CPU / (16UL * baud)) - 1;

    // Set baud rate in UBRR1H and UBRR1L
    UBRR1H = (unsigned char)(ubrr_value >> 8); // High byte of UBRR
    UBRR1L = (unsigned char)(ubrr_value);      // Low byte of UBRR

    // Set frame format: 8 data bits, no parity, 1 stop bit
    // UCSR1C: USART1 Control and Status Register C
    // - UCSZ11 and UCSZ10 bits set to 1 for 8-bit character size
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);

    // Enable receiver and transmitter for UART1
    // UCSR1B: USART1 Control and Status Register B
    // - RXEN1: Receiver Enable
    // - TXEN1: Transmitter Enable
    UCSR1B = (1 << RXEN1) | (1 << TXEN1);
}

/*
 * Function: UART1_Transmit
 * ------------------------
 * Transmits a single character using UART1.
 *
 * data: The character to transmit.
 */
void UART1_Transmit(unsigned char data) {
    // Wait for the transmit buffer to be empty
    while (!(UCSR1A & (1 << UDRE1))) {
        // Wait until UDRE1 is set
    }

    // Load data into UDR1 to send it
    UDR1 = data;
}

/*
 * Function: UART1_Receive
 * -----------------------
 * Receives a single character using UART1.
 *
 * returns: The received character.
 */
unsigned char UART1_Receive(void) {
    // Wait for data to be received
    while (!(UCSR1A & (1 << RXC1))) {
        // Wait until RXC1 is set
    }

    // Get and return the received data from UDR1
    return UDR1;
}

void setup() {
    // Initialize UART1 with a baud rate of 9600
    UART1_Init(9600);
}

void loop() {
    // Receive a character via UART1
    unsigned char received_char = UART1_Receive();

    // Echo the received character back
    UART1_Transmit('114514');
}
