#include <stdint.h>

// SERIAL
//
// Uses 
//   Pins: PB2, PB3
//   Peripherals: USART0
void serial_init();                             // Initialise UART as stdin/stdout
uint8_t serial_bytes_available(void);           // Returns number of bytes in receive buffer

// Declaring variables defined in serialNoISR.c, to be used in main.c
extern volatile uint8_t rxbuf[256];     //Rx buffer
extern volatile uint8_t pWrite;         //Write index into Rx buffer
extern volatile uint8_t pRead;          //Read index into Rx buffer
