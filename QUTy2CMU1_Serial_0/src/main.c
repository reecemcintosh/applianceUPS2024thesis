#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
#include "serial.h" 

////
// Modified the original QUTy_serial_helloworld to poll the value
// of an output pin (PB0) when a key is pressed. The value will 
// appear in the serial monitor. LED0 (PB1) will toggle when a key
// is pressed as an indicator of the polling action. 
////

int main(void) 
{   
    //// Serial Initialisation
    cli();                                                      // Disable interrupts
    CCP = CCP_IOREG_gc;                                         // Configuration change enable			            
    CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm;    // Set clock to 10 MHz (x2 prescaler enabled)
    serial_init();                                              // Init serial & stdio
    sei();                                                      // Enable interrupts
    
    //// LED1 (PB0) Initialisation
    VPORTB.DIR |= PIN0_bm;  // Configures Pin 0 as an output pin, and enables output driver
    VPORTB.OUT |= PIN0_bm;  // Drives Pin 0 high, 0b0000 0001

    //// LED0 (PB1) Initialisation 
    VPORTB.DIR |= PIN1_bm;  // Configures Pin 1 as an output pin, and enables output driver
    VPORTB.OUT |= PIN1_bm;  // Drives Pin 1 high, 0b0000 0001
    // used to indicate when LED1 status is polled

    //// Main loop   
    for (;;) 
    {
        volatile int ledstatusPIN0 = VPORTB.IN & PIN0_bm; // Check the value of PIN0 and assign to variable. Redone each loop to refresh variable.
        // volatile int ledstatusPIN0 = VPORTA.IN; // Check the value of VPORT.IN and assign to variable. Redone each loop to refresh variable.

        if (serial_bytesAvailable()) 
        {
            putchar(getchar()); // Detects keypress with getchar() and sends that keypress via serial to board.
            printf("Computer key press!\n"); // Make sure UPDI/UART switch is flipped to UART
            printf("Value of the red LED pin: %d\n", ledstatusPIN0); // Prints out the value of PIN0 LED1 to serial

            // Toggle LED1 (PB0)
            VPORTB.OUT ^= PIN0_bm; // Toggles PB0, LED1 - LED of interest
            VPORTB.OUT ^= PIN1_bm; // Toggles PB1, LED0 - indicator LED
            _delay_ms(500); // debouncing       
        }
    }
}