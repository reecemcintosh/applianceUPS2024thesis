#include <avr/io.h>
#include <util/delay.h>

int main(void) {
    // Original Code below
    //
    // // Display on
    // PORTB.OUTSET = PIN1_bm;
    // // DISP_EN, DISP_DP as outputs
    // PORTB.DIRSET = (PIN5_bm | PIN1_bm);
    
    // for (;;) {
    //     // 500ms delay (1 Hz flash)
    //     _delay_us(500000);
    //     // Toggle DP
    //     PORTB.OUTTGL = PIN5_bm;
    // }
    
    ////
    // Blinking LEDs attached to PB1 and PB0, where the QUTy board
    // is connected via breadboard to output circuitry mimicking
    // the CMU1 of the Fridge UPS thesis project.
    //
    // The 7-seg display alternates toggles opposite to the LED pins.
    // This is unintentional. 
    ////

    // Set LED1 and LED0 (PB1, PB0) as outputs
    PORTB.DIRSET = PIN0_bm;
    PORTB.DIRSET = PIN1_bm;

    for (;;) {
        // 1s delay (2Hz flash)
        _delay_ms(1000);
        // Toggle both pins
        PORTB.OUTTGL = PIN0_bm;
        PORTB.OUTTGL = PIN1_bm;
    }
}