#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
#include <stdint.h>

#include "qutyserial.h"

////-----------------
// PROGRAM DESCRIPTION
// Reads a voltage input on PA7, from a voltage divider of a 10k pot and resistor (i.e. 1.6k).
// Uses this analog input as the positive input to a differential ADC. Negative input is VREFA.
// External voltage reference VREFA is used for the ADC. This is on PA5. 
// To drive the voltage reference (LM431 in breakout board), PA4 outputs HIGH (~2.8V) through 0K5
// to the zener cathode.
//
// Serial monitor will display ADC value on keypress. Use following formula to convert
// to PA7 voltage.
// ((AIN7 - VREFA) / VREFA) * 128)
// where 128 is from the {-128:127} 8 bit ADC range. 
////-----------------

void adc_init(void)
{
    //-------------------
    // Modified CAB202 function by the same name (from studio_demo_09)
    // Differential conversion, measuring difference between VREFA and AIN7.
    // (AIN7 - VREFA) / VREFA = {-128 <-> 127}
    // ------
    // Adjusted the pin connected to MUXPOS from AIN2 to AIN7
    // Adjusted REFSEL to from VDD to VREFA
    //-------------------

    //// ADC setup
    ADC0.CTRLA = ADC_ENABLE_bm;                                        // Enable ADC
    ADC0.CTRLB = ADC_PRESC_DIV2_gc;                                    // /2 clock prescaler
    ADC0.CTRLC = (4 << ADC_TIMEBASE_gp) | ADC_REFSEL_VREFA_gc;         // Need 4 CLK_PER cycles @ 3.3 MHz for 1us, select VREFA as ref
    ADC0.CTRLE = 64;                                                   // Sample duration of 64
    ADC0.CTRLF = ADC_FREERUN_bm | ADC_LEFTADJ_bm;                      // Free running, left adjust
    ADC0.MUXPOS = ADC_MUXPOS_AIN7_gc;                                  // Select AIN7 (button 3/AIN P0) as positive ADC input
    ADC0.MUXNEG = ADC_MUXNEG_AIN5_gc;                                  // Select AIN5 (VREFA) as negative ADCinput
    ADC0.COMMAND |= ADC_DIFF_bm;                                       // Select Differential ADC conversion
    ADC0.COMMAND |= ADC_MODE_SINGLE_12BIT_gc | ADC_START_IMMEDIATE_gc; // 12-bit resolution, single-ended
}

int main(void)
{
    //// INIT
    // Serial and ADC
    cli();                                                   // Disable interrupts
    CCP = CCP_IOREG_gc;                                      // Configuration change enable
    CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm; // Set clock to 10 MHz (/2 prescaler enabled)
    serial_init();                                           // Init serial & stdio
    adc_init();                                              // Init ADC
    sei();                                                   // Enable interrupts

    //// Pin initializing
    // PA4 provides REG PWR (output, HIGH)
    PORTA.DIR |= PIN4_bm; // Setting PA4 as an output
    PORTA.OUT |= PIN4_bm; // Driving PA4 high

    // PA5 is VREFA (analog input pin), gets defined by ADC initializing

    // PIN A7 is positive analog input (AIN P0)
    PORTA.DIR &= ~PIN7_bm;    // Using Pin as input
    PORTA.PIN7CTRL |= PORT_ISC_INTDISABLE_gc; // Input buffer enabled

    //// Variable defining
    signed resultADC;
    
    //// Main loop
    for (;;)
    {
        if (getchar()) // Detects keypress with getchar()
        {
            // Make sure UPDI/UART switch is flipped to UART

            resultADC = ADC0.RESULT;                     // 12-bit result in bits 15..4
            printf("ADC value: %03d\n", resultADC >> 8); // print the most significant 8 bits
            _delay_ms(1000);
        }
    }
}