#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
#include <stdint.h>

#include "qutyserial.h"

////-----------------
// PROGRAM DESCRIPTION
// Builds on "QUTy2CMU1_SerialADC", except ADC is read via an interrupt serivce routine (ISR)
// instead of computer keypress.
//
// From "QUTy2CMU1_SerialADC":
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

static volatile signed resultADC = 0; // ADC result storage variable. accessed by ISR and main code, so volatile

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

    // PA7 is positive analog input (AIN P0)
    PORTA.DIR &= ~PIN7_bm;    // Using Pin as input
    PORTA.PIN7CTRL |= PORT_ISC_INTDISABLE_gc; // Input buffer enabled

    // PA6 is Button 2 (Switch 3) (input, interrupts enabled)
    // PORTA.DIRCLR |= PIN6_bm; // Using Pin as input [THIS OPERATION ALWAYS MAKES ADC FAIL?]
    PORTA.PIN6CTRL |= PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc; // Enabled pullup (ACTIVE LOW), interrupt on falling edge (ISC_FALLING)

    // Set LED0 (PB0) as outputs
    PORTB.DIRSET = PIN1_bm;

    //// Variable defining
    // signed resultADC; // Moved to above main() since variable is called in ISR

    //// Main loop
    for(;;)
    {
        // _delay_ms(2000);
        // PORTB.OUTTGL = PIN1_bm;
    }
}

//// Interrupt Service Routine (ISR)
// Switch 3 (Button 2, PA6) will prompt ADC reading

ISR(PORTA_PORT_vect)
{
    if (VPORTA.INTFLAGS & PIN6_bm)
    {
        resultADC = ADC0.RESULT;                     // 12-bit result in bits 15..4
        printf("ADC value: %d\n", resultADC >> 4); // print the most significant 8 bits
        _delay_ms(250); // debouncing pause
        VPORTA.INTFLAGS = PIN6_bm; // Clear interrupt flag
    }
}

ISR(USART0_RXC_vect) 
{
    //Add character to Rx buffer
    rxbuf[pWrite++] = USART0.RXDATAL;
    
    // Read ADC and print to terminal
    resultADC = ADC0.RESULT;                     // 12-bit result in bits 15..4
    printf("ADC value (Hex): %.4x\n", resultADC >> 4); // print all 12 bits, hex
    printf("ADC value (Decimal): %d\n", resultADC >> 4); // all 12 bits, dec
}