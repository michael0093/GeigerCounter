/*
 * File:   main.c
 * Author: MichaelC
 *
 * Created on August 8, 2021, 3:05 PM
 */


#include <xc.h>

void main(void) {
    // CONFIG1
    #pragma config FOSC = INTOSCIO  // Oscillator Selection bits (INTRC oscillator; port I/O function on both RA6/OSC2/CLKO pin and RA7/OSC1/CLKI pin)
    #pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
    #pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
    #pragma config MCLRE = OFF      // RA5/MCLR/VPP Pin Function Select bit (RA5/MCLR/VPP pin function is digital I/O, MCLR internally tied to VDD)
    #pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
    #pragma config LVP = OFF         // Low-Voltage Programming Enable bit (RB3/PGM pin has RB3 function, LVP disabled)
    #pragma config CPD = OFF        // Data EE Memory Code Protection bit (Code protection off)
    #pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off)
    #pragma config CCPMX = RB0      // CCP1 Pin Selection bit (CCP1 function on RB0)
    #pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

    // CONFIG2
    #pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
    #pragma config IESO = ON        // Internal External Switchover bit (Internal External Switchover mode enabled)
    
//    A0(an)	HVFB
//    A1(an)	Vbatt
//    A2(an)	Backlight
//    A3(an)	
//    A4(an)	
//    A5(in)	
//    A6	LCD-Reg
//    A7	LCD-Enable
//    B0	Boost (PWM)
//    B1	LCD-Data
//    B2	LCD-Data
//    B3	[PGM] LCD-Data
//    B4(ioc)	LCD-Data
//    B5(ioc)	Tube
//    B6(ioc)	PGC / Buzzer/LED
//    B7(ioc)	PGD / Button

    // Peripheral Setup
    OSCCON = 0b01110000;    // 8MHz internal osc  
    
    PORTA  = 0b00000000;
    TRISA  = 0b00000011;    // HVFB and Vbatt inputs
    PORTB  = 0b00000000;
    TRISB  = 0b10100000;    // Tube, Button inputs
    
    ANSEL  = 0b00000011;    // HVFB and Vbatt analog in
    ADCON0 = 0b10000000;    // ADC clock = Fosc/32, 4us typical
    ADCON1 = 0b10000000;    // Right justified data
    
    OPTION_REG = 0b11111111;    // Timer0 is Fosc/4 = 500ns, prescaler used on WDT not Tmr0
    
    T1CON  = 
    WDTCON = 0b00000011;    // 2.048ms watchdog timeout
    
    PIR1   = 0b00000000;
    PIE1   = 0b01000000;    // ADC interrupt
    INTCON = 0b11001000;    // Global interrupts enabled, PortB Change interrupt enabled
    
    while(1){
        
        asm("CLRWDT");  // Clear 2.048ms watchdog timer
    }
    return;
}
