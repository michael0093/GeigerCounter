/*
 * File:   main.c
 * Author: MichaelC
 *
 * Created on August 8, 2021, 3:05 PM
 */

// CONFIG1
#pragma config FOSC = INTOSCIO  // Oscillator Selection bits (INTRC oscillator; port I/O function on both RA6/OSC2/CLKO pin and RA7/OSC1/CLKI pin)
#pragma config WDTE = ON        // Watchdog Timer Enable bit 
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = OFF      // RA5/MCLR/VPP Pin Function Select bit 
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable bit (RB3/PGM pin has RB3 function, LVP disabled)
#pragma config CPD = OFF        // Data EE Memory Code Protection bit (Code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off)
#pragma config CCPMX = RB0      // CCP1 Pin Selection bit (CCP1 function on RB0)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

// CONFIG2
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#pragma config IESO = ON        // Internal External Switchover bit (Internal External Switchover mode enabled)
    
#include <xc.h>

#define _XTAL_FREQ          8000000

#define ADC_TMR0_RATE       0//210     // Timer0 is Fosc/4/8: up to 1.024ms. (256-206 = rate 200us))
#define TMR1_CLOCK_RATE_H   0x9E    // Timer1 is Fost/4/8: up to 0.262s. (65536-40536 = Rate = 0.1s)
#define TMR1_CLOCK_RATE_L   0x40


#define EVENT_HVFB          0b10000000      // Interrupt driven events
#define EVENT_VBATT         0b01000000
#define EVENT_PORTB         0b00100000
#define EVENT_100ms         0b00010000

// 5x7 display supports battery capacitor symbol 0-6
#define VBATT_6of6          829     // >4.05V
#define VBATT_5of6          808     // >3.95V
#define VBATT_4of6          788     // >3.85V
#define VBATT_3of6          777     // >3.80V
#define VBATT_2of6          773     // >3.78V
#define VBATT_1of6          761     // >3.72V
#define VBATT_0of6          744     // >3.64

#define LCD_RS              PORTAbits.RA6  // RA6 LCD RS Pin
#define LCD_E               PORTAbits.RA7  // RA7 LCD E Pin
#define LCD_DB4             PORTBbits.RB1  // RB1 LCD 4 Pin
#define LCD_DB5             PORTBbits.RB2  // RB2 LCD 5 Pin
#define LCD_DB6             PORTBbits.RB3  // RB3 LCD 6 Pin
#define LCD_DB7             PORTBbits.RB4  // B4 LCD 7 Pin
#define LCD_DELAY           5       // General delay between commands. 1us works, 40us as long as you'd ever need

#define TARGET_HVFB         689     // 6M6/56k divider so (5* TARGET_HVFB/1023) / (56k / (56k+6M6)) eg 689 counts = 400V
#define HV_DUTY_MAX         56      // 100% duty = PR2 value. eg 56/80 = 70%
#define DEFAULT_PID_P       10       // Divided factor
#define DEFAULT_PID_D       0       // Multiplied factor       

//    A0(an)	HVFB
//    A1(an)	Vbatt
//    A2(an)	Backlight
//    A3(an)	-
//    A4(an)	-
//    A5(in)	MCLR
//    A6	LCD-RS
//    A7	LCD-Enable
//    B0	Boost (PWM)
//    B1	LCD-Data
//    B2	LCD-Data
//    B3	[PGM] LCD-Data
//    B4(ioc)	LCD-Data
//    B5(ioc)	Tube
//    B6(ioc)	PGC / Buzzer/LED
//    B7(ioc)	PGD / Button

volatile char events = 0;
volatile unsigned short vbatt = 0;
volatile unsigned short hvfb = 0;

char HV_Startup_Duty = 0;

void nop_delay(volatile unsigned short nops);
void lcd_init();
void lcd_clear();
void lcd_cursor(char row, char column);
void lcd_write_byte(char byteIn, char RS);
void lcd_write_nibble(char byteIn, char RS);
void lcd_write_string(char *stringArray);

char numDigits(unsigned short num);
char intToString(unsigned short number, unsigned short divisor, char* dest);    // Convert long to null terminated string. When divisor=1, you get exactly what you put in

void main(void) {
    
    char numStr[6];
    
    // Peripheral Setup
    OSCCON = 0b01111100;    // 8MHz internal osc  
    
    PORTA  = 0b00000100;
    TRISA  = 0b00100011;    // HVFB and Vbatt inputs, A5 can only be input (MCLR)
    PORTB  = 0b00000000;
    TRISB  = 0b10100000;    // Tube, Button inputs
    
    ANSEL  = 0b00000011;    // HVFB and Vbatt analog in
    ADCON1 = 0b10000000;    // Right justified data
    ADCON0 = 0b11000001;    // ADC clock = Fosc/32, 4us typical, turn on now
    
    OPTION_REG = 0b11000010; // Timer0 is Fosc/4/8: up to 1.024ms
    TMR0   = ADC_TMR0_RATE;  
    
    T1CON  = 0b00110001;    // Timer1 is Fost/4/8: up to 0.262s
    TMR1H  = TMR1_CLOCK_RATE_H;   
    TMR1L  = TMR1_CLOCK_RATE_L;
        
    WDTCON = 0b00000101;    // 4.096ms watchdog timeout, prescaler is used on Timer0
    
    PR2    = 80;            // 24.69kHz PWM period
    T2CON  = 0b00000100;    // TMR2 (PWM) on, period = Fosc/4/1
    CCPR1L = 0;
    CCP1CON = 0b00111100;   // PWM mode, this byte has 2LSBs (0b00XX0000)
    
    PIR1   = 0b00000000;
    PIE1   = 0b01000001;    // ADC and Timer1 interrupts
    INTCON = 0b11100000;    //0b11101000;    // Global interrupts enabled, PortB Change interrupt enabled

    // Welcome screen with version
    lcd_init();
    lcd_write_string(" GEIGER COUNTER");
    lcd_cursor(1,6);
    lcd_write_string("V0.1");
    
    // Load the battery level indicators so we can just pick the right one at runtime
//    lcd_write_byte(0x40, 0);    // CGRAM position 0, battery empty
//    lcd_write_byte(0b01110, 1);
//    lcd_write_byte(0b10001, 1);
//    lcd_write_byte(0b10001, 1);
//    lcd_write_byte(0b10001, 1);
//    lcd_write_byte(0b10001, 1);
//    lcd_write_byte(0b10001, 1);
//    lcd_write_byte(0b10001, 1);
//    lcd_write_byte(0b11111, 1);
//    
//    // CGRAM position 1
//    lcd_write_byte(0b01110, 1);
//    lcd_write_byte(0b10001, 1);
//    lcd_write_byte(0b10001, 1);
//    lcd_write_byte(0b10001, 1);
//    lcd_write_byte(0b10001, 1);
//    lcd_write_byte(0b10001, 1);
//    lcd_write_byte(0b11111, 1);
//    lcd_write_byte(0b11111, 1);
//    
//    // CGRAM position 2
//    lcd_write_byte(0b01110, 1);
//    lcd_write_byte(0b10001, 1);
//    lcd_write_byte(0b10001, 1);
//    lcd_write_byte(0b10001, 1);
//    lcd_write_byte(0b10001, 1);
//    lcd_write_byte(0b11111, 1);
//    lcd_write_byte(0b11111, 1);
//    lcd_write_byte(0b11111, 1);
//    
//    // CGRAM position 3
//    lcd_write_byte(0b01110, 1);
//    lcd_write_byte(0b10001, 1);
//    lcd_write_byte(0b10001, 1);
//    lcd_write_byte(0b10001, 1);
//    lcd_write_byte(0b11111, 1);
//    lcd_write_byte(0b11111, 1);
//    lcd_write_byte(0b11111, 1);
//    lcd_write_byte(0b11111, 1);
//    
//    // CGRAM position 4
//    lcd_write_byte(0b01110, 1);
//    lcd_write_byte(0b10001, 1);
//    lcd_write_byte(0b10001, 1);
//    lcd_write_byte(0b11111, 1);
//    lcd_write_byte(0b11111, 1);
//    lcd_write_byte(0b11111, 1);
//    lcd_write_byte(0b11111, 1);
//    lcd_write_byte(0b11111, 1);
//    
//    // CGRAM position 5
//    lcd_write_byte(0b01110, 1);
//    lcd_write_byte(0b10001, 1);
//    lcd_write_byte(0b11111, 1);
//    lcd_write_byte(0b11111, 1);
//    lcd_write_byte(0b11111, 1);
//    lcd_write_byte(0b11111, 1);
//    lcd_write_byte(0b11111, 1);
//    lcd_write_byte(0b11111, 1);
//    
//    // CGRAM position 6, battery full
//    lcd_write_byte(0b01110, 1);
//    lcd_write_byte(0b11111, 1);
//    lcd_write_byte(0b11111, 1);
//    lcd_write_byte(0b11111, 1);
//    lcd_write_byte(0b11111, 1);
//    lcd_write_byte(0b11111, 1);
//    lcd_write_byte(0b11111, 1);
//    lcd_write_byte(0b11111, 1); 
    
    lcd_write_byte(0x40, 0);    // CGRAM position 0, battery empty
    lcd_write_byte(0b00110, 1);
    lcd_write_byte(0b01001, 1);
    lcd_write_byte(0b01001, 1);
    lcd_write_byte(0b01001, 1);
    lcd_write_byte(0b01001, 1);
    lcd_write_byte(0b01001, 1);
    lcd_write_byte(0b01001, 1);
    lcd_write_byte(0b01111, 1);
    
    // CGRAM position 1
    lcd_write_byte(0b00110, 1);
    lcd_write_byte(0b01001, 1);
    lcd_write_byte(0b01001, 1);
    lcd_write_byte(0b01001, 1);
    lcd_write_byte(0b01001, 1);
    lcd_write_byte(0b01001, 1);
    lcd_write_byte(0b01111, 1);
    lcd_write_byte(0b01111, 1);
    
    // CGRAM position 2
    lcd_write_byte(0b00110, 1);
    lcd_write_byte(0b01001, 1);
    lcd_write_byte(0b01001, 1);
    lcd_write_byte(0b01001, 1);
    lcd_write_byte(0b01001, 1);
    lcd_write_byte(0b01111, 1);
    lcd_write_byte(0b01111, 1);
    lcd_write_byte(0b01111, 1);
    
    // CGRAM position 3
    lcd_write_byte(0b00110, 1);
    lcd_write_byte(0b01001, 1);
    lcd_write_byte(0b01001, 1);
    lcd_write_byte(0b01001, 1);
    lcd_write_byte(0b01111, 1);
    lcd_write_byte(0b01111, 1);
    lcd_write_byte(0b01111, 1);
    lcd_write_byte(0b01111, 1);
    
    // CGRAM position 4
    lcd_write_byte(0b00110, 1);
    lcd_write_byte(0b01001, 1);
    lcd_write_byte(0b01001, 1);
    lcd_write_byte(0b01111, 1);
    lcd_write_byte(0b01111, 1);
    lcd_write_byte(0b01111, 1);
    lcd_write_byte(0b01111, 1);
    lcd_write_byte(0b01111, 1);
    
    // CGRAM position 5
    lcd_write_byte(0b00110, 1);
    lcd_write_byte(0b01001, 1);
    lcd_write_byte(0b01111, 1);
    lcd_write_byte(0b01111, 1);
    lcd_write_byte(0b01111, 1);
    lcd_write_byte(0b01111, 1);
    lcd_write_byte(0b01111, 1);
    lcd_write_byte(0b01111, 1);
    
    // CGRAM position 6, battery full
    lcd_write_byte(0b00110, 1);
    lcd_write_byte(0b01111, 1);
    lcd_write_byte(0b01111, 1);
    lcd_write_byte(0b01111, 1);
    lcd_write_byte(0b01111, 1);
    lcd_write_byte(0b01111, 1);
    lcd_write_byte(0b01111, 1);
    lcd_write_byte(0b01111, 1);
    
    
    char j;
    for(j=0; j<HV_DUTY_MAX+2; j++){     // Show welcome screen for 1s and also ramp up HV. +2 since Needs HV_Startup_Duty to finish >= HV_DUTY_MAX
        __delay_ms(1000/HV_DUTY_MAX);
        HV_Startup_Duty++;
    }
    
    lcd_clear();
            
    while(1){
        char i=0;
        
        // Handle interrupt events
        if(events & EVENT_PORTB){   // Handle button/tube event and clear flag
            events &= ~EVENT_PORTB;
        }
        if(events & EVENT_100ms){   // Update time and clear flag
//            PORTB ^= 0b01000000;
            events &= ~EVENT_100ms;
        }
        
        // 9999CPM 0.1us/hB   0CPM 0.0us/h   B
        lcd_cursor(0,0);
        intToString(vbatt, 1, numStr);
        lcd_write_string(numStr);
        lcd_write_string("CPM ");
        intToString(hvfb, 1, numStr);
        lcd_write_string(numStr);
        lcd_write_string("uS/h   ");
        lcd_cursor(0,15);
        lcd_write_byte(i, 1);   // Battery symbol (0x00:Empty to 0x06:Full)
        __delay_ms(500);
        
        if(i<6){
            i++;
        }else{
            i=0;
        }
        
    }
    return;
}

void __interrupt() isr(void)
{
    static signed short duty = 0;
    static signed short currError = 0;
    static signed short prevError = 0;
    
    if (TMR0IE && TMR0IF) {
        // Timer0 is the ADC sampling trigger
        if (ADCON0 & 0b00000100){
            // Unexpected conversion is still running, abort it and come next time
            ADCON0 &= 0b11111011;
            PORTB |= 0b01000000;
            
        } else if ((ADCON0 & 0b00111000) == 0b00001000){        // Last measurement was Vbatt (ch1)
            ADCON0 |= 0b00000100;   // Start conversion of channel 0 (HVFB) set in ADIF interrupt
                       
        } else { // Last measurement was HVFB or invalid state (explicit: ((ADCON0 & 0b00111000) == 0))
            ADCON0 |= 0b00000100;   // Start conversion of channel 1 (Vbatt) set in ADIF interrupt
        }         
        
        TMR0   = ADC_TMR0_RATE;
        TMR0IF = 0;
        
    } else if (TMR1IE && TMR1IF) {
        // Timer1 is the 0.1s main time base
        events |= EVENT_100ms;
        
        TMR1H  = TMR1_CLOCK_RATE_H;
        TMR1L  = TMR1_CLOCK_RATE_L;
        TMR1IF = 0;
                
    } else if (ADIE && ADIF){
        // ADC conversion complete, update the boost PWM control loop
        if (ADCON0 & 0b00000100){
            // Unexpected conversion is still running, abort it and come next time
            PORTB |= 0b01000000;
        }
                
        if ((ADCON0 & 0b00111000) == 0b00001000){   // Vbatt measurement (ch1) is ready
            vbatt = (ADRESH << 8) | ADRESL;
            ADCON0 &= 0b11000111;   // Select Channel 0 (HVFB) for ADC start in TMR0 interrupt
            
        } else if ((ADCON0 & 0b00111000) == 0){     // HVFB measurement (ch0) is ready
            hvfb = (ADRESH << 8) | ADRESL;
            ADCON0 &= 0b11000111;
            ADCON0 |= 0b00001000;   // Select channel 1 (Vbatt) for ADC start in TMR0 interrupt
            
            // Update the HV control loop
            prevError = currError;
            currError = TARGET_HVFB - hvfb;
            
            duty = duty + (currError/DEFAULT_PID_P) + (currError-prevError)*DEFAULT_PID_D;
    
            if(duty < 0){    // HV disabled or some issue with duty calculation
                duty = 0;
                
            } else if (duty > HV_DUTY_MAX){         // Under normal operation the duty is limited to about 70%
                duty = HV_DUTY_MAX;
                
            } else if (duty > HV_Startup_Duty){     // During startup, duty is further limited to reduce in-rush current
                duty = HV_Startup_Duty;             // Final value of HV_Startup_Duty may be greater than HV_DUTY_MAX so check last
            }
            CCPR1L = (char)(duty); 
            
            asm("CLRWDT");  // Clear 2.048ms watchdog timer
        }
               
        ADIF = 0;
        
    } else if (RBIE && RBIF){
        // PortB change handles button and tube inputs
        events |= EVENT_PORTB;
        
        RBIF = 0;
    }
    return;
}

// 10000 nops = 69.8ms
void nop_delay(volatile unsigned short nops){
    for(; nops>0; nops--){
        asm("nop");
    }
    return;
}

void lcd_init(){
    // Wait at least 40ms after power application (PWRTE covers this)
    
    lcd_write_nibble(0x03, 0);      // Function set 8bit interface
    __delay_ms(6);                  // Min 4.1ms spec
    lcd_write_nibble(0x03, 0);      // Function set 8bit interface
    __delay_us(150);                // Min 100us
    lcd_write_nibble(0x03, 0);      // Function set 8bit interface
    __delay_us(LCD_DELAY);          // Min TABLE6 = 
    lcd_write_nibble(0x02, 0);      // Set 4-bit mode
    __delay_us(LCD_DELAY);
    
    lcd_write_byte(0x28, 0);        // Function set: 4bit, 2-Line, 5x8 characters
    __delay_us(LCD_DELAY);
    lcd_write_byte(0x08, 0);        // Display on/off control: D=0, C=0, B=0
    __delay_us(LCD_DELAY);
    lcd_write_byte(0x01, 0);        // LCD clear
    __delay_us(2000);               // 1500 works too, spec unknown
    lcd_write_byte(0x06, 0);        // Entry mode: incriment cursor, no shift
    __delay_us(LCD_DELAY);
    lcd_write_byte(0x0C, 0);        // On/Off control: D=1 C=0 B=0
    __delay_us(LCD_DELAY);
    return;
}

// Just send a nibble disguised as a byte. Only the LSN (byteIn & 0x0F) is sent
void lcd_write_nibble(char byteIn, char RS){
    LCD_E = 1;       // E=High

    if(RS){
        LCD_RS = 1;  
    } else {
        LCD_RS = 0; 
    }   
    LCD_DB7 = (byteIn & 0x08) >> 3;   // Mask and then invert the bit to send
    LCD_DB6 = (byteIn & 0x04) >> 2;
    LCD_DB5 = (byteIn & 0x02) >> 1;
    LCD_DB4 =  byteIn & 0x01 ;
    __delay_us(LCD_DELAY); 
    LCD_E = 0;      // High->Low clocks in the data
    // The calling function needs to do the command time waiting
    return;
}

// Write a byte as two nibbles, taking care of all the signal lines and timing
void lcd_write_byte(char byteIn, char RS){
    // RS=0: Command, RS=1: Data
    char upperNibble, lowerNibble;
    
    upperNibble = byteIn >> 4;
    lowerNibble = byteIn & 0x0F;
    
    lcd_write_nibble(upperNibble, RS);
    __delay_us(LCD_DELAY);
    lcd_write_nibble(lowerNibble, RS);
    
    // The calling function needs to do the command time waiting
    return;
}

void lcd_cursor(char row, char column){
    char position;
    
    //	Determine the new position
	position = (row * 20) + column;
	
	//	Send the correct commands to the command register of the LCD
	if(position < 20) {
		lcd_write_byte(0x80 | position, 0);
	} else if(position >= 20 && position < 40) {
		lcd_write_byte(0x80 | (position % 20 + 0x40), 0);
    } else if(position >= 41 && position < 60) {
		lcd_write_byte(0x80 | (position % 40 + 0x14), 0);
	} else if(position >= 20 && position < 40) {
		lcd_write_byte(0x80 | (position % 60 + 0x54), 0);
    }
    
    return;
}

void lcd_clear(){
    lcd_write_byte(0x01, 0);    // LCD clear
    __delay_us(2000);           // 1500 works too, spec unknown
    return;
}

void lcd_write_string(char *stringArray){
    
    while (*stringArray){
        lcd_write_byte(*stringArray++, 1);
    }
    
    return;
}

// Returns number of digits in UNSIGNED 'num' (min value = 0 for digits = 1, max value = 4294967295 for digits = 9)
char numDigits(unsigned short num) {
    if (num < 10)
        return 1;
    else if (num < 100)
        return 2;
    else if (num < 1000)
        return 3;
    else if (num < 10000)
        return 4;
    else 
        return 5;
}

// Convert long to null terminated string. When divisor=1, you get exactly what you put in, eg 123 -> "123\0". Divisor=1000: 12345 -> "12.345\0"
char intToString(unsigned short number, unsigned short divisor, char* dest) {
    char i, k;  
    char j=0;   //1=has decimal point
    char digits_decimal, digits_whole;
    unsigned short whole_portion;

    whole_portion = number / divisor;   // integer divide, this is the 'whole' part
    
    if(whole_portion == 0){
        digits_whole = 0;   // Otherwise it will be 1
        digits_decimal = numDigits(divisor)-1;    // decimals are fixed point
        k = 1;              // Allows a leading 0
    } else {
        digits_whole = numDigits(whole_portion);
        k = 0;              // Leading 0 not required
        digits_decimal = numDigits(number) - digits_whole;
    }

    if (number == 0) {
        dest[0] = '0';
        dest[1] = '\0';
        return 1;
    } else {
        for (i = 0; i < digits_decimal; i++) {
            dest[digits_whole+digits_decimal - i +k] = (number % 10) + 48;
            number = number / 10;
        }
        if(digits_decimal != 0){        // choose to print decimal place
            dest[digits_whole+k] = '.';
            j=1;    // yes decimal
        }
                                                // Add decimal pt and print the decimals
        for (i = 0; i < digits_whole+k; i++) {    
            dest[digits_whole - i - 1 + k] = (number % 10) + 48;
            number = number / 10;
        }
        dest[digits_whole+digits_decimal + j + k] = '\0';
    }

    return digits_whole+digits_decimal + j + k;  
}