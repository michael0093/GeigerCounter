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
#pragma config MCLRE = ON      // RA5/MCLR/VPP Pin Function Select bit 
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

#define ADC_TMR0_RATE       100//210     // Timer0 is Fosc/4/8: up to 1.024ms. (256-206 = rate 200us))
#define TMR1_CLOCK_RATE_H   0x9E    // Timer1 is Fost/4/8: up to 0.262s. (65536-40536 = Rate = 0.1s)
#define TMR1_CLOCK_RATE_L   0x40


#define EVENT_HVFB          0b10000000      // Interrupt driven events
#define EVENT_VBATT         0b01000000
#define EVENT_PORTB         0b00100000
#define EVENT_100ms         0b00010000

// 5x8 display supports battery capacitor symbol 0-6 (7 states)
// Calculation is V*1023 / 5180. VDD=5.106V but 5.18V gives more accuracte test results
#define V_SUPPLY            4995                // VDD of CPU when running from battery booster
#define ADCRES              1023
#define VBATT_6of6          (((4050)/V_SUPPLY)*ADCRES)   // >4.05V
#define VBATT_5of6          (((3950)/V_SUPPLY)*ADCRES)   // >3.95V
#define VBATT_4of6          (((3850)/V_SUPPLY)*ADCRES)   // >3.85V
#define VBATT_3of6          (((3800)/V_SUPPLY)*ADCRES)   // >3.80V
#define VBATT_2of6          (((3780)/V_SUPPLY)*ADCRES)   // >3.78V
#define VBATT_1of6          (((3720)/V_SUPPLY)*ADCRES)   // >3.72V
#define VBATT_0of6          (((3640)/V_SUPPLY)*ADCRES)   // >3.64

#define LCD_RS              PORTAbits.RA6  // RA6 LCD RS Pin
#define LCD_E               PORTAbits.RA7  // RA7 LCD E Pin
#define LCD_DB4             PORTBbits.RB4  // RB4 LCD 4 Pin
#define LCD_DB5             PORTBbits.RB3  // RB3 LCD 5 Pin
#define LCD_DB6             PORTBbits.RB2  // RB2 LCD 6 Pin
#define LCD_DB7             PORTBbits.RB1  // RB1 LCD 7 Pin
#define LCD_DELAY           5       // General delay between commands. 1us works, 40us as long as you'd ever need

#define EEWRITE_TIMEOUT     8       // x1ms
#define ALLTIMEMAX_EEADDR   0       // EEPROM address of alltime max, MSByte, LSByte is +1 from this

#define TARGET_HVFB         689     // 6M6/56k divider so (5* TARGET_HVFB/1023) / (56k / (56k+6M6)) eg 689 counts = 400V
#define CURRERROR_LIMIT     35      // +/- this from target above causes error
#define ERR_ACCUMTIME       50      // Main loop cycles before system stop due to HV error
#define ERR_ACCUMDUTY       200     // Int loop cycles before system stop due to HV duty error
#define HV_DUTY_MAX         180     // 100% duty = PR2<<2 value. eg 224/(80<<2) = 70%. typ=150
#define DEFAULT_PID_P       10      // Divided factor
#define DEFAULT_PID_D       0       // Multiplied factor       

#define uSV_DECIMALS        3       // Show uSV conversion to this many decimal places
#define CPM_uSV             900//632    
#define CPM_uSV_DIV         100000  // uSv/h = CPM * CPM_uSV / CPM_uSV_
#define CPM_MAX             9999
#define FAST_COUNT_TIME     100     // x0.1s CPM calculation time window (update rate)
#define SLOW_COUNT_TIME     600     // x0.1s CPM calculation time window (update rate)
#define BATT_TIME           5       // x0.1s Battery symbol update rate

#define COUNTS_PER_PIXEL    1
#define BLOCK_X             5       // Each HD44780 block is 5x7 pixels
#define BLOCK_Y             8
#define PIXELS_PER_BLOCK    (BLOCK_X*BLOCK_Y)   
#define COUNTS_PER_BLOCK    (COUNTS_PER_PIXEL*PIXELS_PER_BLOCK)

#define FAST_COUNT_TIME_MULT (600/FAST_COUNT_TIME)            // Multiply the counts within the COUNT_TIME to get CPM. 600 = 60sec in 0.1s
#define SLOW_COUNT_TIME_MULT 1            // Multiply the counts within the COUNT_TIME to get CPM. 600 = 60sec in 0.1s
#define FAST_COUNTS_MAX      (CPM_MAX/FAST_COUNT_TIME_MULT)   // Stop counting after this many within the COUNT_TIME
#define SLOW_COUNTS_MAX      (CPM_MAX/SLOW_COUNT_TIME_MULT)   // Stop counting after this many within the COUNT_TIME
//#define CPM_uS_SCALE        50       // multiply CPM by this to get uS/hr

#define BTN_PRESS_100ms     2       // Number of 0.1s before registering button press. "2" could be as short as 100ms and as long as 200ms
#define BTN_HOLD_100ms      20      // Number of 0.1s before registering button long-press. Max = 25
#define BL_TIMEOUT          60000   // x0.1s max is 65535. So 60,000 = 10 minutes

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

volatile uint8_t  btn     = 255;
volatile uint16_t vbatt   = 0;
volatile uint16_t hvfb    = 0;
volatile uint16_t counts  = 0;
volatile uint16_t newCnt  = 0;        // =1 when count has increased
volatile uint32_t runtime = 0;        // in 100ms increments
volatile uint16_t cntTime = 0;        // in 100ms increments
volatile uint8_t  errDuty = 0;
volatile uint8_t  errOver = 0;
volatile uint8_t  errUndr = 0;
uint16_t cpm              = 0;
uint32_t usv              = 0;
volatile uint8_t displayState = 0;    // 0: normal screen. 1: session max. 2: alltime max
volatile uint16_t  bl_timeout = BL_TIMEOUT;
uint8_t  countsArray[7] = {0};

//volatile uint8_t dbg = 0;

uint8_t  HV_Startup_Duty = 0;
uint16_t count_max;

//void nop_delay(volatile uint16_t nops);
void lcd_init();
void lcd_clear();
void lcd_cursor(uint8_t row, uint8_t column);
void lcd_write_byte(uint8_t byteIn, uint8_t RS);
void lcd_write_nibble(uint8_t byteIn, uint8_t RS);
void lcd_write_string(uint8_t *stringArray);

void EEPROM_write(uint8_t data, uint8_t addr);
uint8_t EEPROM_read(uint8_t addr);

void clear_graph();
void recalc_graph();
uint8_t numDigits(uint32_t num);
uint8_t intToString(uint32_t number, uint32_t divisor, uint8_t* dest, uint8_t fixedWidth, uint8_t reduceDecimals);    // Convert long to null terminated string. When divisor=1, you get exactly what you put in

void batt_update();

void main(void) {
    
    uint8_t  numStr[8];
    uint8_t  j;
    uint8_t  graphBlock = 0;
    uint16_t sessionHigh = 0;
    uint32_t sessionSum  = 0;
    uint16_t sessionTime = 0;
    uint16_t alltimeHigh;
    uint16_t intg_time;
    uint8_t  count_mult;
    uint8_t  wdtr = 0;
    
    // Peripheral Setup
    OSCCON = 0b01111100;    // 8MHz internal osc  
    
    PORTA  = 0b00000100;
    TRISA  = 0b00100011;    // HVFB and Vbatt inputs, A5 can only be input (MCLR)
    PORTB  = 0b01000000;    // Start with LED/buzzer on (to test)
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
    CCP1CON = 0b00001100;   // PWM mode, this byte has 2LSBs (0b00XX0000)
    
    PIR1   = 0b00000000;
    PIE1   = 0b01000001;    // ADC and Timer1 interrupts
    INTCON = 0b11101000;    //0b11101000;    // Global interrupts enabled, PortB Change interrupt enabled

    // Welcome screen with version
    lcd_init();
    lcd_write_string("  GEIGER V0.5  ");
    
    char str[2];
    intToString(PCON & 0x03, 1, str, 1, 0);     // PCON: X X X X X X /POR /BOR
    lcd_write_string(str);
    if(PCON & 0x03 == 3){
        // Could be a watchdog reset
        wdtr = 1;
    }
    PCON = 0xFF;
        
    clear_graph();
    
    // PORTB 0x80 is button. Brief hold at start = fast mode
    if(PORTB & 0b10000000){
        intg_time = FAST_COUNT_TIME;
        count_mult = FAST_COUNT_TIME_MULT;
        count_max  = FAST_COUNTS_MAX;
        
        lcd_cursor(1,1);
        lcd_write_string("Intg Time=10s");
        
    } else {
        intg_time = SLOW_COUNT_TIME;
        count_mult = SLOW_COUNT_TIME_MULT;
        count_max  = SLOW_COUNTS_MAX;
        
        lcd_cursor(1,1);
        lcd_write_string("Intg Time=60s");
    }
    
    for(j=0; j<HV_DUTY_MAX; j++){     // Show welcome screen for 0.7s and also ramp up HV
        __delay_ms(720/HV_DUTY_MAX);
        HV_Startup_Duty++;
    }
    HV_Startup_Duty = HV_DUTY_MAX;
   
    alltimeHigh  = EEPROM_read(ALLTIMEMAX_EEADDR);
    alltimeHigh  = alltimeHigh << 8;
    alltimeHigh |= EEPROM_read(ALLTIMEMAX_EEADDR +1);
    
    if(alltimeHigh == 0xFFFF){
        // EEPROM is empty (all Fs) so ensure a new value is written by means of setting the max to be 0
        alltimeHigh = 0;
    }
    
    lcd_clear();
    
    PORTB &= ~0b01000000;       // LED/buzzer off 
    runtime = 0;    // Reset runtime counter just before start so that battery icon gets shown immediately
    cntTime = 0;
    
    // Update the battery status
    batt_update();
            
    while(1){
        
        // Note: BL off is like 10mA saving which is < 10% of the total power consumption so hardly worth it

            

        
        
        if(cntTime >= intg_time){   // It is a multiple of the count time, Only do this once per matching cntTime value

            // Update CPM and the alltime/session maximums
            cpm = counts * count_mult;
            usv = cpm * CPM_uSV;            // Still has to be divided by CPM_uSV_DIV to get uSv/h

            cntTime = 0;   // Won't run again until next time rollover
            
            sessionSum = sessionSum + cpm;
            sessionTime++;

            counts = 0;     // new time 'block'
            if (cpm > sessionHigh){
                sessionHigh = cpm;  // new session maximum
                if(sessionHigh > alltimeHigh){
                    // New all-time high, save to EEPROM
                    alltimeHigh = sessionHigh;
                    EEPROM_write( (uint8_t)((alltimeHigh & 0xFF00) >> 8), ALLTIMEMAX_EEADDR );
                    EEPROM_write( (uint8_t)((alltimeHigh & 0xFF)),        ALLTIMEMAX_EEADDR+1 );
                }
            }

            // Move CPM graph onto the next block
            if (graphBlock < 6){
                graphBlock++;
            } else {
                graphBlock = 6;     // Scroll the graph one place
                countsArray[0] = countsArray[1];
                countsArray[1] = countsArray[2];
                countsArray[2] = countsArray[3];
                countsArray[3] = countsArray[4];
                countsArray[4] = countsArray[5];
                countsArray[5] = countsArray[6];
                countsArray[6] = 0;
                recalc_graph();     // Update the CGRAM which causes display update
            }

            // Update the battery status
            batt_update();
        }
        
        // Button short press is handled in interrupt, holds are done here
        if(btn > BTN_HOLD_100ms && btn != 255){
            btn = 255;  // Won't re-trigger until button release
            
            switch (displayState){
                case 0:     // Normal counter screen, reset runtime
                    runtime = 0;
                    cntTime = 0;
                    break;
                    
                case 1:     // Reset session maximum
                    sessionHigh = 0;
                    break;
                    
                case 2:     // Reset session average
                    sessionSum = 0;
                    sessionTime = 0;
                    break;    
                    
                case 3:     // Reset alltime maximum
                    alltimeHigh = 0;
                    EEPROM_write(0, ALLTIMEMAX_EEADDR);
                    EEPROM_write(0, ALLTIMEMAX_EEADDR+1);
                    break;
            }
        }
           
        switch (displayState){
            
            default:
            case 0: // Normal (counter) screen
                // 9999CPM 0.1us/hB   0CPM 0.0us/h   B
                lcd_cursor(0,0);
                intToString(cpm, 1, numStr, 1, 0);
                lcd_write_string(numStr);
                lcd_write_string("CPM ");
                //intToString(counts, 1, numStr, 1, 0);
                intToString(usv, CPM_uSV_DIV, numStr, 0, 2);
                lcd_write_string(numStr);
                lcd_write_byte(0xE4, 1);
                lcd_write_string("S/h      ");
                
                // Integer debugs
//                lcd_cursor(0,0);
//                intToString(hvfb, 1, numStr, 1, 0);
//                lcd_write_string(numStr);
//                lcd_write_string(" ");
//                intToString(errUndr, 1, numStr, 1, 0);
//                lcd_write_string(numStr);
//                lcd_write_string(" ");
//                intToString(errOver, 1, numStr, 1, 0);
//                lcd_write_string(numStr);
//                lcd_write_string("                ");

                // Run Timer
                lcd_cursor(1,0);
                intToString(runtime/36000, 1, numStr, 1, 0);          // Runtime hours
                lcd_write_string(numStr);
                lcd_write_byte(':', 1);
                intToString((runtime%36000)/600, 1, numStr, 2, 0);    // Runtime minutes
                lcd_write_string(numStr);
                lcd_write_byte(':', 1);
                intToString((runtime/10)%60, 1, numStr, 2, 0);        // Runtime seconds
                lcd_write_string(numStr);
                if(wdtr){
                    // Possible WDT reset that the user should be aware of
                    lcd_write_string("w");
                } else {
                    lcd_write_string(" ");
                }

                // Mini graph
                lcd_cursor(1,8);
                for(j=0; j<7; j++){
                    lcd_write_byte(j, 1);   // Display contents of CGRAM 0-6
                }

                // Battery State
                lcd_cursor(1,15);
                lcd_write_byte(7, 1);       // Battery symbol (0x00:Empty to 0x06:Full)
                break;
            
            case 1:     // Session max screen
                lcd_cursor(0,0);
                lcd_write_string("Session Maximum ");
                lcd_cursor(1,0);
                intToString(sessionHigh, 1, numStr, 1, 0);
                lcd_write_string(numStr);
                lcd_write_string("CPM ");
                intToString(sessionHigh*CPM_uSV, CPM_uSV_DIV, numStr, 0, 2);
                lcd_write_string(numStr);
                lcd_write_byte(0xE4, 1);
                lcd_write_string("S/h      ");
                break;
                
            case 2:     // Session avg screen
                lcd_cursor(0,0);
                lcd_write_string("Session Average ");
                lcd_cursor(1,0);
                
                if(sessionTime > 0){
                    intToString(sessionSum/sessionTime, 1, numStr, 1, 0);
                    lcd_write_string(numStr);
                    lcd_write_string("CPM ");
                    intToString((sessionSum/sessionTime)*CPM_uSV, CPM_uSV_DIV, numStr, 0, 2);
                    lcd_write_string(numStr);
                    lcd_write_byte(0xE4, 1);
                    lcd_write_string("S/h      ");
                } else {
                    lcd_write_string("     wait..     ");
                }
                break;    
                
            case 3:     // Alltime max screen
                lcd_cursor(0,0);
                lcd_write_string("All-Time Maximum");
                lcd_cursor(1,0);
                intToString(alltimeHigh, 1, numStr, 1, 0);
                lcd_write_string(numStr);
                lcd_write_string("CPM ");
                intToString(alltimeHigh*CPM_uSV, CPM_uSV_DIV, numStr, 0, 2);
                lcd_write_string(numStr);
                lcd_write_byte(0xE4, 1);
                lcd_write_string("S/h      ");
                break;
        } 
        
        // Check for hardware errors
        if(hvfb > (TARGET_HVFB+CURRERROR_LIMIT) && HV_Startup_Duty == HV_DUTY_MAX){
            errOver++;
        } else if(hvfb < (TARGET_HVFB-CURRERROR_LIMIT) && HV_Startup_Duty == HV_DUTY_MAX){
            errUndr++;
        } else {
            if(errUndr > 0){
                errUndr--;
            }
            if(errOver > 0){
                errOver--;
            }
            if(errDuty > 0){
                errDuty--;
            }
        }
        
        if(errDuty > ERR_ACCUMDUTY){
            CCP1CON = 0;
            CCPR1L  = 0;
            lcd_clear();
            lcd_cursor(0,0);
            lcd_write_string("HV DUTY ERR");
            
            INTCON = 0;   // Disable interrupts 
            while(1){
                asm("CLRWDT");  // Clear 2.048ms watchdog timer
            }
        }
        if(errOver > ERR_ACCUMTIME){
            CCP1CON = 0;
            CCPR1L  = 0;
            lcd_clear();
            lcd_cursor(0,0);
            lcd_write_string("HI HV ERR");
            
            INTCON = 0;   // Disable interrupts 
            while(1){
                asm("CLRWDT");  // Clear 2.048ms watchdog timer
            }
        }
        if(errUndr > ERR_ACCUMTIME){
            CCP1CON = 0;
            CCPR1L  = 0;
            lcd_clear();
            lcd_cursor(0,0);
            lcd_write_string("LOW HV ERR");
            
            INTCON = 0;   // Disable interrupts 
            while(1){
                asm("CLRWDT");  // Clear 2.048ms watchdog timer
            }
        }
        
        // If the count changed, pulse the LED/buzzer
        if(newCnt){
            PORTB |= 0b01000000;    // LED/buzzer on 
            newCnt = 0;             // 'counts' is updated in the ISR
            
            // Update CPM graph block
            if( counts > COUNTS_PER_BLOCK ){
                countsArray[graphBlock] = COUNTS_PER_BLOCK;
            } else {
                countsArray[graphBlock] = counts;
            }
            
            recalc_graph();
        }           
        __delay_ms(15);             // Slow down display update to prevent flicker
        PORTB &= ~0b01000000;       // LED/buzzer off 
        __delay_ms(50);             // Slow down display update to prevent flicker
    }
    return;
}

void __interrupt() isr(void)
{
    static signed short duty = 0;
    static signed short prevError = 0;
    static signed short currError = 0;
//    static signed short integral  = 0;
    
    if (TMR0IE && TMR0IF) {
        // Timer0 is the ADC sampling trigger
        if (ADCON0 & 0b00000100){
            // Unexpected conversion is still running, abort it and come next time
            ADCON0 &= 0b11111011;
            
        } else if ((ADCON0 & 0b00111000) == 0b00001000){        // Last measurement was Vbatt (ch1)
            ADCON0 |= 0b00000100;   // Start conversion of channel 0 (HVFB) set in ADIF interrupt
                       
        } else { // Last measurement was HVFB or invalid state (explicit: ((ADCON0 & 0b00111000) == 0))
            ADCON0 |= 0b00000100;   // Start conversion of channel 1 (Vbatt) set in ADIF interrupt
        }         
        
        TMR0   = ADC_TMR0_RATE;
        TMR0IF = 0;
        
    } else if (TMR1IE && TMR1IF) {
        // Timer1 is the 0.1s main time base
        runtime++;
        cntTime++;
        if(bl_timeout > 0){
            bl_timeout--;   
        } else {
            // Times up, turn off backlight
            PORTA = PORTA & 0b11111011;
        }
        
        if(PORTB & 0b10000000){     // Every 100ms we get closer to a good button register. If IOC detects the pin go low, btn is reset
            if(bl_timeout == 0){
                // Backlight off, btn turns it on, but doesnt do any button actions
                bl_timeout = BL_TIMEOUT;
                PORTA = PORTA | 0b00000100;
                btn = 255;  // Ignore button until release
            }
            if(btn < 254){          // 255 is the 'latched' value, meaning the main code doesnt want to process the button further
                btn++;
            }
        }
               
        TMR1H  = TMR1_CLOCK_RATE_H;
        TMR1L  = TMR1_CLOCK_RATE_L;
        TMR1IF = 0;
                
    } else if (ADIE && ADIF){
        // ADC conversion complete, update the boost PWM control loop
        if (ADCON0 & 0b00000100){
            // Unexpected conversion is still running, abort it and come next time
        } else {
                
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
//                integral += currError;

//                uint16_t v;
//                v = integral/2048;

                //currError = currError / (signed short)(vbatt);

                duty = duty + (currError/8) + ((currError-prevError)/1);
                
                if(duty < 0){    // HV disabled or some issue with duty calculation
                    duty = 0;

                } else if (duty > HV_DUTY_MAX){         // Under normal operation the duty is limited to about 70%
                    duty = HV_DUTY_MAX;
                    errDuty++;

                } else if (duty > HV_Startup_Duty){     // During startup, duty is further limited to reduce in-rush current
                    duty = HV_Startup_Duty;             // Final value of HV_Startup_Duty may be greater than HV_DUTY_MAX so check last
                }
                
                CCP1CON &= 0b11001111;
                CCP1CON |= (((uint8_t)(duty)) & ~0xFC) << 4;   // This byte has 2LSBs (0b00XX0000)
                CCPR1L   = (((uint8_t)(duty)) &  0xFC) >> 2; 

//                CCPR1L   = (uint8_t)(duty);
                        
                asm("CLRWDT");  // Clear 2.048ms watchdog timer
            }
        }
        ADIF = 0;
        
    } else if (RBIE && RBIF){
        // PortB change handles tube input
                
        if(!(PORTB & 0b00100000)){     // Tube pulse detected
            if( counts < count_max) {
                counts++;
            }
            newCnt = 1;             // Tell main to pulse LED/Buzzer
        }
        if(!(PORTB & 0b10000000)){  // Every 100ms we get closer to a good button register. If IOC detects the pin go low, btn is reset
            // Check if this release constitutes a button press event
            if(btn > BTN_PRESS_100ms && btn != 255){
                if(displayState < 3){
                    displayState++;
                } else {
                    displayState = 0;
                }
            }
            btn = 0;
        }
        
        RBIF = 0;
    }
    return;
}

// 10000 nops = 69.8ms
//void nop_delay(volatile uint16_t nops){
//    for(; nops>0; nops--){
//        asm("nop");
//    }
//    return;
//}

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
void lcd_write_nibble(uint8_t byteIn, uint8_t RS){
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
void lcd_write_byte(uint8_t byteIn, uint8_t RS){
    // RS=0: Command, RS=1: Data
    uint8_t upperNibble, lowerNibble;
    
    upperNibble = byteIn >> 4;
    lowerNibble = byteIn & 0x0F;
    
    lcd_write_nibble(upperNibble, RS);
    __delay_us(LCD_DELAY);
    lcd_write_nibble(lowerNibble, RS);
    
    // The calling function needs to do the command time waiting
    return;
}

void lcd_cursor(uint8_t row, uint8_t column){
    uint8_t position;
    
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

void clear_graph(){
    
    uint8_t j;
    // Clear the CGRAM where the graph is stored
    lcd_write_byte(0x40, 0);    // GCRAM 0-6
    for(j=0; j<8; j++){
        lcd_write_byte(0x0, 1); 
        lcd_write_byte(0x0, 1);
        lcd_write_byte(0x0, 1);
        lcd_write_byte(0x0, 1);
        lcd_write_byte(0x0, 1);
        lcd_write_byte(0x0, 1);
        lcd_write_byte(0x0, 1);
        lcd_write_byte(0x0, 1);
    }
}

// Update CGRAM with countsArray[] values. Display is updated immediately when CGRAM changes
void recalc_graph(){
    
    uint8_t row, block, expanded, x, j;
    
    lcd_write_byte(0x40, 0);    // Start at CGRAM position 0, fills 0-6

    for(block=0; block<7; block++){
        x = countsArray[block];
        
        for (row=0; row<BLOCK_Y; row++){
            expanded = 0xFF;
            for(j=0; j<BLOCK_X; j++){
                if(x > 0){
                    x--;
                    expanded = expanded >> 1;
                }
            }
            expanded = ~expanded;
            expanded = expanded >> (8-BLOCK_X);
            lcd_write_byte(expanded, 1);
        }
        
    }      
}

// Display is updated immediately when CGRAM changes
void batt_update() {
    lcd_write_byte(0x78, 0);    // CGRAM position 7, battery symbol
    if(vbatt >= VBATT_6of6){
        // Battery full 6/6
        lcd_write_byte(0b00110, 1);
        lcd_write_byte(0b01111, 1);
        lcd_write_byte(0b01111, 1);
        lcd_write_byte(0b01111, 1);
        lcd_write_byte(0b01111, 1);
        lcd_write_byte(0b01111, 1);
        lcd_write_byte(0b01111, 1);
        lcd_write_byte(0b01111, 1);

    } else if ( vbatt >= VBATT_5of6){
        lcd_write_byte(0b00110, 1);
        lcd_write_byte(0b01001, 1);
        lcd_write_byte(0b01111, 1);
        lcd_write_byte(0b01111, 1);
        lcd_write_byte(0b01111, 1);
        lcd_write_byte(0b01111, 1);
        lcd_write_byte(0b01111, 1);
        lcd_write_byte(0b01111, 1);

    } else if ( vbatt >= VBATT_4of6){
        lcd_write_byte(0b00110, 1);
        lcd_write_byte(0b01001, 1);
        lcd_write_byte(0b01001, 1);
        lcd_write_byte(0b01111, 1);
        lcd_write_byte(0b01111, 1);
        lcd_write_byte(0b01111, 1);
        lcd_write_byte(0b01111, 1);
        lcd_write_byte(0b01111, 1);

    } else if ( vbatt >= VBATT_3of6){
        lcd_write_byte(0b00110, 1);
        lcd_write_byte(0b01001, 1);
        lcd_write_byte(0b01001, 1);
        lcd_write_byte(0b01001, 1);
        lcd_write_byte(0b01111, 1);
        lcd_write_byte(0b01111, 1);
        lcd_write_byte(0b01111, 1);
        lcd_write_byte(0b01111, 1);

    } else if ( vbatt >= VBATT_2of6){
        lcd_write_byte(0b00110, 1);
        lcd_write_byte(0b01001, 1);
        lcd_write_byte(0b01001, 1);
        lcd_write_byte(0b01001, 1);
        lcd_write_byte(0b01001, 1);
        lcd_write_byte(0b01111, 1);
        lcd_write_byte(0b01111, 1);
        lcd_write_byte(0b01111, 1);

    } else if ( vbatt >= VBATT_1of6){
        lcd_write_byte(0b00110, 1);
        lcd_write_byte(0b01001, 1);
        lcd_write_byte(0b01001, 1);
        lcd_write_byte(0b01001, 1);
        lcd_write_byte(0b01001, 1);
        lcd_write_byte(0b01001, 1);
        lcd_write_byte(0b01111, 1);
        lcd_write_byte(0b01111, 1);

    } else if ( vbatt >= VBATT_0of6){
        lcd_write_byte(0b00110, 1);
        lcd_write_byte(0b01001, 1);
        lcd_write_byte(0b01001, 1);
        lcd_write_byte(0b01001, 1);
        lcd_write_byte(0b01001, 1);
        lcd_write_byte(0b01001, 1);
        lcd_write_byte(0b01001, 1);
        lcd_write_byte(0b01111, 1);

    } else {
        lcd_write_byte(0b00110, 1);
        lcd_write_byte(0b01001, 1);
        lcd_write_byte(0b00000, 1);
        lcd_write_byte(0b01001, 1);
        lcd_write_byte(0b00000, 1);
        lcd_write_byte(0b01001, 1);
        lcd_write_byte(0b00000, 1);
        lcd_write_byte(0b01111, 1);
    }
}

void lcd_write_string(uint8_t *stringArray){
    
    while (*stringArray){
        lcd_write_byte(*stringArray++, 1);
    }
    
    return;
}

// Returns number of digits in UNSIGNED 'num' (min value = 0 for digits = 1, max value = 4294967295 for digits = 9)
uint8_t numDigits(uint32_t num) {
    if (num < 10)
        return 1;
    else if (num < 100)
        return 2;
    else if (num < 1000)
        return 3;
    else if (num < 10000)
        return 4;
    else if (num < 100000)
        return 5;
    else if (num < 1000000)
        return 6;
    else if (num < 10000000)
        return 7;
    else if (num < 100000000)
        return 8;
    else if (num < 1000000000)
        return 9;
    else // num <= 4294967295
        return 10;
}

// Convert long to null terminated string. When divisor=1, you get exactly what you put in, eg 123 -> "123\0". Divisor=1000: 12345 -> "12.345\0"
// fixedWidth should be 1 for normal, or the total whole digits if zero padding is required. eg: fixedWidth=2 would give 0-> 00, 5-> 05 32->32. 
uint8_t intToString(uint32_t number, uint32_t divisor, uint8_t* dest, uint8_t fixedWidth, uint8_t reduceDecimals) {
    uint8_t i, k;  
    uint8_t j=0;   //1=has decimal point
    uint8_t digits_decimal, digits_whole;
    uint16_t whole_portion;

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
        fixedWidth--;   // k is 1 instead of 0 so decrement
    }
    
    if(fixedWidth > digits_whole){     // Caller wants more leading digits than the number intrinsically has
        k = k + fixedWidth - digits_whole;  // Offset the start of the string by this many bytes which are pre-filled to ASCII 0
    }

    for (i = 0; i < digits_decimal; i++) {
        dest[digits_whole+digits_decimal +k -i] = (number % 10) + 48;
        number = number / 10;
    }
    if(digits_decimal != 0){        // choose to print decimal place
        dest[digits_whole+k] = '.';
        j=1;    // yes decimal
    }
                                            // Add decimal pt and print the decimals
    for (i = 0; i < digits_whole+k; i++) {    
        dest[digits_whole +k -i -1] = (number % 10) + 48;
        number = number / 10;
    }
    dest[digits_whole+digits_decimal + j + k - reduceDecimals] = '\0';

    return digits_whole+digits_decimal + j + k - reduceDecimals;  
}

void EEPROM_write(uint8_t data, uint8_t addr) {
    uint8_t timeout = 0;
    
    // Check for any writes in progress
    while((EECON1 & 0x02) && (timeout < EEWRITE_TIMEOUT)){
        timeout++;
        __delay_ms(1);
    }
    if(timeout >= EEWRITE_TIMEOUT){
        // Note: EECON1 & 0b00001000 is the write error bit
        PORTB |= 0b01000000;
        return; // Timed out waiting for existing EEPROM write to finish
    }
    
    EEADR  = addr;
    EEDATA = data;
    EECON1 = 0b00000100;    // Write enabled, WREN bit
    
    // Disable interrupts 
    INTCON &= 0b01111111;
            
    // Unlock sequence
    EECON2 = 0x55;
    EECON2 = 0xAA;
    
    EECON1 |= 0b00000010;    // Write the data 
    
    // Enable interrupts 
    INTCON |= 0b10000000;
    EECON1  = 0x0;          // Disable writes

    return;
}

uint8_t EEPROM_read(uint8_t addr) {
    
    EECON1 = 0x00;
    EEADR  = addr;
    
    EECON1 |= 0b00000001;   // Set RD bit to start read
    
    return EEDATA;          // Data is available immediately

}
