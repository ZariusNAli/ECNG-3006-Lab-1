//Zarius Ali
//816032702

// PIC18F4620 Configuration Bit Settings
// CONFIG1H
#pragma config OSC = HS         // Oscillator Selection bits (HS oscillator)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)


#include <xc.h>
#include <stdio.h>

#define WAITING 0 
#define LEDON 1 
#define LEDOFF 2

unsigned int state;
char keyPress = '1';
int tick = 0, receiveChar = 0, turnLedON = 0, turnLedOFF = 1;


void putch(char c)
{
    while(!TXIF)
        continue;
    
    TXREG = c;
    
}


void __interrupt(high_priority) Timerinterupt(void)
{

    if(PIE1bits.TMR2IE && PIR1bits.TMR2IF)
    {
        PIR1bits.TMR2IF = 0;
        tick++;
    }
    if(tick > 10000)
    {
        tick = 0;
    }
    return;
    
}

void __interrupt(low_priority) RCinterupt(void)
{
    //When a character is received, interrupt and record the character into keyPress
    //
    if (PIR1bits.RCIF && PIE1bits.RCIE)
    {
        RCIF = 0;
        keyPress = RCREG;
        receiveChar = 1;
    }
    return;
    
}



void configUSART(void)
{
    RCSTAbits.SPEN = 1;
    RCSTA = 0b10110000;
    SPBRGH = 0x00;
    SPBRG = 0b00011001;
    BAUDCON = 0b00000000;
    INTCONbits.PEIE = 1;
    PIE1bits.RCIE = 1;
    IPR1bits.RCIP = 0;
    TXSTAbits.BRGH = 1;
    TXSTAbits.SYNC = 0;
    TXSTAbits.TXEN = 1;
    TRISCbits.TRISC6 = 1;
    TRISCbits.TRISC7 = 1;
    
}/* configUSART() */


void configLED()
{
    TRISDbits.TRISD2 = 0;
    LATDbits.LATD2 = 0;
    
}/* configLED() */


void configTimer2(void)
{
    T2CON = 0b01111111;
    PIE1bits.TMR2IE = 1;
    PR2 = 0b01001110;
    IPR1bits.TMR2IP = 1;
    
}



void wait()
{
    int now = tick;
    char hold = keyPress;
    while((tick-now)<50)
    {
        if(hold != keyPress)
        { 
            break;
        }
    }
}/* wait() */

void turnLEDoff()
{
    LATDbits.LATD2 = 0;
    turnLedON = 0;
    turnLedOFF = 1;
    receiveChar = 0;
    
}/* turnLEDoff() */

void turnLEDon()
{
    LATDbits.LATD2 = 1;
    turnLedON = 1;
    turnLedOFF = 0;
    receiveChar = 0;
    
}/* turnLEDon() */


void configInterupt()
{
    RCONbits.IPEN = 1;
    INTCONbits.GIE = 1;
    
}/* configInterupt() */



void main(void) {
    
    configLED();
    configTimer2();
    configInterupt();
    configUSART();
    
    int current = 0;
	state = 0;
	printf ("\n\n");
    while(1){
        current = tick;
        if((tick-current) > 0)
        {
            printf("State = %2d \r",state);
            switch(state) 
            { 
                case WAITING:
                    if(receiveChar == 0)
                    {
                        state = WAITING;
                    }
                    else if(turnLedON == 0 && receiveChar == 1)
                    {
                        state = LEDON; 
                    }
                    else if(turnLedOFF == 0 && receiveChar == 1)
                    {
                        state = LEDOFF;
                    }
                break;
                case LEDON:
                    turnLEDon();
                    wait();
                    state = WAITING;
                break;
                case LEDOFF:
                    turnLEDoff();
                    wait();
                    state = WAITING;
                break;
            }
        }
        
    }
    return;
}