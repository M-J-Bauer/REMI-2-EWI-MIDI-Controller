/**
 * Module:     low_level.c
 *
 * Overview:   Low-level platform-specific I/O functions.
 */
#include <xc.h>
#include <stdlib.h>
#include <string.h>
#include "low_level.h"

// ============  MCU Configuration bits  ======================================================
//
// CONFIG1H
#pragma config FOSC = INTIO67    // Oscillator Selection bits (INTIO67: Internal 16MHz osc)
#pragma config PLLCFG = OFF      // 4X PLL Enable
#pragma config PRICLKEN = ON     // Primary clock enable bit
#pragma config FCMEN = OFF       // Fail-Safe Clock Monitor Enable bit
#pragma config IESO = OFF        // Internal/External Oscillator Switchover bit

// CONFIG2L
#pragma config PWRTEN = OFF      // Power-up Timer Enable bit
#pragma config BOREN = SBORDIS   // Brown-out Reset Enable bits->Brown-out Reset enabled
#pragma config BORV = 190        // Brown Out Reset Voltage bits->VBOR set to 1.90 V nominal

// CONFIG2H
#pragma config WDTEN = OFF       // Watchdog Timer Enable bits->Watch dog timer always disabled 
#pragma config WDTPS = 32768     // Watchdog Timer Postscale Select bits->1:32768

// CONFIG3H
#pragma config CCP2MX = PORTC1   // CCP2 MUX bit->CCP2 input/output is multiplexed with RC1
#pragma config PBADEN = ON       // PORTB A/D Enable bit->PORTB<5:0> pins are config'd as analog
#pragma config CCP3MX = PORTB5   // P3A/CCP3 Mux bit->P3A/CCP3 input/output is mux'ed with RB5
#pragma config HFOFST = ON       // HFINTOSC Fast Start-up
#pragma config T3CMX = PORTC0    // Timer3 Clock input mux bit->T3CKI is on RC0
#pragma config P2BMX = PORTD2    // ECCP2 B output mux bit->P2B is on RD2
#pragma config MCLRE = EXTMCLR   // MCLR Pin Enable bit->MCLR pin enabled, RE3 pin disabled

// CONFIG4L
#pragma config STVREN = ON       // Stack Full/Underflow Reset Enable bit
#pragma config LVP = OFF         // Single-Supply ICSP Enable bit
#pragma config XINST = OFF       // Extended Instruction Set Enable bit-
#pragma config DEBUG = OFF       // Background Debug->Disabled

// CONFIG5L
#pragma config CP0 = OFF         // Code Protection Block 0->Block 0 not code-protected
#pragma config CP1 = OFF         // Code Protection Block 1->Block 1 not code-protected
#pragma config CP2 = OFF         // Code Protection Block 2->Block 2 not code-protected
#pragma config CP3 = OFF         // Code Protection Block 3->Block 3 not code-protected

// CONFIG5H
#pragma config CPB = OFF         // Boot Block Code Protection bit->Boot block not protected
#pragma config CPD = OFF         // Data EEPROM Code Protection bit->Data EEPROM not protected

// CONFIG6L
#pragma config WRT0 = OFF        // Write Protection Block 0->Block 0 not write-protected
#pragma config WRT1 = OFF        // Write Protection Block 1->Block 1 not write-protected
#pragma config WRT2 = OFF        // Write Protection Block 2->Block 2 not write-protected
#pragma config WRT3 = OFF        // Write Protection Block 3->Block 3 not write-protected

// CONFIG6H
#pragma config WRTC = OFF        // Configuration Register Write Protection bit
#pragma config WRTB = OFF        // Boot Block Write Protection bit
#pragma config WRTD = OFF        // Data EEPROM Write Protection bit

// CONFIG7L
#pragma config EBTR0 = OFF       // Table Read Protection Block 0 not protected from table reads 
#pragma config EBTR1 = OFF       // Table Read Protection Block 1 not protected from table reads 
#pragma config EBTR2 = OFF       // Table Read Protection Block 2 not protected from table reads 
#pragma config EBTR3 = OFF       // Table Read Protection Block 3 not protected from table reads 

// CONFIG7H
#pragma config EBTRB = OFF       // Boot Block Table Read Protection bit (not protected)
//
//==============================================================================================

volatile bool   v_RTI_flag_1ms_task;
volatile bool   v_RTI_flag_5ms_task;
volatile bool   v_RTI_flag_50ms_task;
volatile uint8  v_TaskOverrunCount;     // Task overruns (max. 100 per 100ms)

static volatile uint16  v_Timer1ReloadVal;
static volatile uint32  v_RTI_tick_counter;

/*
 * Initialize Timer1 to generate 1ms periodic interrupt.
 * Prescaler = 1:2, TMR1 clock = (F_OSC / 2) = 16/2 = 8 MHz
 * Count up from 0xE0C0 to 0xFFFF = 8000 clocks = 1000 us = 1 ms.
 */
void  TMR1_Initialize(void)
{
    // T1CKPS 1:2; T1OSCEN disabled; T1SYNC do_not_synchronize; 
    // TMR1CS FOSC; TMR1ON off; T1RD16 disabled; 
    T1CON = 0x54;

    // T1GSS T1G_pin; TMR1GE disabled; T1GTM disabled; T1GPOL low; 
    // T1GGO done; T1GSPM disabled; 
    T1GCON = 0x00;

    TMR1H = 0xE0;   // TMR1 reload value for 1ms interval = 0xE0C0
    TMR1L = 0xC0;
    v_Timer1ReloadVal = TMR1;
    
    PIR1bits.TMR1IF = 0;  // Clear IF flag before enabling the interrupt.
    PIE1bits.TMR1IE = 1;  // Enable TMR1 interrupt.
    
    TIMER1_START();
}


/**
* Function:  Timer1 interrupt handler.
*
* PreCondition: Timer 1 initialization must be done.
*
* Overview:  Schedules periodic background tasks at 1ms, 5ms and 50ms intervals.
*            Maintains general-purpose timer with 1ms resolution.
*            Counts the number of (1ms) task overruns occurring in 100ms.
*
* Called by: INTERRUPT_InterruptManager()
*/
void  TMR1_InterruptHandler(void)
{
    static uint8 count_to_5 = 0;
    static uint8 count_to_50 = 0;
    
    PIR1bits.TMR1IF = 0;   // Clear TMR1 IRQ flag
 
    // Re-load TMR1 counter for req'd period
    TMR1H = (v_Timer1ReloadVal >> 8);
    TMR1L = (uint8_t) v_Timer1ReloadVal;

    if (v_RTI_flag_1ms_task != 0)  v_TaskOverrunCount++;
    v_RTI_tick_counter++;  // Free-running 1ms timer/counter

    // Trigger periodic background tasks when their respective interval expires...
    v_RTI_flag_1ms_task = 1;  
    if (++count_to_5  >= 5) { v_RTI_flag_5ms_task = 1;  count_to_5 = 0; }
    if (++count_to_50 >= 50) { v_RTI_flag_50ms_task = 1;  count_to_50 = 0; }
}


/**
 * This function returns the value of a free-running 32-bit counter variable,
 * incremented every millisecond by TMR1_InterruptHandler(), above.
 * It's purpose is to implement "non-blocking" time delays and event timers.
 *
 * Typical usage:
 *
 *    static unsigned long eventStartTime;
 *                  :
 *    eventStartTime = millisecondTimer();  // capture the starting time
 *                  :
 *    if ((millisecondTimer() >= (eventStartTime + EVENT_DURATION))  // time's up!
 *    {
 *        // Do what needs to be done TIME_DURATION ms after eventStartTime
 *    }
 *
 * A program can implement many independent event timers, simply by declaring
 * a unique eventStartTime (variable) and a unique EVENT_DURATION (constant)
 * for each independent "event" or delay to be timed.
 *
 * Be sure to declare each eventStartTime as 'static' (permanent) so that it
 * will be kept between multiple calls to the function in which it is defined.
 */
uint32  millisecTimer(void)
{
    uint32  check_time;
    
    // Disable Timer1 IRQ temporarily to prevent possible data corruption by the ISR  
    PIE1bits.TMR1IE = 0; 
    check_time = v_RTI_tick_counter;
    PIE1bits.TMR1IE = 1;
    
    return check_time;
}


/**
  Function:
    void  Delay_x10us(uint16 count_10us)

  Description:
    This routine performs a software delay in units of 10 microseconds,
    (longer if it gets interrupted).  

  Parameters:
    uint8 count_10us = delay time duration, unit = 10us, maximum 250 x10us.

*/
void  Delay_x10us(uint8 count_10us)
{
    while (count_10us-- != 0)
    {
        DELAY_1us(); DELAY_1us(); DELAY_1us(); DELAY_1us();
        DELAY_1us(); DELAY_1us(); DELAY_1us(); DELAY_1us();
    }
}


/**
  Function:
    void  Delay_ms(uint8 time_ms)

  Description:
    This routine performs a software delay in units of milliseconds.
    For use in peripheral initialisation, etc, where timer interrupts may be 
    disabled and blocking of other tasks doesn't matter.  

  Parameters:
    uint8 time_ms = delay time duration, unit = 1ms, maximum 250 ms

*/
void  Delay_ms(uint8 time_ms)
{
    while (time_ms-- != 0)
    {
        Delay_x10us(100);
    }
}


/**
  Function:
    int  strncomp(char *s1, char * s2, int len)

  Description:
    Does the same as strncmp(), but XC8 compiles this without error!
*/
int  strncomp(char *s1, char * s2, int len)
{
    while (len--) 
    {
	if (*s1 == 0 || *s1 != *s2)
	    return (unsigned char)*s1 - (unsigned char)*s2;
		
	s1++;
	s2++;
    }
    return 0;
}


void SYS_Initialize(void)
{
    INT_Initialize();
    PIN_Initialize();
    OSC_Initialize();
    TMR1_Initialize();  
    
//  SPI1_Initialize();
//  I2C2_Initialize();
//  EXT_INT_Initialize();
}


void OSC_Initialize(void)
{
    // SCS INTOSC; IRCF 16MHz_HFINTOSC; IDLEN disabled; 
    OSCCON = 0x72;
    // PRISD enabled; SOSCGO disabled; MFIOSEL disabled; 
    OSCCON2 = 0x04;
    // INTSRC disabled; PLLEN disabled; TUN 0; 
    OSCTUNE = 0x00;
}


void  INT_Initialize(void)
{
    // Disable Interrupt Priority Vectors (16CXXX Compatibility Mode)
    RCONbits.IPEN = 0;
}


void  __interrupt(high_priority)  INT_IRQ_Manager(void)
{
    if (INTCON3bits.INT1IE == 1 && INTCON3bits.INT1IF == 1)
    {
//      INT1_InterruptHandler();    // (not needed)
    }
/* 
    else if(INTCONbits.PEIE == 1 && PIE3bits.BCL2IE == 1 && PIR3bits.BCL2IF == 1)
    {
        I2C2_BusCollisionHandler();    // (not implemented)
    }
    else if(INTCONbits.PEIE == 1 && PIE3bits.SSP2IE == 1 && PIR3bits.SSP2IF == 1)
    {
        I2C2_InterruptHandler();    // (not implemented)
    }
*/
    else if (INTCONbits.PEIE == 1 && PIE1bits.TMR1IE == 1 && PIR1bits.TMR1IF == 1)
    {
        TMR1_InterruptHandler();
    }
}


void  PIN_Initialize(void)
{
    LATE = 0x00;    
    LATD = 0x00;    
    LATA = 0x00;    
    LATB = 0x00;    
    LATC = 0x00;    

    TRISE = 0x07;
    TRISA = 0x3F;   // 0011 1111
    TRISB = 0xFF;
    TRISC = 0x90;   // 1001 0000
    TRISD = 0xB7;   // 1011 0111
    
    ANSELC = 0x00;  
    ANSELB = 0xFF; 
    ANSELD = 0x00;
    ANSELE = 0x07;
    ANSELA = 0x2F;  

    WPUB = 0xFF;
    INTCON2bits.nRBPU = 0;
    
    MRF_CS_HIGH();
    MRF_RESET_LO();
}       


/*
 * Function to initialise the ADC for general-purpose analogue input readings.
 * ADC clock freq. and acquisition time are optimized for accuracy and speed.
 * 
 * Inputs AN10/RB1, AN11/RB4, AN12/RB0, AN13/RB5 are configured as analog inputs.
 */
void  ADC_Initialize(void)
{
    CTMUCONHbits.CTMUEN = 0;     // Disable CTMU !!
    DELAY_1us();
    
    ADCON0 = 0x00;               // GO_nDONE stop; ADC disabled; CHS AN0
    ADCON1 = 0x00;               // TRIGSEL CCP5; NVCFG -Ref = VSS; PVCFG +Ref = VDD
    ADCON2bits.ADFM = 1;         // Right justify result (use MSB and LSB)
    ADCON2bits.ADCS = 5;         // ADC clock = F_OSC / 16; (T_AD = 1us)
    ADCON2bits.ACQT = 2;         // Acq. time = T_AD x 4 = 4 us
    ADRESL = 0x00;               // Reset ADC read sequence
    ADRESH = 0x00;
    ADCON0 = 0x01;               // Enable ADC
    
    ADC10_SET_AS_INPUT();        // Configure req'd pins for ADC input
    ADC11_SET_AS_INPUT();
    ADC12_SET_AS_INPUT();
    ADC13_SET_AS_INPUT();
}    

/*
 * Function to start ADC conversion on a specified channel and read the result.
 * Entry arg is ADC input channel, e.g. chan 12 -> AN12, chan 13 -> AN13.
 * 
 * Return value is the 10-bit result, right justified in a 16-bit word.
 */
uint16  ADC_ReadInput(uint8 chan)
{
    uint16  result = 0;
   
    ADCON0bits.CHS = chan;         // select ADC channel
    DELAY_1us();                   // Delay for ADC MUX to settle (~ 2us ?)
    DELAY_1us();
    ADCON0bits.GO_nDONE = 1;       // Set GO bit to begin A/D conversion
    
    while (ADCON0bits.GO_nDONE)  { /* Wait for A/D conversion done */ ; }
    
    result = ADRESH;
    result = result << 8;          // ADC result, shift High byte
    result = result + ADRESL;      // ADC result, add Low byte
    
    return  result;
}    


// Call-back function for "reset" command.
//
void  BootReset(void)
{
    RESET();
}


/*`````````````````````````````````````````````````````````````````````````````````````
 * These functions maintain configuration data in a reserved block (64 bytes)
 * of flash (program) memory.  (The PIC18..K22 data EEPROM is unusable!)
 * 
 * The reserved 64-byte block is located at FLASH_DATA_BLOCK_BEG (see low-level.h)
 *`````````````````````````````````````````````````````````````````````````````````````
 * 
 * Function  FlashReadByte() reads a byte from flash at the given memory address.
 * The available flash address range is 0 to (FLASH_DATA_BLOCK_BEG + 63).
 */
uint8  FlashReadByte(uint16 faddr)
{
    TBLPTRU = 0x00;
    TBLPTRH = HI_BYTE(faddr);
    TBLPTRL = LO_BYTE(faddr);
    asm("TBLRD");
    return  TABLAT;
}

/*  
 * Function  FlashReadData() copies a number of bytes from flash program memory
 * into data RAM.
 * 
 * Entry arg's:  pdat = pointer to data RAM (destination of copy)
 *               faddr = beginning address in flash PM
 *               nbytes = number of bytes to copy
 */
void  FlashReadData(uint8 *pdat, uint16 faddr, uint8 nbytes)
{
    TBLPTRU = 0x00;
    TBLPTRH = HI_BYTE(faddr);
    TBLPTRL = LO_BYTE(faddr);
    
    while (nbytes--)
    {
        asm("TBLRD*+");
        *pdat++ = TABLAT;
    }
}

/*  
 * Function FlashWriteBlock() copies a chunk of 64 bytes from data RAM 
 * into a block of flash program memory. 
 * 
 * Entry arg's:  pdat = pointer to data RAM (source of data to write)
 *               baddr = beginning address of block in flash PM
 */
void  FlashWriteBlock(uint8 *pdat, uint16 baddr)
{
    uint8   nbytes = 64;
    
    baddr = baddr & 0xFFC0;    // adjust to block boundary
    FlashEraseBlock(baddr);
    
    TBLPTRU = 0x00;
    TBLPTRH = HI_BYTE(baddr);
    TBLPTRL = LO_BYTE(baddr);
    
    while (nbytes--)
    {
        TABLAT = *pdat++;
        asm("TBLWT*+");
    }
    
    TBLPTRL = LO_BYTE(baddr);  // reset table pointer
    
    EECON1bits.EEPGD = 1;      // Execute block program sequence
    EECON1bits.CFGS = 0;
    EECON1bits.WREN = 1;
    EECON1bits.FREE = 0;
    INTCONbits.GIE = 0;
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1;
    ////////
    INTCONbits.GIE = 1;
    EECON1bits.WREN = 0;
}

/*  
 * Function FlashEraseBlock() erases a block of flash program memory. 
 * 
 * Entry arg's:  baddr = beginning address of block in flash PM
 */
void  FlashEraseBlock(uint16 baddr)
{
    TBLPTRU = 0x00;
    TBLPTRH = HI_BYTE(baddr);
    TBLPTRL = LO_BYTE(baddr);
    
    EECON1bits.EEPGD = 1;      // Execute block erase sequence
    EECON1bits.CFGS = 0;
    EECON1bits.WREN = 1;
    EECON1bits.FREE = 1;       // Set Erase Enable bit (auto cleared)
    INTCONbits.GIE = 0;
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1;
    ////////
    INTCONbits.GIE = 1;
    EECON1bits.WREN = 0;
}


// end of file
