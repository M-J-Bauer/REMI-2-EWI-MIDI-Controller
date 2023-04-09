/*****************************************************************************\
 * Filename:        I2C2_drv.c
 *
 * Originated:      08/08/2017   Krystian Jagoda  [krystianjagoda@gmail.com]
 * Modified:        27/02/2023   Michael J Bauer  [www.mjbauer.biz]
 *
 * Architecture:    Advanced 8-Bit PIC MCUs
 * Processor:       PIC18FxxK22
 * Compiler:        MPLAB XC8 v2.50++
 *
 * Description:     Simple I2C functions (I2C channel: 2)
 *
 \****************************************************************************/
#include <xc.h>
#include "I2C2_drv.h"

void I2C2_init(char speed) 
{
    SSP2CON1 = 0x28;            // enable I2C Master mode
    SSP2CON2 = 0x00;            // clear control bits
    SSP2STAT = 0x80;            // disable slew rate control; disable SMBus
    if (speed == I2C_RATE_400K) 
        SSP2ADD  = 0x09;        // BaudRate = 400kHz  (@ F_OSC = 16MHz)
    else  SSP2ADD  = 0x27;      // BaudRate = 100kHz  (@ F_OSC = 16MHz)
    PIR3bits.SSP2IF = 0;        // clear IRQ
    PIR3bits.BCL2IF = 0;        // clear BCL flag
    SSP2CON2bits.SEN = 0;       // force idle condition
}


void I2C2_start(void) 
{
    PIR3bits.SSP2IF = 0;        // clear flag
    while (SSP2STATbits.BF );   // wait for idle condition
    SSP2CON2bits.SEN = 1;       // initiate START condition
    while (!PIR3bits.SSP2IF) ;  // wait for a flag to be set
    PIR3bits.SSP2IF = 0;        // clear flag
}


void I2C2_repStart(void) 
{
    PIR3bits.SSP2IF = 0;        // clear flag
    while ( SSP2STATbits.BF );  // wait for idle condition
    SSP2CON2bits.RSEN = 1;      // initiate Repeated START condition
    while (!PIR3bits.SSP2IF);   // wait for a flag to be set
    PIR3bits.SSP2IF = 0;        // clear flag
}


void I2C2_stop(void) 
{
    PIR3bits.SSP2IF = 0;        // clear flag
    while ( SSP2STATbits.BF );  // wait for idle condition
    SSP2CON2bits.PEN = 1;       // Initiate STOP condition
    while (!PIR3bits.SSP2IF);   // wait for a flag to be set
    PIR3bits.SSP2IF = 0;        // clear flag
}


unsigned char I2C2_write( unsigned char i2cWritedata ) 
{
    PIR3bits.SSP2IF = 0;        // clear interrupt
    while ( SSP2STATbits.BF );  // wait for idle condition
    SSP2BUF = i2cWritedata;     // Load SSPBUF with i2cWritedata (the value to be transmitted)
    while (!PIR3bits.SSP2IF);   // wait for a flag to be set
    PIR3bits.SSP2IF = 0;        // clear flag

    return ( !SSP2CON2bits.ACKSTAT ); // function returns '1' if transmission is acknowledged
}


unsigned char I2C2_read( unsigned char sendAck ) 
{
    unsigned char i2cReaddata;

    PIR3bits.SSP2IF = 0;        // clear interrupt
    while ( SSPSTATbits.BF ) ;  // wait for idle condition
    SSP2CON2bits.RCEN = 1;      // enable receive mode
    while (!PIR3bits.SSP2IF) ;  // wait for a flag to be set
    PIR3bits.SSP2IF = 0;        // clear flag
    i2cReaddata = SSP2BUF;      // Read SSPBUF and put it in i2cReaddata
	
    if (sendAck)  { SSP2CON2bits.ACKDT = 0; }  // send ACK  
    else  { SSP2CON2bits.ACKDT = 1; }  // send NAK 

    SSP2CON2bits.ACKEN = 1;     // send acknowledge sequence
    while (!PIR3bits.SSP2IF) { ; }   // wait for IRQ flag
    PIR3bits.SSP2IF = 0;        // clear flag

    return( i2cReaddata );      // return the value read from SSPBUF
}
