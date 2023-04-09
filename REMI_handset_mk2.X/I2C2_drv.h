/*****************************************************************************\
 * Filename:        I2C2_drv.h
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
#ifndef PIC18_I2C_H
#define	PIC18_I2C_H

#include <xc.h>

#define I2C_RATE_100K  'L'  // low speed (standard))
#define I2C_RATE_400K  'H'  // high speed (preferred))

#define I2C_NAK  0
#define I2C_ACK  1


/********************************************************************
* Function Name: I2C2_init
* Return Value: void
* Parameters:  speed (bit rate) = I2C_RATE_100K or I2C_RATE_400K
* Description: Initialize the PIC18 MSSP2 module in master mode
* Note:        Assume system clock F_OSC = 16MHz
********************************************************************/
void I2C2_init(char speed);



/********************************************************************
* Function Name: I2C2_start
* Return Value: void
* Parameters: void
* Description: Send I2C Start Command
********************************************************************/
void I2C2_start(void);



/********************************************************************
* Function Name: I2C2_repStart
* Return Value: void
* Parameters: void
* Description: Resend I2C Start Command
********************************************************************/
void I2C2_repStart(void);



/********************************************************************
* Function Name: I2C2_stop
* Return Value: void
* Parameters: void
* Description: Send I2C Stop command
********************************************************************/
void I2C2_stop(void);



/********************************************************************
* Function Name: I2C2_write
* Return Value: Status byte for WCOL detection.
* Parameters: Single data byte for I2C2 bus.
* Description: This routine writes a single byte to the
* I2C2 bus.
********************************************************************/
unsigned char I2C2_write( unsigned char i2cWritedata );



/********************************************************************
* Function Name: I2C2_read
* Return Value: contents of SSP2BUF register
* Parameters: ack = 1 and nak = 0
* Description: Read a byte from I2C bus and ACK/NAK device
********************************************************************/
unsigned char I2C2_read( unsigned char ack );


#endif	// PIC18_I2C_H
