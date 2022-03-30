/**
 * File:       EUSART_drv.c
 *
 * Overview:   PIC18F peripheral driver module supporting EUSART1 and EUSART2
 *             in standard asynchronous (UART) mode.
 * 
 * Author:     M.J.Bauer  2017  [www.mjbauer.biz]
 */
#include <stdlib.h>
#include <string.h>

#include "gendef.h"
#include "EUSART_drv.h"

static volatile uint8 dummy;

//========================================================================================
//
// Initialize the serial port on EUSART1.
// Entry arg: Baudrate (e.g. 300, 1200, 9600, 19200, 31250, 38400, 57600)
// No parity, 8 data bits, 1 stop bit, Polled Tx/Rx (Non-interrupt)
//
void  EUSART1_init(uint16 baudrate)
{
    uint16  brg = F_OSC_Hz / ((uint32)baudrate * 4) - 1;  // baud rate register value
    
    // ABDOVF no_overflow; CKTXP async_noninverted_sync_fallingedge; BRG16 16bit_generator; 
    // WUE disabled; ABDEN disabled; DTRXP not_inverted; 
    BAUDCON1 = 0x08;
    // SPEN enabled; RX9 8-bit; CREN enabled; ADDEN disabled; SREN disabled; 
    RCSTA1 = 0x90;
    // TX9 8-bit; TX9D 0; SENDB sync_break_complete; TXEN enabled; SYNC asynchronous; 
    // BRGH hi_speed; CSRC slave_mode; 
    TXSTA1 = 0x24;
 
    SPBRG1 = LO_BYTE(brg);
    SPBRGH1 = HI_BYTE(brg); 
}

/**
*   EUSART1_RxDataAvail() - Checks EUSART receive buffer for data available.
*
*   Returns:  TRUE if EUSART Rx buffer contains unread char(s), else FALSE.
*/
uint8  EUSART1_RxDataAvail()
{
    uint8  result = PIR1bits.RC1IF;
    
    if (RCSTA1bits.OERR)  // EUSART1 RX error - reset
    {
        RCSTA1bits.SPEN = 0; 
        RCSTA1bits.SPEN = 1;
        result = 0;
    }
    
    return  result;
}

/**
*   EUSART#_RxFlush() - Clears EUSART hardware Rx FIFO buffer.
*
*   Returns:  --
*/
void  EUSART1_RxFlush()
{
    while (EUSART1_RxDataAvail())  // Clear Rx FIFO Buffer
    {
        EUSART1_ReadByte();
    }
}

/**
*   EUSART1_ReadByte() - Fetches next unread byte from EUSART RX buffer.
*
*   The function DOES NOT WAIT for data available in the input buffer;
*   the caller should first check using the function EUSARTx_RXdataAvail().
*   If there is no data available, the function returns NUL (0).
*   The input char is NOT echoed back to the EUSART output stream.
*
*   Returns:    Byte from EUSART RX buffer (or 0, if buffer is empty).
*/
uint8  EUSART1_ReadByte()
{
    uint8  b = 0;

    if (EUSART1_RxDataAvail()) b = RCREG1;
    
    return  b;
}

/**
*   EUSART1_WriteByte() - Outputs a byte (arg1) to the EUSART TX register, if empty.
*
*   The function waits for the TX register to be cleared first.
*
*   Returns:  byte sent (arg1).
*/
uint8  EUSART1_WriteByte(uint8 b)
{
    while (PIR1bits.TX1IF == 0) { /* wait for TX reg empty */ };
	
    TXREG1 = b;
    
    return  b;
}

/**
|  EUSART1_WriteString() -- Output a NUL-terminated string.
|
|  The string is expected to be in the data memory (RAM) space.
|  Newline (0x0A) is expanded to CR + LF (0x0D + 0x0A).
*/
void  EUSART1_WriteString( char *pstr )
{
    char c;

    while ( (c = *pstr++) != '\0' )
    {
        if ( c == '\n' )
        {
            EUSART1_WriteByte( '\r' );
            EUSART1_WriteByte( '\n' );
        }
        else   EUSART1_WriteByte( c );
    }
}


//========================================================================================
//
// Initialize the serial port on EUSART2.
// Entry arg: Baudrate (e.g. 300, 1200, 9600, 19200, 31250, 38400, 57600)
// No parity, 8 data bits, 1 stop bit, Polled Tx/Rx (Non-interrupt)
//
void  EUSART2_init(uint16 baudrate)
{
    uint16  brg = F_OSC_Hz / ((uint32)baudrate * 4) - 1;  // baud rate register value
    
    // ABDOVF no_overflow; CKTXP async_noninverted_sync_fallingedge; BRG16 16bit_generator; 
    // WUE disabled; ABDEN disabled; DTRXP not_inverted; 
    BAUDCON2 = 0x08;
    // SPEN enabled; RX9 8-bit; CREN enabled; ADDEN disabled; SREN disabled; 
    RCSTA2 = 0x90;
    // TX9 8-bit; TX9D 0; SENDB sync_break_complete; TXEN enabled; SYNC asynchronous; 
    // BRGH hi_speed; CSRC slave_mode; 
    TXSTA2 = 0x24;

    SPBRG2 = LO_BYTE(brg);
    SPBRGH2 = HI_BYTE(brg); 
}

/**
*   EUSART#_RxDataAvail() - Checks EUSART receive buffer for data available.
*
*   Returns:  TRUE if EUSART Rx buffer contains unread char(s), else FALSE.
*/
uint8  EUSART2_RxDataAvail()
{
    uint8  result = PIR3bits.RC2IF;
    
    if (RCSTA2bits.OERR)  // EUSART1 RX error - reset
    {
        RCSTA2bits.SPEN = 0; 
        RCSTA2bits.SPEN = 1;
        result = 0;
    }
    
    return  result;
}

/**
*   EUSART#_RxFlush() - Clears EUSART hardware Rx FIFO buffer.
*
*   Returns:  --
*/
void  EUSART2_RxFlush(void)
{
    while (EUSART2_RxDataAvail())  // Clear Rx FIFO Buffer
    {
        EUSART2_ReadByte();
    }
}

/**
*   EUSART2_ReadByte() - Fetches next unread char from EUSART RX input buffer.
*
*   The function DOES NOT WAIT for data available in the input buffer;
*   the caller should first check using the function EUSARTx_RXdataAvail().
*   If there is no data available, the function returns NUL (0).
*   The input char is NOT echoed back to the EUSART output stream.
*
*   Returns:    Byte from EUSART RX input buffer (or 0, if buffer is empty).
*/
uint8  EUSART2_ReadByte()
{
    uint8  b = 0;

    if (EUSART2_RxDataAvail()) b = RCREG2;
    
    return  b;
}

/**
*   EUSART2_WriteByte() - Outputs a byte (arg1) to the EUSART TX register, if empty.
*
*   The function waits for the TX register to be cleared first.
*
*   Returns:  byte sent (arg1).
*/
uint8  EUSART2_WriteByte(uint8 b)
{
    while (PIR3bits.TX2IF == 0) { /* wait for TX reg empty */ };
	
    TXREG2 = b;
    
    return  b;
}

/**
|  EUSART2_WriteString() -- Output a NUL-terminated string.
|
|  The string is expected to be in the data memory (RAM) space.
|  Newline (0x0A) is expanded to CR + LF (0x0D + 0x0A).
*/
void  EUSART2_WriteString(char *str)
{
    char c;

    while ( (c = *str++) != '\0' )
    {
        if ( c == '\n' )
        {
            EUSART2_WriteByte( '\r' );
            EUSART2_WriteByte( '\n' );
        }
        else   EUSART2_WriteByte( c );
    }
}

// end of file
