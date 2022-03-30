/**
  MSSP1 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    SPI_drv.c

  @Summary
    This is the generated driver implementation file for the MSSP1 driver using PIC10 / PIC12 / PIC16 / PIC18 MCUs 

  @Description
    This source file provides APIs for MSSP1.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs  - 1.45
        Device            :  PIC18F45K22
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.40
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/
#include <xc.h>
#include "SPI_drv.h"

/**
  Section: Macro Declarations
*/

#define SPI_RX_IN_PROGRESS 0x0

/**
  Section: Module APIs
*/

/**
 * Function:      Initialize and enable MSSP1 for SPI Master Mode operation.
 *                Clock frequency is fixed at FOSC/4.
 * 
 * Entry arg(s):  uint8 ckmode   = SPI clock mode (0..3), see table below
 * 
 * CKE specifies clock edge to begin TX cycle: 0 => Idle-to-Active, 1 => Active-to-Idle
 * CKP specifies clock polarity in Idle state: 0 => Low, 1 => High
 * 
 *     CK mode   |  CKP  |  CKE
 *   ------------+-------+--------
 *     0  (0,0)  |   0   |   1
 *     1  (0,1)  |   0   |   0
 *     2  (1,0)  |   1   |   1
 *     3  (1,1)  |   1   |   0
 * 
 */
void SPI1_Initialize(uint8_t ckmode)
{
    SSP1CON1 = 0x00;  // Disable MSSP1 to reset it
            
    if (ckmode & 1)  SSP1STAT = 0x00;   // CKE = 0
    else  SSP1STAT = 0x40;   // CKE = 1
    
    // Set SCK polarity and enable MSSP1 peripheral
    if (ckmode & 2)  SSP1CON1 = 0x30;   // CKP = 1
    else  SSP1CON1 = 0x20;   // CKP = 0
}


uint8_t SPI1_Exchange8bit(uint8_t data)
{
    // Clear the Write Collision flag, to allow writing
    SSP1CON1bits.WCOL = 0;

    SSP1BUF = data;

    while (SSP1STATbits.BF == SPI_RX_IN_PROGRESS)  { /* WAIT */ }

    return (SSP1BUF);
}


uint8_t SPI1_Exchange8bitBuffer(uint8_t *dataIn, uint8_t bufLen, uint8_t *dataOut)
{
    uint8_t bytesWritten = 0;

    if(bufLen != 0)
    {
        if(dataIn != NULL)
        {
            while(bytesWritten < bufLen)
            {
                if(dataOut == NULL)
                {
                    SPI1_Exchange8bit(dataIn[bytesWritten]);
                }
                else
                {
                    dataOut[bytesWritten] = SPI1_Exchange8bit(dataIn[bytesWritten]);
                }

                bytesWritten++;
            }
        }
        else
        {
            if(dataOut != NULL)
            {
                while(bytesWritten < bufLen )
                {
                    dataOut[bytesWritten] = SPI1_Exchange8bit(DUMMY_DATA);

                    bytesWritten++;
                }
            }
        }
    }

    return bytesWritten;
}


bool SPI1_IsBufferFull(void)
{
    return (SSP1STATbits.BF);
}


bool SPI1_HasWriteCollisionOccured(void)
{
    return (SSP1CON1bits.WCOL);
}


void SPI1_ClearWriteCollisionStatus(void)
{
    SSP1CON1bits.WCOL = 0;
}

/**
 End of File
*/