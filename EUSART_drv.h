/**
 * File:       EUSART_drv.h
 *
 * Overview:   PIC18F peripheral driver def's supporting EUSART1 and EUSART2
 *             in standard asynchronous (UART) mode.
 */
#ifndef EUASRT1_DRV_H
#define EUASRT1_DRV_H

#include <xc.h>
#include <stdbool.h>
#include <stdint.h>

#define F_OSC_Hz  16000000UL  // MCU clock freq.

#define  EUSART1_TX_Ready()  (PIR1bits.TX1IF)
#define  EUSART2_TX_Ready()  (PIR3bits.TX2IF)

void   EUSART1_init(uint16 baud);
uint8  EUSART1_RxDataAvail();
void   EUSART1_RxFlush();
uint8  EUSART1_ReadByte();
uint8  EUSART1_WriteByte(uint8 b);
void   EUSART1_WriteString(char *str);

void   EUSART2_init(uint16 baud);
uint8  EUSART2_RxDataAvail();
void   EUSART2_RxFlush();
uint8  EUSART2_ReadByte();
uint8  EUSART2_WriteByte(uint8 b);
void   EUSART2_WriteString(char *str);

#endif // EUASRT1_DRV_H
