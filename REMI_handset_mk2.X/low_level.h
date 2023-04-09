/**
 *  File:  low_level.h
 *
 *  Note:  Definitions in this file are hardware platform-specific.
 *
 */
#ifndef LOW_LEVEL_H
#define LOW_LEVEL_H

#include <xc.h>
#include <stdbool.h>
#include <stdint.h>
#include "gendef.h"              // MJB's common typedefs, macro defs, etc
#include "I2C2_drv.h"

#define F_OSC_MHz    (16)        // MCU clock freq. (MHz)
#define _XTAL_FREQ  16000000     // K22 instruction cycle time is 4 x OSC period.

#ifdef _PIC18F25K22_H_
#define FLASH_DATA_BLOCK_BEG   0x7F80
#endif
#ifdef _PIC18F44K22_H_
#define FLASH_DATA_BLOCK_BEG   0x3F80
#endif
#ifdef _PIC18F45K22_H_
#define FLASH_DATA_BLOCK_BEG   0x7F80
#endif
#ifdef _PIC18F46K22_H_
#define FLASH_DATA_BLOCK_BEG   0xFF80
#endif

#define PITCH_BEND_SENSOR_ADC_CHAN   10  // RB1/AN10
#define PRESSURE_SENSOR_ADC_CHAN     11  // RB4/AN11
#define BATTERY_VOLTAGE_ADC_CHAN     12  // RB0/AN12
#define MODULATION_SENSOR_ADC_CHAN   13  // RB5/AN13

// Delay for one microsecond @ F_OSC = 16 MHz (Note: T_CYCLE = T_OSC * 4):
#define DELAY_1us()   NOP(); NOP(); NOP(); NOP();

#define ADC08_SET_AS_INPUT()  TRISBbits.TRISB2 = 1;  \
                              ANSELBbits.ANSB2 = 1;
#define ADC09_SET_AS_INPUT()  TRISBbits.TRISB3 = 1;  \
                              ANSELBbits.ANSB3 = 1;
#define ADC10_SET_AS_INPUT()  TRISBbits.TRISB1 = 1;  \
                              ANSELBbits.ANSB1 = 1;
#define ADC11_SET_AS_INPUT()  TRISBbits.TRISB4 = 1;  \
                              ANSELBbits.ANSB4 = 1;
#define ADC12_SET_AS_INPUT()  TRISBbits.TRISB0 = 1;  \
                              ANSELBbits.ANSB0 = 1;
#define ADC13_SET_AS_INPUT()  TRISBbits.TRISB5 = 1;  \
                              ANSELBbits.ANSB5 = 1;

#define TIMER1_IRQ_ENABLE()     (PIE1bits.TMR1IE = 1)
#define TIMER1_IRQ_DISABLE()    (PIE1bits.TMR1IE = 0)
#define TIMER1_START()          (T1CONbits.TMR1ON = 1)
#define TIMER1_STOP()           (T1CONbits.TMR1ON = 0)

#define LED1_INIT()     TRISCbits.TRISC0 = 0
#define LED1_OFF()      LATCbits.LATC0 = 0
#define LED1_ON()       LATCbits.LATC0 = 1
#define LED1_TOGGLE()   LATCbits.LATC0 ^= 1

#define LED2_INIT()     TRISCbits.TRISC1 = 0
#define LED2_OFF()      LATCbits.LATC1 = 0
#define LED2_ON()       LATCbits.LATC1 = 1
#define LED2_TOGGLE()   LATCbits.LATC1 ^= 1

#define BUTTON1_INPUT()      (PORTDbits.RD4)      // Read state of BUTTON.1 pin
#define SWITCH1_INPUT()      (PORTDbits.RD4)      // Read state of SWITCH.1 pin
#define BUTTON2_INPUT()      (PORTDbits.RD5)      // Read state of BUTTON.2 pin
#define SWITCH2_INPUT()      (PORTDbits.RD5)      // Read state of SWITCH.2 pin
#define READ_BUTTON_INPUTS() ((PORTD >> 4) & 3)   // Read both switch inputs
#define MASK_BUTTON_A        0x01
#define MASK_BUTTON_B        0x02

#define MRF_CS_HIGH()        LATCbits.LATC2 = 1
#define MRF_CS_LOW()         LATCbits.LATC2 = 0   // MRF_CS# is active low
#define MRF_SLEEP()          LATDbits.LATD3 = 0
#define MRF_WAKE()           LATDbits.LATD3 = 1   // MRF_WAKE is active high
#define MRF_RESET_HI()       LATDbits.LATD3 = 1
#define MRF_RESET_LO()       LATDbits.LATD3 = 0   // MRF_RESET# is active low
#define MRF_READ_INT()       (PORTDbits.RD2)      // Read state of MRF_INT# pin

#define GlobalInterruptEnable()       (INTCONbits.GIE = 1)
#define GlobalInterruptDisable()      (INTCONbits.GIE = 0)
#define PeripheralInterruptEnable()   (INTCONbits.PEIE = 1)
#define PeripheralInterruptDisable()  (INTCONbits.PEIE = 0)

#ifndef TESTPOINT_TP2_SET_HI
#define TESTPOINT_TP2_CONFIG()    TRISAbits.TRISA6 = 0
#define TESTPOINT_TP2_SET_HI()    LATAbits.LATA6 = 1
#define TESTPOINT_TP2_SET_LO()    LATAbits.LATA6 = 0
#endif

#define milliseconds()    millisecTimer()   // alias

// Global signals for Background Task Executive (in main.c)
extern volatile bool   v_RTI_flag_1ms_task;
extern volatile bool   v_RTI_flag_5ms_task;
extern volatile bool   v_RTI_flag_50ms_task;
extern volatile uint8  v_TaskOverrunCount;   // Task overruns (max. 100 per 100ms)

// Functions defined in low-level.c
//
void  TMR1_Initialize(void);
void  TMR1_InterruptHandler(void);
uint32  millisecTimer(void);

void  Delay_x10us(uint8 count_10us);
void  Delay_ms(uint8 time_ms);

int   strncomp(char *s1, char *s2, int len);

void  PIN_Initialize(void);
void  TMR1_Initialize(void);
void  TMR1_InterruptHandler(void);
void  SYS_Initialize(void);
void  OSC_Initialize(void);

void  INT_Initialize (void);
void  __interrupt(high_priority) INT_IRQ_Manager(void);

void  ADC_Initialize(void);
uint16  ADC_ReadInput(uint8 chan);

uint8 FlashReadByte(uint16 addr);
void  FlashReadData(uint8 *pdat, uint16 faddr, uint8 nbytes);
void  FlashEraseBlock(uint16 baddr);
void  FlashWriteBlock(uint8 *pdat, uint16 baddr);

uint8 MMA8451_Setup(uint8 accel_FS);
void  MMA8451_RegisterWrite(uint8 reg, uint8 bDat);
uint8 MMA8451_RegisterRead(uint8 reg);

void  BootReset(void);

#endif // LOW_LEVEL_H
