/*^^^^
 * File:       touch_sense.h
 *
 * Overview:   PIC18FxxK22 peripheral driver module supporting CTMU and ADC
 *             for capacitive touch-pad sense operation.
 * 
 * Note:       Macro definitions in this header file must be customized to suit
 *             the touch-pad pin assignments (ADC inputs) for the application.
 *
 * Author:     M.J.Bauer  2018  [www.mjbauer.biz]
 */
#ifndef CTMU_TOUCH_H
#define CTMU_TOUCH_H

#include <xc.h>
#include <stdbool.h>
#include <stdint.h>

#include "gendef.h"

//========================================================================================
//        A P P L I C A T I O N - D E P E N D E N T   D E F I N I T I O N S
//        Customized to suit the hardware configuration of the application.
//        `````````````````````````````````````````````````````````````````
#define TOUCH_PADS_MAX            10   // Number of touch-pads serviced (0..10)
#define READING_STABLE_COUNT      4    // Successive stable readings to register on/off state
#define TOUCH_THRESHOLD_DEFAULT   150  // On/Off threshold, unit = ADC count

// Macro to assign an ADC channel to each of the touch-pads serviced.
// There must be at least TOUCH_PADS_MAX channel numbers in the list.
// Channels in the list are in order of touch-pad pin definitions below.
#define TOUCH_PAD_ADC_CHANNELS  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 }

// Macros to configure each touch-pad pin as output and set to logic LOW:
//
#define TOUCH_PAD_00_DISCHARGE()  ANSELAbits.ANSA0 = 0;  \
                                  LATAbits.LATA0 = 0;    \
                                  TRISAbits.TRISA0 = 0;
#define TOUCH_PAD_01_DISCHARGE()  ANSELAbits.ANSA1 = 0;  \
                                  LATAbits.LATA1 = 0;    \
                                  TRISAbits.TRISA1 = 0;
#define TOUCH_PAD_02_DISCHARGE()  ANSELAbits.ANSA2 = 0;  \
                                  LATAbits.LATA2 = 0;    \
                                  TRISAbits.TRISA2 = 0;
#define TOUCH_PAD_03_DISCHARGE()  ANSELAbits.ANSA3 = 0;  \
                                  LATAbits.LATA3 = 0;    \
                                  TRISAbits.TRISA3 = 0;
#define TOUCH_PAD_04_DISCHARGE()  ANSELAbits.ANSA5 = 0;  \
                                  LATAbits.LATA5 = 0;    \
                                  TRISAbits.TRISA5 = 0;
#define TOUCH_PAD_05_DISCHARGE()  ANSELEbits.ANSE0 = 0;  \
                                  LATEbits.LATE0 = 0;    \
                                  TRISEbits.TRISE0 = 0;
#define TOUCH_PAD_06_DISCHARGE()  ANSELEbits.ANSE1 = 0;  \
                                  LATEbits.LATE1 = 0;    \
                                  TRISEbits.TRISE1 = 0;
#define TOUCH_PAD_07_DISCHARGE()  ANSELEbits.ANSE2 = 0;  \
                                  LATEbits.LATE2 = 0;    \
                                  TRISEbits.TRISE2 = 0;
#define TOUCH_PAD_08_DISCHARGE()  ANSELBbits.ANSB2 = 0;  \
                                  LATBbits.LATB2 = 0;    \
                                  TRISBbits.TRISB2 = 0;
#define TOUCH_PAD_09_DISCHARGE()  ANSELBbits.ANSB3 = 0;  \
                                  LATBbits.LATB3 = 0;    \
                                  TRISBbits.TRISB3 = 0;

// Macros to configure each touch-pad pin as ADC analogue input:
//
#define TOUCH_PAD_00_SET_AS_INPUT()  TRISAbits.TRISA0 = 1;  \
                                     ANSELAbits.ANSA0 = 1;
#define TOUCH_PAD_01_SET_AS_INPUT()  TRISAbits.TRISA1 = 1;  \
                                     ANSELAbits.ANSA1 = 1;
#define TOUCH_PAD_02_SET_AS_INPUT()  TRISAbits.TRISA2 = 1;  \
                                     ANSELAbits.ANSA2 = 1;
#define TOUCH_PAD_03_SET_AS_INPUT()  TRISAbits.TRISA3 = 1;  \
                                     ANSELAbits.ANSA3 = 1;
#define TOUCH_PAD_04_SET_AS_INPUT()  TRISAbits.TRISA5 = 1;  \
                                     ANSELAbits.ANSA5 = 1;
#define TOUCH_PAD_05_SET_AS_INPUT()  TRISEbits.TRISE0 = 1;  \
                                     ANSELEbits.ANSE0 = 1;
#define TOUCH_PAD_06_SET_AS_INPUT()  TRISEbits.TRISE1 = 1;  \
                                     ANSELEbits.ANSE1 = 1;
#define TOUCH_PAD_07_SET_AS_INPUT()  TRISEbits.TRISE2 = 1;  \
                                     ANSELEbits.ANSE2 = 1;
#define TOUCH_PAD_08_SET_AS_INPUT()  TRISBbits.TRISB2 = 1;  \
                                     ANSELBbits.ANSB2 = 1;
#define TOUCH_PAD_09_SET_AS_INPUT()  TRISBbits.TRISB3 = 1;  \
                                     ANSELBbits.ANSB3 = 1;
//
//========================================================================================

void    TouchSenseInit();
uint16  TouchPadsGetNumber(void);
void    TouchPadScan();
void    TouchPadSetThreshold(uint8 adc_count);
uint16  TouchPadGetRawADC(uint8 pad);
uint16  TouchPadGetFiltered(uint8 pad);
uint16  TouchPadStates();


#endif  // CTMU_TOUCH_H
