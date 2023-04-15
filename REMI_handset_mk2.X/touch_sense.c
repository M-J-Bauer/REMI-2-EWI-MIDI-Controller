/*^^^^
 * File:       touch_sense.c
 *
 * Overview:   PIC18FxxK22 peripheral driver module supporting CTMU and ADC
 *             for capacitive touch-pad sense operation.
 *
 * Note:       Macro definitions in header file "touch_sense.h" must be customized 
 *             to suit the touch-pad pin assignments (ADC inputs) for the application.
 *
 * Author:     M.J.Bauer  2018++  [www.mjbauer.biz]
 */
#include <xc.h>
#include "low_level.h"
#include "touch_sense.h"

PRIVATE  void  TouchPadReadCycle(uint8 pad);

static  uint8   m_TouchADCchannel[] = TOUCH_PAD_ADC_CHANNELS;
static  uint8   m_ADCRawResult[TOUCH_PADS_MAX];     // raw ADC readings from last scan
static  uint16  m_FilteredReading[TOUCH_PADS_MAX];  // Filter accum. readings, fixed-pt (x256)
static  uint16  m_TouchThreshold;                   // touch on/off threshold value
static  uint16  m_TouchPadStates;                   // de-glitched on/off states (bitwise)

/*
 * Function:   TouchSenseInit()
 *
 * Initializes the CTMU and ADC for touch-pad sense operation.
 */
void  TouchSenseInit()
{
    CTMUCONHbits.CTMUEN = 0;     // CTMU disabled
    CTMUCONHbits.CTMUSIDL = 0;   // CTMU continues to run in idle mode
    CTMUCONHbits.TGEN = 0;       // disable edge delay generation mode
    CTMUCONHbits.EDGEN = 0;      // edges are blocked
    CTMUCONHbits.EDGSEQEN = 0;   // edge sequence not needed
    CTMUCONHbits.IDISSEN = 0;    // Do not ground the current source
    CTMUCONHbits.CTTRIG = 0;     // Trigger Output is disabled
    CTMUCONLbits.EDG2POL = 0;
    CTMUCONLbits.EDG2SEL = 0;    // Edge2 Src = ECCP2 (don't care)
    CTMUCONLbits.EDG1POL = 1;
    CTMUCONLbits.EDG1SEL = 1;    // Edge1 Src = ECCP1 pin (don't care)

    CTMUICONbits.IRNG = 3;       // Current range:  I = 55uA (+/-20%)
    CTMUICONbits.ITRIM = 0;      // Trim:  No adjustment

    ADCON0 = 0x00;               // GO_nDONE stop; ADC disabled; CHS AN0
    ADCON1 = 0x00;               // TRIGSEL CCP5; NVCFG -Ref = VSS; PVCFG +Ref = VDD
    ADCON2bits.ADFM = 0;         // Left justify result (use MSB only)
//  ADCON2bits.ADCS = 5;         // ADC clock = F_OSC / 16; (T_AD = 1us)
    ADCON2bits.ADCS = 1;         // ADC clock = F_OSC / 8; (T_AD = 0.5us)
//  ADCON2bits.ACQT = 1;         // Acq. time = T_AD x 2
    ADCON2bits.ACQT = 2;         // Acq. time = T_AD x 4

    ADRESL = 0x00;               // Reset ADC read sequence
    ADRESH = 0x00;
    ADCON0 = 0x01;               // Enable ADC
    CTMUCONHbits.CTMUEN = 1;     // Enable CTMU
    
    m_TouchThreshold = TOUCH_THRESHOLD_DEFAULT;
    m_TouchPadStates = 0x000;
    
    TESTPOINT_TP2_CONFIG();      // Optional 'scope trigger signal
}


/*
 * Function:  TouchPadsGetNumber()
 *
 * Return the number of touch pads in active service.
 */
uint16  TouchPadsGetNumber(void)
{
    return  (TOUCH_PADS_MAX);
}


/*
 * Function:  TouchPadGetRawADC()
 *
 * Return raw ADC conversion result from last scan of the given pad (arg1).
 * Intended for calibration & diagnostic purposes.
 */
uint16  TouchPadGetRawADC(uint8 pad)
{
    return  m_ADCRawResult[pad];
}


/*
 * Function:  TouchPadGetFiltered()
 *
 * Return filtered ADC reading for the given pad (arg1).
 * Intended for calibration & diagnostic purposes.
 * Values stored in the array m_FilteredReading[] are filter accumulator
 * values in fixed-point format (LS 8 bits = fractional part).
 * The (truncated) integer part is returned.
 */
uint16  TouchPadGetFiltered(uint8 pad)
{
    return  (m_FilteredReading[pad] >> 8);
}


/*
 * Function:  TouchPadStates()   
 *
 * Return de-glitched touch-pad states, one bit per pad,
 * packed into a short integer (max. 16 pads).
 */
uint16  TouchPadStates()
{
    return  m_TouchPadStates;
}


/*
 * Function:  TouchPadSetThreshold()   
 *
 * Set value of m_TouchThreshold.
 */
void  TouchPadSetThreshold(uint8 adc_count)
{
    m_TouchThreshold = adc_count;
}


/*
 * Function:  TouchPadScan()
 *
 * This routine is called at regular intervals to measure the capacitance on all
 * of the touch-pad inputs. Raw ADC readings are stored in array: m_ADCRawResult[].
 * Readings from the first scan (following MCU reset) are discarded.
 * After the first scan, all touch-pads are left grounded. During the scan routine,
 * one pad at a time is configured for touch sense while the others remain grounded.
 * 
 * Scan total execution time is (TOUCH_PADS_MAX * 30) microseconds, approx.
 */
void  TouchPadScan()
{
    static bool init_done = 0;
    static uint8  TouchOnCount[TOUCH_PADS_MAX];   // count of touch 'ON' (low) readings
    static uint8  TouchOffCount[TOUCH_PADS_MAX];  // count of touch 'OFF' (high) readings
    uint8  pad;
    
    TouchSenseInit();
            
    GlobalInterruptDisable();   // Suspend IRQ's during time-critical process
    TESTPOINT_TP2_SET_HI();
    
#if (TOUCH_PADS_MAX >= 1)
    TOUCH_PAD_00_SET_AS_INPUT();
    TouchPadReadCycle(0);
    TOUCH_PAD_00_DISCHARGE();
    DELAY_1us();
#endif
#if (TOUCH_PADS_MAX >= 2)
    TOUCH_PAD_01_SET_AS_INPUT();
    TouchPadReadCycle(1);
    TOUCH_PAD_01_DISCHARGE();
    DELAY_1us();
#endif
#if (TOUCH_PADS_MAX >= 3)
    TOUCH_PAD_02_SET_AS_INPUT();
    TouchPadReadCycle(2);
    TOUCH_PAD_02_DISCHARGE();
    DELAY_1us();
#endif
#if (TOUCH_PADS_MAX >= 4)
    TOUCH_PAD_03_SET_AS_INPUT();
    TouchPadReadCycle(3);
    TOUCH_PAD_03_DISCHARGE();
    DELAY_1us();
#endif
#if (TOUCH_PADS_MAX >= 5)
    TOUCH_PAD_04_SET_AS_INPUT();
    TouchPadReadCycle(4);
    TOUCH_PAD_04_DISCHARGE();
    DELAY_1us();
#endif
#if (TOUCH_PADS_MAX >= 6)
    TOUCH_PAD_05_SET_AS_INPUT();
    TouchPadReadCycle(5);
    TOUCH_PAD_05_DISCHARGE();
    DELAY_1us();
#endif
#if (TOUCH_PADS_MAX >= 7)
    TOUCH_PAD_06_SET_AS_INPUT();
    TouchPadReadCycle(6);
    TOUCH_PAD_06_DISCHARGE();
    DELAY_1us();
#endif
#if (TOUCH_PADS_MAX >= 8)
    TOUCH_PAD_07_SET_AS_INPUT();
    TouchPadReadCycle(7);
    TOUCH_PAD_07_DISCHARGE();
    DELAY_1us();
#endif
#if (TOUCH_PADS_MAX >= 9)
    TOUCH_PAD_08_SET_AS_INPUT();
    TouchPadReadCycle(8);
    TOUCH_PAD_08_DISCHARGE();
    DELAY_1us();
#endif
#if (TOUCH_PADS_MAX >= 10)
    TOUCH_PAD_09_SET_AS_INPUT();
    TouchPadReadCycle(9);
    TOUCH_PAD_09_DISCHARGE();
    DELAY_1us();
#endif
    
    TESTPOINT_TP2_SET_LO();
    GlobalInterruptEnable();   // Allow IRQ's again
    
    if (!init_done)   // first pass after reset...  exit now.
    {
        init_done = TRUE;
        return;
    }
/*    
    // Apply 1st-order IIR filter to raw ADC readings...   |  (Not needed.)
    // Time constant is (64 / scan_period) ms = 64ms if scan_period is 1 ms.
    for (pad = 0 ; pad < TOUCH_PADS_MAX ; pad++)
    {
        m_FilteredReading[pad] -= m_FilteredReading[pad] >> 6;  // (accum / 64)
        m_FilteredReading[pad] += m_ADCRawResult[pad] << 2;     // > (raw_rdg * 256) / 64
    }
*/    
    // Update pad on/off states according to threshold value...
    //
    for (pad = 0 ; pad < TOUCH_PADS_MAX ; pad++)
    {
        if (m_ADCRawResult[pad] < m_TouchThreshold)  // Pad is touched
        { 
            TouchOffCount[pad] = 0;
            if (TouchOnCount[pad] < READING_STABLE_COUNT) TouchOnCount[pad]++;
            else  SET_BIT(m_TouchPadStates, pad);   // register 'ON' state
        }
        else    // Pad is not touched
        { 
            TouchOnCount[pad] = 0;
            if (TouchOffCount[pad] < READING_STABLE_COUNT) TouchOffCount[pad]++;
            else  CLEAR_BIT(m_TouchPadStates, pad);   // register 'OFF' state
        }
    }
}


/*
 * Function:  TouchPadReadCycle()
 *
 * Perform a touch-sense read cycle on the specified pad (arg1).
 */
PRIVATE  void  TouchPadReadCycle(uint8 pad)
{
    ADCON0bits.CHS = m_TouchADCchannel[pad];  // select ADC channel for pad
    DELAY_1us();     // Delay for ADC MUX to settle (~ 1us)
 
    CTMUCONHbits.IDISSEN = 1;      // Drain charge on the A/D circuit
    DELAY_1us(); 
    CTMUCONHbits.IDISSEN = 0;      // Stop discharge of A/D circuit
    ;       
    CTMUCONLbits.EDG2STAT = 0;     // Make sure edge2 is 0
    CTMUCONLbits.EDG1STAT = 1;     // Set edge1 - Start Charge
    DELAY_1us();                   // Allow CTMU charge time (~ 3us)
    DELAY_1us();
    DELAY_1us();
    CTMUCONLbits.EDG1STAT = 0;     // Clear edge1 - Stop Charge
    ADCON0bits.GO_nDONE = 1;       // Set GO to begin A/D conversion
    
    while (ADCON0bits.GO_nDONE)  { ; /* Wait for A/D conversion done */ }
   
    m_ADCRawResult[pad] = ADRESH;  // fetch the result
}

// end of file
