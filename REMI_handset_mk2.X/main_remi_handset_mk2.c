/* ================================================================================================
 *
 * FileName:    main_remi_handset_mk2.c
 * `````````
 * Overview:    Firmware for Bauer "REMI" handset mk2 -- EWI MIDI Controller
 * `````````    `````````````````````````````````````````````````````````````
 * Processor:   PIC18F45K22 or PIC18F25K22  (specified in build options)
 * ``````````
 * Hardware:    REMI mk2 micro-controller board (PIC18..K22)
 * `````````    
 * Compiler:    Microchip MPLAB XC8 v2.31++ (free version) under MPLAB'X IDE v5.45
 * `````````    
 * Notes:       Set up XC8 Linker to exclude on-chip ROM (flash PM) at 0x7F80-0x7FFF.
 * ``````       This space is reserved to store user configuration data (from g_Config).
 * 
 * Originated:  2019  M.J.Bauer  [www.mjbauer.biz]
 * ```````````
 * ================================================================================================
 */
#include <xc.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h> 
#include "main_remi_handset_mk2.h"


// Functions private within this module..........
//
PRIVATE  void   APP_Initialize();
PRIVATE  void   HeartbeatLEDtask();
PRIVATE  void   SendPitchBendUpdate();
PRIVATE  void   PitchBendAutoZeroTask();
PRIVATE  void   SendModulationUpdate();
PRIVATE  void   NoteOnOffStateTask();
PRIVATE  void   ReadSensorsTask();
PRIVATE  void   CalcMidiPressureLevel();
PRIVATE  void   PresetButtonMonitor();
PRIVATE  void   PresetButtonActionTask();
PRIVATE  void   ModeSwitchCheck();
PRIVATE  void   SendRemiIdentMessage();


// ----------------  Global data  -------------------------------
//
uint8   g_FW_version[4];          // firmware version # (major, minor, build, 0)
char   *g_AppTitleCLI;            // Title string output by CLI "ver" command
bool    g_HeartbeatLEDactive;
bool    g_DiagnosticModeActive;
int     g_DiagnosticModeTimeout;  // minutes
uint32  g_DiagModeTimer_ms;
uint8   g_SelfTestErrors;         // Self-test fault codes (0 => no fault)

Config_Params_t  g_Config;

// ----------------  Private data  ------------------------------
//
static  uint16  m_TouchPadStates;
static  uint8   m_NoteOnOffState;
static  uint8   m_ModeSwitchState;
static  uint8   m_OperatingMode;
static  uint16  m_TimeSinceLastNoteOff_ms;

static  uint16  m_PressureSensorReading;
static  uint16  m_PressureQuiescent;
static  uint16  m_PressureThresholdNoteOn;
static  uint16  m_PressureThresholdNoteOff;
static  uint8   m_Pressure_Hi;
static  uint8   m_Pressure_Lo;

static  uint16  m_ModulationSensorReading;
static  uint16  m_ModulationZeroLevel;
static  uint16  m_PitchBendSensorReading;
static  uint16  m_PitchBendZeroLevel;
static  int16   m_PitchTranspose;
static  uint16  m_BatteryVoltsRawReading;


//=================================================================================================

PRIVATE  void  APP_Initialize()
{
    g_FW_version[0] = BUILD_VER_MAJOR;
    g_FW_version[1] = BUILD_VER_MINOR;
    g_FW_version[2] = BUILD_VER_DEBUG;
    
    g_AppTitleCLI = "Bauer {REMI} Handset mk2 -- Service Port CLI \n";
    g_HeartbeatLEDactive = 1;        // 0 = disabled
    g_DiagnosticModeActive = 0;
    g_DiagnosticModeTimeout = 30;    // minutes
    g_SelfTestErrors = 0;
    
    LED1_INIT();
    LED2_INIT();
    EUSART1_init(38400);    // for service port (CLI)
    EUSART2_init(31250);    // for MIDI TX port
    TMR1_Initialize();      // Set up RTI Timer
    TouchSenseInit();
    
    GlobalInterruptEnable();
    PeripheralInterruptEnable();
}


void  main(void)
{
    static uint32 startupTime;
    static uint8  count;
    
    SYS_Initialize();   // Initialize the device
    APP_Initialize();

    // Send a few NUL chars to the USB-serial bridge to sync the baud-rate
    for (count = 0; count < 10; count++)  { EUSART1_WriteByte(0); }
    
    EUSART1_WriteString("\r\n* MCU reset *\n");
    
    if (FetchConfigData() == FALSE)  // Read Config data from Flash
    {
        g_SelfTestErrors |= (1 << TEST_CONFIG_INTEGRITY);
        DefaultConfigData();
    }
    
    TouchPadSetThreshold(g_Config.TouchSenseThreshold);

    // Measure the pressure sensor signal level -- loop for 200ms
    startupTime = millisecTimer();
    while (millisecTimer() < (startupTime + 200))
    {
        if (v_RTI_flag_5ms_task)
        {
            v_RTI_flag_5ms_task = 0;
            LED1_ON();
            ReadSensorsTask();
        }
    }
    
    // Determine the quiescent pressure level and note on/off thresholds
    m_PressureQuiescent = m_PressureSensorReading;
    m_PressureThresholdNoteOn = m_PressureQuiescent + (g_Config.PressureSensorSpan * 5) / 100;
    m_PressureThresholdNoteOff = m_PressureQuiescent + (g_Config.PressureSensorSpan * 4) / 100;
    
    if (m_PressureQuiescent < 8 || m_PressureQuiescent > 255)  // sensor fault
        g_SelfTestErrors |= (1 << TEST_PRESSURE_SENSOR);
    
    m_ModeSwitchState = SWITCH2_INPUT();  // State at power-on/reset
    
    EUSART1_WriteString(g_AppTitleCLI);   // Output CLI startup message
    PrepareForNewCommand();               // Initialize CLI (put prompt)

    while (1)   // loop forever
    {
        BackgroundTaskExec();
        
        ConsoleCLI_Service();
        
        MIDI_TxQueueHandler();
    }
}


/*
 * Background task executive...  
 * This routine runs periodic tasks scheduled by the RTI timer ISR.
 * Also dispatches any pending asynchronous (non-periodic, event-driven) tasks.
 * Called frequently from the main loop and from inside CLI wait loops.
 */
void  BackgroundTaskExec()
{
    static uint8  count_50ms;
    static bool   startMsgSent;
    
    if (v_RTI_flag_1ms_task)     // Do 1ms periodic task
    {
        v_RTI_flag_1ms_task = 0;
        TouchPadScan();
    }
    
    if (v_RTI_flag_5ms_task)     // Do 5ms periodic tasks
    {
        v_RTI_flag_5ms_task = 0;
        ReadSensorsTask();
        m_TouchPadStates = TouchPadStates() & 0x3FF;
        CalcMidiPressureLevel();
        NoteOnOffStateTask();
        PresetButtonMonitor();
    }

    if (v_RTI_flag_50ms_task)    // Do 50ms periodic tasks
    {
        v_RTI_flag_50ms_task = 0;
        LED1_ON();
        HeartbeatLEDtask(); 
        PitchBendAutoZeroTask();

        if (++count_50ms == 4)   // Do 200ms periodic tasks
        {
            count_50ms = 0;
//          ModeSwitchCheck();   // (disabled, pending functional improvement)
            SendRemiIdentMessage();
        }
    }
}


/*
 * Function:     HeartbeatLEDtask()
 *
 * Overview:     Pulses the "system heartbeat" LED at 2Hz (100ms duty).
 *               Periodic B/G task called at 50ms intervals.
 *               Activate using CLI "diag" cmd.
 */
PRIVATE  void  HeartbeatLEDtask()
{
    static short LED_cycle = 0;  // unit = 50ms

    if (g_HeartbeatLEDactive)
    {
        if (LED_cycle >= 10) LED_cycle = 0;
        if (LED_cycle == 0) LED2_ON();
        if (LED_cycle == 2) LED2_OFF();
        LED_cycle++;
    }
}


/**
 * Function:     Get note on/off state.
 *
 * Return val:   0 = IDLE, 1 = PENDING, 2 = IN PROGRESS
 */
uint16  GetNoteOnOffState()
{
    return  m_NoteOnOffState;
}


/**
 * Function:     Get handset PRESET button state.
 *
 * Return val:   (bool) TRUE: button is pressed, FALSE: not pressed
 *               NB: The raw button state (pin RD4) is active low.
 */
bool  isPresetButtonPressed()
{
    return  (BUTTON1_INPUT() == 0);
}


/**
 * Function:     Get raw ADC count value of breath pressure sensor.
 *
 * Return val:   (uint16) ADC count value, 10 bits (0..1023)
 */
uint16  GetPressureRawReading()
{
    return  m_PressureSensorReading;
}


/**
 * Function:     Get pressure value sent in MIDI expression message.
 *
 * Return val:   (uint16) MIDI data value, 14 bits, range 0..16256
 */
uint16  GetMidiPressureLevel()
{
    return  ((uint16)m_Pressure_Hi << 7) + m_Pressure_Lo;
}


/**
 * Function:     Get raw ADC count value of Modulation sensor.
 *
 * Return val:   (uint16) ADC count value, 10 bits (0..1023)
 */
uint16  GetModulationRawReading()
{
    return  m_ModulationSensorReading;
}


/**
 * Function:     Get raw ADC count value of Pitch Bend sensor.
 *
 * Return val:   (uint16) ADC count value, 10 bits (0..1023)
 */
uint16  GetPitchBendRawReading()
{
    return  m_PitchBendSensorReading;
}


/**
 * Function:    Find MIDI note number from key/fingering pattern,
 *              adjusted by the "Pitch Offset" (transpose) parameter value.
 *
 * Entry arg:   (uint16) fingerPattern = Touch-pad electrode bits, incl. octave pads.
 *              Fingering format is dependent on handset variant -- see table below.
 *
 * Return val:  (uint8) MIDI note number between 23 (B0) and 108 (C8).
 *
 * ````````````````````````````````````````````````````````````````````````````````````````
 * Touch-pad configuration for "Bauer EWI" (recorder-like) fingering scheme:
 *
 *                       |  octave   | ----------- s e m i t o n e ----------- | LH4 |
 *    Finger position -> | OT1 | OT2 | LH1 | LH2 | LH3 | RH1 | RH2 | RH3 | RH4 | RH5 |
 *    PIC18__K22 ADC# -> |  9  |  8  |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
 *
 * ````````````````````````````````````````````````````````````````````````````````````````
 */
PRIVATE  uint8  NoteNumberFromKeyPattern(uint16 fingerPattern)
{
    // MIDI note number from finger pattern, before LH4/RH5 and octave selection applied.
    static  uint8   baseNoteNumberLUT[] =
    {
    // RH1..RH4 finger pattern (4 LS bits) =
    // 0000 0001 0010 0011 0100 0101 0110 0111 1000 1001 1010 1011 1100 1101 1110 1111
        37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37, // LH = 000
        48,  48,  48,  48,  48,  48,  48,  48,  47,  46,  46,  46,  45,  44,  44,  44, // LH = 001
        36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36, // LH = 010
        43,  42,  42,  42,  42,  42,  42,  42,  41,  41,  41,  41,  40,  39,  38,  36, // LH = 011
        35,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34, // LH = 100
        34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34, // LH = 101
        33,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32, // LH = 110
        31,  30,  30,  30,  30,  30,  30,  30,  29,  29,  29,  29,  28,  27,  26,  24  // LH = 111
    };

    // Table indicates if a given note must be flattened when pad RH5 is touched.
    // Exception: If LH pads == 010 (C" alt), do not flatten!
    // Table index is note number from baseNoteNumberLUT[] minus 24; i.e. range 0..24.
    static  bool   noteFlattenByRH5[] =
    {
    // 24  25  26  27  28  29  30  31  32  33  34  35  <-- base note #
        1,  0,  1,  0,  1,  0,  0,  1,  0,  1,  0,  1,
    // 36  37  38  39  40  41  42  43  44  45  46  47  48
        1,  0,  1,  0,  1,  0,  0,  1,  0,  1,  0,  1,  0
    };
    
    int16   noteNumber;  // return value
    uint8   baseNote, octave, padLH4RH5;
    uint8   top3Fingers, top7Fingers;
    uint8   octavePads  = (fingerPattern >> 8) & 3;    // Extract OCT1, OCT2

    octavePads = (fingerPattern >> 8) & 3;
    if (octavePads == 2) octave = 2;        // Upper pad => highest octave
    else if (octavePads == 3) octave = 1;   // Both pads => middle octave
    else  octave = 0;                       // Bottom pad => lowest octave

    top3Fingers = (fingerPattern >> 5) & 0x07;   // Get LH1..LH3 into bits 2,1,0
    top7Fingers = (fingerPattern >> 1) & 0x7F;   // Drop LH4/RH5 and strip octave pads
    padLH4RH5 = fingerPattern & 1;
    
    baseNote = baseNoteNumberLUT[top7Fingers];   // Note number before octave applied
    noteNumber = baseNote + (octave * 12) + 24;  // Normal range is 48 (C3) to 96 (C7)

    if (g_Config.FingeringScheme == KEY_SCHEME_RH5_FLATTENS)
    {
        if (padLH4RH5 && (top3Fingers != 2) && noteFlattenByRH5[baseNote-24])
            noteNumber--;  // Flatten the note (^see comment above)
    }
    else  // (g_Config.FingeringScheme == KEY_SCHEME_LH4_SHARPENS)
    {
        // Check notes which must be sharpened if LH4 is touched (C and F only)
        if (baseNote == 24 || baseNote == 29 || baseNote == 36 ||  baseNote == 41 || baseNote == 48)
        {
            if (padLH4RH5) noteNumber++;  // Sharpen the note
        }
    }
    
    // Adjust noteNumber using 'Pitch Offset/Transpose' parameter (+/- 24 semitones)
    noteNumber = (int16) noteNumber + m_PitchTranspose;

    return  (uint8) noteNumber;
}


/*****
 * Function:      Determine 14-bit pressure level from the pressure sensor raw ADC reading.
 *                May be used also to find MIDI velocity value from pressure.
 *
 * Outputs:       (uint8) m_Pressure_Hi = MIDI pressure High byte, or Velocity (0..127)
 *                (uint8) m_Pressure_Lo = MIDI pressure Low byte (0..127)
 *
 * Note:          The function assumes a linear relationship between raw pressure reading
 *                and MIDI pressure (or velocity) value. The external MIDI sound module
 *                should apply an exponential transfer function between MIDI pressure and
 *                audio output amplitude (to compensate for perceived loudness).
 */
PRIVATE  void  CalcMidiPressureLevel()
{
    int32   level;
    uint16  rawSpan = g_Config.PressureSensorSpan;  // unit = ADC counts

    // Calculate pressure as a 14-bit quantity, range 0 to 16256 (127 x 128)
    level = (16256 * (int32)(m_PressureSensorReading - m_PressureQuiescent)) / rawSpan;

    if (level < 0) level = 0;
    if (level > 16256)  level = 16256;

    m_Pressure_Hi = ((uint16) level >> 7) & 0x7F;     // High-order 7 bits
    m_Pressure_Lo = (uint8) level & 0x7F;   // Low-order 7 bits
}


/*^
 * Function:  NoteOnOffStateTask()
 *
 * Background task (state machine) executed every 5 milliseconds.
 *
 * The function monitors data acquired by the touch-pad sensor routine, looking for
 * any change in status that signals an "event", e.g. Note-On, Note-Off, Note-Change,
 * and sends MIDI commands accordingly.
 *
 * The task also sends MIDI Expression (Pressure), Modulation and Pitch-Bend messages
 * at the time intervals configured.
 */
PRIVATE  void  NoteOnOffStateTask()
{
    static  uint32  stateTimer_ms;
    static  uint32  PressureUpdateTimer_ms;
    static  uint32  controllerUpdateTimer_ms;
    static  uint16  LastPressure;
    static  uint8   noteNumPlaying;
    
    uint16  fingerPattern = m_TouchPadStates;
    uint8   noteNumber = NoteNumberFromKeyPattern(fingerPattern);
    uint8   top3Fingers = (fingerPattern >> 5) & 7;  // Get LH1..LH3 into bits 2,1,0
    uint8   octavePads  = (fingerPattern >> 8) & 3;  // Extract OCT+, OCT-
    bool    isValidNote = (octavePads != 0) && (top3Fingers != 0);
    uint8   channel = g_Config.MidiBasicChannel;
    bool    sendNoteOn = 0;
    uint16  pressure_14b = ((uint16)m_Pressure_Hi << 7) + m_Pressure_Lo;
    uint8   velocity = m_Pressure_Hi; 
    uint8   exprnCC = g_Config.MidiExpressionCCnumber;
    
    switch (m_NoteOnOffState)
    {
    case NOTE_OFF_IDLE:
    {
        // A new note is triggered when the pressure sensor signal rises above the 
        // NoteOnPressureThreshold (raw ADC count).
        if (isValidNote && (m_PressureSensorReading >= m_PressureThresholdNoteOn))
        {
            stateTimer_ms = 0;    // start delay for velocity acquisition
            m_NoteOnOffState = NOTE_ON_PENDING;
        }
 
        if (m_TimeSinceLastNoteOff_ms < 50000) m_TimeSinceLastNoteOff_ms += 5;
        break;
    }
    case NOTE_ON_PENDING:
    {
        m_TimeSinceLastNoteOff_ms = 0;

        if (!g_Config.VelocitySenseEnabled)  // use fixed velocity value
        {
            velocity = 64;  
            sendNoteOn = 1;
        }
        else if (stateTimer_ms >= NOTE_ON_VELOCITY_DELAY)  // ready to acquire velocity
        {
            velocity = m_Pressure_Hi;  // capture the pressure reading
            sendNoteOn = 1;
        }
        // else... Don't send Note-On and remain in this state.

        if (sendNoteOn)
        {
            MIDI_SendNoteOn(channel, noteNumber, velocity);
            noteNumPlaying = noteNumber;
            PressureUpdateTimer_ms = 0;
            controllerUpdateTimer_ms = 0;  
            m_NoteOnOffState = NOTE_ON_PLAYING;
        }
        break;
    }
    case NOTE_ON_PLAYING:
    {
        // A Note-off is sent when the pressure sensor signal falls below the Note-Off 
        // pressure threshold.
        if (m_PressureSensorReading < m_PressureThresholdNoteOff)
        {
            MIDI_SendNoteOff(channel, noteNumPlaying);
            m_NoteOnOffState = NOTE_OFF_IDLE; 
            break;
        }
        
        // Look for a change in fingering pattern with any valid note selected;
        // this signals a "Legato" note change.  Remain in this state.
        if (isValidNote && (noteNumber != noteNumPlaying))
        {
            if (g_Config.LegatoModeEnabled)  // Synth is MIDI compliant (in Mono)
            {
                MIDI_SendNoteOn(channel, noteNumber, velocity);  // new note on
                MIDI_SendNoteOff(channel, noteNumPlaying);       // old note off
            }
            else  // Synth is NOT MIDI compliant (and/or POLY mode enabled)
            {
                MIDI_SendNoteOff(channel, noteNumPlaying);       // old note off
                MIDI_SendNoteOn(channel, noteNumber, velocity);  // new note on
            }
            noteNumPlaying = noteNumber;
        }

        // If the pressure update interval (minimum) has expired, and the pressure
        // sensor reading has changed since the last update, send expression CC msg.
        if (PressureUpdateTimer_ms >= g_Config.MidiPressureInterval)
        {
            PressureUpdateTimer_ms = 0;
            if (pressure_14b != LastPressure)  // pressure reading has changed
            {
                if (g_Config.MidiExpressionCCnumber != 0)
                {
                    MIDI_SendControlChange(channel, exprnCC, m_Pressure_Hi);
                    if (g_Config.Send14bitExprnData) 
                        MIDI_SendControlChange(channel, (exprnCC + 32), m_Pressure_Lo);
                }
                LastPressure = pressure_14b;
            }
        }

        // If the modulation controller update interval (minimum) has expired,
        // send modulation CC msg and (if enabled) pitch-bend msg.
        if (controllerUpdateTimer_ms >= g_Config.MidiControllerInterval)
        {
            controllerUpdateTimer_ms = 0;
            SendModulationUpdate();
            if (g_Config.PitchBendEnabled)  SendPitchBendUpdate();
        }
        break;
    }
    default:
        m_NoteOnOffState = NOTE_OFF_IDLE;
        break;

    }  // end switch

    stateTimer_ms += 5;
    PressureUpdateTimer_ms += 5;
    controllerUpdateTimer_ms += 5;
}


/*
 * Pitch-Bend update routine 
 * 
 * Pitch Bend data size is 14 bits (range +/- 8000) for fine control of pitch variation.
 * The Pitch Bend control range is determined by the external MIDI synthesizer.
 *
 * A MIDI Pitch Bend message is sent only if the sensor reading has changed since
 * the last call to this function.
 */
PRIVATE  void  SendPitchBendUpdate()
{
    static  int16  lastValueSent;  // 14-bit signed number
    int16   currentValue;
    uint8   channel = g_Config.MidiBasicChannel;

    currentValue = GetPitchBendData();  

    if (currentValue != lastValueSent)
    {
        MIDI_SendPitchBend(channel, currentValue);
        lastValueSent = currentValue;
    }
}


/*
 * Modulation (Force Sensor) update routine 
 * 
 * A REMI configuration parameter, g_Config.MidiModulationCCnumber, specifies which
 * MIDI Control Change number is to be assigned to the Modulation Pad (default CC = 1).
 * The data sent has a 14-bit (2 byte) format, allowing fine control of the effect.
 *
 * A MIDI Modulation (Control Change) message is sent only if the Modulation level
 * has changed since the last call to this function.
 */
PRIVATE  void  SendModulationUpdate()
{
    static  uint16  lastValueSent;
    uint16  currentValue;
    uint8   channel = g_Config.MidiBasicChannel;
    uint8   ctrlNum = g_Config.MidiModulationCCnumber;
    uint8   dataMSB, dataLSB;

    if (ctrlNum == 0 || ctrlNum >= 0x20)   // Invalid CC# for Modulation
    {
        return;  // Can't send mod'n msg -- bail
    }

    currentValue = GetModulationPadForce();  

    if (currentValue != lastValueSent)
    {
        dataMSB = (currentValue >> 7) & 0x7F;   // send 7 MS bits of 14 bit data
        MIDI_SendControlChange(channel, ctrlNum, dataMSB);
        dataLSB = currentValue & 0x7F;   // send 7 LS bits of 14 bit data
        MIDI_SendControlChange(channel, (ctrlNum + 0x20), dataLSB);
        lastValueSent = currentValue;
    }
}


/*
 * Function monitors the PRESET button input, looking for changes in state.
 * Task called periodically at 5ms intervals from the main background loop.
 * 
 * If the button was pressed since the last call, PresetButtonActionTask() is called.
 * De-glitch, debounce and multiple-hit prevention filters are applied.
 * 
 */
PRIVATE  void  PresetButtonMonitor()
{
    static  uint8  lastState = 0;
    static  uint8  buttonPressTime_ms;
    static  uint8  buttonReleaseTime_ms;
    
    if (lastState == 0)  // Waiting for button hit
    {
        if (buttonPressTime_ms >= 50) 
        {
            PresetButtonActionTask();  // Button Hit detected!
            lastState = 1;
        }
    }
    else  // if (lastState == 1), wait for 200ms "hold-off" time-out
    {
        if (buttonReleaseTime_ms >= 200)  lastState = 0;
    }
     
    if (BUTTON1_INPUT() == 0)  // button is pressed
    {
        if (buttonPressTime_ms < 250) buttonPressTime_ms += 5;  
        buttonReleaseTime_ms = 0;
    }
    else  // button is released
    {
        if (buttonReleaseTime_ms < 250) buttonReleaseTime_ms += 5;
        buttonPressTime_ms = 0;  
    }
}

/*`````````````````````````````````````````````````````````````````````````````````````
 * Function performs any required action when a PRESET button hit is detected.
 * Background task executed every 5 milliseconds.
 * 
 * If the button press is detected while there is no note playing, then...
 *     If there is no touch-pad touched,
 *         then a MIDI Reset and sensor calibrate operations are performed;
 *     otherwise, if one or more top-surface pads are touched,
 *         then the PRESET selected by the (lowest) touch-pad is activated;
 *     otherwise, if the lower octave pad is touched,
 *         then the pitch offset is adjusted down 1 octave (-1);
 *     otherwise, if the upper octave pad is touched,
 *         then the pitch offset is adjusted up 1 octave (+1);
 *     otherwise, if both octave pads are touched together,
 *         then the pitch offset is reset to zero (0).
 * ------------------------------------------------------------------------------------
 */
PRIVATE  void   PresetButtonActionTask()
{
    uint8   pad, lower5pads, padLH4;
    uint8   preset;
    uint8   channel = g_Config.MidiBasicChannel;
    uint8   octvpads;    // Get 2 octave pads
    uint8   top8pads;  // Get 8 upper surface pads
    
    octvpads = m_TouchPadStates >> 8;      // Get 2 octave pads
    top8pads = m_TouchPadStates & 0xFF;    // Get 8 upper surface pads
    
    if (g_Config.TouchPadLayout == LAYOUT_WITH_PAD_AT_LH4)  // Alt. layout...
    {
        // At this point top8pads has bit0 = LH4/RH5. Need to get physical pad layout.
        padLH4 = (top8pads << 4) & 0x10;   // move bit0 into LH4 posn (bit4)
        lower5pads = padLH4 | ((top8pads >> 1) & 0x0F);   // Add 4 LS bits (RH1..RH4)
        top8pads = (top8pads & 0xE0) | lower5pads;      // Add 3 MS bits (LH1..LH3)
    }

    if (m_NoteOnOffState == NOTE_OFF_IDLE)  // No note playing now
    {
        if (m_TouchPadStates == 0) // No pads touched -- send MIDI reset, etc
        {
            MIDI_SendAllSoundOff(channel);
            MIDI_SendResetAllControllers(channel);

            if (g_Config.LegatoModeEnabled)
                MIDI_SendReceiverMode(channel, OMNI_ON_MONO);
            
            // Re-calibrate pressure sensor, assuming pressure equilibrium...
            m_PressureQuiescent = m_PressureSensorReading;
            m_PressureThresholdNoteOn = m_PressureQuiescent + 30;
            m_PressureThresholdNoteOff = m_PressureQuiescent + 25;
        }
        else if (top8pads != 0x00)  // One or more top pad(s) touched - Set Preset
        {
            for (preset = 1, pad = 0;  pad < 8;  pad++, preset++)
            {
                if (preset == 8) preset = 0;
                if (top8pads & 0x01)  break;  // found lowest pad touched
                top8pads = top8pads >> 1;
            }
            InstrumentPresetSelect(preset);  // Activate new Preset
        }
        else if (octvpads == 1)  m_PitchTranspose = -12;  // 1 octave lower
        else if (octvpads == 2)  m_PitchTranspose =  12;  // 1 octave higher
        else if (octvpads == 3)  m_PitchTranspose = 0;    // No pitch offset
    }
}

/*
 * Function:  Instrument PRESET selection.
 * 
 * A MIDI 'Channel Program Select' message is sent to the receiving device with the
 * Program Number (byte) set to whatever value is configured for the active preset, i.e.
 *     g_Config.PresetMidiProgram[preset].
 * 
 * Note: MIDI 'Channel Program' messages are ignored by the REMI Synth Module.
 * 
 * If MIDI System Exclusive messages are enabled...
 * A REMI "PRESET" message is sent to the REMI Synth Module. The message contains
 * the active Preset number (0..7) plus the handset firmware version number, a byte
 * containing self-test error flags, plus various handset configuration parameters.
 */
void  InstrumentPresetSelect(uint8 preset)
{
    static uint8  RemiPresetMsg[16];
    uint8  i, dat;
    uint8  chan = g_Config.MidiBasicChannel;
    uint8  progNum = g_Config.PresetMidiProgram[preset];
    
    // Send General MIDI program change command, if enabled. (ignored by REMI synth) 
    if (g_Config.MidiProgChangeEnabled) MIDI_SendProgramChange(chan, progNum);
    
    // Construct MIDI system exclusive 'PRESET' message...
    RemiPresetMsg[0]  = SYS_EXCLUSIVE_MSG;  // status byte
    RemiPresetMsg[1]  = SYS_EXCL_REMI_ID;   // manuf/product ID
    RemiPresetMsg[2]  = REMI_PRESET_MSG;    // Msg type
    RemiPresetMsg[3]  = preset;             // selected preset # (0..7)
    RemiPresetMsg[4]  = g_FW_version[0];
    RemiPresetMsg[5]  = g_FW_version[1];
    RemiPresetMsg[6]  = g_FW_version[2];
    RemiPresetMsg[7]  = g_SelfTestErrors;
    RemiPresetMsg[8]  = g_Config.FingeringScheme;
    RemiPresetMsg[9]  = g_Config.LegatoModeEnabled;
    RemiPresetMsg[10] = g_Config.VelocitySenseEnabled;
    RemiPresetMsg[11] = SYSTEM_MSG_EOX;     // end-of-msg code
    
    if (g_Config.MidiSysExclMsgEnabled)  
    {
        // Send REMI 'PRESET' msg
        for (i = 0 ; i < SYS_EXCL_MSG_MAX_LENGTH ; i++)
        {
            dat = RemiPresetMsg[i];
            MIDI_PutByte(dat);
            if (dat == SYSTEM_MSG_EOX) break;
        }
    }
}


/*
 * Function:  Send MIDI System Exclusive message -- 'REMI IDENT'
 * 
 * If MIDI System Exclusive messages are allowed...
 * A REMI "IDENT" message is sent periodically to the REMI Synth Module,
 * at intervals not exceeding 250ms, for the purpose of identifying the REMI handset
 * as the controller connected to the synth MIDI IN port.
 */
PRIVATE  void  SendRemiIdentMessage()
{
    if (g_Config.MidiSysExclMsgEnabled)
    {
        MIDI_PutByte(SYS_EXCLUSIVE_MSG);   // status byte
        MIDI_PutByte(SYS_EXCL_REMI_ID);    // manuf/product ID
        MIDI_PutByte(REMI_IDENT_MSG);      // Msg type
        MIDI_PutByte(SYSTEM_MSG_EOX);      // end-of-msg code
    }
}


/*
 * Function monitors the MODE switch, looking for a change in state which,
 * if detected, will result in the new state being recorded and actioned.
 * Task called periodically from the main background loop, if a MODE switch is fitted.
 * 
 * The MODE switch selects a configuration profile to suit either the REMI synth
 * or a "generic" MIDI synth.  Config parameters will remain unchanged until the
 * mode switch is toggled (after handset power-on/reset) or if a parameter is modified
 * using the "config" command.
 */
PRIVATE  void   ModeSwitchCheck()
{
    if (m_ModeSwitchState != SWITCH2_INPUT())  // Switch has been toggled
    {
        if (SWITCH2_INPUT() == 0)  SetConfigProfile(GEN_SYNTH_MODE);
        else  SetConfigProfile(REMI_SYNTH_MODE);
    }
    
    m_ModeSwitchState = SWITCH2_INPUT();
}


/*-------------------------------------------------------------------------------------------------
 *  Functions to support analogue sensor reading
 *-------------------------------------------------------------------------------------------------
 * 
 * Function:  ReadSensorsTask()
 *
 * Background task executed every 5 milliseconds.
 *
 * This task acquires raw ADC readings from various sensors on the handset, e.g.
 * Breath Pressure Sensor, Modulation Lever/Pad, Pitch-Bend Sensor (where fitted).
 * Also reads the battery voltage (divider) input, for the wireless handset.
 */
PRIVATE  void   ReadSensorsTask()
{
    ADC_Initialize();
    
    m_PressureSensorReading = ADC_ReadInput(11);
    m_PitchBendSensorReading = ADC_ReadInput(10);
    m_ModulationSensorReading = ADC_ReadInput(13);
    m_BatteryVoltsRawReading = ADC_ReadInput(12);
}


/*
 * Function:  GetModulationPadForce()
 *
 * Return value:  Unsigned integer in the range 0 ~ 16256 (14 bits)
 *                representing the force applied to the Modulation Pad.
 *
 * Note:  g_Config.ModulationMaximum is the maximum value of m_ModulationSensorReading.
 * 
 * A "dead-band" is deliberately introduced at zero so that a minimum force needs
 * to be applied to the sensor before a non-zero reading is obtained.
 * A square-law transform is applied to the reading to compensate for non-linearity
 * in the FSR-402 characteristic.
 */
uint16   GetModulationPadForce()
{
    int32   linearVal;    // uncompensated reading, linear scale
    int32   compVal;      // compensated for non-linearity and dead-band
    int32   rawSpan;

    linearVal = m_ModulationSensorReading - g_Config.ModulationDeadband;
    if (linearVal < 0) linearVal = 0;
    
    // Scale linearVal such that it spans the full 14 bit range (0 ~ 16256)...
    rawSpan = g_Config.ModulationMaximum - g_Config.ModulationDeadband;
    linearVal = (16256 * linearVal) / rawSpan;
    
    compVal = (linearVal * linearVal) / 16256;   // Apply square-law transform
    if (compVal > 16256)  compVal = 16256;       // Cap at 16256

    return  (uint16) compVal;
}


/* Function:  PitchBendAutoZeroTask() 
 *
 * Background task executed every 50 milliseconds.
 *
 * If at least 1 second has elapsed since the last Note-Off event and there has been no
 * subsequent Note-On event, the function performs an "auto-zero" operation on the
 * Pitch-Bend sensor reading.
 *
 * The last raw ADC reading taken is used to set the Pitch-Bend zero level in preparation
 * for the next note to be played. The task will be executed repetitively until the next 
 * Note-On event occurs.
 * 
 */
PRIVATE  void   PitchBendAutoZeroTask()
{
    if (m_TimeSinceLastNoteOff_ms > 1000 && !g_DiagnosticModeActive)
    {
        m_PitchBendZeroLevel = m_PitchBendSensorReading;  // ADC count at "zero" posn
    }
}


/*
 * Function:  GetPitchBendData()
 *
 * Return value:  Signed integer in the range +/- 8000 representing the displacement
 *                of the Pitch-Bend lever while a note is playing or diagnostic mode 
 *                is active; otherwise zero (0).
 *
 * Note:  If a Note-On event is pending, i.e. if m_NoteOnOffState != NOTE_OFF_IDLE,
 *        then m_TimeSinceLastNoteOff_ms = 0.  The function cannot return a Pitch Bend
 *        value when there is no note playing (except while diagnostic mode is active)
 *        because the auto-zero routine is active in the note-off/idle state.
 * 
 */
int16  GetPitchBendData()
{
    int32  linearVal;
    int32  compVal = 0;

    if (m_TimeSinceLastNoteOff_ms == 0 || g_DiagnosticModeActive)
    {
        linearVal = 16000 * (int32)(m_PitchBendSensorReading - m_PitchBendZeroLevel);
        linearVal = linearVal / g_Config.PitchBendSpan;

//      compVal = (linearVal * linearVal) / 16000;  // Square-law response
//      if (linearVal < 0) compVal = 0 - compVal;
        
        compVal = linearVal;   // Linear response
    }

    return  (int16) compVal;
}


/*-------------------------------------------------------------------------------------------------
 *  Functions to support persistent data storage in a flash memory block.
 *-------------------------------------------------------------------------------------------------
 *
 *  <!> Configuration data will be erased whenever a firmware update is performed.
 *      Firmware updates will cause config. param's to be defaulted.
 *      ````````````````````````````````````````````````````````````
 * 
 *  Function copies data from flash block to a RAM buffer where persistent data
 *  can be accessed by the application. If the operation is successful, the return
 *  value will TRUE; otherwise FALSE.
 * 
 *  ToDo:  Fetch one of two sets of param's, depending on MODE switch state.  <<<<<<<<<<<<<<>
 * 
 *  Return val:  TRUE if the stored data verified OK, otherwise FALSE. 
 */
bool  FetchConfigData() 
{
    bool    result = TRUE;
    uint16  *pWord = (uint16 *) &g_Config;  // first 2 bytes of Config data

    FlashReadData((uint8 *) &g_Config, FLASH_DATA_BLOCK_BEG, sizeof(g_Config));
    
    if (*pWord == 0xFFFF) result = FALSE; 
    if (CheckConfigData() != g_Config.CheckSum)  result = FALSE;
     
    return result; 
}

/*
 * Function writes default values for configuration data in flash memory.
 * These are "factory" defaults which are applied only in the event of erasure or
 * corruption of flash memory data, including firmware update.
 * 
 * ToDo: This function is to be revised such that there are two sets of config. parameters, 
 *       one for each state of the MODE switch.  Both sets are to be defaulted as required.
 */
void  DefaultConfigData(void)
{
    static const  uint8  MidiProgramDefault[] = 
                   { 67, 75, 72, 69, 74, 17, 20, 23 };  
    //   Preset #: [  8,  1,  2,  3,  4,  5,  6,  7 ]
    
    uint8   i;
    
    g_Config.MidiBasicChannel = 1;
    g_Config.MidiSysExclMsgEnabled = 0;
    g_Config.MidiProgChangeEnabled = 0;
    g_Config.MidiExpressionCCnumber = 2; 
    g_Config.MidiModulationCCnumber = 1;
    g_Config.MidiPressureInterval = 5;
    g_Config.MidiControllerInterval = 30;  
    
    g_Config.Send14bitExprnData = 0; 
    g_Config.LegatoModeEnabled = 0; 
    g_Config.VelocitySenseEnabled = 0;
    g_Config.PitchBendEnabled = 0;
    g_Config.TouchPadLayout = LAYOUT_WITH_PAD_AT_RH5;
    g_Config.FingeringScheme = KEY_SCHEME_RH5_FLATTENS;
    g_Config.TouchSenseThreshold = TOUCH_THRESHOLD_DEFAULT;
    
    for (i = 0 ; i < 8 ; i++)
    {
        g_Config.PresetMidiProgram[i] = MidiProgramDefault[i];
    }
    
    g_Config.PressureSensorSpan = 500;
    g_Config.PitchBendSpan = 750;
    g_Config.ModulationMaximum = 750;
    g_Config.ModulationDeadband = 250;

    StoreConfigData();
}

/*
 * Function to set configuration profile according to operating mode.
 * 
 * Entry arg. mode = REMI_SYNTH_MODE or GEN_SYNTH_MODE
 * 
 * Todo:  To be deprecated when revised mode-switch scheme implemented.  <<<<<<<<<<<<<
 */
void  SetConfigProfile(uint8 mode)
{
    if (mode == REMI_SYNTH_MODE)
    {
        g_Config.MidiSysExclMsgEnabled = 1;
        g_Config.MidiExpressionCCnumber = 2;
        g_Config.MidiModulationCCnumber = 1;
        g_Config.MidiPressureInterval = 5;
        g_Config.MidiControllerInterval = 30;  
        g_Config.Send14bitExprnData = 1; 
        g_Config.LegatoModeEnabled = 1; 
        g_Config.VelocitySenseEnabled = 0;
        g_Config.PitchBendEnabled = 0;
    }
    else  // if (mode == GEN_SYNTH_MODE)
    {
        g_Config.MidiSysExclMsgEnabled = 0;
        g_Config.MidiExpressionCCnumber = 2;
        g_Config.MidiModulationCCnumber = 1;
        g_Config.MidiPressureInterval = 5;
        g_Config.MidiControllerInterval = 30;  
        g_Config.Send14bitExprnData = 0; 
        g_Config.LegatoModeEnabled = 0; 
        g_Config.VelocitySenseEnabled = 0;
        g_Config.PitchBendEnabled = 0;
    }

    StoreConfigData();
}

/*
 *  Function copies data from a RAM buffer holding current working values of persistent 
 *  parameters to a flash memory block. A 2-byte checksum is calculated and written in
 *  the next 2 bytes following the stored parameters.
 */
void  StoreConfigData()
{
    g_Config.CheckSum = CheckConfigData();
    
    FlashWriteBlock((uint8 *) &g_Config, FLASH_DATA_BLOCK_BEG);
}

/*
 *  Function calculates a checksum from data in the structure g_Config.
 *  The checksum value is the (unsigned 16-bit) sum of all data bytes in g_Config,
 *  except the checksum word itself, exclusive-OR'd with the constant 0xABBA.
 *
 *  Return val:  (uint16) checksum value
 */
uint16  CheckConfigData()
{
    uint16  checksum = 0;
    uint8   i;
    uint8  *pdat = (uint8 *) &g_Config;
    uint8   datasize = sizeof(g_Config) - 2;  // not including checksum
    
    for (i = 0 ; i < datasize ; i++)
    {
        checksum += *pdat++;
    }

    return (checksum ^ 0xABBA); 
}


// End of File
