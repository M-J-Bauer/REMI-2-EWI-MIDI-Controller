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
PRIVATE  void   SendPitchBendUpdate();
PRIVATE  void   PitchBendAutoZeroTask();
PRIVATE  void   SendModulationUpdate();
PRIVATE  void   NoteOnOffStateTask();
PRIVATE  void   ReadSensorsTask();
PRIVATE  void   CalcMidiPressureLevel(void);
PRIVATE  void   PresetButtonMonitor();
PRIVATE  void   PresetButtonActionTask();
PRIVATE  void   ModeSwitchState();
PRIVATE  void   SendRemiIdentMessage();


// ----------------  Global data  -------------------------------
//
uint8   g_FW_version[4];          // firmware version # (major, minor, build, 0)
char   *g_AppTitleCLI;            // Title string output by CLI "ver" command
uint8   g_SelfTestErrors;         // Self-test fault codes (0 => no fault)
bool    g_NoteOnDisplayActive;    // Set true to enable Note-On display (in CLI)

Config_Params_t  g_Config;

// ----------------  Private data  ------------------------------
//
static  uint16  m_TouchPadStates;
static  uint8   m_NoteOnOffState;
static  uint8   m_SynthMode;
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


//=================================================================================================

PRIVATE  void  APP_Initialize()
{
    g_FW_version[0] = BUILD_VER_MAJOR;
    g_FW_version[1] = BUILD_VER_MINOR;
    g_FW_version[2] = BUILD_VER_DEBUG;
    
    g_AppTitleCLI = "Bauer {REMI} Handset mk2 -- Service Port CLI \n";
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
    
    if (sizeof(g_Config) > 64) g_SelfTestErrors |= (1 << TEST_CONFIG_OVERSIZE);
    
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
    
    m_SynthMode = SWITCH2_INPUT() ? 1 : 0;  // State at power-on/reset

    EUSART1_WriteString(g_AppTitleCLI);   // Output CLI startup message
    Cmnd_ver(1, NULL);

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
        PitchBendAutoZeroTask();

        if (++count_50ms == 4)   // Do 200ms periodic tasks
        {
            count_50ms = 0;
            m_SynthMode = SWITCH2_INPUT() ? 1 : 0; 
            SendRemiIdentMessage();
        }
    }
}


/**
 * Function:     Get note on/off state 
 *
 * Return val:   0 = IDLE, 1 = PENDING, 2 = IN PROGRESS
 */
uint8  GetNoteOnOffState()
{
    return  m_NoteOnOffState;
}


/**
 * Function:     Display (via CLI) last note initiated 
 * 
 * Called whenever a new note is initiated (not a legato note change), 
 * only if the global flag g_NoteOnDisplayActive is set True (using "diag -n 1").
 */
void  DisplayNotePlayed(uint8 noteNum)
{
    static char  noteID[] = { 'C', 'C', 'D', 'E', 'E', 'F', 'F', 'G', 'A', 'A', 'B', 'B' };
    static char  noteAC[] = { '-', '#', '-', 'b', '-', '-', '#', '-', 'b', '-', 'b', '-' };
    
    uint8  octave = (noteNum / 12) - 1;  // e.g. C4 = 60:  60 /12 - 1 = 4
    uint8  note = noteNum % 12;
    
    putstr("\t");  putch(noteID[note]);  
    if (noteAC[note] != '-')  putch(noteAC[note]);  putDecimal(octave, 1);
    putstr("\t");  putDecimal(noteNum, 3);  putNewLine();
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
 *
 * Return val:  (uint8) MIDI note number between 23 (B0) and 108 (C8).
 *
 * ````````````````````````````````````````````````````````````````````````````````````````
 * Touch-pad configuration for all REMI fingering schemes:
 *
 *                       |  octave   | ----------- s e m i t o n e ----------- | LH4 |
 *    Finger position -> | OT1 | OT2 | LH1 | LH2 | LH3 | RH1 | RH2 | RH3 | RH4 | RH5 |
 *    PIC18__K22 ADC# -> |  9  |  8  |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
 *
 * ````````````````````````````````````````````````````````````````````````````````````````
 */
PRIVATE  uint8  NoteNumberFromKeyPattern(uint16 fingerPattern)
{
    // MIDI note number from finger pattern, assuming RH4 = 0, before octave adjusted.
    // This table is used for the REMI simplified fingering schemes (#0 and #1).
    // Also used for the 'German' fingering schemes (#2 and #3) in the lower octave, 
    // with some exceptions.
    static  uint8   baseNoteNumber[] =
    {
    // RH1..RH3 finger pattern (3 LS bits)
    // 000  001  010  011  100  101  110  111   ||  LH1..LH3 (3 MS bits)
        73,  73,  73,  73,  73,  73,  73,  73,  //  000 = C#5
        84,  84,  84,  84,  83,  82,  81,  80,  //  001
        72,  72,  72,  72,  72,  72,  72,  72,  //  010
        79,  78,  78,  78,  77,  77,  76,  74,  //  011
        71,  70,  70,  70,  70,  70,  70,  70,  //  100
        70,  70,  70,  70,  70,  70,  70,  70,  //  101
        69,  68,  68,  68,  68,  68,  68,  68,  //  110
        67,  66,  66,  66,  65,  65,  64,  62   //  111
    };

    // This table is used for the 'German' keying schemes (#2, #3) in the upper octave,
    // (notes C5..E5), where neither of the octave pads is touched.
    static  uint8   NoteNumberGF0[] =
    {
    // RH1..RH3 finger pattern (3 LS bits)
    // 000  001  010  011  100  101  110  111   ||  LH1..LH3 (3 MS bits)
        73,  73,  73,  73,  73,  73,  73,  73,  //  000 = C#5
        71,  71,  71,  71,  71,  71,  71,  71,  //  001 = B5 (NS)^
        74,  74,  74,  74,  74,  74,  74,  74,  //  010 = D5
        76,  76,  76,  76,  73,  73,  76,  75,  //  011 = C#5/Eb5/E5
        48,  48,  48,  48,  48,  48,  48,  48,  //  100 = C6 (NS)^
        73,  73,  73,  73,  73,  73,  73,  73,  //  101 = C#5 alt.
        73,  97,  97,  97,  85,  85,  85,  85,  //  110 = C#5 [C#6/C#7]^
        78,  78,  78,  78,  77,  77,  76,  75   //  111 = Eb5/E5/F5/F#5 [Ab5]
    };    
    
    // [ ^NS denotes 'Not Specified' in the fingering chart. ]
    // This table is used for the 'German' keying schemes (#2, #3) in the upper octaves
    // (notes C#5..G6), where octave pad OCT- is touched , i.e. thumb 'half-hole'.
    static  uint8   NoteNumberGF1[] =
    {
    // RH1..RH3 finger pattern (3 LS bits)
    // 000  001  010  011  100  101  110  111   ||  LH1..LH3 (3 MS bits)
        85,  85,  85,  85,  85,  85,  85,  85,  //  000 = C#6 (NS)^
        88,  88,  88,  88,  88,  88,  88,  88,  //  001 = E6 (NS)^
        88,  88,  88,  88,  88,  88,  88,  88,  //  010 = E6
        96,  96,  96,  87,  96,  96,  96,  96,  //  011 = Eb6 [C7]^
        91,  90,  84,  84,  91,  84,  84,  84,  //  100 = C6/G6 [F#6]
        87,  87,  86,  86,  86,  86,  85,  85,  //  101 = C#6/D6/Eb6
        81,  90,  83,  82,  80,  81,  83,  82,  //  110 = A5/Bb5/B5 [F#6]
        79,  78,  78,  80,  77,  77,  76,  89   //  111 = E5/F5/F#5/G5/Ab5
    };    
    
    uint8   noteNumber;  // return value
    uint8   baseNote, padLH4, padRH4, padRH5;
    uint8   top6Fingers = (fingerPattern >> 2) & 0x3F;  // Drop RH4, RH5 and octave pads
    uint8   octavePads  = (fingerPattern >> 8) & 0x03;  // Extract OCT+, OCT-
    
    padRH4 = (fingerPattern >> 1) & 1;
    padLH4 = padRH5 = fingerPattern & 1;
    noteNumber = baseNote = baseNoteNumber[top6Fingers];  // Note number before octave applied

    // Notes flattened when either pad RH4 or RH5 is touched...
    if ((top6Fingers == 0b111110) && (padRH4 || padRH5)) noteNumber = 63;  // Eb4
    if ((top6Fingers == 0b111000) && (padRH4 || padRH5)) noteNumber = 66;  // Gb4 (F#4)
    if ((top6Fingers == 0b110000) && (padRH4 || padRH5)) noteNumber = 68;  // Ab4
    if ((top6Fingers == 0b100000) && (padRH4 || padRH5)) noteNumber = 70;  // Bb4
    if ((top6Fingers == 0b011110) && (padRH4 || padRH5)) noteNumber = 75;  // Eb5
    if ((top6Fingers == 0b011000) && (padRH4 || padRH5)) noteNumber = 78;  // Gb5 (F#5)
    if ((top6Fingers == 0b001110) && (padRH4 || padRH5)) noteNumber = 80;  // Ab5
    if ((top6Fingers == 0b001100) && (padRH4 || padRH5)) noteNumber = 82;  // Bb5

    if (g_Config.FingeringScheme == KEYING_SCHEME_REMI_STD)  // Pad at RH5
    {
        if ((top6Fingers == 0b111111) && padRH4 && !padRH5) noteNumber = 60;  // C4
        if ((top6Fingers == 0b111111) && !padRH4 && padRH5) noteNumber = 61;  // C#4 (Db4)
        if ((top6Fingers == 0b011111) && padRH4 && !padRH5) noteNumber = 60;  // C5
        if ((top6Fingers == 0b011111) && !padRH4 && padRH5) noteNumber = 61;  // C#5 (Db5)
    }
    else if (g_Config.FingeringScheme == KEYING_SCHEME_REMI_ALT)  // Pad at LH4
    {
        if ((top6Fingers == 0b111111) && padRH4 && !padLH4) noteNumber = 60;  // C4
        if ((top6Fingers == 0b111111) && padRH4 &&  padLH4) noteNumber = 61;  // C#4
        if ((top6Fingers == 0b111100) && !padRH4 && padLH4) noteNumber = 66;  // F#4
        if ((top6Fingers == 0b011111) && padRH4 && !padLH4) noteNumber = 72;  // C5
        if ((top6Fingers == 0b011111) && padRH4 &&  padLH4) noteNumber = 73;  // C#5
        if ((top6Fingers == 0b010000) && !padRH4 && padLH4) noteNumber = 73;  // C#5 alt
        if ((top6Fingers == 0b011100) && !padRH4 && padLH4) noteNumber = 78;  // F#5
    }
    // Apply octave selection... for REMI simplified fingering schemes (#0 and #1)
    if (octavePads == 1) noteNumber -= 12;  // lower 1 octave
    if (octavePads == 2) noteNumber += 12;  // raise 1 octave
    
    if (g_Config.FingeringScheme >= KEYING_SCHEME_GERMAN)  // Scheme #2 or #3
    {
        if (octavePads & 2)  // OCT+ touched ("full hole" keyed)
        {
            noteNumber = baseNote; 
            // Exceptions...
            if ((top6Fingers == 0b111111) &&  padRH4 && !padRH5) noteNumber = 60;  // C4
            if ((top6Fingers == 0b111111) && !padRH4 && padRH5)  noteNumber = 61;  // C#4
            if ((top6Fingers == 0b111110) && (padRH4 || padRH5)) noteNumber = 63;  // Eb4
            if ((top6Fingers == 0b111101) &&  padRH4) noteNumber = 65;  // F4 alt
            if ((top6Fingers == 0b011100) && !padRH4) noteNumber = 70;  // Bb4 alt
            if ((top6Fingers == 0b011000) && !padRH4) noteNumber = 71;  // B4 alt
        }
        else if (octavePads == 1)  // OCT- touched ("half hole" keyed)
        {
            noteNumber = NoteNumberGF1[top6Fingers];
            // Exceptions...
            if ((top6Fingers == 0b110111) && padRH4) noteNumber = 89;  // F6
            if ((top6Fingers == 0b101110) && padRH4) noteNumber = 85;  // C#6
            if ((top6Fingers == 0b101101) && padRH4) noteNumber = 85;  // C#6 alt
        }
        else  noteNumber = NoteNumberGF0[top6Fingers];  // No Octave pad touched
    }
    
    // Adjust noteNumber using 'Pitch Offset' parameter (0..+/-24 semitones)
    noteNumber += g_Config.PitchOffset;

    return  noteNumber;
}


/*****
 * Function:  Determine 14-bit pressure level from the pressure sensor raw ADC reading.
 *            May be used also to find MIDI velocity value from pressure.
 *            Called at regular periodic intervals, Ts = 5ms.
 *
 * Outputs:   (uint8) m_Pressure_Hi = MIDI pressure High byte, or Velocity (0..127)
 *            (uint8) m_Pressure_Lo = MIDI pressure Low byte (0..127)
 *
 * Note 1:    The function assumes a linear relationship between raw pressure reading
 *            and MIDI pressure (or velocity) value. The external MIDI sound module
 *            should apply an exponential transfer function between MIDI pressure and
 *            audio output amplitude (to compensate for perceived loudness).
 * 
 * Note 2:    Raw ADC readings are smoothed using a first-order IIR filter, with
 *            time-constant Tc determined by the sample interval Ts and a constant K, 
 *            by the formula:  Tc = Ts x (0.8 / K)  where K = 1 / 2^N  (N = 1,2,3...).
 *            Example:  Let Ts = 5ms, K = 0.25, then Tc = 5 x 3.2 = 16ms
 */
PRIVATE  void  CalcMidiPressureLevel()
{
    static  int32  pressure;  // smoothed level
    int32   level;
    uint16  rawSpan = g_Config.PressureSensorSpan;  // unit = ADC counts

    // Calculate pressure as a 14-bit quantity, range 0 to 16256 (127 x 128)
    level = (16256 * (int32)(m_PressureSensorReading - m_PressureQuiescent)) / rawSpan;
    if (level < 0) level = 0;
    if (level > 16256)  level = 16256;
    // Apply smoothing filter
    pressure = pressure - (pressure >> 2) + (level >> 2);  // K = 0.25 (N = 2)

    m_Pressure_Hi = (uint8) (pressure >> 7) & 0x7F;  // High-order 7 bits
    m_Pressure_Lo = (uint8) pressure & 0x7F;   // Low-order 7 bits
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
    
    uint8   noteNumber = NoteNumberFromKeyPattern(m_TouchPadStates);
    uint8   top3Fingers = (m_TouchPadStates >> 5) & 7;  // Get LH1..LH3 into bits 2,1,0
    uint8   octavePads  = (m_TouchPadStates >> 8) & 3;  // Extract OCT+, OCT-
    bool    isValidNote = (octavePads != 0) && (top3Fingers != 0);
    uint8   channel = g_Config.MidiBasicChannel[m_SynthMode];
    bool    sendNoteOn = 0;
    uint16  pressure_14b = ((uint16)m_Pressure_Hi << 7) + m_Pressure_Lo;
    uint8   velocity = m_Pressure_Hi; 
    uint8   exprnCC = g_Config.MidiExpressionCCnumber[m_SynthMode];
    
    if (g_Config.FingeringScheme >= KEYING_SCHEME_GERMAN) isValidNote = TRUE;
    
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

        if (!g_Config.VelocitySenseEnabled[m_SynthMode])  // use fixed velocity value
        {
            velocity = 64;  
            sendNoteOn = 1;
        }
        else if (stateTimer_ms >= NOTE_ON_VELOCITY_DELAY)  // ready to acquire velocity
        {
            velocity = m_Pressure_Hi;  // capture the pressure reading
            sendNoteOn = 1;
        }
        // else... Don't send Note-On;  remain in this state.

        if (sendNoteOn)
        {
            MIDI_SendNoteOn(channel, noteNumber, velocity);
            noteNumPlaying = noteNumber;
            if (g_NoteOnDisplayActive) DisplayNotePlayed(noteNumber);
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
            if (g_Config.LegatoModeEnabled[m_SynthMode])  // Synth is MIDI compliant
            {
                MIDI_SendNoteOn(channel, noteNumber, velocity);  // new note on
                MIDI_SendNoteOff(channel, noteNumPlaying);       // old note off
            }
            else  // Synth is NOT MIDI standard compliant (and/or POLY mode enabled)
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
                if (exprnCC != 0)
                {
                    MIDI_SendControlChange(channel, exprnCC, m_Pressure_Hi);
                    if (g_Config.Send14bitExprnData[m_SynthMode]) 
                        MIDI_SendControlChange(channel, (0x20 + exprnCC), m_Pressure_Lo);
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
            if (g_Config.PitchBendEnabled[m_SynthMode]) SendPitchBendUpdate();
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
    uint8   channel = g_Config.MidiBasicChannel[m_SynthMode];

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
    uint8   channel = g_Config.MidiBasicChannel[m_SynthMode];
    uint8   dataMSB, dataLSB;
    uint16  currentValue = GetModulationPadForce();  

    if (currentValue != lastValueSent)
    {
        dataMSB = (currentValue >> 7) & 0x7F;   // send 7 MS bits of 14 bit data
        MIDI_SendControlChange(channel, 0x01, dataMSB);
        if (g_Config.Send14bitExprnData[m_SynthMode])
        {
            dataLSB = currentValue & 0x7F;   // send 7 LS bits of 14 bit data
            MIDI_SendControlChange(channel, 0x21, dataLSB);
        }
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
 *     if there is no touch-pad touched,
 *         then a MIDI 'Reset' and 'All Sound Off' messages are sent;
 *     otherwise, if one top-surface pad and no OCT pads are touched,
 *         then the PRESET selected by that touch-pad is activated;
 *     otherwise, if an octave pad is touched,
 *         then the 'pitch offset' param is set accordingly. (See User Manual.)
 *
 * ------------------------------------------------------------------------------------
 */
PRIVATE  void   PresetButtonActionTask()
{
    uint8   pad, lower5pads, padLH4, preset;
    uint8   channel = g_Config.MidiBasicChannel[m_SynthMode];
    uint8   top8pads;    // upper surface pads
    uint8   octvpads = m_TouchPadStates >> 8;  // octave pads
    
    if (g_Config.FingeringScheme & 1)  // Pad at LH4 position
    {
        // At this point top8pads has bit0 = LH4. Need to get physical pad layout.
        padLH4 = (top8pads << 4) & 0x10;   // move bit0 into LH4 posn (bit4)
        lower5pads = padLH4 | ((top8pads >> 1) & 0x0F);   // Add 4 LS bits (RH1..RH4)
        top8pads = (top8pads & 0xE0) | lower5pads;      // Add 3 MS bits (LH1..LH3)
    }
    else  top8pads = m_TouchPadStates & 0xFF;  // Pad at RH5 position

    if (m_NoteOnOffState == NOTE_OFF_IDLE)  // No note playing now
    {
        if (m_TouchPadStates == 0) // No pads touched -- send MIDI reset, etc
        {
            MIDI_SendAllSoundOff(channel);
            MIDI_SendResetAllControllers(channel);
        }
        else if (octvpads == 0 && top8pads != 0x00)  // One or more top pad(s) touched
        {
            for (preset = 1, pad = 0x01;  pad != 0;  preset++)
            {
                if (top8pads == pad)  // found one pad touched
                {
                    InstrumentPresetSelect(preset);  // Activate new Preset
                    break;  
                }
                pad = (uint8) (pad << 1);  // next higher pad position
            }
        }
        else if (octvpads == 1 && top8pads == 0)  // OCT- pad only...
        {
            g_Config.PitchOffset = -12;  // Shift down 1 octave (bass C3)
            StoreConfigData();
        }
        else if (octvpads != 0 && top8pads == 0x20)  // Any OCT pad(s) + LH3
        {
            g_Config.PitchOffset = -7;  // Shift down: C4 to F3 (bass F3)
            StoreConfigData();
        }
        else if (octvpads == 3 && top8pads == 0)  // OCT+ and OCT- together...
        {
            g_Config.PitchOffset = 0;  // Clear pitch offset (tenor C4)
            StoreConfigData();
        }
        else if (octvpads != 0 && top8pads == 0x80)  // Any OCT pad(s) + LH1
        {
            g_Config.PitchOffset = 5;  // Shift up: C4 to F4 (alto/treble F4)
            StoreConfigData();
        }
        else if (octvpads == 2 && top8pads == 0)  // OCT+ pad only...
        {
            g_Config.PitchOffset =  12;  // Shift up 1 octave (soprano/descant C5)
            StoreConfigData();
        }
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
    uint8  chan = g_Config.MidiBasicChannel[m_SynthMode];
    uint8  progNum = g_Config.PresetMidiProgram[preset & 7];
    
    // Send General MIDI program change command, if enabled. (ignored by REMI synth) 
    if (g_Config.MidiProgChangeEnabled[m_SynthMode]) MIDI_SendProgramChange(chan, progNum);
    
    // Construct MIDI system exclusive 'PRESET' message...
    RemiPresetMsg[0]  = SYS_EXCLUSIVE_MSG;  // status byte
    RemiPresetMsg[1]  = SYS_EXCL_REMI_ID;   // manuf/product ID
    RemiPresetMsg[2]  = REMI_PRESET_MSG;    // Msg type
    RemiPresetMsg[3]  = preset & 7;         // selected preset # (0..7)
    RemiPresetMsg[4]  = g_FW_version[0];
    RemiPresetMsg[5]  = g_FW_version[1];
    RemiPresetMsg[6]  = g_FW_version[2];
    RemiPresetMsg[7]  = g_SelfTestErrors;
    RemiPresetMsg[8]  = g_Config.FingeringScheme;
    RemiPresetMsg[9]  = g_Config.LegatoModeEnabled[m_SynthMode];
    RemiPresetMsg[10] = g_Config.VelocitySenseEnabled[m_SynthMode];
    RemiPresetMsg[11] = SYSTEM_MSG_EOX;     // end-of-msg code
    
    if (g_Config.MidiSysExclMsgEnabled[m_SynthMode])  
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
 * Function returns the Mode Switch input state (0=Low or 1=High).
 */
uint8   GetModeSwitchState()
{
    return m_SynthMode;
}


/*
 * Function:  Send MIDI System Exclusive message -- 'REMI IDENT'
 * 
 * If MIDI System Exclusive messages are enabled AND a note is not currently playing,
 * a REMI "IDENT" message is sent periodically to the REMI Synth Module,
 * at intervals not exceeding 250ms, for the purpose of identifying the REMI handset
 * as the controller connected to the synth MIDI IN port.
 */
PRIVATE  void  SendRemiIdentMessage()
{
    if (g_Config.MidiSysExclMsgEnabled[m_SynthMode] && m_NoteOnOffState == NOTE_OFF_IDLE)
    {
        MIDI_PutByte(SYS_EXCLUSIVE_MSG);   // status byte
        MIDI_PutByte(SYS_EXCL_REMI_ID);    // manuf/product ID
        MIDI_PutByte(REMI_IDENT_MSG);      // Msg type
        MIDI_PutByte(SYSTEM_MSG_EOX);      // end-of-msg code
    }
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
}


/*
 * Function:  GetModulationPadForce()
 *
 * Return value:  modLevel: Unsigned integer in the range 0 ~ 16256 (14 bits)
 *                representing the force applied to the Modulation Pad.
 *
 * Notes:  g_Config.ModulationMaximum is the maximum value of m_ModulationSensorReading.
 * 
 *         A "dead-band" is deliberately introduced at zero so that a minimum force needs
 *         to be applied to the sensor before a non-zero reading is obtained.
 */
uint16   GetModulationPadForce()
{
    int32   modLevel;  // uncompensated for non-linearity
    int32   rawSpan;

    modLevel = m_ModulationSensorReading - g_Config.ModulationDeadband;
    if (modLevel < 0) modLevel = 0;
    
    // Scale modLevel such that it spans the full 14 bit range (0 ~ 16256)...
    rawSpan = g_Config.ModulationMaximum - g_Config.ModulationDeadband;
    modLevel = (16256 * modLevel) / rawSpan;
    if (modLevel > 16256) modLevel = 16256;

    return  (uint16) modLevel;
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
    if (m_TimeSinceLastNoteOff_ms > 1000)
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

    if (m_TimeSinceLastNoteOff_ms == 0)
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
 *  Functions to support persistent data storage in a 64-byte flash memory block.
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
    
    uint8  i;
    uint8 *pdat = (uint8 *) &g_Config;
    
    // Erase (write 0xFF) data in 64-byte block of RAM beginning at &g_Config.
    for (i = 0;  i < 64;  i++)  { *pdat++ = 0xFF; }

    // Default values for Synth Mode [0] -- Generic Monophonic MIDI synth
    g_Config.MidiBasicChannel[0] = 1;
    g_Config.MidiSysExclMsgEnabled[0] = 0;
    g_Config.MidiProgChangeEnabled[0] = 0;
    g_Config.MidiExpressionCCnumber[0] = 2; 
    g_Config.Send14bitExprnData[0] = 0; 
    g_Config.LegatoModeEnabled[0] = 1; 
    g_Config.VelocitySenseEnabled[0] = 0;
    g_Config.PitchBendEnabled[0] = 0;
    
    // Default values for Synth Mode [1] -- Bauer EWI synth (PIC32, Teensy3.2)
    g_Config.MidiBasicChannel[1] = 1;
    g_Config.MidiSysExclMsgEnabled[1] = 1;
    g_Config.MidiProgChangeEnabled[1] = 0;
    g_Config.MidiExpressionCCnumber[1] = 2; 
    g_Config.Send14bitExprnData[1] = 0; 
    g_Config.LegatoModeEnabled[1] = 1; 
    g_Config.VelocitySenseEnabled[1] = 0;
    g_Config.PitchBendEnabled[1] = 0;
    
    g_Config.FingeringScheme = KEYING_SCHEME_REMI_STD;
    g_Config.TouchSenseThreshold = TOUCH_THRESHOLD_DEFAULT;
    g_Config.PressureSensorSpan = 750;
    g_Config.PitchBendSpan = 1000;
    g_Config.ModulationMaximum = 750;
    g_Config.ModulationDeadband = 350;
    
    g_Config.MidiPressureInterval = 5;
    g_Config.MidiControllerInterval = 30;      
    g_Config.PitchOffset = 0;
    
    for (i = 0 ; i < 8 ; i++)
    {
        g_Config.PresetMidiProgram[i] = MidiProgramDefault[i];
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
