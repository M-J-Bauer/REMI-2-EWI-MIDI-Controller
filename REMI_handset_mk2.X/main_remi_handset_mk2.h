/****************************************************************************************
 *
 * File:   main_remi_handset_mk2.h
 *
 * Contains build options and particular def's for the application.
 *
 * =======================================================================================
 */
#ifndef _MAIN_APP_H
#define _MAIN_APP_H

#include "gendef.h"
#include "low_level.h"
#include "console.h"
#include "touch_sense.h"
#include "MIDI_comms_lib.h"

// =======================================================================================
//                       FIRMWARE VERSION NUMBER AND BUILD OPTIONS
//
#define BUILD_VER_MAJOR   1
#define BUILD_VER_MINOR   0
#define BUILD_VER_DEBUG   0
//
// =======================================================================================

// Possible values for g_Config.FingeringScheme...
#define KEY_SCHEME_RH5_FLATTENS   0    // Original keying scheme (LH4/RH5 -> D.flat)
#define KEY_SCHEME_LH4_SHARPENS   1    // Alternate keying scheme (LH4 -> C.sharp)

// Possible values for g_Config.TouchPadLayout...
#define LAYOUT_WITH_PAD_AT_RH5    0    // Recorder-like layout (Pad at RH5)
#define LAYOUT_WITH_PAD_AT_LH4    1    // Sylphyo-style layout (Pad at LH4)

#define NOTE_ON_VELOCITY_DELAY   10    // Delay (ms) from note trigger to get velocity

// Possible values for handset m_OperatingMode:
#define GEN_SYNTH_MODE      'G'   // "Generic synth" operating mode
#define REMI_SYNTH_MODE     'R'   // "REMI synth" operating mode

// MIDI System Exclusive message types unique to REMI...
#define REMI_PRESET_MSG   0x07     // 'REMI PRESET' msg type (set Preset #)
#define REMI_IDENT_MSG    0x30     // 'REMI IDENT' msg type (periodic ID)


enum  Self_test_categories
{
    TEST_CONFIG_INTEGRITY = 0,  // 0
    TEST_RESERVED,              // 1
    TEST_PRESSURE_SENSOR,       // 2
    TEST_MODULATION_SENSOR,     // 3
    NUMBER_OF_SELFTEST_ITEMS
};

enum  NoteOnOff_Task_States
{
    NOTE_OFF_IDLE = 0,          // Waiting for Note-On trigger condition
    NOTE_ON_PENDING,            // Delay to get Note-on velocity (pressure change)
    NOTE_ON_PLAYING,            // Waiting for Note-off or "note-change" event
};

extern  char   *g_AppTitleCLI;             // Title string output by "ver" command
extern  uint8   g_FW_version[];            // firmware version # (major, minor, build, 0)
extern  bool    g_HeartbeatLEDactive;
extern  bool    g_DiagnosticModeActive;
extern  int     g_DiagnosticModeTimeout;   // unit = minutes
extern  uint32  g_DiagModeTimer_ms;
extern  uint8   g_SelfTestErrors;

extern volatile bool  v_RTI_flag_1ms_task;
extern volatile bool  v_RTI_flag_5ms_task;
extern volatile bool  v_RTI_flag_50ms_task;
extern volatile uint8 v_TaskOverrunCount;

// Configuration parameters (persistent data) stored in a block of flash program memory.
// The size of this structure must not exceed 64 bytes.
//
typedef struct Config_Params_Structure
{
    uint8   MidiBasicChannel;         // MIDI OUT channel, range: 1..16 (default: 1)
    uint8   MidiSysExclMsgEnabled;    // MIDI SystemExclusive Messages Enabled (0)
    uint8   MidiExpressionCCnumber;   // MIDI Ctrl Change # for breath/pressure messages
    uint8   MidiModulationCCnumber;   // MIDI Ctrl Change # for Modulation CC messages
    uint8   MidiPressureInterval;     // MIDI pressure TX update interval (5..50 ms)
    uint8   MidiControllerInterval;   // MIDI modulation TX update interval (10..100 ms)
    
    uint8   Use2byteExpression;       // MIDI expression CC msg uses 14 bit data (0)
    uint8   LegatoModeEnabled;        // Legato Mode Enabled (1) [MODE switch override]
    uint8   VelocitySenseEnabled;     // Velocity sensing enabled (1)
    uint8   PitchBendEnabled;         // Pitch-Bend enabled (0)
    uint8   TouchPadLayout;           // Touch-pad layout (pad at LH4 or RH5 pos'n)
    uint8   FingeringScheme;          // Fingering Scheme (LH4/RH5 flat or sharp)
    uint8   TouchSenseThreshold;      // Touch-pad sense ON/OFF threshold (ADC count)
    
    uint8   PresetMidiProgram[8];     // MIDI Program/voice numbers for 8 presets
    
    uint16  PressureSensorSpan;       // Pressure sensor span, ADC count (250..750)
    uint16  PitchBendSpan;            // Pitch-Bend sensor span, ADC count (250..750)
    uint16  ModulationMaximum;        // Modulation sensor maximum, ADC count (250..750)
    uint16  ModulationDeadband;       // Modulation sensor dead-band, ADC count (0..500)
    uint16  CheckSum;                 // Data integrity check

} Config_Params_t;

extern  Config_Params_t  g_Config;


// Public functions defined in "main_remi_handset_mk2.c" ----------------------
//
void    BackgroundTaskExec();

uint16  GetNoteOnOffState();
uint16  GetPressureRawReading();
uint16  GetMidiPressureLevel();
uint16  GetModulationRawReading();
uint16  GetPitchBendRawReading();
uint16  GetModulationPadForce();
int16   GetPitchBendData();

bool    isPresetButtonPressed();
void    InstrumentPresetSelect(uint8 preset);

bool    FetchConfigData();
void    DefaultConfigData(void);
void    SetConfigProfile(uint8 mode);
void    StoreConfigData();
uint16  CheckConfigData();


// External functions.....................
//

#endif // _MAIN_APP_H
