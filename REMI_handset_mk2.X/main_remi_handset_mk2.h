/* =======================================================================================
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
#define BUILD_VER_MINOR   4
#define BUILD_VER_DEBUG   11
//
// =======================================================================================

// Possible values for g_Config.FingeringScheme.. (bit_0 set => pad at LH4)
#define KEYING_SCHEME_REMI_STD    0    // Original REMI keying scheme (RH5)
#define KEYING_SCHEME_REMI_ALT    1    // Alternate EWI keying scheme (LH4)
#define KEYING_SCHEME_GERMAN      2    // German fingering emulation  (RH5)
#define KEYING_SCHEME_GERMAN_ALT  3    // German fingering emulation  (LH4)

#define NOTE_ON_VELOCITY_DELAY   15    // Delay (ms) from note trigger to get velocity

// MIDI System Exclusive message types unique to REMI...
#define REMI_PRESET_MSG   0x07     // 'REMI PRESET' msg type (set Preset #)
#define REMI_IDENT_MSG    0x30     // 'REMI IDENT' msg type (periodic ID)


enum  Self_test_categories
{
    TEST_CONFIG_INTEGRITY = 0,  // 0
    TEST_CONFIG_OVERSIZE,       // 1
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
    // This group of parameters is duplicated to provide alternate synth modes...
    uint8   MidiBasicChannel[2];        // MIDI OUT channel, range: 1..16 (default: 1)
    uint8   MidiSysExclMsgEnabled[2];   // MIDI SystemExclusive Messages Enabled
    uint8   MidiProgChangeEnabled[2];   // MIDI Program Change Messages Enabled
    uint8   MidiExpressionCCnumber[2];  // MIDI Ctrl Change # for breath/pressure messages
    uint8   Send14bitExprnData[2];      // Expression CC msg uses 14 bit data
    uint8   LegatoModeEnabled[2];       // Legato Mode Enabled
    uint8   VelocitySenseEnabled[2];    // Velocity sensing enabled
    uint8   PitchBendEnabled[2];        // Pitch-Bend enabled
    
    // This group of parameters is for "long term" H/W configuration constants...
    uint8   FingeringScheme;          // Fingering Scheme (LH4/RH5 flat or sharp)
    uint8   TouchSenseThreshold;      // Touch-pad sense ON/OFF threshold (ADC count)
    uint8   reserved_1;
    uint16  PressureSensorSpan;       // Pressure sensor span, ADC count (250..750)
    uint16  PitchBendSpan;            // Pitch-Bend sensor span, ADC count (250..750)
    uint16  ModulationMaximum;        // Modulation sensor maximum, ADC count (250..750)
    uint16  ModulationDeadband;       // Modulation sensor dead-band, ADC count (0..500)
    
    // Sundry param's
    uint8   MidiPressureInterval;     // MIDI pressure TX update interval (5..50 ms)
    uint8   MidiControllerInterval;   // MIDI modulation TX update interval (10..100 ms)
    int8    PitchOffset;              // Pitch Offset, semitones (typ. -12, -7, 0, +12)
    
    uint8   PresetMidiProgram[8];     // MIDI Program/voice numbers for 8 presets

    uint16  CheckSum;                 // Data integrity check

} Config_Params_t;

extern  Config_Params_t  g_Config;


// Public functions defined in "main_remi_handset_mk2.c" ----------------------
//
void    BackgroundTaskExec();

uint8   GetNoteOnOffState();
uint8   GetLastNotePlayed();
uint16  GetPressureRawReading();
uint16  GetMidiPressureLevel();
uint16  GetModulationRawReading();
uint16  GetPitchBendRawReading();
uint16  GetModulationPadForce();
int16   GetPitchBendData();

bool    isPresetButtonPressed();
void    InstrumentPresetSelect(uint8);
uint8   GetModeSwitchState();

void    DefaultConfigData(void);
bool    FetchConfigData();
void    StoreConfigData();
uint16  CheckConfigData();

#endif // _MAIN_APP_H
