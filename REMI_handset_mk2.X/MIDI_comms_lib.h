/****************************************************************************************
 *
 * FileName:   MIDI_comms_lib.h 
 *
 *
 * =======================================================================================
 */
#ifndef _MIDI_COMMS_LIB_H
#define _MIDI_COMMS_LIB_H

#include <xc.h>
#include <stdlib.h>
#include <ctype.h>
#include "gendef.h"  
#include "EUSART_drv.h"
#include "low_level.h"

#define MIDI_TXBUFSIZE   32    // MIDI Transmit Queue size (bytes)

#define MIDI_TX_Ready()        EUSART2_TX_Ready()     
#define MIDI_TransmitByte(b)   EUSART2_WriteByte(b)    // see EUSART_drv.h

#define OMNI_ON_POLY      1   // MIDI device responds in Poly mode on all channels
#define OMNI_ON_MONO      2   // MIDI device responds in Mono mode on all channels
#define OMNI_OFF_POLY     3   // MIDI device responds in Poly mode on base channel only
#define OMNI_OFF_MONO     4   // MIDI device responds in Mono mode on base channel only

#define NOTE_OFF_CMD         0x80    // 3-byte message
#define NOTE_ON_CMD          0x90    // 3-byte message 
#define POLY_KEY_PRESS_CMD   0xA0    // 3-byte message
#define CONTROL_CHANGE_CMD   0xB0    // 3-byte message
#define PROGRAM_CHANGE_CMD   0xC0    // 2-byte message
#define CHAN_PRESSURE_CMD    0xD0    // 2-byte message
#define PITCH_BEND_CMD       0xE0    // 3-byte message
#define SYS_EXCLUSIVE_MSG    0xF0    // variable length message
#define SYSTEM_MSG_EOX       0xF7    // system msg terminator
#define CC_MODULATION        1       // Control change High byte
#define CC_BREATH_PRESSURE   2       //    ..     ..     ..
#define CC_CHANNEL_VOLUME    7       //    ..     ..     ..
#define CC_EXPRESSION        11      //    ..     ..     ..
#define SYS_EXCL_REMI_ID     0x73    // arbitrary pick... hope it's free!
#define SYS_EXCL_MSG_MAX_LENGTH  64  // not a MIDI specified requirement!


// MIDI Channel Voice Messages ------------------------------------------------
void   MIDI_SendNoteOn(uint8 chan, uint8 noteNum, uint8 velocity);
void   MIDI_SendNoteOff(uint8 chan, uint8 noteNum);
void   MIDI_SendAfterTouch(uint8 chan, uint8 level);
void   MIDI_SendPitchBend(uint8 chan, uint16 value);
void   MIDI_SendControlChange(uint8 chan, uint8 ctrlNum, uint8 value);
void   MIDI_SendProgramChange(uint8 chan, uint8 progNum);

// MIDI Channel Mode and System Messages --------------------------------------
void   MIDI_SendReceiverMode(uint8 chan, uint8 mode);
void   MIDI_SendAllSoundOff(uint8 chan);
void   MIDI_SendResetAllControllers(uint8 chan);

// MIDI utility functions -----------------------------------------------------
int    MIDI_GetMessageLength(uint8 statusByte);
short  MIDI_PutByte(uint8 dat);
void   MIDI_TxQueueHandler(void);


#endif // _MIDI_COMMS_LIB_H
