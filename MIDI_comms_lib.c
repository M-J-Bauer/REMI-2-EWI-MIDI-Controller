/* ================================================================================================
 *
 * FileName:    MIDI_comms_lib.c
 * 
 * Note:  UART1 Receive and Transmit modes are defined in UART_drv.h
 * 
 * It is highly recommended to enable interrupt-driven input and queued output to
 * prevent blocking of time-critical application functions.
 * 
 * ================================================================================================
 */
#include "MIDI_comms_lib.h"

static  uint8   TxBuffer[MIDI_TXBUFSIZE];       // MIDI serial output TX FIFO buffer
static  uint8  *TxHeadPtr;                      // Pointer to next char to be sent out
static  uint8  *TxTailPtr;                      // Pointer to next free place in queue
static  short   TxBufCount;                     // Number of unsent chars in TX buffer

/*
 * Function:     Transmit MIDI Note-On/Velocity message.
 *
 * Entry args:   chan = MIDI channel number (1..16)
 *
 *               noteNum = MIDI standard note number. (Note #60 = C4 = middle-C.)
 *
 *               velocity = initial output level (gain) of output envelope shaper,
 *                          typically, but depends on external MIDI device operation.
 *                          A value of zero should terminate the note.
 *
 * Output amplitude may be subsequently modified by a MIDI "after-touch" command,
 * or a Control Change message (e.g. breath pressure, expression, channel volume, etc),
 * if the connected MIDI device supports dynamic amplitude control.
 */
void  MIDI_SendNoteOn(uint8 chan, uint8 noteNum, uint8 velocity)
{
    uint8   statusByte = 0x90 | ((chan - 1) & 0xF);

    MIDI_PutByte(statusByte);
    MIDI_PutByte(noteNum & 0x7F);
    MIDI_PutByte(velocity & 0x7F);
}


/*
 * Function:     Transmit MIDI Note-Off message.
 *
 * Entry args:   chan = MIDI channel number (1..16)
 *               noteNum = MIDI note number.
 *
 * Attn:         Note-Off Velocity is set to zero (0). Most MIDI devices ignore it.
 *
 */
void  MIDI_SendNoteOff(uint8 chan, uint8 noteNum)
{
    uint8   statusByte = 0x80 | ((chan - 1) & 0xF);

    MIDI_PutByte(statusByte);
    MIDI_PutByte(noteNum & 0x7F);
    MIDI_PutByte(0);
}


/*
 * Function:     Transmit MIDI Channel After-Touch (Pressure) message.
 *
 * Entry args:   chan = MIDI channel number (1..16)
 *               level = Aftertouch (pressure) level.
 */
void  MIDI_SendAfterTouch(uint8 chan, uint8 level)
{
    uint8   statusByte = 0xD0 | ((chan - 1) & 0xF);

    MIDI_PutByte(statusByte);
    MIDI_PutByte(level & 0x7F);
}


/*
 * Function:     Transmit MIDI Pitch Bend message.
 *
 * Entry args:   chan = MIDI channel number (1..16)
 *               value = Pitch deviation value (14 bits). Unit depends on MIDI device.
 */
void  MIDI_SendPitchBend(uint8 chan, int16 value)
{
    uint8   statusByte = 0xE0 | ((chan - 1) & 0xF);

    MIDI_PutByte(statusByte);
    MIDI_PutByte(value & 0x7F);           // 7 LS bits
    MIDI_PutByte((value >> 7) & 0x7F);    // 7 MS bits
}


/*
 * Function:     Transmit MIDI Control Change message.
 *
 * Entry args:   chan = MIDI channel number (1..16)
 *               ctrlNum = Control Number (0..119) -- not range checked.
 *               value = Controller data value (MSB or LSB).
 *
 * Examples:     ctrlNum = 0x01:0x21  Modulation lever, wheel, etc
 *               ctrlNum = 0x02:0x22  Breath controller
 *               ctrlNum = 0x07:0x27  Channel volume
 *               ctrlNum = 0x0B:0x2B  Expression controller
 *               ctrlNum = 0x0C:0x2C  Effects controller #1
 *               ctrlNum = 0x0D:0x2D  Effects controller #2
 *
 * The sound parameter affected by a Control Change message is dependent on the MIDI
 * sound module design and the patch (program) selected. It may be that some or all
 * controllers are unsupported, hence will have no effect on the sound generated.
 *
 * If ctrlNum < 0x20, value is controller MSB; else value is controller LSB.
 * If ctrlNum >= 0x40, controller data is single byte only.
 */
void  MIDI_SendControlChange(uint8 chan, uint8 ctrlNum, uint8 value)
{
    uint8   statusByte = 0xB0 | ((chan - 1) & 0xF);

    MIDI_PutByte(statusByte);
    MIDI_PutByte(ctrlNum & 0x7F);
    MIDI_PutByte(value & 0x7F);
}


/*
 * Function:     Transmit MIDI Program Change message.
 *
 * Entry args:   chan = MIDI channel number (1..16)
 *               progNum = Program (instrument/voice) number. Depends on MIDI device.
 */
void  MIDI_SendProgramChange(uint8 chan, uint8 progNum)
{
    uint8   statusByte = 0xC0 | ((chan - 1) & 0xF);

    MIDI_PutByte(statusByte);
    MIDI_PutByte(progNum & 0x7F);
}


/*
 * Function:     Transmit MIDI Channel Omni/Poly/Mono (mode change) messages
 *               to set receiver mode. In Mono mode, only one channel is selected (M = 1).
 *
 * Entry args:   chan = MIDI channel number (1..16)
 *               mode = Omni/Poly/Mono  (mode = 1, 2, 3 or 4)
 *
 * Note:         This function also causes 'All Notes Off' to happen.
 */
void  MIDI_SendReceiverMode(uint8 chan, uint8 mode)
{
    uint8   statusByte = 0xB0 | ((chan - 1) & 0xF);

    MIDI_PutByte(statusByte);

    if (mode == OMNI_ON_POLY || mode == OMNI_ON_MONO)
    {
        MIDI_PutByte(125);   // Omni On
        MIDI_PutByte(0);
    }
    else  // Omni Off
    {
        MIDI_PutByte(124);   // Omni Off
        MIDI_PutByte(0);
    }

    MIDI_PutByte(statusByte);

    if (mode == OMNI_ON_POLY || mode == OMNI_OFF_POLY)
    {
        MIDI_PutByte(127);   // Poly mode
        MIDI_PutByte(0);
    }
    else
    {
        MIDI_PutByte(126);   // Mono mode (with M = 1)
        MIDI_PutByte(1);
    }
}


/*
 * Function:     Transmit MIDI Channel 'All Sound Off' (mode change) message.
 *
 * Entry args:   chan = MIDI channel number (1..16)
 */
void  MIDI_SendAllSoundOff(uint8 chan)
{
    uint8   statusByte = 0xB0 | ((chan - 1) & 0xF);

    MIDI_PutByte(statusByte);
    MIDI_PutByte(120);
    MIDI_PutByte(0);
}


/*
 * Function:     Transmit MIDI Channel 'Reset All Controllers' (mode change) message.
 *
 * Entry args:   chan = MIDI channel number (1..16)
 */
void  MIDI_SendResetAllControllers(uint8 chan)
{
    uint8   statusByte = 0xB0 | ((chan - 1) & 0xF);

    MIDI_PutByte(statusByte);
    MIDI_PutByte(121);
    MIDI_PutByte(0);
}


/*
 * Function:     Find length of a given MIDI command/status message.
 *
 * Entry args:   MIDI command/status byte
 * 
 * Return:       (int) message length (bytes), usually 2 or 3, but...
 *               if status byte is SYS_EXCLUSIVE_CMD, return maximum length, or...
 *               if status byte is not supported (unrecognised), return 0.
 */
int  MIDI_GetMessageLength(uint8 statusByte)
{
    uint8  command = statusByte & 0xF0;
    uint8  length = 0;
    
    if (command == PROGRAM_CHANGE_CMD)  
    {
        length = 2;
    }
    else if (command == NOTE_ON_CMD || command == NOTE_OFF_CMD
    ||  command == CONTROL_CHANGE_CMD || command == PITCH_BEND_CMD)  
    {
        length = 3;
    }
    else if (statusByte == SYS_EXCLUSIVE_MSG)
    {
        length = SYS_EXCL_MSG_MAX_LENGTH;  // Manufacturer's choice!
    }
    else  length = 0;  // unsupported message type
    
    return  length;
}


/*
|   MIDI_PutByte() - Puts a byte (dat) into the MIDI TX queue (FIFO buffer).
|
|   If the queue is full, the function returns immediately with ERROR (-1),
|   so that application processes are not blocked.
|
|   Returns:  (short) byte queued (dat), or ERROR if the queue is full.
*/
short  MIDI_PutByte(uint8 dat)
{
    static bool init_done;
    short  retval = dat;
    
    if (!init_done)  // first call only -- initialize TX queue
    {
        TxHeadPtr = TxBuffer;
        TxTailPtr = TxBuffer;
        TxBufCount = 0;
        init_done = TRUE;  
    }
    
    if (TxBufCount < MIDI_TXBUFSIZE)
    {
        *TxTailPtr++ = dat;

        if (TxTailPtr >= (TxBuffer + MIDI_TXBUFSIZE))  // wrap
			TxTailPtr = TxBuffer;
        
        TxBufCount++;
    }
    else  retval = ERROR;  // (-1)

    return  retval;    
}


/*
|  MIDI Transmit Queue Handler...
|
|  Routine called *frequently* from the application main loop to transmit the next byte
|  waiting in the MIDI TX output queue, if any.
|
|  Only one byte (or none) is sent on each call, so that blocking of time-critical 
|  tasks is prevented. If the Tx queue is empty, the function just returns.
*/
void   MIDI_TxQueueHandler(void)
{
    if (TxBufCount != 0)
    {
        if (MIDI_TX_Ready())  
        {
            MIDI_TransmitByte(*TxHeadPtr++);

            if (TxHeadPtr >= (TxBuffer + MIDI_TXBUFSIZE))  // wrap
                TxHeadPtr = TxBuffer;
            
            TxBufCount--;
        }
    }    
}

// end-of-file
