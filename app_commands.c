/*=======================================================================================
 *
 * Module:      app_commands.c 
 * 
 * Overview:    Application-specific CLI command functions.
 *
 *=======================================================================================
 */
#include <xc.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "gendef.h"  
#include "EUSART_drv.h"
#include "low_level.h"
#include "console.h"
#include "touch_sense.h"
#include "main_remi_handset_mk2.h"

#define str2int(s)   atoi((const char *) s)

// External functions and data
extern void  BackgroundTaskExec();
extern char  *g_AppTitleCLI;     // Title string output by "ver" command

// Application-Specific CLI command functions in this module
//
void  Cmnd_preset( int argCount, char * argValue[] );
void  Cmnd_config( int argCount, char * argValue[] );
void  Cmnd_mode( int argCount, char * argValue[] );
void  Cmnd_tsens( int argCount, char * argValue[] );
void  Cmnd_mrf( int argCount, char * argValue[] );

void  EnableDiagnosticMode(void);
void  CancelDiagnosticMode(void);

/*----------------------------------------------------------------------------------------
*                  A P P L I C A T I O N   C O M M A N D   T A B L E
*/
const  CmndTableEntry_t  AppCommands[] =
{
    //   Cmd Name      Attribute      Cmd Function
    //----------     -------------   -------------------
    {    "preset",     GEN_CMD,       Cmnd_preset     },
    {    "config",     GEN_CMD,       Cmnd_config     },
    {    "mode",       GEN_CMD,       Cmnd_mode       },
    {    "tsens",      GEN_CMD,       Cmnd_tsens      },
//  {    "mrf",        GEN_CMD,       Cmnd_mrf        },
    //---------------------------------------------------
//  {    "trace",      DBG_CMD,       Cmnd_trace      },    
    {    "$",          0,             NULL            }   // Dummy last entry
} ;


/*```````````````````````````````````````````````````````````````````````````````````````
 * CLI command function:  Cmnd_preset
 *
 * The "preset" command allows preset MIDI Program numbers to be viewed and modified.
 * If a preset value is changed, it will be committed to non-volatile storage.
 */
void  Cmnd_preset( int argCount, char * argVal[] )
{
    uint8  preset = atoi(argVal[1]);
    uint8  i,  midi_prgm;
    uint8  isBadData = 0;
    
    if (argVal[1][0] == '?' )  // help wanted
    {
        putstr( "Usage #1:  preset    | List Preset MIDI Prgm numbers \n\n" );
        putstr( "Usage #2:  preset <pre#> [=] <value>    | Set Preset value \n" );
        putstr( "   where <pre#> is the Preset number (1..8) \n\n" );
        putstr( "Example:   preset 3 = 67   | Set MIDI Prgm 67 for Preset 3 \n\n" );
    }
    else if (argCount == 1)  // View Presets
    {
        putstr("_____________________\n");
        putstr("Preset | MIDI Prgm # \n");
        putstr("```````|`````````````\n");
        for (preset = 1, i = 1 ; i <= 8 ; i++, preset++)
        {
            putDecimal(preset, 4);  putstr("   |   ");
            putDecimal(g_Config.PresetMidiProgram[i & 7], 3);  NEW_LINE();
        }
        putstr("_______|_____________\n");
    }
    else if (argCount >= 3)   // Set Preset value
    {
        if (argVal[2][0] == '=' && argCount == 4)  midi_prgm = atoi(argVal[3]);
        else  midi_prgm = atoi(argVal[2]);

        if (preset >= 1 && preset <= 8 && midi_prgm < 128)
        {
            if (preset == 8)  preset = 0;  // preset #8 index is [0]
            g_Config.PresetMidiProgram[preset] = midi_prgm;
        }
        else  
        {
            isBadData = 1;
            putstr("! Invalid preset or MIDI prgm # \n");
        }
        
        if (!isBadData)
        {
            StoreConfigData();
            if (preset == 0)  preset = 8;  // for user interface
            putstr("* Preset ");  putDecimal(preset, 1);
            putstr(" : MIDI Prgm = ");  putDecimal(midi_prgm, 3);  
            NEW_LINE();
        }
    }
    else  putstr("! What?  (Try: preset -help) \n");  // Cmd syntax undefined
}


/*```````````````````````````````````````````````````````````````````````````````````````
 * CLI command function:  Cmnd_config
 *
 * The "config" command allows configuration parameters to be viewed or modified.
 * If a parameter value is changed, it will be committed to non-volatile storage.
 */
void  Cmnd_config(int argCount, char * argVal[])
{
    char    nickname[10];   // maximum 7 chars + NUL
    uint16  value;
    uint8   isBadData = 0;
    
    if (argVal[1][0] == '?' )  // help wanted
    {
        putstr( "Usage #1:  config    | List config'n param's \n\n" );
        putstr( "Usage #2:  config <param> [=] <value>    | Set param. value \n" );
        putstr( "   where <param> is a nickname listed by usage #1. \n\n" );
        putstr( "Example:   config  chan = 10    [Set Midi Tx channel = 10] \n\n" );
    }
    else if (argCount == 1)  // View param's  g_Config.Use2byteExpression
    {
        putstr("chan    ");  putDecimal(g_Config.MidiBasicChannel, 4);
        putstr("  Midi Basic Channel (1..16) \n");
        putstr("sysxen  ");  putDecimal(g_Config.MidiSysExclMsgEnabled, 4);
        putstr("  Midi Sys.Excl.Msg Enabled (0,1)\n");
        putstr("expcc   ");  putDecimal(g_Config.MidiExpressionCCnumber, 4);
        putstr("  Midi Expression CC number (0..31)\n");
        putstr("modcc   ");  putDecimal(g_Config.MidiModulationCCnumber, 4);
        putstr("  Midi Modulation CC number (0..31)\n");
        putstr("expint  ");  putDecimal(g_Config.MidiPressureInterval, 4);
        putstr("  Midi Expression Interval (5..50)\n");
        putstr("modint  ");  putDecimal(g_Config.MidiControllerInterval, 4); 
        putstr("  Midi Modulation Interval (10..100)\n");

        putstr("exp2b   ");  putDecimal(g_Config.Use2byteExpression, 4);
        putstr("  Send 2-byte Expression msg (0,1)\n");
        putstr("legen   ");  putDecimal(g_Config.LegatoModeEnabled, 4);
        putstr("  Legato Mode Enabled (0,1)\n");
        putstr("velsen  ");  putDecimal(g_Config.VelocitySenseEnabled, 4);
        putstr("  Velocity Sense Enabled (0,1)\n");
        putstr("benden  ");  putDecimal(g_Config.PitchBendEnabled, 4);
        putstr("  Pitch Bend Enabled (0,1)\n");
        putstr("padlay  ");  putDecimal(g_Config.TouchPadLayout, 4);
        putstr("  Touch Pad Layout (0,1)\n");
        putstr("fingsc  ");  putDecimal(g_Config.FingeringScheme, 4);
        putstr("  Fingering Scheme (0,1,2,..)\n");
        putstr("thres   ");  putDecimal(g_Config.TouchSenseThreshold, 4);
        putstr("  Touch Sense Threshold (max.250)\n");
        
        putstr("prspan  ");  putDecimal(g_Config.PressureSensorSpan, 4);
        putstr("  Pressure Sensor Span (max.700)\n");
        putstr("pbspan  ");  putDecimal(g_Config.PitchBendSpan, 4);
        putstr("  Pitch-Bend Span (max.750)\n");
        putstr("modmax  ");  putDecimal(g_Config.ModulationMaximum, 4);
        putstr("  Modulation Maximum (max.750)\n");
        putstr("modband ");  putDecimal(g_Config.ModulationDeadband, 4);
        putstr("  Modulation Dead-band (max.500)\n");
    }
    else if (argCount >= 3)   // Set param. value and verify EEPROM write
    {
        strncpy(nickname, (const char *) argVal[1], 8);
		nickname[7] = 0;
		
        if (argVal[2][0] == '=' && argCount == 4)  value = atoi(argVal[3]);
        else  value = atoi(argVal[2]);
        
        if (strmatch(nickname, "chan") && value >= 1 && value <= 16)
            g_Config.MidiBasicChannel = value;
        else if (strmatch(nickname, "sysxen") && value <= 1)
            g_Config.MidiSysExclMsgEnabled = value;
        else if (strmatch(nickname, "expcc") && value < 32)
            g_Config.MidiExpressionCCnumber = value;
        else if (strmatch(nickname, "modcc") && value < 32)
            g_Config.MidiModulationCCnumber = value;
        else if (strmatch(nickname, "expint") && value >= 5 && value <= 50)
            g_Config.MidiPressureInterval = value;
        else if (strmatch(nickname, "modint") && value >= 10 && value <= 100)
            g_Config.MidiControllerInterval = value;
        else if (strmatch(nickname, "exp2b") && value <= 1)
            g_Config.Use2byteExpression = value;
        else if (strmatch(nickname, "padlay") && value <= 1)
            g_Config.TouchPadLayout = value;
        else if (strmatch(nickname, "fingsc") && value <= 1)
            g_Config.FingeringScheme = value;
        else if (strmatch(nickname, "legen") && value <= 1)
            g_Config.LegatoModeEnabled = value;
        else if (strmatch(nickname, "benden") && value <= 1)
            g_Config.PitchBendEnabled = value;
        else if (strmatch(nickname, "velsen") && value <= 1)
            g_Config.VelocitySenseEnabled = value;
        else if (strmatch(nickname, "thres") && value >= 50 && value <= 250)
            g_Config.TouchSenseThreshold = value;
        else if (strmatch(nickname, "prspan") && value >= 100 && value <= 700)
            g_Config.PressureSensorSpan = value;
        else if (strmatch(nickname, "pbspan") && value >= 100 && value <= 750)
            g_Config.PitchBendSpan = value;
        else if (strmatch(nickname, "modmax") && value >= 100 && value <= 750)
            g_Config.ModulationMaximum = value;
        else if (strmatch(nickname, "modband") && value <= 500)
            g_Config.ModulationDeadband = value;
        else  
        {
            isBadData = 1;
            putstr("! Invalid nickname or value.\n");
        }
        
        if (!isBadData)  
        {
            StoreConfigData();
            putstr("* New value saved: ");  putDecimal(value, 4);
            NEW_LINE();
        }
    }
    else  putstr("! What?  Try 'config ?' \n");  // Cmd syntax undefined
}


/*```````````````````````````````````````````````````````````````````````````````````````
 * CLI command function:  Cmnd_mode
 *
 * The "mode" command is provided to set the "operating mode" of the handset,
 * i.e. to set the configuration profile to suit the connected synthesizer,
 * which may be a REMI synth module or any "generic" MIDI synth.
 * 
 * This command is useful where there is no MODE switch fitted on the handset.
 * Where a MODE switch is fitted, the "mode" command overrides the switch state,
 * so it is possible that the selected config profile may contradict the mode
 * indicated by the switch position.
 */
void  Cmnd_mode(int argCount, char * argVal[])
{
    char  option = tolower(argVal[1][1]);
    
    if (argVal[1][0] == '?' )  // help wanted
    {
        putstr( "Set operating mode, i.e. configuration profile.\n" );
        putstr( "Usage:  mode {-r|-g} \n" );
        putstr( "  -r : REMI synth,  -g : generic synth mode\n" );
        return;
    }
    
    if (option == 'g')  SetConfigProfile(GEN_SYNTH_MODE);
    if (option == 'r')  SetConfigProfile(REMI_SYNTH_MODE);
    
    if (g_Config.MidiSysExclMsgEnabled)  putstr("= REMI \n");
    else  putstr("= Generic \n");
}


/*```````````````````````````````````````````````````````````````````````````````````````
 * CLI command function:  Cmnd_tsens
 *
 * Diagnostic Utility for testing the touch sense functions.
 */
void  Cmnd_tsens(int argCount, char * argVal[])
{
    uint8  pad;
    char   option = 'l';  
    short  numPads = TouchPadsGetNumber();
    int    filt;
 
    if (argCount > 1)  option = tolower(argVal[1][1]);
    
    if (option == 'l')  // list .. default cmd option
    {
        putstr("________________________________\n");
        putstr("  Pad#  |  ADC raw  |  Filtered \n");
        putstr("````````|```````````|```````````\n");
        for (pad = 0 ; pad < numPads ; pad++)
        {
            putDecimal(pad, 5);  
            putstr("   |  ");
            putDecimal(TouchPadGetRawADC(pad), 5);
            putstr("    |  ");
            filt = TouchPadGetFiltered(pad);
            if (filt > 0) putDecimal(filt, 5);
            else  putstr(" --- ");
            NEW_LINE();
        }
        putstr("________|___________|___________\n");
    }
    else
    {
        putstr("! Invalid option. \n");
        putstr("Usage:  tsens [-l] \n");
    }
}


/*```````````````````````````````````````````````````````````````````````````````````````
 *   Function called by generic "diag" command ...
 */
void  DiagnosticCommandExec(int argCount, char * argValue[])
{
    static  uint32  captureTime;
    char   option = tolower(argValue[1][1]);

    if ( argCount == 1 || *argValue[1] == '?' )   // help wanted
    {
        putstr( "Usage:  diag  <option>  [arg1] ... \n" );
        putstr( "<option> \n" );
//      putstr( " -c :  Set MRF_CS# pin state (arg1 = H/L) \n"); 
        putstr( " -d :  Test Delay function or macro \n"); 
        putstr( " -e :  Show startup self-test Errors \n"); 
        putstr( " -f :  Write Flash data block (destructive) \n"); 
        putstr( " -h :  Heartbeat LED dis/enable (toggle) \n"); 
        putstr( " -m :  MIDI OUT TX test (send 0x0F contin.) \n"); 
        putstr( " -q :  MIDI OUT TX Queue test (IDENT @ 5ms) \n"); 
        putstr( " -o :  Task Overrun check \n"); 
        putstr( " -p :  Show MIDI Pressure data value (14b)\n"); 
        putstr( " -s :  Show note on/off State \n"); 
        putstr( " -v :  Activate diag. mode [timeout, mins]\n");
        putstr( " -x :  Exit (cancel) diagnostic mode. \n");
        ////
        return;
    }

    switch (option)
    {
    case 'c':  // Set MRF_CS# pin state
    {
        char   arg = toupper(*argValue[2]);
        
        if (arg == '1' || arg == 'H')  MRF_CS_HIGH();  
        if (arg == '0' || arg == 'L')  MRF_CS_LOW();
        break;
    }
    case 'd':  // Delay macro/function test (for development)
    {
        putstr("Test execution time of DELAY_1us() macro. \n");
        putstr("Connect scope probe to TP2. \n");
        putstr("TP2 pulses High for 5 iterations of DELAY_1us(). \n");
        putstr("Reset MCU to exit... \n");
        
        GlobalInterruptDisable();    // Suspend IRQ's and B/G tasks
        
        while (1)
        {
            TESTPOINT_TP2_SET_HI();
            DELAY_1us();
            DELAY_1us();
            DELAY_1us();
            DELAY_1us();
            DELAY_1us();
            TESTPOINT_TP2_SET_LO();
            DELAY_1us();
            DELAY_1us();
            DELAY_1us();
        }
        break;
    }
    case 'e':  // Self-test Errors  
    {
        if (g_SelfTestErrors == 0)  
            putstr("* No self-test errors.\n");
        if (g_SelfTestErrors & (1 << TEST_CONFIG_INTEGRITY))  
            putstr("! Error: Config Data (defaulted)\n");
        if (g_SelfTestErrors & (1 << TEST_PRESSURE_SENSOR))   
            putstr("! Error: Pressure Sensor\n");
        if (g_SelfTestErrors & (1 << TEST_MODULATION_SENSOR)) 
            putstr("! Error: Modulation Sensor\n");
        break;
    }
    case 'o':  // Task Overrun check  
    {
        uint8  overruns;
        captureTime = millisecTimer();
        v_TaskOverrunCount = 0;
        while (millisecTimer() < (captureTime + 200))
        {
            BackgroundTaskExec();  // loop for 200ms
        }
        overruns = v_TaskOverrunCount / 2;  // in 100ms
        putstr("Task overruns in 100ms: ");
        putDecimal(overruns, 1);  NEW_LINE();
        break;
    }
    case 'p':  // Show MIDI pressure level (expression CC message data value)
    {
        putDecimal(GetMidiPressureLevel(), 5);  putstr("\n");
        break;
    }
    case 's':  // Show note on/off state
    {
        uint16  state = GetNoteOnOffState();
        
        if (state == NOTE_OFF_IDLE)  putstr("OFF/IDLE (0)\n");
        else if (state == NOTE_ON_PENDING)  putstr("PENDING (1)\n");
        else if (state == NOTE_ON_PLAYING)  putstr("PLAYING (2)\n");
        else  putstr("UNDEFINED (error)\n");
        break;
    }
    case 'f':  // Write flash data block at 0x7FC0  
    {
        static char  *argv[] = { "dump ", "7F00 " };
        
        FlashWriteBlock((uint8 *) &g_Config, 0x7FC0);
        Cmnd_dump(2, &argv[0]);
        break;
    }
    case 'h':  // Enable/Disable (toggle) heartbeat LED flashing
    {
        g_HeartbeatLEDactive = !g_HeartbeatLEDactive;
        break;
    }
    case 'm':  // MIDI transmitter test (send 0x0F byte continuously)
    {
        char  key = 0;
        EnableDiagnosticMode();   // suspend background tasks
        putstr("Sending 0x0F continuously to MIDI TX UART...\n");
        putstr("Hit [Esc] to exit. \n");
        while (key != ASCII_ESC)
        {
            EUSART2_WriteByte(0x0F);  // TX direct to UART2
            
            if (RxDataAvail()) key = getbyte();  // console key hit
        }
        CancelDiagnosticMode();  // resume normal operation
        break;
    }
    case 'q':  // MIDI transmit queue test (send Sys.Excl.IDENT msg continuously)
    {
        char  key = 0;
        EnableDiagnosticMode();   // suspend background tasks
        putstr("Sending Sys.Ex. 'IDENT' msg continuously to MIDI OUT \n");
        putstr("via MIDI transmit queue.  Message interval is 5ms. \n");
        putstr("Hit [Esc] to exit... \n");
        captureTime = millisecTimer();
        
        while (key != ASCII_ESC)
        {
            if (millisecTimer() >= (captureTime + 5))  // every 5ms ...
            {
                MIDI_PutByte(SYS_EXCLUSIVE_MSG);   // status byte (0xF0)
                MIDI_PutByte(SYS_EXCL_REMI_ID);    // manuf/product ID (0x73)
                MIDI_PutByte(REMI_IDENT_MSG);      // Msg type (0x30)
                MIDI_PutByte(SYSTEM_MSG_EOX);      // end-of-msg code (0xF7)
                captureTime = millisecTimer();
            }
            MIDI_TxQueueHandler();
            
            if (RxDataAvail()) key = getbyte();  // console key hit
        }
        CancelDiagnosticMode();  // resume normal operation
        break;
    }
    case 'v':  // Activate diagnostic mode
    {
        int   arg = str2int(argValue[2]);  // minutes

        if (argCount == 3 && arg != 0)  // timeout value supplied
            g_DiagnosticModeTimeout = arg;
        
        EnableDiagnosticMode();
        
        putstr("* Diagnostic mode is active... Timeout in ");
        putDecimal(g_DiagnosticModeTimeout, 1);
        putstr(" minutes.\n");
        putstr("  Use 'diag -x' cmd to cancel anytime.\n");
        break;
    }
    case 'x':  // Exit diagnostic mode
    {
        CancelDiagnosticMode();
        break;
    }
    default:  
    {
        putstr("! What? (Invalid cmd option) \n");
        break;
    }
    } // end switch
}


#if FALSE
/*```````````````````````````````````````````````````````````````````````````````````````
 * CLI command function:  Cmnd_mrf
 *
 * Diagnostic utility functions to support wireless module MRF24J40MA.
 */
void  Cmnd_mrf(int argCount, char * argVal[])
{
    static  rx_info_t  *rx_inf;        // pointer to rx_info
    static  uint16  destNodeAddr = 0;  
    static  char    tx_msg[64];
    static  uint16  pktSeqNum = 0;
    char    textBuf[80];
    char    opt_c1, opt_c2 = 0;
    int     i, arg1, arg2;
    int     rx_len = 0;             // size of rx packet payload
    char   *rx_str, *tx_dat;

    if (argCount == 1 || *argVal[1] == '?' )  // help
    {
        putstr( "Diagnostic utility for MRF24J40 wireless module.\n" );
        putstr( "Usage:  mrf  <option>  [arg1] [arg2] ... \n" );
        putstr( "options \n" );
        putstr( "  -z  : Initialize the device (reset & configure) \n" );
        putstr( "  -d  : Dump handset status structure (as hex bytes) \n" );
/*        
        putstr( "  -rr : Read MRF24J40 register (arg1 = short.addr, hex) \n" );
        putstr( "  -wr : Write MRF24J40 reg. (arg2:data = decimal or hex^) \n" );
        putstr( "        ^ Use $ prefix for hex data (eg. $0F) \n" );
*/
        putstr( "  -p  : Get/Set wireless PAN ID [arg1 = PAN.id, hex] \n" );
        putstr( "  -sn : Get/Set source node address [arg1 = addr, hex] \n" );
        putstr( "  -tx : Transmit test packet [arg1 = destin.node, hex] \n" );
        putstr( "  -rx : View last received packet \n" );
//      putstr( "  -x  : Exit diagnostic mode, resume normal operation.\n" );
        return;
    }

    if (argVal[1][0] == '-')
    {
        opt_c1 = tolower(argVal[1][1]);
        opt_c2 = tolower(argVal[1][2]);
    }

    if (opt_c1 == 'z')  // Initialize the device (reset & configure)
    {
        if (WirelessModuleInit() == ERROR)
        {
            putstr("! Wireless module not detected.\n");
        }
        
        pktSeqNum = 0;
    }
    if (opt_c1 == 'd')    // Dump handset status structure
    {
        tx_dat = (uint8 *) &g_Handset;
        putstr("\n  Tx Data: ");
        
        for (i = 0;  i < sizeof(HandsetStatusPacket_t);  i++)
        {
            putHexByte(*tx_dat++);  putbyte(' ');
        }
        putstr("\n");
    }
    else if (opt_c1 == 'p')  // Get/Set PAN ID  [arg1 = PAN.id, hex]
    {
        if (argCount >= 3)
        {
            arg1 = hexatoi(argVal[2]);
            if (arg1 > 0) 
            {
                g_Wireless_PAN_ID = arg1;
                MRF_SetPAN(g_Wireless_PAN_ID);
            }
        }
        
        g_Wireless_PAN_ID = MRF_GetPAN();
        putstr("PAN ID (from RF module): $");
        putHexWord(g_Wireless_PAN_ID);
        putstr("\n");
    }
    else if (opt_c1 == 's')  // Get/Set source node address [arg1]
    {
        if (argCount >= 3)
        {
            arg1 = hexatoi(argVal[2]);
            if (arg1 > 0) 
            {
                g_HandsetNodeAddress = arg1;
                MRF_Address16Write(g_HandsetNodeAddress);
            }
        }
        
        g_HandsetNodeAddress = MRF_Address16Read();
        putstr("Node Addr (from RF module): $");
        putHexWord(g_HandsetNodeAddress);
        putstr("\n");
    }
	else if (opt_c1 == 'r' && opt_c2 == 'x')  // Display data from last received packet
    {
        rx_inf = MRF_RxInfoGet();
        rx_len = MRF_RxDataLength();
        rx_str = (char *) &rx_inf->rx_data[0];
		if (*rx_str >= 0x80) rx_str++;  // skip message type ID, if any

		// Read and display Rx message (string)
		if (rx_len > 64)  
			putstr("! Rx Error.\n");  // invalid packet
		else  // valid pkt payload
		{
			putstr("* Rx Msg:  ");
			for (i = 0;  i < rx_len;  i++, rx_str++)
			{
				if (isprint(*rx_str))  putbyte(*rx_str);
			}
			putstr("\n");
			putstr("  Link Quality: ");  
			putDecimal(rx_inf->lqi / 25, 1);
			putstr("/10 \n");
			putstr("  Rx Signal: ");  
			putDecimal(rx_inf->rssi / 25, 1);
			putstr("/10 \n");
		}
 	}
    else if (opt_c1 == 't' && opt_c2 == 'x')  // Transmit test packet [arg1 = dest.node]
    {
        if (argCount >= 3)
        {
            arg1 = hexatoi(argVal[2]);
            if (arg1 > 0)  destNodeAddr = arg1;
        }
		
		if (destNodeAddr == 0)   // if uninitialized...
		    destNodeAddr = g_ControllerNodeAddress;  // use default
        
        pktSeqNum++;  // Bump packet sequence number
        wordToHexStr(pktSeqNum, textBuf);
        
        strcpy(tx_msg, "Test Packet #");
        strcat(tx_msg, textBuf);    // Warning: should be (const char *)textBuf !
        
        MRF_TransmitData(destNodeAddr, (char *)tx_msg, strlen(tx_msg));
    }
    // 
    // ...to be continued...
    //
    else  putstr("! What? \n");  // Cmd option undefined
}
#endif

#if 0
/*`````````````````````````````````````````````````````````````````````````````````````````````````
 *   CLI command function:  Cmnd_trace
 *
 *   The "trace" command outputs the debug trace buffer.
 *   Contents of the trace buffer depends on application-specific debug code.
 */
void  Cmnd_trace(int argCount, char *argValue[])
{
    char   textBuf[100];
    int    line;
    float  var1, var2, var3, var4;

    if (!SuperUserAccess()) return;

    if (*argValue[1] == '?')   // help wanted
    {
        putstr( "Usage:  trace \n" );
        putstr( "Dumps the debug trace buffer. \n" );
        ////
        return;
    }

    putstr("   T ms  final  smoothed  atten.duty \n");

    for (line = 0;  line < 100;  line++)
    {
        var1 = FixedToFloat(g_TraceBuffer[line][1]);  // final ampld
        var2 = FixedToFloat(g_TraceBuffer[line][2]);  // smoothed final ampld

        sprintf(textBuf, "%7d  %7.3f  %7.3f  %7d \n",
                g_TraceBuffer[line][0], var1, var2, g_TraceBuffer[line][3]);
        putstr(textBuf);
    }
}
#endif


/*```````````````````````````````````````````````````````````````````````````````````````
 *   Function called by "watch" command function...
 *   Variables to be "watched" are output on a single line (NO NEW_LINE).
 */
void  WatchCommandExec(void)
{
    putstr("TouchPads: 0x");  putHexWord(TouchPadStates());
    putstr(" | Pad0_ADC: ");  putDecimal(TouchPadGetRawADC(0), 4);
    putstr(" | Pressure: ");  putDecimal(GetPressureRawReading(), 4);
    putstr(" | Modul'n: ");  putDecimal(GetModulationRawReading(), 4);
}


/*```````````````````````````````````````````````````````````````````````````````````````
 *   Function called by "default" command function...
 *   Restores "factory default" values to persistent data (in EEPROM).
 */
void  DefaultPersistentData(void)
{
    DefaultConfigData();
}


/**
 * Functions:    void EnableDiagnosticMode(),  void CancelDiagnosticMode()
 *
 * Overview:     Enables system "Diagnostic Mode" which inhibits background tasks
 *               which might otherwise cause contention with diagnostic functions.
 *               Diagnostic Mode is canceled automatically after a preset timeout, or by
 *               a call to function CancelDiagnosticMode(), or by the command "diag -x".
 */
void  EnableDiagnosticMode(void)
{
    g_DiagModeTimer_ms = (unsigned) g_DiagnosticModeTimeout * 60 * 1000;  // ms
    g_DiagnosticModeActive = 1;
}


void  CancelDiagnosticMode(void)
{
    g_DiagnosticModeActive = 0;
}


// end of file
