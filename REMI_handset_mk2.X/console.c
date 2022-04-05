/*________________________________________________________________________________________*\

  File:        console.c

  Author:      M.J.Bauer  [www.mjbauer.biz]

  This module implements a generic console Command Line Interface (CLI).
  The code is meant to be customized to suit a particular application.

\*________________________________________________________________________________________*/

#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include "console.h"
#include "low_level.h"
#include "touch_sense.h"
#include "main_remi_handset_mk2.h"

//---------------------  Application-specific externals  ------------------------------------------
//
extern uint8 g_FW_version[];        // firmware version # (major, minor, build)

// The application must provide these "callback" functions, even if they do nothing:
extern void  DefaultConfigData(void);   // Restore factory defaults to EEPROM
extern void  WatchCommandExec(void);
extern void  BootReset(void);

//-------------------------------  Private data  --------------------------------------------------
//                     
static  char   CLIprompt[] = "\r> ";  // Command line prompt
static  char   CmndLine[CMND_LINE_MAX_LEN+2];  // Command line buffer
static  char  *CmndLinePtr;   // Pointer into Cmnd Line buffer
static  int    iCmndLineLen;  // Cmnd line length (char count)
static  int    m_argCount;    // Number of Cmnd Line args (incl. Cmnd name)
static  char  *m_argValue[CLI_MAX_ARGS];   // Array of pointers to Command Line args
static  char   NullString[] = { '\0' };  // Empty string

static  char   CmndHistoryBuffer[CMD_HIST_BUF_SIZE][CMND_LINE_MAX_LEN+2];
static  int16  CmndHistoryMarker;  // Index to next free place in Command History Buf
static  int16  CmndRecallMarker;   // Index of previous command to be recalled
static  char   HistoryInitialised = FALSE;

/*----------------------------------------------------------------------------------------
*                      C O M M A N D   T A B L E
*/
const  struct  CmndTableEntry_t  Command[] =
{
    //   Cmd Name      Attribute      Cmd Function
    //----------     -------------   -------------------
    {    "*",          GEN_CMD,       Cmnd_remark     },  // Generic commands
    {    "help",       GEN_CMD,       Cmnd_help       },
    {    "lc",         GEN_CMD,       Cmnd_list       },
    {    "cmds",       GEN_CMD,       Cmnd_list       },
    {    "default",    GEN_CMD,       Cmnd_default    },
    {    "ver",        GEN_CMD,       Cmnd_ver        },
    {    "diag",       GEN_CMD,       Cmnd_diag       },
    {    "watch",      GEN_CMD,       Cmnd_watch      },
    {    "reset",      GEN_CMD,       Cmnd_reset      },
    {    "dump",       GEN_CMD,       Cmnd_dump       },
//  {    "eeprom",     GEN_CMD,       Cmnd_eeprom     },
    //---------------------------------------------------
    {    "preset",     APP_CMD,       Cmnd_preset     },  // Application commands
    {    "config",     APP_CMD,       Cmnd_config     },
//  {    "mode",       APP_CMD,       Cmnd_mode       },
//  {    "tsens",      APP_CMD,       Cmnd_tsens      },
    //---------------------------------------------------
    {    "$",          0,             NULL            }   // Dummy last entry
} ;


/*----------------------------------------------------------------------------------------
*                         COMMAND LINE INPUT STREAM HANDLER
*
*   Called frequently from any wait loop(s) in the main process.
*   Must not be called from the Background Task Executive, because command functions
*   which incorporate time delays do call the B/G Task Executive and hence there
*   would be potential risk of infinite recursion; e.g. see Cmnd_watch().
*
*   If the serial port input (RX) buffer is not empty, the function will...
*     .  Input a char from the serial RX buffer;
*     .  Provide simple line editing (Backspace, Ctrl-X);
*     .  Append the char, if printable, to the command line input buffer;
*     .  If CR received (cmd line terminator), call the CommandLineInterpreter();
*     .  If Ctrl+R received, recall previously entered command from history buffer;
*     .  If TAB received, it is converted to a single space;
*     .  Otherwise, ASCII CTRL chars are ignored.
*/
void  ConsoleCLI_Service(void)
{
    char  c;

    if (RxDataAvail())     // char(s) available in serial input buffer
    {
        c = getch();             // Fetch the char... no echo (yet)
        switch (c)
        {
        case ASCII_CAN:                 // Ctrl+X... cancel line...
            EraseLine();
            PrepareForNewCommand();
            break;

        case ASCII_BS:                  // BACKSPACE...
            if (iCmndLineLen > 0)
            {
                CmndLinePtr-- ;           // Remove last-entered char from buffer
                iCmndLineLen-- ;
                putch(ASCII_BS);          // Backspace the VDU cursor
                putch(' ');               // Erase offending char at VDU cursor
                putch(ASCII_BS);          // Re-position the VDU cursor
            }
            break;

        case ASCII_CR:                  // ENTER...
            putch('\r');                  // Echo NewLine
            putch('\n');
            if (iCmndLineLen > 0)         // Got a command string...
            {
                *CmndLinePtr = 0;         // Terminate the command string
                EnterCommandInHistory();    // Enter it into the history buffer
                CommandLineInterpreter();   // Interpret and execute the cmnd.
            }
            PrepareForNewCommand();         // CR
            break;

        case ASCII_DC2:                 // Ctrl+R... recall command
            EraseLine();                    // Trash the current command line
            PrepareForNewCommand();         // Output the prompt
            RecallCommand();                // Retrieve previous command from history
            break;

        case ASCII_TAB:                 // TAB...  convert to single space
            c = ' ';
        //  no break... fall thru to default case
        default:
            if (isprint(c) && iCmndLineLen < CMND_LINE_MAX_LEN)
            {
                putch(c);                 // Echo char
                *CmndLinePtr++ = c;       // Append char to Cmnd Line buffer
                iCmndLineLen++ ;
            }
            break;
        } // end_switch
    }
}


/*----------------------------------------------------------------------------------------
*                    C O M M A N D   L I N E   I N T E R P R E T E R
*
*   Processes the received command line when entered.
*
*   The CommandLineInterpreter() function is called by ConsoleCLI_Service()
*   when a valid command string is entered (CR received). It finds & counts "arguments"
*   in the command line and makes them NUL-terminated strings in-situ.
*   It then searches the Command Table for command name, m_argValue[0],
*   and if the name is found, executes the respective command function.
*
*   If there is a cmd line argument following the cmd name and it is "-help" (Help option),
*   the argument is converted to "?", which is the (simpler) alternative syntax.
*
*   A command string is comprised of a command name and one or more "arguments" separated
*   by one or more spaces, or TABs. The ordering of arguments is determined by particular
*   command functions, so may or may not be important. The degree of syntax checking is also
*   the responsibility of command functions.
*
*   Command names, hexadecimal arg's, and option switches (e.g. "-a") are not case-sensitive;
*   other arguments may or may not be case-sensitive, as interpreted by particular command
*   functions. (The strmatch() function is not case-sensitive.)
*/
void  CommandLineInterpreter(void)
{
    char    c;
    uint8   idxArg, idxCmd;
    char    cmdNameFound = 0;

    CmndLinePtr = CmndLine;             // point to start of Cmnd Line buffer
    m_argCount = 0;

    // This loop finds and terminates (with a NUL) any user-supplied arguments...
    for (idxArg = 0;  idxArg < CLI_MAX_ARGS;  idxArg++)
    {
        if (!isprint(*CmndLinePtr))               // stop at end of line
            break;
        while (*CmndLinePtr == ' ')                 // skip leading spaces
            CmndLinePtr++ ;
        if (!isprint(*CmndLinePtr))               // end of line found
            break;
        m_argValue[idxArg] = CmndLinePtr;            // Make ptr to arg
        m_argCount++ ;

        while ((c = *CmndLinePtr) != ' ')         // find first space after arg
        {
            if (!isprint(c))                        // end of line found
                break;
            CmndLinePtr++ ;
        }
        if (!isprint(*CmndLinePtr))               // stop at end of line
            break;
        *CmndLinePtr++ = 0;                       // NUL-terminate the arg
    }

    // This loop searches the command table for the supplied command name...
    for (idxCmd = 0;  idxCmd < MAX_COMMANDS;  idxCmd++)
    {
        
        if (*Command[idxCmd].Name == '$')     // reached end of table
            break;
        if (strmatch(m_argValue[0], Command[idxCmd].Name))
        {
            cmdNameFound = 1;
            break;
        }
    }

    if (m_argCount > 1)       // If there is one or more user-supplied arg(s)...
    {
        if (strmatch(m_argValue[1], "-help"))       // convert "-help" to '?' ...
            *m_argValue[1] = '?';                    // ... to simplify cmd fn
    }

    if (cmdNameFound)
    {
        (*Command[idxCmd].Function)(m_argCount, m_argValue);     // Do it
    }
    else  putstr("? Undefined command.\n");
}


/*
*   Flush Command Line buffer, clear CLI arg's and output CLI prompt.
*   This function must be called before the first call to ConsoleCLI_Service()
*   to initialize the CLI environment.
*/
void  PrepareForNewCommand(void)
{
    uint8  idx;

    CmndLine[0] = 0;
    CmndLinePtr = CmndLine;  // point to start of Cmnd Line buffer
    iCmndLineLen = 0;
    for (idx = 0;  idx < CLI_MAX_ARGS;  idx++)   // Clear CLI args
        m_argValue[idx] = NullString;
    putstr(CLIprompt);
}


/*
*   Copy the newly entered command into the history buffer for later recall...
*   if it's length is non-zero.
*/
void  EnterCommandInHistory(void)
{
    short  line;

    if (!HistoryInitialised)  // One-time start-up initialization
    {
        for (line = 0 ; line < CMD_HIST_BUF_SIZE ; line++)
        {
            CmndHistoryBuffer[line][0] = 0;    // make empty cmnd string
        }
        CmndHistoryMarker = 0;
        CmndRecallMarker = 0;
        HistoryInitialised = TRUE;
    }

    if (strlen(CmndLine) != 0)   // Not an empty cmnd string
    {
        strncpy(CmndHistoryBuffer[CmndHistoryMarker], CmndLine, CMND_LINE_MAX_LEN);
        CmndRecallMarker = CmndHistoryMarker;
        CmndHistoryMarker++ ;
        if (CmndHistoryMarker >= CMD_HIST_BUF_SIZE) CmndHistoryMarker = 0;
    }
}


/*
*   Recall a previously entered command from the history buffer...
*   The function selects the next previous command from the buffer (if any)
*   as indicated by CmndRecallMarker, and outputs the command string to the user's
*   terminal for editing. At the same time, the selected command is copied to the
*   current command line buffer.
*   The selected command is not executed until the user hits ENTER (CR).
*/
void  RecallCommand(void)
{
    strncpy(CmndLine, CmndHistoryBuffer[CmndRecallMarker], CMND_LINE_MAX_LEN);
    if (CmndRecallMarker == 0) CmndRecallMarker = CMD_HIST_BUF_SIZE;
    --CmndRecallMarker;
    iCmndLineLen = strlen(CmndLine);
    CmndLinePtr = CmndLine + iCmndLineLen;
    *CmndLinePtr = 0;
    putstr(CmndLine);
}


/*
*   Erase the command line on user terminal; cursor remains on same line at col 1.
*/
void  EraseLine(void)
{
    short  col;

    putch('\r');
    for (col=0 ; col < (CMND_LINE_MAX_LEN+2) ; col++)
    {
        putch(' ');
    }
    putch('\r');
}


/*****
*   Function:   strmatch
*
*   Purpose:    Compares two NUL-terminated strings, each up to 255 chars in length.
*               Returns 1 if the strings have the same contents, up to the terminator
*               of either string, except that the comparison is not case-sensitive.
*
*   Args:       (char *) pcStr1       :  pointer to string variable
*               (const char *) phStr2 :  pointer to string literal
*
*   Returns:    (FLAG) 1 if string #1 matches string #2 (case-insensitive), else 0.
*/
uint8  strmatch(char *pcStr1, const char *phStr2)
{
    char    c1, c2;
    uint8   b = 255;
    uint8   yResult = 1;

    while (b-- != 0)
    {
        c1 = tolower(*pcStr1++);
        c2 = tolower(*phStr2++);
        if (c1 != c2)  yResult = 0;
        if (c1 == 0 || c2 == 0)  break;
    }
    return yResult;
}


#ifdef INCLUDE_GETSTR_FUNC
/*****
*   Function:   getstr
*
*   Purpose:    Gets a text string from the console input stream (user terminal).
*               While waiting for input, background tasks are executed.
*               Input is terminated (with a NUL) and the function returns when a CR
*               (ASCII 0x0D) code is received.
*               BACKSPACE removes last received char from the input buffer.
*               TAB is converted to a single space.
*               Otherwise, CTRL chars are ignored.
*
*   Args:       (char *) strBuf :  pointer to input buffer
*               (int) maxLen : maximum length of input string (not incl. NUL)
*
*   Returns:    Length of input string (bytes) up to NUL terminator.
*/
int  getstr(char *strBuf, int maxLen)
{
    char  c;
    char  *inBuf = strBuf;
    char  exit = 0;
    char  length = 0;

    while (!exit)
    {
        if (RxDataAvail()) 
        {
            c = getch();  // no echo (yet)
            switch (c)
            {
            case ASCII_BS:     // BACKSPACE
                if (length > 0)
                {
                    inBuf-- ;  length--;
                    putch(ASCII_BS);
                    putch(' ');
                    putch(ASCII_BS);
                }
                break;

            case ASCII_CR:     // ENTER
                putstr("\n");
                *inBuf = 0;
                exit = 1;
                break;

            case ASCII_TAB:
                c = ' ';
                //\/\/\/\/
            default:
                if (isprint(c) && length < maxLen)
                {
                    putch(c);    // echo
                    *inBuf++ = c;
                    *inBuf = 0;
                    length++;
                }
                break;
            } // end_switch
        }
        BackgroundTaskExec();
    }

    return length;
}
#endif


/*________________________________________________________________________________________*\
*
*              G E N E R I C   C L I   C O M M A N D   F U N C T I O N S
*
*   All CLI command functions have access to user-supplied command line arguments.
*   These are passed to the function via two arguments (parameters)...
*
*     (int) argCount : number of user-supplied command line arguments + 1
*                       (i.e. including the command name, argValue[0])
*
*     (char *) argValue[] : array of pointers to user-supplied argument strings
*
\*________________________________________________________________________________________*/
/*
*
*   CLI command function:  Cmnd_help
*
*   The "help" command (alias "?") gives brief help on CLI usage.  Edit to suit app.
*/
void  Cmnd_help(int argCount, char * argValue[])
{
    putstr("Command arg's and options must be separated by spaces.\n");
    putstr("To list available commands, type 'lc' or 'cmds'.\n");
    putstr("To recall a previously entered command, hit Ctrl+R.\n");
    putstr("Some commands, when entered with the switch '-help' or '?', show command \n");
    putstr("usage information;  otherwise, a command which expects at least one arg, \n");
    putstr("when entered without any, should also show command usage. \n");
/*
    putstr("In usage info, square brackets enclose optional parameters, e.g. [<x1>]; \n");
    putstr("braces enclose alternative options, e.g. {-x|-y }, where '|' means 'OR'. \n");
*/    
}


/**
*   CLI command function:  Cmnd_remark
*
*   The "remark" command (*) allows a comment line to be entered.
*   Handy when a terminal session log file is being created.
*/
void  Cmnd_remark(int argCount, char * argValue[])
{
    if (*argValue[1] == '?')  putstr("Usage:  * [remark] \n");  // help
}


/**
*   CLI command function:  Cmnd_list
*
*   The "lc" command (alias "cmds") lists CLI command names, formatted into 6 columns.
*   Commands are listed in alphabetical order (sorted by initial char only).
*/
void  Cmnd_list(int argCount, char * argValue[])
{
    char    *commandName;
    char    sort_char;
    short   iCmd;
    short   iSpacesToPad;
    short   iColumn = 0;

    for (sort_char = 'a'; sort_char <= 'z'; sort_char++)
    {
        for (iCmd = 1;  iCmd < MAX_COMMANDS;  iCmd++)   // Search command table
        {
            commandName = Command[iCmd].Name;
            if (commandName[0] == '$')  break;            // Reached end of table

            if (commandName[0] == sort_char)
            {
                putstr(commandName);
                iSpacesToPad = 12 - strlen(commandName);
                while (iSpacesToPad-- != 0)
                {
                    putch(' ');
                }
                if (++iColumn >= 6)  { iColumn = 0;  putNewLine(); }
            }
        }
    }
    if (iColumn != 0) putNewLine();
}


/*****
*   CLI command function:  ver
*
*   The "ver" command displays firmware version number and other build information.
*/
void  Cmnd_ver(int argCount, char * argValue[])
{
    putstr("Firmware version ");
    putDecimal(g_FW_version[0], 1);
    putch('.');
    putDecimal(g_FW_version[1], 1);
    putch('.');
    if (g_FW_version[2] < 10) putch('0');
    putDecimal(g_FW_version[2], 1);
    putstr(", ");
    putstr(__DATE__);
    putNewLine();
}


/*****
*   CLI command function:  Cmnd_default
*
*   The "default" command restores "factory default" values to all persistent data,
*   typically held in EEPROM or a reserved block in flash program memory.
*/
void  Cmnd_default(int argCount, char * argValue[])
{
    char  c = 0;
    bool  confirm = FALSE;

    if (argCount == 1)  // Cmd name only...
    {
        putstr("Restore persistent data to factory defaults? (Y/N): ");
        while (c < 0x20)
        {
            if (RxDataAvail()) c = getch();
        }
        putch(c);  putstr("\n");
        if (toupper(c) == 'Y') confirm = TRUE;
    }
    else if (strmatch(argValue[1], "-y")) confirm = TRUE;

    if (confirm) 
    {
        DefaultConfigData();
        putstr("* Default configuration restored.\n");
    }
}


/*****
*   CLI command function:  Cmnd_reset
*
*   The "reset" command invokes a reset of the system.
*   The mode of reset is determined by the application "call-back" function.
*/
void  Cmnd_reset(int argCount, char * argValue[])
{
    BootReset();
}


/*^***
*   CLI command function:  Cmnd_dump
*
*   The "dump" command outputs a 256-byte page of flash program memory.
*   This function is device dependent. Target device is PIC18F..K22.
*
*   Command syntax:  "dump [addr]"
*   ... where <addr> is the start address (hex), masked to a 16-byte boundary.
*   If <addr> is omitted, dump next page.
*
*/
void  Cmnd_dump(int argCount, char *argValue[])
{
    static  uint16  startAddr;
    uint16  addr;
    uint8   row, col;
    char    c;
    
    if (argCount == 1 || *argValue[1] == '?')   // help wanted
    {
        putstr("Dump a page (256 bytes) of flash program memory.\n");
        putstr("Usage:  dump [addr] \n");
        putstr("-- where addr is start address (hex).\n");
        return;
    }
    
    if (argCount >= 2)  // assume arg[1] is <addr>
        startAddr = hexatoi(argValue[1]) & 0xFFF0;  // mask to 16-byte boundary

    for (addr = startAddr, row = 0;  row < 16;  row++)
    {
        putHexWord(addr);
        putstr(" : ");
        for (col = 0;  col < 16;  col++)
        {
            putHexByte(FlashReadByte(addr++));  // print 2-digit hex
            putch(' ');
            if (col % 8 == 7) putch(' ');
        }
        for (addr -= 16, col = 0;  col < 16;  col++)
        {
            c = FlashReadByte(addr++);
            if (isprint(c))  putch(c);  // print ASCII symbol
            else  putch('.');
        }
        putNewLine();
    }
    
    startAddr += 256;  // next page 
}


/*****
*   CLI command function:  Cmnd_watch
*
*   The "watch" command activates a real-time display which shows the value(s)
*   of one or more global application variables.
*   The function exits when it receives ASCII_ESC from the serial input stream,
*   ie. when user hits the [Esc] key on their terminal.
*/
void  Cmnd_watch(int argCount, char * argValue[])
{
    uint32 start_time;
    uint16 elapsed_time = 0;  // millisec since watch started
    char c = 0;

    putstr("Hit [Esc] to quit. \n\n");

    while (c != ASCII_ESC)
    {
        WatchCommandExec();  // App-specific data output

        // Delay 250mS for refresh rate of 5Hz.
        // While waiting, any pending background tasks are executed.
        start_time = milliseconds();
        while ((milliseconds() - start_time) < 250)
        {
            BackgroundTaskExec();
        }
        putch('\r');       // Return VDU cursor to start of output line
        if (kbhit()) c = getch();       // Check for key hit
        if (c == ASCII_CR)  { c = 0;  putNewLine(); }
        elapsed_time += 50 ;
    }
    putNewLine();
}


/*________________________________________________________________________________________*\
*
*            A P P L I C A T I O N   C O M M A N D   F U N C T I O N S
\*________________________________________________________________________________________*/

/**
 *   Application-specific data output function called by "watch" command...
 *   Variables to be "watched" are output on a single line (no newline).
 */
void  WatchCommandExec(void)
{
    putstr("TouchPads: 0x");  putHexWord(TouchPadStates());
    putstr(" | Pad0_ADC: ");  putDecimal(TouchPadGetRawADC(0), 4);
    putstr(" | Pressure: ");  putDecimal(GetPressureRawReading(), 4);
    putstr(" | Modul'n: ");  putDecimal(GetModulationRawReading(), 4);
}


/*```````````````````````````````````````````````````````````````````````````````````````
 * CLI command function:  Cmnd_config
 *
 * The "config" command allows configuration parameters to be viewed or modified.
 * If a parameter value is changed, it will be committed to non-volatile storage.
 */
void  Cmnd_config( int argCount, char *argValue[] )
{
    char    nickname[10];   // maximum 7 chars + NUL
    uint16  value;
    bool    isBadData = FALSE;
    
    if (argValue[1][0] == '?')  // help wanted   
    {
        putstr("Usage #1:  config    | List config'n param's \n\n");
        putstr("Usage #2:  config <param> [=] <value>    | Set param. value \n");
        putstr("   where <param> is a nickname listed by usage #1. \n\n");
        putstr("Example:   config  chan = 10    [Set Midi Tx channel = 10] \n\n");
    }
    else if (argCount == 1)  // View param's 
    {
        putstr("chan    ");  putDecimal(g_Config.MidiBasicChannel, 4);
        putstr("  Midi Basic Channel (1..16) \n");
        putstr("sysxen  ");  putDecimal(g_Config.MidiSysExclMsgEnabled, 4);
        putstr("  Midi Sys.Excl.Msg Enabled (0,1)\n");
        putstr("prchen  ");  putDecimal(g_Config.MidiProgChangeEnabled, 4);
        putstr("  Midi Prog. Change Enabled (0,1)\n");
        putstr("expncc  ");  putDecimal(g_Config.MidiExpressionCCnumber, 4);
        putstr("  Midi Expression CC number (2,7,11)\n");
        putstr("expnint ");  putDecimal(g_Config.MidiPressureInterval, 4);
        putstr("  Midi Expression Interval (5..50 ms)\n");
        putstr("expn14b ");  putDecimal(g_Config.Send14bitExprnData, 4);
        putstr("  Send 14-bit Expression data (0,1)\n");
        putstr("modint  ");  putDecimal(g_Config.MidiControllerInterval, 4); 
        putstr("  Midi Modulation Interval (10..100 ms)\n");
        
        putstr("legato  ");  putDecimal(g_Config.LegatoModeEnabled, 4);
        putstr("  Legato Mode Enabled (0,1)\n");
        putstr("velsen  ");  putDecimal(g_Config.VelocitySenseEnabled, 4);
        putstr("  Velocity Sense Enabled (0,1)\n");
        putstr("pbenden ");  putDecimal(g_Config.PitchBendEnabled, 4);
        putstr("  Pitch Bend Enabled (0,1)\n");
        putstr("padlay  ");  putDecimal(g_Config.TouchPadLayout, 4);
        putstr("  Touch Pad Layout (0,1)\n");
        putstr("fingsc  ");  putDecimal(g_Config.FingeringScheme, 4);
        putstr("  Fingering Scheme (0,1,2,..)\n");
        putstr("thres   ");  putDecimal(g_Config.TouchSenseThreshold, 4);
        putstr("  Touch Sense Threshold (max.250)\n");
        
        putstr("prspan  ");  putDecimal(g_Config.PressureSensorSpan, 4);
        putstr("  Pressure Sensor Span (max.750)\n");
        putstr("pbspan  ");  putDecimal(g_Config.PitchBendSpan, 4);
        putstr("  Pitch-Bend Span (max.1000)\n");
        putstr("modmax  ");  putDecimal(g_Config.ModulationMaximum, 4);
        putstr("  Modulation Maximum (max.1000)\n");
        putstr("modband ");  putDecimal(g_Config.ModulationDeadband, 4);
        putstr("  Modulation Dead-band (max.500)\n");
    }
    else if (argCount >= 3)   // Set param. value and verify EEPROM write
    {
        strncpy(nickname, (const char *) argValue[1], 8);
        nickname[7] = 0;  // limit length to 7 chars
        
        if (argValue[2][0] == '=' && argCount == 4)  value = atoi(argValue[3]);
        else  value = atoi(argValue[2]);
        
        if (strmatch(nickname, "chan") && value >= 1 && value <= 16)
            g_Config.MidiBasicChannel = value;
        else if (strmatch(nickname, "sysxen") && value <= 1)
            g_Config.MidiSysExclMsgEnabled = value;
        else if (strmatch(nickname, "prchen") && value <= 1)
            g_Config.MidiProgChangeEnabled = value;
        else if (strmatch(nickname, "expncc") && value < 32)
            g_Config.MidiExpressionCCnumber = value;
        else if (strmatch(nickname, "expnint") && value >= 5 && value <= 50)
            g_Config.MidiPressureInterval = value;
        else if (strmatch(nickname, "expn14b") && value <= 1)
            g_Config.Send14bitExprnData = value;
        else if (strmatch(nickname, "modint") && value >= 10 && value <= 100)
            g_Config.MidiControllerInterval = value;
        
        else if (strmatch(nickname, "padlay") && value <= 1)
            g_Config.TouchPadLayout = value;
        else if (strmatch(nickname, "fingsc") && value <= 1)
            g_Config.FingeringScheme = value;
        else if (strmatch(nickname, "legato") && value <= 1)
            g_Config.LegatoModeEnabled = value;
        else if (strmatch(nickname, "pbenden") && value <= 1)
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
            isBadData = TRUE;
            putstr("! Invalid nickname or value.\n");
        }
        
        if (!isBadData)  
        {
            StoreConfigData();
            putstr("* New value saved: ");  putDecimal((int)value, 4);
            putNewLine();
        }
    }
    else  putstr("! Invalid command. \n");  // Cmd syntax undefined
}


/*```````````````````````````````````````````````````````````````````````````````````````
 * CLI command function:  Cmnd_preset
 *
 * The "preset" command allows preset MIDI Program numbers to be viewed and modified.
 * If a preset value is changed, it will be committed to non-volatile storage.
 */
void  Cmnd_preset(int argCount, char *argValue[])
{
    uint8  preset = atoi(argValue[1]);
    uint8  i,  midi_prgm;
    bool   isBadData = FALSE;
    
    if (argValue[1][0] == '?')  // help wanted
    {
        putstr("Usage #1:  preset    | List Preset MIDI Prgm numbers \n\n");
        putstr("Usage #2:  preset <pre#> [=] <value>    | Set Preset value \n");
        putstr("   where <pre#> is the Preset number (1..8) \n\n");
        putstr("Example:   preset 3 = 67   | Set MIDI Prgm 67 for Preset 3 \n\n");
    }
    else if (argCount == 1)  // View Presets
    {
        putstr("_____________________\n");
        putstr("Preset | MIDI Prgm # \n");
        putstr("```````|`````````````\n");
        for (preset = 1, i = 1 ; i <= 8 ; i++, preset++)
        {
            putDecimal(preset, 4);  putstr("   |   ");
            putDecimal(g_Config.PresetMidiProgram[i & 7], 3);  
			putNewLine();
        }
        putstr("_______|_____________\n");
    }
    else if (argCount >= 3)   // Set Preset value
    {
        if (argValue[2][0] == '=' && argCount == 4)  
			midi_prgm = atoi(argValue[3]);
        else  midi_prgm = atoi(argValue[2]);

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
            putNewLine();
        }
    }
    else  putstr("! Invalid command. \n");  // Cmd syntax undefined
}


/*****
*   CLI command function:  Cmnd_diag
*
*   The "diag" command runs various application-specific system diagnostics.
*/
void  Cmnd_diag(int argCount, char * argValue[])
{
    static  uint32  captureTime;
    char   option = tolower(argValue[1][1]);

    if (argCount == 1 || *argValue[1] == '?')   // help wanted
    {
        putstr("Usage:  diag  <option>  [arg1] ... \n");
        putstr("<option> \n");
//      putstr(" -d :  Test Delay function or macro \n"); 
        putstr(" -e :  Show startup self-test Errors \n"); 
        putstr(" -f :  Write Flash data block (destructive) \n"); 
        putstr(" -m :  MIDI OUT TX test (send 0x0F continuously) \n"); 
        putstr(" -q :  MIDI OUT TX Queue test (send IDENT @ 5ms) \n"); 
//      putstr(" -o :  Task Overrun check \n"); 
        putstr(" -p :  Show MIDI Pressure data value (14b)\n"); 
        putstr(" -s :  Show note on/off State \n"); 
        return;
    }

    switch (option)
    {
#if 0     
    case 'd':  // Delay test (for development)
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
    case 'o':  // Task Overrun check  
    {
        uint8  overruns;
        captureTime = milliseconds();
        v_TaskOverrunCount = 0;
        while (milliseconds() < (captureTime + 200))
        {
            BackgroundTaskExec();  // loop for 200ms
        }
        overruns = v_TaskOverrunCount / 2;  // in 100ms
        putstr("Task overruns in 100ms: ");
        putDecimal(overruns, 1);  putNewLine();
        break;
    }
#endif
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
    case 'm':  // MIDI transmitter test (send 0x0F byte continuously)
    {
        char  key = 0;
        putstr("Sending 0x0F continuously to MIDI TX UART...\n");
        putstr("Hit [Esc] to exit. \n");
        while (key != ASCII_ESC)
        {
            EUSART2_WriteByte(0x0F);  // TX direct to UART2
            
            if (kbhit()) key = getch();  // console key hit
        }
        break;
    }
    case 'q':  // MIDI transmit queue test (send Sys.Excl.IDENT msg continuously)
    {
        char  key = 0;
        putstr("Sending Sys.Ex. 'IDENT' msg continuously to MIDI OUT \n");
        putstr("via MIDI transmit queue.  Message interval is 5ms. \n");
        putstr("Hit [Esc] to exit... \n");
        captureTime = milliseconds();
        
        while (key != ASCII_ESC)
        {
            if ((milliseconds() - captureTime) < 5)  // every 5ms ...
            {
                captureTime = milliseconds();
                MIDI_PutByte(SYS_EXCLUSIVE_MSG);   // status byte (0xF0)
                MIDI_PutByte(SYS_EXCL_REMI_ID);    // manuf/product ID (0x73)
                MIDI_PutByte(REMI_IDENT_MSG);      // Msg type (0x30)
                MIDI_PutByte(SYSTEM_MSG_EOX);      // end-of-msg code (0xF7)
            }
            MIDI_TxQueueHandler();
            
            if (kbhit()) key = getch();  // console key hit
        }
        break;
    }
    } // end switch
}


#if 0  // Deprecated commands
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
void  Cmnd_mode(int argCount, char * argValue[])
{
    char  option = tolower(argValue[1][1]);
    
    if (argValue[1][0] == '?')  // help wanted
    {
        putstr("Set operating mode, i.e. configuration profile.\n");
        putstr("Usage:  mode {-r|-g} \n");
        putstr("  -r : REMI synth,  -g : generic synth mode\n");
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
void  Cmnd_tsens(int argCount, char * argValue[])
{
    uint8  pad;
    char   option = 'l';  
    short  numPads = TouchPadsGetNumber();
    int    filt;
 
    if (argCount > 1)  option = tolower(argValue[1][1]);
    
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
            putNewLine();
        }
        putstr("________|___________|___________\n");
    }
    else
    {
        putstr("! Invalid option. \n");
        putstr("Usage:  tsens [-l] \n");
    }
}
#endif


/*________________________________________________________________________________________*\
*
*          F O R M A T T E D   N U M E R I C   O U T P U T   F U N C T I O N S
\*________________________________________________________________________________________*/
/*
*
*  Output Boolean value as ASCII '0' or '1'.
*
*  Called by:  CLI command functions (only)
*  Entry args: (uint8) b = Boolean variable (zero or non-zero)
*  Returns:    void
*  Affects:
*/

void  putBoolean(uint8  b)
{
    if (b)  putch('1');
    else  putch ('0');
}


/**
*  Output 4 LS bits of a byte as Hex (or BCD) ASCII char.
*
*  Called by:  CLI command functions (only)
*  Entry args: d = value of Hex digit (0 to 0xf)
*  Returns:    void
*  Affects:    --
*/

void  putHexDigit(uint8 d)
{
    d &= 0x0F;
    if (d < 10)  putch ('0' + d);
    else  putch ('A' + d - 10);
}


/**
*  Output byte as 2 Hex ASCII chars, MSD first.
*
*  Called by:  CLI command functions (only)
*  Entry args: b = byte to output
*  Returns:    void
*  Affects:    --
*/

void  putHexByte(uint8 b)
{
    putHexDigit(b >> 4);
    putHexDigit(b);
}


/**
*  Output 16-bit word as 4 Hex ASCII chars, MSD first.
*
*  Called by:  CLI command functions (only)
*  Entry args: uW = word to output
*  Returns:    void
*  Affects:    --
*/

void  putHexWord(uint16 uW)
{
    putHexDigit((uint8) (uW >> 12));
    putHexDigit((uint8) (uW >> 8));
    putHexDigit((uint8) (uW >> 4));
    putHexDigit((uint8) (uW & 0xF));
}


/**
*  Output 32-bit longword as 8 Hex ASCII chars, MSD first.
*
*  Called by:  CLI command functions (only)
*  Entry args: uL = longword to output
*  Returns:    void
*  Affects:    --
*/

void  putHexLong(uint32 uL)
{
    uint8  count, digit;

    for (count=0 ; count < 8 ; count++)
    {
        digit = (uint8) (uL >> 28);
        putHexDigit(digit);
        uL = uL << 4;
    }
}


/**
*   Function:   putDecimal
*   Called by:  CLI command functions and debug trace output functions
*
*   Purpose:    Outputs a 32-bit word as a signed decimal integer, up to 10 digits & sign,
*               right justified in the specified field, with leading zeros suppressed.
*               If the value is too big to fit into the specified minimum field size,
*               the field will be expanded to accommodate the number of digits (& sign).
*               Negative numbers are output with a 'minus' sign to the left of the first
*               non-zero digit. The field size includes the minus sign. Positive numbers
*               are output with space(s) to the left of the first digit, if the space(s)
*               can fit into the specified minimum field width.
*
*   Args:       (int32) lVal = signed longword to be converted and output
*               (uint8) bFieldSize = minimum number of character places (1..12)
*/
void  putDecimal(int32 lVal, uint8 bFieldSize)
{
    uint8    acDigit[12];     /* ASCII result string, acDigit[0] is LSD */
    int      idx;
    int      iSignLoc = 0;
    uint8    c;
    uint8    yNegative = 0;
    uint8    yLeadingZero = 1;

    if (bFieldSize > 12)  bFieldSize = 12;
    if (bFieldSize < 1)  bFieldSize = 1;
    if (lVal < 0)  { yNegative = 1;  lVal = 0 - lVal; }   /* make value absolute */

    for (idx = 0;  idx < 12;  idx++)      /* begin conversion with LSD */
    {
        c = '0' + (uint8)(lVal % 10);
        acDigit[idx] = c;
        lVal = lVal / 10;
    }

    for (idx = 11;  idx >= 0;  idx--)    /* begin processing with MSD */
    {
        c = acDigit[idx];
        if (idx != 0 && c == '0' && yLeadingZero)    /* leave digit 0 (LSD) alone */
            acDigit[idx] = ' ';

        if (idx == 0 || c != '0')              /* found 1st significant digit (MSD) */
        {
            yLeadingZero = 0;
            if (iSignLoc == 0) iSignLoc = idx + 1;
        }
    }
    if (yNegative) acDigit[iSignLoc] = '-';     /* if +ve, there will be a SPACE */

    for (idx = 11;  idx >= 0;  idx--)    /* begin output with MSD (or sign) */
    {
        c = acDigit[idx];
        if (idx < bFieldSize || c != ' ') putch(c);
    }
}


/*________________________________________________________________________________________*\
*
*              N U M E R I C   C O N V E R S I O N   F U N C T I O N S
\*________________________________________________________________________________________*/
/*
*  Convert Hexadecimal ASCII char (arg) to 4-bit value (returned as unsigned byte).
*
*  Called by:  various, background only
*  Entry args: c = Hex ASCII character
*  Returns:    0xFF if arg is not hex, else digit value as unsigned byte (0..0x0F)
*/

uint8  hexctobin(char c)
{
    if (c >= '0'  &&  c <= '9')
        return (c - '0');
    else if (c >= 'A'  &&  c <= 'F')
        return (c - 'A' + 10);
    else if (c >= 'a'  &&  c <= 'f')
        return (c - 'a' + 10);
    else
        return 0xFF ;
}


/*
*  Convert Hexadecimal ASCII string, up to 4 digits, to 16-bit unsigned word.
*  The string must be stored in the data RAM space.
*  There cannot be any leading white space.
*  Conversion is terminated when a non-Hex char is found.
*
*  Entry args: s = pointer to first char of hex string.
*  Returns:    Unsigned 16bit word (0 to 0xffff).
*              If the target string (1st char) is non-Hex, returns 0.
*/
uint16  hexatoi(char * s)
{
    uint8   ubDigit, ubCount;
    uint16  uwResult = 0;

    for (ubCount = 0;  ubCount < 4;  ubCount++)
    {
        if ((ubDigit = hexctobin(*s++)) == 0xFF)
            break;
        uwResult = 16 * uwResult + ubDigit;
    }
    return  uwResult;
}


uint8  isHexDigit(char c)
{
    if (hexctobin(c) == 0xFF) return 0;
    else  return 1;
}


/*
*   Convert a 16-bit unsigned word to hexadecimal string (4 hex ASCII digits).
*   The result string is NOT terminated.
*/
void  wordToHexStr(uint16 wVal, char *pcResult)
{
    pcResult[0] = hexitoc(wVal >> 12);   /* MSB first */
    pcResult[1] = hexitoc(wVal >> 8);
    pcResult[2] = hexitoc(wVal >> 4);
    pcResult[3] = hexitoc(wVal);
}


/*
*   Convert integer value (4 LS bits) to hexadecimal ASCII character ('0' to 'F').
*   The input value is masked to use only the 4 LS bits.
*/
char  hexitoc(short wDigit)
{
    char  cRetVal;

    wDigit = wDigit & 0xF;
    if (wDigit < 10) cRetVal = '0' + wDigit;
    else  cRetVal = 'A' + (wDigit - 10);

    return  cRetVal;
}

// end-of-file
