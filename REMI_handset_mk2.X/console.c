/*________________________________________________________________________________________*\

  File:        console.c

  Author:      M.J.Bauer  [www.mjbauer.biz]

  This module implements the generic console Command Line user Interface (CLI),
  including command functions common to most applications.

  Additional application-specific command functions should be placed in a separate
  source file (e.g. "app_commands.c") and declared in a separate header file.

\*________________________________________________________________________________________*/

#include  <xc.h>
#include  <ctype.h>
#include  <string.h>
#include  <stdlib.h>
#include  "console.h"

//---------------------  Application-specific externals  ---------------------------------
//
extern uint8   g_FW_version[];    // firmware version # (major, minor, build)
extern char   *g_AppTitleCLI;     // Title string output by "ver" command

// The application must define a table of app-specific commands.
// This table has the same structure as CommonCommands[].  (See app_commands.c)
extern const   CmndTableEntry_t  AppCommands[];

/* The application must define this array for the "set" command, if implemented: 
extern UserSettableParameter_t  UserParam[];  */

// The application must provide these "callback" functions, even if they do nothing:
extern void    DefaultPersistentData(void);   // Restore factory defaults to EEPROM
extern void    WatchCommandExec(void);        // real-time display selected variables
extern void    DiagnosticCommandExec(int argCount, char *argVal[]);  // for "diag" cmd

// External functions which may be referenced by the console module:
extern void    BootReset(void);
extern void    BackgroundTaskExec();
extern uint32  millisecTimer(void);
extern uint8   FlashReadByte(uint16 faddr);

//-------------------------------  Private data  -----------------------------------------
//                     
static  char     gacCLIprompt[] = "\r> ";          // CLI Prompt
static  char     acCmndLine[CMND_LINE_MAX_LEN+2];  // Command Line buffer
static  char    *pcCmndLinePtr;                    // Pointer into Cmnd Line buffer
static  int      iCmndLineLen;                     // Cmnd line length (char count)
static  int      argCount;                         // Number of Cmnd Line args (incl. Cmnd name)
static  char    *argValue[CLI_MAX_ARGS];           // Array of pointers to Command Line args
static  char     szNULLstring[] = { '\0' };        // Empty string

static  char     acCmndHistoryBuffer[CMD_HIST_BUF_SIZE][CMND_LINE_MAX_LEN+2];
static  uint16   wCmndHistoryMarker;      // Index to next free place in Command History Buf
static  uint16   wCmndRecallMarker;       // Index of previous command to be recalled
static  char     yHistoryInitialised = 0;
static  char     ySuperUser = FALSE;


/*----------------------------------------------------------------------------------------
*                      C O M M A N D   T A B L E
*/
const  CmndTableEntry_t  CommonCommands[] =
{
    //   Cmd Name      Attribute      Cmd Function
    //----------     -------------   -------------------
    {    "*",          GEN_CMD,       Cmnd_remark     },
    {    "help",       GEN_CMD,       Cmnd_help       },
    {    "lc",         GEN_CMD,       Cmnd_list       },
    {    "default",    GEN_CMD,       Cmnd_default    },
    {    "ver",        GEN_CMD,       Cmnd_ver        },
    {    "diag",       GEN_CMD,       Cmnd_diag       },
    {    "watch",      GEN_CMD,       Cmnd_watch      },
    {    "dump",       GEN_CMD,       Cmnd_dump       },
    {    "reset",      GEN_CMD,       Cmnd_reset      },
    
#ifdef INCLUDE_KERNEL_RTC_SUPPORT
    {    "date",       GEN_CMD,       Cmnd_time       },
    {    "time",       GEN_CMD,       Cmnd_time       },
    {    "rtcc",       GEN_CMD,       Cmnd_rtcc       },
#endif
    //--------------------------------------------------
    {    "$",          0,             Cmnd_help       }   // Dummy last entry
} ;


/*----------------------------------------------------------------------------------------
*                         COMMAND LINE INPUT STREAM HANDLER
*
*   Called frequently from the main loop and any wait loop(s) in CLI command fn's.
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
void  ConsoleCLI_Service( void )
{
#if USE_CONSOLE_CLI

    char  c;

    if ( RxDataAvail() )     // char(s) available in serial input buffer
    {
        c = getbyte();             // Fetch the char... no echo (yet)
        switch ( c )
        {
        case ASCII_CAN:                 // Ctrl+X... cancel line...
            EraseLine();
            PrepareForNewCommand();
            break;

        case ASCII_BS:                  // BACKSPACE...
            if ( iCmndLineLen > 0 )
            {
                pcCmndLinePtr-- ;           // Remove last-entered char from buffer
                iCmndLineLen-- ;
                putbyte( ASCII_BS );        // Backspace the VDU cursor
                putbyte( ' ' );             // Erase offending char at VDU cursor
                putbyte( ASCII_BS );        // Re-position the VDU cursor
            }
            break;

        case ASCII_CR:                  // ENTER...
            putbyte( '\r' );                // Echo NewLine
            putbyte( '\n' );
            if ( iCmndLineLen > 0 )         // Got a command string...
            {
                *pcCmndLinePtr = 0;         // Terminate the command string
                EnterCommandInHistory();    // Enter it into the history buffer
                CommandLineInterpreter();   // Interpret and execute the cmnd.
            }
            PrepareForNewCommand();         // CR, LF, prompt
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
            if ( isprint( c ) && iCmndLineLen < CMND_LINE_MAX_LEN )
            {
                putbyte( c );               // Echo char
                *pcCmndLinePtr++ = c;       // Append char to Cmnd Line buffer
                iCmndLineLen++ ;
            }
            break;
        } // end_switch
    }
#endif // USE_CONSOLE_CLI
}


/*----------------------------------------------------------------------------------------
*                    C O M M A N D   L I N E   I N T E R P R E T E R
*
*   Processes the received command line when entered.
*
*   The CommandLineInterpreter() function is called by ConsoleCLI_Service()
*   when a valid command string is entered (CR received). It finds & counts "arguments"
*   in the command line and makes them NUL-terminated strings in-situ.
*   It then searches the Command Table for command name, argValue[0],
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
*   functions. The strmatch() function is also not case-sensitive.
*/
void  CommandLineInterpreter( void )
{
    char    c;
    short   idx, cmndIndex;
    char    yCommonCmndNameFound = 0;
    char    yAppCmndNameFound = 0;

    pcCmndLinePtr = acCmndLine;             // point to start of Cmnd Line buffer
    argCount = 0;

    // This loop finds and terminates (with a NUL) any user-supplied arguments...
    for ( idx = 0;  idx < CLI_MAX_ARGS;  idx++ )
    {
        if ( !isprint( *pcCmndLinePtr ) )               // stop at end of line
            break;
        while ( *pcCmndLinePtr == ' ' )                 // skip leading spaces
            pcCmndLinePtr++ ;
        if ( !isprint( *pcCmndLinePtr ) )               // end of line found
            break;
        argValue[idx] = pcCmndLinePtr;                  // Make ptr to arg
        argCount++ ;

        while ( ( c = *pcCmndLinePtr ) != ' ' )         // find first space after arg
        {
            if ( !isprint( c ) )                        // end of line found
                break;
            pcCmndLinePtr++ ;
        }
        if ( !isprint( *pcCmndLinePtr ) )               // stop at end of line
            break;
        *pcCmndLinePtr++ = 0;                           // NUL-terminate the arg
    }

    // This loop searches the common command table for the supplied command name...
    for ( idx = 0;  idx < MAX_COMMANDS;  idx++ )
    {
        if ( *CommonCommands[idx].phzName == '$' )      // reached end of table
            break;
        if ( strmatch( argValue[0], CommonCommands[idx].phzName ) )
        {
            yCommonCmndNameFound = 1;
            cmndIndex = idx;
            break;
        }
    }
    
    // This loop searches the (external) application-specific command table...
    for ( idx = 0;  idx < MAX_COMMANDS;  idx++ )
    {
        if ( *AppCommands[idx].phzName == '$' )         // reached end of table
            break;
        if ( strmatch( argValue[0], AppCommands[idx].phzName ) )
        {
            yAppCmndNameFound = 1;
            cmndIndex = idx;
            break;
        }
    }
    
    if ( argCount > 1 )   // If there is one or more user-supplied arg(s)...
    {
        if ( strmatch( argValue[1], "-help" ) )    // convert "-help" to '?' ...
            *argValue[1] = '?';                    // ... to simplify cmd fn
    }

    if (yAppCmndNameFound)
    {
        (*AppCommands[cmndIndex].Function)(argCount, argValue);  // execute cmd fn
    }
    else if (yCommonCmndNameFound)
    {
        (*CommonCommands[cmndIndex].Function)(argCount, argValue);  // execute fn
    }
    else  
    {
        putstr( "? Undefined command.\n" );
        Cmnd_help( 1, NULL );
    }
}


/*
*   Flush Command Line buffer, clear CLI arg's and output CLI prompt.
*   This function must be called before the first call to ConsoleCLI_Service()
*   to initialize the CLI environment.
*/
void  PrepareForNewCommand( void )
{
    uint8  bArgIndex;

    acCmndLine[0] = 0;
    pcCmndLinePtr = acCmndLine;  // point to start of Cmnd Line buffer
    iCmndLineLen = 0;
    for ( bArgIndex = 0;  bArgIndex < CLI_MAX_ARGS;  bArgIndex++ )   // Clear CLI args
        argValue[bArgIndex] = szNULLstring;
    putstr( gacCLIprompt );
}


/*
*   Copy the newly entered command into the history buffer for later recall...
*   if it's length is non-zero.
*/
void  EnterCommandInHistory( void )
{
    unsigned  line;

    if ( !yHistoryInitialised )
    {
        for (line = 0;  line < CMD_HIST_BUF_SIZE;  line++)
        {
            acCmndHistoryBuffer[line][0] = 0;    // make empty cmnd string
        }
        wCmndHistoryMarker = 0;
        wCmndRecallMarker = 0;
        yHistoryInitialised = 1;
    }

    if (strlen((const char *) acCmndLine) != 0)   // Not an empty cmnd string
    {
        strncpy(acCmndHistoryBuffer[wCmndHistoryMarker], acCmndLine, CMND_LINE_MAX_LEN);
        wCmndRecallMarker = wCmndHistoryMarker;
        wCmndHistoryMarker++ ;
        if ( wCmndHistoryMarker >= CMD_HIST_BUF_SIZE ) wCmndHistoryMarker = 0;
    }
}


/*
*   Recall a previously entered command from the history buffer...
*   The function selects the next previous command from the buffer (if any)
*   as indicated by wCmndRecallMarker, and outputs the command string to the user's
*   terminal for editing. At the same time, the selected command is copied to the
*   current command line buffer.
*   The selected command is not executed until the user hits ENTER (CR).
*/
void  RecallCommand( void )
{
    strncpy( acCmndLine, acCmndHistoryBuffer[wCmndRecallMarker], CMND_LINE_MAX_LEN );
    if ( wCmndRecallMarker == 0 ) wCmndRecallMarker = CMD_HIST_BUF_SIZE;
    --wCmndRecallMarker;
    iCmndLineLen = strlen( acCmndLine );
    pcCmndLinePtr = acCmndLine + iCmndLineLen;
    *pcCmndLinePtr = 0;
    putstr( acCmndLine );
}


/*
*   Erase the command line on user terminal; cursor remains on same line at col 1.
*/
void  EraseLine( void )
{
    short  col;

    putbyte('\r');
    for ( col=0 ; col < (CMND_LINE_MAX_LEN+2) ; col++ )
    {
        putbyte(' ');
    }
    putbyte('\r');
}


/*****
*   Function:   strmatch
*
*   Purpose:    Compares two NUL-terminated strings, each up to 255 chars in length.
*               Returns 1 if the strings have the same contents, up to the terminator
*               of either string, except that the comparison is not case-sensitive.
*
*   Args:       (char *) str1 :  pointer to string variable
*               (char *) str2 :  pointer to string variable
*
*   Returns:    1 if string #1 matches string #2 (case-insensitive), else 0.
*/
uint8  strmatch( char *str1, char *str2 )
{
    char    c1, c2;
    uint8   b = 255;
    uint8   yResult = 1;

    while ( b-- != 0 )
    {
        c1 = tolower( *str1++ );
        c2 = tolower( *str2++ );
        if ( c1 != c2 )  yResult = 0;
        if ( c1 == 0 || c2 == 0 )  break;
    }
    
    return yResult;
}


/*****
*   Function:   putstr
*
*   Purpose:    Sends a text string to the console output stream (user terminal).
*               While waiting for UART TX READY, background tasks are executed.
*
*   Args:       (char *) str :  address of string to be output
*
*/
void  putstr(char *str)
{
    char c;

    while ((c = *str++) != 0)
    {
        do  { BackgroundTaskExec(); }
        until ( TxReady() );
        
        if (c == '\n')
        {
            putbyte('\r');
            putbyte('\n');
        }
        else   putbyte(c);
    }
}


#if FALSE
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
        if ( RxDataAvail() ) 
        {
            c = getbyte();  // no echo (yet)
            switch ( c )
            {
            case ASCII_BS:     // BACKSPACE
                if (length > 0)
                {
                    inBuf-- ;  length--;
                    putbyte( ASCII_BS );
                    putbyte( ' ' );
                    putbyte( ASCII_BS );
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
                    putbyte( c );    // echo
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
*   The "help" command gives brief help on CLI usage.
*/
void  Cmnd_help( int argCount, char * argValue[] )
{
    putstr( "To list available commands, enter: 'lc' \n" );
    putstr( "Command arg's and options must be separated by spaces.\n" );
    putstr( "For command usage help, enter: '<cmd_name> ?' \n" );
    putstr( "To recall previously entered commands, hit Ctrl+R.\n" );
/**    
    putstr( "In usage info, square brackets enclose optional parameters, e.g. [<x1>]; \n" );
    putstr( "braces enclose alternative options, e.g. {-x|-y }, where '|' means 'OR'. \n" );
**/    
}


/**
*   CLI command function:  Cmnd_note
*
*   The "remark" command (*) allows a comment line to be entered.
*/
void  Cmnd_remark( int argCount, char * argValue[] )
{
    if ( *argValue[1] == '?' )   // help wanted
    {
        putstr( "Usage:  * [comment] \n");
        return;
    }
}


/**
*   CLI command function:  Cmnd_list
*
*   The "lc" command lists CLI command names, formatted into 6 columns, unsorted.
*   Common commands are listed first, then app-specific commands.
*/
void  Cmnd_list( int argCount, char * argValue[] )
{
    char    *commandName;
    short   iTable;
    short   iCmd;
    short   iSpacesToPad;
    short   iColumn = 0;
    
    CmndTableEntry_t  *CmdTable = (CmndTableEntry_t *) CommonCommands;

    for (iTable = 0;  iTable < 2;  iTable++)  // 2 tables
    {
        for (iCmd = 0;  iCmd < MAX_COMMANDS;  iCmd++)
        {
            commandName = CmdTable[iCmd].phzName;
            
            if (commandName[0] == '$')  // Reached end of table
                break;            

            putstr( commandName );
            iSpacesToPad = 12 - strlen(commandName);
            while (iSpacesToPad-- != 0)
            {
                putbyte(' ');
            }
            if (++iColumn >= 6)  { iColumn = 0;  NEW_LINE(); }

        }
        CmdTable = (CmndTableEntry_t *) AppCommands;    // next table
    }
    if (iColumn != 0)  NEW_LINE();
}


/*****
*   CLI command function:  ver
*
*   The "ver" command displays firmware version number and other build information.
*/
void  Cmnd_ver( int argCount, char * argValue[] )
{
    putstr(g_AppTitleCLI);   // defined in main application
    
    putstr( "Firmware version " );
    putDecimal( g_FW_version[0], 1 );
    putbyte( '.' );
    putDecimal( g_FW_version[1], 1 );
    putbyte( '.' );
    if (g_FW_version[2] < 10) putbyte('0');
    putDecimal( g_FW_version[2], 1 );
    putstr( ", " );
    putstr( __DATE__ );
    NEW_LINE();
}


/*****
*   CLI command function:  Cmnd_default
*
*   The "default" command restores "factory default" values to all persistent data,
*   typically held in EEPROM, the same as when non-volatile memory becomes corrupted.
*/
void  Cmnd_default(int argCount, char * argValue[])
{
    char  c = 0;
    bool  confirm = FALSE;

    if (argCount == 1)  // Cmd name only...
    {
        putstr( "Restore configuration data to factory defaults? (Y/N): " );
        while (c < 0x20)
        {
            if (RxDataAvail()) c = getbyte();
        }
        putbyte(c);  putstr("\n");
        if (toupper(c) == 'Y') confirm = TRUE;
    }
    else if (strmatch(argValue[1], "-y")) confirm = TRUE;

    if (confirm) 
    {
        DefaultPersistentData();  // Application Call-back
        putstr( "* Default configuration restored.\n" );
    }
}


/*****
*   CLI command function:  Cmnd_reset
*
*   The "reset" command invokes a reset of the system.
*   The mode of reset is determined by the application "call-back" function.
*/
void  Cmnd_reset( int argCount, char * argValue[] )
{
    BootReset();
}


/*****
*   CLI command function:  Cmnd_watch
*
*   The "watch" command activates a real-time display which shows the value(s)
*   of one or more global application variables.
*   The function exits when it receives ASCII_ESC from the serial input stream,
*   ie. when user hits the [Esc] key on their terminal.
*/
void  Cmnd_watch( int argCount, char * argValue[] )
{
    uint32 start_time;
    char c = 0;

    putstr( "Hit [Esc] to quit. \n\n" );

    while ( c != ASCII_ESC )
    {
        // Callback function defined in app-specific code module...
        // Variables to be "watched" are output on a single line (no newline).
        WatchCommandExec();

        // Delay 50mS for refresh rate of 20Hz.
        // While waiting, any pending background tasks are executed.
        start_time = millisecTimer();
        while ( millisecTimer() < (start_time + 50) )
        {
            BackgroundTaskExec();
        }
        putbyte( '\r' );       // Return VDU cursor to start of output line
        if ( gotinput() )  c = getbyte();       // Check for key hit
        if ( c == ASCII_CR )  { c = 0;  NEW_LINE(); }
    }
    NEW_LINE();
}


/*****
*   CLI command function:  Cmnd_diag
*
*   The "diag" command runs various application-specific system diagnostics.
*   DiagnosticCommandExec() function should be in an application-specific code module.
*/
void  Cmnd_diag(int argCount, char * argValue[])
{
    DiagnosticCommandExec(argCount, argValue);
}


#ifdef INCLUDE_KERNEL_RTC_SUPPORT
/*^
*   CLI command function:  Cmnd_time
*
*   The "time" command is for reading or setting the real-time clock (RTC).
*/
void  Cmnd_time( int argCount, char * argValue[] )
{
    short i;
    short errcode;
    uint8 syntax_error = 0;
    char  cBuf[80];

#if defined (RTCC_TYPE_ISL1208) || defined (RTCC_TYPE_MCP79410)

    if ( *argValue[1] == '?' )   // help wanted
    {
        putstr( "Usage (1):  time [hh:mm] [yy-mm-dd] \n" );
        putstr( "   Set system time & date; initialize RTCC device (if present). \n" );
        putstr( "Usage (2):  time -s \n" );
        putstr( "   Synchronize system time & date to hardware RTCC device.\n" );
        putstr( "In both cases, or if no arg supplied, show system time & date.\n" );
        return;
    }
#else  // Use software real-time clock (RTI Timer)

    if ( *argValue[1] == '?' )   // help wanted
    {
        putstr( "Usage:  time [hh:mm] [yy-mm-dd] \n" );
        putstr( "Show time & date. If param(s) supplied, set time/date. \n" );
        return;
    }
#endif

#if defined (RTCC_TYPE_ISL1208) || defined (RTCC_TYPE_MCP79410)
    if (argCount == 2 && strmatch(argValue[1], "-s"))
    {
        if (RTCC_Synchronize() < 0) putstr("RTCC device not responding.\n");
    }
    else 
#endif

    if (argCount > 1)   // time/date arg(s) supplied...
    {
        for (i = 1; i < argCount; i++)
        {
            if (argValue[i][2] == ':')  // assume arg = hh:mm
            {
                gsRTCbuf.hour = atoi(&argValue[i][0]);
                gsRTCbuf.mins = atoi(&argValue[i][3]);
                gsRTCbuf.secs = 0;
            }
            else if (argValue[i][2] == '-')  // assume arg = yy-mm-dd
            {
                if (argValue[i][5] != '-') { syntax_error = 1; break; }
                gsRTCbuf.year = atoi(&argValue[i][0]);
                gsRTCbuf.month = atoi(&argValue[i][3]);
                gsRTCbuf.day = atoi(&argValue[i][6]);
            }
            else syntax_error = 1;
        }
        
        if (!syntax_error)
        {
            RTC_date_time_set();            // Set the software RTC "registers" from buffer

#if defined (RTCC_TYPE_ISL1208) || defined (RTCC_TYPE_MCP79410)
            errcode = RTCC_Initialize();    // Initialize RTCC chip and set time/date regs
            if (errcode == -3) putstr("! RTCC PF flag not reset \n");
            else if (errcode < 0) putstr("! RTCC device not detected \n");
            else putstr("RTCC device initialized \n");
#else
            putstr("  RTCC device not in use. \n");
#endif
        }
        else
        {
            putstr("Syntax error! Try: time hh:mm yy-mm-dd \n");
            return;
        }
    }

    if (RTC_date_time_read())
    {
        sprintf(cBuf, "%02d:%02d:%02d ", gsRTCbuf.hour, gsRTCbuf.mins, gsRTCbuf.secs);
        putstr(cBuf);
        sprintf(cBuf, "20%02d-%02d-%02d ", gsRTCbuf.year, gsRTCbuf.month, gsRTCbuf.day);
        putstr(cBuf);
        putstr(asDayOfWeekName[gsRTCbuf.dow]);  NEW_LINE;
    }
    else putstr("System time/date invalid.\n");
}

#endif // INCLUDE_KERNEL_RTC_SUPPORT


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
void  Cmnd_dump( int argCount, char *argVal[] )
{
    static  uint16  startAddr;
    uint16  addr;
    uint8   row, col;
    char    c;
    
    if (argCount >= 2)  // assume arg[1] is <addr>
        startAddr = hexatoi(argValue[1]) & 0xFFF0;  // mask to 16-byte boundary

    for (addr = startAddr, row = 0;  row < 16;  row++)
    {
        putHexWord(addr);
        putstr(" : ");
        for (col = 0;  col < 16;  col++)
        {
            putHexByte(FlashReadByte(addr++));  // print 2-digit hex
			putbyte(' ');
            if (col % 8 == 7) putbyte(' ');
        }
        for (addr -= 16, col = 0;  col < 16;  col++)
        {
            c = FlashReadByte(addr++);
            if (isprint(c))  putbyte(c);  // print ASCII symbol
            else  putbyte('.');
        }
        NEW_LINE();
    }
    
    startAddr += 256;  // next page 
}


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

void  putBoolean( uint8  b )
{
    if ( b )  putbyte( '1');
    else  putbyte ( '0' );
}


/*****
*  Output 4 LS bits of a byte as Hex (or BCD) ASCII char.
*
*  Called by:  CLI command functions (only)
*  Entry args: d = value of Hex digit (0 to 0xf)
*  Returns:    void
*  Affects:    --
*/

void  putHexDigit( uint8 d )
{
    d &= 0x0F;
    if ( d < 10 )  putbyte ( '0' + d );
    else  putbyte ( 'A' + d - 10 );
}


/*****
*  Output byte as 2 Hex ASCII chars, MSD first.
*
*  Called by:  CLI command functions (only)
*  Entry args: b = byte to output
*  Returns:    void
*  Affects:    --
*/

void  putHexByte( uint8 b )
{
    putHexDigit((uint8) (b >> 4));
    putHexDigit(b);
}


/*****
*  Output 16-bit word as 4 Hex ASCII chars, MSD first.
*
*  Called by:  CLI command functions (only)
*  Entry args: uW = word to output
*  Returns:    void
*  Affects:    --
*/

void  putHexWord( uint16 uW )
{
    putHexDigit( (uint8) (uW >> 12) );
    putHexDigit( (uint8) (uW >> 8) );
    putHexDigit( (uint8) (uW >> 4) );
    putHexDigit( (uint8) (uW & 0xF) );
}


/*****
*  Output 32-bit longword as 8 Hex ASCII chars, MSD first.
*
*  Called by:  CLI command functions (only)
*  Entry args: uL = longword to output
*  Returns:    void
*  Affects:    --
*/

void  putHexLong( uint32 uL )
{
    uint8  count, digit;

    for ( count=0 ; count < 8 ; count++ )
    {
        digit = (uint8) (uL >> 28);
        putHexDigit( digit );
        uL = uL << 4;
    }
}


/*****
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
void  putDecimal( int32 lVal, uint8 bFieldSize )
{
    uint8  acDigit[12];     /* ASCII result string, acDigit[0] is LSD */
    unsigned  idx;
    unsigned  iSignLoc = 0;
    uint8  c;
    bool  yNegative = 0;
    bool  yLeadingZero = 1;

    if ( bFieldSize > 12 )  bFieldSize = 12;
    if ( bFieldSize < 1 )  bFieldSize = 1;
    if ( lVal < 0 )  { yNegative = 1;  lVal = 0 - lVal; }   /* make value absolute */

    for ( idx = 0;  idx < 12;  idx++ )      /* begin conversion with LSD */
    {
        c = '0' + (uint8)(lVal % 10);
        acDigit[idx] = c;
        lVal = lVal / 10;
    }

    for ( idx = 11;  idx <= 11;  idx-- )    /* begin processing with MSD */
    {
        c = acDigit[idx];
        if ( idx != 0 && c == '0' && yLeadingZero )    /* leave digit 0 (LSD) alone */
            acDigit[idx] = ' ';

        if ( idx == 0 || c != '0' )              /* found 1st significant digit (MSD) */
        {
            yLeadingZero = 0;
            if ( iSignLoc == 0 ) iSignLoc = idx + 1;
        }
    }
    if ( yNegative ) acDigit[iSignLoc] = '-';     /* if +ve, there will be a SPACE */

    for ( idx = 11;  idx <= 11;  idx-- )    /* begin output with MSD (or sign) */
    {
        c = acDigit[idx];
        if ( idx < bFieldSize || c != ' ' ) putbyte( c );
    }
}


#ifdef USE_MDD_FILE_SYSTEM
void  fputstr(char *str, FSFILE *stream)
{
    if (stream == NULL) putstr(str);  // => stdout
    else  FSfwrite(str, 1, strlen(str), stream);
}
#endif


/*________________________________________________________________________________________*\
*
*              N U M E R I C   C O N V E R S I O N   F U N C T I O N S
\*________________________________________________________________________________________*/
/*
*
*  Convert decimal ASCII char to 4-bit BCD value (returned as unsigned byte).
*
*  Entry args: c = decimal digit ASCII encoded
*  Returns:    0xFF if arg is not a decimal digit, else unsigned byte (0..9)
*  Affects:    --
*/
uint8  dectobin( char c )
{
    if ( c >= '0'  &&  c <= '9')
        return ( c - '0' );
    else
        return 0xFF ;
}


/*****
*  Convert decimal ASCII string, up to 5 digits, to 16-bit unsigned word.
*  There may be leading zeros, but there cannot be any leading white space.
*  Conversion is terminated when a non-decimal char is found, or when the
*  specified number of characters has been processed.
*
*  Entry args: (char *) pac = pointer to first char of decimal ASCII string
*              (int8)  bNdigs = number of characters to process (max. 5)
*  Returns:    Unsigned 16bit word ( 0 to 0xffff ).
*              If the target string (1st char) is non-numeric, returns 0.
*/
uint16  decatoi( char * pac, int8 bNdigs )
{
    uint8   ubDigit, ubCount;
    uint16  uwResult = 0;

    for ( ubCount = 0;  ubCount < bNdigs;  ubCount++ )
    {
        if ( (ubDigit = dectobin( *pac++ )) == 0xFF )
            break;
        uwResult = 10 * uwResult + ubDigit;
    }
    return  uwResult;
}


/*****
*  Convert Hexadecimal ASCII char (arg) to 4-bit value (returned as unsigned byte).
*
*  Called by:  various, background only
*  Entry args: c = Hex ASCII character
*  Returns:    0xFF if arg is not hex, else digit value as unsigned byte ( 0..0x0F )
*/

uint8  hexctobin( char c )
{
    if ( c >= '0'  &&  c <= '9')
        return ( c - '0' );
    else if ( c >= 'A'  &&  c <= 'F' )
        return ( c - 'A' + 10 );
    else if ( c >= 'a'  &&  c <= 'f' )
        return ( c - 'a' + 10 );
    else
        return 0xFF ;
}


/*****
*  Convert Hexadecimal ASCII string, up to 4 digits, to 16-bit unsigned word.
*  The string must be stored in the data RAM space.
*  There cannot be any leading white space.
*  Conversion is terminated when a non-Hex char is found.
*
*  Entry args: s = pointer to first char of hex string.
*  Returns:    Unsigned 16bit word ( 0 to 0xffff ).
*              If the target string (1st char) is non-Hex, returns 0.
*/

uint16  hexatoi( char * s )
{
    uint8   ubDigit, ubCount;
    uint16  uwResult = 0;

    for ( ubCount = 0;  ubCount < 4;  ubCount++ )
    {
        if ( (ubDigit = hexctobin( *s++ )) == 0xFF )
            break;
        uwResult = 16 * uwResult + ubDigit;
    }
    return  uwResult;
}


uint8  isHexDigit( char c )
{
    if ( hexctobin( c ) == 0xFF ) return 0;
    else  return 1;
}


/*
*   Convert a 16-bit unsigned word to hexadecimal string (4 hex ASCII digits).
*   The result string is NOT terminated.
*/
void  wordToHexStr( uint16 wVal, char *pcResult )
{
    pcResult[0] = hexitoc( wVal >> 12 );   /* MSB first */
    pcResult[1] = hexitoc( wVal >> 8 );
    pcResult[2] = hexitoc( wVal >> 4 );
    pcResult[3] = hexitoc( wVal );
}


/*
*   Convert integer value (4 LS bits) to hexadecimal ASCII character ('0' to 'F').
*   The input value is masked to use only the 4 LS bits.
*/
char  hexitoc( unsigned wDigit )
{
    char  cRetVal;

    wDigit = wDigit & 0xF;
    if ( wDigit < 10 ) cRetVal = (char) ('0' + wDigit);
    else  cRetVal = (char) ('A' + (wDigit - 10));

    return  cRetVal;
}


/*****
*  Convert Hexadecimal ASCII string, up to 8 digits, to 32-bit unsigned long word.
*  There cannot be any leading white space.
*  Conversion is terminated when a non-Hex char is found.
*
*  Entry args: s = pointer to first char of hex string.
*  Returns:    Unsigned 32 bit long word ( 0 to 0xffffffff ).
*              If the target string (1st char) is non-Hex, returns 0.
*/
uint32  long_hexatoi( char * s )
{
    uint8   ubDigit, ubCount;
    uint32  ulResult = (uint32) 0;

    for ( ubCount = 0;  ubCount < 8;  ubCount++ )
    {
        if ( (ubDigit = hexctobin( *s++ )) == 0xFF )
            break;
        ulResult = 16 * ulResult + ubDigit;
    }
    return  ulResult;
}
