/*________________________________________________________________________________________*\

    File:        console.h

    Author:      M.J.Bauer

    Header file for console CLI module.
\*________________________________________________________________________________________*/

#ifndef  _CONSOLE_H_
#define  _CONSOLE_H_

#include  <xc.h>
#include "gendef.h"          // general typedefs, macro defs, etc
#include "EUSART_drv.h"      // UART driver module

/******************************************************************************************/
//            APPLICATION-SPECIFIC DEFINITIONS REQUIRED BY THE CONSOLE CLI
//
#define USE_CONSOLE_CLI      1
#define CLI_USING_UART1      1

#define CLI_MAX_ARGS         8        // Maximum # of Command Line arguments (incl. cmd)
#define CMND_LINE_MAX_LEN   40        // Maximum length of command string
#define CMD_HIST_BUF_SIZE    4        // Maximum number of commands in recall buffer
#define MAX_COMMANDS       250        // Maximum number of CLI commands

#define SU_PASSWORD        "K22"      // Default "super user" password


/******************************************************************************************/
//                External functions required by the console CLI
//
#ifdef CLI_USING_UART1
#define RxDataAvail()   EUSART1_RxDataAvail()
#define RxFlush()       EUSART1_RxFlush()
#define getbyte()       EUSART1_ReadByte()
#define TxReady()       EUSART1_TX_Ready()
#define putbyte(b)      EUSART1_WriteByte(b)
#endif

#ifdef CLI_USING_UART2
#define RxDataAvail()   EUSART2_RxDataAvail()
#define RxFlush()       EUSART2_RxFlush()
#define getbyte()       EUSART2_ReadByte()
#define TxReady()       EUSART2_TX_Ready()
#define putbyte(b)      EUSART2_WriteByte(b)
#endif

#if !defined (CLI_USING_UART1) && !defined (CLI_USING_UART2)
#error "Can't build console-CLI module without serial port UART defined!"
#endif

#define gotinput()      RxDataAvail()    // alias for kbhit()
#define NEW_LINE()      putstr("\n")

/******************************************************************************************/

#ifndef ASCII_NUL  // Common ASCII control codes

#define  ASCII_NUL       0
#define  ASCII_ACK       6
#define  ASCII_BS        8        /* Ctrl+H, Backspace */
#define  ASCII_HT        9
#define  ASCII_TAB       9
#define  ASCII_LF       10
#define  ASCII_CR       13
#define  ASCII_DC2      18        /* Ctrl+R, Device Control 2 */
#define  ASCII_NAK      21
#define  ASCII_CAN      24        /* Ctrl+X, Cancel line */
#define  ASCII_ESC      27
#define  ASCII_SP       32

#endif


enum  eDebugDataType
{
    HEX_BYTE,
    CHAR_BYTE,
    DEC_BYTE,
    HEX_WORD,
    DEC_UWORD,
    DEC_IWORD,
    HEX_LONG,
    DEC_ILONG
};

typedef  void (*CLIfunc)( int argc, char *argv[] );        // pointer to CLI function


// Command table entry looks like this
typedef struct  Cmnd_Table_Entry
{
    char    *phzName;           // pointer to command name (string literal)
    uint8    yAttribute;        // command category (general, app, system, debug, etc)
    CLIfunc  Function;          // pointer to CLI function
    
} CmndTableEntry_t;

// Possible values for command attribute: asCommand.yAttribute
#define  GEN_CMD     'G'        // Generic command
#define  APP_CMD     'A'        // Application-specific cmd
#define  DBG_CMD     'D'        // Debug command (may be hidden by "lc")


typedef struct UserSettableParameters  // For "set" command, where implemented
{
    char   name[12];     // param nick-name, max. 10 chars (last entry = "$")
    char   format;       // display format: 'r' == real, 'i' == integer"
    float *address;      // pointer to global parameter
    float  minValue;     // minimum acceptable value
    float  maxValue;     // maximum acceptable value

} UserSettableParameter_t;


// Prototypes of functions defined in console CLI module...

void    ConsoleCLI_Service( void );
void    CommandLineInterpreter( void );
void    PrepareForNewCommand( void );
void    EnterCommandInHistory( void );
void    RecallCommand( void );
void    EraseLine( void );
uint8   strmatch( char *sr1, char *str2 );
void    putstr(char *str);
int     getstr(char *strBuf, int maxLen);

void    OutputParamValue(char *nickName, float fValue);
void    ListParamNamesValues(void);
void    SetParameterValue(char *nickName, float fValue);
char    SuperUserAccess(void);

// ------------  Common CLI commands  ----------------------

void    Cmnd_remark( int argCount, char * argValue[] );
void    Cmnd_help( int argCount, char * argValue[] );
void    Cmnd_default( int argCount, char * argValue[] );
void    Cmnd_diag( int argCount, char * argValue[] );
void    Cmnd_flags( int argCount, char * argValue[] );
void    Cmnd_list( int argCount, char * argValue[] );
void    Cmnd_reset( int argCount, char * argValue[] );
void    Cmnd_ver( int argCount, char * argValue[] );
void    Cmnd_watch( int argCount, char * argValue[] );
void    Cmnd_su( int argCount, char * argValue[] );
void    Cmnd_time( int argCount, char * argValue[] );
void    Cmnd_rtcc( int argCount, char *argValue[] );

// ---------------- Debug commands  -------------------------

void    Cmnd_dump( int argCount, char * argValue[] );
void    Cmnd_peek( int argCount, char * argValue[] );
void    Cmnd_poke( int argCount, char * argValue[] );
void    Cmnd_pio( int argCount, char * argValue[] );

// Formatted numeric string output functions...

void    putBoolean( uint8 );
void    putHexDigit( uint8 );
void    putHexByte( uint8 );
void    putHexWord( uint16 );
void    putHexLong( uint32 );
void    putDecimal( int32 lVal, uint8 bSize );

// Character & string numeric conversion functions...

uint8   dectobin( char c );
uint16  decatoi( char * pac, int8 bNdigs );
uint32  long_decatoi( char * pac, int8 bNdigs );
uint8   hexctobin( char c );
uint16  hexatoi( char * s );
uint32  long_hexatoi( char * s );
uint8   isHexDigit( char c );
char    hexitoc( unsigned wDigit );
void    wordToHexStr( uint16 wVal, char *pcResult );
void    longToDecimalStr( int32 lVal, char *pcResult, uint8 bFieldSize );

#endif    /* _CONSOLE_H_ */
