/*________________________________________________________________________________________*\

    File:        console.h

    Author:      M.J.Bauer

    Header file for console CLI module.  Customized for REMI mk2 handset application.
\*________________________________________________________________________________________*/

#ifndef  _CONSOLE_H_
#define  _CONSOLE_H_

#include  <xc.h>
#include "gendef.h"          // general typedefs, macro defs, etc
#include "EUSART_drv.h"      // UART driver module

/******************************************************************************************/
//       APPLICATION-SPECIFIC DEFINITIONS REQUIRED BY THE CONSOLE CLI
//
#define USE_CONSOLE_CLI      1
#define CLI_USING_UART1      1
#define CLI_MAX_ARGS         6        // Maximum # of Command Line arguments (incl. cmd)
#define CMND_LINE_MAX_LEN   30        // Maximum length of command string
#define CMD_HIST_BUF_SIZE    4        // Maximum number of commands in recall buffer
#define MAX_COMMANDS       250        // Maximum number of CLI commands

//#define INCLUDE_GETSTR_FUNC         // Option to include getstr() function in build
//
/******************************************************************************************/

//  External UART driver functions required by the console CLI
//
#ifdef CLI_USING_UART1
#define RxDataAvail()   EUSART1_RxDataAvail()
#define RxFlush()       EUSART1_RxFlush()
#define TxReady()       EUSART1_TX_Ready()
#define getch()         EUSART1_ReadByte()
#define putch(c)        EUSART1_WriteByte(c)
#define putstr(s)       EUSART1_WriteString(s)
#define putNewLine()    EUSART1_WriteString("\n")
#endif
// else
#ifdef CLI_USING_UART2
#define RxDataAvail()   EUSART2_RxDataAvail()
#define RxFlush()       EUSART2_RxFlush()
#define TxReady()       EUSART2_TX_Ready()
#define getch()         EUSART2_ReadByte()
#define putch(c)        EUSART2_WriteByte(c)
#define putstr(s)       EUSART2_WriteString(s)
#define putNewLine()    EUSART2_WriteString("\n")
#endif

#if !defined (CLI_USING_UART1) && !defined (CLI_USING_UART2)
#error "Can't build console-CLI module without serial port UART defined!"
#endif

#define kbhit()         RxDataAvail()
#define gotinput()      RxDataAvail()

/******************************************************************************************/

// Application-specific data -- external
extern uint8 g_FW_version[];            // firmware version # (major, minor, build)
extern bool  g_NoteOnDisplayActive;     // Set true to enable Note-On display (in CLI)
extern bool  g_DiagModeActive;          // Diagnostic Mode active flag

// The application must provide these "callback" functions, even if they do nothing:
extern uint32  milliseconds();          // Time elapsed since last MCU reset (ms)
extern void  DefaultConfigData(void);   // Restore factory defaults to EEPROM
extern void  BootReset(void);           // Software-invoked MCU reset (re-boot))
extern void  BackgroundTaskExec(void);  // Background task executive in main module

/******************************************************************************************/

#ifndef ASCII_NUL  // Common ASCII control codes
//
#define  ASCII_NUL       0
#define  ASCII_ACK       6
#define  ASCII_BS        8
#define  ASCII_HT        9
#define  ASCII_TAB       9
#define  ASCII_LF       10
#define  ASCII_CR       13
#define  ASCII_DC2      18 
#define  ASCII_NAK      21
#define  ASCII_CAN      24 
#define  ASCII_ESC      27
#define  ASCII_SP       32
//
#endif

typedef  void (*CLIfunc)(int argc, char *argv[]);  // pointer to CLI function

// Command table entry looks like this
struct  CmndTableEntry_t
{
    char    *Name;        // pointer to command name (string literal)
    uint8    Attribute;   // flag:  1 => end-user (visible); 0 => debug (hidden)
    CLIfunc  Function;    // pointer to command function
};

#define  GEN_CMD     'G'
#define  APP_CMD     'A'
#define  SYS_CMD     'S'
#define  DEBUG_CMD   'D'

// Prototypes of functions defined in console CLI module...
//
void    ConsoleCLI_Service(void);
void    CommandLineInterpreter(void);
void    EnterCommandInHistory(void);
void    RecallCommand(void);
void    EraseLine(void);
uint8   strmatch(char *pcStr1, const char *phzStr2);
int     getstr(char *strBuf, int maxLen);  // optional

// ------------  Common CLI commands  ----------------------
//
void    Cmnd_remark(int argCount, char * argValue[]);
void    Cmnd_help(int argCount, char * argValue[]);
void    Cmnd_list(int argCount, char * argValue[]);
void    Cmnd_default(int argCount, char * argValue[]);
void    Cmnd_reset(int argCount, char * argValue[]);
void    Cmnd_ver(int argCount, char * argValue[]);
void    Cmnd_watch(int argCount, char * argValue[]);
void    Cmnd_dump(int argCount, char *argVal[]);
void    Cmnd_eeprom(int argCount, char * argValue[]);

// ----------  Application-Specific CLI commands  -----------
//
void    WatchCommandExec(void);
void    Cmnd_config(int argCount, char * argValue[]);
void    Cmnd_preset(int argCount, char * argValue[]);
void    Cmnd_diag(int argCount, char * argValue[]);

// Formatted numeric string output functions...
//
void    putBoolean(uint8);
void    putHexDigit(uint8);
void    putHexByte(uint8);
void    putHexWord(uint16);
void    putHexLong(uint32);
void    putDecimal(int32 lVal, uint8 bSize);

// Character & string numeric conversion functions...
//
uint8   hexctobin(char c);
uint16  hexatoi(char * s);
uint32  long_hexatoi(char * s);
uint8   isHexDigit(char c);
char    hexitoc(short wDigit);
void    wordToHexStr(uint16 wVal, char *pcResult);

#endif  // _CONSOLE_H_ 
