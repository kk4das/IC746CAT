/*************************************************************************
   IC746 CAT Library, by KK4DAS, Dean Souleles
   V1.0.2 12/17/2021
      - Send NACK on undocmented command / fix for lates WSJTX hamlib
      
   V1.0.1 2/3/2021
      - various fixes, now works properly with OmniRig and flrig
      - smeter now returns proper BCD code - calibrated to emulate ICOM responses
      
   V1.0 1/24/2021
      - Initial build

   Inspired by: FT857D CAT Library, by Pavel Milanes, CO7WT, pavelmc@gmail.com

   The goal of this lib is to emulate an ICOM 746 CAT functionality
   to work with all ham radio software that include CAT control

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

 * **************************************************************************/

#ifndef IC746_h
#define IC746_h

#include <Arduino.h>

#define CAT_VER "1.1"
/*
   CAT Command definitions from IC746 Manual
*/

// Protocol
#define CAT_PREAMBLE        0xFE  // sent twice at start of command
#define CAT_EOM             0xFD  // end of message
#define CAT_ACK             0xFB  // OK
#define CAT_NACK            0xFA  // No good
#define CAT_RIG_ADDR        0x56  // Rig ID for IC746
#define CAT_CTRL_ADDR       0xE0  // Controller ID


// Commands
#define CAT_SET_TCV_FREQ    0x00  // Not implemented
#define CAT_SET_TCV_MODE    0x01  // Not implemented
#define CAT_READ_BAND_EDGE  0x02  // Not implemented
#define CAT_READ_FREQ       0x03
#define CAT_READ_MODE       0x04
#define CAT_SET_FREQ        0x05
#define CAT_SET_MODE        0x06
#define CAT_SET_VFO         0x07
#define CAT_SEL_MEM         0x08  // Not implemented
#define CAT_WRITE_MEM       0x09  // Not implemented
#define CAT_MEM_TO_VFO      0x0A  // Not implemented
#define CAT_CLEAR_MEM       0x0B  // Not implemented
#define CAT_READ_OFFSET     0x0C  // Not implemented
#define CAT_SET_OFFSET      0x0D  // Not implemented
#define CAT_SCAN            0x0E  // Not implemented
#define CAT_SPLIT           0x0F
#define CAT_SET_RD_STEP     0x10  // Not implemented
#define CAT_SET_RD_ATT      0x11  // Not implemented
#define CAT_SET_RD_ANT      0x12  // Not implemented
#define CAT_SET_UT102       0x13  // Not implemented
#define CAT_SET_RD_PARAMS1  0x14  // Not implemented
#define CAT_READ_SMETER     0x15  // Only impemented read S-Meter
#define CAT_SET_RD_PARAMS2  0x16  // Not implemented (various settings)
#define CAT_READ_ID         0x19  
#define CAT_MISC            0x1A  // Only implemented sub-command 3 Read IF filter 
#define CAT_SET_TONE        0x1B  // Not implemented (VHF/UHF)
#define CAT_PTT             0x1C

/*
   CAT Sub COmmands
*/
// Mode Subcommand
#define CAT_MODE_LSB        0x00
#define CAT_MODE_USB        0x01
#define CAT_MODE_AM         0x02 // Not implemented
#define CAT_MODE_CW         0x03 // Not implemented
#define CAT_MODE_RTTY       0x04 // Not implemented
#define CAT_MODE_FM         0x05 // Not implemented
#define CAT_MODE_CW_R       0x06 // Not implemented
#define CAT_MODE_RTTY_R     0x07 // Not implemented
#define CAT_MODE_FILTER1    0x01 // Required for "read mode"

// VFO Subcommand
#define CAT_VFO_A           0x00
#define CAT_VFO_B           0x01
#define CAT_VFO_A_TO_B      0xA0
#define CAT_VFO_SWAP        0xB0

// Split Subcommand
#define CAT_SPLIT_OFF       0x00
#define CAT_SPLIT_ON        0x01
#define CAT_SIMPLE_DUP      0x10 // Not implemented
#define CAT_MINUS_DUP       0x11 // Not implemented
#define CAT_PLUS_DUP        0x12 // Not implemented

// S-Meter / Squelch Subcommand
#define CAT_READ_SUB_SQL    0x01 // Not implemented (squelch)
#define CAT_READ_SUB_SMETER 0x02

// PTT Subcommand
#define CAT_PTT_RX          0x00
#define CAT_PTT_TX          0x01

// 1A - MISC Subcommands
#define CAT_SET_MEM_CHAN    0x00  // Not implemented
#define CAT_SET_BANDSTACK   0x01  // Not implemented
#define CAT_SET_MEM_KEYER   0x02  // Not implemented
#define CAT_READ_IF_FILTER  0x03  // Hard coded response to keep WSJTX and other CAT controllers happy

// Command Receive States
#define CAT_RCV_WAITING     0  // waiting for 1st preamble byte
#define CAT_RCV_INIT        1  // waiting for 2nd preamble byte
#define CAT_RCV_RECEIVING   2  // waiting for command bytes

// Command buffer (without preamble and EOM)
// |FE|FE|56|E0|cmd|sub-cmd|data|FD|  // Preamble (FE) and EOM (FD) are discarded leaving
// 2 addr bytes , 1 command, 1 sub-command, up to 12 data, (longest is unimplemented edge frequency)
#define CAT_CMD_BUF_LENGTH  16





// defining the funtion type by params
typedef void (*FuncPtrVoid)(void);
typedef long (*FuncPtrVoidLong)(void);
typedef byte (*FuncPtrVoidByte)(void);
typedef void (*FuncPtrBoolean)(boolean);
typedef boolean (*FuncPtrVoidBoolean)(void);
typedef void (*FuncPtrByte)(byte);
typedef void (*FuncPtrLong)(long);

/*
   The class...
*/
class IC746 {
  public:
    // we have two kind of constructors here
    void begin(); // default for the radio 9600 @ 8N2
    void begin(long baudrate, int mode); // custom baudrate and mode
    void check(); // periodic check for serial commands

    // the functions that links the lib with user supplied functions
    void addCATPtt(void (*)(boolean));
    void addCATsplit(void (*)(boolean));
    void addCATAtoB(void (*)(void));
    void addCATSwapVfo(void (*)(void));
    void addCATFSet(void (*)(long));
    void addCATMSet(void (*)(byte));
    void addCATVSet(void (*)(byte));
    void addCATGetFreq(long (*)(void));
    void addCATGetMode(byte (*)(void));
    void addCATGetPtt(boolean (*)(void));
    void addCATSMeter(byte (*)(void));

    boolean enabled     = true;

  private:
    byte cmdBuf[CAT_CMD_BUF_LENGTH];
    byte rcvState       = CAT_RCV_WAITING;
    boolean cmdRcvd     = false;
    int bytesRcvd       = 0;
    int cmdLength       = 0;
    long freq           = 0;
    void setFreq(void);
    void send(byte *, int);
    void sendResponse(byte *buf, int len);
    void sendAck(void);
    void sendNack(void);
    boolean readCmd(void);
    long BCDtoFreq(void);
    void FreqtoBCD(long);
    void SmetertoBCD(byte s);
    void doSmeter();
    void doPtt();
    void doSplit();
    void doSetVfo();
    void doSetFreq();
    void doReadFreq();
    void doSetMode();
    void doReadMode();
    void doMisc();
    void doUnimplemented_1b();
    void doUnimplemented_2b();
    void doTuneStep();
    void doAntSel();
};

#endif
