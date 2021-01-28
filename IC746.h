/*************************************************************************
   IC746 CAT Library, by KK4DAS, Dean Souleles
   11/24/2021

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

#include "Arduino.h"


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
#define CAT_SET_STEP        0x10  // Not implemented
#define CAT_ATT             0x11  // Not implemented
#define CAT_SEL_ANT         0x12  // Not implemented
#define CAT_SET_UT102       0x13  // Not implemented
#define CAT_SET_PARAMS1     0x14  // Not implemented
#define CAT_READ_SMETER     0x15  // Only impemented read S-Meter
#define CAT_SET_PARAMS2     0x16  // Not implemented (various settings)
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

// VFO Subcommand
#define CAT_VFO_A           0x00
#define CAT_VFO_B           0x01
#define CAT_VFO_A_TO_B      0xA0
#define CAT_VFO_SWAP        0xB0

// Split Subcommand
#define CAT_SPLIT_OFF       0x00
#define CAT_SPLIT_ON        0x01
#define CAT_SIMPLE_DUP      0x02 // Not implemented
#define CAT_MINUS_DUP       0x03 // Not implemented
#define CAT_PLUS_DUP        0x04 // Not implemented

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
    // Constructors
    void begin(); // default for the radio 9600 @ 8N2
    void begin(long baudrate, int mode); // custom baudrate and mode
    
    void check(); // periodic check for serial commands

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Callback Functions that link the library to user supplied functions
    ///////////////////////////////////////////////////////////////////////////////////////////

    // addCATPtt - registers a funtion that is called by the cat libary on resceipt of a PTT
    // command to put the rig into either Tx or Rx
    //
    // Example
    // void catSplit(boolean ptt) {
    //   if (ptt) {
    //    logic to put rig into Tx
    //   } else {
    //    logic to put the rig into Rx
    //   }
    // addCATPtt(catPtt);
    void addCATPtt(void (*)(boolean));

    // addCATSplit - registers a funtion that is called by the cat libary on resceipt of a SPLIT
    // command to turn split mode on or off
    //
    // Example
    // void catSplit(boolean split) {
    //   if (split) {
    //    logic to turn on split mode
    //   } else {
    //    logic to turn off split mode
    // }
    // addCATsplit(catSplit);  
    void addCATsplit(void (*)(boolean));
    

    // addCATAtoB - registers a funtion that is called by the cat libary on resceipt of an A=B
    // command to make both VFOs equal
    //
    // Example
    // void cataAtoB() {
    //    logic to make the alternate VFO the same as the Active VFO
    // }
    // addCATAtoB(catAtoB);      
    void addCATAtoB(void (*)(void));

    
    // addCATSwapVfo - registers a funtion that is called by the cat libary on resceipt of a toggle VFO
    // command to switch VFO A and B
    //
    // Example
    // void catSwapVfo() {
    //    logic to swap VFO A and B
    // }
    // addCATSwapVfo(catSwapVFO);   
    void addCATSwapVfo(void (*)(void));
    
    // addCATFSet - registers a funtion that is called by the cat libary on resceipt of SET FREQUENCY
    // command to set the active VFO frequency
    //
    // Example
    // void catSetFreq(long freq) {
    //    logic to tune the radio to freq
    // }
    // addCATFSet(catSetFreq); 
    void addCATFSet(void (*)(long));
    
    // addCATMSet - registers a funtion that is called by the cat libary on resceipt of SET MODE
    // command to set the active VFO MODE (USB or LSB)
    //
    // Example
    // void catSetMode(byte mode) {
    //   if (mode == CAT_MODE_LSB) {
    //      logic to set rig to LSB
    //   } else {
    //    logic to set rig to USB
    //   |
    // }
    // addCATFSet(catMode); 
    void addCATMSet(void (*)(byte));
    

    // addCATVSet - registers a funtion that is called by the cat libary on resceipt of SET VFO
    // command to set the active VFO to VFOA or VFOB)
    //
    // Example
    // void catSetVfo(byte vfo) {
    //   if (vfo == CAT_VFO_A) {
    //      logic to make VFO A active
    //   } else {
    //    logic to mmake VFO B activeB
    //   |
    // }
    // addCATFVet(catSetVfo); 
    void addCATVSet(void (*)(byte));
    
    // addCATGetFreq - registers a funtion that is called by the cat libary on resceipt of READ FREQUENCY
    // command. The user functuion must return the current frequency as a long
    //
    // Example
    // long catGetFreq() {
    //   long freq = logic to tune the radio to freq
    //   return freq;
    // }
    // addCATGetFreq(catGetFreq); 
    void addCATGetFreq(long (*)(void));
    

    // addCATGetMode - registers a funtion that is called by the cat libary on resceipt of READ MODE
    // command. The user functuion must return the current MODE USB or LSB as a byte
    //
    // Example
    // byte catGetMode() {
    //   byte mode = logic set mode to CAT_MODE_USB or CAT_MODE_LSB
    //   return mode;
    // }
    // addCATGetMode(catGetFreq); 
    void addCATGetMode(byte (*)(void));


    // addCATGetPtt- registers a funtion that is called by the cat libary on resceipt of READ PTT
    // command. The user functuion must return the current PTT state CAT_PTT_TX or CAT_PTT_RX as a byte
    //
    // Example
    // byte catGetTxRx() {
    //   byte ptt = logic set mode to CAT_PTT_TX or CAT_PTT_TX
    //   return ptt;
    // }
    // addCATGetPtt(catGetTxRx); 
    void addCATGetPtt(boolean (*)(void));
    

    // addCATGetSmeter - registers a funtion that is called by the cat libary on resceipt of READ SMETER
    // command. The user functuion must return the current S-meter reading as byte
    // S-meter values are in the range 0-16, 0-9 are S0-S9, 10-16 are S9+10 thru S9+60
    //
    // Example
    // byte catGetSMeter() {
    //   byte smeter = logic set smeter to a number from 0-16
    //   return smeter;
    // }
    // addCATGetSmeter(catGetSMeter); 
    void addCATSMeter(byte (*)(void));

    // Set enabled to false to stop processing CAT commands
    boolean enabled     = true;

  private:
    byte cmdBuf[CAT_CMD_BUF_LENGTH];
    byte rcvState       = CAT_RCV_WAITING;
    boolean cmdRcvd     = false;
    int bytesRcvd       = 0;
    int cmdLength       = 0;
    long freq           = 0;
    void setFreq(void);
    void sent(void);
    void send(byte *, int);
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
    void doUnimplemented();
};

#endif
