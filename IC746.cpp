 *************************************************************************
   IC746 CAT Library, by KK4DAS, Dean Souleles
   V1.0.2 12/17/2021
      - Send NACK on receipt of undocumented commands / fix for latest WSJTX Hamlib
      
   V1.0.1 2/3/2021
      - various fixes, now works properly with OmniRig and flrig
      - smeter now returns proper BCD code - calibrated to emulate ICOM responses
      
   V1.0 1/24/2021
      - Initial build
   Inspired by:  ft857d CAT Library, by Pavel Milanes, CO7WT, pavelmc@gmail.com

   Emulates an ICOM 746 CAT functionality to work with all ham radio software that include CAT control

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

***************************************************************************/

#include "Arduino.h"
#include "IC746.h"

//#define DEBUG_CAT
#ifdef DEBUG_CAT
#define DEBUG_CAT_DETAIL
//#define DEBUG_CAT_SMETER
#include <SoftwareSerial.h>
SoftwareSerial catDebug(6, 7); // RX, TX
String dbg;
#endif


// User supplied calback function work variables, must be static
static FuncPtrBoolean catSplit;
static FuncPtrBoolean catSetPtt;
static FuncPtrVoidBoolean catGetPtt;
static FuncPtrVoidLong catGetFreq;
static FuncPtrLong catSetFreq;
static FuncPtrVoidByte catGetMode;
static FuncPtrByte catSetMode;
static FuncPtrVoidByte catGetSmeter;
static FuncPtrByte catSetVFO;
static FuncPtrVoid catAtoB;
static FuncPtrVoid catSwapVfo;


// Command indices
//
// Command structure after preamble and EOM have been discarded
// |56|E0|cmd|sub-cmd|data
//   56 - fixed transceiver address for IC746
//   E0 - fixed cat xontroller address
// The sub-command field varies by command. For somme commands the sub-cmd field is not supplied and the data
// begins immediatedly follwing the command.
//
// The following are the array indedes within the command buffer for command elements that are sent with the command
// or are where we put data to return to the conroller
//
#define CAT_IX_TO_ADDR     0
#define CAT_IX_FROM_ADDR   1
#define CAT_IX_CMD         2
#define CAT_IX_SUB_CMD     3
#define CAT_IX_FREQ        3   // Set Freq has no sub-command
#define CAT_IX_MODE        3   // Get mode has no sub-command
#define CAT_IX_TUNE_STEP   3   // Get step has no sub-command
#define CAT_IX_ANT_SEL     3   // Get amt has no sub-command
#define CAT_IX_PTT         4   // PTT RX/TX indicator
#define CAT_IX_IF_FILTER   4   // IF Filter value
#define CAT_IX_SMETER      4   // S Meter 0-255
#define CAT_IX_SQUELCH     4   // Squelch 0=close, 1= open
#define CAT_IX_ID          5
#define CAT_IX_DATA        4   // Data following sub-comand

// Lentgth of commands that request data 
#define CAT_RD_LEN_NOSUB   3   //  3 bytes - 56 E0 cc
#define CAT_RD_LEN_SUB     4   //  4 bytes - 56 E0 cc ss  (cmd, sub command)

// Length of data responses
#define CAT_SZ_SMETER      6   //  6 bytes - E0 56 15 02 nn nn 
#define CAT_SZ_SQUELCH     5   //  5 bytes - E0 56 15 01 nn
#define CAT_SZ_PTT         5   //  5 bytes - E0 56 1C 00 nn
#define CAT_SZ_FREQ        8   //  8 bytes - E0 56 03 ff ff ff ff ff  (frequency in little endian BCD)
#define CAT_SZ_MODE        5   //  5 bytes - E0 56 04 mm ff  (mode, then filter)
#define CAT_SZ_IF_FILTER   5   //  5 bytes - E0 56 1A 03 nn
#define CAT_SZ_TUNE_STEP   4   //  4 bytes - E0 56 10 nn
#define CAT_SZ_ANT_SEL     4   //  4 bytes - E0 56 12 nn
#define CAT_SZ_ID          5   //  5 bytes - E0 56 19 00 56    (returns RIG ID)
#define CAT_SZ_UNIMP_1B    5   //  5 bytes - E0 56 NN SS 00    (unimplemented commands that require 1 data byte
#define CAT_SZ_UNIMP_2B    6   //  6 bytes - EO 56 NN SS 00 00 (unimplemented commandds that required 2 data bytes



/*
   Contructor, simple constructor, it initiates the serial port in the
   default mode for the radio: 9600 @ 8N2
*/
void IC746::begin() {
  Serial.begin(9600, SERIAL_8N2);
  while (!Serial);;
  Serial.flush();

#ifdef DEBUG_CAT
  catDebug.begin(9600);
  dbg = "CAT Debug Ready:";
  catDebug.println(dbg.c_str());
#endif
}

// Alternative initializer with a custom baudrate and mode
void IC746::begin(long br, int mode) {
  /*
     Allowed Arduino modes for the serial:
      SERIAL_5N1; SERIAL_6N1; SERIAL_7N1; SERIAL_8N1; SERIAL_5N2; SERIAL_6N2;
      SERIAL_7N2; SERIAL_8N2; SERIAL_5E1; SERIAL_6E1; SERIAL_7E1; SERIAL_8E1;
      SERIAL_5E2; SERIAL_6E2; SERIAL_7E2; SERIAL_8E2; SERIAL_5O1; SERIAL_6O1;
      SERIAL_7O1; SERIAL_8O1; SERIAL_5O2; SERIAL_6O2; SERIAL_7O2; SERIAL_8O2
  */
  Serial.begin(br, mode);
  Serial.flush();
#ifdef DEBUG_CAT
  catDebug.begin(9600);
  dbg = "CAT Debug Ready";
  catDebug.println(dbg.c_str());
#endif
}

/*
   Linking user supplied callback functions
*/

// PTT
void IC746::addCATPtt(void (*userFunc)(boolean)) {
  catSetPtt = userFunc;
}

// Split
void IC746::addCATsplit(void (*userFunc)(boolean)) {
  catSplit = userFunc;
}

// VFO A=B - set both VFOs to be the same as the active VFO
void IC746::addCATAtoB(void (*userFunc)(void)) {
  catAtoB = userFunc;
}

// Swap Active VFO
void IC746::addCATSwapVfo(void (*userFunc)(void)) {
  catSwapVfo = userFunc;
}

// Get the freq of operation, the function must return the freq
void IC746::addCATGetFreq(long (*userFunc)(void)) {
  catGetFreq = userFunc;
}

// Get the mode of operation, the function must return the mode
void IC746::addCATGetMode(byte (*userFunc)(void)) {
  catGetMode = userFunc;
}

// GetPTT - function must return true for Tx and false for Rx
void IC746::addCATGetPtt(boolean (*userFunc)(void)) {
  catGetPtt = userFunc;
}


// S meter (user function must return 0-15)
void IC746::addCATSMeter(byte (*userFunc)(void)) {
  catGetSmeter = userFunc;
}

// Set Frequency - user function must accept a long as the freq in hz
void IC746::addCATFSet(void (*userFunc)(long)) {
  catSetFreq = userFunc;
}

// Set Mode -  user function must interpret MODE per the constants in IC746.h
void IC746::addCATMSet(void (*userFunc)(byte)) {
  catSetMode = userFunc;
}

// SEt VFOA or B - user function must interpret VFO per the constants in IC746.h
void IC746::addCATVSet(void (*userFunc)(byte)) {
  catSetVFO = userFunc;
}

////////////////////////////////////////////////////////////////////////////////
// Protocol Message Handling
////////////////////////////////////////////////////////////////////////////////

//
// Send a message back to CAT controller
// Format PREAMBLE, PREAMBLE, MSG, EOM
//
void IC746::send(byte *buf, int len) {
  int i;

  Serial.write(CAT_PREAMBLE);
  Serial.write(CAT_PREAMBLE);

  for (i = 0; i < len; i++) {
    Serial.write(buf[i]);
  }
  Serial.write(CAT_EOM);

#ifdef DEBUG_CAT_DETAIL
  dbg = "sent: ";
  dbg += String(len);
  dbg += ": ";
  for (int i = 0; i < len; i++) {
    dbg += String(buf[i], HEX);
    dbg += " ";
  }
  catDebug.println(dbg.c_str());
#endif

}

//
// sendResponse
// 
void IC746::sendResponse(byte *buf, int len) {
  buf[CAT_IX_FROM_ADDR] = CAT_RIG_ADDR;
  buf[CAT_IX_TO_ADDR] = CAT_CTRL_ADDR;
  send(buf, len);
}

//
// sendAck() - send back hard-code acknowledge message
//
void IC746::sendAck() {
  byte ack[] = {CAT_CTRL_ADDR, CAT_RIG_ADDR, CAT_ACK};
  send(ack, 3);
}

//
// sendNack() - send back hard-code negative-acknowledge message
//
void IC746::sendNack() {
  byte nack[] = {CAT_CTRL_ADDR, CAT_RIG_ADDR, CAT_NACK};
  send(nack, 3);
}


/*
   readCMD - state machine to receive a command from the controller
   States:
      CAT_RCV_WAITING    - scan incoming serial data for first preamble byte
      CAT_RCV_INIT       - second premable byte confirms start of message
      CAT_RCV_RECEIVING  - fill command buffer until EOM received

   Command format
   |FE|FE|56|E0|cmd|sub-cmd|data|FD|

    FE FE = preamble, FD = end of command
    56 = transceiver default address for IC746 (unused)
    E0 = CAT controller default address (unused)

    Upon successful receipt of EOM, protocol requires echo back of enitre message
    On interrupted preamble or buffer overflow (no EOM received), send NAK

    On successful receipt of a command the global array cmdBuf
    will have the received CAT command (without the preamble and EOM)
*/
boolean IC746::readCmd() {
  byte bt;
  boolean cmdRcvd = false;

  while (Serial.available() && !cmdRcvd) {

    bt = byte(Serial.read());

    switch (rcvState) {

      case CAT_RCV_WAITING:   // scan for start of new command
        if (bt == CAT_PREAMBLE) {
          rcvState = CAT_RCV_INIT;
        }
        break;

      case CAT_RCV_INIT:      // check for second preamble byte
        if (bt == CAT_PREAMBLE) {
          rcvState = CAT_RCV_RECEIVING;
        } else {              // error - should not happen, reset and report
          rcvState = CAT_RCV_WAITING;
          bytesRcvd = 0;
          sendNack();
        }
        break;

      case CAT_RCV_RECEIVING:
        switch (bt) {

          case CAT_EOM:        // end of message received, return for processing, reset state

#ifdef DEBUG_CAT_DETAIL
            dbg = "rcvd: ";
            dbg += String(bytesRcvd);
            dbg += ": ";
            for (int i = 0; i < bytesRcvd; i++) {
              dbg += String(cmdBuf[i], HEX);
              dbg += " ";
            }
            catDebug.println(dbg.c_str());
#endif

            send(cmdBuf, bytesRcvd);  // echo received packet for protocol
            cmdRcvd = true;
            rcvState = CAT_RCV_WAITING;
            cmdLength = bytesRcvd;
            bytesRcvd = 0;
            break;

          default:            // fill command buffer
            if (bytesRcvd <= CAT_CMD_BUF_LENGTH) {
              cmdBuf[bytesRcvd] = bt;
              bytesRcvd++;
            } else {           // overflow - should not happen reset for new comand
              rcvState = CAT_RCV_WAITING;
              bytesRcvd = 0;
              sendNack();      // report error
            }
            break;
        }
        break;
    }
  }
  return cmdRcvd;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                        COMMAND PROCESSORS
//
///////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////
// doSmeter() - process the CAT_READ_SMETER Command
// This command has two sub-commands, SMETER and SQUELCH, only SMETER is fully implemented
//
// The SMETER sub command requests the current Return S-meter readingscaled for IC 746 appropriate values
//      It calls a user function returns 0-15  S0-9, +10, +20, +30, +40, +50 +60
//      The mapping of s-units to IC746 equivalents done imperically using the CatBkt cat test program
//
// The second sub-command, SQUELCH request wheterh Squelch is open or closed.  Return a fixed value
// of "Open" to keep the protocol happly
///////////////////////////////////////////////////////////////////////////////////////////////////////

  void IC746::doSmeter() {
  switch (cmdBuf[CAT_IX_SUB_CMD]) {
    case CAT_READ_SUB_SMETER:
      if (catGetSmeter) {
                          //S0  S1  S2  S3  S4  S5  S6  S7   S8   S9  +10  +20  +30  +40  +50  +60
        const byte smap[] = {0, 15, 25, 40, 55, 65, 75, 90, 100, 120, 135, 150, 170, 190, 210, 241};
        byte s = catGetSmeter();

        SmetertoBCD(smap[s]);
  #ifdef DEBUG_CAT_DETAIL
        dbg = "doSmeter- s:";
        dbg += String(s);
        dbg += " map: ";
        dbg += String(smap[s]);
        dbg += " BCD: ";
        for (int i = CAT_IX_SMETER; i < CAT_IX_SMETER+2; i++) {
          dbg += String(cmdBuf[i], HEX);
          dbg += " ";
        }
        catDebug.println(dbg.c_str());
  #endif

      } else {
        cmdBuf[CAT_IX_SMETER] = 0;      // user has not supplied S Meter function - keep the protocol happy
        cmdBuf[CAT_IX_SMETER + 1] = 0;
      }
      sendResponse(cmdBuf, CAT_SZ_SMETER);
      break;

    case CAT_READ_SUB_SQL:        // Squelch condition 0=closed, 1=open
      cmdBuf[CAT_IX_SQUELCH] = 1;
      send(cmdBuf, CAT_SZ_SQUELCH);
      break;
  }
  }



///////////////////////////////////////////////////////////////////////////////////////////////////////
// doPtt() - process the CAT_PTT Command
// CAT_PTT calls the user supplied functions to either put the rig in to Tx or Rx or to request the rigs
// current Tx/Rx state
//
// A "read ptt" request does not have a data byte and is therefore one byte shorter
// than a "set ptt" request.
//      56 | E0 | 1C | 00      - Read request 1C is PTT command, sub-command 0 is uused, no data byte
//      56 | E0 | 1C | 00 | 01 - Set Tx, trailing data bit 1 for Tx, 0 for Rx
////////////////////////////////////////////////////////////////////////////////////////////////////////
void IC746::doPtt() {
  if (cmdLength == CAT_RD_LEN_SUB) {  // Read request
    if (catGetPtt) {
      cmdBuf[CAT_IX_PTT] = catGetPtt();
      sendResponse(cmdBuf, CAT_SZ_PTT);
    }
  } else {               // Set request
    if (catSetPtt) {
      if (cmdBuf[CAT_IX_PTT] == CAT_PTT_TX) {
        catSetPtt(true);
      } else {
        catSetPtt(false);
      }
    }
    sendAck();  // always acknowledge "set" commands
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// doSplit() - process the CAT_SPLIT Command
// Call user supplied function to turn split on or off
///////////////////////////////////////////////////////////////////////////////////////////////////////
void IC746::doSplit() {
  switch (cmdBuf[CAT_IX_SUB_CMD]) {
    case CAT_SPLIT_OFF:
      if (catSplit) {
        catSplit(false);
      }
      break;
    case CAT_SPLIT_ON:
    case CAT_SIMPLE_DUP:
      if (catSplit) {
        catSplit(true);
      }
    default:
      break;
  }
  sendAck();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// doSetVfo() - process the CAT_SET_VFO Command, calls user supplied functions
// SET_VFO with no sub-command selects VFO Tuning vice Memory Tuning (memory tuning is not implemented)
// SET_VFO has four sub-commands:
//    VFOA or VFOB - directs the rig to make the selected VFO the active VFO
//    VFO_A_TO_B - directs the rig to copy the make both VFO frequencies the same as the Active VFO
//    VFO_SWAP - directs the rig to exchange VFOA and VFOB
///////////////////////////////////////////////////////////////////////////////////////////////////////
void IC746::doSetVfo() {

  if (cmdLength == CAT_RD_LEN_NOSUB) {  // No sub-command - sets VFO Tuning vice memory tuning
    sendAck();           // Memory tuning is not implemented so send ack to keep protocol happy
    return;
  }

  switch (cmdBuf[CAT_IX_SUB_CMD]) {
    case CAT_VFO_A:
    case CAT_VFO_B:
      if (catSetVFO) {
        catSetVFO(cmdBuf[CAT_IX_SUB_CMD]);
      }
      break;
    case CAT_VFO_A_TO_B:
      if (catAtoB) {
        catAtoB();
      }
      break;
    case CAT_VFO_SWAP:
      if (catSwapVfo) {
        catSwapVfo();
      }
      break;
  }
  sendAck();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// doSetFreq() - proces CAT_SET_FREQ command
// Call the user supplied function to set the righ frequency
///////////////////////////////////////////////////////////////////////////////////////////////////////
void IC746::doSetFreq() {
  if (catSetFreq) {
    catSetFreq(BCDtoFreq());  // Convert the frequency BCD to Long and call the user function
  }
  sendAck();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// doReadFreq() - process the CAT_READ_FREQ command
// Call the user supplied function to read set the rig frequency
// Frequecies are sent and received in BCD - call the support functions to do the conversion
///////////////////////////////////////////////////////////////////////////////////////////////////////
void IC746::doReadFreq() {
  if (catGetFreq) {
    FreqtoBCD(catGetFreq());  // get the frequency, convert to BCD and stuff it in the response buffer
    sendResponse(cmdBuf, CAT_SZ_FREQ);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// doSetMode() - process the CAT_SET_MODE command
// Call the user function to put the rig into the requested mode.  Only USB or LSB are supported.
///////////////////////////////////////////////////////////////////////////////////////////////////////
void IC746::doSetMode() {
  if (catSetMode) {
    switch (cmdBuf[CAT_IX_SUB_CMD]) {
      case CAT_MODE_LSB:
      case CAT_MODE_USB:
        catSetMode(cmdBuf[CAT_IX_SUB_CMD]);
        break;
    }
  }
  sendAck();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// doReadMode() - process the CAT_READ_MODE command
// Call the user function to query the rig's current mode
///////////////////////////////////////////////////////////////////////////////////////////////////////
void IC746::doReadMode() {
  if (catGetMode) {
    cmdBuf[CAT_IX_MODE] = catGetMode();
    cmdBuf[CAT_IX_MODE+1] = CAT_MODE_FILTER1;  // protocol filter - return reasonable value
    sendResponse(cmdBuf, CAT_SZ_MODE);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// doMisc() - process the CAT_MISC command
//
// The MISC command has several unrelated sub-commands
// The only one implemented here is the sub-command to read the IF Filter Setting
// The code sends a hard-coded response since most homebrew rigs won't have such a setting
// but programs like WSJTX and FLDIGI request it
//
// Commands that "set" values are replied to with an ACK message
///////////////////////////////////////////////////////////////////////////////////////////////////////
void IC746::doMisc() {
  switch (cmdBuf[CAT_IX_SUB_CMD]) {
    case CAT_READ_IF_FILTER:
      cmdBuf[CAT_IX_IF_FILTER] = 0;
      sendResponse(cmdBuf, CAT_SZ_IF_FILTER);
      break;

    // Not implemented
    // Reply with ACK to keep the protocol happy
    case CAT_SET_MEM_CHAN:
    case CAT_SET_BANDSTACK:
    case CAT_SET_MEM_KEYER:
      sendAck();
      break;
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
//                       UNIMPLEMENTED COMMAND STUBS
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
// doUnimplemented() - reasonable processing for features that are not fully implemented
//
// Unimplimented command handling
//
// These following commands are used to both set and read various parameters in the IC-746 that are not
// typically implemented in a homebrew transceiver.
// Commands requesting the state of various parameters require one or two data bytes returned.
// We return zero in all cases which typically means the requsted feature is OFF - eg AGC, NB, VOX, etc.
// Command that "set" various parameters only require an ACK reply
// A "read" request has no data byte and is one byte shorter than a set request  (length =4)
///////////////////////////////////////////////////////////////////////////////////////////////////////
void IC746::doUnimplemented_1b() {
  if (cmdLength == CAT_RD_LEN_SUB) {        // Read request
    cmdBuf[CAT_IX_DATA] = 0;   // return 0 for all read requests
    sendResponse(cmdBuf, CAT_SZ_UNIMP_1B);
  } else {                   // Set parameter request
    sendAck();               // Send an acknowledgement to keep the protocol happy
  }
}

void IC746::doUnimplemented_2b() {
  if (cmdLength == CAT_RD_LEN_SUB) {        // Read request
    cmdBuf[CAT_IX_DATA] = 0;   // return 0 for all read requests
    cmdBuf[CAT_IX_DATA+1] = 0; 
    sendResponse(cmdBuf, CAT_SZ_UNIMP_2B);
  } else {                   // Set parameter request
    sendAck();               // Send an acknowledgement to keep the protocol happy
  }
}

void IC746::doTuneStep() {
  if (cmdLength == CAT_RD_LEN_NOSUB) {             // Read request
    cmdBuf[CAT_IX_TUNE_STEP] = 0;   // return 0 for all read requests
    sendResponse(cmdBuf, CAT_SZ_TUNE_STEP);
  } else {                   // Set parameter request
    sendAck();               // Send an acknowledgement to keep the protocol happy
  }
}

void IC746::doAntSel() {
  if (cmdLength == CAT_RD_LEN_NOSUB) {           // Read request
    cmdBuf[CAT_IX_ANT_SEL] = 0;   // return 0 for all read requests
    sendResponse(cmdBuf, CAT_SZ_ANT_SEL);
  } else {                   // Set parameter request
    sendAck();               // Send an acknowledgement to keep the protocol happy
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
//  check() - process commands from CAT controller, should be called from the sketch main loop
///////////////////////////////////////////////////////////////////////////////////////////////////////
void IC746::check() {

  // do nothing if it was disabled by software
  if (!enabled) return;

  // Receive a CAT Command
  if (!readCmd()) return;

/*
#ifdef DEBUG_CAT_DETAIL
  dbg = "rcvd: ";
  dbg += String(cmdLength);
  dbg += ": ";
  for (int i = 0; i < cmdLength; i++) {
    dbg += String(cmdBuf[i], HEX);
    dbg += " ";
  }
  catDebug.println(dbg.c_str());
#endif
*/
  // Process the command - command opcode is at CAT_IX_CMD location in command buffer
  switch (cmdBuf[CAT_IX_CMD]) {

    case CAT_PTT:
      doPtt();
      break;

    case CAT_SPLIT:
      doSplit();
      break;

    case CAT_SET_VFO:
      doSetVfo();
      break;

    case CAT_SET_FREQ:
      doSetFreq();
      break;

    case CAT_SET_MODE:
      doSetMode();
      break;

    case CAT_READ_MODE:
      doReadMode();
      break;

    case CAT_READ_FREQ:
      doReadFreq();
      break;

    case CAT_READ_SMETER:
      doSmeter();
      break;

    case CAT_MISC:
      doMisc();
      break;

    case CAT_READ_ID:
      cmdBuf[CAT_IX_ID] = CAT_RIG_ADDR;      // Send back the transmitter ID
      send(cmdBuf, CAT_SZ_ID);
      break;

    // Unimplemented commands that request one or two bytes of data from the rig - keep the protocol happy
    case CAT_SET_RD_STEP:
      doTuneStep();
      break;
      
    case CAT_SET_RD_ANT:
      doAntSel();
      break;
      
    case CAT_SET_RD_ATT:
    case CAT_SET_RD_PARAMS2:
      doUnimplemented_1b();
      break;

    case CAT_SET_RD_PARAMS1:
    case CAT_READ_OFFSET:
      doUnimplemented_2b();
      break;
      
    default:                // For all other commands respond with an NACK
#ifdef DEBUG_CAT
      dbg = "unimp cmd: ";
      dbg += String(cmdBuf[CAT_IX_CMD], HEX);
      catDebug.println(dbg.c_str());
#endif
      sendNack();
      break;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                        UTILITY FUNCTIONS
//
///////////////////////////////////////////////////////////////////////////////////////////////////////
//
// BCD FrequencyConversion Routines
// Convert BCD frequency to/from command buffer to/from long
// Format (beginning at first data byte in buffer is
//  Byte 0 10Hz   | 1Hz
//  Byte 1 1KHz   | 100Hz
//  Byte 2 100KHz | 10KHz
//  Byte 3 10MHz  | 1MHz
// Example: 7,123,456 is encoded 56 | 34 | 12 | 07
//
long IC746::BCDtoFreq() {
  long freq;

  freq = cmdBuf[CAT_IX_FREQ] & 0xf;                 // lower 4 bits
  freq += 10L * (cmdBuf[CAT_IX_FREQ] >> 4);          // upper 4 bits
  freq += 100L * (cmdBuf[CAT_IX_FREQ + 1] & 0xf);
  freq += 1000L * (cmdBuf[CAT_IX_FREQ + 1] >> 4);
  freq += 10000L * (cmdBuf[CAT_IX_FREQ + 2] & 0xf);
  freq += 100000L * (cmdBuf[CAT_IX_FREQ + 2] >> 4);
  freq += 1000000L * (cmdBuf[CAT_IX_FREQ + 3] & 0xf);
  freq += 10000000L * (cmdBuf[CAT_IX_FREQ + 3] >> 4);

  return freq;
}

void IC746::FreqtoBCD(long freq) {
  byte ones, tens, hund, thou, ten_thou, hund_thou, mil, ten_mil;

  ones =     byte(freq % 10);
  tens =     byte((freq / 10L) % 10);
  cmdBuf[CAT_IX_FREQ] = byte((tens << 4)) | ones;

  hund =      byte((freq / 100L) % 10);
  thou =      byte((freq / 1000L) % 10);
  cmdBuf[CAT_IX_FREQ + 1] = byte((thou << 4)) | hund;

  ten_thou =  byte((freq / 10000L) % 10);
  hund_thou = byte((freq / 100000L) % 10);
  cmdBuf[CAT_IX_FREQ + 2] = byte((hund_thou << 4)) | ten_thou;

  mil =       byte((freq / 1000000L) % 10);
  ten_mil =   byte(freq / 10000000L);
  cmdBuf[CAT_IX_FREQ + 3] = byte((ten_mil << 4)) | mil;

  cmdBuf[CAT_IX_FREQ + 4] = 0; // fixed

}

void IC746::SmetertoBCD(byte s) {
  byte ones, tens, hund;

  ones =     byte(s % 10);
  tens =     byte((s / byte(10)) % 10);
  cmdBuf[CAT_IX_SMETER+1] = byte((tens << 4)) | ones;

  hund =      byte((s / byte(100)) % 10);
  cmdBuf[CAT_IX_SMETER] = hund;

}
