# ICOM IC-746 CAT Control Emulation for Arduino #

This library implements the ICOM CI-V CAT protocol emulating an IC-746 transceiver.  The libary has been tested on a homebrew transceiver of my creation.  It has been demonstrated to work with an Arduino Nano and Arduino Nano Every although it should work with any MCU with a serial port.  It works with WSJTX in all modes as well as the fldigi suite, Ham Radio Delux, the hamlib rigclt command line, and the CatBkt universal rig AT conntrol tool.  It should work with any hamlib enabled software.

Documentation for the ICOM IC-746 may be found here:

https://www.icomamerica.com/en/downloads/default.aspx?Category=42 

The following functions are implemented
* PTT (ON/OFF)
* VFO A/B Selection
* VFO Swap
* VFO Set A=B
* Split (On/Off)
* Frequency GET/SET
* Mode GET/SET (USB, LSB only)
* S-meter level GET

All other functions are coded to give correct reasonable responses to other CAT commands.

I have found this set of functions to be all that is required for the majorit of logging programs and digital modes such as the WSJTX suite and FLDIGI.


## Supported software ##

So far we have tested support for the following CAT capable software:

* Hamlib
* Fldigi (using Hamlib)
* WSJTX
* CatBKT
* Ham Radio Deluxe

If you use find this library does not work with any particular CAT enabled software please let me know via the "Issues" tab on Github.

## Known limitations ##

Split frequency handling in WSJTX does not work as expected.  The work-around is to configure WSJTX Split for either NONE or "Fake it".  Split functionality has been tested and works as expected using CatBkt.  

## Tips for developers: ##

The library implements the IC-746 version of ICOM's CI-V protocol.  I chose the IC-746 as the target as it is the first in the ICOM linde to implement all of what is needed to run digital modes easily and doesn't have too many extra features to code around.

The CI-V protocol is reasonably well documented but I have made some  some assumptions about appropriate responses. I referred to the IC-7600 CIV referece manual which has much additional detail. I did not have an IC-746 available for comparison, but I tested with the Hamlib rigctl program and all functions work without error.  I also have tested with WSJTX, FLDIGI and Ham Radio Deluxe. I found the program CatBkt to be especially helpful as it exposes many controls that are not used in other programs.  

The library is coded to give reasonable responsed to 100% of the IC-746 cat protocol commands whether or not the feature is actually implemented.

## Inspiration ##

This library has was inspired by the work of Pavel Milanes, C07WT on his Yaes FT-857D library

## Installation ##

Copy the IC746CAT folder to the Arduino libraries folder.

## Usage ##

The libary uses the the hardware serial port "Serial" to communcate with the CAT controller softaware.  The default paramters are baud 19200, N81, the same as the IC-746.

To use the library you must code a few custom call back functions and register them with the library.  Your callbacks will require specific parameters and return values based on the particular command.  Your functions will be called by the CAT libary upon receipt of the relevant CAT commands.  Your functions need to take whatever actions are necessary to implement the command and where requested, return the appropriate value to the library for transmission back to the CAT controller.

in your sketch:
```C++
#include <IC746.h>

IC746 radio = IC746();

long frequency = 7200000L;
byte activVFO = CAT_VFO_A; 

long catGetFreq() {
  return frequency;
}

void catSetVFO(byte vfo) {
  if (vfo == CAT_VFO_A) {
    activeVFO = CAT_VFO_A;
  } else {
    activeVFO = CAT_VFO_B;
  }
}
```

In your setup() function register your callbacks and start the interface:
```C++
radio.addCATGetFreq(catGetFreq);
radio.addCATVSet(catSetVFO);
radio.begin();
```
In your main loop call the check() function:
```C++
radio.check()
```
See the example sketch for more examples.

A word on the example sketch.  It is configured to write debug output to a ILI9341 TFT using the Adafruit libraries, because that is what I had on the bench. It should be straightforward to modify it to use SoftwareSerial or other output device of your choice.  There is also debug code in the library itself to send all received CAT command to a SoftwareSerial port.

## Author & contributors ##

The only author is Dean Souleles, KK4DAS, reachable at kk4das@gmail.com.

## Where to download the latest version? ##

Always download the latest version from the [github repository](https://github.com/KK4DAS/IC746CAT/)


