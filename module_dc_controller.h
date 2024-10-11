//
// module_dc_controller.h
//
// Header for DC controller module for CBUS (ESP32 C++ version)
//
// Very loosely based on John Purbrick and Martin Da Costa's (MERG) design of 2021,
// but essentially a complete rewrite using Mictopython and uasyncio
// Also based on Duncan Greenwood's CBUS library, but CBUS is not integrated in so far.
//
// (c) Ian Blair 11th. October 2024
//
// For license and attributions see associated readme file

/*
  © Martin Da Costa (MERG M6223) 2023
  © Duncan Greenwood (MERG M5767) 2021
  © John Fletcher (MERG M6777) 2021
  © Ian Blair (MERG M6893) 2023
  Including copyrights from CBUS_1in1out and Arduino CBUS Libraries


  This work is licensed under the:
      Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit:
      http://creativecommons.org/licenses/by-nc-sa/4.0/
   or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

   License summary:
    You are free to:
      Share, copy and redistribute the material in any medium or format
      Adapt, remix, transform, and build upon the material

    The licensor cannot revoke these freedoms as long as you follow the license terms.

    Attribution : You must give appropriate credit, provide a link to the license,
                  and indicate if changes were made. You may do so in any reasonable manner,
                  but not in any way that suggests the licensor endorses you or your use.

    NonCommercial : You may not use the material for commercial purposes. **(see note below)

    ShareAlike : If you remix, transform, or build upon the material, you must distribute
                 your contributions under the same license as the original.

    No additional restrictions : You may not apply legal terms or technological measures that
                                 legally restrict others from doing anything the license permits.

   ** For commercial use, please contact the original copyright holder(s) to agree licensing terms

    This software is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE

*/
  
const byte LED[] = { YELLOW_LED, GREEN_LED };     // LED pin connections through typ. 1K8 resistor
const byte SWITCH[] = { RENEG_BUTTON, ERASE_BUTTON };  // Module Switch takes input to 0V.

// Allocation of event variables (counting from zero, "C" style, event numbers are +1)
  #define ONTRACK 0
  #define OFFTRACK 1
  #define VOLUME 2
const byte SOUND_EVENT_VARS[] = {ONTRACK, OFFTRACK, VOLUME }; // Three event variables

const int NUM_LEDS = sizeof(LED) / sizeof(LED[0]);
const int NUM_SWITCHES = sizeof(SWITCH) / sizeof(SWITCH[0]);
const int NUM_SOUND_EVENT_VARS = sizeof(SOUND_EVENT_VARS) / sizeof(SOUND_EVENT_VARS[0]);

// module objects
Bounce moduleSwitch[NUM_SWITCHES];  //  switch as input
LEDControl moduleLED[NUM_LEDS];     //  LED as output
byte switchState[NUM_SWITCHES];

const int GLOBAL_EVS = 0;  // Number event variables for the module as a producer

#define NODE_VARS 4      //sets up number of NVs for module to store variables
//  (Only one currently in use - NV 1 default volume level)

#define NODE_EVENTS 20     //max number of events in case of teaching short events
#define EVENTS_VARS 3   //number of variables per event

#define DEVICE_NUMBERS 1  //number of device numbers. 

//////////////////////////////////////////////////////////////////////////

//Definitions

void printConfig(void);
//
/// command interpreter for serial console input
//


//
///  setup CBUS - runs once at power on called from setup()
//
void setupCBUS(); 

//
///  setup Module - runs once at power on called from setup()
//
void setupModule();

//
///  setupDCcontroller - runs once at power on called from setup()
//

void setupDCcontroller();

void setup();

void loop();

void processSwitches(void);

//
/// called from the CBUS library when a learned event is received
//
void eventhandler(byte index, CANFrame *msg);
