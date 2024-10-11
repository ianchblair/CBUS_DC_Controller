#
# module_dc_controller.cpp
#
# Code for DC controller module for CBUS (ESP32 C++ version)
#
# Very loosely based on John Purbrick and Martin Da Costa's (MERG) design of 2021,
# but essentially a complete rewrite using Mictopython and uasyncio
# Also based on Duncan Greenwood's CBUS library, but CBUS is not integrated in so far.
#
# (c) Ian Blair 3rd. April 2024
#
# For license and attributions see associated readme file
#

import uasyncio as asyncio
from machine import Pin,Timer,UART,DAC,ADC

import aiorepl
import cbus
import cbusconfig
import cbusdefs
import cbusmodule
import mcp2515

import aiorepl

from utime import sleep_us, sleep_ms, sleep
import pindefs_dc_controller_esp32 as pindefs
import dc_controller_defs as defs
import throttle
import logger

#TestPin=Pin(pindefs.PIN_SND_TXD,Pin.OUT)
timer=Timer()
log=logger.logger()
              
// CANsoundb
// CANsound rewritten for Duncan Greenwoo's CBUS library
// This draft Ian Blair 08 Dec 2023 

/*
  © Martin Da Costa (MERG M6223) 2023
  © Duncan Greenwood (MERG M5767) 2021
  © John Fletcher (MERG M6777) 2021
  © Sven Rosvall (MERG M3777) 2021
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

/*
      3rd party libraries needed for compilation:

      Streaming   -- C++ stream style output, v5, (http://arduiniana.org/libraries/streaming/)
      ACAN2515    -- library to support the MCP2515/25625 CAN controller IC
      CBUSSwitch  -- library access required by CBUS and CBUS Config
      CBUSLED     -- library access required by CBUS and CBUS Config
      DFRobotDFplayermini -- library for DF Robot MP3 player
*/
///////////////////////////////////////////////////////////////////////////////////
// Pin Use map MEGA:
// Digital pin 0          Serial0 Rx (Used for control and debug)
// Digital pin 1          Serial0 Tx (Used for control and debug)
// Digital pin 2          Not used
// Digital pin 3          Not Used
// Digital pin 4          Not Used
// Digital pin 5          Not Used
// Digital pin 6 (PWM)    Module Switch 2
// Digital pin 7          Module LED 2
// Digital pin 8          Module LED 1
// Digital pin 9 (PWM)    Module Switch 1

// Digital pin 14         Serial3 Rx (Connection to sound player)
// Digital pin 15         Serial3 Tx (Connection to sound player)
// Digital pin 16         Serial2 Rx (Option for second sound player)
// Digital pin 17         Serial2 Tx (Option for second sound player)
// Digital pin 18         Interupt CAN
// Digital pin 19         Not used

// Digital pin 50 (MOSI)  SI    CAN
// Digital pin 51 (MISO)  SO    CAN
// Digital pin 52 (SCK)   Sck   CAN
// Digital pin 53 (SS)    CS    CAN

// Digital / Analog pin 0     Not Used
// Digital / Analog pin 1     Not Used
// Digital / Analog pin 2     Not Used
// Digital / Analog pin 3     Not Used
// Digital / Analog pin 4     Not Used
// Digital / Analog pin 5     Not Used
// Digital / Analog pin 6     Not Used
// Digital / Analog pin 7     Sound player busy
//////////////////////////////////////////////////////////////////////////

// Uno has one serial port, so we can debug OR use user serial data
// For other Arduinos there may be more, so this is FFS
#define DEBUG 1  // set to 0 for no serial debug
#define MEGA 1
// Serial ports #defined for ease of change later
// Use SerialUSB for Due, Serial or Serial1 for Mega
#define DEBUG_SERIAL Serial
#define DEBUG_SERIAL_PORT_SPEED 115200
#define USER_SERIAL Serial3
#define USER_SERIAL_PORT_SPEED 9600

#if DEBUG
#define DEBUG_PRINT(S) DEBUG_SERIAL << S << endl
#else
#define DEBUG_PRINT(S)
#endif

// 3rd party libraries
#include <Streaming.h>
#include <Bounce2.h>

#include <DFRobotDFPlayerMini.h>  //for the sound card

// CBUS library header files
//#include <CBUS.h>    // CBUS base class
#include <CBUS2515.h>    // CAN controller and CBUS class
#include "LEDControl.h"  // CBUS LEDs
#include <CBUSconfig.h>  // module configuration
#include <cbusdefs.h>    // MERG CBUS constants
#include <CBUSParams.h>

////////////DEFINE MODULE/////////////////////////////////////////////////

// module name
unsigned char mname[7] = { 'S', 'o', 'u', 'n', 'd', 'b', ' ' };

// constants
const byte VER_MAJ = 3;     // code major version
const char VER_MIN = 'b';   // code minor version
const byte VER_BETA = 0;    // code beta sub-version
const byte MODULE_ID = 101;  // CBUS module type

const unsigned long CAN_OSC_FREQ = 8000000;  // Oscillator frequency on the CAN2515 board (else 16000000 for Due CAN controller, and some types of 2515 board)

// Module pins available for use are Pins 3 - 9 and A0 - A5
  #define GREEN_LED 5                  //Merg shield 4 or sound shield 5 green led port
  #define YELLOW_LED 4                 //Merg shield 7 or sound shield 4 yellow led port
  #define RENEG_BUTTON 6               //Merg shield 8 or sound shield 6 reneg push button
  #define ERASE_BUTTON 7               //Merg shield 6 or sound shield 7 EEPRROM erase push button

  
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

//CBUS pins
// const byte CAN_INT_PIN = 2;  // Only pin 2 and 3 support interrupts - Uno
const byte CAN_INT_PIN = 18;  // Pin 18 and others support interrupts - MEGA
//const byte CAN_CS_PIN = 10;  // For Uno
const byte CAN_CS_PIN = 53;  // For Mega

// Reserved pins - used by CBUS, but no explicit definition required
//const byte CAN_SI_PIN = 11 or 54 on Mega
//const byte CAN_SO_PIN = 12 or 55 on Mega
//const byte CAN_SCK_PIN = 13 or 56 on Mega

// CBUS objects
CBUSConfig module_config;       // configuration object
CBUS2515 CBUS(&module_config);  // CBUS object

// Sound player variables
#define DEFAULT_VOLUME 20       // Default used if not set up
int sound_volume = DEFAULT_VOLUME;               // current volume level

//Create the SoundPlayer object
DFRobotDFPlayerMini SoundPlayer1;

// Plays a track at the given volume
void play(int nTrack, int nVolume) 
{
  // If nVolume set to zero use saved volume instead
  if (nVolume == 0) nVolume = sound_volume;
  SoundPlayer1.volume(nVolume); 
  SoundPlayer1.play(nTrack);    
}

void printConfig(void) {
  // code version
  DEBUG_SERIAL << F("> code version = ") << VER_MAJ << VER_MIN << F(" beta ") << VER_BETA << endl;
  DEBUG_SERIAL << F("> compiled on ") << __DATE__ << F(" at ") << __TIME__ << F(", compiler ver = ") << __cplusplus << endl;

  // copyright
  DEBUG_SERIAL << F("> © Martin Da Costa (MERG M6223) 2023") << endl;
  DEBUG_SERIAL << F("> © Duncan Greenwood (MERG M5767) 2021") << endl;
  DEBUG_SERIAL << F("> © John Fletcher (MERG M6777) 2021") << endl;
  DEBUG_SERIAL << F("> © Sven Rosvall (MERG M3777) 2021") << endl;
  DEBUG_SERIAL << F("> © Ian Blair (MERG M6893) 2023") << endl;
}
//
/// command interpreter for serial console input
//


//
///  setup CBUS - runs once at power on called from setup()
//
void setupCBUS() {
  // set config layout parameters
  module_config.EE_NVS_START = 10;
  module_config.EE_NUM_NVS = NODE_VARS; // Only one used (NV1 for volume)
  module_config.EE_EVENTS_START = 50;
  module_config.EE_MAX_EVENTS = 64;
  module_config.EE_NUM_EVS = GLOBAL_EVS + NUM_SOUND_EVENT_VARS;
  module_config.EE_BYTES_PER_EVENT = (module_config.EE_NUM_EVS + 4);

  // initialise and load configuration
  module_config.setEEPROMtype(EEPROM_INTERNAL);
  module_config.begin();

  DEBUG_SERIAL << F("> mode = ") << ((module_config.FLiM) ? "FLiM" : "SLiM") << F(", CANID = ") << module_config.CANID;
  DEBUG_SERIAL << F(", NN = ") << module_config.nodeNum << endl;

  // show code version and copyright notice
  printConfig();

  // set module parameters
  CBUSParams params(module_config);
  params.setVersion(VER_MAJ, VER_MIN, VER_BETA);
  params.setModuleId(MODULE_ID);
  params.setFlags(PF_FLiM | PF_COMBI);

  // assign to CBUS
  CBUS.setParams(params.getParams());
  CBUS.setName(mname);

  // register our CBUS event handler, to receive event messages of learned events
  CBUS.setEventHandler(eventhandler);

  // configure and start CAN bus and CBUS message processing
  CBUS.setNumBuffers(2,1);                  // more buffers = more memory used, fewer = less
  CBUS.setOscFreq(CAN_OSC_FREQ);          // select the crystal frequency of the CAN module
  CBUS.setPins(CAN_CS_PIN, CAN_INT_PIN);  // select pins for CAN bus CE and interrupt connections
  CBUS.begin();
}
//
///  setup Module - runs once at power on called from setup()
//

void setupModule() {
  // configure the module switches, active low
  for (int i = 0; i < NUM_SWITCHES; i++) {
    moduleSwitch[i].attach(SWITCH[i], INPUT_PULLUP);
    moduleSwitch[i].interval(5);
    switchState[i] = false;
  }

  // configure the module LEDs
  for (int i = 0; i < NUM_LEDS; i++) {
    moduleLED[i].setPin(LED[i]);
  }

  DEBUG_SERIAL << "> Module has " << NUM_LEDS << " LEDs and " << NUM_SWITCHES << " switches." << endl;
}

//
///  setupDCcontroller - runs once at power on called from setup()
//

void setupDCcontroller() {
    dcController = dc_controller();   // Instantiate dc controller instance
    dcController.initialise();        // Initialise it
}

void setup() 
{
  DEBUG_SERIAL.begin(DEBUG_SERIAL_PORT_SPEED);
  DEBUG_SERIAL << endl
         << endl
         << F("> ** CBUS Sound Player v3b ** ") << __FILE__ << endl;
  setupCBUS();
  setupModule();
  setupDCcontroller();

  // end of setup
  DEBUG_PRINT(F("> ready"));
}


void loop()
{
  // do CBUS message, switch and LED processing
  CBUS.process();

  // process console commands
  processSerialInput();

  // Run the LED code
  for (int i = 0; i < NUM_LEDS; i++) {
    moduleLED[i].run();
  }

  // test for switch input
  processSwitches();

  // Update DC Controller output
  dvController.tick();

  // Wait for 1ms
  delay_ms(1)
}

void processSwitches(void) {
  bool isSuccess = true;
  for (int i = 0; i < NUM_SWITCHES; i++) {
    moduleSwitch[i].update();
    if (moduleSwitch[i].changed()) {
       if (moduleSwitch[i].fell()) {
          if (SWITCH[i] == RENEG_BUTTON)
           {
              // renegotiate
              CBUS.renegotiate();
           }

          else if (SWITCH[i] == ERASE_BUTTON)
          {
            // Reset module, clear EEPROM
             DEBUG_SERIAL << F(">RESETTING AND WIPING EEPROM") << endl;
              module_config.resetModule();
          }
 
          else
          {
             // Serial << F("> unknown command ") << c << endl;
          }
       }
    }
  }
}

//
/// called from the CBUS library when a learned event is received
//
void eventhandler(byte index, CANFrame *msg) {
  byte opc = msg->data[0];

  DEBUG_PRINT(F("> event handler: index = ") << index << F(", opcode = 0x") << _HEX(msg->data[0]));
  DEBUG_PRINT(F("> event handler: length = ") << msg->len);

  unsigned int node_number = (msg->data[1] << 8) + msg->data[2];
  unsigned int event_number = (msg->data[3] << 8) + msg->data[4];
  
  DEBUG_PRINT(F("> NN = ") << node_number << F(", EN = ") << event_number);
  DEBUG_PRINT(F("> op_code = ") << opc);

  // read event variables
  // +1 as EVs are numbered from 1+GLOBAL_EVS
  byte eval_track_on = module_config.getEventEVval(index, (ONTRACK+1+GLOBAL_EVS));
  byte eval_track_off = module_config.getEventEVval(index, (OFFTRACK+1+GLOBAL_EVS));
  byte eval_volume = module_config.getEventEVval(index, (VOLUME+1+GLOBAL_EVS));

  // Adjust according to case
  switch (opc) {
    case OPC_ACON:
    case OPC_ASON:
      if (eval_track_on !=0) play(eval_track_on, eval_volume);
      break;

    case OPC_ACOF:
    case OPC_ASOF:
      if (eval_track_off !=0) play(eval_track_off, eval_volume);
      break;
      
    default:
      DEBUG_SERIAL << F("> OPC ignored ") << endl;
      break;
  }
}
