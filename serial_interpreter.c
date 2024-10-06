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

void processSerialInput(void) {
  char msgstr[32];
  byte uev;

  if (DEBUG_SERIAL.available()) {
    char c = DEBUG_SERIAL.read();

    switch (c) {

      case 'n':
        // node config
        printConfig();

        // node identity
        DEBUG_SERIAL << F("> CBUS node configuration") << endl;
        DEBUG_SERIAL << F("> mode = ") << (module_config.FLiM ? "FLiM" : "SLiM") << F(", CANID = ") << module_config.CANID << F(", node number = ") << module_config.nodeNum << endl;
        DEBUG_SERIAL << endl;
        break;

      case 'e':
        // EEPROM learned event data table
        DEBUG_SERIAL << F("> stored events ") << endl;
        DEBUG_SERIAL << F("  max events = ") << module_config.EE_MAX_EVENTS << F(" EVs per event = ") << module_config.EE_NUM_EVS << F(" bytes per event = ") << module_config.EE_BYTES_PER_EVENT << endl;

        for (byte j = 0; j < module_config.EE_MAX_EVENTS; j++) {
          if (module_config.getEvTableEntry(j) != 0) {
            ++uev;
          }
        }

        DEBUG_SERIAL << F("  stored events = ") << uev << F(", free = ") << (module_config.EE_MAX_EVENTS - uev) << endl;
        DEBUG_SERIAL << F("  using ") << (uev * module_config.EE_BYTES_PER_EVENT) << F(" of ") << (module_config.EE_MAX_EVENTS * module_config.EE_BYTES_PER_EVENT) << F(" bytes") << endl
               << endl;

        DEBUG_SERIAL << F("  Number of events ") << (module_config.EE_NUM_EVS)  << endl;
        DEBUG_SERIAL << F("  Ev#  |  NNhi |  NNlo |  ENhi |  ENlo | ");

        for (byte j = 0; j < (module_config.EE_NUM_EVS); j++) {
          sprintf(msgstr, "EV%03d | ", j + 1);
          DEBUG_SERIAL << msgstr;
        }

        DEBUG_SERIAL << F("Hash |") << endl;

        DEBUG_SERIAL << F(" ----------------------------------------------------------------------") << endl;

        // for each event data line
        for (byte j = 0; j < module_config.EE_MAX_EVENTS; j++) {
          if (module_config.getEvTableEntry(j) != 0) {
            sprintf(msgstr, "  %03d  | ", j);
            DEBUG_SERIAL << msgstr;

            // for each data byte of this event
            for (byte e = 0; e < (module_config.EE_NUM_EVS + 4); e++) {
              sprintf(msgstr, " 0x%02hx | ", module_config.readEEPROM(module_config.EE_EVENTS_START + (j * module_config.EE_BYTES_PER_EVENT) + e));
              DEBUG_SERIAL << msgstr;
            }

            sprintf(msgstr, "%4d |", module_config.getEvTableEntry(j));
            DEBUG_SERIAL << msgstr << endl;
          }
        }

        DEBUG_SERIAL << endl;

        break;

      // NVs
      case 'v':
        // note NVs number from 1, not 0
        DEBUG_SERIAL << "> Node variables" << endl;
        DEBUG_SERIAL << F("   NV   Val") << endl;
        DEBUG_SERIAL << F("  --------------------") << endl;

        for (byte j = 1; j <= module_config.EE_NUM_NVS; j++) {
          byte v = module_config.readNV(j);
          sprintf(msgstr, " - %02d : %3hd | 0x%02hx", j, v, v);
          DEBUG_SERIAL << msgstr << endl;
        }

        DEBUG_SERIAL << endl
               << endl;

        break;

      // CAN bus status
      case 'c':
        DEBUG_SERIAL << F("> CAN bus status requested") << endl;
        CBUS.printStatus();
        break;

      case 'h':
        // event hash table
        DEBUG_SERIAL << F("> Hash table requested") << endl;
        module_config.printEvHashTable(false);
        break;

      case 'y':
        // reset CAN bus and CBUS message processing
        DEBUG_SERIAL << F("> CAN bus reset requested") << endl;
        CBUS.reset();
        break;

      case '*':
        // reboot
        module_config.reboot();
        break;

      case 'm':
        // free memory
        DEBUG_SERIAL << F("> free SRAM = ") << module_config.freeSRAM() << F(" bytes") << endl;
        break;

      case 'r':
        // renegotiate
        DEBUG_SERIAL << F("> Renegotiation requested") << endl;
        CBUS.renegotiate();
        break;

      case 'z':
        // Reset module, clear EEPROM
        static bool ResetRq = false;
        static unsigned long ResWaitTime;
        if (!ResetRq) {
          // start timeout timer
          DEBUG_SERIAL << F(">Reset & EEPROM wipe requested. Press 'z' again within 2 secs to confirm") << endl;
          ResWaitTime = millis();
          ResetRq = true;
        } else {
          // This is a confirmed request
          // 2 sec timeout
          if (ResetRq && ((millis() - ResWaitTime) > 2000)) {
            DEBUG_SERIAL << F(">timeout expired, reset not performed") << endl;
            ResetRq = false;
          } else {
            //Request confirmed within timeout
            DEBUG_SERIAL << F(">RESETTING AND WIPING EEPROM") << endl;
            module_config.resetModule();
            ResetRq = false;
          }
        }
        break;
      case 't':
        // test sound card
        play(1,DEFAULT_VOLUME);
        DEBUG_SERIAL << F(">Player track 1 volume 20 (default) test performed") << endl;
        break;

      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
      case '9':
      // Alternative tests for specific tracks
      play(int(c-'0'),DEFAULT_VOLUME);
      break;
      
      default:
        // Serial << F("> unknown command ") << c << endl;
        break;
    }
  }
}

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

void setupSound()
{
  //Setup Sound Card
  //get base volume from NVs - only the first NV is used (numbered from 1, not 0)
   sound_volume - module_config.readNV(1);
   if (sound_volume == 0) sound_volume = DEFAULT_VOLUME;  //NV not set so give it a default of 20 in a scale of 0-30
   
   USER_SERIAL.begin (USER_SERIAL_PORT_SPEED);  //User port used for the sound card
//   if (!(SoundPlayer1.begin(USER_SERIAL,true,false)))   
   if (!(SoundPlayer1.begin(USER_SERIAL,false,false)))
   {
    DEBUG_SERIAL.println(F("Unable to begin:"));
    DEBUG_SERIAL.println(F("1.Please recheck the connection!"));
    DEBUG_SERIAL.println(F("2.Please insert the SD card!"));
   }
   //reset sound player and set volume
   SoundPlayer1.reset();
   SoundPlayer1.volume(sound_volume);
   //Serial.print("Volume complete"); 
}

void setup() 
{
  DEBUG_SERIAL.begin(DEBUG_SERIAL_PORT_SPEED);
  DEBUG_SERIAL << endl
         << endl
         << F("> ** CBUS Sound Player v3b ** ") << __FILE__ << endl;
  setupCBUS();
  setupModule();
  setupSound();

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
class dc_controller_mymodule(cbusmodule.cbusmodule):
    def __init__(self):
        super().__init__()

        
        
    def set_throttle(self,forward_not_backwards):
        # Set both outputs to zero before setting/swapping
        self.throttle0.write_output(0)
        self.throttle1.write_output(0)
        # The set throttles according to direction
        # Output throttle will drive trains
        # Return throttle will remain at zero for return rail. 
        if (forward_not_backwards == True):
            self.output_throttle = self.throttle0
            self.return_throttle = self.throttle1
        else:
            self.output_throttle = self.throttle1
            self.return_throttle = self.throttle0
    # delete stored bemf_level
        last_bemf=0
        
    # Filter calculates instantaneous output value based on mode, and phase
    def filter_calc(self, mode, phase, throttle_level):
        # Offset the DC according to the throttle vale
        dc_offset=int(throttle_level*defs.MAX_OP_LEVEL/defs.MAX_THROTTLE_LEVEL)
        switching_phase = int(throttle_level*defs.MAX_PHASE/defs.MAX_THROTTLE_LEVEL)
        if (mode == defs.MODE_DIRECT):
            return_value = dc_offset
        elif ((mode == defs.MODE_TRIANGLE) or mode == (defs.MODE_TRIANGLE_BEMF)):
            if (phase < switching_phase):
                triangle_value = min(int((phase*defs.MAX_OP_LEVEL)/defs.MAX_PHASE),defs.MAX_OP_LEVEL)
            else:
                height = int((switching_phase*defs.MAX_OP_LEVEL)/defs.MAX_PHASE)
                triangle_value = min(height + int(((switching_phase-phase)*defs.MAX_OP_LEVEL)/defs.MAX_PHASE), defs.MAX_OP_LEVEL)
            if (triangle_value < 0):
                triangle_value = 0
            # Only use triangle wave for outputs below 50%
            if (dc_offset<defs.MAX_OP_LEVEL/2):    
                return_value = int(triangle_value*(defs.MAX_OP_LEVEL-(2*dc_offset))/defs.MAX_OP_LEVEL) + dc_offset
            else:
                return_value = dc_offset

        # Extra check to imit output to range of DAC, where offset puts it out of range
        if (return_value < 0):
            return_value = 0
        if (return_value > defs.MAX_OP_LEVEL):
            return_value = defs.MAX_OP_LEVEL
            
        return(return_value)
        
    # This calculates the overall throttle level based on the pot setting bemf measurement and selected mode    
    def calculate_throttle(self, mode, requested_speed, bemf_speed):
        # By default or if mode is direct, output = input
        # Input levels arefrom ADCs, 0..4095 range
        # (level matching TBA)
        output_level=requested_speed
        if ((mode == defs.MODE_TRIANGLE) and (bemf_speed < defs.MAX_BEMF_LEVEL)):
            ## This to be revisited later:-
            ## Martin Da Costa ignores 40 percent limit
            ## but applies feedback in inverse proportion to requested speed instead
            ##if ((requested_speed>=defs.FORTY_PERCENT_THROTTLE)or(bemf_speed>defs.MAX_BEMF_LEVEL)):
            ##    self.last_bemf=bemf_speed
            ##else:
            error_correction = ((self.last_bemf+bemf_speed)*defs.ERROR_SCALE)
            error_level = requested_speed-error_correction
            scaled_error_level = int(error_level*(defs.MAX_THROTTLE_LEVEL-requested_speed)/defs.MAX_THROTTLE_LEVEL)
            if(error_level>=0):
                if((requested_speed + scaled_error_level)>defs.MAX_THROTTLE_LEVEL):
                    output_level = defs.MAX_THROTTLE_LEVEL
                elif((requested_speed + scaled_error_level)>0):
                    output_level = requested_speed + scaled_error_level
                else:
                    output_level=requested_speed
        # save bemf measuremant for next cycle
        self.last_bemf=bemf_speed
        # return calculated throttle value
        return(output_level)

    def initialise(self):
        
        # ***
        # *** CBUS part, bare minimum module init
        # *** Can be omitted for standalone use, TBD
        # ***

        self.cbus = cbus.cbus(
            mcp2515.mcp2515(),
            cbusconfig.cbusconfig(storage_type=cbusconfig.CONFIG_TYPE_FILES),
        )

        # ** change the module name and ID if desired

        self.module_id = 103
        self.module_name = bytes('PYCO   ', 'ascii')
        self.module_params = [
            20,
            cbusdefs.MANU_MERG,
            0,
            self.module_id,
            self.cbus.config.num_events,
            self.cbus.config.num_evs,
            self.cbus.config.num_nvs,
            1,
            7,
            0,
            cbusdefs.PB_CAN,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ]

        # change the CBUS switch and LED pin numbers if desired

        self.cbus.set_leds(21, 20)
        self.cbus.set_switch(22)
        self.cbus.set_name(self.module_name)
        self.cbus.set_params(self.module_params)
        self.cbus.set_event_handler(self.event_handler)
        self.cbus.set_received_message_handler(self.received_message_handler)
        self.cbus.set_sent_message_handler(self.sent_message_handler)

        self.cbus.begin()

        # ***
        # ***
        # Controller part
        # instantiate pins
        t0dac = DAC(pindefs.PIN_DAC0)
        t1dac = DAC(pindefs.PIN_DAC1)
        bemf0adc = ADC(pindefs.PIN_BEMF0,atten=ADC.ATTN_11DB)
        bemf1adc = ADC(pindefs.PIN_BEMF1,atten=ADC.ATTN_11DB)
        self._potadc = ADC(pindefs.PIN_POT,atten=ADC.ATTN_11DB)
        blnk0pin = Pin(pindefs.PIN_BLNK0, Pin.OUT)
        blnk1pin = Pin(pindefs.PIN_BLNK1, Pin.OUT)
        self._dirpin = Pin(pindefs.PIN_DIR, Pin.IN, Pin.PULL_UP)

        
        # Instantiate throttles, using instantiated pins, one for each output
        self.throttle0 = throttle.throttle(t0dac, bemf0adc, blnk0pin)
        self.throttle1 = throttle.throttle(t1dac, bemf1adc, blnk1pin)        
        
        direction = self._dirpin.value()
        self.set_throttle(direction)
        self.last_direction = direction
        self._timer_synchronisation=asyncio.ThreadSafeFlag()

        
    # ***
    # *** coroutines that run in parallel
    # ***
    # *** Throttles coro
    async def throttles_coro(self) -> None:
#        self.logger.log('throttles_coro start')

        while True:
            requested_level = self._potadc.read_u16()
            throttle_value=0
            start_level=0 # exact value of start_level is TBD
            # only act on direction switch when requested_level is below minimum threshold
            if (requested_level<defs.MIN_REQUESTED_LEVEL):
                direction = self._dirpin.value()
            else:
                direction = self.last_direction
            blanking_enabled = False
                
            if (direction == self.last_direction):
                
                # Main waveform loop
                # Performs various actions according to phase value
               for phase in range(defs.MAX_PHASE):
                    
                    await self._timer_synchronisation.wait()
                    # Perform required actions on particular phases
                    # Start all cycles with blanking off on both throttles
                    if (phase == 0):
                       self.output_throttle.clear_blanking()
                       
                    elif (phase == defs.POT_PHASE):
                        requested_level = self._potadc.read()
                        
                    elif (phase == defs.BLANK_PHASE):
                       self.output_throttle.set_blanking()
                       blanking_enabled = True
                       
                    # At end of each cycle recalculate throttle values
                    elif (phase == defs.LAST_PHASE):
                        if (blanking_enabled):
                           bemf_level= self.output_throttle.read_bemf()
                           #bemf_level= 0
                           blanking_enabled = False
                        throttle_value = self.calculate_throttle(defs.MODE_TRIANGLE,requested_level,bemf_level)
                        #throttle_value=requested_level
                
                    
                    # Regardless of above actions set output value
                    # Note that output will only be seen when blanking is not enabled.
                    #output_sample=self.filter_calc(defs.MODE_TRIANGLE,phase,1000)
                    output_sample=self.filter_calc(defs.MODE_TRIANGLE,phase,throttle_value)
                    self.output_throttle.write_output(output_sample)
                    #self.output_throttle.write_output(requested_level)
                    # Then wait for next loop
                    # For testing - for normal operation replaced by separate async task
                    #await asyncio.sleep_us(100)
            else:
                # Otherwise reverse direction
                self.last_direction = direction                
                self.set_throttle(direction)
                # Reset blanking
                self.output_throttle.clear_blanking()
                self.return_throttle.clear_blanking() 
    
                

     # *** user module application task - like Arduino loop()
    async def module_main_loop_coro(self) -> None:
#        self.logger.log('main loop coroutine start')
        while True:
            await asyncio.sleep_ms(1)
            self._timer_synchronisation.set()

    # ***
    # *** module main entry point - like Arduino setup()
    # ***

    async def run(self) -> None:
        # self.logger.log('run start')

        # start coroutines
        self.tb = asyncio.create_task(self.throttle_coro())
        self.tm = asyncio.create_task(self.module_main_loop_coro())

        repl = asyncio.create_task(aiorepl.task(globals()))

#        self.logger.log('module startup complete')
        await asyncio.gather(repl)


# create the module object and run it
mod = dc_controller_mymodule()
mod.initialise()
asyncio.run(mod.run())

# the asyncio scheduler is now in control
# no code after this line is executed

print('*** application has ended ***')
