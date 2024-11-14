#include "cbus_dc_serial_interpreter.h"
#include "arduino.h"
#include "cbus_dc_messages.h"

CBUSConfig m_config;

void cbus_serial_setup(CBUSConfig params)
{
  m_config = params;
}


void printConfig(void) {

  // code version
  Serial << F("> code version = ") << VER_MAJ << VER_MIN << F(" beta ") << VER_BETA << endl;
  Serial << F("> compiled on ") << __DATE__ << F(" at ") << __TIME__ << F(", compiler ver = ") << __cplusplus << endl;
#if CANBUS8MHZ
  Serial << F("> Set for 8Mhz crystal") << endl;
#endif
  //Serial << F("> CS Pin ") << CHIPSELECT << endl;
  //Serial << F("> INT Pin ") << CBUSINTPIN << endl; 
// copyrights collected so far
  Serial << F("> © Mark Riddoch (MERG M1118) 2015") << endl;
  Serial << F("> © Ian Morgan (MERG M2775) 2019") << endl;
  Serial << F("> © David Radcliffe (MERG M3666) 2019") << endl;
  Serial << F("> © Duncan Greenwood (MERG M5767) 2019") << endl;
  Serial << F("> © John Fletcher (MERG M6777) 2019") << endl;
  Serial << F("> © Ian Blair (MERG M6893) 2024") << endl;
#if DEBUG
   byte error_code = (byte)ErrorState::noError;
   Serial << F("> Error code test noError: ") << error_code << endl;
   Serial << F("> Error code text is : ");
   serialPrintErrorln(error_code);
   Serial << F("> The number of controllers is: ") << NUM_CONTROLLERS << endl;
  byte controllerIndex;
  for (controllerIndex = 0; controllerIndex < NUM_CONTROLLERS; controllerIndex++)
  {
    Serial << F("> Controller ") << controllerIndex << F(" is ") << controllers[controllerIndex].DCCAddress << endl;
  }
  #if OLED_DISPLAY || LCD_DISPLAY
    #if OLED_DISPLAY
    Serial << F("> OLED display available") << endl;
    #else
    Serial << F("> LCD display available") << endl;
    #endif
  #endif
  #if CBUS_EVENTS
    Serial << F("> CBUS Events available") << endl;
  #endif
  #if LINKSPRITE
     Serial << F("> LINKSPRITE Output Pins") << endl;
     Serial << F("> pinI1 = ") << pinI1 << endl; //define I1 interface
     Serial << F("> pinI2 = ") << pinI2 << endl; //define I1 interface
     Serial << F("> speedpinA = ") << speedpinA << endl; //define I1 interface
//int pinI3=12;//define I3 interface 
//int pinI4=13;//define I4 interface 
//int speedpinB=10;//enable motor B
  
  #endif
  #endif
  return;
}

void processSerialInput(void) {

  byte uev = 0;
  char msgstr[32], dstr[32];

  if (Serial.available()) {

    char c = Serial.read();

    switch (c) {

      case 'n':

        // node config
        printConfig();

        // node identity
        Serial << F("> CBUS node configuration") << endl;
        Serial << F("> mode = ") << (m_config.FLiM ? "FLiM" : "SLiM") << F(", CANID = ") << m_config.CANID << F(", node number = ") << m_config.nodeNum << endl;
        Serial << endl;
        break;

      case 'e':

        // EEPROM learned event data table
        Serial << F("> stored events ") << endl;

        sprintf(msgstr, "  max events = %d, EVs per event = %d, bytes per event = %d", m_config.EE_MAX_EVENTS, m_config.EE_NUM_EVS, m_config.EE_BYTES_PER_EVENT);
        Serial << msgstr << endl;

        for (byte j = 0; j < m_config.EE_MAX_EVENTS; j++) {
          if (m_config.getEvTableEntry(j) != 0) {
            ++uev;
          }
        }

        Serial << F("  stored events = ") << uev << F(", free = ") << (m_config.EE_MAX_EVENTS - uev) << endl;
        Serial << F("  using ") << (uev * m_config.EE_BYTES_PER_EVENT) << F(" of ") << (m_config.EE_MAX_EVENTS * m_config.EE_BYTES_PER_EVENT) << F(" bytes") << endl << endl;

        Serial << F("  Ev#  |  NNhi |  NNlo |  ENhi |  ENlo | ");

        for (byte j = 0; j < (m_config.EE_NUM_EVS); j++) {
          sprintf(dstr, "EV%03d | ", j + 1);
          Serial << dstr;
        }

        Serial << F("Hash |") << endl;

        Serial << F(" --------------------------------------------------------------") << endl;

        // for each event data line
        for (byte j = 0; j < m_config.EE_MAX_EVENTS; j++) {

          if (m_config.getEvTableEntry(j) != 0) {
            sprintf(dstr, "  %03d  | ", j);
            Serial << dstr;

            // for each data byte of this event
            for (byte e = 0; e < (m_config.EE_NUM_EVS + 4); e++) {
              sprintf(dstr, " 0x%02hx | ", m_config.readEEPROM(m_config.EE_EVENTS_START + (j * m_config.EE_BYTES_PER_EVENT) + e));
              Serial << dstr;
            }

            sprintf(dstr, "%4d |", m_config.getEvTableEntry(j));
            Serial << dstr << endl;
          }
        }

        Serial << endl;

        break;

      // NVs
      case 'v':

        // note NVs number from 1, not 0
        Serial << "> Node variables" << endl;
        Serial << F("   NV   Val") << endl;
        Serial << F("  --------------------") << endl;

        for (byte j = 1; j <= m_config.EE_NUM_NVS; j++) {
          sprintf(msgstr, " - %02d : %3hd | 0x%02hx", j, m_config.readNV(j), m_config.readNV(j));
          Serial << msgstr << endl;
        }

        Serial << endl << endl;

        break;

      // CAN bus status
      case 'c':

        //CBUS.printStatus();
        break;

      case 'h':
        // event hash table
        m_config.printEvHashTable(false);
        break;

      case 'y':
        // reset CAN bus and CBUS message processing
        //CBUS.reset();
        break;

      case '*':
        // reboot
        m_config.reboot();
        break;

      case 'm':
        // free memory
        Serial << F("> free SRAM = ") << m_config.freeSRAM() << F(" bytes") << endl;
        break;

      case 'r':
        // renegotiate
        //CBUS.renegotiate();
        break;

      case 'z':
        // Reset module, clear EEPROM
        static bool ResetRq = false;
        static unsigned long ResWaitTime;
        if (!ResetRq) {
          // start timeout timer
          Serial << F(">Reset & EEPROM wipe requested. Press 'z' again within 2 secs to confirm") << endl;
          ResWaitTime = millis();
          ResetRq = true;
        }
        else {
          // This is a confirmed request
          // 2 sec timeout
          if (ResetRq && ((millis() - ResWaitTime) > 2000)) {
            Serial << F(">timeout expired, reset not performed") << endl;
            ResetRq = false;
          }
          else {
            //Request confirmed within timeout
            Serial << F(">RESETTING AND WIPING EEPROM") << endl;
            m_config.resetModule();
            ResetRq = false;
          }
        }
        break;

      case '\r':
      case '\n':
        Serial << endl;
        break;

      default:
        // Serial << F("> unknown command ") << c << endl;
        break;
    }
  }
}