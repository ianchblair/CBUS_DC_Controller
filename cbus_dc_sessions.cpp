/// Session handling routine for CBUS DC Controller.
///
/// Forked from relevant CANCMD routines
/// 
/// From CANCMD2, MERG John Fletcher et al.
#include <arduino.h>

// CBUS library header files
#include <CBUSESP32.h>              // CAN controller and CBUS class
#include <CBUSswitch.h>             // pushbutton switch
#include <CBUSLED.h>                // CBUS LEDs
#include <CBUSconfig.h>             // module configuration
#include <CBUSParams.h>             // module parameters
#include <cbusdefs.h>               // MERG CBUS constants
#include "cbus_module_defs.h"
#include "dc_controller_defs.h"
#include "cbus_dc_messages.h"        // CBUS message functions
#include "cbus_dc_sessions.h"        // CBUS session functions
#include "dc_controller.h"
#include "throttle.h"


#if SET_INERTIA_RATE
#define INERTIA        3200       // Inertia counter value. Set High
#endif

#define startAddress 1000     // multiplier for DCC address offset from device address. 
// Device 0 uses 1000, device 1 uses 2000,...

bool cancmd_present;
byte deviceAddress = 0;
cbus_dc_messages _messenger;

cbus_dc_sessions::cbus_dc_sessions()
{
  ;
}

void cbus_dc_sessions::setup(void)
{
  // System Reset (Sent by CANCMD on power up)
  cancmd_present = true; // message came from CANCMD, so must be present

  for (int controllerIndex = 0; controllerIndex < NUM_CONTROLLERS; controllerIndex++)
  {
    // release all active controllers
    if (controllers[controllerIndex].session > SF_INACTIVE)
    {
      controllers[controllerIndex].trainController.emergencyStop(); // Emergency Stop
      controllers[controllerIndex].session = SF_INACTIVE;
      // update the speed display.
      //ib displaySpeed(controllerIndex);
    }
    // release all consists
    controllers[controllerIndex].consist = { 0, false, false };
  }
}

// ploc function
void cbus_dc_sessions::ploc(CANFrame *msg, long unsigned int dcc_address, int long_address)
{
  // only interested in the addresses of our analogue outputs
  for (int controllerIndex = 0; controllerIndex < NUM_CONTROLLERS; controllerIndex++)
  {
    if (controllers[controllerIndex].DCCAddress == dcc_address  && controllers[controllerIndex].longAddress == long_address)
    {
      int requestedSpeed = msg->data[4] & 0x7f;
      controllers[controllerIndex].session = msg->data[1];
      if (requestedSpeed == 1)
      {
        // emergency stop
        controllers[controllerIndex].trainController.emergencyStop();
      }
      else
      {
        controllers[controllerIndex].trainController.setSpeedAndDirection((msg->data[4] & 0x80) >> 3, msg->data[4] & 0x7f);
      }
    }
  }
}

// RESTP function
void cbus_dc_sessions::restp(void)
{
  for (int controllerIndex = 0; controllerIndex < NUM_CONTROLLERS; controllerIndex++)
  {
    // stop all active controllers
    if (controllers[controllerIndex].session != SF_INACTIVE)
    {
#if DEBUG
      Serial << F("Controller ") << controllerIndex << F(" active")<< endl;
#endif
      controllers[controllerIndex].trainController.emergencyStop ();
      // update the speed display.
      // IB displaySpeed(controllerIndex);
    }
    else
    {
#if DEBUG
      Serial << F("Controller ") << controllerIndex << F(" inactive")<< endl;
#endif
    }
  }
}

//IB session timeout funtion
// session timeout function  
// increment timeout counters every second

void cbus_dc_sessions::increment(void)
{
  for (byte controllerIndex = 0; controllerIndex < NUM_CONTROLLERS; controllerIndex++)
  {
    if (controllers[controllerIndex].session != SF_INACTIVE)
    {
      ++controllers[controllerIndex].timeout; // increment timeout counter by 1
    }
  }
}

// New routine for update processing which can be called as needed.
void cbus_dc_sessions::updateProcessing(bool updateNow)
{
  byte controllerIndex;
  // No CBus message received this time round the loop, so check the sessions for timeout
  for (controllerIndex = 0; controllerIndex < NUM_CONTROLLERS; controllerIndex++)
  {
    if ((controllers[controllerIndex].session != SF_INACTIVE) && (controllers[controllerIndex].timeout > MAXTIMEOUT))
    {
#if DEBUG
      Serial.print("Session ");
      Serial.print(controllers[controllerIndex].session);
      Serial.print(" Address ");
      Serial.print(controllers[controllerIndex].DCCAddress);
      Serial.println(" Timed Out.");
#endif
      controllers[controllerIndex].trainController.setSpeedAndDirection(0, 0);
      releaseLoco(controllers[controllerIndex].session);
      sendSessionError(controllers[controllerIndex].session, ErrorState::sessionCancelled); // Send session cancelled message out to CABs
    }

    if (updateNow)
    {
      controllers[controllerIndex].trainController.matchToTargets ();
        // update the speed display.
        // IB displaySpeed(controllerIndex);
    }
  }
} 

/* *******************************************************************************
 * functions and procedures
 * *******************************************************************************/
int cbus_dc_sessions::getSessionIndex (byte session)
{
  int controllerIndex;
  for (controllerIndex = 0; controllerIndex < NUM_CONTROLLERS; controllerIndex++)
  {
    if (controllers[controllerIndex].session == session)
    {
      return controllerIndex;
    }
  }
  // session not found, so
  return SF_INACTIVE;
}

int cbus_dc_sessions::getDCCIndex (unsigned int dcc_address, byte long_address)
{
  int controllerIndex;
  for (controllerIndex = 0; controllerIndex < NUM_CONTROLLERS; controllerIndex++)
  {
    if ((controllers[controllerIndex].DCCAddress == dcc_address) && (controllers[controllerIndex].longAddress == long_address))
    {
      return controllerIndex;
    }
  }
  // controller not found, for this DCC address so
  return SF_INACTIVE;
}

/*
 * A loco release command for the given session
 */
void cbus_dc_sessions::releaseLoco(byte session)
{
  int controllerIndex = getSessionIndex(session);
  if (controllerIndex >= 0)
  {
    controllers[controllerIndex].session = SF_INACTIVE;
    controllers[controllerIndex].timeout = 0;
#if DEBUG
  Serial.print("Session ");
  Serial.print(session);
  Serial.print(" Address ");
  Serial.print(controllers[controllerIndex].DCCAddress);
  Serial.println(" Released.");
#endif
    // update the speed display.
    // IB displaySpeed(controllerIndex);
  }
}

/*
 * A QLOC command for the given session from a CAB
 */
void cbus_dc_sessions::queryLoco(byte session)
{  
  int controllerIndex = getSessionIndex(session);
  // only respond if working standalone
  if (cancmd_present == false)
  {
#if DEBUG
    Serial.print("Query Loco Session ");
    Serial.println(session);
#endif
    if (controllerIndex >= 0)
    {
      // session found, send a PLOC
      sendPLOC(session);
    }
    else
    {
      // session not active, so send back error
      sendSessionError(session, ErrorState::noSession);
    }
  }
}

/*
 * The command station has allocated a session to a locomotive
 */
void cbus_dc_sessions::locoSession(byte session, unsigned int address, byte long_address, byte direction_, byte speed_)
{
  int controllerIndex = getDCCIndex (address, long_address);
   #if DEBUG
     Serial.print("locoSession ");
     Serial.print(session);
     if (long_address == 0)
     {
       Serial.print(" Short");
     }
     else
     {
       Serial.print(" Long");
     }
     Serial.print(" DCC address ");
     Serial.println(address);
   #endif
  if (controllerIndex >= 0)
  {
    controllers[controllerIndex].session = session;
    controllers[controllerIndex].trainController.setSpeedAndDirection(direction_, speed_);
    // update the speed display.
    // IB displaySpeed(controllerIndex);
  }
}

/*
 * Keep alive received, so reset the timeout counter
 */
void cbus_dc_sessions::keepaliveSession(byte session)
{
  int controllerIndex = getSessionIndex(session);
  if (controllerIndex >= 0)
  {
    controllers[controllerIndex].timeout = 0;
  }
}

/*
 * A throttle has requested access to a particular loco address
 * This routine is only used if there is no CANCMD on the bus that will
 * allocate sessions.
 */
void cbus_dc_sessions::locoRequest(unsigned int address, byte long_address, byte flags)
{
  int controllerIndex = getDCCIndex(address, long_address);
#if DEBUG
  Serial.println("locoRequest");
#endif
  // only respond if working standalone
  if (cancmd_present == false)
  {
#if DEBUG
    Serial.println("Standalone");
#endif
    if (controllerIndex >= 0)
    {
      // This is a DCC Address associated with our controllers
      if (controllers[controllerIndex].session != SF_INACTIVE)
      {
        // Loco is already used in a session
#if DEBUG
        Serial.print("Loco already allocated to session ");
        Serial.println(controllers[controllerIndex].session);
        Serial.print(F("Flag: "));
        Serial.println(flags);
#endif
        if (flags == 0)
          sendError(address, long_address, ErrorState::locoTaken);    // Send a Taken error
        else if (flags == 1)        // Steal
        {
          sendError(address, long_address, ErrorState::sessionCancelled);
          controllers[controllerIndex].session = SF_INACTIVE;
#if KEYPAD
          controllers[keyFSM.currentLoco].shared = false;
#endif
        }
        else if (flags == 2)        // Share
        {
          sendPLOC(controllers[controllerIndex].session);
#if KEYPAD
          controllers[keyFSM.currentLoco].shared = true;
#endif
        }
        else
          sendError(address, long_address, ErrorState::invalidRequest);
        return;
      }
    }
    else
    {
      // This DCC Address is not associated with any of our controllers
      sendError(address, long_address, ErrorState::invalidRequest);
      return;
    }
    // If we have got this far then the controller is not in use
    // set a new session number for the controller to use
   
    locoSession(controllerIndex, address, long_address, SF_FORWARDS, 0);
 #if DEBUG
        Serial.print("Session Allocated: ");
        Serial.println(controllers[controllerIndex].session);
 #endif
    sendPLOC(controllers[controllerIndex].session);
  }
  // Do nothing if there is a CANCMD present - it will assign sessions.
}

/*
* A throttle has requested access to a particular consist address
* This routine is only used if there is no CANCMD on the bus that will
* allocate sessions.
*/
void cbus_dc_sessions::consistRequest(unsigned int address)
{
  byte index;
#if DEBUG
  Serial.println(F("ConsistRequest"));
#endif
  // only respond if working standalone
  if (cancmd_present == false)
  {
#if DEBUG
    Serial.println(F("Standalone"));

#endif
    if ((address > 0) && address < 128)
    {
      //This is a DCC Address associated with our consists
      boolean found = false;
      byte index;

      for (index = 0; index < NUM_CONTROLLERS; index++)
      {
        if (controllers[index].consist.address == address)
        {
          found = true;
          break;
        }
      }

      if (found == true)
      {
        if (controllers[index].consist.session > 0)
        {
#if DEBUG
          Serial.print(F("Consist in use "));
          Serial.println(address);
#endif
          sendError(address, 0, ErrorState::locoTaken);
          return;
        }
      }
      else
      {
#if DEBUG
        Serial.print(F("Consist not found "));
        Serial.println(address);
#endif
        sendError(address, 0, ErrorState::consistEmpty);
        return;
      }
    }
    else
    {
      // This DCC Address is not associated with any of our consists
#if DEBUG
      Serial.print(F("Invalid consist address: "));
      Serial.println(address);
#endif
      sendError(address, 0, ErrorState::invalidRequest);
      return;
    }
    // If we have got this far then the consist is not in use.
    // Set a new session number for the consist - same as address with MSB to 1.
    // The session id is common across all CANCMDDC instances.
    controllers[index].consist.session = address | 0x80;
#if DEBUG
    Serial.print(F("Consist Session Allocated: "));
    Serial.println(address | 0x80);
#endif
    sendPLOCConsist(address);
  }
  // Do nothing if there is a CANCMD present - it will assign sessions.
}

/**
 * Send a PLOC message in response to a CAB requesting a session for
 * a DCC address
 */
void cbus_dc_sessions::sendPLOC(byte session)
{
  unsigned char buf[8];
  int controllerIndex = getSessionIndex(session);
  // only send this response if working standalone
  if (cancmd_present == false)
  {
   #if DEBUG
     Serial.print("Send PLOC ");
   #endif
    buf[0] = 0xE1; // OPC_PLOC
    buf[1] = session;
    buf[2] = ((controllers[controllerIndex].DCCAddress >> 8) & 0x3f) | (controllers[controllerIndex].longAddress);
    buf[3] = (controllers[controllerIndex].DCCAddress) & 0xff;
    buf[4] = controllers[controllerIndex].trainController.getSpeed() | (controllers[controllerIndex].trainController.getDirection() ? 0x80 : 0);
    buf[5] = 0;  // Zero function bytes
    buf[6] = 0;
    buf[7] = 0;
    _messenger.sendMessage(8, buf);
   #if DEBUG
          Serial.print("CAN msg: ");
          for(int i = 0; i<8; i++)                // Print each byte of the data
          {
            Serial.print(" ");
            if(buf[i] < 0x10)                     // If data byte is less than 0x10, add a leading zero
            {
              Serial.print("0");
            }
            Serial.print(buf[i], HEX);
          }
          Serial.println();
   #endif
  }
}


void cbus_dc_sessions::sendPLOCConsist(byte address)
{
  unsigned char buf[8];
  // only send this response if working standalone
  if (cancmd_present == false)
  {
    // only send this response if 1st device on bus - we don't want up to 8 identical messages sent
    if (deviceAddress == 0)
    {
#if DEBUG
      Serial.print(F("Send PLOC "));
#endif
        buf[0] = 0xE1; // OPC_PLOC
        buf[1] = address | 0x80;
        buf[2] = 0;
        buf[3] = address;
        buf[4] = 0;
        buf[5] = 0;  // Zero function bytes
        buf[6] = 0;
        buf[7] = 0;
        _messenger.sendMessage(8, buf);
#if DEBUG
      Serial.print(F("CAN msg: "));
      for (int i = 0; i < 8; i++)                // Print each byte of the data
      {
        Serial.print(F(" "));
        if (buf[i] < 0x10)                     // If data byte is less than 0x10, add a leading zero
        {
          Serial.print(F("0"));
        }
        Serial.print(buf[i], HEX);
      }
      Serial.println();
#endif
    }
  }
}

void cbus_dc_sessions::addSessionConsist(byte session, byte consist)
{
#if DEBUG
  Serial.print(F("Add to consist: "));
  Serial.println(consist);
#endif

  // does the session belong to this controller?
  byte index = getSessionIndex(session);

  if (index == SF_UNHANDLED)
    return;

  // is the session already in a consist?
  removeSessionConsist(session);

  // add the consist address to the loco
  // remove for now -
  //invalid narrowing conversion from "int" to "unsigned char"
  //controllers[index].consist = { (consist & 0x7f), 0, ((consist & 0x80) == 0x80) };
  controllers[index].consist.address = (consist & 0x7f);
  controllers[index].consist.session = 0;
  controllers[index].consist.reverse = ((consist & 0x80) == 0x80);
  //This works, although the other version does not with that compiler. Oh well.>>>>>>> 9889f22519fb9a2ede20893fbdcefad8d2ff6b54
}

void cbus_dc_sessions::removeSessionConsist(byte session)
{
#if DEBUG
  Serial.print(F("Remove from consist: "));
  Serial.println(session);
#endif

  for (byte i = 0; i < NUM_CONTROLLERS; i++)
  {
    if (controllers[i].consist.address == (session & 0x7f))
    {
      controllers[i].consist.session = 0;
    }
  }
}

void cbus_dc_sessions::setSpeedAndDirection(byte controllerIndex, byte requestedSpeed, byte reverse)
{
  if ((requestedSpeed & 0x7F) == 1)
  {
    // emergency stop
    controllers[controllerIndex].trainController.emergencyStop();
  }
  else
  {
#if DEBUG
    Serial << F("Setting speed to ") << (requestedSpeed & 0x7f) << " with reverse " << reverse << endl;
#endif
    // IB controllers[controllerIndex].trainController.emergencyStopOff();
    controllers[controllerIndex].trainController.setSpeedAndDirection(((requestedSpeed & 0x80) ^ reverse) >> 7, requestedSpeed & 0x7f);
  }
  // update the speed display.
  // IB displaySpeed(controllerIndex);
}
#if SET_INERTIA_RATE
void setInertiaRate(byte session, byte rate)
{
  
}
#else
void cbus_dc_sessions::setSpeedSteps(byte session, byte steps)
{
  // This is only relevent for DCC, so can be ignored
}
#endif

/**
 * Send an error packet labelled with the DCC address
 */
void cbus_dc_sessions::sendError(unsigned int address, byte long_address, ErrorState error_code)
{
  unsigned char buf[4];
  byte code = (byte)error_code;
#if DEBUG
  Serial.print("Send Loco ");
  Serial.print(address);
  Serial.print(" Error ");
  Serial.println(code);
  serialPrintErrorln(code);
#endif
  buf[0] = 0x63; // OPC_ERR
  buf[1] = ((address >> 8) & 0xff) | long_address;
  buf[2] = address & 0xff;
  buf[3] = code;
  _messenger.sendMessage(4, buf);
}

/**
 * Send a session error message to the CABs, labelled with the session number
 */
void cbus_dc_sessions::sendSessionError(byte session, ErrorState error_code)
{
  unsigned char buf[4];
  byte code = (byte)error_code;
#if DEBUG
  Serial.print("Send Session ");
  Serial.print(session);
  Serial.print(" Error ");
  Serial.println(code);
#endif
  buf[0] = 0x63; // OPC_ERR
  buf[1] = session;
  buf[2] = 0;
  buf[3] = code;
  _messenger.sendMessage(4, buf);
}

/**
 * Send a reset signal to all connected CABs
 */
void cbus_dc_sessions::sendReset()
{
  unsigned char buf[1];
  int i;
  buf[0] = 0x07; // OPC_ARST
  for (i=0; i< NUM_CONTROLLERS; i++)
  {
   _messenger.sendMessage(1, buf);
  }
  buf[0] = 0x03; // OPC_BON
  _messenger.sendMessage(1, buf);
}

void cbus_dc_sessions::emergencyStopAll()
{
  unsigned char buf[1];
  // Tell all the cabs
  buf[0] = 0x06; // ESTOP
  _messenger.sendMessage(1, buf);
  // IB beep_counter = 100; // sound buzzer 1 second
#if CBUS_EVENTS
  sendEvent(OPC_ACON,(byte)EventNo::stopEvent);
  stopEventOn = true;
#endif
}

/**
* Stop all DC tracks
* Loop over every session and if it is not free set
* the speed to 0 (stop) or 1 (eStop)
*/
void cbus_dc_sessions::stopAll(boolean emergency)
{
  for (byte controllerIndex = 0; controllerIndex < NUM_CONTROLLERS; controllerIndex++)
  {
    // stop all active controllers
    if (controllers[controllerIndex].session != SF_INACTIVE)
    {
      if (emergency)
        controllers[controllerIndex].trainController.emergencyStop();
      else
        controllers[controllerIndex].trainController.setSpeed(0);
      // update the speed display.
      // IB displaySpeed(controllerIndex);
      sendDSPD(controllerIndex);
    }
  }
#if CBUS_EVENTS
  sendEvent(OPC_ACON,(byte)EventNo::stopEvent);
  stopEventOn = true;
#endif
}

/*
* Send a DSPD message to CABs showing speed/direction
*/
void cbus_dc_sessions::sendDSPD(byte controllerIndex)
{

  unsigned char buf[3];

#if DEBUG
  Serial.print(F("Send DSPD "));
#endif
    buf[0] = 0x47; // OPC_DSPD
    buf[1] = controllers[controllerIndex].session;
    buf[2] = controllers[controllerIndex].trainController.getSpeed() | (controllers[controllerIndex].trainController.getDirection() * 0x80);
    _messenger.sendMessage(3, buf);
  //CAN0.sendMsgBuf(((unsigned long)canId.id) << 5, 3, buf);
#if DEBUG
  Serial.print(F("CAN msg: "));
  for (int i = 0; i < 3; i++)                // Print each byte of the data
  {
    Serial.print(F(" "));
    if (buf[i] < 0x10)                     // If data byte is less than 0x10, add a leading zero
    {
      Serial.print(F("0"));
    }
    Serial.print(buf[i], HEX);
  }
  Serial.println();
#endif

}

#if 0 // IB
// N beeps
void nBeeps (byte numBeeps,
       int durationMillis)
{
  for (int i = 0; i < numBeeps; i++)
  {
  analogWrite(MODULE_SOUNDER, 50);
  delay(durationMillis);
  digitalWrite(MODULE_SOUNDER, HIGH);
  delay(durationMillis);
  }
}



// set up fixed text on display
void setupDisplay()
{
  // we will display controllers in 2 columns on 3 lines (max 6 controllers)
#if OLED_DISPLAY || LCD_DISPLAY
  int line; // 0-2
  int column; // 0 or 1
  int x_pos;
  int y_pos;
  float barsize;
  
  // Clear the buffer.
#if OLED_DISPLAY
  float barsize;
  display.clearDisplay();
  display.setTextSize(1);
#else
  display.clear();
#endif

  for (int controllerIndex=0; controllerIndex < NUM_CONTROLLERS; controllerIndex++)
  {
    line = controllerIndex / 2;
    column = controllerIndex - (line * 2);
#if OLED_DISPLAY
    x_pos = column*67;
    y_pos = line*12;
#else
    x_pos = column * 10;
    y_pos = line;
#endif

    display.setCursor(x_pos, y_pos);
    display.print(controllers[controllerIndex].DCCAddress); // display DCC address
 //   if (init)
//    {
//#if OLED_DISPLAY
//      display.setCursor((x_pos)+30, y_pos);
//#else
//      display.setCursor((x_pos)+5, y_pos);
//#endif
//
//      display.write("Free");
//    }
   }

    display.display();

  if (!barGraphicsLoaded)
    setupBarGraph();

  showingSpeeds = true;

#endif
} 

#if OLED_DISPLAY || LCD_DISPLAY
void showSpeeds()
{
  if (!showingSpeeds)
    setupDisplay();

  for (byte controllerIndex = 0; controllerIndex < NUM_CONTROLLERS; controllerIndex++)
  {
    displaySpeed(controllerIndex);
  }

}
#endif

#endif // of IB