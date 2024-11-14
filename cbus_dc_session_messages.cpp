/// Session Messages handling class for CBUS DC Controller.
/// Inherits from both Messages class and Sessions class
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
#include "cbus_dc_sessions.h"        // CBUS message functions
#include "cbus_dc_messages.h"        // CBUS message functions
#include "dc_controller.h"
#include "throttle.h"

CBUSConfig mod_config;
CBUSESP32 _CBUS; // CBUS Object

//bool cancmd_present = false;
//volatile byte timer_counter = 0;
//volatile byte flash_counter = 0;
//volatile byte beep_counter = 0;
//volatile byte update_counter = 0;
//volatile boolean updateNow = false;
//volatile boolean shutdownFlag = false;


void message_setup(CBUSConfig params)
{
  mod_config = params;
}

/// This replaces the CAN0.SendMsgBuff usage.
/// It uses the CANID of the current configuration.
//bool cbus_dc_messages::sendMessage(byte len, const byte *buf)
bool sendMessage(byte len, const byte *buf)
{
    CANFrame msg;
    msg.id = mod_config.CANID;
    msg.len = len;
    for(byte i = 0;  i< len; i++)
    {
       msg.data[i] = buf[i];
    }
    msg.ext = false;
    msg.rtr = false;
  
    bool res = _CBUS.sendMessage(&msg);
#if DEBUG
    if (res) {
      Serial << F("> sent CBUS message with code [ 0x") << _HEX(buf[0]) << F(" ] and size ") << len << endl;
      if (len > 1) {
        Serial << F("> Data : [");
        for(byte i = 1;  i< len; i++)
         {
            Serial << F(" 0x") << buf[i];
         }
        Serial << F(" ]") << endl;
      }
    } else {
      Serial << F("> error sending CBUS message") << endl;
    }
#endif
    return res;
 }
//
// Execution routines to be added.
//
/**

/*
* Send a DSPD message to CABs showing speed/direction
*/
void sendDSPD(byte controllerIndex)
{

  unsigned char buf[3];

#if DEBUG
  Serial.print(F("Send DSPD "));
#endif
    buf[0] = 0x47; // OPC_DSPD
    buf[1] = controllers[controllerIndex].session;
    buf[2] = controllers[controllerIndex].trainController.getSpeed() | (controllers[controllerIndex].trainController.getDirection() * 0x80);
    sendMessage(3, buf);
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
  float barsize;bool
  
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
* Stop all DC tracks
* Loop over every session and if it is not free set
* the speed to 0 (stop) or 1 (eStop)
*/
void stopAll(boolean emergency)
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
//
/// user-defined frame processing function
/// called from the CBUS library for *every* CAN frame received
/// it receives a pointer to the received CAN frame
//  Added from new version of CBUS_empty

void messagehandler(CANFrame *msg){

  int id;
  long unsigned int dcc_address;
  int long_address;
  byte controllerIndex;
  int lastInBufPtr = 0;
  int lastOutBufPtr = 0;
  //messageRecordType nextMessage;  //Defined in FIFO.h which may not be used.
/*
#if OLED_DISPLAY
  if (MessageBuffer.bufferOverflow == 1)
  {
    display.setCursor(64, 24);
    display.println("Overflow!");
    MessageBuffer.bufferOverflow = 0;
  }
#endif
*/

    //nextMessage = *msg;  // I think this should work for now.
    // This simple thing to do is to avoid nextMessage and get things direct from msg
    //nextMessage = fifoMessageBuffer.getMessage();

    if(msg->len > 0)            // Check to see whether data is received.
    {
 #if DEBUG
          Serial.print("CAN msg: ID ");
          Serial.print(msg->id, HEX);
          Serial.print(" OpCode:");
          for(int i = 0; i< msg->len; i++)                // Print each byte of the data
          {
            if (i == 1)
            {
              Serial.print(" Data:");
            }
            Serial.print(" ");
            if(msg->data[i] < 0x10)                     // If data byte is less than 0x10, add a leading zero
            {
              Serial.print("0");
            }
            Serial.print(msg->data[i], HEX);
          }
          Serial.println();
#endif

      // Perform the action for the received message
      switch (msg->data[0])
      {

        case OPC_ARST:                              // System Reset (Sent by CANCMD on power up)
#if DEBUG
          Serial.println(F("System Reset (Sent by CANCMD on power up)"));
#endif

          //IB TBA sesions_reset();
          // -------------------------------------------------------------------
        case OPC_RTOF:
#if DEBUG
          Serial.println(F("RTOFF - Request Track Off"));
#endif
          stopAll(true);
          break;

          // -------------------------------------------------------------------
        case OPC_RTON:
#if DEBUG
          Serial.println(F("RTON - Request Track On"));
#endif
          break;
         
        // -------------------------------------------------------------------
        case OPC_KLOC:                              // Release loco command
#if DEBUG
          Serial.println(F("REL - Release loco"));
#endif
          releaseLoco(msg->data[1]);
          break;

        // -------------------------------------------------------------------
        case OPC_QLOC:                              // Query loco command
#if DEBUG
          Serial.println(F("QLOC - Query loco"));
#endif
          queryLoco(msg->data[1]);
          break;

        // -------------------------------------------------------------------
        case OPC_DKEEP:                              // CAB Session keep alive command
#if DEBUG
          Serial.println(F("DKEEP - keep alive"));
#endif
          sessions_updateProcessing(true);

          keepaliveSession(msg->data[1]);
          break;
          
       // -------------------------------------------------------------------
       case OPC_RLOC:                              // Request loco session (0x40)
#if DEBUG
          Serial.println(F("RLOC - Request loco session"));
#endif
           dcc_address = msg->data[2] + ((msg->data[1] & 0x3f) << 8);
           long_address = (msg->data[1] & SF_LONG);
 #if DEBUG
           Serial.print("Req Sess. ");
           Serial.print(F(" Addr: "));
           Serial.println(dcc_address);
          if (long_address == 0)
           {
             Serial.print("Short");
           }
           else
           {
             Serial.print("Long");
           }
           Serial.print(" Addr: ");
           Serial.println(dcc_address);
 #endif
           if (getDCCIndex (dcc_address, long_address) != SF_UNHANDLED)
           {
#if DEBUG
             Serial << F("Calling locoRequest (") << dcc_address << F(",") << long_address << F(")")<< endl;
#endif
             locoRequest(dcc_address, long_address, 0);
           } else {
#if DEBUG
             Serial << F("Calling locoRequest not called for (") << dcc_address << F(",") << long_address << F(")")<< endl;
#endif
           }
           break;
           
          // -------------------------------------------------------------------
        case OPC_QCON:
#if DEBUG
          Serial.println(F("Query Consist......."));
#endif
          break;

        // -------------------------------------------------------------------
        case OPC_STMOD:                              // Set Speed Step Range

#if SET_INERTIA_RATE
#if DEBUG
          Serial.println(F("STMOD - Set Inertia Rate"));
#endif
          setInertiaRate(msg->data[1],msg->data[2]);
#else
#if DEBUG
          Serial.println(F("STMOD - Set speed steps"));
#endif
          setSpeedSteps(msg->data[1],msg->data[2]);
#endif
          break;
          
          // -------------------------------------------------------------------
        case OPC_PCON:
#if DEBUG
          Serial.println(F("PCON - Put loco in Consist"));
#endif
          addSessionConsist(msg->data[1], msg->data[2]);
          break;

          // -------------------------------------------------------------------
        case OPC_KCON:
#if DEBUG
          Serial.println(F("KCON - Remove loco from Consist"));
#endif
          removeSessionConsist(msg->data[1]);
          break;

       // -------------------------------------------------------------------
        case OPC_DSPD:                              // Session speed and direction
#if DEBUG
          Serial.println(F("DSPD - Set speed & direction"));
#endif
        {
          byte session = msg->data[1];
          byte requestedSpeed = msg->data[2]; // & 0x7f;
          //byte requestedDirection = msg->data[2] & 0x80;
          controllerIndex = getSessionIndex(session);
          if (controllerIndex > SF_INACTIVE)
          {
            /*if (requestedSpeed == 1)
            {
              // emergency stop
              controllers[controllerIndex].trainController.emergencyStop();
            }
            else */
            //{
               setSpeedAndDirection(controllerIndex, requestedSpeed, 0);
            // controllers[controllerIndex].trainController.setSpeedAndDirection(msg->data[2] & 0x80, requestedSpeed);
            //}
            // update the speed display now done in setSpeedAndDirection
            //displaySpeed(controllerIndex);
          }
          // update processing and reset the timeout
          sessions_updateProcessing(true);
          keepaliveSession(session);

          break;
        }         
       // -------------------------------------------------------------------
        case OPC_GLOC:                              // Request Steal or Share loco session (GLOC)
           dcc_address = msg->data[2] + ((msg->data[1] & 0x3f) << 8);
           long_address = (msg->data[1] & SF_LONG);
           locoRequest(dcc_address, long_address, msg->data[3]);
           break;
          
        // -------------------------------------------------------------------
        case OPC_PLOC:                              // PLOC session Allocate from CANCMD
           cancmd_present = true; // message came from CANCMD, so must be present
           dcc_address = msg->data[3] + ((msg->data[2] & 0x3f) << 8);
           long_address = (msg->data[2] & SF_LONG);
 #if DEBUG
           Serial.print("PLOC from CANCMD. ");
           if (long_address == 0)
           {
             Serial.print("Short");
           }
           else
           {
             Serial.print("Long");
           }
           Serial.print(" Addr: ");
           Serial.println(dcc_address);
 #endif
          //IBPLOC function
          break;
          
        // -------------------------------------------------------------------
        case OPC_RESTP:                              // Emergency stop all OPC_RESTP
        // RESTP function
// sessions_restp()
          // Tell all the CABs and Throttles
          emergencyStopAll();
          break;
 
        // -------------------------------------------------------------------
        case OPC_ACOF:
        case OPC_ACON:
#if DEBUG
           Serial << F("Message handled with Opcode [ 0x") << _HEX(msg->data[0]) << F(" ]")<< endl;
           Serial << F("Test code to see if a message is getting sent") << endl;
#endif
#if ACCESSORY_REQUEST_EVENT
#if USE_SHORT_EVENTS
      {
         // Local variable definition needs to be in { } 
         unsigned int device_number = 513;
         Serial << F("Send request short event with device number ") << device_number << endl;
         sendEvent(OPC_ASRQ,device_number); // Test of short event request.
      }
#else
    //if (moduleSwitch.isPressed() ) { // Send when button is pressed.
      Serial << F("Send request long event") << endl;
      sendEvent(OPC_AREQ,requestEvent); // Test of long event request.
    //}
#endif
#endif

          // All this is here for testing purposes
          // Tell all the CABs and Throttles
          //emergencyStopAll();
          //queryLoco(1);
          //testMessage(1);
          //locoRequest(1,0,0);
          //locoRequest(1001,SF_LONG,0);
        break;
        // -------------------------------------------------------------------
        case OPC_AROF:
        case OPC_ARON:
        case OPC_ARSOF:
        case OPC_ARSON:
        // Process responses to AREQ and ASRQ messages.
        // I may want to set this up to handle only certain device nos.
        // It will also be possible to send data or events based on this information.
        {
        byte local_opcode = msg->data[0];
#if ACCESSORY_REQUEST_EVENT
         unsigned int node_number = (msg->data[1] << 8 ) + msg->data[2];
         unsigned int event_number = (msg->data[3] << 8 ) + msg->data[4];
#if USE_SHORT_EVENTS
        if (local_opcode == OPC_ARSON) {
          Serial << F(" ON message from device ") << event_number << endl;
        } else if (local_opcode == OPC_ARSOF) {
          Serial << F(" OFF message from device ") << event_number << endl;
        }
#endif
        if (local_opcode == OPC_ARON) {
          Serial << F(" ON message from event ") << node_number << F(" event ") << event_number << endl;
        } else if (local_opcode == OPC_AROF) {
          Serial << F(" OFF message from node ") << node_number << F(" event ") << event_number << endl;
        }
#endif   
        }     
        break;
        // -------------------------------------------------------------------
        default:
          // ignore any other CBus messages
#if DEBUG
           Serial << F("Message handled with Opcode [ 0x") << _HEX(msg->data[0]) << F(" ]")<< endl;
#endif
          break;
      }
    }
}
#if 0
// Converted to get an opcode array now done when calling SetFrameHandler.
void cbus_dc_messages::framehandler(CANFrame *msg) {

  // as an example, format and display the received frame

#if DEBUG
  Serial << F("[ ") << (msg->id & 0x7f) << F("] [") << msg->len << F("] [");
  if ( msg->len > 0) {
    for (byte d = 0; d < msg->len; d++) {
      Serial << F(" 0x") << _HEX(msg->data[d]);
    }
  Serial << F(" ]") << endl;
  }

  if (nopcodes > 0) {
    Serial << F("Opcodes [ ");
    for(byte i = 0;  i < nopcodes; i++)
    {
       Serial << F(" 0x") << _HEX(opcodes[i]);
    }
    Serial << F(" ]") << endl;
  }
#endif

  if (nopcodes > 0) {
#if DEBUG
          Serial << F("Message received with Opcode [ 0x") << _HEX(msg->data[0]) << F(" ]")<< endl;
#endif
    for(byte i = 0;  i < nopcodes; i++)
    {
       if ( msg->data[0] == opcodes[i]) {
#if DEBUG
           Serial << F("Message recognised with Opcode [ 0x") << _HEX(opcodes[i]) << F(" ]")<< endl;
#endif
     // This will be executed if the code matches.
           messagehandler(msg);
           break;       
        }
    }
  }
  return;
}
#else
void cbus_dc_messages::framehandler(CANFrame *msg) {

#if DEBUG
          Serial << F("Message received with Opcode [ 0x") << _HEX(msg->data[0]) << F(" ]")<< endl;
#endif
     // This will be executed if the code matches.
  messagehandler(msg);
  return;
}
#endif
// Task to increment timeout counters on active tasks.
void incrementTimeoutCounters()
{
#if DEBUG
    // Temporary output
    Serial << F("incrementTimeoutCounters() called") << endl;
#endif

// session timeout function here

}

void testMessage(byte i)
{
  unsigned char buf[4];
  buf[0] = 0x98;
  buf[1] = 0;
  buf[2] = 0;
  buf[3] = i;
  sendMessage(4,buf);
}




#endif // of IB
