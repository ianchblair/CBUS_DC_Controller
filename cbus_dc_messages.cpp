/// Message handling routine for CBUS DC Controller.
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

CBUSConfig _mod_config;
CBUSESP32 _cbus; // CBUS Object
cbus_dc_sessions _sesssions;

//bool cancmd_present = false;
//volatile byte timer_counter = 0;
//volatile byte flash_counter = 0;
//volatile byte beep_counter = 0;
//volatile byte update_counter = 0;
//volatile boolean updateNow = false;
//volatile boolean shutdownFlag = false;

cbus_dc_messages::cbus_dc_messages()
{
  ;
}

void cbus_dc_messages::messages_setup(CBUSConfig params, CBUSESP32 cbus_params)
{
  _mod_config = params;
  _cbus = cbus_params; 
}


/// Send an event routine built to start sending events based on input from a CANCAB
bool sendEvent(byte opCode,unsigned int eventNo)
{
    CANFrame msg;
    msg.id = _mod_config.CANID;
    msg.len = 5;
    msg.data[0] = opCode;
    msg.data[1] = highByte(_mod_config.nodeNum);
    msg.data[2] = lowByte(_mod_config.nodeNum);
    msg.data[3] = highByte(eventNo); // event number (EN) could be > 255
    msg.data[4] = lowByte(eventNo); //CBUSConfig
    msg.ext = false;
    msg.rtr = false;

    //bool res = CBUS.sendMessage(&msg);
    bool res = _cbus.sendMessage(&msg);
#if DEBUG
    if (res) {
      Serial << F("> sent CBUS event with opCode [ 0x") << _HEX(opCode) << F(" ] and event No ") << eventNo << endl;
    } else {
      Serial << F("> error sending CBUS event wit opcode [ 0x") <<  _HEX(opCode) << F(" ]") << endl;
    }
#endif
    return res;
}


/// Send an event routine built to start sending events based with one extra byte
/// The events can be ACON1 or ACOF1 with 1 byte of data.
bool sendEvent1(byte opCode, unsigned int eventNo, byte item)
{
    CANFrame msg;
    msg.id = _mod_config.CANID;
    msg.len = 6;
    msg.data[0] = opCode;
    msg.data[1] = highByte(_mod_config.nodeNum);
    msg.data[2] = lowByte(_mod_config.nodeNum);
    msg.data[3] = highByte(eventNo); // event number (EN) could be > 255
    msg.data[4] = lowByte(eventNo); 
    msg.data[5] = item;    // The extra byte
    msg.ext = false;
    msg.rtr = false;

    bool res = _cbus.sendMessage(&msg);
#if DEBUG
    if (res) {
      Serial << F("> sent CBUS event with opCode [ 0x") << _HEX(opCode) << F(" ] and event No ") << eventNo << endl;
    } else {
      Serial << F("> error sending CBUS event wit opcode [ 0x") <<  _HEX(opCode) << F(" ]") << endl;
    }
#endif
    return res;
}

/// Send an event routine built to start sending events based with extra bytes
bool sendEventN(byte opCode,unsigned int eventNo, byte n, const byte* buf)
{
  // The events can be ACON1, ACOF1, ACON2, ACOF2, ACON3, ACOF3 with 1 2 or 3 bytes of data.
  // I am not checking the match between opcode and data length (as yet)
  if (n == 0) {
     // no data, use the old method.
     return sendEvent(opCode, eventNo);
  } else {
    CANFrame msg;
    msg.id = _mod_config.CANID;
    msg.len = 5+n;
    msg.data[0] = opCode;
    msg.data[1] = highByte(_mod_config.nodeNum);
    msg.data[2] = lowByte(_mod_config.nodeNum);
    msg.data[3] = highByte(eventNo); // event number (EN) could be > 255
    msg.data[4] = lowByte(eventNo); 
    for(byte i = 0;  i< n; i++)
    {
       msg.data[i+5] = buf[i];
    }
    msg.ext = false;
    msg.rtr = false;

    bool res = _cbus.sendMessage(&msg);
#if DEBUG
    if (res) {
      Serial << F("> sent CBUS event with opCode [ 0x") << _HEX(opCode) << F(" ] and event No ") << eventNo << endl;
    } else {
      Serial << F("> error sending CBUS event wit opcode [ 0x") <<  _HEX(opCode) << F(" ]");
      Serial << F(" with ") << n << F(" data : ");
      for (byte i = 0;  i< n; i++ ) Serial << buf[i] << F(" ");
      Serial << endl;
    }
#endif
    return res;
  }
}


/// This replaces the CAN0.SendMsgBuff usage.
/// It uses the CANID of the current configuration.
bool cbus_dc_messages::sendMessage(byte len, const byte *buf)
{
    CANFrame msg;
    msg.id = _mod_config.CANID;
    msg.len = len;
    for(byte i = 0;  i< len; i++)
    {
       msg.data[i] = buf[i];
    }
    msg.ext = false;
    msg.rtr = false;
  
    bool res = _cbus.sendMessage(&msg);
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
/// user-defined event processing function
/// called from the CBUS library when a learned event is received
/// it receives the event table index and the CAN frame
//

void cbus_dc_messages::eventhandler(byte index, CANFrame *msg) 
{

  // as an example, control an LED if the first EV equals 1

  Serial << F("> event handler: index = ") << index << F(", opcode = 0x") << _HEX(msg->data[0]) << endl;

/* 
 *  I am going to change the event handler to process based on the opcode as is done in the
 *  message handler. The old alternative is now removed.
 */
    byte op_code = msg->data[0];
    unsigned int node_number = (msg->data[1] << 8 ) + msg->data[2];
    // This is not true in all cases.
    // For some it is the device number
    unsigned int event_number = (msg->data[3] << 8 ) + msg->data[4];
    // For now get the first event value
    byte ev = 1;
    //byte evval = config.getEventEVval(index, ev - 1); I think the library has changed.
    byte evval = _mod_config.getEventEVval(index, ev);
    Serial << F("> NN = ") << node_number << F(", EN = ") << event_number << endl;
    Serial << F("> op_code = ") << op_code << endl;
    Serial << F("> EV1 = ") << evval << endl;
    switch (op_code)
    {
         // Event on and off
         // Handle these together based on event no.
         case OPC_ACON:
         case OPC_ACOF:
         if (evval == 1) {
            if (op_code == OPC_ACON) {
               Serial << F("> switching the LED on") << endl;
               //moduleLED.blink();
            } else if (op_code == OPC_ACOF) {
               Serial << F("> switching the LED off") << endl;
               //moduleLED.off();
            }
         }

         // This is my buzzer example changed to use op codes and test off as well as on.
         // Now changed to use the CBUSBUZZER library
         if (evval == 99) { //Corrected bug 
            if ( op_code == OPC_ACON) {
               Serial << F("> switching the LED on") << endl;
               //moduleLED.blink();
#if USE_CBUSBUZZER
               moduleBuzzer.on();
#else
               //tone(buzzer, 1000);
#endif
               Serial << F("> BUZZER ON") << endl;
            } else if ( op_code == OPC_ACOF){
               Serial << F("> switching the LED off") << endl;
               //moduleLED.off();
#if USE_CBUSBUZZER
               moduleBuzzer.off();
#else
               //noTone(buzzer);
#endif
               Serial << F("> BUZZER OFF") << endl;
               }
         }

         break;
#if ACCESSORY_REQUEST_EVENT 
         // Accessory response events
         // Handle these together based on event no.
         case OPC_ARON:
         case OPC_AROF:
            Serial << F("> Handling long event response to remote request ");
            if ( op_code == OPC_ARON) {
               Serial << F(" : remote event is on") << endl;            
            } else if (op_code == OPC_AROF) {
               Serial << F(" : remote event is off") << endl;            
            }
         break;
#if USE_SHORT_EVENTS
         // Accessory response short events
         // Handle these together based on event no.
         case OPC_ARSON:
         case OPC_ARSOF:
            Serial << F("> Handling short event response to remote request ");
            Serial << F(" device number ") << event_number;
            if ( op_code == OPC_ARSON) {
               Serial << F(" : remote event is on") << endl;            
            } else if (op_code == OPC_ARSOF) {
               Serial << F(" : remote event is off") << endl;            
            }
         break;
#endif
#endif
         // Space for more op codes.
         default:
         // ignore any other CBUS events
#if DEBUG
         //Serial << F("Event ignored with Opcode [ 0x") << _HEX(op_code) << F(" ]")<< endl;
#endif
         break;
    }
  

  return;
}

#ifdef CBUS_LONG_MESSAGE
   byte new_message = true;
///
/// Handler to receive a long message 
///
void cbus_dc_messages::longmessagehandler(byte *fragment, unsigned int fragment_len, byte stream_id, byte status){
// I need an example for what goes in here.
     fragment[fragment_len] = 0;
// If the message is complete it will be in fragment and I can do something with it.
     if( new_message) { // Print this only for the start of a message.
        Serial << F("> user long message handler: stream = ") << stream_id << F(", fragment length = ") 
               << fragment_len << F(", fragment = |");
        new_message = false;
     }
     if ( status == CBUS_LONG_MESSAGE_INCOMPLETE ) {
     // handle incomplete message
         Serial.write(fragment, fragment_len);
    } else if (status == CBUS_LONG_MESSAGE_COMPLETE) {
     // handle complete message
        Serial.write(fragment, fragment_len);
        Serial << F("|, status = ") << status << endl;
        new_message = true;  // reset for the next message
     } else {  // CBUS_LONG_MESSAGE_SEQUENCE_ERROR
               // CBUS_LONG_MESSAGE_TIMEOUT_ERROR,
               // CBUS_LONG_MESSAGE_CRC_ERROR
               // raise an error?
        Serial << F("| Message error with  status = ") << status << endl;
        new_message = true;  // reset for the next message
     } 
}
  
#endif
//
/// print code version config details and copyright notice
//


//
/// user-defined frame processing function
/// called from the CBUS library for *every* CAN frame received
/// it receives a pointer to the received CAN frame
//  Added from new version of CBUS_empty


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
  //messagehandler(msg);
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
//
//void testMessage(byte i)
//{
//  unsigned char buf[4];
//  buf[0] = 0x98;
//  buf[1] = 0;
//  buf[2] = 0;
//  buf[3] = i;
//  sendMessage(4,buf);
//}
//
// Execution routines to be added.
//

