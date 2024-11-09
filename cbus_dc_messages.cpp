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

CBUSConfig module_config;
CBUSESP32 CBUS; // CBUS Object

bool cancmd_present = false;
volatile byte timer_counter = 0;
volatile byte flash_counter = 0;
volatile byte beep_counter = 0;
volatile byte update_counter = 0;
volatile boolean updateNow = false;
volatile boolean shutdownFlag = false;

void cbus_message_setup(CBUSConfig params)
{
  module_config = params;
}

/// Send an event routine built to start sending events based on input from a CANCAB
bool sendEvent(byte opCode,unsigned int eventNo)
{
    CANFrame msg;
    msg.id = module_config.CANID;
    msg.len = 5;
    msg.data[0] = opCode;
    msg.data[1] = highByte(module_config.nodeNum);
    msg.data[2] = lowByte(module_config.nodeNum);
    msg.data[3] = highByte(eventNo); // event number (EN) could be > 255
    msg.data[4] = lowByte(eventNo); 
    msg.ext = false;
    msg.rtr = false;

    //bool res = CBUS.sendMessage(&msg);
    bool res = CBUS.sendMessage(&msg);
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
    msg.id = module_config.CANID;
    msg.len = 6;
    msg.data[0] = opCode;
    msg.data[1] = highByte(module_config.nodeNum);
    msg.data[2] = lowByte(module_config.nodeNum);
    msg.data[3] = highByte(eventNo); // event number (EN) could be > 255
    msg.data[4] = lowByte(eventNo); 
    msg.data[5] = item;    // The extra byte
    msg.ext = false;
    msg.rtr = false;

    bool res = CBUS.sendMessage(&msg);
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
    msg.id = module_config.CANID;
    msg.len = 5+n;
    msg.data[0] = opCode;
    msg.data[1] = highByte(module_config.nodeNum);
    msg.data[2] = lowByte(module_config.nodeNum);
    msg.data[3] = highByte(eventNo); // event number (EN) could be > 255
    msg.data[4] = lowByte(eventNo); 
    for(byte i = 0;  i< n; i++)
    {
       msg.data[i+5] = buf[i];
    }
    msg.ext = false;
    msg.rtr = false;

    bool res = CBUS.sendMessage(&msg);
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
bool sendMessage(byte len, const byte *buf)
{
    CANFrame msg;
    msg.id = module_config.CANID;
    msg.len = len;
    for(byte i = 0;  i< len; i++)
    {
       msg.data[i] = buf[i];
    }
    msg.ext = false;
    msg.rtr = false;
  
    bool res = CBUS.sendMessage(&msg);
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

void eventhandler(byte index, CANFrame *msg) 
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
    byte evval = module_config.getEventEVval(index, ev);
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
void longmessagehandler(byte *fragment, unsigned int fragment_len, byte stream_id, byte status){
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

//
/// user-defined frame processing function
/// called from the CBUS library for *every* CAN frame received
/// it receives a pointer to the received CAN frame
//  Added from new version of CBUS_empty
#if 0
// Converted to get an opcode array now done when calling SetFrameHandler.
void framehandler(CANFrame *msg) {

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
void framehandler(CANFrame *msg) {

#if DEBUG
          Serial << F("Message received with Opcode [ 0x") << _HEX(msg->data[0]) << F(" ]")<< endl;
#endif
     // This will be executed if the code matches.
  messagehandler(msg);
  return;
}
#endif
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
// Temporary fix to call update when keep alive comes in.
          updateNow = true;
          updateProcessing();
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
          updateNow = true;
          updateProcessing();
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
void testMessage(byte i)
{
  unsigned char buf[4];
  buf[0] = 0x98;
  buf[1] = 0;
  buf[2] = 0;
  buf[3] = i;
  sendMessage(4,buf);
}
//
// Execution routines to be added.
//

