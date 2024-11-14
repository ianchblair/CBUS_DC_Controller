/// Message handling routine for CBUS DC Controller.
///
/// Forked from relevant CANCMD routines
/// 
/// From CANCMD2, MERG John Fletcher et al.

#ifndef cbus_dc_session_messages_h
#define cbus_dc_session_messages_h

//#include "cbus_dc_sessions.h"
//#include "cbus_dc_messages.h"

class cbus_dc_session_messages: public cbus_dc_messages, public cbus_dc_sessions
{
/// Send an event routine built to start sending events based on input from a CANCAB
bool sendEvent(byte opCode,unsigned int eventNo);

/// Send an event routine built to start sending events based with one extra byte
/// The events can be ACON1 or ACOF1 with 1 byte of data.
bool sendEvent1(byte opCode, unsigned int eventNo, byte item);

/// Send an event routine built to start sending events based with extra bytes
bool sendEventN(byte opCode,unsigned int eventNo, byte n, const byte* buf);

//bool sendMessage(byte len, const byte *buf);

void messagehandler(CANFrame *msg);

// Task to increment timeout counters on active tasks.
void incrementTimeoutCounters();

// New routine for update processing which can be called as needed.
void updateProcessing();
//

/* *******************************************************************************
 * functions and procedures
 * *******************************************************************************/

/*
 * A loco release command for the given session
 */
void releaseLoco(byte session);
/*
 * A QLOC command for the given session from a CAB
 */
void queryLoco(byte session);

/*
 * The command station has allocated a session to a locomotive
 */
void locoSession(byte session, unsigned int address, byte long_address, byte direction_, byte speed_);

/*
 * Keep alive received, so reset the timeout counter
 */
void keepaliveSession(byte session);

/*
 * A throttle has requested access to a particular loco address
 * This routine is only used if there is no CANCMD on the bus that will
 * allocate sessions.
 */
void locoRequest(unsigned int address, byte long_address, byte flags);

/*
* A throttle has requested access to a particular consist address
* This routine is only used if there is no CANCMD on the bus that will
* allocate sessions.
*/
void consistRequest(unsigned int address);

/**
 * Send a PLOC message in response to a CAB requesting a session for
 * a DCC address
 */
void sendPLOC(byte session);

void sendPLOCConsist(byte address);

void addSessionConsist(byte session, byte conist);

void removeSessionConsist(byte session);

void setSpeedAndDirection(byte controllerIndex, byte requestedSpeed, byte reverse);

/**
 * Send an error packet labelled with the DCC address
 */
void sendError(unsigned int address, byte long_address, byte error_code);

/**
 * Send a session error message to the CABs, labelled with the session number
 */
void sendSessionError(byte session, byte error_code);

/**
 * Send a reset signal to all connected CABs
 */
void sendReset();

void emergencyStopAll();

/**
* Stop all DC tracks
* Loop over every session and if it is not free set
* the speed to 0 (stop) or 1 (eStop)
*/
void stopAll(boolean emergency);

/*
* Send a DSPD message to CABs showing speed/direction
*/
void sendDSPD(byte controllerIndex);

void processSerialInput(void);

public:
//cbus_dc_messages();

void message_setup(CBUSConfig params);

/// user-defined event processing function
/// called from the CBUS library when a learned event is received
/// it receives the event table index and the CAN frame
//
void eventhandler(byte index, CANFrame *msg);

void longmessagehandler(byte *fragment, unsigned int fragment_len, byte stream_id, byte status);

void framehandler(CANFrame *msg);

public:
/// This replaces the CAN0.SendMsgBuff usage.
/// It uses the CANID of the current configuration.
bool sendMessage(byte len, const byte *buf);
//

};
#endif