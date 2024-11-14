/// Session handling routine for CBUS DC Controller.
///
/// Forked from relevant CANCMD routines
/// 
/// From CANCMD2, MERG John Fletcher et al.
#ifndef cbus_dc_sessions_h
#define cbus_dc_sessions_h

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
#include "dc_controller.h"
#include "trainController.h"
#include "throttle.h"

#if SET_INERTIA_RATE
#define INERTIA        3200       // Inertia counter value. Set High
#endif

/**
 * Definitions of the flags bits
 */
#define SF_FORWARDS  0x01      // Train is running forwards
#define SF_REVERSE   0x00      // Train is running in reverse
#define SF_LONG      0xC0      // long DCC address. top 2 bits of high byte. both 1 for long, both 0 for short.
#define SF_INACTIVE  -1        // CAB Session is not active
#define SF_UNHANDLED -1        // DCC Address is not associated with an analogue controller (duplicate)
#define SF_LOCAL     -2        // DCC Address is operated only by the keypad, and not part of a CAB Session

#define TONE 1000    // Set the tone for the buzzer

#define MAXTIMEOUT 30      // Max number of seconds before session is timed out
                           // if no stayalive received for the session

enum class ErrorState : byte {
  blankError,
  noError,
  emergencyStop,
  CANbusError,
  locoStackFull,
  locoTaken,
  noSession,
  consistEmpty,
  locoNotFound,
  invalidRequest,
  sessionCancelled,
  motorOverload,
  invalidError
};

#define startAddress 1000     // multiplier for DCC address offset from device address. 
// Device 0 uses 1000, device 1 uses 2000,...
#define deviceAddr 0       // assume only unit on CAN bus (for now)

// NOTE: controllers' index (not the DCC address) is used by the keypad handler. 
// Making the last digit of the DCC address = the index aids clarity for user.
struct {
  int             session;
  unsigned int    DCCAddress;
  byte            longAddress;
  byte            timeout;
  boolean         shared;       // this loco shared by > 1 CAB (this includes the keypad)
  struct {
      byte      address;      // DCC short address of consist. 0 = unused.
      byte      session;      // Session id of consist. 0 = unused.
      boolean   reverse;
  } consist;

  trainControllerClass trainController;
} controllers[NUM_CONTROLLERS] = {
#if LINKSPRITE || TOWNSEND
                // Values taken from the motor shield example code
                {SF_INACTIVE, (startAddress * (deviceAddr + 1)) + 1, SF_LONG, 0, false, { 0, 0, false }, trainControllerClass(pinI1, pinI2, pwmpins[0])}
               ,{SF_INACTIVE, (startAddress * (deviceAddress + 1)) + 2, SF_LONG, 0, false, { 0, 0, false }, trainControllerClass(pinI3, pinI4, pwmpins[1])}
#elseif CBUS
                  {SF_INACTIVE, (startAddress * (deviceAddr + 1)) + 1, SF_LONG, 0, false, { 0, 0, false }, trainControllerClass(22, 23, pwmpins[0])}
                   ,{SF_INACTIVE, (startAddress * (deviceAddr + 1)) + 2, SF_LONG, 0, false, { 0, 0, false }, trainControllerClass(24, 25, pwmpins[1])}
                   ,{SF_INACTIVE, (startAddress * (deviceAddr + 1)) + 3, SF_LONG, 0, false, { 0, 0, false }, trainControllerClass(26, 27, pwmpins[2])}
                   ,{SF_INACTIVE, (startAddress * (deviceAddr + 1)) + 4, SF_LONG, 0, false, { 0, 0, false }, trainControllerClass(28, 29, pwmpins[3])}
                   ,{SF_INACTIVE, (startAddress * (deviceAddr + 1)) + 5, SF_LONG, 0, false, { 0, 0, false }, trainControllerClass(30, 31, pwmpins[4])}
                   ,{SF_INACTIVE, (startAddress * (deviceAddr + 1)) + 6, SF_LONG, 0, false, { 0, 0, false }, trainControllerClass(32, 33, pwmpins[5])}
                 //,{SF_INACTIVE, (startAddress * (deviceAddr + 1)) + 7, SF_LONG, 0, false, { 0, 0, false }, trainControllerClass(14, 15, pwmpins[6])}
                 //,{SF_INACTIVE, (startAddress * (deviceAddr + 1)) + 8, SF_LONG, 0, false, { 0, 0, false }, trainControllerClass(16, 17, pwmpins[7])}
#else
                {SF_INACTIVE, (startAddress * (deviceAddr + 1)) + 1, SF_LONG, 0, false, { 0, 0, false }, trainControllerClass()}
#endif
                                  };

#if ENCODER
#if TOWNSEND
// Townsend uses Martin's encoder.
#include "encoderControllerMD.h"
#else
//#define ENCODER_USE_INTERRUPTS
#define ENCODER_DO_NOT_USE_INTERRUPTS
#include "encoderController.h"
#endif
struct {
  encoderControllerClass encoderController;
} encoders[NUM_CONTROLLERS] = {
#if TOWNSEND // Only 2 controllers in this case. Pins for Martin's encoder in the header file.
                {encoderControllerClass(encoder1, spinwheelClickPin1,encoder_name1)},
                {encoderControllerClass(encoder2, spinwheelClickPin2,encoder_name2)}
#else
#if LINKSPRITE // Only 2 controllers in this case.
                {encoderControllerClass(A8,  A0, 38)},
                {encoderControllerClass(A9,  A1, 40)}
#else
                {encoderControllerClass(A8,  A0, 38)},
                {encoderControllerClass(A9,  A1, 40)},
                {encoderControllerClass(A10, A2, 42)},
                {encoderControllerClass(A11, A3, 44)},
                {encoderControllerClass(A12, A4, 46)},
                {encoderControllerClass(A13, A5, 48)},
                //{Encoder(A14, A6, 9), 0, 0, 0},
                //{Encoder(A15, A7, 36), 0, 0, 0}
#endif
#endif
};
#endif


class cbus_dc_sessions
{
// Functions for reset and for various sessions related messages
//

public:

cbus_dc_sessions();

void setup(void);

void ploc(CANFrame *msg, long unsigned int dcc_address, int long_address);

void restp(void);

//IB session timeout funtion
void increment(void);

// New routine for update processing which can be called as needed.
void updateProcessing(bool updateNow);


// Functions for individual sessions...

int getSessionIndex (byte session);

int getDCCIndex (unsigned int dcc_address, byte long_address);

void setInertiaRate(byte session, byte rate);

void setSpeedSteps(byte session, byte steps);

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

void addSessionConsist(byte session, byte consist);

void removeSessionConsist(byte session);

void setSpeedAndDirection(byte controllerIndex, byte requestedSpeed, byte reverse);

/**
 * Send an error packet labelled with the DCC address
 */
void sendError(unsigned int address, byte long_address, ErrorState error_code);


/**
 * Send a session error message to the CABs, labelled with the session number
 */
void sendSessionError(byte session, ErrorState error_code);


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

// N beeps
void nBeeps (byte numBeeps);

/// command interpreter for serial console input
//
};

#endif

