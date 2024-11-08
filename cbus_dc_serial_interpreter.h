
#ifndef cbus_dc_serial_interpreter_h
#define cbus_dc_serial_interpreter_h

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

void processSerialInput(void);

#endif