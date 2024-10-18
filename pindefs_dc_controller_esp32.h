// Definitions for IB ESP32 projects

#ifndef pindefs_dc_controller_esp32_h
#define pindefs_dc_controller_esp32_h

#include <arduino.h> // for byte definition

// GPIOs

// Controller I/O
// const byte PIN_DAC0 = 25; // # GPIO25 Is Physical pin 9
// const byte PIN_DAC1 = 26; // # GPIO26 Is Physical pin 10

const byte PIN_BEMF0 = 33; // ADC 1_5 is Physical pin 8
const byte PIN_BEMF1 = 27; // ABC 2_7 is Physical pin 11
const byte PIN_POT = 15; // # ADC 2_3 is Physical pin 4

const byte PIN_BLNK0 = 14;
const byte PIN_BLNK1 = 12;

const byte PIN_CRX = 18; // CAN/TWAI Rx
const byte PIN_CTX = 19; // CAN/TWAI Tx

const byte PIN_DIR = 22;

#endif
