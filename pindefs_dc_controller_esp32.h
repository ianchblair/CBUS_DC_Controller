// Definitions for IB ESP32 projects

// GPIOs

// Controller I/O
const byte PIN_DAC0 = 25; // # Is Physical pin 9
const btye PIN_DAC1 = 26; // # Is Physical pin 10
const byte DAC_DAC0 = DAC_CHANNEL_1; // # GPIO25 Is Physical pin 9
const btye DAC_DAC1 = DAC_CHANNEL_2; // # GPIO26 Is Physical pin 10

const byte PIN_ADC1_5 = 33; // # Is Physical pin 8
const byte PIN_ADC2_7 = 27; // # Is Physical pin 11
const byte PIN_ADC2_3 = 15; // # Is Physical pin 4

const byte PIN_BLNK0 = 14;
const byte PIN_BLNK1 = 12;

const byte PIN_CRX = 18; // CAN/TWAI Rx
const byte PIN_CTX = 19; // CAN/TWAI Tx

const byte PIN_DIR = 22;

# Mappings
const byte PIN_BEMF0 = PIN_ADC1_5;
const byte PIN_BEMF1 = PIN_ADC2_7;
const byte PIN_POT = PIN_ADC2_3;



