//
// dc_controller_defs.h
//
// DC controller definitions (ESP32 C++)
//
// (c) Ian Blair 4th. October 2024
//
// For license and attributions see associated readme file
//

//Phase values
//MAX_PHASE=const(1024)
//POT_PHASE=const(32)
//BLANK_PHASE=const(896)
//BEMF_PHASE=const(960)
//LAST_PHASE=const(MAX_PHASE-1)                

const int MAX_PHASE = 16;
const int POT_PHASE = 3;
const int BLANK_PHASE = 14;
const int LAST_PHASE = (MAX_PHASE-1);

// Levels and scale factors
const int MIN_REQUESTED_LEVEL = 10;
const int MAX_THROTTLE_LEVEL = 4095;
const int FORTY_PERCENT_THROTTLE =(MAX_THROTTLE_LEVEL*40/100);
const int MAX_OP_LEVEL = 255;
const int MAX_BEMF_LEVEL = 1000;
const int ERROR_SCALE = 8;
const int INP_SCALE 8; //Equivalent of 8 used for Ardiono. Divide by 2 for ADC reference 2.45V,
//and a further 4 for input range (12 bits, not 10)

// Modes
const int MODE_DIRECT = 0;
const int MODE_TRIANGLE = 1;
