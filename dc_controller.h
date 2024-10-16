// 
//  dc_controller.h
// 
//  Code for standalone DC controller (ESP32 C++ version)
// 
//  Very loosely based on John Purbrick and Martin Da Costa's (MERG) design of 2021,
//  but essentially a complete rewrite using Mictopython and uasyncio
// CBUS interface is omitted in this version
//
// (c) Ian Blair 4th. October 2024
//
// For license and attributions see associated readme file
//
#ifndef dc_controller_h
#define dc_controller_h

#include "throttle.h"

typedef enum
{
  ZERP,
  DIRECT,
  TRIANGLE,
} t_wave_mode;
              
class dc_controller 
{
  throttle output_throttle;
  throttle return_throttle;
  int _last_bemf;
  bool forwards_not_backwards;
  bool _last_direction;
  throttle throttle0;
  throttle throttle1;

  void set_throttle(bool forward_not_backwards);
  void filter_calc(t_wave_mode wave_mode, int phase, int throttle_level);
  void calculate_throttle(t_wave_mode wave_mode, int requested_speed, int bemf_speed);

public:  
  dc_controller(void);
  void setup(void);
  void update(void);
  void wave(void);
};       

#endif


