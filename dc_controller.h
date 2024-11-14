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
              
class dc_controller 
{
  int _last_bemf;
  bool forwards_not_backwards;
  bool _direction;
  bool _last_direction;
  int _requested_level;
  int _bemf_level;
  bool _blanking_enabled;
  int _phase;
  throttle throttle0;
  throttle throttle1;
  throttle output_throttle;
  throttle return_throttle; 

  void set_throttle(bool forward_not_backwards);
  int filter_calc(t_wave_mode wave_mode, int phase, int throttle_level);
  int calculate_throttle(t_wave_mode wave_mode, int requested_speed, int bemf_speed);

public:  
  dc_controller(void);
  void setup(void);
  void update(void);
#ifdef CBUSDAC
  void setSpeedAndDirection(int speed, bool direction);
#endif
  void wave(int _phase);
};       

#endif


