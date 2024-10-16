// 
//  dc_controller.cpp
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

#include "dc_controller_defs.h"
#include "pindefs_dc_controller_esp32.h"
#include "dc_controller.h"
#include "throttle.h"
                      
void dc_controller::set_throttle(bool forward_not_backwards)
{
  // Set both outputs to zero before setting/swapping
  throttle0.write_output(0);
  throttle1.write_output(0);
  // The set throttles according to direction
  // Output throttle will drive trains
  // Return throttle will remain at zero for return rail. 
  if (forward_not_backwards == true)
  {
     output_throttle = throttle0;
     return_throttle = throttle1;
  }
  else
  {
    output_throttle = throttle1;
    return_throttle = throttle0;
  }
  // delete stored bemf_level
  _last_bemf=0;
}
        
// Filter calculates instantaneous output value based on mode, and phase
void dc_controller::filter_calc(t_wave_mode wave_mode, int phase, int throttle_level)
{
  int dc_offset;
  // Offset the DC according to the throttle vale
  dc_offset=int(throttle_level*MAX_OP_LEVEL/MAX_THROTTLE_LEVEL);
  switching_phase = int(throttle_level*MAX_PHASE/MAX_THROTTLE_LEVEL);
  if (mode == MODE_DIRECT)
  {
    return_value = dc_offset;
  }
  else if ((mode == MODE_TRIANGLE) or mode == (MODE_TRIANGLE_BEMF))
  {
    if (phase < switching_phase)
    {
      triangle_value = min(int((phase*MAX_OP_LEVEL)/MAX_PHASE),MAX_OP_LEVEL);
    }
    else
    {
      height = int((switching_phase*MAX_OP_LEVEL)/MAX_PHASE);
      triangle_value = min(height + int(((switching_phase-phase)*MAX_OP_LEVEL)/MAX_PHASE), MAX_OP_LEVEL);
    }
    if (triangle_value < 0)
    {
      triangle_value = 0;
    }
    // Only use triangle wave for outputs below 50%
    if (dc_offset<MAX_OP_LEVEL/2)
    {    
      return_value = int(triangle_value*(MAX_OP_LEVEL-(2*dc_offset))/MAX_OP_LEVEL) + dc_offset;
    }
    else
    {
       return_value = dc_offset;
    }
  }
  // Extra check to imit output to range of DAC, where offset puts it out of range
  if (return_value < 0)
  {   
    return_value = 0;
  }
  if (return_value > MAX_OP_LEVEL)
  {
    return_value = MAX_OP_LEVEL;
  }   
  return(return_value);
}
   
// This calculates the overall throttle level based on the pot setting bemf measurement and selected mode    
dc_controller::calculate_throttle(enum mode, int requested_speed, int bemf_speed)
{
  // By default or if mode is direct, output = input
  // Input levels arefrom ADCs, 0..4095 range
  output_level=requested_speed;
  if ((mode == MODE_TRIANGLE) and (bemf_speed < MAX_BEMF_LEVEL))
  {
    error_correction = ((last_bemf+bemf_speed)*ERROR_SCALE);
    error_level = requested_speed-error_correction;
    scaled_error_level = int(error_level*(MAX_THROTTLE_LEVEL-requested_speed)/MAX_THROTTLE_LEVEL);
    if(error_level>=0)
    {
      if((requested_speed + scaled_error_level)>MAX_THROTTLE_LEVEL)
      {
        output_level = MAX_THROTTLE_LEVEL;
      }
      else if((requested_speed + scaled_error_level)>0)
      {
        output_level = requested_speed + scaled_error_level;
      }
      else
      {
        output_level=requested_speed;
      }
    }
  }
  // save bemf measuremant for next cycle
  last_bemf=bemf_speed;
  // return calculated throttle value
  return(output_level);
}

void dc_controller::dc_controller()
{
  int &output_throttle();
  int &return_throttle();
  int _last_bemf;
  bool _last_direction;
  int _phase = 0;   
  // Assign pins
  t0dac = PIN_DAC0;
  t1dac = PIN_DAC1;
  _potadc = PIN_POT
  pinMode(PIN_BLNK0, OUTPUT)
  pinMode(PIN_BLNK1, OUTPUT)
  pinMode(PIN_DIR, INPUT_PULLUP)
        
  // Instantiate throttles, using pin IDs, one for each output
  throttle0 = throttle.throttle(t0dac, PIN_BEMF0, PIN_BLNK0);
  throttle1 = throttle.throttle(t1dac, PIN_BEMF1, PIN_NLNK1);              
  direction = _dirpin.value();
  set_throttle(direction);
  last_direction = direction;
  _timer_synchronisation=asyncio.ThreadSafeFlag()
  output_throttle.clear_blanking();
  return_throttle.clear_blanking();
  // default these to zero until assigned further down...
  throttle_value=0;
  requested_level=0;
}
        

    
void dc_controller::update()
{      
  //only act on direction switch when requested_level is below minimum threshold
  // Note that there is no debounce.
  if (requested_level<dMIN_REQUESTED_LEVEL)
  {
    direction = _dirpin.value();
  }
  else
  {
    direction = _last_direction;
  }
  blanking_enabled = False;
                
  if (direction == +last_direction)
  {
    wave();
  }
  else
  {
    // Otherwise reverse direction
    last_direction = direction;                
    set_throttle(direction);
    // Reset blanking
    output_throttle.clear_blanking();
    return_throttle.clear_blanking();
  }
}

//
// wave() - runs every tick (assume 1ms for now) 
//   
void dc_controller::wave()
{
  // Perform required actions on particular phases
  // Start all cycles with blanking off on both throttles
  byte output_sample;
  if (_phase == 0)
  {
    output_throttle.clear_blanking();
  }  
  else if (_phase == POT_PHASE)
  {
    requested_level = _potadc.read();
  }
  else if (_phase == BLANK_PHASE)
  {
    self.output_throttle.set_blanking();
    blanking_enabled = True;
  }
  // At end of each cycle recalculate throttle values
  else if (_phase == LAST_PHASE)
  {
    if (blanking_enabled)
    {
      bemf_level= self.output_throttle.read_bemf()                    
      blanking_enabled = False
    }
    throttle_value = self.calculate_throttle(defs.MODE_TRIANGLE,requested_level,bemf_level)
  }
  // Regardless of above actions set output value
  // Note that output will only be seen when blanking is not enabled.
  output_sample=self.filter_calc(MODE_TRIANGLE,phase,throttle_value);
  output_throttle.write_output(output_sample);
  //output_throttle.write_output(requested_level
  _phase++;
  if (_phase >= MAX_PHASE)
    _phase = 0;
}
