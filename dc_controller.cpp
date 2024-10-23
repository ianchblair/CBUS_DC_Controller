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
    output_throttle.initialise(DAC1, PIN_BEMF0, PIN_BLNK0);
    return_throttle.initialise(DAC2, PIN_BEMF1, PIN_BLNK1);
  }
  else
  {
    output_throttle = throttle1;
    return_throttle = throttle0;
    output_throttle.initialise(DAC2, PIN_BEMF1, PIN_BLNK1);
    return_throttle.initialise(DAC1, PIN_BEMF0, PIN_BLNK0);
  }
  // delete stored bemf_level
  _last_bemf=0;
      // Reset blanking
  output_throttle.clear_blanking();
  return_throttle.clear_blanking();
}

// Filter calculates instantaneous output value based on mode, and phase
int dc_controller::filter_calc(t_wave_mode wave_mode, int phase, int throttle_level)
{
  long dc_offset;
  long switching_phase;
  long return_value;
  long triangle_value;
  long height;
  // Offset the DC according to the throttle vale
  dc_offset=throttle_level*MAX_OP_LEVEL/MAX_THROTTLE_LEVEL;
  switching_phase = throttle_level*MAX_PHASE/MAX_THROTTLE_LEVEL;
  if (wave_mode == MODE_DIRECT)
  {
    return_value = dc_offset;
  }
  else if ((wave_mode == MODE_TRIANGLE) or (wave_mode == MODE_TRIANGLE_BEMF))
  {
    if (phase < switching_phase)
    {
      triangle_value = min(((phase*MAX_OP_LEVEL)/MAX_PHASE),MAX_OP_LEVEL);
    }
    else
    {
      height = ((switching_phase*MAX_OP_LEVEL)/MAX_PHASE);
      triangle_value = height + (((switching_phase-phase)*MAX_OP_LEVEL)/MAX_PHASE);
      // Keep value below upper limit
      if (triangle_value > MAX_OP_LEVEL) triangle_value = MAX_OP_LEVEL;
    }
    if (triangle_value < 0)
    {
      triangle_value = 0;
    }
    // Only use triangle wave for outputs below 50%
    if (dc_offset<MAX_OP_LEVEL/2)
    {    
      return_value = (triangle_value*(MAX_OP_LEVEL-(2*dc_offset))/MAX_OP_LEVEL) + dc_offset;
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
  return(int(return_value));
}
   
// This calculates the overall throttle level based on the pot setting bemf measurement and selected mode    
int dc_controller::calculate_throttle(t_wave_mode wave_mode, int requested_speed, int bemf_speed)
{
  long output_level;
  long error_correction;
  long error_level;
  long scaled_error_level;
  // By default or if mode is direct, output = input
  // Input levels arefrom ADCs, 0..4095 range
  output_level=requested_speed;
  if ((wave_mode == MODE_TRIANGLE) and (bemf_speed < MAX_BEMF_LEVEL))
  {
    error_correction = ((_last_bemf+bemf_speed)*ERROR_SCALE);
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
  _last_bemf=bemf_speed;
  // return calculated throttle value
  return(output_level);
}

dc_controller::dc_controller(void)
{ 
  // Assign pins
  //_potadc = PIN_POT;
  pinMode(PIN_BLNK0, OUTPUT);
  pinMode(PIN_BLNK1, OUTPUT);
  pinMode(PIN_DIR, INPUT_PULLUP);
  pinMode(PIN_BEMF0, INPUT);
  pinMode(PIN_BEMF1, INPUT);
  
  // Instantiate throttles
  throttle throttle0 = throttle();
  throttle throttle1 = throttle();
        
  // Initialise throttles, using pin IDs, one for each output
  throttle0.initialise(DAC1, PIN_BEMF0, PIN_BLNK0);
  throttle1.initialise(DAC2, PIN_BEMF1, PIN_BLNK1);

  _direction = digitalRead(PIN_DIR);
  set_throttle(_direction);
  _last_direction = _direction;

  // default these to zero until assigned further down...
  //throttle_value=0;
  //requested_level=0;
}
        

    
void dc_controller::update()
{
  int i;
  //only act on direction switch when requested_level is below minimum threshold
  // Note that there is no debounce.
  if (_requested_level < MIN_REQUESTED_LEVEL)
  {
    _direction = digitalRead(PIN_DIR);
  }
  else
  {
    _direction = _last_direction;
  }
                
  if (_direction == _last_direction)
  {
    for (i=0;i<MAX_PHASE;i++) 
    {
      wave(i);
      delay(1);
    }
  }
  else
  {
    // Otherwise reverse direction
    _last_direction = _direction;                
    set_throttle(_direction);
  }
}

//
// wave() - runs every tick (assume 1ms for now) 
//   
void dc_controller::wave(int _phase)
{
  // Perform required actions on particular phases
  // Start all cycles with blanking off on both throttles
  byte _output_sample;
  int _throttle_value;
  if (_phase == 0)
  {
    output_throttle.clear_blanking();
  }  
  else if (_phase == POT_PHASE)
  {
    _requested_level = analogRead(PIN_POT);
  }
  else if (_phase == BLANK_PHASE)
  {
    output_throttle.set_blanking();
  }
  // At end of each cycle recalculate throttle values
  else if (_phase == LAST_PHASE)
  {
    _bemf_level= output_throttle.read_bemf();                
    
    _throttle_value = calculate_throttle(MODE_TRIANGLE,_requested_level,_bemf_level);
  }
  // Regardless of above actions set output value
  // Note that output will only be seen when blanking is not enabled
  _output_sample=filter_calc(MODE_TRIANGLE,_phase,_throttle_value);
  output_throttle.write_output(_output_sample);
  //output_throttle.write_output(requested_level;
  //_phase++;
  //if (_phase >= MAX_PHASE)
    //_phase = 0;
}

