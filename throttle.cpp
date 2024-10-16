//
// throttle.C
//
// Code for each DC Controller throttle using asyncio library
//
// (c) Ian Blair 5th. October 2024
//
#include "arduino.h"
#include "dc_controller_defs.h"
#include "pindefs_dc_controller_esp32.h"
#include "throttle.h"

throttle::throttle(byte throttle_dac_id, byte bemf_adc_pin_id, byte blnk_pin_id)
{   
  _throttle_dac_id = throttle_dac_id;
  _bemf_adc_id = bemf_adc_pin_id;
  _blanking = blnk_pin_id;
  pinMode(_blanking,OUTPUT);
}
            
// DC Controller throttle routine declarations
int throttle::adc_read(byte adc_pin_id)
{
  int _adc_value;
  _adc_value = analogRead(adc_pin_id);
  return (_adc_value);
}
       
void throttle::dac_write(int dac_value, byte dac_pin)
{
  analogWrite(dac_pin, dac_value);
}
        
void throttle::set_blanking()
{
  digitalWrite(_blanking, LOW);
}

void throttle::clear_blanking()
{
  digitalWrite(_blanking, HIGH);
}
    
u16_t throttle::read_bemf()
{
  return(adc_read(_bemf_adc_id));
}

void throttle::write_output(byte _output_level)
{
  dac_write(_throttle_dac_id, _output_level);
}


 
