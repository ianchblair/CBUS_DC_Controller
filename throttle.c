//
// throttle.C
//
// Code for each DC Controller throttle using asyncio library
//
// (c) Ian Blair 5th. October 2024
//

//from machine import Pin,SPI,Timer,DAC,ADC
#include "driver/adc.h"
#include "driver/dac.h"
#include pindefs_dc_controller_esp32.h
#include dc_controller_defs.h

class throttle()
{
  byte _throttle_dac_id;
  byte _bemf_adc_id;
  byte _bemf_adc_ch_id;
  byte _blanking;

public:
 // DC Controller throttle routine declarations
  throttle(byte throttle_dac_id, byte bemf_adc_pin, byte blnk_pin);
  u16 adc_readadc_read(byte adc_ch_id, byte adc_id);
  void dac_write(int dac_value,dac dac_instance);
  void set_blanking();
  void clear_blanking();  
  u16 read_bemf();
  void write_output(output_level);
}

throttle::throttle(byte throttle_dac_id, byte bemf_adc_pin_id, byte blnk_pin_id)
{   
  _throttle_dac_id = throttle_dac_id;
  if (bemf_adc_pin_id >= ADC_BANK1)
  {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(bemf_adc_id,ADC_ATTEN_DB_11);
    _bemf_adc_id = 1;
    _bemf_adc_read = &adc1_get_raw();
    _bemf_adc_ch_id = ADC1_GPIOn_CHANNEL(adc_pin_id);
  }
  else
  {
    adc2_config_width(ADC_WIDTH_BIT_12);
    adc2_config_channel_atten(bemf_adc_id_ADC_ATTEN_DB_11);
    _bemf_adc_id = 2;
    _bemf_adc_ch_id = ADC2_GPIOn_CHANNEL(adc_pin_id);
    _bemf_adc_read = &adc2_get_raw();
  }   
  _adc_bemf_ch_instance = bemf_adc_ch_id;
  _adc_bemf_instance = bemf_adc_id;
  dac_output_enable(_throttle_dac_id);
  _blanking = blnk_pin;
)
            
// DC Controller throttle routine declarations
int throttle::adc_read(byte adc_ch_id, byte adc_pin_id)
{
  if (_bemf_adc_id == 1) _adc_value = adc1_get_raw(_bemf_adc_ch_id);
  else  if (_bemf_adc_id == 2) _adc_value = adc1_get_raw(_bemf_adc_ch_id);
  else error();
  return(_adc_value);
}
       
void throttle::dac_write(int dac_value,dac dac_instance)
{
  dac_instance.write(dac_value);
}
        
void throttle::set_blanking()
[
  _blanking.value(0);
}

void throttle::clear_blanking()
{
  _blanking.value(1);
}
    
u16 throttle::read_bemf()
{
  return(_adc_bemf_instance.read());
}

void throttle::write_output(output_level)
{
  dac_output_voltage(_dac_id, output_level);
}


 