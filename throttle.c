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


class throttle
{   
    byte _throttle_pin;
    byte _bemf_pin;
    byte _blanking;

    initialise(byte throttle_dac_id, byte bemf_pin, byte blnk_pin)
    {
        _throttle_pin = throttle_pin;
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_0);
    int val = adc1_get_raw(ADC1_CHANNEL_0);
        _adc_bemf_instance = bemf_pin;
        _dac_id = self._throttle_dac_id;
        //dac_output_enable(DAC_CHANNEL_1);
        dac_output_enable(_dac_id);

        _blanking = blnk_pin;
    )
            
    // Controller routines
    u16 adc_read(adc adc_instance)
    {
        _adc_value = adc_instance.read();
        return(_adc_value);
    }
        
    void dac_write(int dac_value,dac dac_instance)
    {
        dac_instance.write(dac_value):
    }
        
    void set_blanking()
    [
        self._blanking.value(0);
    }

    clear_blanking()
    {
        self._blanking.value(1);
    }
    
    read_bemf():
    {
        return(_adc_bemf_instance.read());
    }    
    write_output(output_level)
    {
        dac_output_voltage(_dac_id1, output_level);
    }
}

 