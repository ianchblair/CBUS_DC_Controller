//
// throttle.h
//
// Code for each DC Controller throttle
//
// (c) Ian Blair 11th. October 2024
//

class throttle()
{
 // DC Controller throttle routine declarations
  throttle(byte throttle_dac_id, byte bemf_adc_pin, byte blnk_pin);
  u16 adc_readadc_read(byte adc_ch_id, byte adc_id);
  void dac_write(int dac_value,dac dac_instance);
  void set_blanking();
  void clear_blanking();  
  u16 read_bemf();
  void write_output(output_level);
}
