//
// throttle.h
//
// Code for each DC Controller throttle
//
// (c) Ian Blair 11th. October 2024
//
#ifndef throttle_h
#define throttle_h

class throttle
{
  byte _throttle_dac_id;
  byte _bemf_adc_id;
  static byte _blanking;
  byte _output_level;

public:
 // DC Controller throttle routine declarations
  throttle(byte throttle_dac_id, byte bemf_adc_pin, byte blnk_pin);
  int adc_read(byte adc_id);
  void dac_write(int dac_value, byte dac_instance);
  void set_blanking();
  void clear_blanking();  
  u16_t read_bemf();
  void write_output(byte _output_level);
};
#endif