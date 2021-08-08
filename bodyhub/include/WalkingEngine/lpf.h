#ifndef _lpf_h_
#define _lpf_h_

#include <iostream>

class LowPassFilter2p {
 public:
  // constructor
  LowPassFilter2p(float sample_freq, float cutoff_freq);

  // change parameters
  void set_cutoff_frequency(float sample_freq, float cutoff_freq);

  // apply - Add a new raw value to the filter
  // and retrieve the filtered result
  float apply(float sample);

  // return the cutoff frequency
  float get_cutoff_freq(void) const { return _cutoff_freq; }

 private:
  float _cutoff_freq;
  float _a1;
  float _a2;
  float _b0;
  float _b1;
  float _b2;
  float _delay_element_1;  // buffered sample -1
  float _delay_element_2;  // buffered sample -2
};

#endif
