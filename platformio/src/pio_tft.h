// A class for performing the I/O for TFT controllers such
// as ILI9488, using a 8 bit parallel data path. Uses
// the Pico's PIO feature to output the bytes at a rate
// of about 30M bytes/sec.

#pragma once

#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "stdint.h"

class PioTft {
 public:
  // Clock division of 1 yields an output rate of about 30M bytes/sec.
  // For a fine tuning of the timing, add/delete delays in pio_tft.pio.
  PioTft(uint8_t tft_d0_pin, uint8_t tft_wr_pin, uint16_t clock_div = 1)
      : tft_d0_pin_(tft_d0_pin),
        tft_wr_pin_(tft_wr_pin),
        clock_div_(clock_div) {}

  void begin();

  // Flush pending writes and then set mode.
  //
  // Single byte mode. Writing the LSB byte of the value.
  void set_mode_single_byte();
  // Double byte mode. Writing the MSB then LSB bytes of the value.
  void set_mode_double_byte();

  // Wait until all pending writes were sent out. Call this
  // before changing D/C signal.
  void flush();

  // Writes a single value.
  void write(uint16_t value);

  // Writes multiple values. More efficient for large blocks.
  void multi_write(const uint8_t* keys, uint32_t n, uint16_t* map);
  void multi_write(const uint16_t* values, uint32_t n);
  void multi_write(uint16_t value, uint32_t n);

 private:
  const uint8_t tft_d0_pin_;  // First of 8 pins.
  const uint8_t tft_wr_pin_;
  const uint16_t clock_div_;

  // Actual pio program offset is set by begin().
  uint program_offset_ = 0;

  // void wait_sm_idle();
};