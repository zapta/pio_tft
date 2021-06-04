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
  PioTft(uint8_t tft_wr_pin, uint8_t tft_dc_pin, uint8_t tft_d0_pin,
        uint8_t clock_div = 1)
      : tft_wr_pin_(tft_wr_pin),
        tft_dc_pin_(tft_dc_pin),
        tft_d0_pin_(tft_d0_pin),
        clock_div_(clock_div) {}

  void begin();

  // Output modes
  //
  // One byte per item, DC = high;
  void set_byte_command_mode();
  // One byte per item, DC = low;
  void set_byte_data_mode();
  // Two bytes per item. DC = low.
  void set_pixels_mode();

  // Data transfers.
  //
  // Assumes byte command or data mode.
  void inline write_byte(uint8_t b) { pio_sm_put_blocking(pio_, SM, b); }
  // Assume pixel mode.
  void write_pixels(const uint8_t* pixels, uint32_t n, uint16_t* color_map);
  void write_pixels(const uint16_t* pixels, uint32_t n);
  void write_pixels(uint16_t color, uint32_t n);

 private:
  const uint8_t tft_wr_pin_;
  const uint8_t tft_dc_pin_;
  const uint8_t tft_d0_pin_;  // First of 8 pins.

  const uint8_t clock_div_;

  // Must be constexpr for proper optimizations. If changing,
  // adjust kSmFlevel5FreeMask below. Using SM 0 of the PIO
  // selected below.
  static constexpr uint SM = 0;

  // This is a trickey thing. It provides a mask for pio->flevel register
  // to test if the TX Fifo of sm0 has atleast 5 free words or not. It
  // is done by testing the four bits of fifo level has the pattern 00xx.
  static constexpr uint SM_FLEVEL_FREE_5_MASK = 0x000c;

  PIO const pio_ = pio0;

  // Actual pio program offset is set by begin().
  uint program_offset_ = 0;
  // The offset of the PIO instruction which performs the
  // pull. We track it to detect SM idle condition.
  uint8_t pull_offset_ = 0;

  void wait_sm_idle();
};