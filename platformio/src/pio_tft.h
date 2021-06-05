// A class for performing the I/O for TFT controllers such
// as ILI9488, using a 8 bit parallel data path. Uses
// the Pico's PIO feature to output the bytes at a rate
// of about 30M bytes/sec.

#pragma once

#include "pico/stdlib.h"
#include "stdint.h"

namespace pio_tft {

// Clock division of 1 yields an output rate of about 30M bytes/sec.
// For a finer tuning of the timing, add/remove delays in pio_tft.pio.
void init(uint8_t tft_d0_pin, uint8_t tft_wr_pin, uint16_t clock_div = 1);

// Flush pending writes and then set mode.
//
// Single byte mode. Writing the LSB byte of the value.
void set_mode_single_byte();
// Double byte mode. Writing the MSB then LSB bytes of the value.
void set_mode_double_byte();

// Returns true if the TX FIFO had an overrung since the
// last call to this function. A TX FIFO overrun indicates a bug
// in this driver. For testing.
bool is_overrun();

// Wait until all pending writes were sent out. Call this
// function before changing the D/C signal such that it 
// doesn't affect in-flight values.
void flush();

// Writes a single value.
void write(uint16_t value);

// Writes multiple values. More efficient than calling 
// write() iterativly.
//
// This allows to have a 8 bit/pixel buffer and map it on
// the fly with a lookup table to 16 bits/pixels.
void multi_write(const uint8_t* keys, uint32_t n, uint16_t* map);
// Straight forward block output. 
void multi_write(const uint16_t* values, uint32_t n);
// Repeatetive output of same value. For rect fill, etc.
void multi_write(uint16_t value, uint32_t n);

}  // namespace pio_tft
