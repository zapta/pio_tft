#include "pio_tft.h"

#include <stdio.h>
#include "pio_tft.pio.h"

void PioTft::wait_sm_idle() {
  // Must evaluate fifo empty before pc. Otherwise the loop can exit
  // while the SM still processes the last word.
  while (!pio_sm_is_tx_fifo_empty(pio_, SM) ||
         pio_sm_get_pc(pio_, SM) != pull_offset_) {
  }
}

void PioTft::begin() {
  // Make sure nobody else uses this state machine.
  pio_sm_claim(pio_, SM);

  // Plain gpio output, non PIO related.
  gpio_init_mask(1u << tft_dc_pin_);
  gpio_set_dir_out_masked(1u << tft_dc_pin_);

  // Load the PIO program.
  program_offset_ = pio_add_program(pio_, &tft_io_program);
  pull_offset_ = program_offset_ + tft_io_offset_pull_16;

  // Associate pins with the PIO.
  pio_gpio_init(pio_, tft_wr_pin_);
  for (int i = 0; i < 7; i++) {
    pio_gpio_init(pio_, tft_d0_pin_ + i);
  }

  // Configure the pins to be outputs.
  pio_sm_set_consecutive_pindirs(pio_, SM, tft_wr_pin_, 1, true);
  pio_sm_set_consecutive_pindirs(pio_, SM, tft_d0_pin_, 8, true);

  // Configure the state machine.
  pio_sm_config c = tft_io_program_get_default_config(program_offset_);
  // The pio program declares that a single sideset pin is used.
  // Define it.
  sm_config_set_sideset_pins(&c, tft_wr_pin_);
  // The 8 consecutive pins that are used for data outout.
  sm_config_set_out_pins(&c, tft_d0_pin_, 8);
  // Set clock divider. Value of 1 for max speed.
  sm_config_set_clkdiv_int_frac(&c, clock_div_, 0);
  // Make a single 8 words FIFO from the 4 words TX and RX FIFOs.
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
  // The OSR register shifts to the right, sending the MSB byte
  // first, in a double bytes transfers.
  sm_config_set_out_shift(&c, true, false, 0);
  // Set the SM with the configuration we constructed above.
  pio_sm_init(pio_, SM, program_offset_, &c);

  // Start the state machine.
  pio_sm_set_enabled(pio_, SM, true);
}

void PioTft::set_byte_command_mode() {
  wait_sm_idle();
  gpio_set_mask(1ul << tft_dc_pin_);
  // Select 8 bits program.
  pull_offset_ = program_offset_ + tft_io_offset_pull_8;
  pio_sm_exec(pio_, SM,
              pio_encode_jmp(program_offset_ + tft_io_offset_start_8));
}

void PioTft::set_byte_data_mode() {
  wait_sm_idle();
  // DC low indicates non-command mode.
  gpio_clr_mask(1ul << tft_dc_pin_);
  // Select 8 bits program
  pull_offset_ = program_offset_ + tft_io_offset_pull_8;
  pio_sm_exec(pio_, SM,
              pio_encode_jmp(program_offset_ + tft_io_offset_start_8));
}

void PioTft::set_pixels_mode() {
  wait_sm_idle();
  // DC low indicates non-command mode  mode.
  gpio_clr_mask(1ul << tft_dc_pin_);
  // Select 16 bits program.
  pull_offset_ = program_offset_ + tft_io_offset_pull_16;
  pio_sm_exec(pio_, SM,
              pio_encode_jmp(program_offset_ + tft_io_offset_start_16));
}

// A macro to wait until the TX FIFO has at least 
// 5 free words. 
#define WAIT_FOR_FIFO_5_FREE                       \
  while ((pio_->flevel) & SM_FLEVEL_FREE_5_MASK) { \
  }

// This is the most challanging loop since it also does
// the color mapping and we want to do that at max
// transfer rate of the PIO.
void PioTft::write_pixels(const uint8_t *pixels, uint32_t n,
                         uint16_t *color_map) {
  const uint8_t *p = pixels;
  const uint8_t *const limit = p + n;
  const uint8_t *const limit_minus_20 = limit - 20;

  while (p < limit_minus_20) {
    WAIT_FOR_FIFO_5_FREE;
    pio_->txf[SM] = color_map[p[0]];
    pio_->txf[SM] = color_map[p[1]];
    pio_->txf[SM] = color_map[p[2]];
    pio_->txf[SM] = color_map[p[3]];
    pio_->txf[SM] = color_map[p[4]];

    WAIT_FOR_FIFO_5_FREE;
    pio_->txf[SM] = color_map[p[5]];
    pio_->txf[SM] = color_map[p[6]];
    pio_->txf[SM] = color_map[p[7]];
    pio_->txf[SM] = color_map[p[8]];
    pio_->txf[SM] = color_map[p[9]];

    WAIT_FOR_FIFO_5_FREE;
    pio_->txf[SM] = color_map[p[10]];
    pio_->txf[SM] = color_map[p[11]];
    pio_->txf[SM] = color_map[p[12]];
    pio_->txf[SM] = color_map[p[13]];
    pio_->txf[SM] = color_map[p[14]];

    WAIT_FOR_FIFO_5_FREE;
    pio_->txf[SM] = color_map[p[15]];
    pio_->txf[SM] = color_map[p[16]];
    pio_->txf[SM] = color_map[p[17]];
    pio_->txf[SM] = color_map[p[18]];
    pio_->txf[SM] = color_map[p[19]];

    p += 20;
  }

  while (p < limit) {
    WAIT_FOR_FIFO_5_FREE;
    pio_->txf[SM] = color_map[*p++];
  }
}

void PioTft::write_pixels(const uint16_t *pixels, uint32_t n) {
  const uint16_t *p = pixels;
  const uint16_t *const limit = p + n;
  const uint16_t *const limit_minus_20 = limit - 20;

  while (p < limit_minus_20) {
    WAIT_FOR_FIFO_5_FREE;
    pio_->txf[SM] = p[0];
    pio_->txf[SM] = p[1];
    pio_->txf[SM] = p[2];
    pio_->txf[SM] = p[3];
    pio_->txf[SM] = p[4];

    WAIT_FOR_FIFO_5_FREE;
    pio_->txf[SM] = p[5];
    pio_->txf[SM] = p[6];
    pio_->txf[SM] = p[7];
    pio_->txf[SM] = p[8];
    pio_->txf[SM] = p[9];

    WAIT_FOR_FIFO_5_FREE;
    pio_->txf[SM] = p[10];
    pio_->txf[SM] = p[11];
    pio_->txf[SM] = p[12];
    pio_->txf[SM] = p[13];
    pio_->txf[SM] = p[14];

    WAIT_FOR_FIFO_5_FREE;
    pio_->txf[SM] = p[15];
    pio_->txf[SM] = p[16];
    pio_->txf[SM] = p[17];
    pio_->txf[SM] = p[18];
    pio_->txf[SM] = p[19];

    p += 20;
  }

  while (p < limit) {
    WAIT_FOR_FIFO_5_FREE;
    pio_->txf[SM] = *p++;
  }
}

void PioTft::write_pixels(uint16_t color, uint32_t n) {
  while (n > 5) {
    WAIT_FOR_FIFO_5_FREE;
    pio_->txf[SM] = color;
    pio_->txf[SM] = color;
    pio_->txf[SM] = color;
    pio_->txf[SM] = color;
    pio_->txf[SM] = color;

    WAIT_FOR_FIFO_5_FREE;
    pio_->txf[SM] = color;
    pio_->txf[SM] = color;
    pio_->txf[SM] = color;
    pio_->txf[SM] = color;
    pio_->txf[SM] = color;

    WAIT_FOR_FIFO_5_FREE;
    pio_->txf[SM] = color;
    pio_->txf[SM] = color;
    pio_->txf[SM] = color;
    pio_->txf[SM] = color;
    pio_->txf[SM] = color;

    WAIT_FOR_FIFO_5_FREE;
    pio_->txf[SM] = color;
    pio_->txf[SM] = color;
    pio_->txf[SM] = color;
    pio_->txf[SM] = color;
    pio_->txf[SM] = color;

    n = -20;
  }

  while (n-- > 0) {
    WAIT_FOR_FIFO_5_FREE;
    pio_->txf[SM] = color;
  }
}
