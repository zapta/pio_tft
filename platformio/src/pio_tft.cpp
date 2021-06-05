#include "pio_tft.h"

#include <stdio.h>
#include "hardware/pio.h"
#include "pio_tft.pio.h"

// We use state-machine 0 of PIO 0.  Change as needed.
#define PIO (pio0)
#define SM 0

namespace pio_tft {

// This is a trickey thing. It provides a mask for pio->flevel register
// to test if the TX Fifo of sm0 has atleast 5 free words or not. It
// is done by testing that the four bits of fifo level has the pattern
// 00xx, which means no more than 3 occupied entries in the 8 entries
// TX buffer.
static constexpr uint32_t SM_FLEVEL_FREE_5_MASK = 0x000c << (SM * 8);

// Masks for the FDEBUG register.
static constexpr uint32_t SM_STALL_MASK = 1u << (PIO_FDEBUG_TXSTALL_LSB + SM);
static constexpr uint32_t SM_OVERRUN_MASK = 1u << (PIO_FDEBUG_TXOVER_LSB + SM);

// Updated later with the loading offset of the PIO program.
uint program_offset = 0;

void flush() {
  // Clear the sticky stall status.
  pio0->fdebug = SM_STALL_MASK;
  // Wait until the stall flag is up again.
  while (!(PIO->fdebug & SM_STALL_MASK)); {}
}

// For testing.
bool is_overrun() {
  const bool result = PIO->fdebug & SM_OVERRUN_MASK;
  // Clear for next time.
  PIO->fdebug = SM_OVERRUN_MASK;
  return result;
}

void init(uint8_t tft_d0_pin, uint8_t tft_wr_pin, uint16_t clock_div) {
  // Make sure nobody else uses this state machine.
  pio_sm_claim(PIO, SM);

  // Load the PIO program. Starting by default with 16 bits mode
  // since it's at the begining of the PIO program.
  program_offset = pio_add_program(PIO, &tft_io_program);

  // Associate pins with the PIO.
  pio_gpio_init(PIO, tft_wr_pin);
  for (int i = 0; i < 8; i++) {
    pio_gpio_init(PIO, tft_d0_pin + i);
  }

  // Configure the pins to be outputs.
  pio_sm_set_consecutive_pindirs(PIO, SM, tft_wr_pin, 1, true);
  pio_sm_set_consecutive_pindirs(PIO, SM, tft_d0_pin, 8, true);

  // Configure the state machine.
  pio_sm_config c = tft_io_program_get_default_config(program_offset);
  // The pio program declares that a single sideset pin is used.
  // Define it.
  sm_config_set_sideset_pins(&c, tft_wr_pin);
  // The 8 consecutive pins that are used for data outout.
  sm_config_set_out_pins(&c, tft_d0_pin, 8);
  // Set clock divider. Value of 1 for max speed.
  sm_config_set_clkdiv_int_frac(&c, clock_div, 0);
  // Make a single 8 words FIFO from the 4 words TX and RX FIFOs.
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
  // The OSR register shifts to the right, sending the MSB byte
  // first, in a double bytes transfers.
  sm_config_set_out_shift(&c, true, false, 0);
  // Set the SM with the configuration we constructed above.
  // Default mode is single byte.
  pio_sm_init(PIO, SM, program_offset + tft_io_offset_start_8, &c);

  // Start the state machine.
  pio_sm_set_enabled(PIO, SM, true);
}

void set_mode_single_byte() {
  flush();
  // Force a SM jump.
  pio_sm_exec(PIO, SM, pio_encode_jmp(program_offset + tft_io_offset_start_8));
}

void set_mode_double_byte() {
  flush();
  // Force a SM jump.
  pio_sm_exec(PIO, SM, pio_encode_jmp(program_offset + tft_io_offset_start_16));
}

void write(uint16_t value) { pio_sm_put_blocking(PIO, SM, value); }

// A macro to wait until the TX FIFO has at least
// 5 free words.
#define WAIT_FOR_FIFO_5_FREE                      \
  while ((PIO->flevel) & SM_FLEVEL_FREE_5_MASK) { \
  }

// The write register of the TX FIFO.
#define TX_FIFO  PIO->txf[SM]

// This is the most challanging loop since it also does
// the value mapping and we want to do that at max
// transfer rate of the PIO.
void multi_write(const uint8_t *keys, uint32_t n, uint16_t *map) {
  const uint8_t *p = keys;
  const uint8_t *const limit = p + n;
  const uint8_t *const limit_minus_20 = limit - 20;

  while (p < limit_minus_20) {
    WAIT_FOR_FIFO_5_FREE;
    TX_FIFO = map[p[0]];
    TX_FIFO = map[p[1]];
    TX_FIFO = map[p[2]];
    TX_FIFO = map[p[3]];
    TX_FIFO = map[p[4]];

    WAIT_FOR_FIFO_5_FREE;
    TX_FIFO = map[p[5]];
    TX_FIFO = map[p[6]];
    TX_FIFO = map[p[7]];
    TX_FIFO = map[p[8]];
    TX_FIFO = map[p[9]];

    WAIT_FOR_FIFO_5_FREE;
    TX_FIFO = map[p[10]];
    TX_FIFO = map[p[11]];
    TX_FIFO = map[p[12]];
    TX_FIFO = map[p[13]];
    TX_FIFO = map[p[14]];

    WAIT_FOR_FIFO_5_FREE;
    TX_FIFO = map[p[15]];
    TX_FIFO = map[p[16]];
    TX_FIFO = map[p[17]];
    TX_FIFO = map[p[18]];
    TX_FIFO = map[p[19]];

    p += 20;
  }

  while (p < limit) {
    // NOTE: 1 free would be sufficient.
    WAIT_FOR_FIFO_5_FREE;
    TX_FIFO = map[*p++];
  }
}

void multi_write(const uint16_t *values, uint32_t n) {
  const uint16_t *p = values;
  const uint16_t *const limit = p + n;
  const uint16_t *const limit_minus_20 = limit - 20;

  while (p < limit_minus_20) {
    WAIT_FOR_FIFO_5_FREE;
    TX_FIFO = p[0];
    TX_FIFO = p[1];
    TX_FIFO = p[2];
    TX_FIFO = p[3];
    TX_FIFO = p[4];

    WAIT_FOR_FIFO_5_FREE;
    TX_FIFO = p[5];
    TX_FIFO = p[6];
    TX_FIFO = p[7];
    TX_FIFO = p[8];
    TX_FIFO = p[9];

    WAIT_FOR_FIFO_5_FREE;
    TX_FIFO = p[10];
    TX_FIFO = p[11];
    TX_FIFO = p[12];
    TX_FIFO = p[13];
    TX_FIFO = p[14];

    WAIT_FOR_FIFO_5_FREE;
    TX_FIFO = p[15];
    TX_FIFO = p[16];
    TX_FIFO = p[17];
    TX_FIFO = p[18];
    TX_FIFO = p[19];

    p += 20;
  }

  while (p < limit) {
    // NOTE: 1 free would be sufficient.
    WAIT_FOR_FIFO_5_FREE;
    TX_FIFO = *p++;
  }
}

void multi_write(uint16_t value, uint32_t n) {
  while (n > 20) {
    //  printf("%u\n", n);

    WAIT_FOR_FIFO_5_FREE;
    TX_FIFO = value;
    TX_FIFO = value;
    TX_FIFO = value;
    TX_FIFO = value;
    TX_FIFO = value;

    WAIT_FOR_FIFO_5_FREE;
    TX_FIFO = value;
    TX_FIFO = value;
    TX_FIFO = value;
    TX_FIFO = value;
    TX_FIFO = value;

    WAIT_FOR_FIFO_5_FREE;
    TX_FIFO = value;
    TX_FIFO = value;
    TX_FIFO = value;
    TX_FIFO = value;
    TX_FIFO = value;

    WAIT_FOR_FIFO_5_FREE;
    TX_FIFO = value;
    TX_FIFO = value;
    TX_FIFO = value;
    TX_FIFO = value;
    TX_FIFO = value;

    n -= 20;
  }

  // printf("z1\n");
  while (n-- > 0) {
    // NOTE: 1 free would be sufficient.
    WAIT_FOR_FIFO_5_FREE;
    TX_FIFO = value;
  }
}

}  // namespace pio_tft
