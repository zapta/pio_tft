// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// ------ //
// tft_io //
// ------ //

#define tft_io_wrap_target 0
#define tft_io_wrap 9

#define tft_io_offset_start_16 0u
#define tft_io_offset_start_8 7u

static const uint16_t tft_io_program_instructions[] = {
            //     .wrap_target
    0x98a0, //  0: pull   block           side 1     
    0x6028, //  1: out    x, 8                       
    0x7108, //  2: out    pins, 8         side 0 [1] 
    0xb842, //  3: nop                    side 1     
    0xa0e1, //  4: mov    osr, x                     
    0x7008, //  5: out    pins, 8         side 0     
    0x0000, //  6: jmp    0                          
    0x9aa0, //  7: pull   block           side 1 [2] 
    0x7108, //  8: out    pins, 8         side 0 [1] 
    0x0007, //  9: jmp    7                          
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program tft_io_program = {
    .instructions = tft_io_program_instructions,
    .length = 10,
    .origin = -1,
};

static inline pio_sm_config tft_io_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + tft_io_wrap_target, offset + tft_io_wrap);
    sm_config_set_sideset(&c, 2, true, false);
    return c;
}
#endif

