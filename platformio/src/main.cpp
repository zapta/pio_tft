#include <stdio.h>

#include "pico/stdlib.h"
#include "pio_tft.h"

constexpr uint8_t TFT_WR_PIN = 15;
constexpr uint8_t TFT_DC_PIN = 8;
constexpr uint8_t TFT_D0_PIN = 0;
constexpr uint8_t TFT_CLK_DIV = 1;

static PioTft tft_io(TFT_WR_PIN, TFT_DC_PIN, TFT_D0_PIN, TFT_CLK_DIV);

constexpr int kBfrSize = 100000;
static uint8_t bfr[kBfrSize];

static uint16_t color_map[256];

int main() {
  stdio_init_all();
  tft_io.begin();

  // tft_io.test();

  for (int i = 0; i < 256; i++) {
    color_map[i] = 0xaa55;
  }

  tft_io.set_pixels_mode();
  // tft_io.set_byte_data_mode();
  for (;;) {
    // With color mapping.
    tft_io.write_pixels(bfr, kBfrSize, color_map);
  }
}
