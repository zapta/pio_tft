#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "tft_driver.h"

// This program uses 8 bits data path. Make sure to wire
// ILI9488 IM2=1, IM1=1, IM0=0;

#define COLOR(r8, g8, b8)                  \
  ((uint16_t)(((b8) >> 3) & 0x1FU) << 11 | \
   (uint16_t)(((g8) >> 2) & 0x3FU) << 5 | (uint16_t)(((r8) >> 3) & 0x1FU))

constexpr uint NUM_COLORS = 100;
static uint16_t colors[NUM_COLORS];

void rand_range(uint16_t limit, uint16_t* a, uint16_t* b) {
  uint16_t t1 = ((uint32_t)rand()) % limit;
  uint16_t t2 = ((uint32_t)rand()) % limit;
  if (t1 < t2) {
    *a = t1;
    *b = t2;
  } else {
    // Ok to have t1 == t2;
    *a = t2;
    *b = t1;
  }
}

uint32_t millis() { return to_ms_since_boot(get_absolute_time()); }

int main() {
  stdio_init_all();
  tft_driver::begin();

  for (int i = 0; i < NUM_COLORS; i++) {
    colors[i] = COLOR(rand() % 256, rand() % 256, rand() % 256);
  }

  uint32_t rects = 0;
  uint32_t pixels = 0;
  uint32_t start_millis = millis();
  for (uint i = 0;; i += 1) {
    uint16_t x1, x2;
    uint16_t y1, y2;
    rand_range(480, &x1, &x2);
    rand_range(320, &y1, &y2);
    tft_driver::fill_rect(x1, y1, x2, y2, colors[i % NUM_COLORS]);

    // Track stats.
    rects++;
    pixels += (1 + x2 - x1) * (1 + y2 - y1);
    const uint32_t elapsed_millis = millis() - start_millis;
    if (elapsed_millis >= 1000) {
      const double screens = pixels / (float)(320 * 480);
      printf(
          "Rects: %lu, pixels: %lu, millis: %lu, screens: %5.2f overrun: %d\n",
          rects, pixels, elapsed_millis, screens, tft_driver::is_overrun());
      sleep_ms(1000);
      rects = 0;
      pixels = 0;
      start_millis = millis();
    }
  }
}
