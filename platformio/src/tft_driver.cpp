

#include "tft_driver.h"

#include <stdio.h>

#include "pico/stdlib.h"
#include "pio_tft.h"

// Plain GPIO output pins.
#define TFT_RST_PIN 2  // Active low.
#define TFT_DC_PIN 28   // 1: data, 0: command.
#define TFT_BL_PIN 15  // Active high

// Outputs managed by PioTft.
#define TFT_D0_PIN 6  // First of 8 data pins, [GPIO_6, GPIO_13]
#define TFT_WR_PIN 22  // Active low

#define TFT_RST_HIGH gpio_set_mask(1ul << TFT_RST_PIN)
#define TFT_DC_HIGH gpio_set_mask(1ul << TFT_DC_PIN)
#define TFT_BL_HIGH gpio_set_mask(1ul << TFT_BL_PIN)

#define TFT_RST_LOW gpio_clr_mask(1ul << TFT_RST_PIN)
#define TFT_DC_LOW gpio_clr_mask(1ul << TFT_DC_PIN)
#define TFT_BL_LOW gpio_clr_mask(1ul << TFT_BL_PIN)

// // Assuming landscape mode per memory access command 0x36.
#define WIDTH 480
#define HEIGHT 320

#define ILI9488_SLPOUT 0x11

#define ILI9488_DISPON 0x29

#define ILI9488_CASET 0x2A
#define ILI9488_PASET 0x2B
#define ILI9488_RAMWR 0x2C
#define ILI9488_RAMRD 0x2E

namespace tft_driver {

constexpr uint16_t PIO_CLOCK_DIV = 1;

//static PioTft pio_tft(TFT_D0_PIN, TFT_WR_PIN, PIO_CLOCK_DIV);

bool is_overrun() { return pio_tft::is_overrun(); }

inline void write_command_byte(uint8_t c) {
  // This also flushes any pending writes.
  pio_tft::set_mode_single_byte();
  TFT_DC_LOW;
  pio_tft::write(c);
  // Prepear for data bytes that will follow
  pio_tft::flush();
  TFT_DC_HIGH;
}

// Assuming already in 8 bits data mode.
inline void write_data_byte(uint8_t c) {
  pio_tft::write(c);
  // No need to flush. Ok to data bytes being queued.
}

void begin() {
  // A mask with all gpio output pins we use.
  constexpr uint kOutputMask =
      1ul << TFT_RST_PIN | 1ul << TFT_DC_PIN | 1ul << TFT_BL_PIN;

  gpio_init_mask(kOutputMask);

  // Start with backlight non active, efore we set it as an
  // output, to avoid startup flicker.
  TFT_BL_LOW;

  gpio_set_dir_out_masked(kOutputMask);

 pio_tft::init(TFT_D0_PIN, TFT_WR_PIN, PIO_CLOCK_DIV);

  //pio_tft::begin();

  sleep_ms(5);
  TFT_RST_LOW;
  sleep_ms(20);
  TFT_RST_HIGH;
  sleep_ms(150);

  write_command_byte(0xE0);
  write_data_byte(0x00);
  write_data_byte(0x03);
  write_data_byte(0x09);
  write_data_byte(0x08);
  write_data_byte(0x16);
  write_data_byte(0x0A);
  write_data_byte(0x3F);
  write_data_byte(0x78);
  write_data_byte(0x4C);
  write_data_byte(0x09);
  write_data_byte(0x0A);
  write_data_byte(0x08);
  write_data_byte(0x16);
  write_data_byte(0x1A);
  write_data_byte(0x0F);

  write_command_byte(0XE1);
  write_data_byte(0x00);
  write_data_byte(0x16);
  write_data_byte(0x19);
  write_data_byte(0x03);
  write_data_byte(0x0F);
  write_data_byte(0x05);
  write_data_byte(0x32);
  write_data_byte(0x45);
  write_data_byte(0x46);
  write_data_byte(0x04);
  write_data_byte(0x0E);
  write_data_byte(0x0D);
  write_data_byte(0x35);
  write_data_byte(0x37);
  write_data_byte(0x0F);

  write_command_byte(0XC0);  // Power Control 1
  write_data_byte(0x17);     // Vreg1out
  write_data_byte(0x15);     // Verg2out

  write_command_byte(0xC1);  // Power Control 2
  write_data_byte(0x41);     // VGH,VGL

  write_command_byte(0xC5);  // Power Control 3
  write_data_byte(0x00);
  write_data_byte(0x12);  // Vcom
  write_data_byte(0x80);

  write_command_byte(0x36);  // Memory Access
  write_data_byte(0xe8);     // landscape mode. Swapping and mirroring x, y.

  // NOTE: For 16bit parallel transfer, IM jumpers need to be set
  // as 010.
  write_command_byte(0x3A);  // Interface Pixel Format
  write_data_byte(0x55);     // 16 bit

  write_command_byte(0XB0);  // Interface Mode Control
  write_data_byte(0x80);     // SDO NOT USE

  write_command_byte(0xB1);  // Frame rate
  write_data_byte(0xA0);     // 60Hz

  write_command_byte(0xB4);  // Display Inversion Control
  write_data_byte(0x02);     // 2-dot

  write_command_byte(
      0XB6);  // Display Function Control  RGB/MCU Interface Control

  write_data_byte(0x02);  // MCU
  write_data_byte(0x02);  // Source,Gate scan direction

  write_command_byte(0XE9);  // Set Image Function
  write_data_byte(0x00);     // Disable 24 bit data

  write_command_byte(0xF7);  // Adjust Control
  write_data_byte(0xA9);
  write_data_byte(0x51);
  write_data_byte(0x2C);
  write_data_byte(0x82);  // D7 stream, loose

  write_command_byte(ILI9488_SLPOUT);  // Exit Sleep
  sleep_ms(120);
  write_command_byte(ILI9488_DISPON);  // Display on

  fill_rect(0, 0, 479, 319, 0x1234);
  sleep_ms(50);
  TFT_BL_HIGH;  // Backlight on
}

// This is followed by a stream of pixels to render in this
// rectangle.
void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  write_command_byte(ILI9488_CASET);  // Column addr set
  write_data_byte(x0 >> 8);
  write_data_byte(x0 & 0xFF);  // XSTART
  write_data_byte(x1 >> 8);
  write_data_byte(x1 & 0xFF);  // XEND

  write_command_byte(ILI9488_PASET);  // Row addr set
  write_data_byte(y0 >> 8);
  write_data_byte(y0 & 0xff);  // YSTART
  write_data_byte(y1 >> 8);
  write_data_byte(y1 & 0xff);  // YEND

  // Should follow by pixels.
  write_command_byte(ILI9488_RAMWR);  // write to RAM
  pio_tft::set_mode_double_byte();
  TFT_DC_HIGH;
}

void fill_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
               uint16_t color) {
  const int32_t w_pixels = x2 - x1 + 1;
  const int32_t h_pixels = y2 - y1 + 1;
  const uint32_t pixels_count = w_pixels * h_pixels;

  setAddrWindow(x1, y1, x2, y2);
  pio_tft::multi_write(color, pixels_count);
}

}  // namespace tft_driver
