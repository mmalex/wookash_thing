#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "audio_i2s.pio.h"
#include "ws2812.pio.h"

// screen SPI / gpio
#define SCREEN_PIN_SCL 10
#define SCREEN_PIN_SDA 11
#define SCREEN_PIN_RES 12
#define SCREEN_PIN_DC 13
#define SCREEN_PIN_BLK 14
#define SCREEN_SPI spi1

// audio i2s
#define AUDIO_PIN_DATA 5
#define AUDIO_PIN_BCLK 6
#define AUDIO_PIN_LRCLK 7
#define AUDIO_PIO pio0
#define AUDIO_PIO_FUNCTION GPIO_FUNC_PIO0

// ws2812
#define LEDS_PIN 28
#define LEDS_PIO pio1
#define LEDS_PIO_FUNCTION GPIO_FUNC_PIO1

// i2c for accelerometer
#define ACCEL_PIN_SDA 0
#define ACCEL_PIN_SCL 1
#define ACCEL_I2C i2c0
#define ACCEL_ADDR 0x19 // 0011001b

#define SOFTPOWER_PIN 23
#define BUTTON_PINS 24
    // buttons are:
    // 3 0 aka 8 1
    // 2 1     4 2
    // leds are:
    // 0 1
    // 3 2
    #define BTN_BIT_TL 8
    #define BTN_BIT_TR 1
    #define BTN_BIT_BL 4
    #define BTN_BIT_BR 2
    #define LED_IDX_TL 0
    #define LED_IDX_TR 1
    #define LED_IDX_BL 3
    #define LED_IDX_BR 2

int buttons_get(void) { return ((gpio_get_all() >> BUTTON_PINS) & 15) ^ 15; }

void buttons_init(void) {
  for (int i = 0; i < 4; ++i) {
    gpio_init(BUTTON_PINS + i);
    gpio_set_dir(BUTTON_PINS + i, GPIO_IN);
    gpio_pull_up(BUTTON_PINS + i);
  }
}

// CTRL1 - 20h - ODR[4], MODE[2], LP_MODE[2] -> 0101=100hz, 0110=200hz
// mode 01 is high perf; so we want 0b01010100
// CTRL2 - 21h - BOOT RESET 0 CS_PU_DISC BDU IF_ADD_INC I2C_DISABLE SIM
// we want 0b00000100
#define ACCEL_WHOAMI_ADDR 0x0f
#define ACCEL_CTRL1_ADDR 0x20
#define ACCEL_CTRL2_ADDR 0x21
#define ACCEL_DATA_ADDR 0x28
#define ACCEL_WHOAMI 0x44
#define ACCEL_CTRL1 0b01010100
#define ACCEL_CTRL2 0b00000100

static bool have_accel = false;
static int16_t accel_data_raw[3] = {0};

static int audio_dma_chan = -1;
#define AUDIO_BUFFER_SIZE_SH 9
#define AUDIO_BUFFER_SIZE (1 << AUDIO_BUFFER_SIZE_SH)
static int16_t audio_buf[2][AUDIO_BUFFER_SIZE] __attribute__((aligned(AUDIO_BUFFER_SIZE*2)));
static volatile int audio_buf_idx = 0;

#define BOUNCE_VOL 16000
static int vol = 0;

static void __isr audio_dma_irq0_handler(void) {
  dma_hw->ints0 = 1u << audio_dma_chan;
  audio_buf_idx ^= 1;
  dma_channel_set_read_addr(audio_dma_chan, audio_buf[audio_buf_idx], false);
  dma_channel_set_trans_count(audio_dma_chan, AUDIO_BUFFER_SIZE, true);
  int16_t *dst = audio_buf[audio_buf_idx^1];

  for (int i = 0; i < AUDIO_BUFFER_SIZE; ++i) {
    if (vol>0) { vol=(vol*255)/256; if (vol<2) vol=0; }
    dst[i] = (((rand()&0x7fff)-16384) * vol) >> 15;
  }
}

static int accel_i2c_write(const void *mem, int count) {
  return i2c_write_blocking_until(ACCEL_I2C, ACCEL_ADDR, (const uint8_t *)mem, count, false, make_timeout_time_us(100000));
}
static int accel_i2c_read(void *mem, int count) {
  return i2c_read_blocking_until(ACCEL_I2C, ACCEL_ADDR, (uint8_t *)mem, count, false, make_timeout_time_us(100000));
}
static int accel_i2c_write_reg(int reg, int val) {
  uint8_t data[2] = {reg, val};
  return accel_i2c_write(data, 2);
}
static int accel_i2c_read_regs(int reg, int count, uint8_t *buf) {
  uint8_t data[1] = {reg};
  accel_i2c_write(data, 1);
  return accel_i2c_read(buf, count);
}
static int accel_i2c_read_reg(int reg) {
  uint8_t data[2] = {reg, 0};
  accel_i2c_read_regs(reg, 1, data + 1);
  return data[1];
}

// ST7789 command set
#define CMD_SWRESET 0x01
#define CMD_SLPOUT 0x11
#define CMD_COLMOD 0x3A
#define CMD_MADCTL 0x36
#define CMD_CASET 0x2A
#define CMD_RASET 0x2B
#define CMD_RAMWR 0x2C
#define CMD_INVON 0x21
#define CMD_INVOFF 0x20
#define CMD_NORON 0x13
#define CMD_DISPON 0x29

static void spi_write_byte(uint8_t b) { spi_write_blocking(SCREEN_SPI, &b, 1); }

static void lcd_write_cmd(uint8_t cmd) {
  gpio_put(SCREEN_PIN_DC, 0);
  spi_write_byte(cmd);
}

static void lcd_write_data(uint8_t data) {
  gpio_put(SCREEN_PIN_DC, 1);
  spi_write_byte(data);
}

static void lcd_write_data16(uint16_t data) {
  lcd_write_data((uint8_t)(data >> 8));
  lcd_write_data((uint8_t)(data & 0xFF));
}

static void init_palette(void);

static int spi_dma_chan;
static dma_channel_config spi_dma_cfg;

static uint8_t screen[120][120];
static uint16_t palette[256];

static void lcd_reset(void) {
  gpio_put(SCREEN_PIN_RES, 0);
  sleep_ms(10);
  gpio_put(SCREEN_PIN_RES, 1);
  sleep_ms(150);
}

static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  lcd_write_cmd(CMD_CASET);
  lcd_write_data16(x0);
  lcd_write_data16(x1);

  lcd_write_cmd(CMD_RASET);
  lcd_write_data16(y0);
  lcd_write_data16(y1);

  lcd_write_cmd(CMD_RAMWR);
}

static void screen_init(void) {
  spi_init(SCREEN_SPI, 100 * 1000 * 1000);
  spi_set_format(SCREEN_SPI, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
  gpio_set_function(SCREEN_PIN_SCL, GPIO_FUNC_SPI);
  gpio_set_function(SCREEN_PIN_SDA, GPIO_FUNC_SPI);

  gpio_init(SCREEN_PIN_RES);
  gpio_init(SCREEN_PIN_DC);
  gpio_init(SCREEN_PIN_BLK);

  gpio_set_dir(SCREEN_PIN_RES, GPIO_OUT);
  gpio_set_dir(SCREEN_PIN_DC, GPIO_OUT);
  gpio_set_dir(SCREEN_PIN_BLK, GPIO_OUT);

  gpio_put(SCREEN_PIN_DC, 0);
  gpio_put(SCREEN_PIN_BLK, 0); // backlight off until display is on

  spi_dma_chan = dma_claim_unused_channel(true);

  spi_dma_cfg = dma_channel_get_default_config(spi_dma_chan);
  channel_config_set_transfer_data_size(&spi_dma_cfg, DMA_SIZE_8);
  channel_config_set_dreq(&spi_dma_cfg, spi_get_dreq(SCREEN_SPI, true));
  channel_config_set_read_increment(&spi_dma_cfg, true);
  channel_config_set_write_increment(&spi_dma_cfg, false);
  dma_channel_configure(spi_dma_chan, &spi_dma_cfg, &spi_get_hw(SCREEN_SPI)->dr, NULL, 0, false);

  sleep_ms(40);
  lcd_reset();

  lcd_write_cmd(CMD_SWRESET);
  sleep_ms(150);

  lcd_write_cmd(CMD_SLPOUT);
  sleep_ms(10);

  lcd_write_cmd(CMD_COLMOD);
  lcd_write_data(0x55); // 16-bit color
  sleep_ms(10);

  lcd_write_cmd(CMD_MADCTL);
  lcd_write_data(0x08); // matches Adafruit generic init
  sleep_ms(10);

  lcd_write_cmd(CMD_INVON);
  sleep_ms(10);

  lcd_write_cmd(CMD_NORON);
  sleep_ms(10);

  lcd_write_cmd(CMD_DISPON);
  sleep_ms(10);

  gpio_put(SCREEN_PIN_BLK, 1); // backlight on

  init_palette();
}

static void init_palette(void) {
  for (uint16_t i = 0; i < 256; i++) {
    uint16_t r5 = i >> 3;
    uint16_t g6 = 0;
    uint16_t b5 = 0;
    uint16_t color = (uint16_t)((b5 << 11) | (g6 << 5) | r5);
    if (i == 255) color = 0xffff;
    palette[i] = (color >> 8) + (color << 8);
  }
}

static void lcd_fill_scanline(uint32_t *dst, uint16_t y_src) {
  for (uint16_t x = 0; x < 120; x++) {
    uint16_t color = palette[screen[y_src][x]];
    *dst++ = (uint32_t)color | ((uint32_t)color << 16);
  }
}

static void screen_flip(void) {
  const uint16_t width = 240;
  const uint16_t height = 240;
  static uint32_t scanline[2][120];
  uint32_t *front = scanline[0];
  uint32_t *back = scanline[1];
  const uint32_t line_bytes = width * 2;

  lcd_set_window(0, 0, width - 1, height - 1);
  gpio_put(SCREEN_PIN_DC, 1);

  lcd_fill_scanline(front, 0);
  dma_channel_set_read_addr(spi_dma_chan, front, false);
  dma_channel_set_trans_count(spi_dma_chan, line_bytes, true);

  for (uint16_t y_src = 0; y_src < height / 2; y_src++) {
    if (y_src + 1 < height / 2) {
      lcd_fill_scanline(back, (uint16_t)(y_src + 1));
    }

    dma_channel_wait_for_finish_blocking(spi_dma_chan);
    dma_channel_set_read_addr(spi_dma_chan, front, false);
    dma_channel_set_trans_count(spi_dma_chan, line_bytes, true);
    dma_channel_wait_for_finish_blocking(spi_dma_chan);

    if (y_src + 1 < height / 2) {
      uint32_t *tmp = front;
      front = back;
      back = tmp;
      dma_channel_set_read_addr(spi_dma_chan, front, false);
      dma_channel_set_trans_count(spi_dma_chan, line_bytes, true);
    }
  }
}

void leds_init(void) {
  pio_sm_claim(LEDS_PIO, 0);
  gpio_set_function(LEDS_PIN, LEDS_PIO_FUNCTION);
  uint offset = pio_add_program(LEDS_PIO, &ws2812_program);
  ws2812_program_init(LEDS_PIO, 0, offset, LEDS_PIN, 800000, false);
}

#define RGB(r,g,b)  (((g)<<24)|((r)<<16)|((b)<<8))

static void leds_set(uint32_t grb0[4]) {
  pio_sm_put_blocking(LEDS_PIO, 0, grb0[0]);
  pio_sm_put_blocking(LEDS_PIO, 0, grb0[1]);
  pio_sm_put_blocking(LEDS_PIO, 0, grb0[2]);
  pio_sm_put_blocking(LEDS_PIO, 0, grb0[3]);
}

void accel_init(void) {
  i2c_init(ACCEL_I2C, 400 * 1000);
  gpio_set_function(ACCEL_PIN_SDA, GPIO_FUNC_I2C);
  gpio_set_function(ACCEL_PIN_SCL, GPIO_FUNC_I2C);

  accel_i2c_write_reg(ACCEL_CTRL1_ADDR, ACCEL_CTRL1);
  accel_i2c_write_reg(ACCEL_CTRL2_ADDR, ACCEL_CTRL2);
  int id = accel_i2c_read_reg(ACCEL_WHOAMI_ADDR);
  have_accel = id == ACCEL_WHOAMI;
  printf("Accelerometer ID: %02x expected: %02x\n", id, ACCEL_WHOAMI);
}

void accel_read(void) {
  if (have_accel) {
    accel_i2c_read_regs(ACCEL_DATA_ADDR, 6, (uint8_t *)accel_data_raw);
    printf("Acceleration: %5d %5d %5d\n", accel_data_raw[0], accel_data_raw[1], accel_data_raw[2]);
  } else {
    accel_data_raw[0] = 0;
    accel_data_raw[1] = 0;
    accel_data_raw[2] = 0;
  }
}

void audio_init(void) {
  // audio
  gpio_set_function(AUDIO_PIN_DATA, AUDIO_PIO_FUNCTION);
  gpio_set_function(AUDIO_PIN_BCLK, AUDIO_PIO_FUNCTION);
  gpio_set_function(AUDIO_PIN_LRCLK, AUDIO_PIO_FUNCTION);

  pio_sm_claim(AUDIO_PIO, 0);
  uint offset = pio_add_program(AUDIO_PIO, &audio_i2s_program);
  audio_i2s_program_init(AUDIO_PIO, 0, offset, AUDIO_PIN_DATA, AUDIO_PIN_BCLK);
  static_assert(AUDIO_PIN_BCLK < AUDIO_PIN_LRCLK, "BCLK must be less than LRCLK, otherwise use the swapped program");

  uint32_t system_clock_frequency = clock_get_hz(clk_sys);
  uint32_t sample_freq = 16000;
  uint32_t divider = system_clock_frequency * 4 / sample_freq; // avoid arithmetic overflow
  pio_sm_set_clkdiv_int_frac(AUDIO_PIO, 0, divider >> 8u, divider & 0xffu);

  pio_sm_set_enabled(AUDIO_PIO, 0, true);

  audio_dma_chan = dma_claim_unused_channel(true);
  dma_channel_config audio_dma_cfg = dma_channel_get_default_config(audio_dma_chan);
  channel_config_set_transfer_data_size(&audio_dma_cfg, DMA_SIZE_16);
  channel_config_set_read_increment(&audio_dma_cfg, true);
  channel_config_set_write_increment(&audio_dma_cfg, false);
  channel_config_set_dreq(&audio_dma_cfg, pio_get_dreq(AUDIO_PIO, 0, true));
  dma_channel_set_irq0_enabled(audio_dma_chan, true);
  irq_set_exclusive_handler(DMA_IRQ_0, audio_dma_irq0_handler);
  irq_set_enabled(DMA_IRQ_0, true);
  dma_channel_configure(
      audio_dma_chan,
      &audio_dma_cfg,
      &AUDIO_PIO->txf[0],
      audio_buf[0],
      AUDIO_BUFFER_SIZE,
      true);


}

static inline int maxi(int a, int b) { return a > b ? a : b; }

int main(void) {
  gpio_init(SOFTPOWER_PIN);
  gpio_set_dir(SOFTPOWER_PIN, GPIO_OUT);
  gpio_put(SOFTPOWER_PIN, 0); // 1=off, 0=on
  
  void stdio_rtt_init(void);
  stdio_rtt_init();

  screen_init();
  audio_init();
  leds_init();
  accel_init();
  buttons_init();
  sleep_ms(100);
  uint32_t leds[4] = {}; // grb order
  leds_set(leds);

  static int ballx = 60 * 256;
  static int bally = 60 * 256;
  static int vx = 0;
  static int vy = 0;

  uint32_t last_time_button_press = time_us_32();
  uint32_t tr_btn_not_down_time = time_us_32();
  while (true) {

    int btns = buttons_get();
    uint32_t now = time_us_32();
    if (btns) last_time_button_press = now;
    if (!(btns & BTN_BIT_TR)) tr_btn_not_down_time = now;
    if ((now - last_time_button_press) > 10*1000000 || (now - tr_btn_not_down_time) > 2*1000000) {
        // power down
        gpio_put(SOFTPOWER_PIN, 1);
        sleep_ms(100);
        // if we're still here, it means we must be on usb power...
    }
    memset(leds, 0, sizeof(leds));
    if (btns & BTN_BIT_TL) leds[LED_IDX_TL] = RGB(255,0,0);
    if (btns & BTN_BIT_TR) leds[LED_IDX_TR] = RGB(0,255,0);
    if (btns & BTN_BIT_BL) leds[LED_IDX_BL] = RGB(0,0,255);
    if (btns & BTN_BIT_BR) leds[LED_IDX_BR] = RGB(200,128,0);
    
    leds_set(leds);

    int ax = accel_data_raw[0];
    int ay = accel_data_raw[1];
    vx = (vx * 255) / 256;
    vy = (vy * 255) / 256;
    vx += (ay / 64);
    vy -= (ax / 64);
    ballx += vx;
    bally += vy;
    int bounce = 200;
    #define VOLUME 10
    if (ballx < 15 * 256) {
      ballx = 15 * 256;
      vol = maxi(vol, abs(vx) * VOLUME);
      vx = (abs(vx) * bounce) >> 8;
    }
    if (ballx > (120 - 15) * 256) {
      ballx = (120 - 15) * 256;
      vol = maxi(vol, abs(vx) * VOLUME);
      vx = -(abs(vx) * bounce) >> 8;
    }
    if (bally < 15 * 256) {
      bally = 15 * 256;
      vol = maxi(vol, abs(vy) * VOLUME);
      vy = (abs(vy) * bounce) >> 8;
    }
    if (bally > (120 - 15) * 256) {
      bally = (120 - 15) * 256;
      vol = maxi(vol, abs(vy) * VOLUME);
      vy = -(abs(vy) * bounce) >> 8;
    }
    for (int y = 0; y < 120; y++) {
      int yy = (y * 256 - bally);
      yy *= yy;
      for (int x = 0; x < 120; x++) {
        int xx = (x * 256 - ballx);
        xx *= xx;
        int dd = (xx + yy);
        int c = screen[y][x];
        if (c > 4)
          c -= 4;
        else
          c = 0;
        if (dd < 15 * 15 * 65536) c = 255;
        screen[y][x] = c;
      }
    }
    screen_flip();
    accel_read();
  }
}
