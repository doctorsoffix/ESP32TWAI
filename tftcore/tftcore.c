/*
 * tftcore — High-performance ILI9341 framebuffer engine for MicroPython
 * ESP32-S3 with PSRAM + DMA SPI
 *
 * Features:
 *   - 240x320 16-bit framebuffer in PSRAM
 *   - Dirty-rect tracking (only flush changed areas)
 *   - DMA SPI transfers (CPU free during flush)
 *   - fill_rect, hline, vline, draw_pixel
 *   - 5x7 built-in font
 *   - Gradient fills
 *   - Circle / rounded rect
 *   - Alpha blend (50%)
 *   - Python-callable via MicroPython C module API
 *
 * Pins (hardcoded for AKKU POWER BMS Monitor):
 *   SPI: SCK=3, MOSI=4 (MISO not used)
 *   TFT: CS=10, DC=5, RST=6
 */

#include <string.h>
#include <stdio.h>
#include "py/runtime.h"
#include "py/obj.h"
#include "py/mphal.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ═══════ Config ═══════
#define TFT_W       240
#define TFT_H       320
#define TFT_PIXELS  (TFT_W * TFT_H)
#define TFT_BUFSIZE (TFT_PIXELS * 2)  // 153600 bytes

#define PIN_SCK     3
#define PIN_MOSI    4
#define PIN_CS      10
#define PIN_DC      5
#define PIN_RST     6

#define SPI_FREQ    40000000  // 40 MHz
#define DMA_CHAN     SPI_DMA_CH_AUTO

// ═══════ State ═══════
static uint16_t *framebuf = NULL;       // Main framebuffer in PSRAM
static spi_device_handle_t spi_dev = NULL;
static bool initialized = false;

// Dirty rect tracking
static int dirty_x0, dirty_y0, dirty_x1, dirty_y1;
static bool dirty = false;

static void dirty_mark(int x0, int y0, int x1, int y1) {
    if (!dirty) {
        dirty_x0 = x0; dirty_y0 = y0;
        dirty_x1 = x1; dirty_y1 = y1;
        dirty = true;
    } else {
        if (x0 < dirty_x0) dirty_x0 = x0;
        if (y0 < dirty_y0) dirty_y0 = y0;
        if (x1 > dirty_x1) dirty_x1 = x1;
        if (y1 > dirty_y1) dirty_y1 = y1;
    }
}

static void dirty_reset(void) {
    dirty = false;
    dirty_x0 = TFT_W; dirty_y0 = TFT_H;
    dirty_x1 = 0; dirty_y1 = 0;
}

// ═══════ Low-level SPI ═══════
static void spi_cmd(uint8_t cmd) {
    gpio_set_level(PIN_DC, 0);
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    spi_device_polling_transmit(spi_dev, &t);
}

static void spi_data(const uint8_t *data, int len) {
    if (len == 0) return;
    gpio_set_level(PIN_DC, 1);
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data,
    };
    spi_device_polling_transmit(spi_dev, &t);
}

static void spi_data_dma(const uint8_t *data, int len) {
    if (len == 0) return;
    gpio_set_level(PIN_DC, 1);
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data,
    };
    spi_device_transmit(spi_dev, &t);  // DMA, blocks until done
}

static void set_window(int x0, int y0, int x1, int y1) {
    uint8_t ca[4] = {x0 >> 8, x0 & 0xFF, x1 >> 8, x1 & 0xFF};
    uint8_t ra[4] = {y0 >> 8, y0 & 0xFF, y1 >> 8, y1 & 0xFF};
    spi_cmd(0x2A); spi_data(ca, 4);
    spi_cmd(0x2B); spi_data(ra, 4);
    spi_cmd(0x2C);
}

// ═══════ TFT Init ═══════
static void tft_hw_init(void) {
    // GPIO
    gpio_set_direction(PIN_CS, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_CS, 1);

    // SPI bus
    spi_bus_config_t bus = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = TFT_BUFSIZE,
    };
    spi_bus_initialize(SPI2_HOST, &bus, DMA_CHAN);

    spi_device_interface_config_t dev = {
        .clock_speed_hz = SPI_FREQ,
        .mode = 0,
        .spics_io_num = PIN_CS,
        .queue_size = 1,
        .pre_cb = NULL,
    };
    spi_bus_add_device(SPI2_HOST, &dev, &spi_dev);

    // Reset
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    // Init commands
    spi_cmd(0x01); vTaskDelay(pdMS_TO_TICKS(150));  // SW reset
    spi_cmd(0x11); vTaskDelay(pdMS_TO_TICKS(300));  // Sleep out
    uint8_t px = 0x55;
    spi_cmd(0x3A); spi_data(&px, 1);  // 16-bit color
    uint8_t mx = 0x48;
    spi_cmd(0x36); spi_data(&mx, 1);  // MADCTL
    spi_cmd(0x29); vTaskDelay(pdMS_TO_TICKS(50));  // Display ON
}

// ═══════ Framebuffer Operations ═══════

static inline void fb_pixel(int x, int y, uint16_t c) {
    if (x >= 0 && x < TFT_W && y >= 0 && y < TFT_H) {
        // Store as big-endian for direct SPI transfer
        framebuf[y * TFT_W + x] = (c >> 8) | (c << 8);
    }
}

static inline uint16_t fb_get(int x, int y) {
    uint16_t v = framebuf[y * TFT_W + x];
    return (v >> 8) | (v << 8);  // swap back
}

static void fb_fill_rect(int x, int y, int w, int h, uint16_t c) {
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > TFT_W) w = TFT_W - x;
    if (y + h > TFT_H) h = TFT_H - y;
    if (w <= 0 || h <= 0) return;

    uint16_t swapped = (c >> 8) | (c << 8);
    for (int row = y; row < y + h; row++) {
        uint16_t *p = &framebuf[row * TFT_W + x];
        for (int col = 0; col < w; col++) {
            p[col] = swapped;
        }
    }
    dirty_mark(x, y, x + w - 1, y + h - 1);
}

static void fb_hline(int x, int y, int w, uint16_t c) {
    fb_fill_rect(x, y, w, 1, c);
}

static void fb_vline(int x, int y, int h, uint16_t c) {
    fb_fill_rect(x, y, 1, h, c);
}

static void fb_line(int x0, int y0, int x1, int y1, uint16_t c) {
    int dx = x1 > x0 ? x1 - x0 : x0 - x1;
    int dy = y1 > y0 ? y1 - y0 : y0 - y1;
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    while (1) {
        fb_pixel(x0, y0, c);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx) { err += dx; y0 += sy; }
    }
    int mx0 = x0 < x1 ? x0 : x1, my0 = y0 < y1 ? y0 : y1;
    int mx1 = x0 > x1 ? x0 : x1, my1 = y0 > y1 ? y0 : y1;
    dirty_mark(mx0, my0, mx1, my1);
}

static void fb_circle(int cx, int cy, int r, uint16_t c, bool fill) {
    int x = 0, y = r, d = 3 - 2 * r;
    while (x <= y) {
        if (fill) {
            fb_hline(cx - x, cy + y, 2 * x + 1, c);
            fb_hline(cx - x, cy - y, 2 * x + 1, c);
            fb_hline(cx - y, cy + x, 2 * y + 1, c);
            fb_hline(cx - y, cy - x, 2 * y + 1, c);
        } else {
            fb_pixel(cx+x,cy+y,c); fb_pixel(cx-x,cy+y,c);
            fb_pixel(cx+x,cy-y,c); fb_pixel(cx-x,cy-y,c);
            fb_pixel(cx+y,cy+x,c); fb_pixel(cx-y,cy+x,c);
            fb_pixel(cx+y,cy-x,c); fb_pixel(cx-y,cy-x,c);
        }
        if (d < 0) d += 4 * x + 6;
        else { d += 4 * (x - y) + 10; y--; }
        x++;
    }
    dirty_mark(cx - r, cy - r, cx + r, cy + r);
}

static void fb_rounded_rect(int x, int y, int w, int h, int r, uint16_t c, bool fill) {
    if (r > w/2) r = w/2;
    if (r > h/2) r = h/2;
    if (fill) {
        fb_fill_rect(x + r, y, w - 2*r, h, c);
        fb_fill_rect(x, y + r, r, h - 2*r, c);
        fb_fill_rect(x + w - r, y + r, r, h - 2*r, c);
        // Corners
        int cx0 = x+r, cy0 = y+r;
        int cx1 = x+w-r-1, cy1 = y+h-r-1;
        int px = 0, py = r, d = 3 - 2*r;
        while (px <= py) {
            fb_hline(cx0-px, cy0-py, px+(cx1-cx0)+px+1, c);
            fb_hline(cx0-py, cy0-px, py+(cx1-cx0)+py+1, c);
            fb_hline(cx0-px, cy1+py, px+(cx1-cx0)+px+1, c);
            fb_hline(cx0-py, cy1+px, py+(cx1-cx0)+py+1, c);
            if (d<0) d+=4*px+6; else {d+=4*(px-py)+10;py--;}
            px++;
        }
    } else {
        fb_hline(x+r, y, w-2*r, c);
        fb_hline(x+r, y+h-1, w-2*r, c);
        fb_vline(x, y+r, h-2*r, c);
        fb_vline(x+w-1, y+r, h-2*r, c);
        // Corner arcs
        int cx0=x+r, cy0=y+r, cx1=x+w-r-1, cy1=y+h-r-1;
        int px=0,py=r,d=3-2*r;
        while(px<=py){
            fb_pixel(cx1+px,cy0-py,c);fb_pixel(cx0-px,cy0-py,c);
            fb_pixel(cx1+py,cy0-px,c);fb_pixel(cx0-py,cy0-px,c);
            fb_pixel(cx1+px,cy1+py,c);fb_pixel(cx0-px,cy1+py,c);
            fb_pixel(cx1+py,cy1+px,c);fb_pixel(cx0-py,cy1+px,c);
            if(d<0)d+=4*px+6;else{d+=4*(px-py)+10;py--;}
            px++;
        }
    }
    dirty_mark(x, y, x+w-1, y+h-1);
}

static void fb_gradient_v(int x, int y, int w, int h, uint16_t c1, uint16_t c2) {
    // Vertical gradient from c1 (top) to c2 (bottom)
    int r1=(c1>>11)&0x1F, g1=(c1>>5)&0x3F, b1=c1&0x1F;
    int r2=(c2>>11)&0x1F, g2=(c2>>5)&0x3F, b2=c2&0x1F;
    for (int row = 0; row < h; row++) {
        int r = r1 + (r2-r1)*row/h;
        int g = g1 + (g2-g1)*row/h;
        int b = b1 + (b2-b1)*row/h;
        uint16_t c = (r<<11)|(g<<5)|b;
        fb_hline(x, y+row, w, c);
    }
}

static uint16_t blend50(uint16_t c1, uint16_t c2) {
    int r = (((c1>>11)&0x1F) + ((c2>>11)&0x1F)) >> 1;
    int g = (((c1>>5)&0x3F) + ((c2>>5)&0x3F)) >> 1;
    int b = ((c1&0x1F) + (c2&0x1F)) >> 1;
    return (r<<11)|(g<<5)|b;
}

// ═══════ Flush to Display ═══════

static void flush_rect(int x0, int y0, int x1, int y1) {
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= TFT_W) x1 = TFT_W - 1;
    if (y1 >= TFT_H) y1 = TFT_H - 1;
    if (x0 > x1 || y0 > y1) return;

    int w = x1 - x0 + 1;
    int h = y1 - y0 + 1;

    set_window(x0, y0, x1, y1);

    // Send row by row (each row is contiguous in framebuf)
    gpio_set_level(PIN_DC, 1);
    for (int row = y0; row <= y1; row++) {
        uint8_t *rowdata = (uint8_t *)&framebuf[row * TFT_W + x0];
        spi_transaction_t t = {
            .length = w * 16,
            .tx_buffer = rowdata,
        };
        spi_device_transmit(spi_dev, &t);
    }
}

static void flush_full(void) {
    set_window(0, 0, TFT_W - 1, TFT_H - 1);
    gpio_set_level(PIN_DC, 1);

    // Send in chunks (DMA max ~4KB per transaction is safe)
    uint8_t *buf = (uint8_t *)framebuf;
    int remaining = TFT_BUFSIZE;
    int chunk = 32000;  // ~32KB per DMA transfer
    while (remaining > 0) {
        int len = remaining > chunk ? chunk : remaining;
        spi_transaction_t t = {
            .length = len * 8,
            .tx_buffer = buf,
        };
        spi_device_transmit(spi_dev, &t);
        buf += len;
        remaining -= len;
    }
}

// ═══════ Built-in 5x7 Font ═══════
static const char FONT_CHARS[] = " !%+-./0123456789:ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
static const uint8_t FONT_DATA[] = {
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x5f,0x00,0x00,0x23,0x13,0x08,0x64,0x62,
    0x08,0x08,0x3e,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x00,0x60,0x60,0x00,0x00,
    0x20,0x10,0x08,0x04,0x02,0x3e,0x51,0x49,0x45,0x3e,0x00,0x42,0x7f,0x40,0x00,
    0x42,0x61,0x51,0x49,0x46,0x21,0x41,0x45,0x4b,0x31,0x18,0x14,0x12,0x7f,0x10,
    0x27,0x45,0x45,0x45,0x39,0x3c,0x4a,0x49,0x49,0x30,0x01,0x71,0x09,0x05,0x03,
    0x36,0x49,0x49,0x49,0x36,0x06,0x49,0x49,0x29,0x1e,0x00,0x36,0x36,0x00,0x00,
    0x7e,0x11,0x11,0x11,0x7e,0x7f,0x49,0x49,0x49,0x36,0x3e,0x41,0x41,0x41,0x22,
    0x7f,0x41,0x41,0x22,0x1c,0x7f,0x49,0x49,0x49,0x41,0x7f,0x09,0x09,0x09,0x01,
    0x3e,0x41,0x49,0x49,0x7a,0x7f,0x08,0x08,0x08,0x7f,0x00,0x41,0x7f,0x41,0x00,
    0x20,0x40,0x41,0x3f,0x01,0x7f,0x08,0x14,0x22,0x41,0x7f,0x40,0x40,0x40,0x40,
    0x7f,0x02,0x0c,0x02,0x7f,0x7f,0x04,0x08,0x10,0x7f,0x3e,0x41,0x41,0x41,0x3e,
    0x7f,0x09,0x09,0x09,0x06,0x3e,0x41,0x51,0x21,0x5e,0x7f,0x09,0x19,0x29,0x46,
    0x46,0x49,0x49,0x49,0x31,0x01,0x01,0x7f,0x01,0x01,0x3f,0x40,0x40,0x40,0x3f,
    0x1f,0x20,0x40,0x20,0x1f,0x3f,0x40,0x38,0x40,0x3f,0x63,0x14,0x08,0x14,0x63,
    0x07,0x08,0x70,0x08,0x07,0x61,0x51,0x49,0x45,0x43,0x20,0x54,0x54,0x54,0x78,
    0x7f,0x48,0x44,0x44,0x38,0x38,0x44,0x44,0x44,0x20,0x38,0x44,0x44,0x48,0x7f,
    0x38,0x54,0x54,0x54,0x18,0x08,0x7e,0x09,0x01,0x02,0x0c,0x52,0x52,0x52,0x3e,
    0x7f,0x08,0x04,0x04,0x78,0x00,0x44,0x7d,0x40,0x00,0x20,0x40,0x44,0x3d,0x00,
    0x7f,0x10,0x28,0x44,0x00,0x00,0x41,0x7f,0x40,0x00,0x7c,0x04,0x18,0x04,0x78,
    0x7c,0x08,0x04,0x04,0x78,0x38,0x44,0x44,0x44,0x38,0x7c,0x14,0x14,0x14,0x08,
    0x08,0x14,0x14,0x18,0x7c,0x7c,0x08,0x04,0x04,0x08,0x48,0x54,0x54,0x54,0x20,
    0x04,0x3f,0x44,0x40,0x20,0x3c,0x40,0x40,0x40,0x7c,0x1c,0x20,0x40,0x20,0x1c,
    0x3c,0x40,0x30,0x40,0x3c,0x44,0x28,0x10,0x28,0x44,0x0c,0x50,0x50,0x50,0x3c,
    0x44,0x64,0x54,0x4c,0x44
};

static void fb_text(int x, int y, const char *str, uint16_t fg, uint16_t bg) {
    int nchars = sizeof(FONT_CHARS) - 1;
    while (*str) {
        int idx = -1;
        for (int i = 0; i < nchars; i++) {
            if (FONT_CHARS[i] == *str) { idx = i; break; }
        }
        if (idx < 0) idx = 0;
        const uint8_t *glyph = &FONT_DATA[idx * 5];
        for (int col = 0; col < 5; col++) {
            uint8_t bits = glyph[col];
            for (int row = 0; row < 7; row++) {
                fb_pixel(x + col, y + row, (bits & (1 << row)) ? fg : bg);
            }
        }
        // Gap column
        for (int row = 0; row < 7; row++) {
            fb_pixel(x + 5, y + row, bg);
        }
        x += 6;
        str++;
    }
}

// ═══════ MicroPython Bindings ═══════

// tftcore.init()
static mp_obj_t tftcore_init(void) {
    if (initialized) return mp_const_none;

    // Allocate framebuffer in PSRAM
    framebuf = (uint16_t *)heap_caps_calloc(TFT_PIXELS, sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    if (!framebuf) {
        mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("PSRAM alloc failed"));
    }

    tft_hw_init();
    dirty_reset();
    initialized = true;
    printf("tftcore: init OK, framebuf=%p (%d bytes in PSRAM)\n", framebuf, TFT_BUFSIZE);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(tftcore_init_obj, tftcore_init);

// tftcore.fill(color) — fill entire screen
static mp_obj_t tftcore_fill(mp_obj_t color_obj) {
    uint16_t c = mp_obj_get_int(color_obj);
    fb_fill_rect(0, 0, TFT_W, TFT_H, c);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(tftcore_fill_obj, tftcore_fill);

// tftcore.rect(x, y, w, h, color)
static mp_obj_t tftcore_rect(size_t n_args, const mp_obj_t *args) {
    int x = mp_obj_get_int(args[0]);
    int y = mp_obj_get_int(args[1]);
    int w = mp_obj_get_int(args[2]);
    int h = mp_obj_get_int(args[3]);
    uint16_t c = mp_obj_get_int(args[4]);
    fb_fill_rect(x, y, w, h, c);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(tftcore_rect_obj, 5, 5, tftcore_rect);

// tftcore.line(x0, y0, x1, y1, color)
static mp_obj_t tftcore_line(size_t n_args, const mp_obj_t *args) {
    fb_line(mp_obj_get_int(args[0]), mp_obj_get_int(args[1]),
            mp_obj_get_int(args[2]), mp_obj_get_int(args[3]),
            mp_obj_get_int(args[4]));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(tftcore_line_obj, 5, 5, tftcore_line);

// tftcore.circle(cx, cy, r, color, fill=False)
static mp_obj_t tftcore_circle(size_t n_args, const mp_obj_t *args) {
    int cx = mp_obj_get_int(args[0]);
    int cy = mp_obj_get_int(args[1]);
    int r = mp_obj_get_int(args[2]);
    uint16_t c = mp_obj_get_int(args[3]);
    bool fill = (n_args > 4) ? mp_obj_is_true(args[4]) : false;
    fb_circle(cx, cy, r, c, fill);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(tftcore_circle_obj, 4, 5, tftcore_circle);

// tftcore.rrect(x, y, w, h, r, color, fill=False)
static mp_obj_t tftcore_rrect(size_t n_args, const mp_obj_t *args) {
    int x=mp_obj_get_int(args[0]), y=mp_obj_get_int(args[1]);
    int w=mp_obj_get_int(args[2]), h=mp_obj_get_int(args[3]);
    int r=mp_obj_get_int(args[4]);
    uint16_t c=mp_obj_get_int(args[5]);
    bool fill = (n_args > 6) ? mp_obj_is_true(args[6]) : false;
    fb_rounded_rect(x, y, w, h, r, c, fill);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(tftcore_rrect_obj, 6, 7, tftcore_rrect);

// tftcore.gradient(x, y, w, h, c1, c2)
static mp_obj_t tftcore_gradient(size_t n_args, const mp_obj_t *args) {
    fb_gradient_v(mp_obj_get_int(args[0]), mp_obj_get_int(args[1]),
                  mp_obj_get_int(args[2]), mp_obj_get_int(args[3]),
                  mp_obj_get_int(args[4]), mp_obj_get_int(args[5]));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(tftcore_gradient_obj, 6, 6, tftcore_gradient);

// tftcore.pixel(x, y, color)
static mp_obj_t tftcore_pixel(mp_obj_t x, mp_obj_t y, mp_obj_t c) {
    fb_pixel(mp_obj_get_int(x), mp_obj_get_int(y), mp_obj_get_int(c));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_3(tftcore_pixel_obj, tftcore_pixel);

// tftcore.text(x, y, string, fg, bg) — built-in 5x7 font
static mp_obj_t tftcore_text(size_t n_args, const mp_obj_t *args) {
    int x = mp_obj_get_int(args[0]);
    int y = mp_obj_get_int(args[1]);
    const char *str = mp_obj_str_get_str(args[2]);
    uint16_t fg = mp_obj_get_int(args[3]);
    uint16_t bg = mp_obj_get_int(args[4]);
    fb_text(x, y, str, fg, bg);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(tftcore_text_obj, 5, 5, tftcore_text);

// tftcore.blit(x, y, w, h, buf) — blit raw 16-bit pixel buffer (for custom fonts)
static mp_obj_t tftcore_blit(size_t n_args, const mp_obj_t *args) {
    int x = mp_obj_get_int(args[0]);
    int y = mp_obj_get_int(args[1]);
    int w = mp_obj_get_int(args[2]);
    int h = mp_obj_get_int(args[3]);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[4], &bufinfo, MP_BUFFER_READ);
    uint16_t *src = (uint16_t *)bufinfo.buf;
    int npix = w * h;
    if ((int)bufinfo.len < npix * 2) {
        mp_raise_ValueError(MP_ERROR_TEXT("buffer too small"));
    }
    for (int row = 0; row < h; row++) {
        if (y + row < 0 || y + row >= TFT_H) continue;
        for (int col = 0; col < w; col++) {
            if (x + col < 0 || x + col >= TFT_W) continue;
            framebuf[(y+row)*TFT_W + (x+col)] = src[row*w + col];
        }
    }
    dirty_mark(x, y, x+w-1, y+h-1);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(tftcore_blit_obj, 5, 5, tftcore_blit);

// tftcore.flush() — send dirty region to display
static mp_obj_t tftcore_flush(void) {
    if (!dirty) return mp_const_none;
    flush_rect(dirty_x0, dirty_y0, dirty_x1, dirty_y1);
    dirty_reset();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(tftcore_flush_obj, tftcore_flush);

// tftcore.flush_full() — force full screen refresh
static mp_obj_t tftcore_flush_full(void) {
    flush_full();
    dirty_reset();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(tftcore_flush_full_obj, tftcore_flush_full);

// ═══════ Module Definition ═══════
static const mp_rom_map_elem_t tftcore_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_tftcore) },
    { MP_ROM_QSTR(MP_QSTR_init),     MP_ROM_PTR(&tftcore_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_fill),     MP_ROM_PTR(&tftcore_fill_obj) },
    { MP_ROM_QSTR(MP_QSTR_rect),     MP_ROM_PTR(&tftcore_rect_obj) },
    { MP_ROM_QSTR(MP_QSTR_line),     MP_ROM_PTR(&tftcore_line_obj) },
    { MP_ROM_QSTR(MP_QSTR_circle),   MP_ROM_PTR(&tftcore_circle_obj) },
    { MP_ROM_QSTR(MP_QSTR_rrect),    MP_ROM_PTR(&tftcore_rrect_obj) },
    { MP_ROM_QSTR(MP_QSTR_gradient), MP_ROM_PTR(&tftcore_gradient_obj) },
    { MP_ROM_QSTR(MP_QSTR_pixel),    MP_ROM_PTR(&tftcore_pixel_obj) },
    { MP_ROM_QSTR(MP_QSTR_text),     MP_ROM_PTR(&tftcore_text_obj) },
    { MP_ROM_QSTR(MP_QSTR_blit),     MP_ROM_PTR(&tftcore_blit_obj) },
    { MP_ROM_QSTR(MP_QSTR_flush),    MP_ROM_PTR(&tftcore_flush_obj) },
    { MP_ROM_QSTR(MP_QSTR_flush_full), MP_ROM_PTR(&tftcore_flush_full_obj) },
};
static MP_DEFINE_CONST_DICT(tftcore_globals, tftcore_globals_table);

const mp_obj_module_t tftcore_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&tftcore_globals,
};

MP_REGISTER_MODULE(MP_QSTR_tftcore, tftcore_module);
