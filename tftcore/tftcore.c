/*
 * tftcore v2 — ILI9341 Framebuffer Engine + 3 Built-in Fonts
 * ESP32-S3 PSRAM + DMA SPI
 *
 * Fonts:
 *   text()  — 5x7   tiny   (labels, axis, tiny info)
 *   text2() — 7x14  medium (panel labels, descriptions)
 *   text3() — 10x20 large  (values: voltage, current, SOC)
 *
 * Pins: SCK=3 MOSI=4 CS=10 DC=5 RST=6
 */

#include <string.h>
#include <stdio.h>
#include "py/runtime.h"
#include "py/obj.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TFT_W 240
#define TFT_H 320
#define TFT_PIXELS (TFT_W*TFT_H)
#define TFT_BUFSIZE (TFT_PIXELS*2)

#define PIN_SCK  3
#define PIN_MOSI 4
#define PIN_CS   10
#define PIN_DC   5
#define PIN_RST  6
#define SPI_FREQ 40000000
#define DMA_CHAN SPI_DMA_CH_AUTO

static uint16_t *framebuf = NULL;
static spi_device_handle_t spi_dev = NULL;
static bool initialized = false;

// Dirty rect
static int dirty_x0,dirty_y0,dirty_x1,dirty_y1;
static bool dirty = false;
static void dirty_mark(int x0,int y0,int x1,int y1){
    if(!dirty){dirty_x0=x0;dirty_y0=y0;dirty_x1=x1;dirty_y1=y1;dirty=true;}
    else{if(x0<dirty_x0)dirty_x0=x0;if(y0<dirty_y0)dirty_y0=y0;if(x1>dirty_x1)dirty_x1=x1;if(y1>dirty_y1)dirty_y1=y1;}
}
static void dirty_reset(void){dirty=false;dirty_x0=TFT_W;dirty_y0=TFT_H;dirty_x1=0;dirty_y1=0;}

// SPI
static void spi_cmd(uint8_t cmd){
    gpio_set_level(PIN_DC,0);
    spi_transaction_t t={.length=8,.tx_buffer=&cmd};
    spi_device_polling_transmit(spi_dev,&t);
}
static void spi_data(const uint8_t *data,int len){
    if(!len)return;
    gpio_set_level(PIN_DC,1);
    spi_transaction_t t={.length=len*8,.tx_buffer=data};
    spi_device_polling_transmit(spi_dev,&t);
}
static void set_window(int x0,int y0,int x1,int y1){
    uint8_t ca[4]={x0>>8,x0&0xFF,x1>>8,x1&0xFF};
    uint8_t ra[4]={y0>>8,y0&0xFF,y1>>8,y1&0xFF};
    spi_cmd(0x2A);spi_data(ca,4);
    spi_cmd(0x2B);spi_data(ra,4);
    spi_cmd(0x2C);
}

// TFT init
static void tft_hw_init(void){
    gpio_set_direction(PIN_CS,GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_DC,GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_RST,GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_CS,1);
    spi_bus_config_t bus={.mosi_io_num=PIN_MOSI,.miso_io_num=-1,.sclk_io_num=PIN_SCK,.quadwp_io_num=-1,.quadhd_io_num=-1,.max_transfer_sz=TFT_BUFSIZE};
    spi_bus_initialize(SPI2_HOST,&bus,DMA_CHAN);
    spi_device_interface_config_t dev={.clock_speed_hz=SPI_FREQ,.mode=0,.spics_io_num=PIN_CS,.queue_size=1};
    spi_bus_add_device(SPI2_HOST,&dev,&spi_dev);
    gpio_set_level(PIN_RST,0);vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(PIN_RST,1);vTaskDelay(pdMS_TO_TICKS(120));
    spi_cmd(0x01);vTaskDelay(pdMS_TO_TICKS(150));
    spi_cmd(0x11);vTaskDelay(pdMS_TO_TICKS(300));
    uint8_t px=0x55;spi_cmd(0x3A);spi_data(&px,1);
    uint8_t mx=0x48;spi_cmd(0x36);spi_data(&mx,1);
    spi_cmd(0x29);vTaskDelay(pdMS_TO_TICKS(50));
}

// Framebuffer ops
static inline void fb_pixel(int x,int y,uint16_t c){
    if(x>=0&&x<TFT_W&&y>=0&&y<TFT_H)
        framebuf[y*TFT_W+x]=(c>>8)|(c<<8);
}

static void fb_fill_rect(int x,int y,int w,int h,uint16_t c){
    if(x<0){w+=x;x=0;}if(y<0){h+=y;y=0;}
    if(x+w>TFT_W)w=TFT_W-x;if(y+h>TFT_H)h=TFT_H-y;
    if(w<=0||h<=0)return;
    uint16_t sw=(c>>8)|(c<<8);
    for(int r=y;r<y+h;r++){
        uint16_t *p=&framebuf[r*TFT_W+x];
        for(int i=0;i<w;i++)p[i]=sw;
    }
    dirty_mark(x,y,x+w-1,y+h-1);
}

static void fb_line(int x0,int y0,int x1,int y1,uint16_t c){
    int dx=x1>x0?x1-x0:x0-x1,dy=y1>y0?y1-y0:y0-y1;
    int sx=x0<x1?1:-1,sy=y0<y1?1:-1,err=dx-dy;
    int mx0=x0<x1?x0:x1,my0=y0<y1?y0:y1,mx1=x0>x1?x0:x1,my1=y0>y1?y0:y1;
    while(1){
        fb_pixel(x0,y0,c);
        if(x0==x1&&y0==y1)break;
        int e2=2*err;
        if(e2>-dy){err-=dy;x0+=sx;}
        if(e2<dx){err+=dx;y0+=sy;}
    }
    dirty_mark(mx0,my0,mx1,my1);
}

static void fb_circle(int cx,int cy,int r,uint16_t c,bool fill){
    int x=0,y=r,d=3-2*r;
    while(x<=y){
        if(fill){
            for(int i=cx-x;i<=cx+x;i++){fb_pixel(i,cy+y,c);fb_pixel(i,cy-y,c);}
            for(int i=cx-y;i<=cx+y;i++){fb_pixel(i,cy+x,c);fb_pixel(i,cy-x,c);}
        }else{
            fb_pixel(cx+x,cy+y,c);fb_pixel(cx-x,cy+y,c);
            fb_pixel(cx+x,cy-y,c);fb_pixel(cx-x,cy-y,c);
            fb_pixel(cx+y,cy+x,c);fb_pixel(cx-y,cy+x,c);
            fb_pixel(cx+y,cy-x,c);fb_pixel(cx-y,cy-x,c);
        }
        if(d<0)d+=4*x+6;else{d+=4*(x-y)+10;y--;}
        x++;
    }
    dirty_mark(cx-r,cy-r,cx+r,cy+r);
}

static void fb_rounded_rect(int x,int y,int w,int h,int r,uint16_t c,bool fill){
    if(r>w/2)r=w/2;if(r>h/2)r=h/2;
    if(fill){
        fb_fill_rect(x+r,y,w-2*r,h,c);
        fb_fill_rect(x,y+r,r,h-2*r,c);
        fb_fill_rect(x+w-r,y+r,r,h-2*r,c);
        int cx0=x+r,cy0=y+r,cx1=x+w-r-1,cy1=y+h-r-1;
        int px=0,py=r,d=3-2*r;
        while(px<=py){
            for(int i=cx0-px;i<=cx1+px;i++){fb_pixel(i,cy0-py,c);fb_pixel(i,cy1+py,c);}
            for(int i=cx0-py;i<=cx1+py;i++){fb_pixel(i,cy0-px,c);fb_pixel(i,cy1+px,c);}
            if(d<0)d+=4*px+6;else{d+=4*(px-py)+10;py--;}
            px++;
        }
    }else{
        fb_fill_rect(x+r,y,w-2*r,1,c);fb_fill_rect(x+r,y+h-1,w-2*r,1,c);
        fb_fill_rect(x,y+r,1,h-2*r,c);fb_fill_rect(x+w-1,y+r,1,h-2*r,c);
        int cx0=x+r,cy0=y+r,cx1=x+w-r-1,cy1=y+h-r-1;
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
    dirty_mark(x,y,x+w-1,y+h-1);
}

static void fb_gradient_v(int x,int y,int w,int h,uint16_t c1,uint16_t c2){
    int r1=(c1>>11)&0x1F,g1=(c1>>5)&0x3F,b1=c1&0x1F;
    int r2=(c2>>11)&0x1F,g2=(c2>>5)&0x3F,b2=c2&0x1F;
    for(int row=0;row<h;row++){
        int r=r1+(r2-r1)*row/h,g=g1+(g2-g1)*row/h,b=b1+(b2-b1)*row/h;
        uint16_t c=(r<<11)|(g<<5)|b;
        uint16_t sw=(c>>8)|(c<<8);
        if(y+row>=0&&y+row<TFT_H){
            int x0=x<0?0:x,x1=x+w>TFT_W?TFT_W:x+w;
            uint16_t *p=&framebuf[(y+row)*TFT_W+x0];
            for(int i=0;i<x1-x0;i++)p[i]=sw;
        }
    }
    dirty_mark(x,y,x+w-1,y+h-1);
}

// ═══════ FONT 1: 5x7 ═══════
static const char F1_CHARS[]=" !%+-./0123456789:ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz~";
static const uint8_t F1_DATA[]={
0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x5f,0x00,0x00, 0x23,0x13,0x08,0x64,0x62,
0x08,0x08,0x3e,0x08,0x08, 0x08,0x08,0x08,0x08,0x08, 0x00,0x60,0x60,0x00,0x00,
0x20,0x10,0x08,0x04,0x02, 0x3e,0x51,0x49,0x45,0x3e, 0x00,0x42,0x7f,0x40,0x00,
0x42,0x61,0x51,0x49,0x46, 0x21,0x41,0x45,0x4b,0x31, 0x18,0x14,0x12,0x7f,0x10,
0x27,0x45,0x45,0x45,0x39, 0x3c,0x4a,0x49,0x49,0x30, 0x01,0x71,0x09,0x05,0x03,
0x36,0x49,0x49,0x49,0x36, 0x06,0x49,0x49,0x29,0x1e, 0x00,0x36,0x36,0x00,0x00,
0x7e,0x11,0x11,0x11,0x7e, 0x7f,0x49,0x49,0x49,0x36, 0x3e,0x41,0x41,0x41,0x22,
0x7f,0x41,0x41,0x22,0x1c, 0x7f,0x49,0x49,0x49,0x41, 0x7f,0x09,0x09,0x09,0x01,
0x3e,0x41,0x49,0x49,0x7a, 0x7f,0x08,0x08,0x08,0x7f, 0x00,0x41,0x7f,0x41,0x00,
0x20,0x40,0x41,0x3f,0x01, 0x7f,0x08,0x14,0x22,0x41, 0x7f,0x40,0x40,0x40,0x40,
0x7f,0x02,0x0c,0x02,0x7f, 0x7f,0x04,0x08,0x10,0x7f, 0x3e,0x41,0x41,0x41,0x3e,
0x7f,0x09,0x09,0x09,0x06, 0x3e,0x41,0x51,0x21,0x5e, 0x7f,0x09,0x19,0x29,0x46,
0x46,0x49,0x49,0x49,0x31, 0x01,0x01,0x7f,0x01,0x01, 0x3f,0x40,0x40,0x40,0x3f,
0x1f,0x20,0x40,0x20,0x1f, 0x3f,0x40,0x38,0x40,0x3f, 0x63,0x14,0x08,0x14,0x63,
0x07,0x08,0x70,0x08,0x07, 0x61,0x51,0x49,0x45,0x43, 0x20,0x54,0x54,0x54,0x78,
0x7f,0x48,0x44,0x44,0x38, 0x38,0x44,0x44,0x44,0x20, 0x38,0x44,0x44,0x48,0x7f,
0x38,0x54,0x54,0x54,0x18, 0x08,0x7e,0x09,0x01,0x02, 0x0c,0x52,0x52,0x52,0x3e,
0x7f,0x08,0x04,0x04,0x78, 0x00,0x44,0x7d,0x40,0x00, 0x20,0x40,0x44,0x3d,0x00,
0x7f,0x10,0x28,0x44,0x00, 0x00,0x41,0x7f,0x40,0x00, 0x7c,0x04,0x18,0x04,0x78,
0x7c,0x08,0x04,0x04,0x78, 0x38,0x44,0x44,0x44,0x38, 0x7c,0x14,0x14,0x14,0x08,
0x08,0x14,0x14,0x18,0x7c, 0x7c,0x08,0x04,0x04,0x08, 0x48,0x54,0x54,0x54,0x20,
0x04,0x3f,0x44,0x40,0x20, 0x3c,0x40,0x40,0x40,0x7c, 0x1c,0x20,0x40,0x20,0x1c,
0x3c,0x40,0x30,0x40,0x3c, 0x44,0x28,0x10,0x28,0x44, 0x0c,0x50,0x50,0x50,0x3c,
0x44,0x64,0x54,0x4c,0x44, 0x02,0x04,0x02,0x04,0x02
};
#define F1_W 5
#define F1_H 7
#define F1_N (sizeof(F1_CHARS)-1)

// ═══════ FONT 2: 7x14 (clean medium) ═══════
// Subset: space ! % + - . / 0-9 : A-Z a-z ~
// Each char = 14 bytes (7 cols x 14 rows, column-encoded as 2 bytes per col = uint16_t)
// We use 7 columns, each column is a 14-bit value stored as uint16_t LE
static const char F2_CHARS[]=" !%+-./:0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz~";
static const uint16_t F2_DATA[]={
// space
0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,
// !
0x0000,0x0000,0x0000,0x1F7E,0x0000,0x0000,0x0000,
// %
0x060C,0x0612,0x0190,0x00C0,0x0268,0x1204,0x0C04,
// +
0x0040,0x0040,0x0040,0x03F8,0x0040,0x0040,0x0040,
// -
0x0040,0x0040,0x0040,0x0040,0x0040,0x0040,0x0000,
// .
0x0000,0x0000,0x0000,0x1800,0x1800,0x0000,0x0000,
// /
0x1000,0x0800,0x0400,0x0200,0x0100,0x0080,0x0040,
// :
0x0000,0x0000,0x0198,0x0198,0x0000,0x0000,0x0000,
// 0
0x07F0,0x0808,0x0808,0x0808,0x0808,0x0808,0x07F0,
// 1
0x0000,0x0810,0x0808,0x0FFC,0x0800,0x0800,0x0000,
// 2
0x0C08,0x1008,0x1008,0x1008,0x0908,0x0908,0x0608,
// 3
0x0408,0x0808,0x0808,0x0888,0x0888,0x0888,0x0770,
// 4
0x0180,0x0140,0x0120,0x0110,0x0108,0x1FFC,0x0100,
// 5
0x047C,0x0808,0x0808,0x0808,0x0808,0x0808,0x07F0,
// 6
0x07F0,0x0888,0x0888,0x0888,0x0888,0x0888,0x0470,
// 7
0x0004,0x0004,0x0C04,0x0304,0x00C4,0x0034,0x000C,
// 8
0x0770,0x0888,0x0888,0x0888,0x0888,0x0888,0x0770,
// 9
0x0390,0x0448,0x0448,0x0448,0x0448,0x0448,0x03F0,
// A
0x1E00,0x0380,0x0260,0x0218,0x0260,0x0380,0x1E00,
// B
0x0FFC,0x0888,0x0888,0x0888,0x0888,0x0888,0x0770,
// C
0x07F0,0x0808,0x0808,0x0808,0x0808,0x0808,0x0410,
// D
0x0FFC,0x0808,0x0808,0x0808,0x0808,0x0410,0x03E0,
// E
0x0FFC,0x0888,0x0888,0x0888,0x0888,0x0888,0x0808,
// F
0x0FFC,0x0088,0x0088,0x0088,0x0088,0x0088,0x0008,
// G
0x07F0,0x0808,0x0808,0x0808,0x0888,0x0888,0x0CF0,
// H
0x0FFC,0x0080,0x0080,0x0080,0x0080,0x0080,0x0FFC,
// I
0x0000,0x0808,0x0808,0x0FFC,0x0808,0x0808,0x0000,
// J
0x0600,0x0800,0x0800,0x0800,0x0800,0x0800,0x07FC,
// K
0x0FFC,0x0080,0x0140,0x0220,0x0410,0x0808,0x0000,
// L
0x0FFC,0x0800,0x0800,0x0800,0x0800,0x0800,0x0800,
// M
0x0FFC,0x0010,0x0060,0x0180,0x0060,0x0010,0x0FFC,
// N
0x0FFC,0x0010,0x0020,0x00C0,0x0100,0x0200,0x0FFC,
// O
0x07F0,0x0808,0x0808,0x0808,0x0808,0x0808,0x07F0,
// P
0x0FFC,0x0088,0x0088,0x0088,0x0088,0x0088,0x0070,
// Q
0x07F0,0x0808,0x0808,0x0A08,0x0C08,0x0808,0x17F0,
// R
0x0FFC,0x0088,0x0088,0x0188,0x0288,0x0488,0x0870,
// S
0x0470,0x0888,0x0888,0x0888,0x0888,0x0888,0x0390,
// T
0x0008,0x0008,0x0008,0x0FFC,0x0008,0x0008,0x0008,
// U
0x07FC,0x0800,0x0800,0x0800,0x0800,0x0800,0x07FC,
// V
0x001C,0x00E0,0x0300,0x0C00,0x0300,0x00E0,0x001C,
// W
0x07FC,0x0800,0x0400,0x03F0,0x0400,0x0800,0x07FC,
// X
0x0C0C,0x0210,0x0120,0x00C0,0x0120,0x0210,0x0C0C,
// Y
0x000C,0x0030,0x00C0,0x0F00,0x00C0,0x0030,0x000C,
// Z
0x0C0C,0x0A08,0x0908,0x0888,0x0848,0x0828,0x0818,
// a
0x0700,0x0A80,0x0A80,0x0A80,0x0A80,0x0F00,0x0800,
// b
0x0FFC,0x0880,0x0840,0x0840,0x0840,0x0840,0x0780,
// c
0x0780,0x0840,0x0840,0x0840,0x0840,0x0840,0x0480,
// d
0x0780,0x0840,0x0840,0x0840,0x0880,0x0FFC,0x0000,
// e
0x0780,0x0AC0,0x0AC0,0x0AC0,0x0AC0,0x0AC0,0x0380,
// f
0x0040,0x0FF8,0x0044,0x0044,0x0004,0x0000,0x0000,
// g
0x0380,0x2440,0x2440,0x2440,0x2440,0x2440,0x1FC0,
// h
0x0FFC,0x0080,0x0040,0x0040,0x0040,0x0040,0x0F80,
// i
0x0000,0x0840,0x0840,0x0FBC,0x0800,0x0800,0x0000,
// j
0x1000,0x2000,0x2040,0x2040,0x1FBC,0x0000,0x0000,
// k
0x0FFC,0x0100,0x0280,0x0440,0x0840,0x0000,0x0000,
// l
0x0000,0x0804,0x0804,0x0FFC,0x0800,0x0800,0x0000,
// m
0x0FC0,0x0040,0x0040,0x0F80,0x0040,0x0040,0x0F80,
// n
0x0FC0,0x0080,0x0040,0x0040,0x0040,0x0040,0x0F80,
// o
0x0780,0x0840,0x0840,0x0840,0x0840,0x0840,0x0780,
// p
0x3FC0,0x0440,0x0440,0x0440,0x0440,0x0440,0x0380,
// q
0x0380,0x0440,0x0440,0x0440,0x0440,0x3FC0,0x0000,
// r
0x0FC0,0x0080,0x0040,0x0040,0x0040,0x0080,0x0000,
// s
0x0880,0x0940,0x0940,0x0940,0x0940,0x0640,0x0000,
// t
0x0040,0x07F8,0x0840,0x0840,0x0800,0x0000,0x0000,
// u
0x07C0,0x0800,0x0800,0x0800,0x0800,0x0400,0x0FC0,
// v
0x00C0,0x0300,0x0C00,0x0C00,0x0300,0x00C0,0x0000,
// w
0x07C0,0x0800,0x0400,0x03C0,0x0400,0x0800,0x07C0,
// x
0x0840,0x0480,0x0300,0x0300,0x0480,0x0840,0x0000,
// y
0x0040,0x2080,0x2100,0x1E00,0x0100,0x0080,0x0040,
// z
0x0C40,0x0A40,0x0940,0x08C0,0x0840,0x0840,0x0000,
// ~
0x0040,0x0020,0x0020,0x0040,0x0080,0x0080,0x0040,
};
#define F2_W 7
#define F2_H 14
#define F2_N (sizeof(F2_CHARS)-1)

// ═══════ FONT 3: 10x20 (large values) — stored as 10 cols x 20 rows, column = uint32_t ═══════
static const char F3_CHARS[]=" +-./:0123456789%AVCW";
static const uint32_t F3_DATA[]={
// space
0,0,0,0,0,0,0,0,0,0,
// +
0x00800,0x00800,0x00800,0x00800,0x3FF80,0x00800,0x00800,0x00800,0x00800,0x00000,
// -
0x00800,0x00800,0x00800,0x00800,0x00800,0x00800,0x00800,0x00800,0x00000,0x00000,
// .
0x00000,0x00000,0x00000,0xF0000,0xF0000,0x00000,0x00000,0x00000,0x00000,0x00000,
// /
0x80000,0x40000,0x20000,0x10000,0x08000,0x04000,0x02000,0x01000,0x00800,0x00400,
// :
0x00000,0x00000,0x0C060,0x0C060,0x00000,0x00000,0x00000,0x00000,0x00000,0x00000,
// 0
0x1FE00,0x20100,0x20100,0x20100,0x20100,0x20100,0x20100,0x20100,0x20100,0x1FE00,
// 1
0x00000,0x20200,0x20100,0x3FF00,0x20000,0x20000,0x00000,0x00000,0x00000,0x00000,
// 2
0x30200,0x40100,0x40100,0x40100,0x40100,0x20100,0x20100,0x20100,0x10100,0x10200,
// 3
0x10200,0x20100,0x20100,0x20100,0x21100,0x21100,0x21100,0x21100,0x12A00,0x0C400,
// 4
0x03000,0x02800,0x02400,0x02200,0x02100,0x02080,0x7FFC0,0x02000,0x02000,0x02000,
// 5
0x10FC0,0x20100,0x20100,0x20100,0x20100,0x20100,0x20100,0x20100,0x20200,0x1FC00,
// 6
0x1FC00,0x22200,0x21100,0x21100,0x21100,0x21100,0x21100,0x21100,0x22200,0x1C400,
// 7
0x00040,0x00040,0x30040,0x0C040,0x03040,0x00C40,0x00340,0x000C0,0x00040,0x00040,
// 8
0x1CE00,0x22100,0x21100,0x21100,0x21100,0x21100,0x21100,0x21100,0x22100,0x1CE00,
// 9
0x08E00,0x11100,0x21100,0x21100,0x21100,0x21100,0x21100,0x21100,0x11200,0x0FC00,
// %
0x180C0,0x18120,0x18120,0x00240,0x00480,0x00900,0x01200,0x12400,0x12400,0x0C400,
// A
0x78000,0x0E000,0x03800,0x02600,0x02180,0x02600,0x03800,0x0E000,0x78000,0x00000,
// V
0x00060,0x00380,0x01C00,0x0E000,0x70000,0x0E000,0x01C00,0x00380,0x00060,0x00000,
// C
0x1FE00,0x20100,0x20100,0x20100,0x20100,0x20100,0x20100,0x20100,0x10200,0x00000,
// W
0x1FFC0,0x20000,0x10000,0x0E000,0x10000,0x20000,0x10000,0x0E000,0x01C00,0x003C0,
};
#define F3_W 10
#define F3_H 20
#define F3_N (sizeof(F3_CHARS)-1)

// Generic font renderer
static void fb_text_f1(int x,int y,const char *str,uint16_t fg,uint16_t bg){
    while(*str){
        int idx=-1;
        for(int i=0;i<(int)F1_N;i++){if(F1_CHARS[i]==*str){idx=i;break;}}
        if(idx<0)idx=0;
        const uint8_t *g=&F1_DATA[idx*5];
        for(int c=0;c<5;c++){uint8_t bits=g[c];for(int r=0;r<7;r++)fb_pixel(x+c,y+r,(bits&(1<<r))?fg:bg);}
        for(int r=0;r<7;r++)fb_pixel(x+5,y+r,bg);
        x+=6;str++;
    }
}

static void fb_text_f2(int x,int y,const char *str,uint16_t fg,uint16_t bg){
    while(*str){
        int idx=-1;
        for(int i=0;i<(int)F2_N;i++){if(F2_CHARS[i]==*str){idx=i;break;}}
        if(idx<0)idx=0;
        const uint16_t *g=&F2_DATA[idx*7];
        for(int c=0;c<7;c++){uint16_t bits=g[c];for(int r=0;r<14;r++)fb_pixel(x+c,y+r,(bits&(1<<r))?fg:bg);}
        for(int r=0;r<14;r++)fb_pixel(x+7,y+r,bg);
        x+=8;str++;
    }
}

static void fb_text_f3(int x,int y,const char *str,uint16_t fg,uint16_t bg){
    while(*str){
        int idx=-1;
        for(int i=0;i<(int)F3_N;i++){if(F3_CHARS[i]==*str){idx=i;break;}}
        if(idx<0){x+=11;str++;continue;} // skip unknown
        const uint32_t *g=&F3_DATA[idx*10];
        for(int c=0;c<10;c++){uint32_t bits=g[c];for(int r=0;r<20;r++)fb_pixel(x+c,y+r,(bits&(1<<r))?fg:bg);}
        for(int r=0;r<20;r++)fb_pixel(x+10,y+r,bg);
        x+=11;str++;
    }
}

// Flush
static void flush_rect(int x0,int y0,int x1,int y1){
    if(x0<0)x0=0;if(y0<0)y0=0;if(x1>=TFT_W)x1=TFT_W-1;if(y1>=TFT_H)y1=TFT_H-1;
    if(x0>x1||y0>y1)return;
    int w=x1-x0+1;
    set_window(x0,y0,x1,y1);
    gpio_set_level(PIN_DC,1);
    for(int row=y0;row<=y1;row++){
        spi_transaction_t t={.length=w*16,.tx_buffer=(uint8_t*)&framebuf[row*TFT_W+x0]};
        spi_device_transmit(spi_dev,&t);
    }
}
static void flush_full(void){
    set_window(0,0,TFT_W-1,TFT_H-1);
    gpio_set_level(PIN_DC,1);
    uint8_t *buf=(uint8_t*)framebuf;int rem=TFT_BUFSIZE;int chunk=32000;
    while(rem>0){int len=rem>chunk?chunk:rem;
        spi_transaction_t t={.length=len*8,.tx_buffer=buf};
        spi_device_transmit(spi_dev,&t);buf+=len;rem-=len;}
}

// ═══════ Python Bindings ═══════
static mp_obj_t tftcore_init(void){
    if(initialized)return mp_const_none;
    framebuf=(uint16_t*)heap_caps_calloc(TFT_PIXELS,sizeof(uint16_t),MALLOC_CAP_SPIRAM);
    if(!framebuf)mp_raise_msg(&mp_type_MemoryError,MP_ERROR_TEXT("PSRAM alloc failed"));
    tft_hw_init();dirty_reset();initialized=true;
    printf("tftcore v2: OK, fb=%p (%dB PSRAM), 3 fonts\n",framebuf,TFT_BUFSIZE);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(tftcore_init_obj,tftcore_init);

static mp_obj_t tftcore_fill(mp_obj_t c){fb_fill_rect(0,0,TFT_W,TFT_H,mp_obj_get_int(c));return mp_const_none;}
static MP_DEFINE_CONST_FUN_OBJ_1(tftcore_fill_obj,tftcore_fill);

static mp_obj_t tftcore_rect(size_t n,const mp_obj_t *a){
    fb_fill_rect(mp_obj_get_int(a[0]),mp_obj_get_int(a[1]),mp_obj_get_int(a[2]),mp_obj_get_int(a[3]),mp_obj_get_int(a[4]));return mp_const_none;}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(tftcore_rect_obj,5,5,tftcore_rect);

static mp_obj_t tftcore_line(size_t n,const mp_obj_t *a){
    fb_line(mp_obj_get_int(a[0]),mp_obj_get_int(a[1]),mp_obj_get_int(a[2]),mp_obj_get_int(a[3]),mp_obj_get_int(a[4]));return mp_const_none;}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(tftcore_line_obj,5,5,tftcore_line);

static mp_obj_t tftcore_circle(size_t n,const mp_obj_t *a){
    fb_circle(mp_obj_get_int(a[0]),mp_obj_get_int(a[1]),mp_obj_get_int(a[2]),mp_obj_get_int(a[3]),n>4?mp_obj_is_true(a[4]):false);return mp_const_none;}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(tftcore_circle_obj,4,5,tftcore_circle);

static mp_obj_t tftcore_rrect(size_t n,const mp_obj_t *a){
    fb_rounded_rect(mp_obj_get_int(a[0]),mp_obj_get_int(a[1]),mp_obj_get_int(a[2]),mp_obj_get_int(a[3]),mp_obj_get_int(a[4]),mp_obj_get_int(a[5]),n>6?mp_obj_is_true(a[6]):false);return mp_const_none;}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(tftcore_rrect_obj,6,7,tftcore_rrect);

static mp_obj_t tftcore_gradient(size_t n,const mp_obj_t *a){
    fb_gradient_v(mp_obj_get_int(a[0]),mp_obj_get_int(a[1]),mp_obj_get_int(a[2]),mp_obj_get_int(a[3]),mp_obj_get_int(a[4]),mp_obj_get_int(a[5]));return mp_const_none;}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(tftcore_gradient_obj,6,6,tftcore_gradient);

static mp_obj_t tftcore_pixel(mp_obj_t x,mp_obj_t y,mp_obj_t c){
    fb_pixel(mp_obj_get_int(x),mp_obj_get_int(y),mp_obj_get_int(c));return mp_const_none;}
static MP_DEFINE_CONST_FUN_OBJ_3(tftcore_pixel_obj,tftcore_pixel);

// text() = 5x7
static mp_obj_t tftcore_text(size_t n,const mp_obj_t *a){
    fb_text_f1(mp_obj_get_int(a[0]),mp_obj_get_int(a[1]),mp_obj_str_get_str(a[2]),mp_obj_get_int(a[3]),mp_obj_get_int(a[4]));
    int len=strlen(mp_obj_str_get_str(a[2]));dirty_mark(mp_obj_get_int(a[0]),mp_obj_get_int(a[1]),mp_obj_get_int(a[0])+len*6,mp_obj_get_int(a[1])+7);
    return mp_const_none;}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(tftcore_text_obj,5,5,tftcore_text);

// text2() = 7x14
static mp_obj_t tftcore_text2(size_t n,const mp_obj_t *a){
    fb_text_f2(mp_obj_get_int(a[0]),mp_obj_get_int(a[1]),mp_obj_str_get_str(a[2]),mp_obj_get_int(a[3]),mp_obj_get_int(a[4]));
    int len=strlen(mp_obj_str_get_str(a[2]));dirty_mark(mp_obj_get_int(a[0]),mp_obj_get_int(a[1]),mp_obj_get_int(a[0])+len*8,mp_obj_get_int(a[1])+14);
    return mp_const_none;}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(tftcore_text2_obj,5,5,tftcore_text2);

// text3() = 10x20
static mp_obj_t tftcore_text3(size_t n,const mp_obj_t *a){
    fb_text_f3(mp_obj_get_int(a[0]),mp_obj_get_int(a[1]),mp_obj_str_get_str(a[2]),mp_obj_get_int(a[3]),mp_obj_get_int(a[4]));
    int len=strlen(mp_obj_str_get_str(a[2]));dirty_mark(mp_obj_get_int(a[0]),mp_obj_get_int(a[1]),mp_obj_get_int(a[0])+len*11,mp_obj_get_int(a[1])+20);
    return mp_const_none;}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(tftcore_text3_obj,5,5,tftcore_text3);

// blit
static mp_obj_t tftcore_blit(size_t n,const mp_obj_t *a){
    int x=mp_obj_get_int(a[0]),y=mp_obj_get_int(a[1]),w=mp_obj_get_int(a[2]),h=mp_obj_get_int(a[3]);
    mp_buffer_info_t bi;mp_get_buffer_raise(a[4],&bi,MP_BUFFER_READ);
    uint16_t *src=(uint16_t*)bi.buf;
    for(int r=0;r<h;r++){if(y+r<0||y+r>=TFT_H)continue;for(int c=0;c<w;c++){if(x+c<0||x+c>=TFT_W)continue;framebuf[(y+r)*TFT_W+(x+c)]=src[r*w+c];}}
    dirty_mark(x,y,x+w-1,y+h-1);return mp_const_none;}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(tftcore_blit_obj,5,5,tftcore_blit);

static mp_obj_t tftcore_flush(void){if(!dirty)return mp_const_none;flush_rect(dirty_x0,dirty_y0,dirty_x1,dirty_y1);dirty_reset();return mp_const_none;}
static MP_DEFINE_CONST_FUN_OBJ_0(tftcore_flush_obj,tftcore_flush);

static mp_obj_t tftcore_flush_full(void){flush_full();dirty_reset();return mp_const_none;}
static MP_DEFINE_CONST_FUN_OBJ_0(tftcore_flush_full_obj,tftcore_flush_full);

// Module
static const mp_rom_map_elem_t tftcore_globals_table[]={
    {MP_ROM_QSTR(MP_QSTR___name__),MP_ROM_QSTR(MP_QSTR_tftcore)},
    {MP_ROM_QSTR(MP_QSTR_init),MP_ROM_PTR(&tftcore_init_obj)},
    {MP_ROM_QSTR(MP_QSTR_fill),MP_ROM_PTR(&tftcore_fill_obj)},
    {MP_ROM_QSTR(MP_QSTR_rect),MP_ROM_PTR(&tftcore_rect_obj)},
    {MP_ROM_QSTR(MP_QSTR_line),MP_ROM_PTR(&tftcore_line_obj)},
    {MP_ROM_QSTR(MP_QSTR_circle),MP_ROM_PTR(&tftcore_circle_obj)},
    {MP_ROM_QSTR(MP_QSTR_rrect),MP_ROM_PTR(&tftcore_rrect_obj)},
    {MP_ROM_QSTR(MP_QSTR_gradient),MP_ROM_PTR(&tftcore_gradient_obj)},
    {MP_ROM_QSTR(MP_QSTR_pixel),MP_ROM_PTR(&tftcore_pixel_obj)},
    {MP_ROM_QSTR(MP_QSTR_text),MP_ROM_PTR(&tftcore_text_obj)},
    {MP_ROM_QSTR(MP_QSTR_text2),MP_ROM_PTR(&tftcore_text2_obj)},
    {MP_ROM_QSTR(MP_QSTR_text3),MP_ROM_PTR(&tftcore_text3_obj)},
    {MP_ROM_QSTR(MP_QSTR_blit),MP_ROM_PTR(&tftcore_blit_obj)},
    {MP_ROM_QSTR(MP_QSTR_flush),MP_ROM_PTR(&tftcore_flush_obj)},
    {MP_ROM_QSTR(MP_QSTR_flush_full),MP_ROM_PTR(&tftcore_flush_full_obj)},
};
static MP_DEFINE_CONST_DICT(tftcore_globals,tftcore_globals_table);
const mp_obj_module_t tftcore_module={.base={&mp_type_module},.globals=(mp_obj_dict_t*)&tftcore_globals};
MP_REGISTER_MODULE(MP_QSTR_tftcore,tftcore_module);
