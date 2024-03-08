#pragma once

#include "hw/ssi/ssi.h"
#include "qemu/typedefs.h"
#include "qom/object.h"

#define TYPE_IT8951E "it8951e"
OBJECT_DECLARE_SIMPLE_TYPE(it8951e_state, IT8951E)

#define IT8951E_WIDTH 960
#define IT8951E_HEIGHT 540

#define IT8951E_SPI_BUFFER_SIZE (IT8951E_HEIGHT * IT8951E_WIDTH + 256)

#define IT8951E_GPIO_HRDY         "it8951e-gpio-hrdy"
#define IT8951E_GPIO_MAIN_POWER   "it8951e-gpio-main-power"
#define IT8951E_GPIO_EPD_POWER    "it8951e-gpio-epd-power"
#define IT8951E_GPIO_EXT_POWER    "it8951e-gpio-ext-power"

enum it8951e_power_state {
    IT8951E_POWER_OFF,
    IT8951E_POWER_ON
};


enum it8951e_depth {
    IT8951E_2BPP = 0,
    IT8951E_3BPP,
    IT8951E_4BPP,
    IT8951E_8BPP
};

struct it8951e_rect {
    uint32_t x;
    uint32_t y;
    uint32_t w;
    uint32_t h;
    uint32_t posx;
    uint32_t posy;
    uint32_t dummyx;
    uint32_t dummyw;
    uint32_t maxx;
    uint32_t maxy;
    enum it8951e_depth bpp;
};

struct it8951e_state {
    SSIPeripheral ssidev;
    QemuConsole *con;
    qemu_irq gpio_hrdy;

    bool active;


    int32_t redraw;

    uint8_t framebuffer[IT8951E_WIDTH * IT8951E_HEIGHT];
    uint8_t spi_buffer[IT8951E_SPI_BUFFER_SIZE];
    uint8_t spi_out_buffer[IT8951E_SPI_BUFFER_SIZE];

    uint16_t spi_buffer_pos;
    uint16_t output_bytes;
    uint8_t rotation;

    uint16_t preamble;
    uint32_t needed_bytes;
    uint32_t transferred_bytes;

    int32_t viewport_rotation;

    void *state;
    int tsm;

    uint16_t ptr;
    bool packed_mode;
    uint32_t reg_lisar;

    struct it8951e_rect crsr;

    enum it8951e_power_state epd_power;
    enum it8951e_power_state main_power;
};



