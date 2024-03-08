#pragma once

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_ESP32_ADC "esp32-adc"
OBJECT_DECLARE_SIMPLE_TYPE(Esp32AdcState, ESP32_ADC)

struct Esp32AdcState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    uint32_t regs[256];

    uint32_t sar1_bits;
    uint32_t sar1_sample_bits;
    uint32_t sar1_channel;
    bool sar1_inv;

    uint32_t sar2_bits;
    uint32_t sar2_sample_bits;
    uint32_t sar2_channel;
    bool sar2_inv;

    uint16_t sar1_value[16];
    uint16_t sar2_value[16];

    uint16_t sar1_reset_value[16];
    uint16_t sar2_reset_value[16];

    QEMUTimer *sar1_timer;
    QEMUTimer *sar2_timer;

    qemu_irq irq;
};


