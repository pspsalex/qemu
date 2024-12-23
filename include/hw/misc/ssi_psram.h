#pragma once

#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/ssi/ssi.h"
#include "qom/object.h"
#include "exec/memory.h"
#include "hw/ssi/esp32_spi.h"

typedef struct SsiPsramState {
    SSIPeripheral parent_obj;
    Esp32SpiState *bus;
    uint32_t size_mbytes;
    uint32_t dummy;
    int command;
    int byte_count;
    MemoryRegion data_mr;
} SsiPsramState;

void psram_link_bus(DeviceState *s, Esp32SpiState *bus);

#define TYPE_SSI_PSRAM "ssi_psram"
OBJECT_DECLARE_SIMPLE_TYPE(SsiPsramState, SSI_PSRAM)

