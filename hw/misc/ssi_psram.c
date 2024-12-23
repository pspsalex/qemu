/*
 * ESP-PSRAM basic emulation
 *
 * Copyright (c) 2021-2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "qemu/module.h"
#include "qemu/error-report.h"
#include "hw/qdev-properties.h"
#include "hw/misc/ssi_psram.h"
#include "hw/ssi/esp32_spi.h"

#define CMD_READ_ID 0x9f
#define PSRAM_ID_MFG 0x0d
#define PSRAM_ID_KGD 0x5d

#define FAKE_16MB_ID    0x6a
#define FAKE_32MB_ID    0x8e

static int get_eid_by_size(uint32_t size_mbytes) {
    switch (size_mbytes)
    {
    case 2:
        return 0x00;
    case 4:
        return 0x21;
    case 8:
        return 0x40;
    case 16:
        return FAKE_16MB_ID;
    case 32:
        return FAKE_32MB_ID;
    default:
        qemu_log_mask(LOG_UNIMP, "%s: PSRAM size %" PRIu32 "MB not implemented\n",
                      __func__, size_mbytes);
        return 0;
    }
}

static uint32_t psram_read(SsiPsramState *s)
{
    uint32_t result = 0;
    if (s->command == CMD_READ_ID) {
        uint8_t read_id_response[] = {
            0x00, 0x00, 0x00, 0x00,
            0x00, PSRAM_ID_MFG, PSRAM_ID_KGD,
            get_eid_by_size(s->size_mbytes),
            0xaa, 0xbb, 0xcc, 0xdd, 0xee
        };
        int index = s->byte_count - s->dummy;
        if (index < ARRAY_SIZE(read_id_response)) {
            result = read_id_response[index];
        }
    }
    return result;
}

static void psram_write(SsiPsramState *s, uint32_t value)
{
    if (s->byte_count == s->dummy) {
        s->command = value;
    }
    s->byte_count++;
}

void psram_link_bus(DeviceState *s, Esp32SpiState *bus)
{
    SsiPsramState *state = SSI_PSRAM(s);
    state->bus = bus;
}

static uint32_t psram_transfer(SSIPeripheral *dev, uint32_t value)
{
    SsiPsramState *s = SSI_PSRAM(dev);

    if ((s->bus == NULL) || (s->bus->clk_reg & 0x80000000u)) {
        s->dummy = 0;
    } else {
        s->dummy = 1;
    }
    psram_write(s, value);
    uint32_t r = psram_read(s);
    return r;
}

static int psram_cs(SSIPeripheral *ss, bool select)
{
    SsiPsramState *s = SSI_PSRAM(ss);
    if (!select) {
        s->byte_count = 0;
        s->command = -1;
    }
    return 0;
}

static void psram_realize(SSIPeripheral *ss, Error **errp)
{
    SsiPsramState *s = SSI_PSRAM(ss);
    /* Make sure the given size is supported */
    if (get_eid_by_size(s->size_mbytes) == 0) {
        error_report("[PSRAM] Invalid size %dMB for the PSRAM", s->size_mbytes);
    }

    /* Allocate the actual array that will act as a vritual RAM */
    const uint32_t size_bytes = s->size_mbytes * 1024 * 1024;
    memory_region_init_ram(&s->data_mr, OBJECT(s), "psram.memory_region", size_bytes, &error_fatal);
}

static Property psram_properties[] = {
    DEFINE_PROP_UINT32("size_mbytes", SsiPsramState, size_mbytes, 4),
    DEFINE_PROP_UINT32("dummy", SsiPsramState, dummy, 1),
    DEFINE_PROP_END_OF_LIST(),
};


static void psram_class_init(ObjectClass *klass, void *data)
{
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    k->transfer = psram_transfer;
    k->set_cs = psram_cs;
    k->cs_polarity = SSI_CS_LOW;
    k->realize = psram_realize;
    device_class_set_props(dc, psram_properties);
}

static const TypeInfo psram_info = {
    .name          = TYPE_SSI_PSRAM,
    .parent        = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(SsiPsramState),
    .class_init    = psram_class_init
};

static void psram_register_types(void)
{
    type_register_static(&psram_info);
}

type_init(psram_register_types)
