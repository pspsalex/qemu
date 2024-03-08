/*
 * ESP32 SoC and machine
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "hw/sysbus.h"
#include "hw/i2c/esp32_i2c.h"
#include "hw/xtensa/xtensa_memory.h"
#include "hw/xtensa/esp32.h"
#include "hw/misc/unimp.h"
#include "hw/irq.h"
#include "hw/i2c/i2c.h"
#include "hw/qdev-properties.h"
#include "hw/xtensa/esp32.h"
#include "hw/misc/ssi_psram.h"
#include "hw/display/it8951e.h"
#include "hw/input/gt911.h"
#include "hw/sd/dwc_sdmmc.h"
#include "core-esp32/core-isa.h"
#include "qemu/datadir.h"
#include "sysemu/sysemu.h"
#include "sysemu/reset.h"
#include "sysemu/cpus.h"
#include "sysemu/runstate.h"
#include "sysemu/blockdev.h"
#include "sysemu/block-backend.h"
#include "exec/exec-all.h"
#include "net/net.h"
#include "elf.h"

#include "hw/input/m5paper-btn.h"

#define ESP32_SOC(obj) OBJECT_CHECK(Esp32SocState, (obj), TYPE_ESP32_SOC)

typedef struct Esp32MachineState Esp32MachineState;

#define ESP32_MACHINE(obj) \
    OBJECT_CHECK(Esp32MachineState, (obj), TYPE_ESP32_MACHINE)

typedef struct M5PaperMachineState {
    Esp32MachineState parent_obj;

} M5PaperMachineState;

#define TYPE_M5PAPER_MACHINE MACHINE_TYPE_NAME("m5paper")
OBJECT_DECLARE_SIMPLE_TYPE(M5PaperMachineState, M5PAPER_MACHINE)


static void m5paper_machine_init(MachineState *machine)
{
    M5PaperMachineState *pms = M5PAPER_MACHINE(machine);

    esp32_machine_init(machine);

    Esp32SocState *s = (Esp32SocState *)(&(pms->parent_obj.esp32));

    /*
     * Init SPI devices
     */

    DeviceState *spi_master = DEVICE(&s->spi[3]);
    BusState *spi_bus = qdev_get_child_bus(spi_master, "spi");

    DeviceState *display = qdev_new(TYPE_IT8951E);
    object_property_set_int(OBJECT(display), "cs", 1, &error_fatal);
    qdev_realize_and_unref(display, spi_bus, &error_fatal);

    /*
     * M5Paper:
     * CS is GPIO15, and driven manually, so SSI_GPIO_CS on esp spi can't be
     * connected here
     *
     * Display CS is active low
     * SD CS is active high
     */
    qdev_connect_gpio_out(DEVICE(&s->gpio), 15,
                qdev_get_gpio_in_named(display, SSI_GPIO_CS, 0));

    /* HRDY is GPIO27 */
    qdev_connect_gpio_out_named(display, IT8951E_GPIO_HRDY, 0,
                qdev_get_gpio_in(DEVICE(&s->gpio), 27));

    /* EPD POWER is GPIO23, does reset of EPD */
    qdev_connect_gpio_out(DEVICE(&s->gpio), 23,
                qdev_get_gpio_in_named(display, IT8951E_GPIO_EPD_POWER, 0));

    /* Display POWER is GPIO2, turns on power to the display */
    qdev_connect_gpio_out(DEVICE(&s->gpio), 2,
                qdev_get_gpio_in_named(display, IT8951E_GPIO_MAIN_POWER, 0));

    DeviceState *sd_dev = ssi_create_peripheral((SSIBus *)spi_bus, "ssi-sd");

    qdev_connect_gpio_out(DEVICE(&s->gpio), 4,
                qdev_get_gpio_in_named(sd_dev, SSI_GPIO_CS, 0));

    DriveInfo *dinfo = drive_get(IF_SD, 0, 1);
    DeviceState *card_dev = qdev_new(TYPE_SD_CARD_SPI);
    BlockBackend *blk = dinfo ? blk_by_legacy_dinfo(dinfo) : NULL;
    qdev_prop_set_drive_err(card_dev, "drive", blk, &error_fatal);
    qdev_realize_and_unref(card_dev,
                           qdev_get_child_bus(sd_dev, "sd-bus"),
                           &error_fatal);

    /*
     * Init I2C devices
     */

    /*
     * It should be possible to create an I2C device from the command line,
     * however for this to work the I2C bus must be reachable from
     * sysbus-default.
     *
     * At the moment the peripherals are added to an unrelated bus, to avoid
     * being reset on CPU reset.
     * If we find a way to decouple peripheral reset from sysbus reset,
     * we can move them to the sysbus and thus enable creation of i2c devices.
     */
    DeviceState *i2c_master = DEVICE(&s->i2c[0]);
    I2CBus *i2c_bus = I2C_BUS(qdev_get_child_bus(i2c_master, "i2c"));

    I2CSlave *gt911 = i2c_slave_create_simple(i2c_bus, "gt911", 0x14);

    object_property_set_int(OBJECT(gt911), "rotation",
                            object_property_get_int(OBJECT(display), "rotation",
                                                    &error_fatal),
                            &error_fatal);
    object_property_set_int(OBJECT(gt911), "rotation_offset", 90, &error_fatal);

    /* GT911 INT is GPIO36 */
    qdev_connect_gpio_out_named(&(gt911->qdev), GT911_GPIO_INT, 0,
                qdev_get_gpio_in(DEVICE(&s->gpio), 36));

    sysbus_connect_irq(SYS_BUS_DEVICE(&s->gpio), 0,
                qdev_get_gpio_in(DEVICE(&s->intmatrix), ETS_GPIO_INTR_SOURCE));

    qemu_set_irq(qdev_get_gpio_in(DEVICE(&s->gpio), 39), 1);



    /*
     * Init GPIO devices
     */

    /* Add the M5Paper buttons and link them to the GPIOs */
    DeviceState *dsbuttons = qdev_new(TYPE_M5PAPER_BTN);
    M5PaperBtnState *buttons = M5PAPER_BTN(dsbuttons);
    qdev_realize(DEVICE(buttons), &s->periph_bus, &error_fatal);

    /* BTN_LEFT is GPIO39 */
    qdev_connect_gpio_out_named(dsbuttons, M5PAPER_BTN_LEFT, 0,
                qdev_get_gpio_in(DEVICE(&s->gpio), 39));

    /* BTN_PUSH is GPIO38 */
    qdev_connect_gpio_out_named(dsbuttons, M5PAPER_BTN_PUSH, 0,
                qdev_get_gpio_in(DEVICE(&s->gpio), 38));

    /* BTN_RIGHT is GPIO37 */
    qdev_connect_gpio_out_named(dsbuttons, M5PAPER_BTN_RIGHT, 0,
                qdev_get_gpio_in(DEVICE(&s->gpio), 37));

}


/* Initialize machine type */
static void m5paper_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "M5Paper with ESP32 SOC";
    mc->init = m5paper_machine_init;
    mc->max_cpus = 2;
    mc->default_cpus = 2;
    mc->default_ram_size = 0;
    mc->fixup_ram_size = esp32_fixup_ram_size;
}

static const TypeInfo m5paper_info = {
    .name = TYPE_M5PAPER_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(M5PaperMachineState),
    .class_init = m5paper_machine_class_init,
};

static void m5paper_machine_type_init(void)
{
    type_register_static(&m5paper_info);
}

type_init(m5paper_machine_type_init);

