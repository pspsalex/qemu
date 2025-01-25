/*
 * ESP32 GPIO emulation
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
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/gpio/esp32_gpio.h"

#define MASK_KEEP_U32 (0xFFFFFFFF00000000ull)
#define MASK_KEEP_L32 (0x00000000FFFFFFFFull)

static uint64_t esp32_gpio_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);
    uint64_t r = 0;

    switch (addr) {
    case A_GPIO_PIN0_REG ... A_GPIO_PIN0_REG + 39 * 4:
        r = s->gpio_cfg[(addr - A_GPIO_PIN0_REG) / 4];
        break;

    case A_GPIO_FUNC0_IN_SEL_CFG_REG ...
         A_GPIO_FUNC0_IN_SEL_CFG_REG + 255 * 4:
        r = s->gpio_per_in[(addr - A_GPIO_FUNC0_IN_SEL_CFG_REG) / 4];
        break;

    case A_GPIO_FUNC0_OUT_SEL_CFG_REG ...
         A_GPIO_FUNC0_OUT_SEL_CFG_REG + 39 * 4:
        r = s->gpio_per[(addr - A_GPIO_FUNC0_OUT_SEL_CFG_REG) / 4];
        break;

    case A_GPIO_STRAP:
        r = s->strap_mode;
        break;

    case A_GPIO_IN_REG:
        r = s->gpio_val;
        break;

    case A_GPIO_ENABLE_REG:
        r = s->gpio_out_ena;
        break;

    case A_GPIO_STATUS_REG:
        r = s->gpio_sr;
        break;

    case A_GPIO_IN1_REG:
        r = (s->gpio_val & MASK_KEEP_U32) >> 32;
        break;

    case A_GPIO_ENABLE1_REG:
        r = (s->gpio_out_ena & MASK_KEEP_U32) >> 32;
        break;

    case A_GPIO_STATUS1_REG:
        r = (s->gpio_sr & MASK_KEEP_U32) >> 32;
        break;

    case A_GPIO_ACPU_INT_REG:
        r = (s->gpio_app_int & MASK_KEEP_L32);
        break;

    case A_GPIO_ACPU_INT1_REG:
        r = (s->gpio_app_int & MASK_KEEP_U32) >> 32;
        break;

    case A_GPIO_ACPU_NMI_INT_REG:
        r = (s->gpio_app_nmi & MASK_KEEP_L32);
        break;

    case A_GPIO_ACPU_NMI_INT1_REG:
        r = (s->gpio_app_nmi & MASK_KEEP_U32) >> 32;
        break;

    case A_GPIO_PCPU_INT_REG:
        r = (s->gpio_pro_int & MASK_KEEP_L32);
        break;

    case A_GPIO_PCPU_INT1_REG:
        r = (s->gpio_pro_int & MASK_KEEP_U32) >> 32;
        break;

    case A_GPIO_PCPU_NMI_INT_REG:
        r = (s->gpio_pro_nmi & MASK_KEEP_L32);
        break;

    case A_GPIO_PCPU_NMI_INT1_REG:
        r = (s->gpio_pro_nmi & MASK_KEEP_U32) >> 32;
        break;

    default:
        break;
    }

    return r;
}

static void esp32_gpio_status_update(Esp32GpioState *s)
{
    s->gpio_app_int = (s->gpio_app_int & (~s->gpio_app_int_mask)) |
                        (s->gpio_sr & s->gpio_app_int_mask);
    s->gpio_app_nmi = (s->gpio_app_nmi & (~s->gpio_app_nmi_mask)) |
                        (s->gpio_sr & s->gpio_app_nmi_mask);
    s->gpio_pro_int = (s->gpio_pro_int & (~s->gpio_pro_nmi_mask)) |
                        (s->gpio_sr & s->gpio_pro_nmi_mask);
    s->gpio_pro_nmi = (s->gpio_pro_nmi & (~s->gpio_pro_int_mask)) |
                        (s->gpio_sr & s->gpio_pro_int_mask);

    qemu_set_irq(s->irq, 0);

}

/**
 * Trigger a QEMU IRQ for each GPIO output pin that gets its state changed.
 *
 * Usually triggered by the emulated code, when an output GPIO pin is set or
 * reset
 */
static void esp32_gpio_update(Esp32GpioState *s, uint64_t new_gpio,
                              uint64_t new_ena)
{
    uint64_t mask = 1;
    uint64_t delta = (s->gpio_val ^ new_gpio) ^ (s->gpio_out_ena ^ new_ena);
    uint64_t old_output = s->gpio_val & s->gpio_out_ena;
    uint64_t new_output = new_gpio & new_ena;
    uint64_t change = old_output ^ new_output;

    /*
     * Commit changes, as the GPIO trigger below may loop-back to a nested
     * GPIO update.
     */
    s->gpio_val = new_gpio;
    s->gpio_out_ena = new_ena;

    for (uint8_t bit_num = 0; bit_num < 48; bit_num++) {
        if ((delta & mask) && (change & mask)) {
            qemu_set_irq(s->gpio_irq[bit_num], (new_output & mask) ? 1 : 0);
        }
        mask <<= 1ull;
    }
}


/**
 * Called when an external device pulls one of the input GPIOs high or low.
 *
 * Trigger an interrupt, if needed, so the emulated software can react.
 */
static void esp32_gpio_pull(void *opaque, int n, int level)
{
    Esp32GpioState *s = (Esp32GpioState *)opaque;
    uint64_t value = s->gpio_val;
    uint64_t mask = 1ull << (uint64_t)n;

    if (level) {
        s->gpio_val = s->gpio_val | mask;
    } else {
        s->gpio_val = s->gpio_val & (~mask);
    }

    bool irq = false;

    /* Check if the transition triggers an IRQ */
    switch (FIELD_EX32(s->gpio_cfg[n], GPIO_PIN0_REG, INT_TYPE)) {
    case 1:
        irq = (!(value & mask)) && (s->gpio_val & mask);
        break;

    case 2:
        irq = (value & mask) && (!(s->gpio_val & mask));
        break;

    case 3:
        irq = (value ^ s->gpio_val) & mask;
        break;

    case 4:
        irq = !(s->gpio_val & mask);
        break;

    case 5:
        irq = (s->gpio_val & mask);
        break;

    default:
        break;
    }

    if (irq) {
        /* Update GPIO interrupt Status */
        s->gpio_app_int = s->gpio_app_int | (mask & s->gpio_app_int_mask);
        s->gpio_app_nmi = s->gpio_app_nmi | (mask & s->gpio_app_nmi_mask);
        s->gpio_pro_int = s->gpio_pro_nmi | (mask & s->gpio_pro_nmi_mask);
        s->gpio_pro_nmi = s->gpio_pro_int | (mask & s->gpio_pro_int_mask);

        s->gpio_sr = s->gpio_app_int | s->gpio_app_nmi | s->gpio_pro_int |
            s->gpio_pro_nmi;

        /* If there's any IRQ setup for the current PIN, trigger it */
        if (mask & s->gpio_sr) {
            qemu_set_irq(s->irq, 1);
        }
    } else if (FIELD_EX32(value, GPIO_PIN0_REG, INT_ENA)) {
        qemu_set_irq(s->irq, 0);
    }
}

static void esp32_gpio_pin_update(Esp32GpioState *s, uint32_t gpio_num,
                                  uint64_t value)
{
    uint32_t ena = FIELD_EX32(value, GPIO_PIN0_REG, INT_ENA);
    uint32_t type = FIELD_EX32(value, GPIO_PIN0_REG, INT_TYPE);

    s->gpio_cfg[gpio_num] = value;

    s->gpio_app_int_mask = (type && (ena & 0x01)) ?
                    (s->gpio_app_int_mask | (1ull << gpio_num)) :
                    (s->gpio_app_int_mask & (~(1ull << gpio_num)));

    s->gpio_app_nmi_mask = (type && (ena & 0x02)) ?
                    (s->gpio_app_nmi_mask | (1ull << gpio_num)) :
                    (s->gpio_app_int_mask & (~(1ull << gpio_num)));

    s->gpio_pro_int_mask = (type && (ena & 0x04)) ?
                    (s->gpio_pro_int_mask | (1ull << gpio_num)) :
                    (s->gpio_pro_int_mask & (~(1ull << gpio_num)));

    s->gpio_pro_nmi_mask = (type && (ena & 0x08)) ?
                    (s->gpio_pro_nmi_mask | (1ull << gpio_num)) :
                    (s->gpio_pro_nmi_mask & (~(1ull << gpio_num)));

}


static void esp32_gpio_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);
    value = value & MASK_KEEP_L32;

    switch (addr) {
    case A_GPIO_PIN0_REG ... A_GPIO_PIN0_REG + 39 * 4:
        esp32_gpio_pin_update(s, (addr - A_GPIO_PIN0_REG) / 4, value);
        break;

    case A_GPIO_FUNC0_IN_SEL_CFG_REG ...
         A_GPIO_FUNC0_IN_SEL_CFG_REG + 255 * 4:
        s->gpio_per_in[(addr - A_GPIO_FUNC0_IN_SEL_CFG_REG) / 4] = value;
        break;

    case A_GPIO_FUNC0_OUT_SEL_CFG_REG ...
            A_GPIO_FUNC0_OUT_SEL_CFG_REG + 255 * 4:
        s->gpio_per[(addr - A_GPIO_FUNC0_OUT_SEL_CFG_REG) / 4] = value;
        break;

    case A_GPIO_ENABLE_REG:
        esp32_gpio_update(s, s->gpio_val,
                          (s->gpio_out_ena & MASK_KEEP_U32) | value);
        break;

    case A_GPIO_ENABLE_W1TC_REG:
        esp32_gpio_update(s, s->gpio_val, s->gpio_out_ena & ~value);
        break;

    case A_GPIO_ENABLE_W1TS_REG:
        esp32_gpio_update(s, s->gpio_val, s->gpio_out_ena | value);
        break;

    case A_GPIO_ENABLE1_REG:
        value = value << 32;
        esp32_gpio_update(s, s->gpio_val,
                           (s->gpio_out_ena & MASK_KEEP_L32) | value);
        break;

    case A_GPIO_ENABLE1_W1TC_REG:
        value = value << 32;
        esp32_gpio_update(s, s->gpio_val, s->gpio_out_ena & ~value);
        break;

    case A_GPIO_ENABLE1_W1TS_REG:
        value = value << 32;
        esp32_gpio_update(s, s->gpio_val, s->gpio_out_ena | value);
        break;

    case A_GPIO_OUT_REG:
        esp32_gpio_update(s, (s->gpio_val & MASK_KEEP_U32) | value,
                          s->gpio_out_ena);
        break;

    case A_GPIO_OUT_W1TC_REG:
        esp32_gpio_update(s, s->gpio_val & ~value, s->gpio_out_ena);
        break;

    case A_GPIO_OUT_W1TS_REG:
        esp32_gpio_update(s, s->gpio_val | value, s->gpio_out_ena);
        break;

    case A_GPIO_OUT1_REG:
        value = value << 32;
        esp32_gpio_update(s, (s->gpio_val & MASK_KEEP_L32) | value,
                           s->gpio_out_ena);
        break;

    case A_GPIO_OUT1_W1TC_REG:
        value = value << 32;
        esp32_gpio_update(s, s->gpio_val & ~value, s->gpio_out_ena);
        break;

    case A_GPIO_OUT1_W1TS_REG:
        value = value << 32;
        esp32_gpio_update(s, s->gpio_val | value, s->gpio_out_ena);
        break;

    case A_GPIO_STATUS_REG:
        s->gpio_sr = (s->gpio_sr & MASK_KEEP_U32) | value;
        esp32_gpio_status_update(s);
        break;

    case A_GPIO_STATUS_W1TC_REG:
        s->gpio_sr = s->gpio_sr & ~value;
        esp32_gpio_status_update(s);
        break;

    case A_GPIO_STATUS_W1TS_REG:
        s->gpio_sr = s->gpio_sr | value;
        esp32_gpio_status_update(s);
        break;

    case A_GPIO_STATUS1_REG:
        value = value << 32;
        s->gpio_sr = (s->gpio_sr & MASK_KEEP_L32) | value;
        esp32_gpio_status_update(s);
        break;

    case A_GPIO_STATUS1_W1TC_REG:
        value = value << 32;
        s->gpio_sr = s->gpio_sr & ~value;
        esp32_gpio_status_update(s);
        break;

    case A_GPIO_STATUS1_W1TS_REG:
        value = value << 32;
        s->gpio_sr = s->gpio_sr | value;
        esp32_gpio_status_update(s);
        break;

    default:

        break;
    }
}

static const MemoryRegionOps uart_ops = {
    .read =  esp32_gpio_read,
    .write = esp32_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_gpio_reset(DeviceState *dev)
{
}

static void esp32_gpio_peripheral_set(void *opaque, int n, int level)
{
}


static void esp32_gpio_realize(DeviceState *dev, Error **errp)
{
    Esp32GpioState *s = ESP32_GPIO(dev);

    qdev_init_gpio_out(dev, &s->gpio_irq[0], 48);
    qdev_init_gpio_in(dev, esp32_gpio_pull, 48);

    /*
     * IOMUX_OUT: sends signal out of the device, based on input from
     * the peripheral
     */
    qdev_init_gpio_in_named(dev, esp32_gpio_peripheral_set,
        ESP32_GPIO_IOMUX_OUT, 229);

    /* IOMUX_IN: receives signal from outside, and triggers the peripheral */
    qdev_init_gpio_out_named(dev, &s->iomux_in[0], ESP32_GPIO_IOMUX_IN, 229);
}

static void esp32_gpio_init(Object *obj)
{
   Esp32GpioState *s = ESP32_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    /* Set the default value for the strap_mode property */
    object_property_set_int(obj, "strap_mode", ESP32_STRAP_MODE_FLASH_BOOT,
                            &error_fatal);

    memory_region_init_io(&s->iomem, obj, &uart_ops, s,
                          TYPE_ESP32_GPIO, 0x1000);

    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static Property esp32_gpio_properties[] = {
    /*
     * The strap_mode needs to be explicitly set in the instance init, thus,
     * set the default value to 0.
     */
    DEFINE_PROP_UINT32("strap_mode", Esp32GpioState, strap_mode, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = esp32_gpio_reset;
    dc->realize = esp32_gpio_realize;
    device_class_set_props(dc, esp32_gpio_properties);
}

static const TypeInfo esp32_gpio_info = {
    .name = TYPE_ESP32_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32GpioState),
    .instance_init = esp32_gpio_init,
    .class_init = esp32_gpio_class_init,
    .class_size = sizeof(Esp32GpioClass),
};

static void esp32_gpio_register_types(void)
{
    type_register_static(&esp32_gpio_info);
}

type_init(esp32_gpio_register_types)
