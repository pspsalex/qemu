/*
 * GT911 Touchscreen + M5 Paper Button emulation
 *
 * Copyright (C) 2023 Alex Popescu <alex@247dev.ro>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 *
 * Heavily inspired by the tmp105.c and tsc2005.c drivers
 *
 */

/* I2C device */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "qemu/module.h"
#include "hw/input/gt911.h"
#include "ui/console.h"
#include "migration/vmstate.h"
#include "qapi/visitor.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "qemu/timer.h"
#include "qemu/log.h"

#ifndef GT911_DEBUG_LEVEL
#define GT911_DEBUG_LEVEL 0
#endif

#define DPRINTL(lvl, fmt, args...) do { \
    if (GT911_DEBUG_LEVEL >= lvl) { \
        qemu_log("%s:%d [%d]: " fmt, __func__, __LINE__, lvl, ## args); \
    } \
} while (0)

#define DPRINTF(fmt, args...) DPRINTL(2, fmt, ## args)

#define BADF(fmt, args...) do { \
    qemu_log("%s:%d [X]: " fmt, __func__, __LINE__, ## args); \
    if (GT911_DEBUG_LEVEL) { \
        abort(); \
    } \
} while (0)

#define DSTART() DPRINTL(4, "START\n")
#define DEND()   DPRINTL(4, "END\n")

#define GT911_DEFAULT_XRES (4096u)
#define GT911_DEFAULT_YRES (4096u)

#define GT911_REG_CONFIG_VERSION (0x8047u)
#define GT911_REG_CONFIG_XRES    (0x8048u)
#define GT911_REG_CONFIG_XRES_LB (0x8048u)
#define GT911_REG_CONFIG_XRES_HB (0x8049u)
#define GT911_REG_CONFIG_YRES    (0x804Au)
#define GT911_REG_CONFIG_YRES_LB (0x804Au)
#define GT911_REG_CONFIG_YRES_HB (0x804Bu)
#define GT911_REG_CONFIG_CHKSUM  (0x80ffu)
#define GT911_REG_CONFIG_FRESH   (0x8100u)

#define TYPE_GT911 "gt911"
OBJECT_DECLARE_SIMPLE_TYPE(GT911State, GT911)

struct GT911State {
    I2CSlave i2c;
    uint8_t buf[2];
    qemu_irq pin;

    uint16_t len;
    uint8_t config;
    int16_t temperature;
    int16_t limit[2];
    int faults;
    uint8_t alarm;

    int32_t viewport_rotation;
    int32_t rotation_offset;

    QEMUTimer *timer;
    bool pressed;

    uint8_t ptrbytes;
    uint16_t ptr;
    uint8_t mem[65536];
    uint16_t xres;
    uint16_t yres;
    uint16_t vp_width;
    uint16_t vp_height;
    bool autores;

    QEMUPutMouseEntry *mouse;
};

static void gt911_set_coords(GT911State *s, int x, int y)
{
    s->mem[0x814e] = s->pressed ? 0b10000001 : 0;
    s->mem[0x8150] = (uint8_t)(x & 0xFF);
    s->mem[0x8151] = (uint8_t)((x >> 8) & 0xFF);
    s->mem[0x8152] = (uint8_t)(y & 0xFF);
    s->mem[0x8153] = (uint8_t)((y >> 8) & 0xFF);
    s->mem[0x8154] = 0x80;
    s->mem[0x8155] = 0x00;
    s->mem[0x8156] = 0x00;
    s->mem[0x8157] = 0x01;
}


static void gt911_mouse_event(void *opaque, int dx, int dy, int dz,
                              int buttons_state)
{
    GT911State *s = GT911(opaque);

    int vpx = dx * s->vp_width / (INPUT_EVENT_ABS_MAX - INPUT_EVENT_ABS_MIN);
    int vpy = dy * s->vp_height / (INPUT_EVENT_ABS_MAX - INPUT_EVENT_ABS_MIN);

    int x = vpx;
    int y = vpy;

    switch ((s->viewport_rotation + s->rotation_offset) % 360) {
    case 0:
        x = s->vp_width - vpx;
        y = s->vp_height - vpy;
        break;

    case 90:
        x = s->vp_height - vpy;
        y = vpx;
        break;

    case 180:
        x = vpx;
        y = vpy;
        break;

    case 270:
        x = vpy;
        y = s->vp_width - vpx;
        break;

    default:
        BADF("Unkown viewport rotation: %d. Must be multiple of 90\n",
             s->viewport_rotation);
        break;
    }

    DPRINTL(3, "(%d, %d) --> (%d, %d) --> (%d, %d) @ %d\n", dx, dy, vpx, vpy,
            x, y, s->viewport_rotation);

    if (!s->pressed && buttons_state) {
        s->pressed = true;
    }

    if (s->pressed) {
        if (!buttons_state) {
            s->pressed = false;
        }
        gt911_set_coords(s, x, y);
        qemu_set_irq(s->pin, 1);
        timer_mod(s->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 5 +
                        ((int64_t)s->mem[0x8056] & 0x0F));
    }
}

static void gt911_remove_handler(GT911State *s)
{
    if (s->mouse) {
        qemu_remove_mouse_event_handler(s->mouse);
        s->mouse = NULL;
    }
}

static void gt911_add_handler(GT911State *s)
{
    if (s->mouse == NULL) {
        s->mouse = qemu_add_mouse_event_handler(gt911_mouse_event, s, 1,
                    "QEMU GT911-driven Touchscreen");
    }
}

static int gt911_post_load(void *opaque, int version_id)
{
    GT911State *s = GT911(opaque);

    gt911_remove_handler(s);
    gt911_add_handler(s);

    return 0;
}



static const VMStateDescription vmstate_gt911 = {
    .name = "GT911",
    .version_id = 0,
    .minimum_version_id = 0,
    .post_load = gt911_post_load,
    .fields = (VMStateField[]) {
        /* TODO: fields... */
        VMSTATE_END_OF_LIST()
    }
};


static int gt911_rx(I2CSlave *i2c, uint8_t data)
{
    GT911State *s = GT911(i2c);

    if (s->ptrbytes < 2) {
        if (s->ptr == 0) {
            s->ptr = (s->ptr & 0x00FFu) | (data << 8);
        } else {
            s->ptr = (s->ptr & 0xFF00u) | data;
        }
        s->ptrbytes++;
        return 0;
    }

    s->mem[s->ptr++] = data;
    return 0;
}

static uint8_t gt911_tx(I2CSlave *i2c)
{
    GT911State *s = GT911(i2c);

    DPRINTF("TX: %04x: %02x\n", s->ptr, s->mem[s->ptr]);

    return s->mem[s->ptr++];
}

static int gt911_event(I2CSlave *i2c, enum i2c_event event)
{
    GT911State *s = GT911(i2c);

    DPRINTF("EV: %d\n", event);
    qemu_set_irq(s->pin, 0);

    switch (event) {
    case I2C_START_SEND:
        s->ptr = 0;
        s->len = 0;
        s->ptrbytes = 0;
        break;

    case I2C_START_RECV:
        s->len = 0;
        break;

    case I2C_FINISH:
        break;

    case I2C_START_SEND_ASYNC:
        break;

    case I2C_NACK:
        break;

    default:
        break;
    }

    return 0;
}

static void gt911_reset(DeviceState *dev)
{
    GT911State *s = GT911(dev);

    s->pressed = false;
    s->ptr = 0;
    s->ptrbytes = 0;

    gt911_post_load(s, 0);

}

static void gt911_set_register8b(GT911State *s, uint16_t address, uint8_t value)
{
    s->mem[address] = value;
}

static void gt911_set_register16b(GT911State *s, uint16_t address,
                                  uint16_t value)
{
    gt911_set_register8b(s, address, (uint8_t)(value & 0x00FFu));
    gt911_set_register8b(s, address, (uint8_t)(value >> 8));
}


static void gt911_set_xresolution(GT911State *s, uint16_t resolution)
{

    s->xres = resolution;

    gt911_set_register16b(s, GT911_REG_CONFIG_XRES, resolution);
}

static void gt911_set_yresolution(GT911State *s, uint16_t resolution)
{

    s->yres = resolution;

    gt911_set_register16b(s, GT911_REG_CONFIG_YRES, resolution);
}


static void gt911_get_prop(Object *obj, Visitor *v,
                                       const char *name,
                                       void *opaque, Error **errp)
{
    GT911State *s = GT911(obj);
    int64_t value;
    if (strcmp(name, "xresolution") == 0) {
        value = s->xres;
    } else if (strcmp(name, "yresolution") == 0) {
        value = s->yres;
    } else if (strcmp(name, "rotation_offset") == 0) {
        value  = s->rotation_offset;
    } else {
        value = s->viewport_rotation;
    }
    visit_type_int(v, name, &value, errp);
}


static void gt911_set_resolution_prop(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    GT911State *s = GT911(obj);
    int64_t resolution;

    if (!visit_type_int(v, name, &resolution, errp)) {
        return;
    }

    if (resolution <= 0) {
        error_setg(errp, "%s '%" PRId64 "' is out of range", name, resolution);
        return;
    }

    s->autores = FALSE;
    if (strcmp(name, "xresolution") == 0) {
        gt911_set_xresolution(s, resolution);
    } else {
        gt911_set_yresolution(s, resolution);
    }
}

static void gt911_set_rotation_prop(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    GT911State *s = GT911(obj);
    int64_t rotation;

    if (!visit_type_int(v, name, &rotation, errp)) {
        return;
    }

    rotation %= 360;
    if (rotation < 0) {
        rotation = 360 - rotation;
    }

    if (rotation % 90) {
        error_setg(errp, "Rotation '%" PRId64 "' is not a multiple of 90",
                   rotation);
        return;
    }
    DPRINTL(3, "Set %s to %ld\n", name, rotation);

    if (strcmp(name, "rotation_offset") == 0) {
        s->rotation_offset = rotation;
    } else {
        s->viewport_rotation = (uint16_t) (rotation);
    }
}

static void gt911_timer_tick(void *opaque)
{
    GT911State *s = opaque;
    qemu_set_irq(s->pin, 0);
}


static void gt911_realize(DeviceState *dev, Error **errp)
{
    I2CSlave *i2c = I2C_SLAVE(dev);
    GT911State *s = GT911(i2c);

    qdev_init_gpio_out_named(dev, &s->pin, GT911_GPIO_INT, 1);

    if (s->autores) {
        int xres = qemu_console_get_width(NULL, s->xres);
        int yres = qemu_console_get_height(NULL, s->yres);
        s->vp_width = xres;
        s->vp_height = yres;
        if (((s->viewport_rotation + s->rotation_offset) == 0) ||
            ((s->viewport_rotation + s->rotation_offset) == 180))
        {
            gt911_set_xresolution(s, yres);
            gt911_set_yresolution(s, xres);
        } else {
            gt911_set_xresolution(s, xres);
            gt911_set_yresolution(s, yres);
        }
    }

}

static void gt911_init(Object *object)
{
    GT911State *s = GT911(object);

    s->mouse = NULL;
    s->autores = TRUE;

    s->timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, gt911_timer_tick, s);

    gt911_add_handler(s);

    object_property_add(object, "xresolution", "int", gt911_get_prop,
                        gt911_set_resolution_prop, NULL, NULL);

    object_property_add(object, "yresolution", "int", gt911_get_prop,
                        gt911_set_resolution_prop, NULL, NULL);

    object_property_add(object, "rotation", "int", gt911_get_prop,
                        gt911_set_rotation_prop, NULL, NULL);

    object_property_add(object, "rotation_offset", "int", gt911_get_prop,
                        gt911_set_rotation_prop, NULL, NULL);

}



static void gt911_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    dc->realize = gt911_realize;
    dc->vmsd = &vmstate_gt911;
    dc->reset = gt911_reset;
    k->event = gt911_event;
    k->recv = gt911_tx;
    k->send = gt911_rx;

    set_bit(DEVICE_CATEGORY_INPUT, dc->categories);
}

static const TypeInfo gt911_info = {
    .name          = TYPE_GT911,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(GT911State),
    .instance_init = gt911_init,
    .class_init    = gt911_class_init,
};

static void gt911_register_types(void)
{
    type_register_static(&gt911_info);
}

type_init(gt911_register_types)

