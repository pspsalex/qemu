/*
 * M5Paper Button emulation
 *
 * Copyright (C) 2023 Alex Popescu <alex@247dev.ro>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 *
 */

/* I2C device */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "ui/console.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "qemu/log.h"
#include "hw/qdev-properties.h"
#include "hw/input/m5paper-btn.h"


#ifndef M5PAPER_BTN_DEBUG_LEVEL
#define M5PAPER_BTN_DEBUG_LEVEL 0
#endif

#define DPRINTL(lvl, fmt, args...) do { \
    if (M5PAPER_BTN_DEBUG_LEVEL >= lvl) { \
        qemu_log("%s:%d [%d]: " fmt, __func__, __LINE__, lvl, ## args); \
    } \
} while (0)

#define DPRINTF(fmt, args...) DPRINTL(2, fmt, ## args)

#define BADF(fmt, args...) do { \
    qemu_log("%s:%d [X]: " fmt, __func__, __LINE__, ## args); \
    if (M5PAPER_BTN_DEBUG_LEVEL) { \
        abort(); \
    } \
} while (0)

#define DSTART() DPRINTL(4, "START\n")
#define DEND()   DPRINTL(4, "END\n")




static void m5paper_btn_kbd_event(void *opaque, int keycode)
{
    M5PaperBtnState *s = M5PAPER_BTN(opaque);

    if (keycode == 0xe0) {
        s->extended = true;
        return;
    }

    if (!s->extended) {
        return;
    }

    qemu_irq *irq = NULL;
    bool active_low = false;
    switch (keycode & 0x7F) {
    case 0x4B:
        irq = &s->btn_left;
        active_low = s->btn_left_active_low;
        break;

    case 0x4D:
        irq = &s->btn_right;
        active_low = s->btn_right_active_low;
        break;

    case 0x50:
        irq = &s->btn_push;
        active_low = s->btn_push_active_low;
        break;

    default:
        break;
    }

    if (irq != NULL) {
        qemu_set_irq(*irq, (!(keycode & 0x80)) ^ active_low);
    }
    s->extended = false;
}


static void m5paper_btn_add_handler(M5PaperBtnState *s)
{
    if (s->kbd == NULL) {
        s->kbd = qemu_add_kbd_event_handler(m5paper_btn_kbd_event, s);
    }
}

static int m5paper_btn_post_load(void *opaque, int version_id)
{
    M5PaperBtnState *s = M5PAPER_BTN(opaque);

    m5paper_btn_add_handler(s);

    return 0;
}



static const VMStateDescription vmstate_m5paper_btn = {
    .name = "M5Paper-btn",
    .version_id = 0,
    .minimum_version_id = 0,
    .post_load = m5paper_btn_post_load,
    .fields = (VMStateField[]) {
        /* TODO: fields... */
        VMSTATE_END_OF_LIST()
    }
};


static void m5paper_btn_reset(DeviceState *dev)
{
    M5PaperBtnState *s = M5PAPER_BTN(dev);

    s->pressed = false;

    m5paper_btn_post_load(s, 0);

    qemu_set_irq(s->btn_left, !s->btn_left_active_low);
    qemu_set_irq(s->btn_right, !s->btn_right_active_low);
    qemu_set_irq(s->btn_push, !s->btn_push_active_low);
}

static void m5paper_btn_realize(DeviceState *dev, Error **errp)
{
    M5PaperBtnState *s = M5PAPER_BTN(dev);

    qdev_init_gpio_out_named(dev, &s->btn_left, M5PAPER_BTN_LEFT, 1);
    qdev_init_gpio_out_named(dev, &s->btn_right, M5PAPER_BTN_RIGHT, 1);
    qdev_init_gpio_out_named(dev, &s->btn_push, M5PAPER_BTN_PUSH, 1);

}

static Property m5paper_btn_properties[] = {
    DEFINE_PROP_BOOL("btn-left-active-low", M5PaperBtnState,
                     btn_left_active_low, true),

    DEFINE_PROP_BOOL("btn-right-active-low", M5PaperBtnState,
                     btn_right_active_low, true),

    DEFINE_PROP_BOOL("btn-push-active-low", M5PaperBtnState,
                     btn_push_active_low, true),

    DEFINE_PROP_END_OF_LIST(),
};

static void  m5paper_btn_init(Object *object)
{
    M5PaperBtnState *s = M5PAPER_BTN(object);

    s->kbd = NULL;

    s->btn_left_active_low = false;
    s->btn_right_active_low = false;
    s->btn_push_active_low = false;

    m5paper_btn_add_handler(s);
}

static void m5paper_btn_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = m5paper_btn_realize;
    dc->vmsd = &vmstate_m5paper_btn;
    dc->reset = m5paper_btn_reset;

    device_class_set_props(dc, m5paper_btn_properties);

    set_bit(DEVICE_CATEGORY_INPUT, dc->categories);
}

static const TypeInfo m5paper_btn_info = {
    .name          = TYPE_M5PAPER_BTN,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(M5PaperBtnState),
    .instance_init = m5paper_btn_init,
    .class_init    = m5paper_btn_class_init,
};

static void m5paper_btn_register_types(void)
{
    type_register_static(&m5paper_btn_info);
}

type_init(m5paper_btn_register_types)

