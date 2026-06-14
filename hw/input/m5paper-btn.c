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


static void m5paper_btn_kbd_event(DeviceState *dev, QemuConsole *src,
                               InputEvent *evt)
{
    M5PaperBtnState *s = M5PAPER_BTN(dev);

    const int qcode = qemu_input_key_value_to_qcode(evt->u.key.data->key);

    qemu_irq *irq = NULL;
    bool active_low = false;
    switch (qcode) {
    case Q_KEY_CODE_LEFT:
        irq = &s->btn_left;
        active_low = s->btn_left_active_low;
        break;

    case Q_KEY_CODE_RIGHT:
        irq = &s->btn_right;
        active_low = s->btn_right_active_low;
        break;

    case Q_KEY_CODE_UP:
        irq = &s->btn_push;
        active_low = s->btn_push_active_low;
        break;

    default:
        break;
    }

    if (irq != NULL) {
        qemu_set_irq(*irq, (!evt->u.key.data->down) ^ active_low);
    }
}


static const VMStateDescription vmstate_m5paper_btn = {
    .name = "M5Paper-btn",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (VMStateField[]) {
        /* TODO: fields... */
        VMSTATE_END_OF_LIST()
    }
};


static void m5paper_btn_reset(Object *obj, ResetType type)
{
    M5PaperBtnState *s = M5PAPER_BTN(obj);

    qemu_set_irq(s->btn_left, !s->btn_left_active_low);
    qemu_set_irq(s->btn_right, !s->btn_right_active_low);
    qemu_set_irq(s->btn_push, !s->btn_push_active_low);
}

static const QemuInputHandler m5paper_btn_keyboard_handler = {
    .name  = "QEMU M5Paper Button",
    .mask  = INPUT_EVENT_MASK_KEY,
    .event = m5paper_btn_kbd_event,
};

static void m5paper_btn_realize(DeviceState *dev, Error **errp)
{
    M5PaperBtnState *s = M5PAPER_BTN(dev);

    qemu_input_handler_register(dev, &m5paper_btn_keyboard_handler);

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
}

static void m5paper_btn_class_init(ObjectClass *klass, void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = m5paper_btn_realize;
    dc->vmsd = &vmstate_m5paper_btn;

    rc->phases.enter = NULL;
    rc->phases.hold = m5paper_btn_reset;
    rc->phases.exit = NULL;

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
