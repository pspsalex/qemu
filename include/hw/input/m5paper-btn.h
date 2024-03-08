#pragma once

#include "qemu/typedefs.h"
#include "ui/console.h"
#include "hw/sysbus.h"

#define M5PAPER_BTN_LEFT  "m5paper-btn-left"
#define M5PAPER_BTN_RIGHT "m5paper-btn-right"
#define M5PAPER_BTN_PUSH  "m5paper-btn-push"


#define TYPE_M5PAPER_BTN "m5paper-btn"
OBJECT_DECLARE_SIMPLE_TYPE(M5PaperBtnState, M5PAPER_BTN)

struct M5PaperBtnState {
    SysBusDevice qdev;

    bool pressed;
    bool extended;

    qemu_irq btn_left;
    qemu_irq btn_right;
    qemu_irq btn_push;

    bool btn_left_active_low;
    bool btn_right_active_low;
    bool btn_push_active_low;

    QEMUPutKbdEntry *kbd;
};
