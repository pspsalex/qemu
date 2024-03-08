#pragma once

#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/registerfields.h"

#define TYPE_ESP32_GPIO "esp32.gpio"
#define ESP32_GPIO(obj)             OBJECT_CHECK(Esp32GpioState, (obj), TYPE_ESP32_GPIO)
#define ESP32_GPIO_GET_CLASS(obj)   OBJECT_GET_CLASS(Esp32GpioClass, obj, TYPE_ESP32_GPIO)
#define ESP32_GPIO_CLASS(klass)     OBJECT_CLASS_CHECK(Esp32GpioClass, klass, TYPE_ESP32_GPIO)

REG32(GPIO_OUT_REG, 0x0004)
REG32(GPIO_OUT_W1TS_REG, 0x0008)
REG32(GPIO_OUT_W1TC_REG, 0x000C)

REG32(GPIO_OUT1_REG, 0x0010)
REG32(GPIO_OUT1_W1TS_REG, 0x0014)
REG32(GPIO_OUT1_W1TC_REG, 0x0018)

REG32(GPIO_ENABLE_REG, 0x0020)
REG32(GPIO_ENABLE_W1TS_REG, 0x0024)
REG32(GPIO_ENABLE_W1TC_REG, 0x0028)

REG32(GPIO_ENABLE1_REG, 0x002C)
REG32(GPIO_ENABLE1_W1TS_REG, 0x0030)
REG32(GPIO_ENABLE1_W1TC_REG, 0x0034)


REG32(GPIO_STRAP, 0x0038)
REG32(GPIO_IN_REG, 0x003c)
REG32(GPIO_IN1_REG, 0x0040)

REG32(GPIO_STATUS_REG, 0x0044)
REG32(GPIO_STATUS_W1TS_REG, 0x0048)
REG32(GPIO_STATUS_W1TC_REG, 0x004c)

REG32(GPIO_STATUS1_REG, 0x0050)
REG32(GPIO_STATUS1_W1TS_REG, 0x0054)
REG32(GPIO_STATUS1_W1TC_REG, 0x0058)

REG32(GPIO_ACPU_INT_REG, 0x0060)
REG32(GPIO_ACPU_NMI_INT_REG, 0x0064)
REG32(GPIO_PCPU_INT_REG, 0x0068)
REG32(GPIO_PCPU_NMI_INT_REG, 0x006c)

REG32(GPIO_ACPU_INT1_REG, 0x0074)
REG32(GPIO_ACPU_NMI_INT1_REG, 0x0078)
REG32(GPIO_PCPU_INT1_REG, 0x007C)
REG32(GPIO_PCPU_NMI_INT1_REG, 0x0080)

REG32(GPIO_PIN0_REG, 0x0088)
    FIELD(GPIO_PIN0_REG, SYNC2_BYPASS, 0, 2)
    FIELD(GPIO_PIN0_REG, PAD_DRIVER, 2, 1)
    FIELD(GPIO_PIN0_REG, SYNC1_BYPASS, 3, 2)
    FIELD(GPIO_PIN0_REG, INT_TYPE, 7, 3)
    FIELD(GPIO_PIN0_REG, WAKEUP_ENABLE, 10, 1)
    FIELD(GPIO_PIN0_REG, CONFIG, 11, 2)
    FIELD(GPIO_PIN0_REG, INT_ENA, 13, 4)



REG32(GPIO_FUNC0_OUT_SEL_CFG_REG, 0x0530)

#define ESP32_STRAP_MODE_FLASH_BOOT 0x12
#define ESP32_STRAP_MODE_UART_BOOT  0x0f

typedef struct Esp32GpioState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq gpio_irq[48];
    uint32_t strap_mode;
    uint64_t gpio_val;
    uint64_t gpio_sr;
    uint64_t gpio_out_ena;
    uint64_t gpio_app_int;
    uint64_t gpio_app_nmi;
    uint64_t gpio_pro_int;
    uint64_t gpio_pro_nmi;
    uint64_t gpio_pro_int_mask;
    uint64_t gpio_pro_nmi_mask;
    uint64_t gpio_app_int_mask;
    uint64_t gpio_app_nmi_mask;
    uint32_t gpio_cfg[48];
    uint32_t gpio_per[256];
} Esp32GpioState;

typedef struct Esp32GpioClass {
    SysBusDeviceClass parent_class;
} Esp32GpioClass;
