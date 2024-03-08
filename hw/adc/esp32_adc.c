/*
 * ESP32 SAR ADC emulation.
 *
 * Copyright (C) 2023 Alex Popescu <alex@247dev.ro>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 *
 * This code is inspired by max111x.c, and stm32f2xx_adc.c ADC drivers
 *
 * This is not a complete emulation of the ADC, but only a very basic emulation
 * of SAR1 and SAR2 RTC. It is sufficient to provide a oneshot measurement.
 *
 * Stopping a measurement and more complex scenarios are not tested, and may
 * not work at all.
 *
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/adc/esp32_adc.h"
#include "hw/registerfields.h"
#include "qemu/timer.h"


#ifndef ESP32_ADC_DEBUG_LEVEL
#define ESP32_ADC_DEBUG_LEVEL 0
#endif

#define DPRINTL(lvl, fmt, args...) do { \
    if (ESP32_ADC_DEBUG_LEVEL >= lvl) { \
        qemu_log("%s:%d [%d]: " fmt, __func__, __LINE__, lvl, ## args); \
    } \
} while (0)

#define DPRINT(fmt, args...) DPRINTL(2, fmt, ## args)

#define BADF(fmt, args...) do { \
    qemu_log("%s:%d [X]: " fmt, __func__, __LINE__, ## args); \
    if (IT8951E_DEBUG_LEVEL) { \
        abort(); \
    } \
} while (0)

#define DSTART() DPRINTL(4, "START\n")
#define DEND()   DPRINTL(4, "END\n")

#define ESP32_ADC_SAR1_NUM_INPUTS 8
#define ESP32_ADC_SAR2_NUM_INPUTS 12

REG32(SENS_SAR_READ_CTRL_REG, 0x0000)
    /* [RW] SAR1 default 2 */
    FIELD(SENS_SAR_READ_CTRL_REG, SAR1_CLK_DIV, 0, 8)
    /* [RW] SAR1 default 9 */
    FIELD(SENS_SAR_READ_CTRL_REG, SAR1_SAMPLE_CYCLE, 8, 8)
    /*
     * [RW] Bit width of SAR ADC1,
     *   00: for 9-bit,
     *   01: for 10-bit,
     *   10: for 11-bit,
     *   11: for 12-bit,
     * default: 0
     */
    FIELD(SENS_SAR_READ_CTRL_REG, SAR1_SAMPLE_BIT, 16, 2)
    /*
     * [RW] 1: SAR ADC 1 controlled by DIG ADC1 CTR,
     * 0: by RTC ADC1 CTRL, default 0
     */
    FIELD(SENS_SAR_READ_CTRL_REG, SAR1_DIG_FORCE, 27, 1)
    /* [RW] Invert SAR1 data, default 0 */
    FIELD(SENS_SAR_READ_CTRL_REG, SAR1_DATA_INV, 28, 1)

REG32(SENS_SAR_MEAS_WAIT2_REG, 0x000c)
    /* [RW], default 2 */
    FIELD(SENS_SAR_MEAS_WAIT2_REG, SAR2_RSTB_WAIT, 20, 8)
    /*
     * [RW], default 0,
     *   0: Use FSM to control power down,
     *   2: Force power down,
     *   3: Force power up
     */
    FIELD(SENS_SAR_MEAS_WAIT2_REG, FORCE_XPD_SAR, 18, 2)
    /*
     * [RW], default 0,
     *   0: Use FSM to control power down,
     *   2: Force power down,
     *   3: Force power up
     */
    FIELD(SENS_SAR_MEAS_WAIT2_REG, FORCE_XPD_AMP, 16, 2)
    /* SAR_AMP_WAIT3 */
    FIELD(SENS_SAR_MEAS_WAIT2_REG, SAR_AMP_WAIT3, 0, 16)

REG32(SENS_SAR_START_FORCE_REG, 0x002c)
    /* [RW] Stop SAR1 conversion, default: 0 */
    FIELD(SENS_SAR_START_FORCE_REG, SAR1_STOP, 23, 1)
    /* [RW] Stop SAR2 conversion, default: 0  */
    FIELD(SENS_SAR_START_FORCE_REG, SAR2_STOP, 22, 1)
    /*
     * [RW] SAR2_EN_TEST is active only when reg_sar2_dig_force = 0,
     * default: 0
     */
    FIELD(SENS_SAR_START_FORCE_REG, SAR2_EN_TEST, 4, 1)
    /*
     * [RW] Bit width of SAR ADC2,
     *   00: 9 bits,
     *   01: 10 bits,
     *   10: 11 bits,
     *   11: 12 bits,
     *  default: 0
     */
    FIELD(SENS_SAR_START_FORCE_REG, SAR2_BIT_WIDTH, 2, 2)
    /*
     * [RW] Bit width of SAR ADC1,
     *   00: 9 bits,
     *   01: 10 bits,
     *   10: 11 bits,
     *   11: 12 bits,
     * default: 0
     */
    FIELD(SENS_SAR_START_FORCE_REG, SAR1_BIT_WIDTH, 0, 2)

REG32(SENS_SAR_MEAS_START1_REG, 0x0054)
    /* [RW] SAR ADC1 pad enable bitmap is controlled by SW, default: 0  */
    FIELD(SENS_SAR_MEAS_START1_REG, SAR1_EN_PAD_FORCE, 31, 1)
    /* [RW] SAR ADC1 pad enable bitmap, default: 0  */
    FIELD(SENS_SAR_MEAS_START1_REG, SAR1_EN_PAD, 19, 12)
    /* [RW] SAR ADC1 controller (in RTC) is started by SW, default: 0 */
    FIELD(SENS_SAR_MEAS_START1_REG, MEAS1_START_FORCE, 18, 1)
    /* [RW] SAR ADC1 controller (in RTC) starts conversion, default: 0 */
    FIELD(SENS_SAR_MEAS_START1_REG, MEAS1_START_SAR, 17, 1)
    /* [RO] SAR ADC1 conversion done indication, default: 0 */
    FIELD(SENS_SAR_MEAS_START1_REG, MEAS1_DONE_SAR, 16, 1)
    /* [RO] SAR ADC1 data., default: 0 */
    FIELD(SENS_SAR_MEAS_START1_REG, MEAS1_DATA_SAR, 0, 16)

REG32(SENS_SAR_READ_CTRL2_REG, 0x0090)
    /* [RW] SAR2 default 2 */
    FIELD(SENS_SAR_READ_CTRL2_REG, SAR2_CLK_DIV, 0, 8)
    /* [RW] SAR2 default 9 */
    FIELD(SENS_SAR_READ_CTRL2_REG, SAR2_SAMPLE_CYCLE, 8, 8)
    /*
     * [RW] Bit width of SAR ADC2,
     *   00: for 9-bit,
     *   01: for 10-bit,
     *   10: for 11-bit,
     *   11: for 12-bit,
     *  default: 0
     */
    FIELD(SENS_SAR_READ_CTRL2_REG, SAR2_SAMPLE_BIT, 16, 2)
    /*
     * [RW] 1: SAR ADC2 controlled by DIG ADC2 CTRL or PWDET CTRL,
     * 0: SAR ADC2 controlled by RTC ADC2 CTRL, default: 0
     */
    FIELD(SENS_SAR_READ_CTRL2_REG, SAR2_DIG_FORCE, 28, 1)
    /* [RW] Invert SAR2 data, default 0 */
    FIELD(SENS_SAR_READ_CTRL2_REG, SAR2_DATA_INV, 29, 1)

REG32(SENS_SAR_MEAS_START2_REG, 0x0094)
    FIELD(SENS_SAR_MEAS_START2_REG, SAR2_EN_PAD_FORCE, 31, 1)
    FIELD(SENS_SAR_MEAS_START2_REG, SAR2_EN_PAD, 19, 12)
    FIELD(SENS_SAR_MEAS_START2_REG, MEAS2_START_FORCE, 18, 1)
    FIELD(SENS_SAR_MEAS_START2_REG, MEAS2_START_SAR, 17, 1)
    FIELD(SENS_SAR_MEAS_START2_REG, MEAS2_DONE_SAR, 16, 1)
    FIELD(SENS_SAR_MEAS_START2_REG, MEAS2_DATA_SAR, 0, 16)



/*
 * Do conversion if:
 *   - RTC / DIG set to RTC: SENS.sar_read_ctrl.sar1_dig_force       = 0;
 *   - SW Ctrl ADC, not ULP: SENS.sar_meas_start1.meas1_start_force  = 1;
 *   - Start requested: SENS.sar_meas_start1.meas1_start_sar = 1;
 * SENS.sar_meas_start1.meas1_start_sar = 0;
 */
static void esp32_adc_start(Esp32AdcState *s)
{
    DSTART();
    DPRINTL(3, " FORCE_XPD_SAR     = %08x\n",
            FIELD_EX32(s->regs[A_SENS_SAR_MEAS_WAIT2_REG],
                       SENS_SAR_MEAS_WAIT2_REG, FORCE_XPD_SAR));

    DPRINTL(3, " SAR1_DIG_FORCE    = %08x\n",
            FIELD_EX32(s->regs[A_SENS_SAR_READ_CTRL_REG],
                       SENS_SAR_READ_CTRL_REG, SAR1_DIG_FORCE));

    DPRINTL(3, " MEAS1_START_FORCE = %08x\n",
            FIELD_EX32(s->regs[A_SENS_SAR_MEAS_START1_REG],
                       SENS_SAR_MEAS_START1_REG, MEAS1_START_FORCE));

    DPRINTL(3, " SAR1_EN_PAD_FORCE = %08x\n",
            FIELD_EX32(s->regs[A_SENS_SAR_MEAS_START1_REG],
                       SENS_SAR_MEAS_START1_REG, SAR1_EN_PAD_FORCE));

    DPRINTL(3, " MEAS1_START_SAR   = %08x\n",
            FIELD_EX32(s->regs[A_SENS_SAR_MEAS_START1_REG],
                       SENS_SAR_MEAS_START1_REG, MEAS1_START_SAR));

    if (
            /* Check if SAR in SW Mode and powered up */
            (FIELD_EX32(s->regs[A_SENS_SAR_MEAS_WAIT2_REG],
                        SENS_SAR_MEAS_WAIT2_REG,  FORCE_XPD_SAR)     == 3) &&
            /* Check if SAR1 is in RTC mode */
            (FIELD_EX32(s->regs[A_SENS_SAR_READ_CTRL_REG],
                        SENS_SAR_READ_CTRL_REG,   SAR1_DIG_FORCE)    == 0) &&
            /* Check if RTC ADC is started */
            (FIELD_EX32(s->regs[A_SENS_SAR_MEAS_START1_REG],
                        SENS_SAR_MEAS_START1_REG, MEAS1_START_FORCE) == 1) &&
            /* Check if SW is controlling the RTC ADC bit map */
            (FIELD_EX32(s->regs[A_SENS_SAR_MEAS_START1_REG],
                        SENS_SAR_MEAS_START1_REG, SAR1_EN_PAD_FORCE) == 1) &&
            /* Check if measurement start was requested */
            (FIELD_EX32(s->regs[A_SENS_SAR_MEAS_START1_REG],
                        SENS_SAR_MEAS_START1_REG, MEAS1_START_SAR)   == 1)
    ) {
        /* TODO: calculate delay based on freq / sampling? */
        timer_mod(s->sar1_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 50);
    }
    if (
            /* Check if SAR in SW Mode and powered up */
            (FIELD_EX32(s->regs[A_SENS_SAR_MEAS_WAIT2_REG],
                        SENS_SAR_MEAS_WAIT2_REG,  FORCE_XPD_SAR)     == 3) &&
            /* Check if SAR1 is in RTC mode */
            (FIELD_EX32(s->regs[A_SENS_SAR_READ_CTRL2_REG],
                        SENS_SAR_READ_CTRL2_REG,  SAR2_DIG_FORCE)    == 0) &&
            /* Check if RTC ADC is started */
            (FIELD_EX32(s->regs[A_SENS_SAR_MEAS_START2_REG],
                        SENS_SAR_MEAS_START2_REG, MEAS2_START_FORCE) == 1) &&
            /* Check if SW is controlling the RTC ADC bit map */
            (FIELD_EX32(s->regs[A_SENS_SAR_MEAS_START2_REG],
                        SENS_SAR_MEAS_START2_REG, SAR2_EN_PAD_FORCE) == 1) &&
            /* Check if measurement start was requested */
            (FIELD_EX32(s->regs[A_SENS_SAR_MEAS_START2_REG],
                        SENS_SAR_MEAS_START2_REG, MEAS2_START_SAR)   == 1)
    ) {
        /* TODO: calculate delay based on freq / sampling? */
        timer_mod(s->sar2_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 50);
    }
    DEND();
}

static void esp32_adc_stop(Esp32AdcState *s)
{
    DSTART();
    DEND();
}

static void esp32_adc_sar1_tick(void *opaque)
{
    DSTART();
    Esp32AdcState *s = ESP32_ADC(opaque);

    /* Mark conversion as complete */
    s->regs[A_SENS_SAR_MEAS_START1_REG] = FIELD_DP32(
        s->regs[A_SENS_SAR_MEAS_START1_REG],
        SENS_SAR_MEAS_START1_REG,
        MEAS1_DONE_SAR,
        1);

    /* Write value */
    uint32_t data = s->sar1_value[s->sar1_channel] >> (12 - s->sar1_bits);
    if (s->sar1_inv) {
        data = 4095 - data;
    }
    s->regs[A_SENS_SAR_MEAS_START1_REG] = FIELD_DP32(
        s->regs[A_SENS_SAR_MEAS_START1_REG],
        SENS_SAR_MEAS_START1_REG,
        MEAS1_DATA_SAR,
        data);

    /* Clear the measurement start request? */
    s->regs[A_SENS_SAR_MEAS_START1_REG] = FIELD_DP32(
        s->regs[A_SENS_SAR_MEAS_START1_REG],
        SENS_SAR_MEAS_START1_REG,
        MEAS1_START_SAR,
        0);


    DPRINTL(3, "ADC SAR1 complete: %08x\n",
            s->regs[A_SENS_SAR_MEAS_START1_REG]);
    DEND();
}

static void esp32_adc_sar2_tick(void *opaque)
{
    DSTART();
    Esp32AdcState *s = ESP32_ADC(opaque);

    /* Mark conversion as complete */
    s->regs[A_SENS_SAR_MEAS_START2_REG] = FIELD_DP32(
        s->regs[A_SENS_SAR_MEAS_START2_REG],
        SENS_SAR_MEAS_START2_REG,
        MEAS2_DONE_SAR,
        1);

    /* Write value */
    uint32_t data = s->sar2_value[s->sar2_channel] >> (12 - s->sar2_bits);
    if (s->sar2_inv) {
        data = 4095 - data;
    }
    s->regs[A_SENS_SAR_MEAS_START2_REG] = FIELD_DP32(
        s->regs[A_SENS_SAR_MEAS_START2_REG],
        SENS_SAR_MEAS_START2_REG,
        MEAS2_DATA_SAR,
        data);

    /* Clear the measurement start request? */
    s->regs[A_SENS_SAR_MEAS_START2_REG] = FIELD_DP32(
        s->regs[A_SENS_SAR_MEAS_START2_REG],
        SENS_SAR_MEAS_START2_REG,
        MEAS2_START_SAR,
        0);

    DPRINTL(3, "ADC SAR2 complete: %08x\n",
            s->regs[A_SENS_SAR_MEAS_START2_REG]);
    DEND();
}

static void esp32_adc_write(void *opaque, hwaddr addr,
                       uint64_t val64, unsigned int size)
{
    DSTART();
    Esp32AdcState *s = ESP32_ADC(opaque);
    uint32_t data;

    switch (addr) {
    case A_SENS_SAR_READ_CTRL_REG:
        DPRINTL(3, "write: SENS_SAR_READ_CTRL_REG     :: %08lx\n", val64);
        s->sar1_sample_bits = 9 + FIELD_EX32(val64, SENS_SAR_READ_CTRL_REG,
                                             SAR1_SAMPLE_BIT);

        s->sar1_inv = FIELD_EX32(val64, SENS_SAR_READ_CTRL_REG, SAR1_DATA_INV);
        DPRINTL(3, "       >> SAR1 Clock divider: %d\n",
                FIELD_EX32(val64, SENS_SAR_READ_CTRL_REG, SAR1_CLK_DIV));
        DPRINTL(3, "       >> SAR1 Sample cycle: %d\n",
                FIELD_EX32(val64, SENS_SAR_READ_CTRL_REG, SAR1_SAMPLE_BIT));
        DPRINTL(3, "       >> SAR1 Sample bits: %d\n",
                s->sar1_sample_bits);
        DPRINTL(3, "       >> SAR1 DIG Force: %d\n",
                FIELD_EX32(val64, SENS_SAR_READ_CTRL_REG, SAR1_DIG_FORCE));
        DPRINTL(3, "       >> SAR1 Data invert: %d\n",
                s->sar1_inv);
        break;

    case A_SENS_SAR_READ_CTRL2_REG:
        DPRINTL(3, "write: SENS_SAR_READ_CTRL2_REG     :: %08lx\n", val64);
        s->sar2_sample_bits = 9 + FIELD_EX32(val64, SENS_SAR_READ_CTRL2_REG,
                                             SAR2_SAMPLE_BIT);

        s->sar2_inv = FIELD_EX32(val64, SENS_SAR_READ_CTRL2_REG, SAR2_DATA_INV);
        DPRINTL(3, "       >> SAR2 Clock divider: %d\n",
                FIELD_EX32(val64, SENS_SAR_READ_CTRL2_REG, SAR2_CLK_DIV));
        DPRINTL(3, "       >> SAR2 Sample cycle: %d\n",
                FIELD_EX32(val64, SENS_SAR_READ_CTRL2_REG, SAR2_SAMPLE_BIT));
        DPRINTL(3, "       >> SAR2 Sample bits: %d\n",
                s->sar2_sample_bits);
        DPRINTL(3, "       >> SAR2 DIG Force: %d\n",
                FIELD_EX32(val64, SENS_SAR_READ_CTRL2_REG, SAR2_DIG_FORCE));
        DPRINTL(3, "       >> SAR2 Data invert: %d\n",
                s->sar2_inv);
        break;

    case A_SENS_SAR_MEAS_START1_REG:
        DPRINTL(3, "write: SENS_SAR_MEAS1_CTRL2_REG   :: %08lx\n", val64);
        s->sar1_channel = __builtin_ffs(FIELD_EX32(val64,
                SENS_SAR_MEAS_START1_REG, SAR1_EN_PAD));

        data = FIELD_EX32(s->regs[addr], SENS_SAR_MEAS_START1_REG,
                          MEAS1_START_SAR);

        s->regs[addr] = val64;

        if ((data == 0) && (data != FIELD_EX32(val64, SENS_SAR_MEAS_START1_REG,
            MEAS1_START_SAR))) {
            esp32_adc_start(s);
        } else if ((data == 1) && (data != FIELD_EX32(val64,
            SENS_SAR_MEAS_START1_REG, MEAS1_START_SAR))) {
            esp32_adc_stop(s);
        }

        assert(s->sar1_channel < 16);
        DPRINTL(3, "       >> Activate channel %d\n", s->sar1_channel);
        break;


    case A_SENS_SAR_START_FORCE_REG:
        DPRINTL(3, "write: SENS_SAR_START_FORCE_REG   :: %08lx\n", val64);
        s->sar1_bits = 9 + FIELD_EX32(val64, SENS_SAR_START_FORCE_REG,
                                      SAR1_BIT_WIDTH);

        s->sar2_bits = 9 + FIELD_EX32(val64, SENS_SAR_START_FORCE_REG,
                                      SAR2_BIT_WIDTH);

        DPRINTL(4, "          >> SAR 1 bits: %d\n", s->sar1_bits);
        DPRINTL(4, "          >> SAR 2 bits: %d\n", s->sar2_bits);
        DPRINTL(4, "          >> SAR 2 EN TEST: %d\n",
                FIELD_EX32(val64, SENS_SAR_START_FORCE_REG, SAR2_EN_TEST));
        break;


    default:
        DPRINTL(3, "write: 0x%08lx\n", addr);
        break;
    }
    s->regs[addr] = val64;

    DEND();
}

static uint64_t esp32_adc_read(void *opaque, hwaddr addr,
                                     unsigned int size)
{
    Esp32AdcState *s = ESP32_ADC(opaque);

    return s->regs[addr];
}


static const VMStateDescription vmstate_esp32_adc = {
    .name = TYPE_ESP32_ADC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        /* TODO: add fields */
        VMSTATE_END_OF_LIST()
    }
};

static void esp32_adc_sar1_input_set(void *opaque, int line, int value)
{
    DSTART();
    Esp32AdcState *s = ESP32_ADC(opaque);

    assert(line >= 0 && line < ESP32_ADC_SAR1_NUM_INPUTS);
    s->sar1_value[line] = value;
    DEND();
}

static void esp32_adc_sar2_input_set(void *opaque, int line, int value)
{
    DSTART();
    Esp32AdcState *s = ESP32_ADC(opaque);

    assert(line >= 0 && line < ESP32_ADC_SAR2_NUM_INPUTS);
    s->sar2_value[line] = value;
    DEND();
}


static void esp32_adc_reset(DeviceState *dev)
{
    DSTART();
    Esp32AdcState *s = ESP32_ADC(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[A_SENS_SAR_READ_CTRL_REG] = 0x92;

    s->sar1_bits = 9;
    s->sar2_bits = 9;

    for (int channel = 0; channel < ESP32_ADC_SAR1_NUM_INPUTS; channel++) {
        s->sar1_value[channel] = s->sar1_reset_value[channel];
    }

    for (int channel = 0; channel < ESP32_ADC_SAR2_NUM_INPUTS; channel++) {
        s->sar2_value[channel] = s->sar2_reset_value[channel];
    }


    DEND();
}

static Property esp32_adc_properties[] = {
    /* Reset values for SAR1 ADC inputs */
    DEFINE_PROP_UINT16("sar1input0", Esp32AdcState, sar1_reset_value[0], 2048),
    DEFINE_PROP_UINT16("sar1input1", Esp32AdcState, sar1_reset_value[1], 2048),
    DEFINE_PROP_UINT16("sar1input2", Esp32AdcState, sar1_reset_value[2], 2048),
    DEFINE_PROP_UINT16("sar1input3", Esp32AdcState, sar1_reset_value[3], 2048),
    DEFINE_PROP_UINT16("sar1input4", Esp32AdcState, sar1_reset_value[4], 2048),
    DEFINE_PROP_UINT16("sar1input5", Esp32AdcState, sar1_reset_value[5], 2048),
    DEFINE_PROP_UINT16("sar1input6", Esp32AdcState, sar1_reset_value[6], 2048),
    DEFINE_PROP_UINT16("sar1input7", Esp32AdcState, sar1_reset_value[7], 2048),

    /* Reset values for SAR2 ADC inputs */
    DEFINE_PROP_UINT16("sar2input0", Esp32AdcState, sar2_reset_value[0], 2048),
    DEFINE_PROP_UINT16("sar2input1", Esp32AdcState, sar2_reset_value[1], 2048),
    DEFINE_PROP_UINT16("sar2input2", Esp32AdcState, sar2_reset_value[2], 2048),
    DEFINE_PROP_UINT16("sar2input3", Esp32AdcState, sar2_reset_value[3], 2048),
    DEFINE_PROP_UINT16("sar2input4", Esp32AdcState, sar2_reset_value[4], 2048),
    DEFINE_PROP_UINT16("sar2input5", Esp32AdcState, sar2_reset_value[5], 2048),
    DEFINE_PROP_UINT16("sar2input6", Esp32AdcState, sar2_reset_value[6], 2048),
    DEFINE_PROP_UINT16("sar2input7", Esp32AdcState, sar2_reset_value[7], 2048),
    DEFINE_PROP_UINT16("sar2input8", Esp32AdcState, sar2_reset_value[8], 2048),
    DEFINE_PROP_UINT16("sar2input9", Esp32AdcState, sar2_reset_value[9], 2048),
    DEFINE_PROP_END_OF_LIST(),
};

static const MemoryRegionOps esp32_adc_ops = {
    .read = esp32_adc_read,
    .write = esp32_adc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void esp32_adc_init(Object *obj)
{
    Esp32AdcState *s = ESP32_ADC(obj);

    /* Triggers when conversions are ready */
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    /* Provide ADC data from another device / input */
    qdev_init_gpio_in(DEVICE(obj), esp32_adc_sar1_input_set,
                      ESP32_ADC_SAR1_NUM_INPUTS);
    qdev_init_gpio_in(DEVICE(obj), esp32_adc_sar2_input_set,
                      ESP32_ADC_SAR2_NUM_INPUTS);

    memory_region_init_io(&s->mmio, obj, &esp32_adc_ops, s,
                          TYPE_ESP32_ADC, 0x100);

    s->sar1_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, esp32_adc_sar1_tick, s);
    s->sar2_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, esp32_adc_sar2_tick, s);

    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static void esp32_adc_class_init(ObjectClass *object_class, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(object_class);

    dc->reset = esp32_adc_reset;
    dc->vmsd = &vmstate_esp32_adc;

    device_class_set_props(dc, esp32_adc_properties);
}

static const TypeInfo esp32_adc_info = {
    .name          = TYPE_ESP32_ADC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32AdcState),
    .instance_init = esp32_adc_init,
    .class_init    = esp32_adc_class_init,
};

static void esp32_adc_register_types(void)
{
    type_register_static(&esp32_adc_info);
}

type_init(esp32_adc_register_types)

