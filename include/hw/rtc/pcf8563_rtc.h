// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Real-time clock/calendar PCF8563 with I2C interface.
 *
 * Datasheet: https://www.nxp.com/docs/en/data-sheet/PCF8563.pdf
 *
 * Author (c) 2024 Ilya Chichkov <i.chichkov@yadro.com>
 */

#ifndef HW_PCF8563_RTC_H
#define HW_PCF8563_RTC_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/bitops.h"
#include "hw/qdev-properties.h"
#include "qemu/timer.h"
#include "hw/i2c/i2c.h"

#define  PCF8563_CS1            0x00
#define  PCF8563_CS2            0x01
#define  PCF8563_VLS            0x02
#define  PCF8563_MINUTES        0x03
#define  PCF8563_HOURS          0x04
#define  PCF8563_DAYS           0x05
#define  PCF8563_WEEKDAYS       0x06
#define  PCF8563_CENTURY_MONTHS 0x07
#define  PCF8563_YEARS          0x08
#define  PCF8563_MINUTE_A       0x09
#define  PCF8563_HOUR_A         0x0A
#define  PCF8563_DAY_A          0x0B
#define  PCF8563_WEEKDAY_A      0x0C
#define  PCF8563_CLKOUT_CTL     0x0D
#define  PCF8563_TIMER_CTL      0x0E
#define  PCF8563_TIMER          0x0F

#define TYPE_PCF8563 "pcf8563"
OBJECT_DECLARE_SIMPLE_TYPE(Pcf8563State, PCF8563)

typedef struct Pcf8563State {
    I2CSlave parent_obj;

    struct tm current_time;
    qemu_irq irq;

    uint8_t read_index;
    uint8_t write_index;
    uint8_t reg_addr;

    /* Control and status */
    uint8_t cs1;
    uint8_t cs2;
    /* Counters */
    uint8_t vls;
    uint8_t minutes;
    uint8_t hours;
    uint8_t days;
    uint8_t weekdays;
    uint8_t centure_months;
    uint8_t years;
    /* Alarm registers */
    uint8_t minute_a;
    uint8_t hour_a;
    uint8_t day_a;
    uint8_t weekday_a;
    /* Timer control */
    uint8_t clkout_ctl;
    uint8_t timer_ctl;
    uint8_t timer_cnt;

    QEMUTimer *alarm_timer;
    struct tm tm_alarm;
    bool alarm_irq;
    QEMUTimer *timer;
    time_t time_offset;
    time_t stop_time;
    QEMUTimer *irq_gen_timer;
} Pcf8563State;

#endif /* HW_PCF8563_RTC_H */
