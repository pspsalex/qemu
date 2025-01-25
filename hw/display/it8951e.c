/*
 * IT8951E display driver for M5Paper
 *
 * Copyright (C) 2023 Alex Popescu <alex@247dev.ro>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 *
 * This code is based on the ssd0323.c driver.
 *
 */

#include "qemu/osdep.h"
#include "hw/ssi/ssi.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "ui/console.h"
#include "qom/object.h"
#include "hw/display/it8951e.h"
#include "hw/irq.h"
#include "qemu/log.h"
#include "hw/qdev-properties.h"

#ifndef IT8951E_DEBUG_LEVEL
#define IT8951E_DEBUG_LEVEL 0
#endif

#define DPRINTL(lvl, fmt, args...) do { \
    if (IT8951E_DEBUG_LEVEL >= lvl) { \
        qemu_log("%s:%d [%d]: " fmt, __func__, __LINE__, lvl, ## args); \
    } \
} while (0)

#define DPRINTF(fmt, args...) DPRINTL(2, fmt, ## args)

#define BADF(fmt, args...) do { \
    qemu_log("%s:%d [X]: " fmt, __func__, __LINE__, ## args); \
    if (IT8951E_DEBUG_LEVEL) { \
        abort(); \
    } \
} while (0)

#define DSTART() DPRINTL(4, "START\n")
#define DEND()   DPRINTL(4, "END\n")

/* Main SM */
enum it8951e_msm_state {
    IT8951E_MSM_STANDBY = 0,
    IT8951E_MSM_WAIT_COMMAND,
    IT8951E_MSM_CMD_0302_PRE,
    IT8951E_MSM_CMD_0302,
    IT8951E_MSM_WAIT_LD_AREA_PARM,
    IT8951E_MSM_WAIT_LD_AREA_DATA,
    IT8951E_MSM_WAIT_LD_AREA_DATA_PRE,
    IT8951E_MSM_WAIT_TRANSFER,
    IT8951E_MSM_WAIT_REG_RD_PARM,
    IT8951E_MSM_WAIT_REG_WR_PARM,
    IT8951E_MSM_WAIT_MEM_BST_RD_T_PARM,
    IT8951E_MSM_WAIT_MEM_BST_WR_PARM,
    IT8951E_MSM_WAIT_LD_IMG_PARM,
    IT8951E_MSM_WAIT_REG_WR_VAL,
    IT8951E_MSM_WAIT_REG_RD_VAL,
    IT8951E_MSM_WAIT_DPY_BUF_AREA_PARM,
    IT8951E_MSM_WAIT_SET_VCOM_PARM
};

enum it8951e_msm_event {
    IT8951E_MSM_EV_NONE,
    IT8951E_MSM_EV_RECVB,
    IT8951E_MSM_EV_RECV16B,
    IT8951E_MSM_EV_COMMAND,
    IT8951E_MSM_EV_BEGIN_WRITE,
    IT8951E_MSM_EV_TX_DONE,
    IT8951E_MSM_EV_CS_OFF,
    IT8951E_MSM_EV_IMG_END
};

/* Transfer SM */
enum it8951e_tsm_state {
    IT8951E_TSM_STANDBY = 0,
    IT8951E_TSM_WAITLB,
    IT8951E_TSM_WAIT_COMMANDHB,
    IT8951E_TSM_WAIT_COMMANDLB,
    IT8951E_TSM_WAIT_WRITE_END,
    IT8951E_TSM_SEND_DATA_DUMMY,
    IT8951E_TSM_SEND_DATA
};

enum it8951e_tsm_event {
    IT8951E_TSM_EV_NONE,
    IT8951E_TSM_EV_TRANSFER,
    IT8951E_TSM_EV_READ16B,
    IT8951E_TSM_EV_READ8B,
    IT8951E_TSM_EV_READNB
};


/* Main SM */
char it8951e_msm_event_names[][64] = {
    "IT8951E_MSM_EV_NONE",
    "IT8951E_MSM_EV_RECVB",
    "IT8951E_MSM_EV_RECV16B",
    "IT8951E_MSM_EV_COMMAND",
    "IT8951E_MSM_EV_BEGIN_WRITE",
    "IT8951E_MSM_EV_TX_DONE",
    "IT8951E_MSM_EV_CS_OFF",
    "IT8951E_MSM_EV_IMG_END",
};

/* Transfer SM */
char it8951e_tsm_event_names[][64] = {
    "IT8951E_TSM_EV_NONE",
    "IT8951E_TSM_EV_TRANSFER",
    "IT8951E_TSM_EV_READ16B",
    "IT8951E_TSM_EV_READ8B",
    "IT8951E_TSM_EV_READNB"
};




#define IT8951E_PREAMBLE_COMMAND    (0x6000u)
#define IT8951E_PREAMBLE_WRITE_DATA (0x0000u)
#define IT8951E_PREAMBLE_READ_DATA  (0x1000u)

#define IT8951E_REG_I80CPCR         (0x0004u)

#define IT8951E_REG_LISAR_LW        (0x0208u)
#define IT8951E_REG_LISAR_HW        (0x020au)

#define IT8951E_CMD_SYS_RUN         (0x0001u)
#define IT8951E_CMD_STANDBY         (0x0002u)
#define IT8951E_CMD_SLEEP           (0x0003u)
#define IT8951E_CMD_REG_RD          (0x0010u)
#define IT8951E_CMD_REG_WR          (0x0011u)
#define IT8951E_CMD_MEM_BST_RD_T    (0x0012u)
#define IT8951E_CMD_MEM_BST_RD_S    (0x0013u)
#define IT8951E_CMD_MEM_BST_WR      (0x0014u)
#define IT8951E_CMD_MEM_BST_END     (0x0015u)
#define IT8951E_CMD_LD_IMG          (0x0020u)
#define IT8951E_CMD_LD_AREA         (0x0021u)
#define IT8951E_CMD_LD_IMG_END      (0x0022u)

#define IT8951E_CMD_GET_DEVICE_INFO (0x0302u)
#define IT8951E_CMD_DISPLAY_AREA    (0x0034u)
#define IT8951E_CMD_DPY_BUF_AREA    (0x0037u)
#define IT8951E_CMD_POWER_ON_OFF    (0x0038u)
#define IT8951E_CMD_SET_VCOM        (0x0039u)
#define IT8951E_CMD_FORCE_SET_TEMP  (0x0040u)

#define IT8951E_LD_BUF_SIZE         (2048u)
struct it8951e_device_info_t {
    uint16_t panelWidth;
    uint16_t panelHeight;
    uint16_t imgBufAddrL;
    uint16_t imgBufAddrH;
    char fwVersion[16];
    char lutVersion[16];
} __packed;

#define IT8951E_FB_ADDRESS (0x001236e0)

struct it8951e_device_info_t it8951e_device_info = {
    .panelWidth = __bswap_constant_16(IT8951E_WIDTH),
    .panelHeight = __bswap_constant_16(IT8951E_HEIGHT),
    .imgBufAddrH = __bswap_constant_16(IT8951E_FB_ADDRESS >> 16),
    .imgBufAddrL = __bswap_constant_16(IT8951E_FB_ADDRESS & 0x0000FFFF),
    .fwVersion = "0123456789ABCDE",
    .lutVersion = "fedcbaGHIJKLMNO"
};


static void it8951e_update_display(void *opaque);
static void it8951e_invalidate_display(void *opaque);

static void it8951e_blit(it8951e_state *s, uint8_t data)
{
    /* TODO: Make rotations great again */

    /*
     * ROT = 0, X++, Y++
     * ROT = 1 ==> 90deg --> 0,0 is bottom left; each pixel decreases y to 0, on
     * y = 0, x is incremented and y is reset to HEIGHT
     *
     */

    assert(data < 16);

    if ((s->crsr.posx >= s->crsr.x) &&
        (s->crsr.posx < (s->crsr.x + s->crsr.w)))
    {
        switch (s->rotation) {
        case 0:
            if ((s->crsr.posx <= IT8951E_WIDTH) &&
                (s->crsr.posy <= IT8951E_HEIGHT)) {
                s->framebuffer[s->crsr.posx + s->crsr.posy * IT8951E_WIDTH] =
                    data;
            }
            break;

        case 1:
            if ((s->crsr.posx <= IT8951E_HEIGHT) &&
                (s->crsr.posy <= IT8951E_WIDTH)) {
                s->framebuffer[(IT8951E_HEIGHT - s->crsr.posx) * IT8951E_WIDTH +
                    (s->crsr.posy)] = data;
            }
            break;

        default:
            BADF("Unkwnown blit orient\n");
            break;
        }
    }

    s->crsr.posx++;
    if (s->crsr.posx >= (s->crsr.x + s->crsr.dummyw)) {
        s->crsr.posx = s->crsr.dummyx;
        s->crsr.posy++;
        if (s->crsr.posy >= s->crsr.maxy) {
            s->crsr.posy = s->crsr.y;
            s->redraw = 1;
        }
    }

}

static void it8951e_readb(it8951e_state *s, uint32_t n)
{
    s->needed_bytes = n;
    s->transferred_bytes = 0;
    switch (s->tsm) {
    case IT8951E_TSM_STANDBY:
        s->tsm = IT8951E_TSM_WAITLB;
        break;

    case IT8951E_TSM_WAIT_WRITE_END:
        break;

    default:
        break;
    }
}

typedef void (*it8951_state_func)(it8951e_state *s,
                                  enum it8951e_msm_event event);

static void it8951e_main_sm(it8951e_state *s, enum it8951e_msm_event event)
{
    assert(s->state);
    ((it8951_state_func)s->state)(s, event);
}

static void it8951e_msm_standby(it8951e_state *s, enum it8951e_msm_event event);

/**
 * Main State Machine
 *
@startuml
state c <<choice>>
state c2 <<choice>>
state c3 <<choice>>
state c4 <<choice>>

[*] -> standby
standby --> c : ev_command

c --> wait_reg_rd_parm : [REG_RD]
wait_reg_rd_parm --> wait_reg_rd_parm : ev_begin_write / readb(2)
wait_reg_rd_parm --> wait_reg_rd_val : ev_tx_done
wait_reg_rd_val --> standby : ev_tx_done


c --> wait_reg_wr_parm : [REG_WR]
wait_reg_wr_parm --> wait_reg_wr_parm : ev_begin_write / readb(2)
wait_reg_wr_parm --> wait_reg_wr_val : ev_tx_done
wait_reg_wr_val --> standby : ev_tx_done


c --> wait_mem_bst_rd_t_parm : [MEM_BST_RD_T]
wait_mem_bst_rd_t_parm --> wait_mem_bst_rd_t_parm : ev_begin_write / readb(8)
wait_mem_bst_rd_t_parm --> standby : ev_tx_done


c --> wait_mem_bst_wr_parm : [MEM_BST_WR]
wait_mem_bst_wr_parm --> wait_mem_bst_wr_parm : ev_begin_write / readb(8)
wait_mem_bst_wr_parm --> standby : ev_tx_done


c --> wait_ld_img_parm : [LD_IMG]
wait_ld_img_parm  --> wait_ld_img_parm : ev_begin_write / readb(2)
wait_ld_img_parm --> standby : ev_tx_done

c --> wait_ld_area_parm : [LD_AREA]
wait_ld_area_parm --> wait_ld_area_data : ev_tx_done
wait_ld_area_data --> wait_ld_area_data : ev_begin_write / readb (BUFSZ)
wait_ld_area_data --> wait_ld_area_data : ev_tx_done
wait_ld_area_data --> wait_ld_area_data : ev_cs_off
note on link
buffer flush
end note
wait_ld_area_data --> c4: ev_command
c4 --> standby : [LD_IMG_END]
c4 --> wait_ld_area_data : [else]


c --> wait_dpy_buf_area_parm: [DPY_BUF_AREA]
wait_dpy_buf_area_parm --> wait_dpy_buf_area_parm : ev_begin_write / readb(14)
wait_dpy_buf_area_parm --> standby : ev_tx_done


c --> wait_set_vcom_parm : [SET_VCOM_PARM]
wait_set_vcom_parm --> standby : ev_tx_done


c --> wait_transfer : [GET_DEVICE_INFO]
wait_transfer  --> standby : ev_tx_done


c --> standby : [SYS_RUN || SLEEP || MEM_BST_RD_S || MEM_BST_END || LD_IMG_END]


standby --> c2 : ev_recvb
c2 --> standby : [else]/readb(2)
c2 --> c3 : [transferred == 2]
c3 --> wait_command : [preamble == COMMAND]
c3 --> standby : [else]


@enduml
 **/

static void it8951e_msm_wait_transfer(it8951e_state *s,
                                      enum it8951e_msm_event event)
{
    DPRINTF("MSM: Process event %s in MSM state %s\n",
            it8951e_msm_event_names[event], __func__);

    switch (event) {
    case IT8951E_MSM_EV_TX_DONE:
        s->state = it8951e_msm_standby;
        break;

    default:
        DPRINTF("MSM: Invalid Event %s for MSM state %s\n",
                it8951e_msm_event_names[event], __func__);
        break;
    }
}


static void it8951e_msm_wait_reg_rd_val(it8951e_state *s,
                                        enum it8951e_msm_event event)
{
    DPRINTF("MSM: Process event %s in MSM state %s\n",
            it8951e_msm_event_names[event], __func__);

    switch (event) {
    case IT8951E_MSM_EV_TX_DONE:
        s->state = it8951e_msm_standby;
        break;

    default:
        DPRINTF("MSM: Invalid Event %s for MSM state %s\n",
                it8951e_msm_event_names[event], __func__);
        break;
    }
}

static void it8951e_msm_wait_reg_rd_parm(it8951e_state *s,
                                         enum it8951e_msm_event event)
{
    DPRINTF("MSM: Process event %s in MSM state %s\n",
            it8951e_msm_event_names[event], __func__);

    switch (event) {
    case IT8951E_MSM_EV_BEGIN_WRITE:
        DPRINTF("MSM: requesting write 2b\n");
        it8951e_readb(s, 2);
        break;

    case IT8951E_MSM_EV_TX_DONE:
        s->ptr = (s->spi_buffer[0] << 8) | s->spi_buffer[1];
        DPRINTF("MSM: WAIT_REG_RD_PARM: got 0x%04x\n", s->ptr);
        s->output_bytes = 2;
        s->transferred_bytes = 0;
        s->spi_out_buffer[0] = 0;
        s->spi_out_buffer[1] = 0;
        s->state = it8951e_msm_wait_reg_rd_val;
        break;
    default:
        DPRINTF("MSM: Invalid Event %s for MSM state %s\n",
                it8951e_msm_event_names[event], __func__);
        break;
    }
}

static void it8951e_msm_wait_reg_wr_val(it8951e_state *s,
                                        enum it8951e_msm_event event)
{
    uint16_t data;

    DPRINTF("MSM: Process event %s in MSM state %s\n",
            it8951e_msm_event_names[event], __func__);

    switch (event) {
    case IT8951E_MSM_EV_BEGIN_WRITE:
        DPRINTF("MSM IT8951E_MSM_WAIT_REG_WR_VAL req write 2b\n");
        it8951e_readb(s, 2);
        break;

    case IT8951E_MSM_EV_TX_DONE:
        data = (s->spi_buffer[0] << 8) | s->spi_buffer[1];
        DPRINTF("MSM: REG_WR_VAL: set 0x%04x = 0x%04x\n", s->ptr, data);
        switch (s->ptr) {
        case IT8951E_REG_I80CPCR:
            s->packed_mode = (bool)data;
            break;

        case IT8951E_REG_LISAR_HW:
            s->reg_lisar = (s->reg_lisar & 0x0000FFFFu) | (data << 16);
            DPRINTF("MSM: REG_WR_VAL: LISAR [HW] = 0x%08x\n",
                    s->reg_lisar);
            break;

        case IT8951E_REG_LISAR_LW:
            s->reg_lisar = (s->reg_lisar & 0xFFFF0000u) | data;
            DPRINTF("MSM: REG_WR_VAL: LISAR [LW] = 0x%08x\n",
                    s->reg_lisar);
            break;

        default:
            DPRINTF("MSM: REG_WR_VAL: Unimplemented register\n");
            break;
        }
        s->state = it8951e_msm_standby;
        break;

    default:
        DPRINTF("MSM: Invalid Event %s for MSM state %s\n",
                it8951e_msm_event_names[event], __func__);
        break;
    }
}


static void it8951e_msm_wait_reg_wr_parm(it8951e_state *s,
                                         enum it8951e_msm_event event)
{
    DPRINTF("MSM: Process event %s in MSM state %s\n",
            it8951e_msm_event_names[event], __func__);

    switch (event) {
    case IT8951E_MSM_EV_BEGIN_WRITE:
        DPRINTF("MSM: requesting write 2b\n");
        it8951e_readb(s, 2);
        break;

    case IT8951E_MSM_EV_TX_DONE:
        s->ptr = (s->spi_buffer[0] << 8) | s->spi_buffer[1];
        DPRINTF("MSM: WAIT_REG_WR_PARM: got 0x%04x\n", s->ptr);
        s->state = it8951e_msm_wait_reg_wr_val;
        it8951e_readb(s, 2);
        break;
    default:
        DPRINTF("MSM: Invalid Event %s for MSM state %s\n",
                it8951e_msm_event_names[event], __func__);
        break;
    }
}


static void it8951e_msm_wait_ld_area_data(it8951e_state *s,
                                          enum it8951e_msm_event event)
{
    uint16_t data;

    DPRINTF("MSM: Process event %s in MSM state %s (%d recv out of %d)\n",
            it8951e_msm_event_names[event],
            __func__,
            s->transferred_bytes, s->needed_bytes);

    switch (event) {
    case IT8951E_MSM_EV_BEGIN_WRITE:
        it8951e_readb(s, IT8951E_LD_BUF_SIZE);
        break;

    case IT8951E_MSM_EV_TX_DONE:
        DPRINTF("WAIT_LD_AREA_DATA: TX_DONE with %d bytes "
                "[(%d, %d) -> (%d, %d)] width: %d, height: %d\n",
                s->transferred_bytes, s->crsr.x, s->crsr.y, s->crsr.posx,
                s->crsr.posy, s->crsr.w, s->crsr.h);

        for (uint32_t pos = 0; pos < s->transferred_bytes; pos++) {
            it8951e_blit(s, s->spi_buffer[pos] >> 4);
            it8951e_blit(s, s->spi_buffer[pos] & 0x0F);
        }
        break;

    case IT8951E_MSM_EV_COMMAND:
        data = (s->spi_buffer[0] << 8) | (s->spi_buffer[1]);
        DPRINTF("MSM: LD_AREA_DATA --> EV_COMMAND(0x%04x)\n", data);
        switch (data) {
        case IT8951E_CMD_LD_IMG_END:
            DPRINTF("MSM: >> LD_IMG_END\n");
            s->state = it8951e_msm_standby;
            s->redraw = 1;
            break;

        default:
            DPRINTF("Unhandled command 0x%04x in state %s, event %s",
                    data, __func__, it8951e_msm_event_names[event]);
            break;
        }

        break;

    case IT8951E_MSM_EV_CS_OFF:
        /*
         * Flush the last data in buffer. This event will not change the
         * state of the SM. The EV_COMMAND[LD_IMG_END] stops the fb update
         */
        it8951e_main_sm(s, IT8951E_MSM_EV_TX_DONE);
        break;


    default:
        DPRINTF("MSM: Invalid Event %s for MSM state %s\n",
                it8951e_msm_event_names[event], __func__);
        DPRINTF("     LD_AREA_PARM --> ??? w/ 0x%04x\n",
                (s->spi_buffer[0] << 8) | (s->spi_buffer[1]));
        break;
    }
}



static void it8951e_msm_wait_ld_area_parm(it8951e_state *s,
                                          enum it8951e_msm_event event)
{
    uint16_t data;

    DPRINTF("MSM: Process event %s in MSM state %s\n",
            it8951e_msm_event_names[event], __func__);

    switch (event) {

    case IT8951E_MSM_EV_TX_DONE:
        data = (s->spi_buffer[0] << 8) | s->spi_buffer[1];
        s->crsr.x = (s->spi_buffer[2] << 8) | s->spi_buffer[3];
        s->crsr.y = (s->spi_buffer[4] << 8) | s->spi_buffer[5];
        s->crsr.w = (s->spi_buffer[6] << 8) | s->spi_buffer[7];
        s->crsr.h = (s->spi_buffer[8] << 8) | s->spi_buffer[9];
        s->crsr.bpp = ((data & 0x0030) >> 4);
        DPRINTL(1, "MSM: WAIT_LD_AREA_PARM: got 0x%04x, x = %5d, y=%5d, w=%5d,"
                " h=%5d\n",
                data,
                s->crsr.x,
                s->crsr.y,
                s->crsr.w,
                s->crsr.h
            );

        s->crsr.dummyw = s->crsr.w;
        s->crsr.dummyx = s->crsr.x;

        if (s->crsr.x % 4) {
            s->crsr.dummyx -= s->crsr.x % 4;
        }

        if ((s->crsr.x + s->crsr.w) % 4) {
            s->crsr.dummyw += 4 - ((s->crsr.x + s->crsr.w) % 4) +
                              s->crsr.dummyx;
        }

        DPRINTL(1, "MSM: WAIT_LD_AREA_PARM: corrected:  x = %5d (%d),"
                "          w=%5d --> %5d (%d)\n",
                s->crsr.dummyx, (s->crsr.dummyx % 4),
                s->crsr.dummyw, (s->crsr.dummyx + s->crsr.dummyw),
                ((s->crsr.dummyx + s->crsr.dummyw) % 4)
            );

        DPRINTL(1, "MSM: WAIT_LD_AREA_PARM: Endianness: %s, bpp: %d, "
                "rotation: %d\n", (data & 0x0100) ? "BIG" : "Little",
                2 + s->crsr.bpp, 90 * (data & 0x0003));
        s->rotation = data & 0x0003;
        s->crsr.posx = s->crsr.dummyx;
        s->crsr.posy = s->crsr.y;
        switch (s->rotation) {
        case 0:
        case 2:
            s->crsr.maxx = IT8951E_WIDTH;
            s->crsr.maxy = IT8951E_HEIGHT;
            break;

        case 1:
            s->crsr.maxx = IT8951E_HEIGHT;
            s->crsr.maxy = IT8951E_WIDTH;
            break;

        default:
            BADF("unknown orientation\n");
                break;
        }

        s->state = it8951e_msm_wait_ld_area_data;
        it8951e_readb(s, (s->crsr.dummyw * s->crsr.h) >> 1);

        break;

    default:
        DPRINTF("MSM: Invalid Event %s for MSM state %s\n",
                it8951e_msm_event_names[event], __func__);
        break;
    }
}


static void it8951e_msm_wait_mem_bst_rd_t_parm(it8951e_state *s,
                                               enum it8951e_msm_event event)
{
    DPRINTF("MSM: Process event %s in MSM state %s\n",
            it8951e_msm_event_names[event], __func__);

    switch (event) {
    case IT8951E_MSM_EV_BEGIN_WRITE:
        DPRINTF("MSM: requesting write 8b\n");
        it8951e_readb(s, 8);
        break;

    case IT8951E_MSM_EV_TX_DONE:
        DPRINTF("MSM: WAIT_MEM_BST_RD_T_PARM: got 0x%04x 0x%04x //"
                " 0x%04x 0x%04x\n",
                (s->spi_buffer[0] << 8) | s->spi_buffer[1],
                (s->spi_buffer[2] << 8) | s->spi_buffer[3],
                (s->spi_buffer[4] << 8) | s->spi_buffer[5],
                (s->spi_buffer[6] << 8) | s->spi_buffer[7]
            );
        s->state = it8951e_msm_standby;
        break;
    default:
        DPRINTF("MSM: Invalid Event %s for MSM state %s\n",
                it8951e_msm_event_names[event], __func__);
        break;
    }
}


static void it8951e_msm_wait_mem_bst_wr_parm(it8951e_state *s,
                                             enum it8951e_msm_event event)
{
    DPRINTF("MSM: Process event %s in MSM state %s\n",
            it8951e_msm_event_names[event], __func__);

    switch (event) {
    case IT8951E_MSM_EV_BEGIN_WRITE:
        DPRINTF("MSM: requesting write 8b\n");
        it8951e_readb(s, 8);
        break;

    case IT8951E_MSM_EV_TX_DONE:
        DPRINTF("MSM: WAIT_MEM_BST_WR_PARM: got 0x%04x 0x%04x //"
                " 0x%04x 0x%04x\n",
                (s->spi_buffer[0] << 8) | s->spi_buffer[1],
                (s->spi_buffer[2] << 8) | s->spi_buffer[3],
                (s->spi_buffer[4] << 8) | s->spi_buffer[5],
                (s->spi_buffer[6] << 8) | s->spi_buffer[7]
            );
        s->state = it8951e_msm_standby;
        break;
    default:
        DPRINTF("MSM: Invalid Event %s for MSM state %s\n",
                it8951e_msm_event_names[event], __func__);
        break;
    }
}


static void it8951e_msm_wait_ld_img_parm(it8951e_state *s,
                                         enum it8951e_msm_event event)
{
    DPRINTF("MSM: Process event %s in MSM state %s\n",
            it8951e_msm_event_names[event], __func__);

    switch (event) {
    case IT8951E_MSM_EV_BEGIN_WRITE:
        DPRINTF("MSM: requesting write 2b\n");
        it8951e_readb(s, 2);
        break;

    case IT8951E_MSM_EV_TX_DONE:
        DPRINTF("MSM: WAIT_LD_IMG_PARM: got 0x%04x\n",
                (s->spi_buffer[0] << 8) | s->spi_buffer[1]
            );
        s->state = it8951e_msm_standby;
        break;
    default:
        DPRINTF("MSM: Invalid Event %s for MSM state %s\n",
                it8951e_msm_event_names[event], __func__);
        break;
    }
}


static void it8951e_msm_wait_dpy_buf_area_parm(it8951e_state *s,
                                               enum it8951e_msm_event event)
{
    DPRINTF("MSM: Process event %s in MSM state %s\n",
            it8951e_msm_event_names[event], __func__);

    switch (event) {
    case IT8951E_MSM_EV_BEGIN_WRITE:
        it8951e_readb(s, 14);
        break;

    case IT8951E_MSM_EV_TX_DONE:
        DPRINTF("MSM:    0x0037: {x: %d, y: %d, w: %d, h: %d, mode: %d,"
            " ptr: 0x%08x}\n",
            (s->spi_buffer[0] << 8) | (s->spi_buffer[1]),
            (s->spi_buffer[2] << 8) | (s->spi_buffer[3]),
            (s->spi_buffer[4] << 8) | (s->spi_buffer[5]),
            (s->spi_buffer[6] << 8) | (s->spi_buffer[7]),
            (s->spi_buffer[8] << 8) | (s->spi_buffer[9]),
            (s->spi_buffer[10] << 8) | (s->spi_buffer[11]) |
                (s->spi_buffer[12] << 24) | (s->spi_buffer[13] << 16)
        );

        s->state = it8951e_msm_standby;
        it8951e_invalidate_display(s);
        break;

    default:
        DPRINTF("MSM: Invalid Event %s for MSM state %s\n",
                it8951e_msm_event_names[event], __func__);
        break;
    }
}


static void it8951e_msm_wait_set_vcom_parm(it8951e_state *s,
                                           enum it8951e_msm_event event)
{
    DPRINTF("MSM: Process event %s in MSM state %s\n",
            it8951e_msm_event_names[event], __func__);

    switch (event) {
    case IT8951E_MSM_EV_TX_DONE:
        DPRINTF("MSM: WAIT_SET_VCOM_PARM: set (0x%04x, %d)\n",
                (s->spi_buffer[0] << 8) | s->spi_buffer[1],
                (s->spi_buffer[2] << 8) | s->spi_buffer[3]
            );
        s->state = it8951e_msm_standby;
        break;
    default:
        DPRINTF("MSM: Invalid Event %s for MSM state %s\n",
                it8951e_msm_event_names[event], __func__);
        break;

    }
}



static void it8951e_msm_cmd_0302(it8951e_state *s, enum it8951e_msm_event event)
{
    DPRINTF("MSM: Process event %s in MSM state %s\n",
            it8951e_msm_event_names[event], __func__);

    switch (event) {
    case IT8951E_MSM_EV_RECVB:
        if (s->transferred_bytes != 2) {
            DPRINTF("MSM: IT8951E_MSM_CMD_0302 @ IT8951E_MSM_EV_RECV : %d\n",
                    s->transferred_bytes);
        }

        DPRINTF("MSM: Received preamble %04x\n",
                (s->spi_buffer[0] << 8) | (s->spi_buffer[1]));

        s->state = it8951e_msm_cmd_0302;
        it8951e_readb(s, 2);
        break;

    default:
        DPRINTF("MSM: Invalid Event %s for MSM state %s\n",
                it8951e_msm_event_names[event], __func__);
        break;
    }
}

static void it8951e_msm_cmd_0302_pre(it8951e_state *s,
                                     enum it8951e_msm_event event)
{
    uint16_t data;

    DPRINTF("MSM: Process event %s in MSM state %s\n",
            it8951e_msm_event_names[event], __func__);

    switch (event) {
    case IT8951E_MSM_EV_RECVB:
        if (s->transferred_bytes != 2) {
            DPRINTF("MSM: %s @ %s : %d\n", __func__,
                    it8951e_msm_event_names[event], s->transferred_bytes);
        }
        data = (s->spi_buffer[0] << 8) | (s->spi_buffer[1]);
        DPRINTF("MSM: Received preamble %04x\n", data);

        if (data != 0x1000u) {
            DPRINTF("MSM: Incorrect preamble!");
            s->state = it8951e_msm_standby;
            break;
        }
        s->state = it8951e_msm_cmd_0302;
        it8951e_readb(s, 1);
        break;

    default:
        DPRINTF("MSM: Invalid Event %s for MSM state %s\n",
                it8951e_msm_event_names[event], __func__);
        break;
    }
}

static void it8951e_msm_wait_command(it8951e_state *s,
                                     enum it8951e_msm_event event)
{
    uint16_t data;

    DPRINTF("MSM: Process event %s in MSM state %s\n",
            it8951e_msm_event_names[event], __func__);

    switch (event) {
    case IT8951E_MSM_EV_RECVB:
        if (s->transferred_bytes != 2) {
            DPRINTF("MSM: %s @ %s : %d\n", __func__,
                    it8951e_msm_event_names[event], s->transferred_bytes);
        }
        data = (s->spi_buffer[0] << 8) | (s->spi_buffer[1]);
        DPRINTF("MSM: Received command 0x%04x\n", data);
        switch (data) {
        case 0x0302u:
            s->state = it8951e_msm_cmd_0302_pre;
            break;

        default:
            DPRINTF("MSM: Unknown command!\n");
            break;

        }
        it8951e_readb(s, 2);
        break;

    default:
        DPRINTF("MSM: Invalid Event %s for MSM state %s\n",
                it8951e_msm_event_names[event], __func__);
        break;
    }
}



static void it8951e_msm_standby(it8951e_state *s,
                                enum it8951e_msm_event event)
{
    uint16_t data;

    DPRINTF("MSM: Process event %s in MSM state %s\n",
            it8951e_msm_event_names[event], __func__);

    switch (event) {
    case IT8951E_MSM_EV_COMMAND:
        data = (s->spi_buffer[0] << 8) | (s->spi_buffer[1]);
        DPRINTF("MSM: STANDBY --> EV_COMMAND(0x%04x)\n", data);
        switch (data) {
        case IT8951E_CMD_SYS_RUN:
            DPRINTF("MSM: >> SYS_RUN (NOP)\n");
            s->state = it8951e_msm_standby;
            break;

        case IT8951E_CMD_SLEEP:
            DPRINTF("MSM: >> SLEEP (NOP)\n");
            s->state = it8951e_msm_standby;
            break;

        case IT8951E_CMD_REG_RD:
            /* 6000 00 10 // 0000 12 24 // 1000 xx xx RR RR */
            DPRINTF("MSM: >> REG_RD\n");
            s->state = it8951e_msm_wait_reg_rd_parm;
            it8951e_readb(s, 2);
            break;

        case IT8951E_CMD_REG_WR:
            /*
             * 6000 00 11 // 0000 00 04 // 0000 00 01
             * 6000 00 11 // 0000 02 0a // 0000 12 34
             * 6000 00 11 // 0000 02 08 // 0000 56 78
             * 6000 00 21 // 0000 01 21 00 00 00 00 02 1c 03 c0 // 0000 ff ff...
             */
            DPRINTF("MSM: >> REG_WR\n");
            s->state = it8951e_msm_wait_reg_wr_parm;
            it8951e_readb(s, 2);
            break;

        case IT8951E_CMD_MEM_BST_RD_T:
            DPRINTF("MSM: >> MEM_BST_RD_T>\n");
            s->state = it8951e_msm_wait_mem_bst_rd_t_parm;
            break;

        case IT8951E_CMD_MEM_BST_RD_S:
            DPRINTF("MSM: >> MEM_BST_RD_S\n");
            s->state = it8951e_msm_standby;
            break;

        case IT8951E_CMD_MEM_BST_WR:
            DPRINTF("MSM: >> MEM_BST_WR\n");
            s->state = it8951e_msm_wait_mem_bst_wr_parm;
            break;

        case IT8951E_CMD_MEM_BST_END:
            DPRINTF("MSM: >> MEM_BST_END\n");
            s->state = it8951e_msm_standby;
            break;

        case IT8951E_CMD_LD_IMG:
            DPRINTF("MSM: >> LD_IMG\n");
            s->state = it8951e_msm_wait_ld_img_parm;
            break;

        case IT8951E_CMD_LD_AREA:
            DPRINTF("MSM: >> LD_AREA\n");
            s->state = it8951e_msm_wait_ld_area_parm;
            it8951e_readb(s, 10);
            break;

        case IT8951E_CMD_LD_IMG_END:
            DPRINTF("MSM: >> LD_IMG_END\n");
            s->state = it8951e_msm_standby;
            break;

        case IT8951E_CMD_DPY_BUF_AREA:
            DPRINTF("MSM: >> DPY_BUF_AREA\n");
            s->state = it8951e_msm_wait_dpy_buf_area_parm;
            it8951e_readb(s, 14);
            break;

        case IT8951E_CMD_SET_VCOM:
            DPRINTF("MSM: >> SET_VCOM\n");
            s->state = it8951e_msm_wait_set_vcom_parm;
            it8951e_readb(s, 4);
            break;

        case IT8951E_CMD_GET_DEVICE_INFO:
            DPRINTF("MSM: >> GET_DEVICE_INFO\n");
            s->output_bytes = sizeof(it8951e_device_info);
            memcpy(s->spi_out_buffer, &it8951e_device_info,
                    sizeof(it8951e_device_info));
            DPRINTF("MSM: >> set output bytes to %d and copied data\n",
                    s->output_bytes);
            s->state = it8951e_msm_wait_transfer;
            break;

        default:
            break;
        }
        break;

    case IT8951E_MSM_EV_RECVB:
        if (s->transferred_bytes != 2) {
            DPRINTF("MSM: IT8951E_MSM_STANDBY @ IT8951E_MSM_EV_RECV : %d\n",
                    s->transferred_bytes);
            assert(false);
            it8951e_readb(s, 2);
            break;
        }

        data = (s->spi_buffer[0] << 8) | (s->spi_buffer[1]);
        DPRINTF("it8951e_main_sm --> 0x%04x\n", data);
        switch (data) {
        case IT8951E_PREAMBLE_COMMAND:
            s->state = it8951e_msm_wait_command;
            it8951e_readb(s, 2);
            break;

        default:
            break;
        }
        break;

    default:
        DPRINTF("MSM: Invalid Event %s for MSM state %s\n",
                it8951e_msm_event_names[event], __func__);
        break;
    }
}


/**
 * Transfer state machine

@startuml

state c1 <<choice>>

[*] --> standby

standby --> standby : ev_cs_update[active]
standby --> waitlb : ev_read

standby --> waitlb : ev_transfer
waitlb --> standby : ev_cs_update[active]

waitlb --> c1 : ev_transfer
c1 --> wait_commandhb : [PREAMBLE_COMMAND]
wait_commandhb --> wait_commandlb : ev_transfer
wait_commandlb --> standby : ev_transfer
wait_commandhb --> standby : ev_cs_update[active]

c1 --> send_data_dummy : [PREAMBLE_READ_DATA]
send_data_dummy --> standby : ev_cs_update[active]
send_data_dummy --> send_data : transfer[transferred == 2]

send_data --> send_data : transfer
send_data --> standby : ev_cs_update[active]

c1 --> wait_write_end : [PREAMBLE_WRITE_DATA]
wait_write_end --> wait_write_end : ev_transfer
wait_write_end --> standby : ev_cs_update[active]

@enduml
 */

static uint32_t it8951e_transfer(SSIPeripheral *dev, uint32_t data)
{
    it8951e_state *s = (it8951e_state *)dev;
    uint8_t byte = (uint8_t)data;
    uint32_t result = 0;

    if (!s->active) {
        DPRINTF("transfer %02x ### NO CS ###\n", byte);
        return result;
    }

    if ((s->tsm != IT8951E_TSM_STANDBY) && (s->tsm != IT8951E_TSM_WAITLB)) {
        s->spi_buffer[s->transferred_bytes] = byte;
        s->transferred_bytes++;
    } else {
        s->preamble = s->preamble << 8 | byte;
    }

    switch (s->tsm) {
    case IT8951E_TSM_STANDBY:
        s->tsm = IT8951E_TSM_WAITLB;
        break;

    case IT8951E_TSM_WAITLB:
        switch (s->preamble) {
        case IT8951E_PREAMBLE_COMMAND:
            s->tsm = IT8951E_TSM_WAIT_COMMANDHB;
            s->transferred_bytes = 0;
            break;

        case IT8951E_PREAMBLE_READ_DATA:
            s->tsm = IT8951E_TSM_SEND_DATA_DUMMY;
            break;

        case IT8951E_PREAMBLE_WRITE_DATA:
            s->tsm = IT8951E_TSM_WAIT_WRITE_END;
            break;

        default:
            break;
        }
        break;

    case IT8951E_TSM_WAIT_COMMANDHB:
        s->tsm = IT8951E_TSM_WAIT_COMMANDLB;
        break;

    case IT8951E_TSM_WAIT_COMMANDLB:
        if (s->transferred_bytes == 2) {
            it8951e_main_sm(s, IT8951E_MSM_EV_COMMAND);
            s->tsm = IT8951E_TSM_STANDBY;
            s->transferred_bytes = 0;
        }
        break;

    case IT8951E_TSM_WAIT_WRITE_END:
        /* TODO: replace with circular buffer, maybe of fixed size - 512b */
        if (s->transferred_bytes >= s->needed_bytes) {
            if (s->transferred_bytes == s->needed_bytes) {
                it8951e_main_sm(s, IT8951E_MSM_EV_TX_DONE);
            } else {
                s->transferred_bytes = s->needed_bytes;
            }
        }
        break;


    case IT8951E_TSM_SEND_DATA_DUMMY:
        result = 0;
        if (s->transferred_bytes == 2) {
            s->transferred_bytes = 0;
            s->tsm = IT8951E_TSM_SEND_DATA;
        }
        break;

    case IT8951E_TSM_SEND_DATA:
        if (s->output_bytes >= s->transferred_bytes) {
            result = s->spi_out_buffer[s->transferred_bytes - 1];
            if (s->output_bytes == s->transferred_bytes) {
                it8951e_main_sm(s, IT8951E_MSM_EV_TX_DONE);
                s->output_bytes = 0;
            }
        } else {
            s->transferred_bytes = s->output_bytes;
        }
        break;

    default:
        DPRINTF("Invalid state for transfer event: %d\n", s->tsm);
        break;
    }

    return result;

}

/* TODO: add parameters, s->crsr knows what to update */
static void it8951e_update_display(void *opaque)
{
    it8951e_state *s = (it8951e_state *)opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);
    uint8_t *base, *dest;
    uint8_t *src;
    int x;
    int y;
    int i;
    char *colors[16];
    char colortab[64];
    char *p;
    int dest_width;

    if (!s->redraw) {
        return;
    }

    switch (surface_bits_per_pixel(surface)) {
    case 0:
        return;
    case 15:
        dest_width = 2;
        break;
    case 16:
        dest_width = 2;
        break;
    case 24:
        dest_width = 3;
        break;
    case 32:
        dest_width = 4;
        break;
    default:
        BADF("Bad color depth\n");
        return;
    }

    p = colortab;
    for (i = 0; i < 16; i++) {
        int n;
        colors[i] = p;
        switch (surface_bits_per_pixel(surface)) {
        case 15:
            n = i * 2 + (i >> 3);
            p[0] = n | (n << 5);
            p[1] = (n << 2) | (n >> 3);
            break;
        case 16:
            n = i * 2 + (i >> 3);
            p[0] = n | (n << 6) | ((n << 1) & 0x20);
            p[1] = (n << 3) | (n >> 2);
            break;
        case 24:
        case 32:
            n = (i << 4) | i;
            p[0] = p[1] = p[2] = n;
            break;
        default:
            BADF("Bad color depth\n");
            return;
        }
        p += dest_width;
    }




    int width, height, xx, yy, rotation;

    typedef struct coeff {
        int tx;
        int ty;
        int costh;
        int sinth;
    } coeff;

    const coeff rotparams[4] = {
        {.tx = 0, .ty = 0, .costh =  1, .sinth =  0},
        {.tx = 1, .ty = 0, .costh =  0, .sinth =  1},
        {.tx = 1, .ty = 1, .costh = -1, .sinth =  0},
        {.tx = 0, .ty = 1, .costh =  0, .sinth = -1},
    };



    switch (s->viewport_rotation) {
    case 0:
        width = IT8951E_WIDTH;
        height = IT8951E_HEIGHT;
        rotation = 0;
        break;

    case 90:
        width = IT8951E_HEIGHT;
        height = IT8951E_WIDTH;
        rotation = 1;
        break;

    case 180:
        width = IT8951E_WIDTH;
        height = IT8951E_HEIGHT;
        rotation = 2;
        break;

    case 270:
        width = IT8951E_HEIGHT;
        height = IT8951E_WIDTH;
        rotation = 3;
        break;

    default:
        BADF("Unkown viewport rotation: %d. Must be multiple of 90\n",
             s->viewport_rotation);
        break;
    }



    base = surface_data(surface);
    int val;
    for (y = 0; y < IT8951E_HEIGHT; y++) {
        src = s->framebuffer + y * IT8951E_WIDTH;
        for (x = 0; x < IT8951E_WIDTH; x++) {
            val = *src;
            xx = rotparams[rotation].tx * (width - 1)  +
                rotparams[rotation].costh * x - rotparams[rotation].sinth * y;
            yy = rotparams[rotation].ty * (height - 1) +
                rotparams[rotation].sinth * x + rotparams[rotation].costh * y;
            dest = base + (yy * width + xx) * dest_width;
            memcpy(dest, colors[val], dest_width);
            src++;
        }
    }


    s->redraw = 0;
    dpy_gfx_update(s->con, 0, 0, width, height);
}

static void it8951e_invalidate_display(void *opaque)
{
    it8951e_state *s = (it8951e_state *)opaque;
    s->redraw = 1;
}


static void it8951e_reset(it8951e_state *s)
{
    s->spi_buffer_pos = 0;
    s->packed_mode = false;
    s->rotation = 0;

    s->tsm = IT8951E_TSM_STANDBY;
    s->state = it8951e_msm_standby;

    s->needed_bytes = 0;
    s->transferred_bytes = 0;

    s->crsr.posx = 0;
    s->crsr.posy = 0;
    s->crsr.x = 0;
    s->crsr.y = 0;
    s->crsr.w = IT8951E_WIDTH;
    s->crsr.h = IT8951E_HEIGHT;
    s->crsr.dummyx = 0;
    s->crsr.dummyw = IT8951E_WIDTH;

    s->reg_lisar = IT8951E_FB_ADDRESS;

    memset(s->framebuffer, 0x08, sizeof(s->framebuffer));

    it8951e_invalidate_display(s);
}

/*
 * Order: Main --> EPD
 * Main initializes the entire M5Paper
 * EPD turns the display on
 * GPIO 2 - Main Power
 */
static void it8951e_gpio_main_power(void *opaque, int n, int level)
{
    it8951e_state *s = (it8951e_state *)opaque;
    (void) n;
    DPRINTF("main_power: %s\n", level ? "ON" : "OFF");
    s->main_power = level ? IT8951E_POWER_ON : IT8951E_POWER_OFF;
}

/*
 * GPIO 23 - EPD Power. Can be used as reset line. Main Power has to be on, so
 * that the display is enabled
 */
static void it8951e_gpio_epd_power(void *opaque, int n, int level)
{
    (void) n;

    it8951e_state *s = (it8951e_state *)opaque;
    DPRINTF("epd_power: %s\n", level ? "ON" : "OFF");

    if ((s->epd_power == IT8951E_POWER_OFF) && (level)) {
        DPRINTF("epd_power: doing the hrdy thing\n");
        it8951e_reset(s);
    }
    s->epd_power = level ? IT8951E_POWER_ON : IT8951E_POWER_OFF;

    bool result = (s->epd_power == IT8951E_POWER_ON) &&
                  (s->main_power == IT8951E_POWER_ON);
    DPRINTF("HRDY: level=%d, power=%d, result=%d\n", level, s->main_power,
       result);

    qemu_set_irq(s->gpio_hrdy, result);
}

static int it8951e_post_load(void *opaque, int version_id)
{
    /* TODO: post load */
    (void) opaque;
    (void) version_id;
    return 0;
}

/* TODO: Update fields */
static const VMStateDescription vmstate_it8951e = {
    .name = "it8951e_epd",
    .version_id = 2,
    .minimum_version_id = 2,
    .post_load = it8951e_post_load,
    .fields      = (VMStateField []) {
        VMSTATE_INT32(redraw, it8951e_state),
        VMSTATE_BUFFER(framebuffer, it8951e_state),
        VMSTATE_SSI_PERIPHERAL(ssidev, it8951e_state),
        VMSTATE_END_OF_LIST()
    }
};

static const GraphicHwOps it8951e_ops = {
    .invalidate  = it8951e_invalidate_display,
    .gfx_update  = it8951e_update_display,
};

static int it8951e_cs_update(SSIPeripheral *d, bool select)
{
    it8951e_state *s = (it8951e_state *)d;

    /* Display in standby, and CS is active low ==> begin transmission */
    if (!s->active && !select) {
        s->tsm = IT8951E_TSM_STANDBY;
    }

    s->active = !select;

    return 0;
}

static void it8951e_realize(SSIPeripheral *d, Error **errp)
{
    (void) errp;

    DeviceState *dev = DEVICE(d);
    it8951e_state *s = IT8951E(d);

    s->ssidev.spc->set_cs = it8951e_cs_update;

    s->main_power = IT8951E_POWER_OFF;
    s->epd_power = IT8951E_POWER_OFF;

    s->con = graphic_console_init(dev, 0, &it8951e_ops, s);

    s->active = false;

    int width, height;



    s->viewport_rotation %= 360;
    if (s->viewport_rotation < 0) {
        s->viewport_rotation = 360 - s->viewport_rotation;
    }


    switch (s->viewport_rotation) {
    case 0:
        width = IT8951E_WIDTH;
        height = IT8951E_HEIGHT;
        break;

    case 90:
        height = IT8951E_WIDTH;
        width = IT8951E_HEIGHT;
        break;

    case 180:
        width = IT8951E_WIDTH;
        height = IT8951E_HEIGHT;
        break;

    case 270:
        height = IT8951E_WIDTH;
        width = IT8951E_HEIGHT;
        break;

    default:
        BADF("Unkown viewport rotation: %d. Must be multiple of 90\n",
             s->viewport_rotation);
        break;
    }

    DPRINTF("Resizing console to %d x %d\n", width, height);
    qemu_console_resize(s->con, width, height);

    qdev_init_gpio_out_named(dev, &s->gpio_hrdy, IT8951E_GPIO_HRDY, 1);
    DPRINTF("HRDY: result=1\n");

    qdev_init_gpio_in_named(dev, it8951e_gpio_epd_power,
                            IT8951E_GPIO_EPD_POWER, 1);

    qdev_init_gpio_in_named(dev, it8951e_gpio_main_power,
                            IT8951E_GPIO_MAIN_POWER, 1);
}

static Property it8951e_properties[] = {
    DEFINE_PROP_INT32("rotation",   it8951e_state, viewport_rotation, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void it8951e_class_init(ObjectClass *objClass, void *data)
{
    (void) data;

    DeviceClass *dc = DEVICE_CLASS(objClass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(objClass);

    k->realize = it8951e_realize;
    k->transfer = it8951e_transfer;
    k->cs_polarity = SSI_CS_LOW;
    dc->vmsd = &vmstate_it8951e;
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);

    device_class_set_props(dc, it8951e_properties);

}

static const TypeInfo it8951e_info = {
    .name          = TYPE_IT8951E,
    .parent        = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(it8951e_state),
    .class_init    = it8951e_class_init,
};

static void it8951e_register_types(void)
{
    type_register_static(&it8951e_info);
}

type_init(it8951e_register_types)
