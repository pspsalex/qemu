/*
 * ESP32 SPI controller
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "sysemu/sysemu.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/ssi.h"
#include "hw/ssi/esp32_spi.h"
#include "hw/misc/esp32_flash_enc.h"
#include "sysemu/dma.h"


enum {
    CMD_RES = 0xab,
    CMD_DP = 0xb9,
    CMD_CE = 0x60,
    CMD_BE = 0xD8,
    CMD_SE = 0x20,
    CMD_PP = 0x02,
    CMD_WRSR = 0x1,
    CMD_RDSR = 0x5,
    CMD_RDID = 0x9f,
    CMD_WRDI = 0x4,
    CMD_WREN = 0x6,
    CMD_READ = 0x03,
};


#define ESP32_SPI_REG_SIZE    0x1000

/* #define DEBUG_ESP32_SPI 1 */

#ifdef DEBUG_ESP32_SPI
#define DPRINTF(fmt, ...) \
do { printf("esp32_spi: " fmt , ## __VA_ARGS__); } while (0)
#define BADF(fmt, ...) \
do { \
    fprintf(stderr, "esp32_spi: error: " fmt , ## __VA_ARGS__); abort(); \
} while (0)
#else
#define DPRINTF(fmt, ...) do {} while (0)
#define BADF(fmt, ...) \
do { fprintf(stderr, "esp32_spi: error: " fmt , ## __VA_ARGS__); } while (0)
#endif


static void esp32_spi_do_command(Esp32SpiState* state, uint32_t cmd_reg);

/*
 * Convert one of the hardware "bitlen" registers to a byte count
 * bitlen registers hold number of bits, minus one
 */
static inline int bitlen_to_bytes(uint32_t val)
{
    return (val + 1 + 7) / 8;
}

static uint64_t esp32_spi_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32SpiState *s = ESP32_SPI(opaque);
    uint64_t r = 0;
    switch (addr) {
    case A_SPI_ADDR:
        r = s->addr_reg;
        break;
    case A_SPI_CTRL:
        r = s->ctrl_reg;
        break;
    case A_SPI_STATUS:
        r = s->status_reg;
        break;
    case A_SPI_CTRL1:
        r = s->ctrl1_reg;
        break;
    case A_SPI_CTRL2:
        break;
    case A_SPI_USER:
        r = s->user_reg;
        break;
    case A_SPI_CLOCK:
        r = s->clk_reg;
        break;
    case A_SPI_USER1:
        r = s->user1_reg;
        break;
    case A_SPI_USER2:
        r = s->user2_reg;
        break;
    case A_SPI_MOSI_DLEN:
        r = s->mosi_dlen_reg;
        break;
    case A_SPI_MISO_DLEN:
        r = s->miso_dlen_reg;
        break;
    case A_SPI_PIN:
        r = s->pin_reg;
        break;
    case A_SPI_W0 ... A_SPI_W0 + (ESP32_SPI_BUF_WORDS - 1) * sizeof(uint32_t):
        r = s->data_reg[(addr - A_SPI_W0) / sizeof(uint32_t)];
        break;
    case A_SPI_EXT2:
        r = 0;
        break;
    case A_SPI_SLAVE:
        r = BIT(R_SPI_SLAVE_TRANS_DONE_SHIFT) | BIT(R_SPI_SLAVE_TRANS_INTEN_SHIFT);
        break;
    case A_SPI_CMD:
        break;
    case A_SPI_DMA_OUT_LINK:
        r = s->dma_outlink_reg ;
        break;
    case A_SPI_DMA_CONF:
        r = s->dma_conf_reg;
        break;
    default:
        break;

    }

   return r;
}

static void esp32_spi_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    Esp32SpiState *s = ESP32_SPI(opaque);
    switch (addr) {
    case A_SPI_W0 ... A_SPI_W0 + (ESP32_SPI_BUF_WORDS - 1) * sizeof(uint32_t):
        s->data_reg[(addr - A_SPI_W0) / sizeof(uint32_t)] = value;
        break;
    case A_SPI_ADDR:
        s->addr_reg = value;
        break;
    case A_SPI_CTRL:
        s->ctrl_reg = value;
        break;
    case A_SPI_STATUS:
        s->status_reg = value;
        break;
    case A_SPI_CTRL1:
        s->ctrl1_reg = value;
        break;
    case A_SPI_CTRL2:
        s->ctrl2_reg = value;
        break;
    case A_SPI_USER:
        s->user_reg = value;
        break;
    case A_SPI_CLOCK:
        s->clk_reg = value;
        break;
    case A_SPI_USER1:
        s->user1_reg = value;
        break;
    case A_SPI_USER2:
        s->user2_reg = value;
        break;
    case A_SPI_MOSI_DLEN:
        s->mosi_dlen_reg = value;
        break;
    case A_SPI_MISO_DLEN:
        s->miso_dlen_reg = value;
        break;
    case A_SPI_PIN:
        s->pin_reg = value;
        break;
    case A_SPI_CMD:
        esp32_spi_do_command(s, value);
        break;

    case A_SPI_DMA_OUT_LINK:
        s->dma_outlink_reg = value;
        break;

    case A_SPI_DMA_CONF:
        s->dma_conf_reg = value;
        break;

    }
}

typedef struct Esp32SpiTransaction {
    int cmd_bytes;
    uint32_t cmd;
    int addr_bytes;
    uint32_t addr;
    int data_tx_bytes;
    int data_rx_bytes;
    uint32_t* data;
} Esp32SpiTransaction;

static void esp32_spi_txrx_buffer(Esp32SpiState *s, void *buf, int tx_bytes, int rx_bytes)
{
    int bytes = MAX(tx_bytes, rx_bytes);
    uint8_t *c_buf = (uint8_t*) buf;
    for (int i = 0; i < bytes; ++i) {
        uint8_t byte = 0;
        if (i < tx_bytes) {
            memcpy(&byte, c_buf + i, 1);
        }
        uint32_t res = ssi_transfer(s->spi, byte);
        if (i < rx_bytes) {
            memcpy(c_buf + i, &res, 1);
        }
    }
}

static void esp32_spi_cs_set(Esp32SpiState *s, int value)
{
    for (int i = 0; i < ESP32_SPI_CS_COUNT; ++i) {
        qemu_set_irq(s->cs_gpio[i], ((s->pin_reg & (1 << i)) == 0) ? value : 1);
    }
}

static void esp32_spi_transaction(Esp32SpiState *s, Esp32SpiTransaction *t)
{
    esp32_spi_cs_set(s, 0);
    esp32_spi_txrx_buffer(s, &t->cmd, t->cmd_bytes, 0);
    esp32_spi_txrx_buffer(s, &t->addr, t->addr_bytes, 0);
    esp32_spi_txrx_buffer(s, t->data, t->data_tx_bytes, t->data_rx_bytes);
    esp32_spi_cs_set(s, 1);
}


static void maybe_encrypt_data(Esp32SpiState *s)
{
    Esp32FlashEncryptionState* flash_enc = esp32_flash_encryption_find();
    if (esp32_flash_encryption_enabled(flash_enc)) {
        esp32_flash_encryption_get_result(flash_enc, &s->data_reg[0], 8);
    }
}



/**
 * @brief Read and write arbitrary data from and to the guest machine
 *
 * @param s ESP32 SPI state structure
 * @param addr Guest machine address
 * @param data output ptr
 * @param len bytes to read
 *
 * @returns true if the transfer was a success, false else
 */
static bool esp32_spi_dma_read_guest(Esp32SpiState *s, uint32_t addr,
                                     void *data, uint32_t len)
{
    MemTxResult res = dma_memory_read(&s->dma_as, addr, data, len,
                                      MEMTXATTRS_UNSPECIFIED);
    return res == MEMTX_OK;
}

typedef struct EspDmaLinkedList {
    union {
        struct {
            /* Size of the buffer (mainly used in a receive transaction) */
            uint32_t size:12;
            /*
             * Number of valid bytes in the buffer. In a transmit, written by
             * software. In receive, written by hardware.
             */
            uint32_t length:12;
            /* Reserved */
            uint32_t rsvd_24:6;
            /*
             * Set if curent node is the last one (of the list). Set by software
             * in a transmit transaction, set by the hardware in case of a
             * receive transaction.
             */
            uint32_t suc_eof:1;
            /*
             * 0: CPU can access the buffer, 1: GDMA can access the buffer.
             * Cleared automatically by hardware in a transmit descriptor. In a
             * receive descriptor, cleared by hardware only if
             * GDMA_OUT_AUTO_WRBACK_CHn is set to 1.
             */
            uint32_t owner:1;
        };
        uint32_t val;
    } config;
    uint32_t buf_addr;
    uint32_t next_addr;
} EspDmaLinkedList;

static void esp32_spi_do_dma(Esp32SpiState *s, Esp32SpiTransaction *t)
{
    uint32_t descriptorAddress = FIELD_EX32(s->dma_outlink_reg,
                                            SPI_DMA_OUT_LINK, OUTLINK_ADDR);

    EspDmaLinkedList dmall;

    assert(esp32_spi_dma_read_guest(s, descriptorAddress | 0x3ff00000,
                                    &dmall, 12));

    assert(dmall.next_addr == 0);

    t->addr_bytes = 0;
    t->cmd_bytes = 0;
    t->data_tx_bytes = dmall.config.length;
    t->data = malloc(t->data_tx_bytes);

    assert(esp32_spi_dma_read_guest(s, dmall.buf_addr, t->data,
                                    dmall.config.length));
    esp32_spi_transaction(s, t);

    free(t->data);

    s->dma_outlink_reg = FIELD_DP32(s->dma_outlink_reg,
                                    SPI_DMA_OUT_LINK, OUTLINK_START, 0);

    s->dma_outlink_reg = FIELD_DP32(s->dma_outlink_reg,
                                    SPI_DMA_OUT_LINK, OUTLINK_RESTART, 0);
}


static void esp32_spi_do_command(Esp32SpiState* s, uint32_t cmd_reg)
{
    Esp32SpiTransaction t = {
        .cmd_bytes = 1
    };
    switch (cmd_reg) {
    case R_SPI_CMD_READ_MASK:
        t.cmd = CMD_READ;
        t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg,
                                                  SPI_USER1, ADDR_BITLEN));
        t.addr = bswap32(s->addr_reg) >> (32 - t.addr_bytes * 8);
        t.data = &s->data_reg[0];
        t.data_rx_bytes = bitlen_to_bytes(s->miso_dlen_reg);
        break;

    case R_SPI_CMD_WREN_MASK:
        t.cmd = CMD_WREN;
        break;

    case R_SPI_CMD_WRDI_MASK:
        t.cmd = CMD_WRDI;
        break;

    case R_SPI_CMD_RDID_MASK:
        t.cmd = CMD_RDID;
        t.data = &s->data_reg[0];
        t.data_rx_bytes = 3;
        break;

    case R_SPI_CMD_RDSR_MASK:
        t.cmd = CMD_RDSR;
        t.data = &s->status_reg;
        t.data_rx_bytes = 1;
        break;

    case R_SPI_CMD_WRSR_MASK:
        t.cmd = CMD_WRSR;
        t.data = &s->status_reg;
        t.data_tx_bytes = 1;
        break;

    case R_SPI_CMD_PP_MASK:
        maybe_encrypt_data(s);
        t.cmd = CMD_PP;
        t.data = &s->data_reg[0];
        t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg,
                                                  SPI_USER1, ADDR_BITLEN));
        t.addr = bswap32(s->addr_reg) >> 8;
        t.data = &s->data_reg[0];
        t.data_tx_bytes = s->addr_reg >> 24;
        break;

    case R_SPI_CMD_SE_MASK:
        t.cmd = CMD_SE;
        t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg,
                                                  SPI_USER1, ADDR_BITLEN));
        t.addr = bswap32(s->addr_reg) >> (32 - t.addr_bytes * 8);
        break;

    case R_SPI_CMD_BE_MASK:
        t.cmd = CMD_BE;
        t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg,
                                                  SPI_USER1, ADDR_BITLEN));
        t.addr = bswap32(s->addr_reg) >> (32 - t.addr_bytes * 8);
        break;

    case R_SPI_CMD_CE_MASK:
        t.cmd = CMD_CE;
        break;

    case R_SPI_CMD_DP_MASK:
        t.cmd = CMD_DP;
        break;

    case R_SPI_CMD_RES_MASK:
        t.cmd = CMD_RES;
        t.data = &s->data_reg[0];
        t.data_rx_bytes = 3;
        break;

    case R_SPI_CMD_USR_MASK:
        maybe_encrypt_data(s);
        if (FIELD_EX32(s->dma_outlink_reg, SPI_DMA_OUT_LINK, OUTLINK_START) ||
            FIELD_EX32(s->dma_outlink_reg, SPI_DMA_OUT_LINK, OUTLINK_START)) {
            esp32_spi_do_dma(s, &t);
            return;
        } else {
            if (FIELD_EX32(s->user_reg, SPI_USER, COMMAND) &&
                FIELD_EX32(s->user2_reg, SPI_USER2, COMMAND_BITLEN)) {

                t.cmd = FIELD_EX32(s->user2_reg, SPI_USER2, COMMAND_VALUE);
                t.cmd_bytes = bitlen_to_bytes(FIELD_EX32(s->user2_reg,
                                                         SPI_USER2,
                                                         COMMAND_BITLEN));

            } else {
                t.cmd_bytes = 0;
            }

            if (FIELD_EX32(s->user_reg, SPI_USER, ADDR)) {

                t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg,
                                                          SPI_USER1,
                                                          ADDR_BITLEN));
                t.addr = bswap32(s->addr_reg);
            }

            if (FIELD_EX32(s->user_reg, SPI_USER, MOSI)) {

                t.data = &s->data_reg[0 + FIELD_EX32(s->user_reg,
                                                     SPI_USER,
                                                     MOSI_HIGHPART) * 8];
                t.data_tx_bytes = bitlen_to_bytes(s->mosi_dlen_reg);
                assert(t.data_tx_bytes <= 16 * 4);
            }

            if (FIELD_EX32(s->user_reg, SPI_USER, MISO)) {

                t.data = &s->data_reg[0 + FIELD_EX32(s->user_reg,
                                                     SPI_USER,
                                                     MISO_HIGHPART) * 8];
                t.data_rx_bytes = bitlen_to_bytes(s->miso_dlen_reg);
            }
        }
        break;
    default:
        return;
    }
    esp32_spi_transaction(s, &t);
}


static const MemoryRegionOps esp32_spi_ops = {
    .read =  esp32_spi_read,
    .write = esp32_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_spi_reset(DeviceState *dev)
{
    Esp32SpiState *s = ESP32_SPI(dev);
    s->pin_reg = 0x6;
    s->user1_reg = FIELD_DP32(0, SPI_USER1, ADDR_BITLEN, 23);
    s->user1_reg = FIELD_DP32(s->user1_reg, SPI_USER1, DUMMY_CYCLELEN, 7);
    s->user2_reg = FIELD_DP32(0, SPI_USER2, COMMAND_BITLEN, 4);
    s->user2_reg = FIELD_DP32(s->user2_reg, SPI_USER2, COMMAND_VALUE, 0);
    s->dma_outlink_reg = 0;
    s->dma_conf_reg = 0;
    s->status_reg = 0;
    s->clk_reg = 0x80003043;
}

static void esp32_spi_realize(DeviceState *dev, Error **errp)
{
    Esp32SpiState *s = ESP32_SPI(dev);
    assert(s->soc_mr != NULL);

    address_space_init(&s->dma_as, s->soc_mr, "ssi.esp32.spi.dma");
}

static void esp32_spi_init(Object *obj)
{
    Esp32SpiState *s = ESP32_SPI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_spi_ops, s,
                          TYPE_ESP32_SPI, ESP32_SPI_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    s->spi = ssi_create_bus(DEVICE(s), "spi");
    qdev_init_gpio_out_named(DEVICE(s), &s->cs_gpio[0], SSI_GPIO_CS, ESP32_SPI_CS_COUNT);
}

static Property esp32_spi_properties[] = {
    DEFINE_PROP_LINK("soc_mr", Esp32SpiState, soc_mr, TYPE_MEMORY_REGION,
                     MemoryRegion*),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_spi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = esp32_spi_reset;
    dc->realize = esp32_spi_realize;
    device_class_set_props(dc, esp32_spi_properties);
}

static const TypeInfo esp32_spi_info = {
    .name = TYPE_ESP32_SPI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32SpiState),
    .instance_init = esp32_spi_init,
    .class_init = esp32_spi_class_init
};

static void esp32_spi_register_types(void)
{
    type_register_static(&esp32_spi_info);
}

type_init(esp32_spi_register_types)
