/*
 * BES serial flash controller (best1503, norflaship v2 / CTRL_VER >= 7)
 *
 * This code is licensed under the GPL version 2 or later.
 */

#ifndef HW_MISC_BES_FLASH_H
#define HW_MISC_BES_FLASH_H

#include "hw/core/sysbus.h"

#define TYPE_BES_FLASH "bes.flash"
OBJECT_DECLARE_SIMPLE_TYPE(BesFlashState, BES_FLASH)

#define BES_FLASH_TXFIFO_DEPTH  1024
/* one transaction's RX stream: RX length fields are 13 bits wide */
#define BES_FLASH_RX_MAX        8192
#define BES_NAND_MAX_PAGE       (4096 + 256)

struct BesFlashState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;         /* register window (0x400) */
    MemoryRegion storage_mr;    /* NOR: data backend shared with XIP */
    MemoryRegion window;        /* NAND: memory-mapped cache read window */
    uint8_t *storage;           /* NOR: == storage_mr ram ptr */
    uint32_t size;              /* NOR: logical size (dual-chip view) */
    char *image;                /* NOR: optional backing image (RW) */
    qemu_irq dma_req;           /* DMA request line (level) */

    uint32_t regs[0x400 / 4];

    uint8_t txfifo[BES_FLASH_TXFIFO_DEPTH];
    uint32_t tx_cnt;
    uint8_t rxbuf[BES_FLASH_RX_MAX];
    uint32_t rx_cnt;
    uint32_t rx_pos;

    /* ---- NOR die model ---- */
    bool pp_active;             /* page-program transaction collecting data */
    uint32_t pp_addr;
    uint32_t pp_total;
    uint8_t sr;                 /* status register S0-S7 (WIP/WEL/BP) */
    uint8_t sr2;                /* status register S8-S15 (QE/CMP) */

    /* ---- SPI NAND chip model (enabled by nand-image) ---- */
    char *nand_image;           /* backing file, (page + oob) per page */
    FILE *nand_fp;
    uint32_t jedec_id;          /* NOR die JEDEC ID, MSB first on the wire */
    uint32_t nand_id;           /* 3 ID bytes, MSB first on the wire */
    uint32_t nand_page;         /* data bytes per page */
    uint32_t nand_oob;          /* spare bytes per page */
    uint32_t nand_ppb;          /* pages per block */
    uint32_t nand_blocks;
    uint8_t nand_feat[256];     /* feature registers by address */
    uint8_t nand_cache[BES_NAND_MAX_PAGE];
    bool nand_load_active;      /* PROGRAM LOAD streaming TX into cache */
    uint32_t nand_load_col;
    uint32_t nand_load_left;
    bool nand_dpd;              /* deep power-down */

    Notifier shutdown_notifier; /* NOR image writeback on exit */
};

/* storage backend for XIP mapping in the machine (NOR) */
MemoryRegion *bes_flash_storage(BesFlashState *s);
uint8_t *bes_flash_data(BesFlashState *s);

#endif
