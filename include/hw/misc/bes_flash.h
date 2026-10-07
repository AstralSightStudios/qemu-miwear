/*
 * BES NOR flash controller (best1503, flash ctrl v7)
 *
 * This code is licensed under the GPL version 2 or later.
 */

#ifndef HW_MISC_BES_FLASH_H
#define HW_MISC_BES_FLASH_H

#include "hw/core/sysbus.h"

#define TYPE_BES_FLASH "bes.flash"
OBJECT_DECLARE_SIMPLE_TYPE(BesFlashState, BES_FLASH)

struct BesFlashState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;         /* register window (0x400) */
    MemoryRegion storage_mr;    /* flash data backend (shared with XIP) */
    uint8_t *storage;           /* == storage_mr ram ptr */
    uint32_t size;              /* logical size in bytes (dual-chip view) */
    char *image;                /* optional backing image file (RW) */
    char *rom_image;            /* optional read-only mmap image (big ROMs) */
    void *rom_map;              /* GMappedFile* when rom_image is used */

    uint32_t regs[0x400 / 4];

    uint8_t txfifo[1024];       /* covers one page (256B x dual 2) plus slack */
    uint32_t tx_cnt;
    uint8_t rxfifo[64];
    uint32_t rx_cnt;
    uint32_t rx_pos;

    bool pp_active;             /* page-program transaction collecting data */
    uint32_t pp_addr;
    uint32_t pp_total;

    uint8_t sr;                 /* status register S0-S7 (WIP/WEL/BP) */
    uint8_t sr2;                /* status register S8-S15 (QE/CMP) */

    /* ---- SPI NAND die (route addr==0x61, /dev/nand_data) ----
     * GD5F1GQ5-class: 2048+64B pages, 64 pages/block, 1024 blocks
     * (128MiB data + 4MiB OOB, 2112B per physical page). */
    char *nand_image;           /* writable mmap backend file */
    void *nand_map;             /* GMappedFile* (writable) */
    uint32_t nand_size;         /* backend bytes (pages * 2112) */
    uint8_t nand_sr[256];       /* feature registers by address (A0/B0/C0/D0) */
    uint8_t nand_cache[2112];   /* page cache for read/program */
    uint32_t nand_prog_col;
    bool nand_cache_dirty;

    Notifier shutdown_notifier; /* image writeback on exit */
};

/* storage backend for XIP mapping in the machine */
MemoryRegion *bes_flash_storage(BesFlashState *s);
uint8_t *bes_flash_data(BesFlashState *s);

#endif
