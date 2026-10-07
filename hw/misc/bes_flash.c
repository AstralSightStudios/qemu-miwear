/*
 * BES NOR flash controller (best1503, CHIP_FLASH_CTRL_VER >= 7)
 *
 * Register layout: SDK platform/hal/reg_norflaship_v2.h:21-129
 * Transaction model: SDK platform/hal/hal_norflaship_v2.c:202-295
 *   - write REG_000 {CMD[7:0], ADDR[31:8]} triggers a transaction
 *   - normal mode (REG_004.NEW_CMD_EN=0): RX byte count for read commands
 *     is REG_004.BLOCK_SIZE[24:12] (hal_norflaship_v2.c:202-210,
 *     norflash_drv.c:2761-2780 norflash_read uses blksize + cmd_addr)
 *   - extended mode (NEW_CMD_EN=1): tx_len = REG_004.BLOCK_SIZE[24:12],
 *     rx_len = REG_000.NEW_CMD_RX_LEN[25:13] when RX_EN(bit8) set
 *   - TX FIFO via REG_008, RX FIFO via REG_010
 *   - REG_00C status (VER>=7 layout, reg_norflaship_v2.h:186-202):
 *     BUSY bit0, TXFIFO_EMPTY bit1, TXFIFO_FULL bit2, RXFIFO_EMPTY bit3,
 *     RXFIFO_COUNT[8:4], TXFIFO_EMPCNT[13:9]
 *   - REG_018 bit0/bit1: RX/TX FIFO clear (self-clearing)
 *
 * Dual-chip mode (REG_204 bit19, best1503 Mi Band 10: two 8MB dies
 * interleaved into one 16MB logical device, firmware boot info
 * FLASH_DUAL_CHIP=1 / FLASH_SIZE=0x1000000):
 *   - 1x-wire register reads (0x9F/0x05/0x35/0x4B...): both dies shift
 *     simultaneously, every 4 SCLK the controller stores one FIFO byte
 *     with die0's nibble in [3:0] and die1's nibble in [7:4]; software
 *     de-interleaves (SDK norflash_drv.c:1505-1512, CTRL_VER>6 branch,
 *     verified against firmware sub_C0C4E30). For die bytes C0/C1 the
 *     FIFO pair is: B0 = (C1 & 0xF0) | (C0 >> 4),
 *                   B1 = ((C1 & 0x0F) << 4) | (C0 & 0x0F)
 *   - data reads (0x03/0x0B/...): the controller re-assembles the logical
 *     byte stream itself (even byte from die0, odd from die1); the RX FIFO
 *     carries plain logical bytes (SDK norflash_drv.c:2727-2785 reads
 *     len bytes with no de-interleaving)
 *   - writes: logical byte stream likewise (erase/page sizes double,
 *     SDK norflash_drv.c:996-1037)
 *   - TX_DATA_COPY (REG_204 bit21) duplicates TX bytes to both dies
 *     (register writes); no effect on the logical-view model
 *
 * Flash die model: GigaDevice GD25LQ64C (8MB per die),
 * JEDEC ID c8-60-17 (SDK norflash_gd25lq32c.c:240-241; two dies give
 * the 16MB logical size from the firmware boot info).
 * Command set: SDK platform/drivers/norflash/norflash_gd25q32c.h:57-96
 *
 * Storage backend: RAM, NOR semantics (program 1->0 only, erase -> 0xFF),
 * optionally loaded from / written back to an image file (property
 * "image"). The backend MemoryRegion is shared with the XIP/cached/NC
 * windows so flash writes are immediately visible to XIP.
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "system/runstate.h"
#include "system/system.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/bes_flash.h"

/* GD25Q32C-family command set (norflash_gd25q32c.h:57-96) */
#define FLASH_CMD_ID              0x9F
#define FLASH_CMD_WRITE_ENABLE    0x06
#define FLASH_CMD_WRITE_DISABLE   0x04
#define FLASH_CMD_PAGE_PROGRAM    0x02
#define FLASH_CMD_DUAL_PP         0xA2
#define FLASH_CMD_QUAD_PP         0x32
#define FLASH_CMD_BLOCK32_ERASE   0x52
#define FLASH_CMD_BLOCK64_ERASE   0xD8
#define FLASH_CMD_SECTOR_ERASE    0x20
#define FLASH_CMD_CHIP_ERASE      0x60
#define FLASH_CMD_CHIP_ERASE2     0xC7
#define FLASH_CMD_READ_SR         0x05
#define FLASH_CMD_READ_SR2        0x35
#define FLASH_CMD_WRITE_SR        0x01
#define FLASH_CMD_WRITE_SR2       0x31
#define FLASH_CMD_DTR_QUAD_READ   0xED
#define FLASH_CMD_QUAD_IO_READ    0xEB
#define FLASH_CMD_QUAD_OUT_READ   0x6B
#define FLASH_CMD_DUAL_IO_READ    0xBB
#define FLASH_CMD_DUAL_OUT_READ   0x3B
#define FLASH_CMD_READ            0x03
#define FLASH_CMD_FAST_READ       0x0B
#define FLASH_CMD_DEEP_POWERDOWN  0xB9
#define FLASH_CMD_RELEASE_DP      0xAB
#define FLASH_CMD_ENABLE_QPI      0x38
#define FLASH_CMD_DISABLE_QPI     0xFF
#define FLASH_CMD_HIGH_PERF       0xA3
#define FLASH_CMD_SET_BURST_WRAP  0x77
#define FLASH_CMD_UNIQUE_ID       0x4B
#define FLASH_CMD_READ_SFDP       0x5A
#define FLASH_CMD_ENABLE_RESET    0x66
#define FLASH_CMD_RESET           0x99
#define FLASH_CMD_VOLATILE_SR_EN  0x50
#define FLASH_CMD_PE_SUSPEND      0x75
#define FLASH_CMD_PE_RESUME       0x7A
#define FLASH_CMD_SECREG_ERASE    0x44
#define FLASH_CMD_SECREG_PROGRAM  0x42
#define FLASH_CMD_SECREG_READ     0x48
#define FLASH_CMD_ENTER_4BYTE     0xB7
#define FLASH_CMD_EXIT_4BYTE      0xE9

/* status register bits (norflash_gd25q32c.h:100-112) */
#define FLASH_SR_WIP   (1 << 0)
#define FLASH_SR_WEL   (1 << 1)
#define FLASH_SR2_QE   (1 << 1)

#define FLASH_TXFIFO_DEPTH 1024
#define FLASH_RXFIFO_DEPTH 64

/* register offsets (reg_norflaship_v2.h) */
#define FLASH_REG_000   0x000   /* CMD | ADDR<<8 | NEW_CMD fields */
#define FLASH_REG_004   0x004   /* BLOCK_SIZE / NEW_CMD_EN */
#define FLASH_REG_008   0x008   /* TX FIFO data */
#define FLASH_REG_00C   0x00C   /* status */
#define FLASH_REG_010   0x010   /* RX FIFO data */
#define FLASH_REG_018   0x018   /* FIFO clear (self-clearing) */
#define FLASH_REG_204   0x204   /* dual-chip / data copy / reorder config */

/* REG_000 fields (reg_norflaship_v2.h:131-152) */
#define R000_CMD(x)             ((x) & 0xFF)
#define R000_ADDR(x)            ((x) >> 8)
#define R000_NEW_CMD_RX_LEN(x)  (((x) >> 13) & 0x1FFF)
#define R000_NEW_CMD_RX_EN      (1 << 8)
/* REG_004 fields (reg_norflaship_v2.h:154-168) */
#define R004_BLOCK_SIZE(x)      (((x) >> 12) & 0x1FFF)
#define R004_NEW_CMD_EN         (1 << 0)
/* REG_204 fields (reg_norflaship_v2.h:600-624) */
#define R204_TX_DATA_COPY       (1 << 21)
#define R204_RX_DATA_REORDER    (1 << 20)
#define R204_DUAL_CHIP_MODE     (1 << 19)

/* JEDEC ID of each GD25LQ64C die (SDK norflash_gd25lq32c.c:241) */
static const uint8_t bes_flash_jedec_id[3] = { 0xC8, 0x60, 0x17 };

/* Unique ID: GD dies report 18 bytes (SDK norflash_drv.c:56
 * GD_UNIQUE_ID_LEN). Per-unit values are not published; model a fixed
 * constant so firmware reads are deterministic. */
static const uint8_t bes_flash_unique_id[18] = {
    0x4D, 0x69, 0x42, 0x61, 0x6E, 0x64, 0x31, 0x30, 0x00,
    0xB5, 0xE5, 0x15, 0x03, 0x00, 0x00, 0x00, 0x00, 0x01,
};

static bool bes_flash_dual(BesFlashState *s)
{
    return s->regs[FLASH_REG_204 / 4] & R204_DUAL_CHIP_MODE;
}

/* whether the command has a data-out phase that drains TX FIFO bytes.
 * Command-only operations (WREN/erase/reads/mode changes) leave queued
 * data in the FIFO for a following page-program (SDK norflash_write
 * queues page data before issuing WREN + PAGE_PROGRAM). */
static bool bes_flash_cmd_has_tx(uint8_t cmd)
{
    switch (cmd) {
    case FLASH_CMD_WRITE_ENABLE:
    case FLASH_CMD_WRITE_DISABLE:
    case FLASH_CMD_SECTOR_ERASE:
    case FLASH_CMD_BLOCK32_ERASE:
    case FLASH_CMD_BLOCK64_ERASE:
    case FLASH_CMD_CHIP_ERASE:
    case FLASH_CMD_CHIP_ERASE2:
    case FLASH_CMD_ID:
    case FLASH_CMD_READ_SR:
    case FLASH_CMD_READ_SR2:
    case FLASH_CMD_READ:
    case FLASH_CMD_FAST_READ:
    case FLASH_CMD_DUAL_OUT_READ:
    case FLASH_CMD_DUAL_IO_READ:
    case FLASH_CMD_QUAD_OUT_READ:
    case FLASH_CMD_QUAD_IO_READ:
    case FLASH_CMD_DTR_QUAD_READ:
    case FLASH_CMD_SECREG_READ:
    case FLASH_CMD_DEEP_POWERDOWN:
    case FLASH_CMD_RELEASE_DP:
    case FLASH_CMD_ENABLE_QPI:
    case FLASH_CMD_HIGH_PERF:
    case FLASH_CMD_ENABLE_RESET:
    case FLASH_CMD_RESET:
    case FLASH_CMD_VOLATILE_SR_EN:
    case FLASH_CMD_PE_SUSPEND:
    case FLASH_CMD_PE_RESUME:
    case FLASH_CMD_ENTER_4BYTE:
    case FLASH_CMD_EXIT_4BYTE:
        return false;
    default:
        return true;
    }
}

static void bes_flash_rx_fill(BesFlashState *s, const uint8_t *data,
                              uint32_t len)
{
    uint32_t n = MIN(len, FLASH_RXFIFO_DEPTH);

    memcpy(s->rxfifo, data, n);
    s->rx_cnt = n;
    s->rx_pos = 0;
}

/* Dual-chip 1x-wire RX packing (norflash_drv.c:1505-1512 inverse):
 * both dies respond identically here (same die model, same register
 * state); per-die byte C -> FIFO pair (B0, B1). */
static void bes_flash_rx_fill_dual(BesFlashState *s, const uint8_t *die,
                                   uint32_t len)
{
    uint8_t buf[FLASH_RXFIFO_DEPTH];
    uint32_t i, n = 0;

    for (i = 0; i < len && n + 2 <= sizeof(buf); i++) {
        uint8_t c = die[i];
        buf[n++] = (c & 0xF0) | (c >> 4);        /* die1 hi | die0 hi nibble */
        buf[n++] = ((c & 0x0F) << 4) | (c & 0x0F);
    }
    bes_flash_rx_fill(s, buf, n);
}

/*
 * RX response helper: register-class read on 1x wires.
 * Dual-chip -> nibble-interleaved FIFO; single -> plain bytes.
 */
static void bes_flash_respond_reg(BesFlashState *s, const uint8_t *data,
                                  uint32_t len)
{
    if (bes_flash_dual(s)) {
        bes_flash_rx_fill_dual(s, data, len);
    } else {
        bes_flash_rx_fill(s, data, len);
    }
}

/* erase helper: dual-chip doubles the logical erase granule
 * (norflash_drv.c:996-1013 total/page x2; erase reaches both dies) */
static void bes_flash_erase(BesFlashState *s, uint32_t addr, uint32_t size)
{
    uint32_t gran = bes_flash_dual(s) ? 2 * size : size;
    uint32_t base = addr & ~(gran - 1);

    if (s->rom_map) {   /* read-only mmap backend (romfs) */
        qemu_log_mask(LOG_UNIMP,
                      "bes.flash: erase ignored on read-only backend\n");
        return;
    }
    if (!(s->sr & FLASH_SR_WEL)) {
        return;
    }
    if (base < s->size) {
        memset(s->storage + base, 0xFF, MIN(gran, s->size - base));
    }
    s->sr &= ~FLASH_SR_WEL;
}

/* execute a pending page-program once the full page has been collected.
 * Real hardware streams TX FIFO bytes over SPI while the CPU keeps
 * refilling (norflash_drv.c:2668-2674), so the program command only
 * completes after BLOCK_SIZE bytes. */
static void bes_flash_do_program(BesFlashState *s)
{
    uint32_t n, i;

    /* rom-image backends are read-only mmaps; programming them would
     * segfault the process. Real firmware never programs the romfs
     * partition. */
    if (s->rom_map) {
        qemu_log_mask(LOG_UNIMP,
                      "bes.flash: program ignored on read-only backend\n");
        s->pp_active = false;
        s->tx_cnt = 0;
        return;
    }

    if ((s->sr & FLASH_SR_WEL) && s->pp_addr < s->size) {
        n = MIN(s->pp_total, s->size - s->pp_addr);
        for (i = 0; i < n; i++) {
            s->storage[s->pp_addr + i] &= s->txfifo[i];
        }
        s->sr &= ~FLASH_SR_WEL;
    }
    s->pp_active = false;
    s->tx_cnt = 0;
}

static void bes_flash_exec(BesFlashState *s, uint32_t reg000)
{
    uint8_t cmd = R000_CMD(reg000);
    uint32_t addr = R000_ADDR(reg000);
    bool ext = s->regs[FLASH_REG_004 / 4] & R004_NEW_CMD_EN;
    uint32_t tx_len = R004_BLOCK_SIZE(s->regs[FLASH_REG_004 / 4]);
    /* normal mode: BLOCK_SIZE is the RX count for read commands
     * (hal_norflaship_v2.c:202-210, norflash_drv.c:2761-2766);
     * extended mode: RX length comes from REG_000 when RX_EN */
    uint32_t rx_len = ext ? ((reg000 & R000_NEW_CMD_RX_EN) ?
                             R000_NEW_CMD_RX_LEN(reg000) : 0) : tx_len;

    qemu_log_mask(LOG_UNIMP, "bes.flash: cmd=0x%02x addr=0x%06x ext=%d "
                  "tx_len=%u rx_len=%u dual=%d\n",
                  cmd, addr, ext, tx_len, rx_len, bes_flash_dual(s));

    switch (cmd) {
    case FLASH_CMD_ID: {
        uint32_t n = rx_len ? MIN(rx_len / (bes_flash_dual(s) ? 2 : 1),
                                  (uint32_t)sizeof(bes_flash_jedec_id))
                            : (uint32_t)sizeof(bes_flash_jedec_id);
        bes_flash_respond_reg(s, bes_flash_jedec_id, n);
        break;
    }
    case FLASH_CMD_UNIQUE_ID: {
        uint32_t n = rx_len ? MIN(rx_len / (bes_flash_dual(s) ? 2 : 1),
                                  (uint32_t)sizeof(bes_flash_unique_id))
                            : (uint32_t)sizeof(bes_flash_unique_id);
        bes_flash_respond_reg(s, bes_flash_unique_id, n);
        break;
    }
    case FLASH_CMD_READ_SR: {
        /* GD25Q32C 0x05 reads SR1 and, when more bytes are requested,
         * continues with SR2 (sequential status read). Firmware issues
         * rx_len=2 to fetch both in one command. */
        uint8_t srbuf[2];
        uint32_t n = rx_len ? MIN(rx_len / (bes_flash_dual(s) ? 2 : 1),
                                  (uint32_t)sizeof(srbuf)) : 1;

        srbuf[0] = s->sr;
        srbuf[1] = s->sr2;
        bes_flash_respond_reg(s, srbuf, n);
        break;
    }
    case FLASH_CMD_READ_SR2:
        bes_flash_respond_reg(s, &s->sr2, 1);
        break;
    case FLASH_CMD_READ:
    case FLASH_CMD_FAST_READ:
    case FLASH_CMD_DUAL_OUT_READ:
    case FLASH_CMD_DUAL_IO_READ:
    case FLASH_CMD_QUAD_OUT_READ:
    case FLASH_CMD_QUAD_IO_READ:
    case FLASH_CMD_DTR_QUAD_READ: {
        /* data read: controller re-assembles the logical byte stream
         * itself, RX FIFO carries plain bytes (norflash_drv.c:2768-2780) */
        if (rx_len && addr < s->size) {
            bes_flash_rx_fill(s, s->storage + addr,
                              MIN(rx_len, s->size - addr));
        }
        break;
    }
    case FLASH_CMD_SECREG_READ:
        /* security registers are one-time-programmable area; never
         * programmed on this unit -> erased state 0xFF */
        if (rx_len) {
            uint8_t ff[FLASH_RXFIFO_DEPTH];

            memset(ff, 0xFF, sizeof(ff));
            bes_flash_rx_fill(s, ff, rx_len);
        }
        break;
    case FLASH_CMD_WRITE_ENABLE:
        s->sr |= FLASH_SR_WEL;
        break;
    case FLASH_CMD_WRITE_DISABLE:
        s->sr &= ~FLASH_SR_WEL;
        break;
    case FLASH_CMD_PAGE_PROGRAM:
    case FLASH_CMD_DUAL_PP:
    case FLASH_CMD_QUAD_PP:
    case FLASH_CMD_SECREG_PROGRAM:
        /* start collecting TX bytes; completes when BLOCK_SIZE bytes
         * have been pushed (hardware streams them while the CPU refills,
         * SDK norflash_drv.c:2668-2674 write_txfifo_cont loop) */
        s->pp_addr = addr;
        s->pp_total = (cmd == FLASH_CMD_SECREG_PROGRAM) ? 0 : tx_len;
        s->pp_active = true;
        if (s->tx_cnt >= s->pp_total) {
            bes_flash_do_program(s);
        }
        break;
    case FLASH_CMD_SECTOR_ERASE:
        bes_flash_erase(s, addr, 0x1000);
        break;
    case FLASH_CMD_BLOCK32_ERASE:
        bes_flash_erase(s, addr, 0x8000);
        break;
    case FLASH_CMD_BLOCK64_ERASE:
        bes_flash_erase(s, addr, 0x10000);
        break;
    case FLASH_CMD_CHIP_ERASE:
    case FLASH_CMD_CHIP_ERASE2:
        if (s->rom_map) {
            qemu_log_mask(LOG_UNIMP,
                          "bes.flash: chip erase ignored on read-only backend\n");
        } else if (s->sr & FLASH_SR_WEL) {
            memset(s->storage, 0xFF, s->size);
            s->sr &= ~FLASH_SR_WEL;
        }
        break;
    case FLASH_CMD_WRITE_SR:
        /* 0x01 writes SR1 and, when two bytes are queued, SR2 as well
         * (GD25Q32C sequential status write; drv_norflash_write_status
         * sets QE in SR2 bit1 via a 2-byte write) */
        if (s->tx_cnt > 0) {
            s->sr = s->txfifo[0] & ~FLASH_SR_WIP;
            if (s->tx_cnt > 1) {
                s->sr2 = s->txfifo[1];
            }
            s->tx_cnt = 0;
        }
        break;
    case FLASH_CMD_WRITE_SR2:
        if (s->tx_cnt > 0) {
            s->sr2 = s->txfifo[0];
            s->tx_cnt = 0;
        }
        break;
    case FLASH_CMD_DEEP_POWERDOWN:
    case FLASH_CMD_RELEASE_DP:
    case FLASH_CMD_ENABLE_QPI:
    case FLASH_CMD_DISABLE_QPI:
    case FLASH_CMD_HIGH_PERF:
    case FLASH_CMD_SET_BURST_WRAP:
    case FLASH_CMD_ENABLE_RESET:
    case FLASH_CMD_RESET:
    case FLASH_CMD_VOLATILE_SR_EN:
    case FLASH_CMD_PE_SUSPEND:
    case FLASH_CMD_PE_RESUME:
    case FLASH_CMD_ENTER_4BYTE:
    case FLASH_CMD_EXIT_4BYTE:
        /* known stateless commands: mode flags live in controller
         * registers (already stored); die-side modes do not affect
         * the logical-view model */
        break;
    case FLASH_CMD_READ_SFDP:
        /* GD25LQ64C has SFDP; firmware only does SFDP probing for Puya
         * parts (norflash_drv.c:2441-2487 list). Report erased state so
         * any unexpected probe sees "no table". */
        if (rx_len) {
            uint8_t ff[FLASH_RXFIFO_DEPTH];

            memset(ff, 0xFF, sizeof(ff));
            bes_flash_respond_reg(s, ff, MIN(rx_len,
                                             (uint32_t)sizeof(ff)));
        }
        break;
    case FLASH_CMD_SECREG_ERASE:
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "bes.flash: unknown cmd 0x%02x\n", cmd);
        break;
    }

    /* TX FIFO is NOT drained here: command-only transactions (WREN, erase,
     * reads) leave queued data bytes for a following page-program, and a
     * pending program keeps collecting until pp_total bytes arrive
     * (bes_flash_do_program). */
    if (bes_flash_cmd_has_tx(cmd) && !s->pp_active && tx_len) {
        uint32_t consume = MIN(s->tx_cnt, tx_len);

        if (consume) {
            memmove(s->txfifo, s->txfifo + consume, s->tx_cnt - consume);
            s->tx_cnt -= consume;
        }
    }
}

static uint32_t bes_flash_status(BesFlashState *s)
{
    uint32_t v = 0;

    /* REG_00C, CTRL_VER>=7 layout (reg_norflaship_v2.h:186-202) */
    if (s->rx_cnt - s->rx_pos == 0) {
        v |= (1 << 3);   /* RXFIFO_EMPTY */
    }
    if (s->tx_cnt == 0) {
        v |= (1 << 1);   /* TXFIFO_EMPTY */
    }
    if (s->tx_cnt >= FLASH_TXFIFO_DEPTH) {
        v |= (1 << 2);   /* TXFIFO_FULL */
    }
    v |= ((s->rx_cnt - s->rx_pos) & 0x1F) << 4;         /* RXFIFO_COUNT */
    /* TXFIFO_EMPCNT is a 5-bit field (max 31); an empty FIFO must read
     * 31, not (32 & 0x1F)=0 which firmware would take as "FIFO full"
     * (hal_norflaship_v2.c:132-169) */
    v |= (MIN(FLASH_TXFIFO_DEPTH - s->tx_cnt, 31) & 0x1F) << 9;
    /* BUSY(bit0) = 0: transactions complete instantly */
    return v;
}

static uint64_t bes_flash_read(void *opaque, hwaddr offset, unsigned size)
{
    BesFlashState *s = BES_FLASH(opaque);
    uint64_t v = 0;

    switch (offset) {
    case FLASH_REG_00C:
        v = bes_flash_status(s);
        break;
    case FLASH_REG_010: {
        /* RX FIFO pop unit comes from REG_204 CFG_RXSIZE[18:17]
         * (0=1B/1=2B/2=4B, hal_norflaship_v2.c:989-1002), NOT from the
         * CPU access size: the SDK reads REG_010 as a word even in
         * 1-byte mode and consumes only the low byte
         * (hal_norflaship_v2.c:198-201, norflash_drv.c:1483-1485). */
        uint32_t cfg = (s->regs[FLASH_REG_204 / 4] >> 17) & 0x3;
        uint32_t width = 1 << MIN(cfg, 2);
        uint32_t n;

        for (n = 0; n < width && s->rx_pos < s->rx_cnt; n++) {
            v |= (uint64_t)s->rxfifo[s->rx_pos++] << (n * 8);
        }
        break;
    }
    case FLASH_REG_018:
        v = 0;   /* self-clearing */
        break;
    default:
        if ((offset >> 2) < ARRAY_SIZE(s->regs)) {
            v = s->regs[offset >> 2];
        }
        break;
    }

    qemu_log_mask(LOG_UNIMP, "bes.flash: read  off=0x%03x -> 0x%0*llx\n",
                  (unsigned)offset, size * 2, (unsigned long long)v);
    return v;
}

static void bes_flash_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    BesFlashState *s = BES_FLASH(opaque);

    qemu_log_mask(LOG_UNIMP, "bes.flash: write off=0x%03x = 0x%0*llx\n",
                  (unsigned)offset, size * 2, (unsigned long long)value);

    switch (offset) {
    case FLASH_REG_000:
        s->regs[0] = value;
        bes_flash_exec(s, value);
        break;
    case FLASH_REG_008: {
        /* TX FIFO push, byte-wise regardless of access size */
        uint32_t n;

        for (n = 0; n < size; n++) {
            if (s->tx_cnt < FLASH_TXFIFO_DEPTH) {
                s->txfifo[s->tx_cnt++] = (value >> (n * 8)) & 0xFF;
            }
        }
        /* a pending page-program completes once the full page arrived */
        if (s->pp_active && s->tx_cnt >= s->pp_total) {
            bes_flash_do_program(s);
        }
        break;
    }
    case FLASH_REG_018:
        if (value & 1) {   /* RXFIFOCLR */
            s->rx_cnt = 0;
            s->rx_pos = 0;
        }
        if (value & 2) {   /* TXFIFOCLR */
            s->tx_cnt = 0;
        }
        break;
    default:
        if ((offset >> 2) < ARRAY_SIZE(s->regs)) {
            s->regs[offset >> 2] = value;
        }
        break;
    }
}

static const MemoryRegionOps bes_flash_ops = {
    .read = bes_flash_read,
    .write = bes_flash_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

MemoryRegion *bes_flash_storage(BesFlashState *s)
{
    return &s->storage_mr;
}

uint8_t *bes_flash_data(BesFlashState *s)
{
    return s->storage;
}

/* write the storage back to the backing image on exit */
static void bes_flash_shutdown(Notifier *n, void *opaque)
{
    BesFlashState *s = BES_FLASH(opaque);
    FILE *fp;

    if (!s->image) {
        return;
    }
    fp = fopen(s->image, "r+b");
    if (!fp) {
        fp = fopen(s->image, "wb");
    }
    if (!fp) {
        warn_report("bes.flash: cannot open image '%s' for writeback",
                    s->image);
        return;
    }
    if (fwrite(s->storage, 1, s->size, fp) != s->size) {
        warn_report("bes.flash: short writeback to '%s'", s->image);
    }
    fclose(fp);
}

static void bes_flash_realize(DeviceState *dev, Error **errp)
{
    BesFlashState *s = BES_FLASH(dev);
    FILE *fp;

    /* read-only mmap backend for huge ROMs (resource flash): demand
     * paging, no physical memory cost */
    if (s->rom_image) {
        GMappedFile *mf;
        g_autofree char *name =
            g_strdup_printf("bes.flash.storage.%p", (void *)s);
        uint64_t fsize;

        mf = g_mapped_file_new(s->rom_image, FALSE, NULL);
        if (!mf) {
            error_setg(errp, "bes.flash: cannot mmap rom image '%s'",
                       s->rom_image);
            return;
        }
        fsize = g_mapped_file_get_length(mf);
        if (s->size == 0 || s->size > fsize) {
            s->size = fsize;
        }
        s->rom_map = mf;
        s->storage = (uint8_t *)g_mapped_file_get_contents(mf);
        memory_region_init_ram_ptr(&s->storage_mr, OBJECT(dev), name,
                                   s->size, s->storage);
        memory_region_set_readonly(&s->storage_mr, true);
        info_report("bes.flash: rom image '%s' mapped, 0x%x bytes",
                    s->rom_image, s->size);
    } else {
        if (s->size == 0) {
            error_setg(errp, "bes.flash: size property is required");
            return;
        }

        /* storage backend, NOR erased state = 0xFF. RAMBlock name must be
         * unique across instances (flash0/flash1 share this model). */
        {
            g_autofree char *name =
                g_strdup_printf("bes.flash.storage.%p", (void *)s);

            memory_region_init_ram(&s->storage_mr, OBJECT(dev), name,
                                   s->size, &error_fatal);
        }
        s->storage = memory_region_get_ram_ptr(&s->storage_mr);
        memset(s->storage, 0xFF, s->size);
    }

    /* optional backing image: load content, write back on shutdown */
    if (s->image) {
        fp = fopen(s->image, "rb");
        if (fp) {
            size_t got = fread(s->storage, 1, s->size, fp);

            fclose(fp);
            info_report("bes.flash: loaded 0x%zx/0x%x bytes from '%s'",
                        got, s->size, s->image);
        } else {
            info_report("bes.flash: image '%s' not found, starting erased",
                        s->image);
        }
        s->shutdown_notifier.notify = bes_flash_shutdown;
        qemu_register_shutdown_notifier(&s->shutdown_notifier);
    }

    memory_region_init_io(&s->iomem, OBJECT(dev), &bes_flash_ops, s,
                          "bes.flash", 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void bes_flash_reset(DeviceState *dev)
{
    BesFlashState *s = BES_FLASH(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->tx_cnt = 0;
    s->rx_cnt = 0;
    s->rx_pos = 0;
    s->pp_active = false;
    s->sr = 0;
    s->sr2 = 0;
}

static const Property bes_flash_props[] = {
    DEFINE_PROP_UINT32("size", BesFlashState, size, 0),
    DEFINE_PROP_STRING("image", BesFlashState, image),
    DEFINE_PROP_STRING("rom-image", BesFlashState, rom_image),
};

static void bes_flash_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = bes_flash_realize;
    device_class_set_props(dc, bes_flash_props);
    device_class_set_legacy_reset(dc, bes_flash_reset);
}

static const TypeInfo bes_flash_info = {
    .name = TYPE_BES_FLASH,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BesFlashState),
    .class_init = bes_flash_class_init,
};

static void bes_flash_register_types(void)
{
    type_register_static(&bes_flash_info);
}

type_init(bes_flash_register_types)
