/*
 * BES serial flash controller (best1503, CHIP_FLASH_CTRL_VER >= 7)
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
 * Firmware-verified controller details (vela_ap.bin v3.101.043; the
 * norflaship helpers live in the boot RAM-text section that startup
 * sub_C0C0A1C copies from flash 0x2C0C23A4 to SRAM 0x200765C0, reached
 * through the veneers at 0xC588xxx):
 *   - controller table 0x2009D8F0: id0 = 0x40140000, id1 = 0x40500000
 *   - REG_01C bit1 / bit2: TX / RX DMA enable (sub_C0C4498 / sub_C0C44B4,
 *     set around nandflash_start_tx_dma / nandflash_start_rx_dma and
 *     cleared again by sub_C221270)
 *   - REG_01C[17:16]: RX FIFO access width for DMA, 0 = 1 byte,
 *     1 = 2 bytes, 2 = 4 bytes (sub_C0C44F8; set to 4 bytes before the
 *     DMA page read, back to 1 byte afterwards)
 *   - DMA data port is REG_008 in both directions: the DMA periph address
 *     table (0x2CCDA934, request 28 -> 0x40500008) is used as source for
 *     the RX descriptor and as destination for the TX descriptor
 *
 * Two chip personalities share this controller model:
 *
 * NOR (flash0, code flash). Dual-chip mode (REG_204 bit19, two 8MB dies
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
 *   Die model: GigaDevice GD25LQ64C (8MB per die), JEDEC ID c8-60-17
 *   (SDK norflash_gd25lq32c.c:240-241); property "jedec-id" picks
 *   another die (best1700: GD25LQ128, see best1700.c). Command set: SDK
 *   platform/drivers/norflash/norflash_gd25q32c.h:57-96. Storage backend:
 *   RAM with NOR semantics, optionally loaded from / written back to an
 *   image file (property "image"); shared with the XIP windows.
 *
 * SPI NAND (flash1, /dev/resource + /dev/nand_data), firmware
 * nandflash_drv (sub_C221F4C nandflash_init, controller id 1):
 *   - READ ID: ext cmd 0x9F, 1 TX dummy byte, 3 RX bytes compared with
 *     the firmware ID table 0x2CCD4134 (sub_C221460)
 *   - GET/SET FEATURE 0x0F/0x1F (ext, TX feature address [+ value]);
 *     A0 protection, B0 configuration, C0 status: OIP bit0, WEL bit1,
 *     E_FAIL bit2, P_FAIL bit3, ECC status [6:4] (sub_C2214CC/A8,
 *     sub_C22187E, sub_C221594 / sub_C221500 / sub_C221818)
 *   - PAGE READ 0x13, PROGRAM EXECUTE 0x10, BLOCK ERASE 0xD8, WREN 0x06:
 *     normal-mode commands, row address in REG_000[31:8]
 *     (sub_C221594, sub_C221500, sub_C221818)
 *   - READ FROM CACHE 0x03/0x0B/0x3B/0x6B/0xBB/0xEB: normal mode, column
 *     in REG_000[31:8], length REG_004.BLOCK_SIZE (sub_C221680)
 *   - PROGRAM LOAD 0x02/0x32: normal mode, column in REG_000[31:8], the
 *     BLOCK_SIZE data bytes stream through the TX FIFO while software
 *     (or DMA) keeps refilling it (sub_C2212C4)
 *   - DIE SELECT 0xC2 (ext, 1 TX byte), DPD 0xB9 / release 0xAB (ext),
 *     RESET 0xFF (ext) (sub_C2211E4, sub_C221A60, sub_C221AC8)
 *   - memory-mapped cache reads: the controller's NC window returns the
 *     page cache at column = window offset; the driver reads the
 *     bad-block markers and unaligned tails through 0x30000000 + column
 *     (sub_C2215FC, sub_C221680)
 *   Backing store: a file of (page + oob)-byte pages, page-major; erased
 *   state 0xFF. Chip geometry and ID come from properties.
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "system/runstate.h"
#include "system/system.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/bes_flash.h"
#include "trace.h"

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

/* SPI NAND command set (firmware nandflash_drv, see header comment) */
#define NAND_CMD_RESET            0xFF
#define NAND_CMD_READ_ID          0x9F
#define NAND_CMD_GET_FEATURE      0x0F
#define NAND_CMD_SET_FEATURE      0x1F
#define NAND_CMD_WRITE_ENABLE     0x06
#define NAND_CMD_WRITE_DISABLE    0x04
#define NAND_CMD_PAGE_READ        0x13
#define NAND_CMD_READ_CACHE       0x03
#define NAND_CMD_FAST_READ_CACHE  0x0B
#define NAND_CMD_READ_CACHE_X2    0x3B
#define NAND_CMD_READ_CACHE_X4    0x6B
#define NAND_CMD_READ_CACHE_DIO   0xBB
#define NAND_CMD_READ_CACHE_QIO   0xEB
#define NAND_CMD_PROGRAM_LOAD     0x02
#define NAND_CMD_PROGRAM_LOAD_X4  0x32
#define NAND_CMD_RND_LOAD         0x84
#define NAND_CMD_RND_LOAD_X4      0x34
#define NAND_CMD_PROGRAM_EXEC     0x10
#define NAND_CMD_BLOCK_ERASE      0xD8
#define NAND_CMD_DIE_SELECT       0xC2
#define NAND_CMD_DEEP_POWERDOWN   0xB9
#define NAND_CMD_RELEASE_DP       0xAB

#define NAND_FEAT_PROT            0xA0
#define NAND_FEAT_CFG             0xB0
#define NAND_FEAT_STATUS          0xC0
#define NAND_FEAT_DRIVE           0xD0
#define NAND_ST_OIP               (1 << 0)
#define NAND_ST_WEL               (1 << 1)
#define NAND_ST_EFAIL             (1 << 2)
#define NAND_ST_PFAIL             (1 << 3)
#define NAND_CFG_ECC_EN           (1 << 4)

/* status register bits (norflash_gd25q32c.h:100-112) */
#define FLASH_SR_WIP   (1 << 0)
#define FLASH_SR_WEL   (1 << 1)
#define FLASH_SR2_QE   (1 << 1)

/* RXFIFO_COUNT is a 5-bit field; software only waits on it for short
 * register reads (sub_C0C4DA0 waits count >= len, len < 32) */
#define FLASH_RXFIFO_COUNT_MAX 31

/* register offsets (reg_norflaship_v2.h) */
#define FLASH_REG_000   0x000   /* CMD | ADDR<<8 | NEW_CMD fields */
#define FLASH_REG_004   0x004   /* BLOCK_SIZE / NEW_CMD_EN */
#define FLASH_REG_008   0x008   /* TX FIFO data / DMA data port */
#define FLASH_REG_00C   0x00C   /* status */
#define FLASH_REG_010   0x010   /* RX FIFO data */
#define FLASH_REG_018   0x018   /* FIFO clear (self-clearing) */
#define FLASH_REG_01C   0x01C   /* DMA control */
#define FLASH_REG_204   0x204   /* dual-chip / data copy / reorder config */

/* REG_000 fields (reg_norflaship_v2.h:131-152) */
#define R000_CMD(x)             ((x) & 0xFF)
#define R000_ADDR(x)            ((x) >> 8)
#define R000_NEW_CMD_RX_LEN(x)  (((x) >> 13) & 0x1FFF)
#define R000_NEW_CMD_RX_EN      (1 << 8)
/* REG_004 fields (reg_norflaship_v2.h:154-168) */
#define R004_BLOCK_SIZE(x)      (((x) >> 12) & 0x1FFF)
#define R004_NEW_CMD_EN         (1 << 0)
/* REG_01C fields (firmware, see header comment) */
#define R01C_TX_DMA_EN          (1 << 1)
#define R01C_RX_DMA_EN          (1 << 2)
#define R01C_RX_DMA_WIDTH(x)    (((x) >> 16) & 0x3)
/* REG_204 fields (reg_norflaship_v2.h:600-624) */
#define R204_TX_DATA_COPY       (1 << 21)
#define R204_RX_DATA_REORDER    (1 << 20)
#define R204_DUAL_CHIP_MODE     (1 << 19)

/* JEDEC ID of each GD25LQ64C die (SDK norflash_gd25lq32c.c:241) */
#define BES_FLASH_JEDEC_ID_LEN  3

/* Unique ID: GD dies report 18 bytes (SDK norflash_drv.c:56
 * GD_UNIQUE_ID_LEN). Per-unit values are not published; model a fixed
 * constant so firmware reads are deterministic. */
static const uint8_t bes_flash_unique_id[18] = {
    0x4D, 0x69, 0x42, 0x61, 0x6E, 0x64, 0x31, 0x30, 0x00,
    0xB5, 0xE5, 0x15, 0x03, 0x00, 0x00, 0x00, 0x00, 0x01,
};

static bool bes_flash_is_nand(BesFlashState *s)
{
    return s->nand_fp != NULL;
}

static bool bes_flash_dual(BesFlashState *s)
{
    return s->regs[FLASH_REG_204 / 4] & R204_DUAL_CHIP_MODE;
}

/* DMA request: RX while the transaction still has data to deliver, TX
 * while a streaming load wants bytes or the FIFO has room */
static void bes_flash_update_dma(BesFlashState *s)
{
    uint32_t r01c = s->regs[FLASH_REG_01C / 4];
    bool req = false;

    if ((r01c & R01C_RX_DMA_EN) && s->rx_pos < s->rx_cnt) {
        req = true;
    }
    if ((r01c & R01C_TX_DMA_EN) &&
        ((s->nand_load_active && s->nand_load_left) ||
         s->tx_cnt < BES_FLASH_TXFIFO_DEPTH)) {
        req = true;
    }
    qemu_set_irq(s->dma_req, req);
}

static void bes_flash_rx_fill(BesFlashState *s, const uint8_t *data,
                              uint32_t len)
{
    uint32_t n = MIN(len, BES_FLASH_RX_MAX);

    memcpy(s->rxbuf, data, n);
    s->rx_cnt = n;
    s->rx_pos = 0;
}

/* pad an RX stream to the requested length with a constant byte */
static void bes_flash_rx_fill_pad(BesFlashState *s, const uint8_t *data,
                                  uint32_t len, uint32_t want, uint8_t pad)
{
    want = MIN(want, BES_FLASH_RX_MAX);
    len = MIN(len, want);
    memcpy(s->rxbuf, data, len);
    memset(s->rxbuf + len, pad, want - len);
    s->rx_cnt = want;
    s->rx_pos = 0;
}

/* RX pop unit. REG_01C[17:16] selects the width used for DMA page reads
 * (firmware sub_C0C44F8); otherwise REG_204 CFG_RXSIZE[18:17]
 * (0=1B/1=2B/2=4B, hal_norflaship_v2.c:989-1002). Not the CPU access
 * size: the SDK reads REG_010 as a word even in 1-byte mode and consumes
 * only the low byte (hal_norflaship_v2.c:198-201,
 * norflash_drv.c:1483-1485). */
static uint32_t bes_flash_rx_width(BesFlashState *s)
{
    uint32_t dma = R01C_RX_DMA_WIDTH(s->regs[FLASH_REG_01C / 4]);
    uint32_t cfg = (s->regs[FLASH_REG_204 / 4] >> 17) & 0x3;

    if (dma) {
        return 1 << MIN(dma, 2);
    }
    return 1 << MIN(cfg, 2);
}

static uint32_t bes_flash_rx_pop(BesFlashState *s)
{
    uint32_t width = bes_flash_rx_width(s);
    uint32_t v = 0, n;

    for (n = 0; n < width && s->rx_pos < s->rx_cnt; n++) {
        v |= (uint32_t)s->rxbuf[s->rx_pos++] << (n * 8);
    }
    return v;
}

/* ======================================================================
 * NOR die model
 * ====================================================================== */

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

/* Dual-chip 1x-wire RX packing (norflash_drv.c:1505-1512 inverse):
 * both dies respond identically here (same die model, same register
 * state); per-die byte C -> FIFO pair (B0, B1). */
static void bes_flash_rx_fill_dual(BesFlashState *s, const uint8_t *die,
                                   uint32_t len)
{
    uint8_t buf[64];
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

static void bes_nor_exec(BesFlashState *s, uint8_t cmd, uint32_t addr,
                         uint32_t tx_len, uint32_t rx_len)
{
    switch (cmd) {
    case FLASH_CMD_ID: {
        const uint8_t id[BES_FLASH_JEDEC_ID_LEN] = {
            s->jedec_id >> 16, s->jedec_id >> 8, s->jedec_id,
        };
        uint32_t n = rx_len ? MIN(rx_len / (bes_flash_dual(s) ? 2 : 1),
                                  (uint32_t)sizeof(id))
                            : (uint32_t)sizeof(id);
        bes_flash_respond_reg(s, id, n);
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
    case FLASH_CMD_DTR_QUAD_READ:
        /* data read: controller re-assembles the logical byte stream
         * itself, RX FIFO carries plain bytes (norflash_drv.c:2768-2780) */
        if (rx_len && addr < s->size) {
            bes_flash_rx_fill(s, s->storage + addr,
                              MIN(rx_len, s->size - addr));
        }
        break;
    case FLASH_CMD_SECREG_READ:
        /* security registers are one-time-programmable area; never
         * programmed on this unit -> erased state 0xFF */
        if (rx_len) {
            bes_flash_rx_fill_pad(s, NULL, 0, rx_len, 0xFF);
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
        if (s->sr & FLASH_SR_WEL) {
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
            uint8_t ff[32];

            memset(ff, 0xFF, sizeof(ff));
            bes_flash_respond_reg(s, ff, MIN(rx_len, (uint32_t)sizeof(ff)));
        }
        break;
    case FLASH_CMD_SECREG_ERASE:
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "bes.flash: unknown NOR cmd 0x%02x\n", cmd);
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

/* ======================================================================
 * SPI NAND chip model
 * ====================================================================== */

static uint32_t bes_nand_phys_page(BesFlashState *s)
{
    return s->nand_page + s->nand_oob;
}

static uint32_t bes_nand_rows(BesFlashState *s)
{
    return s->nand_blocks * s->nand_ppb;
}

static bool bes_nand_io(BesFlashState *s, uint32_t row, uint8_t *buf,
                        uint32_t len, bool write)
{
    uint64_t off = (uint64_t)row * bes_nand_phys_page(s);

    if (fseeko(s->nand_fp, off, SEEK_SET) != 0) {
        return false;
    }
    if (write) {
        return fwrite(buf, 1, len, s->nand_fp) == len &&
               fflush(s->nand_fp) == 0;
    }
    return fread(buf, 1, len, s->nand_fp) == len;
}

static void bes_nand_feature_reset(BesFlashState *s)
{
    memset(s->nand_feat, 0, sizeof(s->nand_feat));
    /* power-up defaults: all blocks locked (BP bits set) and internal ECC
     * on. Block-protect ranges are not enforced: firmware nandflash_init
     * always clears the protection bits (sub_C221978) before use. */
    s->nand_feat[NAND_FEAT_PROT] = 0x38;
    s->nand_feat[NAND_FEAT_CFG] = NAND_CFG_ECC_EN;
}

static void bes_nand_consume_tx(BesFlashState *s, uint32_t n)
{
    n = MIN(n, s->tx_cnt);
    if (n) {
        memmove(s->txfifo, s->txfifo + n, s->tx_cnt - n);
        s->tx_cnt -= n;
    }
}

/* PROGRAM LOAD streaming: move bytes into the page cache */
static void bes_nand_load_bytes(BesFlashState *s, const uint8_t *data,
                                uint32_t n)
{
    uint32_t phys = bes_nand_phys_page(s);
    uint32_t i;

    n = MIN(n, s->nand_load_left);
    for (i = 0; i < n; i++) {
        if (s->nand_load_col < phys) {
            s->nand_cache[s->nand_load_col] = data[i];
        }
        s->nand_load_col++;
    }
    s->nand_load_left -= n;
    if (s->nand_load_left == 0) {
        s->nand_load_active = false;
    }
}

static void bes_nand_exec(BesFlashState *s, uint8_t cmd, uint32_t addr,
                          bool ext, uint32_t tx_len, uint32_t rx_len)
{
    uint32_t phys = bes_nand_phys_page(s);
    uint8_t *st = &s->nand_feat[NAND_FEAT_STATUS];
    uint32_t row;

    if (s->nand_dpd && cmd != NAND_CMD_RELEASE_DP) {
        /* a chip in deep power-down ignores everything but release */
        if (ext) {
            bes_nand_consume_tx(s, tx_len);
        }
        return;
    }

    switch (cmd) {
    case NAND_CMD_RESET:
        bes_nand_feature_reset(s);
        s->nand_load_active = false;
        break;
    case NAND_CMD_READ_ID: {
        uint8_t id[3] = { s->nand_id >> 16, s->nand_id >> 8, s->nand_id };

        bes_flash_rx_fill_pad(s, id, sizeof(id), rx_len ? rx_len : 3, 0x00);
        break;
    }
    case NAND_CMD_WRITE_ENABLE:
        *st |= NAND_ST_WEL;
        break;
    case NAND_CMD_WRITE_DISABLE:
        *st &= ~NAND_ST_WEL;
        break;
    case NAND_CMD_GET_FEATURE: {
        uint8_t v = s->nand_feat[s->tx_cnt ? s->txfifo[0] : 0];

        bes_flash_rx_fill_pad(s, &v, 1, rx_len ? rx_len : 1, v);
        break;
    }
    case NAND_CMD_SET_FEATURE:
        if (s->tx_cnt >= 2) {
            uint8_t fa = s->txfifo[0];

            if (fa == NAND_FEAT_STATUS) {
                /* status is read-only */
            } else {
                s->nand_feat[fa] = s->txfifo[1];
            }
        }
        break;
    case NAND_CMD_PAGE_READ:
        row = addr & 0xFFFFFF;
        trace_bes_nand_page_read(row);
        if (row >= bes_nand_rows(s) ||
            !bes_nand_io(s, row, s->nand_cache, phys, false)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "bes.nand: page read of row 0x%x failed\n", row);
            memset(s->nand_cache, 0xFF, phys);
        }
        /* internal ECC on a perfect store: never any bit errors */
        *st &= ~(NAND_ST_OIP | 0x70);
        break;
    case NAND_CMD_READ_CACHE:
    case NAND_CMD_FAST_READ_CACHE:
    case NAND_CMD_READ_CACHE_X2:
    case NAND_CMD_READ_CACHE_X4:
    case NAND_CMD_READ_CACHE_DIO:
    case NAND_CMD_READ_CACHE_QIO: {
        uint32_t col = addr & 0xFFFF;

        if (rx_len) {
            bes_flash_rx_fill_pad(s, s->nand_cache + MIN(col, phys),
                                  col < phys ? phys - col : 0, rx_len, 0xFF);
        }
        break;
    }
    case NAND_CMD_PROGRAM_LOAD:
    case NAND_CMD_PROGRAM_LOAD_X4:
    case NAND_CMD_RND_LOAD:
    case NAND_CMD_RND_LOAD_X4: {
        uint32_t n;

        if (cmd == NAND_CMD_PROGRAM_LOAD || cmd == NAND_CMD_PROGRAM_LOAD_X4) {
            /* plain load resets the cache; random load keeps it */
            memset(s->nand_cache, 0xFF, phys);
        }
        s->nand_load_col = addr & 0xFFFF;
        s->nand_load_left = tx_len;
        s->nand_load_active = tx_len != 0;
        n = MIN(s->tx_cnt, tx_len);
        bes_nand_load_bytes(s, s->txfifo, n);
        bes_nand_consume_tx(s, n);
        return;
    }
    case NAND_CMD_PROGRAM_EXEC:
        row = addr & 0xFFFFFF;
        trace_bes_nand_program(row);
        *st &= ~(NAND_ST_OIP | NAND_ST_PFAIL);
        if (!(*st & NAND_ST_WEL)) {
            break;
        }
        if (row < bes_nand_rows(s)) {
            uint8_t page[BES_NAND_MAX_PAGE];
            uint32_t i;

            if (bes_nand_io(s, row, page, phys, false)) {
                for (i = 0; i < phys; i++) {
                    page[i] &= s->nand_cache[i];    /* program: 1 -> 0 only */
                }
                if (!bes_nand_io(s, row, page, phys, true)) {
                    *st |= NAND_ST_PFAIL;
                }
            } else {
                *st |= NAND_ST_PFAIL;
            }
        } else {
            *st |= NAND_ST_PFAIL;
        }
        *st &= ~NAND_ST_WEL;
        break;
    case NAND_CMD_BLOCK_ERASE:
        row = addr & 0xFFFFFF;
        trace_bes_nand_erase(row);
        *st &= ~(NAND_ST_OIP | NAND_ST_EFAIL);
        if (!(*st & NAND_ST_WEL)) {
            break;
        }
        row -= row % s->nand_ppb;
        if (row < bes_nand_rows(s)) {
            uint8_t ff[BES_NAND_MAX_PAGE];
            uint32_t p;

            memset(ff, 0xFF, phys);
            for (p = 0; p < s->nand_ppb; p++) {
                if (!bes_nand_io(s, row + p, ff, phys, true)) {
                    *st |= NAND_ST_EFAIL;
                    break;
                }
            }
        } else {
            *st |= NAND_ST_EFAIL;
        }
        *st &= ~NAND_ST_WEL;
        break;
    case NAND_CMD_DIE_SELECT:
        if (s->tx_cnt && s->txfifo[0] != 0) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "bes.nand: select of absent die %u\n",
                          s->txfifo[0]);
        }
        break;
    case NAND_CMD_DEEP_POWERDOWN:
        s->nand_dpd = true;
        break;
    case NAND_CMD_RELEASE_DP:
        s->nand_dpd = false;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "bes.nand: unknown cmd 0x%02x\n", cmd);
        break;
    }

    /* extended-mode commands own their TX phase (address/value bytes) */
    if (ext) {
        bes_nand_consume_tx(s, tx_len);
    }
}

/* ======================================================================
 * controller
 * ====================================================================== */

static void bes_flash_exec(BesFlashState *s, uint32_t reg000)
{
    uint8_t cmd = R000_CMD(reg000);
    uint32_t addr = R000_ADDR(reg000);
    bool ext = s->regs[FLASH_REG_004 / 4] & R004_NEW_CMD_EN;
    uint32_t tx_len = R004_BLOCK_SIZE(s->regs[FLASH_REG_004 / 4]);
    /* normal mode: BLOCK_SIZE is the RX count for read commands
     * (hal_norflaship_v2.c:202-210, norflash_drv.c:2761-2766);
     * extended mode: RX length comes from REG_000 when RX_EN. In extended
     * mode REG_000[31:8] carries RX_EN|RX_LEN|TX_LINE|RX_LINE
     * (reg_norflaship_v2.h:139-148), not an address. */
    uint32_t rx_len = ext ? ((reg000 & R000_NEW_CMD_RX_EN) ?
                             R000_NEW_CMD_RX_LEN(reg000) : 0) : tx_len;

    trace_bes_flash_cmd(bes_flash_is_nand(s) ? "nand" : "nor", cmd,
                        ext ? 0 : addr, ext, tx_len, rx_len);

    /* a new transaction replaces whatever was left in the RX stream */
    s->rx_cnt = 0;
    s->rx_pos = 0;
    if (bes_flash_is_nand(s)) {
        bes_nand_exec(s, cmd, ext ? 0 : addr, ext, tx_len, rx_len);
    } else {
        bes_nor_exec(s, cmd, addr, tx_len, rx_len);
    }
}

static uint32_t bes_flash_status(BesFlashState *s)
{
    uint32_t rx_left = s->rx_cnt - s->rx_pos;
    uint32_t v = 0;

    /* REG_00C, CTRL_VER>=7 layout (reg_norflaship_v2.h:186-202) */
    if (rx_left == 0) {
        v |= (1 << 3);   /* RXFIFO_EMPTY */
    }
    if (s->tx_cnt == 0) {
        v |= (1 << 1);   /* TXFIFO_EMPTY */
    }
    if (s->tx_cnt >= BES_FLASH_TXFIFO_DEPTH) {
        v |= (1 << 2);   /* TXFIFO_FULL */
    }
    v |= MIN(rx_left, FLASH_RXFIFO_COUNT_MAX) << 4;     /* RXFIFO_COUNT */
    /* TXFIFO_EMPCNT is a 5-bit field (max 31); an empty FIFO must read
     * 31, not (32 & 0x1F)=0 which firmware would take as "FIFO full"
     * (hal_norflaship_v2.c:132-169) */
    v |= (MIN(BES_FLASH_TXFIFO_DEPTH - s->tx_cnt, 31) & 0x1F) << 9;
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
    case FLASH_REG_008:     /* DMA data port */
    case FLASH_REG_010:
        v = bes_flash_rx_pop(s);
        bes_flash_update_dma(s);
        break;
    case FLASH_REG_018:
        v = 0;   /* self-clearing */
        break;
    default:
        if ((offset >> 2) < ARRAY_SIZE(s->regs)) {
            v = s->regs[offset >> 2];
        }
        break;
    }

    trace_bes_flash_reg_read(bes_flash_is_nand(s) ? "nand" : "nor",
                             (uint32_t)offset, (uint32_t)v);
    return v;
}

static void bes_flash_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    BesFlashState *s = BES_FLASH(opaque);

    trace_bes_flash_reg_write(bes_flash_is_nand(s) ? "nand" : "nor",
                              (uint32_t)offset, (uint32_t)value);

    switch (offset) {
    case FLASH_REG_000:
        s->regs[0] = value;
        bes_flash_exec(s, value);
        break;
    case FLASH_REG_008: {
        /* TX FIFO push, byte-wise regardless of access size */
        uint8_t b[4];
        uint32_t n, i = 0;

        for (n = 0; n < size; n++) {
            b[n] = (value >> (n * 8)) & 0xFF;
        }
        /* a streaming NAND PROGRAM LOAD takes bytes straight into the
         * page cache once its queued FIFO contents are consumed */
        if (s->nand_load_active) {
            uint32_t take = MIN(size, s->nand_load_left);

            bes_nand_load_bytes(s, b, take);
            i = take;
        }
        for (; i < size; i++) {
            if (s->tx_cnt < BES_FLASH_TXFIFO_DEPTH) {
                s->txfifo[s->tx_cnt++] = b[i];
            }
        }
        /* a pending NOR page-program completes once the full page arrived */
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
    bes_flash_update_dma(s);
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

/* NAND memory-mapped window: read-from-cache at column = offset */
static uint64_t bes_flash_window_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    BesFlashState *s = BES_FLASH(opaque);
    uint32_t phys = bes_nand_phys_page(s);
    uint64_t v = 0;
    unsigned n;

    for (n = 0; n < size; n++) {
        uint32_t col = offset + n;

        v |= (uint64_t)(col < phys ? s->nand_cache[col] : 0xFF) << (n * 8);
    }
    return v;
}

static void bes_flash_window_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    qemu_log_mask(LOG_GUEST_ERROR,
                  "bes.nand: write to read-only cache window +0x%x\n",
                  (unsigned)offset);
}

static const MemoryRegionOps bes_flash_window_ops = {
    .read = bes_flash_window_read,
    .write = bes_flash_window_write,
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

/* write the NOR storage back to the backing image on exit */
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

/* open (or create erased) the NAND backing file */
static bool bes_nand_open(BesFlashState *s, Error **errp)
{
    uint64_t want;
    int64_t have;

    if (s->nand_page == 0 || s->nand_ppb == 0 || s->nand_blocks == 0 ||
        s->nand_page + s->nand_oob > BES_NAND_MAX_PAGE) {
        error_setg(errp, "bes.flash: invalid NAND geometry %u+%u x %u x %u",
                   s->nand_page, s->nand_oob, s->nand_ppb, s->nand_blocks);
        return false;
    }
    want = (uint64_t)bes_nand_rows(s) * bes_nand_phys_page(s);

    s->nand_fp = fopen(s->nand_image, "r+b");
    if (!s->nand_fp) {
        uint8_t *ff = g_malloc(1024 * 1024);
        uint64_t off;

        s->nand_fp = fopen(s->nand_image, "w+b");
        if (!s->nand_fp) {
            g_free(ff);
            error_setg_errno(errp, errno, "bes.flash: cannot create NAND "
                             "image '%s'", s->nand_image);
            return false;
        }
        memset(ff, 0xFF, 1024 * 1024);
        for (off = 0; off < want; off += 1024 * 1024) {
            size_t n = MIN(1024 * 1024, want - off);

            if (fwrite(ff, 1, n, s->nand_fp) != n) {
                g_free(ff);
                error_setg(errp, "bes.flash: short write creating '%s'",
                           s->nand_image);
                return false;
            }
        }
        g_free(ff);
        fflush(s->nand_fp);
        info_report("bes.flash: created erased NAND image '%s'",
                    s->nand_image);
    }
    if (fseeko(s->nand_fp, 0, SEEK_END) != 0 ||
        (have = ftello(s->nand_fp)) < 0 || (uint64_t)have != want) {
        error_setg(errp, "bes.flash: NAND image '%s' must be %" PRIu64
                   " bytes ((%u + %u) x %u x %u)", s->nand_image, want,
                   s->nand_page, s->nand_oob, s->nand_ppb, s->nand_blocks);
        fclose(s->nand_fp);
        s->nand_fp = NULL;
        return false;
    }
    info_report("bes.flash: NAND id %06x, %u MiB, image '%s'", s->nand_id,
                (uint32_t)(((uint64_t)bes_nand_rows(s) * s->nand_page) >> 20),
                s->nand_image);
    return true;
}

static void bes_flash_realize(DeviceState *dev, Error **errp)
{
    BesFlashState *s = BES_FLASH(dev);
    FILE *fp;

    memory_region_init_io(&s->iomem, OBJECT(dev), &bes_flash_ops, s,
                          "bes.flash", 0x400);
    /* The DMA engine pulls/pushes the REG_008 data port synchronously
     * when this device raises its request from inside a register access
     * (e.g. the REG_000 write that starts a page read), which the default
     * re-entrancy guard would block. */
    s->iomem.disable_reentrancy_guard = true;
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->dma_req);

    if (s->nand_image) {
        if (!bes_nand_open(s, errp)) {
            return;
        }
        memory_region_init_io(&s->window, OBJECT(dev), &bes_flash_window_ops,
                              s, "bes.flash.nand-window", 0x4000000);
        sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->window);
        return;
    }

    if (s->size == 0) {
        error_setg(errp, "bes.flash: size property is required");
        return;
    }

    /* NOR storage backend, erased state = 0xFF. RAMBlock name must be
     * unique across instances. */
    {
        g_autofree char *name =
            g_strdup_printf("bes.flash.storage.%p", (void *)s);

        memory_region_init_ram(&s->storage_mr, OBJECT(dev), name,
                               s->size, &error_fatal);
    }
    s->storage = memory_region_get_ram_ptr(&s->storage_mr);
    memset(s->storage, 0xFF, s->size);

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
    bes_nand_feature_reset(s);
    s->nand_load_active = false;
    s->nand_dpd = false;
    memset(s->nand_cache, 0xFF, sizeof(s->nand_cache));
}

static const Property bes_flash_props[] = {
    DEFINE_PROP_UINT32("size", BesFlashState, size, 0),
    /* NOR die JEDEC ID; default GD25LQ64C (8MB) */
    DEFINE_PROP_UINT32("jedec-id", BesFlashState, jedec_id, 0xC86017),
    DEFINE_PROP_STRING("image", BesFlashState, image),
    DEFINE_PROP_STRING("nand-image", BesFlashState, nand_image),
    DEFINE_PROP_UINT32("nand-id", BesFlashState, nand_id, 0),
    DEFINE_PROP_UINT32("nand-page-size", BesFlashState, nand_page, 2048),
    DEFINE_PROP_UINT32("nand-oob-size", BesFlashState, nand_oob, 64),
    DEFINE_PROP_UINT32("nand-pages-per-block", BesFlashState, nand_ppb, 64),
    DEFINE_PROP_UINT32("nand-blocks", BesFlashState, nand_blocks, 0),
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
