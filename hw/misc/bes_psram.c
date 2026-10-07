/*
 * BES PSRAM controller pair (best1503, Mi Band 10 SiP PSRAMUHS)
 *
 * Two register blocks, reverse-engineered from the stock firmware
 * (vela_ap.bin). NOTE: these are the PSRAM PHY and the PSRAM memory
 * controller — earlier guesses called them "DSI"; the firmware's
 * hal_psram_calib_range (sub_C0E56E8) operates on these blocks and the
 * register semantics match the SDK reg_psram_mc_v4.h layout.
 *
 * PHY block @0x40168000:
 *   REG_048  training-mode control, bits[12:10] toggled on retry
 *            (firmware sub_C0E4F0C)
 *   REG_04C  config (firmware writes 0x25 during init)
 *   REG_050  calibration kickoff: bits[4:3] = speed grade
 *            (<=50MHz:3, <=80MHz:2, else 1; firmware sub_C0E4FB4),
 *            bits[22:21] = 0b11; writing it starts the PHY delay-line
 *            calibration (firmware sub_C0E4F74)
 *   REG_054  applied sample delay; the eye-scan (sub_C0E56E8) writes
 *            per-point delays here as bits[19:15]=X, bits[9:5]=Y and
 *            stores the final window center as
 *            ((yc*v4)<<5)&0x3E0 | ((xc*v4)<<14)&0xF8000 | base
 *   REG_058  calibration status/result (read-only from software view):
 *            bit8/bit5 = status flags checked by sub_C0E56E8 (0x120),
 *            bit6    = calibration done (sub_C0E4F0C),
 *            bits[5:0] = delay-line base value (scan range)
 *   REG_060/064  PHY timing, computed from the link frequency
 *            (firmware sub_C0E4FB4; verified byte-exact against the
 *            emulator log for 48MHz: 0x02d10002 / 0x00270014)
 *   REG_068  PHY channel enable (firmware writes 1)
 *
 * MC block @0x40160000 (SDK reg_psram_mc_v4.h MGR command engine):
 *   REG_004  MGR_CMD trigger:
 *            1 = write data: TX FIFO words -> PSRAM[REG_008 & 0x3FFFFFF]
 *            2 = read data:  PSRAM[REG_008 & 0x3FFFFFF] -> RX FIFO
 *            5 = mode-register write: MR[REG_008] = REG_00C
 *            (firmware sub_C0E5228 uses 5 for MR config, sub_C0E5308
 *            uses 2 to read back the calibration pattern)
 *   REG_008  MGR_ADDR (26-bit PSRAM physical offset)
 *   REG_00C  MGR_LEN (words) / MR value for cmd 5
 *   REG_014  TX FIFO (write-data source)
 *   REG_018  RX FIFO (read-data sink, firmware sub_C0E52EC)
 *   REG_01C  bit0/bit1 = TX/RX FIFO clear (self-clearing)
 *   REG_404  status (reg_psram_mc_v4.h:914-935): bit0 = BUSY,
 *            bit1 = RXFIFO_R_EMPTY, bits[10:7] = engine state (8=done,
 *            firmware sub_C0E51C0), bits[14:11] = WB fill level
 *            (firmware sub_C0E51E0 waits for 0)
 *
 * Emulated behaviour: PHY delay-line calibration completes instantly on
 * REG_050 write (bit6 set, mid-range base code 0x20). MGR commands
 * execute instantly against the attached PSRAM storage; the eye-scan
 * then passes on every point (ideal memory), yielding a maximal valid
 * window — which is what hal_psram_calib_range needs (window > 1).
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/bes_psram.h"

#define PHY_REG_050  0x050      /* calibration kickoff */
#define PHY_REG_058  0x058      /* calibration status/result */

#define PHY_058_CAL_DONE   (1 << 6)
#define PHY_058_CAL_CODE   0x10 /* mid-range delay-line code */

#define CMD_REG_004  0x004      /* MGR_CMD trigger */
#define CMD_REG_008  0x008      /* MGR_ADDR */
#define CMD_REG_00C  0x00C      /* MGR_LEN / MR value */
#define CMD_REG_014  0x014      /* TX FIFO */
#define CMD_REG_018  0x018      /* RX FIFO */
#define CMD_REG_01C  0x01C      /* FIFO clear */
#define CMD_REG_404  0x404      /* engine status */

#define CMD_404_STATE_DONE  (8 << 7)   /* bits[10:7] = 8 */

static uint32_t bes_psram_cmd_status(BesPsramState *s)
{
    uint32_t v = CMD_404_STATE_DONE;   /* BUSY=0, WB fill=0 */

    if (s->mgr_rx_cnt - s->mgr_rx_pos == 0) {
        v |= (1 << 1);   /* RXFIFO_R_EMPTY */
    }
    return v;
}

static void bes_psram_mgr_exec(BesPsramState *s, uint32_t cmd)
{
    uint32_t addr = s->cmd_regs[CMD_REG_008 / 4] & 0x3FFFFFF;
    uint32_t len = s->cmd_regs[CMD_REG_00C / 4] & 0xFF;
    uint8_t *psram = (uint8_t *)(uintptr_t)s->psram;
    uint32_t i;

    switch (cmd & 0xFF) {
    case 1:   /* write data: TX FIFO -> PSRAM */
        if (psram && addr < s->psram_size) {
            for (i = 0; i < len && i < s->mgr_tx_cnt; i++) {
                uint32_t off = addr + 4 * i;

                if (off + 4 <= s->psram_size) {
                    memcpy(psram + off, &s->mgr_txfifo[i], 4);
                }
            }
        }
        s->mgr_tx_cnt = 0;
        break;
    case 2:   /* read data: PSRAM -> RX FIFO */
        s->mgr_rx_cnt = 0;
        s->mgr_rx_pos = 0;
        if (psram) {
            for (i = 0; i < len && i < ARRAY_SIZE(s->mgr_rxfifo); i++) {
                uint32_t off = addr + 4 * i;
                uint32_t w = 0;

                if (off + 4 <= s->psram_size) {
                    memcpy(&w, psram + off, 4);
                }
                s->mgr_rxfifo[s->mgr_rx_cnt++] = w;
            }
        }
        break;
    case 5:   /* mode-register write */
        s->mr[addr & 0x1F] = len & 0xFF;
        break;
    default:
        /* init/reset/refresh commands (32, 255, ...): stateless here */
        break;
    }
}

static uint64_t bes_psram_phy_read(void *opaque, hwaddr offset, unsigned size)
{
    BesPsramState *s = BES_PSRAM(opaque);
    uint64_t v = 0;

    if ((offset >> 2) < ARRAY_SIZE(s->phy_regs)) {
        v = s->phy_regs[offset >> 2];
    }
    qemu_log_mask(LOG_UNIMP, "bes.psram: phy read  off=0x%03x -> 0x%0*llx\n",
                  (unsigned)offset, size * 2, (unsigned long long)v);
    return v;
}

static void bes_psram_phy_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    BesPsramState *s = BES_PSRAM(opaque);

    qemu_log_mask(LOG_UNIMP, "bes.psram: phy write off=0x%03x = 0x%0*llx\n",
                  (unsigned)offset, size * 2, (unsigned long long)value);

    if ((offset >> 2) >= ARRAY_SIZE(s->phy_regs)) {
        return;
    }
    s->phy_regs[offset >> 2] = value;

    if (offset == PHY_REG_050) {
        /* PHY delay-line calibration: on real hardware the write kicks
         * off an automatic sweep that completes after some microseconds
         * with REG_058 bit6 set and the winning code in bits[5:1]
         * (firmware sub_C0E4F74/sub_C0E4F0C/sub_C0E5168). Model it as
         * instant completion with a fixed mid-range code. */
        s->phy_regs[PHY_REG_058 / 4] =
            PHY_058_CAL_DONE | (PHY_058_CAL_CODE << 1);
    }
}

static uint64_t bes_psram_cmd_read(void *opaque, hwaddr offset, unsigned size)
{
    BesPsramState *s = BES_PSRAM(opaque);
    uint64_t v = 0;

    switch (offset) {
    case CMD_REG_404:
        v = bes_psram_cmd_status(s);
        break;
    case CMD_REG_018:
        if (s->mgr_rx_pos < s->mgr_rx_cnt) {
            v = s->mgr_rxfifo[s->mgr_rx_pos++];
        }
        break;
    case CMD_REG_01C:
        v = 0;   /* self-clearing */
        break;
    default:
        if ((offset >> 2) < ARRAY_SIZE(s->cmd_regs)) {
            v = s->cmd_regs[offset >> 2];
        }
        break;
    }
    qemu_log_mask(LOG_UNIMP, "bes.psram: cmd read  off=0x%03x -> 0x%0*llx\n",
                  (unsigned)offset, size * 2, (unsigned long long)v);
    return v;
}

static void bes_psram_cmd_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    BesPsramState *s = BES_PSRAM(opaque);

    qemu_log_mask(LOG_UNIMP, "bes.psram: cmd write off=0x%03x = 0x%0*llx\n",
                  (unsigned)offset, size * 2, (unsigned long long)value);

    if ((offset >> 2) >= ARRAY_SIZE(s->cmd_regs)) {
        return;
    }

    switch (offset) {
    case CMD_REG_004:
        bes_psram_mgr_exec(s, value);
        break;
    case CMD_REG_014:
        if (s->mgr_tx_cnt < ARRAY_SIZE(s->mgr_txfifo)) {
            s->mgr_txfifo[s->mgr_tx_cnt++] = value;
        }
        break;
    case CMD_REG_01C:
        if (value & 1) {
            s->mgr_tx_cnt = 0;
        }
        if (value & 2) {
            s->mgr_rx_cnt = 0;
            s->mgr_rx_pos = 0;
        }
        break;
    default:
        s->cmd_regs[offset >> 2] = value;
        break;
    }
}

static const MemoryRegionOps bes_psram_phy_ops = {
    .read = bes_psram_phy_read,
    .write = bes_psram_phy_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static const MemoryRegionOps bes_psram_cmd_ops = {
    .read = bes_psram_cmd_read,
    .write = bes_psram_cmd_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void bes_psram_realize(DeviceState *dev, Error **errp)
{
    BesPsramState *s = BES_PSRAM(dev);

    memory_region_init_io(&s->iomem_phy, OBJECT(dev), &bes_psram_phy_ops, s,
                          "bes.psram.phy", 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem_phy);
    memory_region_init_io(&s->iomem_cmd, OBJECT(dev), &bes_psram_cmd_ops, s,
                          "bes.psram.cmd", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem_cmd);
}

static void bes_psram_reset(DeviceState *dev)
{
    BesPsramState *s = BES_PSRAM(dev);

    memset(s->phy_regs, 0, sizeof(s->phy_regs));
    memset(s->cmd_regs, 0, sizeof(s->cmd_regs));
    s->mgr_rx_cnt = 0;
    s->mgr_rx_pos = 0;
    s->mgr_tx_cnt = 0;
    memset(s->mr, 0, sizeof(s->mr));
}

static const Property bes_psram_props[] = {
    DEFINE_PROP_UINT64("psram-ptr", BesPsramState, psram, 0),
    DEFINE_PROP_UINT32("psram-size", BesPsramState, psram_size, 0),
};

static void bes_psram_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = bes_psram_realize;
    device_class_set_props(dc, bes_psram_props);
    device_class_set_legacy_reset(dc, bes_psram_reset);
}

static const TypeInfo bes_psram_info = {
    .name = TYPE_BES_PSRAM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BesPsramState),
    .class_init = bes_psram_class_init,
};

static void bes_psram_register_types(void)
{
    type_register_static(&bes_psram_info);
}

type_init(bes_psram_register_types)
