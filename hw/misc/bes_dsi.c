/*
 * BES DSI controller (best1503, Mi Band 10 AMOLED path)
 *
 * Two register blocks, both reverse-engineered from the stock firmware
 * (vela_ap.bin) — the SDK reg_dsi.h layout does NOT match this chip:
 *
 * PHY block @0x40168000:
 *   REG_048  training-mode control, bits[12:10] toggled between 0x400
 *            and 0x800 on calibration-retry (firmware sub_C0E4F0C)
 *   REG_04C  config (firmware writes 0x25 during init)
 *   REG_050  calibration kickoff: bits[4:3] = speed grade
 *            (<=50MHz:3, <=80MHz:2, else 1; firmware sub_C0E4FB4),
 *            bits[22:21] = 0b11; writing it starts the PHY delay-line
 *            calibration (firmware sub_C0E4F74)
 *   REG_054  applied sample delay, written by software as
 *            calib_code * 33825 (firmware sub_C0E5168)
 *   REG_058  calibration status/result (read-only from software view):
 *            bit6    = calibration done
 *            bits[5:1] = 5-bit delay-line calibration code
 *   REG_060/064  PHY timing, computed from the link frequency
 *            (firmware sub_C0E4FB4; verified byte-exact against the
 *            emulator log for 48MHz: 0x02d10002 / 0x00270014)
 *   REG_068  PHY channel enable (firmware writes 1)
 *
 * Command engine @0x40160000:
 *   REG_004  command trigger (firmware sub_C0E5228 writes 5 to send a
 *            panel command; init sequence also uses 32 and 255)
 *   REG_008/00C  command parameters
 *   REG_014  command payload FIFO, word-wise (firmware sub_C0E520C)
 *   REG_01C  write 3 = queue flush between commands (firmware
 *            sub_C0E51F4)
 *   REG_0C4  init config (firmware sub_C0E5914)
 *   REG_404  status: bit0 = busy (polled until 0, firmware
 *            sub_C0E51D0), bit1 = read-reply pending (polled until 0,
 *            firmware sub_C0E5308 via sub_C0E51B0), bits[10:7] =
 *            engine state, 8 = done (polled after each command,
 *            firmware sub_C0E5228 via sub_C0E51C0)
 *
 * Emulated behaviour: PHY calibration completes instantly on REG_050
 * write with a fixed mid-range code (the real code depends on board
 * trace delays and cannot be derived from documentation; any in-range
 * value is valid). Command engine completes instantly: REG_404 reads
 * always report idle-done (bit0=0, state=8).
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/bes_dsi.h"

#define PHY_REG_050  0x050      /* calibration kickoff */
#define PHY_REG_058  0x058      /* calibration status/result */

#define PHY_058_CAL_DONE   (1 << 6)
#define PHY_058_CAL_CODE   0x10 /* mid-range delay-line code */

#define CMD_REG_404  0x404      /* engine status */
/* REG_404 layout (firmware): bit0 = busy (polled until 0, sub_C0E51D0),
 * bit1 = BTA/read reply pending (polled until 0, sub_C0E5308 via
 * sub_C0E51B0), bits[10:7] = engine state, 8 = command done
 * (polled after each command, sub_C0E5228 via sub_C0E51C0) */
#define CMD_404_STATE_DONE  (8 << 7)

static uint64_t bes_dsi_phy_read(void *opaque, hwaddr offset, unsigned size)
{
    BesDsiState *s = BES_DSI(opaque);
    uint64_t v = 0;

    if ((offset >> 2) < ARRAY_SIZE(s->phy_regs)) {
        v = s->phy_regs[offset >> 2];
    }
    qemu_log_mask(LOG_UNIMP, "bes.dsi: phy read  off=0x%03x -> 0x%0*llx\n",
                  (unsigned)offset, size * 2, (unsigned long long)v);
    return v;
}

static void bes_dsi_phy_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    BesDsiState *s = BES_DSI(opaque);

    qemu_log_mask(LOG_UNIMP, "bes.dsi: phy write off=0x%03x = 0x%0*llx\n",
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

static uint64_t bes_dsi_cmd_read(void *opaque, hwaddr offset, unsigned size)
{
    BesDsiState *s = BES_DSI(opaque);
    uint64_t v = 0;

    if ((offset >> 2) < ARRAY_SIZE(s->cmd_regs)) {
        v = s->cmd_regs[offset >> 2];
    }
    qemu_log_mask(LOG_UNIMP, "bes.dsi: cmd read  off=0x%03x -> 0x%0*llx\n",
                  (unsigned)offset, size * 2, (unsigned long long)v);
    return v;
}

static void bes_dsi_cmd_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    BesDsiState *s = BES_DSI(opaque);

    qemu_log_mask(LOG_UNIMP, "bes.dsi: cmd write off=0x%03x = 0x%0*llx\n",
                  (unsigned)offset, size * 2, (unsigned long long)value);

    if ((offset >> 2) >= ARRAY_SIZE(s->cmd_regs)) {
        return;
    }
    s->cmd_regs[offset >> 2] = value;
    /* every command completes instantly; the status read-back path
     * (REG_404) always reports idle-done */
}

static const MemoryRegionOps bes_dsi_phy_ops = {
    .read = bes_dsi_phy_read,
    .write = bes_dsi_phy_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static const MemoryRegionOps bes_dsi_cmd_ops = {
    .read = bes_dsi_cmd_read,
    .write = bes_dsi_cmd_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void bes_dsi_realize(DeviceState *dev, Error **errp)
{
    BesDsiState *s = BES_DSI(dev);

    memory_region_init_io(&s->iomem_phy, OBJECT(dev), &bes_dsi_phy_ops, s,
                          "bes.dsi.phy", 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem_phy);
    memory_region_init_io(&s->iomem_cmd, OBJECT(dev), &bes_dsi_cmd_ops, s,
                          "bes.dsi.cmd", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem_cmd);
}

static void bes_dsi_reset(DeviceState *dev)
{
    BesDsiState *s = BES_DSI(dev);

    memset(s->phy_regs, 0, sizeof(s->phy_regs));
    memset(s->cmd_regs, 0, sizeof(s->cmd_regs));
    /* engine comes up idle */
    s->cmd_regs[CMD_REG_404 / 4] = CMD_404_STATE_DONE;
}

static void bes_dsi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = bes_dsi_realize;
    device_class_set_legacy_reset(dc, bes_dsi_reset);
}

static const TypeInfo bes_dsi_info = {
    .name = TYPE_BES_DSI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BesDsiState),
    .class_init = bes_dsi_class_init,
};

static void bes_dsi_register_types(void)
{
    type_register_static(&bes_dsi_info);
}

type_init(bes_dsi_register_types)
