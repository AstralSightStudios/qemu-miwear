/*
 * BES LCDC (LCD controller) for best1503 @ 0x40200000.
 *
 * SDK plat_addr_map_best1503.h:283 LCDC_BASE=0x40200000; register names
 * from reg_lcdc_v1.h (spu_irq_ena_tb=+0x1C0, spu_irq_isr=+0x1C4,
 * spu_irq_rsr=+0x1C8).
 *
 * Firmware DBI/SPI command path (firmware sub_C232578):
 *   clear ISR bit18, write cmd to +0x284, pack data words at +0x288..,
 *   start the transfer via +0x280, then poll ISR bit18 (tx done) with a
 *   ~2.5 s tick timeout (sub_C2323E0). Without the done bit the RM690B0
 *   panel init times out and retries forever.
 *
 * Modelled behaviour:
 *   - plain 32-bit register file;
 *   - a write to +0x280 (transfer trigger) completes the transaction
 *     instantly: ISR bit18 (0x40000) sets; command/data words are
 *     logged for later panel-model work.
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/misc/bes_lcdc.h"

#define LCDC_REG_IRQ_ENA    (0x1C0 / 4)
#define LCDC_REG_IRQ_ISR    (0x1C4 / 4)
#define LCDC_REG_IRQ_RSR    (0x1C8 / 4)
#define LCDC_REG_SPU_CTRL   (0x280 / 4)
#define LCDC_REG_SPU_CMD    (0x284 / 4)
#define LCDC_REG_SPU_DATA0  (0x288 / 4)

#define LCDC_ISR_TX_DONE    (1 << 18)

#define LCDC_REG_COUNT      (0x1000 / 4)

struct BesLcdcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t regs[LCDC_REG_COUNT];
};

static void bes_lcdc_update_irq(BesLcdcState *s)
{
    qemu_set_irq(s->irq,
                 !!(s->regs[LCDC_REG_IRQ_ISR] & s->regs[LCDC_REG_IRQ_ENA]));
}

static uint64_t bes_lcdc_read(void *opaque, hwaddr offset, unsigned size)
{
    BesLcdcState *s = opaque;
    uint32_t idx = offset / 4;
    uint64_t v;

    if (idx < LCDC_REG_COUNT) {
        v = s->regs[idx];
    } else {
        v = 0;
        qemu_log_mask(LOG_UNIMP, "bes.lcdc: read off=0x%03x\n", (int)offset);
    }
    return v;
}

static void bes_lcdc_write(void *opaque, hwaddr offset, uint64_t val,
                           unsigned size)
{
    BesLcdcState *s = opaque;
    uint32_t idx = offset / 4;

    if (idx >= LCDC_REG_COUNT) {
        qemu_log_mask(LOG_UNIMP, "bes.lcdc: write off=0x%03x = 0x%08x\n",
                      (int)offset, (unsigned)val);
        return;
    }
    s->regs[idx] = val;

    if (idx == LCDC_REG_SPU_CTRL && (val & 3)) {
        /* transfer trigger: complete instantly, raise tx-done */
        s->regs[LCDC_REG_IRQ_ISR] |= LCDC_ISR_TX_DONE;
        qemu_log_mask(LOG_UNIMP,
                      "bes.lcdc: spu xfer cmd=0x%08x data0=0x%08x\n",
                      s->regs[LCDC_REG_SPU_CMD], s->regs[LCDC_REG_SPU_DATA0]);
    }
    bes_lcdc_update_irq(s);
}

static const MemoryRegionOps bes_lcdc_ops = {
    .read = bes_lcdc_read,
    .write = bes_lcdc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void bes_lcdc_reset(DeviceState *dev)
{
    BesLcdcState *s = BES_LCDC(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static void bes_lcdc_init(Object *obj)
{
    BesLcdcState *s = BES_LCDC(obj);

    memory_region_init_io(&s->iomem, obj, &bes_lcdc_ops, s, "bes.lcdc",
                          0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static void bes_lcdc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, bes_lcdc_reset);
}

static const TypeInfo bes_lcdc_info = {
    .name = TYPE_BES_LCDC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BesLcdcState),
    .instance_init = bes_lcdc_init,
    .class_init = bes_lcdc_class_init,
};

static void bes_lcdc_register_types(void)
{
    type_register_static(&bes_lcdc_info);
}

type_init(bes_lcdc_register_types)
