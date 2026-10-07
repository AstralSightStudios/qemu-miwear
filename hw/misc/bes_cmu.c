/*
 * BES MCU_CMU (Clock & Memory Unit) for best1503
 *
 * Register layout: SDK platform/hal/best1503/reg_cmu_best1503.h:21-156
 *
 * Implemented semantics (all SDK-grounded):
 *  - MEMSC[4] @0xC0..0xCC: RAM power switch latch. Write 1 = power-up
 *    request; read back = latch state. Power-up completes instantly,
 *    so MEMSC_STATUS @0xD0 aggregates MEMSC[i] bit0.
 *    Cold boot: all banks powered, MEMSC[i] reads 1 — firmware polls
 *    hal_memsc_lock() (hal_memsc.c:31-47) waiting for non-zero.
 *  - set/clear register pairs share one state word (BES set/clr
 *    convention, layout in reg_cmu_best1503.h):
 *      HCLK/PCLK/OCLK ENABLE/DISABLE  0x00/0x04, 0x08/0x0C, 0x10/0x14
 *      HRESET/PRESET/ORESET SET/CLR   0x34/0x38, 0x3C/0x40, 0x44/0x48
 *      SYS_CLK_ENABLE/DISABLE         0x60/0x64
 *      CP2MCU_IRQ_SET/CLR             0x98/0x9C
 *      ISIRQ_SET/CLR                  0xA0/0xA4
 *      MCU2CP_IRQ_SET/CLR             0xB8/0xBC
 *      DSI_CLK_ENABLE/DISABLE         0x100/0x104
 *      MCU2SENS_IRQ_SET/CLR           0x128/0x12C
 *      XCLK_ENABLE/DISABLE            0x140/0x144
 *      XRESET_SET/CLR                 0x150/0x154
 *  - HRESET/PRESET/ORESET_PULSE @0x28/0x2C/0x30 and XRESET_PULSE @0x14C:
 *    write-triggered pulse, self-clearing (read back 0).
 *  - BOOTMODE @0x50: cold boot = 0 (hal_bootmode.c reset-reason flags,
 *    none set on cold boot).
 *
 * Everything else: plain RW storage (RAM semantics).
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/bes_cmu.h"

/* set/clr pair table: {set_off, clr_off, state_idx} */
typedef struct {
    uint16_t set_off;
    uint16_t clr_off;
    uint16_t state_idx;
} BesSetClrPair;

static const BesSetClrPair cmu_setclr[] = {
    { 0x00, 0x04, 0 },    /* HCLK_ENABLE/DISABLE */
    { 0x08, 0x0C, 1 },    /* PCLK_ENABLE/DISABLE */
    { 0x10, 0x14, 2 },    /* OCLK_ENABLE/DISABLE */
    { 0x34, 0x38, 3 },    /* HRESET_SET/CLR */
    { 0x3C, 0x40, 4 },    /* PRESET_SET/CLR */
    { 0x44, 0x48, 5 },    /* ORESET_SET/CLR */
    { 0x60, 0x64, 6 },    /* SYS_CLK_ENABLE/DISABLE */
    { 0x98, 0x9C, 7 },    /* CP2MCU_IRQ_SET/CLR */
    { 0xA0, 0xA4, 8 },    /* ISIRQ_SET/CLR */
    { 0xB8, 0xBC, 9 },    /* MCU2CP_IRQ_SET/CLR */
    { 0x100, 0x104, 10 }, /* DSI_CLK_ENABLE/DISABLE */
    { 0x128, 0x12C, 11 }, /* MCU2SENS_IRQ_SET/CLR */
    { 0x140, 0x144, 12 }, /* XCLK_ENABLE/DISABLE */
    { 0x150, 0x154, 13 }, /* XRESET_SET/CLR */
};

#define CMU_OFF_MEMSC0        0xC0
#define CMU_OFF_MEMSC_STATUS  0xD0

static bool cmu_is_pulse(hwaddr offset)
{
    return offset == 0x28 || offset == 0x2C || offset == 0x30 ||
           offset == 0x14C;
}

static const BesSetClrPair *cmu_find_pair(hwaddr offset, bool *is_set)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(cmu_setclr); i++) {
        if (cmu_setclr[i].set_off == offset) {
            *is_set = true;
            return &cmu_setclr[i];
        }
        if (cmu_setclr[i].clr_off == offset) {
            *is_set = false;
            return &cmu_setclr[i];
        }
    }
    return NULL;
}

static uint64_t bes_cmu_read(void *opaque, hwaddr offset, unsigned size)
{
    BesCmuState *s = BES_CMU(opaque);
    uint32_t idx = offset >> 2;
    uint32_t v = 0;
    bool is_set;
    const BesSetClrPair *p;

    if (offset == CMU_OFF_MEMSC_STATUS) {
        /* power-up completes instantly: aggregate MEMSC[i] bit0 */
        int i;

        for (i = 0; i < 4; i++) {
            if (s->regs[(CMU_OFF_MEMSC0 >> 2) + i] & 1) {
                v |= 1u << i;
            }
        }
    } else if (offset == 0x8C) {
        /* WAKEUP_CLK_CFG: firmware (sub_C0E2C74) tests bits[15:12] as
         * the "active wakeup clock source" STATUS — on real hardware the
         * written value is a configuration request while these status
         * bits report the hardware state (no wakeup source active after
         * cold boot). Reporting the raw written value makes the clock
         * switch state machine believe a wakeup clock is active and it
         * skips the frequency switch forever. */
        v = s->regs[idx] & ~0xF000u;
    } else if (cmu_is_pulse(offset)) {
        v = 0;   /* self-clearing pulse */
    } else if ((p = cmu_find_pair(offset, &is_set))) {
        v = s->setclr_state[p->state_idx];
    } else if (idx < ARRAY_SIZE(s->regs)) {
        v = s->regs[idx];
    }

    qemu_log_mask(LOG_UNIMP, "bes.cmu: read  off=0x%03x -> 0x%08x\n",
                  (unsigned)offset, v);
    return v;
}

static void bes_cmu_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    BesCmuState *s = BES_CMU(opaque);
    uint32_t idx = offset >> 2;
    bool is_set;
    const BesSetClrPair *p;

    qemu_log_mask(LOG_UNIMP, "bes.cmu: write off=0x%03x = 0x%08x\n",
                  (unsigned)offset, (unsigned)value);

    if (offset == CMU_OFF_MEMSC_STATUS) {
        return;   /* read-only */
    }
    if (cmu_is_pulse(offset)) {
        return;   /* pulse: consume, no state */
    }
    if ((p = cmu_find_pair(offset, &is_set))) {
        if (is_set) {
            s->setclr_state[p->state_idx] |= value;
        } else {
            s->setclr_state[p->state_idx] &= ~value;
        }
        return;
    }
    if (idx < ARRAY_SIZE(s->regs)) {
        s->regs[idx] = value;
    }
}

static const MemoryRegionOps bes_cmu_ops = {
    .read = bes_cmu_read,
    .write = bes_cmu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void bes_cmu_reset(DeviceState *dev)
{
    BesCmuState *s = BES_CMU(dev);
    int i;

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->setclr_state, 0, sizeof(s->setclr_state));

    /* cold boot: all RAM banks powered, MEMSC[i] reads 1
     * (hal_memsc.c:31-38 hal_memsc_lock waits for non-zero;
     * real hardware powers the banks by default) */
    for (i = 0; i < 4; i++) {
        s->regs[(CMU_OFF_MEMSC0 >> 2) + i] = 1;
    }
}

static void bes_cmu_realize(DeviceState *dev, Error **errp)
{
    BesCmuState *s = BES_CMU(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &bes_cmu_ops, s,
                          "bes.cmu", 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void bes_cmu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = bes_cmu_realize;
    device_class_set_legacy_reset(dc, bes_cmu_reset);
}

static const TypeInfo bes_cmu_info = {
    .name = TYPE_BES_CMU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BesCmuState),
    .class_init = bes_cmu_class_init,
};

static void bes_cmu_register_types(void)
{
    type_register_static(&bes_cmu_info);
}

type_init(bes_cmu_register_types)
