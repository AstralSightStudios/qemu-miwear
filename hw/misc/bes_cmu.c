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
 *      ISIRQ_SET/CLR                  0xA0/0xA4 (mailbox, see below)
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
 * ISIRQ (MCU <-> BT mailbox), bit layout from BES reg_cmu
 * (best2003 SDK reg_0a0/reg_0a4, matching every best1503 firmware use):
 *    0/1  BT2MCU_DATA(1)_DONE      2/3  MCU2BT_DATA(1)_IND (doorbell)
 *   13/14 BT2MCU_DATA(1)_MSK      15/16 MCU2BT_DATA(1)_MSK
 *   17/18 BT2MCU_DATA(1)_INTR     19/20 MCU2BT_DATA(1)_INTR  (raw)
 *   21-24 the same four, masked -> NVIC 34/35 (RX) and 32/33 (TX done)
 * Firmware use: hal_intersys_open sets both masks and clears bits 0/2;
 * send sets IND; the TX-done handler (bit 23) acks with CLR bit 0; the RX
 * handler (bit 21) masks with CLR bit 13, consumes, then reports done to
 * the BT side and unmasks with SET bit 13. The BT side (bes.btss) drops
 * IND and raises MCU2BT_DATA_INTR when it has taken the data, and drives
 * BT2MCU_DATA_INTR while it has data published for the MCU.
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
#define CMU_OFF_ISIRQ_SET     0xA0
#define CMU_OFF_ISIRQ_CLR     0xA4

#define ISIRQ_IND(ch)         (1u << (2 + (ch)))
#define ISIRQ_DONE_CLR(ch)    (1u << (ch))
#define ISIRQ_B2M_MSK(ch)     (1u << (13 + (ch)))
#define ISIRQ_M2B_MSK(ch)     (1u << (15 + (ch)))
#define ISIRQ_B2M_INTR(ch)    (1u << (17 + (ch)))
#define ISIRQ_M2B_INTR(ch)    (1u << (19 + (ch)))
#define ISIRQ_MANAGED         (0x3u << 2 | 0xFu << 13 | 0xFFu << 17)
#define ISIRQ_MANAGED_IF(off) \
    (((off) == CMU_OFF_ISIRQ_SET) ? ISIRQ_MANAGED : 0)

static uint32_t cmu_isirq_value(BesCmuState *s)
{
    uint32_t v = s->isirq_ind | s->isirq_mask | s->isirq_raw;
    int ch;

    for (ch = 0; ch < 2; ch++) {
        if ((s->isirq_raw & ISIRQ_B2M_INTR(ch)) &&
            (s->isirq_mask & ISIRQ_B2M_MSK(ch))) {
            v |= 1u << (21 + ch);
        }
        if ((s->isirq_raw & ISIRQ_M2B_INTR(ch)) &&
            (s->isirq_mask & ISIRQ_M2B_MSK(ch))) {
            v |= 1u << (23 + ch);
        }
    }
    return v;
}

static void cmu_isirq_update(BesCmuState *s)
{
    uint32_t v = cmu_isirq_value(s);
    int ch;

    for (ch = 0; ch < 2; ch++) {
        qemu_set_irq(s->intersys_irq[ch], (v >> (23 + ch)) & 1);
        qemu_set_irq(s->intersys_irq[2 + ch], (v >> (21 + ch)) & 1);
    }
}

/* BT side took the MCU2BT data of channel ch */
static void cmu_bt_ind_ack(void *opaque, int ch, int level)
{
    BesCmuState *s = BES_CMU(opaque);

    if (level) {
        s->isirq_ind &= ~ISIRQ_IND(ch);
        s->isirq_raw |= ISIRQ_M2B_INTR(ch);
        cmu_isirq_update(s);
    }
}

/* BT side has data published for the MCU on channel ch (level) */
static void cmu_bt2mcu(void *opaque, int ch, int level)
{
    BesCmuState *s = BES_CMU(opaque);

    if (level) {
        s->isirq_raw |= ISIRQ_B2M_INTR(ch);
    } else {
        s->isirq_raw &= ~ISIRQ_B2M_INTR(ch);
    }
    cmu_isirq_update(s);
}

static void cmu_isirq_write(BesCmuState *s, bool set, uint32_t value)
{
    uint32_t newly;
    int ch;

    if (set) {
        newly = value & ~s->isirq_ind & (ISIRQ_IND(0) | ISIRQ_IND(1));
        s->isirq_ind |= value & (ISIRQ_IND(0) | ISIRQ_IND(1));
        s->isirq_mask |= value & (0xFu << 13);
        cmu_isirq_update(s);
        for (ch = 0; ch < 2; ch++) {
            if (newly & ISIRQ_IND(ch)) {
                qemu_irq_pulse(s->bt_doorbell[ch]);
            }
        }
    } else {
        for (ch = 0; ch < 2; ch++) {
            if (value & ISIRQ_DONE_CLR(ch)) {
                s->isirq_raw &= ~ISIRQ_M2B_INTR(ch);
            }
        }
        s->isirq_ind &= ~(value & (ISIRQ_IND(0) | ISIRQ_IND(1)));
        s->isirq_mask &= ~(value & (0xFu << 13));
        cmu_isirq_update(s);
    }
}
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
        /* WAKEUP_CLK_CFG（reg_cmu_best1503.h:562-572）：
         *   bits[7:0]  TIMER_WT26 / bits[11:8] TIMER_WTPLL（配置，回读）
         *   bit12/13   LPU_AUTO_SWITCH26/PLL（配置，回读）
         *   bit14/15   LPU_STATUS_26M/PLL（硬件状态：26M 就绪 / PLL 锁定）
         * 理想硬件策略（同 flash/PSRAM 校准）：振荡器与 PLL 瞬时就绪，
         * 状态位恒报 1。若回读写入值或报 0：
         *  - hal_cmu_lpu_busy（固件 sub_C0E2C74）见到 AUTO_SWITCH=1 而
         *    STATUS=0 会永远判定 LPU 忙，时钟切换状态机整体停摆；
         *  - hal_cmu_lpu_wait_26m_ready / sub_C10AB50 里
         *    "while ((WAKEUP_CLK_CFG & CMU_LPU_STATUS_26M) == 0);"
         *    无超时忙等会永久挂死（实测 pc 恒停在该循环）。 */
        v = (s->regs[idx] & ~0xF000u) | 0xC000u;
    } else if (cmu_is_pulse(offset)) {
        v = 0;   /* self-clearing pulse */
    } else if ((p = cmu_find_pair(offset, &is_set))) {
        v = s->setclr_state[p->state_idx];
        if (offset == CMU_OFF_ISIRQ_SET || offset == CMU_OFF_ISIRQ_CLR) {
            v = (v & ~ISIRQ_MANAGED) | cmu_isirq_value(s);
        }
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
            s->setclr_state[p->state_idx] |= value & ~ISIRQ_MANAGED_IF(offset);
        } else {
            s->setclr_state[p->state_idx] &= ~value;
        }
        if (offset == CMU_OFF_ISIRQ_SET || offset == CMU_OFF_ISIRQ_CLR) {
            cmu_isirq_write(s, is_set, value);
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
    s->isirq_ind = 0;
    s->isirq_mask = 0;
    s->isirq_raw = 0;

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
    qdev_init_gpio_out_named(dev, s->intersys_irq, "intersys-irq", 4);
    qdev_init_gpio_out_named(dev, s->bt_doorbell, "bt-doorbell", 2);
    qdev_init_gpio_in_named(dev, cmu_bt_ind_ack, "bt-ind-ack", 2);
    qdev_init_gpio_in_named(dev, cmu_bt2mcu, "bt2mcu", 2);
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
