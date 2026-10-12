/*
 * BES AON_CMU (Always-On Clock & Memory Unit) for best1503
 *
 * Register layout: SDK platform/hal/best1503/reg_aoncmu_best1503.h:21-86
 *
 * Implemented semantics (all SDK-grounded):
 *  - MEMSC[4] @0x24..0x30: RAM power switch latch. Write 1 = power-up
 *    request; read back = latch state. Power-up completes instantly,
 *    so MEMSC_STATUS @0x34 aggregates MEMSC[i] bit0.
 *    (hal_memsc.c:63-88)
 *  - set/clear register pairs share one state word:
 *      TOP_CLK_ENABLE/DISABLE    0x04/0x08
 *      RESET_SET/CLR             0x10/0x14
 *      MOD_CLK_ENABLE/DISABLE    0x40/0x44
 *      TOP_CLK1_ENABLE/DISABLE   0xA0/0xA4
 *      GBL_RESET_SET/CLR         0xC8/0xCC (bit 7 GLOBAL_RESETN: chip reset)
 *      ORESETN_SET/CLR           0xD8/0xDC
 *      MANUAL_OCLK_ENABLE/DISABLE 0xE0/0xE4
 *    (reg_aoncmu_best1503.h:23-86 layout; BES set/clr convention)
 *  - RESET_PULSE @0x0C / GBL_RESET_PULSE @0xC4: write-triggered pulse,
 *    self-clearing (read back 0).
 *  - WRITE_UNLOCK @0x20: bit0 = unlock control, bit1 = status
 *    (reg_aoncmu_best1503.h:220-221).
 *  - CHIP_ID @0x00: read-only, 0x1503 | (10<<16) — revision 10 maps to
 *    metal id 6 (firmware hal_get_metal_id switch table); metal id 0-2
 *    hits "1503 A don't support BTC PD" ASSERT in btdrv init.
 *    (no SDK constant; watch for firmware branches on it).
 *  - BOOTMODE @0x38: cold boot = 0 (hal_bootmode.c:32-36 bits are
 *    watchdog/global/rtc/charger reset flags, none set on cold boot).
 *  - MCU_VTOR @0x78: bits [31:7] are the vector table the MCU core boots
 *    from (INITSVTOR); the firmware stores SCB->VTOR there with bits
 *    [6:0] preserved (RAM routine called from startup sub_C0C1B08 via
 *    sub_C0C22B0).  That table's reset vector is the context-restore
 *    routine, which is how the core resumes after PSC power gating
 *    (bes_psc.c).  Reset value 0 = the boot ROM vectors at 0.
 *
 * Everything else: plain RW storage (RAM semantics).
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/bes_aoncmu.h"
#include "system/runstate.h"
#include "target/arm/arm-powerctl.h"

#define AONCMU_NREGS  (0x200 / 4)   /* cover through CAP_CLK_CFG @0x108 */

/* set/clr pair table: {set_off, clr_off, state_idx} */
typedef struct {
    uint16_t set_off;
    uint16_t clr_off;
    uint16_t state_idx;
} BesSetClrPair;

static const BesSetClrPair aoncmu_setclr[] = {
    { 0x04, 0x08, 0 },   /* TOP_CLK_ENABLE/DISABLE */
    { 0x10, 0x14, 1 },   /* RESET_SET/CLR */
    { 0x40, 0x44, 2 },   /* MOD_CLK_ENABLE/DISABLE */
    { 0xA0, 0xA4, 3 },   /* TOP_CLK1_ENABLE/DISABLE */
    { 0xC8, 0xCC, 4 },   /* GBL_RESET_SET/CLR */
    { 0xD8, 0xDC, 5 },   /* ORESETN_SET/CLR */
    { 0xE0, 0xE4, 6 },   /* MANUAL_OCLK_ENABLE/DISABLE */
};

#define AONCMU_OFF_CHIP_ID      0x00
#define AONCMU_OFF_RESET_PULSE  0x0C
#define AONCMU_OFF_MEMSC0       0x24
#define AONCMU_OFF_MEMSC_STATUS 0x34
#define AONCMU_OFF_GBL_RESET_PULSE 0xC4
#define AONCMU_OFF_GBL_RESET_SET 0xC8
#define AONCMU_GLOBAL_RESETN    (1 << 7)    /* reg_aoncmu_best1502p.h */
#define AONCMU_OFF_MCU_VTOR     0x78
#define AONCMU_MCU_VTOR_MASK    0xFFFFFF80
#define AONCMU_MCU_CPUID        0

static void aoncmu_set_mcu_vtor(uint32_t vtor)
{
    Object *cpuobj = OBJECT(arm_get_cpu_by_id(AONCMU_MCU_CPUID));

    if (cpuobj && object_property_find(cpuobj, "init-svtor")) {
        object_property_set_uint(cpuobj, "init-svtor",
                                 vtor & AONCMU_MCU_VTOR_MASK, &error_abort);
    }
}

static const BesSetClrPair *aoncmu_find_pair(hwaddr offset, bool *is_set)
{
    int i;
    for (i = 0; i < ARRAY_SIZE(aoncmu_setclr); i++) {
        if (aoncmu_setclr[i].set_off == offset) {
            *is_set = true;
            return &aoncmu_setclr[i];
        }
        if (aoncmu_setclr[i].clr_off == offset) {
            *is_set = false;
            return &aoncmu_setclr[i];
        }
    }
    return NULL;
}

static uint64_t bes_aoncmu_read(void *opaque, hwaddr offset, unsigned size)
{
    BesAoncmuState *s = BES_AONCMU(opaque);
    uint32_t idx = offset >> 2;
    uint32_t v = 0;

    if (offset == AONCMU_OFF_CHIP_ID) {
        /* CHIP_ID（reg_aoncmu_best1503.h:22,88-94，只读）：
         *   bits[15:0]  CHIP_ID      bits[31:16] REVISION_ID
         * 固件 hal_get_metal_id（sub_C0C0BF0）：rev=(CHIP_ID>>16)&0xF，
         * 映射表 4→1, 8→2, 12→3, 13→4, 10→6, 11→7, 14→8, 15→9。
         * btdrv_hci_set_poweron_reg（sub_C227A10）：metal id ∈{0,1,2}
         * 时走 "1503 A don't support BTC PD" → ASSERT(0)；
         * {3,4,5} 走分页备份路径；≥6 走主路径。量产 Mi Band 10
         * （BES2700iMP）不可能是 A 版工程片，取 rev=10 → metal id 6
         * （主路径最低档）。低 16 位芯片号填 0x1503（平台代号）。 */
        v = 0x1503 | (10u << 16);
    } else if (offset == AONCMU_OFF_MEMSC_STATUS) {
        /* power-up completes instantly: aggregate MEMSC[i] bit0 */
        int i;
        for (i = 0; i < 4; i++) {
            if (s->regs[(AONCMU_OFF_MEMSC0 >> 2) + i] & 1) {
                v |= 1u << i;
            }
        }
    } else if (offset == AONCMU_OFF_RESET_PULSE ||
               offset == AONCMU_OFF_GBL_RESET_PULSE) {
        v = 0;   /* self-clearing pulse */
    } else {
        bool is_set;
        const BesSetClrPair *p = aoncmu_find_pair(offset, &is_set);
        if (p) {
            v = s->setclr_state[p->state_idx];
        } else if (idx < AONCMU_NREGS) {
            v = s->regs[idx];
        }
    }

    qemu_log_mask(LOG_UNIMP, "bes.aoncmu: read  off=0x%03x -> 0x%08x\n",
                  (unsigned)offset, v);
    return v;
}

static void bes_aoncmu_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    BesAoncmuState *s = BES_AONCMU(opaque);
    uint32_t idx = offset >> 2;
    bool is_set;
    const BesSetClrPair *p = aoncmu_find_pair(offset, &is_set);

    qemu_log_mask(LOG_UNIMP, "bes.aoncmu: write off=0x%03x = 0x%08x\n",
                  (unsigned)offset, (unsigned)value);

    if (offset == AONCMU_OFF_CHIP_ID || offset == AONCMU_OFF_MEMSC_STATUS) {
        return;   /* read-only */
    }
    if (offset == AONCMU_OFF_RESET_PULSE ||
        offset == AONCMU_OFF_GBL_RESET_PULSE) {
        return;   /* pulse: consume, no state */
    }
    if (offset == AONCMU_OFF_GBL_RESET_SET && (value & AONCMU_GLOBAL_RESETN)) {
        /* hal_cmu_sys_reboot: reset_set(HAL_CMU_MOD_GLOBAL) */
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    }
    if (p) {
        if (is_set) {
            s->setclr_state[p->state_idx] |= value;
        } else {
            s->setclr_state[p->state_idx] &= ~value;
        }
        return;
    }
    if (idx < AONCMU_NREGS) {
        s->regs[idx] = value;
    }
    if (offset == AONCMU_OFF_MCU_VTOR) {
        aoncmu_set_mcu_vtor(value);
    }
}

static const MemoryRegionOps bes_aoncmu_ops = {
    .read = bes_aoncmu_read,
    .write = bes_aoncmu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void bes_aoncmu_reset(DeviceState *dev)
{
    BesAoncmuState *s = BES_AONCMU(dev);
    /* AON 域在 WDT/软复位下保持（always-on），仅上电清零。固件靠
     * BOOTMODE @0x38 / SW_INFO @0xF8 跨复位传递启动信息
     * （hal_bootmode.c、hal_cmu_save_subsys_metal_id）。
     * 设备创建时内存零初始化即"上电态"；此处仅保留这两个信息
     * 寄存器，其余按冷启动复位。 */
    uint32_t bootmode = s->regs[0x38 >> 2];
    uint32_t sw_info = s->regs[0xF8 >> 2];
    int i;

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->setclr_state, 0, sizeof(s->setclr_state));
    s->regs[0x38 >> 2] = bootmode;
    s->regs[0xF8 >> 2] = sw_info;
    aoncmu_set_mcu_vtor(0);

    /* 冷启动时所有 RAM bank 已供电：MEMSC[i]=1（hal_memsc.c:63-70
     * hal_memsc_aon_lock 等待非零即"已上电"，真机硬件默认上电）。
     * MEMSC_STATUS 由读路径从 MEMSC 聚合，无需单独初始化。 */
    for (i = 0; i < 4; i++) {
        s->regs[(0x24 >> 2) + i] = 1;
    }
}

static void bes_aoncmu_realize(DeviceState *dev, Error **errp)
{
    BesAoncmuState *s = BES_AONCMU(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &bes_aoncmu_ops, s,
                          "bes.aoncmu", 0x200);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void bes_aoncmu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = bes_aoncmu_realize;
    device_class_set_legacy_reset(dc, bes_aoncmu_reset);
}

static const TypeInfo bes_aoncmu_info = {
    .name = TYPE_BES_AONCMU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BesAoncmuState),
    .class_init = bes_aoncmu_class_init,
};

static void bes_aoncmu_register_types(void)
{
    type_register_static(&bes_aoncmu_info);
}

type_init(bes_aoncmu_register_types)
