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
        if (offset == 0xA0) {
            /* ISIRQ_SET bit3：BT 核活动指示。bt_drv_reg_op 各 API
             * （固件 sub_C2316CC/DC 封装）的等待循环
             * "do { if (bt_rsp_flag) break; ...超时→crash_dump+ASSERT(0)...
             *      } while (ISIRQ_SET & 8);"
             * bit3=1 时无限等待 BT 核响应；bit3=0 走"BT 不在线，
             * 跳过等待"容错路径。本模拟器不实现 BT 协处理器
             * （DESIGN.md：只关注 AP 核），真机上 BT 核从未启动时
             * 该位同样为 0，故 bit3 恒报 0，其余位正常回读。
             * 实测：bit3=1 时 nsh_main 在 bt_drv_reg_op_data_bakeup_init
             * 等 BT 响应 3ms×10 超时 → ASSERT(0) → board_reset(2)。 */
            v &= ~8u;
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
            s->setclr_state[p->state_idx] |= value;
            if (offset == 0xA0) {
                /* ISIRQ_SET：intersys IPC 门铃（BT 核虚拟化）。
                 * AP 侧 hal_intersys_tx 把消息挂上 SRAM 链表后写
                 * bit2(ch0)/bit3(ch1) 叩铃；真机上 BT 核取走消息后
                 * 置 TX-done 挂起 bit23/24 并触发 AP 侧 INTERSYS_TX
                 * 中断（NVIC 32/33）。虚拟 BT 核瞬时取走：
                 * 置挂起 + 拉中断线。AP 的 hal_intersys_tx_irq_handler
                 * 再写 ISIRQ_CLR bit0/1 确认（下方映射清除）。
                 * 实测依据：handler sub_C231034 检测 0xA0&0x800000、
                 * 清 0xA4=1/2；TX 完成回调唤醒等待任务。
                 * 注意：bit3 只虚拟 ch0 —— 本固件 bt_drv_reg_op 把
                 * bit3 用作"等待 BT"自我标记（sub_C2316CC 写 8），
                 * 若对 bit3 也虚拟 TX-done 会产生幽灵 IRQ33 挂起；
                 * 且固件 intersys 只注册了通道 0（sub_C2293CC a1=0）。 */
                if (value & 4) {
                    s->setclr_state[p->state_idx] |= 0x800000u;
                    qemu_irq_raise(s->intersys_irq[0]);
                }
            }
        } else {
            s->setclr_state[p->state_idx] &= ~value;
            if (offset == 0xA4) {
                /* ISIRQ_CLR：handler 确认。bit0/1 -> TX 挂起 23/24；
                 * bit13/14 -> RX 挂起 21/22（RX 方向预留） */
                if (value & 1) {
                    s->setclr_state[p->state_idx] &= ~0x800000u;
                    qemu_irq_lower(s->intersys_irq[0]);
                }
                if (value & 2) {
                    s->setclr_state[p->state_idx] &= ~0x1000000u;
                    qemu_irq_lower(s->intersys_irq[1]);
                }
                if (value & 0x2000) {
                    s->setclr_state[p->state_idx] &= ~0x200000u;
                    qemu_irq_lower(s->intersys_irq[2]);
                }
                if (value & 0x4000) {
                    s->setclr_state[p->state_idx] &= ~0x400000u;
                    qemu_irq_lower(s->intersys_irq[3]);
                }
            }
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
    qdev_init_gpio_out_named(dev, s->intersys_irq, "intersys-irq", 4);
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
