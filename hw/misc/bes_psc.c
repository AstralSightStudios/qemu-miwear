/*
 * BES PSC (power sequence controller) for best1503 @0x40085000
 *
 * Every PSC register is write-keyed: the upper half of a write must be
 * the 0xCAFE key, the lower half carries the value.  All firmware writes
 * follow this (hal_psc_* sub_C0C1800 / sub_C0C18B8 / sub_C0C1970 /
 * sub_C0E8EF4, and the MCU power-down below); read-modify-write sites
 * read the plain value back.  Writes without the key are dropped.
 *
 * MCU power gating, from the firmware's hal_cmu_lpu_sleep mode 4
 * (sub_C10AB50 -> sub_C10BB78, run from RAM):
 *   it saves r4-r12/lr, PSP, MSPLIM/PSPLIM, PRIMASK/BASEPRI/FAULTMASK/
 *   CONTROL on the stack and the stack pointer in SRAM, then writes
 *     +0x18 = 0xCAFE0000, +0x00 = 0xCAFE0001, +0x10 = 0xCAFE0001
 *   and spins in "b ." until the power is cut.
 *   +0x00 bit0 = MCU_PG_AUTO        (the PSC powers the MCU back up by
 *                                    itself on a wake-up interrupt)
 *   +0x10 bit0 = MCU_POWERDN_START  (power the MCU domain off now)
 * Power-up is a core reset.  The CPU then takes its vectors from the
 * table the firmware programmed into AON_CMU +0x78 (bes_aoncmu.c):
 * SRAM 0x20076400, whose reset vector is the context-restore routine
 * (startup sub_C0C1B08 builds the table and stores the restore entry).
 *
 * Wake-up: while the MCU is off, an interrupt that the NVIC would take
 * (the wake sources the firmware left enabled) powers it back up.  The
 * core is gated, not the NVIC, so the pending interrupt survives and is
 * taken once the firmware has restored its context.
 *
 * +0x80..+0x97 (GPIO interrupt block) is modelled by bes.gpio, which is
 * mapped over this window.
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "exec/cpu-interrupt.h"
#include "hw/core/cpu.h"
#include "hw/misc/bes_psc.h"
#include "target/arm/arm-powerctl.h"
#include "trace.h"

#define PSC_KEY                 0xCAFE
#define PSC_REG_MCU_PG          0x00
#define PSC_MCU_PG_AUTO         (1 << 0)
#define PSC_REG_MCU_POWERDN     0x10
#define PSC_MCU_POWERDN_START   (1 << 0)

#define PSC_MCU_CPUID           0
/* PSC power-up latency is not modelled; poll for wake-ups this often */
#define PSC_WAKE_POLL_NS        (100 * SCALE_US)

static void bes_psc_wake_poll(void *opaque)
{
    BesPscState *s = opaque;
    CPUState *cs = arm_get_cpu_by_id(PSC_MCU_CPUID);

    if (cpu_test_interrupt(cs, CPU_INTERRUPT_HARD) &&
        arm_set_cpu_on_and_reset(PSC_MCU_CPUID) ==
            QEMU_ARM_POWERCTL_RET_SUCCESS) {
        trace_bes_psc_mcu_power(true);
        s->mcu_off = false;
        return;
    }
    /* still asleep, or the power-off has not taken effect yet */
    timer_mod(s->wake_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + PSC_WAKE_POLL_NS);
}

static void bes_psc_mcu_power_down(BesPscState *s)
{
    if (s->mcu_off) {
        return;
    }
    if (!(s->regs[PSC_REG_MCU_PG >> 2] & PSC_MCU_PG_AUTO)) {
        qemu_log_mask(LOG_UNIMP, "bes.psc: MCU power-down without "
                      "MCU_PG_AUTO (no wake-up source modelled)\n");
        return;
    }
    trace_bes_psc_mcu_power(false);
    s->mcu_off = true;
    arm_set_cpu_off(PSC_MCU_CPUID);
    timer_mod(s->wake_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + PSC_WAKE_POLL_NS);
}

static uint64_t bes_psc_read(void *opaque, hwaddr offset, unsigned size)
{
    BesPscState *s = opaque;

    return s->regs[offset >> 2];
}

static void bes_psc_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    BesPscState *s = opaque;

    if ((value >> 16) != PSC_KEY) {
        qemu_log_mask(LOG_GUEST_ERROR, "bes.psc: write 0x%08" PRIx64
                      " to +0x%02" HWADDR_PRIx " without the write key\n",
                      value, offset);
        return;
    }
    s->regs[offset >> 2] = value & 0xFFFF;

    if (offset == PSC_REG_MCU_POWERDN && (value & PSC_MCU_POWERDN_START)) {
        bes_psc_mcu_power_down(s);
    }
}

static const MemoryRegionOps bes_psc_ops = {
    .read = bes_psc_read,
    .write = bes_psc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void bes_psc_reset(DeviceState *dev)
{
    BesPscState *s = BES_PSC(dev);

    /* the CPU itself is brought back up by the system reset */
    timer_del(s->wake_timer);
    memset(s->regs, 0, sizeof(s->regs));
    s->mcu_off = false;
}

static void bes_psc_realize(DeviceState *dev, Error **errp)
{
    BesPscState *s = BES_PSC(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &bes_psc_ops, s,
                          "bes.psc", sizeof(s->regs));
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    s->wake_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, bes_psc_wake_poll, s);
}

static void bes_psc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = bes_psc_realize;
    device_class_set_legacy_reset(dc, bes_psc_reset);
}

static const TypeInfo bes_psc_info = {
    .name = TYPE_BES_PSC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BesPscState),
    .class_init = bes_psc_class_init,
};

static void bes_psc_register_types(void)
{
    type_register_static(&bes_psc_info);
}

type_init(bes_psc_register_types)
