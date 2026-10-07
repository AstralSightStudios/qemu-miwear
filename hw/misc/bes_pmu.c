/*
 * BES PMU (analog power management unit) for best1503 — ISPI/PL022 model.
 *
 * The PMU is an analog companion chip hanging off the on-chip ISPI bus.
 * The ISPI master is a BES-flavoured ARM PL022 SSP at 0x40009000
 * (SDK plat_addr_map_best1503.h:227 ISPI_BASE; hal_spi.c:103 uses the
 * generic SPI_T/PL022 driver on it; reg_spi.h gives the PL022 layout).
 *
 * Firmware transfer flow (hal_ispi_xfer, firmware sub_C0E43C0):
 *   drain RX (wait SSPSR.RNE==0), clear interrupts (SSPICR=~0),
 *   then while SSPCR1.SSE: write SSPDR when SSPSR.TNF, read SSPDR when
 *   SSPSR.RNE; finally wait !busy (SSPCR1.SSE==0 || SSPSR.BSY==0).
 * One transfer unit = one 32-bit command word (firmware sub_C0E3EC0):
 *   write cmd = (off << 18) | (val16 << 2)
 *   read  cmd = (off << 18) | (1 << 26); result returns as val16 << 2
 *   offset 0 write of 0xA000|(page<<4) selects the register page;
 *   offset 0 read returns the chip id (0x1530, "1503" in bits[15:4]).
 * (SDK hal_analogif_best1503.c with PADDING_CYCLES=2.)
 *
 * Modelled behaviour:
 *   - SSPSR: TFE|TNF always set (TX always accepted), RNE while an RX
 *     word is pending, BSY clear (transfers complete instantly)
 *   - SSPDR write executes one command word; read pops the RX word
 *   - paged 16-bit register file (5 pages x 256)
 *   - GPADC: 0x3010 reads all-channels-valid, 0x3056+ch returns the
 *     per-channel code (conversion completes instantly)
 *   - WDT: writing offset 0x37 (page 0) with WDT_EN|WDT_RESET_EN
 *     (0x180) both set requests an immediate system reset. Real hardware
 *     resets within tens of microseconds (firmware's "pmu wdt reboot
 *     failed!" error path only runs when the reset does NOT happen).
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/cpu.h"
#include "target/arm/cpu.h"
#include "system/address-spaces.h"
#include "hw/misc/bes_pmu.h"
#include "system/runstate.h"

/* PL022 SSPSR bits */
#define SSP_SR_TFE      (1 << 0)    /* TX FIFO empty */
#define SSP_SR_TNF      (1 << 1)    /* TX FIFO not full */
#define SSP_SR_RNE      (1 << 2)    /* RX FIFO not empty */
#define SSP_SR_BSY      (1 << 4)

/* PMU WDT (SDK pmu_best1503.c pmu_wdt_config/start/feed):
 * 0x36 WDT_RESET_TIMER = reset period in ms, writing it also feeds;
 * 0x37 WDT_CFG bit8 WDT_EN + bit7 WDT_RESET_EN arm the watchdog.
 * The reset fires only when the armed timer expires without feeding —
 * arming alone must NOT reset (the rtc_enable path arms a multi-second
 * watchdog during normal boot; an instantaneous reset would loop). */
static void bes_pmu_wdt_expire(void *opaque)
{
    qemu_log_mask(LOG_UNIMP, "bes.pmu: WDT expired, system reset\n");
    qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
}

static void bes_pmu_wdt_reload(BesPmuState *s)
{
    uint16_t cfg = s->regs[0][BES_PMU_REG_WDT_CFG];
    uint16_t period_ms = s->regs[0][BES_PMU_REG_WDT_RESET_TIMER];

    if ((cfg & 0x180) == 0x180 && period_ms) {
        timer_mod(s->wdt_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + period_ms);
    } else {
        timer_del(s->wdt_timer);
    }
    /* 调试插装：shutdown 序列（周期 ≤2ms 的 WDT 武装）起点定位 */
    if ((cfg & 0x180) == 0x180 && period_ms <= 2 && current_cpu) {
        CPUARMState *env = &(ARM_CPU(current_cpu)->env);
        uint32_t sp = env->regs[13];
        uint32_t words[12] = {0};
        int i;

        for (i = 0; i < 12; i++) {
            uint32_t addr = sp + 4 * i;
            if (address_space_read(&address_space_memory, addr,
                                   MEMTXATTRS_UNSPECIFIED,
                                   &words[i], 4) != MEMTX_OK) {
                break;
            }
        }
        qemu_log_mask(LOG_UNIMP,
                      "bes.pmu: WDT arm period=%ums pc=0x%08x lr=0x%08x sp=0x%08x\n",
                      period_ms, env->regs[15], env->regs[14], sp);
        qemu_log_mask(LOG_UNIMP,
                      "  stk: %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x\n",
                      words[0], words[1], words[2], words[3],
                      words[4], words[5], words[6], words[7],
                      words[8], words[9], words[10], words[11]);
    }
}

static void bes_pmu_exec(BesPmuState *s, uint32_t cmd)
{
    unsigned int off = (cmd >> 18) & 0xFF;

    if (cmd & (1u << 26)) {
        /* read command: produce the RX word */
        if (off == BES_PMU_REG_CHIP_ID) {
            s->ispi_rx = BES_PMU_CHIP_ID_VAL << 2;
        } else if (s->cur_page == 0 && off == 0x10) {
            /* GPADC data-valid bitmap（固件 sub_C22EE84：读 0x3010 等
             * 通道位、写 0x3011 清位、随即读通道值）。转换瞬时完成，
             * 全通道恒 valid；0x3011 的写不影响此读值。 */
            s->ispi_rx = 0xFFFFu << 2;
        } else if (s->cur_page == 0 && off >= 0x56 && off <= 0x5D) {
            /* GPADC 通道结果（0x3056+ch，固件 sub_C22EE84） */
            s->ispi_rx = s->gpadc[off - 0x56] << 2;
        } else {
            s->ispi_rx = s->regs[s->cur_page][off] << 2;
        }
        s->ispi_rx_valid = true;
        return;
    }

    /* write command */
    uint16_t val = (cmd >> 2) & 0xFFFF;

    if (off == BES_PMU_REG_CHIP_ID) {
        /* offset 0 write = page select (0xA000|(page<<4)) */
        if ((val & 0xFF0F) == 0xA000) {
            s->cur_page = (val >> 4) & 0xF;
            if (s->cur_page > 4) {
                s->cur_page = 0;
            }
        }
        return;
    }
    s->regs[s->cur_page][off] = val;

    if (off == BES_PMU_REG_WDT_RESET_TIMER ||
        off == BES_PMU_REG_WDT_CFG) {
        bes_pmu_wdt_reload(s);
    }
}

static uint64_t bes_pmu_read(void *opaque, hwaddr offset, unsigned size)
{
    BesPmuState *s = BES_PMU(opaque);
    uint32_t v = 0;

    switch (offset) {
    case 0x00:                      /* SSPCR0 */
        v = s->sspcr0;
        break;
    case 0x04:                      /* SSPCR1 */
        v = s->sspcr1;
        break;
    case 0x08:                      /* SSPDR: pop RX */
        v = s->ispi_rx;
        s->ispi_rx_valid = false;
        break;
    case 0x0C:                      /* SSPSR */
        v = SSP_SR_TFE | SSP_SR_TNF;
        if (s->ispi_rx_valid) {
            v |= SSP_SR_RNE;
        }
        break;
    case 0x10:                      /* SSPCPSR */
        v = s->ssp_misc[0];
        break;
    case 0x14:                      /* SSPIMSC */
        v = s->ssp_misc[1];
        break;
    case 0x18:                      /* SSPRIS: no interrupt sources */
    case 0x1C:                      /* SSPMIS */
        v = 0;
        break;
    case 0x24:                      /* SSPDMACR */
        v = s->ssp_misc[2];
        break;
    case 0x28:                      /* SSPSRCNT */
        v = s->ssp_misc[3];
        break;
    case 0x2C:                      /* SSPDLYCR */
        v = s->ssp_misc[4];
        break;
    case 0x88:                      /* SSPRXCR (BES ext) */
        v = s->ssp_misc[5];
        break;
    default:
        break;
    }
    qemu_log_mask(LOG_UNIMP, "bes.pmu: read  off=0x%03x -> 0x%08x\n",
                  (unsigned)offset, v);
    return v;
}

static void bes_pmu_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    BesPmuState *s = BES_PMU(opaque);

    qemu_log_mask(LOG_UNIMP, "bes.pmu: write off=0x%03x = 0x%08x\n",
                  (unsigned)offset, (unsigned)value);
    switch (offset) {
    case 0x00:
        s->sspcr0 = (uint32_t)value;
        break;
    case 0x04:
        s->sspcr1 = (uint32_t)value;
        break;
    case 0x08:                      /* SSPDR: one command word per unit */
        bes_pmu_exec(s, (uint32_t)value);
        break;
    case 0x10:
        s->ssp_misc[0] = (uint32_t)value;
        break;
    case 0x14:
        s->ssp_misc[1] = (uint32_t)value;
        break;
    case 0x20:                      /* SSPICR: clear interrupts (none) */
        break;
    case 0x24:
        s->ssp_misc[2] = (uint32_t)value;
        break;
    case 0x28:
        s->ssp_misc[3] = (uint32_t)value;
        break;
    case 0x2C:
        s->ssp_misc[4] = (uint32_t)value;
        break;
    case 0x88:
        s->ssp_misc[5] = (uint32_t)value;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps bes_pmu_ops = {
    .read = bes_pmu_read,
    .write = bes_pmu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void bes_pmu_reset(DeviceState *dev)
{
    BesPmuState *s = BES_PMU(dev);
    int i;

    s->sspcr0 = 0;
    s->sspcr1 = 0;
    memset(s->ssp_misc, 0, sizeof(s->ssp_misc));
    s->ispi_rx = 0;
    s->ispi_rx_valid = false;
    s->cur_page = 0;
    memset(s->regs, 0, sizeof(s->regs));
    timer_del(s->wdt_timer);
    /* GPADC 通道默认码（初值待换算校准，目标是 vbat 通道 4
     * 换算后 ≥3.6VµV，见 PWONV sub_C20AEAC 关机阈值 3600000） */
    for (i = 0; i < 8; i++) {
        s->gpadc[i] = 0x800;
    }
}

static void bes_pmu_realize(DeviceState *dev, Error **errp)
{
    BesPmuState *s = BES_PMU(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &bes_pmu_ops, s,
                          "bes.pmu", 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    s->wdt_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, bes_pmu_wdt_expire, s);
}

static void bes_pmu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = bes_pmu_realize;
    device_class_set_legacy_reset(dc, bes_pmu_reset);
}

static const TypeInfo bes_pmu_info = {
    .name = TYPE_BES_PMU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BesPmuState),
    .class_init = bes_pmu_class_init,
};

static void bes_pmu_register_types(void)
{
    type_register_static(&bes_pmu_info);
}

type_init(bes_pmu_register_types)
