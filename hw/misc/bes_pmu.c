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
 * Analog register address (firmware pmu read/write sub_C0E3EDC /
 * sub_C0E3F8C): dev[15:12] | page[11:8] | offset[7:0]. The device number
 * is the ISPI chip select, SSPCR1[10:7] (sub_C0E4820), so every analog
 * block on the bus (PMU, GPADC, ...) has its own register space; a write
 * of 0xA000 | page << 4 to offset 0 selects the page of the current
 * device.
 *
 * Modelled behaviour:
 *   - SSPSR: TFE|TNF always set (TX always accepted), RNE while an RX
 *     word is pending, BSY clear (transfers complete instantly)
 *   - SSPDR write executes one command word; read pops the RX word
 *   - per-device paged 16-bit register files
 *   - PMU (device 3): chip id at offset 0 (property chip-id: 0x1530 on
 *     best1503, CHIPID 0xC0D on best1502p, reg_pmu_best1502p.h); WDT (0x36 period in ms, 0x37
 *     WDT_EN|WDT_RESET_EN, programmed by pmu_shutdown's fallback
 *     sub_C0E7C00) resets the system when it expires unfed
 *   - PMU 0xF7..0xFE (page 0): a self-timed analog sequence. Writing
 *     0xF8 with bits 14 and 3 set starts it; 0xFC bit 11 reports it
 *     finished. p67 (sub_C0E8684) samples 0xFC once after a 10 us delay
 *     and ignores the result; Band 9 n66 (sub_C109F58, called from the
 *     analog init sub_C100880) polls 0xFC until bit 11 is set. Modelled
 *     as finishing as soon as it is started.
 *   - GPADC (PMU page 0, hal_gpadc sub_C22EE84 / sub_C22EF38): 0x50 is the
 *     conversion-done bitmap, 0x51 its clear, 0x56 + ch the channel code.
 *     Conversions complete instantly. The code is produced from the
 *     channel's input voltage with the converter's typical transfer
 *     curve, the one hal_gpadc_adc2volt_calib (sub_C22EB5C) assumes for
 *     an uncalibrated part: 800 mV -> 15968, 1050 mV -> 20816.
 *     Board-level channel inputs are properties (gpadc-chN-mv).
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/bes_pmu.h"
#include "system/runstate.h"
#include "trace.h"

/* PL022 SSPSR bits */
#define SSP_SR_TFE      (1 << 0)    /* TX FIFO empty */
#define SSP_SR_TNF      (1 << 1)    /* TX FIFO not full */
#define SSP_SR_RNE      (1 << 2)    /* RX FIFO not empty */
#define SSP_SR_BSY      (1 << 4)

/* PMU WDT (SDK pmu_best1503.c pmu_wdt_config/start/feed):
 * 0x4D WDT_IRQ_TIMER then 0x36 WDT_RESET_TIMER (ms) run back to back,
 * writing either feeds (p65 configures 30000 + 100);
 * 0x37 WDT_CFG bit8 WDT_EN + bit7 WDT_RESET_EN arm the watchdog.
 * The reset fires only when the armed timer expires without feeding —
 * arming alone must NOT reset (the rtc_enable path arms a multi-second
 * watchdog during normal boot; an instantaneous reset would loop). */
static void bes_pmu_wdt_expire(void *opaque)
{
    trace_bes_pmu_wdt_expired();
    qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
}

static void bes_pmu_wdt_reload(BesPmuState *s)
{
    uint16_t cfg = s->regs[BES_ISPI_DEV_PMU][0][BES_PMU_REG_WDT_CFG];
    /* the IRQ timer runs first (its interrupt is the firmware's last
     * chance to feed), then the reset timer */
    uint32_t period_ms =
        s->regs[BES_ISPI_DEV_PMU][0][BES_PMU_REG_WDT_IRQ_TIMER] +
        s->regs[BES_ISPI_DEV_PMU][0][BES_PMU_REG_WDT_RESET_TIMER];

    if ((cfg & 0x180) == 0x180 && period_ms) {
        timer_mod(s->wdt_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + period_ms);
    } else {
        timer_del(s->wdt_timer);
    }
}

/* GPADC typical transfer curve (hal_gpadc_adc2volt_calib defaults) */
#define GPADC_LV_MV     800
#define GPADC_LV_CODE   15968
#define GPADC_HV_MV     1050
#define GPADC_HV_CODE   20816

static uint16_t bes_gpadc_code(uint32_t mv)
{
    int64_t code = GPADC_LV_CODE +
        ((int64_t)mv - GPADC_LV_MV) * (GPADC_HV_CODE - GPADC_LV_CODE) /
        (GPADC_HV_MV - GPADC_LV_MV);

    return MIN(MAX(code, 0), 0xFFFF);
}

static unsigned bes_pmu_dev(BesPmuState *s)
{
    return extract32(s->sspcr1, 7, 4);
}

static void bes_pmu_exec(BesPmuState *s, uint32_t cmd)
{
    unsigned dev = bes_pmu_dev(s);
    unsigned page = s->cur_page[dev];
    unsigned off = (cmd >> 18) & 0xFF;
    uint16_t val;

    if (cmd & (1u << 26)) {
        /* read command: produce the RX word */
        if (dev == BES_ISPI_DEV_PMU && off == BES_PMU_REG_CHIP_ID) {
            val = s->chip_id;
        } else if (dev == BES_ISPI_DEV_PMU && page == 0 &&
                   off == BES_PMU_REG_SEQ_STATUS) {
            val = s->regs[dev][page][off];
            if ((s->regs[dev][page][BES_PMU_REG_SEQ_CTRL] &
                 BES_PMU_SEQ_START) == BES_PMU_SEQ_START) {
                val |= BES_PMU_SEQ_DONE;
            }
        } else if (dev == BES_ISPI_DEV_PMU && page == 0 && off == 0x50) {
            val = 0xFF;     /* every channel has a finished conversion */
        } else if (dev == BES_ISPI_DEV_PMU && page == 0 && off == 0x5e) {
            val = 0x8000;   /* power status: bit 15 = powered on, bit 2 = 0 */
        } else if (dev == BES_ISPI_DEV_PMU && page == 0 &&
                   off >= 0x56 && off <= 0x5D) {
            val = bes_gpadc_code(s->gpadc_mv[off - 0x56]);
        } else {
            val = s->regs[dev][page][off];
        }
        trace_bes_ispi_read(dev, page, off, val);
        s->ispi_rx = val << 2;
        s->ispi_rx_valid = true;
        return;
    }

    /* write command */
    val = (cmd >> 2) & 0xFFFF;
    trace_bes_ispi_write(dev, page, off, val);
    if (off == 0) {
        /* offset 0 write = page select (0xA000 | page << 4) */
        if ((val & 0xFF0F) == 0xA000) {
            s->cur_page[dev] = (val >> 4) & 0xF;
            if (s->cur_page[dev] >= BES_ISPI_PAGES) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "bes.pmu: dev %u: bad page %u\n", dev,
                              s->cur_page[dev]);
                s->cur_page[dev] = 0;
            }
        }
        return;
    }
    s->regs[dev][page][off] = val;

    if (dev == BES_ISPI_DEV_PMU && page == 0 &&
        (off == BES_PMU_REG_WDT_RESET_TIMER || off == BES_PMU_REG_WDT_CFG ||
         off == BES_PMU_REG_WDT_IRQ_TIMER)) {
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
    return v;
}

static void bes_pmu_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    BesPmuState *s = BES_PMU(opaque);

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
    s->sspcr0 = 0;
    s->sspcr1 = 0;
    memset(s->ssp_misc, 0, sizeof(s->ssp_misc));
    s->ispi_rx = 0;
    s->ispi_rx_valid = false;
    memset(s->cur_page, 0, sizeof(s->cur_page));
    memset(s->regs, 0, sizeof(s->regs));
    /* 0x4F: PMU power-on reason / power key / VBUS presence (bit 0 = on) */
    s->regs[BES_ISPI_DEV_PMU][0][0x4F] = 1;
    timer_del(s->wdt_timer);
}

static void bes_pmu_realize(DeviceState *dev, Error **errp)
{
    BesPmuState *s = BES_PMU(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &bes_pmu_ops, s,
                          "bes.pmu", 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    s->wdt_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, bes_pmu_wdt_expire, s);
}

/* GPADC channel inputs (mV at the converter pin). Board wiring per the
 * firmware's consumers: ch2 shell NTC and ch5 skin NTC (sub_C3DBCFC /
 * sub_C3DBE40), ch3 VBUS (sub_C3DC740), ch4 VBAT through a 1/4.9 divider
 * (sub_C3DC1B8). Defaults: NTCs at 250 mV (25 C on this board's divider),
 * no charger, battery at 4.0 V. */
static const Property bes_pmu_props[] = {
    DEFINE_PROP_UINT32("gpadc-ch0-mv", BesPmuState, gpadc_mv[0], 0),
    DEFINE_PROP_UINT32("gpadc-ch1-mv", BesPmuState, gpadc_mv[1], 0),
    DEFINE_PROP_UINT32("gpadc-ch2-mv", BesPmuState, gpadc_mv[2], 250),
    DEFINE_PROP_UINT32("gpadc-ch3-mv", BesPmuState, gpadc_mv[3], 0),
    DEFINE_PROP_UINT32("gpadc-ch4-mv", BesPmuState, gpadc_mv[4], 816),
    DEFINE_PROP_UINT32("gpadc-ch5-mv", BesPmuState, gpadc_mv[5], 250),
    DEFINE_PROP_UINT32("gpadc-ch6-mv", BesPmuState, gpadc_mv[6], 0),
    DEFINE_PROP_UINT32("gpadc-ch7-mv", BesPmuState, gpadc_mv[7], 0),
    DEFINE_PROP_UINT32("chip-id", BesPmuState, chip_id, 0x1530),
};

static void bes_pmu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = bes_pmu_realize;
    device_class_set_props(dc, bes_pmu_props);
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
