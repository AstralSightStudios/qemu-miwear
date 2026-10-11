/*
 * BES PMU (analog power management unit, ISPI-attached) for best1503
 *
 * This code is licensed under the GPL version 2 or later.
 */

#ifndef HW_MISC_BES_PMU_H
#define HW_MISC_BES_PMU_H

#include "hw/core/sysbus.h"

#define TYPE_BES_PMU "bes.pmu"
OBJECT_DECLARE_SIMPLE_TYPE(BesPmuState, BES_PMU)

/* ISPI device numbers (SSPCR1[10:7] chip select, firmware sub_C0E4820;
 * analog register address = dev << 12 | page << 8 | offset, firmware
 * pmu read/write sub_C0E3EDC / sub_C0E3F8C) */
#define BES_ISPI_DEVS           16
#define BES_ISPI_PAGES          6
/* PMU (incl. GPADC and WDT): pmu_shutdown sub_C0E7FA0 / its WDT fallback
 * sub_C0E7C00 program 0x3036/0x3037, hal_gpadc uses 0x3050/0x3056+ch */
#define BES_ISPI_DEV_PMU        3

/* PMU indirect register offsets (page 0), SDK reg_pmu_best1503.h */
#define BES_PMU_REG_CHIP_ID     0x00    /* read-only chip id field */
#define BES_PMU_REG_WDT_RESET_TIMER 0x36 /* reset period (ms); write = feed */
#define BES_PMU_REG_WDT_CFG     0x37    /* bit8 WDT_EN, bit7 WDT_RESET_EN */

#define BES_PMU_CHIP_ID_VAL     0x1530  /* "1503" in bits[15:4] */
#define BES_PMU_REG_SEQ_CTRL    0xF8    /* bits 14|3 start the 0xF7..0xFE sequence */
#define BES_PMU_REG_SEQ_STATUS  0xFC    /* bit 11 = sequence finished */
#define BES_PMU_SEQ_START       0x4008
#define BES_PMU_SEQ_DONE        0x0800

struct BesPmuState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    uint32_t sspcr0;            /* +0x00 SSPCR0 */
    uint32_t sspcr1;            /* +0x04 SSPCR1 */
    uint32_t ssp_misc[6];       /* +0x10/+0x14/+0x24/+0x28/+0x2C/+0x88 */
    uint32_t ispi_rx;           /* RX word (last read result << 2) */
    bool     ispi_rx_valid;
    uint8_t  cur_page[BES_ISPI_DEVS];   /* selected page per device */
    uint16_t regs[BES_ISPI_DEVS][BES_ISPI_PAGES][256];
    uint32_t gpadc_mv[8];       /* GPADC channel input voltages (mV) */
    QEMUTimer *wdt_timer;       /* PMU watchdog (0x36 ms period) */
};

#endif
