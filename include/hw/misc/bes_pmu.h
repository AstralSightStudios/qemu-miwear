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

/* PMU indirect register offsets (page 0), SDK reg_pmu_best1503.h */
#define BES_PMU_REG_CHIP_ID     0x00    /* read-only chip id field */
#define BES_PMU_REG_WDT_RESET_TIMER 0x36 /* reset period (ms); write = feed */
#define BES_PMU_REG_WDT_CFG     0x37    /* bit8 WDT_EN, bit7 WDT_RESET_EN */

#define BES_PMU_CHIP_ID_VAL     0x1530  /* "1503" in bits[15:4] */

struct BesPmuState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    uint32_t sspcr0;            /* +0x00 SSPCR0 */
    uint32_t sspcr1;            /* +0x04 SSPCR1 */
    uint32_t ssp_misc[6];       /* +0x10/+0x14/+0x24/+0x28/+0x2C/+0x88 */
    uint32_t ispi_rx;           /* RX word (last read result << 2) */
    bool     ispi_rx_valid;
    uint8_t  cur_page;          /* selected register page (0..4) */
    uint16_t regs[5][256];      /* paged 16-bit register file */
    uint16_t gpadc[8];          /* GPADC channel codes (offsets 0x56..0x5D) */
    QEMUTimer *wdt_timer;       /* PMU watchdog (0x36 ms period) */
};

#endif
