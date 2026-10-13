/*
 * BES ISPI (Internal SPI) master for best1700
 *
 * This code is licensed under the GPL version 2 or later.
 */

#ifndef HW_MISC_BES_ISPI_H
#define HW_MISC_BES_ISPI_H

#include "hw/core/sysbus.h"

#define TYPE_BES_ISPI "bes.ispi"
OBJECT_DECLARE_SIMPLE_TYPE(BesIspiState, BES_ISPI)

#define BES_ISPI_PMU_GPIOS  32      /* "pmu-gpio" lines: PMU GPIO 99..130 */

struct BesIspiState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;         /* register window (0x1000) */

    uint32_t regs[0x1000 / 4];  /* plain storage for non-decoded regs */

    /* transaction state (see bes_ispi.c for the provenance) */
    uint32_t tx;                /* TX command word assembly (LE byte stream) */
    int      tx_cnt;            /* bytes collected (0-3) */
    uint32_t rx;                /* RX response shift register */
    int      rx_bits;           /* RX bits not yet popped */

    uint32_t ctrl;              /* +0x04: chip select in bits [10:7] */

    /* ISPI slaves (PMU, analog PHYs): a register file per chip select */
    uint16_t regs_slave[16][0x400];

    uint32_t pmu_gpio_in;       /* PMU GPIO input levels, survive reset */
    bool ac_on;                 /* VBUS at the PMU's AC_ON detector */
    uint32_t pmu_gpio_isr;      /* PMU GPIO interrupt status */
    qemu_irq pmu_irq;           /* the PMU's interrupt line */
};

#endif
