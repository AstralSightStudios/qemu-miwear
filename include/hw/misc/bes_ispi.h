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

struct BesIspiState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;         /* register window (0x1000) */

    uint32_t regs[0x1000 / 4];  /* plain storage for non-decoded regs */

    /* transaction state (see bes_ispi.c for the provenance) */
    uint32_t tx;                /* TX command word assembly (LE byte stream) */
    int      tx_cnt;            /* bytes collected (0-3) */
    uint32_t rx;                /* RX response shift register */
    int      rx_bits;           /* RX bits not yet popped */

    /* ISPI slave (PMU) model */
    uint16_t pmu_regs[256];     /* slave register file */
    uint32_t pmu_id_seq;        /* reg0 read sequence: chip id -> metal id */
};

#endif
