/*
 * BES best1700 clock units: MCU_CMU and AON_CMU
 *
 * This code is licensed under the GPL version 2 or later.
 */

#ifndef HW_MISC_BEST1700_CMU_H
#define HW_MISC_BEST1700_CMU_H

#include "hw/core/sysbus.h"

#define TYPE_BEST1700_CMU       "best1700.cmu"
#define TYPE_BEST1700_AONCMU    "best1700.aoncmu"

typedef struct Best1700CmuInfo Best1700CmuInfo;

OBJECT_DECLARE_TYPE(Best1700CmuState, Best1700CmuClass, BEST1700_CMU)

#define BEST1700_CMU_SIZE       0x1000
#define BEST1700_CMU_MAX_PAIRS  16

struct Best1700CmuState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    uint32_t regs[BEST1700_CMU_SIZE / 4];
    uint32_t pair_state[BEST1700_CMU_MAX_PAIRS];
    uint32_t memsc_taken;       /* bit n: hardware semaphore n is held */
    int64_t timer_offset;       /* free-running counter = ticks + offset */
};

struct Best1700CmuClass {
    SysBusDeviceClass parent_class;

    const Best1700CmuInfo *info;
};

#endif
