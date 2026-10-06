/*
 * BES AON_CMU for best1503
 *
 * This code is licensed under the GPL version 2 or later.
 */

#ifndef HW_MISC_BES_AONCMU_H
#define HW_MISC_BES_AONCMU_H

#include "hw/core/sysbus.h"

#define TYPE_BES_AONCMU "bes.aoncmu"
OBJECT_DECLARE_SIMPLE_TYPE(BesAoncmuState, BES_AONCMU)

struct BesAoncmuState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t regs[0x200 / 4];
    uint32_t setclr_state[7];
};

#endif
