/*
 * BES MCU_CMU (Clock & Memory Unit) for best1503
 *
 * This code is licensed under the GPL version 2 or later.
 */

#ifndef HW_MISC_BES_CMU_H
#define HW_MISC_BES_CMU_H

#include "hw/core/sysbus.h"

#define TYPE_BES_CMU "bes.cmu"
OBJECT_DECLARE_SIMPLE_TYPE(BesCmuState, BES_CMU)

struct BesCmuState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;         /* register window (0x400) */

    uint32_t regs[0x400 / 4];
    uint32_t setclr_state[16];  /* shared state words of set/clr pairs */
};

#endif
