/*
 * BES PSC (power sequence controller) for best1503
 *
 * This code is licensed under the GPL version 2 or later.
 */

#ifndef HW_MISC_BES_PSC_H
#define HW_MISC_BES_PSC_H

#include "hw/core/sysbus.h"

#define TYPE_BES_PSC "bes.psc"
OBJECT_DECLARE_SIMPLE_TYPE(BesPscState, BES_PSC)

struct BesPscState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    QEMUTimer *wake_timer;      /* polls for a wake-up while the MCU is off */

    uint32_t regs[0x100 / 4];
    bool mcu_off;
};

#endif
