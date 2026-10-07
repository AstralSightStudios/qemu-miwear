/*
 * BES DSI controller (best1503)
 *
 * This code is licensed under the GPL version 2 or later.
 */

#ifndef HW_MISC_BES_DSI_H
#define HW_MISC_BES_DSI_H

#include "hw/core/sysbus.h"

#define TYPE_BES_DSI "bes.dsi"
OBJECT_DECLARE_SIMPLE_TYPE(BesDsiState, BES_DSI)

struct BesDsiState {
    SysBusDevice parent_obj;
    MemoryRegion iomem_phy;     /* PHY window @0x40168000 (0x400) */
    MemoryRegion iomem_cmd;     /* command engine window @0x40160000 (0x1000) */

    uint32_t phy_regs[0x400 / 4];
    uint32_t cmd_regs[0x1000 / 4];
};

#endif
