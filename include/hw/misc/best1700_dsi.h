/*
 * BES best1700 MIPI DSI host (command mode)
 *
 * This code is licensed under the GPL version 2 or later.
 */

#ifndef HW_MISC_BEST1700_DSI_H
#define HW_MISC_BEST1700_DSI_H

#include "hw/core/sysbus.h"

#define TYPE_BEST1700_DSI "best1700.dsi"
OBJECT_DECLARE_SIMPLE_TYPE(Best1700DsiState, BEST1700_DSI)

#define BEST1700_DSI_SIZE   0x1000

struct Best1700DsiState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;

    DeviceState *panel;         /* bes.lcdc that models the panel */
    uint32_t regs[BEST1700_DSI_SIZE / 4];
    uint32_t flags;             /* event flags, bits [23:16] of +0x10 */
    uint8_t payload[256];       /* long packet being assembled */
    unsigned len;
};

#endif
