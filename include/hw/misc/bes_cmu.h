/*
 * BES MCU_CMU (Clock & Memory Unit) for best1503
 *
 * This code is licensed under the GPL version 2 or later.
 */

#ifndef HW_MISC_BES_CMU_H
#define HW_MISC_BES_CMU_H

#include "hw/core/sysbus.h"
#include "hw/core/irq.h"

#define TYPE_BES_CMU "bes.cmu"
OBJECT_DECLARE_SIMPLE_TYPE(BesCmuState, BES_CMU)

struct BesCmuState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;         /* register window (0x400) */

    uint32_t regs[0x400 / 4];
    uint32_t setclr_state[16];  /* shared state words of set/clr pairs */
    /* intersys IPC interrupt lines (virtual BT core doorbell/ack):
     *   [0] tx0 -> NVIC 32   [1] tx1 -> NVIC 33
     *   [2] rx0 -> NVIC 34   [3] rx1 -> NVIC 35 */
    qemu_irq intersys_irq[4];
};

#endif
