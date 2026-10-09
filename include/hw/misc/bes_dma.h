/*
 * BES general purpose DMA controller (best1503 AUDMA/GPDMA)
 *
 * This code is licensed under the GPL version 2 or later.
 */

#ifndef HW_MISC_BES_DMA_H
#define HW_MISC_BES_DMA_H

#include "hw/core/sysbus.h"

#define TYPE_BES_DMA "bes.dma"
OBJECT_DECLARE_SIMPLE_TYPE(BesDmaState, BES_DMA)

#define BES_DMA_CHANNELS    8
#define BES_DMA_REQ_LINES   16

typedef struct BesDmaChan {
    uint32_t src;
    uint32_t dst;
    uint32_t lli;
    uint32_t ctrl;
    uint32_t conf;
    uint32_t ext[5];        /* 2D extension block @0x200 + ch * 0x20 */
} BesDmaChan;

struct BesDmaState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;

    MemoryRegion *downstream;
    AddressSpace downstream_as;

    uint32_t conf;          /* 0x030 */
    uint32_t sync;          /* 0x034 */
    uint8_t tc_raw;
    uint8_t err_raw;
    uint32_t hw_req;        /* request lines driven by peripherals */
    uint32_t soft_req;      /* 0x020-0x02C software requests */
    BesDmaChan chan[BES_DMA_CHANNELS];
    bool running;
    bool rerun;
};

#endif
