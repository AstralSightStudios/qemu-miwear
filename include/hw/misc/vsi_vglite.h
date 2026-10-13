/*
 * VeriSilicon VGLite 2D vector GPU (GCNanoUltraV / GC355 family)
 *
 * This code is licensed under the GPL version 2 or later.
 */

#ifndef HW_MISC_VSI_VGLITE_H
#define HW_MISC_VSI_VGLITE_H

#include "hw/core/sysbus.h"

#define TYPE_VSI_VGLITE "vsi.vglite"
OBJECT_DECLARE_SIMPLE_TYPE(VsiVgliteState, VSI_VGLITE)

#define VGL_STATE_BASE      0x0A00
#define VGL_STATE_COUNT     0x200

struct VsiVgliteState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    MemoryRegion *downstream;
    AddressSpace as;

    /* identification (properties) */
    uint32_t chip_id;
    uint32_t chip_rev;
    uint32_t chip_cid;
    uint32_t chip_date;
    uint32_t eco_id;
    bool unnormalized;          /* image coordinates in texels */

    uint32_t clock_control;
    uint32_t intr_status;
    uint32_t intr_enable;
    uint32_t cmdbuf_addr;
    uint32_t cmdbuf_size;
    uint32_t ext_regs[4];

    uint32_t state[VGL_STATE_COUNT];    /* 0x0A00.. */
    uint64_t ops;                       /* executed draw operations */
};

#endif
