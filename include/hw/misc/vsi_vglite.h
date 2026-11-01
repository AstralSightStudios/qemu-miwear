/*
 * VeriSilicon VGLite 2D vector GPU (GCNanoUltraV / GC355 family)
 *
 * This code is licensed under the GPL version 2 or later.
 */

#ifndef HW_MISC_VSI_VGLITE_H
#define HW_MISC_VSI_VGLITE_H

#include "hw/core/sysbus.h"
#include "qemu/thread.h"

#define TYPE_VSI_VGLITE "vsi.vglite"
OBJECT_DECLARE_SIMPLE_TYPE(VsiVgliteState, VSI_VGLITE)

#define VGL_STATE_BASE      0x0A00
#define VGL_STATE_COUNT     0x200
#define VGL_TEXEL_LUTS      4

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

    /* per-submit counters for the vsi_vglite_stats trace event */
    uint32_t stat_paths;
    uint32_t stat_rects;
    uint64_t stat_edges;
    uint64_t stat_pixels;

    /* scratch kept across draw operations */
    GArray *edges;                      /* flattened path edges */
    void *scratch;                      /* rasterizer work arrays */
    size_t scratch_size;
    uint8_t *path_buf;                  /* path data copy */
    uint32_t path_buf_size;
    uint64_t *texel_lut[VGL_TEXEL_LUTS];  /* decoded 16-bit pixels */
    uint64_t texel_lut_key[VGL_TEXEL_LUTS];
    uint64_t texel_lut_op[VGL_TEXEL_LUTS];  /* last operation using it */
    uint64_t lut_op;
    unsigned texel_lut_next;
    uint64_t clut_lanes[256];           /* image CLUT, decoded */
    uint32_t clut_lanes_src[256];       /* ... from these entries */
    bool clut_lanes_premul;
    bool clut_lanes_valid;

    /* host threads shading the rows of an operation ("render-threads") */
    uint32_t render_threads;            /* 0: automatic */
    unsigned nthreads;
    struct VglWorker *workers;          /* nthreads - 1, started on demand */
    struct VglJob *job;
    QemuSemaphore job_done;
    void *band_scratch;                 /* the vCPU thread's band scratch */
    size_t band_scratch_size;

    /* MIRAGE_VGL_DUMP: submits written out for offline replay */
    char *dump_dir;
    uint32_t dump_seq;
    uint32_t dump_max;
    GHashTable *dump_pages;             /* page address -> contents before */
};

#endif
