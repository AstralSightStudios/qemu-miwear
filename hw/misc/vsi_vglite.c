/*
 * VeriSilicon VGLite 2D vector GPU (GCNanoUltraV / GC355 family)
 *
 * Host interface, from the VGLite kernel driver (Vivante, MIT licensed,
 * vg_lite_hw.h / vg_lite_kernel.c) and the AP firmware's copy of it
 * (vela_ap.bin: kernel entry sub_C10424C, power-up sub_C1040AE, ISR
 * 0xC224C28 on NVIC 70, identification check sub_C0FDA48):
 *   0x000 clock control (clock gate bit1, soft reset bit12, isolate bit19)
 *   0x004 idle: power-up waits until (idle & 0x0B05) == 0x0B05
 *   0x010 interrupt status, cleared by reading (ISR peek)
 *   0x014 interrupt enable
 *   0x020 chip id, 0x024 revision, 0x028 date, 0x030 customer id,
 *   0x0E8 eco id (driver requires 0x265 / 0x2002 / 0x424 / 0)
 *   0x500 command buffer address, 0x504 size in 8-byte units (starts)
 *
 * Command stream (vg_lite.c): 64-bit aligned commands
 *   END(id)         0x0000000i  buffer done, raises interrupt bit id
 *   SEMAPHORE/STALL 0x1/0x2     inter-module sync
 *   STATES(n, a)    0x3nnnaaaa  n values for state registers a.. follow,
 *                               padded to 8 bytes
 *   DATA(n)         0x4nnnnnnn  n * 8 bytes of payload follow: a
 *                               rectangle {x, y, w, h} (u16) for clear /
 *                               blit, or path data for a path draw
 *   CALL(n)         0x6nnnnnnn  execute n * 8 bytes at the address in the
 *                               second word (uploaded paths), until
 *   RETURN          0x70000000
 *   NOP             0x80000000
 * Execution is synchronous; the GPU is always idle when software looks.
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/vsi_vglite.h"
#include "trace.h"

#define VGL_CLOCK_CONTROL   0x000
#define VGL_IDLE            0x004
#define VGL_INTR_STATUS     0x010
#define VGL_INTR_ENABLE     0x014
#define VGL_CHIP_ID         0x020
#define VGL_CHIP_REV        0x024
#define VGL_CHIP_DATE       0x028
#define VGL_CHIP_CID        0x030
#define VGL_ECO_ID          0x0E8
#define VGL_CMDBUF_ADDR     0x500
#define VGL_CMDBUF_SIZE     0x504
#define VGL_EXT_VIDEO_CTRL  0x51C

#define VGL_CLOCK_SOFT_RESET    (1u << 12)

#define VGL_MAX_CALL_DEPTH  4

/* state registers (offset from 0x0A00) */
#define ST_CONTROL          0x00    /* operation, blend, image mode */
#define ST_COLOR            0x02
#define ST_TARGET_CONFIG    0x10
#define ST_TARGET_ADDR      0x11
#define ST_TARGET_STRIDE    0x12
#define ST_TARGET_SIZE      0x13
#define ST_IMAGE_CONFIG     0x25
#define ST_IMAGE_ADDR       0x29
#define ST_IMAGE_STRIDE     0x2B
#define ST_IMAGE_SIZE       0x2F
#define ST_PATH_CONTROL     0x34

static uint32_t vgl_st(VsiVgliteState *s, unsigned reg)
{
    return s->state[reg];
}

static void vgl_update_irq(VsiVgliteState *s)
{
    qemu_set_irq(s->irq, !!(s->intr_status & s->intr_enable));
}

/* ---- operations ---- */

static void vgl_rect_op(VsiVgliteState *s, const uint8_t *d)
{
    unsigned x = lduw_le_p(d), y = lduw_le_p(d + 2);
    unsigned w = lduw_le_p(d + 4), h = lduw_le_p(d + 6);

    s->ops++;
    trace_vsi_vglite_rect(vgl_st(s, ST_CONTROL), vgl_st(s, ST_TARGET_CONFIG),
                          vgl_st(s, ST_TARGET_ADDR), vgl_st(s, ST_IMAGE_CONFIG),
                          vgl_st(s, ST_IMAGE_ADDR), x, y, w, h);
}

static void vgl_path_op(VsiVgliteState *s, hwaddr data, uint32_t bytes)
{
    s->ops++;
    trace_vsi_vglite_path(vgl_st(s, ST_CONTROL), vgl_st(s, ST_PATH_CONTROL),
                          vgl_st(s, ST_TARGET_ADDR), data, bytes);
}

/* ---- command processor ---- */

static void vgl_state_write(VsiVgliteState *s, uint32_t addr, uint32_t v)
{
    if (addr >= VGL_STATE_BASE && addr < VGL_STATE_BASE + VGL_STATE_COUNT) {
        s->state[addr - VGL_STATE_BASE] = v;
    } else {
        qemu_log_mask(LOG_UNIMP, "vsi.vglite: state 0x%04x = 0x%08x\n",
                      addr, v);
    }
}

/* returns false when an END terminated the stream */
static bool vgl_exec(VsiVgliteState *s, hwaddr addr, uint64_t bytes,
                     int depth)
{
    hwaddr end = addr + bytes;

    while (addr + 8 <= end) {
        uint32_t w0 = address_space_ldl_le(&s->as, addr,
                                           MEMTXATTRS_UNSPECIFIED, NULL);
        uint32_t w1 = address_space_ldl_le(&s->as, addr + 4,
                                           MEMTXATTRS_UNSPECIFIED, NULL);

        switch (w0 >> 28) {
        case 0x0:                               /* END */
            s->intr_status |= 1u << (w0 & 0x1F);
            return false;
        case 0x1:                               /* SEMAPHORE */
        case 0x2:                               /* STALL */
        case 0x8:                               /* NOP */
            addr += 8;
            break;
        case 0x3: {                             /* STATE(S) */
            uint32_t count = (w0 >> 16) & 0xFFF;
            uint32_t reg = w0 & 0xFFFF;
            uint32_t i;

            for (i = 0; i < count; i++) {
                uint32_t v = i == 0 ? w1 :
                    address_space_ldl_le(&s->as, addr + 4 + 4 * i,
                                         MEMTXATTRS_UNSPECIFIED, NULL);
                vgl_state_write(s, reg + i, v);
            }
            addr += ROUND_UP(4 * (count + 1), 8);
            break;
        }
        case 0x4: {                             /* DATA */
            uint32_t n = w0 & 0x0FFFFFFF;

            if (vgl_st(s, ST_PATH_CONTROL)) {
                vgl_path_op(s, addr + 8, n * 8);
            } else if (n >= 1) {
                uint8_t rect[8];

                address_space_read(&s->as, addr + 8, MEMTXATTRS_UNSPECIFIED,
                                   rect, sizeof(rect));
                vgl_rect_op(s, rect);
            }
            addr += 8 + (uint64_t)n * 8;
            break;
        }
        case 0x6:                               /* CALL */
            if (depth >= VGL_MAX_CALL_DEPTH) {
                qemu_log_mask(LOG_GUEST_ERROR, "vsi.vglite: CALL nesting\n");
            } else if (!vgl_exec(s, w1, (uint64_t)(w0 & 0x0FFFFFFF) * 8,
                                 depth + 1)) {
                return false;
            }
            addr += 8;
            break;
        case 0x7:                               /* RETURN */
            return true;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "vsi.vglite: bad command "
                          "0x%08x at 0x%" HWADDR_PRIx "\n", w0, addr);
            addr += 8;
            break;
        }
    }
    return true;
}

static void vgl_submit(VsiVgliteState *s)
{
    trace_vsi_vglite_submit(s->cmdbuf_addr, s->cmdbuf_size);
    vgl_exec(s, s->cmdbuf_addr, (uint64_t)s->cmdbuf_size * 8, 0);
    vgl_update_irq(s);
}

/* ---- registers ---- */

static uint64_t vgl_read(void *opaque, hwaddr offset, unsigned size)
{
    VsiVgliteState *s = VSI_VGLITE(opaque);
    uint32_t v;

    switch (offset) {
    case VGL_CLOCK_CONTROL:
        return s->clock_control;
    case VGL_IDLE:
        return 0x7FFFFFFF;          /* every module idle */
    case VGL_INTR_STATUS:
        v = s->intr_status;
        s->intr_status = 0;
        vgl_update_irq(s);
        return v;
    case VGL_INTR_ENABLE:
        return s->intr_enable;
    case VGL_CHIP_ID:
        return s->chip_id;
    case VGL_CHIP_REV:
        return s->chip_rev;
    case VGL_CHIP_DATE:
        return s->chip_date;
    case VGL_CHIP_CID:
        return s->chip_cid;
    case VGL_ECO_ID:
        return s->eco_id;
    case VGL_CMDBUF_ADDR:
        return s->cmdbuf_addr;
    case VGL_CMDBUF_SIZE:
        return s->cmdbuf_size;
    case VGL_EXT_VIDEO_CTRL ... VGL_EXT_VIDEO_CTRL + 0xC:
        return s->ext_regs[(offset - VGL_EXT_VIDEO_CTRL) / 4];
    default:
        qemu_log_mask(LOG_UNIMP, "vsi.vglite: read +0x%03x\n",
                      (unsigned)offset);
        return 0;
    }
}

static void vgl_write(void *opaque, hwaddr offset, uint64_t value,
                      unsigned size)
{
    VsiVgliteState *s = VSI_VGLITE(opaque);

    switch (offset) {
    case VGL_CLOCK_CONTROL:
        if (value & VGL_CLOCK_SOFT_RESET) {
            memset(s->state, 0, sizeof(s->state));
            s->intr_status = 0;
        }
        s->clock_control = value;
        break;
    case VGL_INTR_ENABLE:
        s->intr_enable = value;
        break;
    case VGL_CMDBUF_ADDR:
        s->cmdbuf_addr = value;
        break;
    case VGL_CMDBUF_SIZE:
        s->cmdbuf_size = value;
        vgl_submit(s);
        break;
    case VGL_EXT_VIDEO_CTRL ... VGL_EXT_VIDEO_CTRL + 0xC:
        s->ext_regs[(offset - VGL_EXT_VIDEO_CTRL) / 4] = value;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "vsi.vglite: write +0x%03x = 0x%08x\n",
                      (unsigned)offset, (uint32_t)value);
        break;
    }
    vgl_update_irq(s);
}

static const MemoryRegionOps vgl_ops = {
    .read = vgl_read,
    .write = vgl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void vgl_reset(DeviceState *dev)
{
    VsiVgliteState *s = VSI_VGLITE(dev);

    s->clock_control = 0;
    s->intr_status = 0;
    s->intr_enable = 0;
    s->cmdbuf_addr = 0;
    s->cmdbuf_size = 0;
    memset(s->ext_regs, 0, sizeof(s->ext_regs));
    memset(s->state, 0, sizeof(s->state));
}

static void vgl_realize(DeviceState *dev, Error **errp)
{
    VsiVgliteState *s = VSI_VGLITE(dev);

    if (!s->downstream) {
        error_setg(errp, "vsi.vglite: 'downstream' link not set");
        return;
    }
    address_space_init(&s->as, s->downstream, "vsi.vglite-as");
}

static void vgl_init(Object *obj)
{
    VsiVgliteState *s = VSI_VGLITE(obj);

    memory_region_init_io(&s->iomem, obj, &vgl_ops, s, "vsi.vglite", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const Property vgl_props[] = {
    DEFINE_PROP_LINK("downstream", VsiVgliteState, downstream,
                     TYPE_MEMORY_REGION, MemoryRegion *),
    DEFINE_PROP_UINT32("chip-id", VsiVgliteState, chip_id, 0x265),
    DEFINE_PROP_UINT32("chip-rev", VsiVgliteState, chip_rev, 0x2002),
    DEFINE_PROP_UINT32("chip-cid", VsiVgliteState, chip_cid, 0x424),
    DEFINE_PROP_UINT32("chip-date", VsiVgliteState, chip_date, 0),
    DEFINE_PROP_UINT32("eco-id", VsiVgliteState, eco_id, 0),
};

static void vgl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = vgl_realize;
    device_class_set_props(dc, vgl_props);
    device_class_set_legacy_reset(dc, vgl_reset);
}

static const TypeInfo vgl_info = {
    .name = TYPE_VSI_VGLITE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(VsiVgliteState),
    .instance_init = vgl_init,
    .class_init = vgl_class_init,
};

static void vgl_register_types(void)
{
    type_register_static(&vgl_info);
}

type_init(vgl_register_types)
