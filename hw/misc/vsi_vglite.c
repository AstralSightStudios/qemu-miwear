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
 * Command stream (vg_lite_context.h): 64-bit aligned commands
 *   END(id)         0x0000000i  buffer done, raises interrupt bit id
 *   SEMAPHORE/STALL 0x1/0x2     inter-module sync
 *   STATES(n, a)    0x3nnnaaaa  n values for state registers a.. follow,
 *                               padded to 8 bytes
 *   DATA(n)         0x4nnnnnnn  n * 8 bytes of payload follow: path data
 *                               when the tessellator is enabled
 *                               (0x0A34 bit 24), else a rectangle
 *                               {x, y, w, h} (u16) for clear / blit
 *   CALL(n)         0x6nnnnnnn  execute n * 8 bytes at the address in the
 *                               second word (uploaded paths), until
 *   RETURN          0x70000000
 *   NOP             0x80000000
 * Execution is synchronous; the GPU is always idle when software looks.
 *
 * Drawing state, as programmed by the VGLite 4.x user driver (vg_lite.c,
 * vg_lite_path.c, vg_lite_image.c; the copy bundled with LVGL 9 is the
 * reference) and checked against command streams captured from the
 * firmware (LVGL 9 vg_lite draw unit):
 *   0x0A00 control: [2:0] 1 = rectangle, 2 = tessellated path,
 *          4 = clear rectangle (fill colour, no blending);
 *          [14:12] image mode (0 none: fill colour, 1 normal, 2 multiply
 *          by colour, 3 stencil, 6 recolour); [11:8] blend (1 src-over,
 *          2 dst-over, 3 src-in, 4 dst-in, 5 multiply, 6 screen, 7 darken,
 *          8 lighten, 9 additive, 10 subtract); bit 15 transparent
 *          border; [25:24] paint type (1 linear gradient, 2 radial
 *          gradient, 3 pattern); bit 28 premultiply
 *          the input colour (non-premultiplied target)
 *   0x0A02 colour, ABGR8888
 *   0x0A10 target format ([3:0] format, [5:4] component order),
 *          0x0A11 address, 0x0A12 stride (24-bit formats: stride / 3 * 4),
 *          0x0A13 width | height << 16 (right / bottom clip)
 *   0x0A18-0x0A1A, 0x0A1C-0x0A1E, 0x0A20-0x0A22 image coordinate
 *          generator (float): u = c0 + x0 X + y0 Y, v = c1 + ..,
 *          w = c2 + ..; texel = (u / w * width, v / w * height)
 *   0x0A25 image config: format ([3:0], [5:4] order, [11:9] index / 2222,
 *          [30:29] 24-bit group), [13:12] tiling outside the image
 *          (0 border colour 0x0A27, 1 pad, 2 repeat, 3 reflect), [17:16]
 *          filter, bit 24 premultiply texels; 0x0A29 address, 0x0A2B
 *          stride, 0x0A2F width | height << 16
 *   0x0A04-0x0A09, 0x0A24, 0x0A26, 0x0A28, 0x0A2A, 0x0A2E pattern paint
 *          image (affine coordinate generator)
 *   gradients reuse the paint image as the colour ramp (vg_lite_update_
 *          linear_grad / _radial_grad: texel i holds position
 *          i / (width - 1)) and the generator states as coefficients
 *          (float) of the ramp position g at pixel (x, y):
 *          linear  g = c + x X + y Y   (c 0x0A04, X 0x0A06, Y 0x0A08)
 *          radial  g = c + x X + y Y + sqrt(r), r = rc + x rX + y rY
 *                      + x^2 rXX + y^2 rYY + x y rXY   (rc 0x0A05,
 *                      rX 0x0A07, rY 0x0A09, rXX 0x0A03, rYY 0x0A0A,
 *                      rXY 0x0A0B); the paint image tiling spreads g
 *          outside 0..1
 *   0x0A34 tessellation control: bit 24 enable, [21:20] coordinate format
 *          (s8 / s16 / s32 / fp32), [1:0] quality (anti-aliasing), bit 4
 *          even-odd fill; 0x0A39 / 0x0A3A tessellation window
 *          x | y << 16, w | h << 16; 0x0A3B / 0x0A3C coordinate scale /
 *          bias; 0x0A40-0x0A45 path matrix (float, row major 2x3)
 *   CLUT: 0x0A98 (2), 0x0A9C (4), 0x0AA0 (16), 0x0B00 (256 entries),
 *          ARGB8888
 * Path data: opcode byte padded to the coordinate size, then its
 * coordinates; END 0, CLOSE 1, MOVE 2, LINE 4, QUAD 6, CUBIC 8 (odd =
 * relative). The driver converts arcs and smooth / H / V segments before
 * they reach the hardware.
 *
 * Pixels are composed in premultiplied space and stored premultiplied
 * (LVGL marks GPU-drawn layers as premultiplied).
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qapi/error.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "system/memory.h"
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
#define VGL_MAX_PATH_BYTES  (4 * 1024 * 1024)

/* state registers (offset from 0x0A00) */
#define ST_CONTROL          0x00
#define ST_COLOR            0x02
#define ST_GRAD_RXX         0x03
#define ST_PAINT_C0         0x04    /* .. 0x09: c0 c1 x0 x1 y0 y1 */
#define ST_GRAD_RYY         0x0A
#define ST_GRAD_RXY         0x0B
#define ST_TARGET_CONFIG    0x10
#define ST_TARGET_ADDR      0x11
#define ST_TARGET_STRIDE    0x12
#define ST_TARGET_SIZE      0x13
#define ST_IMAGE_C0         0x18    /* c0 c1 c2 */
#define ST_IMAGE_X0         0x1C    /* x0 x1 x2 */
#define ST_IMAGE_Y0         0x20    /* y0 y1 y2 */
#define ST_PAINT_CONFIG     0x24
#define ST_IMAGE_CONFIG     0x25
#define ST_PAINT_BORDER     0x26
#define ST_IMAGE_BORDER     0x27
#define ST_PAINT_ADDR       0x28
#define ST_IMAGE_ADDR       0x29
#define ST_PAINT_STRIDE     0x2A
#define ST_IMAGE_STRIDE     0x2B
#define ST_PAINT_SIZE       0x2E
#define ST_IMAGE_SIZE       0x2F
#define ST_PATH_CONTROL     0x34
#define ST_TESS_ORIGIN      0x39
#define ST_TESS_SIZE        0x3A
#define ST_PATH_SCALE       0x3B
#define ST_PATH_BIAS        0x3C
#define ST_PATH_MATRIX      0x40    /* .. 0x45 */
#define ST_CLUT2            0x98
#define ST_CLUT4            0x9C
#define ST_CLUT16           0xA0
#define ST_CLUT256          0x100

#define CTRL_OP(c)          ((c) & 7)
#define CTRL_IMAGE_MODE(c)  (((c) >> 12) & 7)
#define CTRL_BLEND(c)       (((c) >> 8) & 0xF)
#define CTRL_PAINT(c)       (((c) >> 24) & 3)
#define CTRL_IN_PREMULT     (1u << 28)
#define CTRL_UNMODELLED     ((1u << 16) | (7u << 17) | (1u << 20))

#define PATH_TESS_ENABLE    (1u << 24)
#define PATH_EVEN_ODD       (1u << 4)

enum { OP_RECT = 1, OP_CLEAR = 4 };
enum { PAINT_COLOR, PAINT_LINEAR, PAINT_RADIAL, PAINT_PATTERN };
enum { IM_NONE = 0, IM_NORMAL = 1, IM_MULTIPLY = 2, IM_STENCIL = 3,
       IM_RECOLOR = 6 };
enum { TILE_COLOR, TILE_PAD, TILE_REPEAT, TILE_REFLECT };
enum { CH_R, CH_G, CH_B, CH_A };

static uint32_t vgl_st(VsiVgliteState *s, unsigned reg)
{
    return s->state[reg];
}

static float vgl_stf(VsiVgliteState *s, unsigned reg)
{
    uint32_t v = s->state[reg];
    float f;

    memcpy(&f, &v, sizeof(f));
    return f;
}

static void vgl_update_irq(VsiVgliteState *s)
{
    qemu_set_irq(s->irq, !!(s->intr_status & s->intr_enable));
}

/* ---- pixel formats ---- */

typedef struct {
    bool valid;
    uint8_t bytes;          /* bytes per pixel; 0 = sub-byte */
    uint8_t bits;           /* bits per pixel for sub-byte formats */
    uint8_t width[4];       /* bits per channel, R G B A */
    uint8_t order[4];       /* channel in each slot, from the LSB */
    bool alpha;             /* the A slot is alpha (else padding) */
    bool lum;               /* L8 */
    bool alpha_only;        /* A8 / A4 */
    bool index;             /* CLUT index */
    bool wide_stride;       /* 24-bit: stride programmed as stride / 3 * 4 */
} VglFormat;

/* component order from the LSB, selected by format bits [5:4] */
static const uint8_t vgl_swizzle[4][4] = {
    { CH_B, CH_G, CH_R, CH_A },
    { CH_A, CH_B, CH_G, CH_R },
    { CH_R, CH_G, CH_B, CH_A },
    { CH_A, CH_R, CH_G, CH_B },
};

static void vgl_fmt_rgba(VglFormat *f, unsigned bytes, unsigned swz,
                         unsigned r, unsigned g, unsigned b, unsigned a,
                         bool alpha)
{
    f->valid = true;
    f->bytes = bytes;
    f->width[CH_R] = r;
    f->width[CH_G] = g;
    f->width[CH_B] = b;
    f->width[CH_A] = a;
    memcpy(f->order, vgl_swizzle[swz & 3], 4);
    f->alpha = alpha;
}

/* render target formats (convert_target_format) */
static VglFormat vgl_target_format(uint32_t cfg)
{
    VglFormat f = { 0 };
    unsigned swz = (cfg >> 4) & 3;

    switch (cfg & 0xF) {
    case 0x0:
        f.valid = f.alpha_only = true;
        f.bytes = 1;
        break;
    case 0x1:
        vgl_fmt_rgba(&f, 2, swz, 5, 6, 5, 0, false);
        break;
    case 0x2:
        vgl_fmt_rgba(&f, 4, swz, 8, 8, 8, 8, false);
        break;
    case 0x3:
        vgl_fmt_rgba(&f, 4, swz, 8, 8, 8, 8, true);
        break;
    case 0x4:
        vgl_fmt_rgba(&f, 2, swz, 4, 4, 4, 4, true);
        break;
    case 0x5:
        vgl_fmt_rgba(&f, 2, swz, 5, 5, 5, 1, true);
        break;
    case 0x6:
        f.valid = f.lum = true;
        f.bytes = 1;
        break;
    case 0x7:
        vgl_fmt_rgba(&f, 1, swz, 2, 2, 2, 2, true);
        break;
    case 0x9:
        vgl_fmt_rgba(&f, 3, swz, 8, 8, 8, 0, false);
        f.wide_stride = true;
        break;
    case 0xA:
        vgl_fmt_rgba(&f, 3, swz, 5, 6, 5, 8, true);
        f.wide_stride = true;
        break;
    }
    return f;
}

/* image formats (convert_source_format) */
static VglFormat vgl_source_format(uint32_t cfg)
{
    VglFormat f = { 0 };
    unsigned swz = (cfg >> 4) & 3;

    switch ((cfg >> 29) & 3) {
    case 1:
        vgl_fmt_rgba(&f, 3, swz, 8, 8, 8, 0, false);
        f.wide_stride = true;
        return f;
    case 2:
        vgl_fmt_rgba(&f, 3, swz, 5, 6, 5, 8, true);
        f.wide_stride = true;
        return f;
    case 3:
        return f;                       /* planar 8565: not modelled */
    }
    switch ((cfg >> 9) & 7) {
    case 1: case 2: case 3: case 4:     /* index 1 / 2 / 4 / 8 */
        f.valid = f.index = true;
        f.bits = 1 << (((cfg >> 9) & 7) - 1);
        return f;
    case 5:
        vgl_fmt_rgba(&f, 1, swz, 2, 2, 2, 2, true);
        return f;
    case 0:
        break;
    default:
        return f;                       /* ETC2 and others: not modelled */
    }
    switch (cfg & 0xF) {
    case 0x0:
        f.valid = f.lum = true;
        f.bytes = 1;
        break;
    case 0x1:
        f.valid = f.alpha_only = true;
        f.bits = 4;
        break;
    case 0x2:
        f.valid = f.alpha_only = true;
        f.bytes = 1;
        break;
    case 0x3:
        vgl_fmt_rgba(&f, 2, swz, 4, 4, 4, 4, true);
        break;
    case 0x4:
        vgl_fmt_rgba(&f, 2, swz, 5, 5, 5, 1, true);
        break;
    case 0x5:
        vgl_fmt_rgba(&f, 2, swz, 5, 6, 5, 0, false);
        break;
    case 0x6:
        vgl_fmt_rgba(&f, 4, swz, 8, 8, 8, 8, false);
        break;
    case 0x7:
        vgl_fmt_rgba(&f, 4, swz, 8, 8, 8, 8, true);
        break;
    }
    return f;
}

static int vgl_expand(uint32_t v, unsigned bits)
{
    uint32_t max = (1u << bits) - 1;

    return bits >= 8 ? (int)(v & 0xFF) : (int)((v * 255 + max / 2) / max);
}

/* raw packed pixel -> RGBA 0..255 as stored */
static void vgl_unpack(const VglFormat *f, uint32_t raw, int c[4])
{
    unsigned i, shift = 0;

    if (f->alpha_only) {
        int a = f->bits == 4 ? vgl_expand(raw, 4) : (int)(raw & 0xFF);

        c[CH_R] = c[CH_G] = c[CH_B] = 255;
        c[CH_A] = a;
        return;
    }
    if (f->lum) {
        c[CH_R] = c[CH_G] = c[CH_B] = raw & 0xFF;
        c[CH_A] = 255;
        return;
    }
    c[CH_A] = 255;
    for (i = 0; i < 4; i++) {
        unsigned ch = f->order[i], w = f->width[ch];

        if (!w) {
            continue;
        }
        if (ch != CH_A || f->alpha) {
            c[ch] = vgl_expand(raw >> shift, w);
        }
        shift += w;
    }
}

static uint32_t vgl_pack(const VglFormat *f, const int c[4])
{
    uint32_t raw = 0;
    unsigned i, shift = 0;

    if (f->alpha_only) {
        return c[CH_A];
    }
    if (f->lum) {
        return (c[CH_R] * 54 + c[CH_G] * 183 + c[CH_B] * 19 + 128) >> 8;
    }
    for (i = 0; i < 4; i++) {
        unsigned ch = f->order[i], w = f->width[ch];
        uint32_t v;

        if (!w) {
            continue;
        }
        v = (ch == CH_A && !f->alpha) ? 255 : (uint32_t)c[ch];
        raw |= ((v * ((1u << w) - 1) + 127) / 255) << shift;
        shift += w;
    }
    return raw;
}

static void vgl_premultiply(int c[4])
{
    c[CH_R] = (c[CH_R] * c[CH_A] + 127) / 255;
    c[CH_G] = (c[CH_G] * c[CH_A] + 127) / 255;
    c[CH_B] = (c[CH_B] * c[CH_A] + 127) / 255;
}

static void vgl_abgr(uint32_t v, int c[4])
{
    c[CH_R] = v & 0xFF;
    c[CH_G] = (v >> 8) & 0xFF;
    c[CH_B] = (v >> 16) & 0xFF;
    c[CH_A] = v >> 24;
}

/* ---- guest memory windows ---- */

typedef struct {
    AddressSpace *as;
    hwaddr addr;
    uint64_t len;
    uint8_t *host;              /* direct mapping, or NULL: slow path */
    bool write;
} VglMem;

static void vgl_mem_open(VglMem *m, AddressSpace *as, hwaddr addr,
                         uint64_t len, bool write)
{
    hwaddr plen = len;

    m->as = as;
    m->addr = addr;
    m->len = len;
    m->write = write;
    m->host = len ? address_space_map(as, addr, &plen, write,
                                      MEMTXATTRS_UNSPECIFIED) : NULL;
    if (m->host && plen < len) {
        address_space_unmap(as, m->host, plen, false, 0);
        m->host = NULL;
    }
}

static void vgl_mem_close(VglMem *m)
{
    if (m->host) {
        address_space_unmap(m->as, m->host, m->len, m->write,
                            m->write ? m->len : 0);
        m->host = NULL;
    }
}

static void vgl_mem_read(VglMem *m, uint64_t off, void *buf, unsigned n)
{
    if (off + n > m->len) {
        memset(buf, 0, n);
    } else if (m->host) {
        memcpy(buf, m->host + off, n);
    } else {
        address_space_read(m->as, m->addr + off, MEMTXATTRS_UNSPECIFIED,
                           buf, n);
    }
}

static void vgl_mem_write(VglMem *m, uint64_t off, const void *buf,
                          unsigned n)
{
    if (off + n > m->len) {
        return;
    } else if (m->host) {
        memcpy(m->host + off, buf, n);
    } else {
        address_space_write(m->as, m->addr + off, MEMTXATTRS_UNSPECIFIED,
                            buf, n);
    }
}

static uint32_t vgl_mem_pixel(VglMem *m, uint64_t off, unsigned bytes)
{
    uint8_t b[4] = { 0 };

    vgl_mem_read(m, off, b, bytes);
    return ldl_le_p(b);
}

/* ---- image sampler ---- */

typedef struct {
    VglFormat fmt;
    VglMem mem;
    uint32_t stride, w, h;
    unsigned tile, filter;
    bool premul;
    bool perspective;
    int border[4];
    float c[3], dx[3], dy[3];
    const uint32_t *clut;
} VglSampler;

static bool vgl_sampler_open(VsiVgliteState *s, VglSampler *sm, bool paint)
{
    uint32_t cfg = vgl_st(s, paint ? ST_PAINT_CONFIG : ST_IMAGE_CONFIG);
    uint32_t size = vgl_st(s, paint ? ST_PAINT_SIZE : ST_IMAGE_SIZE);
    uint32_t stride = vgl_st(s, paint ? ST_PAINT_STRIDE : ST_IMAGE_STRIDE);
    hwaddr addr = vgl_st(s, paint ? ST_PAINT_ADDR : ST_IMAGE_ADDR);
    int i;

    memset(sm, 0, sizeof(*sm));
    sm->fmt = vgl_source_format(cfg);
    if (!sm->fmt.valid) {
        qemu_log_mask(LOG_UNIMP, "vsi.vglite: image format 0x%08x\n", cfg);
        return false;
    }
    if (stride & 0x10000000) {
        qemu_log_mask(LOG_UNIMP, "vsi.vglite: tiled image source\n");
    }
    sm->stride = stride & 0x0FFFFFFF;
    if (sm->fmt.wide_stride) {
        sm->stride = sm->stride / 4 * 3;
    }
    sm->w = size & 0xFFFF;
    sm->h = size >> 16;
    sm->tile = (cfg >> 12) & 3;
    sm->filter = (cfg >> 16) & 3;
    sm->premul = cfg & (1u << 24);
    vgl_abgr(vgl_st(s, paint ? ST_PAINT_BORDER : ST_IMAGE_BORDER),
             sm->border);
    vgl_premultiply(sm->border);
    if (paint) {
        sm->c[0] = vgl_stf(s, ST_PAINT_C0);
        sm->c[1] = vgl_stf(s, ST_PAINT_C0 + 1);
        sm->dx[0] = vgl_stf(s, ST_PAINT_C0 + 2);
        sm->dx[1] = vgl_stf(s, ST_PAINT_C0 + 3);
        sm->dy[0] = vgl_stf(s, ST_PAINT_C0 + 4);
        sm->dy[1] = vgl_stf(s, ST_PAINT_C0 + 5);
        sm->c[2] = 1.0f;
    } else {
        for (i = 0; i < 3; i++) {
            sm->c[i] = vgl_stf(s, ST_IMAGE_C0 + i);
            sm->dx[i] = vgl_stf(s, ST_IMAGE_X0 + i);
            sm->dy[i] = vgl_stf(s, ST_IMAGE_Y0 + i);
        }
        sm->perspective = sm->dx[2] != 0.0f || sm->dy[2] != 0.0f ||
                          sm->c[2] != 1.0f;
    }
    if (sm->fmt.index) {
        unsigned base = sm->fmt.bits == 8 ? ST_CLUT256 :
                        sm->fmt.bits == 4 ? ST_CLUT16 :
                        sm->fmt.bits == 2 ? ST_CLUT4 : ST_CLUT2;

        sm->clut = &s->state[base];
    }
    if (!sm->w || !sm->h) {
        return false;
    }
    /*
     * The last row needs only its pixels: a gradient's colour ramp is a
     * single row and the driver leaves its stride at 0 (q63 firmware).
     */
    vgl_mem_open(&sm->mem, &s->as, addr,
                 (uint64_t)sm->stride * (sm->h - 1) +
                 (sm->fmt.bytes ? (uint64_t)sm->w * sm->fmt.bytes
                                : ((uint64_t)sm->w * sm->fmt.bits + 7) / 8),
                 false);
    return true;
}

static void vgl_sampler_close(VglSampler *sm)
{
    vgl_mem_close(&sm->mem);
}

static int vgl_wrap(int i, int n, unsigned tile)
{
    switch (tile) {
    case TILE_PAD:
        return i < 0 ? 0 : i >= n ? n - 1 : i;
    case TILE_REPEAT:
        i %= n;
        return i < 0 ? i + n : i;
    case TILE_REFLECT: {
        int p = i % (2 * n);

        p = p < 0 ? p + 2 * n : p;
        return p < n ? p : 2 * n - 1 - p;
    }
    default:
        return (i < 0 || i >= n) ? -1 : i;
    }
}

/* premultiplied texel at integer coordinates */
static void vgl_texel(VglSampler *sm, int i, int j, int c[4])
{
    const VglFormat *f = &sm->fmt;
    uint32_t raw;

    i = vgl_wrap(i, sm->w, sm->tile);
    j = vgl_wrap(j, sm->h, sm->tile);
    if (i < 0 || j < 0) {
        memcpy(c, sm->border, sizeof(sm->border));
        return;
    }
    if (f->bits) {
        uint64_t bit = (uint64_t)i * f->bits;
        uint8_t byte = 0;

        vgl_mem_read(&sm->mem, (uint64_t)j * sm->stride + bit / 8, &byte, 1);
        /* first pixel in the most significant bits */
        raw = (byte >> (8 - f->bits - bit % 8)) & ((1u << f->bits) - 1);
    } else {
        raw = vgl_mem_pixel(&sm->mem,
                            (uint64_t)j * sm->stride + (uint64_t)i * f->bytes,
                            f->bytes);
    }
    if (f->index) {
        uint32_t e = sm->clut[raw];     /* ARGB8888 */

        c[CH_B] = e & 0xFF;
        c[CH_G] = (e >> 8) & 0xFF;
        c[CH_R] = (e >> 16) & 0xFF;
        c[CH_A] = e >> 24;
    } else {
        vgl_unpack(f, raw, c);
    }
    if (f->alpha_only) {
        c[CH_R] = c[CH_G] = c[CH_B] = c[CH_A];
    } else if (sm->premul) {
        vgl_premultiply(c);
    }
}

static void vgl_sample(VglSampler *sm, int x, int y, int c[4])
{
    float u = sm->c[0] + sm->dx[0] * x + sm->dy[0] * y;
    float v = sm->c[1] + sm->dx[1] * x + sm->dy[1] * y;
    float s, t;
    int k;

    if (sm->perspective) {
        float w = sm->c[2] + sm->dx[2] * x + sm->dy[2] * y;

        if (w == 0.0f) {
            memcpy(c, sm->border, sizeof(sm->border));
            return;
        }
        u /= w;
        v /= w;
    }
    s = u * sm->w;
    t = v * sm->h;
    if (sm->filter == 0) {
        vgl_texel(sm, (int)floorf(s), (int)floorf(t), c);
        return;
    }
    {
        float fs = s - 0.5f, ft = t - 0.5f;
        int i0 = (int)floorf(fs), j0 = (int)floorf(ft);
        int wx = (int)((fs - i0) * 256.0f), wy = (int)((ft - j0) * 256.0f);
        int t00[4], t10[4], t01[4], t11[4];

        vgl_texel(sm, i0, j0, t00);
        vgl_texel(sm, i0 + 1, j0, t10);
        vgl_texel(sm, i0, j0 + 1, t01);
        vgl_texel(sm, i0 + 1, j0 + 1, t11);
        for (k = 0; k < 4; k++) {
            int top = t00[k] * (256 - wx) + t10[k] * wx;
            int bot = t01[k] * (256 - wx) + t11[k] * wx;

            c[k] = (top * (256 - wy) + bot * wy + 32768) >> 16;
        }
    }
}

/* gradient colour ramp: image row 0, texel i holding position i / (w - 1) */
static void vgl_ramp(VglSampler *sm, double g, int c[4])
{
    float s;
    int i0, wx, t0[4], t1[4], k;

    if (!isfinite(g) || (sm->tile == TILE_COLOR && (g < 0.0 || g > 1.0))) {
        memcpy(c, sm->border, sizeof(sm->border));
        return;
    }
    s = g * (sm->w - 1);
    if (sm->filter == 0) {
        vgl_texel(sm, (int)floorf(s + 0.5f), 0, c);
        return;
    }
    i0 = (int)floorf(s);
    wx = (int)((s - i0) * 256.0f);
    vgl_texel(sm, i0, 0, t0);
    vgl_texel(sm, i0 + 1, 0, t1);
    for (k = 0; k < 4; k++) {
        c[k] = (t0[k] * (256 - wx) + t1[k] * wx + 128) >> 8;
    }
}

/* ---- pixel engine ---- */

typedef struct {
    VsiVgliteState *s;
    uint32_t ctrl;
    unsigned image_mode, blend, paint;
    int color[4];               /* as programmed */
    int color_pm[4];            /* premultiplied when requested */
    VglFormat fmt;
    VglMem mem;
    uint32_t stride;
    int clip_w, clip_h;
    int win_y0;                 /* first row mapped in mem */
    bool use_image, use_paint;
    VglSampler image, pattern;
    double grad[9];             /* c X Y rc rX rY rXX rYY rXY */
} VglPE;

static int vgl_mul(int a, int b)
{
    return (a * b + 127) / 255;
}

static void vgl_paint(VglPE *pe, int x, int y, int p[4])
{
    int t[4], k;

    if (pe->use_paint && pe->paint != PAINT_PATTERN) {
        const double *g = pe->grad;
        double pos = g[0] + g[1] * x + g[2] * y;

        if (pe->paint == PAINT_RADIAL) {
            double r = g[3] + g[4] * x + g[5] * y + g[6] * x * x +
                       g[7] * y * y + g[8] * x * y;

            pos += sqrt(MAX(r, 0.0));
        }
        vgl_ramp(&pe->pattern, pos, p);
        return;
    }
    if (pe->use_paint) {
        vgl_sample(&pe->pattern, x, y, p);
        if (pe->image_mode == IM_MULTIPLY) {
            for (k = 0; k < 4; k++) {
                p[k] = vgl_mul(p[k], pe->color[k]);
            }
        }
        return;
    }
    if (!pe->use_image) {
        memcpy(p, pe->color_pm, sizeof(pe->color_pm));
        return;
    }
    vgl_sample(&pe->image, x, y, t);
    switch (pe->image_mode) {
    case IM_MULTIPLY:
        for (k = 0; k < 4; k++) {
            p[k] = vgl_mul(t[k], pe->color[k]);
        }
        break;
    case IM_STENCIL:
    case IM_RECOLOR:
        for (k = 0; k < 4; k++) {
            p[k] = vgl_mul(pe->color_pm[k], t[CH_A]);
        }
        break;
    default:
        memcpy(p, t, sizeof(t));
        break;
    }
}

/* blend premultiplied source s (already scaled by coverage) onto d */
static void vgl_blend(unsigned mode, const int s[4], int d[4], int cov)
{
    int sa = s[CH_A], da = d[CH_A], r[4], k;

    for (k = 0; k < 4; k++) {
        int sc = s[k], dc = d[k], v;

        switch (mode) {
        case 0x1:                       /* src over */
            v = sc + vgl_mul(dc, 255 - sa);
            break;
        case 0x2:                       /* dst over */
            v = vgl_mul(sc, 255 - da) + dc;
            break;
        case 0x3:                       /* src in */
            v = vgl_mul(sc, da) + (dc * (256 - cov) >> 8);
            break;
        case 0x4:                       /* dst in: D * Sa (by coverage) */
            v = vgl_mul(dc, sa) + (dc * (256 - cov) >> 8);
            break;
        case 0x5:                       /* multiply */
            v = vgl_mul(sc, 255 - da) + vgl_mul(dc, 255 - sa) +
                vgl_mul(sc, dc);
            break;
        case 0x6:                       /* screen */
            v = sc + dc - vgl_mul(sc, dc);
            break;
        case 0x7:                       /* darken */
            v = MIN(sc + vgl_mul(dc, 255 - sa), dc + vgl_mul(sc, 255 - da));
            break;
        case 0x8:                       /* lighten */
            v = MAX(sc + vgl_mul(dc, 255 - sa), dc + vgl_mul(sc, 255 - da));
            break;
        case 0x9:                       /* additive */
            v = sc + dc;
            break;
        case 0xA:                       /* subtract: D * (1 - Sa) */
            v = vgl_mul(dc, 255 - sa);
            break;
        case 0xC:                       /* LVGL subtract */
            v = dc - sc;
            break;
        default:                        /* none: replace (by coverage) */
            v = sc + (dc * (256 - cov) >> 8);
            break;
        }
        r[k] = v < 0 ? 0 : v > 255 ? 255 : v;
    }
    memcpy(d, r, sizeof(r));
}

static bool vgl_pe_open(VsiVgliteState *s, VglPE *pe, int y0, int y1)
{
    uint32_t tcfg = vgl_st(s, ST_TARGET_CONFIG);
    uint32_t size = vgl_st(s, ST_TARGET_SIZE);
    uint32_t stride = vgl_st(s, ST_TARGET_STRIDE);

    memset(pe, 0, sizeof(*pe));
    pe->s = s;
    pe->ctrl = vgl_st(s, ST_CONTROL);
    pe->image_mode = CTRL_IMAGE_MODE(pe->ctrl);
    pe->blend = CTRL_BLEND(pe->ctrl);
    pe->paint = CTRL_PAINT(pe->ctrl);
    vgl_abgr(vgl_st(s, ST_COLOR), pe->color);
    memcpy(pe->color_pm, pe->color, sizeof(pe->color));
    if (pe->ctrl & CTRL_IN_PREMULT) {
        vgl_premultiply(pe->color_pm);
    }
    if (pe->ctrl & CTRL_UNMODELLED) {
        qemu_log_mask(LOG_UNIMP, "vsi.vglite: control 0x%08x: colour "
                      "transform / pixel matrix / mask not modelled\n",
                      pe->ctrl);
    }
    pe->fmt = vgl_target_format(tcfg);
    if (!pe->fmt.valid) {
        qemu_log_mask(LOG_UNIMP, "vsi.vglite: target format 0x%08x\n", tcfg);
        return false;
    }
    if (stride & 0x10000000) {
        qemu_log_mask(LOG_UNIMP, "vsi.vglite: tiled render target\n");
    }
    pe->stride = stride & 0x0FFFFFFF;
    if (pe->fmt.wide_stride) {
        pe->stride = pe->stride / 4 * 3;
    }
    pe->clip_w = size & 0xFFFF;
    pe->clip_h = size >> 16;
    if (pe->paint != PAINT_COLOR) {
        pe->use_paint = vgl_sampler_open(s, &pe->pattern, true);
        if (!pe->use_paint) {
            return false;
        }
    }
    if (pe->paint == PAINT_LINEAR || pe->paint == PAINT_RADIAL) {
        static const uint8_t reg[9] = {
            ST_PAINT_C0, ST_PAINT_C0 + 2, ST_PAINT_C0 + 4,
            ST_PAINT_C0 + 1, ST_PAINT_C0 + 3, ST_PAINT_C0 + 5,
            ST_GRAD_RXX, ST_GRAD_RYY, ST_GRAD_RXY,
        };
        int i;

        for (i = 0; i < 9; i++) {
            pe->grad[i] = vgl_stf(s, reg[i]);
        }
        if (trace_event_get_state_backends(TRACE_VSI_VGLITE_RAMP) &&
            pe->pattern.fmt.bytes) {
            unsigned bpp = pe->pattern.fmt.bytes, w = pe->pattern.w;

            trace_vsi_vglite_ramp(vgl_st(s, ST_PAINT_CONFIG), w,
                                  vgl_mem_pixel(&pe->pattern.mem, 0, bpp),
                                  vgl_mem_pixel(&pe->pattern.mem,
                                                (w / 2) * bpp, bpp),
                                  vgl_mem_pixel(&pe->pattern.mem,
                                                (w - 1) * bpp, bpp));
        }
    }
    if (pe->image_mode != IM_NONE && !pe->use_paint) {
        pe->use_image = vgl_sampler_open(s, &pe->image, false);
        if (!pe->use_image) {
            if (pe->use_paint) {
                vgl_sampler_close(&pe->pattern);
            }
            return false;
        }
    }
    pe->win_y0 = y0;
    vgl_mem_open(&pe->mem, &s->as,
                 vgl_st(s, ST_TARGET_ADDR) + (hwaddr)y0 * pe->stride,
                 (uint64_t)pe->stride * (y1 - y0), true);
    return true;
}

static void vgl_pe_close(VglPE *pe)
{
    vgl_mem_close(&pe->mem);
    if (pe->use_image) {
        vgl_sampler_close(&pe->image);
    }
    if (pe->use_paint) {
        vgl_sampler_close(&pe->pattern);
    }
}

/* shade pixels x0..x1-1 of row y; cov[] = coverage 0..256, or NULL */
static void vgl_pe_span(VglPE *pe, int y, int x0, int x1, const int *cov)
{
    unsigned bpp = pe->fmt.bytes;
    uint64_t row = (uint64_t)(y - pe->win_y0) * pe->stride;
    int x, k;

    for (x = x0; x < x1; x++) {
        int c = cov ? cov[x - x0] : 256;
        int p[4], d[4];
        uint32_t raw;
        uint8_t b[4];

        if (c <= 0) {
            continue;
        }
        vgl_paint(pe, x, y, p);
        if (c < 256) {
            for (k = 0; k < 4; k++) {
                p[k] = (p[k] * c + 128) >> 8;
            }
        }
        raw = vgl_mem_pixel(&pe->mem, row + (uint64_t)x * bpp, bpp);
        vgl_unpack(&pe->fmt, raw, d);
        if (pe->fmt.alpha_only) {
            d[CH_R] = d[CH_G] = d[CH_B] = d[CH_A];
        }
        vgl_blend(pe->blend, p, d, c);
        raw = vgl_pack(&pe->fmt, d);
        stl_le_p(b, raw);
        vgl_mem_write(&pe->mem, row + (uint64_t)x * bpp, b, bpp);
    }
}

/* ---- rectangle operation (clear / blit) ---- */

static void vgl_rect_op(VsiVgliteState *s, const uint8_t *d)
{
    int x = (int16_t)lduw_le_p(d), y = (int16_t)lduw_le_p(d + 2);
    int w = lduw_le_p(d + 4), h = lduw_le_p(d + 6);
    uint32_t size = vgl_st(s, ST_TARGET_SIZE);
    int x0 = MAX(x, 0), y0 = MAX(y, 0);
    int x1 = MIN(x + w, (int)(size & 0xFFFF));
    int y1 = MIN(y + h, (int)(size >> 16));
    unsigned op = CTRL_OP(vgl_st(s, ST_CONTROL));
    VglPE pe;
    int row;

    s->ops++;
    trace_vsi_vglite_rect(vgl_st(s, ST_CONTROL), vgl_st(s, ST_TARGET_CONFIG),
                          vgl_st(s, ST_TARGET_ADDR), vgl_st(s, ST_IMAGE_CONFIG),
                          vgl_st(s, ST_IMAGE_ADDR), x, y, w, h);
    if ((op != OP_RECT && op != OP_CLEAR) || x1 <= x0 || y1 <= y0) {
        return;
    }
    if (!vgl_pe_open(s, &pe, y0, y1)) {
        return;
    }
    for (row = y0; row < y1; row++) {
        vgl_pe_span(&pe, row, x0, x1, NULL);
    }
    vgl_pe_close(&pe);
}

/* ---- path operation ---- */

typedef struct {
    float x0, y0, x1, y1;       /* y0 < y1 */
    int dir;
} VglEdge;

typedef struct {
    GArray *edges;
    float m[6];
    float scale, bias;
} VglFlat;

static void vgl_edge(VglFlat *fl, float x0, float y0, float x1, float y1)
{
    VglEdge e;

    if (y0 == y1 || !isfinite(x0) || !isfinite(y0) || !isfinite(x1) ||
        !isfinite(y1)) {
        return;
    }
    if (y0 < y1) {
        e = (VglEdge) { x0, y0, x1, y1, 1 };
    } else {
        e = (VglEdge) { x1, y1, x0, y0, -1 };
    }
    g_array_append_val(fl->edges, e);
}

static void vgl_xform(VglFlat *fl, float x, float y, float *ox, float *oy)
{
    x = x * fl->scale + fl->bias;
    y = y * fl->scale + fl->bias;
    *ox = fl->m[0] * x + fl->m[1] * y + fl->m[2];
    *oy = fl->m[3] * x + fl->m[4] * y + fl->m[5];
}

static int vgl_segments(float dd)
{
    int n = (int)ceilf(sqrtf(dd * 8.0f));

    return n < 1 ? 1 : n > 128 ? 128 : n;
}

/* curves are flattened in device space; points already transformed */
static void vgl_quad(VglFlat *fl, const float *p)
{
    float ddx = p[0] - 2 * p[2] + p[4], ddy = p[1] - 2 * p[3] + p[5];
    int n = vgl_segments(sqrtf(ddx * ddx + ddy * ddy)), i;
    float px = p[0], py = p[1];

    for (i = 1; i <= n; i++) {
        float t = (float)i / n, mt = 1 - t;
        float x = mt * mt * p[0] + 2 * mt * t * p[2] + t * t * p[4];
        float y = mt * mt * p[1] + 2 * mt * t * p[3] + t * t * p[5];

        vgl_edge(fl, px, py, x, y);
        px = x;
        py = y;
    }
}

static void vgl_cubic(VglFlat *fl, const float *p)
{
    float d1x = p[0] - 2 * p[2] + p[4], d1y = p[1] - 2 * p[3] + p[5];
    float d2x = p[2] - 2 * p[4] + p[6], d2y = p[3] - 2 * p[5] + p[7];
    float dd = MAX(sqrtf(d1x * d1x + d1y * d1y),
                   sqrtf(d2x * d2x + d2y * d2y)) * 1.5f;
    int n = vgl_segments(dd), i;
    float px = p[0], py = p[1];

    for (i = 1; i <= n; i++) {
        float t = (float)i / n, mt = 1 - t;
        float a = mt * mt * mt, b = 3 * mt * mt * t, c = 3 * mt * t * t;
        float e = t * t * t;
        float x = a * p[0] + b * p[2] + c * p[4] + e * p[6];
        float y = a * p[1] + b * p[3] + c * p[5] + e * p[7];

        vgl_edge(fl, px, py, x, y);
        px = x;
        py = y;
    }
}

static float vgl_coord(const uint8_t *p, unsigned fmt)
{
    float f;

    switch (fmt) {
    case 0:
        return (int8_t)p[0];
    case 1:
        return (int16_t)lduw_le_p(p);
    case 2:
        return (int32_t)ldl_le_p(p);
    default: {
        uint32_t v = ldl_le_p(p);

        memcpy(&f, &v, sizeof(f));
        return f;
    }
    }
}

static void vgl_flatten(VsiVgliteState *s, VglFlat *fl, const uint8_t *d,
                        uint32_t len, unsigned fmt)
{
    static const uint8_t ncoord[] = { 0, 0, 2, 2, 2, 2, 4, 4, 6, 6, 0 };
    unsigned cs = fmt == 0 ? 1 : fmt == 1 ? 2 : 4;
    float sx = 0, sy = 0, cx = 0, cy = 0;       /* path space */
    uint32_t off = 0;
    bool open = false;

    while (off < len) {
        uint8_t op = d[off];
        float a[6], p[8];
        unsigned n, i;
        bool rel;

        if (op == 0) {
            break;
        }
        if (op >= ARRAY_SIZE(ncoord)) {
            qemu_log_mask(LOG_GUEST_ERROR, "vsi.vglite: path opcode 0x%02x\n",
                          op);
            break;
        }
        off = ROUND_UP(off + 1, cs);
        n = ncoord[op];
        if (off + n * cs > len) {
            break;
        }
        for (i = 0; i < n; i++) {
            a[i] = vgl_coord(d + off + i * cs, fmt);
        }
        off += n * cs;
        rel = op >= 2 && (op & 1);
        if (rel) {
            for (i = 0; i < n; i += 2) {
                a[i] += cx;
                a[i + 1] += cy;
            }
        }
        switch (op & ~1) {
        case 0x0:                       /* CLOSE (op 1) */
            if (open) {
                vgl_xform(fl, cx, cy, &p[0], &p[1]);
                vgl_xform(fl, sx, sy, &p[2], &p[3]);
                vgl_edge(fl, p[0], p[1], p[2], p[3]);
            }
            cx = sx;
            cy = sy;
            open = false;
            break;
        case 0x2:                       /* MOVE: closes the open subpath */
            if (open) {
                vgl_xform(fl, cx, cy, &p[0], &p[1]);
                vgl_xform(fl, sx, sy, &p[2], &p[3]);
                vgl_edge(fl, p[0], p[1], p[2], p[3]);
            }
            sx = cx = a[0];
            sy = cy = a[1];
            open = false;
            break;
        case 0x4:                       /* LINE */
            vgl_xform(fl, cx, cy, &p[0], &p[1]);
            vgl_xform(fl, a[0], a[1], &p[2], &p[3]);
            vgl_edge(fl, p[0], p[1], p[2], p[3]);
            cx = a[0];
            cy = a[1];
            open = true;
            break;
        case 0x6:                       /* QUAD */
            vgl_xform(fl, cx, cy, &p[0], &p[1]);
            vgl_xform(fl, a[0], a[1], &p[2], &p[3]);
            vgl_xform(fl, a[2], a[3], &p[4], &p[5]);
            vgl_quad(fl, p);
            cx = a[2];
            cy = a[3];
            open = true;
            break;
        case 0x8:                       /* CUBIC */
            vgl_xform(fl, cx, cy, &p[0], &p[1]);
            vgl_xform(fl, a[0], a[1], &p[2], &p[3]);
            vgl_xform(fl, a[2], a[3], &p[4], &p[5]);
            vgl_xform(fl, a[4], a[5], &p[6], &p[7]);
            vgl_cubic(fl, p);
            cx = a[4];
            cy = a[5];
            open = true;
            break;
        case 0xA:                       /* BREAK */
            break;
        }
    }
    if (open) {                         /* fills close every subpath */
        float p[4];

        vgl_xform(fl, cx, cy, &p[0], &p[1]);
        vgl_xform(fl, sx, sy, &p[2], &p[3]);
        vgl_edge(fl, p[0], p[1], p[2], p[3]);
    }
}

static gint vgl_edge_cmp(gconstpointer a, gconstpointer b)
{
    const VglEdge *ea = a, *eb = b;

    return ea->y0 < eb->y0 ? -1 : ea->y0 > eb->y0;
}

typedef struct {
    float x;
    int dir;
} VglCross;

static gint vgl_cross_cmp(gconstpointer a, gconstpointer b)
{
    const VglCross *ca = a, *cb = b;

    return ca->x < cb->x ? -1 : ca->x > cb->x;
}

/*
 * Scanline coverage: quality 0 samples each pixel centre; higher
 * qualities use four sub-scanlines with exact horizontal coverage.
 */
static void vgl_raster(VglPE *pe, GArray *ea, int rx0, int ry0, int rx1,
                       int ry1, bool even_odd, unsigned quality)
{
    VglEdge *e = (VglEdge *)ea->data;
    unsigned ne = ea->len, nsub = quality ? 4 : 1;
    int w = rx1 - rx0, y, x;
    float *acc = g_new(float, w);
    int *cov = g_new(int, w);
    VglCross *xs = g_new(VglCross, ne + 1);

    g_array_sort(ea, vgl_edge_cmp);
    e = (VglEdge *)ea->data;
    for (y = ry0; y < ry1; y++) {
        unsigned k;
        int lo = w, hi = -1;

        memset(acc, 0, sizeof(float) * w);
        for (k = 0; k < nsub; k++) {
            float sy = y + (k + 0.5f) / nsub, wt = 1.0f / nsub;
            unsigned i, nc = 0;
            int wind = 0;

            for (i = 0; i < ne && e[i].y0 <= sy; i++) {
                if (sy < e[i].y1) {
                    xs[nc].x = e[i].x0 + (sy - e[i].y0) *
                               (e[i].x1 - e[i].x0) / (e[i].y1 - e[i].y0);
                    xs[nc].dir = e[i].dir;
                    nc++;
                }
            }
            if (nc < 2) {
                continue;
            }
            qsort(xs, nc, sizeof(*xs), vgl_cross_cmp);
            for (i = 0; i + 1 < nc; i++) {
                float xa, xb;
                int ia, ib;

                wind += xs[i].dir;
                if (!(even_odd ? (wind & 1) : wind)) {
                    continue;
                }
                xa = MAX(xs[i].x, (float)rx0);
                xb = MIN(xs[i + 1].x, (float)rx1);
                if (xb <= xa) {
                    continue;
                }
                if (!quality) {
                    ia = (int)ceilf(xa - 0.5f);
                    ib = (int)ceilf(xb - 0.5f);
                    for (x = MAX(ia, rx0); x < MIN(ib, rx1); x++) {
                        acc[x - rx0] = 1.0f;
                    }
                    lo = MIN(lo, ia - rx0);
                    hi = MAX(hi, ib - rx0);
                    continue;
                }
                ia = (int)floorf(xa);
                ib = (int)floorf(xb);
                if (ia == ib) {
                    acc[ia - rx0] += (xb - xa) * wt;
                } else {
                    acc[ia - rx0] += (ia + 1 - xa) * wt;
                    for (x = ia + 1; x < ib; x++) {
                        acc[x - rx0] += wt;
                    }
                    if (ib < rx1) {
                        acc[ib - rx0] += (xb - ib) * wt;
                    }
                }
                lo = MIN(lo, ia - rx0);
                hi = MAX(hi, ib - rx0 + 1);
            }
        }
        lo = MAX(lo, 0);
        hi = MIN(hi, w);
        if (hi <= lo) {
            continue;
        }
        for (x = lo; x < hi; x++) {
            int c = (int)(acc[x] * 256.0f + 0.5f);

            cov[x] = c > 256 ? 256 : c;
        }
        vgl_pe_span(pe, y, rx0 + lo, rx0 + hi, cov + lo);
    }
    g_free(xs);
    g_free(cov);
    g_free(acc);
}

static void vgl_path_op(VsiVgliteState *s, hwaddr data, uint32_t bytes)
{
    uint32_t pctrl = vgl_st(s, ST_PATH_CONTROL);
    uint32_t org = vgl_st(s, ST_TESS_ORIGIN), sz = vgl_st(s, ST_TESS_SIZE);
    uint32_t tsize = vgl_st(s, ST_TARGET_SIZE);
    int rx0 = org & 0xFFFF, ry0 = org >> 16;
    int rx1 = MIN(rx0 + (int)(sz & 0xFFFF), (int)(tsize & 0xFFFF));
    int ry1 = MIN(ry0 + (int)(sz >> 16), (int)(tsize >> 16));
    VglFlat fl;
    uint8_t *buf;
    VglPE pe;
    int i;

    s->ops++;
    trace_vsi_vglite_path(vgl_st(s, ST_CONTROL), pctrl,
                          vgl_st(s, ST_TARGET_ADDR), data, bytes);
    if (rx1 <= rx0 || ry1 <= ry0 || !bytes || bytes > VGL_MAX_PATH_BYTES) {
        return;
    }
    buf = g_malloc(bytes);
    address_space_read(&s->as, data, MEMTXATTRS_UNSPECIFIED, buf, bytes);
    fl.edges = g_array_new(false, false, sizeof(VglEdge));
    for (i = 0; i < 6; i++) {
        fl.m[i] = vgl_stf(s, ST_PATH_MATRIX + i);
    }
    fl.scale = vgl_stf(s, ST_PATH_SCALE);
    fl.bias = vgl_stf(s, ST_PATH_BIAS);
    vgl_flatten(s, &fl, buf, bytes, (pctrl >> 20) & 3);
    if (fl.edges->len && vgl_pe_open(s, &pe, ry0, ry1)) {
        vgl_raster(&pe, fl.edges, rx0, ry0, rx1, ry1,
                   pctrl & PATH_EVEN_ODD, pctrl & 3);
        vgl_pe_close(&pe);
    }
    g_array_free(fl.edges, true);
    g_free(buf);
}

/* ---- command processor ---- */

static void vgl_state_write(VsiVgliteState *s, uint32_t addr, uint32_t v)
{
    trace_vsi_vglite_state(addr, v);
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

            if (vgl_st(s, ST_PATH_CONTROL) & PATH_TESS_ENABLE) {
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
