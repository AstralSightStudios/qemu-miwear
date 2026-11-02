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
 *          w = c2 + ..; texel = (u / w * width, v / w * height), or
 *          (u / w, v / w) for hardware with the math precision fix
 *          (property "unnormalized"; VGLite 4.x built with
 *          gcFEATURE_VG_MATH_PRECISION_FIX programs the steps in texels:
 *          the best1502p driver in p65)
 *   0x0A25 image config: format ([3:0], [5:4] order, [11:9] index / 2222,
 *          [30:29] 24-bit group), [13:12] tiling outside the image
 *          (0 border colour 0x0A27, 1 pad, 2 repeat, 3 reflect), [17:16]
 *          filter (0 point, 1 linear, 2 bilinear, 3 3x3 Gaussian), bit 24
 *          premultiply texels; 0x0A29 address, 0x0A2B stride, 0x0A2D
 *          origin x | y << 16 and 0x0A2F width | height << 16 of the
 *          rectangle used as the image (vg_lite_blit_rect programs its
 *          source rectangle there; the coordinate generator and the tiling
 *          are relative to it). Texel coordinates
 *          are not centred: point
 *          and Gaussian sampling take texel floor(s), (bi)linear blends
 *          floor(s) and floor(s) + 1 by frac(s). The q63 firmware's blur
 *          (466 -> 93 Gaussian, 2x bilinear steps down to 23, a 3x3
 *          Gaussian pass, bilinear steps back up) programs its offsets for
 *          exactly that: 2:1 steps sample half way between two texels.
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
 * Host side: the common cases (host-mapped memory, formats of 8-bit or
 * smaller channels, point and (bi)linear images, the usual blend modes)
 * take fast paths that compute exactly what the generic per-pixel code
 * does; property "render-threads" (0: automatic) shades the rows of large
 * operations on several host threads; trace event vsi_vglite_stats
 * reports each submit's work and host time; MIRAGE_VGL_DUMP captures
 * submits for offline replay (see vgl_dump_touch()).
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qapi/error.h"
#include "qemu/cutils.h"
#include "qemu/log.h"
#include "qemu/thread.h"
#include "qemu/timer.h"
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
#define ST_IMAGE_ORIGIN     0x2D    /* x | y << 16 */
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

    v &= max;
    return bits >= 8 ? (int)v : (int)((v * 255 + max / 2) / max);
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

static inline QEMU_ALWAYS_INLINE void vgl_premultiply(int c[4])
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

/*
 * Fast pixel access for the formats made of channels of at most 8 bits
 * (everything but A4 / A8 / L8 and CLUT indices): the position and width
 * of each channel, and tables holding exactly what vgl_expand() and the
 * rounding in vgl_pack() compute, so both paths produce the same values.
 */
typedef struct {
    uint8_t shift[4];           /* R G B A */
    uint8_t bits[4];            /* 0: not stored, reads as 255 (alpha) */
    uint32_t fixed;             /* padding alpha bits, stored as ones */
} VglLayout;

static uint8_t vgl_expand_lut[9][256];  /* [bits][v] = vgl_expand(v, bits) */
static uint8_t vgl_reduce_lut[9][256];  /* [bits][c]: c * max / 255, rounded */

static void vgl_init_luts(void)
{
    unsigned bits, v;

    for (bits = 1; bits <= 8; bits++) {
        for (v = 0; v < 256; v++) {
            vgl_expand_lut[bits][v] = vgl_expand(v, bits);
            vgl_reduce_lut[bits][v] = (v * ((1u << bits) - 1) + 127) / 255;
        }
    }
}

static bool vgl_layout(const VglFormat *f, VglLayout *l)
{
    unsigned i, shift = 0;

    memset(l, 0, sizeof(*l));
    if (!f->valid || !f->bytes || f->alpha_only || f->lum || f->index) {
        return false;
    }
    for (i = 0; i < 4; i++) {
        unsigned ch = f->order[i], w = f->width[ch];

        if (!w) {
            continue;
        }
        if (ch != CH_A || f->alpha) {
            l->shift[ch] = shift;
            l->bits[ch] = w;
        } else {
            l->fixed |= ((1u << w) - 1) << shift;
        }
        shift += w;
    }
    return true;
}

/* vgl_unpack() for a format with a layout */
static inline QEMU_ALWAYS_INLINE void vgl_unpack_fast(const VglLayout *l,
                                                   uint32_t raw, int c[4])
{
    int k;

    for (k = 0; k < 4; k++) {
        unsigned b = l->bits[k];

        c[k] = b ? vgl_expand_lut[b][(raw >> l->shift[k]) & ((1u << b) - 1)]
                 : 255;
    }
}

/* vgl_pack() for a format with a layout; c[] must be 0..255 */
static inline QEMU_ALWAYS_INLINE uint32_t vgl_pack_fast(const VglLayout *l,
                                                     const int c[4])
{
    uint32_t raw = l->fixed;
    int k;

    for (k = 0; k < 4; k++) {
        if (l->bits[k]) {
            raw |= (uint32_t)vgl_reduce_lut[l->bits[k]][c[k]] << l->shift[k];
        }
    }
    return raw;
}

static inline QEMU_ALWAYS_INLINE uint32_t vgl_ld_px(const uint8_t *p,
                                                 unsigned bytes)
{
    switch (bytes) {
    case 1:
        return p[0];
    case 2:
        return lduw_le_p(p);
    case 3:
        return p[0] | p[1] << 8 | p[2] << 16;
    default:
        return ldl_le_p(p);
    }
}

static inline QEMU_ALWAYS_INLINE void vgl_st_px(uint8_t *p, unsigned bytes,
                                             uint32_t v)
{
    switch (bytes) {
    case 1:
        p[0] = v;
        break;
    case 2:
        stw_le_p(p, v);
        break;
    case 3:
        p[0] = v;
        p[1] = v >> 8;
        p[2] = v >> 16;
        break;
    default:
        stl_le_p(p, v);
        break;
    }
}

/*
 * A pixel as four 16-bit lanes, R | G << 16 | B << 32 | A << 48: the hot
 * paths work on all four channels at once where no lane can overflow.
 */
#define LANES_1     0x0001000100010001ull
#define LANES_FF    0x00FF00FF00FF00FFull

static inline QEMU_ALWAYS_INLINE uint64_t vgl_lanes(const int c[4])
{
    return (uint64_t)c[CH_R] | (uint64_t)c[CH_G] << 16 |
           (uint64_t)c[CH_B] << 32 | (uint64_t)c[CH_A] << 48;
}

static inline QEMU_ALWAYS_INLINE void vgl_unlanes(uint64_t v, int c[4])
{
    c[CH_R] = v & 0xFFFF;
    c[CH_G] = (v >> 16) & 0xFFFF;
    c[CH_B] = (v >> 32) & 0xFFFF;
    c[CH_A] = v >> 48;
}

/*
 * x / 255 in every lane, for lanes up to 65534 (checked for each value:
 * (x + 1 + (x >> 8)) >> 8 == x / 255 for 0 <= x <= 65534); intermediate
 * lanes stay below 65536. vgl_mul() of 0..255 values is
 * vgl_lanes_div255(a * b + 127), at most 65152.
 */
static inline QEMU_ALWAYS_INLINE uint64_t vgl_lanes_div255(uint64_t x)
{
    return ((x + LANES_1 + ((x >> 8) & LANES_FF)) >> 8) & LANES_FF;
}

/* MIN(v, 255) in every lane, for lanes up to 1023 */
static inline QEMU_ALWAYS_INLINE uint64_t vgl_lanes_sat(uint64_t v)
{
    uint64_t over = (v >> 8) & (3 * LANES_1);

    return (v | ((over | over >> 1) & LANES_1) * 0xFF) & LANES_FF;
}

/* a * b in every lane, for lanes up to 255 */
static inline QEMU_ALWAYS_INLINE uint64_t vgl_lanes_mul(uint64_t a, uint64_t b)
{
    return (a & 0xFFFF) * (b & 0xFFFF) |
           ((a >> 16) & 0xFFFF) * ((b >> 16) & 0xFFFF) << 16 |
           ((a >> 32) & 0xFFFF) * ((b >> 32) & 0xFFFF) << 32 |
           (a >> 48) * (b >> 48) << 48;
}

static inline QEMU_ALWAYS_INLINE uint32_t vgl_pack_lanes(const VglLayout *l,
                                                      uint64_t v)
{
    uint32_t raw = l->fixed;
    int k;

    for (k = 0; k < 4; k++) {
        if (l->bits[k]) {
            raw |= (uint32_t)vgl_reduce_lut[l->bits[k]][(v >> 16 * k) & 0xFF]
                   << l->shift[k];
        }
    }
    return raw;
}

/*
 * Every 16-bit pixel of a layout decoded to lanes (and premultiplied, for
 * images the sampler premultiplies), kept for the last few layouts: a
 * 16-bit texel or render target pixel read is then one table lookup.
 * Building a table costs about as much as decoding a few ten thousand
 * pixels: for a smaller operation (area pixels) only a kept table is
 * used, else NULL. An operation uses up to three tables (pattern, image,
 * target), all valid until its end: a table it got is never replaced
 * (s->lut_op counts operations, see vgl_pe_open()).
 */
#define VGL_LUT16_AREA      16384

static const uint64_t *vgl_lut16(VsiVgliteState *s, const VglLayout *l,
                                 bool premul, uint64_t area)
{
    uint64_t key = premul;
    uint64_t *lut;
    uint32_t raw;
    unsigned i;

    for (i = 0; i < 4; i++) {
        key = key << 9 | l->shift[i] << 4 | l->bits[i];
    }
    for (i = 0; i < VGL_TEXEL_LUTS; i++) {
        if (s->texel_lut[i] && s->texel_lut_key[i] == key) {
            s->texel_lut_op[i] = s->lut_op;
            return s->texel_lut[i];
        }
    }
    if (area < VGL_LUT16_AREA) {
        return NULL;
    }
    do {
        i = s->texel_lut_next++ % VGL_TEXEL_LUTS;
    } while (s->texel_lut[i] && s->texel_lut_op[i] == s->lut_op);
    s->texel_lut_op[i] = s->lut_op;
    if (!s->texel_lut[i]) {
        s->texel_lut[i] = g_new(uint64_t, 0x10000);
    }
    lut = s->texel_lut[i];
    s->texel_lut_key[i] = key;
    for (raw = 0; raw < 0x10000; raw++) {
        int c[4];

        vgl_unpack_fast(l, raw, c);
        if (premul) {
            vgl_premultiply(c);
        }
        lut[raw] = vgl_lanes(c);
    }
    return lut;
}

/* ---- guest memory windows ---- */

typedef struct {
    AddressSpace *as;
    hwaddr addr;
    uint64_t len;
    uint8_t *host;              /* direct mapping, or NULL: slow path */
    bool write;
} VglMem;

static void vgl_dump_touch(VsiVgliteState *s, hwaddr addr, uint64_t len);

static void vgl_mem_open(VsiVgliteState *s, VglMem *m, hwaddr addr,
                         uint64_t len, bool write)
{
    AddressSpace *as = &s->as;
    hwaddr plen = len;

    vgl_dump_touch(s, addr, len);
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

/* how vgl_texel() reads texels inside a host-mapped image */
enum {
    TEX_SLOW,                   /* vgl_mem_read() and vgl_unpack() */
    TEX_RGBA,                   /* VglLayout */
    TEX_LUT16,                  /* 16-bit texel -> decoded texel table */
    TEX_INDEX8,
    TEX_A8,
    TEX_L8,
};

typedef struct {
    VglFormat fmt;
    VglMem mem;
    uint32_t stride, w, h;
    unsigned tile, filter;
    bool premul;
    bool perspective;
    bool unnormalized;          /* u, v already in texels */
    int border[4];
    float c[3], dx[3], dy[3];
    const uint32_t *clut;
    unsigned kind;              /* TEX_* */
    unsigned tbytes;            /* bytes per texel of the fast kinds */
    VglLayout layout;
    const uint64_t *lut;        /* TEX_LUT16: texel -> lanes */
    const uint64_t *clut_lanes; /* TEX_INDEX8 image: index -> lanes */
} VglSampler;

/* pick the fast texel reader for a host-mapped image (area: of the op) */
static void vgl_sampler_fast(VsiVgliteState *s, VglSampler *sm, uint64_t area)
{
    const VglFormat *f = &sm->fmt;

    if (vgl_layout(f, &sm->layout)) {
        sm->tbytes = f->bytes;
        if (f->bytes == 2) {
            sm->lut = vgl_lut16(s, &sm->layout, sm->premul, area);
        }
        sm->kind = sm->lut ? TEX_LUT16 : TEX_RGBA;
    } else if (f->index && f->bits == 8) {
        sm->tbytes = 1;
        sm->kind = TEX_INDEX8;
    } else if (f->alpha_only && f->bytes == 1) {
        sm->tbytes = 1;
        sm->kind = TEX_A8;
    } else if (f->lum) {
        sm->tbytes = 1;
        sm->kind = TEX_L8;
    }
}

static bool vgl_sampler_open(VsiVgliteState *s, VglSampler *sm, bool paint,
                             uint64_t area)
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
    sm->premul = !paint && !(cfg & (1u << 24));
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
        sm->unnormalized = s->unnormalized;
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
    if (!paint) {
        uint32_t origin = vgl_st(s, ST_IMAGE_ORIGIN);
        uint32_t ox = origin & 0xFFFF, oy = origin >> 16;

        if (sm->fmt.bytes) {
            addr += (uint64_t)oy * sm->stride + (uint64_t)ox * sm->fmt.bytes;
        } else if (origin) {
            addr += (uint64_t)oy * sm->stride + (uint64_t)ox * sm->fmt.bits / 8;
            if ((ox * sm->fmt.bits) % 8) {
                qemu_log_mask(LOG_UNIMP, "vsi.vglite: sub-byte image origin "
                              "x %u\n", ox);
            }
        }
    }
    /*
     * The last row needs only its pixels: a gradient's colour ramp is a
     * single row and the driver leaves its stride at 0 (q63 firmware).
     */
    vgl_mem_open(s, &sm->mem, addr,
                 (uint64_t)sm->stride * (sm->h - 1) +
                 (sm->fmt.bytes ? (uint64_t)sm->w * sm->fmt.bytes
                                : ((uint64_t)sm->w * sm->fmt.bits + 7) / 8),
                 false);
    if (sm->mem.host) {
        vgl_sampler_fast(s, sm, area);
    }
    if (sm->kind == TEX_INDEX8 && !paint &&
        (!s->clut_lanes_valid || s->clut_lanes_premul != sm->premul ||
         memcmp(s->clut_lanes_src, sm->clut, sizeof(s->clut_lanes_src)))) {
        /* the CLUT entries as vgl_texel_kind() reads them, for the lanes */
        memcpy(s->clut_lanes_src, sm->clut, sizeof(s->clut_lanes_src));
        s->clut_lanes_premul = sm->premul;
        s->clut_lanes_valid = true;
        for (i = 0; i < 256; i++) {
            uint32_t e = sm->clut[i];   /* ARGB8888 */
            int c[4];

            c[CH_B] = e & 0xFF;
            c[CH_G] = (e >> 8) & 0xFF;
            c[CH_R] = (e >> 16) & 0xFF;
            c[CH_A] = e >> 24;
            if (sm->premul) {
                vgl_premultiply(c);
            }
            s->clut_lanes[i] = vgl_lanes(c);
        }
    }
    if (sm->kind == TEX_INDEX8 && !paint) {
        sm->clut_lanes = s->clut_lanes;
    }
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

/*
 * vgl_texel() for a texel at p inside a host-mapped image of the given
 * kind (sm->kind, or that value as a constant for the compiler to fold):
 * the window covers every texel of the image, see vgl_sampler_open().
 */
static inline QEMU_ALWAYS_INLINE void vgl_texel_kind(const VglSampler *sm,
                                                  unsigned kind,
                                                  const uint8_t *p, int c[4])
{
    uint32_t e;

    switch (kind) {
    case TEX_LUT16:
        vgl_unlanes(sm->lut[lduw_le_p(p)], c);
        return;
    case TEX_RGBA:
        vgl_unpack_fast(&sm->layout, vgl_ld_px(p, sm->tbytes), c);
        break;
    case TEX_INDEX8:
        e = sm->clut[p[0]];             /* ARGB8888 */
        c[CH_B] = e & 0xFF;
        c[CH_G] = (e >> 8) & 0xFF;
        c[CH_R] = (e >> 16) & 0xFF;
        c[CH_A] = e >> 24;
        break;
    case TEX_A8:
        c[CH_R] = c[CH_G] = c[CH_B] = c[CH_A] = p[0];
        return;
    default:                            /* TEX_L8 */
        c[CH_R] = c[CH_G] = c[CH_B] = p[0];
        c[CH_A] = 255;
        break;
    }
    if (sm->premul) {
        vgl_premultiply(c);
    }
}

static inline QEMU_ALWAYS_INLINE void vgl_texel_fast(const VglSampler *sm,
                                                  const uint8_t *p, int c[4])
{
    vgl_texel_kind(sm, sm->kind, p, c);
}

/* premultiplied texel at integer coordinates */
static void vgl_texel(VglSampler *sm, int i, int j, int c[4])
{
    const VglFormat *f = &sm->fmt;
    uint32_t raw;

    if (sm->kind != TEX_SLOW && (unsigned)i < sm->w && (unsigned)j < sm->h) {
        /* inside the image: no tiling mode moves the texel */
        vgl_texel_fast(sm, sm->mem.host + (uint64_t)j * sm->stride +
                           (uint64_t)i * sm->tbytes, c);
        return;
    }
    i = vgl_wrap(i, sm->w, sm->tile);
    j = vgl_wrap(j, sm->h, sm->tile);
    if (i < 0 || j < 0) {
        memcpy(c, sm->border, sizeof(sm->border));
        return;
    }
    if (sm->kind != TEX_SLOW) {
        /* tiling leaves i, j inside the image */
        vgl_texel_fast(sm, sm->mem.host + (uint64_t)j * sm->stride +
                           (uint64_t)i * sm->tbytes, c);
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

/*
 * Image sample at pixel (x, y). sm may be a copy of the sampler that the
 * caller keeps in registers, kind is sm->kind (possibly as a constant);
 * texels vgl_texel() has to fetch (outside the image, or not host-mapped)
 * are read through orig, the sampler itself.
 */
static inline QEMU_ALWAYS_INLINE void vgl_sample_in(const VglSampler *sm,
                                                   unsigned kind,
                                                   VglSampler *orig,
                                                   int x, int y, int c[4])
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
    s = sm->unnormalized ? u : u * sm->w;
    t = sm->unnormalized ? v : v * sm->h;
    if (sm->filter == 0) {
        int i = (int)floorf(s), j = (int)floorf(t);

        if (kind != TEX_SLOW && (unsigned)i < sm->w && (unsigned)j < sm->h) {
            vgl_texel_kind(sm, kind, sm->mem.host + (uint64_t)j * sm->stride +
                                     (uint64_t)i * sm->tbytes, c);
            return;
        }
        vgl_texel(orig, i, j, c);
        return;
    }
    if (sm->filter == 3) {
        static const int g[3] = { 1, 2, 1 };
        int i0 = (int)floorf(s), j0 = (int)floorf(t), acc[4] = { 0 };
        int tx[4], i, j;

        for (j = -1; j <= 1; j++) {
            for (i = -1; i <= 1; i++) {
                vgl_texel(orig, i0 + i, j0 + j, tx);
                for (k = 0; k < 4; k++) {
                    acc[k] += tx[k] * g[i + 1] * g[j + 1];
                }
            }
        }
        for (k = 0; k < 4; k++) {
            c[k] = (acc[k] + 8) >> 4;
        }
        return;
    }
    {
        int i0 = (int)floorf(s), j0 = (int)floorf(t);
        int wx = (int)((s - i0) * 256.0f), wy = (int)((t - j0) * 256.0f);
        int t00[4], t10[4], t01[4], t11[4];

        if (kind != TEX_SLOW && (unsigned)i0 < sm->w - 1 &&
            (unsigned)j0 < sm->h - 1) {
            const uint8_t *p = sm->mem.host + (uint64_t)j0 * sm->stride +
                               (uint64_t)i0 * sm->tbytes;

            vgl_texel_kind(sm, kind, p, t00);
            vgl_texel_kind(sm, kind, p + sm->tbytes, t10);
            vgl_texel_kind(sm, kind, p + sm->stride, t01);
            vgl_texel_kind(sm, kind, p + sm->stride + sm->tbytes, t11);
        } else {
            vgl_texel(orig, i0, j0, t00);
            vgl_texel(orig, i0 + 1, j0, t10);
            vgl_texel(orig, i0, j0 + 1, t01);
            vgl_texel(orig, i0 + 1, j0 + 1, t11);
        }
        for (k = 0; k < 4; k++) {
            int top = t00[k] * (256 - wx) + t10[k] * wx;
            int bot = t01[k] * (256 - wx) + t11[k] * wx;

            c[k] = (top * (256 - wy) + bot * wy + 32768) >> 16;
        }
    }
}

static void vgl_sample(VglSampler *sm, int x, int y, int c[4])
{
    vgl_sample_in(sm, sm->kind, sm, x, y, c);
}

/*
 * The bilinear footprint (i0, j0) and weights (wx, wy) vgl_sample_in()
 * computes for pixels x..x+n-1 of row y of an affine sampler, in a loop
 * simple enough to be vectorized: the same operations on each lane.
 */
static inline QEMU_ALWAYS_INLINE void vgl_coords(const VglSampler *sm,
                                                int x, int y, int n,
                                                int *i0, int *j0,
                                                int *wx, int *wy)
{
    const float c0 = sm->c[0], c1 = sm->c[1];
    const float dx0 = sm->dx[0], dx1 = sm->dx[1];
    const float dy0 = sm->dy[0], dy1 = sm->dy[1];
    const uint32_t w = sm->w, h = sm->h;
    const bool unnormalized = sm->unnormalized;
    int i;

    for (i = 0; i < n; i++) {
        float u = c0 + dx0 * (x + i) + dy0 * y;
        float v = c1 + dx1 * (x + i) + dy1 * y;
        float s = unnormalized ? u : u * w;
        float t = unnormalized ? v : v * h;

        i0[i] = (int)floorf(s);
        j0[i] = (int)floorf(t);
        wx[i] = (int)((s - i0[i]) * 256.0f);
        wy[i] = (int)((t - j0[i]) * 256.0f);
    }
}

/* the texel at p of a fast kind as lanes, as vgl_texel_kind() reads it */
static inline QEMU_ALWAYS_INLINE uint64_t vgl_texel_lanes(const VglSampler *sm,
                                                       unsigned kind,
                                                       const uint8_t *p)
{
    int c[4];

    switch (kind) {
    case TEX_LUT16:
        return sm->lut[lduw_le_p(p)];
    case TEX_INDEX8:
        return sm->clut_lanes[p[0]];
    default:
        vgl_texel_kind(sm, kind, p, c);
        return vgl_lanes(c);
    }
}

/*
 * The point or (bi)linear sample of an affine sampler of a fast kind at
 * footprint (i0, j0) with weights (wx, wy) from vgl_coords(), when the
 * footprint lies inside the image; returns false, having computed
 * nothing, otherwise. Inside the image the weights are 0..255: the texel
 * coordinate s is small, so s - i0 is exact and below 1, or NaN, which
 * converts to 0 for both. The texels are then blended in lanes, two and
 * four channels at a time: no lane overflows (t * (256 - w) + t' * w <=
 * 255 * 256 in 16 bits, then below 2^25 in 32 bits), so each channel
 * gets the per-channel result.
 */
static inline QEMU_ALWAYS_INLINE bool vgl_fetch_lanes(const VglSampler *sm,
                                                     unsigned kind, bool point,
                                                     int i0, int j0,
                                                     int wx, int wy,
                                                     uint64_t *out)
{
    const uint64_t lo = 0x0000FFFF0000FFFFull, half = 0x0000800000008000ull;
    const unsigned tb = kind == TEX_LUT16 ? 2 : sm->tbytes;
    const uint8_t *p;
    uint64_t top, bot, rb, ga;

    if (point) {
        if ((unsigned)i0 >= sm->w || (unsigned)j0 >= sm->h) {
            return false;
        }
        *out = vgl_texel_lanes(sm, kind, sm->mem.host +
                                         (uint64_t)j0 * sm->stride +
                                         (uint64_t)i0 * tb);
        return true;
    }
    if ((unsigned)i0 >= sm->w - 1 || (unsigned)j0 >= sm->h - 1) {
        return false;
    }
    p = sm->mem.host + (uint64_t)j0 * sm->stride + (uint64_t)i0 * tb;
    top = vgl_texel_lanes(sm, kind, p) * (256 - wx) +
          vgl_texel_lanes(sm, kind, p + tb) * wx;
    p += sm->stride;
    bot = vgl_texel_lanes(sm, kind, p) * (256 - wx) +
          vgl_texel_lanes(sm, kind, p + tb) * wx;
    /* R and B, G and A in 32-bit lanes */
    rb = (top & lo) * (256 - wy) + (bot & lo) * wy + half;
    ga = ((top >> 16) & lo) * (256 - wy) + ((bot >> 16) & lo) * wy + half;
    *out = ((rb >> 16) & lo) | ((ga >> 16) & lo) << 16;
    return true;
}

/*
 * vgl_sample_in() for an affine, (bi)linear TEX_LUT16 sampler whose 2x2
 * footprint at (x, y) lies inside the image (else false, see above)
 */
static inline QEMU_ALWAYS_INLINE bool vgl_bilinear16(const VglSampler *sm,
                                                    int x, int y,
                                                    uint64_t *out)
{
    int i0, j0, wx, wy;

    vgl_coords(sm, x, y, 1, &i0, &j0, &wx, &wy);
    return vgl_fetch_lanes(sm, TEX_LUT16, false, i0, j0, wx, wy, out);
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
    bool fast;                  /* mem is host-mapped and fmt has a layout */
    VglLayout layout;
    const uint64_t *dlut;       /* fast 16-bit target: pixel -> lanes */
    bool exact;                 /* fast, every bit a channel: round trips */
    bool use_image, use_paint;
    VglSampler image, pattern;
    double grad[9];             /* c X Y rc rX rY rXX rYY rXY */
} VglPE;

static inline QEMU_ALWAYS_INLINE int vgl_mul(int a, int b)
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
static inline QEMU_ALWAYS_INLINE void vgl_blend(unsigned mode, const int s[4],
                                             int d[4], int cov)
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

/* area: pixels the operation may cover, to weigh setup costs */
static bool vgl_pe_open(VsiVgliteState *s, VglPE *pe, int y0, int y1,
                        uint64_t area)
{
    uint32_t tcfg = vgl_st(s, ST_TARGET_CONFIG);
    uint32_t size = vgl_st(s, ST_TARGET_SIZE);
    uint32_t stride = vgl_st(s, ST_TARGET_STRIDE);

    memset(pe, 0, sizeof(*pe));
    pe->s = s;
    s->lut_op++;                /* a new user of the vgl_lut16() tables */
    pe->ctrl = vgl_st(s, ST_CONTROL);
    pe->image_mode = CTRL_IMAGE_MODE(pe->ctrl);
    pe->blend = CTRL_BLEND(pe->ctrl);
    pe->paint = CTRL_PAINT(pe->ctrl);
    vgl_abgr(vgl_st(s, ST_COLOR), pe->color);
    memcpy(pe->color_pm, pe->color, sizeof(pe->color));
    /*
     * Drivers set bit 28 (in_premult) when the render target is not
     * premultiplied; the pen colour from LVGL is already premultiplied.
     */
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
        pe->use_paint = vgl_sampler_open(s, &pe->pattern, true, area);
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
        pe->use_image = vgl_sampler_open(s, &pe->image, false, area);
        if (!pe->use_image) {
            if (pe->use_paint) {
                vgl_sampler_close(&pe->pattern);
            }
            return false;
        }
    }
    pe->win_y0 = y0;
    vgl_mem_open(s, &pe->mem,
                 vgl_st(s, ST_TARGET_ADDR) + (hwaddr)y0 * pe->stride,
                 (uint64_t)pe->stride * (y1 - y0), true);
    pe->fast = pe->mem.host && vgl_layout(&pe->fmt, &pe->layout);
    if (pe->fast) {
        const VglLayout *l = &pe->layout;

        if (pe->fmt.bytes == 2) {
            pe->dlut = vgl_lut16(s, l, false, area);
        }
        /* reduce(expand(v)) == v for every channel width (checked) */
        pe->exact = !l->fixed && l->bits[0] + l->bits[1] + l->bits[2] +
                                 l->bits[3] == 8 * pe->fmt.bytes;
    }
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

/*
 * What the fast span loops read, copied to locals first: every pixel
 * store might alias the device state otherwise, and the compiler would
 * reload it all for each pixel.
 */
typedef struct {
    VglSampler im;
    VglLayout lay;
    unsigned bpp, mode;
    bool use_image;
    bool bilinear16;            /* vgl_bilinear16() applies to im */
    bool white;                 /* opaque white colour */
    int color[4], color_pm[4];
} VglSpan;

static inline QEMU_ALWAYS_INLINE void vgl_span_init(VglPE *pe, VglSpan *sp,
                                                   unsigned kind)
{
    sp->im = pe->image;
    sp->lay = pe->layout;
    sp->bpp = pe->fmt.bytes;
    sp->mode = pe->image_mode;
    sp->use_image = pe->use_image;
    sp->bilinear16 = kind == TEX_LUT16 && !sp->im.perspective &&
                     (sp->im.filter == 1 || sp->im.filter == 2);
    sp->white = vgl_st(pe->s, ST_COLOR) == 0xFFFFFFFF;
    memcpy(sp->color, pe->color, sizeof(sp->color));
    memcpy(sp->color_pm, pe->color_pm, sizeof(sp->color_pm));
}

/*
 * One pixel of vgl_pe_span() for a host-mapped target with a layout and
 * the fill colour or image paint (not a gradient or pattern paint): the
 * operations of the generic loop, with the image texel kind and the blend
 * mode possibly constants for the compiler to fold.
 */
static inline QEMU_ALWAYS_INLINE void vgl_pixel_fast(VglPE *pe,
                                                    const VglSpan *sp,
                                                    uint8_t *px, int x, int y,
                                                    int c, unsigned blend,
                                                    unsigned kind)
{
    int p[4], d[4], t[4], k;

    if (!sp->use_image) {
        memcpy(p, sp->color_pm, sizeof(p));
    } else {
        bool in_range = false;          /* 0 <= t[k] <= 255 for sure */
        uint64_t l;

        if (sp->bilinear16 && vgl_bilinear16(&sp->im, x, y, &l)) {
            vgl_unlanes(l, t);
            in_range = true;
        } else {
            vgl_sample_in(&sp->im, kind, &pe->image, x, y, t);
        }
        switch (sp->mode) {
        case IM_MULTIPLY:
            /* vgl_mul(t, 255) == t for t >= 0 */
            if (sp->white && in_range) {
                memcpy(p, t, sizeof(t));
                break;
            }
            for (k = 0; k < 4; k++) {
                p[k] = vgl_mul(t[k], sp->color[k]);
            }
            break;
        case IM_STENCIL:
        case IM_RECOLOR:
            for (k = 0; k < 4; k++) {
                p[k] = vgl_mul(sp->color_pm[k], t[CH_A]);
            }
            break;
        default:
            memcpy(p, t, sizeof(t));
            break;
        }
    }
    if (c < 256) {
        for (k = 0; k < 4; k++) {
            p[k] = (p[k] * c + 128) >> 8;
        }
    }
    vgl_unpack_fast(&sp->lay, vgl_ld_px(px, sp->bpp), d);
    vgl_blend(blend, p, d, c);
    vgl_st_px(px, sp->bpp, vgl_pack_fast(&sp->lay, d));
}

/* vgl_pe_span() through vgl_pixel_fast(); dst is row y */
static inline QEMU_ALWAYS_INLINE uint64_t vgl_span_fast(VglPE *pe,
                                                       uint8_t *dst,
                                                       int y, int x0, int x1,
                                                       const int *cov,
                                                       unsigned blend,
                                                       unsigned kind)
{
    VglSpan sp;
    uint64_t shaded = 0;
    int x;

    vgl_span_init(pe, &sp, kind);
    for (x = x0; x < x1; x++) {
        int c = cov ? cov[x - x0] : 256;

        if (c > 0) {
            vgl_pixel_fast(pe, &sp, dst + (uint64_t)x * sp.bpp, x, y, c,
                           blend, kind);
            shaded++;
        }
    }
    return shaded;
}

/* pixels sampled ahead of shading in vgl_span_lanes() */
#define VGL_BATCH   32

/*
 * vgl_pe_span() on four channels at once, for a target with a layout of
 * bpp bytes and the fill colour or an affine image of a fast kind read
 * with a point or (bi)linear filter, blend none / src-over / multiply /
 * screen / additive (kind, point, bpp and blend may be constants for the
 * compiler to fold; kind TEX_SLOW stands for the fill colour). These are
 * the steps of vgl_pixel_fast(), and every value is 0..255 here, so each
 * lane operation gives the per-channel result: tint and stencil products
 * and coverage scaling stay below 65536 per lane, vgl_mul() is
 * vgl_lanes_div255(), and the blend results (0 to 3 * 255) are clamped by
 * vgl_lanes_sat(). Pixels whose footprint leaves the image go through
 * vgl_pixel_fast().
 */
static inline QEMU_ALWAYS_INLINE uint64_t vgl_span_lanes(VglPE *pe,
                                                        uint8_t *dst,
                                                        int y, int x0, int x1,
                                                        const int *cov,
                                                        unsigned blend,
                                                        unsigned kind,
                                                        bool point,
                                                        unsigned bpp)
{
    const bool image = kind != TEX_SLOW;
    const uint64_t *dlut = pe->dlut;
    const bool exact = pe->exact;
    /*
     * When the image and the target do not share memory (host pointers:
     * aliased guest addresses map to the same host memory), sampling a
     * batch of pixels before shading any of them reads the same texels as
     * going pixel by pixel, and the samples no longer wait on the stores.
     */
    const bool batch = image &&
        (pe->image.mem.host + pe->image.mem.len <= pe->mem.host ||
         pe->mem.host + pe->mem.len <= pe->image.mem.host);
    uint64_t shaded = 0, cpm, color, lb[VGL_BATCH];
    int i0[VGL_BATCH], j0[VGL_BATCH], wx[VGL_BATCH], wy[VGL_BATCH];
    bool ok[VGL_BATCH];
    VglSpan sp;
    int x, i, n, grey = -1;

    vgl_span_init(pe, &sp, kind);
    cpm = vgl_lanes(sp.color_pm);
    color = vgl_lanes(sp.color);
    if (!sp.white && sp.color[0] == sp.color[1] && sp.color[0] == sp.color[2] &&
        sp.color[0] == sp.color[3]) {
        grey = sp.color[0];
    }
    for (x = x0; x < x1; x += n) {
        n = batch ? MIN(x1 - x, VGL_BATCH) : 1;
        if (image) {
            vgl_coords(&sp.im, x, y, n, i0, j0, wx, wy);
            for (i = 0; i < n; i++) {
                ok[i] = (!cov || cov[x + i - x0] > 0) &&
                        vgl_fetch_lanes(&sp.im, kind, point, i0[i], j0[i],
                                        wx[i], wy[i], &lb[i]);
            }
        }
        for (i = 0; i < n; i++) {
            int c = cov ? cov[x + i - x0] : 256;
            uint8_t *px = dst + (uint64_t)(x + i) * bpp;
            uint64_t l = cpm, d, v;
            unsigned sa;

            if (c <= 0) {
                continue;
            }
            shaded++;
            if (image) {
                if (!ok[i]) {
                    vgl_pixel_fast(pe, &sp, px, x + i, y, c, blend, kind);
                    continue;
                }
                l = lb[i];
                switch (sp.mode) {
                case IM_MULTIPLY:
                    if (grey >= 0) {
                        /* a grey tint (a faded white): one product */
                        l = vgl_lanes_div255(l * grey + 127 * LANES_1);
                    } else if (!sp.white) {
                        l = vgl_lanes_div255(vgl_lanes_mul(l, color) +
                                             127 * LANES_1);
                    }
                    break;
                case IM_STENCIL:
                case IM_RECOLOR:
                    l = vgl_lanes_div255(cpm * (l >> 48) + 127 * LANES_1);
                    break;
                }
            }
            if (c < 256) {
                l = ((l * c + 128 * LANES_1) >> 8) & LANES_FF;
            }
            /*
             * Nothing to add: src-over, additive and screen give the target
             * pixel back (vgl_mul(d, 255) == d), and it packs to what it was.
             */
            if (!l && exact && (blend == 0x1 || blend == 0x6 || blend == 0x9)) {
                continue;
            }
            sa = l >> 48;
            if (blend == 0x1 && sa == 255) {
                /* vgl_mul(d, 0) == 0 */
                vgl_st_px(px, bpp, vgl_pack_lanes(&sp.lay, l));
                continue;
            }
            if (bpp == 2 && dlut) {
                d = dlut[lduw_le_p(px)];
            } else {
                int dc[4];

                vgl_unpack_fast(&sp.lay, vgl_ld_px(px, bpp), dc);
                d = vgl_lanes(dc);
            }
            switch (blend) {
            case 0x1:                   /* src over */
                v = l + vgl_lanes_div255(d * (255 - sa) + 127 * LANES_1);
                break;
            case 0x5:                   /* multiply: at most 3 * 255 */
                v = vgl_lanes_div255(l * (255 - (d >> 48)) + 127 * LANES_1) +
                    vgl_lanes_div255(d * (255 - sa) + 127 * LANES_1) +
                    vgl_lanes_div255(vgl_lanes_mul(l, d) + 127 * LANES_1);
                break;
            case 0x6:                   /* screen: vgl_mul(s, d) <= s */
                v = l + d -
                    vgl_lanes_div255(vgl_lanes_mul(l, d) + 127 * LANES_1);
                break;
            case 0x9:                   /* additive */
                v = l + d;
                break;
            default:                    /* none: replace (by coverage) */
                v = l + ((d * (256 - c) >> 8) & LANES_FF);
                break;
            }
            vgl_st_px(px, bpp, vgl_pack_lanes(&sp.lay, vgl_lanes_sat(v)));
        }
    }
    return shaded;
}

/*
 * Shade pixels x0..x1-1 of row y; cov[] = coverage 0..256, or NULL.
 * Returns the number of pixels shaded.
 */
static uint64_t vgl_pe_span(VglPE *pe, int y, int x0, int x1, const int *cov)
{
    unsigned bpp = pe->fmt.bytes;
    uint64_t row = (uint64_t)(y - pe->win_y0) * pe->stride;
    uint8_t *dst = NULL;
    uint64_t shaded = 0;
    int x, k;

    /* the whole span inside the host mapping: no per-pixel checks */
    if (pe->fast && row + (uint64_t)x1 * bpp <= pe->mem.len) {
        dst = pe->mem.host + row;
    }
    if (dst && !pe->use_paint) {
        const VglSampler *im = &pe->image;
        unsigned kind = pe->use_image ? im->kind : TEX_SLOW;
        bool point = im->filter == 0, lanes;

        /*
         * Four channels at once for the fill colour and affine point or
         * (bi)linear images of a fast kind, with the common blend modes;
         * inlined for BAL2D's 16-bit images onto a 16-bit target.
         */
        lanes = (!pe->use_image ||
                 (kind != TEX_SLOW && !im->perspective && im->filter != 3)) &&
                (pe->blend == 0x0 || pe->blend == 0x1 || pe->blend == 0x5 ||
                 pe->blend == 0x6 || pe->blend == 0x9);
        if (lanes && kind == TEX_LUT16 && !point && bpp == 2) {
            switch (pe->blend) {
            case 0x0:
                shaded = vgl_span_lanes(pe, dst, y, x0, x1, cov, 0x0,
                                        TEX_LUT16, false, 2);
                break;
            case 0x1:
                shaded = vgl_span_lanes(pe, dst, y, x0, x1, cov, 0x1,
                                        TEX_LUT16, false, 2);
                break;
            case 0x5:
                shaded = vgl_span_lanes(pe, dst, y, x0, x1, cov, 0x5,
                                        TEX_LUT16, false, 2);
                break;
            case 0x6:
                shaded = vgl_span_lanes(pe, dst, y, x0, x1, cov, 0x6,
                                        TEX_LUT16, false, 2);
                break;
            default:
                shaded = vgl_span_lanes(pe, dst, y, x0, x1, cov, 0x9,
                                        TEX_LUT16, false, 2);
                break;
            }
        } else if (lanes && pe->blend == 0x1) {
            shaded = vgl_span_lanes(pe, dst, y, x0, x1, cov, 0x1, kind,
                                    point, bpp);
        } else if (lanes) {
            shaded = vgl_span_lanes(pe, dst, y, x0, x1, cov, pe->blend, kind,
                                    point, bpp);
        } else {
            shaded = vgl_span_fast(pe, dst, y, x0, x1, cov, pe->blend, kind);
        }
        return shaded;
    }
    for (x = x0; x < x1; x++) {
        int c = cov ? cov[x - x0] : 256;
        int p[4], d[4];
        uint32_t raw;
        uint8_t b[4];

        if (c <= 0) {
            continue;
        }
        shaded++;
        vgl_paint(pe, x, y, p);
        if (c < 256) {
            for (k = 0; k < 4; k++) {
                p[k] = (p[k] * c + 128) >> 8;
            }
        }
        if (dst) {
            uint8_t *px = dst + (uint64_t)x * bpp;

            vgl_unpack_fast(&pe->layout, vgl_ld_px(px, bpp), d);
            vgl_blend(pe->blend, p, d, c);
            vgl_st_px(px, bpp, vgl_pack_fast(&pe->layout, d));
            continue;
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
    return shaded;
}

/*
 * Rows y0..y1-1 of a draw operation, to be shaded in bands of rows (see
 * vgl_par_rows()): a path operation's rows (raster), or columns x0..x1-1
 * of each row of a rectangle.
 */
typedef struct VglJob {
    VglPE *pe;
    const struct VglRaster *raster;
    int x0, x1;
    int y0, y1;
    int band, nbands;           /* rows per band, bands */
    int next;                   /* next band to take (atomic) */
} VglJob;

static uint64_t vgl_par_rows(VsiVgliteState *s, VglJob *job);

/* ---- timing model ("timing") ---- */

/*
 * What a GC355 spends on one operation, added to the submit's modelled time:
 * the setup, tessellating the path's window (coverage is computed over the
 * whole window, plus work per edge), then filling the covered pixels at one
 * per clock, or slower when memory cannot keep up: the target is read (when
 * blending) and written, and about one texel is fetched per pixel (the
 * texture cache serves the rest of a bilinear footprint). The figures are
 * properties; emu/TIMING.md gives their sources.
 */
static void vgl_cost(VsiVgliteState *s, const VglPE *pe, bool clear,
                     uint64_t pixels, uint64_t window, uint64_t edges)
{
    const VglSampler *sm = pe->use_paint ? &pe->pattern :
                           pe->use_image ? &pe->image : NULL;
    double mhz = s->gpu_mhz ? s->gpu_mhz : 208;
    double mbps = s->mem_mbps ? s->mem_mbps : 400;
    double setup, fill, bytes, dst;

    if (!s->timing) {
        return;
    }
    dst = pe->fmt.bytes ? pe->fmt.bytes : 1;
    bytes = dst * (clear || pe->blend == 0 ? 1 : 2);
    fill = 1;
    if (sm && !clear) {
        bytes += sm->fmt.bytes ? sm->fmt.bytes : 1;
        if (sm->filter == 3) {
            fill = 2;               /* 3x3 Gaussian */
        }
    }
    setup = s->op_cycles + window / 32.0 + edges * 8.0;
    s->cost_ns += setup * 1000.0 / mhz +
                  MAX(pixels * fill * 1000.0 / mhz,
                      pixels * bytes * 1000.0 / mbps);
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
    uint64_t px;
    VglJob job;
    VglPE pe;

    s->ops++;
    s->stat_rects++;
    trace_vsi_vglite_rect(vgl_st(s, ST_CONTROL), vgl_st(s, ST_TARGET_CONFIG),
                          vgl_st(s, ST_TARGET_ADDR), vgl_st(s, ST_IMAGE_CONFIG),
                          vgl_st(s, ST_IMAGE_ADDR), x, y, w, h);
    if ((op != OP_RECT && op != OP_CLEAR) || x1 <= x0 || y1 <= y0) {
        return;
    }
    if (!vgl_pe_open(s, &pe, y0, y1, (uint64_t)(x1 - x0) * (y1 - y0))) {
        return;
    }
    job = (VglJob) { .pe = &pe, .x0 = x0, .x1 = x1, .y0 = y0, .y1 = y1 };
    px = vgl_par_rows(s, &job);
    s->stat_pixels += px;
    vgl_cost(s, &pe, op == OP_CLEAR, px, 0, 0);
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

/*
 * An edge crossing the current sub-scanline. Sub-scanlines only move
 * down, so an edge joins the active list once (y0 <= sy) and leaves it
 * for good (y1 <= sy): the list holds exactly the edges with
 * y0 <= sy < y1, the set a scan of all edges would find.
 */
typedef struct {
    float x;
    int dir;
    const VglEdge *e;
} VglActive;

/* where edge e crosses sub-scanline sy */
static inline float vgl_cross(const VglEdge *e, float sy)
{
    return e->x0 + (sy - e->y0) * (e->x1 - e->x0) / (e->y1 - e->y0);
}

static void *vgl_grow(void **buf, size_t *have, size_t need)
{
    if (need > *have) {
        g_free(*buf);
        *have = MAX(need, *have * 2);
        *buf = g_malloc(*have);
    }
    return *buf;
}

/* a path operation's edges, shared by the bands of its rows */
typedef struct VglRaster {
    VglPE *pe;
    const VglEdge *e;
    unsigned ne;
    int rx0, ry0, rx1, ry1;
    bool even_odd;
    unsigned quality;
    const uint32_t *order;      /* edge indices by the row of their top end */
    const uint32_t *end;        /* end[r]: end of row r's edges in order[] */
} VglRaster;

/* scratch vgl_raster_rows() needs */
static size_t vgl_raster_scratch(const VglRaster *r)
{
    return (sizeof(VglActive) + sizeof(uint32_t)) * r->ne +
           (sizeof(float) + sizeof(int)) * (r->rx1 - r->rx0);
}

/*
 * Scanline coverage of rows ya..yb-1: quality 0 samples each pixel
 * centre; higher qualities use four sub-scanlines with exact horizontal
 * coverage. Returns the number of pixels shaded.
 *
 * Crossings are sorted by x; the order of equal crossings does not
 * matter: the span between them is empty and skipped, and the winding
 * number after them is the same in any order. So a band of rows may start
 * with its active edges in any order.
 */
static uint64_t vgl_raster_rows(const VglRaster *r, int ya, int yb,
                                void *scratch)
{
    const VglEdge *e = r->e;
    const bool even_odd = r->even_odd;
    const unsigned quality = r->quality, nsub = quality ? 4 : 1;
    const int rx0 = r->rx0, rx1 = r->rx1, w = rx1 - rx0;
    VglActive *act = scratch;
    float *acc = (float *)(act + r->ne);
    int *cov = (int *)(acc + w);
    uint32_t *pend = (uint32_t *)(cov + w);
    unsigned i, j, nact = 0, npend = 0;
    unsigned next = ya > r->ry0 ? r->end[ya - r->ry0 - 1] : 0;
    uint64_t shaded = 0;
    int y, x;

    /*
     * Edges starting above the band wait as well: its first sub-scanline
     * takes the ones still crossing it and drops the others.
     */
    for (i = 0; i < next; i++) {
        pend[npend++] = r->order[i];
    }
    memset(acc, 0, sizeof(float) * w);
    for (y = ya; y < yb; y++) {
        unsigned k;
        /* quality 0: run0..run1-1 waits to be shaded (none if run1 <= run0) */
        int lo = w, hi = -1, run0 = 0, run1 = 0;

        /* edges whose top end lies in this row wait in pend[] */
        while (next < r->end[y - r->ry0]) {
            pend[npend++] = r->order[next++];
        }
        if (!nact && !npend) {
            continue;
        }
        for (k = 0; k < nsub; k++) {
            float sy = y + (k + 0.5f) / nsub, wt = 1.0f / nsub;
            unsigned n = 0;
            int wind = 0;

            for (i = 0; i < nact; i++) {
                if (sy < act[i].e->y1) {
                    act[n].e = act[i].e;
                    act[n].dir = act[i].dir;
                    act[n].x = vgl_cross(act[i].e, sy);
                    n++;
                }
            }
            for (i = 0, j = 0; i < npend; i++) {
                const VglEdge *pe0 = &e[pend[i]];

                if (pe0->y0 > sy) {
                    pend[j++] = pend[i];
                } else if (sy < pe0->y1) {
                    act[n].e = pe0;
                    act[n].dir = pe0->dir;
                    act[n].x = vgl_cross(pe0, sy);
                    n++;
                }
            }
            npend = j;
            nact = n;
            /* insertion sort: the order rarely changes between rows */
            for (i = 1; i < nact; i++) {
                VglActive a = act[i];

                for (j = i; j > 0 && act[j - 1].x > a.x; j--) {
                    act[j] = act[j - 1];
                }
                act[j] = a;
            }
            if (nact < 2) {
                continue;
            }
            for (i = 0; i + 1 < nact; i++) {
                float xa, xb;
                int ia, ib;

                wind += act[i].dir;
                if (!(even_odd ? (wind & 1) : wind)) {
                    continue;
                }
                xa = MAX(act[i].x, (float)rx0);
                xb = MIN(act[i + 1].x, (float)rx1);
                if (xb <= xa) {
                    continue;
                }
                if (!quality) {
                    /*
                     * Pixel centres in [xa, xb) are covered. Spans come
                     * left to right and do not overlap: shade them as
                     * runs at full coverage.
                     */
                    ia = MAX((int)ceilf(xa - 0.5f), rx0);
                    ib = MIN((int)ceilf(xb - 0.5f), rx1);
                    if (ia >= ib) {
                        continue;
                    }
                    if (run1 <= run0 || ia > run1) {
                        /* not adjacent to the waiting run: shade that */
                        if (run1 > run0) {
                            shaded += vgl_pe_span(r->pe, y, run0, run1, NULL);
                        }
                        run0 = ia;
                    }
                    run1 = ib;
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
        if (run1 > run0) {
            shaded += vgl_pe_span(r->pe, y, run0, run1, NULL);
        }
        /* every acc[] write above lies in [lo, hi) once clipped */
        lo = MAX(lo, 0);
        hi = MIN(hi, w);
        if (hi <= lo) {
            continue;
        }
        for (x = lo; x < hi; x++) {
            int c = (int)(acc[x] * 256.0f + 0.5f);

            cov[x] = c > 256 ? 256 : c;
        }
        shaded += vgl_pe_span(r->pe, y, rx0 + lo, rx0 + hi, cov + lo);
        memset(acc + lo, 0, sizeof(float) * (hi - lo));
    }
    return shaded;
}

/* ---- parallel rendering ---- */

/*
 * The rows of a draw operation can be shaded on several host threads
 * (property "render-threads"). Every pixel the operation writes belongs
 * to one row, so shading bands of rows in any order leaves the same
 * memory as going row by row, provided that the rows do not overlap in
 * memory (the stride covers the columns written) and that no image the
 * operation reads shares memory with the target. Worker threads only
 * touch host-mapped windows, never the memory API. Operations still run
 * one after the other and the vCPU waits for each: the GPU stays
 * synchronous.
 */
#define VGL_PAR_AREA        8192    /* pixels: smaller operations stay serial */
#define VGL_PAR_ROWS        8       /* rows of the smallest band */

typedef struct VglWorker {
    QemuThread thread;
    QemuSemaphore go;
    VsiVgliteState *s;
    void *scratch;
    size_t scratch_size;
    uint64_t shaded;
    bool exit;
} VglWorker;

static uint64_t vgl_job_band(VglJob *job, int ya, int yb, void **scratch,
                             size_t *scratch_size)
{
    uint64_t shaded = 0;
    int y;

    if (job->raster) {
        return vgl_raster_rows(job->raster, ya, yb,
                               vgl_grow(scratch, scratch_size,
                                        vgl_raster_scratch(job->raster)));
    }
    for (y = ya; y < yb; y++) {
        shaded += vgl_pe_span(job->pe, y, job->x0, job->x1, NULL);
    }
    return shaded;
}

/* take bands until none is left */
static uint64_t vgl_job_work(VglJob *job, void **scratch, size_t *scratch_size)
{
    uint64_t shaded = 0;
    int b;

    while ((b = qatomic_fetch_inc(&job->next)) < job->nbands) {
        int ya = job->y0 + b * job->band;

        shaded += vgl_job_band(job, ya, MIN(ya + job->band, job->y1),
                               scratch, scratch_size);
    }
    return shaded;
}

static void *vgl_worker(void *opaque)
{
    VglWorker *w = opaque;
    VsiVgliteState *s = w->s;

    for (;;) {
        qemu_sem_wait(&w->go);
        if (w->exit) {
            return NULL;
        }
        w->shaded = vgl_job_work(s->job, &w->scratch, &w->scratch_size);
        qemu_sem_post(&s->job_done);
    }
}

static void vgl_workers_start(VsiVgliteState *s)
{
    unsigned i;

    s->workers = g_new0(VglWorker, s->nthreads - 1);
    qemu_sem_init(&s->job_done, 0);
    for (i = 0; i < s->nthreads - 1; i++) {
        VglWorker *w = &s->workers[i];

        w->s = s;
        qemu_sem_init(&w->go, 0);
        qemu_thread_create(&w->thread, "vglite-render", vgl_worker, w,
                           QEMU_THREAD_JOINABLE);
    }
}

static void vgl_workers_stop(VsiVgliteState *s)
{
    unsigned i;

    if (!s->workers) {
        return;
    }
    for (i = 0; i < s->nthreads - 1; i++) {
        VglWorker *w = &s->workers[i];

        w->exit = true;
        qemu_sem_post(&w->go);
        qemu_thread_join(&w->thread);
        qemu_sem_destroy(&w->go);
        g_free(w->scratch);
    }
    qemu_sem_destroy(&s->job_done);
    g_free(s->workers);
    s->workers = NULL;
}

/* host memory a and b overlap */
static bool vgl_mem_overlap(const VglMem *a, const VglMem *b)
{
    return a->host < b->host + b->len && b->host < a->host + a->len;
}

/* bands of the job may be shaded concurrently (see above) */
static bool vgl_par_safe(const VglPE *pe, int x1)
{
    const VglSampler *sm[2] = {
        pe->use_image ? &pe->image : NULL,
        pe->use_paint ? &pe->pattern : NULL,
    };
    int i;

    if (!pe->mem.host || (uint64_t)x1 * pe->fmt.bytes > pe->stride) {
        return false;
    }
    for (i = 0; i < 2; i++) {
        if (sm[i] && (!sm[i]->mem.host || vgl_mem_overlap(&sm[i]->mem,
                                                          &pe->mem))) {
            return false;
        }
    }
    return true;
}

/* shade the rows of a job; returns the number of pixels shaded */
static uint64_t vgl_par_rows(VsiVgliteState *s, VglJob *job)
{
    int rows = job->y1 - job->y0;
    int x0 = job->raster ? job->raster->rx0 : job->x0;
    int x1 = job->raster ? job->raster->rx1 : job->x1;
    unsigned i, n = 0;
    uint64_t shaded;

    job->band = rows;
    job->nbands = 1;
    job->next = 0;
    if (s->nthreads > 1 && rows >= 2 * VGL_PAR_ROWS &&
        (uint64_t)(x1 - x0) * rows >= VGL_PAR_AREA &&
        vgl_par_safe(job->pe, x1)) {
        /* a few bands per thread, taken in turn, even out uneven rows */
        job->band = MAX(VGL_PAR_ROWS, DIV_ROUND_UP(rows, 4 * s->nthreads));
        job->nbands = DIV_ROUND_UP(rows, job->band);
        n = MIN(s->nthreads - 1, job->nbands - 1);
    }
    if (n) {
        if (!s->workers) {
            vgl_workers_start(s);
        }
        s->job = job;
        for (i = 0; i < n; i++) {
            qemu_sem_post(&s->workers[i].go);
        }
    }
    shaded = vgl_job_work(job, &s->band_scratch, &s->band_scratch_size);
    for (i = 0; i < n; i++) {
        qemu_sem_wait(&s->job_done);
    }
    for (i = 0; i < n; i++) {
        shaded += s->workers[i].shaded;
    }
    return shaded;
}

/*
 * Fill a path: its edges are bucketed by row once, then the rows are
 * rasterized in bands. Returns the number of pixels shaded.
 */
static uint64_t vgl_raster(VsiVgliteState *s, VglPE *pe, GArray *ea,
                           int rx0, int ry0, int rx1, int ry1, bool even_odd,
                           unsigned quality)
{
    const VglEdge *e = (const VglEdge *)ea->data;
    unsigned i, ne = ea->len, rows = ry1 - ry0;
    uint32_t *order, *row, *end;
    VglRaster r;
    VglJob job;

    order = vgl_grow(&s->scratch, &s->scratch_size,
                     sizeof(uint32_t) * (2 * ne + rows + 1));
    row = order + ne;
    end = row + ne;
    /*
     * Counting sort of the edges by the row holding their top end (the
     * first row for edges starting above the window). Edges below the
     * window or above its first sub-scanline never cross a sub-scanline.
     */
    memset(end, 0, sizeof(uint32_t) * (rows + 1));
    for (i = 0; i < ne; i++) {
        if (e[i].y0 >= ry1 || e[i].y1 <= ry0) {
            row[i] = UINT32_MAX;
            continue;
        }
        row[i] = e[i].y0 < ry0 ? 0 : (int)floorf(e[i].y0) - ry0;
        end[row[i] + 1]++;
    }
    for (i = 0; i < rows; i++) {
        end[i + 1] += end[i];
    }
    for (i = 0; i < ne; i++) {
        if (row[i] != UINT32_MAX) {
            order[end[row[i]]++] = i;
        }
    }
    /* now end[r] is the end of row r's edges in order[] */
    r = (VglRaster) {
        .pe = pe, .e = e, .ne = ne,
        .rx0 = rx0, .ry0 = ry0, .rx1 = rx1, .ry1 = ry1,
        .even_odd = even_odd, .quality = quality,
        .order = order, .end = end,
    };
    job = (VglJob) { .pe = pe, .raster = &r, .y0 = ry0, .y1 = ry1 };
    return vgl_par_rows(s, &job);
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
    VglPE pe;
    int i;

    s->ops++;
    s->stat_paths++;
    trace_vsi_vglite_path(vgl_st(s, ST_CONTROL), pctrl,
                          vgl_st(s, ST_TARGET_ADDR), data, bytes);
    if (rx1 <= rx0 || ry1 <= ry0 || !bytes || bytes > VGL_MAX_PATH_BYTES) {
        return;
    }
    if (bytes > s->path_buf_size) {
        g_free(s->path_buf);
        s->path_buf = g_malloc(bytes);
        s->path_buf_size = bytes;
    }
    vgl_dump_touch(s, data, bytes);
    address_space_read(&s->as, data, MEMTXATTRS_UNSPECIFIED, s->path_buf,
                       bytes);
    fl.edges = s->edges;
    g_array_set_size(fl.edges, 0);
    for (i = 0; i < 6; i++) {
        fl.m[i] = vgl_stf(s, ST_PATH_MATRIX + i);
    }
    fl.scale = vgl_stf(s, ST_PATH_SCALE);
    fl.bias = vgl_stf(s, ST_PATH_BIAS);
    vgl_flatten(s, &fl, s->path_buf, bytes, (pctrl >> 20) & 3);
    s->stat_edges += fl.edges->len;
    if (fl.edges->len &&
        vgl_pe_open(s, &pe, ry0, ry1, (uint64_t)(rx1 - rx0) * (ry1 - ry0))) {
        uint64_t px = vgl_raster(s, &pe, fl.edges, rx0, ry0, rx1, ry1,
                                 pctrl & PATH_EVEN_ODD, pctrl & 3);

        s->stat_pixels += px;
        vgl_cost(s, &pe, false, px, (uint64_t)(rx1 - rx0) * (ry1 - ry0),
                 fl.edges->len);
        vgl_pe_close(&pe);
    }
}

/* ---- submit capture (MIRAGE_VGL_DUMP) ---- */

/*
 * MIRAGE_VGL_DUMP=dir writes each submit to dir/NNNNNN.vgl while the
 * directory exists (create and remove it to start and stop capturing; at
 * most MIRAGE_VGL_DUMP_MAX files, default 1000) so that a host tool can
 * replay it against another version of this model. Every 4 KiB page the
 * submit reads or writes is captured the first time it is touched; writes
 * only go to pages captured before (the render target window is touched
 * when it is opened), so the first copy is the page before the submit.
 * File, little endian: "VGLD", version 1, sequence, command buffer
 * address and size (8-byte units), flags (bit 0 unnormalized), state
 * count, page count, the state registers before the submit, then per
 * page its address | 1 when the submit changed it, the page before and,
 * when changed, after.
 */
#define VGL_DUMP_PAGE       4096
#define VGL_DUMP_MAX_SPAN   (64 * 1024 * 1024)

static void vgl_dump_touch(VsiVgliteState *s, hwaddr addr, uint64_t len)
{
    hwaddr page;

    if (!s->dump_pages || !len || len > VGL_DUMP_MAX_SPAN) {
        return;
    }
    for (page = addr & ~(hwaddr)(VGL_DUMP_PAGE - 1); page < addr + len;
         page += VGL_DUMP_PAGE) {
        uint8_t *copy;

        if (g_hash_table_contains(s->dump_pages, GSIZE_TO_POINTER(page))) {
            continue;
        }
        copy = g_malloc(VGL_DUMP_PAGE);
        address_space_read(&s->as, page, MEMTXATTRS_UNSPECIFIED, copy,
                           VGL_DUMP_PAGE);
        g_hash_table_insert(s->dump_pages, GSIZE_TO_POINTER(page), copy);
    }
}

static int vgl_dump_page_cmp(const void *a, const void *b)
{
    gsize pa = GPOINTER_TO_SIZE(*(gpointer const *)a);
    gsize pb = GPOINTER_TO_SIZE(*(gpointer const *)b);

    return pa < pb ? -1 : pa > pb;
}

static void vgl_dump_put32(FILE *f, uint32_t v)
{
    uint8_t b[4];

    stl_le_p(b, v);
    fwrite(b, 1, 4, f);
}

static void vgl_dump_begin(VsiVgliteState *s, uint32_t *state)
{
    if (!s->dump_dir || s->dump_seq >= s->dump_max ||
        !g_file_test(s->dump_dir, G_FILE_TEST_IS_DIR)) {
        return;
    }
    memcpy(state, s->state, sizeof(s->state));
    s->dump_pages = g_hash_table_new_full(g_direct_hash, g_direct_equal,
                                          NULL, g_free);
}

static void vgl_dump_end(VsiVgliteState *s, const uint32_t *state)
{
    g_autofree char *path = NULL;
    g_autofree gpointer *pages = NULL;
    uint8_t *after;
    guint i, n;
    FILE *f;

    if (!s->dump_pages) {
        return;
    }
    path = g_strdup_printf("%s/%06u.vgl", s->dump_dir, s->dump_seq++);
    f = fopen(path, "wb");
    if (!f) {
        qemu_log_mask(LOG_UNIMP, "vsi.vglite: cannot write %s\n", path);
        g_hash_table_destroy(s->dump_pages);
        s->dump_pages = NULL;
        return;
    }
    pages = g_hash_table_get_keys_as_array(s->dump_pages, &n);
    qsort(pages, n, sizeof(*pages), vgl_dump_page_cmp);
    fwrite("VGLD", 1, 4, f);
    vgl_dump_put32(f, 1);
    vgl_dump_put32(f, s->dump_seq - 1);
    vgl_dump_put32(f, s->cmdbuf_addr);
    vgl_dump_put32(f, s->cmdbuf_size);
    vgl_dump_put32(f, s->unnormalized);
    vgl_dump_put32(f, VGL_STATE_COUNT);
    vgl_dump_put32(f, n);
    for (i = 0; i < VGL_STATE_COUNT; i++) {
        vgl_dump_put32(f, state[i]);
    }
    after = g_malloc(VGL_DUMP_PAGE);
    for (i = 0; i < n; i++) {
        gsize page = GPOINTER_TO_SIZE(pages[i]);
        const uint8_t *before = g_hash_table_lookup(s->dump_pages, pages[i]);
        bool changed;

        address_space_read(&s->as, page, MEMTXATTRS_UNSPECIFIED, after,
                           VGL_DUMP_PAGE);
        changed = memcmp(before, after, VGL_DUMP_PAGE) != 0;
        vgl_dump_put32(f, page | changed);
        fwrite(before, 1, VGL_DUMP_PAGE, f);
        if (changed) {
            fwrite(after, 1, VGL_DUMP_PAGE, f);
        }
    }
    g_free(after);
    fclose(f);
    g_hash_table_destroy(s->dump_pages);
    s->dump_pages = NULL;
}

/* ---- command processor ---- */

static uint32_t vgl_ldl(VsiVgliteState *s, hwaddr addr)
{
    vgl_dump_touch(s, addr, 4);
    return address_space_ldl_le(&s->as, addr, MEMTXATTRS_UNSPECIFIED, NULL);
}

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
        uint32_t w0 = vgl_ldl(s, addr);
        uint32_t w1 = vgl_ldl(s, addr + 4);

        switch (w0 >> 28) {
        case 0x0:                               /* END */
            if (s->timing) {
                s->pending_bits |= 1u << (w0 & 0x1F);
            } else {
                s->intr_status |= 1u << (w0 & 0x1F);
            }
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
                uint32_t v = i == 0 ? w1 : vgl_ldl(s, addr + 4 + 4 * i);
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

                vgl_dump_touch(s, addr + 8, sizeof(rect));
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

/* raises the END interrupts of the submits whose modelled time is over */
static void vgl_done_cb(void *opaque)
{
    VsiVgliteState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    unsigned i = 0, n;

    while (i < s->done_n && s->done_q[i].at <= now) {
        s->intr_status |= s->done_q[i].bits;
        i++;
    }
    n = s->done_n - i;
    memmove(s->done_q, s->done_q + i, n * sizeof(s->done_q[0]));
    s->done_n = n;
    if (n) {
        timer_mod(s->done_timer, s->done_q[0].at);
    }
    vgl_update_irq(s);
}

/* the submit just executed completes after its modelled time */
static void vgl_queue_done(VsiVgliteState *s)
{
    double mbps = s->mem_mbps ? s->mem_mbps : 400;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    /* fetching the command buffer, and the interrupt round trip */
    double ns = s->cost_ns + s->cmdbuf_size * 8 * 1000.0 / mbps + 2000.0;
    int64_t at = MAX(now, s->busy_until) + (int64_t)ns;

    s->busy_until = at;
    s->busy_ns += (uint64_t)ns;
    if (s->done_n == ARRAY_SIZE(s->done_q)) {
        /* more in flight than a driver keeps: merge into the last one */
        s->done_q[s->done_n - 1].at = at;
        s->done_q[s->done_n - 1].bits |= s->pending_bits;
    } else {
        s->done_q[s->done_n].at = at;
        s->done_q[s->done_n].bits = s->pending_bits;
        s->done_n++;
    }
    timer_mod(s->done_timer, s->done_q[0].at);
}

static void vgl_submit(VsiVgliteState *s)
{
    bool stats = trace_event_get_state_backends(TRACE_VSI_VGLITE_STATS);
    int64_t t0 = stats ? get_clock() : 0;
    g_autofree uint32_t *state = NULL;

    trace_vsi_vglite_submit(s->cmdbuf_addr, s->cmdbuf_size);
    s->stat_paths = s->stat_rects = 0;
    s->stat_edges = s->stat_pixels = 0;
    if (s->dump_dir) {
        state = g_new(uint32_t, VGL_STATE_COUNT);
        vgl_dump_begin(s, state);
    }
    s->cost_ns = 0;
    s->pending_bits = 0;
    vgl_exec(s, s->cmdbuf_addr, (uint64_t)s->cmdbuf_size * 8, 0);
    if (s->timing) {
        vgl_queue_done(s);
    }
    vgl_dump_end(s, state);
    if (stats) {
        trace_vsi_vglite_stats(s->stat_paths, s->stat_rects, s->stat_edges,
                               s->stat_pixels, (get_clock() - t0) / 1000);
    }
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
        if (s->timing &&
            qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) < s->busy_until) {
            return 0x7FFFFFFF & ~0x0B05u;   /* front end and engines busy */
        }
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
            s->done_n = 0;
            s->busy_until = 0;
            if (s->done_timer) {
                timer_del(s->done_timer);
            }
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
    s->done_n = 0;
    s->busy_until = 0;
    if (s->done_timer) {
        timer_del(s->done_timer);
    }
}

static void vgl_realize(DeviceState *dev, Error **errp)
{
    VsiVgliteState *s = VSI_VGLITE(dev);
    const char *dir;

    if (!s->downstream) {
        error_setg(errp, "vsi.vglite: 'downstream' link not set");
        return;
    }
    address_space_init(&s->as, s->downstream, "vsi.vglite-as");
    if (s->timing) {
        s->done_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, vgl_done_cb, s);
    }
    /* 0: half the host CPUs, up to four */
    s->nthreads = s->render_threads ? s->render_threads :
                  MIN(4, MAX(1, g_get_num_processors() / 2));
    dir = g_getenv("MIRAGE_VGL_DUMP");
    if (dir && *dir) {
        const char *max = g_getenv("MIRAGE_VGL_DUMP_MAX");
        unsigned long n;

        s->dump_dir = g_strdup(dir);
        s->dump_max = max && !qemu_strtoul(max, NULL, 0, &n) ? n : 1000;
    }
}

static void vgl_init(Object *obj)
{
    VsiVgliteState *s = VSI_VGLITE(obj);

    memory_region_init_io(&s->iomem, obj, &vgl_ops, s, "vsi.vglite", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->edges = g_array_new(false, false, sizeof(VglEdge));
}

static void vgl_finalize(Object *obj)
{
    VsiVgliteState *s = VSI_VGLITE(obj);
    unsigned i;

    vgl_workers_stop(s);
    if (s->done_timer) {
        timer_free(s->done_timer);
    }
    g_array_free(s->edges, true);
    g_free(s->scratch);
    g_free(s->band_scratch);
    g_free(s->path_buf);
    for (i = 0; i < VGL_TEXEL_LUTS; i++) {
        g_free(s->texel_lut[i]);
    }
    g_free(s->dump_dir);
}

static const Property vgl_props[] = {
    DEFINE_PROP_LINK("downstream", VsiVgliteState, downstream,
                     TYPE_MEMORY_REGION, MemoryRegion *),
    DEFINE_PROP_UINT32("chip-id", VsiVgliteState, chip_id, 0x265),
    DEFINE_PROP_UINT32("chip-rev", VsiVgliteState, chip_rev, 0x2002),
    DEFINE_PROP_UINT32("chip-cid", VsiVgliteState, chip_cid, 0x424),
    DEFINE_PROP_UINT32("chip-date", VsiVgliteState, chip_date, 0),
    DEFINE_PROP_UINT32("eco-id", VsiVgliteState, eco_id, 0),
    DEFINE_PROP_BOOL("unnormalized", VsiVgliteState, unnormalized, false),
    DEFINE_PROP_UINT32("render-threads", VsiVgliteState, render_threads, 0),
    DEFINE_PROP_BOOL("timing", VsiVgliteState, timing, false),
    DEFINE_PROP_UINT32("gpu-mhz", VsiVgliteState, gpu_mhz, 208),
    DEFINE_PROP_UINT32("mem-mbps", VsiVgliteState, mem_mbps, 400),
    DEFINE_PROP_UINT32("op-cycles", VsiVgliteState, op_cycles, 150),
};

static void vgl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    vgl_init_luts();
    dc->realize = vgl_realize;
    device_class_set_props(dc, vgl_props);
    device_class_set_legacy_reset(dc, vgl_reset);
}

static const TypeInfo vgl_info = {
    .name = TYPE_VSI_VGLITE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(VsiVgliteState),
    .instance_init = vgl_init,
    .instance_finalize = vgl_finalize,
    .class_init = vgl_class_init,
};

static void vgl_register_types(void)
{
    type_register_static(&vgl_info);
}

type_init(vgl_register_types)
