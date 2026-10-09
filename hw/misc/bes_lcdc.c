/*
 * BES LCDC (LCD controller) for best1503 @ 0x40200000, driving a QSPI
 * (DBI-style) AMOLED panel through its serial panel unit (SPU).
 *
 * Everything below is taken from the AP firmware's LCDC HAL and
 * framebuffer driver (vela_ap.bin v3.101.043):
 *   - IRQ: NVIC 67 (sub_C231DE4); handler sub_C225644 reads ISR +0x1C4,
 *     reports frame done to the display PM thread and clears ISR by
 *     writing 0 (sub_C232218 / sub_C232280)
 *   - IRQ enable +0x1C0: frame done bit 21 (bit 22 in the other interface
 *     mode), bit 23 (sub_C232224 / sub_C232258)
 *   - ISR decoding in the frame-done path (sub_C2263C0): frame end is
 *     bit 21 (bit 22 in the other mode, sub_C232294); the frame only counts
 *     as transferred (driver clears its tx-busy flag, retires the panned
 *     buffer and unblocks FBIO_WAITFORVSYNC) when a completion status bit
 *     is also set: bit 31 / 27 (sub_C2322AC) or bit 30 / 26 (sub_C2322B8).
 *     Bits 29 / 25 / 16 are reported as configuration errors
 *     (sub_C2322C4). Which completion bit belongs to which output path is
 *     not visible in the firmware; the model reports bit 27 for the panel
 *     transfer.
 *   - frame start: +0x224 bit0 (sub_C232364)
 *   - SPU command channel: command word at +0x284 = instr << 24 |
 *     dcs << 8 (QSPI panel framing, instr 0x02 write / 0x03 read),
 *     parameters MSB-first from +0x288, trigger +0x280, completion ISR
 *     bit 18 polled by sub_C232578 / sub_C2323E0
 *   - layers (register pointer tables 0x2CCDADCC..0x2CCDAE44):
 *       layer  ctrl   fbaddr stride pos    size
 *         0    0x264  0x0F4  0x0FC  0x100  0x104
 *         1    0x190  0x0C0  0x0E0  0x0E8  0x0EC
 *         2    0x080  0x034  0x03C  0x040  0x044
 *     ctrl bit0 = enable, [11:8] = pixel format (hal_lcdc_lpfc_fmt:
 *     FB_FMT_RGB16_565 -> 0, RGB24 -> 2, RGB32 -> 4), stride bits[15:0],
 *     pos/size x | y << 16 (12 bits each)
 *   - SPU output: ctrl +0x188, size +0x118 (w | h << 16), default colour
 *     +0x124
 * The panel side follows the MIPI DCS commands seen in the SPU stream:
 * SLPOUT/SLPIN, DISPON/DISPOFF, TEON/TEOFF, CASET/RASET. With TE enabled
 * and the panel awake, it pulses its TE output at the refresh rate; the
 * board wires TE to a GPIO interrupt that paces frame starts (display PM
 * thread sub_C224ECC).
 *
 * A started frame is composed from the enabled layers into the panel's
 * frame memory (GRAM) at the window set by the last CASET/RASET, and
 * completes after the QSPI transfer time with the frame-done IRQ.  The
 * QEMU graphic console shows what the panel emits: the GRAM while the
 * panel is out of sleep with the display on, black otherwise (an AMOLED
 * panel in SLPIN / DISPOFF is dark; the firmware sends both when the
 * screen times out).
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "system/address-spaces.h"
#include "ui/console.h"
#include "hw/misc/bes_lcdc.h"
#include "trace.h"

#define LCDC_REG_COUNT      (0x1000 / 4)

#define LCDC_IRQ_ENA        0x1C0
#define LCDC_IRQ_ISR        0x1C4
#define LCDC_FRAME_START    0x224
#define LCDC_SPU_CTRL       0x280
#define LCDC_SPU_CMD        0x284
#define LCDC_SPU_DATA0      0x288
#define LCDC_SPU_OUT_SIZE   0x118
#define LCDC_SPU_BGCOLOR    0x124

#define LCDC_ISR_SPU_DONE   (1u << 18)
#define LCDC_ISR_FRAME_DONE (3u << 21)
#define LCDC_ISR_XFER_DONE  (1u << 27)

#define DCS_SLPIN           0x10
#define DCS_SLPOUT          0x11
#define DCS_DISPOFF         0x28
#define DCS_DISPON          0x29
#define DCS_CASET           0x2A
#define DCS_RASET           0x2B
#define DCS_TEOFF           0x34
#define DCS_TEON            0x35

#define TE_PULSE_NS         (100 * 1000)

struct BesLcdcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq te;
    QemuConsole *con;

    uint32_t regs[LCDC_REG_COUNT];

    /* panel */
    uint32_t panel_w;
    uint32_t panel_h;
    uint32_t refresh_hz;
    uint32_t frame_us;          /* QSPI transfer time of one frame */
    bool sleeping;
    bool display_on;
    bool te_on;
    uint16_t win_x0, win_x1, win_y0, win_y1;
    uint32_t *gram;             /* panel frame memory, panel_w x panel_h */
    QEMUTimer *te_timer;
    bool te_high;
    QEMUTimer *frame_timer;
    bool frame_busy;
};

typedef struct {
    uint16_t ctrl, fb, stride, pos, size;
} BesLcdcLayerRegs;

static const BesLcdcLayerRegs lcdc_layers[] = {
    { 0x264, 0x0F4, 0x0FC, 0x100, 0x104 },
    { 0x190, 0x0C0, 0x0E0, 0x0E8, 0x0EC },
    { 0x080, 0x034, 0x03C, 0x040, 0x044 },
};

static uint32_t lcdc_reg(BesLcdcState *s, hwaddr off)
{
    return s->regs[off / 4];
}

static void bes_lcdc_update_irq(BesLcdcState *s)
{
    qemu_set_irq(s->irq, !!(lcdc_reg(s, LCDC_IRQ_ISR) &
                            lcdc_reg(s, LCDC_IRQ_ENA)));
}

/* ---- panel: visible output ---- */

static bool bes_lcdc_panel_visible(BesLcdcState *s)
{
    return !s->sleeping && s->display_on;
}

/* copy a GRAM rectangle to the console, or black while the panel is dark */
static void bes_lcdc_show(BesLcdcState *s, unsigned x0, unsigned y0,
                          unsigned w, unsigned h)
{
    DisplaySurface *surf = qemu_console_surface(s->con);
    bool visible = bes_lcdc_panel_visible(s);
    unsigned y;

    if (!surf || surface_bits_per_pixel(surf) != 32) {
        return;
    }
    for (y = y0; y < y0 + h; y++) {
        uint8_t *dst = (uint8_t *)surface_data(surf) +
                       y * surface_stride(surf) + x0 * 4;

        if (visible) {
            memcpy(dst, &s->gram[y * s->panel_w + x0], w * 4);
        } else {
            memset(dst, 0, w * 4);
        }
    }
    qemu_console_update(s->con, x0, y0, w, h);
}

/* ---- panel: TE output ---- */

static bool bes_lcdc_te_running(BesLcdcState *s)
{
    return s->te_on && !s->sleeping && s->refresh_hz;
}

static void bes_lcdc_te_tick(void *opaque)
{
    BesLcdcState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (s->te_high) {
        s->te_high = false;
        qemu_irq_lower(s->te);
        timer_mod(s->te_timer, now + NANOSECONDS_PER_SECOND / s->refresh_hz
                  - TE_PULSE_NS);
    } else if (bes_lcdc_te_running(s)) {
        s->te_high = true;
        qemu_irq_raise(s->te);
        timer_mod(s->te_timer, now + TE_PULSE_NS);
    }
}

static void bes_lcdc_te_update(BesLcdcState *s)
{
    if (bes_lcdc_te_running(s)) {
        if (!timer_pending(s->te_timer)) {
            timer_mod(s->te_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      NANOSECONDS_PER_SECOND / s->refresh_hz);
        }
    } else if (!s->te_high) {
        timer_del(s->te_timer);
    }
}

static void bes_lcdc_panel_cmd(BesLcdcState *s, uint32_t cmdword)
{
    uint8_t instr = cmdword >> 24;
    uint8_t dcs = cmdword >> 8;
    uint32_t p = lcdc_reg(s, LCDC_SPU_DATA0);
    bool was_visible = bes_lcdc_panel_visible(s);

    trace_bes_lcdc_panel_cmd(instr, dcs, p);
    if (instr != 0x02) {
        return;                         /* reads and pixel writes */
    }
    switch (dcs) {
    case DCS_SLPIN:
        s->sleeping = true;
        break;
    case DCS_SLPOUT:
        s->sleeping = false;
        break;
    case DCS_DISPOFF:
        s->display_on = false;
        break;
    case DCS_DISPON:
        s->display_on = true;
        break;
    case DCS_TEOFF:
        s->te_on = false;
        break;
    case DCS_TEON:
        s->te_on = true;
        break;
    case DCS_CASET:
        s->win_x0 = p >> 16;
        s->win_x1 = p & 0xFFFF;
        break;
    case DCS_RASET:
        s->win_y0 = p >> 16;
        s->win_y1 = p & 0xFFFF;
        break;
    }
    bes_lcdc_te_update(s);
    if (bes_lcdc_panel_visible(s) != was_visible) {
        bes_lcdc_show(s, 0, 0, s->panel_w, s->panel_h);
    }
}

/* ---- frame composition ---- */

static uint32_t lcdc_pixel(const uint8_t *p, unsigned fmt)
{
    switch (fmt) {
    case 0: {                           /* RGB565 */
        uint16_t v = lduw_le_p(p);
        uint32_t r = (v >> 11) & 0x1F, g = (v >> 5) & 0x3F, b = v & 0x1F;

        return 0xFF000000 | (r << 19 | (r >> 2) << 16) |
               (g << 10 | (g >> 4) << 8) | (b << 3 | b >> 2);
    }
    case 2:                             /* RGB888, B G R in memory */
        return 0xFF000000 | p[2] << 16 | p[1] << 8 | p[0];
    case 4:                             /* ARGB8888 */
    default:
        return ldl_le_p(p);
    }
}

static unsigned lcdc_bpp(unsigned fmt)
{
    switch (fmt) {
    case 0:
        return 2;
    case 2:
        return 3;
    case 4:
        return 4;
    default:
        return 0;
    }
}

static void bes_lcdc_compose(BesLcdcState *s)
{
    uint32_t out = lcdc_reg(s, LCDC_SPU_OUT_SIZE);
    uint32_t ow = out & 0xFFF, oh = (out >> 16) & 0xFFF;
    uint32_t bg = lcdc_reg(s, LCDC_SPU_BGCOLOR) | 0xFF000000;
    uint32_t *line;
    uint8_t *src;
    unsigned x, y, l;

    if (!ow || !oh) {
        ow = s->win_x1 - s->win_x0 + 1;
        oh = s->win_y1 - s->win_y0 + 1;
    }
    ow = MIN(ow, s->panel_w - MIN(s->win_x0, s->panel_w));
    oh = MIN(oh, s->panel_h - MIN(s->win_y0, s->panel_h));
    if (!ow || !oh) {
        return;
    }

    line = g_new(uint32_t, ow);
    src = g_malloc(ow * 4);
    for (y = 0; y < oh; y++) {
        for (x = 0; x < ow; x++) {
            line[x] = bg;
        }
        for (l = 0; l < ARRAY_SIZE(lcdc_layers); l++) {
            const BesLcdcLayerRegs *r = &lcdc_layers[l];
            uint32_t ctrl = lcdc_reg(s, r->ctrl);
            unsigned fmt = (ctrl >> 8) & 0xF, bpp = lcdc_bpp(fmt);
            uint32_t pos = lcdc_reg(s, r->pos), size = lcdc_reg(s, r->size);
            unsigned lx = pos & 0xFFF, ly = (pos >> 16) & 0xFFF;
            unsigned lw = size & 0xFFF, lh = (size >> 16) & 0xFFF;
            uint32_t stride = lcdc_reg(s, r->stride) & 0xFFFF;
            unsigned x0, x1;

            if (!(ctrl & 1) || !bpp || y < ly || y >= ly + lh ||
                lx >= ow) {
                continue;
            }
            x0 = lx;
            x1 = MIN(lx + lw, ow);
            address_space_read(&address_space_memory,
                               lcdc_reg(s, r->fb) + (y - ly) * stride,
                               MEMTXATTRS_UNSPECIFIED, src,
                               (x1 - x0) * bpp);
            for (x = x0; x < x1; x++) {
                uint32_t px = lcdc_pixel(src + (x - x0) * bpp, fmt);
                uint32_t a = px >> 24;

                if (a == 0xFF) {
                    line[x] = px;
                } else if (a) {
                    uint32_t d = line[x], o = 0;
                    int sh;

                    for (sh = 0; sh < 24; sh += 8) {
                        uint32_t cs = (px >> sh) & 0xFF;
                        uint32_t cd = (d >> sh) & 0xFF;

                        o |= ((cs * a + cd * (255 - a)) / 255) << sh;
                    }
                    line[x] = 0xFF000000 | o;
                }
            }
        }
        memcpy(&s->gram[(s->win_y0 + y) * s->panel_w + s->win_x0],
               line, ow * 4);
    }
    g_free(src);
    g_free(line);
    bes_lcdc_show(s, s->win_x0, s->win_y0, ow, oh);
    trace_bes_lcdc_frame(ow, oh, s->win_x0, s->win_y0);
}

static void bes_lcdc_frame_done(void *opaque)
{
    BesLcdcState *s = opaque;
    uint32_t done = lcdc_reg(s, LCDC_IRQ_ENA) & LCDC_ISR_FRAME_DONE;

    s->frame_busy = false;
    s->regs[LCDC_FRAME_START / 4] &= ~1u;
    s->regs[LCDC_IRQ_ISR / 4] |= (done ? done : (1u << 21)) |
                                 LCDC_ISR_XFER_DONE;
    bes_lcdc_update_irq(s);
}

static void bes_lcdc_frame_start(BesLcdcState *s)
{
    if (s->frame_busy) {
        qemu_log_mask(LOG_GUEST_ERROR, "bes.lcdc: frame start while busy\n");
        return;
    }
    s->frame_busy = true;
    if (s->display_on && !s->sleeping) {
        bes_lcdc_compose(s);
    }
    timer_mod(s->frame_timer,
              qemu_clock_get_us(QEMU_CLOCK_VIRTUAL) + s->frame_us);
}

/* ---- registers ---- */

static uint64_t bes_lcdc_read(void *opaque, hwaddr offset, unsigned size)
{
    BesLcdcState *s = opaque;

    return s->regs[offset / 4];
}

static void bes_lcdc_write(void *opaque, hwaddr offset, uint64_t val,
                           unsigned size)
{
    BesLcdcState *s = opaque;

    s->regs[offset / 4] = val;

    switch (offset) {
    case LCDC_SPU_CTRL:
        if (val & 3) {
            /* register access to the panel: completes instantly */
            bes_lcdc_panel_cmd(s, lcdc_reg(s, LCDC_SPU_CMD));
            s->regs[LCDC_IRQ_ISR / 4] |= LCDC_ISR_SPU_DONE;
        }
        break;
    case LCDC_FRAME_START:
        if (val & 1) {
            bes_lcdc_frame_start(s);
        }
        break;
    }
    bes_lcdc_update_irq(s);
}

static const MemoryRegionOps bes_lcdc_ops = {
    .read = bes_lcdc_read,
    .write = bes_lcdc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void bes_lcdc_invalidate(void *opaque)
{
    BesLcdcState *s = opaque;

    qemu_console_update(s->con, 0, 0, s->panel_w, s->panel_h);
}

static const GraphicHwOps bes_lcdc_gfx_ops = {
    .invalidate = bes_lcdc_invalidate,
};

static void bes_lcdc_reset(DeviceState *dev)
{
    BesLcdcState *s = BES_LCDC(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->sleeping = true;
    s->display_on = false;
    s->te_on = false;
    s->win_x0 = 0;
    s->win_y0 = 0;
    s->win_x1 = s->panel_w - 1;
    s->win_y1 = s->panel_h - 1;
    s->frame_busy = false;
    s->te_high = false;
    memset(s->gram, 0, s->panel_w * s->panel_h * 4);
    bes_lcdc_show(s, 0, 0, s->panel_w, s->panel_h);
    timer_del(s->te_timer);
    timer_del(s->frame_timer);
    qemu_irq_lower(s->te);
}

static void bes_lcdc_realize(DeviceState *dev, Error **errp)
{
    BesLcdcState *s = BES_LCDC(dev);

    s->te_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, bes_lcdc_te_tick, s);
    s->frame_timer = timer_new_us(QEMU_CLOCK_VIRTUAL, bes_lcdc_frame_done, s);
    s->con = qemu_graphic_console_create(dev, 0, &bes_lcdc_gfx_ops, s);
    qemu_console_resize(s->con, s->panel_w, s->panel_h);
    s->gram = g_new0(uint32_t, s->panel_w * s->panel_h);
}

static void bes_lcdc_init(Object *obj)
{
    BesLcdcState *s = BES_LCDC(obj);

    memory_region_init_io(&s->iomem, obj, &bes_lcdc_ops, s, "bes.lcdc",
                          0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_out_named(DEVICE(obj), &s->te, "te", 1);
}

static const Property bes_lcdc_props[] = {
    DEFINE_PROP_UINT32("panel-width", BesLcdcState, panel_w, 336),
    DEFINE_PROP_UINT32("panel-height", BesLcdcState, panel_h, 480),
    DEFINE_PROP_UINT32("refresh-hz", BesLcdcState, refresh_hz, 60),
    DEFINE_PROP_UINT32("frame-us", BesLcdcState, frame_us, 8000),
};

static void bes_lcdc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = bes_lcdc_realize;
    device_class_set_props(dc, bes_lcdc_props);
    device_class_set_legacy_reset(dc, bes_lcdc_reset);
}

static const TypeInfo bes_lcdc_info = {
    .name = TYPE_BES_LCDC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BesLcdcState),
    .instance_init = bes_lcdc_init,
    .class_init = bes_lcdc_class_init,
};

static void bes_lcdc_register_types(void)
{
    type_register_static(&bes_lcdc_info);
}

type_init(bes_lcdc_register_types)
