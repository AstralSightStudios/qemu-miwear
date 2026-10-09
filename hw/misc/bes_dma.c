/*
 * BES general purpose DMA controller (best1503, two instances)
 *
 * The register file is ARM PL080 compatible plus a BES 2D-addressing
 * extension. Everything below is taken from the AP firmware HAL
 * (vela_ap.bin v3.101.043, hal_dma.c):
 *   - instance bases 0x40120000 / 0x40130000, 8 channels each
 *     (table 0x2CCDA91C, channel counts 0x2CCDA8FC)
 *   - NVIC lines 3 / 4 (hal_dma_open sub_C22E8D0, u16 table 0x2CCDA918)
 *   - global: IntStatus 0x000, IntTCStatus 0x004, IntTCClear 0x008,
 *     IntErrorStatus 0x00C, IntErrClr 0x010, EnbldChns 0x01C,
 *     Configuration 0x030 (irq handler sub_C22DF70/sub_C22DF0C,
 *     hal_dma_chan_busy sub_C22E7D0, hal_dma_open writes 0x030 |= 9)
 *   - per channel @0x100 + ch * 0x20: SrcAddr, DestAddr, LLI, Control,
 *     Configuration (hal_dma_sg_2d_start sub_C22E0C4); the LLI item is
 *     {src, dst, next, control} (desc builder sub_C22DFA8)
 *   - Control: TransferSize[11:0], SBSize[14:12], DBSize[17:15],
 *     SWidth[20:18], DWidth[23:21], SI bit26, DI bit27, I bit31
 *   - Configuration: E bit0, SrcPeriph[5:1], DestPeriph[10:6],
 *     FlowCntrl[13:11], IE bit14, ITC bit15, A bit17 (polled by
 *     hal_dma_stop sub_C22E460 after setting H bit18)
 *   - 2D extension @0x200 + ch * 0x20: words 0..3 hold the source and
 *     destination {count[10:0], stride[31:11]} pairs, word 4 bit0/bit1
 *     enable source/destination 2D mode (sub_C22E0C4)
 *
 * Peripheral flow (FlowCntrl 1-3) is paced by request lines: each
 * peripheral drives its line (gpio input n = DMA request number from the
 * firmware periph table 0x2CCDA974 / 0x2CCDA924) high while it can accept
 * or supply data. Transfers run element by element while the request is
 * asserted, so a peripheral FIFO that drains mid-transfer stalls the
 * channel until it raises the request again.
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "system/address-spaces.h"
#include "hw/misc/bes_dma.h"
#include "trace.h"

#define DMA_CONF_E          (1u << 0)

#define DMA_CCONF_E         (1u << 0)
#define DMA_CCONF_IE        (1u << 14)
#define DMA_CCONF_ITC       (1u << 15)
#define DMA_CCONF_A         (1u << 17)
#define DMA_CCONF_H         (1u << 18)

#define DMA_CCTRL_SI        (1u << 26)
#define DMA_CCTRL_DI        (1u << 27)
#define DMA_CCTRL_I         (1u << 31)

#define DMA_EXT_SRC_2D      (1u << 0)
#define DMA_EXT_DST_2D      (1u << 1)

static uint8_t bes_dma_tc_mask(BesDmaState *s)
{
    uint8_t m = 0;
    int c;

    for (c = 0; c < BES_DMA_CHANNELS; c++) {
        if (s->chan[c].conf & DMA_CCONF_ITC) {
            m |= 1 << c;
        }
    }
    return m;
}

static uint8_t bes_dma_err_mask(BesDmaState *s)
{
    uint8_t m = 0;
    int c;

    for (c = 0; c < BES_DMA_CHANNELS; c++) {
        if (s->chan[c].conf & DMA_CCONF_IE) {
            m |= 1 << c;
        }
    }
    return m;
}

static void bes_dma_update_irq(BesDmaState *s)
{
    uint8_t st = (s->tc_raw & bes_dma_tc_mask(s)) |
                 (s->err_raw & bes_dma_err_mask(s));

    qemu_set_irq(s->irq, st != 0);
}

static bool bes_dma_chan_ready(BesDmaState *s, BesDmaChan *ch)
{
    uint32_t req = s->hw_req | s->soft_req;
    unsigned flow = extract32(ch->conf, 11, 3);
    unsigned src_id = extract32(ch->conf, 1, 5);
    unsigned dst_id = extract32(ch->conf, 6, 5);

    switch (flow) {
    case 0:     /* memory to memory */
        return true;
    case 1:     /* memory to peripheral */
        return req & (1u << dst_id);
    case 2:     /* peripheral to memory */
        return req & (1u << src_id);
    case 3:     /* peripheral to peripheral */
        return (req & (1u << src_id)) && (req & (1u << dst_id));
    default:
        return false;
    }
}

/* finish the current LLI item: chain to the next one or stop the channel */
static void bes_dma_item_done(BesDmaState *s, int c)
{
    BesDmaChan *ch = &s->chan[c];
    uint32_t next = ch->lli & ~3u;
    bool irq = ch->ctrl & DMA_CCTRL_I;

    if (next) {
        ch->src = address_space_ldl_le(&s->downstream_as, next,
                                       MEMTXATTRS_UNSPECIFIED, NULL);
        ch->dst = address_space_ldl_le(&s->downstream_as, next + 4,
                                       MEMTXATTRS_UNSPECIFIED, NULL);
        ch->lli = address_space_ldl_le(&s->downstream_as, next + 8,
                                       MEMTXATTRS_UNSPECIFIED, NULL);
        ch->ctrl = address_space_ldl_le(&s->downstream_as, next + 12,
                                        MEMTXATTRS_UNSPECIFIED, NULL);
    } else {
        ch->conf &= ~DMA_CCONF_E;
    }
    if (irq) {
        s->tc_raw |= 1 << c;
    }
    trace_bes_dma_item_done(c, next, irq);
}

/* move one element; returns false if the channel cannot make progress */
static bool bes_dma_step(BesDmaState *s, int c)
{
    BesDmaChan *ch = &s->chan[c];
    uint32_t size = ch->ctrl & 0xFFF;
    unsigned sw = 1u << extract32(ch->ctrl, 18, 3);
    unsigned dw = 1u << extract32(ch->ctrl, 21, 3);
    unsigned xsize, n;
    uint8_t buf[4];

    if (size == 0) {
        bes_dma_item_done(s, c);
        return true;
    }
    if (sw > 4 || dw > 4) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "bes.dma: chan %d: invalid width sw=%u dw=%u\n",
                      c, sw, dw);
        s->err_raw |= 1 << c;
        ch->conf &= ~DMA_CCONF_E;
        return false;
    }
    xsize = MAX(sw, dw);
    if ((size * sw) % dw) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "bes.dma: chan %d: size %u x sw %u not a multiple "
                      "of dw %u\n", c, size, sw, dw);
        s->err_raw |= 1 << c;
        ch->conf &= ~DMA_CCONF_E;
        return false;
    }
    for (n = 0; n < xsize; n += sw) {
        address_space_read(&s->downstream_as, ch->src,
                           MEMTXATTRS_UNSPECIFIED, buf + n, sw);
        if (ch->ctrl & DMA_CCTRL_SI) {
            ch->src += sw;
        }
    }
    for (n = 0; n < xsize; n += dw) {
        address_space_write(&s->downstream_as, ch->dst,
                            MEMTXATTRS_UNSPECIFIED, buf + n, dw);
        if (ch->ctrl & DMA_CCTRL_DI) {
            ch->dst += dw;
        }
    }
    size -= xsize / sw;
    ch->ctrl = (ch->ctrl & ~0xFFFu) | size;
    if (size == 0) {
        bes_dma_item_done(s, c);
    }
    return true;
}

static void bes_dma_run(BesDmaState *s)
{
    int c;

    if (!(s->conf & DMA_CONF_E)) {
        return;
    }
    /* request lines toggle synchronously while a transfer touches the
     * peripheral; nested calls only flag another pass */
    if (s->running) {
        s->rerun = true;
        return;
    }
    s->running = true;
    do {
        s->rerun = false;
        for (c = 0; c < BES_DMA_CHANNELS; c++) {
            BesDmaChan *ch = &s->chan[c];

            while ((ch->conf & (DMA_CCONF_E | DMA_CCONF_H)) == DMA_CCONF_E) {
                unsigned flow = extract32(ch->conf, 11, 3);

                if (flow >= 4) {
                    qemu_log_mask(LOG_UNIMP, "bes.dma: chan %d: peripheral "
                                  "flow control (%u) not modelled\n",
                                  c, flow);
                    break;
                }
                if (ch->ext[4] & (DMA_EXT_SRC_2D | DMA_EXT_DST_2D)) {
                    qemu_log_mask(LOG_UNIMP, "bes.dma: chan %d: 2D mode "
                                  "0x%x not modelled\n", c, ch->ext[4]);
                    break;
                }
                if (!bes_dma_chan_ready(s, ch)) {
                    break;
                }
                if (!bes_dma_step(s, c)) {
                    break;
                }
            }
        }
    } while (s->rerun);
    s->running = false;
    bes_dma_update_irq(s);
}

static void bes_dma_set_req(void *opaque, int line, int level)
{
    BesDmaState *s = BES_DMA(opaque);
    uint32_t old = s->hw_req;

    s->hw_req = deposit32(s->hw_req, line, 1, level != 0);
    if (level && !(old & (1u << line))) {
        bes_dma_run(s);
    }
}

static uint32_t bes_dma_enabled(BesDmaState *s)
{
    uint32_t v = 0;
    int c;

    for (c = 0; c < BES_DMA_CHANNELS; c++) {
        if (s->chan[c].conf & DMA_CCONF_E) {
            v |= 1u << c;
        }
    }
    return v;
}

static uint64_t bes_dma_read(void *opaque, hwaddr offset, unsigned size)
{
    BesDmaState *s = BES_DMA(opaque);
    uint8_t tc = s->tc_raw & bes_dma_tc_mask(s);
    uint8_t err = s->err_raw & bes_dma_err_mask(s);
    uint64_t v = 0;

    if (offset >= 0x100 && offset < 0x200) {
        BesDmaChan *ch = &s->chan[(offset - 0x100) >> 5];

        switch (offset & 0x1F) {
        case 0x00:
            v = ch->src;
            break;
        case 0x04:
            v = ch->dst;
            break;
        case 0x08:
            v = ch->lli;
            break;
        case 0x0C:
            v = ch->ctrl;
            break;
        case 0x10:
            /* A (bit17): transfers complete synchronously, so the channel
             * FIFO never holds data when software looks */
            v = ch->conf & ~DMA_CCONF_A;
            break;
        default:
            goto bad;
        }
        return v;
    }
    if (offset >= 0x200 && offset < 0x300) {
        BesDmaChan *ch = &s->chan[(offset - 0x200) >> 5];
        unsigned w = (offset & 0x1F) >> 2;

        if (w >= ARRAY_SIZE(ch->ext)) {
            goto bad;
        }
        return ch->ext[w];
    }

    switch (offset) {
    case 0x000:
        v = tc | err;
        break;
    case 0x004:
        v = tc;
        break;
    case 0x00C:
        v = err;
        break;
    case 0x014:
        v = s->tc_raw;
        break;
    case 0x018:
        v = s->err_raw;
        break;
    case 0x01C:
        v = bes_dma_enabled(s);
        break;
    case 0x020:
    case 0x024:
    case 0x028:
    case 0x02C:
        v = s->soft_req;
        break;
    case 0x030:
        v = s->conf;
        break;
    case 0x034:
        v = s->sync;
        break;
    default:
        goto bad;
    }
    return v;

bad:
    qemu_log_mask(LOG_UNIMP, "bes.dma: read of unknown offset 0x%03x\n",
                  (unsigned)offset);
    return 0;
}

static void bes_dma_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    BesDmaState *s = BES_DMA(opaque);

    trace_bes_dma_write((unsigned)offset, (uint32_t)value);

    if (offset >= 0x100 && offset < 0x200) {
        BesDmaChan *ch = &s->chan[(offset - 0x100) >> 5];

        switch (offset & 0x1F) {
        case 0x00:
            ch->src = value;
            break;
        case 0x04:
            ch->dst = value;
            break;
        case 0x08:
            ch->lli = value;
            break;
        case 0x0C:
            ch->ctrl = value;
            break;
        case 0x10:
            ch->conf = value & ~DMA_CCONF_A;
            break;
        default:
            goto bad;
        }
        bes_dma_run(s);
        bes_dma_update_irq(s);
        return;
    }
    if (offset >= 0x200 && offset < 0x300) {
        BesDmaChan *ch = &s->chan[(offset - 0x200) >> 5];
        unsigned w = (offset & 0x1F) >> 2;

        if (w >= ARRAY_SIZE(ch->ext)) {
            goto bad;
        }
        ch->ext[w] = value;
        return;
    }

    switch (offset) {
    case 0x008:
        s->tc_raw &= ~value;
        break;
    case 0x010:
        s->err_raw &= ~value;
        break;
    case 0x020:
    case 0x024:
    case 0x028:
    case 0x02C:
        /* software requests are single-shot: one pass, then released */
        s->soft_req |= value & 0xFFFF;
        bes_dma_run(s);
        s->soft_req = 0;
        break;
    case 0x030:
        s->conf = value;
        bes_dma_run(s);
        break;
    case 0x034:
        s->sync = value;
        break;
    default:
        goto bad;
    }
    bes_dma_update_irq(s);
    return;

bad:
    qemu_log_mask(LOG_UNIMP, "bes.dma: write of unknown offset 0x%03x "
                  "= 0x%08x\n", (unsigned)offset, (uint32_t)value);
}

static const MemoryRegionOps bes_dma_ops = {
    .read = bes_dma_read,
    .write = bes_dma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void bes_dma_reset(DeviceState *dev)
{
    BesDmaState *s = BES_DMA(dev);

    s->conf = 0;
    s->sync = 0;
    s->tc_raw = 0;
    s->err_raw = 0;
    s->soft_req = 0;
    s->running = false;
    s->rerun = false;
    memset(s->chan, 0, sizeof(s->chan));
}

static void bes_dma_realize(DeviceState *dev, Error **errp)
{
    BesDmaState *s = BES_DMA(dev);

    if (!s->downstream) {
        error_setg(errp, "bes.dma: 'downstream' link not set");
        return;
    }
    address_space_init(&s->downstream_as, s->downstream, "bes.dma-ds");
}

static void bes_dma_init(Object *obj)
{
    BesDmaState *s = BES_DMA(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &bes_dma_ops, s, "bes.dma",
                          0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_in_named(DEVICE(obj), bes_dma_set_req, "dma-req",
                            BES_DMA_REQ_LINES);
}

static const Property bes_dma_props[] = {
    DEFINE_PROP_LINK("downstream", BesDmaState, downstream,
                     TYPE_MEMORY_REGION, MemoryRegion *),
};

static void bes_dma_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = bes_dma_realize;
    device_class_set_props(dc, bes_dma_props);
    device_class_set_legacy_reset(dc, bes_dma_reset);
}

static const TypeInfo bes_dma_info = {
    .name = TYPE_BES_DMA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BesDmaState),
    .instance_init = bes_dma_init,
    .class_init = bes_dma_class_init,
};

static void bes_dma_register_types(void)
{
    type_register_static(&bes_dma_info);
}

type_init(bes_dma_register_types)
