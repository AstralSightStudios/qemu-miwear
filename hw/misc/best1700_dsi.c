/*
 * BES best1700 MIPI DSI host (command mode), DSI0 @0x401D0000
 *
 * Register use from the best1700 BSP (hal_dsi.o; .rodata.dsi_base =
 * {0x401D0000, 0x401D8000}, hal_dsi_irqn = 28):
 *   +0x10  interrupt / status. Bits [23:16] are event flags that a write
 *          of 1 clears (every helper rewrites the register with the other
 *          flag bits zero): bit 23 TE (hal_dsi_irq_is_te), bits 22/20/18
 *          errors (hal_dsi_irq_is_err). Bit 31 enables the TE interrupt
 *          (hal_dsi_teirq_enable), bit 14 the controller interrupt
 *          (hal_dsi_irq_disable clears it). Bits [12:8] count received
 *          bytes (dsi_read polls them).
 *   +0x14  transmit control: writing the packet length (bits 31:16) and
 *          a mode sends the packet (dsi_send_cmd); bits 5:4 read as busy.
 *   +0x18  packet header: data type | byte1 << 8 | byte2 << 16, or the
 *          word count in bits 23:8 for a long packet
 *          (hal_dsi_send_cmd_list: 0x05 DCS write, 0x15 DCS write with one
 *          parameter, 0x39 DCS long write; hal_dsi_generic_write: 0x13,
 *          0x23, 0x29 generic writes)
 *   +0x1C  long packet payload, four bytes that more follow
 *   +0x20..0x2C  the last one to four payload bytes (0x20 = 1 ... 0x2C = 4)
 *   +0x48  bit 1 enables the panel's TE reporting (hal_dsi_teirq_enable)
 *
 * DCS writes go to the panel model (bes_lcdc_panel_dcs). The panel's TE
 * output (gpio input "te") latches the TE flag.
 *
 * DCS reads (best1502p dsi_read, p65 sub_C96D7E0): the firmware clears
 * flag bit 19, sends 0x37 (maximum return size in byte 1) and 0x06 (the
 * DCS command in byte 1) with +0x14 = len << 16 | 0xE106, then reads +0x10
 * until bits [12:8] (bytes waiting) are 0 with bit 19 (read done) set;
 * each read of +0x10 returns the next byte in bits [7:0] and consumes it.
 * The bytes are one pad byte, then the response packet: data identifier,
 * two data bytes (short) or the word count (long), ECC, payload. With the
 * "panel-id" property set, DCS 0x04 returns its three ID bytes (low byte
 * first) and any other read returns zeros. Without it reads are not
 * answered and the firmware times out (q63 reads the ID once at init).
 * Everything else is plain storage.
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/misc/bes_lcdc.h"
#include "hw/misc/best1700_dsi.h"
#include "trace.h"

#define DSI_IRQ             0x10
#define DSI_TX_CTRL         0x14
#define DSI_TX_HDR          0x18
#define DSI_TX_DATA         0x1C
#define DSI_TX_LAST1        0x20
#define DSI_TX_LAST4        0x2C
#define DSI_TE_CTRL         0x48

#define DSI_IRQ_FLAGS       0x00FF0000u
#define DSI_IRQ_TE          (1u << 23)
#define DSI_IRQ_TE_EN       (1u << 31)
#define DSI_IRQ_RX_DONE     (1u << 19)
#define DSI_IRQ_RX_COUNT    0x1F00u
#define DSI_TE_CTRL_EN      (1u << 1)

#define DT_DCS_WRITE0       0x05
#define DT_DCS_WRITE1       0x15
#define DT_DCS_LONG         0x39
#define DT_DCS_READ         0x06
#define DT_MAX_RET_SIZE     0x37
#define DT_RESP_LONG        0x1C
#define DT_RESP_SHORT1      0x21
#define DT_RESP_SHORT2      0x22

static void best1700_dsi_update_irq(Best1700DsiState *s)
{
    qemu_set_irq(s->irq, (s->flags & DSI_IRQ_TE) &&
                         (s->regs[DSI_IRQ / 4] & DSI_IRQ_TE_EN));
}

static void best1700_dsi_te(void *opaque, int n, int level)
{
    Best1700DsiState *s = opaque;

    if (level && (s->regs[DSI_TE_CTRL / 4] & DSI_TE_CTRL_EN)) {
        s->flags |= DSI_IRQ_TE;
        best1700_dsi_update_irq(s);
    }
}

static void best1700_dsi_push(Best1700DsiState *s, uint32_t v, unsigned n)
{
    while (n-- && s->len < sizeof(s->payload)) {
        s->payload[s->len++] = v;
        v >>= 8;
    }
}

static void best1700_dsi_send(Best1700DsiState *s)
{
    uint32_t hdr = s->regs[DSI_TX_HDR / 4];
    uint8_t b1 = hdr >> 8, b2 = hdr >> 16;

    if (s->panel) {
        switch (hdr & 0x3F) {
        case DT_DCS_WRITE0:
            bes_lcdc_panel_dcs(s->panel, b1, NULL, 0);
            break;
        case DT_DCS_WRITE1:
            bes_lcdc_panel_dcs(s->panel, b1, &b2, 1);
            break;
        case DT_DCS_LONG:
            if (s->len) {
                bes_lcdc_panel_dcs(s->panel, s->payload[0], s->payload + 1,
                                   s->len - 1);
            }
            break;
        }
    }
    s->len = 0;
}

static void best1700_dsi_dcs_read(Best1700DsiState *s)
{
    uint8_t cmd = s->regs[DSI_TX_HDR / 4] >> 8;
    unsigned n = MIN(MAX(s->max_ret, 1), sizeof(s->rx) - 5), i;
    uint8_t data[sizeof(s->rx)] = { 0 };

    if (!s->panel_id) {
        return;
    }
    if (cmd == 0x04) {
        data[0] = s->panel_id;
        data[1] = s->panel_id >> 8;
        data[2] = s->panel_id >> 16;
    } else if (cmd == 0xDA) {
        data[0] = s->panel_id;
    } else if (cmd == 0xDB) {
        data[0] = s->panel_id >> 8;
    } else if (cmd == 0xDC) {
        data[0] = s->panel_id >> 16;
    } else if (cmd == 0x0A) {
        /*
         * MIPI DCS Get Power Mode (0x0A): bit 7 booster on, bit 5 partial,
         * bit 4 sleep out (0 = out), bit 2 display on.
         * RM690C0 / RM690F0 panel_simple and ESD check on best1502p (q65)
         * expect 0xB4.
         */
        data[0] = 0xB4;
    }
    s->rx_len = 0;
    s->rx[s->rx_len++] = 0;
    if (n <= 2) {
        s->rx[s->rx_len++] = n == 1 ? DT_RESP_SHORT1 : DT_RESP_SHORT2;
        s->rx[s->rx_len++] = data[0];
        s->rx[s->rx_len++] = data[1];
        s->rx[s->rx_len++] = 0;
    } else {
        s->rx[s->rx_len++] = DT_RESP_LONG;
        s->rx[s->rx_len++] = n;
        s->rx[s->rx_len++] = 0;
        s->rx[s->rx_len++] = 0;
        for (i = 0; i < n; i++) {
            s->rx[s->rx_len++] = data[i];
        }
    }
    s->rx_pos = 0;
    s->flags |= DSI_IRQ_RX_DONE;
}

static uint64_t best1700_dsi_read(void *opaque, hwaddr offset, unsigned size)
{
    Best1700DsiState *s = opaque;
    uint32_t v;

    switch (offset) {
    case DSI_IRQ:
        v = (s->regs[DSI_IRQ / 4] & ~(DSI_IRQ_FLAGS | DSI_IRQ_RX_COUNT | 0xFF)) |
            s->flags;
        if (s->rx_pos < s->rx_len) {
            v |= (s->rx_len - s->rx_pos) << 8 | s->rx[s->rx_pos++];
        }
        return v;
    case DSI_TX_CTRL:
        return s->regs[DSI_TX_CTRL / 4] & ~0x30u;       /* never busy */
    default:
        return s->regs[offset / 4];
    }
}

static void best1700_dsi_write(void *opaque, hwaddr offset, uint64_t val,
                               unsigned size)
{
    Best1700DsiState *s = opaque;

    trace_best1700_dsi_write(offset, val);
    switch (offset) {
    case DSI_IRQ:
        s->flags &= ~(val & DSI_IRQ_FLAGS);
        s->regs[DSI_IRQ / 4] = val & ~DSI_IRQ_FLAGS;
        break;
    case DSI_TX_HDR:
        s->regs[DSI_TX_HDR / 4] = val;
        s->len = 0;
        break;
    case DSI_TX_DATA:
        best1700_dsi_push(s, val, 4);
        break;
    case DSI_TX_LAST1 ... DSI_TX_LAST4:
        best1700_dsi_push(s, val, (offset - DSI_TX_LAST1) / 4 + 1);
        break;
    case DSI_TX_CTRL:
        s->regs[DSI_TX_CTRL / 4] = val;
        /* dsi_send_cmd: length << 16 | 0x0005 or | 0xE104; the other
         * values written here switch link modes */
        if ((val >> 16) &&
            ((val & 0xFFFF) == 0x0005 || (val & 0xFFFF) == 0xE104)) {
            if ((s->regs[DSI_TX_HDR / 4] & 0x3F) == DT_MAX_RET_SIZE) {
                s->max_ret = s->regs[DSI_TX_HDR / 4] >> 8 & 0xFF;
                s->len = 0;
            } else {
                best1700_dsi_send(s);
            }
        } else if ((val >> 16) && (val & 0xFFFF) == 0xE106 &&
                   (s->regs[DSI_TX_HDR / 4] & 0x3F) == DT_DCS_READ) {
            best1700_dsi_dcs_read(s);
        }
        break;
    default:
        s->regs[offset / 4] = val;
        break;
    }
    best1700_dsi_update_irq(s);
}

static const MemoryRegionOps best1700_dsi_ops = {
    .read = best1700_dsi_read,
    .write = best1700_dsi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void best1700_dsi_reset(DeviceState *dev)
{
    Best1700DsiState *s = BEST1700_DSI(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->flags = 0;
    s->len = 0;
    s->rx_len = s->rx_pos = 0;
    s->max_ret = 0;
}

static void best1700_dsi_init(Object *obj)
{
    Best1700DsiState *s = BEST1700_DSI(obj);

    memory_region_init_io(&s->iomem, obj, &best1700_dsi_ops, s,
                          "best1700.dsi", BEST1700_DSI_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_in_named(DEVICE(obj), best1700_dsi_te, "te", 1);
}

static const Property best1700_dsi_props[] = {
    DEFINE_PROP_LINK("panel", Best1700DsiState, panel, TYPE_DEVICE,
                     DeviceState *),
    DEFINE_PROP_UINT32("panel-id", Best1700DsiState, panel_id, 0),
};

static void best1700_dsi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, best1700_dsi_props);
    device_class_set_legacy_reset(dc, best1700_dsi_reset);
}

static const TypeInfo best1700_dsi_info = {
    .name = TYPE_BEST1700_DSI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Best1700DsiState),
    .instance_init = best1700_dsi_init,
    .class_init = best1700_dsi_class_init,
};

static void best1700_dsi_register_types(void)
{
    type_register_static(&best1700_dsi_info);
}

type_init(best1700_dsi_register_types)
