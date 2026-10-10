/*
 * BES GPIO controller v2 (best1503) with the PSC GPIO interrupt block
 *
 * Register layout of one bank, from the AP firmware HAL (hal_gpio_v2:
 * pin_set sub_C22F3B0, pin_clr sub_C22F41C, pin_dir sub_C22F338,
 * get_val sub_C22F4AC, setup_irq sub_C22F580, irq handler sub_C22F274),
 * register names as in BES hal_gpio_v2.c:
 *   0x00 DR_SET             0x04 DDR_SET (1 = output)
 *   0x0C DR_CLR             0x10 DDR_CLR
 *   0x18 INTEN_CLR          0x30 INTEN_SET
 *   0x1C INTMASK_CLR        0x34 INTMASK_SET
 *   0x20 INTTYPE_LEVEL_CLR  0x38 INTTYPE_LEVEL_SET (1 = edge)
 *   0x24 INT_POLARITY_CLR   0x3C INT_POLARITY_SET (1 = rising / high)
 *   0x28 DEBOUNCE_CLR       0x48 DEBOUNCE_SET
 *   0x44 RAW_INTSTATUS      0x4C PORTA_EOI
 *   0x50 EXT_PORT (pin levels)
 * Banks: 0x40081000 / 0x40089000 / 0x4008A000 (firmware table
 * 0x2CCDAB08), 32 + 32 + 16 pins; all share GPIO_IRQn = 44.
 *
 * With PSC_GPIO_IRQ_CTRL (as built) the always-on power sub-controller
 * owns GPIO interrupt routing: hal_psc_gpio_irq_enable sets per-bank bits
 * at PSC +0x80/+0x84/+0x88 and the irq handler reads the per-bank status
 * from PSC +0x8C/+0x90/+0x94 (sub_C22C4CC / sub_C22C568), then acks
 * each bank through PORTA_EOI.
 *
 * Interrupt semantics follow the DesignWare APB GPIO the v2 block is
 * derived from: an enabled edge-type pin latches on its active edge until
 * EOI; a level-type pin reports while its level matches the polarity.
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/bes_gpio.h"
#include "trace.h"

static uint32_t bes_gpio_levels(BesGpioBank *b)
{
    return (b->dr & b->ddr) | (b->ext & ~b->ddr);
}

static uint32_t bes_gpio_level_raw(BesGpioBank *b)
{
    uint32_t lvl = bes_gpio_levels(b);
    uint32_t active = (lvl & b->polarity) | (~lvl & ~b->polarity);

    return active & b->inten & ~b->inttype_level;
}

static uint32_t bes_gpio_status(BesGpioState *s, int n)
{
    BesGpioBank *b = &s->bank[n];

    return (b->raw | bes_gpio_level_raw(b)) & b->inten & ~b->intmask;
}

static void bes_gpio_update(BesGpioState *s)
{
    bool level = false;
    int n;

    for (n = 0; n < BES_GPIO_BANKS; n++) {
        if (bes_gpio_status(s, n) & s->psc_en[n]) {
            level = true;
        }
    }
    qemu_set_irq(s->irq, level);
}

/* recompute pin levels after any change; latch edges, drive outputs */
static void bes_gpio_levels_changed(BesGpioState *s, int n, uint32_t old)
{
    BesGpioBank *b = &s->bank[n];
    uint32_t now = bes_gpio_levels(b);
    uint32_t rise = now & ~old, fall = old & ~now;
    uint32_t edge = (rise & b->polarity) | (fall & ~b->polarity);
    uint32_t changed_out = (now ^ old) & b->ddr;
    int i;

    b->raw |= edge & b->inten & b->inttype_level;
    for (i = 0; i < 32; i++) {
        if (changed_out & (1u << i)) {
            qemu_set_irq(s->out[n * 32 + i], (now >> i) & 1);
        }
    }
    bes_gpio_update(s);
}

static void bes_gpio_set_input(void *opaque, int pin, int level)
{
    BesGpioState *s = BES_GPIO(opaque);
    BesGpioBank *b = &s->bank[pin / 32];
    uint32_t old = bes_gpio_levels(b);

    b->ext = deposit32(b->ext, pin % 32, 1, level != 0);
    bes_gpio_levels_changed(s, pin / 32, old);
}

static uint64_t bes_gpio_bank_read(void *opaque, hwaddr offset,
                                   unsigned size, int n)
{
    BesGpioState *s = BES_GPIO(opaque);
    BesGpioBank *b = &s->bank[n];

    switch (offset) {
    case 0x00:
    case 0x0C:
        return b->dr;
    case 0x04:
    case 0x10:
        return b->ddr;
    case 0x18:
    case 0x30:
        return b->inten;
    case 0x1C:
    case 0x34:
        return b->intmask;
    case 0x20:
    case 0x38:
        return b->inttype_level;
    case 0x24:
    case 0x3C:
        return b->polarity;
    case 0x28:
    case 0x48:
        return b->debounce;
    case 0x40:                          /* masked status */
        return bes_gpio_status(s, n);
    case 0x44:                          /* raw status */
        return (b->raw | bes_gpio_level_raw(b)) & b->inten;
    case 0x50:
        return bes_gpio_levels(b);
    default:
        qemu_log_mask(LOG_UNIMP, "bes.gpio: bank%d read +0x%02x\n", n,
                      (unsigned)offset);
        return 0;
    }
}

static void bes_gpio_bank_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size, int n)
{
    BesGpioState *s = BES_GPIO(opaque);
    BesGpioBank *b = &s->bank[n];
    uint32_t old = bes_gpio_levels(b);
    uint32_t v = value;

    switch (offset) {
    case 0x00:
        b->dr |= v;
        break;
    case 0x0C:
        b->dr &= ~v;
        break;
    case 0x04:
        b->ddr |= v;
        break;
    case 0x10:
        b->ddr &= ~v;
        break;
    case 0x30:
        b->inten |= v;
        break;
    case 0x18:
        b->inten &= ~v;
        b->raw &= ~v;
        break;
    case 0x34:
        b->intmask |= v;
        break;
    case 0x1C:
        b->intmask &= ~v;
        break;
    case 0x38:
        b->inttype_level |= v;
        break;
    case 0x20:
        b->inttype_level &= ~v;
        break;
    case 0x3C:
        b->polarity |= v;
        break;
    case 0x24:
        b->polarity &= ~v;
        break;
    case 0x48:
        b->debounce |= v;
        break;
    case 0x28:
        b->debounce &= ~v;
        break;
    case 0x4C:                          /* EOI: clear latched edges */
        b->raw &= ~v;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "bes.gpio: bank%d write +0x%02x = 0x%08x\n",
                      n, (unsigned)offset, v);
        return;
    }
    bes_gpio_levels_changed(s, n, old);
}

#define BES_GPIO_BANK_OPS(n)                                                \
static uint64_t bes_gpio_read##n(void *o, hwaddr a, unsigned sz)           \
{                                                                           \
    return bes_gpio_bank_read(o, a, sz, n);                                 \
}                                                                           \
static void bes_gpio_write##n(void *o, hwaddr a, uint64_t v, unsigned sz)  \
{                                                                           \
    bes_gpio_bank_write(o, a, v, sz, n);                                    \
}

BES_GPIO_BANK_OPS(0)
BES_GPIO_BANK_OPS(1)
BES_GPIO_BANK_OPS(2)

static const MemoryRegionOps bes_gpio_bank_ops[BES_GPIO_BANKS] = {
#define OPS(n) {                                                            \
    .read = bes_gpio_read##n, .write = bes_gpio_write##n,                  \
    .endianness = DEVICE_LITTLE_ENDIAN,                                     \
    .impl.min_access_size = 4, .impl.max_access_size = 4,                   \
    .valid.min_access_size = 4, .valid.max_access_size = 4,                 \
}
    OPS(0), OPS(1), OPS(2),
#undef OPS
};

/*
 * PSC GPIO interrupt block: per-bank enable and status words at the
 * offsets of the "psc-en<n>" / "psc-st<n>" properties (best1503: +0x00..
 * +0x08 enable, +0x0C..+0x14 status). Other words of the window are plain
 * storage (best1700 maps the window over the whole PSC block).
 */
static int bes_gpio_psc_reg(BesGpioState *s, hwaddr offset, bool *status)
{
    int n;

    for (n = 0; n < BES_GPIO_BANKS; n++) {
        if (offset == s->psc_en_off[n] || offset == s->psc_st_off[n]) {
            *status = offset == s->psc_st_off[n];
            return n;
        }
    }
    return -1;
}

static uint64_t bes_gpio_psc_read(void *opaque, hwaddr offset, unsigned size)
{
    BesGpioState *s = BES_GPIO(opaque);
    bool status;
    int n = bes_gpio_psc_reg(s, offset, &status);

    if (n < 0) {
        return s->psc_regs[offset / 4];
    }
    return status ? bes_gpio_status(s, n) & s->psc_en[n] : s->psc_en[n];
}

static void bes_gpio_psc_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    BesGpioState *s = BES_GPIO(opaque);
    bool status;
    int n = bes_gpio_psc_reg(s, offset, &status);

    if (n < 0) {
        s->psc_regs[offset / 4] = value;
    } else if (!status) {
        s->psc_en[n] = value;
        bes_gpio_update(s);
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "bes.gpio: PSC status +0x%02x is "
                      "read-only\n", (unsigned)offset);
    }
}

static const MemoryRegionOps bes_gpio_psc_ops = {
    .read = bes_gpio_psc_read,
    .write = bes_gpio_psc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void bes_gpio_reset(DeviceState *dev)
{
    BesGpioState *s = BES_GPIO(dev);
    int n;

    for (n = 0; n < BES_GPIO_BANKS; n++) {
        uint32_t ext = s->bank[n].ext;  /* external levels survive reset */

        memset(&s->bank[n], 0, sizeof(s->bank[n]));
        s->bank[n].ext = ext;
        s->psc_en[n] = 0;
    }
    memset(s->psc_regs, 0, sizeof(s->psc_regs));
}

static void bes_gpio_realize(DeviceState *dev, Error **errp)
{
    BesGpioState *s = BES_GPIO(dev);

    if (s->psc_size > sizeof(s->psc_regs)) {
        error_setg(errp, "bes.gpio: psc-size too large");
        return;
    }
    memory_region_init_io(&s->psc_mr, OBJECT(dev), &bes_gpio_psc_ops, s,
                          "bes.gpio-psc", s->psc_size);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->psc_mr);
}

static const Property bes_gpio_props[] = {
    DEFINE_PROP_UINT32("psc-size", BesGpioState, psc_size, 0x18),
    DEFINE_PROP_UINT32("psc-en0", BesGpioState, psc_en_off[0], 0x00),
    DEFINE_PROP_UINT32("psc-en1", BesGpioState, psc_en_off[1], 0x04),
    DEFINE_PROP_UINT32("psc-en2", BesGpioState, psc_en_off[2], 0x08),
    DEFINE_PROP_UINT32("psc-st0", BesGpioState, psc_st_off[0], 0x0C),
    DEFINE_PROP_UINT32("psc-st1", BesGpioState, psc_st_off[1], 0x10),
    DEFINE_PROP_UINT32("psc-st2", BesGpioState, psc_st_off[2], 0x14),
};

static void bes_gpio_init(Object *obj)
{
    BesGpioState *s = BES_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    int n;

    for (n = 0; n < BES_GPIO_BANKS; n++) {
        g_autofree char *name = g_strdup_printf("bes.gpio%d", n);

        memory_region_init_io(&s->bank_mr[n], obj, &bes_gpio_bank_ops[n], s,
                              name, 0x100);
        sysbus_init_mmio(sbd, &s->bank_mr[n]);
    }
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_in(DEVICE(obj), bes_gpio_set_input, BES_GPIO_PINS);
    qdev_init_gpio_out(DEVICE(obj), s->out, BES_GPIO_PINS);
}

static void bes_gpio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = bes_gpio_realize;
    device_class_set_props(dc, bes_gpio_props);
    device_class_set_legacy_reset(dc, bes_gpio_reset);
}

static const TypeInfo bes_gpio_info = {
    .name = TYPE_BES_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BesGpioState),
    .instance_init = bes_gpio_init,
    .class_init = bes_gpio_class_init,
};

static void bes_gpio_register_types(void)
{
    type_register_static(&bes_gpio_info);
}

type_init(bes_gpio_register_types)
