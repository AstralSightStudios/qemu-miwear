/*
 * HP4560X single-cell battery charger (I2C)
 *
 * No datasheet; the register use is the Xiaomi Smart Band 9 (n66) AP
 * driver's (hp4560x_initialize in sub_C39F92C: I2C0, address 0x07):
 *   0x0A     chip id, 0xE0 (the Band 11 q66 charger table also takes its
 *            "hp4570" by 0xE0 at 0x0A)
 *   0x08     status: bit 1 power good, bits [5:3] the charge phase, 0 not
 *            charging, 3 (0x18) done, others charging; bit 2 VINDPM
 *            (detect_worker sub_C165DA0)
 *   0x01     bit 3 set = charging disabled
 *   0x0B     bit 6 IINDPM
 * A register pointer write starts every transfer; reads and writes then
 * walk consecutive registers. Writes are stored.
 *
 * The input lines are the board's: "vbus" = a charger is plugged in,
 * "full" = the cell is charged (charge done instead of fast charge).
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/misc/hp4560x.h"
#include "trace.h"

#define HP_STATUS           0x08
#define HP_STATUS_PG        0x02
#define HP_CHG_FAST         (2 << 3)
#define HP_CHG_DONE         (3 << 3)
#define HP_CHIP_ID          0x0A
#define HP_CHIP_ID_VAL      0xE0

struct HP4560XState {
    I2CSlave parent_obj;

    uint8_t regs[256];
    uint8_t ptr;
    bool ptr_set;               /* first byte of a write transfer seen */
    bool vbus;
    bool full;
};

static uint8_t hp4560x_reg(HP4560XState *s, uint8_t reg)
{
    switch (reg) {
    case HP_CHIP_ID:
        return HP_CHIP_ID_VAL;
    case HP_STATUS:
        return s->vbus ? HP_STATUS_PG |
                         (s->full ? HP_CHG_DONE : HP_CHG_FAST) : 0;
    default:
        return s->regs[reg];
    }
}

static int hp4560x_event(I2CSlave *i2c, enum i2c_event event)
{
    HP4560XState *s = HP4560X(i2c);

    if (event == I2C_START_SEND) {
        s->ptr_set = false;
    }
    return 0;
}

static uint8_t hp4560x_recv(I2CSlave *i2c)
{
    HP4560XState *s = HP4560X(i2c);
    uint8_t v = hp4560x_reg(s, s->ptr);

    trace_hp4560x_read(s->ptr, v);
    s->ptr++;
    return v;
}

static int hp4560x_send(I2CSlave *i2c, uint8_t data)
{
    HP4560XState *s = HP4560X(i2c);

    if (!s->ptr_set) {
        s->ptr = data;
        s->ptr_set = true;
        return 0;
    }
    trace_hp4560x_write(s->ptr, data);
    s->regs[s->ptr++] = data;
    return 0;
}

static void hp4560x_reset(DeviceState *dev)
{
    HP4560XState *s = HP4560X(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->ptr = 0;
    s->ptr_set = false;
}

static void hp4560x_vbus(void *opaque, int n, int level)
{
    HP4560X(opaque)->vbus = level;
}

static void hp4560x_full(void *opaque, int n, int level)
{
    HP4560X(opaque)->full = level;
}

static void hp4560x_init(Object *obj)
{
    qdev_init_gpio_in_named(DEVICE(obj), hp4560x_vbus, "vbus", 1);
    qdev_init_gpio_in_named(DEVICE(obj), hp4560x_full, "full", 1);
}

static void hp4560x_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->event = hp4560x_event;
    k->recv = hp4560x_recv;
    k->send = hp4560x_send;
    device_class_set_legacy_reset(dc, hp4560x_reset);
}

static const TypeInfo hp4560x_info = {
    .name = TYPE_HP4560X,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(HP4560XState),
    .instance_init = hp4560x_init,
    .class_init = hp4560x_class_init,
};

static void hp4560x_register_types(void)
{
    type_register_static(&hp4560x_info);
}

type_init(hp4560x_register_types)
