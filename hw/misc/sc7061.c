/*
 * Southchip SC7061 single-cell battery charger (I2C)
 *
 * No datasheet; the register use is the Xiaomi Smart Band 10 Pro (p67)
 * AP driver's (charger probe sub_C404878: I2C3, address 0x07, 400 kHz;
 * Band 11 q66 finds it on I2C0 at 0x07 from its charger table
 * off_2CA09A20):
 *   0x01     chip revision: the p67 probe (sub_C403B1C) takes 7 or 4,
 *            the q66 one 7, the Band 9 Pro (n67) one 6; reading 0xC6 at
 *            0x60 instead means an HP4570
 *   0x30     status: bit 3 power good (sc7061 online op sub_C4039BC),
 *            bit 2 DPM, bits [6:5] the charge phase (state op
 *            sub_C403304: 0x40 / 0x20 charging, 0 not)
 *   0x10..   configuration the probe writes from its table
 *            (register, keep mask, value), update-bits style
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
#include "hw/core/qdev-properties.h"
#include "hw/misc/sc7061.h"
#include "trace.h"

#define SC_REVISION         0x01
#define SC_STATUS           0x30
#define SC_STATUS_PG        0x08
#define SC_CHG_FAST         0x40
#define SC_CHG_DONE         0x60

struct SC7061State {
    I2CSlave parent_obj;

    uint8_t regs[256];
    uint8_t ptr;
    bool ptr_set;               /* first byte of a write transfer seen */
    bool vbus;
    bool full;

    uint8_t revision;
};

static uint8_t sc7061_reg(SC7061State *s, uint8_t reg)
{
    switch (reg) {
    case SC_REVISION:
        return s->revision;
    case SC_STATUS:
        return s->vbus ? SC_STATUS_PG |
                         (s->full ? SC_CHG_DONE : SC_CHG_FAST) : 0;
    default:
        return s->regs[reg];
    }
}

static int sc7061_event(I2CSlave *i2c, enum i2c_event event)
{
    SC7061State *s = SC7061(i2c);

    if (event == I2C_START_SEND) {
        s->ptr_set = false;
    }
    return 0;
}

static uint8_t sc7061_recv(I2CSlave *i2c)
{
    SC7061State *s = SC7061(i2c);
    uint8_t v = sc7061_reg(s, s->ptr);

    trace_sc7061_read(s->ptr, v);
    s->ptr++;
    return v;
}

static int sc7061_send(I2CSlave *i2c, uint8_t data)
{
    SC7061State *s = SC7061(i2c);

    if (!s->ptr_set) {
        s->ptr = data;
        s->ptr_set = true;
        return 0;
    }
    trace_sc7061_write(s->ptr, data);
    s->regs[s->ptr++] = data;
    return 0;
}

static void sc7061_reset(DeviceState *dev)
{
    SC7061State *s = SC7061(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->ptr = 0;
    s->ptr_set = false;
}

static void sc7061_vbus(void *opaque, int n, int level)
{
    SC7061(opaque)->vbus = level;
}

static void sc7061_full(void *opaque, int n, int level)
{
    SC7061(opaque)->full = level;
}

static void sc7061_init(Object *obj)
{
    qdev_init_gpio_in_named(DEVICE(obj), sc7061_vbus, "vbus", 1);
    qdev_init_gpio_in_named(DEVICE(obj), sc7061_full, "full", 1);
}

static const Property sc7061_props[] = {
    DEFINE_PROP_UINT8("revision", SC7061State, revision, 7),
};

static void sc7061_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->event = sc7061_event;
    k->recv = sc7061_recv;
    k->send = sc7061_send;
    device_class_set_props(dc, sc7061_props);
    device_class_set_legacy_reset(dc, sc7061_reset);
}

static const TypeInfo sc7061_info = {
    .name = TYPE_SC7061,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(SC7061State),
    .instance_init = sc7061_init,
    .class_init = sc7061_class_init,
};

static void sc7061_register_types(void)
{
    type_register_static(&sc7061_info);
}

type_init(sc7061_register_types)
