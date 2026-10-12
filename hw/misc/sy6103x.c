/*
 * Silergy SY6103X single-cell battery charger (I2C)
 *
 * No datasheet; the register use is the REDMI Watch 6 (p65) AP driver's
 * (charger probe sub_C4DE158 on I2C0, address 0x07, 400 kHz):
 *   0x60     chip id, the probe (sub_C4F443C) wants 0x98
 *   0x08     status, bits [4:3] the charge state (sub_C4F3C50:
 *            0 not charging, 1 pre-charge, 2 fast charge, 3 done)
 *   0x00..   configuration the probe writes from its table
 *            (register, keep mask, value) and reads back
 * A register pointer write starts every transfer; reads and writes then
 * walk consecutive registers. Writes are stored; the charger reports no
 * charging (the board has no USB power).
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/misc/sy6103x.h"
#include "trace.h"

#define SY_STATUS           0x08
#define SY_CHIP_ID          0x60
#define SY_CHIP_ID_VAL      0x98

struct SY6103XState {
    I2CSlave parent_obj;

    uint8_t regs[256];
    uint8_t ptr;
    bool ptr_set;               /* first byte of a write transfer seen */
};

static int sy6103x_event(I2CSlave *i2c, enum i2c_event event)
{
    SY6103XState *s = SY6103X(i2c);

    if (event == I2C_START_SEND) {
        s->ptr_set = false;
    }
    return 0;
}

static uint8_t sy6103x_recv(I2CSlave *i2c)
{
    SY6103XState *s = SY6103X(i2c);
    uint8_t v = s->regs[s->ptr];

    trace_sy6103x_read(s->ptr, v);
    s->ptr++;
    return v;
}

static int sy6103x_send(I2CSlave *i2c, uint8_t data)
{
    SY6103XState *s = SY6103X(i2c);

    if (!s->ptr_set) {
        s->ptr = data;
        s->ptr_set = true;
        return 0;
    }
    trace_sy6103x_write(s->ptr, data);
    if (s->ptr != SY_CHIP_ID && s->ptr != SY_STATUS) {
        s->regs[s->ptr] = data;
    }
    s->ptr++;
    return 0;
}

static void sy6103x_reset(DeviceState *dev)
{
    SY6103XState *s = SY6103X(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[SY_CHIP_ID] = SY_CHIP_ID_VAL;
    s->ptr = 0;
    s->ptr_set = false;
}

static void sy6103x_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->event = sy6103x_event;
    k->recv = sy6103x_recv;
    k->send = sy6103x_send;
    device_class_set_legacy_reset(dc, sy6103x_reset);
}

static const TypeInfo sy6103x_info = {
    .name = TYPE_SY6103X,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(SY6103XState),
    .class_init = sy6103x_class_init,
};

static void sy6103x_register_types(void)
{
    type_register_static(&sy6103x_info);
}

type_init(sy6103x_register_types)
