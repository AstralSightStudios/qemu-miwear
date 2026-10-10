/*
 * SGM25062 six-channel I2C load switch.
 *
 * Register map: SG Micro SGM25062 datasheet, April 2023, Rev. A.1,
 * pp. 18-24. Channels with slot 0 follow LDSW_EN; the other channels
 * follow the seven-slot power-up/down sequencer. Model the digital
 * controls and six power outputs, not analogue ramp/discharge timing.
 * EN is tied high unless the board connects the "enable" input.
 *
 * The q63 sensor firmware's bes_switch_setup uses BTH I2C2 at 0x18;
 * sgm25062_register reads CHIPID and then writes the channel controls.
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/core/irq.h"
#include "hw/misc/sgm25062.h"
#include "migration/vmstate.h"
#include "qemu/timer.h"
#include "trace.h"

#define SGM_CHIPID      0x00
#define SGM_ENABLE      0x02
#define SGM_DISCHARGE   0x03
#define SGM_SEQ12       0x05
#define SGM_SEQ_CTRL    0x08
#define SGM_STATUS      0x0B
#define SGM_RESET       0x69
#define SGM_CHANNELS    6

struct SGM25062State {
    I2CSlave parent_obj;
    uint8_t regs[12];
    uint8_t ptr;
    bool ptr_set;
    bool enabled;
    int8_t direction;
    uint8_t slot;
    QEMUTimer *timer;
    qemu_irq power[SGM_CHANNELS];
};

static unsigned sgm25062_slot(SGM25062State *s, unsigned channel)
{
    return (s->regs[SGM_SEQ12 + channel / 2] >> (3 * (channel % 2))) & 7;
}

static void sgm25062_update(SGM25062State *s)
{
    unsigned i;

    for (i = 0; i < SGM_CHANNELS; i++) {
        uint8_t mask = 1 << i;

        if (!s->enabled) {
            s->regs[SGM_STATUS] &= ~mask;
        } else if (!sgm25062_slot(s, i)) {
            s->regs[SGM_STATUS] = (s->regs[SGM_STATUS] & ~mask) |
                                  (s->regs[SGM_ENABLE] & mask);
        }
        qemu_set_irq(s->power[i], !!(s->regs[SGM_STATUS] & mask));
    }
}

static void sgm25062_schedule(SGM25062State *s)
{
    int64_t period = (700 + 600 * (s->regs[SGM_SEQ_CTRL] >> 6)) * 1000;

    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + period);
}

static void sgm25062_tick(void *opaque)
{
    SGM25062State *s = opaque;
    unsigned i;

    for (i = 0; i < SGM_CHANNELS; i++) {
        if (sgm25062_slot(s, i) == s->slot) {
            if (s->direction > 0) {
                s->regs[SGM_STATUS] |= 1 << i;
            } else {
                s->regs[SGM_STATUS] &= ~(1 << i);
            }
        }
    }
    sgm25062_update(s);
    s->slot += s->direction;
    if (s->slot == 0 || s->slot == 8) {
        s->slot = 0;
        s->direction = 0;
    } else {
        sgm25062_schedule(s);
    }
}

static void sgm25062_reset_regs(SGM25062State *s)
{
    timer_del(s->timer);
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[SGM_CHIPID] = 0x32;
    s->regs[SGM_DISCHARGE] = 0x3F;
    s->slot = 0;
    s->direction = 0;
    sgm25062_update(s);
}

static void sgm25062_enable(void *opaque, int n, int level)
{
    SGM25062State *s = opaque;

    s->enabled = !!level;
    if (!level) {
        sgm25062_reset_regs(s);
        s->ptr_set = false;
    }
}

static int sgm25062_event(I2CSlave *i2c, enum i2c_event event)
{
    SGM25062State *s = SGM25062(i2c);

    if (event == I2C_START_SEND) {
        s->ptr_set = false;
    }
    return s->enabled ? 0 : -1;
}

static uint8_t sgm25062_recv(I2CSlave *i2c)
{
    SGM25062State *s = SGM25062(i2c);
    uint8_t reg = s->ptr++;
    uint8_t value = reg < sizeof(s->regs) ? s->regs[reg] : 0;

    if (reg == SGM_SEQ_CTRL) {
        value |= s->slot | (s->direction ? 8 : 0);
    }
    trace_sgm25062_read(reg, value);
    return value;
}

static int sgm25062_send(I2CSlave *i2c, uint8_t value)
{
    SGM25062State *s = SGM25062(i2c);
    uint8_t reg;

    if (!s->ptr_set) {
        s->ptr = value;
        s->ptr_set = true;
        return 0;
    }
    reg = s->ptr++;
    trace_sgm25062_write(reg, value);
    if (reg == SGM_RESET) {
        /* Other values are ignored; this register always reads zero. */
        if (value == 0xB0) {
            sgm25062_reset_regs(s);
        }
    } else if (reg == SGM_SEQ_CTRL) {
        unsigned command = (value >> 4) & 3;

        s->regs[reg] = value & 0xC0;
        if (command == 1 || command == 2) {
            s->direction = command == 1 ? 1 : -1;
            s->slot = command == 1 ? 1 : 7;
            sgm25062_schedule(s);
        }
    } else if (reg >= SGM_ENABLE && reg < SGM_STATUS) {
        s->regs[reg] = value & 0x3F;
    }
    sgm25062_update(s);
    return 0;
}

static void sgm25062_reset(DeviceState *dev)
{
    SGM25062State *s = SGM25062(dev);

    s->enabled = true;
    s->ptr = 0;
    s->ptr_set = false;
    sgm25062_reset_regs(s);
}

static void sgm25062_init(Object *obj)
{
    SGM25062State *s = SGM25062(obj);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sgm25062_tick, s);
    qdev_init_gpio_in_named(DEVICE(obj), sgm25062_enable, "enable", 1);
    qdev_init_gpio_out_named(DEVICE(obj), s->power, "power", SGM_CHANNELS);
}

static void sgm25062_finalize(Object *obj)
{
    SGM25062State *s = SGM25062(obj);

    timer_free(s->timer);
}

static int sgm25062_post_load(void *opaque, int version_id)
{
    sgm25062_update(opaque);
    return 0;
}

static const VMStateDescription vmstate_sgm25062 = {
    .name = "sgm25062",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = sgm25062_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, SGM25062State),
        VMSTATE_UINT8_ARRAY(regs, SGM25062State, 12),
        VMSTATE_UINT8(ptr, SGM25062State),
        VMSTATE_BOOL(ptr_set, SGM25062State),
        VMSTATE_BOOL(enabled, SGM25062State),
        VMSTATE_INT8(direction, SGM25062State),
        VMSTATE_UINT8(slot, SGM25062State),
        VMSTATE_TIMER_PTR(timer, SGM25062State),
        VMSTATE_END_OF_LIST()
    },
};

static void sgm25062_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->event = sgm25062_event;
    k->recv = sgm25062_recv;
    k->send = sgm25062_send;
    dc->vmsd = &vmstate_sgm25062;
    device_class_set_legacy_reset(dc, sgm25062_reset);
}

static const TypeInfo sgm25062_info = {
    .name = TYPE_SGM25062,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(SGM25062State),
    .instance_init = sgm25062_init,
    .instance_finalize = sgm25062_finalize,
    .class_init = sgm25062_class_init,
};

static void sgm25062_register_types(void)
{
    type_register_static(&sgm25062_info);
}

type_init(sgm25062_register_types)
