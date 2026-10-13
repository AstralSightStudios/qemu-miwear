/*
 * SG Micro SGM41551 single-cell battery charger (I2C)
 *
 * No datasheet; register usage is from the Xiaomi Watch S5 (q63) AP driver
 * (board_charger_ic_initialize in sub_1087D7A0: I2C3, address 0x6B, 100 kHz;
 * sgm41551_initialized in sub_1087F324):
 *   0x38     part number / chip id: probe checks ((val >> 3) & 7) == 0;
 *            reading 0x58 instead indicates an SC89620
 *   0x1F     VBUS status: bit 7 = 1 offline/fault (signed < 0), bit 7 = 0
 *            online (signed >= 0); bits [4:3] = 1 adapter connected;
 *            status worker requires (reg 0x1F | (reg 0x18 & 3)) == 0 for full
 *   0x1E     charge status: bits [4:3] = 0 not charging, 2 fast charge (0x10),
 *            3 charge done (0x18)
 *   0x16     charge phase / config: bits [5:4] = 0x20 when charge done,
 *            0x10 when fast charging
 *   0x1D     fault register: 0 = normal (bits 2, 3, 0 indicate faults)
 *   0x18     status register: bits [1:0] = 0
 *   0x00..   configuration written during init and runtime
 *
 * Consecutive byte reads/writes auto-increment the register pointer.
 * Writes are stored.
 *
 * The input lines are the board's:
 *   "vbus" = charger is plugged in
 *   "full" = cell is charged (charge done instead of fast charge)
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/sgm41551.h"
#include "trace.h"

#define SGM41551_CHG_CONFIG     0x16
#define SGM41551_CHG_DONE_PHASE 0x20
#define SGM41551_CHG_FAST_PHASE 0x10

#define SGM41551_STATUS         0x18

#define SGM41551_FAULT          0x1D

#define SGM41551_CHG_STAT       0x1E
#define SGM41551_STAT_NOT_CHG   (0 << 3)
#define SGM41551_STAT_FAST_CHG  (2 << 3)
#define SGM41551_STAT_CHG_DONE  (3 << 3)

#define SGM41551_VBUS_STAT      0x1F
#define SGM41551_VBUS_OFFLINE   0x80
#define SGM41551_VBUS_ADAPTER   0x08

#define SGM41551_PART_NO        0x38
#define SGM41551_PART_NO_DEFAULT 0x00

struct SGM41551State {
    I2CSlave parent_obj;

    uint8_t regs[256];
    uint8_t ptr;
    bool ptr_set;
    bool vbus;
    bool full;

    uint8_t part_no;
};

static uint8_t sgm41551_reg(SGM41551State *s, uint8_t reg)
{
    switch (reg) {
    case SGM41551_PART_NO:
        return s->part_no;
    case SGM41551_VBUS_STAT:
        if (!s->vbus) {
            return SGM41551_VBUS_OFFLINE;
        }
        return s->full ? 0x00 : SGM41551_VBUS_ADAPTER;
    case SGM41551_CHG_STAT:
        if (!s->vbus) {
            return (s->regs[reg] & ~0x18) | SGM41551_STAT_NOT_CHG;
        }
        return (s->regs[reg] & ~0x18) |
               (s->full ? SGM41551_STAT_CHG_DONE : SGM41551_STAT_FAST_CHG);
    case SGM41551_CHG_CONFIG:
        if (!s->vbus) {
            return s->regs[reg] & ~0x30;
        }
        return (s->regs[reg] & ~0x30) |
               (s->full ? SGM41551_CHG_DONE_PHASE : SGM41551_CHG_FAST_PHASE);
    case SGM41551_FAULT:
        return 0;
    case SGM41551_STATUS:
        return s->regs[reg] & ~0x03;
    default:
        return s->regs[reg];
    }
}

static int sgm41551_event(I2CSlave *i2c, enum i2c_event event)
{
    SGM41551State *s = SGM41551(i2c);

    if (event == I2C_START_SEND) {
        s->ptr_set = false;
    }
    return 0;
}

static uint8_t sgm41551_recv(I2CSlave *i2c)
{
    SGM41551State *s = SGM41551(i2c);
    uint8_t v = sgm41551_reg(s, s->ptr);

    trace_sgm41551_read(s->ptr, v);
    s->ptr++;
    return v;
}

static int sgm41551_send(I2CSlave *i2c, uint8_t data)
{
    SGM41551State *s = SGM41551(i2c);

    if (!s->ptr_set) {
        s->ptr = data;
        s->ptr_set = true;
        return 0;
    }
    trace_sgm41551_write(s->ptr, data);
    s->regs[s->ptr++] = data;
    return 0;
}

static void sgm41551_reset(DeviceState *dev)
{
    SGM41551State *s = SGM41551(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->ptr = 0;
    s->ptr_set = false;
}

static void sgm41551_vbus(void *opaque, int n, int level)
{
    SGM41551(opaque)->vbus = level;
}

static void sgm41551_full(void *opaque, int n, int level)
{
    SGM41551(opaque)->full = level;
}

static void sgm41551_init(Object *obj)
{
    qdev_init_gpio_in_named(DEVICE(obj), sgm41551_vbus, "vbus", 1);
    qdev_init_gpio_in_named(DEVICE(obj), sgm41551_full, "full", 1);
}

static const Property sgm41551_props[] = {
    DEFINE_PROP_UINT8("part-no", SGM41551State, part_no,
                      SGM41551_PART_NO_DEFAULT),
};

static void sgm41551_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->event = sgm41551_event;
    k->recv = sgm41551_recv;
    k->send = sgm41551_send;
    device_class_set_props(dc, sgm41551_props);
    device_class_set_legacy_reset(dc, sgm41551_reset);
}

static const TypeInfo sgm41551_info = {
    .name = TYPE_SGM41551,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(SGM41551State),
    .instance_init = sgm41551_init,
    .class_init = sgm41551_class_init,
};

static void sgm41551_register_types(void)
{
    type_register_static(&sgm41551_info);
}

type_init(sgm41551_register_types)
