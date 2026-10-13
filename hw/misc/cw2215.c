/*
 * CellWise CW2215 battery fuel gauge (I2C, CW221x family)
 *
 * Register map of the CW221x family (CellWise datasheet; same layout as
 * the cw2217 driver in mainline Linux):
 *   0x00     chip id, 0xA0
 *   0x02/03  cell voltage, 14 bits big-endian, 312.5 uV per LSB
 *   0x04/05  state of charge: integer percent, then 1/256 percent
 *   0x06     temperature, 0.5 degC per LSB from -40 degC
 *   0x08     mode: 0x00 active, 0x30 restart, 0xF0 sleep
 *   0x0A     GPIO configuration, 0x0B SOC alert (bit 7 = profile
 *            updated flag), 0x0C/0D temperature limits
 *   0x0E/0F  current, signed 16 bits
 *   0x10-9F  battery profile, written by the host driver
 *   0xA0-A9  host temperature, user configuration, cycle count, state of
 *            health (percent), IC state (bits 2-3 set once the gauge has
 *            taken the profile), standby current; 0xAB firmware version
 * A register pointer write starts every transfer; reads and writes then
 * walk consecutive registers.
 *
 * The q63 (Xiaomi Watch S5) AP finds it at address 0x64 on its I2C
 * controller 3 (0x40076000); without it the battery driver registers no
 * gauge and the boot-time low battery check (low_vol_loop_chargeing)
 * keeps the device in its charging loop for 20 minutes.
 *
 * Voltage, charge, temperature and health come from properties; the
 * board moves charge and voltage at run time (cw2215_set_battery).
 * Writes are stored, so the driver reads back the profile it wrote; the
 * gauge reports itself ready at once.
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/cw2215.h"
#include "trace.h"

#define CW_CHIP_ID          0x00
#define CW_VCELL_H          0x02
#define CW_SOC_INT          0x04
#define CW_SOC_DECIMAL      0x05
#define CW_TEMP             0x06
#define CW_MODE_CONFIG      0x08
#define CW_SOC_ALERT        0x0B
#define CW_CURRENT_H        0x0E
#define CW_CYCLE_H          0xA4
#define CW_SOH              0xA6
#define CW_IC_STATE         0xA7
#define CW_FW_VERSION       0xAB

#define CW_IC_VCHIP_ID      0xA0
#define CW_IC_READY_MARK    0x0C
#define CW_UPDATE_FLG       0x80

struct CW2215State {
    I2CSlave parent_obj;

    uint8_t regs[256];
    uint8_t ptr;
    bool ptr_set;               /* first byte of a write transfer seen */

    uint32_t voltage_mv;
    uint32_t soc;
    int32_t temp;
    uint32_t soh;
};

/* the measured values, refreshed from the properties on every read */
static uint8_t cw2215_reg(CW2215State *s, uint8_t reg)
{
    uint32_t vcell = s->voltage_mv * 16 / 5;        /* 312.5 uV units */

    switch (reg) {
    case CW_CHIP_ID:
        return CW_IC_VCHIP_ID;
    case CW_VCELL_H:
        return (vcell >> 8) & 0x3F;
    case CW_VCELL_H + 1:
        return vcell;
    case CW_SOC_INT:
        return MIN(s->soc, 100);
    case CW_SOC_DECIMAL:
        return 0;
    case CW_TEMP:
        return CLAMP((s->temp + 40) * 2, 0, 255);
    case CW_CURRENT_H:
    case CW_CURRENT_H + 1:
        return 0;
    case CW_SOH:
        return MIN(s->soh, 100);
    case CW_IC_STATE:
        return s->regs[reg] | CW_IC_READY_MARK;
    default:
        return s->regs[reg];
    }
}

void cw2215_set_battery(CW2215State *s, uint32_t soc, uint32_t voltage_mv)
{
    s->soc = soc;
    s->voltage_mv = voltage_mv;
}

static int cw2215_event(I2CSlave *i2c, enum i2c_event event)
{
    CW2215State *s = CW2215(i2c);

    if (event == I2C_START_SEND) {
        s->ptr_set = false;
    }
    return 0;
}

static uint8_t cw2215_recv(I2CSlave *i2c)
{
    CW2215State *s = CW2215(i2c);
    uint8_t v = cw2215_reg(s, s->ptr);

    trace_cw2215_read(s->ptr, v);
    s->ptr++;
    return v;
}

static int cw2215_send(I2CSlave *i2c, uint8_t data)
{
    CW2215State *s = CW2215(i2c);

    if (!s->ptr_set) {
        s->ptr = data;
        s->ptr_set = true;
        return 0;
    }
    trace_cw2215_write(s->ptr, data);
    s->regs[s->ptr++] = data;
    return 0;
}

static void cw2215_reset(DeviceState *dev)
{
    CW2215State *s = CW2215(dev);

    /* a gauge that has been running with a profile: active, updated */
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[CW_MODE_CONFIG] = 0x00;
    s->regs[CW_SOC_ALERT] = CW_UPDATE_FLG;
    s->regs[CW_FW_VERSION] = 0x10;
    s->ptr = 0;
    s->ptr_set = false;
}

static const Property cw2215_props[] = {
    DEFINE_PROP_UINT32("voltage-mv", CW2215State, voltage_mv, 3950),
    DEFINE_PROP_UINT32("soc", CW2215State, soc, 80),
    DEFINE_PROP_INT32("temp", CW2215State, temp, 25),
    DEFINE_PROP_UINT32("soh", CW2215State, soh, 100),
};

static void cw2215_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->event = cw2215_event;
    k->recv = cw2215_recv;
    k->send = cw2215_send;
    device_class_set_props(dc, cw2215_props);
    device_class_set_legacy_reset(dc, cw2215_reset);
}

static const TypeInfo cw2215_info = {
    .name = TYPE_CW2215,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(CW2215State),
    .class_init = cw2215_class_init,
};

static void cw2215_register_types(void)
{
    type_register_static(&cw2215_info);
}

type_init(cw2215_register_types)
