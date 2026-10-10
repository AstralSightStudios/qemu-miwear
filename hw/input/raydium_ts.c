/*
 * Raydium RM6D03x capacitive touch controller (I2C)
 *
 * Protocol as used by Xiaomi Watch S5 (q63) AP firmware:
 * - I2C slave address 0x39 on AP I2C0 (0x40006000)
 * - Interrupt pin: GPIO 73 (active low)
 * - Register access modes:
 *   1) 6-byte command [0x66, a0, a1, a2, a3, cmd]:
 *      - 0x500009BC: returns 4 bytes, high word is 0xF303 (chip id)
 *      - 0x50000064: returns 4 bytes (touch id)
 *      - 0x50008938: returns 4 bytes (channel count 28x28)
 *   2) 1-byte register address:
 *      - reg 0x06: firmware version 0x000F1804 (4 bytes)
 *      - reg 0x08: display resolution 466x466 (4 bytes)
 *      - reg 0x00: 16-byte touch report packet:
 *          [0] = report update flag (1 if new event)
 *          [1] = active points count (1 if down/move, 0 if up)
 *          [2] = gesture (0)
 *          [3] = 0x40 (normal touch status flag)
 *          [4] = point 0 id (0)
 *          [5..6] = point 0 X (little endian)
 *          [7..8] = point 0 Y (little endian)
 *          [13] = state (1=DOWN, 2=UP, 3=MOVE)
 *        This layout is provisional: it has not been checked against the
 *        driver's report parser yet.
 * The probe passes (chip id, touch id, firmware version equal to the one
 * in /vendor/touch, so no upgrade); the CC calibration handshake that
 * follows is not modelled and times out after about 3 s.
 *
 * Touches come from the host pointer (absolute pointer device).
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/input/raydium_ts.h"
#include "ui/input.h"
#include "trace.h"

#define RAYDIUM_QUEUE   16

typedef struct {
    uint8_t state;      /* 1=DOWN, 2=UP, 3=MOVE */
    uint16_t x, y;
} RaydiumEvent;

struct RaydiumTsState {
    I2CSlave parent_obj;
    qemu_irq irq;
    QemuInputHandlerState *input;

    /* properties */
    uint32_t width;
    uint32_t height;

    /* I2C state */
    uint8_t write_buf[16];
    uint8_t write_len;
    uint32_t reg_addr;
    uint8_t reg8;
    bool is_extended;
    uint8_t read_idx;
    bool reading_report;

    /* event queue */
    RaydiumEvent q[RAYDIUM_QUEUE];
    unsigned q_head, q_count;

    /* host pointer */
    int abs_x, abs_y;
    bool btn, btn_reported;
    int last_x, last_y;
};

static void raydium_ts_update_irq(RaydiumTsState *s)
{
    /* Active-low interrupt line */
    qemu_set_irq(s->irq, s->q_count == 0);
}

static void raydium_ts_push(RaydiumTsState *s, uint8_t state, int x, int y)
{
    RaydiumEvent *tail;

    if (s->q_count == RAYDIUM_QUEUE) {
        return;
    }
    tail = &s->q[(s->q_head + s->q_count) % RAYDIUM_QUEUE];
    tail->state = state;
    tail->x = x;
    tail->y = y;
    s->q_count++;
    trace_raydium_ts_event(state, x, y);
    raydium_ts_update_irq(s);
}

static int raydium_ts_event(I2CSlave *i2c, enum i2c_event event)
{
    RaydiumTsState *s = RAYDIUM_TS(i2c);

    switch (event) {
    case I2C_START_SEND:
        s->write_len = 0;
        break;
    case I2C_START_RECV:
        s->read_idx = 0;
        s->reading_report = (!s->is_extended && s->reg8 == 0x00);
        break;
    case I2C_FINISH:
        if (s->reading_report) {
            s->reading_report = false;
            if (s->q_count) {
                s->q_head = (s->q_head + 1) % RAYDIUM_QUEUE;
                s->q_count--;
                /* Raise then lower to generate an edge if more events remain */
                qemu_set_irq(s->irq, 1);
                if (s->q_count) {
                    qemu_set_irq(s->irq, 0);
                }
            }
        }
        s->write_len = 0;
        break;
    default:
        break;
    }
    return 0;
}

static int raydium_ts_send(I2CSlave *i2c, uint8_t data)
{
    RaydiumTsState *s = RAYDIUM_TS(i2c);

    if (s->write_len < sizeof(s->write_buf)) {
        s->write_buf[s->write_len] = data;
    }
    s->write_len++;

    if (s->write_len == 1) {
        if (data == 0x66) {
            s->is_extended = true;
        } else {
            s->is_extended = false;
            s->reg8 = data;
            trace_raydium_ts_write(data, 0);
        }
    } else if (s->is_extended && s->write_len == 5) {
        s->reg_addr = s->write_buf[1] |
                      (s->write_buf[2] << 8) |
                      (s->write_buf[3] << 16) |
                      (s->write_buf[4] << 24);
        trace_raydium_ts_write(s->reg_addr, 0);
    }
    return 0;
}

static uint8_t raydium_ts_recv(I2CSlave *i2c)
{
    RaydiumTsState *s = RAYDIUM_TS(i2c);
    uint8_t byte = 0;

    if (s->is_extended) {
        switch (s->reg_addr) {
        case 0x500009BC:
            /* Chip ID: 0xF303 (bytes: 0x00, 0x00, 0x03, 0xF3) */
            switch (s->read_idx) {
            case 0: byte = 0x00; break;
            case 1: byte = 0x00; break;
            case 2: byte = 0x03; break;
            case 3: byte = 0xF3; break;
            default: byte = 0x00; break;
            }
            break;
        case 0x50000064:
            /* Touch ID: 4 bytes */
            switch (s->read_idx) {
            case 0: byte = 0x01; break;
            case 1: byte = 0x02; break;
            case 2: byte = 0x03; break;
            case 3: byte = 0x04; break;
            default: byte = 0x00; break;
            }
            break;
        case 0x50008938:
            /* Channel num: 28 x 28 channels */
            switch (s->read_idx) {
            case 0: byte = 28; break;
            case 1: byte = 28; break;
            case 2: byte = 0x00; break;
            case 3: byte = 0x00; break;
            default: byte = 0x00; break;
            }
            break;
        default:
            byte = 0;
            break;
        }
        trace_raydium_ts_read(s->reg_addr, byte);
    } else {
        switch (s->reg8) {
        case 0x06:
            /* Firmware version: 0x000F1804 */
            switch (s->read_idx) {
            case 0: byte = 0x04; break;
            case 1: byte = 0x18; break;
            case 2: byte = 0x0F; break;
            case 3: byte = 0x00; break;
            default: byte = 0x00; break;
            }
            break;
        case 0x08:
            /* Display resolution: width, height */
            switch (s->read_idx) {
            case 0: byte = s->width & 0xFF; break;
            case 1: byte = (s->width >> 8) & 0xFF; break;
            case 2: byte = s->height & 0xFF; break;
            case 3: byte = (s->height >> 8) & 0xFF; break;
            default: byte = 0x00; break;
            }
            break;
        case 0x00:
            /* 16-byte touch report packet */
            if (s->q_count) {
                RaydiumEvent *ev = &s->q[s->q_head];
                switch (s->read_idx) {
                case 0: byte = 0x01; break;
                case 1: byte = (ev->state == 2) ? 0 : 1; break;
                case 2: byte = 0x00; break;
                case 3: byte = 0x40; break;
                case 4: byte = 0x00; break;
                case 5: byte = ev->x & 0xFF; break;
                case 6: byte = (ev->x >> 8) & 0xFF; break;
                case 7: byte = ev->y & 0xFF; break;
                case 8: byte = (ev->y >> 8) & 0xFF; break;
                case 13: byte = ev->state; break;
                default: byte = 0x00; break;
                }
            } else {
                switch (s->read_idx) {
                case 3: byte = 0x40; break;
                default: byte = 0x00; break;
                }
            }
            break;
        default:
            byte = 0;
            break;
        }
        trace_raydium_ts_read(s->reg8, byte);
    }

    s->read_idx++;
    return byte;
}

/* ---- host pointer ---- */

static void raydium_ts_input_event(DeviceState *dev, QemuConsole *src,
                                   QemuInputEvent *evt)
{
    RaydiumTsState *s = RAYDIUM_TS(dev);

    switch (evt->type) {
    case INPUT_EVENT_KIND_ABS: {
        InputMoveEvent *m = &evt->abs;
        int64_t v = m->value - INPUT_EVENT_ABS_MIN;
        int64_t range = INPUT_EVENT_ABS_MAX - INPUT_EVENT_ABS_MIN + 1;

        if (m->axis == INPUT_AXIS_X) {
            s->abs_x = v * s->width / range;
        } else if (m->axis == INPUT_AXIS_Y) {
            s->abs_y = v * s->height / range;
        }
        break;
    }
    case INPUT_EVENT_KIND_BTN: {
        InputBtnEvent *b = &evt->btn;

        if (b->button == INPUT_BUTTON_LEFT) {
            s->btn = b->down;
        }
        break;
    }
    default:
        break;
    }
}

static void raydium_ts_input_sync(DeviceState *dev)
{
    RaydiumTsState *s = RAYDIUM_TS(dev);
    int x = MAX(MIN(s->abs_x, (int)s->width - 1), 0);
    int y = MAX(MIN(s->abs_y, (int)s->height - 1), 0);

    if (s->btn != s->btn_reported) {
        if (s->btn) {
            raydium_ts_push(s, 1, x, y); /* DOWN */
        } else {
            raydium_ts_push(s, 2, s->last_x, s->last_y); /* UP */
        }
        s->btn_reported = s->btn;
    } else if (s->btn && (x != s->last_x || y != s->last_y)) {
        raydium_ts_push(s, 3, x, y); /* MOVE */
    }

    if (s->btn) {
        s->last_x = x;
        s->last_y = y;
    }
}

static const QemuInputHandler raydium_ts_input_handler = {
    .name  = "Raydium RM6D03x Touchscreen",
    .mask  = INPUT_EVENT_MASK_BTN | INPUT_EVENT_MASK_ABS,
    .event = raydium_ts_input_event,
    .sync  = raydium_ts_input_sync,
};

static void raydium_ts_reset(DeviceState *dev)
{
    RaydiumTsState *s = RAYDIUM_TS(dev);

    s->write_len = 0;
    s->reg_addr = 0;
    s->reg8 = 0;
    s->is_extended = false;
    s->read_idx = 0;
    s->reading_report = false;
    s->q_head = 0;
    s->q_count = 0;
    s->abs_x = 0;
    s->abs_y = 0;
    s->btn = false;
    s->btn_reported = false;
    s->last_x = 0;
    s->last_y = 0;
    raydium_ts_update_irq(s);
}

static void raydium_ts_realize(DeviceState *dev, Error **errp)
{
    RaydiumTsState *s = RAYDIUM_TS(dev);

    s->input = qemu_input_handler_register(dev, &raydium_ts_input_handler);
}

static void raydium_ts_unrealize(DeviceState *dev)
{
    RaydiumTsState *s = RAYDIUM_TS(dev);

    if (s->input) {
        qemu_input_handler_unregister(s->input);
        s->input = NULL;
    }
}

static void raydium_ts_init(Object *obj)
{
    RaydiumTsState *s = RAYDIUM_TS(obj);

    qdev_init_gpio_out_named(DEVICE(obj), &s->irq, "irq", 1);
}

static const Property raydium_ts_props[] = {
    DEFINE_PROP_UINT32("width", RaydiumTsState, width, 466),
    DEFINE_PROP_UINT32("height", RaydiumTsState, height, 466),
};

static void raydium_ts_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    dc->realize = raydium_ts_realize;
    dc->unrealize = raydium_ts_unrealize;
    device_class_set_legacy_reset(dc, raydium_ts_reset);
    device_class_set_props(dc, raydium_ts_props);
    k->event = raydium_ts_event;
    k->recv = raydium_ts_recv;
    k->send = raydium_ts_send;
}

static const TypeInfo raydium_ts_info = {
    .name = TYPE_RAYDIUM_TS,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(RaydiumTsState),
    .instance_init = raydium_ts_init,
    .class_init = raydium_ts_class_init,
};

static void raydium_ts_register_types(void)
{
    type_register_static(&raydium_ts_info);
}

type_init(raydium_ts_register_types)
