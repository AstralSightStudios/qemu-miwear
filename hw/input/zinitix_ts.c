/*
 * Zinitix ZTW6xx capacitive touch controller (I2C)
 *
 * Protocol as used by the AP firmware's driver
 * (vendor/bes/drivers/best1503_ep/drivers/input/ztx/zinitix.c in
 * vela_ap.bin; register names as in the mainline Linux zinitix driver):
 *   - 16-bit little-endian register addresses; register values are 16-bit
 *     little-endian and multi-byte reads walk consecutive registers
 *   - a 2-byte write with no data is a command: 0x0003 clears the
 *     interrupt (sent after every report), 0x0000..0x0007 soft reset /
 *     wake-up / idle / sleep / calibrate
 *   - power-up (ts_power_sequence sub_C3F5B1C): 0xC000 = 1, read 0xCC00,
 *     command 0xC004, 0xC002 = 1, 0xC001 = 1
 *   - identification (sub_C3F5DA8): chip id at 0xCC00 (the driver accepts
 *     0xE623 / 0xE631), 0x09D3 = 9 then a 12-byte read at 0x09D2 whose
 *     bytes 8-9 are the hardware id, firmware version 0x0012, minor
 *     version 0x0121, register version 0x0013
 *   - firmware update check (ts_check_need_upgrade sub_C3F78AC): the
 *     checksum result register 0x012C must read 0x55AA and 0x0014 must
 *     equal the hardware id, otherwise the driver reflashes the controller
 *   - ESD watchdog (sub_C3F815C, every few seconds): register 0x001C must
 *     read 0x5A49 while the controller runs, otherwise it is reset (the
 *     real device log has no such resets)
 *   - report (sub_C3F5F94): 10 bytes at 0x0080 = status (u16), finger
 *     mask, time stamp, x, y (u16), width, sub status; sub status bit 1
 *     down, bits 0/2 present/move, bit 3 up; status bit 4 palm
 *   - interrupt: active-low line, asserted while a report is pending
 *   - screen off (ztw662_ts_suspend sub_C3F6D6C): gesture enable mask to
 *     0x00AF (0x0002 / 0x0004 / 0x0010 / 0x0016 by display state),
 *     command 0x0003, then command 0x0005 (sleep).  Asleep, the controller
 *     only reports wake gestures: the interrupt handler (sub_C3F5F94, while
 *     suspended) reads the gesture code at 0x0126, 1 = double tap,
 *     4 = slide up (honoured per the touch_double_click / touch_slide_up
 *     settings; with both off the firmware forces double tap on), then
 *     clears the interrupt with 0x0003.  Resume (sub_C3F67BC) reruns the
 *     power-up sequence, which wakes the controller.
 * Real hardware (Xiaomi Smart Band 10 Pro log): chip id 0xE623, hardware
 * id 0x4702, firmware 0x9 / 0x0 / 0x12.
 *
 * Touches come from the host pointer (absolute pointer device): the left
 * button is the finger.  While asleep the model plays the controller's
 * gesture engine: two taps within 500 ms are a double tap, a lift-off a
 * quarter of the panel above the touch-down is a slide up.
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/input/zinitix_ts.h"
#include "ui/input.h"
#include "trace.h"

#define ZT_REG_COUNT            0x10000

#define ZT_CMD_SWRESET          0x0000
#define ZT_CMD_WAKEUP           0x0001
#define ZT_CMD_CLEAR_INT        0x0003
#define ZT_CMD_SLEEP            0x0005
#define ZT_REG_FW_VERSION       0x0012
#define ZT_REG_REG_VERSION      0x0013
#define ZT_REG_HW_ID            0x0014
#define ZT_REG_SUPPORTED_FINGER 0x0015
#define ZT_REG_ESD_CHECK        0x001C
#define ZT_ESD_ALIVE            0x5A49
#define ZT_REG_POINT_STATUS     0x0080
#define ZT_REG_X_RESOLUTION     0x00C0
#define ZT_REG_Y_RESOLUTION     0x00C1
#define ZT_REG_GESTURE_EN       0x00AF
#define ZT_REG_MINOR_FW_VERSION 0x0121
#define ZT_REG_GESTURE          0x0126
#define ZT_REG_CHECKSUM_RESULT  0x012C
#define ZT_CHECKSUM_OK          0x55AA
#define ZT_REG_INFO             0x09D2      /* 12 bytes, hw id at +8 */
#define ZT_REG_CHIP_ID          0xCC00
#define ZT_REG_VENDOR_CMD       0xC000      /* first step of power-up */

#define ZT_GESTURE_DOUBLE_TAP   1
#define ZT_GESTURE_SLIDE_UP     4
#define ZT_DOUBLE_TAP_MS        500
#define ZT_TAP_SLOP             24          /* pixels */

/* status / sub status bits (Linux zinitix.c) */
#define ZT_ST_PT_CNT_CHANGE     (1u << 0)
#define ZT_ST_DOWN              (1u << 1)
#define ZT_ST_MOVE              (1u << 2)
#define ZT_ST_UP                (1u << 3)
#define ZT_ST_PT_EXIST          (1u << 11)
#define ZT_SUB_EXIST            (1u << 0)
#define ZT_SUB_DOWN             (1u << 1)
#define ZT_SUB_MOVE             (1u << 2)
#define ZT_SUB_UP               (1u << 3)

#define ZT_QUEUE                16

typedef struct {
    uint16_t status;
    uint8_t sub;
    uint16_t x, y;
} ZinitixEvent;

struct ZinitixTsState {
    I2CSlave parent_obj;
    qemu_irq irq;
    QemuInputHandlerState *input;

    /* properties */
    uint32_t chip_id;
    uint32_t hw_id;
    uint32_t fw_version;
    uint32_t fw_minor;
    uint32_t reg_version;
    uint32_t width;
    uint32_t height;

    uint16_t regs[ZT_REG_COUNT];
    uint16_t ptr;               /* register addressed by the last write */
    unsigned nsent;             /* bytes of the current write */
    unsigned rd_off;            /* byte offset of the current read */

    ZinitixEvent q[ZT_QUEUE];
    unsigned q_head, q_count;

    /* sleep (screen off): wake-gesture detection only */
    bool asleep;
    uint16_t gesture;           /* pending gesture code, 0 = none */
    int down_x, down_y;
    int64_t last_tap_ms;
    int last_tap_x, last_tap_y;

    /* host pointer */
    int abs_x, abs_y;
    bool btn, btn_reported;
    int last_x, last_y;
};

static void zt_update_irq(ZinitixTsState *s)
{
    qemu_set_irq(s->irq, !s->q_count && !s->gesture);   /* active low */
}

static void zt_push(ZinitixTsState *s, uint16_t status, uint8_t sub,
                    int x, int y)
{
    ZinitixEvent *tail;

    if (s->q_count) {
        tail = &s->q[(s->q_head + s->q_count - 1) % ZT_QUEUE];
        /* coalesce moves the guest has not fetched yet */
        if (s->q_count > 1 && (sub & ZT_SUB_MOVE) &&
            (tail->sub & ZT_SUB_MOVE)) {
            tail->x = x;
            tail->y = y;
            return;
        }
    }
    if (s->q_count == ZT_QUEUE) {
        return;
    }
    tail = &s->q[(s->q_head + s->q_count) % ZT_QUEUE];
    tail->status = status;
    tail->sub = sub;
    tail->x = x;
    tail->y = y;
    s->q_count++;
    trace_zinitix_ts_event(status, sub, x, y);
    zt_update_irq(s);
}

static uint16_t zt_reg_read(ZinitixTsState *s, uint16_t reg)
{
    const ZinitixEvent *e = s->q_count ? &s->q[s->q_head] : NULL;

    switch (reg) {
    case ZT_REG_POINT_STATUS:
        return e ? e->status : 0;
    case ZT_REG_POINT_STATUS + 1:       /* finger mask | time stamp */
        return e && (e->sub & ZT_SUB_EXIST) ? 1 : 0;
    case ZT_REG_POINT_STATUS + 2:
        return e ? e->x : 0;
    case ZT_REG_POINT_STATUS + 3:
        return e ? e->y : 0;
    case ZT_REG_POINT_STATUS + 4:       /* width | sub status */
        return e ? (e->sub << 8) | (e->sub & ZT_SUB_UP ? 0 : 8) : 0;
    case ZT_REG_GESTURE:
        return s->gesture;
    default:
        return s->regs[reg];
    }
}

static void zt_set_asleep(ZinitixTsState *s, bool asleep)
{
    s->asleep = asleep;
    s->gesture = 0;
    s->q_head = s->q_count = 0;
    s->last_tap_ms = INT64_MIN;
    s->btn_reported = s->btn;
    zt_update_irq(s);
}

static void zt_command(ZinitixTsState *s, uint16_t cmd)
{
    trace_zinitix_ts_command(cmd);
    switch (cmd) {
    case ZT_CMD_CLEAR_INT:
        if (s->gesture) {
            s->gesture = 0;
            qemu_irq_raise(s->irq);
        }
        if (s->q_count) {
            s->q_head = (s->q_head + 1) % ZT_QUEUE;
            s->q_count--;
            /* release the line so the next report is a fresh edge */
            qemu_irq_raise(s->irq);
        }
        zt_update_irq(s);
        break;
    case ZT_CMD_SWRESET:
    case ZT_CMD_WAKEUP:
        zt_set_asleep(s, false);
        break;
    case ZT_CMD_SLEEP:
        zt_set_asleep(s, true);
        break;
    default:
        break;                          /* wake-up / mode / vendor */
    }
}

static int zt_event(I2CSlave *i2c, enum i2c_event event)
{
    ZinitixTsState *s = ZINITIX_TS(i2c);

    switch (event) {
    case I2C_START_SEND:
        s->nsent = 0;
        break;
    case I2C_START_RECV:
        s->rd_off = 0;
        break;
    case I2C_FINISH:
        if (s->nsent == 2) {
            zt_command(s, s->ptr);
        }
        s->nsent = 0;
        break;
    default:
        break;
    }
    return 0;
}

static int zt_send(I2CSlave *i2c, uint8_t data)
{
    ZinitixTsState *s = ZINITIX_TS(i2c);
    unsigned n = s->nsent++;

    if (n < 2) {
        s->ptr = deposit32(s->ptr, n * 8, 8, data);
        s->rd_off = 0;
    } else {
        uint16_t reg = s->ptr + (n - 2) / 2;

        s->regs[reg] = deposit32(s->regs[reg], ((n - 2) & 1) * 8, 8, data);
        if ((n - 2) & 1) {
            trace_zinitix_ts_write(reg, s->regs[reg]);
            if (reg == ZT_REG_VENDOR_CMD && s->asleep) {
                zt_set_asleep(s, false);
            }
        }
    }
    return 0;
}

static uint8_t zt_recv(I2CSlave *i2c)
{
    ZinitixTsState *s = ZINITIX_TS(i2c);
    uint16_t v = zt_reg_read(s, s->ptr + s->rd_off / 2);
    uint8_t b = (s->rd_off & 1) ? v >> 8 : v;

    s->rd_off++;
    return b;
}

/* ---- host pointer ---- */

static void zt_input_event(DeviceState *dev, QemuConsole *src,
                           QemuInputEvent *evt)
{
    ZinitixTsState *s = ZINITIX_TS(dev);

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

/* asleep: only a finished double tap or slide up wakes the host */
static void zt_gesture_sync(ZinitixTsState *s, int x, int y)
{
    int64_t now;

    if (s->btn == s->btn_reported) {
        return;
    }
    s->btn_reported = s->btn;
    if (s->btn) {
        s->down_x = x;
        s->down_y = y;
        return;
    }
    if (!s->regs[ZT_REG_GESTURE_EN] || s->gesture) {
        return;
    }
    if (s->down_y - y >= (int)s->height / 4) {
        s->gesture = ZT_GESTURE_SLIDE_UP;
    } else if (ABS(x - s->down_x) <= ZT_TAP_SLOP &&
               ABS(y - s->down_y) <= ZT_TAP_SLOP) {
        now = qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL);
        if (now - s->last_tap_ms <= ZT_DOUBLE_TAP_MS &&
            ABS(x - s->last_tap_x) <= ZT_TAP_SLOP * 2 &&
            ABS(y - s->last_tap_y) <= ZT_TAP_SLOP * 2) {
            s->gesture = ZT_GESTURE_DOUBLE_TAP;
            s->last_tap_ms = INT64_MIN;
        } else {
            s->last_tap_ms = now;
            s->last_tap_x = x;
            s->last_tap_y = y;
        }
    }
    if (s->gesture) {
        trace_zinitix_ts_gesture(s->gesture);
        zt_update_irq(s);
    }
}

static void zt_input_sync(DeviceState *dev)
{
    ZinitixTsState *s = ZINITIX_TS(dev);
    int x = MIN(s->abs_x, (int)s->width - 1);
    int y = MIN(s->abs_y, (int)s->height - 1);

    if (s->asleep) {
        zt_gesture_sync(s, x, y);
        return;
    }
    if (s->btn != s->btn_reported) {
        if (s->btn) {
            zt_push(s, ZT_ST_PT_CNT_CHANGE | ZT_ST_DOWN | ZT_ST_PT_EXIST,
                    ZT_SUB_EXIST | ZT_SUB_DOWN, x, y);
        } else {
            zt_push(s, ZT_ST_PT_CNT_CHANGE | ZT_ST_UP, ZT_SUB_UP,
                    s->last_x, s->last_y);
        }
        s->btn_reported = s->btn;
    } else if (s->btn && (x != s->last_x || y != s->last_y)) {
        zt_push(s, ZT_ST_MOVE | ZT_ST_PT_EXIST, ZT_SUB_EXIST | ZT_SUB_MOVE,
                x, y);
    }
    if (s->btn) {
        s->last_x = x;
        s->last_y = y;
    }
}

static const QemuInputHandler zt_input_handler = {
    .name  = "Zinitix touchscreen",
    .mask  = INPUT_EVENT_MASK_BTN | INPUT_EVENT_MASK_ABS,
    .event = zt_input_event,
    .sync  = zt_input_sync,
};

/* ---- device ---- */

static void zt_reset(DeviceState *dev)
{
    ZinitixTsState *s = ZINITIX_TS(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[ZT_REG_CHIP_ID] = s->chip_id;
    s->regs[ZT_REG_INFO + 4] = s->hw_id;
    s->regs[ZT_REG_HW_ID] = s->hw_id;
    s->regs[ZT_REG_FW_VERSION] = s->fw_version;
    s->regs[ZT_REG_MINOR_FW_VERSION] = s->fw_minor;
    s->regs[ZT_REG_REG_VERSION] = s->reg_version;
    s->regs[ZT_REG_CHECKSUM_RESULT] = ZT_CHECKSUM_OK;
    s->regs[ZT_REG_ESD_CHECK] = ZT_ESD_ALIVE;
    s->regs[ZT_REG_SUPPORTED_FINGER] = 1;
    s->regs[ZT_REG_X_RESOLUTION] = s->width;
    s->regs[ZT_REG_Y_RESOLUTION] = s->height;
    s->ptr = 0;
    s->nsent = 0;
    s->rd_off = 0;
    zt_set_asleep(s, false);
}

static void zt_realize(DeviceState *dev, Error **errp)
{
    ZinitixTsState *s = ZINITIX_TS(dev);

    s->input = qemu_input_handler_register(dev, &zt_input_handler);
    qemu_input_handler_activate(s->input);
}

static void zt_init(Object *obj)
{
    ZinitixTsState *s = ZINITIX_TS(obj);

    qdev_init_gpio_out_named(DEVICE(obj), &s->irq, "irq", 1);
}

static const Property zt_props[] = {
    DEFINE_PROP_UINT32("chip-id", ZinitixTsState, chip_id, 0xE623),
    DEFINE_PROP_UINT32("hw-id", ZinitixTsState, hw_id, 0x4702),
    DEFINE_PROP_UINT32("fw-version", ZinitixTsState, fw_version, 0x9),
    DEFINE_PROP_UINT32("fw-minor", ZinitixTsState, fw_minor, 0x0),
    DEFINE_PROP_UINT32("reg-version", ZinitixTsState, reg_version, 0x12),
    DEFINE_PROP_UINT32("width", ZinitixTsState, width, 336),
    DEFINE_PROP_UINT32("height", ZinitixTsState, height, 480),
};

static void zt_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    dc->realize = zt_realize;
    device_class_set_legacy_reset(dc, zt_reset);
    device_class_set_props(dc, zt_props);
    k->event = zt_event;
    k->send = zt_send;
    k->recv = zt_recv;
}

static const TypeInfo zt_info = {
    .name = TYPE_ZINITIX_TS,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(ZinitixTsState),
    .instance_init = zt_init,
    .class_init = zt_class_init,
};

static void zt_register_types(void)
{
    type_register_static(&zt_info);
}

type_init(zt_register_types)
