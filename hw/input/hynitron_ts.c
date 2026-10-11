/*
 * Hynitron CST92xx capacitive touch controller (I2C)
 *
 * Protocol as used by the AP firmware's drivers: Xiaomi's cst92xx.c
 * (Band 11 q66 4.100.155, vendor/xiaomi/miwear/drivers/input/hynitron)
 * and Hynitron's CST92xx_MCU_Driver_V3.8 (Band 9 n66 3.1.32, "hyn92xx").
 * Every transfer starts with a 16-bit command, high byte first; a read
 * returns the data of the last command written.
 *   - bootloader (enter_bootloader sub_C6E65A8 / sub_C153E94): after a
 *     reset, A0 01 AA enters it and A0 02 reads 55 B0; A0 01 00 leaves
 *   - chip id (read_chip_id sub_C8EECF4 / sub_C154790), bootloader memory
 *     read sub_C6E5BC4: A0 10 <bank>, A0 0C <addr, little-endian>,
 *     A0 04 E4 starts it, A0 04 reads 0 when done, A0 18 reads the word.
 *     Bank 1 / 0x077C is 0xCACA << 16 | part number, the drivers accept
 *     0x9217 and 0x9220; bank 0 / 0x7FC0 is the module id, which picks
 *     the firmware image the driver may reflash.  The default module id
 *     is listed in neither firmware's table, so no update is attempted.
 *   - work mode (set_work_mode sub_C6E5E38): D1 1E, then a mode command
 *     D1 <m> (09 normal, 01 debug info, 04 gesture sleep, 05 deep sleep,
 *     07 idle sleep, 23/24 wear detection on/off, 25/26 wrist turn
 *     on/off); 00 02 reads the controller's echo, byte 1 = <m>
 *   - firmware info, in debug info mode (get_firmware_info sub_C6E6260):
 *     D1 FC, D2 00, D2 04 (project id | ic type << 16), D2 08 (version)
 *     and D2 1C, their sum; D2 0C firmware checksum; D1 F4 tx / rx / key
 *     channel counts
 *   - ESD watchdog (sub_C6E66A8, sub_C1586EC; every 50 ms): D0 48, then a
 *     read whose byte 0 is 0x20 while the controller runs and byte 1 the
 *     wrist turn (bit 1) and wear detection (bit 0) states
 *   - report (read_point sub_C6E5C84 / sub_C154400): 10 bytes at D0 00:
 *     [0] id << 4 | state (6 down, 7 move, 0 up), [1] x >> 4, [2] y >> 4,
 *     [3] (x & 15) << 4 | (y & 15), [4] gesture << 4 or palm bit 7,
 *     [5] finger count, [6] 0xAB.  The driver acknowledges it with
 *     00 05 00 (Band 11) or D0 00 AB (Band 9).
 *   - interrupt: active-low line, asserted while a report is pending
 *   - gesture sleep: only wake gestures are reported, [4] = 1 single tap,
 *     2 double tap, 4 slide up
 *
 * Touches come from the host pointer (absolute pointer device): the left
 * button is the finger.  In gesture sleep the model plays the gesture
 * engine: two taps within 500 ms are a double tap, a lift-off a quarter of
 * the panel above the touch-down is a slide up.
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/bswap.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/input/hynitron_ts.h"
#include "ui/input.h"
#include "trace.h"

#define HYN_CMD_BOOT            0xA001  /* AA enters the bootloader, 00 leaves */
#define HYN_CMD_BOOT_ACK        0xA002  /* reads 55 B0 in the bootloader */
#define HYN_CMD_MEM_BANK        0xA010
#define HYN_CMD_MEM_ADDR        0xA00C
#define HYN_CMD_MEM_CTRL        0xA004  /* E4 = read; reads 0 when done */
#define HYN_CMD_MEM_DATA        0xA018
#define HYN_CMD_ECHO            0x0002
#define HYN_CMD_ACK             0x0005
#define HYN_CMD_POINT           0xD000
#define HYN_CMD_ESD             0xD048
#define HYN_MODE                0xD100  /* D1 <mode> */
#define HYN_INFO_MAGIC          0xD1FC
#define HYN_INFO_CHANNELS       0xD1F4
#define HYN_INFO_0              0xD200
#define HYN_INFO_IC             0xD204
#define HYN_INFO_VERSION        0xD208
#define HYN_INFO_CHECKSUM       0xD20C
#define HYN_INFO_SUM            0xD21C

#define HYN_MODE_DEBUG_INFO     0x01
#define HYN_MODE_GESTURE_SLEEP  0x04
#define HYN_MODE_DEEP_SLEEP     0x05
#define HYN_MODE_IDLE_SLEEP     0x07
#define HYN_MODE_NORMAL         0x09
#define HYN_MODE_WEAR_ON        0x23
#define HYN_MODE_WEAR_OFF       0x24
#define HYN_MODE_WRIST_ON       0x25
#define HYN_MODE_WRIST_OFF      0x26

#define HYN_STATE_UP            0
#define HYN_STATE_DOWN          6
#define HYN_STATE_MOVE          7
#define HYN_REPORT_MARK         0xAB
#define HYN_ESD_ALIVE           0x20

#define HYN_GESTURE_SINGLE_TAP  1
#define HYN_GESTURE_DOUBLE_TAP  2
#define HYN_GESTURE_SLIDE_UP    4
#define HYN_DOUBLE_TAP_MS       500
#define HYN_TAP_SLOP            24      /* pixels */

#define HYN_QUEUE               16
#define HYN_BUF                 16

typedef struct {
    uint8_t state;
    uint16_t x, y;
} HynitronEvent;

struct HynitronTsState {
    I2CSlave parent_obj;
    qemu_irq irq;
    QemuInputHandlerState *input;

    /* properties */
    uint32_t part_no;
    uint32_t module_id;
    uint32_t project_id;
    uint32_t fw_version;
    uint32_t fw_checksum;
    uint32_t tx_num;
    uint32_t rx_num;
    uint32_t width;
    uint32_t height;

    /* current transfer */
    uint8_t wbuf[HYN_BUF];
    unsigned nsent;
    uint16_t cmd;               /* command of the last write */
    unsigned rd_off;

    /* controller state */
    bool boot;                  /* in the bootloader */
    uint8_t mem_bank;
    uint16_t mem_addr;
    uint8_t mode;               /* last D1 <mode> */
    bool wear, wrist;
    bool asleep;                /* gesture sleep: wake gestures only */
    bool deep;                  /* deep / idle sleep: no reports */

    HynitronEvent q[HYN_QUEUE];
    unsigned q_head, q_count;
    uint8_t gesture;            /* pending wake gesture, 0 = none */
    int down_x, down_y;
    int64_t last_tap_ms;
    int last_tap_x, last_tap_y;

    /* host pointer */
    int abs_x, abs_y;
    bool btn, btn_reported;
    int last_x, last_y;
};

static void hyn_update_irq(HynitronTsState *s)
{
    qemu_set_irq(s->irq, !s->q_count && !s->gesture);   /* active low */
}

static void hyn_push(HynitronTsState *s, uint8_t state, int x, int y)
{
    HynitronEvent *tail;

    if (s->q_count) {
        tail = &s->q[(s->q_head + s->q_count - 1) % HYN_QUEUE];
        /* coalesce moves the guest has not fetched yet */
        if (s->q_count > 1 && state == HYN_STATE_MOVE &&
            tail->state == HYN_STATE_MOVE) {
            tail->x = x;
            tail->y = y;
            return;
        }
    }
    if (s->q_count == HYN_QUEUE) {
        return;
    }
    tail = &s->q[(s->q_head + s->q_count) % HYN_QUEUE];
    tail->state = state;
    tail->x = x;
    tail->y = y;
    s->q_count++;
    trace_hynitron_ts_event(state, x, y);
    hyn_update_irq(s);
}

static void hyn_mode(HynitronTsState *s, uint8_t mode);

/* the report or wake gesture was fetched */
static void hyn_ack(HynitronTsState *s)
{
    if (s->gesture) {
        s->gesture = 0;
        if (s->asleep) {
            hyn_mode(s, HYN_MODE_NORMAL);
        }
        qemu_irq_raise(s->irq);
    } else if (s->q_count) {
        s->q_head = (s->q_head + 1) % HYN_QUEUE;
        s->q_count--;
        /* release the line so the next report is a fresh edge */
        qemu_irq_raise(s->irq);
    }
    hyn_update_irq(s);
}

static void hyn_sleep(HynitronTsState *s, bool asleep, bool deep)
{
    s->asleep = asleep;
    s->deep = deep;
    s->gesture = 0;
    s->q_head = s->q_count = 0;
    s->last_tap_ms = INT64_MIN;
    s->btn_reported = s->btn;
    hyn_update_irq(s);
}

static void hyn_mode(HynitronTsState *s, uint8_t mode)
{
    switch (mode) {
    case HYN_MODE_NORMAL:
    case HYN_MODE_DEBUG_INFO:
        hyn_sleep(s, false, false);
        break;
    case HYN_MODE_GESTURE_SLEEP:
        hyn_sleep(s, true, false);
        break;
    case HYN_MODE_DEEP_SLEEP:
    case HYN_MODE_IDLE_SLEEP:
        hyn_sleep(s, false, true);
        break;
    case HYN_MODE_WEAR_ON:
    case HYN_MODE_WEAR_OFF:
        s->wear = mode == HYN_MODE_WEAR_ON;
        break;
    case HYN_MODE_WRIST_ON:
    case HYN_MODE_WRIST_OFF:
        s->wrist = mode == HYN_MODE_WRIST_ON;
        break;
    default:
        break;
    }
    s->mode = mode;
}

/* a complete write: the command and its data */
static void hyn_write(HynitronTsState *s)
{
    uint8_t *d = s->wbuf + 2;
    unsigned n = MIN(s->nsent, HYN_BUF) - 2;

    trace_hynitron_ts_write(s->cmd, n ? d[0] : 0, n);
    switch (s->cmd) {
    case HYN_CMD_BOOT:
        if (n) {
            s->boot = d[0] == 0xAA;
        }
        break;
    case HYN_CMD_MEM_BANK:
        if (n) {
            s->mem_bank = d[0];
        }
        break;
    case HYN_CMD_MEM_ADDR:
        if (n >= 2) {
            s->mem_addr = lduw_le_p(d);
        }
        break;
    case HYN_CMD_ACK:
        hyn_ack(s);
        break;
    case HYN_CMD_POINT:
        if (n && d[0] == HYN_REPORT_MARK) {
            hyn_ack(s);
        }
        break;
    default:
        if ((s->cmd & 0xFF00) == HYN_MODE && !s->boot) {
            hyn_mode(s, s->cmd & 0xFF);
        }
        break;
    }
}

static uint32_t hyn_mem_word(HynitronTsState *s)
{
    if (s->mem_bank == 1 && s->mem_addr == 0x077C) {
        return 0xCACA0000u | (s->part_no & 0xFFFF);
    }
    if (s->mem_bank == 0 && s->mem_addr == 0x7FC0) {
        return s->module_id;
    }
    return 0;
}

static uint8_t hyn_read_byte(HynitronTsState *s, unsigned off)
{
    const HynitronEvent *e = s->q_count ? &s->q[s->q_head] : NULL;
    uint32_t info0 = 0xCACA0000u | (s->part_no & 0xFFFF);
    uint32_t ic = (s->part_no & 0xFFFF) << 16 | (s->project_id & 0xFFFF);
    uint32_t w;

    switch (s->cmd) {
    case HYN_CMD_BOOT_ACK:
        return s->boot ? (off ? 0xB0 : 0x55) : 0;
    case HYN_CMD_MEM_CTRL:
        return 0;                       /* memory read finished */
    case HYN_CMD_MEM_DATA:
        return off < 4 ? hyn_mem_word(s) >> (off * 8) : 0;
    case HYN_CMD_ECHO:
        return off == 1 ? s->mode : 0;
    case HYN_CMD_ESD:
        if (off == 0) {
            return HYN_ESD_ALIVE;
        }
        return off == 1 ? (s->wrist ? 2 : 0) | (s->wear ? 1 : 0) : 0;
    case HYN_CMD_POINT:
        switch (off) {
        case 0:
            return e ? e->state : 0;    /* finger id 0 */
        case 1:
            return e ? e->x >> 4 : 0;
        case 2:
            return e ? e->y >> 4 : 0;
        case 3:
            return e ? (e->x & 15) << 4 | (e->y & 15) : 0;
        case 4:
            return s->gesture << 4;
        case 5:
            return e ? 1 : 0;
        case 6:
            return HYN_REPORT_MARK;
        default:
            return 0;
        }
    case HYN_INFO_MAGIC:
        w = info0;
        break;
    case HYN_INFO_0:
        w = 0;
        break;
    case HYN_INFO_IC:
        w = ic;
        break;
    case HYN_INFO_VERSION:
        w = s->fw_version;
        break;
    case HYN_INFO_SUM:
        w = info0 + ic + s->fw_version;
        break;
    case HYN_INFO_CHECKSUM:
        w = s->fw_checksum;
        break;
    case HYN_INFO_CHANNELS:
        w = (s->rx_num & 0xFF) << 16 | (s->tx_num & 0xFF);
        break;
    default:
        return 0;
    }
    return off < 4 ? w >> (off * 8) : 0;
}

static int hyn_event(I2CSlave *i2c, enum i2c_event event)
{
    HynitronTsState *s = HYNITRON_TS(i2c);

    switch (event) {
    case I2C_START_SEND:
        s->nsent = 0;
        break;
    case I2C_START_RECV:
        s->rd_off = 0;
        break;
    case I2C_FINISH:
        if (s->nsent >= 2) {
            hyn_write(s);
        }
        s->nsent = 0;
        break;
    default:
        break;
    }
    return 0;
}

static int hyn_send(I2CSlave *i2c, uint8_t data)
{
    HynitronTsState *s = HYNITRON_TS(i2c);

    if (s->nsent < HYN_BUF) {
        s->wbuf[s->nsent] = data;
    }
    if (++s->nsent == 2) {
        s->cmd = s->wbuf[0] << 8 | s->wbuf[1];
    }
    return 0;
}

static uint8_t hyn_recv(I2CSlave *i2c)
{
    HynitronTsState *s = HYNITRON_TS(i2c);
    uint8_t b = hyn_read_byte(s, s->rd_off++);

    trace_hynitron_ts_read(s->cmd, s->rd_off - 1, b);
    return b;
}

/* ---- host pointer ---- */

static void hyn_input_event(DeviceState *dev, QemuConsole *src,
                            QemuInputEvent *evt)
{
    HynitronTsState *s = HYNITRON_TS(dev);

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

/* gesture sleep: only a finished double tap or slide up wakes the host */
static void hyn_gesture_sync(HynitronTsState *s, int x, int y)
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
    if (s->gesture) {
        return;
    }
    if (s->down_y - y >= (int)s->height / 4) {
        s->gesture = HYN_GESTURE_SLIDE_UP;
    } else if (ABS(x - s->down_x) <= HYN_TAP_SLOP &&
               ABS(y - s->down_y) <= HYN_TAP_SLOP) {
        now = qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL);
        if (now - s->last_tap_ms <= HYN_DOUBLE_TAP_MS &&
            ABS(x - s->last_tap_x) <= HYN_TAP_SLOP * 2 &&
            ABS(y - s->last_tap_y) <= HYN_TAP_SLOP * 2) {
            s->gesture = HYN_GESTURE_DOUBLE_TAP;
            s->last_tap_ms = INT64_MIN;
        } else {
            s->last_tap_ms = now;
            s->last_tap_x = x;
            s->last_tap_y = y;
        }
    }
    if (s->gesture) {
        trace_hynitron_ts_gesture(s->gesture);
        hyn_update_irq(s);
    }
}

static void hyn_input_sync(DeviceState *dev)
{
    HynitronTsState *s = HYNITRON_TS(dev);
    int x = MIN(s->abs_x, (int)s->width - 1);
    int y = MIN(s->abs_y, (int)s->height - 1);

    if (s->boot || s->deep) {
        s->btn_reported = s->btn;
        return;
    }
    if (s->asleep) {
        hyn_gesture_sync(s, x, y);
        return;
    }
    if (s->btn != s->btn_reported) {
        if (s->btn) {
            hyn_push(s, HYN_STATE_DOWN, x, y);
        } else {
            hyn_push(s, HYN_STATE_UP, s->last_x, s->last_y);
        }
        s->btn_reported = s->btn;
    } else if (s->btn && (x != s->last_x || y != s->last_y)) {
        hyn_push(s, HYN_STATE_MOVE, x, y);
    }
    if (s->btn) {
        s->last_x = x;
        s->last_y = y;
    }
}

static const QemuInputHandler hyn_input_handler = {
    .name  = "Hynitron touchscreen",
    .mask  = INPUT_EVENT_MASK_BTN | INPUT_EVENT_MASK_ABS,
    .event = hyn_input_event,
    .sync  = hyn_input_sync,
};

/* ---- device ---- */

static void hyn_reset(DeviceState *dev)
{
    HynitronTsState *s = HYNITRON_TS(dev);

    s->nsent = 0;
    s->cmd = 0;
    s->rd_off = 0;
    s->boot = false;
    s->mem_bank = 0;
    s->mem_addr = 0;
    s->mode = HYN_MODE_NORMAL;
    s->wear = s->wrist = false;
    hyn_sleep(s, false, false);
}

static void hyn_realize(DeviceState *dev, Error **errp)
{
    HynitronTsState *s = HYNITRON_TS(dev);

    s->input = qemu_input_handler_register(dev, &hyn_input_handler);
    qemu_input_handler_activate(s->input);
}

static void hyn_init(Object *obj)
{
    HynitronTsState *s = HYNITRON_TS(obj);

    qdev_init_gpio_out_named(DEVICE(obj), &s->irq, "irq", 1);
}

static const Property hyn_props[] = {
    DEFINE_PROP_UINT32("part-no", HynitronTsState, part_no, 0x9217),
    DEFINE_PROP_UINT32("module-id", HynitronTsState, module_id, 0),
    DEFINE_PROP_UINT32("project-id", HynitronTsState, project_id, 0x0001),
    DEFINE_PROP_UINT32("fw-version", HynitronTsState, fw_version, 0x0100),
    DEFINE_PROP_UINT32("fw-checksum", HynitronTsState, fw_checksum, 0),
    DEFINE_PROP_UINT32("tx-num", HynitronTsState, tx_num, 12),
    DEFINE_PROP_UINT32("rx-num", HynitronTsState, rx_num, 24),
    DEFINE_PROP_UINT32("width", HynitronTsState, width, 212),
    DEFINE_PROP_UINT32("height", HynitronTsState, height, 520),
};

static void hyn_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    dc->realize = hyn_realize;
    device_class_set_legacy_reset(dc, hyn_reset);
    device_class_set_props(dc, hyn_props);
    k->event = hyn_event;
    k->send = hyn_send;
    k->recv = hyn_recv;
}

static const TypeInfo hyn_info = {
    .name = TYPE_HYNITRON_TS,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(HynitronTsState),
    .instance_init = hyn_init,
    .class_init = hyn_class_init,
};

static void hyn_register_types(void)
{
    type_register_static(&hyn_info);
}

type_init(hyn_register_types)
