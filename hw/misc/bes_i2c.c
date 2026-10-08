/*
 * BES I2C master controller for best1503 — DesignWare APb-I2C compatible.
 *
 * Register layout from SDK reg_i2cip.h (DW_apb_i2c flavour):
 *   0x00 CON, 0x04 TAR, 0x08 SAR, 0x10 DATA_CMD (bit8 READ, bit9 STOP,
 *   bit10 RESTART), 0x14..0x28 SCL HCNT/LCNT, 0x2C INT_STATUS,
 *   0x30 INT_MASK, 0x34 RAW_INT_STATUS, 0x38 RX_TL, 0x3C TX_TL,
 *   0x40 CLR_INTR .. 0x68 CLR_GEN_CALL (read-to-clear), 0x6C ENABLE,
 *   0x70 STATUS (bit0 ACT, bit1 TFNF, bit2 TFE, bit3 RFNE, bit4 RFF),
 *   0x74 TXFLR, 0x78 RXFLR, 0x9C ENABLE_STATUS.
 *
 * Firmware flow (hal_i2c.c, polled — no IRQ):
 *   write: wait TFNF, push DATA_CMD=data|WRITE(|RESTART)(|STOP),
 *          finally wait (STATUS & (TFE|ACT)) == TFE.
 *   read:  push DATA_CMD=READ(|STOP), wait RFNE, pop DATA_CMD.
 *
 * Modelled behaviour: transfers complete instantly.
 *   - WRITE bytes collect in the TX fifo; a STOP (or a following READ
 *     phase) flushes them to the slave as one write segment (first byte
 *     is the slave register pointer).
 *   - READ performs the restart-read: pending TX bytes go out as the
 *     register address, then one byte is clocked in per READ entry.
 *   - STATUS: TFE|TFNF always set, RFNE while RX data pending, ACT=0.
 *
 * Slaves: generic per-address 256-byte register files. Every 7-bit
 * address ACKs (the watch carries charger/fuel-gauge/touch/etc. on the
 * same bus; unanswered probes would stall boot in retry loops).
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/misc/bes_i2c.h"

/* DATA_CMD bits */
#define DCMD_DATA(x)    ((x) & 0xFF)
#define DCMD_READ       (1 << 8)
#define DCMD_STOP       (1 << 9)
#define DCMD_RESTART    (1 << 10)

/* RAW_INT bits */
#define INTR_RX_UNDER   (1 << 0)
#define INTR_RX_OVER    (1 << 1)
#define INTR_RX_FULL    (1 << 2)
#define INTR_TX_OVER    (1 << 3)
#define INTR_TX_EMPTY   (1 << 4)
#define INTR_RD_REQ     (1 << 5)
#define INTR_TX_ABRT    (1 << 6)
#define INTR_RX_DONE    (1 << 7)
#define INTR_ACTIVITY   (1 << 8)
#define INTR_STOP_DET   (1 << 9)
#define INTR_START_DET  (1 << 10)
#define INTR_GEN_CALL   (1 << 11)

/* STATUS bits */
#define ST_ACT          (1 << 0)
#define ST_TFNF         (1 << 1)
#define ST_TFE          (1 << 2)
#define ST_RFNE         (1 << 3)
#define ST_RFF          (1 << 4)

#define I2C_FIFO_DEPTH  8

typedef struct BesI2cSlave {
    bool inited;
    uint8_t ptr;
    uint8_t regs[256];
} BesI2cSlave;

struct BesI2cState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t con, tar, sar;
    uint32_t int_mask, raw_int;
    uint32_t rx_tl, tx_tl;
    uint32_t enable;
    uint32_t scl_cfg[6];

    uint8_t txfifo[I2C_FIFO_DEPTH];
    uint32_t tx_cnt;
    uint8_t rxfifo[I2C_FIFO_DEPTH];
    uint32_t rx_r, rx_w, rx_cnt;    /* ring: rx_cnt = unread bytes (0..8) */

    bool started;       /* a write segment has addressed a slave */
    BesI2cSlave slaves[128];
};

static BesI2cSlave *bes_i2c_slave(BesI2cState *s, uint32_t addr7)
{
    BesI2cSlave *sl = &s->slaves[addr7 & 0x7F];

    if (!sl->inited) {
        memset(sl->regs, 0, sizeof(sl->regs));
        sl->ptr = 0;
        sl->inited = true;
    }
    return sl;
}

/* push the pending TX segment to the slave: first byte is the register
 * pointer, remaining bytes are written sequentially */
static void bes_i2c_flush_tx(BesI2cState *s)
{
    BesI2cSlave *sl;
    uint32_t i;

    if (!s->tx_cnt) {
        return;
    }
    sl = bes_i2c_slave(s, s->tar);
    sl->ptr = s->txfifo[0];
    for (i = 1; i < s->tx_cnt; i++) {
        sl->regs[sl->ptr++] = s->txfifo[i];
    }
    s->tx_cnt = 0;
    s->started = true;
}

static uint8_t bes_i2c_slave_read(BesI2cState *s)
{
    BesI2cSlave *sl = bes_i2c_slave(s, s->tar);

    return sl->regs[sl->ptr++];
}

static void bes_i2c_update_irq(BesI2cState *s)
{
    /* TX_EMPTY reflects the empty TX fifo; RX_FULL when over threshold */
    if (s->tx_cnt == 0) {
        s->raw_int |= INTR_TX_EMPTY;
    } else {
        s->raw_int &= ~INTR_TX_EMPTY;
    }
    if (s->rx_cnt > s->rx_tl) {
        s->raw_int |= INTR_RX_FULL;
    } else {
        s->raw_int &= ~INTR_RX_FULL;
    }
    qemu_set_irq(s->irq, !!(s->raw_int & s->int_mask));
}

static void bes_i2c_data_cmd_write(BesI2cState *s, uint32_t val)
{
    if (val & DCMD_READ) {
        /* read phase: flush any pending register-address segment first */
        bes_i2c_flush_tx(s);
        if (s->rx_cnt < I2C_FIFO_DEPTH) {
            s->rxfifo[s->rx_w] = bes_i2c_slave_read(s);
            s->rx_w = (s->rx_w + 1) % I2C_FIFO_DEPTH;
            s->rx_cnt++;
        } else {
            s->raw_int |= INTR_RX_OVER;
        }
    } else {
        if (s->tx_cnt < I2C_FIFO_DEPTH) {
            s->txfifo[s->tx_cnt++] = DCMD_DATA(val);
        } else {
            s->raw_int |= INTR_TX_OVER;
        }
    }
    if (val & DCMD_STOP) {
        bes_i2c_flush_tx(s);
        s->started = false;
        s->raw_int |= INTR_STOP_DET;
    }
}

static uint64_t bes_i2c_read(void *opaque, hwaddr offset, unsigned size)
{
    BesI2cState *s = opaque;
    uint64_t v = 0;

    switch (offset) {
    case 0x00: v = s->con; break;
    case 0x04: v = s->tar; break;
    case 0x08: v = s->sar; break;
    case 0x10: /* DATA_CMD read pops the RX fifo */
        if (s->rx_cnt) {
            v = s->rxfifo[s->rx_r];
            s->rx_r = (s->rx_r + 1) % I2C_FIFO_DEPTH;
            s->rx_cnt--;
        } else {
            s->raw_int |= INTR_RX_UNDER;
        }
        break;
    case 0x14: case 0x18: case 0x1C:
    case 0x20: case 0x24: case 0x28:
        v = s->scl_cfg[(offset - 0x14) / 4]; break;
    case 0x2C: v = s->raw_int & s->int_mask; break;
    case 0x30: v = s->int_mask; break;
    case 0x34: v = s->raw_int; break;
    case 0x38: v = s->rx_tl; break;
    case 0x3C: v = s->tx_tl; break;
    case 0x40: s->raw_int = 0; v = 0; break;              /* CLR_INTR */
    case 0x44: s->raw_int &= ~INTR_RX_UNDER; v = 0; break;
    case 0x48: s->raw_int &= ~INTR_RX_OVER;  v = 0; break;
    case 0x4C: s->raw_int &= ~INTR_TX_OVER;  v = 0; break;
    case 0x50: s->raw_int &= ~INTR_RD_REQ;   v = 0; break;
    case 0x54: s->raw_int &= ~INTR_TX_ABRT;  v = 0; break;
    case 0x58: s->raw_int &= ~INTR_RX_DONE;  v = 0; break;
    case 0x5C: s->raw_int &= ~INTR_ACTIVITY; v = 0; break;
    case 0x60: s->raw_int &= ~INTR_STOP_DET; v = 0; break;
    case 0x64: s->raw_int &= ~INTR_START_DET; v = 0; break;
    case 0x68: s->raw_int &= ~INTR_GEN_CALL; v = 0; break;
    case 0x6C: v = s->enable; break;
    case 0x70: /* STATUS: transfers complete instantly */
        v = ST_TFE | ST_TFNF;
        if (s->rx_cnt) {
            v |= ST_RFNE;
        }
        if (s->rx_cnt >= I2C_FIFO_DEPTH) {
            v |= ST_RFF;
        }
        break;
    case 0x74: v = s->tx_cnt; break;
    case 0x78: v = s->rx_cnt; break;
    case 0x9C: v = s->enable; break;   /* ENABLE_STATUS: never busy */
    case 0xF4: v = 0x00070707; break;  /* COMP_PARAM_1: fifo depth 8 */
    case 0xF8: v = 0x3131352A; break;  /* COMP_VERSION "115*" */
    case 0xFC: v = 0x44570140; break;  /* COMP_TYPE = DW_apb_i2c */
    default:
        qemu_log_mask(LOG_UNIMP, "bes.i2c: read off=0x%03x\n", (int)offset);
        break;
    }
    bes_i2c_update_irq(s);
    return v;
}

static void bes_i2c_write(void *opaque, hwaddr offset, uint64_t val,
                          unsigned size)
{
    BesI2cState *s = opaque;

    switch (offset) {
    case 0x00: s->con = val; break;
    case 0x04:
        if (s->tar != (val & 0x3FF)) {
            s->tar = val & 0x3FF;
            s->started = false;
        }
        break;
    case 0x08: s->sar = val; break;
    case 0x10: bes_i2c_data_cmd_write(s, val); break;
    case 0x14: case 0x18: case 0x1C:
    case 0x20: case 0x24: case 0x28:
        s->scl_cfg[(offset - 0x14) / 4] = val; break;
    case 0x30: s->int_mask = val; break;
    case 0x38: s->rx_tl = val & 0xFF; break;
    case 0x3C: s->tx_tl = val & 0xFF; break;
    case 0x6C:
        s->enable = val & 1;
        if (!s->enable) {
            s->tx_cnt = 0;
            s->rx_r = s->rx_w = s->rx_cnt = 0;
            s->started = false;
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "bes.i2c: write off=0x%03x = 0x%08x\n",
                      (int)offset, (unsigned)val);
        break;
    }
    bes_i2c_update_irq(s);
}

static const MemoryRegionOps bes_i2c_ops = {
    .read = bes_i2c_read,
    .write = bes_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void bes_i2c_reset(DeviceState *dev)
{
    BesI2cState *s = BES_I2C(dev);

    s->con = s->tar = s->sar = 0;
    s->int_mask = 0;
    s->raw_int = INTR_TX_EMPTY;
    s->rx_tl = 0;
    s->tx_tl = 0;
    s->enable = 0;
    memset(s->scl_cfg, 0, sizeof(s->scl_cfg));
    s->tx_cnt = 0;
    s->rx_r = s->rx_w = s->rx_cnt = 0;
    s->started = false;
    memset(s->slaves, 0, sizeof(s->slaves));
}

static void bes_i2c_init(Object *obj)
{
    BesI2cState *s = BES_I2C(obj);

    memory_region_init_io(&s->iomem, obj, &bes_i2c_ops, s, "bes.i2c", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static void bes_i2c_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, bes_i2c_reset);
}

static const TypeInfo bes_i2c_info = {
    .name = TYPE_BES_I2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BesI2cState),
    .instance_init = bes_i2c_init,
    .class_init = bes_i2c_class_init,
};

static void bes_i2c_register_types(void)
{
    type_register_static(&bes_i2c_info);
}

type_init(bes_i2c_register_types)
