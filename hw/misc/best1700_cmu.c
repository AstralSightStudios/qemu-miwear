/*
 * BES best1700 clock units: MCU_CMU @0x40000000 and AON_CMU @0x40080000,
 * and the intersys half of BTH_CMU @0x50000000
 *
 * Layout from the best1700 BSP objects (open-vela vendor_bes,
 * boards/best1700_ep/aos_evb/configs/ap/libnx_bestbsp_ap.a); the q63
 * firmware links the same HAL.
 *
 * Both follow the BES set/clear convention: a write to the SET (ENABLE)
 * register sets the written bits of a state word, a write to the CLR
 * (DISABLE) register clears them, and either one reads the state word
 * (hal_cmu_clock_get_status reads the DISABLE register).
 *
 * MCU_CMU:
 *  - clocks of devices 0..164 (hal_cmu_clock_enable/disable):
 *      0x00/0x04  0x08/0x0C  0x10/0x14  0x160/0x164  0x244/0x248
 *      0x178/0x17C
 *  - resets (hal_cmu_reset_set/clear), pulse registers self-clear
 *    (hal_cmu_reset_pulse):
 *      0x34/0x38 pulse 0x28   0x3C/0x40 pulse 0x2C   0x44/0x48 pulse 0x30
 *      0x170/0x174 pulse 0x16C   0x254/0x258 pulse 0x250
 *      0x188/0x18C pulse 0x184
 *  - hardware semaphores 0..7 at 0xC0 + 4n (hal_memsc_lock/unlock): a
 *    read takes a free semaphore and returns 1, or returns 0 while it is
 *    held; writing 1 releases it. 0xE0 has bit n set while semaphore n is
 *    free (hal_memsc_avail).
 *  - PLLs 0..2 at 0xE4/0xE8 (hal_cmu_pll_enable/disable): bit n powers
 *    PLL n, bit 4+n gates its output and reads as its lock status
 *    (hal_cmu_get_pll_status); the PLLs lock at once.
 *  - AXI bus port low-power handshake (hal_cmu_axi_bus_port_force_lp/
 *    auto_lp): request bits written to 0x1A0, acknowledged at once in
 *    0x1AC. The acknowledge bit equals the request bit below bit 25 and
 *    sits one higher from bit 25 up (wifi: request 1<<25, polls 1<<26;
 *    the q63 TEE: request 1<<26 | 1<<11, polls 1<<27 | 1<<11).
 *  - fast system timer, 6 MHz (hal_sys_timer_delay_ns: 6 ticks/us),
 *    64 bits at 0x288/0x28C; writing bit 24 of 0x74 loads it from
 *    0x290/0x294 (hal_fast_sys_timer64_set, hal_cmu_fast_timer_load).
 *  - intersys mailbox to the BTH subsystem (hal_m55c0_2_bthc0_best1700.o,
 *    hal_m55c1_2_bthc0 and the sensor firmware's side of both): each side
 *    has a word, MCU_CMU set 0x98 / clear 0x9C and BTH_CMU set 0x134 /
 *    clear 0x138. In it bit 16+g carries data to the other side for group
 *    g (M55C0 channels 0/1 are groups 0/1, M55C1 channels 0/1 groups 2/3)
 *    and bit g says the other side took it: the receiver clears the data
 *    bit in the sender's word and the hardware sets the done bit, which
 *    the sender then clears. Each side masks its interrupts (MCU_CMU set
 *    0x1F4 / clear 0x1F8, BTH_CMU 0x168 / 0x16C: rx enable g, tx done
 *    enable 16+g) and reads two status words (receive: MCU_CMU 0xEC,
 *    BTH_CMU 0x174; tx done: MCU_CMU 0x224, BTH_CMU 0x170), raw in bit g
 *    and masked in bit 16+g. Every group drives an "intersys-rx" and an
 *    "intersys-tx" line.
 *  - mailbox to the Bluetooth core (the M55C1 firmware's hal_intersys,
 *    4 channels), set 0x140 / clear 0x144: bit 4+ch the M55's data
 *    indication (the doorbell), bit 8+ch / 12+ch enable the receive and
 *    "BT took the data" interrupts, bits 24+ch / 28+ch report them. The
 *    BT side takes the data by dropping the indication, which raises
 *    28+ch (cleared by writing bit ch to 0x144); 24+ch follows its
 *    "data for the M55" line. The receive handler masks with bit 8+ch in
 *    0x144 and unmasks once it has told the BT core it is done. Every
 *    channel drives a "tx done" and a "rx" interrupt line (bt-irq ch and
 *    4+ch).
 *  - inter-core interrupts between the two M55 cores, set 0x90 / clear
 *    0x94 (hal_m55c0_2_m55c1_best1700.o, built for either core). Per
 *    channel c (0, 1): bit 15+c M55C0 data for M55C1, bit 12+c M55C0 done
 *    with M55C1's data, bit 21+c M55C1 data for M55C0, bit 18+c M55C1
 *    done with M55C0's data. Every bit drives its own "ipc" line; the
 *    board routes them to the NVIC lines 88/89 (data) and 91/92 (done)
 *    of the receiving core.
 *
 * AON_CMU:
 *  - clocks of devices 165..201: 0x40/0x44  0x180/0x184
 *  - resets: 0x10/0x14 pulse 0x0C   0x190/0x194 pulse 0x18C
 *            0xC8/0xCC pulse 0xC4
 *  - AON hardware semaphores 0..15 at 0x100 + 4n, free mask at 0x140
 *    (hal_memsc_aon_lock/unlock/avail)
 *  - system timer, 16 kHz (hal_sys_timer_systick_hz: 16000.0f), 64 bits
 *    at 0x50/0x54 (hal_sys_timer64_get)
 *  - boot mode at 0x38 (hal_hw_bootmode_get): plain storage, cold boot 0
 *  - M55C1 start (hal_cmu_m55c1_cpu_reset_clear / hal_cmu_m55c1_start_cpu,
 *    hal_maincmu_best1700.o): its vector table address in bits [31:7] of
 *    0xB4, and bit 1 of 0x1A4 holds the core; the first release after a
 *    hold raises "cpu-run", a new hold lowers it.
 *
 * Everything else is plain storage; accesses are logged with -d unimp.
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "hw/misc/best1700_cmu.h"

typedef struct Best1700CmuPair {
    uint16_t set;
    uint16_t clr;
} Best1700CmuPair;

struct Best1700CmuInfo {
    const char *name;
    const Best1700CmuPair *pairs;
    unsigned num_pairs;
    const uint16_t *pulses;
    unsigned num_pulses;
    hwaddr memsc;               /* first semaphore */
    unsigned num_memsc;
    hwaddr memsc_avail;
    hwaddr pll_set;             /* 0: none */
    hwaddr timer;               /* counter, low word; high word follows */
    uint32_t timer_hz;
    hwaddr timer_load;          /* 0: not loadable */
    hwaddr timer_load_ctrl;
    uint32_t timer_load_bit;
    hwaddr lp_req;              /* 0: no bus low-power handshake */
    hwaddr lp_ack;
    hwaddr ipc_set;             /* 0: no inter-core interrupt pair */
    hwaddr hold;                /* 0: no second core to start */
    uint32_t hold_bit;
    hwaddr isys_set;            /* 0: no intersys mailbox */
    hwaddr isys_clr;
    hwaddr isys_mask_set;
    hwaddr isys_mask_clr;
    hwaddr isys_rx_stat;
    hwaddr isys_tx_stat;
    hwaddr bt_set;              /* 0: no mailbox to the Bluetooth core */
    hwaddr bt_clr;
};

static const Best1700CmuPair cmu_pairs[] = {
    { 0x000, 0x004 }, { 0x008, 0x00C }, { 0x010, 0x014 },
    { 0x160, 0x164 }, { 0x244, 0x248 }, { 0x178, 0x17C },
    { 0x034, 0x038 }, { 0x03C, 0x040 }, { 0x044, 0x048 },
    { 0x170, 0x174 }, { 0x254, 0x258 }, { 0x188, 0x18C },
    { 0x0E4, 0x0E8 }, { 0x090, 0x094 },
};

static const uint16_t cmu_pulses[] = {
    0x028, 0x02C, 0x030, 0x16C, 0x250, 0x184,
};

static const Best1700CmuInfo cmu_info = {
    .name = "best1700.cmu",
    .pairs = cmu_pairs,
    .num_pairs = ARRAY_SIZE(cmu_pairs),
    .pulses = cmu_pulses,
    .num_pulses = ARRAY_SIZE(cmu_pulses),
    .memsc = 0xC0,
    .num_memsc = 8,
    .memsc_avail = 0xE0,
    .pll_set = 0xE4,
    .timer = 0x288,
    .timer_hz = 6000000,
    .timer_load = 0x290,
    .timer_load_ctrl = 0x74,
    .timer_load_bit = 1u << 24,
    .lp_req = 0x1A0,
    .lp_ack = 0x1AC,
    .ipc_set = 0x090,
    .isys_set = 0x098,
    .isys_clr = 0x09C,
    .isys_mask_set = 0x1F4,
    .isys_mask_clr = 0x1F8,
    .isys_rx_stat = 0x0EC,
    .isys_tx_stat = 0x224,
    .bt_set = 0x140,
    .bt_clr = 0x144,
};

static const Best1700CmuPair aoncmu_pairs[] = {
    { 0x040, 0x044 }, { 0x180, 0x184 },
    { 0x010, 0x014 }, { 0x190, 0x194 }, { 0x0C8, 0x0CC },
};

static const uint16_t aoncmu_pulses[] = {
    0x00C, 0x18C, 0x0C4,
};

static const Best1700CmuInfo aoncmu_info = {
    .name = "best1700.aoncmu",
    .pairs = aoncmu_pairs,
    .num_pairs = ARRAY_SIZE(aoncmu_pairs),
    .pulses = aoncmu_pulses,
    .num_pulses = ARRAY_SIZE(aoncmu_pulses),
    .memsc = 0x100,
    .num_memsc = 16,
    .memsc_avail = 0x140,
    .timer = 0x50,
    .timer_hz = 16000,
    .hold = 0x1A4,
    .hold_bit = 1u << 1,
};

/* BTH_CMU @0x50000000: only the intersys half is modelled */
static const Best1700CmuInfo bthcmu_info = {
    .name = "best1700.bthcmu",
    .isys_set = 0x134,
    .isys_clr = 0x138,
    .isys_mask_set = 0x168,
    .isys_mask_clr = 0x16C,
    .isys_rx_stat = 0x174,
    .isys_tx_stat = 0x170,
};

static const Best1700CmuInfo *cmu_info_of(Best1700CmuState *s)
{
    return BEST1700_CMU_GET_CLASS(s)->info;
}

static int cmu_find_pair(const Best1700CmuInfo *info, hwaddr offset,
                         bool *is_set)
{
    unsigned i;

    for (i = 0; i < info->num_pairs; i++) {
        if (info->pairs[i].set == offset || info->pairs[i].clr == offset) {
            *is_set = info->pairs[i].set == offset;
            return i;
        }
    }
    return -1;
}

static bool cmu_is_pulse(const Best1700CmuInfo *info, hwaddr offset)
{
    unsigned i;

    for (i = 0; i < info->num_pulses; i++) {
        if (info->pulses[i] == offset) {
            return true;
        }
    }
    return false;
}

static uint64_t cmu_ticks(const Best1700CmuInfo *info)
{
    return muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), info->timer_hz,
                    NANOSECONDS_PER_SECOND);
}

#define ISYS_G_MASK MAKE_64BIT_MASK(0, BEST1700_ISYS_GROUPS)

static uint32_t isys_rx_raw(Best1700CmuState *s)
{
    return s->isys_peer ? (s->isys_peer->isys_word >> 16) & ISYS_G_MASK : 0;
}

static uint32_t isys_rx_stat(Best1700CmuState *s)
{
    uint32_t raw = isys_rx_raw(s);

    return raw | (raw & s->isys_mask & ISYS_G_MASK) << 16;
}

static uint32_t isys_tx_stat(Best1700CmuState *s)
{
    uint32_t raw = s->isys_word & ISYS_G_MASK;

    return raw | (raw & (s->isys_mask >> 16)) << 16;
}

static void isys_update_one(Best1700CmuState *s)
{
    uint32_t rx = isys_rx_stat(s) >> 16, tx = isys_tx_stat(s) >> 16;
    int g;

    for (g = 0; g < BEST1700_ISYS_GROUPS; g++) {
        qemu_set_irq(s->isys_rx[g], (rx >> g) & 1);
        qemu_set_irq(s->isys_tx[g], (tx >> g) & 1);
    }
}

static void isys_update(Best1700CmuState *s)
{
    isys_update_one(s);
    if (s->isys_peer) {
        isys_update_one(s->isys_peer);
    }
}

void best1700_cmu_link_intersys(Best1700CmuState *mcu, Best1700CmuState *bth)
{
    mcu->isys_peer = bth;
    bth->isys_peer = mcu;
}

#define BT_IND(ch)      (1u << (4 + (ch)))
#define BT_RX_EN(ch)    (1u << (8 + (ch)))
#define BT_TX_EN(ch)    (1u << (12 + (ch)))
#define BT_RX(ch)       (1u << (24 + (ch)))
#define BT_TX_DONE(ch)  (1u << (28 + (ch)))
#define BT_WORD_MASK    0xFFF0u

static void bt_update(Best1700CmuState *s)
{
    int ch;

    for (ch = 0; ch < BEST1700_BT_ISYS_CHANNELS; ch++) {
        qemu_set_irq(s->bt_irq[ch], (s->bt_raw & BT_TX_DONE(ch)) &&
                                    (s->bt_word & BT_TX_EN(ch)));
        qemu_set_irq(s->bt_irq[BEST1700_BT_ISYS_CHANNELS + ch],
                     (s->bt_raw & BT_RX(ch)) && (s->bt_word & BT_RX_EN(ch)));
    }
}

/* the BT core took the M55's data on channel ch */
static void bt_ind_ack(void *opaque, int ch, int level)
{
    Best1700CmuState *s = BEST1700_CMU(opaque);

    if (level) {
        s->bt_word &= ~BT_IND(ch);
        s->bt_raw |= BT_TX_DONE(ch);
        bt_update(s);
    }
}

/* the BT core has data published for the M55 on channel ch (level) */
static void bt_data(void *opaque, int ch, int level)
{
    Best1700CmuState *s = BEST1700_CMU(opaque);

    if (level) {
        s->bt_raw |= BT_RX(ch);
    } else {
        s->bt_raw &= ~BT_RX(ch);
    }
    bt_update(s);
}

static void bt_write(Best1700CmuState *s, bool set, uint32_t value)
{
    int ch;

    if (set) {
        uint32_t newly = value & ~s->bt_word;

        s->bt_word |= value & BT_WORD_MASK;
        bt_update(s);
        for (ch = 0; ch < BEST1700_BT_ISYS_CHANNELS; ch++) {
            if (newly & BT_IND(ch)) {
                qemu_irq_pulse(s->bt_doorbell[ch]);
            }
        }
    } else {
        for (ch = 0; ch < BEST1700_BT_ISYS_CHANNELS; ch++) {
            if (value & (1u << ch)) {
                s->bt_raw &= ~BT_TX_DONE(ch);
            }
        }
        s->bt_word &= ~(value & BT_WORD_MASK);
        bt_update(s);
    }
}

static void cmu_update_ipc(Best1700CmuState *s, uint32_t word)
{
    int i;

    for (i = 0; i < 32; i++) {
        qemu_set_irq(s->ipc[i], (word >> i) & 1);
    }
}

static uint64_t best1700_cmu_read(void *opaque, hwaddr offset, unsigned size)
{
    Best1700CmuState *s = BEST1700_CMU(opaque);
    const Best1700CmuInfo *info = cmu_info_of(s);
    hwaddr memsc_end = info->memsc + 4 * info->num_memsc;
    uint32_t v;
    bool is_set;
    int pair;

    if (info->bt_set && (offset == info->bt_set || offset == info->bt_clr)) {
        v = s->bt_word | s->bt_raw;
    } else if (info->isys_set && (offset == info->isys_set ||
                                  offset == info->isys_clr)) {
        v = s->isys_word;
    } else if (info->isys_set && (offset == info->isys_mask_set ||
                                  offset == info->isys_mask_clr)) {
        v = s->isys_mask;
    } else if (info->isys_set && offset == info->isys_rx_stat) {
        v = isys_rx_stat(s);
    } else if (info->isys_set && offset == info->isys_tx_stat) {
        v = isys_tx_stat(s);
    } else if (info->num_memsc && offset >= info->memsc &&
               offset < memsc_end && !(offset & 3)) {
        uint32_t bit = 1u << ((offset - info->memsc) / 4);

        v = !(s->memsc_taken & bit);
        s->memsc_taken |= bit;
    } else if (info->num_memsc && offset == info->memsc_avail) {
        v = ~s->memsc_taken & MAKE_64BIT_MASK(0, info->num_memsc);
    } else if (info->timer_hz &&
               (offset == info->timer || offset == info->timer + 4)) {
        uint64_t t = cmu_ticks(info) + s->timer_offset;

        v = offset == info->timer ? (uint32_t)t : (uint32_t)(t >> 32);
    } else if ((pair = cmu_find_pair(info, offset, &is_set)) >= 0) {
        v = s->pair_state[pair];
        if (info->pll_set && info->pairs[pair].set == info->pll_set) {
            v |= (v & 0x7) << 4;    /* powered PLLs report lock */
        }
    } else if (cmu_is_pulse(info, offset)) {
        v = 0;
    } else if (info->lp_req && offset == info->lp_ack) {
        uint32_t req = s->regs[info->lp_req / 4];

        v = (req & MAKE_64BIT_MASK(0, 25)) |
            ((req & ~MAKE_64BIT_MASK(0, 25)) << 1);
    } else {
        v = s->regs[offset / 4];
    }

    qemu_log_mask(LOG_UNIMP, "%s: read  off=0x%03x -> 0x%08x\n",
                  info->name, (unsigned)offset, v);
    return v;
}

static void best1700_cmu_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    Best1700CmuState *s = BEST1700_CMU(opaque);
    const Best1700CmuInfo *info = cmu_info_of(s);
    hwaddr memsc_end = info->memsc + 4 * info->num_memsc;
    bool is_set;
    int pair;

    qemu_log_mask(LOG_UNIMP, "%s: write off=0x%03x = 0x%08x\n",
                  info->name, (unsigned)offset, (unsigned)value);

    if (info->bt_set && (offset == info->bt_set || offset == info->bt_clr)) {
        bt_write(s, offset == info->bt_set, value);
    } else if (info->isys_set && offset == info->isys_set) {
        s->isys_word |= value;
        isys_update(s);
    } else if (info->isys_set && offset == info->isys_clr) {
        /* clearing a data bit tells the sender it was taken */
        uint32_t taken = (s->isys_word & value) >> 16;

        s->isys_word &= ~value;
        s->isys_word |= taken & ~value & ISYS_G_MASK;
        isys_update(s);
    } else if (info->isys_set && offset == info->isys_mask_set) {
        s->isys_mask |= value;
        isys_update(s);
    } else if (info->isys_set && offset == info->isys_mask_clr) {
        s->isys_mask &= ~value;
        isys_update(s);
    } else if (info->isys_set && (offset == info->isys_rx_stat ||
                                  offset == info->isys_tx_stat)) {
        /* read-only */
    } else if (info->num_memsc && offset >= info->memsc &&
               offset < memsc_end && !(offset & 3)) {
        if (value & 1) {
            s->memsc_taken &= ~(1u << ((offset - info->memsc) / 4));
        }
    } else if ((info->num_memsc && offset == info->memsc_avail) ||
               (info->timer_hz &&
                (offset == info->timer || offset == info->timer + 4)) ||
               cmu_is_pulse(info, offset)) {
        /* read-only, or a pulse with nothing to reset here */
    } else if ((pair = cmu_find_pair(info, offset, &is_set)) >= 0) {
        if (is_set) {
            s->pair_state[pair] |= value;
        } else {
            s->pair_state[pair] &= ~value;
        }
        if (info->ipc_set && info->pairs[pair].set == info->ipc_set) {
            cmu_update_ipc(s, s->pair_state[pair]);
        }
    } else if (info->hold && offset == info->hold) {
        s->regs[offset / 4] = value;
        if (value & info->hold_bit) {
            s->cpu_held = true;
            qemu_set_irq(s->cpu_run, 0);
        } else if (s->cpu_held) {
            qemu_set_irq(s->cpu_run, 1);
        }
    } else if (info->timer_load && offset == info->timer_load_ctrl &&
               (value & info->timer_load_bit)) {
        uint64_t load = s->regs[info->timer_load / 4] |
                        (uint64_t)s->regs[info->timer_load / 4 + 1] << 32;

        s->timer_offset = load - cmu_ticks(info);
        s->regs[offset / 4] = value & ~info->timer_load_bit;
    } else {
        s->regs[offset / 4] = value;
    }
}

static const MemoryRegionOps best1700_cmu_ops = {
    .read = best1700_cmu_read,
    .write = best1700_cmu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void best1700_cmu_reset(DeviceState *dev)
{
    Best1700CmuState *s = BEST1700_CMU(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->pair_state, 0, sizeof(s->pair_state));
    s->memsc_taken = 0;
    s->timer_offset = 0;
    s->cpu_held = false;
    s->isys_word = 0;
    s->isys_mask = 0;
    s->bt_word = 0;
    s->bt_raw = 0;
}

static void best1700_cmu_realize(DeviceState *dev, Error **errp)
{
    Best1700CmuState *s = BEST1700_CMU(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &best1700_cmu_ops, s,
                          cmu_info_of(s)->name, BEST1700_CMU_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    if (cmu_info_of(s)->ipc_set) {
        qdev_init_gpio_out_named(dev, s->ipc, "ipc", 32);
    }
    if (cmu_info_of(s)->hold) {
        qdev_init_gpio_out_named(dev, &s->cpu_run, "cpu-run", 1);
    }
    if (cmu_info_of(s)->isys_set) {
        qdev_init_gpio_out_named(dev, s->isys_rx, "intersys-rx",
                                 BEST1700_ISYS_GROUPS);
        qdev_init_gpio_out_named(dev, s->isys_tx, "intersys-tx",
                                 BEST1700_ISYS_GROUPS);
    }
    if (cmu_info_of(s)->bt_set) {
        qdev_init_gpio_out_named(dev, s->bt_doorbell, "bt-doorbell",
                                 BEST1700_BT_ISYS_CHANNELS);
        qdev_init_gpio_out_named(dev, s->bt_irq, "bt-irq",
                                 2 * BEST1700_BT_ISYS_CHANNELS);
        qdev_init_gpio_in_named(dev, bt_ind_ack, "bt-ind-ack",
                                BEST1700_BT_ISYS_CHANNELS);
        qdev_init_gpio_in_named(dev, bt_data, "bt2mcu",
                                BEST1700_BT_ISYS_CHANNELS);
    }
}

static void best1700_cmu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    Best1700CmuClass *cc = BEST1700_CMU_CLASS(klass);

    cc->info = data;
    dc->realize = best1700_cmu_realize;
    device_class_set_legacy_reset(dc, best1700_cmu_reset);
}

static const TypeInfo best1700_cmu_types[] = {
    {
        .name = TYPE_BEST1700_CMU,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Best1700CmuState),
        .class_size = sizeof(Best1700CmuClass),
        .class_init = best1700_cmu_class_init,
        .class_data = &cmu_info,
    }, {
        .name = TYPE_BEST1700_AONCMU,
        .parent = TYPE_BEST1700_CMU,
        .class_init = best1700_cmu_class_init,
        .class_data = &aoncmu_info,
    }, {
        .name = TYPE_BEST1700_BTHCMU,
        .parent = TYPE_BEST1700_CMU,
        .class_init = best1700_cmu_class_init,
        .class_data = &bthcmu_info,
    },
};

DEFINE_TYPES(best1700_cmu_types)
