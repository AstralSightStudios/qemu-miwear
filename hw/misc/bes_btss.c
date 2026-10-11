/*
 * BES Bluetooth subsystem stand-in (best1503, best1700)
 *
 * The AP firmware talks to the Bluetooth core through the inter-core
 * mailbox ("intersys"). The BT core itself is not emulated; this device
 * plays its side of the mailbox protocol and answers HCI traffic like a
 * dual-mode controller. The over-the-air side (inquiry, paging, links,
 * pairing, ACL) is modelled by bes_btss_link.c against one peer
 * controller whose host attaches through a chardev.
 *
 * Mailbox protocol (AP firmware hal_intersys: open sub_C231128, send
 * sub_C2314DC, TX irq sub_C231034, RX irq sub_C2315E4, RX done
 * sub_C2315BC; register bits from BES reg_cmu ISIRQ, see bes_cmu.c):
 *   - shared pointer table in BT RAM: word [8 * ch + 4] holds the address
 *     of the AP's "in flight" list head for channel ch, word [8 * ch]
 *     the address of the BT core's list head (read by the AP)
 *   - message descriptor {next, type (u8 @4), len @8, data @12}; type is
 *     the mailbox id within the channel
 *   - AP -> BT: the AP publishes its list and sets MCU2BT_DATA_IND; the
 *     BT core consumes the messages, drops the indication and raises the
 *     MCU2BT done interrupt (ind_ack line)
 *   - BT -> AP: the BT core publishes its list and raises BT2MCU_DATA
 *     (bt2mcu line); the AP consumes it and acknowledges by writing
 *     bit 2 << ch to BT_CMU + 0x54
 * best1700 (the M55C1 "cp" firmware's hal_intersys) has the same
 * protocol with two differences, set by properties: word [8 * ch] holds
 * the first BT -> MCU descriptor itself ("rx-head-direct"), and the
 * done bit is 1 << (18 + ch) ("data-done-shift" 18).
 *
 * The best1700 host carries HCI on channel 0 only ("hci-channel" 0); on
 * channel 1 it sends 8-byte {user, msg} requests to the BT core
 * (bt_intersys_oper_btcore) and has no receive handler, so anything
 * published there trips its "INTERSYS-RX: Handler missing" assertion.
 *
 * The HCI_Revision of Read Local Version Information ("hci-revision") is
 * the controller firmware date, year << 9 | month << 5 | day: the
 * best1700 host (bt_drv_update_btc_sw_version) turns it into 0xYYMMDD
 * and bt_drv_reg_op_init asserts unless it is later than 0x240324.
 *
 * HCI (vendor/bes/chips/bes/bes_serial_hci2.c, bthci_send/receive):
 * H4 framed packets. Commands get a Command Complete (or Command Status
 * for commands whose completion is a later event) with spec-sized return
 * parameters describing an idle dual-mode controller. The driver's send
 * path waits for the Command Complete of vendor opcode 0xFC77 before it
 * talks to a controller it believes is asleep; a generic status-only
 * Command Complete satisfies that handshake.
 *
 * Timing: the controller takes the AP's messages immediately (TX done)
 * but answers after a processing latency (latency-us, default 1 ms).
 * Messages go out in order, each once its own latency has passed. Two
 * host behaviours bound that latency (q63 cp firmware, vela_cp.bin):
 *   - On HCI_Reset bthci_send marks the controller asleep; bthci_receive
 *     answers the reset's Command Complete with the 0xFC77 wake command
 *     and, on that command's Command Complete, resets the TX-done
 *     semaphore. If this lands before the thread that sent the reset has
 *     taken its TX-done, the reset wipes it and bthci_send asserts
 *     (bes_serial_hci2.c line 343) after 10 s. HCI_Reset therefore
 *     completes after reset-latency-us, like a real controller that
 *     needs a while to reset, which leaves the sending thread time to
 *     take its TX-done (q63: 100 ms; with 1 ms the race was lost about
 *     once in 30 boots, with 10 ms still under host load).
 *   - The Bluetooth driver's init (btdrv) opens mailbox (0, 0) with its
 *     own receive handler, sends vendor commands without waiting for
 *     their completion (btdrv_SendData only waits for TX done) and closes
 *     the mailbox again; bthci opens it later. A Command Complete that
 *     arrives in between finds no handler: "INTERSYS-RX: Handler missing"
 *     (hal_intersys_rx_irq_handler line 544): rx_done re-enables a
 *     channel's receive interrupt whether or not it is open. A real
 *     controller answers well before that. With latency-us 0 an answer is
 *     published within the very doorbell access that carried the command,
 *     so the host takes it in its receive interrupt before the send call
 *     returns (q63; at 50 us or 1 ms one boot in about 20 failed).
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "crypto/aes.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "system/address-spaces.h"
#include "bes_btss_int.h"
#include "trace.h"

#define BTCMU_OFF_MCU_DATA_DONE 0x54    /* AP: bit (2 << ch) = RX consumed */

/* BT -> MCU descriptor area layout inside the exchange memory */
#define EXCH_CHAN_SPAN          0x8000
#define EXCH_HEAD_OFF           0x0000
#define EXCH_DESC_OFF           0x0010
#define EXCH_DESC_SIZE          20
#define EXCH_DATA_OFF           0x0400

static uint32_t btss_ldl(BesBtssState *s, uint32_t addr)
{
    return address_space_ldl_le(&s->as, addr, MEMTXATTRS_UNSPECIFIED, NULL);
}

static void btss_stl(BesBtssState *s, uint32_t addr, uint32_t val)
{
    address_space_stl_le(&s->as, addr, val, MEMTXATTRS_UNSPECIFIED, NULL);
}

/* ---- BT -> MCU path ---- */

static void btss_publish(BesBtssState *s, int ch)
{
    BesBtssChan *c = &s->chan[ch];
    uint32_t base = s->exch_base + ch * EXCH_CHAN_SPAN;
    uint32_t data = base + EXCH_DATA_OFF;
    uint32_t prev = 0;
    unsigned n;

    int64_t now = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);

    if (c->inflight || !c->q_count || c->q[c->q_head].ready_us > now) {
        return;
    }
    for (n = 0; n < c->q_count; n++) {
        BesBtssMsg *m = &c->q[(c->q_head + n) % BES_BTSS_QUEUE_LEN];
        uint32_t desc = base + EXCH_DESC_OFF + n * EXCH_DESC_SIZE;
        uint8_t hdr[EXCH_DESC_SIZE] = { 0 };

        if (m->ready_us > now ||
            data + m->len > base + EXCH_CHAN_SPAN ||
            desc + EXCH_DESC_SIZE > base + EXCH_DATA_OFF) {
            break;
        }
        address_space_write(&s->as, data, MEMTXATTRS_UNSPECIFIED,
                            m->data, m->len);
        hdr[4] = c->mbx;
        stl_le_p(hdr + 8, m->len);
        stl_le_p(hdr + 12, data);
        address_space_write(&s->as, desc, MEMTXATTRS_UNSPECIFIED,
                            hdr, sizeof(hdr));
        if (prev) {
            btss_stl(s, prev, desc);
        } else {
            btss_stl(s, base + EXCH_HEAD_OFF, desc);
        }
        prev = desc;
        data += ROUND_UP(m->len, 4);
    }
    c->inflight = n;
    btss_stl(s, s->shared_table + 8 * ch,
             s->rx_head_direct ? base + EXCH_DESC_OFF : base + EXCH_HEAD_OFF);
    trace_bes_btss_publish(ch, n);
    qemu_set_irq(s->bt2mcu[ch], 1);
}

/* wake up when the next unpublished message is due */
static void btss_schedule(BesBtssState *s)
{
    int64_t next = INT64_MAX;
    int ch;

    for (ch = 0; ch < BES_BTSS_CHANNELS; ch++) {
        BesBtssChan *c = &s->chan[ch];

        if (!c->inflight && c->q_count) {
            next = MIN(next, c->q[c->q_head].ready_us);
        }
    }
    if (next == INT64_MAX) {
        timer_del(s->publish_timer);
    } else {
        timer_mod(s->publish_timer, next);
    }
}

static void btss_queue(BesBtssState *s, int ch, const uint8_t *data,
                       unsigned len)
{
    BesBtssChan *c = &s->chan[ch];
    BesBtssMsg *m;

    if (c->q_count == BES_BTSS_QUEUE_LEN || len > BES_BTSS_MSG_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "bes.btss: ch%d: dropping %u-byte message\n", ch, len);
        return;
    }
    m = &c->q[(c->q_head + c->q_count) % BES_BTSS_QUEUE_LEN];
    memcpy(m->data, data, len);
    m->len = len;
    /* published once the controller has "processed" the request, and
     * never ahead of what was queued before it */
    m->ready_us = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL) + s->reply_us;
    if (c->q_count) {
        const BesBtssMsg *last =
            &c->q[(c->q_head + c->q_count - 1) % BES_BTSS_QUEUE_LEN];

        m->ready_us = MAX(m->ready_us, last->ready_us);
    }
    c->q_count++;
    if (!s->reply_us) {
        /* no latency: out within the access that caused it */
        btss_publish(s, ch);
    }
    btss_schedule(s);
}

static void btss_publish_timer(void *opaque)
{
    BesBtssState *s = BES_BTSS(opaque);
    int ch;

    for (ch = 0; ch < BES_BTSS_CHANNELS; ch++) {
        btss_publish(s, ch);
    }
    btss_schedule(s);
}

unsigned btss_band_queue_free(BesBtssState *s)
{
    return BES_BTSS_QUEUE_LEN - s->chan[s->hci_ch].q_count;
}

bool btss_band_acl(BesBtssState *s, const uint8_t *pkt, unsigned len)
{
    if (!btss_band_queue_free(s)) {
        return false;
    }
    btss_queue(s, s->hci_ch, pkt, len);
    return true;
}

static void btss_rx_done(BesBtssState *s, int ch)
{
    BesBtssChan *c = &s->chan[ch];

    if (!c->inflight) {
        return;
    }
    c->q_head = (c->q_head + c->inflight) % BES_BTSS_QUEUE_LEN;
    c->q_count -= c->inflight;
    c->inflight = 0;
    btss_stl(s, s->exch_base + ch * EXCH_CHAN_SPAN + EXCH_HEAD_OFF, 0);
    qemu_set_irq(s->bt2mcu[ch], 0);
    btss_publish(s, ch);
    btss_schedule(s);
    if (ch == s->hci_ch) {
        btss_link_band_drained(s);
    }
}

/* ---- HCI controller ---- */

/* an event to the host of controller "side" */
void btss_event(BesBtssState *s, int side, uint8_t code,
                const uint8_t *params, unsigned plen)
{
    uint8_t pkt[3 + 255];

    plen = MIN(plen, 255u);
    pkt[0] = H4_EVT;
    pkt[1] = code;
    pkt[2] = plen;
    memcpy(pkt + 3, params, plen);
    trace_bes_btss_event(side, code, plen);
    if (side == BTSS_BAND) {
        btss_queue(s, s->hci_ch, pkt, 3 + plen);
    } else {
        btss_link_peer_send(s, pkt, 3 + plen);
    }
}

void btss_cmd_complete(BesBtssState *s, int side, uint16_t opcode,
                       const uint8_t *rp, unsigned rp_len)
{
    uint8_t p[3 + 252];

    p[0] = 1;                       /* Num_HCI_Command_Packets */
    stw_le_p(p + 1, opcode);
    memcpy(p + 3, rp, MIN(rp_len, 252u));
    btss_event(s, side, HCI_EVT_CMD_COMPLETE, p, 3 + MIN(rp_len, 252u));
}

void btss_cmd_status(BesBtssState *s, int side, uint16_t opcode,
                     uint8_t status)
{
    uint8_t p[4] = { status, 1, opcode & 0xFF, opcode >> 8 };

    btss_event(s, side, HCI_EVT_CMD_STATUS, p, sizeof(p));
}

/* commands whose completion is reported by a later event */
static bool hci_uses_cmd_status(uint16_t opcode)
{
    static const uint16_t cs[] = {
        0x0401, 0x0405, 0x0406, 0x0408, 0x0409, 0x040A, 0x040F, 0x0411,
        0x0413, 0x0415, 0x0417, 0x0419, 0x041B, 0x041C, 0x041D, 0x041F,
        0x0428, 0x0429, 0x042A, 0x043D, 0x043E,
        0x200D, 0x2013, 0x2016, 0x2019, 0x2025, 0x2026, 0x2032, 0x2043,
        0x2064, 0x2066, 0x2068, 0x206A,
    };
    int i;

    for (i = 0; i < ARRAY_SIZE(cs); i++) {
        if (cs[i] == opcode) {
            return true;
        }
    }
    return false;
}

/*
 * Return parameter sizes (status excluded) of the read-type commands, per
 * Bluetooth Core Specification v5.3 Vol 4 Part E section 7. Commands not
 * listed return only a status. Contents default to zero; the ones a host
 * interprets are filled in by hci_fill_return().
 */
static const struct {
    uint16_t opcode;
    uint8_t len;
} hci_ret_len[] = {
    /* link policy */
    { 0x0809, 3 },      /* Role Discovery */
    { 0x080C, 4 },      /* Read Link Policy Settings */
    { 0x080D, 2 },      /* Write Link Policy Settings */
    { 0x080E, 2 },      /* Read Default Link Policy Settings */
    /* controller & baseband */
    { 0x0C08, 2 },      /* Flush */
    { 0x0C09, 1 },      /* Read PIN Type */
    { 0x0C0D, 4 },      /* Read Stored Link Key */
    { 0x0C11, 1 },      /* Write Stored Link Key */
    { 0x0C12, 2 },      /* Delete Stored Link Key */
    { 0x0C14, 248 },    /* Read Local Name */
    { 0x0C15, 2 },      /* Read Connection Accept Timeout */
    { 0x0C17, 2 },      /* Read Page Timeout */
    { 0x0C19, 1 },      /* Read Scan Enable */
    { 0x0C1B, 4 },      /* Read Page Scan Activity */
    { 0x0C1D, 4 },      /* Read Inquiry Scan Activity */
    { 0x0C1F, 1 },      /* Read Authentication Enable */
    { 0x0C23, 3 },      /* Read Class of Device */
    { 0x0C25, 2 },      /* Read Voice Setting */
    { 0x0C27, 4 },      /* Read Automatic Flush Timeout */
    { 0x0C28, 2 },      /* Write Automatic Flush Timeout */
    { 0x0C29, 1 },      /* Read Num Broadcast Retransmissions */
    { 0x0C2B, 1 },      /* Read Hold Mode Activity */
    { 0x0C2D, 3 },      /* Read Transmit Power Level */
    { 0x0C2E, 1 },      /* Read Synchronous Flow Control Enable */
    { 0x0C36, 4 },      /* Read Link Supervision Timeout */
    { 0x0C37, 2 },      /* Write Link Supervision Timeout */
    { 0x0C38, 1 },      /* Read Number Of Supported IAC */
    { 0x0C39, 4 },      /* Read Current IAC LAP (one IAC) */
    { 0x0C42, 1 },      /* Read Inquiry Scan Type */
    { 0x0C44, 1 },      /* Read Inquiry Mode */
    { 0x0C46, 1 },      /* Read Page Scan Type */
    { 0x0C48, 1 },      /* Read AFH Channel Assessment Mode */
    { 0x0C51, 241 },    /* Read Extended Inquiry Response */
    { 0x0C55, 1 },      /* Read Simple Pairing Mode */
    { 0x0C57, 32 },     /* Read Local OOB Data */
    { 0x0C58, 1 },      /* Read Inquiry Response TX Power Level */
    { 0x0C5A, 1 },      /* Read Default Erroneous Data Reporting */
    { 0x0C61, 2 },      /* Read Logical Link Accept Timeout */
    { 0x0C66, 1 },      /* Read Flow Control Mode */
    { 0x0C68, 5 },      /* Read Enhanced Transmit Power Level */
    { 0x0C6A, 4 },      /* Read Best Effort Flush Timeout */
    { 0x0C6C, 2 },      /* Read LE Host Support */
    { 0x0C76, 7 },      /* Read Synchronization Train Parameters */
    { 0x0C79, 1 },      /* Read Secure Connections Host Support */
    { 0x0C7B, 4 },      /* Read Authenticated Payload Timeout */
    { 0x0C7C, 2 },      /* Write Authenticated Payload Timeout */
    { 0x0C7D, 64 },     /* Read Local OOB Extended Data */
    { 0x0C7E, 2 },      /* Read Extended Page Timeout */
    { 0x0C80, 2 },      /* Read Extended Inquiry Length */
    /* informational */
    { 0x1001, 8 },      /* Read Local Version Information */
    { 0x1002, 64 },     /* Read Local Supported Commands */
    { 0x1003, 8 },      /* Read Local Supported Features */
    { 0x1004, 10 },     /* Read Local Extended Features */
    { 0x1005, 7 },      /* Read Buffer Size */
    { 0x1009, 6 },      /* Read BD_ADDR */
    { 0x100A, 6 },      /* Read Data Block Size */
    { 0x100B, 2 },      /* Read Local Supported Codecs (none) */
    { 0x100C, 2 },      /* Read Local Simple Pairing Options */
    { 0x100D, 2 },      /* Read Local Supported Codecs v2 (none) */
    { 0x100E, 1 },      /* Read Local Supported Codec Capabilities */
    { 0x100F, 6 },      /* Read Local Supported Controller Delay */
    /* status */
    { 0x1401, 4 },      /* Read Failed Contact Counter */
    { 0x1402, 2 },      /* Reset Failed Contact Counter */
    { 0x1403, 3 },      /* Read Link Quality */
    { 0x1405, 3 },      /* Read RSSI */
    { 0x1406, 13 },     /* Read AFH Channel Map */
    { 0x1407, 8 },      /* Read Clock */
    { 0x1408, 3 },      /* Read Encryption Key Size */
    /* LE */
    { 0x2002, 3 },      /* LE Read Buffer Size v1 */
    { 0x2003, 8 },      /* LE Read Local Supported Features */
    { 0x2007, 1 },      /* LE Read Advertising Channel TX Power */
    { 0x200F, 1 },      /* LE Read Filter Accept List Size */
    { 0x2015, 7 },      /* LE Read Channel Map */
    { 0x2017, 16 },     /* LE Encrypt */
    { 0x2018, 8 },      /* LE Rand */
    { 0x201A, 2 },      /* LE LTK Request Reply */
    { 0x201B, 2 },      /* LE LTK Request Negative Reply */
    { 0x201C, 8 },      /* LE Read Supported States */
    { 0x201F, 2 },      /* LE Test End */
    { 0x2020, 2 },      /* LE Remote Conn Param Request Reply */
    { 0x2021, 2 },      /* LE Remote Conn Param Request Neg Reply */
    { 0x2022, 2 },      /* LE Set Data Length */
    { 0x2023, 4 },      /* LE Read Suggested Default Data Length */
    { 0x202A, 1 },      /* LE Read Resolving List Size */
    { 0x202B, 6 },      /* LE Read Peer Resolvable Address */
    { 0x202C, 6 },      /* LE Read Local Resolvable Address */
    { 0x202F, 8 },      /* LE Read Maximum Data Length */
    { 0x2030, 4 },      /* LE Read PHY */
    { 0x2036, 1 },      /* LE Set Extended Advertising Parameters */
    { 0x203A, 2 },      /* LE Read Maximum Advertising Data Length */
    { 0x203B, 1 },      /* LE Read Number of Supported Adv Sets */
    { 0x2046, 1 },      /* LE Read Periodic Advertiser List Size */
    { 0x2049, 2 },      /* LE Read Transmit Power */
    { 0x204A, 4 },      /* LE Read RF Path Compensation */
    { 0x2060, 6 },      /* LE Read Buffer Size v2 */
    { 0x2061, 11 },     /* LE Read ISO TX Sync */
    { 0x2075, 5 },      /* LE Enhanced Read Transmit Power Level */
};

/* LMP feature pages 0 and 2 (page 1 holds the host's features) */
static const uint8_t hci_lmp_features[8] = {
    0xBF, 0xFE, 0xCF, 0xFE, 0xDB, 0xFF, 0x7B, 0x87
};
static const uint8_t hci_lmp_features_p2[8] = {
    0x00, 0x03,                     /* Secure Connections (Controller), Ping */
};

void btss_lmp_features(BesBtssState *s, int side, unsigned page,
                       uint8_t *out)
{
    BesBtssHost *h = &s->host[side];

    memset(out, 0, 8);
    switch (page) {
    case 0:
        memcpy(out, hci_lmp_features, 8);
        break;
    case 1:
        out[0] = (h->ssp ? 0x01 : 0) | (h->le ? 0x02 : 0) |
                 (h->sc ? 0x08 : 0);
        break;
    case 2:
        memcpy(out, hci_lmp_features_p2, 8);
        break;
    }
}

/* fill the interpreted return values (rp[0] is the status) */
static void hci_fill_return(BesBtssState *s, int side, uint16_t opcode,
                            const uint8_t *cmd, unsigned len, uint8_t *rp)
{
    BesBtssHost *h = &s->host[side];

    /* commands carrying a connection handle echo it first */
    switch (opcode) {
    case 0x0809: case 0x080C: case 0x080D: case 0x0C08: case 0x0C27:
    case 0x0C28: case 0x0C2D: case 0x0C36: case 0x0C37: case 0x0C68:
    case 0x0C6A: case 0x0C7B: case 0x0C7C: case 0x1401: case 0x1402:
    case 0x1403: case 0x1405: case 0x1406: case 0x1408: case 0x201A:
    case 0x201B: case 0x2015: case 0x2020: case 0x2021: case 0x2022:
    case 0x2030: case 0x2075:
        if (len >= 6) {
            memcpy(rp + 1, cmd + 4, 2);
        }
        break;
    }

    switch (opcode) {
    case 0x1001:                    /* Read Local Version Information */
        rp[1] = 0x0B;               /* HCI 5.2 */
        stw_le_p(rp + 2, s->hci_revision);
        rp[4] = 0x0B;               /* LMP 5.2 */
        stw_le_p(rp + 5, 0x02B0);   /* Bestechnic (Shenzhen) */
        break;
    case 0x1002:                    /* Read Local Supported Commands */
        memset(rp + 1, 0xFF, 64);
        break;
    case 0x1003:                    /* Read Local Supported Features */
        btss_lmp_features(s, side, 0, rp + 1);
        break;
    case 0x1004:                    /* Read Local Extended Features */
        rp[1] = len > 4 ? cmd[4] : 0;
        rp[2] = 2;                  /* max page */
        btss_lmp_features(s, side, rp[1], rp + 3);
        break;
    case 0x1005:                    /* Read Buffer Size */
        stw_le_p(rp + 1, 1021);
        rp[3] = 64;
        stw_le_p(rp + 4, 8);
        stw_le_p(rp + 6, 4);
        break;
    case 0x100A:                    /* Read Data Block Size */
        /* the ACL buffers of Read Buffer Size, as blocks of one packet */
        stw_le_p(rp + 1, 1021);
        stw_le_p(rp + 3, 1021);
        stw_le_p(rp + 5, 8);
        break;
    case 0x1009:                    /* Read BD_ADDR */
        memcpy(rp + 1, h->bdaddr, 6);
        break;
    case 0x100C:                    /* Read Local Simple Pairing Options */
        rp[2] = 16;                 /* max encryption key size */
        break;
    case 0x0C14:                    /* Read Local Name */
        memcpy(rp + 1, h->name, 248);
        break;
    case 0x0C15:                    /* Read Connection Accept Timeout */
        stw_le_p(rp + 1, 0x1F40);
        break;
    case 0x0C17:                    /* Read Page Timeout */
        stw_le_p(rp + 1, h->page_timeout);
        break;
    case 0x0C19:                    /* Read Scan Enable */
        rp[1] = h->scan_enable;
        break;
    case 0x0C23:                    /* Read Class of Device */
        memcpy(rp + 1, h->cod, 3);
        break;
    case 0x0C44:                    /* Read Inquiry Mode */
        rp[1] = h->inquiry_mode;
        break;
    case 0x0C51:                    /* Read Extended Inquiry Response */
        memcpy(rp + 1, h->eir, 241);
        break;
    case 0x0C55:                    /* Read Simple Pairing Mode */
        rp[1] = h->ssp;
        break;
    case 0x0C79:                    /* Read Secure Connections Host Support */
        rp[1] = h->sc;
        break;
    case 0x0C1B:                    /* Read Page Scan Activity */
    case 0x0C1D:                    /* Read Inquiry Scan Activity */
        stw_le_p(rp + 1, 0x0800);
        stw_le_p(rp + 3, 0x0012);
        break;
    case 0x0C25:                    /* Read Voice Setting */
        stw_le_p(rp + 1, 0x0060);
        break;
    case 0x0C38:                    /* Read Number Of Supported IAC */
        rp[1] = 1;
        break;
    case 0x0C39:                    /* Read Current IAC LAP: GIAC */
        rp[1] = 1;
        rp[2] = 0x33;
        rp[3] = 0x8B;
        rp[4] = 0x9E;
        break;
    case 0x0C6C:                    /* Read LE Host Support */
        rp[1] = h->le;
        break;
    case 0x0C7B:                    /* Read Authenticated Payload Timeout */
        stw_le_p(rp + 3, 3000);
        break;
    case 0x1408:                    /* Read Encryption Key Size */
        rp[3] = 16;
        break;
    case 0x2002:                    /* LE Read Buffer Size v1 */
    case 0x2060:                    /* LE Read Buffer Size v2 (no ISO) */
        stw_le_p(rp + 1, 251);
        rp[3] = 8;
        break;
    case 0x2003:                    /* LE Read Local Supported Features */
        rp[1] = 0xFF;
        rp[2] = 0x79;
        rp[3] = 0x01;
        break;
    case 0x200F:                    /* LE Read Filter Accept List Size */
    case 0x202A:                    /* LE Read Resolving List Size */
    case 0x2046:                    /* LE Read Periodic Advertiser List */
        rp[1] = 8;
        break;
    case 0x2015:                    /* LE Read Channel Map: all 37 */
        memset(rp + 3, 0xFF, 4);
        rp[7] = 0x1F;
        break;
    case 0x2017: {                  /* LE Encrypt: AES-128(key, plaintext) */
        AES_KEY key;
        uint8_t k[16], pt[16], ct[16];
        int i;

        if (len < 4 + 32) {
            rp[0] = 0x12;           /* invalid HCI command parameters */
            break;
        }
        /* HCI carries both little-endian (least significant octet first) */
        for (i = 0; i < 16; i++) {
            k[i] = cmd[4 + 15 - i];
            pt[i] = cmd[20 + 15 - i];
        }
        AES_set_encrypt_key(k, 128, &key);
        AES_encrypt(pt, ct, &key);
        for (i = 0; i < 16; i++) {
            rp[1 + i] = ct[15 - i];
        }
        break;
    }
    case 0x2018: {                  /* LE Rand */
        static uint64_t x = 0x2545F4914F6CDD1DULL;

        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        stq_le_p(rp + 1, x);
        break;
    }
    case 0x201C:                    /* LE Read Supported States */
        memset(rp + 1, 0xFF, 5);
        rp[6] = 0x03;
        break;
    case 0x2023:                    /* LE Read Suggested Default Data Len */
        stw_le_p(rp + 1, 27);
        stw_le_p(rp + 3, 328);
        break;
    case 0x202F:                    /* LE Read Maximum Data Length */
        stw_le_p(rp + 1, 251);
        stw_le_p(rp + 3, 2120);
        stw_le_p(rp + 5, 251);
        stw_le_p(rp + 7, 2120);
        break;
    case 0x2030:                    /* LE Read PHY: 1M/1M */
        rp[3] = 1;
        rp[4] = 1;
        break;
    case 0x203A:                    /* LE Read Max Advertising Data Length */
        stw_le_p(rp + 1, 1650);
        break;
    case 0x203B:                    /* LE Read Num Supported Adv Sets */
        rp[1] = 4;
        break;
    case 0x2049:                    /* LE Read Transmit Power */
        rp[1] = (uint8_t)-20;
        rp[2] = 10;
        break;
    }
}

static void btss_hci_execute(BesBtssState *s, int side, const uint8_t *cmd,
                             unsigned len)
{
    uint16_t opcode;
    uint8_t rp[252];
    unsigned n = 1;                 /* status only by default */
    int i;

    if (len < 4) {
        return;
    }
    opcode = lduw_le_p(cmd + 1);
    trace_bes_btss_hci_cmd(side, opcode, cmd[3]);
    if (trace_event_get_state_backends(TRACE_BES_BTSS_HCI_PARAMS)) {
        g_autofree char *hex = g_malloc0(2 * (len - 4) + 1);
        unsigned k;

        for (k = 4; k < len; k++) {
            sprintf(hex + 2 * (k - 4), "%02x", cmd[k]);
        }
        trace_bes_btss_hci_params(opcode, hex);
    }

    if (btss_link_command(s, side, opcode, cmd + 4, len - 4)) {
        return;
    }
    if (hci_uses_cmd_status(opcode)) {
        btss_cmd_status(s, side, opcode, 0x00);
        return;
    }
    memset(rp, 0, sizeof(rp));
    for (i = 0; i < ARRAY_SIZE(hci_ret_len); i++) {
        if (hci_ret_len[i].opcode == opcode) {
            n = 1 + hci_ret_len[i].len;
            break;
        }
    }
    hci_fill_return(s, side, opcode, cmd, len, rp);
    btss_cmd_complete(s, side, opcode, rp, n);
}

/* one H4 command packet from the host of controller "side" */
void btss_hci_command(BesBtssState *s, int side, const uint8_t *cmd,
                      unsigned len)
{
    /* a reset takes the controller a while (see the header) */
    if (side == BTSS_BAND && len >= 4 && lduw_le_p(cmd + 1) == 0x0C03 &&
        s->reset_latency_us) {
        s->reply_us = s->reset_latency_us;
    }
    btss_hci_execute(s, side, cmd, len);
    s->reply_us = s->latency_us;
}

/* one MCU -> BT message */
static void btss_mcu_message(BesBtssState *s, int ch, uint8_t mbx,
                             const uint8_t *data, unsigned len)
{
    trace_bes_btss_rx(ch, mbx, len, len ? data[0] : 0);
    s->chan[ch].mbx = mbx;
    if (!len) {
        return;
    }
    if (s->hci_channel >= 0 && ch != s->hci_channel) {
        /* taken, never answered: the host has no receive handler there */
        qemu_log_mask(LOG_UNIMP, "bes.btss: ch%d: %u-byte control message "
                      "0x%02x ignored\n", ch, len, data[0]);
        return;
    }
    switch (data[0]) {
    case H4_CMD:
        s->hci_ch = ch;
        btss_hci_command(s, BTSS_BAND, data, len);
        break;
    case H4_ACL:
        btss_link_acl(s, BTSS_BAND, data, len);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "bes.btss: ch%d: H4 type 0x%02x "
                      "(%u bytes) ignored\n", ch, data[0], len);
        break;
    }
}

/* MCU2BT_DATA_IND for channel ch: consume the AP's in-flight list */
static void btss_doorbell(void *opaque, int ch, int level)
{
    BesBtssState *s = BES_BTSS(opaque);
    uint32_t head_var, desc;
    unsigned guard = 0;

    if (!level) {
        return;
    }
    head_var = btss_ldl(s, s->shared_table + 8 * ch + 4);
    desc = head_var ? btss_ldl(s, head_var) : 0;
    while (desc && guard++ < 64) {
        uint32_t next = btss_ldl(s, desc);
        uint8_t mbx = address_space_ldub(&s->as, desc + 4,
                                         MEMTXATTRS_UNSPECIFIED, NULL);
        uint32_t len = btss_ldl(s, desc + 8);
        uint32_t data = btss_ldl(s, desc + 12);
        uint8_t buf[BES_BTSS_MSG_MAX];

        len = MIN(len, (uint32_t)sizeof(buf));
        address_space_read(&s->as, data, MEMTXATTRS_UNSPECIFIED, buf, len);
        btss_mcu_message(s, ch, mbx, buf, len);
        desc = next;
    }
    /* messages taken (or a bare doorbell, e.g. bt_drv register requests
     * that only wait for the indication to drop) */
    qemu_irq_pulse(s->ind_ack[ch]);
}

/* ---- BT_CMU register window ---- */

static uint64_t btss_btcmu_read(void *opaque, hwaddr offset, unsigned size)
{
    BesBtssState *s = BES_BTSS(opaque);

    return s->btcmu_regs[offset >> 2];
}

static void btss_btcmu_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    BesBtssState *s = BES_BTSS(opaque);
    int ch;

    if (offset == BTCMU_OFF_MCU_DATA_DONE) {
        for (ch = 0; ch < BES_BTSS_CHANNELS; ch++) {
            if (value & (1u << (s->data_done_shift + ch))) {
                btss_rx_done(s, ch);
            }
        }
        return;
    }
    qemu_log_mask(LOG_UNIMP, "bes.btss: BT_CMU write +0x%03x = 0x%08x\n",
                  (unsigned)offset, (uint32_t)value);
    s->btcmu_regs[offset >> 2] = value;
}

static const MemoryRegionOps btss_btcmu_ops = {
    .read = btss_btcmu_read,
    .write = btss_btcmu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void bes_btss_reset(DeviceState *dev)
{
    BesBtssState *s = BES_BTSS(dev);

    memset(s->btcmu_regs, 0, sizeof(s->btcmu_regs));
    memset(s->chan, 0, sizeof(s->chan));
    s->reply_us = s->latency_us;
    timer_del(s->publish_timer);
    btss_link_reset(s);
}

static void bes_btss_realize(DeviceState *dev, Error **errp)
{
    BesBtssState *s = BES_BTSS(dev);

    if (!s->downstream) {
        error_setg(errp, "bes.btss: 'downstream' link not set");
        return;
    }
    if (s->exch_size < BES_BTSS_CHANNELS * EXCH_CHAN_SPAN) {
        error_setg(errp, "bes.btss: exchange memory too small");
        return;
    }
    address_space_init(&s->as, s->downstream, "bes.btss-as");
    s->publish_timer = timer_new_us(QEMU_CLOCK_VIRTUAL, btss_publish_timer,
                                    s);
    s->reply_us = s->latency_us;
    btss_link_realize(s, errp);
}

static void bes_btss_init(Object *obj)
{
    BesBtssState *s = BES_BTSS(obj);

    memory_region_init_io(&s->btcmu, obj, &btss_btcmu_ops, s, "bes.bt-cmu",
                          0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->btcmu);
    qdev_init_gpio_in_named(DEVICE(obj), btss_doorbell, "doorbell",
                            BES_BTSS_CHANNELS);
    qdev_init_gpio_out_named(DEVICE(obj), s->ind_ack, "ind-ack",
                             BES_BTSS_CHANNELS);
    qdev_init_gpio_out_named(DEVICE(obj), s->bt2mcu, "bt2mcu",
                             BES_BTSS_CHANNELS);
}

static const Property bes_btss_props[] = {
    DEFINE_PROP_LINK("downstream", BesBtssState, downstream,
                     TYPE_MEMORY_REGION, MemoryRegion *),
    DEFINE_PROP_UINT32("shared-table", BesBtssState, shared_table, 0),
    DEFINE_PROP_UINT32("exch-base", BesBtssState, exch_base, 0),
    DEFINE_PROP_UINT32("exch-size", BesBtssState, exch_size, 0),
    DEFINE_PROP_UINT32("latency-us", BesBtssState, latency_us, 1000),
    DEFINE_PROP_UINT32("reset-latency-us", BesBtssState, reset_latency_us, 0),
    DEFINE_PROP_BOOL("rx-head-direct", BesBtssState, rx_head_direct, false),
    DEFINE_PROP_UINT16("hci-revision", BesBtssState, hci_revision, 0),
    DEFINE_PROP_INT32("hci-channel", BesBtssState, hci_channel, -1),
    DEFINE_PROP_UINT8("data-done-shift", BesBtssState, data_done_shift, 2),
    DEFINE_PROP_STRING("bdaddr", BesBtssState, bdaddr_str[BTSS_BAND]),
    DEFINE_PROP_STRING("peer-bdaddr", BesBtssState, bdaddr_str[BTSS_PEER]),
    DEFINE_PROP_CHR("chardev", BesBtssState, radio),
};

static void bes_btss_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = bes_btss_realize;
    device_class_set_props(dc, bes_btss_props);
    device_class_set_legacy_reset(dc, bes_btss_reset);
}

static const TypeInfo bes_btss_info = {
    .name = TYPE_BES_BTSS,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BesBtssState),
    .instance_init = bes_btss_init,
    .class_init = bes_btss_class_init,
};

static void bes_btss_register_types(void)
{
    type_register_static(&bes_btss_info);
}

type_init(bes_btss_register_types)
