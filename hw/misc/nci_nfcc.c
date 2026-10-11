/*
 * NFC controller (NFCC) speaking NCI over I2C
 *
 * Wire protocol as used by the AP firmware's I2C driver (nfc_tms_read /
 * nfc_write, sub_C713460 / sub_C713378, and the probe in sub_C713784):
 *   - the host writes one NCI packet per I2C write transaction
 *   - the DATA-READY line (host GPIO input, high = packet waiting) is
 *     asserted while the controller has output queued; the host then reads
 *     the 3-byte NCI header and, in a second read transaction, the payload
 *   - the VEN line (host output) resets the controller: low holds it in
 *     reset, the rising edge boots it (announced with a CORE_RESET_NTF
 *     only when the power-on-ntf property is set, see nci_ven)
 *   - a read transaction ends at the end of a packet: further bytes read
 *     in it are filler, the next read starts with the next packet
 * The probe identifies the chip family from a manufacturer byte in the
 * CORE_RESET_NTF that follows CORE_RESET_RSP (property hw-major, the byte
 * the NXP stack keeps as ROM version: high nibble 0xD matches the TMS
 * entry of the probe table, low nibble 1 the NXP one; the NXP stack then
 * wants ROM 0x01 and firmware major 0x10 unless the chip is a TMS one).
 * The host stack compares the firmware version in the same notification
 * with the one of the firmware file it carries and downloads it again on
 * a mismatch; the controller reports the version the host expects
 * (property fw-version, the file's own version number) so no download is
 * started.
 *
 * Packets and field layouts follow NFC Forum NCI 2.0.  The controller
 * answers the core and RF-discovery configuration commands (it reports no
 * RF discovery targets) and has the NFC execution environments of the
 * board, in the NFCEE discovery order the host stack relies on:
 *   0x01  the HCI network (HCI access interface).  SeApi_Init (sub_C71CCB8)
 *         skips an NFCEE with that interface when it collects the secure
 *         elements, and the HCI NFCEE configuration (sub_C727430) counts it
 *         on top of the NFCEEs enabled in the controller configuration
 *   0xC0  the embedded secure element (APDU interface), an HCI host
 *   0x10  the NDEF NFCEE, enabled by the host with NFCEE_MODE_SET
 * The host compares the number it discovers with the NFCEEs enabled by the
 * configuration (A0ED eSE, A095 NDEF NFCEE, A0EC/A0D4 the SWP lines of a
 * UICC) and waits for the missing ones otherwise (sub_C727430, 20 s).
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/cutils.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/nci_nfcc.h"
#include "trace.h"

#define NCI_MT_DATA     0
#define NCI_MT_CMD      1
#define NCI_MT_RSP      2
#define NCI_MT_NTF      3

#define NCI_GID_CORE    0x0
#define NCI_GID_RF      0x1
#define NCI_GID_NFCEE   0x2

#define NCI_OID_CORE_RESET          0x00
#define NCI_OID_CORE_INIT           0x01
#define NCI_OID_CORE_SET_CONFIG     0x02
#define NCI_OID_CORE_GET_CONFIG     0x03
#define NCI_OID_CORE_CONN_CREATE    0x04
#define NCI_OID_CORE_CONN_CLOSE     0x05
#define NCI_OID_CORE_CONN_CREDITS   0x06
#define NCI_OID_CORE_SET_POWER_SUB  0x09
#define NCI_OID_RF_DISCOVER_MAP     0x00
#define NCI_OID_RF_SET_LISTEN_ROUTING 0x01
#define NCI_OID_RF_GET_LISTEN_ROUTING 0x02
#define NCI_OID_RF_DISCOVER         0x03
#define NCI_OID_RF_DEACTIVATE       0x06
#define NCI_OID_RF_NFCEE_DISC_REQ   0x0A
#define NCI_OID_NFCEE_DISCOVER      0x00
#define NCI_OID_NFCEE_MODE_SET      0x01

#define NCI_STATUS_OK               0x00
#define NCI_STATUS_REJECTED         0x01
#define NCI_STATUS_SYNTAX_ERROR     0x05
#define NCI_STATUS_NFCEE_UNKNOWN    0x9F    /* "not found" in NCI 2.0 */

#define NCI_NFCEE_IF_APDU           0x00
#define NCI_NFCEE_IF_HCI_ACCESS     0x01
#define NCI_NFCEE_IF_PROPRIETARY    0x80
#define NCI_NFCEE_ENABLED           0x00
#define NCI_NFCEE_DISABLED          0x01

#define NCI_LISTEN_A                0x80    /* RF technology and mode */
#define NCI_LISTEN_B                0x81
#define NCI_PROTOCOL_ISO_DEP        0x04

#define NFCEE_ESE                   0xC0    /* the embedded NFCEE's id */

#define NCI_MAX_PACKET              260
#define NCI_QUEUE                   16
#define NCI_MAX_PARAMS              64

typedef struct {
    uint8_t id;
    uint8_t interface;
    bool enabled_at_boot;
    uint8_t listen[2];          /* card emulation technologies, 0 = none */
} NciNfcee;

static const NciNfcee nci_nfcees[] = {
    { 0x01, NCI_NFCEE_IF_HCI_ACCESS, true, { 0 } },
    { NFCEE_ESE, NCI_NFCEE_IF_APDU, true, { NCI_LISTEN_A, NCI_LISTEN_B } },
    { 0x10, NCI_NFCEE_IF_PROPRIETARY, false, { NCI_LISTEN_A } },
};
#define NCI_NUM_NFCEE   ARRAY_SIZE(nci_nfcees)

typedef struct {
    uint16_t id;
    uint8_t len;
    uint8_t val[64];
} NciParam;

typedef struct {
    uint16_t len;
    uint8_t data[NCI_MAX_PACKET];
} NciPacket;

/*
 * HCI network (ETSI TS 102 622) behind the static HCI connection (conn 1):
 * the host controller with its administration gate, the terminal host (the
 * NFC stack on the AP, 0x01) and the eSE host (0xC0).  The AP's HCI layer
 * (sub_C727B6C) runs the ETSI 12 eSE setup over it: identity management
 * pipe -> GATES_LIST -> APDU pipe -> EVT_ATR -> APDU gate registry, and
 * then needs the eSE's connectivity pipe (sub_C726300).
 */
#define HCI_HOST_CONTROLLER     0x00
#define HCI_HOST_TERMINAL       0x01
#define HCI_HOST_ESE            NFCEE_ESE

#define HCI_PIPE_ADMIN          0x01
#define HCI_PIPES               0x80

#define HCI_GATE_ID_MGMT        0x05
#define HCI_GATE_APDU           0x30
#define HCI_GATE_CONNECTIVITY   0x41

#define HCI_CB                  0x80    /* last fragment of a message */
#define HCI_TYPE_CMD            0
#define HCI_TYPE_EVT            1
#define HCI_TYPE_RSP            2

#define HCI_ANY_SET_PARAMETER   0x01
#define HCI_ANY_GET_PARAMETER   0x02
#define HCI_ANY_OPEN_PIPE       0x03
#define HCI_ANY_CLOSE_PIPE      0x04
#define HCI_ADM_CREATE_PIPE     0x10
#define HCI_ADM_DELETE_PIPE     0x11
#define HCI_ADM_NOTIFY_PIPE_CREATED 0x12
#define HCI_ADM_CLEAR_ALL_PIPE  0x14

#define HCI_ANY_OK              0x00
#define HCI_ANY_E_CMD_PAR_UNKNOWN 0x02
#define HCI_ANY_E_NOK           0x03
#define HCI_ADM_E_NO_PIPES      0x04
#define HCI_ANY_E_REG_PAR_UNKNOWN 0x05
#define HCI_ANY_E_PIPE_NOT_OPENED 0x06
#define HCI_ANY_E_CMD_NOT_SUPPORTED 0x07

/* administration gate registry */
#define HCI_REG_SESSION_IDENTITY 0x01
#define HCI_REG_WHITELIST       0x03
#define HCI_REG_HOST_LIST       0x04
#define HCI_REG_HOST_TYPE       0x06
#define HCI_REG_HOST_TYPE_LIST  0x07
/* identity management gate registry */
#define HCI_REG_GATES_LIST      0x06
/* APDU application gate registry */
#define HCI_REG_MAX_C_APDU_SIZE 0x01
#define HCI_REG_MAX_WAIT_TIME   0x02

/* APDU gate events */
#define HCI_EVT_C_APDU          0x10    /* terminal host -> eSE */
#define HCI_EVT_ABORT           0x11
#define HCI_EVT_R_APDU          0x10    /* eSE -> terminal host */
#define HCI_EVT_ATR             0x12

/*
 * The host stack expects these pipe identifiers: SeApi's HCI callback
 * (sub_C71C884) takes eSE connectivity events on 0x16 and APDU traffic on
 * 0x19, SeApi_WiredGetAtr (sub_C71E67C) sends EVT_ABORT on 0x19, and the
 * HCI layer reassembles long messages on 0x0A (UICC connectivity), 0x16
 * and 0x19 (sub_C727B6C).  Other pipes come from the remaining ids.
 */
#define HCI_PIPE_UICC_CONN      0x0A
#define HCI_PIPE_ESE_CONN       0x16
#define HCI_PIPE_ESE_APDU       0x19

#define HCI_MSG_MAX             300

typedef struct {
    bool used;
    bool open;
    uint8_t src_host, src_gate, dst_host, dst_gate;
} HciPipe;

struct NciNfccState {
    I2CSlave parent_obj;
    qemu_irq data_ready;
    QEMUTimer *ready_timer;     /* raises DATA-READY a little later */
    QEMUTimer *boot_timer;      /* power-on notification */

    uint32_t hw_major;          /* identification byte of CORE_RESET_NTF */
    uint32_t fw_version;        /* 16 bit, in the manufacturer info */
    bool power_on_ntf;          /* announce power-on with CORE_RESET_NTF */

    bool in_reset;              /* VEN low */
    bool nfcee_enabled[NCI_NUM_NFCEE];
    bool booted;
    NciParam params[NCI_MAX_PARAMS];
    unsigned num_params;
    /* HCI network */
    HciPipe pipes[HCI_PIPES];
    uint8_t hci_session_id[8];
    uint8_t hci_whitelist[4];
    unsigned hci_whitelist_len;
    uint8_t hci_host_type[2];
    uint8_t hci_msg[HCI_MSG_MAX];   /* HCP message being reassembled */
    unsigned hci_msg_len;
    uint8_t ese_channels;           /* bit n: logical channel n open */
    NciPacket q[NCI_QUEUE];
    unsigned q_head, q_count;
    unsigned rd_off;            /* bytes of the head packet already read */
    bool rd_eop;                /* this read transaction ended a packet */

    uint8_t wbuf[NCI_MAX_PACKET];
    unsigned wlen;
};

/* ---- output queue ---- */

/*
 * The line rises a short time after data is queued, like a real controller
 * that needs to finish its own work first: the host enables its interrupt
 * only after resetting the chip and writing a command.
 */
#define NCI_READY_DELAY_NS      (200 * SCALE_US)
#define NCI_BOOT_DELAY_NS       (10 * SCALE_MS)

static void nci_update_ready(NciNfccState *s)
{
    if (!s->q_count) {
        timer_del(s->ready_timer);
        qemu_irq_lower(s->data_ready);
    } else if (!timer_pending(s->ready_timer)) {
        timer_mod(s->ready_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  NCI_READY_DELAY_NS);
    }
}

static void nci_ready_tick(void *opaque)
{
    NciNfccState *s = opaque;

    qemu_set_irq(s->data_ready, s->q_count != 0);
}

static void nci_push(NciNfccState *s, uint8_t mt_gid, uint8_t oid,
                     const uint8_t *payload, unsigned plen)
{
    NciPacket *p;

    if (s->q_count == NCI_QUEUE || plen + 3 > NCI_MAX_PACKET) {
        return;
    }
    p = &s->q[(s->q_head + s->q_count) % NCI_QUEUE];
    p->data[0] = mt_gid;
    p->data[1] = oid;
    p->data[2] = plen;
    memcpy(&p->data[3], payload, plen);
    p->len = 3 + plen;
    s->q_count++;
    trace_nci_nfcc_push(p->len, s->q_count);
    nci_update_ready(s);
}

static void nci_rsp(NciNfccState *s, uint8_t gid, uint8_t oid,
                    const uint8_t *payload, unsigned plen)
{
    nci_push(s, (NCI_MT_RSP << 5) | gid, oid, payload, plen);
}

static void nci_ntf(NciNfccState *s, uint8_t gid, uint8_t oid,
                    const uint8_t *payload, unsigned plen)
{
    nci_push(s, (NCI_MT_NTF << 5) | gid, oid, payload, plen);
}

static void nci_flush(NciNfccState *s)
{
    s->q_head = s->q_count = 0;
    s->rd_off = 0;
    nci_update_ready(s);
}

/* ---- command handling ---- */

static void nci_core_reset(NciNfccState *s, const uint8_t *pl, unsigned len)
{
    /* RSP: status, NCI version, configuration status */
    const uint8_t rsp[] = { NCI_STATUS_OK, 0x20, 0x01 };
    /*
     * NTF: reason (command), config status kept, NCI version,
     * manufacturer id, manufacturer specific info length + info
     */
    const uint8_t ntf[] = { 0x02, 0x00, 0x20, 0x00, 0x05, 0x00, 0x00,
                            s->hw_major, s->fw_version >> 8,
                            s->fw_version & 0xFF };

    nci_rsp(s, NCI_GID_CORE, NCI_OID_CORE_RESET, rsp, 1);
    nci_ntf(s, NCI_GID_CORE, NCI_OID_CORE_RESET, ntf, sizeof(ntf));
}

static void nci_core_init(NciNfccState *s)
{
    /*
     * status, NFCC features (4), max logical connections, max routing
     * table size (2), max control packet payload, max data packet
     * payload, number of credits, max NFC-V frame (2), RF interfaces
     * (count, then {interface, number of extensions}).
     * Feature bit 3 (0x08) indicates HCI network support (static connection 1
     * for HCI to NFCEE).
     */
    const uint8_t rsp[] = { NCI_STATUS_OK,
                            0x0B, 0x00, 0x00, 0x00,
                            0x04, 0x00, 0x40, 0xFF, 0xFF, 0x01,
                            0x00, 0x00,
                            0x02, 0x01, 0x00, 0x04, 0x00 };

    nci_rsp(s, NCI_GID_CORE, NCI_OID_CORE_INIT, rsp, sizeof(rsp));
}

static NciParam *nci_get_param(NciNfccState *s, uint16_t id)
{
    for (unsigned i = 0; i < s->num_params; i++) {
        if (s->params[i].id == id) {
            return &s->params[i];
        }
    }
    return NULL;
}

static void nci_set_param(NciNfccState *s, uint16_t id, const uint8_t *val,
                          unsigned len)
{
    NciParam *p = nci_get_param(s, id);
    if (!p) {
        if (s->num_params >= NCI_MAX_PARAMS) {
            return;
        }
        p = &s->params[s->num_params++];
        p->id = id;
    }
    p->len = MIN(len, sizeof(p->val));
    memcpy(p->val, val, p->len);
}

static void nci_core_get_config(NciNfccState *s, const uint8_t *pl,
                                unsigned len)
{
    /*
     * Parameter ids are one byte, or two when the first is 0xA0 (vendor).
     * Returns the parameters written by CORE_SET_CONFIG and the ones the
     * controller itself provides (nci_reset).
     */
    uint8_t rsp[NCI_MAX_PACKET];
    unsigned n = len ? pl[0] : 0, i = 1, o = 2, count = 0;

    rsp[0] = NCI_STATUS_OK;
    while (count < n && i < len) {
        unsigned idlen = (pl[i] & 0xF0) == 0xA0 ? 2 : 1;
        if (i + idlen > len) {
            break;
        }
        uint16_t id = pl[i];
        if (idlen == 2) {
            id = (pl[i] << 8) | pl[i + 1];
        }
        NciParam *p = nci_get_param(s, id);
        unsigned vlen = 1;
        if (p) {
            vlen = p->len;
        } else if (id == 0xA00F) {
            vlen = 32;
        }
        if (o + idlen + 1 + vlen > 250) {
            break;
        }
        memcpy(&rsp[o], &pl[i], idlen);
        o += idlen;
        rsp[o++] = vlen;
        if (p) {
            memcpy(&rsp[o], p->val, p->len);
        } else {
            memset(&rsp[o], 0, vlen);
        }
        o += vlen;
        i += idlen;
        count++;
    }
    rsp[1] = count;
    nci_rsp(s, NCI_GID_CORE, NCI_OID_CORE_GET_CONFIG, rsp, o);
}

static int nci_find_nfcee(uint8_t id)
{
    for (unsigned i = 0; i < NCI_NUM_NFCEE; i++) {
        if (nci_nfcees[i].id == id) {
            return i;
        }
    }
    return -1;
}

/*
 * RF_NFCEE_DISCOVERY_REQ_NTF: the technologies an NFCEE card-emulates,
 * entries {type (add), length, NFCEE id, technology and mode, protocol}
 */
static void nci_nfcee_disc_req(NciNfccState *s, unsigned i)
{
    uint8_t ntf[1 + 2 * 5];
    unsigned n = 0, o = 1;

    for (unsigned t = 0; t < ARRAY_SIZE(nci_nfcees[i].listen); t++) {
        if (nci_nfcees[i].listen[t]) {
            ntf[o++] = 0x00;
            ntf[o++] = 3;
            ntf[o++] = nci_nfcees[i].id;
            ntf[o++] = nci_nfcees[i].listen[t];
            ntf[o++] = NCI_PROTOCOL_ISO_DEP;
            n++;
        }
    }
    if (n) {
        ntf[0] = n;
        nci_ntf(s, NCI_GID_RF, NCI_OID_RF_NFCEE_DISC_REQ, ntf, o);
    }
}

static void nci_command(NciNfccState *s, const uint8_t *pkt, unsigned len)
{
    uint8_t gid = pkt[0] & 0x0F, oid = pkt[1] & 0x3F;
    const uint8_t *pl = pkt + 3;
    unsigned plen = MIN((unsigned)pkt[2], len - 3);
    uint8_t status = NCI_STATUS_OK;

    trace_nci_nfcc_command(gid, oid, plen);
    switch (gid << 8 | oid) {
    case NCI_GID_CORE << 8 | NCI_OID_CORE_RESET:
        nci_core_reset(s, pl, plen);
        break;
    case NCI_GID_CORE << 8 | NCI_OID_CORE_INIT:
        nci_core_init(s);
        break;
    case NCI_GID_CORE << 8 | NCI_OID_CORE_SET_CONFIG: {
        const uint8_t rsp[] = { NCI_STATUS_OK, 0x00 };   /* none invalid */
        unsigned n = plen ? pl[0] : 0, i = 1;

        while (n-- && i < plen) {
            unsigned idlen = (pl[i] & 0xF0) == 0xA0 ? 2 : 1;
            if (i + idlen >= plen) {
                break;
            }
            uint16_t id = pl[i];
            if (idlen == 2) {
                id = (pl[i] << 8) | pl[i + 1];
            }
            unsigned vlen = pl[i + idlen];
            if (i + idlen + 1 + vlen <= plen) {
                nci_set_param(s, id, &pl[i + idlen + 1], vlen);
            }
            i += idlen + 1 + vlen;
        }
        nci_rsp(s, gid, oid, rsp, sizeof(rsp));
        break;
    }
    case NCI_GID_CORE << 8 | NCI_OID_CORE_GET_CONFIG:
        nci_core_get_config(s, pl, plen);
        break;
    case NCI_GID_NFCEE << 8 | NCI_OID_NFCEE_DISCOVER: {
        const uint8_t rsp[] = { NCI_STATUS_OK, NCI_NUM_NFCEE };

        nci_rsp(s, gid, oid, rsp, sizeof(rsp));
        for (unsigned i = 0; i < NCI_NUM_NFCEE; i++) {
            /* id, status, one interface, no information TLVs */
            const uint8_t ntf[] = {
                nci_nfcees[i].id,
                s->nfcee_enabled[i] ? NCI_NFCEE_ENABLED : NCI_NFCEE_DISABLED,
                0x01, nci_nfcees[i].interface, 0x00
            };

            nci_ntf(s, gid, oid, ntf, sizeof(ntf));
        }
        /*
         * card emulation of the NFCEEs already enabled; the host takes it
         * only once the discovery is complete (sub_C725880)
         */
        for (unsigned i = 0; i < NCI_NUM_NFCEE; i++) {
            if (s->nfcee_enabled[i]) {
                nci_nfcee_disc_req(s, i);
            }
        }
        break;
    }
    case NCI_GID_NFCEE << 8 | NCI_OID_NFCEE_MODE_SET: {
        int i = plen >= 2 ? nci_find_nfcee(pl[0]) : -1;

        if (i < 0 || pl[1] > 1) {
            status = i < 0 ? NCI_STATUS_NFCEE_UNKNOWN : NCI_STATUS_REJECTED;
            nci_rsp(s, gid, oid, &status, 1);
            break;
        }
        s->nfcee_enabled[i] = pl[1] == 1;
        nci_rsp(s, gid, oid, &status, 1);
        /* NFCEE_MODE_SET_NTF: status */
        nci_ntf(s, gid, oid, &status, 1);
        if (s->nfcee_enabled[i]) {
            nci_nfcee_disc_req(s, i);
        }
        break;
    }
    default:
        /*
         * RF discovery configuration, routing, deactivation, connection
         * management, power sub-state and vendor commands: accept
         */
        if (!(pkt[0] & 0x10)) {
            nci_rsp(s, gid, oid, &status, 1);
        }
        qemu_log_mask(LOG_UNIMP, "nci.nfcc: command gid 0x%x oid 0x%02x "
                      "(%u bytes) accepted without effect\n", gid, oid, plen);
        break;
    }
}

/* ---- HCI network ---- */

/*
 * one HCP message from the host controller or the eSE to the terminal
 * host, on the static HCI connection; fragmented with the CB bit
 */
static void hci_send(NciNfccState *s, uint8_t pipe, uint8_t type,
                     uint8_t instr, const uint8_t *data, unsigned len)
{
    uint8_t pkt[2 + HCI_MSG_MAX];
    unsigned off = 0;
    const unsigned frag = 0xFF - 1;     /* NCI payload less the HCP header */

    pkt[1] = type << 6 | instr;
    if (len) {
        memcpy(&pkt[2], data, len);
    }
    len++;                              /* message header */
    do {
        unsigned n = MIN(len - off, frag);
        uint8_t buf[0xFF];

        buf[0] = pipe | (off + n == len ? HCI_CB : 0);
        memcpy(&buf[1], &pkt[1 + off], n);
        nci_push(s, (NCI_MT_DATA << 5) | 1, 0x00, buf, n + 1);
        off += n;
    } while (off < len);
}

static void hci_rsp(NciNfccState *s, uint8_t pipe, uint8_t code,
                    const uint8_t *data, unsigned len)
{
    hci_send(s, pipe, HCI_TYPE_RSP, code, data, len);
}

static int hci_alloc_pipe(NciNfccState *s, uint8_t src_gate, uint8_t dst_host,
                          uint8_t dst_gate)
{
    int id = -1;

    if (dst_host == HCI_HOST_ESE && dst_gate == HCI_GATE_APDU) {
        id = HCI_PIPE_ESE_APDU;
    } else if (dst_host == HCI_HOST_TERMINAL &&
               dst_gate == HCI_GATE_CONNECTIVITY) {
        id = HCI_PIPE_ESE_CONN;
    } else {
        for (int i = 0x02; i < HCI_PIPES; i++) {
            if (!s->pipes[i].used && i != HCI_PIPE_UICC_CONN &&
                i != HCI_PIPE_ESE_CONN && i != HCI_PIPE_ESE_APDU) {
                id = i;
                break;
            }
        }
    }
    if (id < 0 || s->pipes[id].used) {
        return -1;
    }
    s->pipes[id] = (HciPipe) { .used = true, .src_gate = src_gate,
                               .dst_host = dst_host, .dst_gate = dst_gate };
    return id;
}

/*
 * The eSE keeps a pipe from its connectivity gate to the terminal host's,
 * for its transaction events (EVT_TRANSACTION on 0x16, sub_C71C884).  It
 * creates the pipe once the terminal host lets it (whitelist), and again
 * after the terminal host clears all pipes; the host controller notifies
 * the terminal host with ADM_NOTIFY_PIPE_CREATED (sub_C72663C).
 */
static void hci_ese_connect(NciNfccState *s)
{
    bool allowed = false;
    int id;

    for (unsigned i = 0; i < s->hci_whitelist_len; i++) {
        allowed |= s->hci_whitelist[i] == HCI_HOST_ESE;
    }
    if (!allowed || s->pipes[HCI_PIPE_ESE_CONN].used) {
        return;
    }
    id = hci_alloc_pipe(s, HCI_GATE_CONNECTIVITY, HCI_HOST_TERMINAL,
                        HCI_GATE_CONNECTIVITY);
    if (id >= 0) {
        const uint8_t notify[] = { HCI_HOST_ESE, HCI_GATE_CONNECTIVITY,
                                   HCI_HOST_TERMINAL, HCI_GATE_CONNECTIVITY,
                                   id };

        s->pipes[id].src_host = HCI_HOST_ESE;
        hci_send(s, HCI_PIPE_ADMIN, HCI_TYPE_CMD, HCI_ADM_NOTIFY_PIPE_CREATED,
                 notify, sizeof(notify));
    }
}

static void hci_admin_cmd(NciNfccState *s, uint8_t instr, const uint8_t *d,
                          unsigned len)
{
    uint8_t rsp[16];
    unsigned rlen = 0;
    uint8_t code = HCI_ANY_OK;
    int id;

    switch (instr) {
    case HCI_ANY_OPEN_PIPE:
    case HCI_ANY_CLOSE_PIPE:
        break;
    case HCI_ANY_SET_PARAMETER:
        if (len < 1) {
            code = HCI_ANY_E_CMD_PAR_UNKNOWN;
        } else if (d[0] == HCI_REG_SESSION_IDENTITY && len == 9) {
            memcpy(s->hci_session_id, &d[1], 8);
        } else if (d[0] == HCI_REG_WHITELIST &&
                   len - 1 <= sizeof(s->hci_whitelist)) {
            s->hci_whitelist_len = len - 1;
            memcpy(s->hci_whitelist, &d[1], len - 1);
        } else if (d[0] == HCI_REG_HOST_TYPE && len == 3) {
            memcpy(s->hci_host_type, &d[1], 2);
        } else {
            code = HCI_ANY_E_REG_PAR_UNKNOWN;
        }
        break;
    case HCI_ANY_GET_PARAMETER:
        if (len < 1) {
            code = HCI_ANY_E_CMD_PAR_UNKNOWN;
        } else if (d[0] == HCI_REG_SESSION_IDENTITY) {
            memcpy(rsp, s->hci_session_id, 8);
            rlen = 8;
        } else if (d[0] == HCI_REG_WHITELIST) {
            memcpy(rsp, s->hci_whitelist, s->hci_whitelist_len);
            rlen = s->hci_whitelist_len;
        } else if (d[0] == HCI_REG_HOST_LIST) {
            rsp[0] = HCI_HOST_CONTROLLER;
            rsp[1] = HCI_HOST_TERMINAL;
            rsp[2] = HCI_HOST_ESE;
            rlen = 3;
        } else if (d[0] == HCI_REG_HOST_TYPE) {
            memcpy(rsp, s->hci_host_type, 2);
            rlen = 2;
        } else if (d[0] == HCI_REG_HOST_TYPE_LIST) {
            /*
             * in HOST_LIST order: host controller, terminal host (0x01xx),
             * eSE (0x03xx; sub_C726AA8 takes 0x0300 as host 0xC0)
             */
            static const uint8_t types[] = { 0x00, 0x00, 0x01, 0x00,
                                             0x03, 0x00 };

            memcpy(rsp, types, sizeof(types));
            rlen = sizeof(types);
        } else {
            code = HCI_ANY_E_REG_PAR_UNKNOWN;
        }
        break;
    case HCI_ADM_CREATE_PIPE:
        /* source gate, destination host, destination gate */
        if (len < 3 || d[1] != HCI_HOST_ESE) {
            code = HCI_ANY_E_NOK;
            break;
        }
        id = hci_alloc_pipe(s, d[0], d[1], d[2]);
        if (id < 0) {
            code = HCI_ADM_E_NO_PIPES;
            break;
        }
        s->pipes[id].src_host = HCI_HOST_TERMINAL;
        rsp[0] = HCI_HOST_TERMINAL;
        rsp[1] = d[0];
        rsp[2] = d[1];
        rsp[3] = d[2];
        rsp[4] = id;
        rlen = 5;
        break;
    case HCI_ADM_DELETE_PIPE:
        if (len < 1 || d[0] < 0x02 || d[0] >= HCI_PIPES ||
            !s->pipes[d[0]].used) {
            code = HCI_ANY_E_CMD_PAR_UNKNOWN;
        } else {
            s->pipes[d[0]] = (HciPipe) { 0 };
        }
        break;
    case HCI_ADM_CLEAR_ALL_PIPE:
        /*
         * the terminal host's pipes and the eSE's pipes to it go; the
         * eSE is notified and creates its pipe again once allowed
         */
        for (int i = 0x02; i < HCI_PIPES; i++) {
            s->pipes[i] = (HciPipe) { 0 };
        }
        memset(s->hci_session_id, 0xFF, sizeof(s->hci_session_id));
        s->ese_channels = 1;
        break;
    default:
        code = HCI_ANY_E_CMD_NOT_SUPPORTED;
        break;
    }
    hci_rsp(s, HCI_PIPE_ADMIN, code, rsp, rlen);
    if (code == HCI_ANY_OK && (instr == HCI_ADM_CLEAR_ALL_PIPE ||
        (instr == HCI_ANY_SET_PARAMETER && d[0] == HCI_REG_WHITELIST))) {
        hci_ese_connect(s);
    }
}

/*
 * ATR of the eSE (T=1).  nfc_exception_recovery (sub_C7355C8) reads bytes
 * 8 and 10 as the ASCII digits of the secure element version; they are in
 * the historical bytes "eSE 1.0" here.
 */
static const uint8_t ese_atr[] = {
    0x3B, 0x88, 0x80, 0x01,                     /* TS T0 TD1 TD2 */
    'e', 'S', 'E', ' ', '1', '.', '0', ' ',     /* historical bytes */
    0x88 ^ 0x80 ^ 0x01 ^ 'e' ^ 'S' ^ 'E' ^ ' ' ^ '1' ^ '.' ^ '0' ^ ' ',
                                                /* TCK: T0 .. last byte */
};

/*
 * The eSE's card manager without installed applets: logical channels and
 * SELECT work, no application is found
 */
static void ese_apdu(NciNfccState *s, uint8_t pipe, const uint8_t *c,
                     unsigned len)
{
    uint8_t r[3];
    unsigned rlen = 2;
    uint8_t ins = len >= 4 ? c[1] : 0;
    uint8_t chan = len >= 1 ? (c[0] & 0x40 ? 4 + (c[0] & 0x0F) : c[0] & 3) : 0;

    if (len < 4) {
        r[0] = 0x67;                            /* wrong length */
        r[1] = 0x00;
    } else if (chan >= 8 || !(s->ese_channels & BIT(chan))) {
        r[0] = 0x68;                            /* channel not supported */
        r[1] = 0x81;
    } else if (ins == 0x70 && c[2] == 0x00) {   /* MANAGE CHANNEL open */
        /*
         * P2 = 0: the card picks the channel and returns its number;
         * otherwise P2 names the channel to open (ISO/IEC 7816-4)
         */
        unsigned n = c[3];

        if (!n) {
            for (n = 1; n < 4 && (s->ese_channels & BIT(n)); n++) {
            }
        }
        if (n >= 4 || (s->ese_channels & BIT(n))) {
            r[0] = 0x6A;                        /* function not supported */
            r[1] = 0x81;
        } else {
            s->ese_channels |= BIT(n);
            if (c[3]) {
                r[0] = 0x90;
                r[1] = 0x00;
            } else {
                r[0] = n;
                r[1] = 0x90;
                r[2] = 0x00;
                rlen = 3;
            }
        }
    } else if (ins == 0x70 && c[2] == 0x80) {   /* MANAGE CHANNEL close */
        if (c[3] == 0 || c[3] >= 8 || !(s->ese_channels & BIT(c[3]))) {
            r[0] = 0x68;
            r[1] = 0x81;
        } else {
            s->ese_channels &= ~BIT(c[3]);
            r[0] = 0x90;
            r[1] = 0x00;
        }
    } else if (ins == 0xA4) {                   /* SELECT */
        r[0] = 0x6A;                            /* file or application */
        r[1] = 0x82;                            /* not found */
    } else {
        r[0] = 0x6D;                            /* INS not supported */
        r[1] = 0x00;
    }
    hci_send(s, pipe, HCI_TYPE_EVT, HCI_EVT_R_APDU, r, rlen);
}

/* a message on a pipe between the terminal host and an eSE gate */
static void hci_ese_msg(NciNfccState *s, uint8_t pipe, uint8_t type,
                        uint8_t instr, const uint8_t *d, unsigned len)
{
    HciPipe *p = &s->pipes[pipe];
    uint8_t gate = p->src_host == HCI_HOST_ESE ? p->src_gate : p->dst_gate;
    uint8_t rsp[HCI_MSG_MAX] = { 0 };
    unsigned rlen = 0;
    uint8_t code = HCI_ANY_OK;

    if (type == HCI_TYPE_RSP) {
        return;
    }
    if (type == HCI_TYPE_EVT) {
        if (!p->open || gate != HCI_GATE_APDU) {
            return;
        }
        if (instr == HCI_EVT_C_APDU) {
            ese_apdu(s, pipe, d, len);
        } else if (instr == HCI_EVT_ABORT) {
            s->ese_channels = 1;
            hci_send(s, pipe, HCI_TYPE_EVT, HCI_EVT_ATR, ese_atr,
                     sizeof(ese_atr));
        }
        return;
    }

    switch (instr) {
    case HCI_ANY_OPEN_PIPE:
        p->open = true;
        break;
    case HCI_ANY_CLOSE_PIPE:
        p->open = false;
        break;
    case HCI_ANY_GET_PARAMETER:
        if (!p->open) {
            code = HCI_ANY_E_PIPE_NOT_OPENED;
        } else if (len < 1) {
            code = HCI_ANY_E_CMD_PAR_UNKNOWN;
        } else if (gate == HCI_GATE_ID_MGMT && d[0] == HCI_REG_GATES_LIST) {
            rsp[0] = HCI_GATE_ID_MGMT;
            rsp[1] = HCI_GATE_APDU;
            rsp[2] = HCI_GATE_CONNECTIVITY;
            rlen = 3;
        } else if (gate == HCI_GATE_APDU && d[0] == HCI_REG_MAX_C_APDU_SIZE) {
            stw_be_p(rsp, 261);                 /* short APDU, 255 data */
            rlen = 2;
        } else if (gate == HCI_GATE_APDU && d[0] == HCI_REG_MAX_WAIT_TIME) {
            stw_be_p(rsp, 1000);                /* ms */
            rlen = 2;
        } else {
            code = HCI_ANY_E_REG_PAR_UNKNOWN;
        }
        break;
    case HCI_ANY_SET_PARAMETER:
        code = HCI_ANY_E_REG_PAR_UNKNOWN;
        break;
    default:
        code = HCI_ANY_E_CMD_NOT_SUPPORTED;
        break;
    }
    hci_rsp(s, pipe, code, rsp, rlen);
    /* the APDU application gate announces itself on an opened pipe */
    if (instr == HCI_ANY_OPEN_PIPE && gate == HCI_GATE_APDU) {
        s->ese_channels = 1;
        hci_send(s, pipe, HCI_TYPE_EVT, HCI_EVT_ATR, ese_atr,
                 sizeof(ese_atr));
    }
}

/* a complete HCP message from the terminal host */
static void hci_msg(NciNfccState *s, uint8_t pipe, const uint8_t *m,
                    unsigned len)
{
    uint8_t type = m[0] >> 6, instr = m[0] & 0x3F;

    trace_nci_nfcc_hci(pipe, type, instr, len - 1);
    if (pipe == HCI_PIPE_ADMIN) {
        if (type == HCI_TYPE_CMD) {
            hci_admin_cmd(s, instr, m + 1, len - 1);
        }
    } else if (pipe < HCI_PIPES && s->pipes[pipe].used) {
        hci_ese_msg(s, pipe, type, instr, m + 1, len - 1);
    } else if (type == HCI_TYPE_CMD) {
        hci_rsp(s, pipe, HCI_ANY_E_NOK, NULL, 0);
    }
}

static void nci_data(NciNfccState *s, const uint8_t *pkt, unsigned len)
{
    uint8_t conn_id = pkt[0] & 0x0F;
    const uint8_t *pl = pkt + 3;
    unsigned plen = MIN((unsigned)pkt[2], len - 3);
    /* the packet's credit goes back to the host */
    const uint8_t credits[] = { 0x01, conn_id, 0x01 };

    nci_ntf(s, NCI_GID_CORE, NCI_OID_CORE_CONN_CREDITS, credits,
            sizeof(credits));
    if (conn_id != 1 || plen < 1) {
        return;
    }
    /*
     * HCP packets: pipe | CB, then the message (first fragment carries
     * the message header)
     */
    if (s->hci_msg_len + plen - 1 > sizeof(s->hci_msg)) {
        s->hci_msg_len = 0;
        return;
    }
    memcpy(&s->hci_msg[s->hci_msg_len], pl + 1, plen - 1);
    s->hci_msg_len += plen - 1;
    if (pl[0] & HCI_CB) {
        if (s->hci_msg_len) {
            hci_msg(s, pl[0] & 0x7F, s->hci_msg, s->hci_msg_len);
        }
        s->hci_msg_len = 0;
    }
}

/* ---- I2C ---- */

static int nci_event(I2CSlave *i2c, enum i2c_event event)
{
    NciNfccState *s = NCI_NFCC(i2c);

    switch (event) {
    case I2C_START_SEND:
        s->wlen = 0;
        break;
    case I2C_START_RECV:
        s->rd_eop = false;
        break;
    case I2C_FINISH:
        trace_nci_nfcc_write(s->wlen);
        if (trace_event_get_state_backends(TRACE_NCI_NFCC_PACKET) && s->wlen) {
            GString *hex = qemu_hexdump_line(NULL, s->wbuf,
                                             MIN(s->wlen, 48), 0, 0);

            trace_nci_nfcc_packet(hex->str);
            g_string_free(hex, true);
        }
        if (s->wlen >= 3 && s->booted && !s->in_reset) {
            if (s->wbuf[0] == 0x02 && s->wbuf[1] == 0x05) {
                /*
                 * During NFA enable the host's NCI layer writes a 32-byte
                 * buffer headed 02 05 01 00 (the following SET_CONFIG
                 * shifted by one byte, then stale data) and waits for a
                 * response to it as to a command with GID 2 / OID 5;
                 * without one it times out after 2 s and NFA_Enable
                 * fails.  The controller answers it as an unknown
                 * command.
                 */
                const uint8_t rsp[] = { NCI_STATUS_SYNTAX_ERROR };

                nci_rsp(s, NCI_GID_NFCEE, 0x05, rsp, 1);
            } else if ((s->wbuf[0] >> 5) == NCI_MT_CMD) {
                nci_command(s, s->wbuf, s->wlen);
            } else if ((s->wbuf[0] >> 5) == NCI_MT_DATA) {
                nci_data(s, s->wbuf, s->wlen);
            }
        }
        s->wlen = 0;
        break;
    default:
        break;
    }
    return 0;
}

static int nci_send(I2CSlave *i2c, uint8_t data)
{
    NciNfccState *s = NCI_NFCC(i2c);

    if (s->in_reset) {
        return -1;                      /* a controller in reset NAKs */
    }
    if (s->wlen < sizeof(s->wbuf)) {
        s->wbuf[s->wlen++] = data;
    }
    return 0;
}

static uint8_t nci_recv(I2CSlave *i2c)
{
    NciNfccState *s = NCI_NFCC(i2c);
    NciPacket *p;
    uint8_t b;

    /*
     * Reading past the end of a packet returns filler bytes until the
     * transaction ends; the next read starts with the next packet.  The
     * p67 driver reads a one-byte payload as two bytes and drops the
     * second, the q63 one reads exactly the payload length.
     */
    if (!s->q_count || s->rd_eop) {
        return 0xFF;
    }
    p = &s->q[s->q_head];
    trace_nci_nfcc_recv(s->rd_off, p->data[s->rd_off]);
    b = p->data[s->rd_off++];
    if (s->rd_off == p->len) {
        s->q_head = (s->q_head + 1) % NCI_QUEUE;
        s->q_count--;
        s->rd_off = 0;
        s->rd_eop = true;
        /*
         * one interrupt per packet: drop the line, then raise it again
         * for the next queued packet so the host sees a new edge
         */
        qemu_irq_lower(s->data_ready);
        nci_update_ready(s);
    }
    return b;
}

/*
 * VEN (host output, active high enable): reset on low, boot on high.  The
 * controller takes commands right away (the q63 stack writes CORE_RESET a
 * few milliseconds after raising VEN).  It announces power-on with a
 * CORE_RESET_NTF only when power-on-ntf is set: the q63 driver's chip
 * probe (nfc_check_chipinfo, 0x1088DA68) reads the reply to its
 * CORE_RESET_CMD as the first two packets, and the p67 driver skips such
 * a notification ("recv nfcc power on ntf, continue") either way.
 */
static void nci_boot_tick(void *opaque)
{
    NciNfccState *s = opaque;
    const uint8_t ntf[] = { 0x01 };         /* reset: power on */

    nci_ntf(s, NCI_GID_CORE, NCI_OID_CORE_RESET, ntf, 1);
}

static void nci_ven(void *opaque, int n, int level)
{
    NciNfccState *s = NCI_NFCC(opaque);

    if (!level) {
        s->in_reset = true;
        s->booted = false;
        timer_del(s->boot_timer);
        nci_flush(s);
        return;
    }
    if (s->in_reset) {
        s->in_reset = false;
        s->booted = true;
        trace_nci_nfcc_boot();
        if (s->power_on_ntf) {
            timer_mod(s->boot_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      NCI_BOOT_DELAY_NS);
        }
    }
}

static void nci_reset(DeviceState *dev)
{
    NciNfccState *s = NCI_NFCC(dev);

    s->in_reset = false;
    s->booted = true;
    s->wlen = 0;
    s->num_params = 0;
    /*
     * HCI network: no pipes, SESSION_IDENTITY at its default (all 0xFF),
     * so the terminal host finds its stored session gone and sets the
     * network up again
     */
    memset(s->pipes, 0, sizeof(s->pipes));
    memset(s->hci_session_id, 0xFF, sizeof(s->hci_session_id));
    s->hci_whitelist_len = 0;
    memset(s->hci_host_type, 0, sizeof(s->hci_host_type));
    s->hci_msg_len = 0;
    s->ese_channels = 1;
    for (unsigned i = 0; i < NCI_NUM_NFCEE; i++) {
        s->nfcee_enabled[i] = nci_nfcees[i].enabled_at_boot;
    }
    /*
     * A014: JCOP OS state of the eSE, byte 1 = 3 (initialised; SeApi_Init
     * queries it, "Querying JCOP State").  A0EB: HCI session identity of
     * the eSE; all 0xFF means its HCI initialisation has not finished and
     * the host polls it (sub_C7275D8)
     */
    static const uint8_t jcop_state[32] = { 0x47, 0x03 };
    static const uint8_t ese_session[8] = {
        0x45, 0x53, 0x45, 0x00, 0x00, 0x00, 0x00, 0x01
    };
    nci_set_param(s, 0xA014, jcop_state, sizeof(jcop_state));
    nci_set_param(s, 0xA0EB, ese_session, sizeof(ese_session));
    timer_del(s->boot_timer);
    nci_flush(s);
}

static void nci_realize(DeviceState *dev, Error **errp)
{
    NciNfccState *s = NCI_NFCC(dev);

    s->ready_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, nci_ready_tick, s);
    s->boot_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, nci_boot_tick, s);
}

static void nci_init(Object *obj)
{
    NciNfccState *s = NCI_NFCC(obj);

    qdev_init_gpio_out_named(DEVICE(obj), &s->data_ready, "data-ready", 1);
    qdev_init_gpio_in_named(DEVICE(obj), nci_ven, "ven", 1);
}

static const Property nci_props[] = {
    DEFINE_PROP_UINT32("hw-major", NciNfccState, hw_major, 0xD1),
    DEFINE_PROP_UINT32("fw-version", NciNfccState, fw_version, 0x1C00),
    DEFINE_PROP_BOOL("power-on-ntf", NciNfccState, power_on_ntf, false),
};

static void nci_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    dc->realize = nci_realize;
    device_class_set_legacy_reset(dc, nci_reset);
    device_class_set_props(dc, nci_props);
    k->event = nci_event;
    k->send = nci_send;
    k->recv = nci_recv;
}

static const TypeInfo nci_info = {
    .name = TYPE_NCI_NFCC,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(NciNfccState),
    .instance_init = nci_init,
    .class_init = nci_class_init,
};

static void nci_register_types(void)
{
    type_register_static(&nci_info);
}

type_init(nci_register_types)
