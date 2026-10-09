/*
 * BES Bluetooth subsystem stand-in: the virtual air interface
 *
 * Two BR/EDR controllers share one piconet: the band's (its host is the
 * AP firmware, over the mailbox, see bes_btss.c) and a peer's, whose host
 * drives the "chardev" backend as an H4 UART transport (Bluetooth Core
 * Specification v5.3 Vol 4 Part A). Both answer the same HCI command set
 * (Vol 4 Part E); what would cross the radio -- inquiry, paging, the LMP
 * authentication and Secure Simple Pairing exchanges, encryption start
 * and ACL-U data -- is resolved here between the two hosts, each seeing
 * the events its own controller would report (the message sequence
 * charts of Vol 2 Part F).
 *
 * Nothing over the air is observable by either host, so it is not
 * modelled: procedures complete as soon as both hosts have answered, the
 * link key of a pairing is random rather than derived from the ECDH
 * exchange, encryption is a link flag and ACL payloads pass unchanged.
 * Legacy (PIN) pairing is not modelled; both hosts here enable Secure
 * Simple Pairing.
 *
 * Attaching a peer host, e.g. over TCP:
 *   -chardev socket,id=bt,host=127.0.0.1,port=5560,server=on,wait=off
 *   -global driver=bes.btss,property=chardev,value=bt
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/guest-random.h"
#include "chardev/char-fe.h"
#include "bes_btss_int.h"
#include "trace.h"

#define BTSS_ACL_HANDLE         0x0001
#define BTSS_GIAC               0x9E8B33
#define BTSS_PAGE_TIMEOUT_DFLT  0x2000
#define BTSS_MANUFACTURER       0x02B0  /* Bestechnic (Shenzhen) */
#define BTSS_BAND_RESERVE       8       /* events one peer packet can raise */

/* events (Vol 4 Part E 7.7) */
#define EVT_INQUIRY_COMPLETE    0x01
#define EVT_INQUIRY_RESULT      0x02
#define EVT_CONN_COMPLETE       0x03
#define EVT_CONN_REQUEST        0x04
#define EVT_DISCONN_COMPLETE    0x05
#define EVT_AUTH_COMPLETE       0x06
#define EVT_REMOTE_NAME         0x07
#define EVT_ENCRYPT_CHANGE      0x08
#define EVT_REMOTE_FEATURES     0x0B
#define EVT_REMOTE_VERSION      0x0C
#define EVT_ROLE_CHANGE         0x12
#define EVT_NUM_COMPLETED       0x13
#define EVT_MODE_CHANGE         0x14
#define EVT_LINK_KEY_REQUEST    0x17
#define EVT_LINK_KEY_NOTIFY     0x18
#define EVT_CLOCK_OFFSET        0x1C
#define EVT_INQUIRY_RESULT_RSSI 0x22
#define EVT_REMOTE_EXT_FEATURES 0x23
#define EVT_EXT_INQUIRY_RESULT  0x2F
#define EVT_IO_CAP_REQUEST      0x31
#define EVT_IO_CAP_RESPONSE     0x32
#define EVT_USER_CONFIRM        0x33
#define EVT_USER_PASSKEY        0x34
#define EVT_SSP_COMPLETE        0x36
#define EVT_PASSKEY_NOTIFY      0x3B

/* error codes (Vol 1 Part F) */
#define ERR_UNKNOWN_CONN        0x02
#define ERR_PAGE_TIMEOUT        0x04
#define ERR_AUTH_FAILURE        0x05
#define ERR_KEY_MISSING         0x06
#define ERR_CONN_TIMEOUT        0x08
#define ERR_CONN_EXISTS         0x0B
#define ERR_CMD_DISALLOWED      0x0C
#define ERR_ACCEPT_TIMEOUT      0x10
#define ERR_INVALID_PARAMS      0x12
#define ERR_LOCAL_HOST_TERM     0x16
#define ERR_PAIRING_NOT_ALLOWED 0x18

/* IO capabilities (Vol 4 Part E 7.1.29) */
#define IO_DISPLAY_YES_NO       0x01
#define IO_KEYBOARD_ONLY        0x02
#define IO_NO_IO                0x03

static inline int other(int side)
{
    return side ^ 1;
}

/* whether the controller of "side" is on the air */
static bool side_present(BesBtssState *s, int side)
{
    return side == BTSS_BAND || s->radio_open;
}

static bool addr_is(BesBtssState *s, int side, const uint8_t *addr)
{
    return !memcmp(s->host[side].bdaddr, addr, 6);
}

static bool link_handle_is(BesBtssState *s, const uint8_t *p)
{
    return s->link.state == BTSS_LINK_UP &&
           (lduw_le_p(p) & 0x0FFF) == s->link.handle;
}

/* ---- event builders ---- */

/* events whose only parameter is the remote BD_ADDR */
static void ev_addr(BesBtssState *s, int side, uint8_t code)
{
    btss_event(s, side, code, s->host[other(side)].bdaddr, 6);
}

static void ev_status_handle(BesBtssState *s, int side, uint8_t code,
                             uint8_t status)
{
    uint8_t p[3] = { status };

    stw_le_p(p + 1, s->link.handle);
    btss_event(s, side, code, p, sizeof(p));
}

static void ev_conn_complete(BesBtssState *s, int side, uint8_t status,
                             const uint8_t *addr)
{
    uint8_t p[11] = { status };

    stw_le_p(p + 1, status ? 0 : s->link.handle);
    memcpy(p + 3, addr, 6);
    p[9] = 0x01;                    /* ACL */
    p[10] = 0x00;                   /* encryption off */
    btss_event(s, side, EVT_CONN_COMPLETE, p, sizeof(p));
}

static void ev_disconn_complete(BesBtssState *s, int side, uint8_t reason)
{
    uint8_t p[4] = { 0x00 };

    stw_le_p(p + 1, s->link.handle);
    p[3] = reason;
    btss_event(s, side, EVT_DISCONN_COMPLETE, p, sizeof(p));
}

static void ev_role_change(BesBtssState *s, int side)
{
    uint8_t p[8] = { 0x00 };

    memcpy(p + 1, s->host[other(side)].bdaddr, 6);
    p[7] = s->link.central == side ? 0x00 : 0x01;
    btss_event(s, side, EVT_ROLE_CHANGE, p, sizeof(p));
}

static void ev_mode_change(BesBtssState *s, int side, uint8_t mode,
                           uint16_t interval)
{
    uint8_t p[6] = { 0x00 };

    stw_le_p(p + 1, s->link.handle);
    p[3] = mode;
    stw_le_p(p + 4, interval);
    btss_event(s, side, EVT_MODE_CHANGE, p, sizeof(p));
}

/* Command Complete of the commands that return Status + BD_ADDR */
static void cmd_complete_addr(BesBtssState *s, int side, uint16_t opcode,
                              uint8_t status, const uint8_t *addr)
{
    uint8_t rp[7] = { status };

    memcpy(rp + 1, addr, 6);
    btss_cmd_complete(s, side, opcode, rp, sizeof(rp));
}

static void link_down(BesBtssState *s)
{
    trace_bes_btss_link("link down");
    memset(&s->link, 0, sizeof(s->link));
}

/* ---- inquiry ---- */

static void inquiry_complete(BesBtssState *s, int side)
{
    uint8_t status = 0x00;

    timer_del(s->host[side].inq_timer);
    btss_event(s, side, EVT_INQUIRY_COMPLETE, &status, 1);
}

static void inquiry_band_expired(void *opaque)
{
    inquiry_complete(opaque, BTSS_BAND);
}

static void inquiry_peer_expired(void *opaque)
{
    inquiry_complete(opaque, BTSS_PEER);
}

/* the response of the other controller, in the inquirer's Inquiry_Mode */
static void inquiry_result(BesBtssState *s, int side)
{
    BesBtssHost *t = &s->host[other(side)];
    uint8_t p[255] = { 1 };         /* Num_Responses */
    uint8_t code;
    unsigned n;

    memcpy(p + 1, t->bdaddr, 6);
    p[7] = 0x01;                    /* Page_Scan_Repetition_Mode R1 */
    switch (s->host[side].inquiry_mode) {
    case 0:
        memcpy(p + 10, t->cod, 3);  /* p[8..9] reserved */
        code = EVT_INQUIRY_RESULT;
        n = 15;
        break;
    case 1:
        memcpy(p + 9, t->cod, 3);
        p[14] = (uint8_t)-40;       /* RSSI */
        code = EVT_INQUIRY_RESULT_RSSI;
        n = 15;
        break;
    default:
        memcpy(p + 9, t->cod, 3);
        p[14] = (uint8_t)-40;
        memcpy(p + 15, t->eir + 1, 240);
        code = EVT_EXT_INQUIRY_RESULT;
        n = 255;
        break;
    }
    btss_event(s, side, code, p, n);
}

static void link_inquiry(BesBtssState *s, int side, const uint8_t *p,
                         unsigned plen)
{
    BesBtssHost *h = &s->host[side];
    BesBtssHost *t = &s->host[other(side)];
    uint32_t lap;

    if (plen < 5 || p[3] < 0x01 || p[3] > 0x30) {
        btss_cmd_status(s, side, 0x0401, ERR_INVALID_PARAMS);
        return;
    }
    if (timer_pending(h->inq_timer)) {
        btss_cmd_status(s, side, 0x0401, ERR_CMD_DISALLOWED);
        return;
    }
    btss_cmd_status(s, side, 0x0401, 0x00);
    lap = p[0] | p[1] << 8 | p[2] << 16;
    if (side_present(s, other(side)) && (t->scan_enable & 0x01) &&
        lap == t->iac) {
        inquiry_result(s, side);
        if (p[4] == 1) {            /* Num_Responses reached */
            inquiry_complete(s, side);
            return;
        }
    }
    timer_mod(h->inq_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + p[3] * 1280);
}

static void link_inquiry_cancel(BesBtssState *s, int side)
{
    uint8_t status = timer_pending(s->host[side].inq_timer) ?
                     0x00 : ERR_CMD_DISALLOWED;

    timer_del(s->host[side].inq_timer);
    btss_cmd_complete(s, side, 0x0402, &status, 1);
}

static void link_remote_name(BesBtssState *s, int side, const uint8_t *p,
                             unsigned plen)
{
    int t = other(side);
    uint8_t ev[255] = { 0 };

    if (plen < 6) {
        btss_cmd_status(s, side, 0x0419, ERR_INVALID_PARAMS);
        return;
    }
    btss_cmd_status(s, side, 0x0419, 0x00);
    memcpy(ev + 1, p, 6);
    if (addr_is(s, t, p) && side_present(s, t) &&
        (s->link.state == BTSS_LINK_UP || (s->host[t].scan_enable & 0x02))) {
        memcpy(ev + 7, s->host[t].name, 248);
    } else {
        ev[0] = ERR_PAGE_TIMEOUT;
    }
    btss_event(s, side, EVT_REMOTE_NAME, ev, sizeof(ev));
}

/* ---- connection setup ---- */

static void link_create_connection(BesBtssState *s, int side,
                                   const uint8_t *p, unsigned plen)
{
    BesBtssHost *h = &s->host[side];
    int t = other(side);
    uint8_t ev[10];

    if (plen < 13) {
        btss_cmd_status(s, side, 0x0405, ERR_INVALID_PARAMS);
        return;
    }
    if (s->link.state != BTSS_LINK_IDLE) {
        btss_cmd_status(s, side, 0x0405, addr_is(s, t, p) ?
                        ERR_CONN_EXISTS : ERR_CMD_DISALLOWED);
        return;
    }
    btss_cmd_status(s, side, 0x0405, 0x00);
    if (!addr_is(s, t, p) || !side_present(s, t) ||
        !(s->host[t].scan_enable & 0x02)) {
        ev_conn_complete(s, side, ERR_PAGE_TIMEOUT, p);
        return;
    }
    trace_bes_btss_link("paging");
    s->link.state = BTSS_LINK_PAGING;
    s->link.central = side;
    memcpy(ev, h->bdaddr, 6);
    memcpy(ev + 6, h->cod, 3);
    ev[9] = 0x01;                   /* ACL */
    btss_event(s, t, EVT_CONN_REQUEST, ev, sizeof(ev));
}

static void link_create_connection_cancel(BesBtssState *s, int side,
                                          const uint8_t *p, unsigned plen)
{
    if (plen < 6) {
        return;
    }
    if (s->link.state != BTSS_LINK_PAGING || s->link.central != side ||
        !addr_is(s, other(side), p)) {
        cmd_complete_addr(s, side, 0x0408, s->link.state == BTSS_LINK_UP ?
                          ERR_CONN_EXISTS : ERR_UNKNOWN_CONN, p);
        return;
    }
    cmd_complete_addr(s, side, 0x0408, 0x00, p);
    ev_conn_complete(s, side, ERR_UNKNOWN_CONN, p);
    link_down(s);
}

static void link_accept(BesBtssState *s, int side, const uint8_t *p,
                        unsigned plen)
{
    int i;

    if (plen < 7 || s->link.state != BTSS_LINK_PAGING ||
        s->link.central == side || !addr_is(s, other(side), p)) {
        btss_cmd_status(s, side, 0x0409, ERR_UNKNOWN_CONN);
        return;
    }
    btss_cmd_status(s, side, 0x0409, 0x00);
    trace_bes_btss_link("link up");
    s->link.state = BTSS_LINK_UP;
    s->link.handle = BTSS_ACL_HANDLE;
    if (p[6] == 0x00) {             /* the acceptor becomes central */
        s->link.central = side;
        for (i = 0; i < BTSS_SIDES; i++) {
            ev_role_change(s, i);
        }
    }
    for (i = 0; i < BTSS_SIDES; i++) {
        ev_conn_complete(s, i, 0x00, s->host[other(i)].bdaddr);
    }
}

static void link_reject(BesBtssState *s, int side, const uint8_t *p,
                        unsigned plen)
{
    if (plen < 7 || s->link.state != BTSS_LINK_PAGING ||
        s->link.central == side || !addr_is(s, other(side), p)) {
        btss_cmd_status(s, side, 0x040A, ERR_UNKNOWN_CONN);
        return;
    }
    btss_cmd_status(s, side, 0x040A, 0x00);
    ev_conn_complete(s, other(side), p[6], s->host[side].bdaddr);
    link_down(s);
}

static void link_disconnect(BesBtssState *s, int side, const uint8_t *p,
                            unsigned plen)
{
    if (plen < 3 || !link_handle_is(s, p)) {
        btss_cmd_status(s, side, 0x0406, ERR_UNKNOWN_CONN);
        return;
    }
    btss_cmd_status(s, side, 0x0406, 0x00);
    ev_disconn_complete(s, side, ERR_LOCAL_HOST_TERM);
    ev_disconn_complete(s, other(side), p[2]);
    link_down(s);
}

/* the link ends without a host asking (controller reset, peer gone) */
static void link_lost(BesBtssState *s, int gone)
{
    int t = other(gone);

    switch (s->link.state) {
    case BTSS_LINK_UP:
        ev_disconn_complete(s, t, ERR_CONN_TIMEOUT);
        break;
    case BTSS_LINK_PAGING:
        ev_conn_complete(s, t, s->link.central == t ?
                         ERR_PAGE_TIMEOUT : ERR_ACCEPT_TIMEOUT,
                         s->host[gone].bdaddr);
        break;
    default:
        return;
    }
    link_down(s);
}

/* ---- remote information ---- */

static void link_remote_info(BesBtssState *s, int side, uint16_t opcode,
                             const uint8_t *p, unsigned plen)
{
    uint8_t ev[13] = { 0x00 };
    int t = other(side);

    if (plen < 2 || !link_handle_is(s, p)) {
        btss_cmd_status(s, side, opcode, ERR_UNKNOWN_CONN);
        return;
    }
    btss_cmd_status(s, side, opcode, 0x00);
    stw_le_p(ev + 1, s->link.handle);
    switch (opcode) {
    case 0x041B:                    /* Read Remote Supported Features */
        btss_lmp_features(s, t, 0, ev + 3);
        btss_event(s, side, EVT_REMOTE_FEATURES, ev, 11);
        break;
    case 0x041C:                    /* Read Remote Extended Features */
        ev[3] = plen > 2 ? p[2] : 0;
        ev[4] = 2;                  /* max page */
        btss_lmp_features(s, t, ev[3], ev + 5);
        btss_event(s, side, EVT_REMOTE_EXT_FEATURES, ev, 13);
        break;
    case 0x041D:                    /* Read Remote Version Information */
        ev[3] = 0x0B;               /* LMP 5.2 */
        stw_le_p(ev + 4, BTSS_MANUFACTURER);
        btss_event(s, side, EVT_REMOTE_VERSION, ev, 8);
        break;
    case 0x041F:                    /* Read Clock Offset */
        btss_event(s, side, EVT_CLOCK_OFFSET, ev, 5);
        break;
    }
}

static void link_switch_role(BesBtssState *s, int side, const uint8_t *p,
                             unsigned plen)
{
    int i;

    if (plen < 7 || s->link.state != BTSS_LINK_UP ||
        !addr_is(s, other(side), p)) {
        btss_cmd_status(s, side, 0x080B, ERR_UNKNOWN_CONN);
        return;
    }
    btss_cmd_status(s, side, 0x080B, 0x00);
    s->link.central = p[6] == 0x00 ? side : other(side);
    for (i = 0; i < BTSS_SIDES; i++) {
        ev_role_change(s, i);
    }
}

static void link_sniff(BesBtssState *s, int side, uint16_t opcode,
                       const uint8_t *p, unsigned plen)
{
    int i;

    if (plen < 2 || !link_handle_is(s, p)) {
        btss_cmd_status(s, side, opcode, ERR_UNKNOWN_CONN);
        return;
    }
    btss_cmd_status(s, side, opcode, 0x00);
    for (i = 0; i < BTSS_SIDES; i++) {
        if (opcode == 0x0803 && plen >= 4) {
            ev_mode_change(s, i, 0x02, lduw_le_p(p + 2));   /* sniff */
        } else {
            ev_mode_change(s, i, 0x00, 0);                  /* active */
        }
    }
}

/* ---- authentication, Secure Simple Pairing, encryption ---- */

static void encryption_on(BesBtssState *s)
{
    uint8_t p[4] = { 0x00 };
    int i;

    /* AES-CCM when both hosts use Secure Connections, E0 otherwise */
    s->link.encrypted = s->host[0].sc && s->host[1].sc ? 0x02 : 0x01;
    stw_le_p(p + 1, s->link.handle);
    p[3] = s->link.encrypted;
    for (i = 0; i < BTSS_SIDES; i++) {
        btss_event(s, i, EVT_ENCRYPT_CHANGE, p, sizeof(p));
    }
}

static void auth_done(BesBtssState *s, uint8_t status)
{
    int init = s->link.sec_init;
    uint8_t p[4] = { status };

    trace_bes_btss_auth(init, status);
    s->link.sec = BTSS_SEC_IDLE;
    if (!status) {
        s->link.authenticated = true;
    }
    if (s->link.auth_requested) {
        s->link.auth_requested = false;
        ev_status_handle(s, init, EVT_AUTH_COMPLETE, status);
    }
    if (s->link.encrypt_pending) {
        s->link.encrypt_pending = false;
        if (!status) {
            encryption_on(s);
        } else {
            stw_le_p(p + 1, s->link.handle);
            btss_event(s, init, EVT_ENCRYPT_CHANGE, p, sizeof(p));
        }
    }
}

static void auth_start(BesBtssState *s, int init)
{
    s->link.sec_init = init;
    s->link.sec = BTSS_SEC_KEY_INIT;
    ev_addr(s, init, EVT_LINK_KEY_REQUEST);
}

static void pairing_start(BesBtssState *s)
{
    int init = s->link.sec_init;

    if (!s->host[0].ssp || !s->host[1].ssp) {
        qemu_log_mask(LOG_UNIMP, "bes.btss: legacy PIN pairing\n");
        auth_done(s, ERR_PAIRING_NOT_ALLOWED);
        return;
    }
    s->link.sec = BTSS_SEC_IO_INIT;
    ev_addr(s, init, EVT_IO_CAP_REQUEST);
}

static void pairing_done(BesBtssState *s, uint8_t status)
{
    uint8_t p[23];
    uint8_t key[16];
    int i;

    for (i = 0; i < BTSS_SIDES; i++) {
        p[0] = status;
        memcpy(p + 1, s->host[other(i)].bdaddr, 6);
        btss_event(s, i, EVT_SSP_COMPLETE, p, 7);
    }
    if (status) {
        auth_done(s, status);
        return;
    }
    qemu_guest_getrandom_nofail(key, sizeof(key));
    for (i = 0; i < BTSS_SIDES; i++) {
        memcpy(p, s->host[other(i)].bdaddr, 6);
        memcpy(p + 6, key, 16);
        /* (un)authenticated combination key, P-256 or P-192 */
        if (s->host[0].sc && s->host[1].sc) {
            p[22] = s->link.mitm ? 0x08 : 0x07;
        } else {
            p[22] = s->link.mitm ? 0x05 : 0x04;
        }
        btss_event(s, i, EVT_LINK_KEY_NOTIFY, p, sizeof(p));
    }
    auth_done(s, 0x00);
}

/*
 * Both IO capabilities known: pick the association model (Vol 3 Part C
 * 5.2.2.6) and ask each host for its part.
 */
static void pairing_associate(BesBtssState *s)
{
    BesBtssHost *h;
    uint8_t ev[10];
    uint32_t rnd;
    int a = s->host[0].io_cap, b = s->host[1].io_cap;
    int i;

    qemu_guest_getrandom_nofail(&rnd, sizeof(rnd));
    s->link.numeric = rnd % 1000000;
    s->link.sec = BTSS_SEC_CONFIRM;
    s->link.passkey_entry = a != IO_NO_IO && b != IO_NO_IO &&
                            (a == IO_KEYBOARD_ONLY || b == IO_KEYBOARD_ONLY);
    s->link.mitm = s->link.passkey_entry ||
                   (a == IO_DISPLAY_YES_NO && b == IO_DISPLAY_YES_NO);
    for (i = 0; i < BTSS_SIDES; i++) {
        h = &s->host[i];
        h->replied = false;
        memcpy(ev, s->host[other(i)].bdaddr, 6);
        if (!s->link.passkey_entry) {
            /* numeric comparison, or Just Works: the host confirms */
            stl_le_p(ev + 6, s->link.numeric);
            btss_event(s, i, EVT_USER_CONFIRM, ev, 10);
        } else if (h->io_cap == IO_KEYBOARD_ONLY) {
            btss_event(s, i, EVT_USER_PASSKEY, ev, 6);
        } else {
            /* the displaying side shows the passkey the other enters */
            h->replied = true;
            h->passkey = s->link.numeric;
            stl_le_p(ev + 6, s->link.numeric);
            btss_event(s, i, EVT_PASSKEY_NOTIFY, ev, 10);
        }
    }
}

static void pairing_check_replies(BesBtssState *s)
{
    if (!s->host[0].replied || !s->host[1].replied) {
        return;
    }
    if (s->link.passkey_entry && s->host[0].passkey != s->host[1].passkey) {
        pairing_done(s, ERR_AUTH_FAILURE);
    } else {
        pairing_done(s, 0x00);
    }
}

static void link_security(BesBtssState *s, int side, uint16_t opcode,
                          const uint8_t *p, unsigned plen)
{
    BesBtssHost *h = &s->host[side];
    int init = s->link.sec_init, resp = other(init);
    BesBtssSecState sec = s->link.sec;
    bool ok;
    uint8_t ev[9];

    if (plen < 6) {
        return;
    }
    /* the reply must answer the request this host has outstanding */
    switch (opcode) {
    case 0x040B: case 0x040C:
        ok = (sec == BTSS_SEC_KEY_INIT && side == init) ||
             (sec == BTSS_SEC_KEY_RESP && side == resp);
        break;
    case 0x042B: case 0x0434:
        ok = (sec == BTSS_SEC_IO_INIT && side == init) ||
             (sec == BTSS_SEC_IO_RESP && side == resp);
        break;
    default:
        ok = sec == BTSS_SEC_CONFIRM && !h->replied;
        break;
    }
    ok = ok && s->link.state == BTSS_LINK_UP && addr_is(s, other(side), p);
    cmd_complete_addr(s, side, opcode, ok ? 0x00 : ERR_CMD_DISALLOWED, p);
    if (!ok) {
        return;
    }

    switch (opcode) {
    case 0x040B:                    /* Link Key Request Reply */
        if (plen < 22) {
            break;
        }
        h->key_valid = true;
        memcpy(h->key, p + 6, 16);
        if (side == init) {
            s->link.sec = BTSS_SEC_KEY_RESP;
            ev_addr(s, resp, EVT_LINK_KEY_REQUEST);
        } else {
            auth_done(s, memcmp(s->host[init].key, h->key, 16) ?
                      ERR_AUTH_FAILURE : 0x00);
        }
        break;
    case 0x040C:                    /* Link Key Request Negative Reply */
        h->key_valid = false;
        if (side == init) {
            pairing_start(s);
        } else {
            auth_done(s, ERR_KEY_MISSING);
        }
        break;
    case 0x042B:                    /* IO Capability Request Reply */
        if (plen < 9) {
            break;
        }
        h->io_cap = p[6];
        h->oob = p[7];
        h->auth_req = p[8];
        memcpy(ev, h->bdaddr, 6);
        memcpy(ev + 6, p + 6, 3);
        btss_event(s, other(side), EVT_IO_CAP_RESPONSE, ev, 9);
        if (side == init) {
            s->link.sec = BTSS_SEC_IO_RESP;
            ev_addr(s, resp, EVT_IO_CAP_REQUEST);
        } else {
            pairing_associate(s);
        }
        break;
    case 0x0434:                    /* IO Capability Request Negative Reply */
        pairing_done(s, plen > 6 ? p[6] : ERR_PAIRING_NOT_ALLOWED);
        break;
    case 0x042C:                    /* User Confirmation Request Reply */
        if (s->link.passkey_entry) {
            pairing_done(s, ERR_AUTH_FAILURE);
            break;
        }
        h->replied = true;
        pairing_check_replies(s);
        break;
    case 0x042E:                    /* User Passkey Request Reply */
        if (!s->link.passkey_entry || plen < 10) {
            pairing_done(s, ERR_AUTH_FAILURE);
            break;
        }
        h->replied = true;
        h->passkey = ldl_le_p(p + 6);
        pairing_check_replies(s);
        break;
    case 0x042D:                    /* User Confirmation Negative Reply */
    case 0x042F:                    /* User Passkey Negative Reply */
        pairing_done(s, ERR_AUTH_FAILURE);
        break;
    }
}

static void link_auth_requested(BesBtssState *s, int side, const uint8_t *p,
                                unsigned plen)
{
    if (plen < 2 || !link_handle_is(s, p)) {
        btss_cmd_status(s, side, 0x0411, ERR_UNKNOWN_CONN);
        return;
    }
    if (s->link.sec != BTSS_SEC_IDLE) {
        btss_cmd_status(s, side, 0x0411, ERR_CMD_DISALLOWED);
        return;
    }
    btss_cmd_status(s, side, 0x0411, 0x00);
    s->link.auth_requested = true;
    auth_start(s, side);
}

static void link_set_encryption(BesBtssState *s, int side, const uint8_t *p,
                                unsigned plen)
{
    uint8_t ev[4] = { 0x00 };
    int i;

    if (plen < 3 || !link_handle_is(s, p)) {
        btss_cmd_status(s, side, 0x0413, ERR_UNKNOWN_CONN);
        return;
    }
    btss_cmd_status(s, side, 0x0413, 0x00);
    if (!p[2]) {
        s->link.encrypted = 0;
        stw_le_p(ev + 1, s->link.handle);
        for (i = 0; i < BTSS_SIDES; i++) {
            btss_event(s, i, EVT_ENCRYPT_CHANGE, ev, sizeof(ev));
        }
    } else if (s->link.authenticated) {
        encryption_on(s);
    } else {
        /* encryption needs the link key: authenticate first */
        s->link.encrypt_pending = true;
        if (s->link.sec == BTSS_SEC_IDLE) {
            auth_start(s, side);
        }
    }
}

/* ---- HCI commands ---- */

void btss_link_host_reset(BesBtssState *s, int side)
{
    BesBtssHost *h = &s->host[side];

    if (h->inq_timer) {
        timer_del(h->inq_timer);
    }
    if (s->link.state != BTSS_LINK_IDLE) {
        link_lost(s, side);
    }
    memset(h->cod, 0, sizeof(h->cod));
    memset(h->name, 0, sizeof(h->name));
    memset(h->eir, 0, sizeof(h->eir));
    h->scan_enable = 0;
    h->inquiry_mode = 0;
    h->iac = BTSS_GIAC;
    h->page_timeout = BTSS_PAGE_TIMEOUT_DFLT;
    h->acl_len = 0;
    h->ssp = h->sc = h->le = false;
    h->key_valid = h->replied = false;
}

/* commands that only configure the controller: record what the air sees */
static void link_record(BesBtssState *s, int side, uint16_t opcode,
                        const uint8_t *p, unsigned plen)
{
    BesBtssHost *h = &s->host[side];

    switch (opcode) {
    case 0x0C03:                    /* Reset */
        btss_link_host_reset(s, side);
        break;
    case 0x0C13:                    /* Write Local Name */
        memcpy(h->name, p, MIN(plen, 248u));
        break;
    case 0x0C18:                    /* Write Page Timeout */
        if (plen >= 2) {
            h->page_timeout = lduw_le_p(p);
        }
        break;
    case 0x0C1A:                    /* Write Scan Enable */
        if (plen >= 1) {
            h->scan_enable = p[0];
        }
        break;
    case 0x0C24:                    /* Write Class of Device */
        memcpy(h->cod, p, MIN(plen, 3u));
        break;
    case 0x0C33:                    /* Host Buffer Size */
        if (plen >= 2) {
            h->acl_len = lduw_le_p(p);
        }
        break;
    case 0x0C3A:                    /* Write Current IAC LAP (first) */
        if (plen >= 4 && p[0]) {
            h->iac = p[1] | p[2] << 8 | p[3] << 16;
        }
        break;
    case 0x0C45:                    /* Write Inquiry Mode */
        if (plen >= 1) {
            h->inquiry_mode = p[0];
        }
        break;
    case 0x0C52:                    /* Write Extended Inquiry Response */
        memcpy(h->eir, p, MIN(plen, 241u));
        break;
    case 0x0C56:                    /* Write Simple Pairing Mode */
        if (plen >= 1) {
            h->ssp = p[0];
        }
        break;
    case 0x0C6D:                    /* Write LE Host Support */
        if (plen >= 1) {
            h->le = p[0];
        }
        break;
    case 0x0C7A:                    /* Write Secure Connections Host Support */
        if (plen >= 1) {
            h->sc = p[0];
        }
        break;
    }
}

/*
 * Commands of controller "side" that involve the air interface. Returns
 * false for the ones the front end answers generically.
 */
bool btss_link_command(BesBtssState *s, int side, uint16_t opcode,
                       const uint8_t *p, unsigned plen)
{
    switch (opcode) {
    case 0x0401:
        link_inquiry(s, side, p, plen);
        return true;
    case 0x0402:
        link_inquiry_cancel(s, side);
        return true;
    case 0x0405:
        link_create_connection(s, side, p, plen);
        return true;
    case 0x0406:
        link_disconnect(s, side, p, plen);
        return true;
    case 0x0408:
        link_create_connection_cancel(s, side, p, plen);
        return true;
    case 0x0409:
        link_accept(s, side, p, plen);
        return true;
    case 0x040A:
        link_reject(s, side, p, plen);
        return true;
    case 0x040B: case 0x040C:
    case 0x042B: case 0x042C: case 0x042D: case 0x042E: case 0x042F:
    case 0x0434:
        link_security(s, side, opcode, p, plen);
        return true;
    case 0x0411:
        link_auth_requested(s, side, p, plen);
        return true;
    case 0x0413:
        link_set_encryption(s, side, p, plen);
        return true;
    case 0x0419:
        link_remote_name(s, side, p, plen);
        return true;
    case 0x041B: case 0x041C: case 0x041D: case 0x041F:
        link_remote_info(s, side, opcode, p, plen);
        return true;
    case 0x0803: case 0x0804:
        link_sniff(s, side, opcode, p, plen);
        return true;
    case 0x080B:
        link_switch_role(s, side, p, plen);
        return true;
    default:
        link_record(s, side, opcode, p, plen);
        return false;
    }
}

/* ---- ACL data ---- */

static void num_completed(BesBtssState *s, int side)
{
    uint8_t p[5] = { 1 };

    stw_le_p(p + 1, s->link.handle);
    stw_le_p(p + 3, 1);
    btss_event(s, side, EVT_NUM_COMPLETED, p, sizeof(p));
}

/* one H4 ACL packet from the host of controller "side" */
void btss_link_acl(BesBtssState *s, int side, const uint8_t *pkt,
                   unsigned len)
{
    uint8_t frag[BES_BTSS_MSG_MAX];
    uint16_t hf, dlen;
    unsigned max, off, n;

    if (len < 5) {
        return;
    }
    hf = lduw_le_p(pkt + 1);
    dlen = lduw_le_p(pkt + 3);
    trace_bes_btss_acl(side, hf, dlen);
    if (dlen != len - 5 || !link_handle_is(s, pkt + 1)) {
        qemu_log_mask(LOG_GUEST_ERROR, "bes.btss: side %d: ACL for handle "
                      "0x%03x (%u bytes) dropped\n", side, hf & 0x0FFF, dlen);
        return;
    }
    if (side == BTSS_BAND) {
        btss_link_peer_send(s, pkt, len);
    } else {
        /* the band's host takes fragments of its buffer size */
        max = BES_BTSS_MSG_MAX - 5;
        if (s->host[BTSS_BAND].acl_len) {
            max = MIN(max, s->host[BTSS_BAND].acl_len);
        }
        off = 0;
        do {
            n = MIN(max, dlen - off);
            frag[0] = H4_ACL;
            /* continuing fragments: Packet_Boundary_Flag 01 */
            stw_le_p(frag + 1, off ? (hf & 0x0FFF) | 0x1000 : hf);
            stw_le_p(frag + 3, n);
            memcpy(frag + 5, pkt + 5 + off, n);
            btss_band_acl(s, frag, 5 + n);
            off += n;
        } while (off < dlen);
    }
    num_completed(s, side);
}

/* ---- the peer's H4 transport ---- */

void btss_link_peer_send(BesBtssState *s, const uint8_t *pkt, unsigned len)
{
    if (s->radio_open) {
        qemu_chr_fe_write_all(&s->radio, pkt, len);
    }
}

/* run the complete packets received from the peer's host */
static void radio_process(BesBtssState *s)
{
    unsigned off = 0, avail, need;
    const uint8_t *p;

    while (off < s->rlen) {
        p = s->rbuf + off;
        avail = s->rlen - off;
        if (p[0] == H4_CMD && avail >= 4) {
            need = 4 + p[3];
        } else if (p[0] == H4_ACL && avail >= 5) {
            need = 5 + lduw_le_p(p + 3);
        } else if (p[0] == H4_CMD || p[0] == H4_ACL) {
            break;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "bes.btss: peer: H4 type 0x%02x "
                          "skipped\n", p[0]);
            off++;
            continue;
        }
        if (need > sizeof(s->rbuf)) {
            qemu_log_mask(LOG_GUEST_ERROR, "bes.btss: peer: %u-byte packet "
                          "exceeds the controller buffers\n", need);
            off++;
            continue;
        }
        /* held until the band's host has room for what it may raise */
        if (avail < need || btss_band_queue_free(s) < BTSS_BAND_RESERVE) {
            break;
        }
        if (p[0] == H4_CMD) {
            btss_hci_command(s, BTSS_PEER, p, need);
        } else {
            btss_link_acl(s, BTSS_PEER, p, need);
        }
        off += need;
    }
    memmove(s->rbuf, s->rbuf + off, s->rlen - off);
    s->rlen -= off;
}

static int radio_can_receive(void *opaque)
{
    BesBtssState *s = BES_BTSS(opaque);

    return sizeof(s->rbuf) - s->rlen;
}

static void radio_receive(void *opaque, const uint8_t *buf, int size)
{
    BesBtssState *s = BES_BTSS(opaque);

    size = MIN(size, (int)(sizeof(s->rbuf) - s->rlen));
    memcpy(s->rbuf + s->rlen, buf, size);
    s->rlen += size;
    radio_process(s);
}

static void radio_event(void *opaque, QEMUChrEvent event)
{
    BesBtssState *s = BES_BTSS(opaque);

    switch (event) {
    case CHR_EVENT_OPENED:
        trace_bes_btss_link("peer attached");
        s->radio_open = true;
        s->rlen = 0;
        btss_link_host_reset(s, BTSS_PEER);
        break;
    case CHR_EVENT_CLOSED:
        trace_bes_btss_link("peer detached");
        btss_link_host_reset(s, BTSS_PEER);
        s->radio_open = false;
        s->rlen = 0;
        break;
    default:
        break;
    }
}

void btss_link_band_drained(BesBtssState *s)
{
    if (s->rlen) {
        radio_process(s);
    }
    qemu_chr_fe_accept_input(&s->radio);
}

/* ---- device lifecycle ---- */

static bool parse_bdaddr(const char *str, uint8_t *addr)
{
    unsigned b[6];
    int i;

    if (sscanf(str, "%2x:%2x:%2x:%2x:%2x:%2x", &b[0], &b[1], &b[2], &b[3],
               &b[4], &b[5]) != 6) {
        return false;
    }
    for (i = 0; i < 6; i++) {
        addr[i] = b[5 - i];         /* HCI order: least significant first */
    }
    return true;
}

void btss_link_realize(BesBtssState *s, Error **errp)
{
    static const char *const dflt[BTSS_SIDES] = {
        [BTSS_BAND] = "C0:FF:EE:15:03:01",
        [BTSS_PEER] = "C0:FF:EE:00:00:01",
    };
    int i;

    for (i = 0; i < BTSS_SIDES; i++) {
        const char *str = s->bdaddr_str[i] ? s->bdaddr_str[i] : dflt[i];

        if (!parse_bdaddr(str, s->host[i].bdaddr)) {
            error_setg(errp, "bes.btss: bad BD_ADDR '%s'", str);
            return;
        }
    }
    s->host[BTSS_BAND].inq_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                                                inquiry_band_expired, s);
    s->host[BTSS_PEER].inq_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                                                inquiry_peer_expired, s);
    qemu_chr_fe_set_handlers(&s->radio, radio_can_receive, radio_receive,
                             radio_event, NULL, s, NULL, true);
}

void btss_link_reset(BesBtssState *s)
{
    btss_link_host_reset(s, BTSS_BAND);
    /* an attached peer keeps the configuration its host wrote */
    if (!s->radio_open) {
        btss_link_host_reset(s, BTSS_PEER);
    }
    link_down(s);
}
