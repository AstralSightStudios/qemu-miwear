/*
 * BES dual-mode controller: LE initiating and link-layer procedures.
 *
 * This is the controller side of the existing two-host virtual air model.
 * It matches actual enabled advertising sets and carries opaque ACL packets;
 * no host L2CAP, ATT, CoC, or application protocol is implemented here.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "bes_btss_int.h"

#define LE_HANDLE               2
#define ADV_CONNECTABLE         (1 << 0)
#define ADV_DIRECTED            (1 << 2)
#define ADV_LEGACY              (1 << 4)
#define UNKNOWN_CONN            0x02
#define CONNECTION_EXISTS       0x0B
#define COMMAND_DISALLOWED      0x0C
#define UNSUPPORTED_FEATURE     0x11
#define INVALID_PARAMS          0x12
#define LOCAL_HOST_TERM         0x16
#define CONNECTION_TIMEOUT      0x08
#define ACCEPT_LIST_FULL        0x07

bool btss_le_handle(BesBtssState *s, uint16_t handle)
{
    return s->le_link.up && (handle & 0x0fff) == s->le_link.handle;
}

void btss_le_event(BesBtssState *s, int side, const uint8_t *p, unsigned len)
{
    BesBtssHost *h = &s->host[side];

    if (len && p[0] <= 64 && p[0] &&
        (h->event_mask & (UINT64_C(1) << 61)) &&
        (h->le_event_mask & (UINT64_C(1) << (p[0] - 1)))) {
        btss_event(s, side, 0x3e, p, len);
    }
}

static void le_complete(BesBtssState *s, int side, uint8_t status)
{
    BesBtssLeLink *link = &s->le_link;
    BesBtssHost *h = &s->host[side];
    uint8_t p[31] = { 0 };
    bool enhanced = h->le_event_mask & (UINT64_C(1) << 9);
    unsigned params = enhanced ? 24 : 12;

    p[0] = enhanced ? 0x0a : 0x01;
    p[1] = status;
    if (!status) {
        stw_le_p(p + 2, link->handle);
        p[4] = link->central == side ? 0 : 1;
        p[5] = link->addr_type[side ^ 1];
        memcpy(p + 6, link->addr[side ^ 1], 6);
        /* No resolving list: RPAs in Enhanced Connection Complete are zero. */
        stw_le_p(p + params, link->interval);
        stw_le_p(p + params + 2, link->latency);
        stw_le_p(p + params + 4, link->timeout);
        p[params + 6] = 0;          /* Central Clock Accuracy: 500 ppm */
    }
    btss_le_event(s, side, p, params + 7);
}

static bool accept_contains(BesBtssHost *h, uint8_t type,
                            const uint8_t addr[6])
{
    unsigned i;

    for (i = 0; i < ARRAY_SIZE(h->accept); i++) {
        if (h->accept[i].used && h->accept[i].type == type &&
            !memcmp(h->accept[i].addr, addr, 6)) {
            return true;
        }
    }
    return false;
}

static const uint8_t *adv_address(BesBtssHost *h, BesBtssAdvSet *a)
{
    if (!(a->own_addr_type & 1)) {
        return h->bdaddr;
    }
    return a->handle == BTSS_ADV_LEGACY ? h->le_random : a->random;
}

static void le_check(void *opaque)
{
    BesBtssState *s = opaque;
    unsigned side, i;

    if (s->le_link.up || !s->radio_open) {
        return;
    }
    for (side = 0; side < BTSS_SIDES; side++) {
        BesBtssHost *h = &s->host[side];
        BesBtssHost *target = &s->host[side ^ 1];
        BesBtssLeInitiator *init = &h->initiating;
        const uint8_t *local = (init->own_type & 1) ? h->le_random : h->bdaddr;

        if (!init->pending) {
            continue;
        }
        for (i = 0; i < BTSS_ADV_SETS; i++) {
            BesBtssAdvSet *a = &target->adv[i];
            const uint8_t *remote = adv_address(target, a);
            uint8_t remote_type = a->own_addr_type & 1;
            uint8_t terminated[6] = { 0x12, 0x00, a->handle, 0, 0, 0 };

            if (!a->used || !a->enabled || !(a->props & ADV_CONNECTABLE) ||
                (remote_type && !(a->handle == BTSS_ADV_LEGACY ?
                                  target->le_random_set : a->random_set))) {
                continue;
            }
            if (init->filter_policy ? !accept_contains(h, remote_type, remote) :
                (init->peer_type != remote_type || memcmp(init->peer, remote, 6))) {
                continue;
            }
            if ((a->props & ADV_DIRECTED) &&
                (a->direct_addr_type != (init->own_type & 1) ||
                 memcmp(a->direct_addr, local, 6))) {
                continue;
            }
            if ((a->filter_policy & 2) &&
                !accept_contains(target, init->own_type & 1, local)) {
                continue;
            }
            s->le_link = (BesBtssLeLink) {
                .up = true, .central = side, .handle = LE_HANDLE,
                .interval = init->interval, .latency = init->latency,
                .timeout = init->timeout,
            };
            s->le_link.addr_type[side] = init->own_type & 1;
            s->le_link.addr_type[side ^ 1] = remote_type;
            memcpy(s->le_link.addr[side], local, 6);
            memcpy(s->le_link.addr[side ^ 1], remote, 6);
            init->pending = false;
            a->enabled = false;    /* Only the set that accepted CONNECT_IND. */
            le_complete(s, side, 0);
            le_complete(s, side ^ 1, 0);
            if (a->handle != BTSS_ADV_LEGACY) {
                stw_le_p(terminated + 3, LE_HANDLE);
                btss_le_event(s, side ^ 1, terminated, sizeof(terminated));
            }
            return;
        }
    }
}

void btss_le_changed(BesBtssState *s)
{
    /* Completion follows Command Status/Complete, never inside record(). */
    timer_mod(s->le_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000);
}

static bool connection_params(const uint8_t *p)
{
    uint16_t scan_interval = lduw_le_p(p);
    uint16_t scan_window = lduw_le_p(p + 2);
    uint16_t min = lduw_le_p(p + 4), max = lduw_le_p(p + 6);
    uint16_t latency = lduw_le_p(p + 8), timeout = lduw_le_p(p + 10);

    return scan_interval >= 4 && scan_interval <= 0x4000 && scan_window >= 4 &&
           scan_window <= scan_interval && min >= 6 && min <= max &&
           max <= 0x0c80 && latency <= 0x01f3 && timeout >= 0x0a &&
           timeout <= 0x0c80 &&
           (uint32_t)timeout * 4 > (uint32_t)(latency + 1) * max &&
           lduw_le_p(p + 12) <= lduw_le_p(p + 14);
}

static void le_create(BesBtssState *s, int side, uint16_t opcode,
                       const uint8_t *p, unsigned plen)
{
    BesBtssHost *h = &s->host[side];
    BesBtssLeInitiator init = { .pending = true };
    uint8_t block[16];
    unsigned i, count = 0;

    if (h->initiating.pending) {
        btss_cmd_status(s, side, opcode, COMMAND_DISALLOWED);
        return;
    }
    if (s->le_link.up) {
        btss_cmd_status(s, side, opcode, CONNECTION_EXISTS);
        return;
    }
    if (opcode == 0x200d) {
        if (plen != 25) {
            goto invalid;
        }
        init.filter_policy = p[4];
        init.peer_type = p[5];
        memcpy(init.peer, p + 6, 6);
        init.own_type = p[12];
        memcpy(block, p, 4);
        memcpy(block + 4, p + 13, 12);
        if (!connection_params(block)) {
            goto invalid;
        }
    } else {
        if (plen < 10 || !p[9] || (p[9] & ~0x07)) {
            goto invalid;
        }
        for (i = 0; i < 3; i++) {
            count += !!(p[9] & (1 << i));
        }
        if (plen != 10 + 16 * count) {
            goto invalid;
        }
        /* The air model supports LE 1M only. Do not claim 2M/Coded. */
        if (p[9] != 1) {
            btss_cmd_status(s, side, opcode, UNSUPPORTED_FEATURE);
            return;
        }
        init.filter_policy = p[0];
        init.own_type = p[1];
        init.peer_type = p[2];
        memcpy(init.peer, p + 3, 6);
        memcpy(block, p + 10, 16);
        if (!connection_params(block)) {
            goto invalid;
        }
    }
    if (init.filter_policy > 1 || init.peer_type > 1 || init.own_type > 3) {
        goto invalid;
    }
    if ((init.own_type & 1) && !h->le_random_set) {
        btss_cmd_status(s, side, opcode, INVALID_PARAMS);
        return;
    }
    init.interval = lduw_le_p(block + 4);
    init.latency = lduw_le_p(block + 8);
    init.timeout = lduw_le_p(block + 10);
    h->initiating = init;
    btss_cmd_status(s, side, opcode, 0);
    btss_le_changed(s);
    return;
invalid:
    btss_cmd_status(s, side, opcode, INVALID_PARAMS);
}

static void le_cancel(BesBtssState *s, int side, unsigned plen)
{
    uint8_t status = plen ? INVALID_PARAMS :
                     !s->host[side].initiating.pending ? COMMAND_DISALLOWED : 0;

    btss_cmd_complete(s, side, 0x200e, &status, 1);
    if (!status) {
        s->host[side].initiating.pending = false;
        le_complete(s, side, UNKNOWN_CONN);
    }
}

static void le_disconnect_event(BesBtssState *s, int side, uint8_t reason)
{
    uint8_t p[4] = { 0, 0, 0, reason };

    stw_le_p(p + 1, s->le_link.handle);
    btss_event(s, side, 0x05, p, sizeof(p));
}

static void le_update(BesBtssState *s, int side, const uint8_t *p, unsigned plen)
{
    uint8_t block[16] = { 4, 0, 4, 0 };
    uint8_t ev[10] = { 0x03, 0 };
    unsigned i;

    if (plen != 14) {
        btss_cmd_status(s, side, 0x2013, INVALID_PARAMS);
        return;
    }
    if (!btss_le_handle(s, lduw_le_p(p))) {
        btss_cmd_status(s, side, 0x2013, UNKNOWN_CONN);
        return;
    }
    memcpy(block + 4, p + 2, 12);
    if (!connection_params(block)) {
        btss_cmd_status(s, side, 0x2013, INVALID_PARAMS);
        return;
    }
    btss_cmd_status(s, side, 0x2013, 0);
    s->le_link.interval = lduw_le_p(p + 2);
    s->le_link.latency = lduw_le_p(p + 6);
    s->le_link.timeout = lduw_le_p(p + 8);
    stw_le_p(ev + 2, s->le_link.handle);
    stw_le_p(ev + 4, s->le_link.interval);
    stw_le_p(ev + 6, s->le_link.latency);
    stw_le_p(ev + 8, s->le_link.timeout);
    for (i = 0; i < BTSS_SIDES; i++) {
        btss_le_event(s, i, ev, sizeof(ev));
    }
}

static void le_accept_list(BesBtssState *s, int side, uint16_t opcode,
                           const uint8_t *p, unsigned plen)
{
    BesBtssHost *h = &s->host[side];
    uint8_t status = 0;
    unsigned i;
    int free = -1;

    if (h->initiating.pending) {
        status = COMMAND_DISALLOWED;
    } else if (opcode == 0x2010) {
        if (plen) {
            status = INVALID_PARAMS;
        } else {
            memset(h->accept, 0, sizeof(h->accept));
        }
    } else if (plen != 7 || p[0] > 1) {
        status = INVALID_PARAMS;
    } else {
        for (i = 0; i < ARRAY_SIZE(h->accept); i++) {
            if (h->accept[i].used && h->accept[i].type == p[0] &&
                !memcmp(h->accept[i].addr, p + 1, 6)) {
                if (opcode == 0x2012) {
                    h->accept[i].used = false;
                }
                goto done;
            }
            if (!h->accept[i].used) {
                free = i;
            }
        }
        if (opcode == 0x2011) {
            if (free < 0) {
                status = ACCEPT_LIST_FULL;
            } else {
                h->accept[free].used = true;
                h->accept[free].type = p[0];
                memcpy(h->accept[free].addr, p + 1, 6);
            }
        } else {
            status = INVALID_PARAMS;
        }
    }
done:
    btss_cmd_complete(s, side, opcode, &status, 1);
}

bool btss_le_command(BesBtssState *s, int side, uint16_t opcode,
                     const uint8_t *p, unsigned plen)
{
    BesBtssHost *h = &s->host[side];
    uint8_t status = 0, ev[12] = { 0 };
    bool valid;

    switch (opcode) {
    case 0x0c01:                    /* Set Event Mask */
    case 0x2001:                    /* LE Set Event Mask */
        status = plen == 8 ? 0 : INVALID_PARAMS;
        if (!status) {
            if (opcode == 0x0c01) {
                h->event_mask = ldq_le_p(p);
            } else {
                h->le_event_mask = ldq_le_p(p);
            }
        }
        btss_cmd_complete(s, side, opcode, &status, 1);
        return true;
    case 0x200d: case 0x2043:
        le_create(s, side, opcode, p, plen);
        return true;
    case 0x200e:
        le_cancel(s, side, plen);
        return true;
    case 0x2010: case 0x2011: case 0x2012:
        le_accept_list(s, side, opcode, p, plen);
        return true;
    case 0x2013:
        le_update(s, side, p, plen);
        return true;
    case 0x2016:                    /* LE Read Remote Features */
        valid = plen == 2 && btss_le_handle(s, lduw_le_p(p));
        btss_cmd_status(s, side, opcode,
                        plen != 2 ? INVALID_PARAMS : valid ? 0 : UNKNOWN_CONN);
        if (valid) {
            ev[0] = 0x04;
            stw_le_p(ev + 2, s->le_link.handle);
            /* The implemented LE 1M unencrypted ACL has no optional LL features. */
            btss_le_event(s, side, ev, sizeof(ev));
        }
        return true;
    case 0x041d:                    /* Universal Read Remote Version */
        if (plen != 2 || !btss_le_handle(s, lduw_le_p(p))) {
            return false;          /* Classic implementation handles the rest. */
        }
        btss_cmd_status(s, side, opcode, 0);
        ev[3] = 0x0b;
        stw_le_p(ev + 1, s->le_link.handle);
        stw_le_p(ev + 4, 0x02b0);
        btss_event(s, side, 0x0c, ev, 8);
        return true;
    case 0x0406:
        if (plen != 3 || !btss_le_handle(s, lduw_le_p(p))) {
            return false;
        }
        /* HCI Disconnect accepts the listed reason codes, not any octet. */
        valid = p[2] == 0x05 || p[2] == 0x13 || p[2] == 0x14 ||
                p[2] == 0x15 || p[2] == 0x1a || p[2] == 0x29 || p[2] == 0x3b;
        btss_cmd_status(s, side, opcode, valid ? 0 : INVALID_PARAMS);
        if (valid) {
            le_disconnect_event(s, side, LOCAL_HOST_TERM);
            le_disconnect_event(s, side ^ 1, p[2]);
            memset(&s->le_link, 0, sizeof(s->le_link));
        }
        return true;
    case 0x2019:                    /* LE Start Encryption: no LL encryption model */
    case 0x2032:                    /* LE Set PHY: only 1M is modelled */
        btss_cmd_status(s, side, opcode, UNSUPPORTED_FEATURE);
        return true;
    default:
        return false;
    }
}

void btss_le_host_reset(BesBtssState *s, int side)
{
    BesBtssHost *h = &s->host[side];

    h->initiating.pending = false;
    if (s->le_link.up) {
        le_disconnect_event(s, side ^ 1, CONNECTION_TIMEOUT);
        memset(&s->le_link, 0, sizeof(s->le_link));
    }
    h->event_mask = UINT64_C(0x00001fffffffffff);
    h->le_event_mask = 0x1f;
    h->le_random_set = false;
    memset(h->accept, 0, sizeof(h->accept));
}

void btss_le_realize(BesBtssState *s)
{
    s->le_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, le_check, s);
}
