/*
 * BES Bluetooth subsystem stand-in (best1503, best1700): inter-core
 * mailbox peer, HCI controller and a virtual air interface to one peer
 * device
 *
 * This code is licensed under the GPL version 2 or later.
 */

#ifndef HW_MISC_BES_BTSS_H
#define HW_MISC_BES_BTSS_H

#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "chardev/char-fe.h"

#define TYPE_BES_BTSS "bes.btss"
OBJECT_DECLARE_SIMPLE_TYPE(BesBtssState, BES_BTSS)

#define BES_BTSS_CHANNELS   2
#define BES_BTSS_MSG_MAX    1024
#define BES_BTSS_QUEUE_LEN  128
#define BES_BTSS_RADIO_BUF  4096

typedef struct BesBtssMsg {
    int64_t ready_us;                   /* published from this time on */
    uint16_t len;
    uint8_t data[BES_BTSS_MSG_MAX];
} BesBtssMsg;

typedef struct BesBtssChan {
    BesBtssMsg q[BES_BTSS_QUEUE_LEN];   /* BT -> MCU messages */
    unsigned q_head;
    unsigned q_count;
    unsigned inflight;                  /* messages published to the MCU */
    uint8_t mbx;                        /* mailbox id the MCU talks on */
} BesBtssChan;

/*
 * The virtual air interface joins two controllers: the band's own (its
 * host is the AP firmware, over the mailbox) and the peer's (its host is
 * whatever drives the "chardev" backend as an H4 HCI transport, e.g. a
 * phone-side Bluetooth stack). Link Manager procedures between the two
 * are resolved directly instead of being carried over a radio.
 */
enum {
    BTSS_BAND,
    BTSS_PEER,
    BTSS_SIDES,
};

/* per-controller state written by its host */
typedef struct BesBtssHost {
    uint8_t bdaddr[6];                  /* least significant octet first */
    uint8_t scan_enable;                /* bit 0 inquiry scan, bit 1 page scan */
    uint8_t cod[3];
    char name[249];
    uint8_t eir[241];                   /* FEC_Required + 240 octets */
    uint8_t inquiry_mode;
    uint32_t iac;                       /* LAP inquiry scan listens on */
    uint16_t page_timeout;
    uint16_t acl_len;                   /* Host Buffer Size; 0 = unstated */
    bool ssp;                           /* host supported features */
    bool sc;
    bool le;
    /* Secure Simple Pairing inputs of the procedure in progress */
    uint8_t io_cap;
    uint8_t oob;
    uint8_t auth_req;
    bool key_valid;
    uint8_t key[16];
    bool replied;
    uint32_t passkey;
    QEMUTimer *inq_timer;               /* Inquiry_Length expiry */
} BesBtssHost;

typedef enum {
    BTSS_LINK_IDLE,
    BTSS_LINK_PAGING,                   /* Connection Request outstanding */
    BTSS_LINK_UP,
} BesBtssLinkState;

typedef enum {
    BTSS_SEC_IDLE,
    BTSS_SEC_KEY_INIT,                  /* Link Key Request to the initiator */
    BTSS_SEC_KEY_RESP,                  /* ... and to the responder */
    BTSS_SEC_IO_INIT,                   /* IO Capability Request, initiator */
    BTSS_SEC_IO_RESP,                   /* ... and responder */
    BTSS_SEC_CONFIRM,                   /* user confirmation / passkey */
} BesBtssSecState;

typedef struct BesBtssLink {
    BesBtssLinkState state;
    int central;                        /* side in the central role */
    uint16_t handle;
    BesBtssSecState sec;
    int sec_init;                       /* side that started authentication */
    bool auth_requested;                /* by Authentication_Requested */
    bool encrypt_pending;               /* Set_Connection_Encryption waits */
    bool authenticated;
    uint8_t encrypted;                  /* Encryption_Enabled value */
    bool mitm;                          /* association model protects MITM */
    bool passkey_entry;
    uint32_t numeric;
} BesBtssLink;

struct BesBtssState {
    SysBusDevice parent_obj;
    MemoryRegion btcmu;                 /* BT_CMU register window */
    uint32_t btcmu_regs[0x400 / 4];

    MemoryRegion *downstream;
    AddressSpace as;
    uint32_t shared_table;              /* mailbox pointer table (BT RAM) */
    uint32_t exch_base;                 /* BT exchange memory */
    uint32_t exch_size;
    bool rx_head_direct;                /* [8 * ch] = first descriptor */
    uint8_t data_done_shift;            /* BT_CMU +0x54 done bit of ch 0 */
    uint16_t hci_revision;              /* controller firmware date */
    int32_t hci_channel;                /* only channel with HCI; -1: any */

    qemu_irq ind_ack[BES_BTSS_CHANNELS];    /* BT took MCU2BT data */
    qemu_irq bt2mcu[BES_BTSS_CHANNELS];     /* BT2MCU data indication */

    BesBtssChan chan[BES_BTSS_CHANNELS];
    uint32_t latency_us;                /* controller response latency */
    uint32_t reset_latency_us;          /* ... of HCI_Reset */
    uint32_t reply_us;                  /* latency of what is queued now */
    QEMUTimer *publish_timer;
    int hci_ch;                         /* channel the band's host uses */

    char *bdaddr_str[BTSS_SIDES];       /* "bdaddr", "peer-bdaddr" */
    BesBtssHost host[BTSS_SIDES];
    BesBtssLink link;

    CharFrontend radio;                 /* the peer's H4 transport */
    bool radio_open;
    int64_t air_free_ns;                /* end of the last ACL baseband packet */
    QEMUTimer *air_timer;               /* the channel is free again */
    uint8_t rbuf[BES_BTSS_RADIO_BUF];
    unsigned rlen;
};

#endif
