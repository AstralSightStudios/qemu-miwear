/*
 * BES Bluetooth subsystem stand-in (best1503): inter-core mailbox peer
 * and a minimal HCI controller
 *
 * This code is licensed under the GPL version 2 or later.
 */

#ifndef HW_MISC_BES_BTSS_H
#define HW_MISC_BES_BTSS_H

#include "hw/core/sysbus.h"
#include "qemu/timer.h"

#define TYPE_BES_BTSS "bes.btss"
OBJECT_DECLARE_SIMPLE_TYPE(BesBtssState, BES_BTSS)

#define BES_BTSS_CHANNELS   2
#define BES_BTSS_MSG_MAX    1024
#define BES_BTSS_QUEUE_LEN  32

typedef struct BesBtssMsg {
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

struct BesBtssState {
    SysBusDevice parent_obj;
    MemoryRegion btcmu;                 /* BT_CMU register window */
    uint32_t btcmu_regs[0x400 / 4];

    MemoryRegion *downstream;
    AddressSpace as;
    uint32_t shared_table;              /* mailbox pointer table (BT RAM) */
    uint32_t exch_base;                 /* BT exchange memory */
    uint32_t exch_size;

    qemu_irq ind_ack[BES_BTSS_CHANNELS];    /* BT took MCU2BT data */
    qemu_irq bt2mcu[BES_BTSS_CHANNELS];     /* BT2MCU data indication */

    BesBtssChan chan[BES_BTSS_CHANNELS];
    uint8_t bdaddr[6];
    uint32_t latency_us;                /* controller response latency */
    QEMUTimer *publish_timer;
};

#endif
