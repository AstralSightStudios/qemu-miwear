/*
 * BES best1700 clock units: MCU_CMU and AON_CMU
 *
 * This code is licensed under the GPL version 2 or later.
 */

#ifndef HW_MISC_BEST1700_CMU_H
#define HW_MISC_BEST1700_CMU_H

#include "hw/core/sysbus.h"

#define TYPE_BEST1700_CMU       "best1700.cmu"
#define TYPE_BEST1700_AONCMU    "best1700.aoncmu"
#define TYPE_BEST1700_BTHCMU    "best1700.bthcmu"

typedef struct Best1700CmuInfo Best1700CmuInfo;

OBJECT_DECLARE_TYPE(Best1700CmuState, Best1700CmuClass, BEST1700_CMU)

#define BEST1700_CMU_SIZE       0x1000
#define BEST1700_CMU_MAX_PAIRS  16

/* intersys mailbox groups: M55C0 channels 0/1, M55C1 channels 0/1, ... */
#define BEST1700_ISYS_GROUPS    8

/* MCU_CMU mailbox channels to the Bluetooth core */
#define BEST1700_BT_ISYS_CHANNELS   4

/* AON_CMU: boot address of the M55C1 core, bits [31:7] (its own view) */
#define BEST1700_AONCMU_M55C1_VTOR  0xB4

struct Best1700CmuState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    uint32_t regs[BEST1700_CMU_SIZE / 4];
    uint32_t pair_state[BEST1700_CMU_MAX_PAIRS];
    uint32_t memsc_taken;       /* bit n: hardware semaphore n is held */
    int64_t timer_offset;       /* free-running counter = ticks + offset */

    qemu_irq ipc[32];           /* MCU_CMU: one line per inter-core bit */
    qemu_irq cpu_run;           /* AON_CMU: M55C1 released from hold */
    bool cpu_held;              /* hold bit seen set since reset */

    /* intersys mailbox (MCU_CMU and BTH_CMU) */
    uint32_t isys_word;         /* data to the peer 16+g, peer done with it g */
    uint32_t isys_mask;         /* rx enable g, tx done enable 16+g */
    Best1700CmuState *isys_peer;
    qemu_irq isys_rx[BEST1700_ISYS_GROUPS];
    qemu_irq isys_tx[BEST1700_ISYS_GROUPS];

    /* MCU_CMU: TrustZone trigger between the M55C0 security states */
    uint32_t tz_word;
    qemu_irq tz_trigger[32];

    /* MCU_CMU: mailbox to the Bluetooth core */
    uint32_t bt_word;           /* indications 4+ch, enables 8+ch / 12+ch */
    uint32_t bt_raw;            /* BT data 24+ch, BT took MCU data 28+ch */
    qemu_irq bt_doorbell[BEST1700_BT_ISYS_CHANNELS];
    qemu_irq bt_irq[2 * BEST1700_BT_ISYS_CHANNELS];    /* tx done, rx */
};

/* pair the intersys mailbox halves of the MCU_CMU and the BTH_CMU */
void best1700_cmu_link_intersys(Best1700CmuState *mcu, Best1700CmuState *bth);

struct Best1700CmuClass {
    SysBusDeviceClass parent_class;

    const Best1700CmuInfo *info;
};

#endif
