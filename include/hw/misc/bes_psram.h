/*
 * BES PSRAM controller pair (best1503): PHY + memory controller
 *
 * This code is licensed under the GPL version 2 or later.
 */

#ifndef HW_MISC_BES_PSRAM_H
#define HW_MISC_BES_PSRAM_H

#include "hw/core/sysbus.h"

#define TYPE_BES_PSRAM "bes.psram"
OBJECT_DECLARE_SIMPLE_TYPE(BesPsramState, BES_PSRAM)

struct BesPsramState {
    SysBusDevice parent_obj;
    MemoryRegion iomem_phy;     /* PHY window @0x40168000 (0x400) */
    MemoryRegion iomem_cmd;     /* MC window @0x40160000 (0x1000) */

    uint32_t phy_regs[0x400 / 4];
    uint32_t cmd_regs[0x1000 / 4];

    /* PSRAM storage (physical offset 0), set by the machine */
    uint64_t psram;             /* host pointer to the PSRAM storage */
    uint32_t psram_size;

    /* MGR command engine FIFOs (reg_psram_mc_v4.h) */
    uint32_t mgr_rxfifo[16];
    uint32_t mgr_rx_cnt;
    uint32_t mgr_rx_pos;
    uint32_t mgr_txfifo[16];
    uint32_t mgr_tx_cnt;
    uint8_t mr[32];             /* mode registers */
};

#endif
