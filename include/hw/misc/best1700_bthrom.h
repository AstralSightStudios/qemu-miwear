/*
 * BES best1700 BTH core boot ROM stand-in
 *
 * This code is licensed under the GPL version 2 or later.
 */

#ifndef HW_MISC_BEST1700_BTHROM_H
#define HW_MISC_BEST1700_BTHROM_H

#include "hw/core/sysbus.h"
#include "system/memory.h"

#define TYPE_BEST1700_BTHROM "best1700.bthrom"
OBJECT_DECLARE_SIMPLE_TYPE(Best1700BthRomState, BEST1700_BTHROM)

struct Best1700BthRomState {
    SysBusDevice parent_obj;
    MemoryRegion rom;           /* mmio 0: the ROM, at rom-base */
    MemoryRegion hc;            /* mmio 1: hypercall window, at hc-base */

    MemoryRegion *memory;       /* the BTH core's view, for arguments */
    AddressSpace as;
    uint32_t rom_base;
    uint32_t hc_base;
    uint32_t flash_size;

    uint32_t args;
    uint32_t ret[2];
    GString *line;              /* trace output not yet ended by '\n' */
};

#endif
