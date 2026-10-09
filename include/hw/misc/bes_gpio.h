/*
 * BES GPIO controller v2 (best1503) with the PSC GPIO interrupt block
 *
 * This code is licensed under the GPL version 2 or later.
 */

#ifndef HW_MISC_BES_GPIO_H
#define HW_MISC_BES_GPIO_H

#include "hw/core/sysbus.h"

#define TYPE_BES_GPIO "bes.gpio"
OBJECT_DECLARE_SIMPLE_TYPE(BesGpioState, BES_GPIO)

#define BES_GPIO_BANKS      3
#define BES_GPIO_PINS       (BES_GPIO_BANKS * 32)

typedef struct BesGpioBank {
    uint32_t dr;            /* output data */
    uint32_t ddr;           /* direction, 1 = output */
    uint32_t inten;
    uint32_t intmask;
    uint32_t inttype_level; /* 1 = edge sensitive */
    uint32_t polarity;      /* 1 = rising edge / active high */
    uint32_t debounce;
    uint32_t raw;           /* latched edge / level interrupt status */
    uint32_t ext;           /* externally driven input levels */
} BesGpioBank;

struct BesGpioState {
    SysBusDevice parent_obj;
    MemoryRegion bank_mr[BES_GPIO_BANKS];
    MemoryRegion psc_mr;
    qemu_irq irq;
    qemu_irq out[BES_GPIO_PINS];

    BesGpioBank bank[BES_GPIO_BANKS];
    uint32_t psc_en[BES_GPIO_BANKS];
};

#endif
