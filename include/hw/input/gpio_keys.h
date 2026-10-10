/*
 * GPIO keys: host keys pressing a board's push buttons
 *
 * This code is licensed under the GPL version 2 or later.
 */

#ifndef HW_INPUT_GPIO_KEYS_H
#define HW_INPUT_GPIO_KEYS_H

#include "hw/core/qdev.h"
#include "ui/input.h"

#define TYPE_GPIO_KEYS "gpio-keys"
OBJECT_DECLARE_SIMPLE_TYPE(GpioKeysState, GPIO_KEYS)

struct GpioKeysState {
    DeviceState parent_obj;

    uint32_t num_keys;
    uint32_t *qcodes;           /* QKeyCode of the host key behind key n */
    uint32_t active_low;        /* bit n: key n pulls its line low */
    bool *pressed;
    qemu_irq *out;              /* "key" n */
    QemuInputHandlerState *hs;
};

#endif
