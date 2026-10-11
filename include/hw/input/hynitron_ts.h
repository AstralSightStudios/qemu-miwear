/*
 * Hynitron CST92xx capacitive touch controller (I2C)
 *
 * This code is licensed under the GPL version 2 or later.
 */

#ifndef HW_INPUT_HYNITRON_TS_H
#define HW_INPUT_HYNITRON_TS_H

#include "hw/i2c/i2c.h"

#define TYPE_HYNITRON_TS "hynitron.ts"
OBJECT_DECLARE_SIMPLE_TYPE(HynitronTsState, HYNITRON_TS)

#endif
