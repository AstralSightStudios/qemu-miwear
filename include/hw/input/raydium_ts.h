/*
 * Raydium RM6D03x capacitive touch controller (I2C)
 *
 * This code is licensed under the GPL version 2 or later.
 */

#ifndef HW_INPUT_RAYDIUM_TS_H
#define HW_INPUT_RAYDIUM_TS_H

#include "hw/i2c/i2c.h"
#include "qom/object.h"

#define TYPE_RAYDIUM_TS "raydium-ts"
OBJECT_DECLARE_SIMPLE_TYPE(RaydiumTsState, RAYDIUM_TS)

#endif
