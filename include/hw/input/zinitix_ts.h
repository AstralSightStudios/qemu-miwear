/*
 * Zinitix ZTW6xx capacitive touch controller (I2C)
 *
 * This code is licensed under the GPL version 2 or later.
 */

#ifndef HW_INPUT_ZINITIX_TS_H
#define HW_INPUT_ZINITIX_TS_H

#include "hw/i2c/i2c.h"

#define TYPE_ZINITIX_TS "zinitix.ts"
OBJECT_DECLARE_SIMPLE_TYPE(ZinitixTsState, ZINITIX_TS)

#endif
