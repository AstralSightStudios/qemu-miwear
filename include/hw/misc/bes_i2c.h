/*
 * BES I2C master controller (DesignWare APb-I2C compatible) for best1503.
 *
 * This code is licensed under the GPL version 2 or later.
 */
#ifndef HW_MISC_BES_I2C_H
#define HW_MISC_BES_I2C_H

#include "hw/core/sysbus.h"

#define TYPE_BES_I2C "bes.i2c"
OBJECT_DECLARE_SIMPLE_TYPE(BesI2cState, BES_I2C)

#endif
