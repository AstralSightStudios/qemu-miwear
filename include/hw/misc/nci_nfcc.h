/*
 * NFC controller speaking NCI over I2C
 *
 * This code is licensed under the GPL version 2 or later.
 */

#ifndef HW_MISC_NCI_NFCC_H
#define HW_MISC_NCI_NFCC_H

#include "hw/i2c/i2c.h"

#define TYPE_NCI_NFCC "nci.nfcc"
OBJECT_DECLARE_SIMPLE_TYPE(NciNfccState, NCI_NFCC)

#endif
