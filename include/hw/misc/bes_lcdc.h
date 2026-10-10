/*
 * BES LCDC (LCD controller) for best1503.
 *
 * This code is licensed under the GPL version 2 or later.
 */
#ifndef HW_MISC_BES_LCDC_H
#define HW_MISC_BES_LCDC_H

#include "hw/core/sysbus.h"

#define TYPE_BES_LCDC "bes.lcdc"
OBJECT_DECLARE_SIMPLE_TYPE(BesLcdcState, BES_LCDC)

/* a MIPI DCS write sent to the panel by a DSI host: command + parameters */
void bes_lcdc_panel_dcs(DeviceState *lcdc, uint8_t dcs, const uint8_t *param,
                        unsigned n);

#endif
