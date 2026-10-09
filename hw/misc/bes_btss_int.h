/*
 * BES Bluetooth subsystem stand-in: interfaces between the mailbox / HCI
 * front end (bes_btss.c) and the link layer (bes_btss_link.c)
 *
 * This code is licensed under the GPL version 2 or later.
 */

#ifndef HW_MISC_BES_BTSS_INT_H
#define HW_MISC_BES_BTSS_INT_H

#include "hw/misc/bes_btss.h"

#define H4_CMD                  0x01
#define H4_ACL                  0x02
#define H4_EVT                  0x04

#define HCI_EVT_CMD_COMPLETE    0x0E
#define HCI_EVT_CMD_STATUS      0x0F

/* front end (bes_btss.c) */
void btss_event(BesBtssState *s, int side, uint8_t code,
                const uint8_t *params, unsigned plen);
void btss_cmd_status(BesBtssState *s, int side, uint16_t opcode,
                     uint8_t status);
void btss_cmd_complete(BesBtssState *s, int side, uint16_t opcode,
                       const uint8_t *rp, unsigned rp_len);
void btss_hci_command(BesBtssState *s, int side, const uint8_t *cmd,
                      unsigned len);
bool btss_band_acl(BesBtssState *s, const uint8_t *pkt, unsigned len);
unsigned btss_band_queue_free(BesBtssState *s);
void btss_lmp_features(BesBtssState *s, int side, unsigned page,
                       uint8_t *out);

/* link layer (bes_btss_link.c) */
bool btss_link_command(BesBtssState *s, int side, uint16_t opcode,
                       const uint8_t *p, unsigned plen);
void btss_link_acl(BesBtssState *s, int side, const uint8_t *pkt,
                   unsigned len);
void btss_link_peer_send(BesBtssState *s, const uint8_t *pkt, unsigned len);
void btss_link_band_drained(BesBtssState *s);
void btss_link_host_reset(BesBtssState *s, int side);
void btss_link_realize(BesBtssState *s, Error **errp);
void btss_link_reset(BesBtssState *s);

#endif
