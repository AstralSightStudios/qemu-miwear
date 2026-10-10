/*
 * Xiaomi Vela devices on BES chips: SoCs, board (device) profiles and the
 * machines that join them
 *
 * A device is a chip plus a board. The chip (best1503, best1700) fixes the
 * CPU, the memory map and the on-chip peripherals; the board fixes what is
 * soldered around it -- panel, touch and NFC controllers, fuel gauge, the
 * SPI NAND, push buttons -- and how it is wired (I2C bus and address, GPIO
 * lines).
 * Devices sharing a chip run different firmware and differ only in their
 * board profile.
 *
 * The firmware names its chip in the boot info (CHIP=) and its product in
 * the image ("miwear.watch.<code>..."), so both are normally found from
 * -kernel alone; "profile=<name>" picks a board explicitly.
 *
 * This code is licensed under the GPL version 2 or later.
 */

#ifndef HW_ARM_BES_BOARD_H
#define HW_ARM_BES_BOARD_H

#include "hw/core/boards.h"
#include "hw/misc/bes_image.h"
#include "qapi/qapi-types-ui.h"

#define BES_BOARD_MAX_KEYS      8

typedef struct BesBoardKey {
    const char *name;
    QKeyCode qcode;             /* host key that presses it */
    int gpio;                   /* SoC GPIO line it drives */
    bool active_low;
} BesBoardKey;

/* an I2C peripheral on the board; type NULL = not fitted */
typedef struct BesBoardI2C {
    const char *type;
    int bus;                    /* SoC I2C controller index */
    uint8_t addr;
    int irq_gpio;               /* interrupt / data-ready line, -1 = none */
} BesBoardI2C;

typedef struct BesBoard {
    const char *name;           /* profile=<name> */
    const char *desc;
    const char *chip;           /* the firmware's CHIP= */
    const char *product;        /* the firmware's "miwear.watch.<product>" */

    /* display panel; 0 x 0 = no display modelled */
    uint32_t panel_width;
    uint32_t panel_height;
    int lcd_te_gpio;            /* tearing-effect line, -1 = none */

    BesBoardI2C touch;
    BesBoardI2C nfc;
    BesBoardI2C gauge;          /* battery fuel gauge */
    int nfc_ven_gpio;           /* NFC enable, SoC output */

    /* SPI NAND on the second flash controller; nand_blocks 0 = none */
    uint32_t nand_id;
    uint32_t nand_page_size;
    uint32_t nand_oob_size;
    uint32_t nand_pages_per_block;
    uint32_t nand_blocks;

    BesBoardKey keys[BES_BOARD_MAX_KEYS];   /* name NULL ends the list */
} BesBoard;

/* a chip: builds the SoC around the firmware and the board's devices */
typedef struct BesSoc {
    const char *chip;           /* as in CHIP= */
    void (*init)(MachineState *ms, BesImage *img, const BesBoard *board);
    /* optional: runs on system reset after every device has been reset */
    void (*reset)(MachineState *ms);
} BesSoc;

void bes_soc_register(const BesSoc *soc);

/*
 * Machines: "miwear" takes the chip from the firmware, "best1503" and
 * "best1700" insist on theirs. All take profile=, flash-image=,
 * nand-image= and tee-image=.
 */
#define TYPE_BES_MACHINE MACHINE_TYPE_NAME("bes-vela")
OBJECT_DECLARE_TYPE(BesMachineState, BesMachineClass, BES_MACHINE)

struct BesMachineState {
    MachineState parent_obj;

    char *profile;
    char *flash_image;
    char *nand_image;
    char *tee_image;

    const BesSoc *soc;
};

struct BesMachineClass {
    MachineClass parent_class;

    const char *chip;           /* NULL: the firmware's */
};

/* the TEE image to boot (tee-image=, else a vela_tee.bin next to the
 * -kernel firmware), NULL for none; g_free() it */
char *bes_machine_tee_image(MachineState *ms);

/* the board's push buttons as a gpio-keys device, NULL if it has none */
DeviceState *bes_board_create_keys(const BesBoard *board);

#endif
