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
    int gpio;                   /* GPIO it drives, firmware hal_gpio
                                 * numbering (best1700: 99.. = PMU GPIO) */
    bool active_low;
} BesBoardKey;

/* an I2C peripheral on the board; type NULL = not fitted */
typedef struct BesBoardI2C {
    const char *type;
    int bus;                    /* SoC I2C controller index */
    uint8_t addr;
    int irq_gpio;               /* interrupt / data-ready line, -1 = none */
    bool hyn92xx;               /* true = Hynitron hyn92xx driver, false = Xiaomi cst92xx */
    uint32_t part_no;           /* touch IC part number / Zinitix
                                 * hardware id (0 = default) */
    uint32_t fw_version;        /* Zinitix register version (0 = default) */
} BesBoardI2C;

/* how the firmware's soft gauge reads its VBAT -> charge table */
typedef enum BesOcvLookup {
    BES_OCV_UP_TO = 1,          /* L% for VBAT in (ocv[L - 1], ocv[L]] */
    BES_OCV_FROM,               /* L% from ocv[L] (interpolating) */
} BesOcvLookup;

/*
 * What the battery and the charger plug drive on the board. A fuel gauge
 * IC (BesBoard.gauge) reports the charge itself; otherwise the firmware's
 * soft gauge converts VBAT through a GPADC divider with its own table.
 */
typedef struct BesBoardBattery {
    const uint32_t *ocv_uv;     /* the firmware's VBAT (uV) at 0..100%;
                                 * NULL = no soft gauge */
    BesOcvLookup lookup;
    uint32_t ui_permille;       /* level shown = table level x this / 1000
                                 * (BES_OCV_FROM); 0 = 1000 */
    int vbat_ch;                /* GPADC channel of the VBAT divider */
    uint32_t vbat_ratio;        /* VBAT / channel input, x1000 */
    uint32_t load_uv;           /* the gauge looks VBAT up this much lower
                                 * while discharging */
    int vbus_ch;                /* GPADC channel sensing VBUS */
    uint32_t vbus_ratio;        /* VBUS / channel input, x1000; 0 = none */
    int plug_gpio;              /* plug-in detect line, 0 = none */
    bool plug_active_low;
    bool pmu_ac_on;             /* VBUS reaches the PMU's AC_ON detector */
} BesBoardBattery;

typedef struct BesBoard {
    const char *name;           /* profile=<name> */
    const char *desc;
    const char *chip;           /* the firmware's CHIP= */
    const char *product;        /* the firmware's "miwear.watch.<product>" */

    /* display panel; 0 x 0 = no display modelled */
    uint32_t panel_width;
    uint32_t panel_height;
    int lcd_te_gpio;            /* tearing-effect line, -1 = none */
    /* GRAM column / row where the visible area starts (CASET / RASET of a
     * full-screen frame), 0 for panels whose GRAM is the visible area */
    uint32_t panel_x_offset;
    uint32_t panel_y_offset;
    /* panel on a MIPI DSI host (best1502p 0x40230000) instead of the
     * LCDC's QSPI: its DCS 0x04 ID bytes, low byte first; 0 = no DSI */
    uint32_t dsi_panel_id;

    BesBoardI2C touch;
    BesBoardI2C light;          /* ambient light / PPG front end, -1 irq */
    BesBoardI2C nfc;
    BesBoardI2C gauge;          /* battery fuel gauge */
    BesBoardI2C charger;        /* battery charger */
    BesBoardI2C sensor_switch;  /* load switch on the sensor core's I2C */
    int nfc_ven_gpio;           /* NFC enable, SoC output */
    /*
     * what the NFC controller reports in CORE_RESET_NTF: the byte the
     * driver's chip probe and the NXP stack read as ROM / hardware version,
     * and the firmware version (major << 8 | minor); 0 = model default
     */
    uint8_t nfc_rom_version;
    uint16_t nfc_fw_version;

    /* SPI NAND on the second flash controller; nand_blocks 0 = none */
    uint32_t nand_id;
    uint32_t nand_page_size;
    uint32_t nand_oob_size;
    uint32_t nand_pages_per_block;
    uint32_t nand_blocks;

    BesBoardKey keys[BES_BOARD_MAX_KEYS];   /* name NULL ends the list */

    /*
     * PMU GPADC channel inputs in mV, what the board wires to each channel
     * (battery divider, NTCs, hardware-version strap); all 0 = the PMU
     * model's defaults (the p67 wiring)
     */
    uint32_t gpadc_mv[8];

    BesBoardBattery battery;
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
 * nand-image=, tee-image= and sensor-image=.
 */
#define TYPE_BES_MACHINE MACHINE_TYPE_NAME("bes-vela")
OBJECT_DECLARE_TYPE(BesMachineState, BesMachineClass, BES_MACHINE)

/*
 * The battery's links into the board's devices, filled in by the SoC
 * (any may be NULL): the PMU's GPADC and AC_ON detector, the fuel gauge,
 * the charger's "vbus" / "full" inputs and the plug detect line.
 */
typedef struct BesBatteryLinks {
    struct BesPmuState *pmu;
    struct CW2215State *gauge;
    qemu_irq charger_vbus;
    qemu_irq charger_full;
    qemu_irq plug;
    qemu_irq ac_on;             /* the PMU's AC_ON detector, if not pmu's */
} BesBatteryLinks;

struct BesMachineState {
    MachineState parent_obj;

    char *profile;
    char *flash_image;
    char *nand_image;
    char *tee_image;
    char *sensor_image;
    char *sensor_nor;

    const BesSoc *soc;
    const BesBoard *board;

    /* battery-level / battery-charging: the cell and the charger plug */
    uint8_t battery_level;
    bool battery_charging;
    BesBatteryLinks battery;
};

struct BesMachineClass {
    MachineClass parent_class;

    const char *chip;           /* NULL: the firmware's */
};

/* the TEE image to boot (tee-image=, else a vela_tee.bin next to the
 * -kernel firmware), NULL for none; g_free() it */
char *bes_machine_tee_image(MachineState *ms);

/* the sensor hub image (sensor-image=, else a vela_sensor.bin next to the
 * -kernel firmware), NULL for none; g_free() it */
char *bes_machine_sensor_image(MachineState *ms);

/* the board's push buttons as a gpio-keys device, NULL if it has none */
DeviceState *bes_board_create_keys(const BesBoard *board);

/* hand the board's battery links to the machine, which drives them from
 * battery-level / battery-charging (now and after every reset) */
void bes_machine_attach_battery(MachineState *ms, const BesBatteryLinks *l);

#endif
