/*
 * Xiaomi Vela devices on BES chips: board profiles, the SoC registry and
 * the "miwear" / "best1503" / "best1700" machines (see bes_board.h)
 *
 * 板级事实（屏幕、触摸/NFC 控制器及其总线/地址/中断脚、SPI NAND、按键）
 * 只写在这里的设备 profile 里，芯片文件（best1503.c、best1700.c）只持有
 * 芯片事实。每一项都要有固件或真机出处，与 DESIGN.md 的纪律相同；未查实
 * 的外设不写（= 不装配），而不是猜一个值。
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qobject/qlist.h"
#include "hw/core/qdev-properties.h"
#include "hw/arm/bes_board.h"
#include "hw/arm/machines-qom.h"
#include "hw/input/gpio_keys.h"
#include "hw/input/zinitix_ts.h"
#include "hw/misc/cw2215.h"
#include "hw/misc/nci_nfcc.h"
#include "system/reset.h"
#include "target/arm/cpu-qom.h"

static const BesBoard bes_boards[] = {
    {
        .name = "p67",
        .desc = "Xiaomi Smart Band 10 Pro",
        .chip = "best1503",
        .product = "p67",
        /* 336x480：固件 LCDC 初始化 sub_C225BA4 的层/输出尺寸；TE 接
         * GPIO 47（LCD TE 引脚配置 sub_C0E3DF0 返回 47，经 sub_C232740
         * 配成边沿中断）。 */
        .panel_width = 336,
        .panel_height = 480,
        .lcd_te_gpio = 47,
        /* Zinitix ZTW662（真机日志 chip id 0xE623）。bes_ts_zinitix_setup
         * (sub_C3F84CC) 打开 I2C 总线 5；板级配置 0x2CD0605C = {I2C 地址
         * 0x20, 400kHz, INT pinset 0x1004C (GPIO 76), RESET 0x1404E}。 */
        .touch = { TYPE_ZINITIX_TS, 5, 0x20, 76 },
        /* bes sn110 驱动 nfc_driver_p67_initialize（sub_C713784）：I2C
         * 总线 6、地址 0x28、400kHz，VEN=69、中断(数据就绪)=77。 */
        .nfc = { TYPE_NCI_NFCC, 6, 0x28, 77 },
        .nfc_ven_gpio = 69,
        /* 固件 ID 表 0x2CCD4134 第 1 项（0x2CCD3FE4）：ID c8-f4-c8，
         * 2048+64B 页，128KB 块，512MB。选 512MB 的依据：
         * bes_board_nand_initialize（sub_C20AB44）按容量选分区表，只有
         * 512MB 表的 resource 分区（0x19000 页 = 200MB）装得下
         * vela_resource.bin（romfs 头标 171754096 字节）。 */
        .nand_id = 0xC8F4C8,
        .nand_page_size = 2048,
        .nand_oob_size = 64,
        .nand_pages_per_block = 64,
        .nand_blocks = 4096,
        /* 无实体按键 */
    },
    {
        /* 板级外设逐个查实后补入 */
        .name = "q63",
        .desc = "Xiaomi Watch S5 (best1700)",
        .chip = "best1700",
        .product = "q63",
        /* rm6d031 圆形 AMOLED，DSI 命令模式：AP 日志 "DSI0 init:
         * width:466"、"lcdc_periphconfig: rel_screen_w = 466,
         * rel_screen_h = 466"（实测）；TE 固定接 GPIO 47，由
         * best1700.c 直接连线，不经此处的 lcd_te_gpio */
        .panel_width = 466,
        .panel_height = 466,
        .lcd_te_gpio = -1,
        .touch = { .irq_gpio = -1 },
        .nfc = { .irq_gpio = -1 },
        /* CW2215：AP 电池驱动 board_cw2215_initialize 在 I2C 3
         * （0x40076000）地址 0x64 读芯片 ID（实测 I2C 访问） */
        .gauge = { TYPE_CW2215, 3, 0x64, -1 },
        .nfc_ven_gpio = -1,
    },
};

/* machine-level facts of each chip (the SoCs register their code later) */
static const struct {
    const char *chip;
    const char *desc;
    const char *cpu_type;
} bes_chips[] = {
    { "best1503", "BEST1503 / BES2700iMP (Cortex-M33)",
      ARM_CPU_TYPE_NAME("cortex-m33") },
    { "best1700", "BEST1700 (Cortex-M55)",
      ARM_CPU_TYPE_NAME("cortex-m55") },
};

static GSList *bes_socs;

void bes_soc_register(const BesSoc *soc)
{
    bes_socs = g_slist_append(bes_socs, (gpointer)soc);
}

static const BesSoc *bes_soc_find(const char *chip)
{
    GSList *l;

    for (l = bes_socs; l; l = l->next) {
        const BesSoc *soc = l->data;

        if (g_str_equal(soc->chip, chip)) {
            return soc;
        }
    }
    return NULL;
}

static void bes_board_list(void)
{
    int i;

    printf("Board profiles (profile=<name>):\n");
    for (i = 0; i < ARRAY_SIZE(bes_boards); i++) {
        printf("  %-8s %-9s %s (firmware miwear.watch.%s*)\n",
               bes_boards[i].name, bes_boards[i].chip, bes_boards[i].desc,
               bes_boards[i].product);
    }
}

/* the board named by profile=, or the one whose product the firmware is */
static const BesBoard *bes_board_resolve(const char *profile,
                                         const BesImage *img,
                                         const char *chip, Error **errp)
{
    const BesBoard *found = NULL;
    int i, matches = 0;

    if (profile) {
        for (i = 0; i < ARRAY_SIZE(bes_boards); i++) {
            if (g_str_equal(bes_boards[i].name, profile)) {
                found = &bes_boards[i];
            }
        }
        if (!found) {
            error_setg(errp, "unknown board profile '%s' "
                       "(profile=help lists them)", profile);
        } else if (!g_str_equal(found->chip, chip)) {
            error_setg(errp, "board profile '%s' is a %s board, the "
                       "firmware is for %s", profile, found->chip, chip);
            found = NULL;
        }
        return found;
    }

    for (i = 0; i < ARRAY_SIZE(bes_boards); i++) {
        g_autofree char *tag = g_strconcat("miwear.watch.",
                                           bes_boards[i].product, NULL);

        if (g_str_equal(bes_boards[i].chip, chip) &&
            bes_image_contains(img, tag)) {
            found = &bes_boards[i];
            matches++;
        }
    }
    if (matches == 1) {
        return found;
    }
    error_setg(errp, "%s %s firmware; pick a board with profile=<name> "
               "(profile=help lists them)",
               matches ? "several board profiles match this" :
                         "no board profile matches this", chip);
    return NULL;
}

DeviceState *bes_board_create_keys(const BesBoard *board)
{
    DeviceState *dev;
    QList *qcodes;
    uint32_t active_low = 0;
    int n;

    if (!board->keys[0].name) {
        return NULL;
    }
    dev = qdev_new(TYPE_GPIO_KEYS);
    qcodes = qlist_new();
    for (n = 0; n < BES_BOARD_MAX_KEYS && board->keys[n].name; n++) {
        qlist_append_int(qcodes, board->keys[n].qcode);
        if (board->keys[n].active_low) {
            active_low |= 1u << n;
        }
    }
    qdev_prop_set_array(dev, "qcodes", qcodes);
    qdev_prop_set_uint32(dev, "active-low", active_low);
    qdev_realize_and_unref(dev, NULL, &error_fatal);
    return dev;
}

/* ---- machines ---- */

static void bes_machine_init(MachineState *ms)
{
    BesMachineState *bms = BES_MACHINE(ms);
    BesMachineClass *bmc = BES_MACHINE_GET_CLASS(ms);
    BesImage *img = g_new0(BesImage, 1);    /* lives as long as the machine */
    const BesBoard *board;
    const BesSoc *soc;
    const char *chip;
    Error *err = NULL;

    if (bms->profile && g_str_equal(bms->profile, "help")) {
        bes_board_list();
        exit(0);
    }
    if (!ms->kernel_filename) {
        error_report("-kernel <firmware image> is required (vela_ap.bin)");
        exit(1);
    }
    if (!bes_image_load(img, ms->kernel_filename, &err)) {
        error_report_err(err);
        exit(1);
    }
    chip = img->chip[0] ? img->chip : NULL;
    if (bmc->chip) {
        if (chip && !g_str_equal(chip, bmc->chip)) {
            error_report("the firmware is for %s, not %s (use -M miwear to "
                         "follow the firmware)", chip, bmc->chip);
            exit(1);
        }
        chip = bmc->chip;
    }
    if (!chip) {
        error_report("the firmware does not name its chip (no CHIP= in its "
                     "boot info); use -M best1503 or -M best1700");
        exit(1);
    }
    soc = bes_soc_find(chip);
    if (!soc) {
        error_report("no %s SoC in this build", chip);
        exit(1);
    }
    board = bes_board_resolve(bms->profile, img, chip, &err);
    if (!board) {
        error_report_err(err);
        exit(1);
    }
    info_report("%s board '%s' (%s); firmware: xip=0x%08x cached=0x%08x "
                "nc=0x%08x entry=0x%08x msp=0x%08x size=0x%zx",
                chip, board->name, board->desc, img->xip_base,
                img->cached_base, img->nc_base, img->entry, img->msp,
                img->size);
    bms->soc = soc;
    soc->init(ms, img, board);
}

static void bes_machine_reset(MachineState *ms, ResetType type)
{
    BesMachineState *bms = BES_MACHINE(ms);

    qemu_devices_reset(type);
    if (bms->soc->reset) {
        bms->soc->reset(ms);
    }
}

static char *bes_get_profile(Object *obj, Error **errp)
{
    return g_strdup(BES_MACHINE(obj)->profile);
}

static void bes_set_profile(Object *obj, const char *val, Error **errp)
{
    g_free(BES_MACHINE(obj)->profile);
    BES_MACHINE(obj)->profile = g_strdup(val);
}

static char *bes_get_flash_image(Object *obj, Error **errp)
{
    return g_strdup(BES_MACHINE(obj)->flash_image);
}

static void bes_set_flash_image(Object *obj, const char *val, Error **errp)
{
    g_free(BES_MACHINE(obj)->flash_image);
    BES_MACHINE(obj)->flash_image = g_strdup(val);
}

static char *bes_get_nand_image(Object *obj, Error **errp)
{
    return g_strdup(BES_MACHINE(obj)->nand_image);
}

static void bes_set_nand_image(Object *obj, const char *val, Error **errp)
{
    g_free(BES_MACHINE(obj)->nand_image);
    BES_MACHINE(obj)->nand_image = g_strdup(val);
}

static char *bes_get_tee_image(Object *obj, Error **errp)
{
    return g_strdup(BES_MACHINE(obj)->tee_image);
}

static void bes_set_tee_image(Object *obj, const char *val, Error **errp)
{
    g_free(BES_MACHINE(obj)->tee_image);
    BES_MACHINE(obj)->tee_image = g_strdup(val);
}

char *bes_machine_tee_image(MachineState *ms)
{
    BesMachineState *bms = BES_MACHINE(ms);
    g_autofree char *dir = NULL;
    char *path;

    if (bms->tee_image) {
        return *bms->tee_image ? g_strdup(bms->tee_image) : NULL;
    }
    dir = g_path_get_dirname(ms->kernel_filename);
    path = g_build_filename(dir, "vela_tee.bin", NULL);
    if (g_file_test(path, G_FILE_TEST_IS_REGULAR)) {
        return path;
    }
    g_free(path);
    return NULL;
}

static void bes_machine_finalize(Object *obj)
{
    BesMachineState *bms = BES_MACHINE(obj);

    g_free(bms->profile);
    g_free(bms->flash_image);
    g_free(bms->nand_image);
    g_free(bms->tee_image);
}

static void bes_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->init = bes_machine_init;
    mc->reset = bes_machine_reset;
    mc->ignore_memory_transaction_failures = true;
    /* best1700 runs a second M55 (M55C1); the vCPU count sizes MTTCG's
     * contexts. best1503 creates one CPU and leaves the second unused. */
    mc->default_cpus = 2;
    mc->min_cpus = 1;
    mc->max_cpus = 2;

    object_class_property_add_str(oc, "profile", bes_get_profile,
                                  bes_set_profile);
    object_class_property_set_description(oc, "profile",
                                          "board (device) profile; default: "
                                          "the one the firmware is for, "
                                          "'help' lists them");
    object_class_property_add_str(oc, "flash-image", bes_get_flash_image,
                                  bes_set_flash_image);
    object_class_property_set_description(oc, "flash-image",
                                          "backing image file of the boot "
                                          "NOR flash (loaded at startup, "
                                          "written back on exit; the -kernel "
                                          "firmware is installed into its AP "
                                          "partition)");
    object_class_property_add_str(oc, "nand-image", bes_get_nand_image,
                                  bes_set_nand_image);
    object_class_property_set_description(oc, "nand-image",
                                          "backing file of the board's SPI "
                                          "NAND (resource romfs + /data); "
                                          "default emu/nand.img, created "
                                          "erased if missing");
    object_class_property_add_str(oc, "tee-image", bes_get_tee_image,
                                  bes_set_tee_image);
    object_class_property_set_description(oc, "tee-image",
                                          "secure-world (TEE) image booted "
                                          "before the AP on chips that have "
                                          "one; default: vela_tee.bin next "
                                          "to the -kernel firmware, empty "
                                          "for none");
}

static void miwear_machine_class_init(ObjectClass *oc, const void *data)
{
    MACHINE_CLASS(oc)->desc = "Xiaomi Vela device on a BES chip (chip and "
                              "board from the firmware)";
}

static void bes_chip_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    const char *chip = data;
    int i;

    BES_MACHINE_CLASS(oc)->chip = chip;
    for (i = 0; i < ARRAY_SIZE(bes_chips); i++) {
        if (g_str_equal(bes_chips[i].chip, chip)) {
            static const char *valid[ARRAY_SIZE(bes_chips)][2];

            valid[i][0] = bes_chips[i].cpu_type;
            mc->desc = bes_chips[i].desc;
            mc->default_cpu_type = bes_chips[i].cpu_type;
            mc->valid_cpu_types = valid[i];
        }
    }
}

static const TypeInfo bes_machine_types[] = {
    {
        .name = TYPE_BES_MACHINE,
        .parent = TYPE_MACHINE,
        .abstract = true,
        .instance_size = sizeof(BesMachineState),
        .instance_finalize = bes_machine_finalize,
        .class_size = sizeof(BesMachineClass),
        .class_init = bes_machine_class_init,
    }, {
        .name = MACHINE_TYPE_NAME("miwear"),
        .parent = TYPE_BES_MACHINE,
        .class_init = miwear_machine_class_init,
        .interfaces = arm_machine_interfaces,
    }, {
        .name = MACHINE_TYPE_NAME("best1503"),
        .parent = TYPE_BES_MACHINE,
        .class_init = bes_chip_machine_class_init,
        .class_data = "best1503",
        .interfaces = arm_machine_interfaces,
    }, {
        .name = MACHINE_TYPE_NAME("best1700"),
        .parent = TYPE_BES_MACHINE,
        .class_init = bes_chip_machine_class_init,
        .class_data = "best1700",
        .interfaces = arm_machine_interfaces,
    },
};

DEFINE_TYPES(bes_machine_types)
