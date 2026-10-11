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
#include "hw/input/hynitron_ts.h"
#include "hw/input/raydium_ts.h"
#include "hw/misc/cw2215.h"
#include "hw/misc/sgm25062.h"
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
        .name = "q66",
        .desc = "Xiaomi Smart Band 11",
        .chip = "best1503",
        .product = "q66",
        /* 依据为 q66 4.100.155 IDB（4.100.177 实测行为一致）。
         * 212x520：panel_simple 动态识别（DCS 0xDA 读 ID）落空后用
         * panel.ic.DEFAULT，panel_getvideoinfo 日志 xres 212 yres 520；
         * TE 脚 = sub_C0C5B78 返回的 0x41（te_gpio_irq_handler 校验
         * byte_200FB7A8，由 sub_C934DE4 写入）。 */
        .panel_width = 212,
        .panel_height = 520,
        .lcd_te_gpio = 65,
        /* 触摸配置表 0x200812C4（get_touch_cfg sub_C3659B4，32 字节
         * 一项）：cst92xx 0x5A / zinitix 0x20 / sitronix 0x55，均在 mi2c
         * 适配器 3（sub_C6DF2A8(3)；适配器 N = hal I2C N，
         * sub_C35CBC0），400kHz；引脚表 0x200812A0：reset 28、
         * interrupt 27、vcc_en 74。board_hynitron_setup 先试 cst92xx，
         * 只有 I2C 不应答时才转 zinitix，故装 CST92xx。 */
        .touch = { TYPE_HYNITRON_TS, 3, 0x5A, 27, false, 0x9217 },
        /* 板号 0（board.binfos "bt"）：board_bsp_product_init 记
         * "not support nfc"，NFC 栈不启动，不装 */
        /* NAND：固件按 512MB 表分区（实测日志 resource first 0
         * nblocks 65536、nand_data first 65536 nblocks 196608），
         * vela_resource.bin（120946688 字节）装入 resource 分区 */
        .nand_id = 0xC8F4C8,
        .nand_page_size = 2048,
        .nand_oob_size = 64,
        .nand_pages_per_block = 64,
        .nand_blocks = 4096,
        /* GPADC（adc ioctl sub_C357D7C）：ch2 电池电压，soft_gauge
         * 配置 0x20081324 换算 vbat = 0 + 7.0 * mV，4.0V = 571 mV；ch5
         * 电池 NTC（get_bat_temp_cool sub_C6EF428，表 0x20081370 中
         * 443..457 mV = 25 C）、ch3 第二路 NTC，均取 450 mV。 */
        .gpadc_mv = { [2] = 571, [3] = 450, [5] = 450 },
        /* 无实体按键 */
    },
    {
        .name = "n66",
        .desc = "Xiaomi Smart Band 9",
        .chip = "best1503",
        .product = "n66",
        /* 依据为 n66 3.1.32 IDB。192x490：LCD 识别（rm690b0 / rm690c0 /
         * sh8601z1 读 ID）落空后默认 rm690b0，日志 "rel_screen_w = 192,
         * rel_screen_h = 490"。TE 脚未实测，沿用 GPIO 65；运行中没有
         * "Bad te gpio" 报错。 */
        .panel_width = 192,
        .panel_height = 490,
        /* 实测 SPU DCS 流：CASET 24..215（RASET 0..489），即 192 列的可见区
         * 从 GRAM 第 24 列开始；偏移若取 14 则窗口超出 192 列，画面右移
         * 10 像素并被截掉右侧 */
        .panel_x_offset = 24,
        .lcd_te_gpio = 65,
        /* board_tp_initialize（sub_C3AC660）：I2C 3（sub_C1689A8(3)），
         * 先探测 Zinitix @0x20（devid 须为 0xE623），再探测 CST92xx
         * @0x5A；引脚 INT 0x1041B（GPIO 27）、RESET 0x1401C（28）、
         * 供电 0x1404A / 0x1404D。 */
        .touch = { TYPE_HYNITRON_TS, 3, 0x5A, 27, true, 0x9220 },
        /* NAND：固件分区 nand_system first 0 nblocks 65536、nand_data
         * first 65536 nblocks 65536（实测日志），/system 是 YAFFS2
         * （emu/mknand.py --profile n66 --system <固件目录>）；固件 ID 表
         * 0xC739F9C 含 c8-f4-c8（512MB，2048+64B 页） */
        .nand_id = 0xC8F4C8,
        .nand_page_size = 2048,
        .nand_oob_size = 64,
        .nand_pages_per_block = 64,
        .nand_blocks = 4096,
        /* 板号 ADC 用 PMU 默认 ch4 816 mV：hw 版本表 0x2C698FC8
         * 701..900 mV = 3（实测 "get_hw_ver: vol=815, hwver=3"）。
         * 无实体按键 */
    },
    {
        .name = "o66",
        .desc = "Xiaomi Smart Band 10",
        .chip = "best1503",
        .product = "o66",
        /* 依据为 o66 3.1.28 IDB：rm690b0 vtable 0x2c10b9e4 给出 212x520
         * （0xD4 x 0x208）；实测 SPU DCS 流 CASET 24..235（RASET 0..519），
         * 可见区从 GRAM 第 24 列开始。TE 脚未实测，沿用 q66 的 GPIO 65；
         * 运行中没有 "Bad te gpio" 报错 */
        .panel_width = 212,
        .panel_height = 520,
        .panel_x_offset = 24,
        .lcd_te_gpio = 65,
        /* CST9217M @0x5A on I2C3，INT GPIO 27 */
        .touch = { TYPE_HYNITRON_TS, 3, 0x5A, 27, true, 0x9220 },
        /* NAND：固件 512MB 表 nand_system first 0 nblocks 65536、nand_data
         * first 65536 nblocks 196608；/system 是 YAFFS2 */
        .nand_id = 0xC8F4C8,
        .nand_page_size = 2048,
        .nand_oob_size = 64,
        .nand_pages_per_block = 64,
        .nand_blocks = 4096,
        /* 板载 GPADC：ch2=电池分压(571 mV 对应 4.0V)，ch3/ch5=NTC 测温
         * (450 mV 对应室温 25℃，低过 84 mV 会报 70℃ 触发过温保护) */
        .gpadc_mv = { [2] = 571, [3] = 450, [5] = 450 },
    },
    {
        .name = "n67",
        .desc = "Xiaomi Smart Band 9 Pro",
        .chip = "best1503",
        .product = "n67",
        .panel_width = 336,
        .panel_height = 480,
        .lcd_te_gpio = 47,
        /* CST9217 @0x5A on I2C3，INT GPIO 27 */
        /* 触摸在 I2C5（0x40063000）：固件对 0x5A 的访问全部落在该总线上，INT GPIO 76 (0x4C) */
        .touch = { TYPE_HYNITRON_TS, 5, 0x5A, 76, true, 0x9217 },
        /* W1150 light device on I2C5 at 0x48 (sub_C3913A0 w11xx init) */
        .light = { "w1150.als", 5, 0x48, -1 },
        /* NAND：固件表 nand_system first 0 nblocks 57600、nand_data
         * first 57600 nblocks 73472；/system 是 YAFFS2 目录树 */
        .nand_id = 0xC8F4C8,
        .nand_page_size = 2048,
        .nand_oob_size = 64,
        .nand_pages_per_block = 64,
        .nand_blocks = 4096,
        /* 未配 GPADC：沿用 n66 的 ch2/ch3/ch5 标定时固件把电池读成 38 mV，判低电量关机 */
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
        /* Raydium RM6D03x：挂载在 AP I2C0 (0x40006000)，从机地址 0x39，
         * 中断 GPIO 73（实测 AP 日志与驱动反汇编） */
        .touch = { TYPE_RAYDIUM_TS, 0, 0x39, 73 },
        /* NXP SN110，同 p67 的 bes sn110 驱动与固件文件
         * NXP_01_10_6E.bin：board_sn110_initialize（0x1088DA68，由
         * 0x10878E20 以总线 4 调用）配 I2C 4（0x40077000）地址 0x28、
         * 400kHz，中断(数据就绪)=90，VEN=91（另有 DWL_REQ=89、电源=70，
         * 模型不需要）。没有它 miwear 主线程在支付宝 eSE 开通道上等
         * 两个 20 秒超时才画出第一页。 */
        .nfc = { TYPE_NCI_NFCC, 4, 0x28, 90 },
        /* CW2215：AP 电池驱动 board_cw2215_initialize 在 I2C 3
         * （0x40076000）地址 0x64 读芯片 ID（实测 I2C 访问） */
        .gauge = { TYPE_CW2215, 3, 0x64, -1 },
        /* sensor bes_switch_setup (0x0c09af0c): bus 2, address 0x18.
         * Its bus number matches the BTH hal_i2c controller ID. */
        .sensor_switch = { TYPE_SGM25062, 2, 0x18, -1 },
        .nfc_ven_gpio = 91,
        /* 协议栈只接受与自带固件 NXP_01_10_6E.bin 相同的 ROM 01 / 固件
         * 10.6E（sub_10EE2380：ROM 版本须为 1、固件主版本须为 0x10，否则
         * NFA_Enable 报 0xF0 "Unexpected firmware"；文件首帧 c0 00 6e 10
         * 即 6E/10）；ROM 低半字节 1 也是探测表 NXP 项的匹配值。 */
        .nfc_rom_version = 0x01,
        .nfc_fw_version = 0x106E,
        /* 两个侧键接 PMU GPIO，按下接地：AP 板级按键初始化
         * sub_1081D65C 的按键表（PSRAM 0x185F0C0C，8 字节一项：键号、
         * 引脚、iomux 配置、有效电平 0）= {键 2: 114, 键 1: 113}。
         * 实测（miwear 输入管理器日志 btn_queue_cb）：114 是 HOME（表冠），
         * 113 是 DOWN。宿主键 Home / Page Down。 */
        .keys = {
            { "home", Q_KEY_CODE_HOME, 114, true },
            { "down", Q_KEY_CODE_PGDN, 113, true },
        },
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

    /*
     * A firmware also names products it is compatible with (q66 lists two
     * o66 codes in its model table), so the board is the product the
     * image names most often; a tie is ambiguous.
     */
    unsigned best = 0;

    for (i = 0; i < ARRAY_SIZE(bes_boards); i++) {
        g_autofree char *tag = g_strconcat("miwear.watch.",
                                           bes_boards[i].product, NULL);
        unsigned n;

        if (!g_str_equal(bes_boards[i].chip, chip)) {
            continue;
        }
        n = bes_image_count(img, tag);
        if (n > best) {
            found = &bes_boards[i];
            best = n;
            matches = 1;
        } else if (n && n == best) {
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

static char *bes_get_sensor_image(Object *obj, Error **errp)
{
    return g_strdup(BES_MACHINE(obj)->sensor_image);
}

static void bes_set_sensor_image(Object *obj, const char *val, Error **errp)
{
    g_free(BES_MACHINE(obj)->sensor_image);
    BES_MACHINE(obj)->sensor_image = g_strdup(val);
}

static char *bes_get_sensor_nor(Object *obj, Error **errp)
{
    return g_strdup(BES_MACHINE(obj)->sensor_nor);
}

static void bes_set_sensor_nor(Object *obj, const char *val, Error **errp)
{
    g_free(BES_MACHINE(obj)->sensor_nor);
    BES_MACHINE(obj)->sensor_nor = g_strdup(val);
}

/* the image named by a property, else <name> next to -kernel; "" = none */
static char *bes_machine_sibling_image(MachineState *ms, const char *prop,
                                       const char *name)
{
    g_autofree char *dir = NULL;
    char *path;

    if (prop) {
        return *prop ? g_strdup(prop) : NULL;
    }
    dir = g_path_get_dirname(ms->kernel_filename);
    path = g_build_filename(dir, name, NULL);
    if (g_file_test(path, G_FILE_TEST_IS_REGULAR)) {
        return path;
    }
    g_free(path);
    return NULL;
}

char *bes_machine_sensor_image(MachineState *ms)
{
    return bes_machine_sibling_image(ms, BES_MACHINE(ms)->sensor_image,
                                     "vela_sensor.bin");
}

char *bes_machine_tee_image(MachineState *ms)
{
    return bes_machine_sibling_image(ms, BES_MACHINE(ms)->tee_image,
                                     "vela_tee.bin");
}

static void bes_machine_finalize(Object *obj)
{
    BesMachineState *bms = BES_MACHINE(obj);

    g_free(bms->profile);
    g_free(bms->flash_image);
    g_free(bms->nand_image);
    g_free(bms->tee_image);
    g_free(bms->sensor_image);
    g_free(bms->sensor_nor);
}

static void bes_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->init = bes_machine_init;
    mc->reset = bes_machine_reset;
    mc->ignore_memory_transaction_failures = true;
    /* best1700 also runs its second M55 (M55C1) and the BTH core; the
     * vCPU count sizes MTTCG's contexts. best1503 creates one CPU and
     * leaves the others unused. */
    mc->default_cpus = 3;
    mc->min_cpus = 1;
    mc->max_cpus = 3;

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
    object_class_property_add_str(oc, "sensor-image", bes_get_sensor_image,
                                  bes_set_sensor_image);
    object_class_property_set_description(oc, "sensor-image",
                                          "sensor hub image, executed in "
                                          "place from the BTH NOR flash on "
                                          "chips that have one; default: "
                                          "vela_sensor.bin next to the "
                                          "-kernel firmware, empty for none");
    object_class_property_add_str(oc, "sensor-nor", bes_get_sensor_nor,
                                  bes_set_sensor_nor);
    object_class_property_set_description(oc, "sensor-nor",
                                          "backing image file of the sensor "
                                          "hub's own NOR flash (loaded at "
                                          "startup, written back on exit; "
                                          "the sensor image is installed at "
                                          "its start); see emu/mksensornor.py");
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
