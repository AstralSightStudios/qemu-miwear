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
#include "hw/misc/sy6103x.h"
#include "hw/misc/sgm25062.h"
#include "hw/misc/nci_nfcc.h"
#include "hw/misc/sc7061.h"
#include "hw/misc/hp4560x.h"
#include "hw/misc/sgm41551.h"
#include "hw/misc/bes_pmu.h"
#include "hw/core/irq.h"
#include "qapi/visitor.h"
#include "system/reset.h"
#include "target/arm/cpu-qom.h"

/*
 * The soft gauges' VBAT -> charge tables, VBAT in uV by percent, copied
 * from each firmware (the table its gauge picks at room temperature for
 * the battery the emulated board reports). Firmware versions of a
 * product share them.
 */
/* p67: 0x2CD05168 (sub_C3DC460, 11..50 C) */
static const uint32_t bes_ocv_p67[101] = {
    3500100, 3554200, 3605400, 3630600, 3640800, 3645000,
    3650200, 3655600, 3660000, 3665400, 3671800, 3680200,
    3688600, 3695700, 3702200, 3709000, 3715200, 3721100,
    3726100, 3730400, 3735100, 3739700, 3745000, 3750000,
    3754600, 3759600, 3763900, 3768600, 3773200, 3777500,
    3782200, 3786500, 3791200, 3796100, 3800800, 3805800,
    3810400, 3815700, 3821300, 3826500, 3832100, 3838000,
    3844200, 3850100, 3856300, 3862800, 3869600, 3876400,
    3883600, 3891300, 3898800, 3906800, 3915500, 3924500,
    3934400, 3945600, 3957400, 3969800, 3982200, 3994200,
    4006000, 4017800, 4028000, 4039500, 4051000, 4062800,
    4074200, 4086000, 4096200, 4108000, 4120100, 4131900,
    4144000, 4156400, 4168500, 4180900, 4193000, 4205100,
    4217100, 4229600, 4241300, 4253700, 4265200, 4277300,
    4288500, 4302700, 4313600, 4323500, 4333700, 4343900,
    4353200, 4362200, 4371200, 4379600, 4388000, 4395700,
    4403500, 4411200, 4419000, 4428000, 4438500,
};
/* n67: 0x2C5E3670 (3.1.175, sub_C154950; -10..50 C) */
static const uint32_t bes_ocv_n67[101] = {
    3500200, 3557500, 3603000, 3639300, 3658300, 3665100,
    3670000, 3672500, 3675000, 3676800, 3679300, 3685400,
    3693400, 3699600, 3705700, 3711200, 3718000, 3722900,
    3728500, 3732800, 3736500, 3742000, 3746900, 3751200,
    3756800, 3761100, 3766000, 3770300, 3774000, 3778900,
    3782600, 3786900, 3791200, 3795500, 3799200, 3804100,
    3808400, 3813400, 3817700, 3822600, 3827500, 3833000,
    3838600, 3844100, 3849700, 3855200, 3861300, 3867500,
    3874300, 3880400, 3887800, 3894600, 3902600, 3910600,
    3919200, 3928400, 3938900, 3949300, 3961000, 3972100,
    3983800, 3994800, 4005900, 4016400, 4026800, 4037300,
    4048400, 4058800, 4069900, 4081000, 4092600, 4103700,
    4114800, 4126500, 4137500, 4149200, 4160900, 4173200,
    4184900, 4196600, 4208300, 4220000, 4232300, 4244000,
    4255700, 4267300, 4279000, 4289500, 4300600, 4311000,
    4321500, 4331900, 4342400, 4351600, 4360900, 4370700,
    4379300, 4388500, 4397100, 4405800, 4415000,
};
/* p65: the 10 C and up table of sub_C4F71F0 (file offset 0x1AFC48) */
static const uint32_t bes_ocv_p65[101] = {
    3500000, 3573000, 3623000, 3643000, 3653000, 3657000,
    3662000, 3665000, 3669000, 3677000, 3686000, 3693000,
    3700000, 3706000, 3712000, 3717000, 3722000, 3726000,
    3729000, 3734000, 3738000, 3742000, 3746000, 3750000,
    3753000, 3756000, 3761000, 3764000, 3768000, 3772000,
    3775000, 3779000, 3783000, 3788000, 3791000, 3796000,
    3801000, 3806000, 3810000, 3816000, 3821000, 3826000,
    3831000, 3837000, 3843000, 3849000, 3855000, 3862000,
    3868000, 3876000, 3882000, 3891000, 3900000, 3908000,
    3917000, 3928000, 3938000, 3949000, 3960000, 3970000,
    3981000, 3992000, 4002000, 4012000, 4023000, 4034000,
    4044000, 4056000, 4066000, 4077000, 4089000, 4101000,
    4112000, 4124000, 4136000, 4148000, 4160000, 4172000,
    4184000, 4196000, 4208000, 4220000, 4232000, 4244000,
    4255000, 4267000, 4279000, 4289000, 4301000, 4311000,
    4322000, 4331000, 4342000, 4352000, 4362000, 4371000,
    4381000, 4391000, 4402000, 4415000, 4432000,
};
/* q65: the 10 C and up table of sub_C4FA4DC (file offset 0x1A26CC) */
static const uint32_t bes_ocv_q65[101] = {
    3500000, 3573000, 3623000, 3643000, 3653000, 3657000,
    3662000, 3665000, 3669000, 3677000, 3686000, 3693000,
    3700000, 3706000, 3712000, 3717000, 3722000, 3726000,
    3729000, 3734000, 3738000, 3742000, 3746000, 3750000,
    3753000, 3756000, 3761000, 3764000, 3768000, 3772000,
    3775000, 3779000, 3783000, 3788000, 3791000, 3796000,
    3801000, 3806000, 3810000, 3816000, 3821000, 3826000,
    3831000, 3837000, 3843000, 3849000, 3855000, 3862000,
    3868000, 3876000, 3882000, 3891000, 3900000, 3908000,
    3917000, 3928000, 3938000, 3949000, 3960000, 3970000,
    3981000, 3992000, 4002000, 4012000, 4023000, 4034000,
    4044000, 4056000, 4066000, 4077000, 4089000, 4101000,
    4112000, 4124000, 4136000, 4148000, 4160000, 4172000,
    4184000, 4196000, 4208000, 4220000, 4232000, 4244000,
    4255000, 4267000, 4279000, 4289000, 4301000, 4311000,
    4322000, 4416000, 4426000, 4437000, 4447000, 4458000,
    4468000, 4479000, 4489000, 4500000, 4510000,
};
/* n66 and o66: 0x2C72F0F8 (sub_C4E3A38, battery id 0 at 25 C) */
static const uint32_t bes_ocv_n66[101] = {
    3478000, 3540000, 3590000, 3631000, 3665000, 3686000,
    3692000, 3697000, 3698000, 3700000, 3701000, 3707000,
    3714000, 3722000, 3727000, 3733000, 3739000, 3745000,
    3750000, 3755000, 3759000, 3762000, 3768000, 3772000,
    3777000, 3783000, 3788000, 3793000, 3799000, 3804000,
    3808000, 3812000, 3816000, 3820000, 3823000, 3828000,
    3832000, 3836000, 3841000, 3845000, 3850000, 3855000,
    3860000, 3866000, 3871000, 3882000, 3889000, 3895000,
    3901000, 3908000, 3916000, 3924000, 3932000, 3941000,
    3952000, 3964000, 3976000, 3989000, 4001000, 4012000,
    4023000, 4033000, 4043000, 4054000, 4064000, 4075000,
    4085000, 4096000, 4108000, 4119000, 4130000, 4142000,
    4153000, 4165000, 4176000, 4189000, 4200000, 4211000,
    4224000, 4235000, 4248000, 4259000, 4283000, 4294000,
    4306000, 4317000, 4328000, 4339000, 4349000, 4360000,
    4370000, 4379000, 4389000, 4398000, 4408000, 4417000,
    4425000, 4435000, 4444000, 4454000, 4464000,
};
/* q66: 0x2008213C (4.100.155, sub_C36582C "xwd": no battery id; 25..45 C) */
static const uint32_t bes_ocv_q66[101] = {
    3451000, 3531000, 3591000, 3637000, 3674000, 3689000,
    3693000, 3697000, 3698000, 3699000, 3700000, 3703000,
    3711000, 3719000, 3724000, 3730000, 3736000, 3743000,
    3748000, 3752000, 3756000, 3760000, 3764000, 3769000,
    3774000, 3779000, 3783000, 3788000, 3793000, 3797000,
    3801000, 3805000, 3808000, 3812000, 3816000, 3820000,
    3823000, 3828000, 3831000, 3836000, 3840000, 3845000,
    3850000, 3855000, 3860000, 3866000, 3871000, 3877000,
    3883000, 3889000, 3896000, 3903000, 3911000, 3919000,
    3928000, 3940000, 3952000, 3964000, 3976000, 3988000,
    3998000, 4008000, 4018000, 4028000, 4039000, 4049000,
    4060000, 4071000, 4082000, 4093000, 4104000, 4115000,
    4127000, 4138000, 4150000, 4162000, 4174000, 4186000,
    4198000, 4210000, 4222000, 4234000, 4246000, 4259000,
    4270000, 4283000, 4294000, 4307000, 4318000, 4329000,
    4340000, 4350000, 4361000, 4371000, 4382000, 4392000,
    4402000, 4412000, 4422000, 4432000, 4442000,
};

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
        /* charger probe sub_C404878: I2C3 0x07, SC7061 (revision 7 or 4
         * at 0x01) else HP4570; without one the soft gauge is not
         * registered either */
        .charger = { TYPE_SC7061, 3, 0x07, -1 },
        /* soft gauge: VBAT = 4.9 x ch4 (sub_C3DC1B8), VBUS = 6.8824 x ch3
         * (sub_C3DC740) */
        .battery = {
            .ocv_uv = bes_ocv_p67, .lookup = BES_OCV_UP_TO,
            .vbat_ch = 4, .vbat_ratio = 4900,
            .vbus_ch = 3, .vbus_ratio = 6882,
            .pmu_ac_on = true,
        },
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
        /* charger table off_2CA09A20: SC7061 (0x07 at 0x01), HP4570,
         * SY6105, all on I2C0 0x07 (get_charger_cfg 0x200815BC) */
        .charger = { TYPE_SC7061, 0, 0x07, -1 },
        /* plug detect: "plugin" device's "detect" pin, GPIO 83, low =
         * plugged in (board_plug_in_initialize in sub_C35C370) */
        .battery = {
            .ocv_uv = bes_ocv_q66, .lookup = BES_OCV_FROM,
            .ui_permille = 1055,
            .vbat_ch = 2, .vbat_ratio = 7000,
            .plug_gpio = 83, .plug_active_low = true,
            .pmu_ac_on = true,
        },
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
        /* hp4560x_initialize (sub_C39F92C): I2C0 0x07; plug-in reaches it
         * through the PMU charger interrupt (hp4560x_charger_handler) */
        .charger = { TYPE_HP4560X, 0, 0x07, -1 },
        /* GPADC: ch3 the battery NTC (sub_C166A34, table 0x2C5EF3A8
         * 443..459 mV = 25 C; 0 mV reads 70 C), ch4 the board id strap */
        .gpadc_mv = { [3] = 450, [4] = 816, [5] = 250 },
        /* soft gauge: VBAT = 7.0 x ch2 (sub_C166994); coulomb counting
         * moves its charge by at most 1% a poll, shown x1.055
         * (gauge_detect_worker sub_C1672A0) */
        .battery = {
            .ocv_uv = bes_ocv_n66, .lookup = BES_OCV_FROM,
            .ui_permille = 1055,
            .vbat_ch = 2, .vbat_ratio = 7000,
            .pmu_ac_on = true,
        },
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
        /* the Band 9 charger code and tables: hp4560x then sc7061 */
        .charger = { TYPE_HP4560X, 0, 0x07, -1 },
        .battery = {
            .ocv_uv = bes_ocv_n66, .lookup = BES_OCV_FROM,
            .ui_permille = 1055,
            .vbat_ch = 2, .vbat_ratio = 7000,
            .pmu_ac_on = true,
        },
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
        /* charger probe sub_C0C9F18: I2C3 0x07, SY6103X (0x98 at 0x60),
         * then HP4560, then SC7061 (6 at 0x01) */
        .charger = { TYPE_SY6103X, 3, 0x07, -1 },
        /* soft gauge: VBAT = 4.9 x ch4 (sub_C0E3BB0), looked up 50 mV
         * lower while discharging (sub_C154950); VBUS = 6.8824 x ch3
         * (sub_C0E3D00); online comes from the PMU charger interrupt */
        .battery = {
            .ocv_uv = bes_ocv_n67, .lookup = BES_OCV_UP_TO,
            .vbat_ch = 4, .vbat_ratio = 4900, .load_uv = 50000,
            .vbus_ch = 3, .vbus_ratio = 6882,
            .pmu_ac_on = true,
        },
    },
    {
        .name = "p65",
        .desc = "REDMI Watch 6",
        .chip = "best1502p",
        .product = "p65",
        /* lcdc log "rel_screen_w = 432, rel_screen_h = 514"; the rm690c0's
         * full-screen CASET is 24..455 */
        .panel_width = 432,
        .panel_height = 514,
        .panel_x_offset = 24,
        /* rm690c0 "vsn p01" (lcd probe sub_C4DE4DC accepts 0x200135/45,
         * 0x200235/45 for rm690c0, 0x3001xx for icna3312) */
        .dsi_panel_id = 0x200145,
        /* charger probe sub_C4DE158: I2C0 0x07, SY6103X then SC7061 */
        .charger = { TYPE_SY6103X, 0, 0x07, -1 },
        /* W1150 on I2C5 at 0x48 (sub_C4DEE58 / sub_C4EE348); without it the
         * probe's error path frees a static pointer and asserts */
        .light = { "w1150.als", 5, 0x48, -1 },
        /* touch (sub_C4DE8BC): Zinitix on I2C5 at 0x20, interrupt pin
         * 0x10C4C = GPIO 76, reset 0x1404D = GPIO 77; Hynitron fallback.
         * Hardware id and register version as in the driver's bundled
         * image (0x3C16DD64: 0x4502, fw 9.0, reg 0xD2), else it reflashes
         * the controller for 25 s on every boot */
        .touch = { TYPE_ZINITIX_TS, 5, 0x20, 76, false, 0x4502, 0xD2 },
        /* GPADC, read through /dev/adc by the battery driver: ch2 skin
         * NTC (sub_C4F6F90), ch3 shell NTC (sub_C4F6D98), both 100k
         * against a 200k pull-up to 1.8 V (table 0x3C17F730), so 600 mV =
         * 25 C (0 mV reads 126 C: shutdown "temperature too high");
         * ch4 VBUS / 6 (sub_C4F72D8, 4.0..5.5 V = plugged in), left at 0;
         * ch6 VBAT / 5 (sub_C4F70F0, boot shuts down below 3300 mV):
         * 800 mV = 4.0 V */
        .gpadc_mv = { [2] = 600, [3] = 600, [6] = 800 },
        /* soft gauge sub_C4F5C88: VBAT = 5 x ch6, VBUS = 6 x ch4 */
        .battery = {
            .ocv_uv = bes_ocv_p65, .lookup = BES_OCV_UP_TO,
            .vbat_ch = 6, .vbat_ratio = 5000,
            .vbus_ch = 4, .vbus_ratio = 6000,
            .pmu_ac_on = true,
        },
        /* NAND：固件分区表（512MB 档）resource first 0 nblocks 112640、
         * nand_data first 112640 nblocks 149504 */
        .nand_id = 0xC8F4C8,
        .nand_page_size = 2048,
        .nand_oob_size = 64,
        .nand_pages_per_block = 64,
        .nand_blocks = 4096,
        /* TE: hal_dsi_te_gpio_irq_handler pin (0x3C48EA0E at run time) */
        .lcd_te_gpio = 65,
        /* board_button table 0x3C2B4D40: button 0 GPIO 59 (active low),
         * button 1 "pin 82" = the PMU power key (best1503.c) */
        .keys = {
            { "home", Q_KEY_CODE_HOME, 59, true },
            { "down", Q_KEY_CODE_PGDN, 82, false },
        },
    },
    {
        .name = "q65",
        .desc = "REDMI Watch 7",
        .chip = "best1502p",
        .product = "q65",
        .panel_width = 432,
        .panel_height = 514,
        .panel_x_offset = 24,
        /* rm690c0_tm panel ID (DA=0x45, DB=0x02, DC=0x20) */
        .dsi_panel_id = 0x200245,
        /* charger probe: SC7061 on I2C0 at 0x07 (fallback HP4570) */
        .charger = { TYPE_SC7061, 0, 0x07, -1 },
        /* W1150 on I2C5 at 0x48 */
        .light = { "w1150.als", 5, 0x48, -1 },
        /* touch: Hynitron CST9317 on I2C5 at 0x5A, interrupt GPIO 76 (0x4C) */
        .touch = { TYPE_HYNITRON_TS, 5, 0x5A, 76, true, 0x9317 },
        /* GPADC: ch2 skin NTC, ch3 shell NTC (600 mV = 25 C), ch6 VBAT / 5 (800 mV = 4.0 V) */
        .gpadc_mv = { [2] = 600, [3] = 600, [6] = 800 },
        .battery = {
            .ocv_uv = bes_ocv_q65, .lookup = BES_OCV_UP_TO,
            .vbat_ch = 6, .vbat_ratio = 5000,
            .vbus_ch = 4, .vbus_ratio = 6000,
            .pmu_ac_on = true,
        },
        /* NAND: 512MB, resource first 0 nblocks 120320 */
        .nand_id = 0xC8F4C8,
        .nand_page_size = 2048,
        .nand_oob_size = 64,
        .nand_pages_per_block = 64,
        .nand_blocks = 4096,
        .lcd_te_gpio = 65,
        /* board_button: button 0 GPIO 59 (active low), button 1 pin 83 = PMU power key */
        .keys = {
            { "home", Q_KEY_CODE_HOME, 59, true },
            { "down", Q_KEY_CODE_PGDN, 83, false },
        },
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
        /* SGM41551 charger: board_charger_ic_initialize on I2C3 0x6B
         * (probed chip id 0 at 0x38); plug detect driven through the PMU's
         * AC_ON (hal_gpio 163) */
        .charger = { TYPE_SGM41551, 3, 0x6B, -1 },
        .battery = { .pmu_ac_on = true },
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
    { "best1502p", "BEST1502P (Cortex-M33)",
      ARM_CPU_TYPE_NAME("cortex-m33") },
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

/* ---- battery ---- */

/* the level the board's soft gauge shows for VBAT (uV), as it reads its
 * table */
static unsigned bes_battery_shown(const BesBoardBattery *b, uint64_t uv)
{
    const uint32_t *t = b->ocv_uv;
    uint64_t milli;
    unsigned i;

    if (b->lookup == BES_OCV_UP_TO) {
        for (i = 0; i < 100 && uv > t[i]; i++) {
        }
        return i;
    }
    if (uv < t[0]) {
        return 0;
    }
    for (i = 0; i < 100 && uv >= t[i + 1]; i++) {
    }
    milli = i == 100 ? 100000 :
            i * 1000 + (uv - t[i]) * 1000 / (t[i + 1] - t[i]);
    return MIN(milli * (b->ui_permille ? b->ui_permille : 1000) / 1000000,
               100);
}

/*
 * The VBAT divider's GPADC input (uV) at which the board's soft gauge
 * shows level%. The firmware reads whole millivolts at the pin, so this
 * is the whole millivolt nearest the middle of the level's VBAT band that
 * still reads as that level.
 */
static uint32_t bes_battery_vbat_pin_uv(const BesBoardBattery *b,
                                        unsigned level, bool charging)
{
    const uint32_t *t = b->ocv_uv;
    uint32_t load = charging ? 0 : b->load_uv;
    uint64_t mid;
    int64_t mv, best = -1;

    if (b->lookup == BES_OCV_UP_TO) {
        mid = level ? (t[level - 1] + t[level]) / 2 : t[0] - 20000;
    } else {
        /* table level at the middle of [level, level + 1) once scaled,
         * in 1/1000 percent */
        uint32_t milli = (level * 2 + 1) * 500ULL * 1000 /
                         (b->ui_permille ? b->ui_permille : 1000);
        unsigned i = milli / 1000;

        mid = i >= 100 ? t[100] + 10000 :
              t[i] + (uint64_t)(t[i + 1] - t[i]) * (milli % 1000) / 1000;
    }
    mid += load;
    for (mv = mid / b->vbat_ratio - 4; mv <= mid / b->vbat_ratio + 4; mv++) {
        uint64_t uv = mv * b->vbat_ratio;

        if (uv >= load && bes_battery_shown(b, uv - load) == level &&
            (best < 0 || llabs((int64_t)uv - (int64_t)mid) <
                         llabs(best * b->vbat_ratio - (int64_t)mid))) {
            best = mv;
        }
    }
    if (best < 0) {
        best = (mid + b->vbat_ratio / 2) / b->vbat_ratio;
    }
    /* a quarter millivolt over, so the conversion back truncates to it */
    return best * 1000 + 250;
}

/* drive the board from battery-level / battery-charging */
static void bes_battery_apply(BesMachineState *bms)
{
    const BesBoardBattery *b;
    BesBatteryLinks *l = &bms->battery;
    bool plugged = bms->battery_charging;
    bool full = bms->battery_level >= 100;

    if (!bms->board) {
        return;
    }
    b = &bms->board->battery;
    if (l->gauge) {
        /* a typical Li-ion cell, 3.5 V empty .. 4.4 V full */
        cw2215_set_battery(l->gauge, bms->battery_level,
                           3500 + 9 * bms->battery_level);
    }
    if (l->pmu && b->ocv_uv) {
        bes_pmu_set_gpadc_uv(l->pmu, b->vbat_ch,
                             bes_battery_vbat_pin_uv(b, bms->battery_level,
                                                     plugged));
    }
    if (l->pmu && b->vbus_ratio) {
        bes_pmu_set_gpadc_uv(l->pmu, b->vbus_ch,
                             plugged ? 5000000000ULL / b->vbus_ratio : 0);
    }
    if (b->pmu_ac_on) {
        qemu_set_irq(l->pmu ? qdev_get_gpio_in_named(DEVICE(l->pmu), "ac-on",
                                                     0) : l->ac_on,
                     plugged);
    }
    qemu_set_irq(l->charger_vbus, plugged);
    qemu_set_irq(l->charger_full, full);
    qemu_set_irq(l->plug, plugged != b->plug_active_low);
}

void bes_machine_attach_battery(MachineState *ms, const BesBatteryLinks *l)
{
    BesMachineState *bms = BES_MACHINE(ms);

    bms->battery = *l;
    bes_battery_apply(bms);
}

static void bes_get_battery_level(Object *obj, Visitor *v, const char *name,
                                  void *opaque, Error **errp)
{
    visit_type_uint8(v, name, &BES_MACHINE(obj)->battery_level, errp);
}

static void bes_set_battery_level(Object *obj, Visitor *v, const char *name,
                                  void *opaque, Error **errp)
{
    BesMachineState *bms = BES_MACHINE(obj);
    uint8_t level;

    if (!visit_type_uint8(v, name, &level, errp)) {
        return;
    }
    if (level > 100) {
        error_setg(errp, "battery-level is a percentage (0..100)");
        return;
    }
    bms->battery_level = level;
    bes_battery_apply(bms);
}

static bool bes_get_battery_charging(Object *obj, Error **errp)
{
    return BES_MACHINE(obj)->battery_charging;
}

static void bes_set_battery_charging(Object *obj, bool value, Error **errp)
{
    BesMachineState *bms = BES_MACHINE(obj);

    bms->battery_charging = value;
    bes_battery_apply(bms);
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
    bms->board = board;
    soc->init(ms, img, board);
}

static void bes_machine_reset(MachineState *ms, ResetType type)
{
    BesMachineState *bms = BES_MACHINE(ms);

    qemu_devices_reset(type);
    if (bms->soc->reset) {
        bms->soc->reset(ms);
    }
    bes_battery_apply(bms);
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

static void bes_machine_instance_init(Object *obj)
{
    BES_MACHINE(obj)->battery_level = 80;
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
    object_class_property_add(oc, "battery-level", "uint8",
                              bes_get_battery_level, bes_set_battery_level,
                              NULL, NULL);
    object_class_property_set_description(oc, "battery-level",
                                          "battery charge in percent "
                                          "(default 80), settable while "
                                          "running: the fuel gauge's charge, "
                                          "or the cell voltage the "
                                          "firmware's soft gauge reads it "
                                          "from");
    object_class_property_add_bool(oc, "battery-charging",
                                   bes_get_battery_charging,
                                   bes_set_battery_charging);
    object_class_property_set_description(oc, "battery-charging",
                                          "a charger is plugged in (VBUS, "
                                          "plug detect, charger IC status); "
                                          "settable while running");
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
        .instance_init = bes_machine_instance_init,
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
