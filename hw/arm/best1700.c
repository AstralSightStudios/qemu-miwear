/*
 * BEST1700 machine —— q63 AP 固件（Vela/NuttX）模拟
 *
 * 当前进度：boot 早期，卡在固件 chip id 校验（0x101C22B0 死循环）。
 *
 * 本文件只保留已实证的内容。所有外设（UART/TIMER/CMU/I2C/...）的
 * 地址、型号、中断号均未在 q63 固件上实证，一律不挂载；查实一个
 * 加一个，严禁从 best1503 照搬。
 *
 * 已实证内容：
 *   - CPU = Cortex-M55：固件含 MVE 指令（0xFF 前缀），M33 上
 *     Undefined Instruction 实证。
 *   - CPACR 使能 FPU/MVE：等效真机 BL2 遗留状态（真机 BL2 已开），
 *     模拟器从 AP entry 直接启动缺这一步。
 *   - flash 布局：全部来自 bes_image 解析镜像头。xip==cached==
 *     0x101C0000 对本固件是正常的（boot cfg FLASH_BASE=0x10000000
 *     + OTA_CODE_OFFSET=0x1C0000），不是 bug。
 *   - SRAM 4MB @0x20000000：镜像头 msp=0x2035ffc0。
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "hw/core/boards.h"
#include "hw/core/cpu.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-clock.h"
#include "hw/arm/armv7m.h"
#include "target/arm/cpu.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "hw/misc/bes_image.h"
#include "hw/misc/bes_flash.h"
#include "hw/misc/bes_cmu.h"
#include "hw/misc/bes_ispi.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "system/system.h"

#define BEST1700_SRAM_BASE       0x20000000
#define BEST1700_SRAM_SIZE       0x400000  /* q63 镜像头 msp=0x2035ffc0 */
/* RAMX 执行别名基址：执行轨迹 0x4BD000 = 0x00200000 + SRAM偏移0x2BD000
 * （q63 实证）；与 best1503 的 RAMX0_BASE 同值 */
#define BEST1700_RAMX_BASE       0x00200000

/*
 * 内核 NVIC 配置：q63 固件依据未查实，暂沿用常见配置。
 * TODO: 从固件（SCB 中断表大小/优先级写值）实证后修正。
 */
#define BEST1700_NUM_IRQ         80
#define BEST1700_NUM_PRIO_BITS   3

/* MCU_CMU @0x40000000：q63 实证——BSP hal_cmu_best1700.o 的
 * hal_cmu_clock_enable/disable/reset_set 等函数直接以此基址访问
 * HCLK/PCLK/OCLK ENABLE/DISABLE 三件套；boot ROM 亦经 CMU+0xC0+dev*4
 * 查询设备时钟状态、经 CMU+0x288 读自由运行计数器作延时。 */
#define BEST1700_CMU_BASE        0x40000000

/* ISPI 主控（控制器 0）@0x4000A000：q63 实证——hal_spi.o
 * .rodata.spi = {0x4000A000, 0x40008000, 0x40009000}，hal_ispi_send/
 * recv 固定用控制器 0；固件运行时在 SRAM 0x202C2280 自建的基址表
 * 与之一致（实测 dump）。PMU（片选 7）寄存器 0 经此读芯片标识。 */
#define BEST1700_ISPI_BASE       0x4000A000

/* CPU 时钟：频率值未实证（仅影响 SysTick 计数速率，不影响取指执行） */
#define BEST1700_SYSCLK_FRQ      208000000ULL

/*
 * 关于 0x0 伪 boot 向量表：
 * 真机上电从内置 boot ROM 启动，boot ROM 完成最小初始化后跳转固件
 * 入口。我们没有 boot ROM 镜像；0x0 伪向量表（初始 MSP + 固件入口）
 * 是 rehosting 中模拟 boot ROM 跳转效果的标准替代手段，不修改固件
 * 任何字节。
 */
static BesImage g_img;
static MemoryRegion g_bootvec;
static ARMCPU *g_cpu;

/* -M best1700,flash-image=<file>：完整 flash 镜像作为 bes_flash 后端
 * 并在退出时写回；缺省则由 -kernel 固件按 OTA_CODE_OFFSET 构造后端
 * 内容。 */
static char *g_flash_image;

static char *best1700_get_flash_image(Object *obj, Error **errp)
{
    return g_strdup(g_flash_image);
}

static void best1700_set_flash_image(Object *obj, const char *val,
                                     Error **errp)
{
    g_free(g_flash_image);
    g_flash_image = g_strdup(val);
}

/*
 * CPU 对象不随系统 reset 自动复位（hw/arm/boot.c:1186 注释），
 * 且 armv7m realize 阶段的初始 reset 发生在内存建好之前。
 * 注册回调保证系统 reset 时 CPU 重新取向量（此时 bootvec 已就位）。
 */
static void best1700_cpu_reset(void *opaque)
{
    ARMCPU *cpu = ARM_CPU(opaque);
    CPUARMState *env = &cpu->env;

    cpu_reset(CPU(opaque));
    /* 真机 BL2 已使能 FPU/MVE（CPACR.CP10/CP11）；模拟器从 AP entry
     * 直接启动缺了这一步，MVE 指令（0xFF 前缀）会 Undefined
     * Instruction。复位后直接使能（等效 BL2 遗留状态，不碰固件）。 */
    env->v7m.cpacr[M_REG_S] |= 0x00F00000;
    env->v7m.cpacr[M_REG_NS] |= 0x00F00000;
}

static void best1700_bootvec_reset(void *opaque)
{
    /* 每次系统 reset 重填（在 CPU reset 之前执行），等效真机 boot ROM */
    uint32_t *vec = memory_region_get_ram_ptr(&g_bootvec);

    vec[0] = g_img.msp;
    vec[1] = g_img.entry | 1;   /* Thumb */
}

static void best1700_init(MachineState *machine)
{
    MemoryRegion *sysmem = get_system_memory();
    static MemoryRegion flash, flash_cached, flash_nc, sram, ramx;
    Clock *sysclk;
    Error *err = NULL;
    DeviceState *armv7m;

    /* ---- parse firmware (single source of truth for all layout) ---- */
    if (!machine->kernel_filename) {
        error_report("best1700: -kernel <firmware image> is required "
                     "(e.g. vela_ap.bin)");
        exit(1);
    }
    if (!bes_image_load(&g_img, machine->kernel_filename, &err)) {
        error_report_err(err);
        exit(1);
    }
    info_report("best1700: firmware: xip=0x%08x cached=0x%08x nc=0x%08x "
                "entry=0x%08x msp=0x%08x size=0x%zx",
                g_img.xip_base, g_img.cached_base, g_img.nc_base,
                g_img.entry, g_img.msp, g_img.size);

    sysclk = clock_new(OBJECT(machine), "SYSCLK");
    clock_set_hz(sysclk, BEST1700_SYSCLK_FRQ);

    /* ---- CPU + NVIC + SysTick ---- */
    armv7m = qdev_new(TYPE_ARMV7M);
    object_property_add_child(OBJECT(machine), "armv7m", OBJECT(armv7m));
    qdev_prop_set_uint32(armv7m, "num-irq", BEST1700_NUM_IRQ);
    qdev_prop_set_uint8(armv7m, "num-prio-bits", BEST1700_NUM_PRIO_BITS);
    qdev_prop_set_string(armv7m, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m55"));
    qdev_connect_clock_in(armv7m, "cpuclk", sysclk);
    qdev_connect_clock_in(armv7m, "refclk", sysclk);
    object_property_set_link(OBJECT(armv7m), "memory",
                             OBJECT(sysmem), &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(armv7m), &error_fatal);
    g_cpu = ARMV7M(armv7m)->cpu;

    /* ---- flash：bes_flash 设备持有存储后端，XIP/cached/NC 三别名共享 ----
     * 真机同一物理 flash 有多个总线窗口，固件经不同窗口访问代码与
     * 数据，故按 bes_image 解析出的三基址挂三个别名。固件写入后端
     * 偏移 OTA_CODE_OFFSET（真机 flash 布局，解析自 boot info）；
     * -M flash-image=<file> 提供其余内容，-kernel 固件每次启动照样
     * 装入 AP 分区（等同 OTA 升级），同一镜像可跨固件版本使用。 */
    DeviceState *fc = qdev_new(TYPE_BES_FLASH);
    {
        uint32_t flash_xip_start = g_img.xip_base - g_img.ota_code_offset;
        uint32_t flash_cached_start = g_img.cached_base - g_img.ota_code_offset;
        uint32_t flash_nc_start = g_img.nc_base - g_img.ota_code_offset;

        qdev_prop_set_uint32(fc, "size", g_img.flash_size);
        if (g_flash_image) {
            qdev_prop_set_string(fc, "image", g_flash_image);
        }
        sysbus_realize_and_unref(SYS_BUS_DEVICE(fc), &error_fatal);

        memcpy(bes_flash_data(BES_FLASH(fc)) + g_img.ota_code_offset,
               g_img.data, g_img.size);

        memory_region_init_alias(&flash, NULL, "best1700.flash_xip",
                                 bes_flash_storage(BES_FLASH(fc)), 0,
                                 g_img.flash_size);
        memory_region_add_subregion(sysmem, flash_xip_start, &flash);
        memory_region_init_alias(&flash_cached, NULL, "best1700.flash_cached",
                                 bes_flash_storage(BES_FLASH(fc)), 0,
                                 g_img.flash_size);
        memory_region_add_subregion(sysmem, flash_cached_start, &flash_cached);
        memory_region_init_alias(&flash_nc, NULL, "best1700.flash_nc",
                                 bes_flash_storage(BES_FLASH(fc)), 0,
                                 g_img.flash_size);
        memory_region_add_subregion(sysmem, flash_nc_start, &flash_nc);
    }

    /* flash 控制器 MMIO：q63 固件未实证控制器地址，暂不映射
     * （bes_flash 设备仅作存储后端使用）。 */

    /* ---- SRAM ---- */
    memory_region_init_ram(&sram, NULL, "best1700.sram", BEST1700_SRAM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, BEST1700_SRAM_BASE, &sram);
    /* RAMX 执行别名：q63 实证——固件把 sram_text/boot_text_sram 复制到
     * SRAM VMA（如 0x202BD000，偏移 0x2BD000）后，经 RAMX 窗口执行，
     * 执行轨迹 0x4BD000 = 0x00200000 + 0x2BD000 精确吻合。sram_text
     * 由固件 start 自己复制（与 1503 的 startup_main.S 流程相同），
     * 不需要模拟器代劳。 */
    memory_region_init_alias(&ramx, NULL, "best1700.ramx", &sram, 0,
                             BEST1700_SRAM_SIZE);
    memory_region_add_subregion(sysmem, BEST1700_RAMX_BASE, &ramx);

    /* ---- .data 段预加载（模拟真机 BL2 的镜像加载） ----
     * 真机 BL2 跳转入库前已完成 .data 重定位。地址全部来自镜像头
     * 解析（data_start/data_end/data_lma），不写死。 */
    {
        uint32_t cached_start = g_img.cached_base - g_img.ota_code_offset;
        memcpy(memory_region_get_ram_ptr(&sram) +
               (g_img.data_start - BEST1700_SRAM_BASE),
               bes_flash_data(BES_FLASH(fc)) +
               (g_img.data_lma - cached_start),
               g_img.data_end - g_img.data_start);
    }

    /* ---- 伪 boot 向量表 @0x0（模拟 boot ROM 跳转，不碰固件） ---- */
    memory_region_init_ram(&g_bootvec, NULL, "best1700.bootvec", 0x1000,
                           &error_fatal);
    memory_region_set_readonly(&g_bootvec, true);
    memory_region_add_subregion(sysmem, 0, &g_bootvec);

    /* boot ROM 低址 RAM（0x1000-0x200000）：真机 boot ROM 在此留有栈/
     * 数据（boot_text_sram 访问的 0x94B40 即在此区，实证），缺失时其
     * 写读自检被总线丢弃而死循环。 */
    {
        static MemoryRegion bootram;
        memory_region_init_ram(&bootram, NULL, "best1700.bootram",
                               0x200000 - 0x1000, &error_fatal);
        memory_region_add_subregion(sysmem, 0x1000, &bootram);
    }

    qemu_register_reset(best1700_bootvec_reset, NULL);
    qemu_register_reset(best1700_cpu_reset, g_cpu);

    /* ---- MCU_CMU @0x40000000（q63 实证，见上方宏注释） ----
     * intersys/BT 邮箱线未连接：BT 子系统未建模，用到再实证。 */
    {
        DeviceState *cmu = qdev_new(TYPE_BES_CMU);

        sysbus_realize_and_unref(SYS_BUS_DEVICE(cmu), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(cmu), 0, BEST1700_CMU_BASE);
    }

    /* ---- ISPI 主控 @0x4000A000（q63 实证，见上方宏注释） ---- */
    {
        DeviceState *ispi = qdev_new(TYPE_BES_ISPI);

        sysbus_realize_and_unref(SYS_BUS_DEVICE(ispi), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(ispi), 0, BEST1700_ISPI_BASE);
    }

    /*
     * 其余外设：一律不挂。除 CMU / ISPI 外尚未实证任何外设的
     * 地址/型号/中断号。待办（查实后才允许添加）：
     *   - ISPI 控制器 1/2（0x40008000/0x40009000，同表实证，
     *     固件用到时再挂）
     *   - UART / TIMER / NAND 控制器等
     */
}

static void best1700_machine_init(MachineClass *mc)
{
    static const char * const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-m55"),
        NULL
    };

    mc->desc = "BEST1700 (Cortex-M55, Vela AP, q63)";
    mc->init = best1700_init;
    mc->valid_cpu_types = valid_cpu_types;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m55");
    mc->ignore_memory_transaction_failures = true;

    object_class_property_add_str(OBJECT_CLASS(mc), "flash-image",
                                  best1700_get_flash_image,
                                  best1700_set_flash_image);
    object_class_property_set_description(OBJECT_CLASS(mc), "flash-image",
                                          "backing image file for the boot "
                                          "flash (loaded at startup, written "
                                          "back on exit; the -kernel firmware "
                                          "is installed into its AP "
                                          "partition)");
}

DEFINE_MACHINE_ARM("best1700", best1700_machine_init)
