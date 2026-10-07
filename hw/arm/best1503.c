/*
 * BEST1503 (BES2700iMP) machine —— 小米手环10 AP 固件（Vela/NuttX）模拟
 *
 * 设计约束（DESIGN.md）：
 *   - 所有固件布局来自 bes_image 解析，本文件不写死固件地址
 *   - 外设基地址集中在下方地址表（出处：SDK plat_addr_map_best1503.h）
 *   - 未实现外设一律 unimp 占位打日志，严禁猜值
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
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "hw/misc/unimp.h"
#include "hw/misc/bes_image.h"
#include "hw/misc/bes_aoncmu.h"
#include "hw/misc/bes_flash.h"
#include "hw/misc/bes_psram.h"
#include "hw/misc/bes_cmu.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "system/system.h"

/*
 * 外设地址表（SDK: platform/hal/best1503/plat_addr_map_best1503.h:218-257）
 * UART/TIMER 型号依据：
 *   UART  = ARM PL011（SDK platform/hal/reg_uart.h:22-54 标准布局）
 *   TIMER = ARM SP804（SDK platform/hal/reg_timer.h:24-40 标准布局）
 * 中断号依据：SDK platform/cmsis/inc/best1503.h IRQn 表
 *   UART0/1/2 = 17/18/26 (:81/:82/:91)
 *   MCU_TIMER00/01 = 12/13 (:76/:77)，MCU_TIMER10/11 = 27/28 (:92/:93)
 */
#define BEST1503_MCU_TIMER0_BASE 0x40002000
#define BEST1503_MCU_TIMER1_BASE 0x40003000
#define BEST1503_UART0_BASE      0x4000B000
#define BEST1503_UART1_BASE      0x4000C000
#define BEST1503_UART2_BASE      0x4000D000
#define BEST1503_AON_CMU_BASE    0x40080000
/* flash0 控制器：SDK plat_addr_map_best1503.h:265 REAL_FLASH_CTRL_BASE；
 * flash1 控制器 :261 REAL_FLASH1_CTRL_BASE=0x40500000 —— AP 固件运行期间
 * 不访问 flash1（resource romfs 在独立大容量存储上，挂载方式待固件访问时
 * 经 unimp 日志识别后再建模），故暂不挂 flash1。 */
#define BEST1503_FLASH_CTRL_BASE  0x40140000
/* PSRAM PHY：SDK plat_addr_map_best1503.h:284 候选值，经固件运行时行为
 * 证实（PHY 校准时序公式 0x60/0x64 在 48MHz 下逐位吻合，见 bes_psram.c） */
#define BEST1503_PSRAM_PHY_BASE  0x40168000
/* PSRAM MC（MGR 命令引擎）：固件逆向确认（sub_C0E5228 MR 写/
 * sub_C0E5308 读，寄存器语义与 SDK reg_psram_mc_v4.h 吻合） */
#define BEST1503_PSRAM_MC_BASE   0x40160000
/* MCU_CMU：SDK plat_addr_map_best1503.h（CMU_BASE）；reg 布局
 * reg_cmu_best1503.h:21-156 */
#define BEST1503_CMU_BASE        0x40000000
/* PSRAM（SiP 封装 PSRAMUHS）：cached 0x3C000000 / NC 0x38000000 —— 固件
 * sub_C0C0AC8 段复制目标（0x3C000000）与 sub_C0E5634 校准 pattern 地址
 * （0x38000000/0x38100000）证实；SDK 里的 0x1C000000/0x18000000 占位值
 * 都是错的。大小 32MB：固件 sub_C700128 堆检查引用堆顶标记
 * MEMORY[0x3DFC0000]（= 0x3C000000 + 0x1FC0000），SDK 的 8MB 占位值
 * 会让堆校验全部误判并引发日志洪峰。 */
#define BEST1503_PSRAM_BASE      0x3C000000
#define BEST1503_PSRAM_NC_BASE   0x38000000
#define BEST1503_PSRAM_SIZE      0x02000000

#define BEST1503_UART0_IRQ       17
#define BEST1503_UART1_IRQ       18
#define BEST1503_UART2_IRQ       26
/* hal_sys_timer 走 slow timer（MCU_TIMER0）子定时器 1（SDK hal_timer.c:53-54
 * SLOW_TIMER_SUB_ID_WITH_IRQ=1）→ MCU_TIMER01_IRQn=13。
 * BES 硬件把 dual-timer 两子定时器分接两个 NVIC 中断；QEMU sp804 只有一路
 * 合并输出，故连子定时器 1 对应的号（近似，注释留存）。 */
#define BEST1503_MCU_TIMER0_IRQ  13
#define BEST1503_MCU_TIMER1_IRQ  28   /* MCU_TIMER11_IRQn（fast timer 同理） */
#define BEST1503_AON_TIMER0_IRQ  45   /* AON_TIMER00_IRQn（best1503.h:110） */
#define BEST1503_AON_TIMER0_BASE 0x40084000

/* 外设窗口整体兜底（未建模区域侦察用） */
#define BEST1503_PERIPH_BASE     0x40000000
#define BEST1503_PERIPH_SIZE     0x10000000

/*
 * SRAM：plat_addr_map_best1503.h:44-63
 *   RAM0_BASE=0x20000000，RAM0..RAM3 连续，总量 0x160000（1.4MB，
 *   由 maskrom dump 大小 1441792 证实，见该文件 :53-60 注释）。
 * RAMX 执行别名：RAMX0_BASE=0x00200000（:45），大小同 SRAM。
 */
#define BEST1503_SRAM_BASE       0x20000000
#define BEST1503_SRAM_SIZE       0x160000
#define BEST1503_RAMX_BASE       0x00200000

/*
 * 时钟：
 *   系统时钟最大档 208MHz（SDK hal_cmu.h:70-79 HAL_CMU_FREQ_T 枚举含 208M）
 *   MCU_TIMER0 = slow = 16KHz（SDK hal_cmu_best1503.c:599 注释 "16K"）
 *   MCU_TIMER1 = fast = 6.5MHz（SDK hal_cmu_best1503.c:614 注释 "6.5M"）
 */
#define BEST1503_SYSCLK_FRQ      208000000ULL
#define BEST1503_SLOW_TIMER_FRQ  16000
#define BEST1503_FAST_TIMER_FRQ  6500000
/* AON_TIMER0：AON 域 slow 档 32K（hal_cmu.h:71 HAL_CMU_FREQ_32K；
 * 晶振精度 32768/32000 待实测校准，不影响延时推进） */
#define BEST1503_AON_TIMER_FRQ   32768

/* best1503.h:215 __NVIC_PRIO_BITS=3；:145 USER_IRQn_QTY=78 → 取整 80 */
#define BEST1503_NUM_IRQ         80
#define BEST1503_NUM_PRIO_BITS   3

/*
 * 关于 0x0 伪 boot 向量表的依据：
 * 真机上电从内置 mask ROM 启动（ROMX_BASE=0x00020000，plat_addr_map:33），
 * mask ROM 完成最小初始化后跳转固件入口 stub。我们没有 mask ROM 镜像，
 * 无法真实模拟 boot ROM；0x0 伪向量表（初始 MSP + 固件入口）是 rehosting
 * 中模拟 boot ROM 跳转效果的标准替代手段，不修改固件任何字节。
 */
static BesImage g_img;
static MemoryRegion g_bootvec;
static ARMCPU *g_cpu;

/* -M best1503,flash-image=<file>：完整 16MB flash 镜像（含 BL2/AP/参数区），
 * 作为 bes_flash 后端并在退出时写回；缺省则由 -kernel 固件按
 * OTA_CODE_OFFSET 构造后端内容。 */
static char *g_flash_image;

static char *best1503_get_flash_image(Object *obj, Error **errp)
{
    return g_strdup(g_flash_image);
}

static void best1503_set_flash_image(Object *obj, const char *val,
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
static void best1503_cpu_reset(void *opaque)
{
    cpu_reset(CPU(opaque));
}

static void best1503_bootvec_reset(void *opaque)
{
    /* 每次系统 reset 重填（在 CPU reset 之前执行），等效真机 boot ROM */
    uint32_t *vec = memory_region_get_ram_ptr(&g_bootvec);

    vec[0] = g_img.msp;
    vec[1] = g_img.entry | 1;   /* Thumb */
}

static void best1503_init(MachineState *machine)
{
    MemoryRegion *sysmem = get_system_memory();
    static MemoryRegion flash, flash_cached, flash_nc, flash_xnc, sram, ramx;
    static MemoryRegion psram, psram_nc;
    Clock *sysclk;
    Error *err = NULL;
    DeviceState *armv7m;

    /* ---- parse firmware (single source of truth for all layout) ---- */
    if (!machine->kernel_filename) {
        error_report("best1503: -kernel <firmware image> is required "
                     "(e.g. vela_ap.bin)");
        exit(1);
    }
    if (!bes_image_load(&g_img, machine->kernel_filename, &err)) {
        error_report_err(err);
        exit(1);
    }
    info_report("best1503: firmware: xip=0x%08x cached=0x%08x nc=0x%08x "
                "entry=0x%08x msp=0x%08x size=0x%zx",
                g_img.xip_base, g_img.cached_base, g_img.nc_base,
                g_img.entry, g_img.msp, g_img.size);

    sysclk = clock_new(OBJECT(machine), "SYSCLK");
    clock_set_hz(sysclk, BEST1503_SYSCLK_FRQ);

    /* ---- CPU + NVIC + SysTick ---- */
    armv7m = qdev_new(TYPE_ARMV7M);
    object_property_add_child(OBJECT(machine), "armv7m", OBJECT(armv7m));
    qdev_prop_set_uint32(armv7m, "num-irq", BEST1503_NUM_IRQ);
    qdev_prop_set_uint8(armv7m, "num-prio-bits", BEST1503_NUM_PRIO_BITS);
    qdev_prop_set_string(armv7m, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m33"));
    qdev_prop_set_bit(armv7m, "enable-bitband", true);
    qdev_connect_clock_in(armv7m, "cpuclk", sysclk);
    qdev_connect_clock_in(armv7m, "refclk", sysclk);
    object_property_set_link(OBJECT(armv7m), "memory",
                             OBJECT(sysmem), &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(armv7m), &error_fatal);
    g_cpu = ARMV7M(armv7m)->cpu;

    /* ---- flash：bes_flash 设备持有存储后端，XIP/cached/NC 三别名共享 ----
     * 真机同一物理 flash 有多个总线窗口（SDK plat_addr_map_best1503.h:
     * 205-208：cached 0x2C/NC 0x28/XIP 0x0C/FLASHX_NC 0x08），固件经不同
     * 窗口访问代码与数据，故按 bes_image 解析出的三基址挂三个别名。
     * 固件写入后端偏移 OTA_CODE_OFFSET（真机 flash 布局，解析自 boot
     * info）；若经 -M flash-image=<file> 提供完整 flash 镜像，则镜像
     * 文件即后端内容（含 BL2/AP/参数区），不再单独写入固件。 */
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
        sysbus_mmio_map(SYS_BUS_DEVICE(fc), 0, BEST1503_FLASH_CTRL_BASE);

        if (!g_flash_image) {
            memcpy(bes_flash_data(BES_FLASH(fc)) + g_img.ota_code_offset,
                   g_img.data, g_img.size);
        }

        memory_region_init_alias(&flash, NULL, "best1503.flash_xip",
                                 bes_flash_storage(BES_FLASH(fc)), 0,
                                 g_img.flash_size);
        memory_region_add_subregion(sysmem, flash_xip_start, &flash);
        memory_region_init_alias(&flash_cached, NULL, "best1503.flash_cached",
                                 bes_flash_storage(BES_FLASH(fc)), 0,
                                 g_img.flash_size);
        memory_region_add_subregion(sysmem, flash_cached_start, &flash_cached);
        memory_region_init_alias(&flash_nc, NULL, "best1503.flash_nc",
                                 bes_flash_storage(BES_FLASH(fc)), 0,
                                 g_img.flash_size);
        memory_region_add_subregion(sysmem, flash_nc_start, &flash_nc);

        /* 第四窗口 FLASHX_NC：SDK plat_addr_map_best1503.h:207-208
         * REAL_FLASHX_BASE(0x0C000000)/REAL_FLASHX_NC_BASE(0x08000000)
         * 差 0x4000000（与 FLASH/FLASH_NC 对一致）。固件 sub_C0C2054
         * 读此窗口 +0x2000 的启动标志位（BL2/参数区），缺失时读到 0
         * 会被当作"首次启动"而走完整 early-init 路径。 */
        memory_region_init_alias(&flash_xnc, NULL, "best1503.flash_xnc",
                                 bes_flash_storage(BES_FLASH(fc)), 0,
                                 g_img.flash_size);
        memory_region_add_subregion(sysmem, flash_xip_start - 0x4000000,
                                    &flash_xnc);
    }

    /* ---- SRAM：SDK 确认物理总量 1.4MB（不依赖固件布局计算） ---- */
    memory_region_init_ram(&sram, NULL, "best1503.sram", BEST1503_SRAM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, BEST1503_SRAM_BASE, &sram);
    /* RAMX 执行别名（QEMU alias 原生同步） */
    memory_region_init_alias(&ramx, NULL, "best1503.ramx", &sram, 0,
                             BEST1503_SRAM_SIZE);
    memory_region_add_subregion(sysmem, BEST1503_RAMX_BASE, &ramx);

    /* ---- .data 段预加载（模拟真机 BL2 的镜像加载） ----
     * 真机 BL2 把 AP 镜像写入 flash 后跳转入库前已完成 .data 重定位；
     * 固件 start 尾部的复制循环只是冗余保险（SDK startup_main.S 同一
     * 流程）。缺了它，BootInit（sub_C0C0B48）里 early DSI init 的
     * TR_INFO -> NuttX syslog -> circbuf UDIV(tail % size, size=0) 会在
     * SCB_CCR.DIV_0_TRP（sub_C0C09CC 置位）下直接 HardFault。 */
    {
        uint32_t cached_start = g_img.cached_base - g_img.ota_code_offset;
        memcpy(memory_region_get_ram_ptr(&sram) +
               (g_img.data_start - BEST1503_SRAM_BASE),
               bes_flash_data(BES_FLASH(fc)) +
               (g_img.data_lma - cached_start),
               g_img.data_end - g_img.data_start);
    }

    /* ---- PSRAM：cached 窗口 + NC 别名（固件 sub_C0C0AC8 段复制证实） ---- */
    memory_region_init_ram(&psram, NULL, "best1503.psram",
                           BEST1503_PSRAM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, BEST1503_PSRAM_BASE, &psram);
    memory_region_init_alias(&psram_nc, NULL, "best1503.psram_nc",
                             &psram, 0, BEST1503_PSRAM_SIZE);
    memory_region_add_subregion(sysmem, BEST1503_PSRAM_NC_BASE,
                                &psram_nc);

    /* ---- 伪 boot 向量表 @0x0（模拟 boot ROM 跳转，不碰固件） ---- */
    memory_region_init_ram(&g_bootvec, NULL, "best1503.bootvec", 0x1000,
                           &error_fatal);
    memory_region_set_readonly(&g_bootvec, true);
    memory_region_add_subregion(sysmem, 0, &g_bootvec);
    qemu_register_reset(best1503_bootvec_reset, NULL);
    qemu_register_reset(best1503_cpu_reset, g_cpu);

    /* ---- 外设：仅挂有 SDK 依据（布局+型号+中断号）的设备 ---- */
    {
        static const struct {
            uint32_t base;
            int irq;
        } uarts[3] = {
            { BEST1503_UART0_BASE, BEST1503_UART0_IRQ },
            { BEST1503_UART1_BASE, BEST1503_UART1_IRQ },
            { BEST1503_UART2_BASE, BEST1503_UART2_IRQ },
        };
        static const struct {
            uint32_t base;
            uint32_t freq;
            int irq;
        } timers[] = {
            { BEST1503_MCU_TIMER0_BASE, BEST1503_SLOW_TIMER_FRQ,
              BEST1503_MCU_TIMER0_IRQ },
            { BEST1503_MCU_TIMER1_BASE, BEST1503_FAST_TIMER_FRQ,
              BEST1503_MCU_TIMER1_IRQ },
            { BEST1503_AON_TIMER0_BASE, BEST1503_AON_TIMER_FRQ,
              BEST1503_AON_TIMER0_IRQ },
        };
        DeviceState *nvic = armv7m;   /* armv7m 容器的 gpio-in 即 NVIC 外部中断线 */
        int i;

        for (i = 0; i < 3; i++) {
            DeviceState *uart = qdev_new("pl011");
            Chardev *chr = serial_hd(i);
            if (chr) {
                qdev_prop_set_chr(uart, "chardev", chr);
            }
            qdev_connect_clock_in(uart, "clk", sysclk);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(uart), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(uart), 0, uarts[i].base);
            sysbus_connect_irq(SYS_BUS_DEVICE(uart), 0,
                               qdev_get_gpio_in(nvic, uarts[i].irq));
        }
        for (i = 0; i < ARRAY_SIZE(timers); i++) {
            DeviceState *tmr = qdev_new("sp804");
            qdev_prop_set_uint32(tmr, "freq0", timers[i].freq);
            qdev_prop_set_uint32(tmr, "freq1", timers[i].freq);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(tmr), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(tmr), 0, timers[i].base);
            sysbus_connect_irq(SYS_BUS_DEVICE(tmr), 0,
                               qdev_get_gpio_in(nvic, timers[i].irq));
        }
    }

    /* ---- 外设窗口兜底：unimp 侦察日志 ---- */
    create_unimplemented_device("bes.periph", BEST1503_PERIPH_BASE,
                                BEST1503_PERIPH_SIZE);

    /* AON_CMU @0x40080000（后注册覆盖 unimp 窗口） */
    {
        DeviceState *aoncmu = qdev_new(TYPE_BES_AONCMU);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(aoncmu), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(aoncmu), 0, BEST1503_AON_CMU_BASE);
    }

    /* PSRAM 控制器对 @0x40168000（PHY）+ @0x40160000（MC/MGR 引擎） */
    {
        DeviceState *psramc = qdev_new(TYPE_BES_PSRAM);
        qdev_prop_set_uint64(psramc, "psram-ptr",
                             (uint64_t)(uintptr_t)
                             memory_region_get_ram_ptr(&psram));
        qdev_prop_set_uint32(psramc, "psram-size", BEST1503_PSRAM_SIZE);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(psramc), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(psramc), 0, BEST1503_PSRAM_PHY_BASE);
        sysbus_mmio_map(SYS_BUS_DEVICE(psramc), 1, BEST1503_PSRAM_MC_BASE);
    }

    /* MCU_CMU @0x40000000（覆盖 unimp 窗口） */
    {
        DeviceState *cmu = qdev_new(TYPE_BES_CMU);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(cmu), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(cmu), 0, BEST1503_CMU_BASE);
    }
}

static void best1503_machine_init(MachineClass *mc)
{
    static const char * const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-m33"),
        NULL
    };

    mc->desc = "BEST1503 / BES2700iMP (Cortex-M33, Xiaomi Vela AP)";
    mc->init = best1503_init;
    mc->valid_cpu_types = valid_cpu_types;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m33");
    mc->ignore_memory_transaction_failures = true;

    object_class_property_add_str(OBJECT_CLASS(mc), "flash-image",
                                  best1503_get_flash_image,
                                  best1503_set_flash_image);
    object_class_property_set_description(OBJECT_CLASS(mc), "flash-image",
                                          "backing image file for the 16MB "
                                          "boot flash (loaded at startup, "
                                          "written back on exit)");
}

DEFINE_MACHINE_ARM("best1503", best1503_machine_init)
