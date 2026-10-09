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
#include "qemu/log.h"
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
#include "hw/misc/bes_pmu.h"
#include "hw/misc/bes_i2c.h"
#include "hw/misc/bes_lcdc.h"
#include "hw/misc/bes_dma.h"
#include "hw/misc/bes_btss.h"
#include "hw/misc/bes_gpio.h"
#include "hw/misc/bes_psc.h"
#include "hw/input/zinitix_ts.h"
#include "hw/misc/vsi_vglite.h"
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
/* flash 控制器：固件控制器表 0x2009D8F0（id0=0x40140000，id1=0x40500000；
 * 与 SDK plat_addr_map_best1503.h:265/:261 一致）。
 * flash0 = 代码 NOR；flash1 = SPI NAND（/dev/resource + /dev/nand_data，
 * 固件 bes_nand_initialize sub_C19FDF0 以控制器 id 1 调 nandflash_init）。 */
#define BEST1503_FLASH_CTRL_BASE  0x40140000
#define BEST1503_FLASH1_CTRL_BASE 0x40500000
/* flash1 NC 窗口：NAND 驱动经 0x30000000 + column 直读页缓存（坏块标记
 * sub_C2215FC、非对齐尾部 sub_C221680 里的立即数 0x30000000）。 */
#define BEST1503_FLASH1_NC_BASE   0x30000000
/* SPI NAND 芯片：固件 ID 表 0x2CCD4134 第 1 项（0x2CCD3FE4）：
 * ID c8-f4-c8，2048+64B 页，128KB 块，512MB。选 512MB 的依据：
 * bes_board_nand_initialize（sub_C20AB44）按容量选分区表，只有 512MB 表
 * 的 resource 分区（0x19000 页 = 200MB，表 0x2CCC72C0）装得下
 * vela_resource.bin（romfs 头标 171754096 字节）；128/256MB 表只有
 * 0x10E00 页 = 141.5MB。整机实际贴片型号未知，表中 512MB/2KB 页的
 * 候选里取首项。 */
#define BEST1503_NAND_ID          0xC8F4C8
#define BEST1503_NAND_PAGE        2048
#define BEST1503_NAND_OOB         64
#define BEST1503_NAND_PPB         64
#define BEST1503_NAND_BLOCKS      4096
/* 通用 DMA ×2：固件 hal_dma 基址表 0x2CCDA91C、NVIC 号表 0x2CCDA918
 * （hal_dma_open sub_C22E8D0 装向量）。flash1 的 DMA 请求 = 外设号 28
 * → DMA1 请求线 6（外设表 0x2CCDA924 下标 6，数据口 0x40500008）。 */
#define BEST1503_DMA0_BASE        0x40120000
#define BEST1503_DMA1_BASE        0x40130000
#define BEST1503_DMA0_IRQ         3
#define BEST1503_DMA1_IRQ         4
#define BEST1503_FLASH1_DMA_REQ   6
/* BT 子系统（BT 核本体不模拟，bes.btss 扮演邮箱对端 + 空闲 HCI 控制器）：
 *   BT_CMU 0xD0330000（SDK；固件 RX 完成写 +0x54，sub_C2315BC）
 *   邮箱指针表 0xC0004000（固件 hal_intersys 表 0x2CCDACC0/0x2CCDACC8）
 *   BT 交换内存 0xD0210000/64KB（SDK BT_EXCH_MEM；AP 实测不写） */
#define BEST1503_BT_CMU_BASE      0xD0330000
#define BEST1503_BT_RAM_BASE      0xC0000000
#define BEST1503_BT_MBOX_TABLE    0xC0004000
#define BEST1503_BT_EXCH_BASE     0xD0210000
#define BEST1503_BT_EXCH_SIZE     0x10000
/* GPIO v2 ×3 bank（固件 bank 表 0x2CCDAB08）+ PSC GPIO 中断块
 * （hal_psc_gpio_irq_enable/get_status：PSC 0x40085080..0x40085094），
 * 共用 GPIO_IRQn=44（hal_gpio_setup_irq sub_C22F580）。 */
#define BEST1503_GPIO0_BASE       0x40081000
#define BEST1503_GPIO1_BASE       0x40089000
#define BEST1503_GPIO2_BASE       0x4008A000
#define BEST1503_PSC_GPIO_IRQ     0x40085080
/* PSC（电源时序控制器；固件 hal_psc_* sub_C0C1970 等以此为基址）：
 * MCU 掉电/唤醒见 bes_psc.c；+0x80 起的 GPIO 中断块由 bes_gpio 覆盖映射 */
#define BEST1503_PSC_BASE         0x40085000
#define BEST1503_GPIO_IRQ         44
/* LCDC：NVIC 67（sub_C231DE4）；面板 TE 接 GPIO 47（板级：固件
 * LCD TE 引脚配置 sub_C0E3DF0 返回 47，经 sub_C232740 配成边沿中断）；
 * 面板 336x480（固件 LCDC 初始化 sub_C225BA4 的层/输出尺寸）。 */
#define BEST1503_LCDC_BASE        0x40200000
#define BEST1503_LCDC_IRQ         67
#define BEST1503_LCD_TE_GPIO      47

/* 触摸：Zinitix ZTW662（真机日志 chip id 0xE623）。bes_ts_zinitix_setup
 * (sub_C3F84CC) 打开 I2C 总线 5；板级配置 0x2CD0605C = {I2C 地址 0x20,
 * 400kHz, INT pinset 0x1004C (GPIO 76), RESET pinset 0x1404E (GPIO 78)}。 */
#define BEST1503_TP_I2C_BUS       5
#define BEST1503_TP_I2C_ADDR      0x20
#define BEST1503_TP_INT_GPIO      76
#define BEST1503_PANEL_W          336
#define BEST1503_PANEL_H          480
/* 2D GPU（VeriSilicon VGLite，GCNanoUltraV 类）：寄存器 0x40220000
 * （固件 VGLite HAL 基址 0x200C8C30 初值），NVIC 70（ISR 0xC224C28 装在
 * 向量偏移 0x158）。芯片标识是驱动 vg_lite_options 要求的值
 * （sub_C0FDA48：0x265 / 0x2002 / 0x424 / eco 0）。 */
#define BEST1503_GPU_BASE         0x40220000
#define BEST1503_GPU_IRQ          70
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
/* PSRAM 执行别名（同 flash 0x2C→0x0C、SRAM 0x20→0x00 的成对窗口）：
 * startup sub_C0C0AC8 把代码段复制到 0x3C000000（0x80500 字节），
 * 固件经 veneer 跳 0x1C000000+off 执行（如 sub_C588C30 → 0x1C058FAD）。
 * 缺失时取指落空滑行进 SRAM 向量表 → NOCP UsageFault → pmu_shutdown。 */
#define BEST1503_PSRAMX_BASE     0x1C000000

#define BEST1503_UART0_IRQ       17
#define BEST1503_UART1_IRQ       18
#define BEST1503_UART2_IRQ       26
/* I2C 主控（DesignWare APB-I2C 兼容；SDK plat_addr_map_best1503.h:223-242
 * 基址、cmsis/inc/best1503.h IRQn） */
#define BEST1503_I2C0_BASE       0x40005000
#define BEST1503_I2C1_BASE       0x40006000
#define BEST1503_I2C2_BASE       0x40060000
#define BEST1503_I2C3_BASE       0x40061000
#define BEST1503_I2C4_BASE       0x40062000
#define BEST1503_I2C5_BASE       0x40063000
/* 0x40064000：SDK 头标 I2S2_BASE，但固件按 I2C id=6 使用（DW 寄存器布局，
 * STATUS@+0x70 实证）；SDK 标错同 0x40168000=PSRAM 先例，以固件为准 */
#define BEST1503_I2C6_BASE       0x40064000
#define BEST1503_I2C0_IRQ        14
#define BEST1503_I2C1_IRQ        29
#define BEST1503_I2C2_IRQ        64
#define BEST1503_I2C3_IRQ        65
#define BEST1503_I2C4_IRQ        53
#define BEST1503_I2C5_IRQ        54
#define BEST1503_I2C6_IRQ        55
/* BES 硬件把 dual-timer 两子定时器分接两个 NVIC 中断（best1503.h IRQn）：
 *   MCU_TIMER00/01 = 12/13，MCU_TIMER10/11 = 27/28，AON_TIMER00/01 = 45/46。
 * hal_sys_timer 走 slow timer（MCU_TIMER0）子定时器 1（SDK hal_timer.c:53-54
 * SLOW_TIMER_SUB_ID_WITH_IRQ=1）→ MCU_TIMER01_IRQn=13。
 * 实测（本固件 v3.101.043 调试）：OS tick 在 AON_TIMER0 子定时器 2
 * （oneshot+INTEN），固件使能 AON_TIMER01_IRQn=46 等待唤醒；
 * sp804 设 split-irq=true 后 irq0=子定时器1 线、irq1=子定时器2 线。 */
#define BEST1503_MCU_TIMER00_IRQ 12
#define BEST1503_MCU_TIMER01_IRQ 13
#define BEST1503_MCU_TIMER10_IRQ 27
#define BEST1503_MCU_TIMER11_IRQ 28
#define BEST1503_AON_TIMER00_IRQ 45   /* AON_TIMER00_IRQn（best1503.h:110） */
#define BEST1503_AON_TIMER01_IRQ 46
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
/* -M best1503,nand-image=<file>：SPI NAND 后端（每页 page+oob 字节，
 * 页序排列）。缺省 emu/nand.img；不存在时创建全擦除镜像。出厂内容
 * （resource romfs）由 emu/mknand.py 生成。 */
static char *g_nand_image;

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

static char *best1503_get_nand_image(Object *obj, Error **errp)
{
    return g_strdup(g_nand_image);
}

static void best1503_set_nand_image(Object *obj, const char *val,
                                    Error **errp)
{
    g_free(g_nand_image);
    g_nand_image = g_strdup(val);
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
    I2CBus *i2c_bus[7];
    static MemoryRegion flash, flash_cached, flash_nc, flash_xnc, sram, ramx;
    static MemoryRegion psram, psram_nc, psramx;
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
     * info）；-M flash-image=<file> 提供其余内容（BL2、参数区、nv/mode
     * 分区的出厂数据，见 emu/mknor.py），-kernel 固件每次启动照样装入
     * AP 分区（等同 OTA 升级），同一镜像可跨固件版本使用。 */
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

        memcpy(bes_flash_data(BES_FLASH(fc)) + g_img.ota_code_offset,
               g_img.data, g_img.size);

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
    memory_region_init_alias(&psramx, NULL, "best1503.psramx",
                             &psram, 0, BEST1503_PSRAM_SIZE);
    memory_region_add_subregion(sysmem, BEST1503_PSRAMX_BASE, &psramx);

    /* ---- BT 子系统共享 RAM（BT 核本体不模拟，只挂存储） ----
     * SDK plat_addr_map_best1503.h:303-306：
     *   BT_RAM_BASE      0xC0000000  64KB（AP↔BT 共享，bt_drv 的
     *                    data_backup 指针表区 0xC0004xxx/0xC0008xxx/0xC0009xxx）
     *   BT_EXCH_MEM_BASE 0xD0210000  64KB（mailbox 交换内存，固件 MPU
     *                    配置日志含 0xD0200000 区域项）
     * 不挂的后果（实测）：bakeup_init 写 [0xC0004070]=备份指针被总线丢弃，
     * backup_write 读回 0 当指针用 → 空指针写入 → MemManage DACCVIOL
     * （MMFAR=0）→ dump → board_reset(2) → 停机。BT 核不存在时该区域
     * 仅作 AP 自写自读的存储，足够让 bt_drv 的备份机制闭环。 */
    static MemoryRegion bt_ram, bt_exch;
    memory_region_init_ram(&bt_ram, NULL, "best1503.bt_ram", 0x10000,
                           &error_fatal);
    memory_region_add_subregion(sysmem, BEST1503_BT_RAM_BASE, &bt_ram);
    memory_region_init_ram(&bt_exch, NULL, "best1503.bt_exch",
                           BEST1503_BT_EXCH_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, BEST1503_BT_EXCH_BASE, &bt_exch);

    /* ---- flash1：SPI NAND（resource romfs + yaffs /data） ---- */
    DeviceState *fc1 = qdev_new(TYPE_BES_FLASH);
    qdev_prop_set_string(fc1, "nand-image",
                         g_nand_image ? g_nand_image : "emu/nand.img");
    qdev_prop_set_uint32(fc1, "nand-id", BEST1503_NAND_ID);
    qdev_prop_set_uint32(fc1, "nand-page-size", BEST1503_NAND_PAGE);
    qdev_prop_set_uint32(fc1, "nand-oob-size", BEST1503_NAND_OOB);
    qdev_prop_set_uint32(fc1, "nand-pages-per-block", BEST1503_NAND_PPB);
    qdev_prop_set_uint32(fc1, "nand-blocks", BEST1503_NAND_BLOCKS);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(fc1), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(fc1), 0, BEST1503_FLASH1_CTRL_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(fc1), 1, BEST1503_FLASH1_NC_BASE);

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
            int irq0;   /* 子定时器 1 的 NVIC 线 */
            int irq1;   /* 子定时器 2 的 NVIC 线 */
        } timers[] = {
            { BEST1503_MCU_TIMER0_BASE, BEST1503_SLOW_TIMER_FRQ,
              BEST1503_MCU_TIMER00_IRQ, BEST1503_MCU_TIMER01_IRQ },
            { BEST1503_MCU_TIMER1_BASE, BEST1503_FAST_TIMER_FRQ,
              BEST1503_MCU_TIMER10_IRQ, BEST1503_MCU_TIMER11_IRQ },
            { BEST1503_AON_TIMER0_BASE, BEST1503_AON_TIMER_FRQ,
              BEST1503_AON_TIMER00_IRQ, BEST1503_AON_TIMER01_IRQ },
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
            /* BES 硬件两子定时器分接两路 NVIC，不做 OR 合并 */
            qdev_prop_set_bit(tmr, "split-irq", true);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(tmr), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(tmr), 0, timers[i].base);
            sysbus_connect_irq(SYS_BUS_DEVICE(tmr), 0,
                               qdev_get_gpio_in(nvic, timers[i].irq0));
            sysbus_connect_irq(SYS_BUS_DEVICE(tmr), 1,
                               qdev_get_gpio_in(nvic, timers[i].irq1));
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

    /* PSC @0x40085000：优先级低于 bes_gpio 的 PSC GPIO 中断子块 */
    {
        DeviceState *psc = qdev_new(TYPE_BES_PSC);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(psc), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(psc), 0, BEST1503_PSC_BASE, -1);
    }

    /* PMU（模拟电源管理芯片）ISPI 窗口 @0x40009000：
     * 固件 MMIO 指针表 0x200B08B8[0] 实证；SDK hal_analogif_best1503.c */
    {
        DeviceState *pmu = qdev_new(TYPE_BES_PMU);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(pmu), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(pmu), 0, 0x40009000);
    }

    /* I2C 主控 ×5（board init 注册 0/2/3/5 + I2C1；NVIC 见 SDK best1503.h） */
    {
        static const struct {
            uint32_t base;
            int irq;
        } i2cs[] = {
            { BEST1503_I2C0_BASE, BEST1503_I2C0_IRQ },
            { BEST1503_I2C1_BASE, BEST1503_I2C1_IRQ },
            { BEST1503_I2C2_BASE, BEST1503_I2C2_IRQ },
            { BEST1503_I2C3_BASE, BEST1503_I2C3_IRQ },
            { BEST1503_I2C4_BASE, BEST1503_I2C4_IRQ },
            { BEST1503_I2C5_BASE, BEST1503_I2C5_IRQ },
            { BEST1503_I2C6_BASE, BEST1503_I2C6_IRQ },
        };
        DeviceState *nvic = armv7m;
        int i;

        for (i = 0; i < ARRAY_SIZE(i2cs); i++) {
            DeviceState *i2c = qdev_new(TYPE_BES_I2C);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(i2c), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(i2c), 0, i2cs[i].base);
            sysbus_connect_irq(SYS_BUS_DEVICE(i2c), 0,
                               qdev_get_gpio_in(nvic, i2cs[i].irq));
            i2c_bus[i] = I2C_BUS(qdev_get_child_bus(i2c, "i2c"));
        }
    }

    /* GPIO + LCDC @0x40200000（SDK plat_addr_map_best1503.h:283）：QSPI
     * 面板命令通道、TE、帧合成输出到图形控制台、帧完成中断。 */
    {
        DeviceState *gpio = qdev_new(TYPE_BES_GPIO);
        DeviceState *lcdc = qdev_new(TYPE_BES_LCDC);

        sysbus_realize_and_unref(SYS_BUS_DEVICE(gpio), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(gpio), 0, BEST1503_GPIO0_BASE);
        sysbus_mmio_map(SYS_BUS_DEVICE(gpio), 1, BEST1503_GPIO1_BASE);
        sysbus_mmio_map(SYS_BUS_DEVICE(gpio), 2, BEST1503_GPIO2_BASE);
        sysbus_mmio_map(SYS_BUS_DEVICE(gpio), 3, BEST1503_PSC_GPIO_IRQ);
        sysbus_connect_irq(SYS_BUS_DEVICE(gpio), 0,
                           qdev_get_gpio_in(armv7m, BEST1503_GPIO_IRQ));

        qdev_prop_set_uint32(lcdc, "panel-width", BEST1503_PANEL_W);
        qdev_prop_set_uint32(lcdc, "panel-height", BEST1503_PANEL_H);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(lcdc), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(lcdc), 0, BEST1503_LCDC_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(lcdc), 0,
                           qdev_get_gpio_in(armv7m, BEST1503_LCDC_IRQ));
        qdev_connect_gpio_out_named(lcdc, "te", 0,
                                    qdev_get_gpio_in(gpio,
                                                     BEST1503_LCD_TE_GPIO));

        {
            I2CSlave *tp = i2c_slave_new(TYPE_ZINITIX_TS,
                                         BEST1503_TP_I2C_ADDR);

            qdev_prop_set_uint32(DEVICE(tp), "width", BEST1503_PANEL_W);
            qdev_prop_set_uint32(DEVICE(tp), "height", BEST1503_PANEL_H);
            i2c_slave_realize_and_unref(tp, i2c_bus[BEST1503_TP_I2C_BUS],
                                        &error_fatal);
            qdev_connect_gpio_out_named(DEVICE(tp), "irq", 0,
                                        qdev_get_gpio_in(gpio,
                                                    BEST1503_TP_INT_GPIO));
        }
    }

    /* 2D GPU */
    {
        DeviceState *gpu = qdev_new(TYPE_VSI_VGLITE);

        object_property_set_link(OBJECT(gpu), "downstream", OBJECT(sysmem),
                                 &error_fatal);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(gpu), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(gpu), 0, BEST1503_GPU_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(gpu), 0,
                           qdev_get_gpio_in(armv7m, BEST1503_GPU_IRQ));
    }

    /* 通用 DMA ×2（PL080 兼容 + BES 2D 扩展，见 bes_dma.c） */
    {
        static const struct {
            uint32_t base;
            int irq;
        } dmas[] = {
            { BEST1503_DMA0_BASE, BEST1503_DMA0_IRQ },
            { BEST1503_DMA1_BASE, BEST1503_DMA1_IRQ },
        };
        DeviceState *dma[2];
        int i;

        for (i = 0; i < ARRAY_SIZE(dmas); i++) {
            dma[i] = qdev_new(TYPE_BES_DMA);
            object_property_set_link(OBJECT(dma[i]), "downstream",
                                     OBJECT(sysmem), &error_fatal);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(dma[i]), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(dma[i]), 0, dmas[i].base);
            sysbus_connect_irq(SYS_BUS_DEVICE(dma[i]), 0,
                               qdev_get_gpio_in(armv7m, dmas[i].irq));
        }
        sysbus_connect_irq(SYS_BUS_DEVICE(fc1), 0,
                           qdev_get_gpio_in_named(dma[1], "dma-req",
                                                  BEST1503_FLASH1_DMA_REQ));
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
        int i;

        sysbus_realize_and_unref(SYS_BUS_DEVICE(cmu), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(cmu), 0, BEST1503_CMU_BASE);
        /* intersys IPC（虚拟 BT 核）：tx0/tx1/rx0/rx1 → NVIC 32/33/34/35
         * IRQ 号依据固件 hal_intersys 注册表（0x2CCDACC8 字节表
         * 0x20/0x21/0x22/0x23，handler hal_intersys_tx/rx_irq_handler）。 */
        for (i = 0; i < 4; i++) {
            qdev_connect_gpio_out_named(cmu, "intersys-irq", i,
                                        qdev_get_gpio_in(armv7m, 32 + i));
        }

        /* BT 子系统：CMU ISIRQ 邮箱的对端 */
        DeviceState *btss = qdev_new(TYPE_BES_BTSS);

        object_property_set_link(OBJECT(btss), "downstream", OBJECT(sysmem),
                                 &error_fatal);
        qdev_prop_set_uint32(btss, "shared-table", BEST1503_BT_MBOX_TABLE);
        qdev_prop_set_uint32(btss, "exch-base", BEST1503_BT_EXCH_BASE);
        qdev_prop_set_uint32(btss, "exch-size", BEST1503_BT_EXCH_SIZE);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(btss), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(btss), 0, BEST1503_BT_CMU_BASE);
        for (i = 0; i < 2; i++) {
            qdev_connect_gpio_out_named(cmu, "bt-doorbell", i,
                qdev_get_gpio_in_named(btss, "doorbell", i));
            qdev_connect_gpio_out_named(btss, "ind-ack", i,
                qdev_get_gpio_in_named(cmu, "bt-ind-ack", i));
            qdev_connect_gpio_out_named(btss, "bt2mcu", i,
                qdev_get_gpio_in_named(cmu, "bt2mcu", i));
        }
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
                                          "written back on exit; the "
                                          "-kernel firmware is installed "
                                          "into its AP partition)");
    object_class_property_add_str(OBJECT_CLASS(mc), "nand-image",
                                  best1503_get_nand_image,
                                  best1503_set_nand_image);
    object_class_property_set_description(OBJECT_CLASS(mc), "nand-image",
                                          "backing file of the SPI NAND "
                                          "(resource romfs + /data); "
                                          "default emu/nand.img, created "
                                          "erased if missing");
}

DEFINE_MACHINE_ARM("best1503", best1503_machine_init)
