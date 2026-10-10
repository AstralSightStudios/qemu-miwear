/*
 * BEST1700 SoC —— q63 AP 固件（Vela/NuttX）模拟（machine 与板级 profile
 * 见 bes_board.c）
 *
 * 启动链：真机 boot ROM → BL → 安全世界 TEE（vela_tee.bin，NuttX +
 * OP-TEE，flash 0xC0000）→ 非安全 AP（vela_ap.bin，flash 0x1C0000）。
 * 这里从 TEE 入口以安全态启动（-kernel 旁的 vela_tee.bin 或
 * tee-image=），TEE 配好 SAU/MPC/TGU 后把 AP 拉进非安全态，AP 经 NSC
 * veneer 调 TEE 服务；没有 TEE 时退而在复位时直接把 AP 放进非安全态。
 *
 * 当前进度：TEE 启动完成并拉起 AP，AP 过 nx_start 继续初始化。
 *
 * 本文件只保留已实证的内容：外设的地址/型号/中断号来自 BSP（open-vela
 * vendor_bes dev-ai-contest-2026 分支 boards/best1700_ep 的
 * libnx_bestbsp_ap.a 等目标文件）或 q63 运行时实测，各处注明出处；
 * 不从 best1503 照搬。
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
#include "qemu/module.h"
#include "qemu/units.h"
#include "hw/core/boards.h"
#include "hw/core/cpu.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-clock.h"
#include "hw/arm/armv7m.h"
#include "target/arm/cpu.h"
#include "target/arm/tcg/idau.h"
#include "hw/arm/boot.h"
#include "hw/arm/bes_board.h"
#include "hw/misc/bes_image.h"
#include "hw/misc/bes_flash.h"
#include "hw/misc/best1700_cmu.h"
#include "hw/misc/bes_ispi.h"
#include "hw/misc/bes_psram.h"
#include "hw/misc/unimp.h"
#include "chardev/char-fe.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "system/system.h"

#define BEST1700_SRAM_BASE       0x20000000
#define BEST1700_SRAM_SIZE       0x400000  /* q63 镜像头 msp=0x2035ffc0 */
/* RAMX 执行别名基址：执行轨迹 0x4BD000 = 0x00200000 + SRAM偏移0x2BD000
 * （q63 实证）；与 best1503 的 RAMX0_BASE 同值 */
#define BEST1700_RAMX_BASE       0x00200000
/* SRAM 安全别名：地址位 26。q63 TEE 的栈/数据在 0x2410xxxx–0x2416xxxx
 * （vela_tee.bin 镜像头），BSP _best1000.lds 的 RAM ORIGIN 带
 * "& ~0x04000000" 去掉此位；与 AP 的 0x20xxxxxx 是同一块 SRAM。 */
#define BEST1700_SRAM_S_BASE     0x24000000

/* BTH（蓝牙主机子系统）RAM：BSP hal_cmu_boot_setup_sp_ram 打开 BTH
 * 时钟后以 0x23020000–0x23040000 作启动栈（q63 TEE 同样如此，实测）；
 * hal_psc gen_bth_shr_ram_mask 的共享区为 0x230C0000 起 13×128KB。
 * 大小取覆盖到 0x23260000 的 4MB 窗口（上界未实证）。 */
#define BEST1700_BTH_RAM_BASE    0x23000000
#define BEST1700_BTH_RAM_SIZE    0x00400000
/* AON 邮箱 RAM：BSP _best1000.lds AON_MAILBOX ORIGIN 0x23C00000，
 * LENGTH 0xB000-0x20（末 0x20 另作他用）；q63 AP 睡眠判定读
 * 0x23c06fdd（实测）。大小取 64KB（上界未实证）。 */
#define BEST1700_AON_RAM_BASE    0x23C00000
#define BEST1700_AON_RAM_SIZE    0x00010000

/* PSRAM：cached 0x18000000 / NC 0x38000000，64MB（BSP _best1000.lds
 * MEMORY：PSRAM ORIGIN 0x18000000、PSRAM_NC = PSRAM - 0x18000000 +
 * 0x38000000，"64M Bytes"）。.data/.psram_data 链接在此（q63 的 UART
 * 基址表在 0x185fd298，实测），固件 start 自己从 flash 复制。 */
#define BEST1700_PSRAM_BASE      0x18000000
#define BEST1700_PSRAM_NC_BASE   0x38000000
#define BEST1700_PSRAM_SIZE      0x04000000

/* 外设空间（AON/SYS AHB/APB 0x40000000、BTH 0x50000000）各 64MB，
 * 安全别名同样在地址位 26（0x44000000 / 0x54000000）：q63 TEE 经
 * 0x44190000 访问 flash 控制器（实测），BSP hal_cmu_best1700 的
 * addr_remap_dev_to_cpu 也保留此位。未建模的寄存器读 0、写忽略，
 * -d unimp 记录访问（侦察用，不代替建模）。 */
#define BEST1700_PERIPH_BASE     0x40000000
#define BEST1700_BTH_PERIPH_BASE 0x50000000
#define BEST1700_PERIPH_SIZE     0x04000000
#define BEST1700_S_ALIAS         0x04000000

/* NOR flash 控制器（norflaship v2，与 best1503 同 IP，bes_flash 建模）：
 * q63 TEE 的控制器表 = {0x44190000, 0x50140000, 0x44198000}（安全别名，
 * 实测 SRAM 0x241005fc），BSP hal_norflaship_v2.o 引用 0x40190000/
 * 0x40198000。id0 = 代码 NOR。 */
#define BEST1700_FLASH_CTRL_BASE 0x40190000

/* PSRAM 控制器（MC v6 + PHY）：BSP hal_psram_v6.o 以 0x40320000 访问
 * MGR 命令引擎、以 0x40328000 访问 PHY（q63 TEE 同，实测） */
#define BEST1700_PSRAM_MC_BASE   0x40320000
#define BEST1700_PSRAM_PHY_BASE  0x40328000

/*
 * NVIC：外部中断 120 个（BSP chips/bes/include/irq.h：
 * CONFIG_ARCH_CHIP_BES1700 → NR_IRQS = 16 + 120）。优先级位数沿用
 * BES 的 3 位（未实证）。
 */
#define BEST1700_NUM_IRQ         120
#define BEST1700_NUM_PRIO_BITS   3

/* MCU_CMU @0x40000000 / AON_CMU @0x40080000：BSP hal_cmu_best1700.o、
 * hal_maincmu_best1700.o、hal_memsc.o、hal_timer.o 直接以此基址访问
 * （寄存器语义见 best1700_cmu.c）。 */
#define BEST1700_CMU_BASE        0x40000000
#define BEST1700_AON_CMU_BASE    0x40080000

/* ISPI 主控（控制器 0）@0x4000A000：q63 实证——hal_spi.o
 * .rodata.spi = {0x4000A000, 0x40008000, 0x40009000}，hal_ispi_send/
 * recv 固定用控制器 0；固件运行时在 SRAM 0x202C2280 自建的基址表
 * 与之一致（实测 dump）。PMU（片选 7）寄存器 0 经此读芯片标识。 */
#define BEST1700_ISPI_BASE       0x4000A000

/* UART = PL011（BES reg_uart，与 best1503 同 IP）。基址与中断号：
 * BSP hal_uart.o .rodata.uart = {0x4000B000 .. irq 22}, {0x4000C000 ..
 * irq 24}, {0x40011000 .. irq 26}（q63 运行时 PSRAM 0x185fd298 的同表
 * 一致，实测） */
#define BEST1700_UART0_BASE      0x4000B000
#define BEST1700_UART1_BASE      0x4000C000
#define BEST1700_UART2_BASE      0x40011000
#define BEST1700_UART0_IRQ       22
#define BEST1700_UART1_IRQ       24
#define BEST1700_UART2_IRQ       26

/* TIMER = ARM SP804（hal_timer.o：子定时器间距 0x20，+0x8 控制、+0xC
 * 清中断）。hal_timer.o .boot_rodata = {0x40083000, 0x40005000}，
 * .rodata.irq_num = {58, 59, 72, 73}（timer00/01/10/11）。timer1 由
 * hal_sys_timer_open 选快时钟（hal_cmu_timer_select_clock(.., 1)），
 * 快时钟 6MHz（hal_sys_timer_delay_ns 6 tick/us）；timer0 在 AON，
 * 慢时钟按 hal 名义值 16kHz（hal_sys_timer_systick_hz 16000.0f）。 */
#define BEST1700_TIMER0_BASE     0x40083000
#define BEST1700_TIMER1_BASE     0x40005000
#define BEST1700_TIMER00_IRQ     58
#define BEST1700_TIMER01_IRQ     59
#define BEST1700_TIMER10_IRQ     72
#define BEST1700_TIMER11_IRQ     73
#define BEST1700_SLOW_TIMER_FRQ  16000
#define BEST1700_FAST_TIMER_FRQ  6000000

/* CPU 时钟：频率值未实证（仅影响 SysTick 计数速率，不影响取指执行） */
#define BEST1700_SYSCLK_FRQ      208000000ULL

/*
 * 关于 0x0 伪 boot 向量表：
 * 真机上电从内置 boot ROM 启动，boot ROM 完成最小初始化后跳转固件
 * 入口。我们没有 boot ROM 镜像；0x0 伪向量表（初始 MSP + 固件入口）
 * 是 rehosting 中模拟 boot ROM 跳转效果的标准替代手段，不修改固件
 * 任何字节。
 */
static BesImage *g_img;
static BesImage *g_tee;         /* 安全世界镜像，NULL = 无 */
static MemoryRegion g_bootvec;

static ARMv7MState *g_armv7m;
static MemoryRegion g_periph, g_bth_periph;

/*
 * IDAU：地址位 26 为安全别名（0x04xxxxxx ITCM、0x24xxxxxx SRAM、
 * 0x44xxxxxx 外设……，见各宏注释），IDAU 报安全；其余报非安全，由
 * SAU 决定。q63 TEE 的 SAU 只把 0x0–0x0FFFFFFF、0x101C0000–0x17FFFFFF、
 * 0x18200000–0xFFFFFFFF 等划为 NS（实测 SAU_RBAR/RLAR 写值），自身
 * 却在 0x041xxxxx 执行 ITCM 代码，靠 IDAU 的"更安全者优先"留在安全态；
 * 缺 IDAU 则取指即 SecureFault INVTRAN（实测）。PPB 豁免。
 */
#define TYPE_BEST1700_IDAU "best1700-idau"

static void best1700_idau_check(IDAUInterface *ii, uint32_t address,
                                int *iregion, bool *exempt, bool *ns,
                                bool *nsc)
{
    *iregion = address >> 26;
    *exempt = (address & 0xeff00000) == 0xe0000000;
    *ns = address >= 0x60000000 || !(address & BEST1700_S_ALIAS);
    *nsc = false;
}

static void best1700_idau_class_init(ObjectClass *oc, const void *data)
{
    IDAU_INTERFACE_CLASS(oc)->check = best1700_idau_check;
}

static const TypeInfo best1700_idau_info = {
    .name = TYPE_BEST1700_IDAU,
    .parent = TYPE_OBJECT,
    .class_init = best1700_idau_class_init,
    .interfaces = (const InterfaceInfo[]) {
        { TYPE_IDAU_INTERFACE },
        { }
    },
};

/*
 * Cortex-M55 实现定义寄存器 0xE001E000–0xE001EFFF（MSCR/PFCR/ITCMCR/
 * DTCMCR，ITGU/DTGU），QEMU 的 M55 未建模（读写落到 PPB 默认区）。
 * q63 TEE 的 tz_TGU_setup 读 ITGU_CFG/DTGU_CFG（+0x504/+0x604）：
 * 块大小 = 1 << (BLKSZ[3:0] + 5)，块数 = 1 << NUMBLKS[11:8]，要求
 * ITCM 0x130000–0x1A0000、DTCM 0x160000 起的块号不越界，否则断言
 * （"bit_offset <= block_num"，实测）。按 ITCM 2MB / DTCM 4MB、32KB
 * 块报：ITGU 64 块、DTGU 128 块，PRESENT（bit31）。其余为普通存储
 * （TGU 的安全划分 QEMU 不执行）。
 */
#define BEST1700_M55_IMPDEF_BASE 0xE001E000
#define M55_ITGU_CFG             0x504
#define M55_DTGU_CFG             0x604
#define M55_TGU_CFG(numblks)     (0x80000000u | ((numblks) << 8) | 10)

static uint32_t g_m55_impdef[0x1000 / 4];

/*
 * MPC（ARM SIE-200 存储保护控制器）寄存器：q63 TEE 的 mpc_arm_config
 * 读 BLK_CFG（+0x14，块大小 = 1 << (BLK_CFG + 5)）与 BLK_MAX（+0x10，
 * LUT 字数 - 1），要求所配区间的末块号 <= (BLK_MAX + 1) * 32，再经
 * CTRL（+0x00，bit8 = BLK_IDX 自增）/ BLK_IDX（+0x18）/ BLK_LUT（+0x1C）
 * 写入安全属性。实例地址取自 TEE 的 MPC 表（安全别名 0x4404xxxx–
 * 0x4406xxxx，每个 SYS RAM 1MB 块一个，另有 TCM/flash 等），实测。
 * 只存寄存器、不做访问过滤（QEMU 不执行这层安全划分）。报 32KB 块、
 * 128 个 LUT 字（每个 MPC 可覆盖 128MB），任何区间都放得下。
 */
#define MPC_CTRL                 0x00
#define MPC_BLK_MAX              0x10
#define MPC_BLK_CFG              0x14
#define MPC_BLK_IDX              0x18
#define MPC_BLK_LUT              0x1C
#define MPC_CTRL_AUTOINC         (1u << 8)
#define MPC_LUT_WORDS            128

typedef struct Best1700Mpc {
    MemoryRegion mr;
    uint32_t ctrl;
    uint32_t idx;
    uint32_t lut[MPC_LUT_WORDS];
} Best1700Mpc;

static uint64_t best1700_mpc_read(void *opaque, hwaddr offset, unsigned size)
{
    Best1700Mpc *m = opaque;
    uint32_t v = 0;

    switch (offset) {
    case MPC_CTRL:
        v = m->ctrl;
        break;
    case MPC_BLK_MAX:
        v = MPC_LUT_WORDS - 1;
        break;
    case MPC_BLK_CFG:
        v = 10;                         /* 32KB */
        break;
    case MPC_BLK_IDX:
        v = m->idx;
        break;
    case MPC_BLK_LUT:
        v = m->lut[m->idx % MPC_LUT_WORDS];
        if (m->ctrl & MPC_CTRL_AUTOINC) {
            m->idx++;
        }
        break;
    }
    return v;
}

static void best1700_mpc_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    Best1700Mpc *m = opaque;

    switch (offset) {
    case MPC_CTRL:
        m->ctrl = value;
        break;
    case MPC_BLK_IDX:
        m->idx = value;
        break;
    case MPC_BLK_LUT:
        m->lut[m->idx % MPC_LUT_WORDS] = value;
        if (m->ctrl & MPC_CTRL_AUTOINC) {
            m->idx++;
        }
        break;
    }
}

static const MemoryRegionOps best1700_mpc_ops = {
    .read = best1700_mpc_read,
    .write = best1700_mpc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static const uint32_t best1700_mpc_bases[] = {
    0x40042000, 0x40044000, 0x40046000, 0x4004C000, 0x4004E000, 0x40050000,
    0x40054000, 0x40056000,
    0x4005A000, 0x4005C000, 0x4005E000, 0x40060000,
    0x40062000, 0x40064000, 0x40066000, 0x40068000,
};

static uint64_t best1700_m55_impdef_read(void *opaque, hwaddr offset,
                                         unsigned size)
{
    switch (offset) {
    case M55_ITGU_CFG:
        return M55_TGU_CFG(6);
    case M55_DTGU_CFG:
        return M55_TGU_CFG(7);
    default:
        return g_m55_impdef[offset / 4];
    }
}

static void best1700_m55_impdef_write(void *opaque, hwaddr offset,
                                      uint64_t value, unsigned size)
{
    g_m55_impdef[offset / 4] = value;
}

static const MemoryRegionOps best1700_m55_impdef_ops = {
    .read = best1700_m55_impdef_read,
    .write = best1700_m55_impdef_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

/* 外设挂进外设容器（容器同时出现在 NS 与安全别名地址） */
static void best1700_map(SysBusDevice *sbd, int n, hwaddr addr)
{
    MemoryRegion *mr = sysbus_mmio_get_region(sbd, n);

    if (addr >= BEST1700_BTH_PERIPH_BASE) {
        memory_region_add_subregion(&g_bth_periph,
                                    addr - BEST1700_BTH_PERIPH_BASE, mr);
    } else {
        memory_region_add_subregion(&g_periph, addr - BEST1700_PERIPH_BASE,
                                    mr);
    }
}

static void best1700_periph_init(MemoryRegion *sysmem, MemoryRegion *mr,
                                 const char *name, hwaddr base)
{
    static MemoryRegion aliases[2];
    static int n;
    DeviceState *unimp = qdev_new(TYPE_UNIMPLEMENTED_DEVICE);
    g_autofree char *alias = g_strdup_printf("%s_s", name);

    memory_region_init(mr, NULL, name, BEST1700_PERIPH_SIZE);
    qdev_prop_set_string(unimp, "name", name);
    qdev_prop_set_uint64(unimp, "size", BEST1700_PERIPH_SIZE);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(unimp), &error_fatal);
    memory_region_add_subregion_overlap(mr, 0,
        sysbus_mmio_get_region(SYS_BUS_DEVICE(unimp), 0), -1000);
    memory_region_add_subregion(sysmem, base, mr);
    memory_region_init_alias(&aliases[n], NULL, alias, mr, 0,
                             BEST1700_PERIPH_SIZE);
    memory_region_add_subregion(sysmem, base + BEST1700_S_ALIAS,
                                &aliases[n++]);
}

/*
 * 系统复位：CPU 对象不随系统 reset 自动复位（hw/arm/boot.c:1186 注释），
 * 且 armv7m realize 阶段的初始 reset 发生在内存建好之前，故在此重新
 * 复位 CPU（bootvec 已就位）。在所有设备（含 NVIC）复位之后执行
 * （BesSoc.reset），否则 NVIC 复位会清掉下面设的 ITNS。
 */
static void best1700_reset(MachineState *ms)
{
    ARMv7MState *armv7m = g_armv7m;
    CPUARMState *env = &armv7m->cpu->env;
    uint32_t secure_msp;
    unsigned i;

    cpu_reset(CPU(armv7m->cpu));
    /* 真机 BL2 已使能 FPU/MVE（CPACR.CP10/CP11）；模拟器从 AP entry
     * 直接启动缺了这一步，MVE 指令（0xFF 前缀）会 Undefined
     * Instruction。复位后直接使能（等效 BL2 遗留状态，不碰固件）。 */
    env->v7m.cpacr[M_REG_S] |= 0x00F00000;
    env->v7m.cpacr[M_REG_NS] |= 0x00F00000;

    if (g_tee) {
        /* 有 TEE：CPU 以安全态从 TEE 入口启动（bootvec），由 TEE 配置
         * SAU/ITNS 并把 AP 拉进非安全态，与真机一致 */
        return;
    }

    /*
     * 无 TEE 时的退路——AP 固件跑在 TrustZone 非安全态：NuttX 按 NS 编译，新任务的
     * EXC_RETURN 为 0xFFFFFFAC（ES=0、S=0，q63 实测首个 SVC 返回），
     * 在安全态下返回即 INVPC。真机由安全引导配好后 BXNS 进入 AP；
     * 这里在复位时做同样的移交：SAU 全部内存为 NS（ALLNS，SAU 关）、
     * 外部中断全部路由 NS（ITNS）、BusFault/HardFault/NMI 归 NS
     * （AIRCR.BFHFNMINS）、协处理器（FPU/MVE）允许 NS 使用（NSACR），
     * 然后以 NS 态从固件入口启动，NS MSP 取自镜像头。
     */
    env->sau.ctrl = 2;                          /* ALLNS */
    env->v7m.aircr |= R_V7M_AIRCR_BFHFNMINS_MASK;
    env->v7m.nsacr = 0xcff;
    for (i = NVIC_INTERNAL_VECTORS; i < armv7m->nvic.num_irq; i++) {
        armv7m->nvic.itns[i] = true;
    }
    /* 复位后两态都用 MSP（CONTROL.SPSEL=0）：交换银行化的栈指针 */
    secure_msp = env->regs[13];
    env->regs[13] = g_img->msp;
    env->v7m.other_ss_msp = secure_msp;
    env->v7m.other_ss_psp = env->v7m.other_sp;
    env->v7m.other_sp = 0;
    env->v7m.secure = false;
    env->regs[15] = g_img->entry & ~1;
    env->thumb = true;
    arm_rebuild_hflags(env);    /* 安全态改变影响 TCG 缓存的 MMU 索引 */
}

static void best1700_bootvec_reset(void *opaque)
{
    /* 每次系统 reset 重填（在 CPU reset 之前执行），等效真机 boot ROM */
    uint32_t *vec = memory_region_get_ram_ptr(&g_bootvec);

    const BesImage *boot = g_tee ? g_tee : g_img;

    vec[0] = boot->msp;
    vec[1] = boot->entry | 1;   /* Thumb */
}

/* 模拟 BL2 的镜像加载：.data 从 flash 复制到 RAM（地址全部来自镜像头） */
static void best1700_preload_data(const BesImage *im, uint8_t *flash,
                                  uint8_t *sram)
{
    uint32_t start = im->data_start & ~(BEST1700_SRAM_S_BASE ^
                                        BEST1700_SRAM_BASE);
    uint32_t len = im->data_end - im->data_start;

    if (start < BEST1700_SRAM_BASE ||
        start + len > BEST1700_SRAM_BASE + BEST1700_SRAM_SIZE) {
        error_report("best1700: .data at 0x%08x is outside SRAM",
                     im->data_start);
        exit(1);
    }
    memcpy(sram + (start - BEST1700_SRAM_BASE),
           flash + (im->data_lma - im->flash_base), len);
}

static void best1700_init(MachineState *machine, BesImage *img,
                          const BesBoard *board)
{
    BesMachineState *bms = BES_MACHINE(machine);
    MemoryRegion *sysmem = get_system_memory();
    static MemoryRegion flash, flash_cached, flash_nc, sram, ramx;
    static MemoryRegion psram, psram_nc, sram_s, bth_ram, aon_ram;
    Clock *sysclk;
    DeviceState *armv7m;

    /* firmware layout: parsed by the machine (single source of truth) */
    g_img = img;

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
    object_property_set_link(OBJECT(armv7m), "idau",
                             object_new_with_props(TYPE_BEST1700_IDAU,
                                                   OBJECT(machine), "idau",
                                                   &error_fatal, NULL),
                             &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(armv7m), &error_fatal);
    {
        static MemoryRegion impdef;

        memory_region_init_io(&impdef, NULL, &best1700_m55_impdef_ops, NULL,
                              "best1700.m55-impdef", 0x1000);
        memory_region_add_subregion(&ARMV7M(armv7m)->container,
                                    BEST1700_M55_IMPDEF_BASE, &impdef);
    }

    /* ---- flash：bes_flash 设备持有存储后端，XIP/cached/NC 三别名共享 ----
     * 真机同一物理 flash 有多个总线窗口，固件经不同窗口访问代码与
     * 数据，故按 bes_image 解析出的三基址挂三个别名。固件写入后端
     * 偏移 OTA_CODE_OFFSET（真机 flash 布局，解析自 boot info）；
     * -M flash-image=<file> 提供其余内容，-kernel 固件每次启动照样
     * 装入 AP 分区（等同 OTA 升级），同一镜像可跨固件版本使用。 */
    DeviceState *fc = qdev_new(TYPE_BES_FLASH);
    {
        uint32_t flash_xip_start = g_img->xip_base - g_img->ota_code_offset;
        uint32_t flash_cached_start = g_img->cached_base - g_img->ota_code_offset;
        uint32_t flash_nc_start = g_img->nc_base - g_img->ota_code_offset;

        qdev_prop_set_uint32(fc, "size", g_img->flash_size);
        /* 双片（FLASH_DUAL_CHIP=1）共 FLASH_SIZE=0x2000000，每片 16MB：
         * GD25LQ128（C8-60-18）。TEE bes_chip_flash_init 按 ID 容量字节
         * 算总大小并与 FLASH_SIZE 比对，8MB 片报不符即断言（实测）。 */
        qdev_prop_set_uint32(fc, "jedec-id", 0xC86018);
        if (bms->flash_image) {
            qdev_prop_set_string(fc, "image", bms->flash_image);
        }
        sysbus_realize_and_unref(SYS_BUS_DEVICE(fc), &error_fatal);

        memcpy(bes_flash_data(BES_FLASH(fc)) + g_img->ota_code_offset,
               g_img->data, g_img->size);

        /* 安全世界（TEE）：q63 vela_tee.bin 自带 OTA_CODE_OFFSET=0xC0000，
         * AP 经其 NSC 入口（SG veneer，如 0x10165e70）调用安全服务 */
        g_autofree char *tee_path = bes_machine_tee_image(machine);
        if (tee_path) {
            Error *err = NULL;

            g_tee = g_new0(BesImage, 1);
            if (!bes_image_load(g_tee, tee_path, &err)) {
                error_report_err(err);
                exit(1);
            }
            if (g_strcmp0(g_tee->chip, img->chip) ||
                g_tee->flash_base != img->flash_base ||
                g_tee->ota_code_offset + g_tee->size >
                img->ota_code_offset) {
                error_report("%s does not fit below the AP image "
                             "(chip %s, offset 0x%x, size 0x%zx)", tee_path,
                             g_tee->chip, g_tee->ota_code_offset,
                             g_tee->size);
                exit(1);
            }
            memcpy(bes_flash_data(BES_FLASH(fc)) + g_tee->ota_code_offset,
                   g_tee->data, g_tee->size);
            info_report("best1700: TEE %s: entry=0x%08x msp=0x%08x",
                        tee_path, g_tee->entry, g_tee->msp);
        } else {
            warn_report("best1700: no TEE image (vela_tee.bin); the AP is "
                        "started non-secure directly and its secure calls "
                        "will fail");
        }

        memory_region_init_alias(&flash, NULL, "best1700.flash_xip",
                                 bes_flash_storage(BES_FLASH(fc)), 0,
                                 g_img->flash_size);
        memory_region_add_subregion(sysmem, flash_xip_start, &flash);
        memory_region_init_alias(&flash_cached, NULL, "best1700.flash_cached",
                                 bes_flash_storage(BES_FLASH(fc)), 0,
                                 g_img->flash_size);
        memory_region_add_subregion(sysmem, flash_cached_start, &flash_cached);
        memory_region_init_alias(&flash_nc, NULL, "best1700.flash_nc",
                                 bes_flash_storage(BES_FLASH(fc)), 0,
                                 g_img->flash_size);
        memory_region_add_subregion(sysmem, flash_nc_start, &flash_nc);
    }

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
    memory_region_init_alias(&sram_s, NULL, "best1700.sram_s", &sram, 0,
                             BEST1700_SRAM_SIZE);
    memory_region_add_subregion(sysmem, BEST1700_SRAM_S_BASE, &sram_s);

    /* ---- BTH RAM / AON 邮箱 RAM（见宏注释） ---- */
    memory_region_init_ram(&bth_ram, NULL, "best1700.bth_ram",
                           BEST1700_BTH_RAM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, BEST1700_BTH_RAM_BASE, &bth_ram);
    memory_region_init_ram(&aon_ram, NULL, "best1700.aon_ram",
                           BEST1700_AON_RAM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, BEST1700_AON_RAM_BASE, &aon_ram);

    /* ---- PSRAM：cached 窗口 + NC 别名 ---- */
    memory_region_init_ram(&psram, NULL, "best1700.psram",
                           BEST1700_PSRAM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, BEST1700_PSRAM_BASE, &psram);
    memory_region_init_alias(&psram_nc, NULL, "best1700.psram_nc",
                             &psram, 0, BEST1700_PSRAM_SIZE);
    memory_region_add_subregion(sysmem, BEST1700_PSRAM_NC_BASE, &psram_nc);

    /* ---- .data 段预加载（模拟真机 BL2 的镜像加载） ----
     * 真机 BL2 跳转入库前已完成 .data 重定位。地址全部来自镜像头
     * 解析（data_start/data_end/data_lma），不写死。 */
    best1700_preload_data(g_img, bes_flash_data(BES_FLASH(fc)),
                          memory_region_get_ram_ptr(&sram));
    if (g_tee) {
        best1700_preload_data(g_tee, bes_flash_data(BES_FLASH(fc)),
                              memory_region_get_ram_ptr(&sram));
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
        static MemoryRegion bootram, bootvec_s, bootram_s;
        memory_region_init_ram(&bootram, NULL, "best1700.bootram",
                               0x200000 - 0x1000, &error_fatal);
        memory_region_add_subregion(sysmem, 0x1000, &bootram);
        /* 安全别名（地址位 26，同 SRAM）：q63 TEE 把 ITCM 代码放在
         * 0x041042f4 等处并经 veneer 调用（实测） */
        memory_region_init_alias(&bootvec_s, NULL, "best1700.bootvec_s",
                                 &g_bootvec, 0, 0x1000);
        memory_region_add_subregion(sysmem, BEST1700_SRAM_S_BASE -
                                    BEST1700_SRAM_BASE, &bootvec_s);
        memory_region_init_alias(&bootram_s, NULL, "best1700.bootram_s",
                                 &bootram, 0, 0x200000 - 0x1000);
        memory_region_add_subregion(sysmem, BEST1700_SRAM_S_BASE -
                                    BEST1700_SRAM_BASE + 0x1000, &bootram_s);
    }

    qemu_register_reset(best1700_bootvec_reset, NULL);
    g_armv7m = ARMV7M(armv7m);

    best1700_periph_init(sysmem, &g_periph, "best1700.periph",
                         BEST1700_PERIPH_BASE);
    best1700_periph_init(sysmem, &g_bth_periph, "best1700.bth_periph",
                         BEST1700_BTH_PERIPH_BASE);

    /* ---- NOR flash 控制器（见宏注释），即上面的存储后端 ---- */
    best1700_map(SYS_BUS_DEVICE(fc), 0, BEST1700_FLASH_CTRL_BASE);

    /* ---- PSRAM 控制器（见宏注释） ---- */
    {
        DeviceState *psramc = qdev_new(TYPE_BES_PSRAM);

        qdev_prop_set_uint64(psramc, "psram-ptr", (uint64_t)(uintptr_t)
                             memory_region_get_ram_ptr(&psram));
        qdev_prop_set_uint32(psramc, "psram-size", BEST1700_PSRAM_SIZE);
        qdev_prop_set_bit(psramc, "v6", true);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(psramc), &error_fatal);
        best1700_map(SYS_BUS_DEVICE(psramc), 0, BEST1700_PSRAM_PHY_BASE);
        best1700_map(SYS_BUS_DEVICE(psramc), 1, BEST1700_PSRAM_MC_BASE);
    }

    /* ---- UART / TIMER（见上方宏注释） ---- */
    {
        static const struct {
            uint32_t base;
            int irq;
        } uarts[] = {
            { BEST1700_UART0_BASE, BEST1700_UART0_IRQ },
            { BEST1700_UART1_BASE, BEST1700_UART1_IRQ },
            { BEST1700_UART2_BASE, BEST1700_UART2_IRQ },
        };
        static const struct {
            uint32_t base;
            uint32_t freq;
            int irq0;   /* 子定时器 1 的 NVIC 线 */
            int irq1;   /* 子定时器 2 的 NVIC 线 */
        } timers[] = {
            { BEST1700_TIMER0_BASE, BEST1700_SLOW_TIMER_FRQ,
              BEST1700_TIMER00_IRQ, BEST1700_TIMER01_IRQ },
            { BEST1700_TIMER1_BASE, BEST1700_FAST_TIMER_FRQ,
              BEST1700_TIMER10_IRQ, BEST1700_TIMER11_IRQ },
        };
        int i;

        for (i = 0; i < ARRAY_SIZE(uarts); i++) {
            DeviceState *uart = qdev_new("pl011");
            Chardev *chr = serial_hd(i);

            if (chr) {
                qdev_prop_set_chr(uart, "chardev", chr);
            }
            qdev_connect_clock_in(uart, "clk", sysclk);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(uart), &error_fatal);
            best1700_map(SYS_BUS_DEVICE(uart), 0, uarts[i].base);
            sysbus_connect_irq(SYS_BUS_DEVICE(uart), 0,
                               qdev_get_gpio_in(armv7m, uarts[i].irq));
        }
        for (i = 0; i < ARRAY_SIZE(timers); i++) {
            DeviceState *tmr = qdev_new("sp804");

            qdev_prop_set_uint32(tmr, "freq0", timers[i].freq);
            qdev_prop_set_uint32(tmr, "freq1", timers[i].freq);
            /* BES 两子定时器分接两路 NVIC，不做 OR 合并 */
            qdev_prop_set_bit(tmr, "split-irq", true);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(tmr), &error_fatal);
            best1700_map(SYS_BUS_DEVICE(tmr), 0, timers[i].base);
            sysbus_connect_irq(SYS_BUS_DEVICE(tmr), 0,
                               qdev_get_gpio_in(armv7m, timers[i].irq0));
            sysbus_connect_irq(SYS_BUS_DEVICE(tmr), 1,
                               qdev_get_gpio_in(armv7m, timers[i].irq1));
        }
    }

    /* ---- MPC 寄存器（见 best1700_mpc_read 前注释） ---- */
    {
        static Best1700Mpc mpcs[ARRAY_SIZE(best1700_mpc_bases)];
        int i;

        for (i = 0; i < ARRAY_SIZE(mpcs); i++) {
            memory_region_init_io(&mpcs[i].mr, NULL, &best1700_mpc_ops,
                                  &mpcs[i], "best1700.mpc", 0x1000);
            memory_region_add_subregion(&g_periph, best1700_mpc_bases[i] -
                                        BEST1700_PERIPH_BASE, &mpcs[i].mr);
        }
    }

    /* ---- MCU_CMU / AON_CMU（见上方宏注释） ----
     * intersys/BT 邮箱未建模：BT 子系统用到再查。 */
    {
        DeviceState *cmu = qdev_new(TYPE_BEST1700_CMU);
        DeviceState *aoncmu = qdev_new(TYPE_BEST1700_AONCMU);

        sysbus_realize_and_unref(SYS_BUS_DEVICE(cmu), &error_fatal);
        best1700_map(SYS_BUS_DEVICE(cmu), 0, BEST1700_CMU_BASE);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(aoncmu), &error_fatal);
        best1700_map(SYS_BUS_DEVICE(aoncmu), 0, BEST1700_AON_CMU_BASE);
    }

    /* ---- ISPI 主控 @0x4000A000（q63 实证，见上方宏注释） ---- */
    {
        DeviceState *ispi = qdev_new(TYPE_BES_ISPI);

        sysbus_realize_and_unref(SYS_BUS_DEVICE(ispi), &error_fatal);
        best1700_map(SYS_BUS_DEVICE(ispi), 0, BEST1700_ISPI_BASE);
    }

    /*
     * 其余外设：一律不挂（落到上面的 unimp 兜底）。查实地址/型号/中断
     * 号后再加。待办：
     *   - ISPI 控制器 1/2（0x40008000/0x40009000，同表实证，
     *     固件用到时再挂）
     *   - flash 控制器 id1/id2（0x50140000/0x40198000）、DMA、GPIO 等
     */
}

static const BesSoc best1700_soc = {
    .chip = "best1700",
    .init = best1700_init,
    .reset = best1700_reset,
};

static void best1700_register(void)
{
    type_register_static(&best1700_idau_info);
    bes_soc_register(&best1700_soc);
}

type_init(best1700_register)
