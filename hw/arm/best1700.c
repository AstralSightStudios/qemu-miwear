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
#include "qemu/timer.h"
#include "hw/core/boards.h"
#include "hw/core/cpu.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-clock.h"
#include "hw/arm/armv7m.h"
#include "target/arm/cpu.h"
#include "target/arm/tcg/idau.h"
#include "target/arm/arm-powerctl.h"
#include "target/arm/multiprocessing.h"
#include "hw/arm/boot.h"
#include "hw/arm/bes_board.h"
#include "hw/misc/bes_image.h"
#include "hw/misc/bes_flash.h"
#include "hw/misc/best1700_cmu.h"
#include "hw/misc/bes_ispi.h"
#include "hw/misc/bes_psram.h"
#include "hw/misc/bes_dma.h"
#include "hw/misc/bes_btss.h"
#include "hw/misc/vsi_vglite.h"
#include "hw/misc/bes_i2c.h"
#include "hw/i2c/i2c.h"
#include "hw/misc/bes_stub.h"
#include "hw/misc/bes_lcdc.h"
#include "hw/misc/bes_gpio.h"
#include "hw/misc/best1700_dsi.h"
#include "hw/misc/best1700_bthrom.h"
#include "hw/core/or-irq.h"
#include "hw/core/split-irq.h"
#include "hw/sd/sdhci.h"
#include "hw/sd/sd.h"
#include "system/blockdev.h"
#include "system/block-backend.h"
#include "hw/core/irq.h"
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
#define BEST1700_AON_RAM_SIZE    0x00020000

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

/* I2C 主控 ×5（DesignWare，bes_i2c）：BSP hal_i2c.o .rodata.i2c_desc
 * 每项 {基址, DMA 请求, 中断号 @+0xA} = {0x40006000, 50},
 * {0x40007000, 51}, {0x40075000, 52}, {0x40076000, 53},
 * {0x40077000, 54}。q63 AP 在 I2C2（0x40075000）上轮询 STATUS（实测）。
 * 板上器件未接入：bes_i2c 对无模型地址回退为通用寄存器文件。 */
#define BEST1700_NUM_I2C         5

/* eMMC 主控 @0x40150000：标准 SDHCI 寄存器（BSP sdhci_unifiedAl.o 的
 * sdh_* 访问器：0x08 参数、0x0C 传输模式、0x0E 命令、0x2C 时钟、
 * 0x30–0x3A 中断状态/使能），另有厂商区（DWC MSHC，q63 读改写
 * 0x800/0x818–0x824，实测）按普通存储。q63 在首次访问后即使能 NVIC
 * 26（sdhci_emmc_int_init，实测）；BSP 把 26 记为 UART2 的中断，两者
 * 经或门共用。卡是 QEMU 的 eMMC，由 -drive if=sd,index=0 提供，镜像
 * 由 emu/mkemmc.py 生成（GPT，各分区名即 /dev/<name>）。 */
#define BEST1700_EMMC_BASE       0x40150000
#define BEST1700_EMMC_IRQ        26

/* 显示：LCDC @0x401C0000（与 best1503 同一 IP：图层寄存器、帧启动
 * 0x224、中断 0x1C0/0x1C4 一致，hal_lcdc.o；hal_lcdc_irqn = 27）经
 * DSI0 @0x401D0000（hal_dsi.o dsi_base，hal_dsi_irqn = 28）接面板；面板
 * TE 接 GPIO 47（见下）。 */
#define BEST1700_LCDC_BASE       0x401C0000
#define BEST1700_LCDC_IRQ        27
#define BEST1700_DSI0_BASE       0x401D0000
#define BEST1700_DSI0_IRQ        28

/* GPIO v2 ×3 bank（bes_gpio，与 best1503 同 IP；BSP hal_gpio_v2.o 引用
 * 0x40081000/0x40089000/0x4008A000，_hal_gpio_irq_handler 挂 NVIC 2）。
 * GPIO 中断经 PSC 路由，best1700 的各 bank 使能/状态字分散在 PSC 内：
 * hal_psc_best1700.o m55c0_gpio_irq_mask = {0x400850FC, 0x400851A4,
 * 0x400851C8}、m55c0_gpio_irq_status = {0x40085108, 0x400851A8,
 * 0x400851CC}；PSC 其余寄存器暂按普通存储。面板 TE 接 GPIO 47
 * （q63 hal_dsi 的 dsi_ctrl[0].te_gpio = 47，运行时读出）。 */
#define BEST1700_GPIO0_BASE      0x40081000
#define BEST1700_GPIO1_BASE      0x40089000
#define BEST1700_GPIO2_BASE      0x4008A000
#define BEST1700_PSC_BASE        0x40085000
#define BEST1700_GPIO_IRQ        2
#define BEST1700_LCD_TE_GPIO     47

/* 通用 DMA ×3（PL080 兼容 + BES 2D 扩展，bes_dma）：BSP hal_dma.o
 * .rodata.dma = {0x401A0000, 0x401A8000, 0x40260000}、irq_type =
 * {37, 38, 39}。q63 AP 的 UART0 控制台走 DMA0：TX 通道 DestPeriph=5、
 * RX 通道 SrcPeriph=4（实测通道配置 0xC940/0xD008）。PL011 无 TX DMA
 * 请求线，其发送 FIFO 在模拟里永不满，TX 请求线恒为高。RX 通道是 128
 * 字节一段的循环链表，UART 只开接收超时中断（IMSC=0x40、DMACR=3）：DMA
 * 读空 FIFO 后由超时中断告诉驱动取已收部分，PL011 的 "dma-rx" 即此。 */
#define BEST1700_DMA0_BASE       0x401A0000
#define BEST1700_DMA1_BASE       0x401A8000
#define BEST1700_DMA2_BASE       0x40260000
#define BEST1700_DMA0_IRQ        37
#define BEST1700_DMA1_IRQ        38
#define BEST1700_DMA2_IRQ        39
#define BEST1700_UART0_DMA_TX    5
#define BEST1700_UART0_DMA_RX    4

/* 总线主设备（DMA 等）看到的 TCM：DTCM AXI 视图 0x21000000 = CPU
 * 0x20000000（4MB）、ITCM AXI 视图 0x21800000 = CPU 0x0（2MB），BSP
 * hal_cmu_best1700.o addr_remap_dev_to_cpu；q63 AP 的 UART DMA 源地址
 * 0x213449f4 即此（实测）。 */
#define BEST1700_DTCM_AXI_BASE   0x21000000
#define BEST1700_ITCM_AXI_BASE   0x21800000

/* 蓝牙子系统（BT 核本体不模拟，bes.btss 扮演邮箱对端 + 空闲 HCI 控制
 * 器）。蓝牙 host 跑在 M55C1（cp 固件）上，依据其 hal_intersys（AP 把
 * vela_cp.bin 装在 PSRAM 0x1B300000，实测）：
 *   - 邮箱指针表 0x68004000（cp 表 0x1B5ECB2C / 0x1B5ECB3C）：[8*ch]
 *     是 BT 发给 cp 的首个描述符，[8*ch+4] 是 cp 发送链表头变量的地址
 *   - 门铃 / 完成 / 中断：MCU_CMU 0x140/0x144（见 best1700_cmu.c）
 *   - cp 收完后写 BT_CMU 0x6E030000 +0x54 bit 18+ch（ch 0/1；ch 2/3 在
 *     +0xB0，未建模）
 *   - cp 的 NVIC：tx done {102, 103, 29, 30}、rx {104, 105, 43, 119}
 *     （cp 表 0x1B5ECB4C / 0x1B5ECB50）
 *   - 通道 0 走 HCI（btdrv 与 bes_serial_hci2）；通道 1 只发 8 字节的
 *     {user, msg} 给 BT 核（bt_intersys_oper_btcore），cp 在其上不收
 * cp 的 btdrv_SendData 在任务里忙等 tx done，没有对端时它不让出 CPU，
 * cp 的 rpmsg 随之停摆，sensor 等发送缓冲超时断言，AP 再整机复位。
 * BT RAM 在 0x68000000（cp 还经 DMA 往 0x683FFF00 起写入，大小未查证，
 * 取 8MB）；BT→cp 的描述符与数据放在其中 0x68010000 起 64KB。 */
#define BEST1700_BT_RAM_BASE     0x68000000
#define BEST1700_BT_RAM_SIZE     0x00800000
#define BEST1700_BT_MBOX_TABLE   0x68004000
#define BEST1700_BT_EXCH_BASE    0x68010000
#define BEST1700_BT_EXCH_SIZE    0x00010000
#define BEST1700_BT_CMU_BASE     0x6E030000
/* 控制器固件日期（HCI_Revision = 年 << 9 | 月 << 5 | 日）：cp 的
 * bt_drv_reg_op_init 要求晚于 2024-03-24（"Please merge
 * MCU_WAKEUP_BT_V2 code"），取 2025-06-01 */
#define BEST1700_BT_FW_DATE      (25 << 9 | 6 << 5 | 1)

static const int best1700_bt_tx_irq[] = { 102, 103, 29, 30 };
static const int best1700_bt_rx_irq[] = { 104, 105, 43, 119 };

/* GPU：VeriSilicon VGLite（GCNanoUltraV，hw/misc/vsi_vglite.c，与 best1503
 * 同一型号）。AP 的 sub_108ACBD8 以 {寄存器 0x401F0000, 显存池 0x22500000,
 * 0x80000} 注册设备、中断挂 NVIC 33（优先级 0xA0）；vg_lite_init 核对芯片
 * ID 0x265、版本 0x2002、0x30 号寄存器 0x420（ITCM 0x13E3EC，best1503 的
 * 是 0x424）。显存池（命令缓冲等）是一块独立 RAM，大小取池的大小。
 * 没有 GPU 时上电流程无限等空闲位，miwear 初始化停在状态 0，约 145 秒后
 * 被它的看门狗复位。 */
#define BEST1700_GPU_BASE        0x401F0000
#define BEST1700_GPU_IRQ         33
#define BEST1700_GPU_RAM_BASE    0x22500000
#define BEST1700_GPU_RAM_SIZE    0x00080000

/* 第二颗 M55（M55C1，即固件里的 "cp"）：跑 cp_image 分区的 vela_cp.bin
 * （蓝牙 host 服务 + WiFi 协议栈，见其字符串）。本核 ITCM（0x0 起 2MB）、
 * DTCM（0x20000000 起 4MB）在总线上位于 0x20E00000 / 0x21C00000（BSP
 * hal_maincmu_best1700.o dsp_m55c1_addr_cpu_to_dev），其余地址与 AP 同一
 * 总线。AP 经 AON_CMU 放核（best1700_cmu.c）：q63 实测先把 {SP 0x200dffa0,
 * 入口 0x3b300011} 写到总线 0x21cdff00，0xB4 = 0x200dff00（本核视角），
 * 再置/清 0x1A4 bit1。 */
#define BEST1700_M55C1_ITCM_BASE 0x20E00000
#define BEST1700_M55C1_ITCM_SIZE 0x00200000
#define BEST1700_M55C1_DTCM_BASE 0x21C00000
#define BEST1700_M55C1_DTCM_SIZE 0x00400000

/* BTH 子系统的 BTHC0 核（固件里的 "sensor" 核，跑 vela_sensor.bin：传感器、
 * GNSS、算法，/misc 所在的 littlefs）。它有自己的 NOR，挂在 flash 控制器 1
 * （寄存器 0x50140000）上，镜像就在这颗 NOR 的偏移 0 原地执行：
 *  - AP 视角的 XIP 窗口 0x28000000（q63 TEE 的 flash 窗口表：控制器 0/1/2
 *    → 0x30000000/0x28000000/0x34000000；AP 在此查 sensor 镜像头，
 *    "subsys_check_boot_struct: boot_addr=0x28000000"）
 *  - BTH 核视角：取指 0x0C000000、数据 0x2C000000（镜像入口跳 0x0c09d6c8，
 *    .data 自 0x2c310d4c 复制到 0x23010228；栈在 BTH RAM 0x2315efe0）
 * 容量：sensor 的 bes_chip_flash_init 断言 hal_norflash_get_size 为
 * 0x800000（"Please change the FLASH_SIZE in common.mk"），MTD 分区
 * bth_misc（256 字节块 0x7800 起 0x400 块，即 7.5MB 起 256KB）与
 * bth_factory（0x7C00 起 0x10 块）也在 8MB 内，按 GD25LQ64（C8-60-17）
 * 建模。镜像不由 AP 装载（AP 只装 cp 与 audio），真机由 BTH
 * 自行从这颗 NOR 启动；模拟器在系统复位时直接从镜像入口放 BTH 核运行。 */
#define BEST1700_BTH_FLASH_CTRL_BASE 0x50140000
#define BEST1700_BTH_FLASH_BASE  0x28000000
#define BEST1700_BTH_FLASH_SIZE  0x00800000
#define BEST1700_BTH_CODE_BASE   0x0C000000
#define BEST1700_BTH_DATA_BASE   0x2C000000
#define BEST1700_BTH_ENTRY       0x10       /* 镜像头后的入口跳板 */

/* 两颗 M55 间的中断（MCU_CMU 0x90/0x94 各位，见 best1700_cmu.c）：
 * hal_m55c0_2_m55c1_best1700.o 两核版本的 rx_irq_id = {88, 89}、
 * tx_irq_id = {91, 92}，即各核 NVIC 上同号：88+c 收到对方通道 c 的数据，
 * 91+c 对方已取走本方通道 c 的数据。 */
#define BEST1700_IPC_RX_IRQ      88
#define BEST1700_ISYS_RX_IRQ     96
#define BEST1700_ISYS_TX_IRQ     94
#define BEST1700_IPC_TX_IRQ      91

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
/* hal_gpio 的 99..164 号脚是 PMU GPIO（见 bes_ispi.c） */
#define BEST1700_PMU_GPIO_FIRST  99
/* PMU 中断线：AP pmu_set_irq_handler（ITCM 0x137E3C）把 0x134B9C 装在
 * 向量 16 + 23、优先级 0xA0，并开 NVIC 23 */
#define BEST1700_PMU_IRQ         23

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
/* M55C1 自己的一对 SP804：vela_cp.bin 的 hal_timer 基址表 {0x40084000,
 * 0x40003000}（0x1b304b48）、irq_num = {60, 61, 68, 69}（0x1b304a00），
 * 与 BSP apc1 版 hal_timer.o 相同；中断只接 M55C1 的 NVIC。 */
#define BEST1700_CP_TIMER0_BASE  0x40084000
#define BEST1700_CP_TIMER1_BASE  0x40003000
#define BEST1700_CP_TIMER00_IRQ  60
#define BEST1700_CP_TIMER01_IRQ  61
#define BEST1700_CP_TIMER10_IRQ  68
#define BEST1700_CP_TIMER11_IRQ  69
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
static ARMv7MState *g_cp;       /* M55C1 */
static ARMv7MState *g_bth;      /* BTHC0，NULL = 无 sensor 镜像 */
static DeviceState *g_aoncmu;
static MemoryRegion g_periph, g_bth_periph;
static DeviceState *g_irq26;
static DeviceState *g_uart0;    /* 控制台，RX DMA 请求接 DMA0 */

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

/*
 * DWT 周期计数器（0xE0001000，CTRL +0x0 / CYCCNT +0x4）：QEMU 的 M 核
 * 未建模 DWT（读落到 PPB 默认区为 0）。M55C1 的 vela_cp.bin 启动即以
 * CYCCNT 做忙等延时，读 0 则永远等不到（实测 PC 0x100bf4 轮询
 * 0xE0001004）。按 CPU 时钟由虚拟时间推出计数，写 CYCCNT 改基准；
 * CTRL 为普通存储（NOCYCCNT 位 25 读 0，即有周期计数器）。
 */
typedef struct Best1700Dwt {
    MemoryRegion mr;
    uint32_t ctrl;
    uint32_t base;              /* CYCCNT = cycles(now) + base */
} Best1700Dwt;

static uint32_t best1700_dwt_cycles(void)
{
    return muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
                    BEST1700_SYSCLK_FRQ, NANOSECONDS_PER_SECOND);
}

static uint64_t best1700_dwt_read(void *opaque, hwaddr offset, unsigned size)
{
    Best1700Dwt *d = opaque;

    switch (offset) {
    case 0x0:
        return d->ctrl;
    case 0x4:
        return best1700_dwt_cycles() + d->base;
    default:
        return 0;
    }
}

static void best1700_dwt_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    Best1700Dwt *d = opaque;

    switch (offset) {
    case 0x0:
        d->ctrl = value;
        break;
    case 0x4:
        d->base = value - best1700_dwt_cycles();
        break;
    }
}

static const MemoryRegionOps best1700_dwt_ops = {
    .read = best1700_dwt_read,
    .write = best1700_dwt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void best1700_dwt_init(ARMv7MState *armv7m, Best1700Dwt *d)
{
    memory_region_init_io(&d->mr, NULL, &best1700_dwt_ops, d,
                          "best1700.dwt", 0x1000);
    memory_region_add_subregion(&armv7m->container, 0xE0001000, &d->mr);
}

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

    cpu_reset(CPU(g_cp->cpu));          /* M55C1 等 AP 放核 */
    if (g_bth) {
        /* BTHC0 自 NOR 启动（见宏注释）：入口跳板，栈由复位处理自设 */
        CPUARMState *benv = &g_bth->cpu->env;

        cpu_reset(CPU(g_bth->cpu));
        benv->regs[15] = BEST1700_BTH_CODE_BASE + BEST1700_BTH_ENTRY;
        benv->thumb = true;
    }
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

/* AON_CMU 放核/扣核（见 M55C1 宏注释）：向量表地址为本核视角 */
static void best1700_cp_run(void *opaque, int n, int level)
{
    ARMCPU *cpu = g_cp->cpu;
    uint64_t id = arm_cpu_mp_affinity(cpu);

    if (level) {
        cpu->init_svtor = BEST1700_CMU(g_aoncmu)->regs[
            BEST1700_AONCMU_M55C1_VTOR / 4] & ~0x7fu;
        arm_set_cpu_on_and_reset(id);
    } else if (cpu->power_state != PSCI_OFF) {
        arm_set_cpu_off(id);
    }
}

static void best1700_cp_init(MachineState *machine, MemoryRegion *sysmem,
                             Clock *sysclk)
{
    static MemoryRegion itcm, dtcm, view, bus, itcm_local, dtcm_local;
    static MemoryRegion impdef;
    static Best1700Dwt dwt;
    DeviceState *cp;

    memory_region_init_ram(&itcm, NULL, "best1700.m55c1_itcm",
                           BEST1700_M55C1_ITCM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, BEST1700_M55C1_ITCM_BASE, &itcm);
    memory_region_init_ram(&dtcm, NULL, "best1700.m55c1_dtcm",
                           BEST1700_M55C1_DTCM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, BEST1700_M55C1_DTCM_BASE, &dtcm);

    /* 本核视图：总线 + 自己的 TCM 盖在 0x0 / 0x20000000 */
    memory_region_init(&view, NULL, "best1700.m55c1", 0x100000000ULL);
    memory_region_init_alias(&bus, NULL, "best1700.m55c1_bus", sysmem, 0,
                             0x100000000ULL);
    memory_region_add_subregion_overlap(&view, 0, &bus, 0);
    memory_region_init_alias(&itcm_local, NULL, "best1700.m55c1_itcm_local",
                             &itcm, 0, BEST1700_M55C1_ITCM_SIZE);
    memory_region_add_subregion_overlap(&view, 0, &itcm_local, 1);
    memory_region_init_alias(&dtcm_local, NULL, "best1700.m55c1_dtcm_local",
                             &dtcm, 0, BEST1700_M55C1_DTCM_SIZE);
    memory_region_add_subregion_overlap(&view, BEST1700_SRAM_BASE,
                                        &dtcm_local, 1);

    cp = qdev_new(TYPE_ARMV7M);
    object_property_add_child(OBJECT(machine), "armv7m-cp", OBJECT(cp));
    qdev_prop_set_uint32(cp, "num-irq", BEST1700_NUM_IRQ);
    qdev_prop_set_uint8(cp, "num-prio-bits", BEST1700_NUM_PRIO_BITS);
    qdev_prop_set_string(cp, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m55"));
    qdev_prop_set_bit(cp, "start-powered-off", true);
    qdev_connect_clock_in(cp, "cpuclk", sysclk);
    qdev_connect_clock_in(cp, "refclk", sysclk);
    object_property_set_link(OBJECT(cp), "memory", OBJECT(&view),
                             &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(cp), &error_fatal);
    memory_region_init_io(&impdef, NULL, &best1700_m55_impdef_ops, NULL,
                          "best1700.m55c1-impdef", 0x1000);
    memory_region_add_subregion(&ARMV7M(cp)->container,
                                BEST1700_M55_IMPDEF_BASE, &impdef);
    best1700_dwt_init(ARMV7M(cp), &dwt);
    g_cp = ARMV7M(cp);
}

/* BTHC0 与它的 NOR（见宏注释）；无 sensor 镜像则都不建 */
static void best1700_bth_init(MachineState *machine, MemoryRegion *sysmem,
                              Clock *sysclk, const BesBoard *board)
{
    static MemoryRegion window, view, bus, code, data, impdef;
    static Best1700Dwt dwt;
    g_autofree char *path = bes_machine_sensor_image(machine);
    g_autofree char *image = NULL;
    gsize len;
    GError *gerr = NULL;
    DeviceState *fc, *bth, *rom;
    MemoryRegion *storage;

    if (!path) {
        warn_report("best1700: no sensor hub image (vela_sensor.bin); "
                    "the AP's /misc and sensors will not come up");
        return;
    }
    if (!g_file_get_contents(path, &image, &len, &gerr)) {
        error_report("best1700: %s", gerr->message);
        exit(1);
    }
    if (len > BEST1700_BTH_FLASH_SIZE) {
        error_report("best1700: %s is larger than the BTH flash", path);
        exit(1);
    }

    fc = qdev_new(TYPE_BES_FLASH);
    qdev_prop_set_uint32(fc, "size", BEST1700_BTH_FLASH_SIZE);
    qdev_prop_set_uint32(fc, "jedec-id", 0xC86017);
    /* sensor-nor= keeps the rest of the NOR (its /misc littlefs with the
     * factory properties, see emu/mksensornor.py) across runs */
    if (BES_MACHINE(machine)->sensor_nor) {
        qdev_prop_set_string(fc, "image", BES_MACHINE(machine)->sensor_nor);
    }
    sysbus_realize_and_unref(SYS_BUS_DEVICE(fc), &error_fatal);
    memcpy(bes_flash_data(BES_FLASH(fc)), image, len);
    best1700_map(SYS_BUS_DEVICE(fc), 0, BEST1700_BTH_FLASH_CTRL_BASE);
    storage = bes_flash_storage(BES_FLASH(fc));
    memory_region_init_alias(&window, NULL, "best1700.bth_flash", storage, 0,
                             BEST1700_BTH_FLASH_SIZE);
    memory_region_add_subregion(sysmem, BEST1700_BTH_FLASH_BASE, &window);

    /* 本核视图：总线 + flash 的取指/数据别名 */
    memory_region_init(&view, NULL, "best1700.bthc0", 0x100000000ULL);
    memory_region_init_alias(&bus, NULL, "best1700.bthc0_bus", sysmem, 0,
                             0x100000000ULL);
    memory_region_add_subregion_overlap(&view, 0, &bus, 0);
    memory_region_init_alias(&code, NULL, "best1700.bth_flash_code", storage,
                             0, BEST1700_BTH_FLASH_SIZE);
    memory_region_add_subregion_overlap(&view, BEST1700_BTH_CODE_BASE, &code,
                                        1);
    memory_region_init_alias(&data, NULL, "best1700.bth_flash_data", storage,
                             0, BEST1700_BTH_FLASH_SIZE);
    memory_region_add_subregion_overlap(&view, BEST1700_BTH_DATA_BASE, &data,
                                        1);

    /* BTH 掩膜 ROM 的替身（best1700_bthrom.c），只在本核视图里；其
     * hypercall 窗口放在外设区（固件开 MPU 后 ROM 之外的空洞不可访问） */
    rom = qdev_new(TYPE_BEST1700_BTHROM);
    object_property_set_link(OBJECT(rom), "memory", OBJECT(&view),
                             &error_fatal);
    qdev_prop_set_uint32(rom, "flash-size", BEST1700_BTH_FLASH_SIZE);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(rom), &error_fatal);
    memory_region_add_subregion_overlap(&view, 0x03000000,
        sysbus_mmio_get_region(SYS_BUS_DEVICE(rom), 0), 1);
    memory_region_add_subregion_overlap(&view, 0x5FFFF000,
        sysbus_mmio_get_region(SYS_BUS_DEVICE(rom), 1), 1);

    bth = qdev_new(TYPE_ARMV7M);
    object_property_add_child(OBJECT(machine), "armv7m-bth", OBJECT(bth));
    qdev_prop_set_uint32(bth, "num-irq", BEST1700_NUM_IRQ);
    qdev_prop_set_uint8(bth, "num-prio-bits", BEST1700_NUM_PRIO_BITS);
    qdev_prop_set_string(bth, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m55"));
    qdev_connect_clock_in(bth, "cpuclk", sysclk);
    qdev_connect_clock_in(bth, "refclk", sysclk);
    object_property_set_link(OBJECT(bth), "memory", OBJECT(&view),
                             &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(bth), &error_fatal);
    memory_region_init_io(&impdef, NULL, &best1700_m55_impdef_ops, NULL,
                          "best1700.bthc0-impdef", 0x1000);
    memory_region_add_subregion(&ARMV7M(bth)->container,
                                BEST1700_M55_IMPDEF_BASE, &impdef);
    best1700_dwt_init(ARMV7M(bth), &dwt);
    g_bth = ARMV7M(bth);

    /* BTH 的 SP804 ×3：sensor 的 hal_timer_setup 自 RAM 表（0x230034e0）
     * 取基址 {0x4008B000, 0x50002000, 0x50003000}，自 0x2c293fbc 取中断
     * {19, 20}, {23, 24}, {25, 26}。0 号在 AON 区、走慢时钟，hwtimer
     * （tickless 闹钟）用其子定时器 0；1、2 号时钟未查实，按快时钟。
     * BES 定时器的计数含 0 态，Load + 1 个周期才中断：hal_timer 重装
     * （sensor 0x0c0a2980）为 n 个节拍写 Load = n - 1。少数一拍时，
     * hwtimer_handler（0x0c0a41e4）读系统定时器发现闹钟未到，只记一次
     * "irq early" 便返回、不再重设硬件定时器，此后 tickless 睡眠永不
     * 唤醒（sensor 卡在 rptun 建链的 10ms 轮询里，cp/AP 随之停在启动
     * 中途），故 SP804 开 count-zero。 */
    {
        static const struct {
            uint32_t base, freq;
            int irq0, irq1;
        } btimers[] = {
            { 0x4008B000, BEST1700_SLOW_TIMER_FRQ, 19, 20 },
            { 0x50002000, BEST1700_FAST_TIMER_FRQ, 23, 24 },
            { 0x50003000, BEST1700_FAST_TIMER_FRQ, 25, 26 },
        };
        int i;

        for (i = 0; i < ARRAY_SIZE(btimers); i++) {
            DeviceState *tmr = qdev_new("sp804");

            qdev_prop_set_uint32(tmr, "freq0", btimers[i].freq);
            qdev_prop_set_uint32(tmr, "freq1", btimers[i].freq);
            qdev_prop_set_bit(tmr, "split-irq", true);
            qdev_prop_set_bit(tmr, "count-zero", true);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(tmr), &error_fatal);
            best1700_map(SYS_BUS_DEVICE(tmr), 0, btimers[i].base);
            sysbus_connect_irq(SYS_BUS_DEVICE(tmr), 0,
                               qdev_get_gpio_in(bth, btimers[i].irq0));
            sysbus_connect_irq(SYS_BUS_DEVICE(tmr), 1,
                               qdev_get_gpio_in(bth, btimers[i].irq1));
        }
    }
    /* BTH hal_i2c's i2c_desc table: five DesignWare controllers,
     * each routed to the BTH NVIC (not the AP's I2C interrupts). */
    {
        static const uint32_t bases[] = {
            0x50006000, 0x50007000, 0x50008000, 0x5000A000, 0x5000D000,
        };
        int i;

        for (i = 0; i < ARRAY_SIZE(bases); i++) {
            DeviceState *i2c = qdev_new(TYPE_BES_I2C);

            sysbus_realize_and_unref(SYS_BUS_DEVICE(i2c), &error_fatal);
            best1700_map(SYS_BUS_DEVICE(i2c), 0, bases[i]);
            sysbus_connect_irq(SYS_BUS_DEVICE(i2c), 0,
                               qdev_get_gpio_in(bth, 39 + i));
            if (board->sensor_switch.type && board->sensor_switch.bus == i) {
                I2CBus *bus = I2C_BUS(qdev_get_child_bus(i2c, "i2c"));

                i2c_slave_create_simple(bus, board->sensor_switch.type,
                                        board->sensor_switch.addr);
            }
        }
    }

    /* BTH hal_uart table (0x2c294204): UART0/1/2 in BTH subsystem */
    {
        static const struct {
            uint32_t base;
            int irq;
        } buarts[] = {
            { 0x5000B000, 22 },
            { 0x5000C000, 24 },
            { 0x50071000, 26 },
        };
        int i;

        for (i = 0; i < ARRAY_SIZE(buarts); i++) {
            DeviceState *uart = qdev_new("pl011");

            qdev_connect_clock_in(uart, "clk", sysclk);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(uart), &error_fatal);
            best1700_map(SYS_BUS_DEVICE(uart), 0, buarts[i].base);
            sysbus_connect_irq(SYS_BUS_DEVICE(uart), 0,
                               qdev_get_gpio_in(bth, buarts[i].irq));
        }
    }
    info_report("best1700: sensor hub %s (%zu bytes) in the BTH flash",
                path, (size_t)len);
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

        static Best1700Dwt dwt;

        memory_region_init_io(&impdef, NULL, &best1700_m55_impdef_ops, NULL,
                              "best1700.m55-impdef", 0x1000);
        memory_region_add_subregion(&ARMV7M(armv7m)->container,
                                    BEST1700_M55_IMPDEF_BASE, &impdef);
        best1700_dwt_init(ARMV7M(armv7m), &dwt);
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

        /* TCM 的 AXI 视图（见宏注释），含安全别名 */
        static MemoryRegion axi[4];
        static const char *const names[4] = {
            "best1700.dtcm_axi", "best1700.dtcm_axi_s",
            "best1700.itcm_axi", "best1700.itcm_axi_s",
        };
        int i;

        for (i = 0; i < 4; i++) {
            bool itcm = i >= 2;

            memory_region_init_alias(&axi[i], NULL, names[i],
                                     itcm ? &bootram : &sram, 0,
                                     itcm ? 0x200000 - 0x1000
                                          : BEST1700_SRAM_SIZE);
            memory_region_add_subregion(sysmem,
                (itcm ? BEST1700_ITCM_AXI_BASE + 0x1000
                      : BEST1700_DTCM_AXI_BASE) +
                (i & 1 ? BEST1700_S_ALIAS : 0), &axi[i]);
        }
    }

    qemu_register_reset(best1700_bootvec_reset, NULL);
    g_armv7m = ARMV7M(armv7m);
    best1700_cp_init(machine, sysmem, sysclk);

    best1700_periph_init(sysmem, &g_periph, "best1700.periph",
                         BEST1700_PERIPH_BASE);
    best1700_periph_init(sysmem, &g_bth_periph, "best1700.bth_periph",
                         BEST1700_BTH_PERIPH_BASE);
    best1700_bth_init(machine, sysmem, sysclk, board);

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
            bool cp;    /* 接 M55C1 的 NVIC */
        } timers[] = {
            { BEST1700_TIMER0_BASE, BEST1700_SLOW_TIMER_FRQ,
              BEST1700_TIMER00_IRQ, BEST1700_TIMER01_IRQ, false },
            { BEST1700_TIMER1_BASE, BEST1700_FAST_TIMER_FRQ,
              BEST1700_TIMER10_IRQ, BEST1700_TIMER11_IRQ, false },
            { BEST1700_CP_TIMER0_BASE, BEST1700_SLOW_TIMER_FRQ,
              BEST1700_CP_TIMER00_IRQ, BEST1700_CP_TIMER01_IRQ, true },
            { BEST1700_CP_TIMER1_BASE, BEST1700_FAST_TIMER_FRQ,
              BEST1700_CP_TIMER10_IRQ, BEST1700_CP_TIMER11_IRQ, true },
        };
        int i;

        /* NVIC 26 由 UART2 与 eMMC 主控共用（见 BEST1700_EMMC_IRQ） */
        g_irq26 = qdev_new(TYPE_OR_IRQ);
        qdev_prop_set_uint16(g_irq26, "num-lines", 2);
        qdev_realize_and_unref(g_irq26, NULL, &error_fatal);
        qdev_connect_gpio_out(g_irq26, 0,
                              qdev_get_gpio_in(armv7m, BEST1700_EMMC_IRQ));

        for (i = 0; i < ARRAY_SIZE(uarts); i++) {
            DeviceState *uart = qdev_new("pl011");
            Chardev *chr = serial_hd(i);

            if (chr) {
                qdev_prop_set_chr(uart, "chardev", chr);
            }
            qdev_connect_clock_in(uart, "clk", sysclk);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(uart), &error_fatal);
            best1700_map(SYS_BUS_DEVICE(uart), 0, uarts[i].base);
            if (uarts[i].base == BEST1700_UART0_BASE) {
                g_uart0 = uart;
            }
            sysbus_connect_irq(SYS_BUS_DEVICE(uart), 0,
                               uarts[i].irq == BEST1700_EMMC_IRQ ?
                               qdev_get_gpio_in(g_irq26, 0) :
                               qdev_get_gpio_in(armv7m, uarts[i].irq));
        }
        for (i = 0; i < ARRAY_SIZE(timers); i++) {
            DeviceState *tmr = qdev_new("sp804");
            DeviceState *nvic = timers[i].cp ? DEVICE(g_cp) : armv7m;

            qdev_prop_set_uint32(tmr, "freq0", timers[i].freq);
            qdev_prop_set_uint32(tmr, "freq1", timers[i].freq);
            /* BES 两子定时器分接两路 NVIC，不做 OR 合并 */
            qdev_prop_set_bit(tmr, "split-irq", true);
            /* 计数含 0 态：Load + 1 个周期才中断（见 BTH 定时器处说明） */
            qdev_prop_set_bit(tmr, "count-zero", true);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(tmr), &error_fatal);
            best1700_map(SYS_BUS_DEVICE(tmr), 0, timers[i].base);
            sysbus_connect_irq(SYS_BUS_DEVICE(tmr), 0,
                               qdev_get_gpio_in(nvic, timers[i].irq0));
            sysbus_connect_irq(SYS_BUS_DEVICE(tmr), 1,
                               qdev_get_gpio_in(nvic, timers[i].irq1));
        }
    }

    /* ---- I2C（见宏注释） ---- */
    I2CBus *ap_i2c_bus[BEST1700_NUM_I2C];
    {
        static const struct {
            uint32_t base;
            int irq;
        } i2cs[BEST1700_NUM_I2C] = {
            { 0x40006000, 50 }, { 0x40007000, 51 }, { 0x40075000, 52 },
            { 0x40076000, 53 }, { 0x40077000, 54 },
        };
        int i;

        for (i = 0; i < ARRAY_SIZE(i2cs); i++) {
            DeviceState *i2c = qdev_new(TYPE_BES_I2C);

            sysbus_realize_and_unref(SYS_BUS_DEVICE(i2c), &error_fatal);
            best1700_map(SYS_BUS_DEVICE(i2c), 0, i2cs[i].base);
            sysbus_connect_irq(SYS_BUS_DEVICE(i2c), 0,
                               qdev_get_gpio_in(armv7m, i2cs[i].irq));
            ap_i2c_bus[i] = I2C_BUS(qdev_get_child_bus(i2c, "i2c"));
        }

        /* 板级 I2C 器件（profile） */
        if (board->gauge.type) {
            i2c_slave_create_simple(ap_i2c_bus[board->gauge.bus],
                                    board->gauge.type, board->gauge.addr);
        }
    }

    /* ---- GPIO + PSC GPIO 中断块（见宏注释） ---- */
    DeviceState *gpio = qdev_new(TYPE_BES_GPIO);
    {
        static const uint32_t en[] = { 0x0FC, 0x1A4, 0x1C8 };
        static const uint32_t st[] = { 0x108, 0x1A8, 0x1CC };
        int i;

        qdev_prop_set_uint32(gpio, "psc-size", 0x200);
        for (i = 0; i < 3; i++) {
            g_autofree char *pe = g_strdup_printf("psc-en%d", i);
            g_autofree char *ps = g_strdup_printf("psc-st%d", i);

            qdev_prop_set_uint32(gpio, pe, en[i]);
            qdev_prop_set_uint32(gpio, ps, st[i]);
        }
        sysbus_realize_and_unref(SYS_BUS_DEVICE(gpio), &error_fatal);
        best1700_map(SYS_BUS_DEVICE(gpio), 0, BEST1700_GPIO0_BASE);
        best1700_map(SYS_BUS_DEVICE(gpio), 1, BEST1700_GPIO1_BASE);
        best1700_map(SYS_BUS_DEVICE(gpio), 2, BEST1700_GPIO2_BASE);
        best1700_map(SYS_BUS_DEVICE(gpio), 3, BEST1700_PSC_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(gpio), 0,
                           qdev_get_gpio_in(armv7m, BEST1700_GPIO_IRQ));
    }

    /* 板上的触摸控制器：报点坐标系 = 面板尺寸，中断脚接 GPIO */
    if (board->touch.type) {
        I2CSlave *tp = i2c_slave_new(board->touch.type,
                                     board->touch.addr);

        qdev_prop_set_uint32(DEVICE(tp), "width", board->panel_width);
        qdev_prop_set_uint32(DEVICE(tp), "height", board->panel_height);
        i2c_slave_realize_and_unref(tp, ap_i2c_bus[board->touch.bus],
                                    &error_fatal);
        if (board->touch.irq_gpio >= 0) {
            qdev_connect_gpio_out_named(DEVICE(tp), "irq", 0,
                                        qdev_get_gpio_in(gpio,
                                                    board->touch.irq_gpio));
        }
    }

    /* 板上的 NFC 控制器：数据就绪脚入、VEN 脚出 */
    if (board->nfc.type) {
        I2CSlave *nfc = i2c_slave_new(board->nfc.type, board->nfc.addr);

        if (board->nfc_rom_version) {
            qdev_prop_set_uint32(DEVICE(nfc), "hw-major",
                                 board->nfc_rom_version);
        }
        if (board->nfc_fw_version) {
            qdev_prop_set_uint32(DEVICE(nfc), "fw-version",
                                 board->nfc_fw_version);
        }
        i2c_slave_realize_and_unref(nfc, ap_i2c_bus[board->nfc.bus],
                                    &error_fatal);
        qdev_connect_gpio_out_named(DEVICE(nfc), "data-ready", 0,
                                    qdev_get_gpio_in(gpio,
                                                     board->nfc.irq_gpio));
        qdev_connect_gpio_out(gpio, board->nfc_ven_gpio,
                              qdev_get_gpio_in_named(DEVICE(nfc), "ven", 0));
    }

    /* ---- 显示：LCDC + DSI0（见宏注释），面板尺寸来自板级 profile ---- */
    if (board->panel_width) {
        DeviceState *lcdc = qdev_new(TYPE_BES_LCDC);
        DeviceState *dsi = qdev_new(TYPE_BEST1700_DSI);
        DeviceState *te_split = qdev_new(TYPE_SPLIT_IRQ);

        qdev_prop_set_uint32(lcdc, "panel-width", board->panel_width);
        qdev_prop_set_uint32(lcdc, "panel-height", board->panel_height);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(lcdc), &error_fatal);
        best1700_map(SYS_BUS_DEVICE(lcdc), 0, BEST1700_LCDC_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(lcdc), 0,
                           qdev_get_gpio_in(armv7m, BEST1700_LCDC_IRQ));

        object_property_set_link(OBJECT(dsi), "panel", OBJECT(lcdc),
                                 &error_fatal);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dsi), &error_fatal);
        best1700_map(SYS_BUS_DEVICE(dsi), 0, BEST1700_DSI0_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(dsi), 0,
                           qdev_get_gpio_in(armv7m, BEST1700_DSI0_IRQ));
        /* 面板 TE 既到 GPIO 47，也到 DSI0：AP 的显示线程在 DSI0 中断里
         * 等 TE（lcdc_wait_frame_done 的信号量由 DSI0 中断处理 post） */
        qdev_prop_set_uint32(te_split, "num-lines", 2);
        qdev_realize_and_unref(te_split, NULL, &error_fatal);
        qdev_connect_gpio_out(te_split, 0,
                              qdev_get_gpio_in(gpio,
                                               BEST1700_LCD_TE_GPIO));
        qdev_connect_gpio_out(te_split, 1,
                              qdev_get_gpio_in_named(dsi, "te", 0));
        qdev_connect_gpio_out_named(lcdc, "te", 0,
                                    qdev_get_gpio_in(te_split, 0));
    }

    /* ---- GPU（见宏注释） ---- */
    {
        static MemoryRegion gpu_ram;
        DeviceState *gpu = qdev_new(TYPE_VSI_VGLITE);

        memory_region_init_ram(&gpu_ram, NULL, "best1700.gpu_ram",
                               BEST1700_GPU_RAM_SIZE, &error_fatal);
        memory_region_add_subregion(sysmem, BEST1700_GPU_RAM_BASE, &gpu_ram);
        qdev_prop_set_uint32(gpu, "chip-cid", 0x420);
        object_property_set_link(OBJECT(gpu), "downstream", OBJECT(sysmem),
                                 &error_fatal);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(gpu), &error_fatal);
        best1700_map(SYS_BUS_DEVICE(gpu), 0, BEST1700_GPU_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(gpu), 0,
                           qdev_get_gpio_in(armv7m, BEST1700_GPU_IRQ));
    }

    /* ---- eMMC 主控 + 卡（见宏注释） ---- */
    {
        DeviceState *sdhci = qdev_new(TYPE_SYSBUS_SDHCI);
        DeviceState *vendor = qdev_new(TYPE_BES_STUB);
        DriveInfo *dinfo = drive_get(IF_SD, 0, 0);

        qdev_prop_set_uint8(sdhci, "sd-spec-version", 3);
        /* QEMU 默认能力 + 8 位总线（eMMC） */
        qdev_prop_set_uint64(sdhci, "capareg", 0x057834b4 | (1 << 18));
        sysbus_realize_and_unref(SYS_BUS_DEVICE(sdhci), &error_fatal);
        memory_region_add_subregion_overlap(&g_periph,
            BEST1700_EMMC_BASE - BEST1700_PERIPH_BASE,
            sysbus_mmio_get_region(SYS_BUS_DEVICE(sdhci), 0), 1);
        sysbus_connect_irq(SYS_BUS_DEVICE(sdhci), 0,
                           qdev_get_gpio_in(g_irq26, 1));

        qdev_prop_set_string(vendor, "tag", "emmc.vendor");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(vendor), &error_fatal);
        memory_region_add_subregion_overlap(&g_periph,
            BEST1700_EMMC_BASE - BEST1700_PERIPH_BASE,
            sysbus_mmio_get_region(SYS_BUS_DEVICE(vendor), 0), 0);

        if (dinfo) {
            DeviceState *card = qdev_new(TYPE_EMMC);

            /* q63 的 BES mmc 核心取 CID 序列号低 16 位作 RCA，NuttX
             * mmcsd 重新枚举时固定用 RCA 1，之后 BES 层仍以旧 RCA 发
             * CMD13（实测）；序列号低 16 位取 1 使两者一致。 */
            qdev_prop_set_uint32(card, "serial", 0x17000001);

            qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(dinfo),
                                    &error_fatal);
            qdev_realize_and_unref(card, qdev_get_child_bus(sdhci, "sd-bus"),
                                   &error_fatal);
        } else {
            warn_report("best1700: no eMMC (-drive if=sd,index=0,file=...; "
                        "build one with emu/mkemmc.py)");
        }
    }

    /* ---- 通用 DMA（见宏注释） ---- */
    {
        static const struct {
            uint32_t base;
            int irq;
        } dmas[] = {
            { BEST1700_DMA0_BASE, BEST1700_DMA0_IRQ },
            { BEST1700_DMA1_BASE, BEST1700_DMA1_IRQ },
            { BEST1700_DMA2_BASE, BEST1700_DMA2_IRQ },
        };
        DeviceState *dma[ARRAY_SIZE(dmas)];
        int i;

        for (i = 0; i < ARRAY_SIZE(dmas); i++) {
            dma[i] = qdev_new(TYPE_BES_DMA);
            object_property_set_link(OBJECT(dma[i]), "downstream",
                                     OBJECT(sysmem), &error_fatal);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(dma[i]), &error_fatal);
            best1700_map(SYS_BUS_DEVICE(dma[i]), 0, dmas[i].base);
            sysbus_connect_irq(SYS_BUS_DEVICE(dma[i]), 0,
                               qdev_get_gpio_in(armv7m, dmas[i].irq));
        }
        qemu_irq_raise(qdev_get_gpio_in_named(dma[0], "dma-req",
                                              BEST1700_UART0_DMA_TX));
        qdev_connect_gpio_out_named(g_uart0, "dma-rx", 0,
            qdev_get_gpio_in_named(dma[0], "dma-req", BEST1700_UART0_DMA_RX));
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

    /* ---- MCU_CMU / AON_CMU（见上方宏注释），含两核间中断、M55C1
     * 放核（见 M55C1 宏注释）与 BT 子系统邮箱（见 BEST1700_BT_* 宏
     * 注释）。 ---- */
    {
        static MemoryRegion bt_ram;
        DeviceState *cmu = qdev_new(TYPE_BEST1700_CMU);
        DeviceState *aoncmu = qdev_new(TYPE_BEST1700_AONCMU);
        DeviceState *btss = qdev_new(TYPE_BES_BTSS);
        int c;

        sysbus_realize_and_unref(SYS_BUS_DEVICE(cmu), &error_fatal);
        best1700_map(SYS_BUS_DEVICE(cmu), 0, BEST1700_CMU_BASE);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(aoncmu), &error_fatal);
        best1700_map(SYS_BUS_DEVICE(aoncmu), 0, BEST1700_AON_CMU_BASE);
        for (c = 0; c < 2; c++) {
            qdev_connect_gpio_out_named(cmu, "ipc", 21 + c,
                qdev_get_gpio_in(armv7m, BEST1700_IPC_RX_IRQ + c));
            qdev_connect_gpio_out_named(cmu, "ipc", 18 + c,
                qdev_get_gpio_in(armv7m, BEST1700_IPC_TX_IRQ + c));
            qdev_connect_gpio_out_named(cmu, "ipc", 15 + c,
                qdev_get_gpio_in(DEVICE(g_cp), BEST1700_IPC_RX_IRQ + c));
            qdev_connect_gpio_out_named(cmu, "ipc", 12 + c,
                qdev_get_gpio_in(DEVICE(g_cp), BEST1700_IPC_TX_IRQ + c));
        }
        /* TrustZone trigger（hal_tz_trigger）：TEE 置 bit 12 → AP 的
         * NVIC 75（AP sub_108BFB64 挂 75、写 0x1E8 bit 12 应答）；AP 置
         * bit 13 → TEE 的 NVIC 74（TEE 0x101161C4 挂 74、应答 bit 13）。
         * 没有它 AP 收不到 TEE 的 rpmsg，optee 客户端
         * （TEEC_InitializeContext）永远等不到回复。 */
        qdev_connect_gpio_out_named(cmu, "tz-trigger", 12,
                                    qdev_get_gpio_in(armv7m, 75));
        qdev_connect_gpio_out_named(cmu, "tz-trigger", 13,
                                    qdev_get_gpio_in(armv7m, 74));
        g_aoncmu = aoncmu;
        qdev_connect_gpio_out_named(aoncmu, "cpu-run", 0,
                                    qemu_allocate_irq(best1700_cp_run,
                                                      NULL, 0));

        /* BTH_CMU 的 intersys 半边（见 best1700_cmu.c）：组 0/1 为 M55C0
         * 通道 0/1，组 2/3 为 M55C1 通道 0/1。M55 两核的 NVIC 号相同：
         * hal_m55c0_2_bthc0 / hal_m55c1_2_bthc0 的 rx_irq_id = {96, 97}、
         * tx_irq_id = {94, 95}；BTH 侧取自 sensor 固件的 IPC 配置：
         * M55C0 组 rx {73, 74} / tx {65, 66}，M55C1 组 rx {75, 76} /
         * tx {67, 68}。 */
        DeviceState *bthcmu = qdev_new(TYPE_BEST1700_BTHCMU);

        sysbus_realize_and_unref(SYS_BUS_DEVICE(bthcmu), &error_fatal);
        best1700_map(SYS_BUS_DEVICE(bthcmu), 0, BEST1700_BTH_PERIPH_BASE);
        best1700_cmu_link_intersys(BEST1700_CMU(cmu), BEST1700_CMU(bthcmu));
        for (c = 0; c < 4; c++) {
            DeviceState *m55 = c < 2 ? armv7m : DEVICE(g_cp);

            qdev_connect_gpio_out_named(cmu, "intersys-rx", c,
                qdev_get_gpio_in(m55, BEST1700_ISYS_RX_IRQ + c % 2));
            qdev_connect_gpio_out_named(cmu, "intersys-tx", c,
                qdev_get_gpio_in(m55, BEST1700_ISYS_TX_IRQ + c % 2));
            if (g_bth) {
                qdev_connect_gpio_out_named(bthcmu, "intersys-rx", c,
                    qdev_get_gpio_in(DEVICE(g_bth), 73 + c));
                qdev_connect_gpio_out_named(bthcmu, "intersys-tx", c,
                    qdev_get_gpio_in(DEVICE(g_bth), 65 + c));
            }
        }

        /* BT 子系统：MCU_CMU 蓝牙邮箱的对端 */
        memory_region_init_ram(&bt_ram, NULL, "best1700.bt_ram",
                               BEST1700_BT_RAM_SIZE, &error_fatal);
        memory_region_add_subregion(sysmem, BEST1700_BT_RAM_BASE, &bt_ram);
        object_property_set_link(OBJECT(btss), "downstream", OBJECT(sysmem),
                                 &error_fatal);
        qdev_prop_set_uint32(btss, "shared-table", BEST1700_BT_MBOX_TABLE);
        qdev_prop_set_uint32(btss, "exch-base", BEST1700_BT_EXCH_BASE);
        qdev_prop_set_uint32(btss, "exch-size", BEST1700_BT_EXCH_SIZE);
        qdev_prop_set_bit(btss, "rx-head-direct", true);
        qdev_prop_set_uint8(btss, "data-done-shift", 18);
        qdev_prop_set_uint16(btss, "hci-revision", BEST1700_BT_FW_DATE);
        qdev_prop_set_int32(btss, "hci-channel", 0);
        /* 回复时序（见 bes_btss.c 头注释）：btdrv 发完配置命令即关邮箱，
         * 回复须远早于此；HCI_Reset 则像真控制器一样要一段时间 */
        qdev_prop_set_uint32(btss, "latency-us", 0);
        qdev_prop_set_uint32(btss, "reset-latency-us", 100000);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(btss), &error_fatal);
        memory_region_add_subregion(sysmem, BEST1700_BT_CMU_BASE,
            sysbus_mmio_get_region(SYS_BUS_DEVICE(btss), 0));
        for (c = 0; c < 2; c++) {
            qdev_connect_gpio_out_named(cmu, "bt-doorbell", c,
                qdev_get_gpio_in_named(btss, "doorbell", c));
            qdev_connect_gpio_out_named(btss, "ind-ack", c,
                qdev_get_gpio_in_named(cmu, "bt-ind-ack", c));
            qdev_connect_gpio_out_named(btss, "bt2mcu", c,
                qdev_get_gpio_in_named(cmu, "bt2mcu", c));
        }
        for (c = 0; c < BEST1700_BT_ISYS_CHANNELS; c++) {
            qdev_connect_gpio_out_named(cmu, "bt-irq", c,
                qdev_get_gpio_in(DEVICE(g_cp), best1700_bt_tx_irq[c]));
            qdev_connect_gpio_out_named(cmu, "bt-irq",
                                        BEST1700_BT_ISYS_CHANNELS + c,
                qdev_get_gpio_in(DEVICE(g_cp), best1700_bt_rx_irq[c]));
        }
    }

    /* ---- ISPI 主控 @0x4000A000（q63 实证，见上方宏注释） ---- */
    DeviceState *ispi = qdev_new(TYPE_BES_ISPI);

    sysbus_realize_and_unref(SYS_BUS_DEVICE(ispi), &error_fatal);
    best1700_map(SYS_BUS_DEVICE(ispi), 0, BEST1700_ISPI_BASE);
    qdev_connect_gpio_out_named(ispi, "pmu-irq", 0,
                                qdev_get_gpio_in(armv7m, BEST1700_PMU_IRQ));

    /* 板上的按键：宿主按键 → GPIO 输入；99 号起是 PMU GPIO（经 ISPI） */
    {
        DeviceState *keys = bes_board_create_keys(board);
        int n;

        for (n = 0; keys && n < BES_BOARD_MAX_KEYS && board->keys[n].name;
             n++) {
            int pin = board->keys[n].gpio;

            qdev_connect_gpio_out_named(keys, "key", n,
                pin >= BEST1700_PMU_GPIO_FIRST ?
                qdev_get_gpio_in_named(ispi, "pmu-gpio",
                                       pin - BEST1700_PMU_GPIO_FIRST) :
                qdev_get_gpio_in(gpio, pin));
        }
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
