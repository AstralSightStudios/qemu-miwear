/*
 * BES ISPI (Internal SPI) master + PMU slave model for best1700
 *
 * 实证来源（contest2026 best1700_ep BSP，libnx_bestbsp_ap.a）：
 *
 * 控制器基址：
 *   hal_spi.o .rodata.spi = {0x4000A000, 0x40008000, 0x40009000}；
 *   hal_ispi_send/recv wrapper 固定使用控制器 0 → 0x4000A000。
 *   q63 固件运行时在 SRAM 0x202C2280 自建的基址表与之一致（实测 dump）。
 *
 * 寄存器语义（hal_spi.o hal_spi_send_id/hal_spi_recv_id/hal_spi_busy_id）：
 *   +0x04  bit1      引擎运行：send/recv 收发循环与 busy_id 的前置，
 *                    恒报 1（busy_id: +0x04&2 ? (+0x0C>>4)&1 : 0）
 *   +0x08  DATA      TX 写 / RX 读（收发共用）
 *   +0x0C  bit1      TX 可写（恒 1）
 *          bit2      RX 非空
 *          bit4      busy（恒 0，否则 busy_id 死等）
 *   +0x20  INTC      写 1 清中断（消费）
 *   +0x30  CNT_RST   写 0 复位传输计数（复位 TX 收集状态）
 *   其余偏移（含 +0x88 等 hal_spi_get_ctrl_id 回读的配置寄存器）：
 *                    普通 RAM 存储。
 *
 * 事务模型（hal_analogif_best1700.o hal_analogif_reg_read/reg_write）：
 *   命令字 32 位，小端字节流逐字节/半字/字写入 DATA 收集：
 *     读：(reg << 18) | 0x10000000（bit28 = 读标志）
 *     写：(reg << 18) | (value16 << 2)
 *   reg 10 位（bit[27:18]），值在 bit[17:2]：hal_analogif_best1700.o
 *   hal_analogif_reg_read 的普通 ISPI 路径（reg & 0x3FF，<< 0x12，
 *   orr 0x10000000）与 hal_analogif_rawwrite（((reg << 16) | value)
 *   << 2）。q63 TEE 的 pmu_boot_init 与 BSP 逐条对得上（写 0x3F =
 *   0x1100 发 0x00FC4400，读 0x4F 发 0x113C0000，实测）。另一条
 *   bit24 读标志、8 位地址的路径（hal_ispi_recv_ex）用于片选 0/3 的
 *   其他从设备，此处未建模。
 *   SPI 全双工：命令字 4 字节发完的同一时钟内从机已回 4 字节 ——
 *   本模型在命令字收集满时立即执行并把 32 位应答装入 RX，
 *   固件按传输宽度从 DATA 逐字节/半字/字弹出，取低 16 位为寄存器值
 *   （运行时实证：boot ROM recv 循环 tx=命令缓冲 len=4，发完即等
 *   +0x0C bit2 读 DATA，再无第二次 TX 写）。
 *
 * 从设备按片选分开：片选 = +0x04 bit[10:7]（hal_spi.o
 *   hal_ispi_activate_cs: bfi r3, cs, #7, #4）。q63 用到片选 0/2/5/6/7
 *   （PMU 及 PSRAM 模拟 PHY 等，实测），各自一套 10 位地址寄存器。
 *
 * 芯片标识：
 *   hal_analogif_open 连续两次读 reg0 做芯片校验：第一次要求
 *   >>4 == 0x170（chip id），第二次 == 0x18F（metal id；
 *   pmu_best1700.o read_hw_metal_id 的 switch 范围 0x183-0x18F 互证，
 *   且其低 4 位为修订号，读 0 走默认路径）。两次读分别在片选 7 与
 *   片选 0（PMU）上（q63 TEE/AP 实测：先置 +0x04 = 0x382 读，再置
 *   0x002 读），故片选 7 的 reg0 复位值 0x1700、片选 0 的 0x18F0。
 *   PMU 型号：pmu_best1700.o read_hw_metal_id 在 reg0>>4 == 0x18F 时再读
 *   reg1，0x650 = PMU_CHIP_TYPE_1806P、0x1050 = 1810P；q63 AP 含
 *   pmu1806p_gpio_setup_irq 等 1806P 专用代码（字符串实证），TEE 的
 *   pmu_open 遇 1803/1808 即断言，故 reg1 复位值 0x0650。其余寄存器为普通存储（DCDC/GPIO 等配置
 *   由固件写入后回读，闭环即可），除：
 *   reg 0x4F：pmu_boot_init（pmu_best1700.o）写 bit15|bit13（0xA000）
 *   启动 DCDC 校准，50ms 内轮询 bit[7:6] == 01 为完成，超时死循环。
 *   校准即刻完成：写入带 bit15 时 bit[7:6] 置 01。
 *   reg 0xBC / 0xBD：PMU GPIO 99..114 / 115..130 的输入电平（bit n =
 *   99 + n / 115 + n 号脚），只读。q63 AP 的 hal_gpio_pin_get_val
 *   （sub_108BB5AC）把 99..164 号脚交给 PMU GPIO 驱动（ITCM 0x138F74），
 *   1806P（reg 0x202C3CD6 记的 PMU 类型 6）上 ≤130 号脚读这两个寄存器；
 *   方向/输出等配置走其他寄存器（ITCM 0x139060）。电平由 "pmu-gpio"
 *   输入线给出（第 n 根 = 99 + n 号脚），板上的按键接在这里。
 *
 * PSRAM 模拟 PHY（片选 5，psramphy_best1700.o 以 0x5051/0x5052 访问，
 * 高 4 位即片选）：reg 0x51 为 DLL 状态，bit6 = 锁定、bit7 = 全 0、
 * bit8 = 全 1、bit[5:0] = 延时输入码（psramphy_is_dll_lock/
 * _all_zero/_all_one/psramphy_dll_dly_in_get）。DLL 即刻锁定在中间码
 * 0x10，否则 hal_psram_phy_dll_range_dyn_config 断言失败（q63 TEE 实测）。
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/bes_ispi.h"

#define ISPI_OFF_STATUS       0x00
#define ISPI_OFF_CTRL         0x04
#define ISPI_OFF_DATA         0x08
#define ISPI_OFF_FIFO         0x0C
#define ISPI_OFF_INTC         0x20
#define ISPI_OFF_CNT_RST      0x30

#define ISPI_CS(s)            (((s)->ctrl >> 7) & 0xF)
#define ISPI_CS_PMU           0
#define ISPI_CS_PSRAM_PHY     5
#define ISPI_CS_CHIPID        7

#define PMU_REG_GPIO_IN0      0xBC    /* PMU GPIO 99..114 */
#define PMU_REG_GPIO_IN1      0xBD    /* PMU GPIO 115..130 */

static void bes_ispi_tx(BesIspiState *s, uint32_t value, unsigned size)
{
    /* 命令字按小端字节流收集，满 4 字节立即执行（全双工，应答同时入 RX） */
    while (size--) {
        s->tx |= (value & 0xFF) << (8 * s->tx_cnt);
        value >>= 8;
        if (++s->tx_cnt == 4) {
            uint32_t cmd = s->tx;
            uint16_t reg = (cmd >> 18) & 0x3FF;

            if (cmd & 0x10000000) {
                /* 读命令：从设备寄存器值。SPI 全双工应答带 2 位前导：
                 * 从机在收到命令后第 2 拍才开始回数据，32 位 RX 里
                 * 有效值在 bit[17:2]（固件 (rx>>2)&0xFFFF 取用——
                 * 运行时实证：RX=0x1700 时固件算得 buf=0x05C0）。 */
                uint16_t v;

                v = s->regs_slave[ISPI_CS(s)][reg];
                if (ISPI_CS(s) == ISPI_CS_PSRAM_PHY && reg == 0x51) {
                    v = (v & ~0x1FF) | 0x40 | 0x10;
                } else if (ISPI_CS(s) == ISPI_CS_PMU &&
                           (reg == PMU_REG_GPIO_IN0 ||
                            reg == PMU_REG_GPIO_IN1)) {
                    v = s->pmu_gpio_in >> (reg == PMU_REG_GPIO_IN1 ? 16 : 0);
                }
                s->rx = (uint32_t)v << 2;
                qemu_log_mask(LOG_UNIMP, "bes.ispi: cs%u read reg 0x%03x -> "
                              "0x%04x\n", ISPI_CS(s), reg, v);
            } else {
                /* 写命令：值在 bit[17:2]；写事务的 RX 内容无效 */
                uint16_t *r = s->regs_slave[ISPI_CS(s)];

                r[reg] = (cmd >> 2) & 0xFFFF;
                if (ISPI_CS(s) == ISPI_CS_PMU && reg == 0x4F &&
                    (r[reg] & 0x8000)) {
                    r[reg] = (r[reg] & ~0xC0) | 0x40;
                }
                s->rx = 0;
                qemu_log_mask(LOG_UNIMP, "bes.ispi: cs%u write reg 0x%03x = "
                              "0x%04x\n", ISPI_CS(s), reg, r[reg]);
            }
            s->rx_bits = 32;
            s->tx = 0;
            s->tx_cnt = 0;
        }
    }
}

/* PMU GPIO input line n = pin 99 + n */
static void bes_ispi_pmu_gpio_in(void *opaque, int n, int level)
{
    BesIspiState *s = BES_ISPI(opaque);

    s->pmu_gpio_in = deposit32(s->pmu_gpio_in, n, 1, level != 0);
}

static uint64_t bes_ispi_read(void *opaque, hwaddr offset, unsigned size)
{
    BesIspiState *s = BES_ISPI(opaque);
    uint32_t v;

    switch (offset) {
    case ISPI_OFF_CTRL:
        v = s->ctrl | 0x2;              /* bit1: 引擎运行 */
        break;
    case ISPI_OFF_DATA:
        /* RX 按读取宽度从低位弹出 */
        if (size >= 4) {
            v = s->rx;
        } else if (size == 2) {
            v = s->rx & 0xFFFF;
        } else {
            v = s->rx & 0xFF;
        }
        s->rx >>= 8 * size;
        s->rx_bits = (s->rx_bits > 8 * (int)size) ?
                     s->rx_bits - 8 * size : 0;
        break;
    case ISPI_OFF_FIFO:
        v = 0x2 | (s->rx_bits ? 0x4 : 0);
        break;
    default:
        v = s->regs[offset >> 2];
        break;
    }

    qemu_log_mask(LOG_UNIMP, "bes.ispi: read  off=0x%03x -> 0x%08x\n",
                  (unsigned)offset, v);
    return v;
}

static void bes_ispi_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    BesIspiState *s = BES_ISPI(opaque);

    qemu_log_mask(LOG_UNIMP, "bes.ispi: write off=0x%03x = 0x%08x\n",
                  (unsigned)offset, (unsigned)value);

    switch (offset) {
    case ISPI_OFF_DATA:
        bes_ispi_tx(s, (uint32_t)value, size);
        break;
    case ISPI_OFF_CNT_RST:
        s->tx = 0;                      /* 计数复位：丢弃收集状态 */
        s->tx_cnt = 0;
        break;
    case ISPI_OFF_INTC:
        break;                          /* 清中断：消费 */
    case ISPI_OFF_CTRL:
        s->ctrl = (uint32_t)value & ~0x2u;  /* bit1 恒报运行；片选等存下 */
        break;
    case ISPI_OFF_FIFO:
        break;                          /* 状态寄存器：只读 */
    default:
        s->regs[offset >> 2] = (uint32_t)value;
        break;
    }
}

static const MemoryRegionOps bes_ispi_ops = {
    .read = bes_ispi_read,
    .write = bes_ispi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void bes_ispi_reset(DeviceState *dev)
{
    BesIspiState *s = BES_ISPI(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->tx = 0;
    s->tx_cnt = 0;
    s->rx = 0;
    s->rx_bits = 0;
    s->ctrl = 0;
    memset(s->regs_slave, 0, sizeof(s->regs_slave));
    s->regs_slave[ISPI_CS_CHIPID][0] = 0x1700;  /* chip id 0x170 */
    s->regs_slave[ISPI_CS_PMU][0] = 0x18F0;     /* metal id 0x18F */
    s->regs_slave[ISPI_CS_PMU][1] = 0x0650;     /* PMU 1806P */
}

static void bes_ispi_realize(DeviceState *dev, Error **errp)
{
    BesIspiState *s = BES_ISPI(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &bes_ispi_ops, s,
                          "bes.ispi", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    qdev_init_gpio_in_named(dev, bes_ispi_pmu_gpio_in, "pmu-gpio",
                            BES_ISPI_PMU_GPIOS);
}

static void bes_ispi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = bes_ispi_realize;
    device_class_set_legacy_reset(dc, bes_ispi_reset);
}

static const TypeInfo bes_ispi_info = {
    .name = TYPE_BES_ISPI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BesIspiState),
    .class_init = bes_ispi_class_init,
};

static void bes_ispi_register_types(void)
{
    type_register_static(&bes_ispi_info);
}

type_init(bes_ispi_register_types)
