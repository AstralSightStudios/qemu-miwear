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
 *     读：((reg & 0xFF) << 16) | 0x10000000（bit28 = 读标志，
 *          q63 运行时实证；EVB BSP hal_analogif_best1700.o 为
 *          bit24（0x01000000），boot ROM 版本差异，以固件为准）
 *     写：(reg << 16) | value16
 *   SPI 全双工：命令字 4 字节发完的同一时钟内从机已回 4 字节 ——
 *   本模型在命令字收集满时立即执行并把 32 位应答装入 RX，
 *   固件按传输宽度从 DATA 逐字节/半字/字弹出，取低 16 位为寄存器值
 *   （运行时实证：boot ROM recv 循环 tx=命令缓冲 len=4，发完即等
 *   +0x0C bit2 读 DATA，再无第二次 TX 写）。
 *
 * PMU 从设备：
 *   hal_analogif_open 连续两次读 reg0 做芯片校验：第一次要求
 *   >>4 == 0x170（chip id），第二次 == 0x18F（metal id；
 *   pmu_best1700.o read_hw_metal_id 的 switch 范围 0x183-0x18F 互证，
 *   且其低 4 位为修订号，读 0 走默认路径）。故 reg0 按读取序回报
 *   0x1700、之后恒 0x18F0。其余寄存器为普通存储（DCDC/GPIO 等配置
 *   由固件写入后回读，闭环即可）。
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

static void bes_ispi_tx(BesIspiState *s, uint32_t value, unsigned size)
{
    /* 命令字按小端字节流收集，满 4 字节立即执行（全双工，应答同时入 RX） */
    while (size--) {
        s->tx |= (value & 0xFF) << (8 * s->tx_cnt);
        value >>= 8;
        if (++s->tx_cnt == 4) {
            uint32_t cmd = s->tx;
            uint8_t reg = (cmd >> 16) & 0xFF;

            if (cmd & 0x10000000) {
                /* 读命令：从设备寄存器值。SPI 全双工应答带 2 位前导：
                 * 从机在收到命令后第 2 拍才开始回数据，32 位 RX 里
                 * 有效值在 bit[17:2]（固件 (rx>>2)&0xFFFF 取用——
                 * 运行时实证：RX=0x1700 时固件算得 buf=0x05C0）。 */
                uint16_t v;

                if (reg == 0) {
                    /* PMU 芯片标识：先报 chip id 0x170，再报 metal id 0x18F */
                    v = s->pmu_id_seq++ ? 0x18F0 : 0x1700;
                } else {
                    v = s->pmu_regs[reg];
                }
                s->rx = (uint32_t)v << 2;
                qemu_log_mask(LOG_UNIMP, "bes.ispi: read reg 0x%02x -> 0x%04x\n",
                              reg, v);
            } else {
                /* 写命令：低 16 位为值；写事务的 RX 内容无效 */
                s->pmu_regs[reg] = cmd & 0xFFFF;
                s->rx = 0;
                qemu_log_mask(LOG_UNIMP, "bes.ispi: write reg 0x%02x = 0x%04x\n",
                              reg, (unsigned)(cmd & 0xFFFF));
            }
            s->rx_bits = 32;
            s->tx = 0;
            s->tx_cnt = 0;
        }
    }
}

static uint64_t bes_ispi_read(void *opaque, hwaddr offset, unsigned size)
{
    BesIspiState *s = BES_ISPI(opaque);
    uint32_t v;

    switch (offset) {
    case ISPI_OFF_CTRL:
        v = 0x2;                        /* bit1: 引擎运行 */
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
    memset(s->pmu_regs, 0, sizeof(s->pmu_regs));
    s->pmu_id_seq = 0;
}

static void bes_ispi_realize(DeviceState *dev, Error **errp)
{
    BesIspiState *s = BES_ISPI(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &bes_ispi_ops, s,
                          "bes.ispi", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
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
