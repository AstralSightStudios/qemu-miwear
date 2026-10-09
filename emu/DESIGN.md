# BEST1503 QEMU 模拟器设计文档

> 本文档是重写后的正式设计。所有行为必须有依据（SDK 定义或逆向实测），严禁假设。
> 未经用户确认前不写代码。

## 0. 硬性原则

### 0.1 模拟边界
1. **不 hook / 不 patch 固件本体**（代码与内部数据结构都不碰）。模拟只发生在硬件接口层（MMIO 寄存器、中断、总线协议）。支持多版本固件——换固件不需要重新逆向。
2. **模拟实机固件**，不是 goldfish 或任何现成虚拟平台；外设模型按 best1503 实机建。
3. **只模拟 AP 核**；BT/sensor 核、蓝牙、PMU 等一律 stub。

### 0.2 实现纪律
4. **固件地址一律从镜像头解析**（bes_image，与 IDA loader 同逻辑），不写死；换固件版本自动适配。
5. **模块化**：每个外设独立设备模块，基地址通过属性/机器地址表传入；同厂商其他芯片新增 machine 即可复用。
6. **严禁假设**：任何寄存器行为必须有出处（SDK 头文件 文件:行号 / 固件逆向实测），写进代码注释。没搞明白的外设不实现，用 unimp 占位打日志。
7. **模拟逻辑完整，严禁占位**：每个外设必须按语义完整实现（寄存器模型 + 协议/状态机 + 后端），**严禁用 unimp/stub 占位绕过，严禁"待确认/待逆向/TODO"说辞**——要么查清写实，要么不挂。
8. **遇到之前解决过的问题，先查 SDK 分析 / 本文档 / PROGRESS.md，不要重写或瞎写**。
9. **写代码必须结合 IDA 对照固件实际行为验证**（SDK 是参考，固件是事实；SDK 可能与固件对不上，以固件为准）。**逆向分析一律用 IDA 看逻辑（反编译/调用链/基本块），禁止靠搜字符串/搜字节猜函数位置**。IDB（vela_ap.bin.i64）已由用户完成分析，直接用。
10. **严禁跑偏，只关注 AP 核固件（vela_ap.bin）**。

### 0.3 工程纪律
10. **每完成一个驱动/模块立即 commit**（提交信息规范：初始支持用 "initial support for ..."，后续用 "add/fix ..."）。
11. **每次变更更新 PROGRESS.md**（含"最新变更"栏）。
12. **日志/报错信息全部用英文**。
13. **编译只增量 `make`**，不要重跑 configure（meson 自动处理新文件）。

## 1. 模块划分

```
hw/arm/best1503.c              machine：只做组装，不持有任何写死的地址
hw/misc/bes_image.c|.h         BES 固件镜像解析器（所有布局信息的唯一来源）
hw/misc/bes_stub.c|.h          通用占位设备：RAM 语义(写后读回)+访问日志；
                               仅用于"已确认可用 RAM 语义"的区域，不猜值
外设复用 QEMU 现成设备（已验证兼容，见 §3）：
  pl011    —— UART（SDK platform/hal/reg_uart.h:22-54 证实 PL011 布局）
  sp804    —— TIMER（SDK platform/hal/reg_timer.h:24-40 证实 SP804 布局）
后续新增（每个先完成寄存器级逆向再写）：
  bes_cmu / bes_aoncmu / bes_flash_ctrl / bes_display / bes_i2c ...
```

## 2. 固件解析（bes_image.c）——逻辑与 best1503_vela.py 一致

从镜像字节解析（全部为实测格式，loader 已验证）：

| 字段 | 来源 |
|---|---|
| magic/version 校验 | 头 [0x00]=0xFFFFFFFF, [0x04]=0x00050000 |
| boot info 指针（NC 别名） | 头 [0x0C] |
| mapped 入口 | 文件偏移 0x64 的字（奇数 Thumb 指针） |
| **XIP 基址** | (mapped 入口 & ~1) − 0x14 |
| 镜像 NC 基址 | 文件最后 4 字节 |
| FLASH_BASE / FLASH_NC_BASE / OTA_CODE_OFFSET | boot info 文本 `KEY=value` 解析 |
| **cached 别名** | FLASH_BASE + OTA_CODE_OFFSET |
| **NC 别名** | FLASH_NC_BASE + OTA_CODE_OFFSET |
| 初始 MSP / MSPLIM | 文件偏移 0x68 / 0x6C |
| .data LMA/VMA、.bss 范围 | 文件偏移 0x70–0x80 |

machine 用解析结果建立 flash 三别名（QEMU memory_region_init_alias 原生支持）与 SRAM/RAMX 映射。

## 3. 启动流程（不碰固件的硬件语义启动）

1. 0x0 放一块小 ROM 作为"伪 boot 向量表"：内容为 `[解析出的 MSP, 入口|1]`。
2. **必须注册 CPU reset 回调**（`qemu_register_reset` → `cpu_reset`）。依据：hw/arm/boot.c:1186-1193 注释明确 CPU 对象不随系统 reset 自动复位；实测不注册时 armv7m realize 阶段的过早 reset 取向量得 0，导致 lockup（已踩过并验证修复）。
3. CPU 按 ARMv8-M 规范从 0x0 取向量 → 进入固件 stub。

## 4. 内存映射（全部由解析结果驱动）

| 区域 | 地址来源 |
|---|---|
| flash XIP / cached / NC 三别名 | 解析器 |
| SRAM + RAMX 执行别名 | 解析器（MSP 等落在 SRAM 区） |
| 外设窗口 | machine 地址表（每芯片一份，可替换） |
| 未实现区域 | unimplemented-device 打日志（侦察） |

## 5. 外设实现纪律

每个外设开工前必须完成：
1. **寄存器布局确认**：SDK 头文件（首选）或固件逆向（记录函数地址+访问序列）。
2. **状态语义确认**：每个"状态位"写清楚真机上谁置位、固件等什么。
3. 实现后在 PROGRESS.md 记录：外设、地址、依据出处、验证结果。

已确认可直接复用的：
- UART = PL011（reg_uart.h:22-54 标准布局；BES 扩展 OVSAMP@0x4C 等不影响）
- TIMER = SP804（reg_timer.h:24-40 标准布局）

已确认的等待点（逆向实测，需按语义实现而非喂值）：
- AON_CMU MEMSC/MEMSC_STATUS（0x40080024/0x34）：RAM 电源状态，上电即就绪（reg_aoncmu_best1503.h:31-32）
- flash 控制器 REG_00C（0x40140000/0x40500000）：BUSY/FIFO 状态（reg_norflaship_v2.h:175-191, hal_norflaship_v2.c:251-271）
- ISPI（0x40009000）+0x04/+0x0C：传输完成标志
- 显示引擎（0x40160000）+0x404：多字段状态机（bit0 busy、bits[19:15] 状态枚举、bits[10:7]/[14:11] 计数）
- DSI（0x40168000）+0x58：就绪标志

## 6. PSRAM 校准的正确做法（示例：什么叫"完整逻辑"）

固件 `hal_psram_calib_range` 报 "dqs window is too small" 的根源：校准算法对 PSRAM 数据窗口做"写 pattern→读回比对"。**正确模拟 = 在 PSRAM 数据窗口映射一块真 RAM**（写后读回恒一致 → 所有延迟点有效 → 窗口最大 → 校准自然通过）。禁止给校准状态寄存器喂魔法值。

## 7. 复用与同厂商芯片支持

- 外设模块零内嵌地址；machine 层持有地址表（best1503 一份，未来 best1502p 等另加一份）。
- bes_image 解析器与芯片无关（BES 镜像格式通用）。

## 8. 验收顺序（每步有明确判据）

1. 解析器：对 vela_ap.bin / vela_bl2.bin 解析结果与 best1503_vela.py 输出一致。
2. machine 启动：`-d int` 日志出现 `Loaded reset SP <固件MSP> PC <固件入口>`。
3. 外设逐个实现，每步 boot 推进有可观测证据（UART 输出 / PC 轨迹 / 日志）。
