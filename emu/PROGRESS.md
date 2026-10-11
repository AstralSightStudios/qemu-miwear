# MiWear-Emu 技术文档（接班指南）

> 本项目目标：在 PC 上模拟小米手环 10 的实机 AP 固件 `vela_ap.bin`，最终能跑桌面 UI、触摸、快应用。
> 本文档记录所有**已验证的技术事实**、实验结论与出处，供后续 agent/协作者直接接班。
> 约定：标注【实测】= 在模拟器/反汇编中实际观察验证；【SDK】= 来自 SDK 源码（注明文件:行号）；【待核实】= 推断，未验证。

---

## 1. 设计红线（不可违反）

1. **不得 hook/patch 固件本体**。模拟器必须工作在硬件接口层（MMIO 寄存器、中断、总线协议），目标是支持多版本固件——换固件版本不需要重新逆向函数地址。
2. 触摸注入走 **I2C 总线级模拟**（模拟触摸 IC 对 I2C 事务的应答）；显示截取走 **QSPI/显示总线命令流**级别。均不允许 hook AP 固件函数。
3. 只模拟 AP 核（best1503 的 Cortex-M33）。BT 核、蓝牙、传感器、PMU 等一律 stub。

## 2. 目标硬件与固件事实

### 2.1 芯片

- 市场型号 **BES2700iMP**，恒玄内部平台代号 **best1503**【实测：固件 boot info `CHIP=best1503`】
- CPU：**Cortex-M33（Armv8-M Mainline），带 FPU，无 Helium/MVE**【实测：capstone 扫 563 万条指令，q 寄存器命中 351 条（0.006%）且均为 `vacge.f32`/`vcge.u8` 等 NEON 形式——MVE 无此指令形式，判定为线性扫描数据区噪声】
- **注意**：不是 BES2800（那是双核 Cortex-M55 的另一代产品）。此前曾担心 M55/Helium 导致 Unicorn 不可用，已排除。
- 双核架构：AP 核 + BT/sensor 核（`FLASH_DUAL_CHIP=1`，核间走 mailbox IPC，见 §4.3）

### 2.2 固件镜像格式（vela_ap.bin v3.101.043）

解析逻辑见仓库已有 `best1503_vela.py`（IDA 9.4 loader，可直接参考复用）。

- 16 字节头：`[0x00]=0xFFFFFFFF` magic，`[0x04]=0x00050000` 版本，`[0x08]=0`，`[0x0C]`=boot info 的 NC 别名指针
- 文件偏移 `0x10`：入口 stub `14 48 00 47` = `LDR R0,[PC,#0x50]; BX R0`（Thumb）
- 文件偏移 `0x64`：mapped 入口指针（奇数 Thumb 指针）；`0x68/0x6C`：MSP/MSPLIM 初值；`0x70-0x80`：.data LMA/VMA 与 .bss 范围
- 文件最后 4 字节：NC 别名映射基址（用于定位 boot info）
- boot info 文本（实测值）：
  ```
  CHIP=best1503  KERNEL=NUTTX  FLASH_DUAL_CHIP=1
  FLASH_BASE=0x2C000000  FLASH_NC_BASE=0x28000000  FLASH_SIZE=0x1000000
  OTA_CODE_OFFSET=0xC0000  NV_REC_DEV_VER=2
  USER_SEC_SIZE=0x2000  FACT_SEC_SIZE=0x2000  AUD_SEC_SIZE=0
  ```

### 2.3 内存映射（本固件实测）

| 区域 | 地址 | 说明 |
|---|---|---|
| Flash XIP 执行别名 | `0x0C0C0000` (= 0x0C000000+OTA) | 代码取指，Thumb |
| Flash cached 别名 | `0x2C0C0000` (= FLASH_BASE+OTA) | rodata 引用主要用此 |
| Flash NC 别名 | `0x280C0000` (= FLASH_NC_BASE+OTA) | boot info 指针用此 |
| SRAM 数据别名 | `0x20000000`+ | DATA `[0x200C7A70,0x200EB9AC)`，BSS `[0x200EB9AC,0x201411B8)`，MSP top `0x2015FEC0`，MSPLIM `0x2015F2C0` |
| SRAM 执行别名 RAMX | `0x00200000`+ | 与 0x20000000 同一物理 RAM；ramfunc 实际运行于此（实测 PC 落入 0x00297xxx） |
| **VTOR（RAM 向量表）** | `0x20076400` | 固件启动时复制向量表到 RAM 并写 VTOR【实测：W4 *0xE000ED08=0x20076400 @pc=0x0c0c092e；systick 向量=0x0c0c0901 有效】 |
| flash 尾部参数区 | `0x07FFA000` / `0x07FEE000` | 一次性标志锁存（见 §2.4 sub_C0C2054） |
| AON_CMU | `0x40080000` | 【SDK】plat_addr_map_best1503.h:248 |
| 外设窗口 | `0x40000000`-`0x5FFFFFFF` | CMU/UART/TIMER/I2C/GPIO 等 |

### 2.4 启动流程（IDA 反汇编 + Unicorn 实测轨迹）

```
0x0C0C0010 _start:          LDR R0,[PC,#0x50]; BX R0   → mapped_startup
0x0C0C0014 mapped_startup:
  BL sub_C0C1AF4      ← MSP 未设置时即调用（真机 ROM bootloader 已留有效 SP）
                        【实测】读 AONCMU+0x5C (0x4008005C)
  MSP=0x2015FEC0, MSPLIM=0x2015F2C0, CONTROL=0, ISB
  BL sub_C0C090C      ← 【实测】写 VTOR=0x20076400
  BL sub_C0C09CC
  BL sub_C0C0B48      ← 内部调 sub_C0C2054：按参数读 0x07FEE000/0x07FFA000 bit0 做一次性锁存
                        （bit0=0 → 写 +4=1 再写 +0=1；bit0=1 → 写 +0x38=1）
                        【实测】再读 0x07FFC008；写 0x40086008=0xFFFFFFFF（AONCMU+0x6008，疑中断状态清除）
                        之后跳转 RAMX 0x002979xx 执行 ramfunc
  拷贝 .data: 0x2CDC40E8 → [0x200C7A70,0x200EB9AC)
  清零 .bss:  [0x200EB9AC,0x201411B8)
  BL sub_C23466C      ← 主初始化（NuttX 方向）
```

**当前卡点**【实测】：ramfunc 内执行 WFI 后 halt（Unicorn 每轮 emu_start 立即返回，PC 停在 0x0029D3BA 附近，残留 R3=0x40080024）。0x40080024 = AON_CMU+0x24 = `MEMSC[4]`【SDK：reg_aoncmu_best1503.h:31，RAM 电源控制，非状态位】。真机上此时应有中断（timer/PMU）唤醒；模拟器需投递中断。SDK 中 WFI 等待点参考：hal_cmu_best1503.c:2917（睡眠路径）、hal_sleep.c:606（idle）。

## 3. Unicorn 路线实验档案（已否决，保留作侦察工具）

骨架：`emu/bes_emu.py`（可跑通 early boot 到 WFI 卡点，带 MMIO 去重日志输出到 `emu/mmio_log.txt`）。

### 3.1 已解决的坑（unicorn 2.1.4, Python 3.11, Windows）

1. `emu_start` 的 begin 地址必须 `|1`（Thumb 位），即使 M 核——否则第一条指令即 INSN_INVALID【实验 /tmp/min_m33.py】
2. CPU model 常量名是 `unicorn.arm_const.UC_CPU_ARM_CORTEX_M33`（不是 UC_ARM_CPU_M33），须 `ctl_set_cpu_model()` 且在**任何 mem_map 之前**调用
3. 所有 mem hook（含 valid 的 UC_HOOK_MEM_WRITE）回调签名均为 6 参数：`(uc, access, addr, size, value, user)`；签名错了 FFI 层静默吞异常
4. M 核对未映射地址**取指**不走 FETCH_UNMAPPED hook，而是走 HardFault 异常路径【实验 /tmp/fetch_test.py：UC_ERR_EXCEPTION】。RAMX 别名解法：预映射 0x00200000-0x00600000 + UC_HOOK_MEM_WRITE 在 RAM 低区（0x20000000-0x200C8000，避开栈/堆高频区）做 RAM→RAMX 单向同步

### 3.2 致命缺陷实证（全系统模拟不可行）

| 实验 | 现象 | 结论 |
|---|---|---|
| SCS 区域访问 | 写 VTOR(0xE000ED08) 落入用户 unmapped hook | unicorn M33 **无内置 SCS/NVIC 寄存器模型** |
| /tmp/svc_test2.py | SVC 执行后 INTR(intno=2) 通知一次，**未进 handler**，顺序执行后续指令 | SVC 系统调用丢失，NuttX syscall 全灭 |
| /tmp/exc_step.py | 手工压栈+IPSR=15+LR=0xFFFFFFF9 进 handler：普通指令可执行；`bx lr` 时 **PC=0xFFFFFFF8**（EXC_RETURN 被当普通分支目标）→ UC_ERR_EXCEPTION | **EXC_RETURN 异常返回语义缺失**，PendSV 任务切换无法工作 |

unicorn 倒是有完整特殊寄存器暴露（IPSR/CONTROL/PRIMASK/BASEPRI/MSP/PSP/XPSR），即"能看不能用"——异常进入/退出硬件语义需全手工实现。

**结论**：RTOS 三要素（SVC、PendSV、SysTick 投递）+ EXC_RETURN 全残缺，补全等于用 Python 重写 NVIC+异常模型，且每个语义偏差都是极难调的 RTOS 随机死锁。否决全系统模拟。**保留价值**：bes_emu.py 作为 MMIO 侦察工具（跑 early boot 收集外设访问序列）。

## 4. SDK 资源与 MMIO 布局

SDK 树：`MiBand10-BES2700iMP-BEST1503-Hacking/bes2700impForDisplayRe/`（含 platform/hal/best1503 全套寄存器头文件、rtos/、app/bes2700imp/board/、drivers/）

### 4.1 关键头文件

- `platform/hal/best1503/plat_addr_map_best1503.h` — 全部基地址（AON_CMU_BASE=0x40080000 @:248 等）
- `platform/hal/best1503/reg_aoncmu_best1503.h` — AONCMU 寄存器（0x24=MEMSC[4] @:31）
- `platform/hal/best1503/reg_cmu_best1503.h` — CMU 寄存器
- `platform/hal/best1503/hal_cmu_best1503.c` — 时钟 HAL（WFI@:2917 在睡眠路径）
- `platform/hal/reg_uart.h` — **UART 是 ARM PL011 兼容**（DR/FR/IBRD/FBRD/LCR_H/CR/IMSC/RIS/MIS/ICR + PID/CID，BES 扩展 OVSAMP@0x4C/OVSAMPST@0x50/RXEXT@0x54）→ **QEMU 现成 pl011 设备可直接用**

### 4.1.1 外设基地址表（plat_addr_map_best1503.h:218-324）

| 地址 | 外设 | 地址 | 外设 |
|---|---|---|---|
| 0x40000000 | CMU | 0x4000B000 | **UART0** |
| 0x40001000 | MCU_WDT | 0x4000C000 | **UART1**（SDK 默认 console @921600）|
| 0x40002000 | MCU_TIMER0 | 0x4000D000 | **UART2** |
| 0x40003000 | MCU_TIMER1 | 0x40005000 | I2C0 |
| 0x40004000 | MCU_TIMER2 | 0x40006000 | I2C1 |
| 0x40007000 | SPI | 0x40009000 | ISPI |
| 0x4000A000 | SPI3 | 0x40060000-0x40063000 | I2C2-I2C5 |
| 0x40080000 | AON_CMU | 0x40081000 | AON_GPIO |
| 0x40082000 | AON_WDT | 0x40084000 | AON_TIMER0（TIMER0_BASE）|
| 0x40086000 | **AON_IOMUX** | 0x40089000 | AON_TIMER1 |
| 0x40140000 | flash 控制器(片0) | 0x40500000 | flash 控制器(片1) |
| 0xD0300000 | BT_UART | 0xD0330000 | BT_CMU |

**勘误**：之前记录的 `写 0x40086008=0xFFFFFFFF` 属于 **AON_IOMUX+0x08**（不是 AONCMU+0x6008）。

### 4.2 核间 IPC（AP↔BT）【SDK】

- MCU 侧：CMU `ISIRQ_SET@0x400000A0`/`CLR@0xA4`，`MCU2BT_INTMASK0/1@0xB0/0xB4`，`MCU2CP_IRQ_SET/CLR@0xB8/0xBC`（reg_cmu_best1503.h:57-90）
- 共享内存队列表指针 `0xC0004000`（hal_intersys.c:106-120）；大块数据 `BT_EXCH_MEM 0xD0210000`
- MCU NVIC 中断号：ISDONE=32/33/57/58，ISDATA=34/35/59/60（best1503.h:97-100）
- **模拟器策略**：BTCMU/ISIRQ 写吞掉；固件等 BT 应答时按 mmio log 决定伪造 ISDATA 中断或伪造共享内存应答

### 4.3 Flash 布局【SDK+实测】

- sector=0x2000；双片控制器 `0x40140000`（片0）/`0x40500000`（片1），dual-chip 模式位 hal_norflaship_v2.c:976-986
- vela_ap.bin 位于 flash0 offset 0xC0000；OTA 偏移规则 config/common.mk:4062-4095
- flash 尾部自底向上（tail_section.lds.S:37-78）：userdata @ 0x1000000-0x4000，factory @ 0x1000000-0x2000【未逐字节验证】

### 4.4 待核实清单

1. DSI/LCDC/PSRAM 窗口（SDK 为占位值）
2. NuttX tick 源：SysTick 还是平台 TIMER（NuttX 源码不在 SDK 树内）
3. 触摸/传感器 I2C 总线号与器件地址（SDK 为 TODO 占位）
4. AON_CMU CHIP_ID（0x40080000）实机期望值——先返 0x1503 类值观察分支
5. 日志 UART 口号与波特率（SDK 默认 UART1@921600，bootrom 常 UART0）——模拟器把 UART0/1/2 DR 写全部打印观察

## 5. QEMU 路线（当前方向）

环境：msys2（mingw64）+ QEMU 源码 `E:\miwear-emu\qemu-miwear`（git 分支 `miwear-best1503`，基于 stable tag **v11.1.2**，2026-09-28）。

### 5.1 编译环境（已趟平的坑）

- 调用方式：`/c/msys64/usr/bin/env MSYSTEM=MINGW64 HOME=/home/Admintrator /c/msys64/usr/bin/bash -lc '...'`
  - **必须显式设 HOME**，否则 meson 报 "Could not determine home directory"
- 依赖（pacman 已装）：mingw-w64-x86_64-{gcc,meson,ninja,glib2,pixman,pkgconf,python,dtc,libcbor,SDL2} + msys2 git（meson 下载 subproject 需要 git 程序；也可用 Git for Windows 手动 clone 到 subprojects/ 代替）
- configure：`mkdir build && cd build && ../configure --target-list=arm-softmmu --disable-werror`（已通过）
- build：`cd build && make -j$(nproc)`，产物 `build/qemu-system-arm.exe`

### 5.2 machine 骨架（已写，待编译验证）

`hw/arm/best1503.c` + Kconfig(`config BEST1503`, select ARM_V7M/UNIMP) + meson.build 注册。

设计要点：
- `TYPE_ARMV7M` 容器 + cpu-type=cortex-m33 → NVIC/SysTick/SVC/PendSV/EXC_RETURN 全部正确（换路线的根本原因）
- flash 三别名用 `memory_region_init_alias`（QEMU 原生别名同步，无 Unicorn 的 RAMX 同步问题）
- 0x0 伪 boot 向量表（初始 MSP + reset→XIP stub），硬件语义启动，不碰固件
- 固件加载：`-kernel vela_ap.bin` → `rom_add_file_fixed`（不用 armv7m_load_kernel，它按标准向量表解析会误读 BES 头）
- 外设窗口 0x40000000-0x50000000 v1 全部 `create_unimplemented_device`（打日志做侦察）
- num-irq=128, prio-bits=4【待核实】，sysclk=208MHz【参考 1502p，待核实】

### 5.3 外设实现顺序（按 boot 依赖）

AONCMU/CMU 魔术值 → UART（console, qemu chardev 直通终端）→ TIMER/SysTick → flash 控制器 stub → QSPI 显示命令流截取 → I2C 触摸 IC 模拟

## 6. 进度清单

- [x] 固件格式解析与 IDA loader（best1503_vela.py）
- [x] 芯片架构确认（M33 无 Helium）
- [x] Unicorn 骨架 + MMIO 侦察（emu/bes_emu.py，跑到 early boot WFI 卡点）
- [x] Unicorn 致命缺陷实证与路线否决
- [x] SDK MMIO 布局初步分析
- [x] QEMU 编译环境搭建（msys2 + v11.1.2，configure 通过）
- [x] best1503 machine 骨架（hw/arm/best1503.c + Kconfig/meson 注册）
- [x] QEMU 编译通过（macOS / Homebrew）
- [x] machine 冒烟测试（启动固件，看 unimp 日志轨迹）
- [x] boot 到 miwear 应用（NuttX 全部服务起来，LVGL UI 主循环运行）
- [x] 显示链路（LCDC 帧路径 + TE + VGLite GPU 光栅化 → QEMU 图形控制台；开机向导"选择语言"页可见）
- [x] 触摸注入（Zinitix ZTW662 I2C 模型，I2C5@0x20，INT=GPIO76；宿主鼠标=手指，emu/touch.py 脚本注入）
- [x] CPU 掉电休眠/唤醒（PSC + AON_CMU 复位向量）、熄屏/手势亮屏、系统时钟校准
- [x] NSH 交互 shell（调试探针式注入，emu/nsh.py；生产固件本身无 shell）
- [x] NFC 控制器（NCI 2.0 over I2C6@0x28，VEN=GPIO69、数据就绪=GPIO77）+ eSE（HCI/ETSI 12，无 applet）；NFC 栈初始化不再卡 42 秒
- [x] 越过配对 / 出厂预置绑定（p67 经 emu/mknor.py 写入 persist 属性；q63 经 emu/mksensornor.py --bound KEY 写入 Sensor NOR，CP 核自动迁移至 miwear_record.db 并跳过向导直接进入表盘）
- [x] 蓝牙端到端通信与 AstroBox 接入（p67/q63 双向广播与寻呼重连、SSP 配对、链路加密、RFCOMM 5 / SPP、L1 握手、ADMIN LOGIN 鉴权读取电量与设备信息全部实测通过）
- [x] 小米手环 11（q66，best1503）：PSRAM MRR、按板 GPADC、CST92xx 触摸，进开机向导
- [ ] 快应用环境验证

## 7. 最新变更

- **2026-10-11 (5)（q66 小米手环 11：进开机向导，触摸可用）**：
  q66 4.100.177 与 p67 同为 best1503（boot info `CHIP=best1503`），差异全在板级与两处
  芯片外设行为：
  ①**PSRAM 厂商识别**：sub_C0C7D10 以 MGR 命令 6（模式寄存器读）读 MR1，轮询 REG_404 bit1
  等 RX FIFO，[4:0]=0x0D 走 APMemory 路径；此前命令 6 只在 v6（best1700）实现，读回 0
  → ASSERT hal_psram_polling:2266，_assert 打日志又递归异常直至栈溢出。bes_psram 现对
  best1503 也执行 MRR，MR1 复位为 property vendor-id（best1503 = 0x0D）。
  ②**电池电压**：GPADC 通道接法随板而变——q66 电池在 ch2（adc ioctl sub_C357D7C），
  soft_gauge 配置 0x20081324 换算 vbat = 7.0 × mV；沿用 p67 的 ch2=250 mV（p67 的 NTC）
  得 1750 mV，开机 5 秒即 `miwear_system_poweroff reason:LOWBAT`。板级 profile 新增
  gpadc_mv[]（q66：ch2 571 mV = 4.0 V，ch3/ch5 NTC 450 mV = 25 ℃，表 0x20081370）。
  ③**触摸 Hynitron CST92xx**（新模型 hw/input/hynitron_ts.c）：配置表 0x200812C4 先试
  cst92xx @0x5A（mi2c 3，INT 27），不应答才转 zinitix。协议见文件头（A0 xx 引导、
  bank/addr 读 0xCACA<<16|0x9217、D1 xx 工作模式回显、D0 00 报点、D0 48 ESD）。
  ④**产品识别**：q66 固件的型号表还列了 o6610/o66lj，按"镜像里出现次数最多的产品"选板
  （bes_image_count），q66 17 次 vs o66 2 次。
  ⑤**屏与存储**：212×520（panel.ic.DEFAULT），TE = GPIO 65（sub_C0C5B78）；/misc 是
  `mount -t littlefs -o ro /dev/facmode /misc`（0xFC0000），/nv 才在 0xFE0000 →
  mknor.py --profile q66；resource 分区 65536 页 → mknand.py --profile q66。
  实测：带 vela_resource.bin 启动约 4 秒进"设置语言"，点 English 进入配对二维码页；
  出厂 did/mac_bt 生效。未做：板号 ADC（hw version）为 0 → 固件按非 NFC 版运行。

- **2026-10-11 (4)（q63：预置绑定数据流打通、LE广播与Page重连、AstroBox端到端鉴权通信跑通）**：
  ①**q63 绑定数据流逆向与 Sensor NOR 预置（emu/mksensornor.py --bound KEY）**：
  深入逆向 CP 核（vela_cp.bin）：CP 核启动时（`sub_1B3F1A80` `miwear_record_read_legacy`）
  遍历 persist 属性表（位于 `0x1B621D70`，包含 `persist.miwear_token_key`、`miwear_beacon_key`、
  `miwear_ir_key`、`miwear_user_id` 等），若长度为偶数则 hex-decode 后写入 `/data/misc/bt/miwear_record.db`
  （UnQLite 格式嵌入式数据库），写完后删除旧 persist 属性。
  手环的绑定状态受普通属性控制（`device_boot_handle` `0x1B3F4044` 与 `save_bond_info` `0x1B3F3AE8`）：
    * `persist.setup_wizard=1`（1 = 已绑定 bonded；0 = 未绑定；2 = 新绑定进行中）；
    * `persist.miwear_companion_device_type=0`（0 = Android 手机；1 = iOS）；
    * `persist.miwear_companion_device_mac`（绑定的手机/主机蓝牙 MAC 地址，12 位小端十六进制字符串，如 C0:FF:EE:00:00:01 写作 010000eecfc0）。
  CP 核安全策略（`allow_link_connect_pair_request` `0x1B3F9D90`）严格限制只允许该伴侣设备地址配对。
  `emu/mksensornor.py` 新增 `--bound KEY [--companion ADDR] [--user-id ID]` 选项，一键生成已绑定的
  Sensor NOR 镜像，手环启动后自动完成数据迁移并跳过向导，直达表盘主界面！
  ②**LE 广播与扫描、Page 重连链路模型（hw/misc/bes_btss_link.c）**：
  已绑定的手环不再应答 BR/EDR Inquiry 搜索，而是按蓝牙规范通过 LE Extended Advertising 发送重连信标（beacon）。
  完善 `bes_btss_link` 的 LE 广播与扫描能力：
    * 实现了 LE Set Advertising / Scan / Extended Advertising 命令集（0x2005..0x2042）；
    * 支持在对端开启扫描时生成标准 LE Advertising Report（0x02）及 LE Extended Advertising Report（0x0D）；
    * 寻呼（Page）支持标准的 Page_Timeout 等待周期（`page_timer`，按 0.625 ms 时钟槽换算超时）：
      当手环交替处于 LE 广播与 Page Scan 窗口时，Page 请求保持挂起并在目标控制器打开 Page Scan 时立即握手，
      彻底解决了快速寻呼时因目标暂未就绪返回 0x04 (Page Timeout) 的偶发失败。
  ③**AstroBox-NG 蓝牙桥支持与端到端闭环验证**：
  AstroBox 蓝牙桥（AstroBox-NG 的 bluetooth 模块及前端设置页）：
    * 开启后台 LE 扫描监听，已绑定手环开机发出的重连广播可自动捕获并显示；
    * 支持"按地址添加手环"（`add_device`，默认 `C0:FF:EE:15:03:01`），通过 Remote Name Request 解析名称；
    * 实测验证：q63 从寻呼连接、SSP 配对（手环弹窗配对码并确认）、链路加密（mode 2）、SDP 查询（RFCOMM 5 通道）、
      SPP 链路建立、L1 帧握手、ADMIN LOGIN（AuthKey）鉴权全部顺利通过！成功读取设备序列号（`QEMU17000000000001`）、
      固件版本（`4.101.020`）、型号（`M2606W1`）及电量（82%），端到端协议链路 100% 畅通！

- **2026-10-11 (3)（q63：40 秒进向导、配色复原、蓝牙偶发 panic 根治）**：
  ①**启动慢**：80 秒里有 40 秒是 miwear 主线程在支付宝 eSE 开通道上等两次
  20 秒超时——q63 没接 NFC。q63 与 p67 同一 sn110 驱动、同一固件文件
  NXP_01_10_6E.bin：I2C 4（0x40077000）地址 0x28，数据就绪 GPIO 90、
  VEN 91（board_sn110_initialize 0x1088DA68，总线号由 0x10878E20 传入）。
  nci_nfcc 两处按 p67 写的行为不兼容 q63：1 字节载荷后的填充字节（q63
  按帧长精读，残字节错位下一包头）→ 改为"读事务越过包尾给填充、下次读
  从新包开始"；VEN 上电 NTF（q63 探测把 RSP、NTF 当前两包读）→ 改为
  property power-on-ntf，默认关（p67 驱动本就跳过它）；VEN 拉高即可收
  命令。NXP 栈只认 ROM 0x01 + 固件主版本 0x10（sub_10EE2380，否则
  NFA_Enable 0xF0 "Unexpected firmware"），固件版本须等于文件的 0x106E
  （否则进下载模式）→ 板级 nfc_rom_version / nfc_fw_version。现在 NFC 栈
  约 1 秒起来，40 秒进向导。
  ②**"偏色"**：渐变色带是单行图像，q63 驱动不设其 stride（0x0A2A=0），
  采样器按 stride×高=0 字节映射，所有线性/径向渐变全透明，向导背景成了
  黑色；向导本是浅色设计（固件显式设白底按钮、黑字、#333333 标题，
  0x10D0ADFC/0x10D0D3BC），黑底上就像反色。修后首页是金色光球、语言页
  浅蓝灰渐变。另查明：0x0A1B 是缓存刷新（0x11 flush、0x11000 失效图像
  缓存）；0x0A00 bit28 / 0x0A10 bit8 由驱动按"目标未预乘"设置
  （vg_lite_blit 0x101D2E70、vg_lite_draw 0x101D5220），LVGL 给的画笔色
  本已预乘，模型在 bit28 时再预乘一次画笔色是错的（半透明填充偏暗），
  尚未改。
  ③**蓝牙偶发 panic（并行负载下约 1/12～1/16）**：在三个核的 _assert
  入口（AP 0x1086E6F4、cp 0x1B33DED4、sensor 0x0C061494）下 gdbstub
  断点批量抓到两种，都是 bes_btss 回复时序：
  (a) cp hal_intersys_rx_irq_handler:544 "Handler missing"——btdrv 初始化
  open(ch0,mbx0) 发厂商命令、只等 TX done 就 close，bthci 稍后才 open；
  最后一条 FC77 的 CC（1 ms 后发布）落进空档；
  (b) cp bes_serial_hci2.c:343——HCI_Reset 后 bthci_receive 发 FC77 唤醒，
  其 CC 复位 TX-done 信号量（0x1B30F226），若早于发复位的线程取走
  TX-done，bthci_send 等满 10 秒断言。
  修法：消息逐条记可发布时间、保序；q63 普通回复 50 µs（latency-us），
  "复位后紧跟的 FC77"的 CC 延迟 10 ms（新 property wake-latency-us，
  默认 0 = 不区分，p67 不变）。之前以为"sensor 先 panic"是误读：
  "[M55C0_2_BTHC0] Got panic msg" 是链路名，不代表发起方。
  调试手段：emu/assertwatch.py <gdb 端口>（QEMU 加 -S -gdb，三核 _assert
  断点，打印首个断言的核、文件:行、寄存器），并行多开几路批量跑抓偶发；
  -trace bes_btss_* 看 HCI；-trace bes_i2c_* 新增
  DATA_CMD / RX 逐字节；-trace vsi_vglite_ramp 看渐变色带。
  ④**q63 NSH 交互体验（达到 p67 水准）**：
  查明 UART0 控制台在 q63 上需攒满 128 字节 DMA 方才响应的根因：
  逆向 `hal_uart_open`（0x108C03EC）发现，固件配置的 NVIC 中断号取自
  结构体 b4 字节：UART0 是 55（0x37）、UART1 是 56、UART2 是 57
  （I2C 0..4 为 50..54，后接 UART 55..57，定时器 58..59 连号）！
  原 QEMU 误把 UART0 接到 NVIC 22（实为 TX DMA 外设 ID），导致
  PL011 的接收超时中断 INT_RT 送不达 CPU。改正中断号为 55 后，
  单字符与回车即时触发超时中断并交付 NuttX 驱动，终端交互延迟达
  0.001 秒；同步更新 `emu/run.sh` 自动路由 q63 控制台至 UART0 socket。

- **2026-10-11 (2)（q63：GPU 出图、触摸/按键可用、偶发启动停滞根治）**：
  现在 q63 约 80 秒进入 OOBE 欢迎页，点箭头进"选择语言"，再进配对页。
  ①**GPU**：q63 的 VGLite 与 best1503 同一型号，改用现成的
  hw/misc/vsi_vglite.c（删掉临时的 vglite 骨架；客户 ID 属性 0x420）。
  q63 固件用标准 4.x 命令编码（CALL=0x6、NOP=0x8，与 vsi_vglite 一致；
  早先"CALL=0x18000000"的结论是错的）。补了线性/径向渐变（色带在
  paint 图像槽，系数在 0x0A03..0x0A0B，公式见文件头）和 clear（op 4）。
  帧缓冲 0x20160000/0x201FF100 双缓冲，GPU 经 DTCM AXI 别名 0x21xxxxxx
  访问；LCDC 与 LVGL 都是内存 B,G,R（开机 logo 本来就是蓝色）。
  ②**侧键**：两个键是 PMU GPIO 113（DOWN）/114（HOME，表冠），低有效，
  电平在 PMU 0xBC/0xBD；原来读 0 = 一直按着，输入管理器刷长按、事件队列
  溢出。bes_ispi 加 "pmu-gpio" 输入与 PMU GPIO 中断（每 8 脚一组，
  0xBE+{0,6,11,17}：+0 清、+1 使能；状态 0xDE/0xDF；"pmu-irq" 接 AP
  NVIC 23）。宿主 Home = HOME，Page Down = DOWN。
  ③**触摸**：驱动先读固件状态 0x50000918 要 bit14；抬起必须报点数 1；
  驱动在中断里关脚中断、读完才开，模型改为读走后 8ms 再拉低 INT、
  100ms 未读重发；CC 数据寄存器 0x5000093C 报非 0 免去开机校准烧录。
  ④**偶发启动停滞（约 1/6，开机 logo 不动）根因**：BES 定时器计数含 0
  态（hal_timer 为 n 拍写 Load = n - 1），QEMU 的 SP804 早一拍中断；
  hwtimer_handler 两次读系统定时器都判"未到"时只记 irq early 就返回、
  不再重设硬件定时器，该核所有 tickless 睡眠永不唤醒（sensor 卡在
  cp 侧 rptun 建链的 10ms 轮询里，cp/AP 随之停住；"控制台停在 connmand
  而系统继续跑"也是同一原因，NSH 睡死）。arm_timer 加 count-zero 属性，
  best1700 全部 SP804 打开。修后 12 次启动全部进 OOBE。
  ⑤仍未解决：cp 偶发 panic（蓝牙协议栈 bluelet_thread 写 HCI 时，约
  1/30，崩溃头被 AP 的大段转储冲掉，尚未抓到断言内容）→ AP 复位 → TEE
  "Start pmu reboot" 后模拟器停住；OOBE 文字颜色固件给的就是 #333333
  （是否设计如此待真机对照）；控制中心部分图标资源缺失；TEE 复位不生效。
  新增调试手段：-trace best1700_cmu_isys（核间邮箱）、arm_timer_*、
  raydium_ts_*；sensor 日志每次 hal_trace_printf 一行；QMP
  input-send-event 点屏（坐标 0..32767 映射到 466）、send-key 按键。

- **2026-10-11（q63：卡死/重启的连环原因查清，卡在 GPU）**：
  用户现象：MI logo → 黑屏 → 再 logo → HyperOS 图标 → 卡死。逐个查清：
  ①**内存被冲**（sensor 向量表清零 / TEE 向量表填满 0x75 → lockup 或
  TEE SecureFault 复位）：best1700_bthrom 的超级调用分三步（写参数指针、
  写调用号、读返回值）共用窗口状态，sensor 线程在两步之间被切换，另一
  线程的调用覆盖了参数指针，于是 memcpy/memset 用了失效栈上的垃圾参数
  （实抓 memcpy(0, 0x2305b55c, 0x2305b55c)）。改为一条 STRD 传指针+调用
  号、返回值写回调用者栈上 r0/r1 槽位、POP 取回，完全可重入。
  ②**卡在 connmand**（AP 100% "busy" 实为空闲循环）：链条是 nsh_main
  做 boardctl(FINALINIT)→触摸初始化→pm_relax→LCD pm_notify 等 LCD 锁；
  hpwork 持锁在关屏流程里读属性 persist.screen.touch_double_click，等
  kvdbd；kvdbd 打开 /misc 等 sensor 的 rpmsgfs 端点；sensor 的 rptun 线程
  和 AppBringUp 都睡在 usleep 里醒不来——sensor 的 tickless 定时器
  （SP804 @0x4008B000，one-shot）到期停住后驱动只写 Load 重装，QEMU 的
  arm_timer 不会因写 Load 重启停住的 one-shot 计数器（TRM 要求会）。
  修 hw/timer/arm_timer.c。
  ③**约 60 秒 sensor panic → 整机复位**：sensor 断言 rpmsg_virtio.c:386
  （等发往 cp 的 TX 缓冲超时）。cp 卡在 btdrv_SendData 里忙等"蓝牙核已
  取走"中断，cp 的 rpmsg 停摆。实现 cp↔蓝牙核邮箱：MCU_CMU 0x140/0x144
  （best1700_cmu.c）、BT RAM 0x68000000、指针表 0x68004000、BT_CMU
  0x6E030000，复用 bes.btss（新属性 rx-head-direct / data-done-shift /
  hci-channel / hci-revision，详见提交说明）。现在 cp 跑完厂商初始化，
  bluetoothd 把适配器开到 BLE+BR/EDR On。
  ④**约 145 秒 miwear 初始化看门狗 panic**（miwear_init.c:152，"wdt cb"）：
  miwear 初始化状态机停在 0；状态 1 的回调 sub_10BFF9DC（UI 框架初始化）
  里 vg_lite_init(480,480) 的 GPU 上电流程无限等待：
  `do { 延时 2,4,8…ms } while (0xB05 & ~读(0x04))`。GPU 是 VeriSilicon
  VGLite（GCNanoUltraV）：寄存器 0x401F0000，NVIC 33，命令/显存池
  0x22500000 起 512KB（sub_108ACBD8），vg_lite_init 核对芯片 ID
  （寄存器 0x20=0x265、0x24=0x2002、0x30=0x420，ITCM 0x13E3EC）。
  **下一步：VGLite 模型**（至少初始化通过、命令缓冲执行完回中断；要看
  到界面还需清屏/贴图/路径填充）。
  ⑤其它：TE 同时接 DSI0（显示线程在 DSI0 中断里等 TE）；Raydium RM6D03x
  触摸模型（探测通过，报点格式未验证，CC 校准握手超时 3 秒）；PL011 加
  RX DMA 请求，q63 控制台可输入（驱动没开 UART0 的 NVIC 22，目前凑满
  128 字节的 DMA 段才送达；23 号不是 UART，是 ISPI/PMU 一类）；
  emu/nsh.py 支持 q63（名字在 PSRAM 的 .rodata 副本里，自动推导别名）。
  ⑥仍未解决：TEE 打印 "Start pmu reboot" 后模拟器不真正复位（停在那里，
  这就是用户看到的"卡死"的最后形态）；sensor 的 /misc littlefs 挂载失败
  （NOR 里没有文件系统，"Corrupted dir pair"）；cp 经 DMA 写 BT RAM
  0x68400000 的控制器代码不执行。
  调试手段（本轮好用的）：emu/nsh.py inject + 凑满 128 字节的命令跑 ps /
  dumpstack <pid> / rptun dump all（输出在 RAM 日志，pmemsave PSRAM 后用
  "[MM/DD hh:mm:ss]" 正则提取）；sensor 日志在 BTH RAM（同样提取）；在
  TCB 里找指向自身栈的指针即保存的上下文（+0 为 SP），从 SP 往上扫返回
  地址得到真实调用链；-trace best1700_bth_log 看 sensor 早期 hal_trace。

- **2026-10-10 (5)（q63：eMMC、显示、cp 与 sensor 核；偶发重启查清）**：
  ①**存储**：SDHCI 0x40150000（NVIC 26，与 UART2 线或）+ QEMU eMMC
  （`-drive if=sd,index=0`）；emu/mkemmc.py 从固件包生成 GPT 镜像，
  run.sh 用 EMMC=；emu/emmcfs.py 从宿主读 exFAT（nv/data/userlog 里有
  log_service 的日志文件）。BES mmc 用 CID 序列号低半作 RCA，故序列号
  以 0x0001 结尾。顺带修了 sdhci 两处 ADMA2/SDMA 边界问题。
  ②**显示**：LCDC 0x401C0000（IRQ 27）→ DSI0 0x401D0000（IRQ 28，命令
  模式，DCS 写入面板状态）；面板 466x466 rm6d031，TE=GPIO 47；GPIO 三组
  0x40081000/0x40089000/0x4008A000（NVIC 2），PSC 中断字位置可配。现状：
  只出一帧开机 logo（layer0 RGB24 fb 0x21160000，位置 1,1），随后
  DISPOFF/SLPIN，之后不再出帧——UI 没有点亮屏幕，原因待查（1700 的
  VGLite 基址/中断也还没确认）。
  ③**cp = M55C1**（vela_cp.bin：BT host 服务 + WiFi 栈）：AP 把它装进
  PSRAM，在 M55C1 DTCM（总线 0x21C00000）写向量表，AON_CMU 0xB4=VTOR、
  0x1A4 bit1 放开即运行。核间中断字 MCU_CMU 0x90/0x94（每通道 data/done
  各一位，NVIC 88/89 收、91/92 发）；cp 自带 SP804 0x40084000/0x40003000
  与 DWT（cp 忙等 CYCCNT）。AP↔cp 的 hal_rmt_ipc + rptun 已通（cp 日志
  进 AP）；WiFi 设备永远 not ready，AP 等约 70 秒后放弃（在启动关键路径上）。
  ④**sensor = BTH 核 BTHC0**（vela_sensor.bin）：独立 8MB NOR
  （GD25LQ64，AP 视图 0x28000000，自身视图代码 0x0C000000/数据
  0x2C000000），入口 +0x10。没有 BTH 掩膜 ROM，自写替身
  best1700_bthrom.c（libc、trace printf、tick 换算、晶振 24MHz、16kHz
  系统定时器、NOR 大小与几何、睡眠=WFI，其余返回 0 并记 trace）。
  M55↔BTH 走 intersys 邮箱（MCU_CMU 0x98/0x9C、BTH_CMU 0x134/0x138，
  bit 16+g 数据、bit g 完成；M55 收 96/97 发 94/95，BTH 按 AP/cp 组
  73–76/65–68）。BTH 定时器 0x4008B000（19/20，16kHz）、0x50002000、
  0x50003000；IRQ 9 是 BTH 看门狗。AON RAM 扩到 128KB。sensor 的 syslog
  在 BTH RAM（0x23000000 起）里，是 "[sensor] ..." 行，pmemsave 后可读。
  AP 经 rpmsgfs 从 sensor 挂 /misc，所以没有 sensor 时 kvdbd 及其后的
  一切都卡住；有了 sensor 后 AP 能起到 miwear、chargerd、healthd、
  account、wpa_supplicant 等。cp↔sensor 的隧道还没起来。
  ⑤**电量计**：CW2215 @ I2C3（0x40076000）地址 0x64；没有它 AP 的低电
  检查会在充电循环里卡 20 分钟。
  ⑥**偶发重启（实测 1/3～1/2 的启动在 20 多秒时整机重启）已查清**：TEE 报
  "arm_securefault: Attribution unit violation"，其实是 AP 的
  board_reset() 故意写安全地址 0x24200000 让 TEE 复位整机（之前先打 10
  轮 cp/sensor/audio 的 PC/LR/SP）。AP 调 board_reset 是因为
  M55C0_2_BTHC0 通道收到 sensor 的 panic 消息（0xFFFFFF10）。sensor
  panic 的原因：AppBringUp 里 bes_switch_setup 在 sensor 的 I2C 总线 2
  上探测 SGM25062（地址 0x18，400kHz，先把 ioexpander 引脚 0x44 拉高），
  读寄存器 0x07 失败 → sgm25062_register 释放了实例，却没清全局指针
  0x2303ebf8；之后 sgm25062 的寄存器读写函数用这个悬空指针去拿锁，
  KASan 报 use-after-free 并断言。是否发生取决于时序，所以是偶发的。
  修法：让探测成功——sensor 侧 I2C 控制器（hal_i2c 表：id0
  0x50006000、id1 0x50007000、id2 0x50008000、id3 0x5000A000，每项后
  跟两个时钟号和中断号 39–42），总线 2 用 id 2；再挂一个 SGM25062 模型
  （0x07 读芯片 ID、0x69 写 0xAE 复位、0x02 按位配置）。奇怪的是 -d unimp
  里看不到 sensor 访问 0x50006000–0x5000A000，传输可能在碰硬件前就失败
  了，先查这一点。
  调试手段：在 m_helper.c 的 AUVIOL 分支临时打印 PC/地址并
  qemu_system_vmstop_request 暂停，再 pmemsave AP SRAM/BTH RAM 回溯。
  AP 的 rodata 字符串被拷到 PSRAM：运行地址 0x18xxxxxx = 镜像 VA +
  0x07FBADB4。
  下一步：SGM25062 + sensor I2C（消除重启）；显示不出帧；触摸（Raydium
  @ I2C0 0x40006000 地址 0x39）；cp 的 BT host → BT 控制器 → AstroBox；
  /misc（bth_misc littlefs）里的出厂属性。

- **2026-10-10 (4)（q63/best1700：TEE 启动并拉起 AP）**：
  BSP 来源：open-vela/vendor_bes 的 dev-ai-contest-2026 分支，
  boards/best1700_ep/aos_evb/configs/ap/{libnx_bestbsp_ap.a,
  libbeschip_ap.a,libbesboard_ap.a,_best1000.lds}（目标文件带符号与
  重定位，无调试信息；llvm-objdump 反汇编即可对照）。q63 = 小米手表
  S5（TEE 版本串 "NuttX Xiaomi Watch S5"）。
  ①**启动链**：AP 的 NuttX 按 TrustZone 非安全编译（新任务
  EXC_RETURN=0xFFFFFFAC），并经 NSC veneer 调 0x10165xxx 的安全服务；
  那里是 vela_tee.bin（签名镜像 v0x00050001 + AVB 尾，
  OTA_CODE_OFFSET=0xC0000，NuttX + OP-TEE）。现从 TEE 入口安全启动
  （-kernel 旁的 vela_tee.bin 自动装入，或 -M tee-image=），TEE 配好
  SAU/MPC/TGU 后拉起 AP。bes_image 支持签名头（按入口 stub 形态定位
  字段）和 AVB 尾。
  ②**地址位 26 = 安全别名**（0x04/0x24/0x44/0x54…），IDAU 按此报
  安全；TEE 的 SAU 只划 NS 区，靠 IDAU 留在安全态。SRAM、ITCM/boot
  RAM、外设容器都映射两份。新增 RAM：BTH RAM 0x23000000（TEE 启动栈
  0x23020000–0x23040000）、AON 邮箱 RAM 0x23C00000、PSRAM 0x18000000
  （NC 0x38000000，.data/.psram_data 在此）。
  ③**外设**：MCU_CMU/AON_CMU 新模型 best1700_cmu.c（时钟/复位 set/clr
  对、MEMSC 试锁、PLL 锁定 0xE4/0xE8、AXI 低功耗握手 0x1A0/0x1AC、
  6MHz 快定时器 CMU+0x288、16kHz 系统定时器 AON+0x50）；UART=PL011
  0x4000B000/0x4000C000/0x40011000（中断 22/24/26）；TIMER=SP804
  0x40083000（16kHz，58/59）/0x40005000（6MHz，72/73）；NOR 控制器
  0x40190000（bes_flash，GD25LQ128 双片 32MB）；PSRAM MC v6 0x40320000
  + PHY 0x40328000（bes_psram v6）；SIE-200 MPC 寄存器（不过滤）；
  M55 TGU 配置（0xE001E504/604）。NVIC 120 个外部中断。
  ④**ISPI 帧格式更正**：读 (reg<<18)|0x10000000、写
  (reg<<18)|(val<<2)、10 位地址，按片选（+0x04 bit[10:7]）分开寄存器；
  PMU=片选 0（1806P：reg0=0x18F0、reg1=0x0650），片选 7 reg0=0x1700，
  PSRAM 模拟 PHY=片选 5（reg 0x51 DLL 锁定），PMU reg 0x4F DCDC 校准
  即刻完成。
  ⑤**DMA / I2C**：AP 控制台经 DMA0 走 UART0（TX 请求 5、RX 4，实测
  通道配置），DMA 用 TCM 的 AXI 视图地址（DTCM 0x21000000、ITCM
  0x21800000）；三个 DMA（0x401A0000/0x401A8000/0x40260000，中断
  37–39）与五个 I2C（hal_i2c i2c_desc）已挂。
  现状：AP 打出启动日志、跑完 rc.sysinit（romfs/fatfs 挂载全失败：
  没有存储），rptun 连上 TEE；执行 "rptun start /dev/rptun/audio"
  （HiFi4 音频 DSP）时 AP 崩溃，AP 的复位路径故意写安全地址
  0x24200000 触发 SecureFault，TEE 打印 "Panic remote cpu ap" 后
  PMU 重启。
  下一步：①存储——0x40150000 是 SDHCI（16 位寄存器 0x2C/0x34–0x3A，
  CMD1 SEND_OP_COND、参数 0x40200000），q63 资源分区（vela_system/
  app/…）在 eMMC 上，需 eMMC 模型 + 分区表；②远端核（audio DSP、
  sensor、cp）的 rptun 启动——要么让其失败可回退，要么做最小握手。

- **2026-10-10 (3)（设备 profile 抽象 + best1700 IDA loader）**：
  设备 = 芯片 + 板子。芯片文件（best1503.c/best1700.c）只放芯片事实并以
  BesSoc 注册；板级外设（面板尺寸/TE 脚、触摸与 NFC 控制器的总线/地址/
  中断脚、SPI NAND、按键）放在 hw/arm/bes_board.c 的 profile 里：p67
  （小米手环 10 Pro，原 best1503.c 的板级常量连同出处搬入）、q63（只含
  已查实项）。新 machine `miwear` 按固件 boot info 的 CHIP= 选芯片、按
  镜像里的 "miwear.watch.<产品>" 选 profile；`best1503`/`best1700` 仍可
  强制芯片；三者都有 profile=（help 列出）/flash-image=/nand-image=。
  emu/run.sh 改用 `-M miwear`，可设 PROFILE=。按键走新设备 gpio-keys
  （宿主按键 / QMP send-key → GPIO 线，可设低有效）。p67 回归：自动选
  p67、NAND/触摸/蓝牙安装均正常。
  IDA：emu/ida/bes_vela.py 取代 best1503_vela.py（装到 ~/.idapro/loaders，
  旧文件改名 .bak），支持 cached 别名与 XIP 重合（q63 0x101C0000）、空
  .bss、best1700 用 ARMv8.1-M。q63 IDB：/Volumes/EXT0/Canopus/fwbins/
  xiaomi-q63-4.101.020/vela_ap.bin.i64。q63 已知启动路径：sub_101C03D0/
  sub_101C0474 把镜像代码拷到 0x130000/0x13C180/0x197D00（M55 ITCM 区，
  现由 bootram 承接），sub_101C1FE4 经 hal_memsc_lock（CMU+0xC0+id*4 读=
  试锁）与 ISPI@0x4000A000 读 0x7000 要求 bits[15:4]==0x170、读 0x0000
  比较 0x18F；ISPI 帧格式同 1503 但偏移 10 位、读标志 bit28。

- **2026-10-10 (2)（蓝牙空口时序 + 快应用安装）**：
  ㊿**快应用传输慢（9 KB/s）的根因 = 空口没有时间**：AstroBox 的 SAR 发送
  窗口故意超过手环通告的 TX_WIN=3（软上限 32~38 帧，手机 App 同样做法），
  靠空口速率把数据摊开。原 bes_btss_link 把对端 ACL 瞬间交给手环，300 KB
  一次涌入，手环接收路径丢掉大部分，主机只能逐帧重传；而手环 SAR v2 收到
  的未确认帧数不到阈值（sub_C7D59F8：≥2·TX_WIN/3 或 >4，TX_WIN 为主机在
  L1START_REQ 里给的值）时不立即 ACK，而是起 1200 ms 累积 ACK 定时器，于是
  每 12 KB 一片要等 1.2 s。现按 Core v5.3 Vol 2 Part B 6.7 的 EDR 3 Mb/s
  包计空口占用（3-DH1/3-DH3/3-DH5 最多 83/552/1021 字节、1/3/5 时隙，各加
  1 个回程时隙，625 µs/时隙），对端→手环的 ACL 等信道空闲再送，手环→对端
  的 ACL 也占信道。实测（AstroBox examples/emulator install）：300 KB
  32 s → 1.8 s，1.7 MB 9.7 s（约 170 KB/s），无重传。
  **InstallFailed** 与模拟器无关：社区资源 moe.mcns.Eclipse 261006.1 的
  manifest.json / manifest-watch.json 与包内 META-INF/CERT → hash.json 的
  SHA-256 不符（签名后被改过），手环日志 `app_verify_unzip failed: -1`、
  `rpk install failed: -100`；同一应用 261002.1 摘要一致、安装成功。

- **2026-10-10（NFC 控制器 + eSE，hw/misc/nci_nfcc.c）**：
  ㊻**42 秒卡顿的真正原因**：支付宝 alipay_select_channel（sub_C4D5DF4）往 NFC
  栈任务队列投消息 39（开通道）后等信号量 21 秒，csi_ese_open_channel
  （sub_C4D5EBC）失败再投 41（关通道）再等 21 秒。消息只在 nfc_stack_bridge_task
  （sub_C735A80）的主循环里处理，而主循环要等 nxp_stack_init 成功才进入，
  **init 失败则任务直接退出、永不收消息**——所以"不模拟 eSE、让它快速失败"
  也躲不掉 42 秒，必须让整个 NFC 栈初始化成功。
  ㊼**NFCEE 布局**（NCI 2.0，按发现顺序）：0x01 HCI 网络（接口 HCI access）、
  0xC0 eSE（APDU 接口）、0x10 NDEF NFCEE（上电禁用，主机 NFCEE_MODE_SET 打开）。
  依据：SeApi_Init（sub_C71CCB8）收集 SE 时跳过接口为 HCI access 的 NFCEE；
  HCI 的 NFCEE 配置（sub_C727430）把它算 1 个，再加 GET_CONFIG A0ED/A095/A0EC/
  A0D4 中非 0 的个数，多于已发现个数就开定时器等 20 秒（配置项 60）——原模型把
  eSE 报成 HCI access、A095 硬改 0、其余 NFCEE 用伪造状态 0x22 隐藏，正好踩中。
  NFCEE 使能后发 RF_NFCEE_DISCOVERY_REQ_NTF（格式按 sub_C72DF4C 解析）；
  EE 层 API 事件在 em_state=INIT 时被丢弃（sub_C72579C）。A0EB = eSE 的 HCI 会话
  标识，全 FF 表示 eSE 未就绪、主机每 50ms 轮询（sub_C7275D8），模型给非 FF 值。
  ㊽**HCI 网络（ETSI TS 102 622）**：主机控制器管理门（会话标识、白名单、
  HOST_LIST/HOST_TYPE_LIST、CREATE/DELETE/CLEAR_ALL_PIPE）、管道表；eSE 主机
  有身份管理门 0x05（GATES_LIST={05,30,41}）、APDU 应用门 0x30（打开管道/
  EVT_ABORT 时发 EVT_ATR，寄存器 MAX_C-APDU_SIZE/MAX_WAIT_TIME，C-APDU →
  R-APDU）、连接门 0x41（白名单允许后 eSE 自建到终端主机的管道，主控发
  ADM_NOTIFY_PIPE_CREATED，sub_C72663C 接收）。ETSI12 配置流程见 sub_C726300：
  身份管道 → GATES_LIST 含 0x30 → 建 APDU 管道 → EVT_ATR → 读寄存器 1、2 →
  须已有连接管道 → 成功。**管道号固定**：eSE 连接 0x16、APDU 0x19（SeApi 的 HCI
  回调 sub_C71C884、SeApi_WiredGetAtr sub_C71E67C 写死），UICC 连接 0x0A 保留。
  eSE 上没有装任何 applet：MANAGE CHANNEL 正常，SELECT 一律 6A82，所以支付宝
  "未开卡"并立即返回。ATR 历史字节第 8、10 字节是 nfc_exception_recovery 读的
  版本数字（"1"、"0"）。
  ㊾NFA 启用期间 NCI 层会写一个 32 字节、头为 `02 05 01 00` 的错位缓冲
  （其后 SET_CONFIG 左移一字节 + 旧数据），并当作 GID2/OID5 命令等响应，
  2 秒超时则 NFA_Enable 失败；模型按未知命令回 `42 05 01 05`。来源函数未定位。
  **实测**：NFC 栈约 1 秒初始化完成（NFA_Enable、HCI、EeRegister、ETSI12、
  电源链路全部无超时），ese_read_kv 在 00:00:05 返回（原 00:00:48），开机 16 秒
  已显示表盘。调试工具（未入库）：QMP 轮询 nfa_ee_cb/nfa_hci_cb/SeApi 状态。

- **2026-10-09 (5)（掉电休眠、熄屏/亮屏、时钟、NSH，未提交）**：
  ㊵**"上滑进启动器就崩溃"= CPU 掉电后永不唤醒**：空闲时 hal_sleep_deep_sleep
  （sub_C10B5B8）→ hal_cmu_lpu_sleep(4)（sub_C10AB50）在栈上存 CMU 寄存器，
  sub_C10BB78（RAM 中执行）保存 r4-r12/lr/PSP/MSPLIM/PSPLIM/PRIMASK/BASEPRI/
  FAULTMASK/CONTROL、SP 存 SRAM，再写 PSC +0x18=0xCAFE0000、+0x00=0xCAFE0001
  （MCU_PG_AUTO）、+0x10=0xCAFE0001（MCU_POWERDN_START）后 `b .` 等断电。唤醒
  = 内核复位，向量表取自 **AON_CMU +0x78**（固件写 SCB->VTOR|旧值低 7 位，
  RAM 函数经 sub_C0C22B0 调用）：startup sub_C0C1B08 在 SRAM 0x20076400 建
  向量表并把复位向量设为恢复函数。模拟器原先 PSC 落在 unimp → 永久死循环，
  画面停在最后一帧。现：**bes_psc**（新，写钥匙 0xCAFE，GPIO 中断子块仍归
  bes_gpio 覆盖映射）掉电时 arm_set_cpu_off，NVIC 有可取中断即
  arm_set_cpu_on_and_reset；**bes_aoncmu +0x78 → CPU init-svtor**（同
  iotkit-sysctl INITSVTOR 做法，系统复位清 0 回到 boot ROM 向量）。实测 4 分钟
  掉电/唤醒约 1400 次，启动器/引导页/应用列表正常。
  ㊶**系统时钟 2 倍速**：AON_TIMER0 实为 16384Hz（32.768K 二分频）：
  bes_freerun.c µs = ticks*1e6>>14，bes_oneshot.c 余量 /16384e6。原按 32768
  计数，uptime 走 62s/墙钟 30s；改后 61s/60s，5s 熄屏也恢复为真实 5s。
  ㊷**熄屏显示**：固件 5s 无操作发 0x51=0、DISPOFF、SLPIN（"panel sleep"）。
  bes_lcdc 改为帧写入面板 GRAM，控制台只在 SLPOUT+DISPON 时显示 GRAM，否则
  黑屏（原来一直显示最后一帧，像死机）。
  ㊸**触摸手势唤醒**（zinitix_ts）：熄屏挂起 sub_C3F6D6C 写 0x00AF 手势使能
  掩码 → 命令 0x0003 → 命令 0x0005（SLEEP）；挂起时中断处理 sub_C3F5F94 读
  0x0126 手势码：1=双击、4=上滑（受 touch_double_click/touch_slide_up 设置
  约束，二者都关时固件强制开双击）；恢复 sub_C3F67BC 重跑上电序列（0xC000=1…）。
  模型：SLEEP 后只做手势检测（500ms 内两次点击=双击，抬起点比按下点高 1/4
  屏=上滑），写 0xC000 / WAKEUP / SWRESET 退出。**宿主双击或上滑即可亮屏**。
  ㊹**NSH**：生产固件 rcS 末行 `exit` 使 init NSH 结束（nsh_main sub_C6EE76C →
  nsh_initialize 跑 rc.sysinit/rcS），串口特殊字符只开了 FORCE_PANIC(0x1F)/
  SIGINT（uart_check_special sub_C1ABC64），无 TTY_LAUNCH，miwear_product_load
  只在内置 JSON 损坏时失败，所有 system() 调用都是固定字符串——**没有合法的
  数据入口**。emu/nsh.py `inject`：机器 -S 启动，经 gdbstub 在 nx_bringup 创建
  init 任务后再调一次固件自己的 task_spawn（同一 attr/argv），拉起内建 `sh`
  （nsh_system，无 rcS，继承 /dev/console）。不改固件字节；task_spawn 调用点
  （"nsh_main" 字面量加载后的 BL）与 builtin 表（{name,prio,stack,main}）均从
  镜像结构定位，本机 band 9/9 Pro/10 Pro/11 七个版本均唯一命中。
  `NSH=5600 emu/run.sh <out> 0` + `emu/nsh.py console 5600`，提示符 `ap>`；
  属调试探针性质，与"只在硬件接口层工作"的红线擦边，按需开启。
  ㊺**cocoa**：GUI 模式加 show-cursor=on（宿主光标常显）；ui/cocoa.m 绝对坐标
  模式下按下即抓取并下发、接受激活窗口的点击（原来后台启动的窗口前一两次
  点击会被吞掉）。emu/ramlog.py 修复日志簇贴 PSRAM 末尾时输出为空。
  仍存在：NFC eSE ese_read_kv 超时使 UI 初始化停约 42s（㉟）。

- **2026-10-09 (4)（蓝牙空口端到端 + NOR 出厂数据，commit 20ce0d98fd..）**：
  ㊱**蓝牙桥端到端跑通**（AstroBox-NG 侧的 `bluetooth::emu` + `corelib` bind）：
  emu/run.sh `BT_PORT=<port>` 挂出 bes.btss 对端 H4 控制器；AstroBox 的
  `examples/emulator` 作主机栈：inquiry→page→SSP 配对（NoInputNoOutput，手环
  弹框确认）→加密→SDP 查 SPP 通道→RFCOMM→打开 SPP→miwear L1 握手，全部实测
  通过（`bonded`, `encryption mode 2`, `SPP open ch5`, `L1StartRsp`）。配对确认
  按钮用 emu/touch.py 点手环屏（code 一致即接受，NoInputNoOutput Just Works）。
  ㊲**NOR 出厂数据（emu/mknor.py + flash-image 常驻）**：手环蓝牙绑定要读
  `/misc/factory.prop`（littlefs "nv" 分区 @0xFE0000，固件自己的 mount/loader
  sub_C6FFC58）。缺失时 `get_did_` 失败、`miwear_did_get failed`，BindInfo 无
  设备身份。mknor.py 按固件实测几何（littlefs v2.1、8K×16、read/prog 512、
  name_max 32）写 did/psn/mac_bt。机器改为「-kernel 固件每启动装入 AP 分区、
  flash-image 只带 BL2+出厂分区」，一张 NOR 跨固件版本通用。实测 `/misc` 正常
  挂载、`fac_prop not need restore`、BindInfo 含 DID。
  ㊳**绑定模式阻塞（关键发现）**：此固件 miwear 配置 `verify=0`（.data
  0x200DEFE0: pid=0 verify=0 oob=4），手机 SPP 链路上 BindStartV2 一律走
  **SERVER_PSK 绑定**（sub_C7D0238：仅 conn 子类型 0x10 的板-板连接或
  verify==1 才走 local bind）。实测手环回的 BindInfoV2 verify_mode=ServerPsk。
  PSK 绑定要云端服务器私钥签名（sub_C7DAAC8 验签），**离线无法伪造**；AstroBox
  只实现了 APP_LOCAL（bind.rs），对不上。verify 是固件 .data 常量、非属性，
  不改固件本体改不了。→ 现实路线：把手环预置为「已绑定」（NV 写 miwear_token_key
  等）+ AstroBox 用导出的 authkey 走 AUTH（主流用法，连已绑定设备），而非首绑。
  ㊴**AstroBox 侧（/Users/searchstars/Documents/GitHub/ab/AstroBox-NG，未提交）**：
  `bluetooth::emu`（桥）+ `router`（按地址分流到桥/平台）+ `examples/emulator`；
  `corelib` 新增 `components/bind.rs`（P-256 ECDH + HKDF，local bind）、
  `create_device_with_options`、auth.rs 带 app_device_id。bind.rs 的 `pb::`
  别名与 prost 生成的 `pb` crate 撞名，已改 `proto::`（编译/单测通过）。

- **2026-10-09 (3)（触摸 + I2C 总线化，commits ba1b73bd2e..）**：
  ㉜**bes_i2c 改为真正的 QEMU I2C 总线主控**：首字节（空闲/STOP/RESTART 之后）发起 start_send/start_recv，逐字节 i2c_send/i2c_recv，STOP 结束；只有无模型应答的地址才退回通用 256B 寄存器文件（板上大量未建模芯片：传感器、充电、NFC……）。
  ㉝**Zinitix ZTW6xx 触摸（hw/input/zinitix_ts.c）**：全部来自固件 ztx/zinitix.c：16 位小端寄存器协议，2 字节无数据写 = 命令（0x0003 清中断）；上电序列 0xC000=1/读 0xCC00/命令 0xC004/0xC002=1/0xC001=1；识别 0xCC00 芯片号（驱动只接受 0xE623/0xE631）、0x09D3=9 后 0x09D2 读 12 字节其 [8..9]=硬件 ID、0x0012/0x0121/0x0013 固件版本；**0x012C 必须 = 0x55AA 且 0x0014 = 硬件 ID**，否则驱动 34 秒重刷触摸固件（ts_check_need_upgrade sub_C3F78AC）；**0x001C 必须 = 0x5A49**，否则 ESD 看门狗每 ~5s 复位触摸（sub_C3F815C）；报点 0x0080 读 10 字节 {status,finger_mask,ts,x,y,width,sub_status}，INT 低有效。数值按真机日志：chip 0xE623、HW 0x4702、fw 0x9/0x0/0x12。板级：bes_ts_zinitix_setup(sub_C3F84CC) 开 I2C5，配置 0x2CD0605C = {地址 0x20, 400kHz, INT pin 76, RESET pin 78}；电源 GPIO 52。宿主绝对坐标指针 → 面板坐标，左键=手指；QMP input-send-event 可脚本化（emu/touch.py tap/swipe）。
    验证：点"English" → 配对引导页；点击/上滑 → 二维码页（關於/系統操作）→ 系統操作（重啟/關機/恢復原廠設定）→ 关机确认框，全部正确渲染与响应。
  `GUI=1 emu/run.sh <out> 0` 打开 cocoa 窗口（鼠标即手指），秒数 0 = 不限时。
  ㉞**当前阻塞：配对**。未绑定设备只会停在开机向导（persist.setup_wizard 未置位，日志反复 "setupwizard_status is unbound"）。真机靠手机 App 经 BLE 绑定；AT 命令走蓝牙 lcproto，不是 UART；生产版 uart2 无 NSH 交互；system_cipher 只是用户锁屏密码页，不是隐藏入口。可选路线：①实现足够的 BT 控制器/主机侧以完成绑定协议（工作量大）；②在 emu/mknand.py 里生成"已出厂配置"的 /data（需要 yaffs2 + 属性库写入器，属于预置用户数据而非改固件）；③继续找设备侧合法入口。
  ㉟**NFC**：真机为 TMS NFC（NCI over I2C：CORE_RESET 20 00 01 00 → 60 00 01 01 ff / 40 00 01 00 ff …）。未建模时支付宝 eSE ese_read_kv 超时使 UI 初始化停约 42 秒，待建 NCI 控制器模型。
- **2026-10-09 (2)（显示链路打通 + GPU 光栅化，commits c12a95ed1b..43b4d96717）**：
  ㉗**GPIO v2 + PSC GPIO 中断（bes_gpio）、LCDC 帧路径与 TE、VGLite 前端**（c12a95ed1b）：见各文件头注释。TE = LCDC "te" 输出 → GPIO 47，60Hz。
  ㉘**LCDC 帧完成必须带传输完成状态位**（a3317ce06a）：fb 驱动帧完成路径 sub_C2263C0 要求 ISR 同时有帧结束位（bit21，另一接口模式 bit22，sub_C232294）和一个完成状态位（bit31/27 sub_C2322AC 或 bit30/26 sub_C2322B8），bit29/25/16 视为配置错误（sub_C2322C4）。只给 bit21 时驱动的 tx-busy 标志（dword_200FE148）不清 → dpm 线程（sub_C224ECC）永远停在"发送中"(state 3)、pan 队列不出队 → miwear_fbdev 的 FBIO_WAITFORVSYNC 永远阻塞 → 整个 UI 只出 1 帧。现在帧完成置 bit21|bit27（哪个完成位对应哪个输出通路固件里看不出，选 bit27 已注明）。
  ㉙**日志**：log_service 启动后 syslog 不再走 uart2，而是进 PSRAM 中的 RAM 环形缓冲。`emu/ramlog.py <qmp端口>` 经 QMP pmemsave 抓 PSRAM 并提取最长日志簇（找到完整开机日志）。
  ㉚**VGLite 光栅化（vsi_vglite）**（43b4d96717）：GPU = GCNanoUltraV（0x265），状态语义来自 LVGL 9 内置的 VGLite 4.x 用户态驱动（vg_lite.c / vg_lite_path.c / vg_lite_image.c，MIT），并用固件实际命令流核对：屏幕渲染目标 = SRAM 0x20000000 BGR888 336x480（stride 寄存器为 stride/3*4），字形为 S16 轮廓、矩阵缩放 1/256，圆角卡片为 FP32 路径；颜色 0x0A02 为 ABGR8888（LVGL 传入时已预乘）。实现：矩形 clear/blit（坐标生成器含透视、最近邻/双线性、border/pad/repeat/reflect、normal/multiply/stencil/recolor、CLUT 索引格式），路径填充（s8/s16/s32/fp32、二次/三次曲线展平、非零/奇偶规则、4 子扫描线精确水平覆盖率抗锯齿），pattern 绘制，预乘空间混合（全部 VGLite blend）写入所有 8/16/24/32 位目标格式。效果：开机向导"选择语言"页（简体中文/繁體中文/English）正确显示。
    预乘约定：0x0A00 bit28 时 PE 对输入色乘 alpha（与驱动变量名 in_premult 及 LVGL ThorVG 参考模拟器行为一致），0x0A25 bit24 对纹素预乘，输出按预乘存储（LVGL 把 GPU 画的 layer 标为预乘）。
  ㉛**其他观察**：①`**crash:1` 只表示 /data/offlinelog/ 下存在 crash* 文件（之前 miwear 软看门狗 panic 留下的），两个分支都会走到 boot-screen_up-end，UI 不是卡死而是静态页面无重绘；②NFC eSE（alipay ese_read_kv）超时使 UI 初始化停约 42 秒（00:00:09→00:00:51），待建模 NFC 或忽略；③触摸：真机为 Zinitix（zinitix_*、ztw662），模拟器里 Hynitron 探测失败、Zinitix 驱动在通用 I2C 从机上"成功"（chipid 0x1），需要真正的 ZTW662 模型。
- **2026-10-09（macOS 迁移 + NAND/DMA/PMU/PSRAMX/BT 子系统，commits 514b95ade1..d3ef54e19f）**：
  环境：macOS（Apple M4）+ Homebrew；`configure --target-list=arm-softmmu --disable-werror --enable-debug-info --disable-docs --disable-gtk --disable-vte --disable-gcrypt`（/usr/local 下有 x86_64 的 gtk/gcrypt 会被 pkg-config 误选导致链接失败）。运行脚本 `emu/run.sh <outdir> <秒> [qemu 参数]`（须在仓库根目录跑，机器里的相对路径 emu/nand.img）。调试辅助：`emu/hmp.py <qmp端口> <HMP命令>…`（配合 `-qmp tcp:127.0.0.1:端口,server,nowait`），`emu/sample_pc.py`。macOS 无 `timeout` 命令。
  **Apple Hypervisor 不可用**：HVF 只能跑 AArch64(EL1) 客户机，Cortex-M33 是 ARMv8-M（Thumb-only，M-profile 异常模型），Apple Silicon 不执行 AArch32/Thumb，只能 TCG。
  ⑳**勘误：romdrv「mask ROM 驻留驱动」结论错误，已删除**。0xC588xxx veneer 的 RAMX 目标（0x2765C0..0x2B08E8）是固件自己的启动 RAM-text 段：startup sub_C0C0A1C 把 flash 0x2C0C23A4..0x2C0FC6CC 复制到 SRAM 0x200765C0（例：veneer→0x296709 = flash 0xC0E24EC hal_cmu_clock_enable）。去掉 romdrv 后启动完全一致；romdrv 还会写固件 BSS（0x200B0908/0x200FFF00），属于针对固件的 hack。
  ㉑**SPI NAND 在 flash1（0x40500000），不在 flash0**：bes_nand_initialize(sub_C19FDF0) 以控制器 id 1 调 nandflash_init(sub_C221F4C)；控制器基址表 0x2009D8F0={0x40140000,0x40500000}。/dev/resource(romfs) 与 /dev/nand_data(yaffs)、coredump、nand_nv 都是该 NAND 的分区（bes_board_nand_initialize sub_C20AB44，按容量选 3 张分区表）。只有 512MB 表的 resource 分区（0x19000 页=200MB）装得下 vela_resource.bin（romfs 171754096B），故选固件 ID 表(0x2CCD4134)里的 c8-f4-c8（2048+64B 页、128KB 块、512MB）。实机贴片型号未知。
    NAND 协议（全部 IDA 实证，见 bes_flash.c 头注释）：READ ID 0x9F(ext,1 dummy)、GET/SET FEATURE 0x0F/0x1F(A0/B0/C0)、PAGE READ 0x13/PROGRAM EXEC 0x10/ERASE 0xD8（normal 模式，行地址在 REG_000[31:8]）、READ CACHE 0x03/0B/3B/6B/BB/EB（列地址在 REG_000[31:8]，长度=BLOCK_SIZE）、PROGRAM LOAD 0x02/0x32（TX FIFO 流式）、DIE SELECT 0xC2、DPD 0xB9/0xAB。坏块标记与非对齐尾部经 NC 窗口 0x30000000+列 直读页缓存。控制器：REG_01C bit1/bit2=TX/RX DMA 使能、[17:16]=RX DMA 宽度；DMA 数据口是 REG_008（双向）。
    后端：`emu/nand.img`（页序、每页 page+oob=2112B、擦除态 0xFF），由 `python3 emu/mknand.py --resource <vela_resource.bin>` 生成（出厂烧录步骤；resource 分区首页=0）。/data 挂载、yaffs checkpoint 跨次启动持久化已验证。
  ㉒**通用 DMA（bes_dma，新）**：0x40120000/0x40130000，8 通道，NVIC 3/4（hal_dma_open sub_C22E8D0）；PL080 兼容寄存器 + BES 2D 扩展块(0x200+ch*0x20，存储，2D 传输记 unimp)；外设请求线流控（flash1 = DMA1 请求线 6，固件外设表 0x2CCDA924）。
  ㉓**PMU 是 ISPI 设备 3，GPADC 物理建模**：模拟寄存器地址 = dev[15:12]|page[11:8]|off[7:0]，dev 是 SSPCR1[10:7] 片选（sub_C0E3EDC/sub_C0E4820）。WDT=0x3036/0x3037（pmu_shutdown 的 WDT 兜底 sub_C0E7C00），chip id=0x3000。GPADC：0x3050 转换完成位图（之前误作 0x10）、0x3056+ch 码值；码值由通道输入电压按 hal_gpadc_adc2volt_calib 默认曲线（800mV↔15968、1050mV↔20816）生成，输入电压是属性 gpadc-chN-mv（ch2/ch5 NTC 250mV≈25℃、ch3 VBUS、ch4 VBAT 经 1/4.9 分压，默认 4.0V）。修复了 PWONV 低电无充电器 → system("poweroff") → PMU WDT 2ms 的"重启循环"。
  ㉔**PSRAM 执行别名 0x1C000000**：startup sub_C0C0AC8 把代码复制到 0x3C000000，固件经 0x1C000000+off 执行（veneer sub_C588C30→0x1C058FAD）。缺失时取指落空 → 滑进 SRAM 向量表 → NOCP UsageFault → pmu_shutdown。
  ㉕**BT 子系统（bes_btss，新）+ ISIRQ 邮箱按 SDK 位定义建模**：位定义取自 OpenHarmony device_soc_bestechnic 的 best2003 reg_cmu（reg_0a0/0a4），与固件全部用法一致：bit2/3 MCU2BT IND（门铃）、13-16 掩码、17/18 BT2MCU 原始中断、19/20 MCU2BT 完成原始中断、21-24 掩码后 → NVIC 34/35/32/33。共享指针表 BT RAM 0xC0004000（[8ch+4]=AP 在途链表头地址，[8ch]=BT 链表头地址），描述符 {next,type@4,len@8,data@12}；BT→AP 数据放 BT_EXCH 0xD0210000（AP 实测不写），RX 完成=写 BT_CMU+0x54。HCI：H4 帧，Command Complete/Status 按 Core 规范返回参数长度；**控制器响应须有处理延迟（默认 1ms）**：驱动在 HCI_Reset 时清"已唤醒"标志，复位后重放的 0xFC77 完成事件会 nxsem_reset TX 信号量，瞬时应答会抹掉发送线程尚未消费的 TX-done → bthci_send 10s 超时 assert(bes_serial_hci2.c:343)。现在 bluetoothd 完整跑完 HCI 初始化（厂商命令、patch 下载、reset、本地信息、LE 设置）。
  ㉖**当前阻塞点**：miwear 应用软看门狗 wdt_cb（miwear_init.c:139，sub_C4B819C，1.2s 周期）panic —— miwear_wdt_feed（sub_C4B80D4）由 UI 主循环里的 21s 定时器喂，UI 循环（sub_C4B85B8 打开 /dev/fb0 初始化显示）没在跑，推测阻塞在显示链路（LCDC 帧传输/TE/完成中断）。下一步：显示链路。
  其他：真机 16MB NOR dump 在 ~/Library/Logs/moe.astralsight.astrobox/device/data/offlinelog/besdump（逻辑视图，AP 区与 vela_ap.bin 逐块一致；用时须拷贝，flash-image 会回写）；同目录 tmp.log 为真机运行日志（电池 4.16V、温度 27℃、SC7061 充电 IC REG60=0x02/REG01=0x07）。
  设计要求（用户）：同芯片换固件版本也要能启动，不得针对特定固件写死；芯片/板级事实做成设备属性。

- **2026-10-08（LCDC + 日志通道定位，待 commit）**：
  ⑰**LCDC @0x40200000（bes_lcdc）**：DBI/SPI 面板命令通道模型——写 +0x280 触发传输即置 ISR(+0x1C4) bit18(tx-done)，固件 sub_C232578 的轮询等待通过。**RM690B0 面板初始化命令流完整跑完**（COLMOD 0x3A→…→SLPOUT 0x11→DISPON 0x29，实测 4250 次传输），屏幕点亮。
  ⑱**UART 日志通道实证**：uart1 只有早期直出（rtc_enable 消息截断在 "nu" 是**固件自身的通道切换**，circbuf 0x200C8238 里消息完整）；**uart2 才是主日志口**（完整 boot 日志：App start、版本 3.101.043、Customer CONBINE_LTALM078_T3.101.043）；uart0 无输出。lldb_server 已接三路 uart。
  ⑲**当前阻塞点**：系统每轮 boot 到 "log_service [14:100]" 后重启（约 20 轮/次）。直接原因链：`bes_nand_initialize: bes nandflash_init error [2]` → /dev/nand_data 不存在 → mount /data 失败 → /data/persist.db IO error → log_service 阶段重启。**待做：NAND 通道**（bes_nand 挂在哪个控制器/怎么发命令，待逆向 sub_C221F4C 初始化序列）。
- **2026-10-08（c5f66b5a46）**：boot 循环重启根治 + I2C 子系统：
  ⑬**NOR RC 校准修复（boot 循环根治）**：bes_flash 旧的"addr==0x61 路由 NAND"判据是错的——NEW_CMD 模式（REG_004.NEW_CMD_EN）下 REG_000 bits[31:8] 不是地址而是 RX_EN|RX_LEN|TX_LINE|RX_LINE（reg_norflaship_v2.h:139-148）。NOR 校准发 0x9F 时 reg000=0x619F（RX_EN|rx_len=3）被误路由到 NAND 后端返回 NAND ID(c8-81-c8)，与期望 NOR ID(c8-60-17) 比对失败→11 个采样点全败→hal_norflash_open(sub_C0C3978) 错误码 5（ctx+0x14 实测）→bes_chip_flash_init(sub_C234488)→show_id_state RAM 副本（veneer 0xC588FC0→RAMX 0x2782B4）assert "opened"（行 2560，assert 入口 sub_C1E4180）→board_reset 循环。删路由后 open 成功（ctx[0].byte0=1 实测），不再重启。真 NAND 流量从未出现（NAND 探测在 board init 更后面），路由待真实流量再定。
  ⑭**I2C 主控 ×7**（DesignWare APB-I2C 兼容，SDK reg_i2cip.h）：I2C0-6 @0x40005000/6000/60000/61000/62000/63000/64000（IRQ 14/29/64/65/53/54/55）。**0x40064000 SDK 标 I2S2_BASE 但固件按 I2C id=6 用（STATUS@+0x70 实测），SDK 标错同 PSRAM/DSI 先例**。轮询模型：传输瞬时完成，TFE|TFNF 恒 1、ACT=0，从机=按地址的 256B 寄存器文件（首字节=寄存器指针）。修 SC7061 充电器驱动 hal_i2c_mst_write/read 忙等卡死。
  ⑮**I2C RX 必须环形 FIFO**：固件流水线式"发一批 READ 命令→读 1 个→再发 1 个"，线性 cnt/pos 模型写下标越过 8 字节数组踩 rx_cnt（RXFLR 读出 0xFFFFFFFD，drain 循环 40 亿次卡死）；改环形（rx_r/rx_w/rx_cnt）后正常。
  ⑯**PMU GPADC 默认 0xFFF**：vbat 通道读数高于 PWONV 3.6V 关机阈值。
  另：bes_chip_flash_init 的 flash 尺寸检查（sub_C588DD8→RAMX 0x2782A4）读 0x40084004=AON_TIMER0 计数器取负作 tick（实测反汇编 0x2009D3E0）；hal_i2c 超时阈值 0x4000 tick。
- **2026-10-08**：boot 推进跨越多道关卡，修复 12 项（commit 6afb5e6927）：
  ①**SP804 双路 IRQ 拆分**（`split-irq` 属性）：BES 硬件 dual-timer 两子定时器分接两路 NVIC（AON_TIMER00/01=45/46、MCU_TIMER00/01=12/13、MCU_TIMER10/11=27/28），原 OR 合并输出使 OS tick（AON_TIMER0 子2，oneshot+INTEN）中断落到未使能的 IRQ45，CPU 在 `hal_cmu_sleep`(sub_C10B5B8)→WFI 永久睡死；拆分后 IRQ46 pending 正常唤醒。
  ②**CMU 0x8C WAKEUP_CLK_CFG** bits[15:14]（LPU_STATUS_26M/PLL）恒报就绪（reg_cmu_best1503.h:562-572）：原回读写入值/报 0 使 `hal_cmu_lpu_busy/wait_26m_ready` 与 sub_C10AB50 的无超时忙等永久挂死。
  ③**CMU ISIRQ bit3 恒 0**：bt_drv_reg_op 各 API 的"等 BT 核响应"循环以 bit3 为继续条件，bit3=1 时 3ms×10 超时→crash_dump→ASSERT(0)；本模拟器不实现 BT 协处理器，bit3 恒 0 走"BT 不在线跳过"容错路径。
  ④**BT_RAM 0xC0000000(64KB) + BT_EXCH 0xD0210000(64KB) 挂载**（plat_addr_map_best1503.h:303-306）：缺失时 bt_drv bakeup_init 写 [0xC0004070]=备份指针被总线丢弃，backup_write 读回 0 当指针用 → 空指针写入 → MemManage(DACCVIOL, MMFAR=0) → board_reset(2)。
  ⑤**AON_CMU CHIP_ID=0x1503|(10<<16)**：固件 hal_get_metal_id（sub_C0C0BF0）rev=(CHIP_ID>>16)&0xF，映射表 10→6；metal id∈{0,1,2} 时 btdrv_hci_set_poweron_reg 走 "1503 A don't support BTC PD"→ASSERT(0)。
  ⑥**intersys IPC 虚拟 BT TX-done**：bes_cmu 监听 ISIRQ_SET bit2 门铃→置 TX-done 挂起 bit23→拉 NVIC32（IRQ 号由固件注册表 0x2CCDACC8 字节表 0x20-0x23 实证），hal_intersys_tx_irq_handler 清 ISIRQ_CLR bit0 确认；修 AP 发 IPC 消息后等 TX 完成回调（sub_C2291CC→i_1=1）死等。
  ⑦**bes_pmu 新设备 @0x40009000**（PL022/ISPI 主控窗口 + PMU 模拟芯片）：SSPDR 命令字协议（写 (off<<18)|(val<<2)、读 (off<<18)|(1<<26)）、分页寄存器文件（chip id 0x1530）、GPADC（0x3010 恒全通道 valid、0x3056+ch 通道码）、**WDT 定时器模型**（0x36=ms 周期、写=喂狗、0x37 bit7+8 武装、到期才复位——初版"写 0x37 立即复位"错误导致 rtc_enable 启用看门狗即循环复位）。GPADC vbat≥3.6V 后 PWONV "plug not in, will shutdown" 低电关机路径跳过。
  ⑧**AON reset 保留 BOOTMODE(0x38)/SW_INFO(0xF8)**（AON 域 WDT/软复位保持，跨复位传 boot 信息）。
  ⑨**boot ROM 驻留驱动 helper**（emu/romdrv 交叉编译 + machine 预加载到 SRAM 0x20066600）：AP thunk 表 0xC588xxx（LDR.W PC 绝对跳）的 31 项目标落在 RAMX 0x276660-0x2788FC，实体为芯片 mask ROM 驻留（AP/BL2/OTA/dump 均不含）；按 SDK hal_norflaship_v2.c/norflash_drv.c 语义用 NDK clang 交叉编译等价实现（libc memcpy/set/move + norflaship 寄存器级函数 + 未映射配置项 ret0/1 stub），**链接到 RAMX 地址空间**（首版 VMA 链接致 b.w 相对跳在 RAMX 执行时错位，已修正为 RAMX 基址）。
  ⑩**SPI NAND 通道**（bes_flash 0x61 路由）：/dev/nand_data（GD5F1GQ5 类，ID c8-81-c8 自固件 ID 表 0x2CCD400C）与代码 NOR 同挂 flash0 控制器（bes.nand0 日志实证非 flash1）；命令集 0x9F/0xFF/0x06/0x04/0x0F/0x1F/0x13/0x03/0x0B/0x02/0x84/0x10/0xD8，页 2112B（2048+64 oob）×64×1024=132MB **可写 mmap 稀疏后端**（emu/nand_data.img，不存在则创建填 0xFF）。
  ⑪调试基础设施：bes_pmu/bes_flash 的 PC/栈插装（current_cpu）、条件 watchpoint 工作流、NDK lldb + HTTP 服务（emu/lldb_server.py，QEMU 参数现含 flash-image=dump/besdump）。
  ⑫**真机 16MB flash dump**（dump/besdump）：AP 区与 OTA vela_ap.bin 4K 块比对 0 差异；BL2 区不含 mask ROM 驻留段（实证 thunk 实体只能在 mask ROM）。
  当前状态：boot 经 flash 校准（0x9f+0x61 探测为 flash 校准时序扫描，非 bes_nand mount 路径）→ NuttX init → PM 循环；循环重启根因锁定为 persist property（/data on SPI NAND）不可用——bes_board_nand_initialize（sub_C20AB44→sub_C19FDF0→sub_C222490→sub_C221F4C）未被调用（条件 watchpoint 实证 STRH 0xF00 从未执行），NuttX init 在 board 初始化序列前某处的 reset 点排查中。
- **2026-10-07 (4)**：flash 控制器完整重写并提交（commit `ecd31e69b5`，旧版被用户 revert 后重写）。关键事实全部经 SDK 原文 + 固件 IDA + 运行时日志三方核实：①**手环10 是双片 GD25LQ64C**（每片 8MB，JEDEC ID **c8-60-17**，boot info FLASH_DUAL_CHIP=1/FLASH_SIZE=0x1000000 → 逻辑 16MB；此前 c8-60-18 结论错误已纠正）；②双片 1x 线寄存器读 RX FIFO 为**半字节交织**（B0=(C1&F0)|(C0>>4)，B1=((C1&F)<<4)|(C0&F)，SDK norflash_drv.c:1505-1512 与固件 sub_C0C4E30 互证）；③**RX FIFO 弹出单位由 REG_204 CFG_RXSIZE 决定而非 CPU 访问宽度**（SDK read_rxfifo 字访问只消费低字节，旧实现按访问宽度弹 4 字节导致 empty_wait 死锁）；④普通模式（NEW_CMD_EN=0）RX 长度=REG_004 BLOCK_SIZE。新驱动：完整 0x400 寄存器窗、normal/extended 双事务模式、双片打包/逻辑视图、GD25Q32C 全命令集、**支持 -M flash-image=<file> 外部镜像后端**（启动加载、退出写回）。运行验证：reset 序列→双片读 ID→11 点采样延迟校准→SR1/SR2 读写→SET_BURST_WRAP 全部通过，固件推进到显示初始化（DSI 0x40168000 读循环，SDK plat_addr_map_best1503.h:284 DSI_BASE，待建模）。另确认：vela_resource.bin 为 romfs（164MB），在独立大容量存储上（非 16MB NOR），其控制器待固件访问时经 unimp 识别。
- **2026-10-07 (3)**：全部推倒重写（旧试错代码已 revert）。按 DESIGN.md 正式实现：bes_image 固件解析器（与 IDA loader 同逻辑，地址零硬编码）、best1503 machine（static 状态 + reset 回调 + flash 三别名 + SRAM/RAMX + PL011×3 + SP804×4 + unimp 兜底）、bes_stub 占位设备。编译通过，冒烟验证：固件正确启动推进到 AON_CMU MEMSC 卡点。已 commit（qemu-miwear, miwear-best1503 分支）。日志全部英文。MEMSC 语义已从 SDK 完全搞清（hal_memsc.c:31-89）。
- **2026-10-07 (2)**：QEMU 编译环境搭建完成（v11.1.2 stable / msys2 mingw64；踩坑：HOME 未设导致 meson 失败、缺 libcbor、缺 dtc、subproject 需 git）；初版 machine（后被推翻）。
- **2026-10-07**：初始化本文档。完成 Unicorn 路线全部验证实验并否决（SVC/EXC_RETURN/NVIC 残缺，详见 §3.2）；确定转 QEMU 路线；emu/bes_emu.py 保留为 MMIO 侦察工具；整理 SDK MMIO 布局与启动流程实测数据。
