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
- [ ] 越过配对（未绑定 → 一直停在开机向导；见 7. ㉞）
- [ ] 快应用环境验证

## 7. 最新变更

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
