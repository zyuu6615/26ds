# 01 · STM32F407 主控固件

车载平衡滚球系统的**主控固件**。负责循迹、双电机闭环、舵机摆杆控制、LCD 实时显示，
以及通过串口接收视觉模块的钢球坐标并闭环。

- **MCU**：STM32F407ZGT6
- **构建**：CMake ≥ 3.22 + Ninja + `arm-none-eabi-gcc`（C11）
- **调试**：OpenOCD + Cortex-Debug（VS Code 按 F5）
- **配置**：STM32CubeMX（`touch_board_host.ioc`）

> 本仓库原为独立仓库 `DS`，历史提交已保留。主分支为**位置环 + PD 控制**，
> 另有一条 **串级 PID（速度环 + 位置环）** 分支作为对照实现。

---

## 目录结构

```
01_firmware_stm32/
├── applications/          # 应用层：全部业务代码集中在这里
│   ├── app.c/.h           #   调度入口：10ms 内环 / 20ms 外环 / 显示
│   ├── task.c/.h          #   六个赛题科目的状态机与参数
│   ├── tracking.c/.h      #   循迹外环：灰度质心 -> 转向 PID -> 差速
│   ├── pid.c/.h           #   通用位置式 PID（微分先行 + 积分限幅）
│   ├── motor.c/.h         #   TB6612FNG 双路电机驱动
│   ├── encoder.c/.h       #   TIM1/TIM3 正交编码器，软件补 16→32 位回绕
│   ├── grayscale.c/.h     #   I2C 8 路灰度传感器
│   ├── servo.c/.h         #   舵机 PWM 驱动 + 往复自检演示
│   ├── vision.c/.h        #   UART4 视觉协议接收（环形缓冲 + 超时判活）
│   ├── ball.c/.h          #   钢球位置环 PD + 加速度前馈
│   ├── key.c/.h           #   4 路按键消抖与边沿检测
│   └── ball_notes.md      #   球杆闭环调参实测记录（踩坑复盘）
├── Core/                  # CubeMX 生成的 HAL 初始化与中断
├── Drivers/               # CMSIS + STM32F4xx HAL 驱动（ST 官方，未改动）
├── cmake/                 # 工具链文件与 CubeMX 源码列表
├── lcd_driver/            # SPI LCD 驱动 + GB2312 全字库（见下方编码说明）
├── tools/gen_gb2312_font.py  # 字库生成脚本
├── openocd.cfg            # 下载器配置
└── STM32F407XX_FLASH.ld   # 链接脚本
```

---

## 调度层次

调度是严格的**时间片 + 串级**结构（见 `app.c` 文件头）：

| 周期 | 任务 | 说明 |
| :--- | :--- | :--- |
| 10ms | `Key_Scan()` | 按键消抖（连续 3 次相同电平 = 30ms） |
| 10ms | 速度内环 | 编码器 → 速度 PID → PWM |
| 20ms | `Task_Update()` | 按键事件、计时、里程、终点判定、分段状态机 |
| 20ms | `Track_Update()` | 8 路灰度 → 加权质心 → 转向 PID → 左右目标转速 |
| 20ms | `Ball_Update()` | 读取新帧 → 位置环 PD → 舵机脉宽 |

> **为什么内环必须比外环快**：外环算出的目标转速若还未被执行完就被下一拍改写，
> 等效于给整个回路引入额外延迟，系统会自激振荡。`APP_OUTER_EVERY` 用
> `TRACK_PERIOD_MS / APP_CTRL_MS` 计算而非硬编码，避免改一处忘另一处。

---

## 引脚分配

| 功能 | 引脚 | 外设 / 复用 |
| :--- | :--- | :--- |
| 左电机 PWM | PC6 | TIM8_CH1（20kHz） |
| 右电机 PWM | PC7 | TIM8_CH2（20kHz） |
| 左电机方向 | PF0 / PF1 | GPIO（AIN1 / AIN2） |
| 右电机方向 | PF2 / PF3 | GPIO（BIN1 / BIN2） |
| 驱动使能 | PF4 | GPIO（STBY，低电平待机） |
| 左编码器 | PB4 / PA7 | TIM3_CH1 / CH2（AF2，TI12 四倍频） |
| 右编码器 | PE9 / PE11 | TIM1_CH1 / CH2（AF1，TI12 四倍频） |
| 灰度传感器 | PB8 / PB9 | I2C1（7 位地址 0x4C，5V 上拉） |
| 舵机 | PD15 | TIM4_CH4（50Hz，500–2500us） |
| 视觉串口 | PA0 / PA1 | UART4（115200 8N1，中断接收） |
| LCD | — | SPI（240×320，见 `lcd_driver/`） |
| 按键 | PC2 / PC0 / PF9 / PF7 | KEY1–KEY4，内部上拉 |

---

## 关键实现

### 循迹外环（`tracking.c`）

加权质心求横向偏差 → 转向 PID → 左右轮差速。几处实测定下来的关键参数：

| 参数 | 值 | 依据 |
| :--- | :--- | :--- |
| 探头间距 | 12mm | 实测。位置数组 ±6/±18/±30/±42mm |
| `TRACK_WEIGHT_NOISE_TH` | 50 | 远端探头信噪比差但力臂长，直接排除而非滤波（不引入相位滞后） |
| `TRACK_STEER_KP/KD` | 2.0 / 0.25 | 探头间距从 10 改 12mm 后，同样的位置偏差算出的 mm 数大 1.2 倍，增益需按比例缩小 |
| `TRACK_BASE_RPM` | 150 | 6.14m 赛道，100rpm 跑 20s；要进 15s 需平均 ≥117rpm |
| `TRACK_CURVE_SLOWDOWN` | 1.2 rpm/mm | 偏差越大弯越急，按比例减速；高速循迹能过弯的关键 |
| `TRACK_CROSS_MIN_CH` | 4 | 终点横线宽 50mm，居中只覆盖 4 路探头；原先设 7 永远凑不满，正是车到 A 点停不下来的原因 |

**横线（起点/终点）判据**用两个条件与急弯区分：同时变黑的路数 ≥4 **且** 偏差 ≤20mm。
急弯时线斜穿阵列虽也能点亮 3~4 路，但质心明显偏向一侧。

### 钢球位置环（`ball.c`）

摆杆倾角决定球的**加速度**（双积分对象），所以必须带微分项。

- `BALL_KP = 25.0`，`BALL_KD = 45.0`，`BALL_KI = 0`（纯 PD）
- 输出限幅 ±300us，速度项 LPF 系数 0.30
- **加速度前馈**：`2.0 us/(rpm·s⁻¹)`，车辆加速时反向补偿惯性力
- **dt 钳位** `[10ms, 150ms]`：微分对时间噪声极敏感
- **帧同步**：以 `Vision_GetFrameCount()` 变化为触发，每拍只消费一个新帧，
  避免拿同一个陈旧坐标反复做微分
- **断链回中**：超过 200ms 无有效帧立即回水平位。拿着过期坐标控制比不控制更危险

舵机水平点 `SERVO_LEVEL_US = 1588` 是实测整定值。**水平点偏移直接决定目标点是否可达**——
偏差大于起步门槛时，两个停止点会落在目标同一侧。

### 视觉协议（`vision.c`）

```
$BALL,<valid>,<x_cm>,<vx_pixel_s>,<confidence>,<frame_time_ms>[,<touch_cm>]*\n
```

- UART4 中断 + 512B 环形缓冲（大小取 2 的幂，用掩码取模代替取余）
- 单帧长度上限 64B，超长直接丢弃，防止一个坏字节把解析器带偏
- `VISION_TIMEOUT_MS = 200`，超时判定断链
- 视觉端时间戳只能用于判断视觉端自身是否卡帧，**不可与本机 `HAL_GetTick()` 做差**

### 通用 PID（`pid.c`）

两个容易出事的工程细节：

1. **微分作用在测量值而非误差上**。目标值阶跃时，误差微分会产生巨大尖峰
   （derivative kick）；作用在测量值上即可消除，而目标不变时两者完全等价。
2. **积分独立限幅（抗积分饱和）**。电机堵转时积分会累加到天文数字，
   负载恢复后需要很久才能退饱和，表现为长时间失控。

### 编码器（`encoder.c`）

TIM1/TIM3 均为 16 位，硬件 CNT 在 0~65535 回绕；软件补成 32 位累计值。
`ENCODER_COUNTS_PER_REV = 500ppr × 4 × 28 = 56000`（MG513 减速电机 + GMR 编码器）。
MG513 空载 370rpm 时约 345000 计数/秒，即 **95ms 必须读一次**，5~20ms 的控制周期余量充足。

---

## 构建与烧录

```bash
cmake --preset Debug
cmake --build --preset Debug
```

或使用 VS Code：任务 `OpenOCD: flash`（依赖 `CMake: build`，用 `openocd.cfg` 下载并复位）。

**依赖**：`arm-none-eabi-gcc`、CMake ≥ 3.22、Ninja、OpenOCD。
`CMakeLists.txt` 中的 `-u _printf_float` 不可删除——`LCD_DisplayDecimals()` 依赖
newlib-nano 的浮点 printf 支持。

---

## ⚠ 关于 `lcd_driver/` 的文件编码

该目录下的文件是 **GB2312/GBK 编码**，这是**刻意的**，不要转成 UTF-8：

| 文件 | 原因 |
| :--- | :--- |
| `gb2312_font.c` | 由 `tools/gen_gb2312_font.py` 以 `gb2312` 编码生成，字模索引按 GB2312 区位码排列（`addr = (GBH-0xA1)*94 + (GBL-0xA1)`），文件头中文注释也是 GB2312 字节 |
| `lcd_fonts.c/.h`、`lcd_image.c/.h` | 第三方（反客科技）字模数据与驱动，中文字模以 GBK 字符串作为索引，保持原编码以免字节序列被改动 |

在 GitHub 网页上这些文件的中文注释会显示为乱码，属**预期现象**。本地查看请指定编码：

```powershell
Get-Content .\lcd_driver\lcd_fonts.h -Encoding Default
```

```bash
iconv -f GBK -t UTF-8 lcd_driver/lcd_fonts.h   # Linux / macOS
```

`tools/gen_gb2312_font.py` 中的 `out_c` 是作者本机的绝对路径，重新生成前请改成
你本地的路径。该脚本需要 `Pillow` 与 Windows 自带的 `simsun.ttc`。

---

## 补充阅读

- [`applications/ball_notes.md`](applications/ball_notes.md) —— 球杆闭环的实测记录与踩坑复盘，
  包含"两个停止点落在目标同一侧意味着什么"这类从实测反推出来的判据。
- 各模块头文件都有详细的设计说明注释，建议从 `applications/app.h` 和 `task.h` 开始读。
