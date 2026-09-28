# 01 · STM32F407 主控固件

平衡滚球系统的主控。负责循迹、双电机闭环、舵机摆杆控制、LCD 显示，
以及接收 MaixCAM2 的钢珠坐标并做球杆闭环。

- **MCU**：STM32F407ZGT6
- **构建**：CMake ≥ 3.22 + Ninja + `arm-none-eabi-gcc`
- **调试**：OpenOCD + Cortex-Debug

## 目录

```
applications/          应用层，业务代码都在这里
  app.c/.h             调度入口：10ms 内环 / 20ms 外环 / 显示
  task.c/.h            六个赛题科目的状态机与参数
  tracking.c/.h        循迹外环：灰度质心 -> 转向 PID -> 差速
  pid.c/.h             位置式 PID（微分先行 + 积分限幅）
  motor.c/.h           TB6612FNG 双路电机驱动
  encoder.c/.h         TIM1/TIM3 正交编码器，软件补 16->32 位回绕
  grayscale.c/.h       I2C 8 路灰度传感器
  servo.c/.h           舵机 PWM 驱动 + 往复自检演示
  vision.c/.h          UART4 视觉协议接收
  ball.c/.h            钢球位置环 PD + 加速度前馈
  key.c/.h             4 路按键消抖与边沿检测
  ball_notes.md        球杆闭环调参实测记录
Core/                  CubeMX 生成的 HAL 初始化与中断
Drivers/               CMSIS + STM32F4xx HAL（ST 官方，未改动）
cmake/                 工具链文件与源码列表
lcd_driver/            SPI LCD 驱动 + GB2312 全字库（见下方编码说明）
tools/gen_gb2312_font.py   字库生成脚本
openocd.cfg            下载器配置
STM32F407XX_FLASH.ld   链接脚本
```

## 调度层次

| 周期 | 任务 |
| :--- | :--- |
| 10ms | `Key_Scan()`、速度内环（编码器 → 速度 PID → PWM） |
| 20ms | `Track_Update()` 循迹外环、`Task_Update()` 任务调度、`Ball_Update()` 球杆闭环 |

内环周期必须快于外环，两者由 `APP_OUTER_EVERY` 关联，改一处不会漏另一处。

## 引脚分配

| 功能 | 引脚 | 复用 |
| :--- | :--- | :--- |
| 左/右电机 PWM | PC6 / PC7 | TIM8_CH1 / CH2（20kHz） |
| 左/右电机方向 | PF0/PF1、PF2/PF3 | GPIO |
| 驱动使能 STBY | PF4 | GPIO |
| 左编码器 | PB4 / PA7 | TIM3_CH1 / CH2（AF2） |
| 右编码器 | PE9 / PE11 | TIM1_CH1 / CH2（AF1） |
| 灰度传感器 | PB8 / PB9 | I2C1（地址 0x4C） |
| 舵机 | PD15 | TIM4_CH4（50Hz，500–2500us） |
| 视觉串口 | PA0 / PA1 | UART4（115200 8N1） |
| 按键 | PC2 / PC0 / PF9 / PF7 | KEY1–KEY4 |

## 视觉协议

```
$BALL,<valid>,<x_cm>,<vx_pixel_s>,<confidence>,<frame_time_ms>[,<touch_cm>]*\n
```

`vision.c` 用 UART4 中断 + 512B 环形缓冲接收，单帧上限 64B，
超过 `VISION_TIMEOUT_MS`(200ms) 无有效帧则判定断链。

## 构建

```bash
cmake --preset Debug
cmake --build --preset Debug
```

或用 VS Code 任务 `OpenOCD: flash`。依赖 `arm-none-eabi-gcc`、CMake ≥ 3.22、Ninja、OpenOCD。

`CMakeLists.txt` 里的 `-u _printf_float` 不可删除，`LCD_DisplayDecimals()` 依赖它。

