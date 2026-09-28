# 车载平衡滚球运动控制系统

> 2026 年全国大学生电子设计竞赛（TI 杯）赛区赛暨模拟电子系统设计专题赛选拔赛
> **H 题 · 省一等奖**

一辆四轮小车沿黑线循迹行驶，同时用一根舵机驱动的摆杆把钢珠稳定在指定位置。

```
MaixCAM2（NPU 视觉）──UART 115200──► STM32F407（控制）──► 舵机 / 双电机
     识别钢珠位置                     球杆闭环 + 循迹 + 显示
```

## 仓库结构

| 目录 | 内容 |
| :--- | :--- |
| [`01_firmware_stm32/`](01_firmware_stm32/) | 主控固件：循迹、双电机串级 PID、舵机、LCD、视觉串口 |
| [`02_vision_maixcam2/`](02_vision_maixcam2/) | MaixCAM2 视觉：NPU YOLO26 检测钢珠 + 多级容错跟踪 |
| [`03_mechanical_3d/`](03_mechanical_3d/) | 整车与摆杆机构 3D 模型（SolidWorks + 可打印 STEP） |
| [`04_dataset_tools/`](04_dataset_tools/) | 训练数据制作（视频抽帧）与标注格式说明 |

各模块的构建方式、参数含义与设计说明见各自的 README。

## 赛题任务

| 科目 | 要求 |
| :--- | :--- |
| 任务一 | 舵机在安全行程内往复摆动，车不动 |
| 任务二 | 从 A 点顺时针循线一圈停回 A 点，≤20s，停车偏差 ≤2cm |
| 任务三 | 车静止，把钢珠从中心 O 送到 +5cm 再折返 −5cm |
| 任务四 | 带球顺时针通过 B 点，≤8s |
| 任务五 | 球置中心 O，整圈通过 A，≤30s |
| 任务六 | 球置于摆杆任意指定位置并稳定在该位置附近 |

六科目全部实现。控制参数见 `01_firmware_stm32/applications/task.h`。

## 致谢

方案调研阶段参考了 H 题相关的开源实现

第三方组件：MaixPy / MaixCAM2 SDK（Sipeed）、OpenCV、Ultralytics YOLO、
X-AnyLabeling、ST HAL 与 CMSIS、反客科技 LCD 驱动与字模。

## 许可

原创代码以 MIT 协议开源，见 [LICENSE](LICENSE)。
`01_firmware_stm32/lcd_driver/` 内含第三方字模数据，版权归原作者；
SolidWorks 标准件参考模型仅供学习交流。