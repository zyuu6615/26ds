# 02 · MaixCAM2 视觉模块

平衡滚球系统的视觉前端。在 MaixCAM2 上用 NPU 检测钢珠，换算成摆杆轴线上的
厘米位置，通过串口发给 STM32 主控。

**只负责识别与上报，不参与运动控制。**

- **平台**：MaixCAM2（AX 系列 SoC，带 NPU）
- **框架**：MaixPy
- **模型**：YOLO26 单类检测（`ball`），640×160 输入
- **通信**：UART `/dev/ttyS4` @ 115200 8N1

## 文件

| 文件 | 作用 |
| :--- | :--- |
| [`main.py`](main.py) | 主程序：采集、推理、轴线标定、触摸交互、串口上报、屏幕叠加 |
| [`hybrid_tracker.py`](hybrid_tracker.py) | `HybridBallTracker` —— 多级容错跟踪 |
| [`ball_position.py`](ball_position.py) | 像素 → 物理量换算与自适应 α-β 滤波 |
| [`app.yaml`](app.yaml) | MaixPy 应用打包描述 |
| `models/` | 模型描述文件（`*.mud`）。权重 `*.axmodel` 未入库 |

代码本身注释极简，阈值与标定值都维护在本文档的参数表里。

## 处理流程

```
相机帧 → NPU YOLO26 → 候选筛选 → [丢失时] 多级降级跟踪
                                        │
        像素坐标 → 正交投影到标定轴 → 比例 → cm
                                        │
        自适应 α-β 滤波 → 串口 $BALL → STM32
```

正常帧只跑 YOLO。丢框时按代价从低到高降级：
局部模板匹配 → CLAHE 亮度增强后重检 → 低频全画面模板重定位。
未采用全图 Hough 与 FFT，边缘算力不足。

## 参数表

### 标定（`main.py`）

| 常量 | 值 | 说明 |
| :--- | :--- | :--- |
| `AXIS_START_PX` | `(67, 80)` | 摆杆轴线左端，量在**模型输入图**上 |
| `AXIS_END_PX` | `(392, 81)` | 右端像素坐标 |
| `AXIS_START_CM` | `2.5` | 左端实物位置（cm） |
| `AXIS_END_CM` | `15.5` | 右端实物位置（cm） |
| `LENS_CORR_STRENGTH` | `0.6` | 镜头畸变校正强度 |

换模型分辨率必须重新量这四个标定值。

### 检测与降级（`hybrid_tracker.py`）

| 参数 | 值 | 说明 |
| :--- | :--- | :--- |
| `raw_confidence` | `0.18` | 原图门限，压低先拿候选 |
| `enhanced_confidence` | `0.15` | 增强图门限 |
| `acquire_confidence` | `0.28` | 冷启动门限（无历史位置可参考） |
| `iou_threshold` | `0.45` | NMS 阈值 |
| `enhanced_retry_interval` | `2` | 增强图重检间隔（帧） |
| `global_template_interval` | `3` | 全画面模板搜索间隔 |
| `template_refresh_interval` | `5` | 模板刷新间隔 |
| `template_threshold` | `0.48` | 局部模板匹配阈值 |
| `global_template_threshold` | `0.60` | 全画面模板阈值 |
| `reset_ms` | `900` | 失联超时后整体复位 |

模板验证用 CLAHE 后的相对纹理区分钢珠与水管端帽（都是圆形暗块）：
内圈灰度标准差 ≥ `51.0` 且 P90−P10 ≥ `138.0`。这两个值在赛场光照下标定。

亮度归一化：整帧均值 `<95` 或 `>170` 时做 gamma 校正（目标 `125/255`，
钳位 `[0.55, 1.75]`）；否则用 CLAHE 拉 LAB 的 L 通道。

### 滤波（`ball_position.py`）

| 参数 | 值 |
| :--- | :--- |
| α（静止 → 快动） | `0.20 → 0.95` |
| β | `0.01 → 0.12` |
| `dt` 钳位 | `[5ms, 100ms]` |
| 重置超时 | `250ms` |
| 速度平滑 | `0.65 / 0.35` |

## 串口协议

```
$BALL,<valid>,<x_cm>,<vx_pixel_s>,<confidence>,<frame_time_ms>[,<touch_cm>]*\n
```

`frame_time_ms` 是视觉模块自己的时间戳，只能用于判断视觉端是否卡帧，
不可与 STM32 的 `HAL_GetTick()` 做差。`touch_cm` 是触摸屏指定的目标位置。
主控侧解析见 [`../01_firmware_stm32/applications/vision.c`](../01_firmware_stm32/applications/vision.c)。

## 模型权重

`models/` 下只保留了可读的 `.mud` 描述文件，权重 `*.axmodel`（约 2.6MB/个）未入库。

自行复现：用 [`../04_dataset_tools/`](../04_dataset_tools/) 抽帧 → 标注单类 `ball`
→ 训练 YOLO26 → 转成 AX 系列 NPU 格式 → 连同 `.mud` 放进 `models/<name>/`
→ 更新 `main.py` 里的 `MODEL_PATH`，**并重新标定 `AXIS_*` 四个值**。

## 运行

把 3 个 `.py`、`app.yaml` 与 `models/` 一起按 MaixPy 应用打包后推送到设备。
调试开关都在 `main.py` 顶部（`DEBUG_LOG` / `DRAW_DEBUG` / `SHOW_FPS` /
`USE_WEBRTC` / `PRINT_PROTOCOL` / `TOUCH_*` / `LENS_CORR_*`）。
