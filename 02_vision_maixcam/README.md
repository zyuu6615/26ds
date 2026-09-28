# 02 · MaixCAM 视觉模块

车载平衡滚球系统的**视觉前端**。运行在 MaixCAM / MaixCAM2 上，用 NPU 检测钢珠，
把像素坐标换算成摆杆轴线上的物理位置（cm），并通过串口发给 STM32 主控。

**本模块只负责识别与上报，不参与运动控制。**

- **平台**：MaixCAM / MaixCAM2（AX 系列 SoC，带 NPU）
- **框架**：MaixPy（`maix` 包）
- **模型**：YOLO26 单类检测（`ball`），NPU 加速
- **通信**：UART `/dev/ttyS4` @ 115200 8N1

---

## 文件说明

| 文件 | 作用 |
| :--- | :--- |
| [`main.py`](main.py) | 主程序：相机采集、模型推理、轴线标定、触摸交互、串口上报、屏幕调试叠加 |
| [`hybrid_tracker.py`](hybrid_tracker.py) | `HybridBallTracker` —— NPU + CPU 多级容错跟踪器 |
| [`ball_position.py`](ball_position.py) | 像素 → 物理量的几何换算与自适应 α-β 滤波器 |
| [`app.yaml`](app.yaml) | MaixPy 应用打包描述（id / 版本 / 文件清单） |
| `models/` | 模型描述文件（`*.mud`）。**权重 `*.axmodel` 未入库**，见下方说明 |

> **代码本身没有注释**，所有阈值、标定值和算法取舍都集中在本 README 里维护。
> 改参数前先看下面的「参数总表」。

---

## 参数总表

### 标定（`main.py`）

| 常量 | 值 | 说明 |
| :--- | :--- | :--- |
| `AXIS_START_PX` | `(67, 80)` | 摆杆轴线左端，量在**模型输入图**上，不是相机原图 |
| `AXIS_END_PX` | `(392, 81)` | 右端像素坐标 |
| `AXIS_START_CM` | `2.5` | 左端对应的实物位置（cm） |
| `AXIS_END_CM` | `15.5` | 右端对应的实物位置（cm） |
| `LENS_CORR_STRENGTH` | `0.6` | 镜头畸变校正强度。不校正时画面边缘的摆杆会弯成弧线 |

**换模型分辨率必须重新量这四个数。** 640×160 模型的 y 坐标在 80 附近；
若换成 480×160，`AXIS_*_PX` 整体要重标。

### 检测门限（`hybrid_tracker.py`）

| 参数 | 值 | 说明 |
| :--- | :--- | :--- |
| `raw_confidence` | `0.18` | 原图 YOLO 门限。压得很低，先拿一堆候选再靠位置/尺寸挑 |
| `enhanced_confidence` | `0.15` | 增强图门限。画面经归一化后噪声更大，所以比原图更低 |
| `acquire_confidence` | `0.28` | 冷启动门限。没有任何历史位置可参考，只能靠分数，所以最高 |
| `iou_threshold` | `0.45` | NMS 抑制阈值 |
| `position_gate` | `max(48*scale, size*4.5)` | 允许的预测位移半径 |
| `score < 0.38` 且超出 gate | — | 低分框只在上一位置附近才认，避免反光点把目标拽走 |

### 降级策略调度（`hybrid_tracker.py`）

| 参数 | 值 | 说明 |
| :--- | :--- | :--- |
| `enhanced_retry_interval` | `2` | 增强图重检隔 2 帧跑一次（NMS 比较费时） |
| `global_template_interval` | `3` | 全画面模板搜索隔 3 帧跑一次 |
| `template_refresh_interval` | `5` | 模板每 5 帧刷新一次 |
| `template_threshold` | `0.48` | 局部模板匹配阈值 |
| `global_template_threshold` | `0.60` | 全画面模板阈值（比局部严，防误定位） |
| `reset_ms` | `900` | 失联超过 900ms 整体复位，不再外推 |

### 模板验证（`_verify_global_template`）

| 参数 | 值 | 说明 |
| :--- | :--- | :--- |
| `texture >= 51.0` | — | CLAHE 后内圈灰度标准差下限 |
| `dynamic_range >= 138.0` | — | 内圈 P90−P10 动态范围下限 |
| `tileGridSize=(8, 4)` | — | 整帧 CLAHE 网格 |
| `tileGridSize=(4, 4)` | — | 模板 CLAHE 网格。模板块小，网格再粗就糊了 |

> 这两个阈值（51 / 138）用来区分钢珠和水管端帽——**两者都是圆形暗块**，
> 只能靠相对纹理分辨。它们是在赛场光照下标定的，换环境要重测。

### 亮度归一化（`_normalize_rgb`）

| 参数 | 值 | 说明 |
| :--- | :--- | :--- |
| 触发区间 | `< 95` 或 `> 170` | 整帧均值落在此区间外才做 gamma 校正 |
| gamma 目标 | `125/255` | 拉到中间亮度 |
| gamma 钳位 | `[0.55, 1.75]` | 防止极端画面把 gamma 拉到离谱的值 |

区间内则改用 CLAHE 拉 LAB 的 L 通道（处理一边亮一边暗的情况）。

### 自适应滤波（`ball_position.py`）

| 参数 | 值 | 说明 |
| :--- | :--- | :--- |
| `alpha` | `0.20 → 0.95` | 静止强滤波、快动快跟随 |
| `beta` | `0.01 → 0.12` | 速度项增益 |
| `motion` | `min(1, err/10)²` | 用残差平方做权重，小误差更"迟钝" |
| `dt` 钳位 | `[5ms, 100ms]` | 防抖动或卡帧时速度估计爆炸 |
| `reset_ms` | `250` | 丢目标超过 250ms 重置，避免错误外推 |
| 速度平滑 | `0.65 / 0.35` | YOLO 框中心本身在跳，测量速度要滤 |

---

## 处理流水线

```
相机帧 ──► NPU YOLO26 ──► 候选筛选 ──► [丢失时] 多级降级跟踪
                                              │
                        ┌─────────────────────┘
                        ▼
        像素坐标 ──► 正交投影到标定轴 ──► 比例系数 → cm
                        │
                        ▼
              自适应 α-β 滤波平滑 ──► 串口 $BALL 帧 ──► STM32
```

### 1. NPU 推理

```python
detector = nn.YOLO26(model=..., dual_buff=not LOW_LATENCY_MODE)
```

`LOW_LATENCY_MODE = True` 对应 `dual_buff=False`：**检测结果对应当前输入帧**，
而不是流水线里更早的历史帧。这会牺牲一点吞吐，但直接降低位置反馈的延迟——
在闭环里，延迟比帧率更致命。

### 2. 多级容错跟踪（`HybridBallTracker`）

正常帧只跑 YOLO。一旦瞬时丢失，按代价从低到高依次降级：

| 级别 | 策略 | 触发频率 |
| :--- | :--- | :--- |
| 0 | YOLO 原始检测（低门限 0.18 取候选，再用时序位置与尺寸选最佳目标） | 每帧 |
| 1 | **小窗口模板匹配**（阈值 0.48），只在上次位置附近搜索 | 每 2 帧 |
| 2 | **CLAHE 自适应亮度增强后重跑 YOLO**（门限降到 0.15） | 每 2 帧 |
| 3 | **低频全画面归一化模板重定位**（阈值 0.60） | 每 3 帧 |

设计取舍：**刻意避开全图 Hough 圆搜索与 FFT**。这两种算法在 MaixCAM 的
CPU/NPU 结构上代价过高，而钢珠在赛场上的丢失原因主要是反光与亮度变化，
用 CLAHE + 模板跟踪的组合性价比远高于全图搜索。模板每 5 帧刷新一次，
超时 900ms 未重新锁定则整体复位。

候选筛选使用 `iou_threshold = 0.45` 抑制重叠框，并综合**时序位置**和**目标尺寸**
选择最佳目标——单靠置信度会在反光点上选错。

### 3. 像素 → 物理量（`ball_position.py`）

把 2D 检测点**正交投影**到标定轴（青线）上：

```
ratio  = (P - A) · (B - A) / |B - A|²      # 沿轴的比例系数
cm     = start_cm + ratio * (end_cm - start_cm)
dist   = |P - 投影点|                       # 到轴的垂直距离，作为质量指标
```

机器人场景的实用价值在于：**只要钢珠质心被正确检出，即使它偏离标定轴，
轴向位置依然准确**；垂直距离则作为一个额外的可信度指标上报。

标定方法（现场实测）：

| 常量 | 值 | 说明 |
| :--- | :--- | :--- |
| `AXIS_START_PX` | `(67, 80)` | 摆杆轴线左端在**模型输入图**中的像素坐标 |
| `AXIS_END_PX` | `(392, 81)` | 右端像素坐标 |
| `AXIS_START_CM` | `2.5` | 左端对应实物位置（cm） |
| `AXIS_END_CM` | `15.5` | 右端对应实物位置（cm） |

> `main.py` 中 `AXIS_*_PX` 的 y 坐标取 80/81，与 `app.yaml` 里配置的
> **640×160** 模型输入尺寸对应。若更换模型分辨率，这四个常量必须重新标定。

### 4. 自适应 α-β 滤波

按预测误差动态调节增益，兼顾静止精度与动态跟随：

```python
motion = min(1.0, error / 10.0) ** 2
alpha  = 0.20 + (0.95 - 0.20) * motion   # 静止强滤波，快速运动快速跟随
beta   = 0.01 + (0.12 - 0.01) * motion
```

- `dt` 钳位在 `[5ms, 100ms]`，防止抖动或卡帧时速度估计爆炸
- `mark_missing()`：超过 `reset_ms = 250ms` 无观测则**整体重置**，避免错误外推

### 5. 触摸屏在线标定

`TOUCH_ENABLE` 打开后，可直接在屏幕上点击摆杆上的物理位置，
`screen_to_image()` 把屏幕坐标换算回图像坐标，再算出该点对应的 cm 并随帧下发。

这省掉了"改代码 → 重新打包 → 烧写"的迭代成本。`TOUCH_SEND_EDGE_ONLY` 控制是
按下只发一次（`True`）还是按住拖动持续发（`False`）。

### 6. 其他图像处理

`LENS_CORR_ENABLE` + `LENS_CORR_STRENGTH = 0.6`：镜头畸变校正。广角镜头下
摆杆在画面边缘的直线会被弯成弧线，不校正会让"沿轴投影"的假设失效。

---

## 串口协议

每识别成功一帧发送一行：

```
$BALL,<valid>,<x_cm>,<vx_pixel_s>,<confidence>,<frame_time_ms>[,<touch_cm>]*\n
```

| 字段 | 含义 |
| :--- | :--- |
| `valid` | `1` = 本帧坐标有效，`0` = 无效（此时后续数值填 0） |
| `x_cm` | 沿摆杆轴线的位置，单位 cm |
| `vx_pixel_s` | 估算的横向像素速度，像素/秒 |
| `confidence` | YOLO 检测置信度 |
| `frame_time_ms` | **视觉模块自己的**时间戳。只能用于判断视觉端是否卡帧，**不可**与 STM32 的 `HAL_GetTick()` 做差 |
| `touch_cm` | 可选。触摸屏指定的目标位置，单位 cm |

主控侧解析实现在
[`../01_firmware_stm32/applications/vision.c`](../01_firmware_stm32/applications/vision.c)，
超时判活 `VISION_TIMEOUT_MS = 200`。

---

## 关于模型权重

`models/` 下仅保留体积很小、人类可读的 [`.mud`](models/yolo26_all_maixcam2_yolo26_640_160/yolo26_all.mud)
描述文件（记录输入尺寸、预处理 mean/scale、类别标签）。**权重文件 `*.axmodel`
（NPU 版 2.6MB / VNPU 版 2.7MB）未入库**，原因是体积过大。

```ini
[basic]
type = axmodel
model_npu  = yolo26_all_npu.axmodel
model_vnpu = yolo26_all_vnpu.axmodel

[extra]
model_type = yolo26
type       = detector
input_type = rgb
labels     = ball
```

**自己复现模型**的路径：

1. 用 [`../04_dataset_tools/`](../04_dataset_tools/) 从原始视频抽帧（或自行采集）
2. 用 X-AnyLabeling / Labelme 做单类 `ball` 多边形标注
3. 训练 YOLO26，再转换为 AX 系列 NPU 支持的 `.axmodel` 格式
4. 连同 `.mud` 一起放进 `models/<name>/`，并同步更新 `main.py` 中的 `*_MODEL_PATH`

（`main.py` 里预留了 MaixCAM 与 MaixCAM2 两条模型路径，按 `sys.device_name()`
自动选择。MaixCAM 分支指向 `..._480_160/`，该权重同样未入库。）

---

## 运行

把 `main.py`、`hybrid_tracker.py`、`ball_position.py`、`app.yaml` 与 `models/`
一起按 MaixPy 应用打包后推送到设备运行。`app.yaml` 里的 `files` 列表需要与
实际文件一一对应。

调试开关都在 `main.py` 顶部：

```python
DEBUG_LOG = False     # 串口打印调试信息
DRAW_DEBUG = False    # 屏幕叠加检测框与轴线
SHOW_FPS = False      # 显示帧率
USE_WEBRTC = True     # 网页推流预览
PRINT_PROTOCOL = False# 打印串口协议内容
```
