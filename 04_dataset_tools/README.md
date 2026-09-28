# 04 · 训练数据工具

为 [`../02_vision_maixcam2/`](../02_vision_maixcam2/) 的 YOLO26 钢珠检测模型准备训练数据。

**本仓库只含脚本，不含图片数据** —— 标注帧与视频约 90MB，请用下面的脚本自行生成。

## 文件

| 文件 | 作用 |
| :--- | :--- |
| [`video_to_frames.py`](video_to_frames.py) | 把视频按固定间隔抽帧保存为图片（OpenCV） |

## 用法

```bash
# 每帧都保存
python video_to_frames.py raw/2.mp4

# 指定输出目录、每 5 帧存一张、png 格式
python video_to_frames.py raw/2.mp4 -o ./2_frames -i 5 -f png

# 缩放到 640x480
python video_to_frames.py raw/2.mp4 --size 640 480
```

| 参数 | 默认 | 说明 |
| :--- | :--- | :--- |
| `video` | — | 输入视频路径 |
| `-o, --output` | 视频同目录 `<名字>_frames/` | 输出目录 |
| `-i, --interval` | `1` | 每多少帧保存一张 |
| `-f, --format` | `jpg` | `jpg` / `jpeg` / `png` |
| `--size W H` | 保持原分辨率 | 输出分辨率，宽高须同时给出 |

> ⚠ 输出的 `frame_%06d` 是**保存序号**，不是原视频帧号。`-i > 1` 时两者不同，
> 需要回溯具体时刻请自行乘上 `interval`。

## 标注格式

每张图配一个同名 `.json`，为 X-AnyLabeling / Labelme 兼容格式：

```json
{
  "version": "4.0.0-beta.13",
  "flags": {},
  "shapes": [],
  "imagePath": "frame_000400.jpg",
  "imageHeight": 480,
  "imageWidth": 640,
  "imageData": null,
  "description": ""
}
```

有目标时 `shapes` 里是 `{"label": "ball", ...}`；**空 `shapes` 是刻意保留的负样本**，
用于压制反光点等圆形干扰物的误检，不要删除。

## 完整流程

1. **采集**：实地拍摄钢珠在摆杆上滚动的视频，覆盖不同光照、背景、钢珠位置与运动模糊。
2. **抽帧**：`video_to_frames.py`，间隔取 `2` 左右通常够用。
3. **标注**：X-AnyLabeling 单类 `ball`，**务必包含足够的负样本帧**。
4. **训练**：YOLO26 单类训练，可降低输入分辨率换取 NPU 帧率。
5. **转换**：导出 AX 系列 NPU 的 `.axmodel`。
6. **部署**：放进 `02_vision_maixcam2/models/<name>/`，更新 `main.py` 的 `MODEL_PATH`，
   **并重新标定 `AXIS_START_PX` / `AXIS_END_PX` / `AXIS_START_CM` / `AXIS_END_CM`**。
