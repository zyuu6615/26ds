#!/usr/bin/env python3

import argparse
import os
import sys
from pathlib import Path

import cv2


# ====================== 逐帧抽取 ======================


def video_to_frames(
    video_path: str,
    output_dir: str | None = None,
    interval: int = 2,
    img_format: str = "jpg",
    target_width: int | None = None,
    target_height: int | None = None,
) -> None:
    # ---------- 参数校验 ----------
    if not os.path.isfile(video_path):
        raise FileNotFoundError(f"视频文件不存在: {video_path}")

    if interval < 1:
        raise ValueError(f"interval 必须 >= 1，当前为 {interval}")

    img_format = img_format.lower().lstrip(".")
    if img_format not in ("jpg", "jpeg", "png"):
        raise ValueError(f"不支持的图片格式: {img_format}，请使用 jpg 或 png")

    # ---------- 输出文件夹 ----------
    if output_dir is None:
        video_name = Path(video_path).stem
        output_dir = os.path.join(os.path.dirname(video_path) or ".", f"{video_name}_frames")

    os.makedirs(output_dir, exist_ok=True)
    print(f"输出文件夹: {output_dir}")

    # ---------- 打开视频 ----------
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise IOError(f"无法打开视频文件: {video_path}")

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"视频信息: 总帧数={total_frames}, FPS={fps:.2f}")
    print(f"保存间隔: 每 {interval} 帧保存一张")
    if target_width is not None and target_height is not None:
        print(f"目标分辨率: {target_width}×{target_height}")
    else:
        print("保持原始分辨率")
    print(f"图片格式: {img_format}")
    print("-" * 50)

    # ---------- 扩展名映射 ----------
    ext_map = {"jpg": ".jpg", "jpeg": ".jpg", "png": ".png"}
    ext = ext_map[img_format]

    # ---------- 逐帧处理 ----------
    saved_count = 0
    frame_idx = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if frame_idx % interval == 0:
            if target_width is not None and target_height is not None:
                if frame.shape[1] != target_width or frame.shape[0] != target_height:
                    frame = cv2.resize(frame, (target_width, target_height),
                                       interpolation=cv2.INTER_AREA)

            seq = saved_count + 1
            filename = f"frame_{seq:06d}{ext}"
            filepath = os.path.join(output_dir, filename)

            if img_format in ("jpg", "jpeg"):
                cv2.imwrite(filepath, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
            else:
                cv2.imwrite(filepath, frame, [cv2.IMWRITE_PNG_COMPRESSION, 3])

            saved_count += 1

            progress = (frame_idx + 1) / total_frames * 100 if total_frames > 0 else 0
            print(f"\r进度: {progress:5.1f}% 已保存 {saved_count} 张图片", end="", flush=True)

        frame_idx += 1

    # ---------- 收尾 ----------
    cap.release()
    print(f"\n{'=' * 50}")
    print(f"完成！共处理 {frame_idx} 帧，保存 {saved_count} 张图片到: {output_dir}")


# ====================== 命令行入口 ======================


def main() -> None:
    parser = argparse.ArgumentParser(
        description="将视频逐帧截取为图片，保持原始分辨率",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  python video_to_frames.py video.mp4
  python video_to_frames.py video.mp4 -o ./output -i 5 -f png
  python video_to_frames.py video.mp4 --size 640 480
        """,
    )
    parser.add_argument("video", help="视频文件路径")
    parser.add_argument("-o", "--output", default=None, help="输出文件夹路径（默认自动创建）")
    parser.add_argument("-i", "--interval", type=int, default=1, help="每隔多少帧保存一张（默认 1）")
    parser.add_argument("-f", "--format", default="jpg", choices=["jpg", "jpeg", "png"],
                        help="图片格式（默认 jpg）")
    parser.add_argument("--size", nargs=2, type=int, default=None, metavar=("W", "H"),
                        help="输出分辨率（可选，不指定则保持原始分辨率）")

    args = parser.parse_args()

    try:
        video_to_frames(
            video_path=args.video,
            output_dir=args.output,
            interval=args.interval,
            img_format=args.format,
            target_width=args.size[0] if args.size else None,
            target_height=args.size[1] if args.size else None,
        )
    except Exception as e:
        print(f"\n错误: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
