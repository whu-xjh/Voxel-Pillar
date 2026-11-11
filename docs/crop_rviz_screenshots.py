#!/usr/bin/env python3
"""裁剪 RViz 截图：只保留中间的黑色背景 + 着色点云视口，去掉四周工具栏/状态栏/任务栏等边框。

原理
----
1. UI 区域（标题栏、菜单栏、工具栏、Time 面板、状态栏、系统任务栏）由
   「中性偏亮」的像素构成（R、G、B 都高于阈值且接近灰色）；
2. 黑色视口内即使有很亮的点云（黄/绿/橙），也不是中性色（某个通道接近 0），
   因此不会被误判为 UI；
3. 按行统计亮色占比，找出最长的一段「非 UI」行条带，即视口的行范围；
   再在该行范围内按列做同样处理，得到列范围（可兼容左侧 Displays 面板）；
4. 对该包围盒加 padding 后裁剪保存。

用法
----
# 批量处理目录，输出到 <目录>/cropped/
python3 docs/crop_rviz_screenshots.py /mnt/g/JunhaoXu/voxel-pillar/output

# 指定输出目录，四周留 10px 黑边
python3 docs/crop_rviz_screenshots.py /mnt/g/JunhaoXu/voxel-pillar/output -o /mnt/g/JunhaoXu/voxel-pillar/output/cropped --padding 10

# 单张图片
python3 docs/crop_rviz_screenshots.py path/to/cave01.png -o out/

依赖：numpy、Pillow（pip install numpy pillow）
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable, Tuple

import numpy as np
from PIL import Image

IMAGE_EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}


def _longest_run(keep: np.ndarray) -> Tuple[int, int]:
    """返回 bool 数组中最长连续 True 段的 [start, end) 区间；全 False 时返回 (0, len)。"""
    padded = np.concatenate(([False], keep, [False]))
    diff = np.diff(padded.astype(np.int8))
    starts = np.flatnonzero(diff == 1)
    ends = np.flatnonzero(diff == -1)
    if starts.size == 0:
        return 0, len(keep)
    lengths = ends - starts
    best = int(np.argmax(lengths))
    return int(starts[best]), int(ends[best])


def find_viewport_bbox(
    img: np.ndarray,
    light_thresh: int = 180,
    band_frac: float = 0.5,
    padding: int = 0,
) -> Tuple[int, int, int, int]:
    """在 RGB 图像数组中定位黑色视口的包围盒。

    参数
    ----
    img          : HxWx3 的 RGB 数组（uint8）。
    light_thresh : 判定「中性亮色 UI 像素」的通道下限（R、G、B 需同时大于该值）。
    band_frac    : 一行/列中 UI 像素占比超过该值，则该行/列视为 UI 边框。
    padding      : 裁剪框向四周扩张的像素数（黑色留边）。

    返回
    ----
    (left, top, right, bottom)，右/下为开区间，可直接用于 numpy 切片。
    """
    if img.ndim == 2:  # 灰度图没有通道差异，直接按亮度判 UI
        light = img > light_thresh
    else:
        light = np.min(img[..., :3], axis=-1) > light_thresh

    h, w = light.shape

    # 第一遍：按行找最长的非 UI 条带（视口的行范围；同时排除上方工具栏与下方状态栏/任务栏）
    row_ui = light.mean(axis=1) > band_frac
    top, bottom = _longest_run(~row_ui)

    # 第二遍：在视口行范围内按列找最长的非 UI 条带（兼容左侧 Displays 面板等）
    col_ui = light[top:bottom].mean(axis=0) > band_frac
    left, right = _longest_run(~col_ui)

    # 加 padding 并裁到图像范围内
    left = max(0, left - padding)
    top = max(0, top - padding)
    right = min(w, right + padding)
    bottom = min(h, bottom + padding)
    return left, top, right, bottom


def crop_viewport(img: np.ndarray, **kwargs) -> np.ndarray:
    """裁剪单张图像数组，只保留黑色视口部分。kwargs 同 find_viewport_bbox。"""
    left, top, right, bottom = find_viewport_bbox(img, **kwargs)
    return img[top:bottom, left:right]


def crop_file(src: Path, dst: Path, padding: int = 0, overwrite: bool = False) -> Tuple[int, int, Tuple[int, int, int, int]]:
    """裁剪单张图片文件并保存到 dst，返回 (原图尺寸, 裁后尺寸, bbox)。"""
    if dst.exists() and not overwrite:
        raise FileExistsError(f"输出文件已存在（用 --overwrite 覆盖）: {dst}")
    img = np.asarray(Image.open(src).convert("RGB"))
    left, top, right, bottom = find_viewport_bbox(img, padding=padding)
    cropped = img[top:bottom, left:right]
    dst.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(cropped).save(dst)
    return img.shape[:2], cropped.shape[:2], (left, top, right, bottom)


def _iter_images(inputs: Iterable[Path]) -> Iterable[Path]:
    for p in inputs:
        if p.is_dir():
            yield from sorted(q for q in p.iterdir() if q.suffix.lower() in IMAGE_EXTS)
        else:
            yield p


def main() -> None:
    parser = argparse.ArgumentParser(description="批量裁剪 RViz 截图的黑色点云视口，去除四周 UI 边框。")
    parser.add_argument("inputs", nargs="+", type=Path, help="图片文件或包含图片的目录")
    parser.add_argument("-o", "--outdir", type=Path, default=None,
                        help="输出目录（默认：<输入目录>/cropped/）")
    parser.add_argument("--padding", type=int, default=0, help="裁剪框四周保留的黑色边距像素（默认 0）")
    parser.add_argument("--suffix", type=str, default="", help='输出文件名后缀（默认与原文件同名）')
    parser.add_argument("--overwrite", action="store_true", help="覆盖已存在的输出文件")
    args = parser.parse_args()

    files = [p for p in _iter_images(args.inputs) if p.suffix.lower() in IMAGE_EXTS and p.exists()]
    if not files:
        parser.error("没有找到可处理的图片")

    # 默认输出到第一个输入的目录下的 cropped/
    outdir = args.outdir
    if outdir is None:
        base = files[0].parent if files[0].parent.name else Path(".")
        outdir = base / "cropped"

    for src in files:
        dst = outdir / f"{src.stem}{args.suffix}{src.suffix}"
        try:
            (h, w), (ch, cw), bbox = crop_file(src, dst, padding=args.padding, overwrite=args.overwrite)
            print(f"{src.name}: {w}x{h} -> {cw}x{ch}  bbox(l,t,r,b)={bbox}  => {dst}")
        except FileExistsError as e:
            print(f"跳过 {src.name}: {e}")


if __name__ == "__main__":
    main()
