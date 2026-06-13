#!/usr/bin/env python3

import argparse
import math
import random
from pathlib import Path
from typing import Dict, List, Optional

from PIL import Image, ImageDraw


IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".bmp", ".webp"}
SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_INPUT_DIR = SCRIPT_DIR / "source_images"
DEFAULT_OUTPUT_DIR = SCRIPT_DIR.parent / "textures" / "generated_targets"


def build_target(
    source_img: Image.Image,
    canvas_size_px: int = 2048,
    outer_diam_ratio: float = 0.75,
    inner_diam_ratio: float = 0.5,
) -> Image.Image:
    size_px = canvas_size_px
    outer_px = int(canvas_size_px * outer_diam_ratio)
    inner_px = int(canvas_size_px * inner_diam_ratio)

    canvas = Image.new("RGB", (size_px, size_px), (0, 0, 0))
    draw = ImageDraw.Draw(canvas)
    cx, cy = size_px // 2, size_px // 2

    draw.ellipse(
        [(cx - outer_px // 2, cy - outer_px // 2), (cx + outer_px // 2, cy + outer_px // 2)],
        fill=(128, 128, 128),
    )
    draw.ellipse(
        [(cx - inner_px // 2, cy - inner_px // 2), (cx + inner_px // 2, cy + inner_px // 2)],
        fill=(255, 255, 255),
    )

    inscribed_side = int(inner_px / math.sqrt(2))
    source_img = source_img.convert("RGB").resize((inscribed_side, inscribed_side), Image.BILINEAR)

    mask = Image.new("L", (inner_px, inner_px), 0)
    mask_draw = ImageDraw.Draw(mask)
    mask_draw.ellipse([(0, 0), (inner_px, inner_px)], fill=255)

    layer = Image.new("RGB", (inner_px, inner_px), (255, 255, 255))
    offset = ((inner_px - inscribed_side) // 2, (inner_px - inscribed_side) // 2)
    layer.paste(source_img, offset)

    canvas.paste(layer, (cx - inner_px // 2, cy - inner_px // 2), mask)
    return canvas


def save_target(source_path: Path, class_name: str, index: int, output_dir: Path, canvas_size_px: int) -> Path:
    image = Image.open(source_path)
    target = build_target(image, canvas_size_px=canvas_size_px)
    output_path = output_dir / f"target_{class_name}_{index:02d}.png"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    target.save(output_path)
    return output_path


def collect_images(input_dir: Path) -> Dict[str, List[Path]]:
    if not input_dir.exists():
        raise FileNotFoundError(f"Input directory not found: {input_dir}")

    grouped: Dict[str, List[Path]] = {}
    for class_dir in sorted(path for path in input_dir.iterdir() if path.is_dir()):
        images = sorted(
            path for path in class_dir.iterdir()
            if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
        )
        if images:
            grouped[class_dir.name] = images

    if grouped:
        return grouped

    raise FileNotFoundError(f"No images found in {input_dir}")


def batch_generate(
    input_dir: Path,
    output_dir: Path,
    num_per_class: int,
    seed: Optional[int],
    canvas_size_px: int,
) -> None:
    rng = random.Random(seed)
    grouped = collect_images(input_dir)

    for class_name, images in grouped.items():
        chosen = images if num_per_class <= 0 or num_per_class >= len(images) else rng.sample(images, num_per_class)
        for index, image_path in enumerate(chosen, start=1):
            output_path = save_target(image_path, class_name, index, output_dir, canvas_size_px)
            print(f"Saved: {output_path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate CRAIC target images from local source images.")
    parser.add_argument("--input-dir", type=Path, default=DEFAULT_INPUT_DIR)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--num-per-class", type=int, default=2)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--canvas-size", type=int, default=2048, help="Output square texture size in pixels.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    batch_generate(
        input_dir=args.input_dir.resolve(),
        output_dir=args.output_dir.resolve(),
        num_per_class=args.num_per_class,
        seed=args.seed,
        canvas_size_px=args.canvas_size,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
