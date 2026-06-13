#!/usr/bin/env python3

import argparse
import json
import random
import re
import shutil
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import qrcode
from PIL import Image
from qrcode.constants import ERROR_CORRECT_M


SCRIPT_DIR = Path(__file__).resolve().parent
TEXTURES_DIR = SCRIPT_DIR.parent / "textures"
GENERATED_TARGETS_DIR = TEXTURES_DIR / "generated_targets"
QR_OUTPUT_PATH = TEXTURES_DIR / "qr_placeholder.png"
MANIFEST_PATH = TEXTURES_DIR / "current_field_assets.json"

TARGET_SLOTS = [
    ("top_left", TEXTURES_DIR / "target_top_left.png"),
    ("top_right", TEXTURES_DIR / "target_top_right.png"),
    ("bottom_left", TEXTURES_DIR / "target_bottom_left.png"),
    ("bottom_right", TEXTURES_DIR / "target_bottom_right.png"),
]

TARGET_NAME_RE = re.compile(r"^target_(.+)_(\d+)\.png$")


def parse_target_name(path: Path) -> Tuple[str, int]:
    match = TARGET_NAME_RE.match(path.name)
    if not match:
        raise ValueError(f"Unexpected target file name: {path.name}")
    return match.group(1), int(match.group(2))


def load_targets(generated_dir: Path) -> Dict[str, List[Path]]:
    grouped: Dict[str, List[Path]] = {}
    for path in sorted(generated_dir.glob("target_*.png")):
        class_name, _ = parse_target_name(path)
        grouped.setdefault(class_name, []).append(path)
    if len(grouped) < 4:
        raise FileNotFoundError(
            f"Need at least 4 target classes under {generated_dir}, found {len(grouped)}."
        )
    return grouped


def choose_targets(grouped: Dict[str, List[Path]], rng: random.Random) -> List[Tuple[str, Path]]:
    class_names = rng.sample(sorted(grouped.keys()), 4)
    chosen: List[Tuple[str, Path]] = []
    for class_name in class_names:
        chosen.append((class_name, rng.choice(grouped[class_name])))
    return chosen


def copy_targets(chosen: List[Tuple[str, Path]]) -> List[Dict[str, str]]:
    placements: List[Dict[str, str]] = []
    for (slot_name, output_path), (class_name, source_path) in zip(TARGET_SLOTS, chosen):
        shutil.copy2(source_path, output_path)
        placements.append(
            {
                "slot": slot_name,
                "class_name": class_name,
                "source_file": source_path.name,
                "output_file": output_path.name,
            }
        )
    return placements


def generate_qr_payload(class_names: List[str], rng: random.Random, landing: Optional[str]) -> str:
    chosen_classes = rng.sample(class_names, 2)
    chosen_landing = landing if landing is not None else rng.choice(["left", "right"])
    return f"{chosen_classes[0]},{chosen_classes[1]},{chosen_landing}"


def generate_qr_image(payload: str, output_path: Path, size_px: int = 1024) -> None:
    qr = qrcode.QRCode(
        version=None,
        error_correction=ERROR_CORRECT_M,
        box_size=16,
        border=4,
    )
    qr.add_data(payload)
    qr.make(fit=True)
    qr_img = qr.make_image(fill_color="black", back_color="white").convert("RGB")
    qr_img = qr_img.resize((size_px, size_px), Image.NEAREST)
    qr_img.save(output_path)


def write_manifest(manifest_path: Path, payload: str, placements: List[Dict[str, str]]) -> None:
    landing = payload.split(",")[-1]
    data = {
        "qr_payload": payload,
        "landing": landing,
        "placements": placements,
    }
    manifest_path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Randomize CRAIC target textures and QR code.")
    parser.add_argument(
        "--generated-targets",
        type=Path,
        default=GENERATED_TARGETS_DIR,
        help=f"Directory containing generated target PNGs. Default: {GENERATED_TARGETS_DIR}",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Random seed for reproducible target and QR selection.",
    )
    parser.add_argument(
        "--landing",
        choices=["left", "right"],
        default=None,
        help="Force the QR landing direction instead of sampling it randomly.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rng = random.Random(args.seed)
    grouped = load_targets(args.generated_targets.resolve())
    chosen = choose_targets(grouped, rng)
    placements = copy_targets(chosen)
    class_names = [class_name for class_name, _ in chosen]
    payload = generate_qr_payload(class_names, rng, args.landing)
    generate_qr_image(payload, QR_OUTPUT_PATH)
    write_manifest(MANIFEST_PATH, payload, placements)

    print("Selected targets:")
    for placement in placements:
        print(
            f"  {placement['slot']}: {placement['class_name']} "
            f"({placement['source_file']} -> {placement['output_file']})"
        )
    print(f"QR payload: {payload}")
    print(f"Updated QR texture: {QR_OUTPUT_PATH}")
    print(f"Wrote manifest: {MANIFEST_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
