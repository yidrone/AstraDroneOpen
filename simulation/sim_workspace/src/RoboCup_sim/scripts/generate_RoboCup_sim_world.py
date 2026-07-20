#!/usr/bin/env python3

import argparse
import math
import random
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, Iterable, List, NamedTuple, Optional, Tuple


SIM_ROOT = Path(__file__).resolve().parents[4]  # simulation/
WORLDS_ROOT = SIM_ROOT / "astra_gazebo_worlds"

STATIC_WORLD_PATH = WORLDS_ROOT / "RoboCup_sim" / "RoboCup_sim.world"
RUNTIME_WORLD_PATH = WORLDS_ROOT / "generated" / "RoboCup_sim_runtime.world"

OBSTACLE_PAIRS = [
    ("box_1", "tree_1"),
    ("box_2", "tree_2"),
    ("box_3", "tree_3"),
    ("box_4", "tree_4"),
]

STANDARD_TARGETS = [
    "target_tent",
    "target_bunker",
    "target_bridge",
    "target_tank",
    "target_armored_vehicle",
]
RED_CROSS_TARGET = "target_red_cross"

# A area only. Keep clear of the B corridor at the north side and the fixed takeoff pad.
A_BOUNDS = (-4.2, 4.2, -3.0, 2.7)  # min_x, max_x, min_y, max_y
TAKEOFF_CENTER = (0.0, -4.0)

BOX_Z = 0.4
TREE_ON_BOX_Z = 0.8
STANDARD_TARGET_Z = 0.01
RED_CROSS_TARGET_Z = 0.01

OBSTACLE_GROUP_RADIUS = 0.9
STANDARD_TARGET_RADIUS = 0.65
RED_CROSS_TARGET_RADIUS = 0.3
TAKEOFF_KEEP_OUT_RADIUS = 1.45

MIN_OBSTACLE_DISTANCE = 1.9
MIN_TARGET_DISTANCE = 1.45
MIN_OBSTACLE_TARGET_DISTANCE = 1.6


class Placement(NamedTuple):
    name: str
    x: float
    y: float
    radius: float
    kind: str


def fmt(value: float) -> str:
    text = f"{value:.6f}".rstrip("0").rstrip(".")
    return text if text and text != "-0" else "0"


def parse_pose(text: Optional[str]) -> List[float]:
    values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    if text:
        parts = text.split()
        for idx, part in enumerate(parts[:6]):
            values[idx] = float(part)
    return values


def pose_text(x: float, y: float, z: float, r: float = 0.0, p: float = 0.0, yaw: float = 0.0) -> str:
    return " ".join(fmt(v) for v in (x, y, z, r, p, yaw))


def indent(elem: ET.Element, level: int = 0) -> None:
    pad = "\n" + level * "  "
    child_pad = "\n" + (level + 1) * "  "
    children = list(elem)
    if children:
        if not elem.text or not elem.text.strip():
            elem.text = child_pad
        for child in children:
            indent(child, level + 1)
        if not children[-1].tail or not children[-1].tail.strip():
            children[-1].tail = pad
    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = pad


def get_world(root: ET.Element) -> ET.Element:
    world = root.find("world")
    if world is None:
        raise RuntimeError("No <world> element found")
    return world


def find_model(world: ET.Element, name: str) -> ET.Element:
    for model in world.findall("model"):
        if model.attrib.get("name") == name:
            return model
    raise RuntimeError(f"Model not found: {name}")


def set_model_pose(world: ET.Element, name: str, x: float, y: float, z: float,
                   yaw: Optional[float] = None) -> None:
    model = find_model(world, name)
    pose = model.find("pose")
    if pose is None:
        pose = ET.SubElement(model, "pose")
    current = parse_pose(pose.text)
    pose.text = pose_text(x, y, z, current[3], current[4], current[5] if yaw is None else yaw)


def remove_saved_state(world: ET.Element) -> None:
    for state in list(world.findall("state")):
        world.remove(state)


def distance(a: Placement, b: Placement) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def required_clearance(a: Placement, b: Placement) -> float:
    if a.kind == "takeoff" or b.kind == "takeoff":
        return max(a.radius, b.radius)
    if a.kind == "obstacle" and b.kind == "obstacle":
        return MIN_OBSTACLE_DISTANCE
    if a.kind == "target" and b.kind == "target":
        return MIN_TARGET_DISTANCE
    return MIN_OBSTACLE_TARGET_DISTANCE


def is_valid(candidate: Placement, placed: Iterable[Placement]) -> bool:
    for existing in placed:
        if distance(candidate, existing) < required_clearance(candidate, existing):
            return False
    return True


def sample_position(rng: random.Random, name: str, radius: float, kind: str,
                    placed: List[Placement]) -> Placement:
    min_x, max_x, min_y, max_y = A_BOUNDS
    for _ in range(10000):
        candidate = Placement(
            name=name,
            x=rng.uniform(min_x + radius, max_x - radius),
            y=rng.uniform(min_y + radius, max_y - radius),
            radius=radius,
            kind=kind,
        )
        if is_valid(candidate, placed):
            return candidate
    raise RuntimeError(f"Unable to sample a valid A-area position for {name}")


def randomize_world(world: ET.Element, rng: random.Random) -> Dict[str, Placement]:
    placed: List[Placement] = [
        Placement("takeoff_zone", TAKEOFF_CENTER[0], TAKEOFF_CENTER[1], TAKEOFF_KEEP_OUT_RADIUS, "takeoff")
    ]
    placements: Dict[str, Placement] = {}

    for box_name, tree_name in OBSTACLE_PAIRS:
        p = sample_position(rng, box_name, OBSTACLE_GROUP_RADIUS, "obstacle", placed)
        placed.append(p)
        placements[box_name] = p
        placements[tree_name] = p
        set_model_pose(world, box_name, p.x, p.y, BOX_Z)
        set_model_pose(world, tree_name, p.x, p.y, TREE_ON_BOX_Z)

    for target_name in STANDARD_TARGETS:
        try:
            find_model(world, target_name)
        except RuntimeError:
            continue
        p = sample_position(rng, target_name, STANDARD_TARGET_RADIUS, "target", placed)
        placed.append(p)
        placements[target_name] = p
        set_model_pose(world, target_name, p.x, p.y, STANDARD_TARGET_Z)

    try:
        find_model(world, RED_CROSS_TARGET)
    except RuntimeError:
        return placements

    p = sample_position(rng, RED_CROSS_TARGET, RED_CROSS_TARGET_RADIUS, "target", placed)
    placements[RED_CROSS_TARGET] = p
    set_model_pose(world, RED_CROSS_TARGET, p.x, p.y, RED_CROSS_TARGET_Z, yaw=0.0)
    return placements


def write_world(root: ET.Element, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    indent(root)
    tree = ET.ElementTree(root)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    output.write_text(output.read_text(encoding="utf-8") + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate a randomized RoboCup_sim runtime world.")
    parser.add_argument("--template", type=Path, default=STATIC_WORLD_PATH)
    parser.add_argument("--output", type=Path, default=RUNTIME_WORLD_PATH)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--print-placements", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rng = random.Random(args.seed)

    tree = ET.parse(args.template)
    root = tree.getroot()
    world = get_world(root)
    remove_saved_state(world)
    placements = randomize_world(world, rng)
    write_world(root, args.output)

    print(f"generated: {args.output}", file=sys.stderr)
    if args.seed is not None:
        print(f"seed: {args.seed}", file=sys.stderr)
    if args.print_placements:
        for name in sorted(placements):
            p = placements[name]
            print(f"{name}: {fmt(p.x)} {fmt(p.y)}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
