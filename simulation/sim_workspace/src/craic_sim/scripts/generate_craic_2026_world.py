#!/usr/bin/env python3

import argparse
import math
import random
from pathlib import Path
from typing import Tuple


SIM_ROOT = Path(__file__).resolve().parents[4]  # simulation/
MODELS_ROOT = SIM_ROOT / "astra_gazebo_models"
WORLDS_ROOT = SIM_ROOT / "astra_gazebo_worlds"

ARENA_MODEL_DIR = MODELS_ROOT / "craic_2026_arena"
RING_MODEL_DIR = MODELS_ROOT / "craic_2026_ring"
RING_MESH_DIR = RING_MODEL_DIR / "meshes"
RING_MESH_PATH = RING_MESH_DIR / "ring.obj"
STATIC_WORLD_PATH = WORLDS_ROOT / "craic_2026.world"
RUNTIME_WORLD_PATH = WORLDS_ROOT / "generated" / "craic_2026_runtime.world"

ARENA_WIDTH = 9.0
ARENA_HEIGHT = 6.0
ARENA_WALL_HEIGHT = 3.0
TARGET_SIZE = 0.8
QR_SIZE = 0.2
OBSTACLE_SIZE = 0.6
OBSTACLE_SEGMENT_COUNT = 4
TAKEOFF_RULE_X = 1.5
TAKEOFF_RULE_Y = 3.0

RING_RULE_MIN_X = 6.3
RING_RULE_MAX_X = 8.7
RING_DEFAULT_RULE_X = 7.5
RING_CENTER_Y = 4.6
RING_CENTER_Z = 1.6
RING_OUTER_DIAMETER = 1.2
RING_INNER_DIAMETER = 0.9
RING_VISUAL_DEPTH = 0.045
RING_SUPPORT_X = 0.42
RING_SUPPORT_RADIUS = 0.018
RING_SUPPORT_HEIGHT = 1.0


def fmt(value: float) -> str:
    return f"{value:.6f}".rstrip("0").rstrip(".")


def rule_to_world_xy(rule_x: float, rule_y: float) -> Tuple[float, float]:
    return rule_x - TAKEOFF_RULE_X, TAKEOFF_RULE_Y - rule_y


def material_script_block(material_name: str) -> str:
    return f"""
          <material>
            <script>
              <uri>model://craic_2026_arena/materials/scripts</uri>
              <uri>model://craic_2026_arena/materials/textures</uri>
              <name>{material_name}</name>
            </script>
          </material>"""


def colored_material_block(ambient: str, diffuse: str, specular: str = "0.1 0.1 0.1 1", emissive: str = "0 0 0 1") -> str:
    return f"""
          <material>
            <ambient>{ambient}</ambient>
            <diffuse>{diffuse}</diffuse>
            <specular>{specular}</specular>
            <emissive>{emissive}</emissive>
          </material>"""


def make_box_model(name: str, x: float, y: float, z: float, sx: float, sy: float, sz: float,
                   ambient: str, diffuse: str, transparency: float = 0.0) -> str:
    transparency_tag = f"\n          <transparency>{fmt(transparency)}</transparency>" if transparency > 0 else ""
    return f"""
    <model name='{name}'>
      <static>1</static>
      <pose>{fmt(x)} {fmt(y)} {fmt(z)} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <box>
              <size>{fmt(sx)} {fmt(sy)} {fmt(sz)}</size>
            </box>
          </geometry>
          <surface>
            <contact><ode/></contact>
            <bounce/>
            <friction>
              <torsional><ode/></torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>{fmt(sx)} {fmt(sy)} {fmt(sz)}</size>
            </box>
          </geometry>{colored_material_block(ambient, diffuse)}{transparency_tag}
        </visual>
      </link>
    </model>"""


def make_obstacle_model(x: float, y: float) -> str:
    segment_colors = [
        ("0.98 0.8 0.2 1", "0.98 0.8 0.2 1"),
        ("0.22 0.22 0.24 1", "0.22 0.22 0.24 1"),
        ("0.88 0.27 0.24 1", "0.88 0.27 0.24 1"),
        ("0.35 0.4 0.95 1", "0.35 0.4 0.95 1"),
    ]
    collision_blocks = []
    visuals = []
    for idx, (ambient, diffuse) in enumerate(segment_colors):
        segment_z = OBSTACLE_SIZE * (idx + 0.5)
        collision_blocks.append(
            f"""
        <collision name='collision_{idx}'>
          <pose>0 0 {fmt(segment_z)} 0 0 0</pose>
          <geometry>
            <box>
              <size>{fmt(OBSTACLE_SIZE)} {fmt(OBSTACLE_SIZE)} {fmt(OBSTACLE_SIZE)}</size>
            </box>
          </geometry>
          <surface>
            <contact><ode/></contact>
            <bounce/>
            <friction>
              <torsional><ode/></torsional>
              <ode/>
            </friction>
          </surface>
        </collision>"""
        )
        visuals.append(
            f"""
        <visual name='segment_{idx}'>
          <pose>0 0 {fmt(segment_z)} 0 0 0</pose>
          <geometry>
            <box>
              <size>{fmt(OBSTACLE_SIZE)} {fmt(OBSTACLE_SIZE)} {fmt(OBSTACLE_SIZE)}</size>
            </box>
          </geometry>{colored_material_block(ambient, diffuse, specular="0.18 0.18 0.18 1")}
        </visual>"""
        )

    return f"""
    <model name='obstacle_cube'>
      <static>1</static>
      <pose>{fmt(x)} {fmt(y)} 0 0 0 0</pose>
      <link name='link'>
{''.join(collision_blocks)}{''.join(visuals)}
      </link>
    </model>"""


def make_plane_marker(name: str, x: float, y: float, size: float, material_name: str,
                      z: float = 0.03, yaw: float = 0.0) -> str:
    return f"""
    <link name='{name}'>
      <pose>{fmt(x)} {fmt(y)} 0 0 0 {fmt(yaw)}</pose>
      <collision name='collision'>
        <pose>0 0 0.01 0 0 0</pose>
        <geometry>
          <box>
            <size>{fmt(size)} {fmt(size)} 0.02</size>
          </box>
        </geometry>
      </collision>
      <visual name='marker_visual'>
        <pose>0 0 {fmt(z)} 0 0 0</pose>
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>{fmt(size)} {fmt(size)}</size>
          </plane>
        </geometry>{material_script_block(material_name)}
      </visual>
    </link>"""


def make_special_target(x: float, y: float) -> str:
    light_specs = [
        ("light_red_1", -0.28, -0.28, "0.9 0.15 0.15 1", "0.6 0.05 0.05 1"),
        ("light_blue_1", -0.28, 0.28, "0.2 0.4 0.95 1", "0.05 0.15 0.6 1"),
        ("light_red_2", 0.28, 0.28, "0.9 0.15 0.15 1", "0.6 0.05 0.05 1"),
        ("light_blue_2", 0.28, -0.28, "0.2 0.4 0.95 1", "0.05 0.15 0.6 1"),
    ]
    light_visuals = []
    for light_name, lx, ly, ambient, emissive in light_specs:
        light_visuals.append(
            f"""
      <visual name='{light_name}'>
        <pose>{fmt(lx)} {fmt(ly)} 0.07 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>0.05</radius>
          </sphere>
        </geometry>{colored_material_block(ambient, ambient, emissive=emissive)}
      </visual>"""
        )

    return f"""
    <link name='special_target'>
      <pose>{fmt(x)} {fmt(y)} 0 0 0 0</pose>
      <collision name='collision'>
        <pose>0 0 0.01 0 0 0</pose>
        <geometry>
          <box>
            <size>{fmt(TARGET_SIZE)} {fmt(TARGET_SIZE)} 0.02</size>
          </box>
        </geometry>
      </collision>
      <visual name='marker_visual'>
        <pose>0 0 0.03 0 0 0</pose>
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>{fmt(TARGET_SIZE)} {fmt(TARGET_SIZE)}</size>
          </plane>
        </geometry>{material_script_block("CRAIC/SpecialTarget")}
      </visual>{''.join(light_visuals)}
    </link>"""


def make_zone_overlay(name: str, x: float, y: float, sx: float, sy: float) -> str:
    return f"""
    <link name='{name}'>
      <pose>{fmt(x)} {fmt(y)} 0 0 0 0</pose>
      <visual name='visual'>
        <pose>0 0 0.021 0 0 0</pose>
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>{fmt(sx)} {fmt(sy)}</size>
          </plane>
        </geometry>{material_script_block("CRAIC/ZoneOverlay")}
      </visual>
    </link>"""


def make_cylinder_link(name: str, x: float, y: float, z: float, radius: float, length: float,
                       ambient: str, diffuse: str, roll: float = 0.0,
                       pitch: float = 0.0, yaw: float = 0.0,
                       collision: bool = True) -> str:
    collision_block = f"""
      <collision name='collision'>
        <geometry>
          <cylinder>
            <radius>{fmt(radius)}</radius>
            <length>{fmt(length)}</length>
          </cylinder>
        </geometry>
      </collision>""" if collision else ""
    return f"""
    <link name='{name}'>
      <pose>{fmt(x)} {fmt(y)} {fmt(z)} {fmt(roll)} {fmt(pitch)} {fmt(yaw)}</pose>{collision_block}
      <visual name='visual'>
        <geometry>
          <cylinder>
            <radius>{fmt(radius)}</radius>
            <length>{fmt(length)}</length>
          </cylinder>
        </geometry>{colored_material_block(ambient, diffuse, specular="0.55 0.55 0.55 1")}
      </visual>
    </link>"""


def build_arena_model() -> str:
    takeoff_x, takeoff_y = rule_to_world_xy(1.5, 3.0)
    qr_x, qr_y = rule_to_world_xy(3.3, 3.0)
    obstacle_x, obstacle_y = rule_to_world_xy(5.8, 3.0)
    ring_zone_center_x, ring_zone_center_y = rule_to_world_xy(7.5, 4.6)

    top_left_x, top_left_y = rule_to_world_xy(5.1, 4.6)
    top_right_x, top_right_y = rule_to_world_xy(3.3, 4.6)
    bottom_left_x, bottom_left_y = rule_to_world_xy(5.1, 1.4)
    bottom_right_x, bottom_right_y = rule_to_world_xy(3.3, 1.4)
    special_x, special_y = rule_to_world_xy(7.5, 2.0)
    landing_right_x, landing_right_y = rule_to_world_xy(1.5, 4.6)
    landing_left_x, landing_left_y = rule_to_world_xy(1.5, 1.4)

    links = [
        make_box_model("competition_floor", 3.0, 0.0, 0.005,
                       ARENA_WIDTH, ARENA_HEIGHT, 0.01, "0.88 0.88 0.88 1", "0.88 0.88 0.88 1"),
        make_box_model("north_wall", 3.0, 3.0, ARENA_WALL_HEIGHT / 2.0,
                       ARENA_WIDTH, 0.05, ARENA_WALL_HEIGHT, "0.8 0.85 0.9 0.35", "0.8 0.85 0.9 0.35", transparency=0.55),
        make_box_model("south_wall", 3.0, -3.0, ARENA_WALL_HEIGHT / 2.0,
                       ARENA_WIDTH, 0.05, ARENA_WALL_HEIGHT, "0.8 0.85 0.9 0.35", "0.8 0.85 0.9 0.35", transparency=0.55),
        make_box_model("west_wall", -1.5, 0.0, ARENA_WALL_HEIGHT / 2.0,
                       0.05, ARENA_HEIGHT, ARENA_WALL_HEIGHT, "0.8 0.85 0.9 0.35", "0.8 0.85 0.9 0.35", transparency=0.55),
        make_box_model("east_wall", 7.5, 0.0, ARENA_WALL_HEIGHT / 2.0,
                       0.05, ARENA_HEIGHT, ARENA_WALL_HEIGHT, "0.8 0.85 0.9 0.35", "0.8 0.85 0.9 0.35", transparency=0.55),
        make_obstacle_model(obstacle_x, obstacle_y),
    ]

    marker_links = [
        make_zone_overlay("ring_movement_zone", ring_zone_center_x, ring_zone_center_y, 2.4, 1.6),
        make_plane_marker("takeoff_pad", takeoff_x, takeoff_y, TARGET_SIZE, "CRAIC/TakeoffPad", yaw=-1.570796),
        make_plane_marker("landing_right_pad", landing_right_x, landing_right_y, TARGET_SIZE, "CRAIC/LandingRight", yaw=-1.570796),
        make_plane_marker("landing_left_pad", landing_left_x, landing_left_y, TARGET_SIZE, "CRAIC/LandingLeft", yaw=-1.570796),
        make_plane_marker("qr_code_marker", qr_x, qr_y, QR_SIZE, "CRAIC/QRCode", z=0.031),
        make_plane_marker("target_top_left", top_left_x, top_left_y, TARGET_SIZE, "CRAIC/TargetTopLeft"),
        make_plane_marker("target_top_right", top_right_x, top_right_y, TARGET_SIZE, "CRAIC/TargetTopRight"),
        make_plane_marker("target_bottom_left", bottom_left_x, bottom_left_y, TARGET_SIZE, "CRAIC/TargetBottomLeft"),
        make_plane_marker("target_bottom_right", bottom_right_x, bottom_right_y, TARGET_SIZE, "CRAIC/TargetBottomRight"),
        make_special_target(special_x, special_y),
    ]

    return f"""<?xml version='1.0'?>
<sdf version='1.7'>
  <model name='craic_2026_arena'>
{''.join(links)}
{''.join(marker_links)}
  </model>
</sdf>
"""


def build_ring_model() -> str:
    segment_radius = (RING_OUTER_DIAMETER + RING_INNER_DIAMETER) / 4.0
    segment_count = 24
    segment_length = (2 * math.pi * segment_radius) / segment_count
    segment_width = 0.04
    segment_radial_thickness = (RING_OUTER_DIAMETER - RING_INNER_DIAMETER) / 2.0
    collisions = []
    for idx in range(segment_count):
        angle = 2 * math.pi * idx / segment_count
        px = segment_radius * math.cos(angle)
        pz = segment_radius * math.sin(angle)
        pitch = angle + math.pi / 2.0
        collisions.append(
            f"""
      <collision name='ring_collision_{idx}'>
        <pose>{fmt(px)} 0 {fmt(pz)} 0 {fmt(pitch)} 0</pose>
        <geometry>
          <box>
            <size>{fmt(segment_length)} {fmt(segment_width)} {fmt(segment_radial_thickness)}</size>
          </box>
        </geometry>
      </collision>
"""
        )

    outer_radius = RING_OUTER_DIAMETER / 2.0
    support_top_z = RING_SUPPORT_HEIGHT
    brace_anchor_z = RING_CENTER_Z - math.sqrt(max(outer_radius ** 2 - RING_SUPPORT_X ** 2, 0.0))
    brace_dx = outer_radius - RING_SUPPORT_X
    brace_dz = brace_anchor_z - support_top_z
    brace_length = math.sqrt(brace_dx ** 2 + brace_dz ** 2)
    brace_pitch = math.atan2(brace_dx, brace_dz)

    supports = []
    for idx, px in enumerate((-RING_SUPPORT_X, RING_SUPPORT_X), start=1):
        side_sign = -1.0 if px < 0 else 1.0
        supports.append(
            make_cylinder_link(
                f"ring_support_{idx}",
                px,
                0.0,
                RING_SUPPORT_HEIGHT / 2.0,
                RING_SUPPORT_RADIUS,
                RING_SUPPORT_HEIGHT,
                "0.42 0.44 0.46 1",
                "0.42 0.44 0.46 1",
            )
        )
        supports.append(
            make_cylinder_link(
                f"ring_support_foot_{idx}",
                px,
                0.0,
                0.012,
                0.065,
                0.024,
                "0.18 0.18 0.2 1",
                "0.18 0.18 0.2 1",
            )
        )
        supports.append(
            make_cylinder_link(
                f"ring_brace_{idx}",
                px + side_sign * brace_dx / 2.0,
                0.0,
                support_top_z + brace_dz / 2.0,
                0.012,
                brace_length,
                "0.32 0.33 0.35 1",
                "0.32 0.33 0.35 1",
                pitch=side_sign * brace_pitch,
            )
        )
        supports.append(
            make_cylinder_link(
                f"ring_clamp_{idx}",
                px + side_sign * 0.05,
                0.0,
                brace_anchor_z + 0.02,
                0.03,
                RING_VISUAL_DEPTH + 0.015,
                "0.24 0.24 0.26 1",
                "0.24 0.24 0.26 1",
                roll=1.570796,
            )
        )

    return f"""<?xml version='1.0'?>
<sdf version='1.7'>
  <model name='craic_2026_ring'>
    <static>1</static>
    <link name='ring_frame'>
      <pose>0 0 {fmt(RING_CENTER_Z)} 0 0 0</pose>
{''.join(collisions)}
      <visual name='ring_visual'>
        <geometry>
          <mesh>
            <uri>model://craic_2026_ring/meshes/ring.obj</uri>
          </mesh>
        </geometry>{colored_material_block("0.8 0.82 0.84 1", "0.8 0.82 0.84 1", specular="0.9 0.9 0.9 1", emissive="0.02 0.02 0.02 1")}
      </visual>
    </link>
{''.join(supports)}
  </model>
</sdf>
"""


def build_ring_obj() -> str:
    major_radius = (RING_OUTER_DIAMETER + RING_INNER_DIAMETER) / 4.0
    radial_radius = (RING_OUTER_DIAMETER - RING_INNER_DIAMETER) / 4.0
    depth_radius = RING_VISUAL_DEPTH / 2.0
    major_segments = 64
    tube_segments = 20

    vertices = []
    normals = []
    faces = []

    for i in range(major_segments):
        u = 2.0 * math.pi * i / major_segments
        cu = math.cos(u)
        su = math.sin(u)
        for j in range(tube_segments):
            v = 2.0 * math.pi * j / tube_segments
            cv = math.cos(v)
            sv = math.sin(v)

            ring_radius = major_radius + radial_radius * cv
            x = ring_radius * cu
            y = depth_radius * sv
            z = ring_radius * su

            nx = depth_radius * cv * cu
            ny = radial_radius * sv
            nz = depth_radius * cv * su
            normal_len = math.sqrt(nx * nx + ny * ny + nz * nz)
            nx /= normal_len
            ny /= normal_len
            nz /= normal_len

            vertices.append((x, y, z))
            normals.append((nx, ny, nz))

    def vertex_index(i: int, j: int) -> int:
        return i * tube_segments + j + 1

    for i in range(major_segments):
        ni = (i + 1) % major_segments
        for j in range(tube_segments):
            nj = (j + 1) % tube_segments
            a = vertex_index(i, j)
            b = vertex_index(ni, j)
            c = vertex_index(ni, nj)
            d = vertex_index(i, nj)
            faces.append((a, b, c))
            faces.append((a, c, d))

    lines = ["# Auto-generated CRAIC 2026 ring visual mesh"]
    lines.extend(f"v {fmt(x)} {fmt(y)} {fmt(z)}" for x, y, z in vertices)
    lines.extend(f"vn {fmt(nx)} {fmt(ny)} {fmt(nz)}" for nx, ny, nz in normals)
    lines.extend(f"f {a}//{a} {b}//{b} {c}//{c}" for a, b, c in faces)
    return "\n".join(lines) + "\n"


def build_world(ring_rule_x: float) -> str:
    if not (RING_RULE_MIN_X <= ring_rule_x <= RING_RULE_MAX_X):
        raise ValueError(f"ring rule x must be within [{RING_RULE_MIN_X}, {RING_RULE_MAX_X}]")

    ring_world_x, ring_world_y = rule_to_world_xy(ring_rule_x, RING_CENTER_Y)
    return f"""<sdf version='1.7'>
  <world name='craic_2026'>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.85 0.85 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.35 0.2 -0.9</direction>
    </light>
    <scene>
      <ambient>0.65 0.65 0.65 1</ambient>
      <background>0.96 0.97 1.0 1</background>
      <shadows>true</shadows>
    </scene>
    <physics type='ode'>
      <max_step_size>0.004</max_step_size>
      <real_time_update_rate>250</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>3 -7 6 0.95 0 1.57</pose>
      </camera>
    </gui>
    <include>
      <uri>model://sun</uri>
    </include>
    <model name='craic_arena_instance'>
      <static>1</static>
      <pose>0 0 0 0 0 0</pose>
      <include>
        <uri>model://craic_2026_arena</uri>
      </include>
    </model>
    <model name='craic_ring_instance'>
      <static>1</static>
      <pose>{fmt(ring_world_x)} {fmt(ring_world_y)} 0 0 0 0</pose>
      <include>
        <uri>model://craic_2026_ring</uri>
      </include>
    </model>
  </world>
</sdf>
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate CRAIC 2026 Gazebo Classic assets and worlds.")
    parser.add_argument("--output", type=Path, default=RUNTIME_WORLD_PATH, help="Output world path.")
    parser.add_argument("--ring-x", type=float, default=None,
                        help="Ring x in rule coordinates (meters), valid range [6.3, 8.7].")
    parser.add_argument("--seed", type=int, default=None, help="Random seed used for ring x sampling in rule coordinates.")
    parser.add_argument("--write-static", action="store_true",
                        help="Write the arena model, ring model, and default static world.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.write_static:
        ARENA_MODEL_DIR.mkdir(parents=True, exist_ok=True)
        RING_MODEL_DIR.mkdir(parents=True, exist_ok=True)
        RING_MESH_DIR.mkdir(parents=True, exist_ok=True)
        WORLDS_ROOT.mkdir(parents=True, exist_ok=True)
        (ARENA_MODEL_DIR / "model.sdf").write_text(build_arena_model(), encoding="utf-8")
        (RING_MODEL_DIR / "model.sdf").write_text(build_ring_model(), encoding="utf-8")
        RING_MESH_PATH.write_text(build_ring_obj(), encoding="utf-8")
        STATIC_WORLD_PATH.write_text(build_world(RING_DEFAULT_RULE_X), encoding="utf-8")
        print(f"[CRAIC] Static assets written under {ARENA_MODEL_DIR} and {RING_MODEL_DIR}")
        print(f"[CRAIC] Static world written to {STATIC_WORLD_PATH}")

    if args.ring_x is not None:
        ring_rule_x = args.ring_x
    else:
        rng = random.Random(args.seed)
        ring_rule_x = rng.uniform(RING_RULE_MIN_X, RING_RULE_MAX_X)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(build_world(ring_rule_x), encoding="utf-8")
    print(f"[CRAIC] Runtime world written to {args.output.resolve()}")
    print(f"[CRAIC] Ring rule x={ring_rule_x:.3f} m, world pose=({rule_to_world_xy(ring_rule_x, RING_CENTER_Y)[0]:.3f}, {rule_to_world_xy(ring_rule_x, RING_CENTER_Y)[1]:.3f})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
