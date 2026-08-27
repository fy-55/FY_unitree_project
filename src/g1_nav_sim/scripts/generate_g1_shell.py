#!/usr/bin/env python3
"""Generate a rigid, visual-only G1 shell inside the planar proxy model."""

import argparse
import math
from pathlib import Path
import re
import struct
import xml.etree.ElementTree as ET


BEGIN_MARKER = "      <!-- BEGIN GENERATED G1 SHELL VISUALS -->"
END_MARKER = "      <!-- END GENERATED G1 SHELL VISUALS -->"


def identity():
    return (
        ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        (0.0, 0.0, 0.0),
    )


def rpy_to_rotation(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def multiply_rotation(left, right):
    return tuple(
        tuple(sum(left[row][k] * right[k][col] for k in range(3))
              for col in range(3))
        for row in range(3)
    )


def rotate(rotation, vector):
    return tuple(
        sum(rotation[row][col] * vector[col] for col in range(3))
        for row in range(3)
    )


def compose(parent, child):
    parent_rotation, parent_translation = parent
    child_rotation, child_translation = child
    rotated_translation = rotate(parent_rotation, child_translation)
    return (
        multiply_rotation(parent_rotation, child_rotation),
        tuple(parent_translation[i] + rotated_translation[i] for i in range(3)),
    )


def transform_point(transform, point):
    rotation, translation = transform
    rotated = rotate(rotation, point)
    return tuple(rotated[i] + translation[i] for i in range(3))


def rotation_to_rpy(rotation):
    horizontal = math.hypot(rotation[0][0], rotation[1][0])
    pitch = math.atan2(-rotation[2][0], horizontal)
    if horizontal > 1.0e-9:
        roll = math.atan2(rotation[2][1], rotation[2][2])
        yaw = math.atan2(rotation[1][0], rotation[0][0])
    else:
        roll = math.atan2(-rotation[1][2], rotation[1][1])
        yaw = 0.0
    return roll, pitch, yaw


def parse_vector(text, default):
    if not text:
        return default
    values = tuple(float(value) for value in text.split())
    if len(values) != 3:
        raise ValueError(f"Expected three values, got: {text}")
    return values


def origin_transform(element):
    origin = element.find("origin")
    if origin is None:
        return identity()
    xyz = parse_vector(origin.get("xyz"), (0.0, 0.0, 0.0))
    rpy = parse_vector(origin.get("rpy"), (0.0, 0.0, 0.0))
    return rpy_to_rotation(*rpy), xyz


def mesh_vertices(mesh_path):
    data = mesh_path.read_bytes()
    if len(data) >= 84:
        triangle_count = struct.unpack_from("<I", data, 80)[0]
        if 84 + triangle_count * 50 == len(data):
            for index in range(triangle_count):
                values = struct.unpack_from("<12fH", data, 84 + index * 50)
                yield values[3:6]
                yield values[6:9]
                yield values[9:12]
            return

    text = data.decode("ascii", errors="ignore")
    number = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
    pattern = re.compile(rf"\bvertex\s+({number})\s+({number})\s+({number})")
    found = False
    for match in pattern.finditer(text):
        found = True
        yield tuple(float(match.group(i)) for i in range(1, 4))
    if not found:
        raise ValueError(f"Unsupported or empty STL file: {mesh_path}")


def fmt(value):
    if abs(value) < 5.0e-10:
        value = 0.0
    return f"{value:.9g}"


def pose_text(transform, z_offset):
    rotation, translation = transform
    roll, pitch, yaw = rotation_to_rpy(rotation)
    values = (
        translation[0],
        translation[1],
        translation[2] + z_offset,
        roll,
        pitch,
        yaw,
    )
    return " ".join(fmt(value) for value in values)


def collect_link_transforms(robot):
    parent_joint = {}
    for joint in robot.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        parent_joint[child.get("link")] = (parent.get("link"), origin_transform(joint))

    cache = {"base_link": identity()}
    visiting = set()

    def resolve(link_name):
        if link_name in cache:
            return cache[link_name]
        if link_name in visiting:
            raise ValueError(f"Joint cycle detected at link: {link_name}")
        if link_name not in parent_joint:
            raise ValueError(f"Link is not connected to base_link: {link_name}")
        visiting.add(link_name)
        parent_name, joint_transform = parent_joint[link_name]
        transform = compose(resolve(parent_name), joint_transform)
        visiting.remove(link_name)
        cache[link_name] = transform
        return transform

    for link in robot.findall("link"):
        resolve(link.get("name"))
    return cache


def collect_visuals(robot, link_transforms):
    visuals = []
    seen = set()
    for link in robot.findall("link"):
        link_name = link.get("name")
        for visual_index, visual in enumerate(link.findall("visual")):
            visual_transform = compose(link_transforms[link_name], origin_transform(visual))
            color_element = visual.find("material/color")
            color = "0.7 0.7 0.7 1"
            if color_element is not None and color_element.get("rgba"):
                color = color_element.get("rgba")
            for mesh_index, mesh in enumerate(visual.findall("geometry/mesh")):
                filename = mesh.get("filename")
                if not filename:
                    continue
                mesh_name = Path(filename).name
                key = (link_name, mesh_name, pose_text(visual_transform, 0.0))
                if key in seen:
                    continue
                seen.add(key)
                visuals.append({
                    "name": f"shell_{link_name}_{visual_index}_{mesh_index}",
                    "mesh_name": mesh_name,
                    "transform": visual_transform,
                    "color": color,
                })
    return visuals


def calculate_bounds(visuals, mesh_directory):
    lower = [math.inf, math.inf, math.inf]
    upper = [-math.inf, -math.inf, -math.inf]
    for visual in visuals:
        mesh_path = mesh_directory / visual["mesh_name"]
        if not mesh_path.is_file():
            raise FileNotFoundError(f"G1 mesh does not exist: {mesh_path}")
        for vertex in mesh_vertices(mesh_path):
            point = transform_point(visual["transform"], vertex)
            for axis in range(3):
                lower[axis] = min(lower[axis], point[axis])
                upper[axis] = max(upper[axis], point[axis])
    return lower, upper


def generated_block(visuals, z_offset, bounds, mesh_resource_root):
    lower, upper = bounds
    lines = [
        BEGIN_MARKER,
        "      <!--",
        "        Generated from the official 29-DoF G1 URDF at zero joint angles.",
        "        These meshes are visual-only; navigation collision and dynamics",
        "        remain the simple planar proxy defined above.",
        f"        Pelvis height above base_footprint: {fmt(z_offset)} m.",
        "        Shell bounds after alignment (m):",
        "        "
        f"x=[{fmt(lower[0])}, {fmt(upper[0])}], "
        f"y=[{fmt(lower[1])}, {fmt(upper[1])}], "
        f"z=[{fmt(lower[2] + z_offset)}, {fmt(upper[2] + z_offset)}].",
        "      -->",
    ]
    for visual in visuals:
        lines.extend([
            f"      <visual name=\"{visual['name']}\">",
            f"        <pose>{pose_text(visual['transform'], z_offset)}</pose>",
            "        <geometry>",
            "          <mesh>",
            "            <uri>"
            f"file://{mesh_resource_root}/meshes/{visual['mesh_name']}"
            "</uri>",
            "          </mesh>",
            "        </geometry>",
            "        <material>",
            f"          <ambient>{visual['color']}</ambient>",
            f"          <diffuse>{visual['color']}</diffuse>",
            "        </material>",
            "        <cast_shadows>true</cast_shadows>",
            "      </visual>",
        ])
    lines.append(END_MARKER)
    return "\n".join(lines)


def main():
    script_path = Path(__file__).resolve()
    source_directory = script_path.parents[2]
    default_description = source_directory / "g1_nav_description"
    default_simulation = script_path.parents[1]

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--urdf",
        type=Path,
        default=default_description / "urdf/g1_nav.urdf.xacro",
    )
    parser.add_argument(
        "--mesh-directory",
        type=Path,
        default=default_description / "meshes",
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=default_simulation / "models/g1_nav_proxy/model.sdf",
    )
    parser.add_argument("--mesh-resource-root", default="g1_nav_description")
    parser.add_argument("--ground-clearance", type=float, default=0.002)
    args = parser.parse_args()

    robot = ET.parse(args.urdf).getroot()
    link_transforms = collect_link_transforms(robot)
    visuals = collect_visuals(robot, link_transforms)
    raw_bounds = calculate_bounds(visuals, args.mesh_directory)
    z_offset = -raw_bounds[0][2] + args.ground_clearance

    model_text = args.model.read_text(encoding="utf-8")
    begin = model_text.find(BEGIN_MARKER)
    end = model_text.find(END_MARKER)
    if begin < 0 or end < 0 or end < begin:
        raise ValueError("Generated G1 shell marker pair is missing from model.sdf")
    end += len(END_MARKER)
    block = generated_block(
        visuals,
        z_offset,
        raw_bounds,
        args.mesh_resource_root,
    )
    args.model.write_text(model_text[:begin] + block + model_text[end:], encoding="utf-8")

    print(f"Generated {len(visuals)} G1 shell visuals")
    print(f"Pelvis height: {z_offset:.6f} m")
    print(f"Lowest shell point: {raw_bounds[0][2] + z_offset:.6f} m")
    print(f"Highest shell point: {raw_bounds[1][2] + z_offset:.6f} m")
    print(f"Updated: {args.model}")


if __name__ == "__main__":
    main()
