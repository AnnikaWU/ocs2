#!/usr/bin/env python3

import argparse
import copy
import math
import xml.etree.ElementTree as ET
from pathlib import Path


def parse_vector(text, default):
    if text is None:
        return list(default)
    return [float(value) for value in text.split()]


def format_float(value):
    return f"{value:.10g}"


def snap_to_reference(value, reference, tolerance):
    return reference if abs(value - reference) < tolerance else value


def snap_vector_to_reference(values, reference, tolerance):
    return [snap_to_reference(value, ref, tolerance) for value, ref in zip(values, reference)]


def format_vector(values):
    return " ".join(format_float(value) for value in values)


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def mat_vec_mul(matrix, vector):
    return [sum(matrix[row][col] * vector[col] for col in range(3)) for row in range(3)]


def make_sphere_collision(name, xyz, radius):
    collision = ET.Element("collision")
    if name:
        collision.set("name", name)
    origin = ET.SubElement(collision, "origin")
    origin.set("xyz", format_vector(xyz))
    origin.set("rpy", "0 0 0")
    geometry = ET.SubElement(collision, "geometry")
    sphere = ET.SubElement(geometry, "sphere")
    sphere.set("radius", format_float(radius))
    return collision


def sphere_signature(collision, tolerance):
    geometry = collision.find("geometry")
    sphere = geometry.find("sphere") if geometry is not None else None
    if sphere is None:
        return None

    origin = collision.find("origin")
    xyz = parse_vector(origin.get("xyz") if origin is not None else None, [0.0, 0.0, 0.0])
    rpy = parse_vector(origin.get("rpy") if origin is not None else None, [0.0, 0.0, 0.0])
    radius = float(sphere.get("radius"))

    scale = 1.0 / tolerance
    values = xyz + rpy + [radius]
    return tuple(int(round(value * scale)) for value in values)


def deduplicate_spheres(collisions, tolerance):
    seen = set()
    unique = []
    dropped = 0
    for collision in collisions:
        signature = sphere_signature(collision, tolerance)
        if signature is not None and signature in seen:
            dropped += 1
            continue
        if signature is not None:
            seen.add(signature)
        unique.append(collision)
    return unique, dropped


def spherize_cylinder(collision, base_name, radius, length, step_ratio, snap_tolerance):
    origin = collision.find("origin")
    xyz = parse_vector(origin.get("xyz") if origin is not None else None, [0.0, 0.0, 0.0])
    rpy = parse_vector(origin.get("rpy") if origin is not None else None, [0.0, 0.0, 0.0])
    rotation = rpy_to_matrix(*rpy)

    step = max(radius * step_ratio, 1e-6)
    sphere_count = max(1, int(math.ceil(length / step)) + 1)
    if sphere_count == 1:
        offsets = [0.0]
    else:
        offsets = [-0.5 * length + length * i / (sphere_count - 1) for i in range(sphere_count)]

    sphere_collisions = []
    for index, offset in enumerate(offsets):
        rotated = mat_vec_mul(rotation, [0.0, 0.0, offset])
        center = [xyz[i] + rotated[i] for i in range(3)]
        center = snap_vector_to_reference(center, xyz, snap_tolerance)
        sphere_collisions.append(make_sphere_collision(f"{base_name}_sphere_{index}", center, radius))
    return sphere_collisions


def spherize_box(collision, base_name, size_text, step_ratio, snap_tolerance):
    size = parse_vector(size_text, [0.0, 0.0, 0.0])
    if any(dimension <= 0.0 for dimension in size):
        raise ValueError(f"Invalid box size: {size_text}")

    origin = collision.find("origin")
    xyz = parse_vector(origin.get("xyz") if origin is not None else None, [0.0, 0.0, 0.0])
    rpy = parse_vector(origin.get("rpy") if origin is not None else None, [0.0, 0.0, 0.0])
    rotation = rpy_to_matrix(*rpy)

    radius = min(size) * 0.5
    step = max(radius * step_ratio, 1e-6)
    axes = []
    for dimension in size:
        count = max(1, int(math.ceil(dimension / step)) + 1)
        if count == 1:
            axes.append([0.0])
        else:
            axes.append([-0.5 * dimension + dimension * i / (count - 1) for i in range(count)])

    sphere_collisions = []
    index = 0
    for x in axes[0]:
        for y in axes[1]:
            for z in axes[2]:
                rotated = mat_vec_mul(rotation, [x, y, z])
                center = [xyz[i] + rotated[i] for i in range(3)]
                center = snap_vector_to_reference(center, xyz, snap_tolerance)
                sphere_collisions.append(make_sphere_collision(f"{base_name}_sphere_{index}", center, radius))
                index += 1
    return sphere_collisions


def spherize_collision(collision, base_name, cylinder_step_ratio, box_step_ratio, snap_tolerance):
    geometry = collision.find("geometry")
    if geometry is None:
        raise ValueError("collision element has no geometry")

    sphere = geometry.find("sphere")
    if sphere is not None:
        return [copy.deepcopy(collision)]

    cylinder = geometry.find("cylinder")
    if cylinder is not None:
        radius = float(cylinder.get("radius"))
        length = float(cylinder.get("length"))
        if radius <= 0.0 or length <= 0.0:
            raise ValueError("cylinder radius and length must be positive")
        return spherize_cylinder(collision, base_name, radius, length, cylinder_step_ratio, snap_tolerance)

    box = geometry.find("box")
    if box is not None:
        return spherize_box(collision, base_name, box.get("size"), box_step_ratio, snap_tolerance)

    mesh = geometry.find("mesh")
    if mesh is not None:
        raise ValueError("mesh collision found; use foam_batch_spherize.sh for mesh URDFs")

    raise ValueError("unsupported collision geometry")


def main():
    parser = argparse.ArgumentParser(description="Convert primitive URDF collision geometry to sphere-only collision geometry.")
    parser.add_argument("--input", required=True, help="Input URDF")
    parser.add_argument("--output", required=True, help="Output sphere URDF")
    parser.add_argument("--cylinder-step-ratio", type=float, default=0.5, help="Cylinder sampling step as radius multiple")
    parser.add_argument("--box-step-ratio", type=float, default=1.0, help="Box sampling step as radius multiple")
    parser.add_argument("--deduplicate-tolerance", type=float, default=1e-9, help="Tolerance for dropping duplicate sphere collisions")
    parser.add_argument(
        "--snap-tolerance",
        type=float,
        default=1e-14,
        help="Snap generated coordinates back to the source collision origin component when the difference is below this tolerance",
    )
    args = parser.parse_args()

    tree = ET.parse(args.input)
    root = tree.getroot()
    total_spheres = 0
    dropped_spheres = 0

    for link in root.findall("link"):
        original_collisions = list(link.findall("collision"))
        for collision in original_collisions:
            link.remove(collision)
        link_spheres = []
        link_name = link.get("name", "link")
        for collision_index, collision in enumerate(original_collisions):
            base_name = collision.get("name") or f"{link_name}_collision_{collision_index}"
            link_spheres.extend(spherize_collision(collision, base_name, args.cylinder_step_ratio, args.box_step_ratio, args.snap_tolerance))
        link_spheres, dropped = deduplicate_spheres(link_spheres, args.deduplicate_tolerance)
        total_spheres += len(link_spheres)
        dropped_spheres += dropped
        for sphere_collision in link_spheres:
            link.append(sphere_collision)

    ET.indent(tree, space="  ")
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    print(f"Wrote {total_spheres} sphere collision elements to {output}")
    if dropped_spheres:
        print(f"Dropped {dropped_spheres} duplicate sphere collision elements")


if __name__ == "__main__":
    main()
