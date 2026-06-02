#!/usr/bin/env python3

import argparse
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


PAIR_RE = re.compile(r'\[[^\]]+\]\s+"([^",]+)\s*,\s*([^"]+)"')


def parse_urdf(path):
    root = ET.parse(path).getroot()
    links = {}
    for link in root.findall("link"):
        name = link.get("name")
        if name:
            links[name] = link
    joints = {joint.get("name") for joint in root.findall("joint") if joint.get("name")}
    return links, joints


def collision_links(links):
    return {name for name, link in links.items() if link.findall("collision")}


def parse_vector(value, expected_size, label):
    if value is None:
        return [0.0] * expected_size
    parts = value.split()
    if len(parts) != expected_size:
        raise ValueError(f"{label} should have {expected_size} entries, got {len(parts)}")
    parsed = [float(part) for part in parts]
    if not all(math.isfinite(number) for number in parsed):
        raise ValueError(f"{label} contains a non-finite value")
    return parsed


def validate_sphere_collisions(links):
    sphere_count = 0
    errors = []
    for link_name, link in links.items():
        for index, collision in enumerate(link.findall("collision")):
            geometry = collision.find("geometry")
            sphere = geometry.find("sphere") if geometry is not None else None
            if sphere is None:
                errors.append(f"{link_name}: collision[{index}] is not a sphere")
                continue

            radius_text = sphere.get("radius")
            try:
                radius = float(radius_text) if radius_text is not None else float("nan")
                if not math.isfinite(radius) or radius <= 0.0:
                    raise ValueError
            except ValueError:
                errors.append(f"{link_name}: collision[{index}] has invalid sphere radius {radius_text!r}")
                continue

            origin = collision.find("origin")
            try:
                if origin is not None:
                    parse_vector(origin.get("xyz"), 3, f"{link_name}: collision[{index}] origin xyz")
                    parse_vector(origin.get("rpy"), 3, f"{link_name}: collision[{index}] origin rpy")
            except ValueError as exc:
                errors.append(str(exc))
                continue

            sphere_count += 1
    return sphere_count, errors


def pairs_from_info(path):
    text = Path(path).read_text()
    return [(match.group(1).strip(), match.group(2).strip()) for match in PAIR_RE.finditer(text)]


def validate_pairs(path, mesh_collision_links, sphere_collision_links):
    errors = []
    pairs = pairs_from_info(path)
    for link1, link2 in pairs:
        missing_mesh = [link for link in (link1, link2) if link not in mesh_collision_links]
        missing_sphere = [link for link in (link1, link2) if link not in sphere_collision_links]
        if missing_mesh:
            errors.append(f"{link1}, {link2}: missing from mesh URDF collision links: {','.join(missing_mesh)}")
        if missing_sphere:
            errors.append(f"{link1}, {link2}: missing from sphere URDF collision links: {','.join(missing_sphere)}")
    return len(pairs), errors


def main():
    parser = argparse.ArgumentParser(description="Validate structural consistency between mesh and sphere URDF collision models.")
    parser.add_argument("--mesh-urdf", required=True, help="Original mesh/primitives URDF")
    parser.add_argument("--sphere-urdf", required=True, help="Spherized URDF")
    parser.add_argument("--pairs", help="Optional OCS2 collisionLinkPairs info file to validate against both URDFs")
    args = parser.parse_args()

    mesh_links, mesh_joints = parse_urdf(args.mesh_urdf)
    sphere_links, sphere_joints = parse_urdf(args.sphere_urdf)
    mesh_collision_links = collision_links(mesh_links)
    sphere_collision_links = collision_links(sphere_links)

    errors = []
    missing_links = sorted(set(mesh_links) - set(sphere_links))
    extra_links = sorted(set(sphere_links) - set(mesh_links))
    missing_joints = sorted(mesh_joints - sphere_joints)
    extra_joints = sorted(sphere_joints - mesh_joints)
    missing_collision_links = sorted(mesh_collision_links - sphere_collision_links)

    if missing_links:
        errors.append("Missing links in sphere URDF: " + ", ".join(missing_links))
    if extra_links:
        errors.append("Extra links in sphere URDF: " + ", ".join(extra_links))
    if missing_joints:
        errors.append("Missing joints in sphere URDF: " + ", ".join(missing_joints))
    if extra_joints:
        errors.append("Extra joints in sphere URDF: " + ", ".join(extra_joints))
    if missing_collision_links:
        errors.append("Links lost all collision geometry in sphere URDF: " + ", ".join(missing_collision_links))

    sphere_count, sphere_errors = validate_sphere_collisions(sphere_links)
    errors.extend(sphere_errors)

    pair_count = None
    if args.pairs:
        pair_count, pair_errors = validate_pairs(args.pairs, mesh_collision_links, sphere_collision_links)
        errors.extend(pair_errors)

    print(f"Mesh URDF links: {len(mesh_links)}")
    print(f"Mesh collision links: {len(mesh_collision_links)}")
    print(f"Sphere URDF links: {len(sphere_links)}")
    print(f"Sphere collision links: {len(sphere_collision_links)}")
    print(f"Sphere collisions: {sphere_count}")
    if pair_count is not None:
        print(f"Validated collisionLinkPairs: {pair_count}")

    if errors:
        print("Consistency errors:", file=sys.stderr)
        for error in errors:
            print(f"  {error}", file=sys.stderr)
        return 1

    print("Sphere URDF is structurally consistent with mesh URDF")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
