#!/usr/bin/env python3

import argparse
import itertools
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


PAIR_RE = re.compile(r'\[[^\]]+\]\s+"([^",]+)\s*,\s*([^"]+)"')


def collision_links_from_urdf(urdf_path):
    root = ET.parse(urdf_path).getroot()
    links = []
    for link in root.findall("link"):
        name = link.get("name")
        if name and link.findall("collision"):
            links.append(name)
    return links


def disabled_pairs_from_srdf(srdf_path):
    if not srdf_path:
        return set()
    root = ET.parse(srdf_path).getroot()
    pairs = set()
    for elem in root.findall("disable_collisions"):
        link1 = elem.get("link1")
        link2 = elem.get("link2")
        if link1 and link2:
            pairs.add(tuple(sorted((link1, link2))))
    return pairs


def pairs_from_info(info_path, topic="collisionLinkPairs"):
    text = Path(info_path).read_text()
    topic_index = text.find(topic)
    if topic_index < 0:
        raise RuntimeError(f'Could not find topic "{topic}" in {info_path}')
    open_index = text.find("{", topic_index)
    if open_index < 0:
        raise RuntimeError(f'Could not find opening brace for "{topic}" in {info_path}')

    depth = 0
    close_index = None
    for index in range(open_index, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                close_index = index
                break
    if close_index is None:
        raise RuntimeError(f'Could not find closing brace for "{topic}" in {info_path}')

    block = text[open_index + 1 : close_index]
    return [(match.group(1).strip(), match.group(2).strip()) for match in PAIR_RE.finditer(block)]


def format_pairs(pairs, topic="collisionLinkPairs"):
    lines = [topic, "{"]
    for index, (link1, link2) in enumerate(pairs):
        lines.append(f'  [{index}] "{link1}, {link2}"')
    lines.append("}")
    return "\n".join(lines) + "\n"


def replace_topic_block(text, topic, replacement):
    topic_index = text.find(topic)
    if topic_index < 0:
        raise RuntimeError(f'Could not find topic "{topic}"')
    open_index = text.find("{", topic_index)
    if open_index < 0:
        raise RuntimeError(f'Could not find opening brace for "{topic}"')

    depth = 0
    close_index = None
    for index in range(open_index, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                close_index = index
                break
    if close_index is None:
        raise RuntimeError(f'Could not find closing brace for "{topic}"')

    line_start = text.rfind("\n", 0, topic_index) + 1
    indent = re.match(r"[ \t]*", text[line_start:topic_index]).group(0)
    replacement = "\n".join(indent + line if line else line for line in replacement.rstrip().splitlines())
    return text[:line_start] + replacement + text[close_index + 1 :]


def export_pairs(args):
    links = collision_links_from_urdf(args.urdf)
    disabled = disabled_pairs_from_srdf(args.srdf)
    all_pairs = {tuple(sorted(pair)) for pair in itertools.combinations(links, 2)}
    if args.mode == "enabled":
        pairs = sorted(all_pairs - disabled)
    else:
        pairs = sorted(disabled & all_pairs)
    output = format_pairs(pairs, args.topic)
    if args.output:
        Path(args.output).write_text(output)
    else:
        print(output, end="")
    print(f"Exported {len(pairs)} {args.mode} collision link pairs from {len(links)} collision links", file=sys.stderr)


def validate_pairs(args):
    valid_links = set(collision_links_from_urdf(args.urdf))
    pairs = pairs_from_info(args.pairs, args.topic)
    invalid = invalid_pairs(pairs, valid_links)
    print(f"Loaded {len(pairs)} pairs from {args.pairs}")
    print(f"URDF has {len(valid_links)} links with collision geometry")
    if invalid:
        print("Invalid pairs:")
        for link1, link2, missing in invalid:
            print(f"  {link1}, {link2}  missing={','.join(missing)}")
        return 1
    print("All pairs reference links with collision geometry")
    return 0


def invalid_pairs(pairs, valid_links):
    invalid = []
    for link1, link2 in pairs:
        missing = [link for link in (link1, link2) if link not in valid_links]
        if missing:
            invalid.append((link1, link2, missing))
    return invalid


def validate_pair_list_or_raise(pairs, urdf_path):
    valid_links = set(collision_links_from_urdf(urdf_path))
    invalid = invalid_pairs(pairs, valid_links)
    if invalid:
        lines = [f"{len(invalid)} pair(s) do not reference collision links in {urdf_path}:"]
        for link1, link2, missing in invalid:
            lines.append(f"  {link1}, {link2}  missing={','.join(missing)}")
        raise RuntimeError("\n".join(lines))


def patch_task(args):
    pairs = pairs_from_info(args.pairs, args.topic)
    if args.urdf:
        validate_pair_list_or_raise(pairs, args.urdf)
    replacement = format_pairs(pairs, args.topic)
    task_path = Path(args.task)
    output_path = Path(args.output) if args.output else task_path
    output_path.write_text(replace_topic_block(task_path.read_text(), args.topic, replacement))
    print(f"Patched {len(pairs)} pairs into {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Export, validate, and patch OCS2 collisionLinkPairs without GUI.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    export_parser = subparsers.add_parser("export", help="Export OCS2 collisionLinkPairs from URDF and optional SRDF")
    export_parser.add_argument("--urdf", required=True)
    export_parser.add_argument("--srdf")
    export_parser.add_argument("--output")
    export_parser.add_argument("--topic", default="collisionLinkPairs")
    export_parser.add_argument("--mode", choices=["enabled", "disabled"], default="enabled")
    export_parser.set_defaults(func=export_pairs)

    validate_parser = subparsers.add_parser("validate", help="Validate an OCS2 collisionLinkPairs info file against a URDF")
    validate_parser.add_argument("--urdf", required=True)
    validate_parser.add_argument("--pairs", required=True)
    validate_parser.add_argument("--topic", default="collisionLinkPairs")
    validate_parser.set_defaults(func=validate_pairs)

    patch_parser = subparsers.add_parser("patch-task", help="Patch a collisionLinkPairs block into a task.info")
    patch_parser.add_argument("--task", required=True)
    patch_parser.add_argument("--pairs", required=True)
    patch_parser.add_argument("--output")
    patch_parser.add_argument("--urdf", help="Validate pair link names against this URDF before writing")
    patch_parser.add_argument("--topic", default="collisionLinkPairs")
    patch_parser.set_defaults(func=patch_task)

    args = parser.parse_args()
    try:
        result = args.func(args)
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    if isinstance(result, int):
        return result
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
