#!/usr/bin/env python3
"""Summarize scalar/SIMD speedups from Google Benchmark JSON."""

import argparse
import json
import math
import re
import statistics
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple


TIME_TO_NS = {"ns": 1.0, "us": 1e3, "ms": 1e6, "s": 1e9}
MODE_PATTERN = re.compile(r"(?<=/)(?P<prefix>Normal)?(?P<mode>Scalar|Simd)(?=/)")


class DataError(RuntimeError):
    pass


def canonical_name(name: str) -> Tuple[str, str]:
    matches = list(MODE_PATTERN.finditer(name))
    if len(matches) != 1:
        raise DataError(f"benchmark name must contain exactly one Scalar or Simd token: {name!r}")
    match = matches[0]
    prefix = match.group("prefix") or ""
    return name[: match.start()] + prefix + "{mode}" + name[match.end() :], match.group("mode").lower()


def normalized_time(entry: Dict[str, object], field: str) -> float:
    try:
        value = float(entry[field])
        factor = TIME_TO_NS[str(entry["time_unit"])]
    except (KeyError, TypeError, ValueError) as error:
        raise DataError(f"invalid benchmark time entry: {entry}") from error
    result = value * factor
    if not math.isfinite(result) or result <= 0.0:
        raise DataError(f"benchmark time must be finite and positive: {entry}")
    return result


def select_time(entries: Iterable[Dict[str, object]], aggregate: str, field: str) -> float:
    materialized = list(entries)
    preferred = [entry for entry in materialized if entry.get("aggregate_name") == aggregate]
    if not preferred and aggregate != "mean":
        preferred = [entry for entry in materialized if entry.get("aggregate_name") == "mean"]
    if not preferred:
        preferred = [entry for entry in materialized if not entry.get("aggregate_name")]
    if not preferred:
        raise DataError("no usable timing row for benchmark")
    return statistics.median(normalized_time(entry, field) for entry in preferred)


def summarize(document: Dict[str, object], aggregate: str, field: str, name_filter: str = "") -> List[Dict[str, object]]:
    raw_entries = document.get("benchmarks")
    if not isinstance(raw_entries, list):
        raise DataError("JSON does not contain a benchmarks array")
    filter_pattern = re.compile(name_filter) if name_filter else None
    grouped: Dict[str, Dict[str, List[Dict[str, object]]]] = {}
    for raw_entry in raw_entries:
        if not isinstance(raw_entry, dict):
            continue
        name_value = raw_entry.get("run_name") or raw_entry.get("name")
        if not isinstance(name_value, str) or not MODE_PATTERN.search(name_value):
            continue
        key, mode = canonical_name(name_value)
        if filter_pattern is not None and not filter_pattern.search(key):
            continue
        grouped.setdefault(key, {}).setdefault(mode, []).append(raw_entry)

    if not grouped:
        raise DataError("no scalar/SIMD benchmark pairs matched")
    rows: List[Dict[str, object]] = []
    for key in sorted(grouped):
        modes = grouped[key]
        missing = [mode for mode in ("scalar", "simd") if mode not in modes]
        if missing:
            raise DataError(f"{key}: missing {', '.join(missing)} result")
        scalar_ns = select_time(modes["scalar"], aggregate, field)
        simd_ns = select_time(modes["simd"], aggregate, field)
        rows.append(
            {
                "benchmark": key.replace("{mode}", "[Scalar|Simd]"),
                "scalar_ns": scalar_ns,
                "simd_ns": simd_ns,
                "speedup": scalar_ns / simd_ns,
            }
        )
    return rows


def parse_speedup_rule(text: str) -> Tuple[re.Pattern, float]:
    try:
        pattern_text, ratio_text = text.rsplit("=", 1)
        ratio = float(ratio_text)
        pattern = re.compile(pattern_text)
    except (ValueError, re.error) as error:
        raise argparse.ArgumentTypeError("expected REGEX=RATIO") from error
    if not pattern_text or not math.isfinite(ratio) or ratio <= 0.0:
        raise argparse.ArgumentTypeError("expected non-empty REGEX and positive finite RATIO")
    return pattern, ratio


def gate_speedups(rows: Sequence[Dict[str, object]], rules: Sequence[Tuple[re.Pattern, float]]) -> List[str]:
    failures: List[str] = []
    for pattern, minimum in rules:
        matches = [row for row in rows if pattern.search(str(row["benchmark"]))]
        if not matches:
            failures.append(f"speedup rule {pattern.pattern!r} matched no benchmark")
            continue
        for row in matches:
            speedup = float(row["speedup"])
            if speedup < minimum:
                failures.append(f"{row['benchmark']}: {speedup:.3f}x is below {minimum:.3f}x")
    return failures


def print_human(rows: Sequence[Dict[str, object]], failures: Sequence[str]) -> None:
    print("benchmark\tscalar_ns\tsimd_ns\tspeedup")
    for row in rows:
        print(f"{row['benchmark']}\t{row['scalar_ns']:.3f}\t{row['simd_ns']:.3f}\t{row['speedup']:.3f}x")
    for failure in failures:
        print(f"failure: {failure}")


def run_self_test() -> None:
    document = {
        "benchmarks": [
            {"run_name": "Jacobian/Scalar/629", "cpu_time": 300.0, "time_unit": "ns", "aggregate_name": "median"},
            {"run_name": "Jacobian/Simd/629", "cpu_time": 180.0, "time_unit": "ns", "aggregate_name": "median"},
            {"run_name": "Distance/Scalar/20", "cpu_time": 2.0, "time_unit": "us"},
            {"run_name": "Distance/Simd/20", "cpu_time": 1000.0, "time_unit": "ns"},
            {"run_name": "Jacobian/TiledScalar/629", "cpu_time": 400.0, "time_unit": "ns"},
        ]
    }
    rows = summarize(document, "median", "cpu_time")
    assert len(rows) == 2
    jacobian = next(row for row in rows if "Jacobian" in str(row["benchmark"]))
    assert abs(float(jacobian["speedup"]) - 5.0 / 3.0) < 1e-12
    assert not gate_speedups(rows, [(re.compile("Jacobian"), 1.5)])
    assert gate_speedups(rows, [(re.compile("Jacobian"), 2.0)])
    print("self-test: PASS")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Pair Scalar and Simd Google Benchmark JSON rows and report scalar/SIMD speedup.")
    parser.add_argument("input", nargs="?", type=Path, help="Google Benchmark --benchmark_out JSON file")
    parser.add_argument("--aggregate", default="median", help="preferred aggregate_name; falls back to mean then raw rows")
    parser.add_argument("--time-field", choices=("cpu_time", "real_time"), default="cpu_time")
    parser.add_argument("--filter", default="", help="regular expression applied to canonical benchmark names")
    parser.add_argument(
        "--minimum-speedup",
        action="append",
        default=[],
        type=parse_speedup_rule,
        metavar="REGEX=RATIO",
        help="fail if any matched pair is below the ratio; may be repeated",
    )
    parser.add_argument("--json", action="store_true", help="emit machine-readable JSON")
    parser.add_argument("--self-test", action="store_true", help="run built-in pairing, unit conversion, and gate checks")
    return parser


def main(argv: Sequence[str] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.self_test:
        run_self_test()
        return 0
    if args.input is None:
        parser.error("input is required unless --self-test is used")
    try:
        with args.input.open("r", encoding="utf-8") as stream:
            document = json.load(stream)
        rows = summarize(document, args.aggregate, args.time_field, args.filter)
        failures = gate_speedups(rows, args.minimum_speedup)
    except (OSError, json.JSONDecodeError, DataError) as error:
        print(f"input error: {error}", file=sys.stderr)
        return 2

    if args.json:
        print(json.dumps({"passed": not failures, "failures": failures, "benchmarks": rows}, indent=2, sort_keys=True))
    else:
        print_human(rows, failures)
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())
