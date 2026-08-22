#!/usr/bin/env python3
"""Gate collision-debug MPC TSV output against a committed baseline."""

import argparse
import csv
import io
import json
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, TextIO, Tuple


ROLLOUT_METRICS = ("total_cost_signed_diff", "total_cost_abs_diff")
LINK_PAIR_METRIC = "max_link_pair_min_abs_diff"
ALIGNMENT_METRICS = ("observation_time",)
REQUIRED_COLUMNS = (
    ("status", "mpc_run")
    + ALIGNMENT_METRICS
    + ROLLOUT_METRICS
    + (LINK_PAIR_METRIC,)
)


class DataError(RuntimeError):
    pass


@dataclass(frozen=True)
class Sample:
    run: int
    values: Dict[str, float]


@dataclass(frozen=True)
class Dataset:
    label: str
    samples: Dict[int, Sample]
    non_ok_rows: Tuple[str, ...]


@dataclass(frozen=True)
class GateConfig:
    max_link_pair_distance: float = 0.004
    persistent_increase_fraction: float = 0.95
    absolute_tolerance: float = 1e-12
    relative_tolerance: float = 1e-9
    minimum_common_runs: int = 20
    skip_first: int = 0
    allow_non_ok: bool = False


def _finite_float(value: str, label: str, line_number: int, column: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as error:
        raise DataError(f"{label}:{line_number}: invalid {column!r}: {value!r}") from error
    if not math.isfinite(parsed):
        raise DataError(f"{label}:{line_number}: non-finite {column!r}: {value!r}")
    return parsed


def read_dataset(stream: TextIO, label: str) -> Dataset:
    reader = csv.DictReader(stream, delimiter="\t")
    if reader.fieldnames is None:
        raise DataError(f"{label}: missing TSV header")
    missing = [column for column in REQUIRED_COLUMNS if column not in reader.fieldnames]
    if missing:
        raise DataError(f"{label}: missing required columns: {', '.join(missing)}")

    samples: Dict[int, Sample] = {}
    non_ok_rows: List[str] = []
    for line_number, row in enumerate(reader, start=2):
        status = (row.get("status") or "").strip()
        if status != "ok":
            non_ok_rows.append(f"line {line_number}: {status or '<empty>'}")
            continue
        try:
            run = int(row["mpc_run"])
        except (TypeError, ValueError) as error:
            raise DataError(f"{label}:{line_number}: invalid mpc_run: {row.get('mpc_run')!r}") from error
        if run in samples:
            raise DataError(f"{label}:{line_number}: duplicate mpc_run {run}")
        values = {
            metric: _finite_float(row[metric], label, line_number, metric)
            for metric in ALIGNMENT_METRICS + ROLLOUT_METRICS + (LINK_PAIR_METRIC,)
        }
        samples[run] = Sample(run=run, values=values)

    if not samples:
        raise DataError(f"{label}: no rows with status=ok")
    return Dataset(label=label, samples=samples, non_ok_rows=tuple(non_ok_rows))


def read_dataset_file(path: Path) -> Dataset:
    try:
        with path.open("r", encoding="utf-8", newline="") as stream:
            return read_dataset(stream, str(path))
    except OSError as error:
        raise DataError(f"cannot read {path}: {error}") from error


def _mean(values: Iterable[float]) -> float:
    materialized = list(values)
    return statistics.fmean(materialized) if materialized else math.nan


def evaluate_gate(baseline: Dataset, candidate: Dataset, config: GateConfig) -> Dict[str, object]:
    failures: List[str] = []
    if not config.allow_non_ok:
        if baseline.non_ok_rows:
            failures.append(f"baseline contains {len(baseline.non_ok_rows)} non-ok row(s)")
        if candidate.non_ok_rows:
            failures.append(f"candidate contains {len(candidate.non_ok_rows)} non-ok row(s)")

    common_runs = sorted(set(baseline.samples).intersection(candidate.samples))
    common_runs = common_runs[config.skip_first :]
    if len(common_runs) < config.minimum_common_runs:
        failures.append(
            f"only {len(common_runs)} common run(s) after skipping {config.skip_first}; "
            f"need at least {config.minimum_common_runs}"
        )

    alignment_summary: Dict[str, object] = {}
    for metric in ALIGNMENT_METRICS:
        baseline_values = [baseline.samples[run].values[metric] for run in common_runs]
        candidate_values = [candidate.samples[run].values[metric] for run in common_runs]
        deltas = [
            candidate_value - baseline_value
            for baseline_value, candidate_value in zip(baseline_values, candidate_values)
        ]
        mismatched = [
            abs(delta)
            > config.absolute_tolerance + config.relative_tolerance * abs(baseline_value)
            for baseline_value, delta in zip(baseline_values, deltas)
        ]
        mismatch_count = sum(mismatched)
        if mismatch_count:
            failures.append(
                f"{metric} differs in {mismatch_count}/{len(common_runs)} aligned runs"
            )
        alignment_summary[metric] = {
            "mismatch_count": mismatch_count,
            "max_abs_delta": max((abs(delta) for delta in deltas), default=math.nan),
        }

    rollout_summary: Dict[str, object] = {}
    for metric in ROLLOUT_METRICS:
        baseline_values = [baseline.samples[run].values[metric] for run in common_runs]
        candidate_values = [candidate.samples[run].values[metric] for run in common_runs]
        deltas = [candidate_value - baseline_value for baseline_value, candidate_value in zip(baseline_values, candidate_values)]
        increased = [
            delta > config.absolute_tolerance + config.relative_tolerance * abs(baseline_value)
            for baseline_value, delta in zip(baseline_values, deltas)
        ]
        increase_count = sum(increased)
        increase_fraction = increase_count / len(common_runs) if common_runs else math.nan
        persistent = bool(common_runs) and increase_fraction >= config.persistent_increase_fraction
        if persistent:
            failures.append(
                f"{metric} increased in {increase_count}/{len(common_runs)} aligned runs "
                f"({increase_fraction:.1%})"
            )
        rollout_summary[metric] = {
            "baseline_mean": _mean(baseline_values),
            "candidate_mean": _mean(candidate_values),
            "delta_mean": _mean(deltas),
            "delta_median": statistics.median(deltas) if deltas else math.nan,
            "delta_max": max(deltas) if deltas else math.nan,
            "increase_count": increase_count,
            "increase_fraction": increase_fraction,
            "persistent_increase": persistent,
        }

    candidate_runs = sorted(candidate.samples)[config.skip_first :]
    baseline_runs = sorted(baseline.samples)[config.skip_first :]
    candidate_link_values = [candidate.samples[run].values[LINK_PAIR_METRIC] for run in candidate_runs]
    baseline_link_values = [baseline.samples[run].values[LINK_PAIR_METRIC] for run in baseline_runs]
    candidate_link_max = max(candidate_link_values) if candidate_link_values else math.nan
    baseline_link_max = max(baseline_link_values) if baseline_link_values else math.nan
    link_pair_passed = bool(candidate_link_values) and candidate_link_max <= config.max_link_pair_distance
    if not candidate_link_values:
        failures.append("candidate has no link-pair samples after warmup skipping")
    elif not link_pair_passed:
        failures.append(
            f"{LINK_PAIR_METRIC}={candidate_link_max:.9g} m exceeds "
            f"{config.max_link_pair_distance:.9g} m"
        )

    return {
        "passed": not failures,
        "failures": failures,
        "baseline": {
            "label": baseline.label,
            "ok_rows": len(baseline.samples),
            "non_ok_rows": len(baseline.non_ok_rows),
        },
        "candidate": {
            "label": candidate.label,
            "ok_rows": len(candidate.samples),
            "non_ok_rows": len(candidate.non_ok_rows),
        },
        "common_runs": len(common_runs),
        "skip_first": config.skip_first,
        "alignment": alignment_summary,
        "rollout": rollout_summary,
        "link_pair": {
            "metric": LINK_PAIR_METRIC,
            "baseline_max_m": baseline_link_max,
            "candidate_max_m": candidate_link_max,
            "limit_m": config.max_link_pair_distance,
            "passed": link_pair_passed,
        },
    }


def print_human(summary: Dict[str, object]) -> None:
    print("MPC collision regression gate:", "PASS" if summary["passed"] else "FAIL")
    print(f"aligned runs: {summary['common_runs']} (skip_first={summary['skip_first']})")
    alignment = summary["alignment"]
    assert isinstance(alignment, dict)
    for metric in ALIGNMENT_METRICS:
        item = alignment[metric]
        assert isinstance(item, dict)
        print(
            f"{metric}: max_abs_delta={item['max_abs_delta']:.9g}, "
            f"mismatched={item['mismatch_count']}/{summary['common_runs']}"
        )
    rollout = summary["rollout"]
    assert isinstance(rollout, dict)
    for metric in ROLLOUT_METRICS:
        item = rollout[metric]
        assert isinstance(item, dict)
        print(
            f"{metric}: baseline_mean={item['baseline_mean']:.9g}, "
            f"candidate_mean={item['candidate_mean']:.9g}, delta_mean={item['delta_mean']:.9g}, "
            f"increased={item['increase_count']}/{summary['common_runs']} "
            f"({item['increase_fraction']:.1%})"
        )
    link_pair = summary["link_pair"]
    assert isinstance(link_pair, dict)
    print(
        f"{LINK_PAIR_METRIC}: baseline_max={link_pair['baseline_max_m']:.9g} m, "
        f"candidate_max={link_pair['candidate_max_m']:.9g} m, limit={link_pair['limit_m']:.9g} m"
    )
    failures = summary["failures"]
    assert isinstance(failures, list)
    for failure in failures:
        print(f"failure: {failure}")


def _fixture(
    rows: Sequence[Tuple[int, float, float, float]],
    label: str,
    observation_offset: float = 0.0,
) -> Dataset:
    header = (
        "status\tmpc_run\tobservation_time\ttotal_cost_signed_diff\t"
        "total_cost_abs_diff\tmax_link_pair_min_abs_diff\n"
    )
    body = "".join(
        f"ok\t{run}\t{0.01 * run + observation_offset}\t{signed}\t{absolute}\t{link}\n"
        for run, signed, absolute, link in rows
    )
    return read_dataset(io.StringIO(header + body), label)


def run_self_test() -> None:
    baseline = _fixture([(1, 1.0, 1.0, 0.003), (2, 2.0, 2.0, 0.0035), (3, 3.0, 3.0, 0.0038)], "baseline")
    candidate_ok = _fixture([(1, 1.0, 1.0, 0.003), (2, 1.9, 1.9, 0.0036), (3, 3.1, 2.9, 0.0039)], "candidate")
    config = GateConfig(minimum_common_runs=3)
    assert evaluate_gate(baseline, candidate_ok, config)["passed"]

    candidate_bias = _fixture([(1, 1.1, 1.1, 0.003), (2, 2.1, 2.1, 0.003), (3, 3.1, 3.1, 0.003)], "bias")
    assert not evaluate_gate(baseline, candidate_bias, config)["passed"]

    candidate_distance = _fixture([(1, 1.0, 1.0, 0.0041), (2, 2.0, 2.0, 0.003), (3, 3.0, 3.0, 0.003)], "distance")
    assert not evaluate_gate(baseline, candidate_distance, config)["passed"]
    candidate_misaligned = _fixture(
        [(1, 1.0, 1.0, 0.003), (2, 2.0, 2.0, 0.003), (3, 3.0, 3.0, 0.003)],
        "misaligned",
        observation_offset=0.001,
    )
    assert not evaluate_gate(baseline, candidate_misaligned, config)["passed"]
    print("self-test: PASS")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Compare debug MPC TSV output by mpc_run. A rollout metric fails when its candidate value is above baseline, "
            "outside tolerance, in a persistent fraction of aligned runs. The candidate link-pair distance uses a hard limit."
        )
    )
    parser.add_argument("baseline", nargs="?", type=Path, help="TSV from committed code")
    parser.add_argument("candidate", nargs="?", type=Path, help="TSV from the candidate build")
    parser.add_argument("--max-link-pair-distance", type=float, default=0.004, metavar="METERS")
    parser.add_argument("--persistent-increase-fraction", type=float, default=0.95, metavar="FRACTION")
    parser.add_argument("--absolute-tolerance", type=float, default=1e-12)
    parser.add_argument("--relative-tolerance", type=float, default=1e-9)
    parser.add_argument("--minimum-common-runs", type=int, default=20)
    parser.add_argument("--skip-first", type=int, default=0, help="ignore this many initial runs as warmup")
    parser.add_argument("--allow-non-ok", action="store_true", help="do not fail solely because a TSV contains non-ok rows")
    parser.add_argument("--json", action="store_true", help="emit machine-readable JSON")
    parser.add_argument("--self-test", action="store_true", help="run built-in parser and gate checks")
    return parser


def main(argv: Sequence[str] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.self_test:
        run_self_test()
        return 0
    if args.baseline is None or args.candidate is None:
        parser.error("baseline and candidate are required unless --self-test is used")
    if not 0.0 < args.persistent_increase_fraction <= 1.0:
        parser.error("--persistent-increase-fraction must be in (0, 1]")
    if args.minimum_common_runs <= 0 or args.skip_first < 0:
        parser.error("--minimum-common-runs must be positive and --skip-first must be nonnegative")
    if args.max_link_pair_distance < 0.0 or args.absolute_tolerance < 0.0 or args.relative_tolerance < 0.0:
        parser.error("distance and numeric tolerances must be nonnegative")

    config = GateConfig(
        max_link_pair_distance=args.max_link_pair_distance,
        persistent_increase_fraction=args.persistent_increase_fraction,
        absolute_tolerance=args.absolute_tolerance,
        relative_tolerance=args.relative_tolerance,
        minimum_common_runs=args.minimum_common_runs,
        skip_first=args.skip_first,
        allow_non_ok=args.allow_non_ok,
    )
    try:
        summary = evaluate_gate(read_dataset_file(args.baseline), read_dataset_file(args.candidate), config)
    except DataError as error:
        print(f"input error: {error}", file=sys.stderr)
        return 2

    if args.json:
        print(json.dumps(summary, indent=2, sort_keys=True))
    else:
        print_human(summary)
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
