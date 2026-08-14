#!/usr/bin/env python3
"""Merge parallel TIMD-Merge area-stat shards into one CSV per sequence and QP."""

from __future__ import annotations

import argparse
import csv
import os
import sys
from collections import defaultdict
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, MutableMapping, Sequence, Tuple


SCHEMA_VERSION = 1
INT_FIELDS = (
    "schema_version",
    "qp",
    "layer_id",
    "frame_skip",
    "frames_to_encode",
    "poc",
    "source_frame",
    "width",
    "height",
    "area",
    "configured_max_area",
    "configured_template_threshold",
    "configured_large_template_size",
    "derive_calls",
    "area_eligible_calls",
    "intra_slice_derive_calls",
    "intra_slice_area_eligible_calls",
    "neighbour_available_calls",
    "candidate_available_calls",
    "multi_candidate_calls",
    "template_ranked_calls",
    "large_template_calls",
    "final_luma_cus",
    "final_intra_cus",
    "final_timd_cus",
    "final_timd_merge_cus",
    "final_intra_slice_luma_cus",
    "final_intra_slice_timd_merge_cus",
)
COUNT_FIELDS = (
    "derive_calls",
    "area_eligible_calls",
    "intra_slice_derive_calls",
    "intra_slice_area_eligible_calls",
    "neighbour_available_calls",
    "candidate_available_calls",
    "multi_candidate_calls",
    "template_ranked_calls",
    "large_template_calls",
    "final_luma_cus",
    "final_intra_cus",
    "final_timd_cus",
    "final_timd_merge_cus",
    "final_intra_slice_luma_cus",
    "final_intra_slice_timd_merge_cus",
)
CONFIG_FIELDS = (
    "configured_max_area",
    "configured_template_threshold",
    "configured_large_template_size",
)
OUTPUT_FIELDS = (
    "width",
    "height",
    "area",
    "configured_max_area",
    "configured_template_threshold",
    "configured_large_template_size",
    "area_limit_status",
    "large_template_applied",
    "derive_calls",
    "area_eligible_calls",
    "area_rejected_calls",
    "intra_slice_derive_calls",
    "intra_slice_area_eligible_calls",
    "intra_slice_area_rejected_calls",
    "neighbour_available_calls",
    "candidate_available_calls",
    "multi_candidate_calls",
    "template_ranked_calls",
    "large_template_calls",
    "final_luma_cus",
    "final_intra_cus",
    "final_timd_cus",
    "final_timd_merge_cus",
    "final_intra_slice_luma_cus",
    "final_intra_slice_timd_merge_cus",
    "counted_source_frames",
    "parallel_frame_copies",
    "overlap_frames_removed",
    "conflicting_overlap_frames",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read .shards/<sequence>/QP<qp>/*.csv below the input root, remove the "
            "one-frame overlap used by Parallel_Coding.py, and create "
            "<output-root>/<sequence>/QP<qp>.csv."
        )
    )
    parser.add_argument(
        "--input-root",
        type=Path,
        default=Path("timd_merge_area_stats"),
        help="statistics root containing .shards (default: timd_merge_area_stats)",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=None,
        help="summary root (default: same as --input-root)",
    )
    parser.add_argument(
        "--expected-qps",
        default="22,27,32,37",
        help="comma-separated QPs expected for every sequence (default: 22,27,32,37)",
    )
    return parser.parse_args()


def parse_expected_qps(value: str) -> set[int]:
    try:
        return {int(item.strip()) for item in value.split(",") if item.strip()}
    except ValueError as error:
        raise ValueError(f"invalid --expected-qps value: {value!r}") from error


def read_shards(shard_root: Path) -> List[Dict[str, object]]:
    paths = sorted(shard_root.glob("*/QP*/*.csv"))
    if not paths:
        raise FileNotFoundError(f"no statistic shards found below {shard_root}")

    rows: List[Dict[str, object]] = []
    required = set(INT_FIELDS) | {"sequence", "shard", "bitstream"}
    for path in paths:
        with path.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames is None:
                raise ValueError(f"{path}: missing CSV header")
            missing = required - set(reader.fieldnames)
            if missing:
                raise ValueError(f"{path}: missing fields: {', '.join(sorted(missing))}")

            for line, source in enumerate(reader, start=2):
                row: Dict[str, object] = dict(source)
                try:
                    for field in INT_FIELDS:
                        row[field] = int(source[field])
                except (KeyError, TypeError, ValueError) as error:
                    raise ValueError(f"{path}:{line}: invalid integer field") from error

                if row["schema_version"] != SCHEMA_VERSION:
                    raise ValueError(
                        f"{path}:{line}: schema {row['schema_version']} is not supported; expected {SCHEMA_VERSION}"
                    )
                if row["area"] != row["width"] * row["height"]:
                    raise ValueError(f"{path}:{line}: area does not equal width * height")
                for field in COUNT_FIELDS:
                    if row[field] < 0:
                        raise ValueError(f"{path}:{line}: negative counter {field}")
                if row["area_eligible_calls"] > row["derive_calls"]:
                    raise ValueError(f"{path}:{line}: eligible calls exceed derivation calls")
                if row["intra_slice_derive_calls"] > row["derive_calls"]:
                    raise ValueError(f"{path}:{line}: I-slice calls exceed derivation calls")
                if row["intra_slice_area_eligible_calls"] > row["intra_slice_derive_calls"]:
                    raise ValueError(f"{path}:{line}: I-slice eligible calls exceed I-slice calls")
                if row["large_template_calls"] > row["template_ranked_calls"]:
                    raise ValueError(f"{path}:{line}: large-template calls exceed ranked-template calls")
                row["source_path"] = str(path)
                rows.append(row)
    return rows


def frame_signature(rows: Sequence[Mapping[str, object]]) -> Tuple[Tuple[int, ...], ...]:
    values = []
    for row in sorted(rows, key=lambda item: (int(item["width"]), int(item["height"]))):
        values.append(
            (
                int(row["width"]),
                int(row["height"]),
                *(int(row[field]) for field in CONFIG_FIELDS),
                *(int(row[field]) for field in COUNT_FIELDS),
            )
        )
    return tuple(values)


def select_unique_frames(
    rows: Sequence[Mapping[str, object]],
) -> Tuple[List[Mapping[str, object]], int, int, int, int]:
    variants: Dict[
        Tuple[int, int], Dict[Tuple[int, str], List[Mapping[str, object]]]
    ] = defaultdict(lambda: defaultdict(list))
    for row in rows:
        frame_key = (int(row["layer_id"]), int(row["source_frame"]))
        variant_key = (int(row["frame_skip"]), str(row["shard"]))
        variants[frame_key][variant_key].append(row)

    selected: List[Mapping[str, object]] = []
    parallel_frame_copies = 0
    overlap_frames_removed = 0
    conflicting_overlap_frames = 0
    for frame_variants in variants.values():
        parallel_frame_copies += len(frame_variants)
        overlap_frames_removed += len(frame_variants) - 1
        signatures = {frame_signature(candidate) for candidate in frame_variants.values()}
        if len(signatures) > 1:
            conflicting_overlap_frames += 1

        # Parallel_Coding.py gives the later shard ownership of its first, overlapping source frame.
        selected_key = max(frame_variants)
        selected.extend(frame_variants[selected_key])

    return selected, len(variants), parallel_frame_copies, overlap_frames_removed, conflicting_overlap_frames


def atomic_write_csv(path: Path, rows: Iterable[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=OUTPUT_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    os.replace(temporary, path)


def summarize_group(
    output_root: Path,
    sequence: str,
    qp: int,
    rows: Sequence[Mapping[str, object]],
) -> Path:
    selected, counted_frames, parallel_copies, overlaps_removed, conflicts = select_unique_frames(rows)
    encoder_configs = {tuple(int(row[field]) for field in CONFIG_FIELDS) for row in selected}
    if len(encoder_configs) != 1:
        raise ValueError(f"{sequence} QP{qp} mixes different encoder constants: {sorted(encoder_configs)}")

    totals: Dict[Tuple[int, int], MutableMapping[str, int]] = {}
    encoder_config = next(iter(encoder_configs))

    for row in selected:
        size = (int(row["width"]), int(row["height"]))
        target = totals.setdefault(size, {field: 0 for field in COUNT_FIELDS})
        for field in COUNT_FIELDS:
            target[field] += int(row[field])

    output_rows: List[MutableMapping[str, object]] = []
    for (width, height), count in sorted(totals.items(), key=lambda item: (item[0][0] * item[0][1], item[0])):
        if count["derive_calls"] == 0 and count["final_timd_merge_cus"] == 0:
            continue

        max_area, template_threshold, large_template_size = encoder_config
        area = width * height
        if count["intra_slice_derive_calls"] == 0:
            area_limit_status = "NOT_TESTED_IN_I_SLICE"
        elif area <= 16:
            area_limit_status = "MIN_AREA_REJECTED"
        elif area > max_area and count["intra_slice_area_eligible_calls"] > 0:
            area_limit_status = "LIMIT_BYPASSED"
        elif area > max_area:
            area_limit_status = "REJECTED"
        elif area > template_threshold and count["intra_slice_area_eligible_calls"] > 0:
            area_limit_status = "EXTENDED_RANGE_HIT"
        elif count["intra_slice_area_eligible_calls"] > 0:
            area_limit_status = "BASE_RANGE_HIT"
        else:
            area_limit_status = "NO_ELIGIBLE_HIT"

        output: MutableMapping[str, object] = {
            "width": width,
            "height": height,
            "area": area,
            "configured_max_area": max_area,
            "configured_template_threshold": template_threshold,
            "configured_large_template_size": large_template_size,
            "area_limit_status": area_limit_status,
            "large_template_applied": "YES" if count["large_template_calls"] else "NO",
            **count,
            "area_rejected_calls": count["derive_calls"] - count["area_eligible_calls"],
            "intra_slice_area_rejected_calls": (
                count["intra_slice_derive_calls"] - count["intra_slice_area_eligible_calls"]
            ),
            "counted_source_frames": counted_frames,
            "parallel_frame_copies": parallel_copies,
            "overlap_frames_removed": overlaps_removed,
            "conflicting_overlap_frames": conflicts,
        }
        output_rows.append(output)

    output_path = output_root / sequence / f"QP{qp}.csv"
    atomic_write_csv(output_path, output_rows)

    extended = [row for row in output_rows if int(row["area"]) > int(row["configured_template_threshold"])]
    if extended:
        detail = "; ".join(
            f"{row['width']}x{row['height']}: status={row['area_limit_status']}, "
            f"I-eligible={row['intra_slice_area_eligible_calls']}, large_tpl={row['large_template_calls']}, "
            f"I-final={row['final_intra_slice_timd_merge_cus']}"
            for row in extended
        )
    else:
        detail = "no block above the template threshold entered TIMD-Merge"
    print(
        f"{sequence:24s} QP{qp:>2}: frames={counted_frames}, overlaps_removed={overlaps_removed}, "
        f"conflicts={conflicts}; {detail}"
    )
    return output_path


def main() -> int:
    args = parse_args()
    output_root = args.output_root if args.output_root is not None else args.input_root
    try:
        rows = read_shards(args.input_root / ".shards")
        expected_qps = parse_expected_qps(args.expected_qps)
    except (FileNotFoundError, OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    groups: Dict[Tuple[str, int], List[Mapping[str, object]]] = defaultdict(list)
    for row in rows:
        groups[(str(row["sequence"]), int(row["qp"]))].append(row)

    written: List[Path] = []
    try:
        for (sequence, qp), group_rows in sorted(groups.items()):
            written.append(summarize_group(output_root, sequence, qp, group_rows))
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    if expected_qps:
        for sequence in sorted({sequence for sequence, _ in groups}):
            present = {qp for candidate, qp in groups if candidate == sequence}
            missing = sorted(expected_qps - present)
            if missing:
                print(
                    f"warning: {sequence} is missing expected QPs: {','.join(map(str, missing))}",
                    file=sys.stderr,
                )

    print(f"wrote {len(written)} QP summaries below {output_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
