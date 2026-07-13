#!/usr/bin/env python3

import argparse
import csv
import re
import sys
from collections import Counter, defaultdict
from pathlib import Path


META_RE = re.compile(r"^META sequence=(\S+) qp=(-?\d+) bitstream=(.*)$")
ENC_RE = re.compile(
    r"^ENC selected_eip_calls=(\d+) mts_calls=(\d+) changed_calls=(\d+)$"
)
DEC_RE = re.compile(
    r"^DEC final_eip=(\d+) final_eip_with_residual=(\d+) "
    r"mts_blocks=(\d+) changed_blocks=(\d+)$"
)
CHANGED_RE = re.compile(
    r"^CHANGED (ENC|DEC) .* old=H(-?\d+)/V(-?\d+) new=H(-?\d+)/V(-?\d+)$"
)
QP_DIR_RE = re.compile(r"^QP(-?\d+)$", re.IGNORECASE)
TRANSFORM_NAMES = {
    0: "DCT2",
    1: "DCT8",
    2: "DST7",
    3: "DCT5",
    4: "DST4",
    5: "DST1",
    6: "IDTR",
    7: "KLT0",
    8: "KLT1",
}


class LogRecord:
    def __init__(self, path):
        self.path = path
        self.sequence = None
        self.qp = None
        self.bitstream = None
        self.enc = None
        self.dec = None
        self.changed_samples = Counter()
        self.errors = []


def fallback_identity(record, root):
    try:
        parts = record.path.relative_to(root).parts
    except ValueError:
        parts = ()

    if record.sequence is None and len(parts) >= 1:
        record.sequence = parts[0]
    if record.qp is None and len(parts) >= 2:
        match = QP_DIR_RE.match(parts[1])
        if match:
            record.qp = int(match.group(1))

    if record.sequence is None:
        record.sequence = "unknown_sequence"
    if record.qp is None:
        record.qp = -1


def transform_pair(horizontal, vertical):
    horizontal = int(horizontal)
    vertical = int(vertical)
    return "H{}({})/V{}({})".format(
        horizontal, TRANSFORM_NAMES.get(horizontal, "UNKNOWN"),
        vertical, TRANSFORM_NAMES.get(vertical, "UNKNOWN"),
    )


def parse_log(path, root):
    record = LogRecord(path)
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError as error:
        record.errors.append("cannot read: {}".format(error))
        fallback_identity(record, root)
        return record

    for line_number, raw_line in enumerate(lines, 1):
        line = raw_line.strip()
        if not line:
            continue

        match = META_RE.match(line)
        if match:
            if record.sequence is not None:
                record.errors.append("duplicate META at line {}".format(line_number))
            record.sequence = match.group(1)
            record.qp = int(match.group(2))
            record.bitstream = match.group(3)
            continue

        match = ENC_RE.match(line)
        if match:
            if record.enc is not None:
                record.errors.append("duplicate ENC at line {}".format(line_number))
            record.enc = {
                "selected": int(match.group(1)),
                "mts": int(match.group(2)),
                "changed": int(match.group(3)),
            }
            continue

        match = DEC_RE.match(line)
        if match:
            if record.dec is not None:
                record.errors.append("duplicate DEC at line {}".format(line_number))
            record.dec = {
                "final_eip": int(match.group(1)),
                "with_residual": int(match.group(2)),
                "mts": int(match.group(3)),
                "changed": int(match.group(4)),
            }
            continue

        match = CHANGED_RE.match(line)
        if match:
            role = match.group(1)
            old_pair = transform_pair(match.group(2), match.group(3))
            new_pair = transform_pair(match.group(4), match.group(5))
            record.changed_samples[(role, old_pair, new_pair)] += 1
            continue

        if line.startswith(("META ", "ENC ", "DEC ", "CHANGED ")):
            record.errors.append("unrecognized line {}: {}".format(line_number, line))

    fallback_identity(record, root)
    return record


def new_group():
    return {
        "logs": 0,
        "complete": 0,
        "enc_logs": 0,
        "dec_logs": 0,
        "enc_selected": 0,
        "enc_mts": 0,
        "enc_changed": 0,
        "dec_final_eip": 0,
        "dec_with_residual": 0,
        "dec_mts": 0,
        "dec_changed": 0,
        "samples": Counter(),
    }


def add_record(group, record):
    group["logs"] += 1
    if record.enc is not None:
        group["enc_logs"] += 1
        group["enc_selected"] += record.enc["selected"]
        group["enc_mts"] += record.enc["mts"]
        group["enc_changed"] += record.enc["changed"]
    if record.dec is not None:
        group["dec_logs"] += 1
        group["dec_final_eip"] += record.dec["final_eip"]
        group["dec_with_residual"] += record.dec["with_residual"]
        group["dec_mts"] += record.dec["mts"]
        group["dec_changed"] += record.dec["changed"]
    if record.enc is not None and record.dec is not None:
        group["complete"] += 1
    group["samples"].update(record.changed_samples)


def add_group(total, group):
    for key in (
        "logs",
        "complete",
        "enc_logs",
        "dec_logs",
        "enc_selected",
        "enc_mts",
        "enc_changed",
        "dec_final_eip",
        "dec_with_residual",
        "dec_mts",
        "dec_changed",
    ):
        total[key] += group[key]
    total["samples"].update(group["samples"])


def percent(numerator, denominator):
    if denominator == 0:
        return "-"
    return "{:.2f}%".format(100.0 * numerator / denominator)


def ratio(numerator, denominator):
    if denominator == 0:
        return "-"
    return "{:.2f}x".format(float(numerator) / denominator)


def verdict(group):
    if group["dec_changed"] > 0:
        return "FINAL_CHANGED"
    if group["enc_changed"] > 0:
        return "ENC_RDO_ONLY"
    if group["dec_final_eip"] == 0:
        return "NO_FINAL_EIP"
    if group["dec_mts"] == 0:
        return "NO_FINAL_MTS"
    return "SAME_FINAL_TR"


def print_table(title, headers, rows, right_aligned):
    widths = [len(header) for header in headers]
    for row in rows:
        for index, value in enumerate(row):
            widths[index] = max(widths[index], len(str(value)))

    print("\n{}".format(title))
    print("  ".join(
        header.rjust(widths[index]) if index in right_aligned else header.ljust(widths[index])
        for index, header in enumerate(headers)
    ))
    print("  ".join("-" * width for width in widths))
    for row in rows:
        print("  ".join(
            str(value).rjust(widths[index]) if index in right_aligned else str(value).ljust(widths[index])
            for index, value in enumerate(row)
        ))


def write_csv(path, grouped_items):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow([
            "sequence",
            "qp",
            "log_files",
            "complete_files",
            "enc_selected_eip_calls",
            "enc_mts_calls",
            "enc_changed_calls",
            "enc_change_percent",
            "dec_final_eip",
            "dec_final_eip_with_residual",
            "dec_mts_blocks",
            "dec_changed_blocks",
            "final_change_percent",
            "encoder_rdo_per_final_mts",
            "status",
        ])
        for (sequence, qp), group in grouped_items:
            writer.writerow([
                sequence,
                qp,
                group["logs"],
                group["complete"],
                group["enc_selected"],
                group["enc_mts"],
                group["enc_changed"],
                percent(group["enc_changed"], group["enc_mts"]),
                group["dec_final_eip"],
                group["dec_with_residual"],
                group["dec_mts"],
                group["dec_changed"],
                percent(group["dec_changed"], group["dec_mts"]),
                ratio(group["enc_mts"], group["dec_mts"]),
                verdict(group),
            ])


def print_evidence(total):
    print("\nEvidence summary")
    print("  Complete RAS logs : {}/{}".format(total["complete"], total["logs"]))
    if total["complete"] != total["logs"]:
        print("  Coverage warning  : conclusions below use only the ENC/DEC summaries that are present.")
    print("  Encoder RDO change: {}/{} ({})".format(
        total["enc_changed"], total["enc_mts"],
        percent(total["enc_changed"], total["enc_mts"]),
    ))
    print("  Final DEC change  : {}/{} ({})".format(
        total["dec_changed"], total["dec_mts"],
        percent(total["dec_changed"], total["dec_mts"]),
    ))

    if total["dec_changed"] == 0 and total["enc_changed"] > 0:
        print("  RD conclusion     : transform choices changed only during encoder RDO; none reached final decoding.")
        print("  Time conclusion   : encoding time may move, while unchanged bitrate/PSNR is expected.")
        print("  Decoder conclusion: this log does not support a decoder speedup caused by the EIP change.")
    elif total["dec_changed"] == 0:
        print("  RD conclusion     : no old/new transform difference was observed in a final decoded block.")
        print("  Time conclusion   : bitrate/PSNR should stay unchanged; measured time changes are likely runtime noise.")
    else:
        print("  RD conclusion     : some final EIP implicit-MTS transform choices changed.")
        print("  Time conclusion   : encoder and decoder can use a different transform-kernel mix.")
        print("  Metric conclusion : unchanged BD-rate means the final changes were too sparse, neutral, or cancelled out.")

    print("  Complexity note   : this modification changes operation mix/constant cost, not asymptotic complexity.")


def print_samples(total):
    if not total["samples"]:
        return

    print("\nSaved transform-change examples (diagnostic sample, not full distribution)")
    for (role, old_pair, new_pair), count in total["samples"].most_common(12):
        print("  {:3s} {} -> {} : {} sample(s)".format(role, old_pair, new_pair, count))


def main():
    parser = argparse.ArgumentParser(
        description="Aggregate EIP implicit-MTS logs by sequence and QP."
    )
    parser.add_argument(
        "log_root", nargs="?", default="EIP_LOG",
        help="EIP_LOG directory (default: ./EIP_LOG)",
    )
    parser.add_argument("--sequence", help="only summarize one exact sequence name")
    parser.add_argument("--qp", type=int, help="only summarize one QP")
    parser.add_argument("--csv", type=Path, help="also write the per-QP summary to CSV")
    args = parser.parse_args()

    root = Path(args.log_root).expanduser().resolve()
    if not root.is_dir():
        print("error: EIP log directory does not exist: {}".format(root), file=sys.stderr)
        return 2

    records = []
    for path in sorted(root.glob("*/QP*/*.log")):
        record = parse_log(path, root)
        if args.sequence is not None and record.sequence != args.sequence:
            continue
        if args.qp is not None and record.qp != args.qp:
            continue
        records.append(record)

    if not records:
        print("error: no matching EIP log files found under {}".format(root), file=sys.stderr)
        return 1

    groups = defaultdict(new_group)
    for record in records:
        add_record(groups[(record.sequence, record.qp)], record)

    grouped_items = sorted(groups.items(), key=lambda item: (item[0][0], item[0][1]))
    total = new_group()
    for _, group in grouped_items:
        add_group(total, group)

    final_rows = []
    encoder_rows = []
    for (sequence, qp), group in grouped_items:
        final_rows.append([
            sequence,
            qp,
            group["logs"],
            "{}/{}".format(group["complete"], group["logs"]),
            group["dec_final_eip"],
            group["dec_with_residual"],
            group["dec_mts"],
            group["dec_changed"],
            percent(group["dec_changed"], group["dec_mts"]),
            verdict(group),
        ])
        encoder_rows.append([
            sequence,
            qp,
            group["enc_selected"],
            group["enc_mts"],
            group["enc_changed"],
            percent(group["enc_changed"], group["enc_mts"]),
            ratio(group["enc_mts"], group["dec_mts"]),
        ])

    print("EIP summary root: {}".format(root))
    print_table(
        "Final bitstream impact (decoder counts are definitive)",
        ["Sequence", "QP", "Logs", "Complete", "FinalEIP", "WithRes", "FinalMTS", "Changed", "Change%", "Status"],
        final_rows,
        {1, 2, 3, 4, 5, 6, 7, 8},
    )
    print_table(
        "Encoder RDO workload (raw candidate-search calls)",
        ["Sequence", "QP", "SelectedEIP", "MTSCalls", "ChangedCalls", "Change%", "RDO/Final"],
        encoder_rows,
        {1, 2, 3, 4, 5, 6},
    )
    print_evidence(total)
    print_samples(total)

    problems = []
    for record in records:
        if record.enc is None:
            problems.append("{}: missing ENC summary".format(record.path))
        if record.dec is None:
            problems.append("{}: missing DEC summary".format(record.path))
        problems.extend("{}: {}".format(record.path, error) for error in record.errors)
    if problems:
        print("\nWarnings")
        for problem in problems[:20]:
            print("  {}".format(problem))
        if len(problems) > 20:
            print("  ... and {} more".format(len(problems) - 20))

    if args.csv is not None:
        csv_path = args.csv.expanduser().resolve()
        write_csv(csv_path, grouped_items)
        print("\nCSV written: {}".format(csv_path))

    return 0


if __name__ == "__main__":
    sys.exit(main())
