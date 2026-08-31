#!/usr/bin/env python3

import csv
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
import summarize_timd_usage as summary


class SummarizeTimdUsageTest(unittest.TestCase):
    def test_schema3_fractional_bits_and_combined_output(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "stats"
            shard = root / ".shards" / "TestSequence" / "QP22" / "shard.csv"
            shard.parent.mkdir(parents=True)

            row = {field: 0 for field in summary.SUM_FIELDS}
            row.update(
                {
                    "slices": 1,
                    "ctus": 1,
                    "luma_cus": 10,
                    "intra_pred_luma_cus": 9,
                    "timd_total": 5,
                    "timd_normal": 3,
                    "timdsad": 1,
                    "timd_merge": 1,
                    "timd_merge_available": 4,
                    "timd_merge_not_selected": 3,
                    "timd_merge_flag_coded": 4,
                    "timd_merge_flag_zero": 3,
                    "timd_merge_flag_one": 1,
                    "luma_samples": 640,
                    "intra_pred_luma_samples": 576,
                    "timd_samples": 320,
                    "timd_normal_samples": 192,
                    "timdsad_samples": 64,
                    "timd_merge_samples": 64,
                    "timd_merge_available_samples": 256,
                    "timd_merge_not_selected_samples": 192,
                    "timd_merge_flag_coded_samples": 256,
                    "timd_merge_flag_zero_samples": 192,
                    "timd_merge_flag_one_samples": 64,
                    "timd_merge_flag_frac_bits": 4 * summary.FRAC_BITS_SCALE,
                    "timd_merge_flag_zero_frac_bits": summary.FRAC_BITS_SCALE + summary.FRAC_BITS_SCALE // 2,
                    "timd_merge_flag_one_frac_bits": 2 * summary.FRAC_BITS_SCALE + summary.FRAC_BITS_SCALE // 2,
                    "timd_merge_flag_ctx0_coded": 3,
                    "timd_merge_flag_ctx0_zero": 2,
                    "timd_merge_flag_ctx0_one": 1,
                    "timd_merge_flag_ctx0_frac_bits": 3 * summary.FRAC_BITS_SCALE + summary.FRAC_BITS_SCALE // 2,
                    "timd_merge_flag_ctx0_zero_frac_bits": summary.FRAC_BITS_SCALE,
                    "timd_merge_flag_ctx0_one_frac_bits": 2 * summary.FRAC_BITS_SCALE
                    + summary.FRAC_BITS_SCALE // 2,
                    "timd_merge_flag_ctx1_coded": 1,
                    "timd_merge_flag_ctx1_zero": 1,
                    "timd_merge_flag_ctx1_frac_bits": summary.FRAC_BITS_SCALE // 2,
                    "timd_merge_flag_ctx1_zero_frac_bits": summary.FRAC_BITS_SCALE // 2,
                }
            )

            identity = {
                "schema_version": 3,
                "sequence": "TestSequence",
                "qp": 22,
                "layer_id": 0,
                "shard": "shard",
                "bitstream": "test.bin",
                "frame_skip": 0,
                "frames_to_encode": 1,
                "poc": 0,
                "source_frame": 0,
            }
            with shard.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=tuple(identity) + summary.SUM_FIELDS)
                writer.writeheader()
                writer.writerow({**identity, **row})

            with mock.patch.object(
                sys,
                "argv",
                ["summarize_timd_usage.py", "--input-root", str(root), "--expected-qps", "22"],
            ):
                self.assertEqual(summary.main(), 0)

            with (root / "timd_usage_summary.csv").open("r", encoding="utf-8", newline="") as stream:
                output = list(csv.DictReader(stream))

            self.assertEqual(len(output), 3)
            self.assertEqual(output[0]["timd_merge_flag_estimated_bits"], "4.0000000000")
            self.assertEqual(output[0]["timd_merge_flag_zero_avg_estimated_bits_per_bin"], "0.5000000000")
            self.assertEqual(output[0]["timd_merge_flag_one_avg_estimated_bits_per_bin"], "2.5000000000")
            self.assertEqual(output[0]["timd_merge_flag_ctx1_zero_estimated_bits"], "0.5000000000")


if __name__ == "__main__":
    unittest.main()
