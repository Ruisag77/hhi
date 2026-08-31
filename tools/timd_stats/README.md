# TIMD final-selection statistics

The encoder records only the final luma CUs visited while writing the real
slice bitstream. Candidate generation and RDO estimation passes are not
counted.

Statistics are enabled by default on this experimental branch. The existing
parallel commands do not need extra arguments. Optional encoder arguments are:

```text
--TimdUsageStats=0
--TimdUsageStatsRoot=/path/to/an/experiment-specific/root
```

A relative root is resolved from the encoder process working directory. Each
parallel encoder writes its own shard:

```text
timd_usage_stats/.shards/<sequence>/QP<qp>/<bitstream-stem>.csv
```

Using an experiment-specific root is recommended so shards left by a previous
run cannot enter a later experiment.

After all parallel encoders have finished, run:

```bash
python3 tools/timd_stats/summarize_timd_usage.py \
  --input-root timd_usage_stats
```

This creates one file for every sequence and QP:

```text
timd_usage_stats/<sequence>/QP22.csv
timd_usage_stats/<sequence>/QP27.csv
timd_usage_stats/<sequence>/QP32.csv
timd_usage_stats/<sequence>/QP37.csv
```

It also creates a combined file containing every sequence and QP:

```text
timd_usage_stats/timd_usage_summary.csv
```

The supplied RAS launcher overlaps adjacent shards by one source frame. The
summarizer detects this from `frame_skip + poc`, removes the overlap, and lets
the later-starting shard own the boundary frame. It reports both the number of
removed overlaps and any overlaps whose counters disagree. Use
`--keep-overlaps` only when the desired population is encoder executions rather
than unique source frames.

Each QP summary contains three rows: `TIMD` (ordinary TIMD), `TIMDSAD`, and
`TIMDMerge`. It reports CU counts, frequency among all final luma CUs,
frequency among final `MODE_INTRA` luma CUs, share within the TIMD family, and
the corresponding luma-sample-weighted coverage.

The summary also separates TIMD-Merge availability from actual syntax cost:

- `timd_merge_available_cus`: final TIMD-family CUs for which a Merge candidate was available;
- `timd_merge_not_selected_cus`: those available candidates that did not win;
- `timd_merge_flag_coded`: final CUs for which the current syntax actually writes a Merge flag;
- `timd_merge_flag_zero`: ordinary-TIMD winners that pay a coded `merge_flag=0`;
- `timd_merge_flag_one`: selected TIMD-Merge CUs;
- `timd_merge_loss_without_flag_cus`: available Merge candidates that lost, but whose flag was omitted (normally because TIMDSAD won).

## TIMD-Merge fractional-bit fields

Shard schema version 3 appends fractional-bit statistics measured immediately before each final
`timd_merge_flag` bin is written. The measurement uses the actual final CABAC context state and VTM's
fractional-bit table. It is suitable for attributing syntax cost, but it is not a byte-exact difference in
the terminated arithmetic bitstream.

Raw shard columns ending in `_frac_bits` are integer fixed-point values with `1 << 15` units per bit. They
are split by flag value and by the two existing TIMD-Merge contexts:

- `ctx0`: luma CU area is at least 64;
- `ctx1`: luma CU area is less than 64.

The summarizer converts these values to decimal `*_estimated_bits` columns and also reports average
estimated bits per bin and estimated bits per final luma CU. Schema-2 shards cannot be mixed with the new
schema, so use a new experiment-specific statistics root when rerunning encodes.
