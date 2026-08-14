# TIMD-Merge 块面积日志

该日志只写入独立 CSV 文件，不混入编码器标准输出。默认开启，默认根目录为：

```text
timd_merge_area_stats
```

每个并行编码进程依据 bitstream 名写独立分片：

```text
timd_merge_area_stats/.shards/<序列>/QP<qp>/<序列>_RAS_<分段>.csv
```

因此现有 `run_class.sh` 和 `Parallel_Coding.py` 无需修改。需要关闭或指定另一个实验目录时，可在
`EncoderAppStatic` 命令中加入：

```text
--TimdMergeAreaStats=0
--TimdMergeAreaStatsRoot=/path/to/experiment/timd_merge_area_stats
```

编码全部完成后，在编码器工作目录执行：

```bash
python3 tools/timd_merge_area_stats/summarize_timd_merge_area.py \
  --input-root timd_merge_area_stats
```

输出结构为：

```text
timd_merge_area_stats/
├── <序列1>/
│   ├── QP22.csv
│   ├── QP27.csv
│   ├── QP32.csv
│   └── QP37.csv
├── <序列2>/
│   ├── QP22.csv
│   ├── QP27.csv
│   ├── QP32.csv
│   └── QP37.csv
└── .shards/                 # 并行进程原始分片
```

汇总器会识别 `Parallel_Coding.py` 相邻分段重复编码的 1 帧，通过 `frame_skip + poc` 去重，重复边界帧归后一个分段。

重点列：

- `derive_calls`：该尺寸进入 TIMD-Merge 推导函数的次数；
- `area_eligible_calls`：通过当前面积限制的次数；
- `intra_slice_derive_calls` / `intra_slice_area_eligible_calls`：只统计真正受最大面积常量约束的 I 帧切片；
- `area_limit_status=EXTENDED_RANGE_HIT`：面积大于 1024、且在 I 帧中通过当前最大面积限制；
- `area_limit_status=REJECTED`：该面积在 I 帧中被最大面积限制拒绝；
- `candidate_available_calls`：找到了有效邻居候选；
- `large_template_calls` / `large_template_applied`：实际使用大模板进行候选排序；
- `final_timd_merge_cus`：最终码流选择该尺寸 TIMD-Merge 的 CU 数。
- `final_intra_slice_timd_merge_cus`：其中发生在 I 帧切片内的最终选中数。

对于当前分支，CSV 每行还会写出编译进程序的三个常量：最大面积、模板面积阈值和大模板尺寸。汇总时如果不同分片混用了不同版本的编码器，脚本会报错。
