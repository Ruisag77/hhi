#!/bin/bash
# -*- coding: gbk -*-
set -e

# 使用说明
usage() {
    echo "Usage: $0 -c class_name [-s seq1,seq2,...] [-q qp1,qp2,...]"
    echo "Example: $0 -c classC"
    echo "         $0 -c classD -s RaceHorses,BasketballPass -q 32,37"
    exit 1
}

# 默认参数
class_name=""
sequences=""
qp_list=""

# 解析参数
while getopts ":c:s:q:" opt; do
    case ${opt} in
        c )
            class_name=$OPTARG
            ;;
        s )
            sequences=$OPTARG
            ;;
        q )
            qp_list=$OPTARG
            ;;
        \? )
            usage
            ;;
    esac
done

if [[ -z "$class_name" ]]; then
    usage
fi

# 所有 class 的序列定义
declare -A sequence_map
sequence_map["classC"]="RaceHorsesC_832x480_30:300 BQMall_832x480_60:600 PartyScene_832x480_50:500 BasketballDrill_832x480_50:500"
sequence_map["classD"]="RaceHorses_416x240_30:300 BQSquare_416x240_60:600 BlowingBubbles_416x240_50:500 BasketballPass_416x240_50:500"
sequence_map["classF"]="SlideShow_1280x720_20:500 SlideEditing_1280x720_30:300 BasketballDrillText_832x480_50:500 ArenaOfValor_1920x1080_60_8bit_420:600"
sequence_map["classB"]="BasketballDrive_1920x1080_50:500 BQTerrace_1920x1080_60:600 MarketPlace_1920x1080_60fps_10bit_420:600 RitualDance_1920x1080_60fps_10bit_420:600 Cactus_1920x1080_50:500"
sequence_map["classE"]="FourPeople_1280x720_60:600 Johnny_1280x720_60:600 KristenAndSara_1280x720_60:600"
sequence_map["classTGM"]="ChineseEditing_1920x1080_60fps_8bit_420pf:600 Console_1920x1080_60fps_8bit_420pf:600 Desktop_1920x1080_60fps_8bit_420pf:600 FlyingGraphics_1920x1080_60fps_8bit_420pf:300"

# 默认 QP 列表
default_qps=(37 32 27 22)

# 解析序列
IFS=' ' read -r -a all_seq_entries <<< "${sequence_map[$class_name]}"

selected_seqs=()
if [[ -z "$sequences" ]]; then
    # 没指定序列，使用全部
    for entry in "${all_seq_entries[@]}"; do
        selected_seqs+=("$entry")
    done
else
    IFS=',' read -r -a user_seq_names <<< "$sequences"
    for entry in "${all_seq_entries[@]}"; do
        seq_base=$(echo "$entry" | cut -d'_' -f1)
        for user_seq in "${user_seq_names[@]}"; do
            if [[ "$seq_base" == "$user_seq" ]]; then
                selected_seqs+=("$entry")
            fi
        done
    done
fi

# 解析 QP
if [[ -z "$qp_list" ]]; then
    qps=("${default_qps[@]}")
else
    IFS=',' read -r -a qps <<< "$qp_list"
fi

# 执行任务
for seq_entry in "${selected_seqs[@]}"; do
    seq=$(echo "$seq_entry" | cut -d':' -f1)
    frames=$(echo "$seq_entry" | cut -d':' -f2)
    name=${seq%%_*}

    for qp in "${qps[@]}"; do
        echo "Launching: $seq  (class: $class_name, QP: $qp)"
        mkdir -p "./output/${name}_RAS_Result_qp${qp}_enc"
        mkdir -p "./output/${name}_RAS_Result_qp${qp}_dec"
        python3 Parallel_Coding.py -C intra -c "$class_name" -i "$seq" -q "$qp" -f "$frames" &
    done
done

# 等待所有任务完成
wait
echo "? All parallel tasks for $class_name completed."
