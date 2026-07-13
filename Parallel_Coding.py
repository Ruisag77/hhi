# -*- coding: gbk -*-

import os
import hashlib
import argparse
from functools import partial
import sys
import time
import subprocess
import psutil
import getpass

enc_cmd_list = []
dec_cmd_list = []
MAX_RUNNING_PROCESSES = 800

def md5sum(file_name):
    with open(file_name, mode='rb') as f:
        d = hashlib.md5()
        for buf in iter(partial(f.read, 128), b''):
            d.update(buf)
        return d.hexdigest()

def compare_md5(file_name1, file_name2):
    m1 = md5sum(file_name1)
    m2 = md5sum(file_name2)
    print(f'{file_name1}:{m1} |f {file_name2}: {m2}')
    return m1 == m2

def getIntraperiod(fps):
    frame_rate = fps[:2]
    limits = {
        '20': '32',
        '24': '32',
        '30': '32',
        '50': '64',
        '60': '64'
    }
    return limits.get(frame_rate, None)

def createAIEncodeCommands(seqclass_name, seq, qp, frame_count):
    seq_name = seq.split("_")[0]
    intra_period = getIntraperiod(seq.split("_")[2])
    ras_nums = int(frame_count) // int(intra_period)
    rest_num = int(frame_count) % int(intra_period)

    if seqclass_name == 'classC' and seq_name == 'RaceHorses':
        seq_name = 'RaceHorsesC'

    config_name = f"{seqclass_name}.cfg"
    if seqclass_name == 'TGM':
        config_name = 'classF.cfg'

    cfg_path = f"/data/rui_data/RPR-BVI-GenerateTrainData/hhi/nextcode/cfg/encoder_intra_nextRt500.cfg"
    # classcfg_path = f"/data/rui_data/RPR-BVI-GenerateTrainData/ECM-ECM-10.0/ECM-ECM-10.0/cfg/per-class/{config_name}"
    seqcfg_path = f"/data/rui_data/RPR-BVI-GenerateTrainData/hhi/nextcode/cfg/per-sequence/{seq_name}.cfg"
    seq_path = f"/data/testsequence/{seqclass_name}/{seq}.yuv"

    for ras in range(ras_nums):
        cmd = f'./EncoderAppStatic -c {cfg_path}  -c {seqcfg_path} -i {seq_path} ' \
              f'-b ./output/{seq_name}_RAS_Result_qp{qp}_enc/{seq_name}_RAS_{ras}.bin ' \
              f'-o ./output/{seq_name}_RAS_Result_qp{qp}_enc/{seq_name}_RAS_{ras}_rec.yuv ' \
              f'-q {qp} -f {int(intra_period) + 1} -fs {int(intra_period) * ras} ' \
              f'--PrintHexPSNR=1 > ./output/{seq_name}_RAS_Result_qp{qp}_enc/{seq_name}_RAS_{ras}.log\n'
        enc_cmd_list.append(cmd)

    if rest_num:
        cmd = f'./EncoderAppStatic -c {cfg_path}  -c {seqcfg_path} -i {seq_path} ' \
              f'-b ./output/{seq_name}_RAS_Result_qp{qp}_enc/{seq_name}_RAS_{ras_nums}.bin ' \
              f'-o ./output/{seq_name}_RAS_Result_qp{qp}_enc/{seq_name}_RAS_{ras_nums}_rec.yuv ' \
              f'-q {qp} -f {rest_num} -fs {int(intra_period) * ras_nums} ' \
              f'--PrintHexPSNR=1 > ./output/{seq_name}_RAS_Result_qp{qp}_enc/{seq_name}_RAS_{ras_nums}.log\n'
        enc_cmd_list.append(cmd)

def createRAEncodeCommands(seqclass_name, seq, qp, frame_count):
    seq_name = seq.split("_")[0]
    intra_period = getIntraperiod(seq.split("_")[2])
    ras_nums = int(frame_count) // int(intra_period)
    rest_num = int(frame_count) % int(intra_period)

    config_name = f'{seqclass_name}_randomaccess.cfg' if seqclass_name != 'classF' else f'{seqclass_name}.cfg'
    if seqclass_name == 'classC' and seq_name == 'RaceHorses':
        seq_name = 'RaceHorsesC'
    if seqclass_name == 'TGM':
        config_name = 'classF.cfg'

    cfg_path = f"/data/rui_data/ECM-ECM-18.0/cfg/encoder_randomaccess_ecm.cfg"
    seqcfg_path = f"/data/rui_data/ECM-ECM-18.0/cfg/per-sequence/{seq_name}.cfg"
    classcfg_path = f"/data/rui_data/ECM-ECM-18.0/cfg/per-class/{config_name}"
    seq_path = f"/data/testsequence/{seqclass_name}/{seq}.yuv"

    for ras in range(ras_nums):
        cmd = f'./EncoderAppStatic -c {cfg_path}  -c {seqcfg_path} -i {seq_path} ' \
              f'-b ./output/{seq_name}_RAS_Result_qp{qp}_enc/{seq_name}_RAS_{ras}.bin ' \
              f'-o ./output/{seq_name}_RAS_Result_qp{qp}_enc/{seq_name}_RAS_{ras}_rec.yuv ' \
              f'-q {qp} -f {int(intra_period) + 1} -fs {int(intra_period) * ras} ' \
              f'--IntraPeriod={intra_period} --PrintHexPSNR=1 > ./output/{seq_name}_RAS_Result_qp{qp}_enc/{seq_name}_RAS_{ras}.log\n'
        enc_cmd_list.append(cmd)

    if rest_num:
        cmd = f'./EncoderAppStatic -c {cfg_path}  -c {seqcfg_path} -i {seq_path} ' \
              f'-b ./output/{seq_name}_RAS_Result_qp{qp}_enc/{seq_name}_RAS_{ras_nums}.bin ' \
              f'-o ./output/{seq_name}_RAS_Result_qp{qp}_enc/{seq_name}_RAS_{ras_nums}_rec.yuv ' \
              f'-q {qp} -f {rest_num} -fs {int(intra_period) * ras_nums} ' \
              f'--IntraPeriod={intra_period} --PrintHexPSNR=1 > ./output/{seq_name}_RAS_Result_qp{qp}_enc/{seq_name}_RAS_{ras_nums}.log\n'
        enc_cmd_list.append(cmd)

def createDecodeCommands(seqclass_name, seq, qp, frame_count):
    seq_name = seq.split("_")[0]
    intra_period = getIntraperiod(seq.split("_")[2])
    ras_nums = int(frame_count) // int(intra_period)
    rest_num = int(frame_count) % int(intra_period)

    if seqclass_name == 'classC' and seq_name == 'RaceHorses':
        seq_name = 'RaceHorsesC'

    for ras in range(ras_nums):
        cmd = f'./DecoderAppStatic -b ./output/{seq_name}_RAS_Result_qp{qp}_enc/{seq_name}_RAS_{ras}.bin ' \
              f'-o ./output/{seq_name}_RAS_Result_qp{qp}_dec/{seq_name}_RAS_{ras}_dec.yuv ' \
              f'> ./output/{seq_name}_RAS_Result_qp{qp}_dec/{seq_name}_RAS_{ras}_dec.log\n'
        dec_cmd_list.append(cmd)

    if rest_num:
        cmd = f'./DecoderAppStatic -b ./output/{seq_name}_RAS_Result_qp{qp}_enc/{seq_name}_RAS_{ras_nums}.bin ' \
              f'-o ./output/{seq_name}_RAS_Result_qp{qp}_dec/{seq_name}_RAS_{ras_nums}_dec.yuv ' \
              f'> ./output/{seq_name}_RAS_Result_qp{qp}_dec/{seq_name}_RAS_{ras_nums}_dec.log\n'
        dec_cmd_list.append(cmd)

def check_consistency(seqclass_name, seq, qp, frame_count):
    seq_name = seq.split("_")[0]
    intra_period = getIntraperiod(seq.split("_")[2])
    ras_nums = int(frame_count) // int(intra_period)
    rest_num = int(frame_count) % int(intra_period)

    if seqclass_name == 'classC' and seq_name == 'RaceHorses':
        seq_name = 'RaceHorsesC'

    for ras in range(ras_nums):
        encoded_yuv_path = f"./output/{seq_name}_RAS_Result_qp{qp}_enc/{seq_name}_RAS_{ras}_rec.yuv"
        decoded_yuv_path = f"./output/{seq_name}_RAS_Result_qp{qp}_dec/{seq_name}_RAS_{ras}_dec.yuv"
        if compare_md5(encoded_yuv_path, decoded_yuv_path):
            os.remove(encoded_yuv_path)
            os.remove(decoded_yuv_path)
        else:
            print(f"[error] codec consistency error")
            sys.exit(2)

    if rest_num:
        encoded_yuv_path = f"./output/{seq_name}_RAS_Result_qp{qp}_enc/{seq_name}_RAS_{ras_nums}_rec.yuv"
        decoded_yuv_path = f"./output/{seq_name}_RAS_Result_qp{qp}_dec/{seq_name}_RAS_{ras_nums}_dec.yuv"
        if compare_md5(encoded_yuv_path, decoded_yuv_path):
            os.remove(encoded_yuv_path)
            os.remove(decoded_yuv_path)
        else:
            print(f"[error] codec consistency error")
            sys.exit(2)

def count_running_processes_by_name(name_substring, username=None):
    if username is None:
        username = getpass.getuser()
    count = 0
    for proc in psutil.process_iter(attrs=['cmdline', 'username']):
        try:
            if proc.info['username'] == username:
                cmdline = proc.info['cmdline']
                if isinstance(cmdline, list) and any(name_substring.lower() in part.lower() for part in cmdline):
                    count += 1
        except (psutil.NoSuchProcess, psutil.AccessDenied, KeyError):
            continue
    return count

def run_commands_with_limit(cmd_list, action_name, process_keyword, username=None):
    running_procs = []
    for idx, cmd in enumerate(cmd_list):
        while count_running_processes_by_name(process_keyword, username=username) >= MAX_RUNNING_PROCESSES:
            time.sleep(5)
        print(f'---Start---the {action_name}_{idx} process')
        proc = subprocess.Popen(cmd, shell=True)
        running_procs.append(proc)

    for proc in running_procs:
        proc.wait()
    print(f'***All {action_name} processes finished***')

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Encoding the specified sequence in parallel.")
    parser.add_argument('--SeqClass', '-c', type=str, help="class name")
    parser.add_argument('--CFG', '-C', type=str, default="intra", help="configuration name")
    parser.add_argument('--SEQ', '-i', type=str, help="seq name")
    parser.add_argument('--QP', '-q', type=str,  default='37', help="qp value")
    parser.add_argument('--FrameCount', '-f', type=str, help="frame count")

    args = parser.parse_args()
    seqClass = args.SeqClass
    cfgName = args.CFG
    seq = args.SEQ
    qp = args.QP
    frameCount = args.FrameCount

    if cfgName == 'intra':
        createAIEncodeCommands(seqClass, seq, qp, frameCount)
    else:
        createRAEncodeCommands(seqClass, seq, qp, frameCount)

    createDecodeCommands(seqClass, seq, qp, frameCount)

    run_commands_with_limit(enc_cmd_list, "Encoding", "EncoderApp", username="rui_data")
    run_commands_with_limit(dec_cmd_list, "Decoding", "DecoderApp", username="rui_data")

    check_consistency(seqClass, seq, qp, frameCount)
    print("consistency check passed!")
