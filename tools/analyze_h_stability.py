"""只读复核 H 稳定目标；从 CRC 原帧重解码，不采用日志中的派生角度。

50 Hz 遥测只能验收已采样的表现，不能证明两帧之间没有瞬态。
没有明确的外部松手时刻证据时，保持时长只称为连续健康 H 观测时长。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from host.adc_status import ADC_CONDITIONING_FIRMWARE, measurement_ready
from host.core import _decode_frame
from host.tuning import H_LQI_PROFILES
from tools.analyze_control_trace import rebuild

TARGET = dict(hold_s=60.0, tail_s=20.0, arm_peak_to_peak_deg=5.0,
              advanced_arm_peak_to_peak_deg=2.0, theta_zero_rms_deg=2.0,
              saturated_samples=0)
RAD_Q10_TO_DEG = 180 / (math.pi * 1024)


def _metrics(rows):
    """保留直流偏置；RMS 围绕标定零度，绝不以去均值标准差替代。"""
    if not rows:
        return None
    theta = [r['theta_q10'] * RAD_Q10_TO_DEG for r in rows]
    arm = [r['arm_q10'] * RAD_Q10_TO_DEG for r in rows]
    command = [r['command_permille'] for r in rows]
    times = [r['_elapsed_s'] for r in rows]
    times_ms = [r['_elapsed_ms'] for r in rows]
    tmean, amean = statistics.fmean(times), statistics.fmean(arm)
    denominator = sum((t-tmean)**2 for t in times)
    slope = (sum((t-tmean)*(a-amean) for t, a in zip(times, arm)) / denominator
             if denominator else None)
    first = [a for t, a in zip(times_ms, arm) if t <= times_ms[0]+1000]
    last = [a for t, a in zip(times_ms, arm) if t >= times_ms[-1]-1000]
    return dict(samples=len(rows), duration_s=(times_ms[-1]-times_ms[0])/1000,
                theta_zero_rms_deg=math.sqrt(statistics.fmean(v*v for v in theta)),
                theta_mean_deg=statistics.fmean(theta), theta_stddev_deg=statistics.pstdev(theta),
                theta_abs_max_deg=max(map(abs, theta)), arm_min_deg=min(arm), arm_max_deg=max(arm),
                arm_peak_to_peak_deg=max(arm)-min(arm), arm_mean_deg=amean,
                arm_drift_deg=statistics.fmean(last)-statistics.fmean(first),
                arm_linear_drift_deg_s=slope,
                command_rms_permille=math.sqrt(statistics.fmean(v*v for v in command)),
                command_abs_max_permille=max(map(abs, command)),
                saturated_samples=sum(abs(v) >= 1000 for v in command))


def _health(row):
    if row.get('firmware_version') not in H_LQI_PROFILES:
        return 'unknown_firmware'
    required = ('device_time_ms', 'sample_counter', 'control_loop_us', 'sensor_flags',
                'adc_quality_reason', 'h_capture_arm_q10', 'h_integral_q8',
                'h_control_age_ms', 'h_control_flags', 'motor_command_permille')
    if any(key not in row for key in required):
        return 'missing_required_fields'
    if row['control_loop_us'] != 1000:
        return 'unknown_control_period'
    if row['h_control_flags'] & ~15 or row['h_control_age_ms'] > 512:
        return 'unknown_h_status'
    if (row['firmware_version'] != ADC_CONDITIONING_FIRMWARE and
            (row['sensor_flags'] & ~0x1f or row['adc_quality_reason'] & 0x80)):
        return 'unknown_measurement_status'
    if row['fault']:
        return 'fault'
    if row['state'] != 2 or not row['h_control_flags'] & 1:
        return 'inactive_h'
    if not row.get('diagnostic_supported') or any(row.get(key) for key in ('stop_pressed', 'input_fault', 'sensor_fault')):
        return 'stopped_or_unhealthy_diagnostics'
    if row['calibrated'] != 1 or not measurement_ready(row):
        return 'unhealthy_measurement'
    if row['firmware_version'] != ADC_CONDITIONING_FIRMWARE and row['adc_quality_reason'] & 0x5f:
        return 'unhealthy_adc_window'
    if abs(row['command_permille']) > 1000:
        return 'invalid_command'
    # Exact known released profiles use MOTOR_SIGN=-1; gated-zero is not active drive.
    if row['motor_command_permille'] != -row['command_permille']:
        return 'motor_command_not_applied'
    return None


def _boundary(previous, current):
    if current['firmware_version'] != previous['firmware_version']:
        return 'firmware_change'
    if current['h_capture_arm_q10'] != previous['h_capture_arm_q10']:
        return 'capture_reference_change'
    if current['h_control_age_ms'] < previous['h_control_age_ms']:
        return 'h_restart'
    if (current['sequence']-previous['sequence']) & 0xffff != 1:
        return 'sequence_gap_or_reset'
    if (current['device_time_ms']-previous['device_time_ms']) & 0xffffffff != 20:
        return 'device_time_gap_or_reset'
    if (current['sample_counter']-previous['sample_counter']) & 0xffffffff != 20:
        return 'sample_counter_gap_or_reset'
    return None


def analyze_telemetry(records, *, hands_off=None):
    """hands_off 显式为 {segment: 一起算序号, offset_s: 秒, evidence: 说明}。

    该证据来自用户/外部记录，不能从 H 命令、状态或文件时长推断。
    CRC 错帧、未知版本、不健康帧和时间缺口均切段，不插值或拼接。
    """
    if hands_off is not None:
        if (type(hands_off.get('segment')) is not int or hands_off['segment'] < 1 or
                type(hands_off.get('offset_s')) not in (int, float) or
                not math.isfinite(hands_off['offset_s']) or hands_off['offset_s'] < 0 or
                not isinstance(hands_off.get('evidence'), str) or not hands_off['evidence'].strip()):
            raise ValueError('hands_off_requires_segment_offset_and_external_evidence')
    segments, group, excluded, errors = [], [], {}, []
    decoded_count = total = 0

    def close(reason):
        nonlocal group
        if group:
            segments.append((group, reason))
            group = []

    for line, stored in enumerate(records, 1):
        total += 1
        try:
            if isinstance(stored, str):
                stored = json.loads(stored)
            raw_hex = stored.get('raw_hex')
            if not isinstance(raw_hex, str) or len(raw_hex) > 510:
                raise ValueError('missing_or_oversized_raw_frame')
            row = _decode_frame(bytes.fromhex(raw_hex))
            decoded_count += 1
        except (ValueError, AttributeError, TypeError) as error:
            close('invalid_raw_frame')
            if len(errors) < 20:
                errors.append(dict(line=line, error=str(error)))
            excluded['invalid_raw_frame'] = excluded.get('invalid_raw_frame', 0)+1
            continue
        reason = _health(row)
        if reason:
            close(reason)
            excluded[reason] = excluded.get(reason, 0)+1
            continue
        if group:
            reason = _boundary(group[-1], row)
            if reason:
                close(reason)
        row['_elapsed_ms'] = len(group)*20
        row['_elapsed_s'] = row['_elapsed_ms']/1000
        row['_source_line'] = line
        group.append(row)
    close('end_of_file')
    if hands_off and hands_off['segment'] > len(segments):
        raise ValueError('hands_off_segment_not_present')
    results = []
    for index, (rows, ending) in enumerate(segments, 1):
        duration = rows[-1]['_elapsed_s']
        tail = [r for r in rows if r['_elapsed_ms'] >= rows[-1]['_elapsed_ms']-20000]
        confirmation = hands_off if hands_off and hands_off['segment'] == index else None
        if confirmation and confirmation['offset_s'] > duration:
            raise ValueError('hands_off_offset_outside_segment')
        released = [r for r in rows if r['_elapsed_s'] >= confirmation['offset_s']] if confirmation else []
        release_duration = (released[-1]['_elapsed_ms']-released[0]['_elapsed_ms'])/1000 if released else None
        insufficient = []
        if confirmation is None:
            insufficient.append('hands_off_unconfirmed')
        if release_duration is None or release_duration < TARGET['hold_s']:
            insufficient.append('less_than_60s_confirmed_hands_off')
        if duration < TARGET['tail_s']:
            insufficient.append('less_than_20s_tail')
        if ending not in ('inactive_h', 'end_of_file'):
            insufficient.append('segment_ended_by_'+ending)
        tail_metrics = _metrics(tail)
        failures = []
        if duration >= TARGET['tail_s']:
            if tail_metrics['arm_peak_to_peak_deg'] > TARGET['arm_peak_to_peak_deg']:
                failures.append('arm_peak_to_peak_above_5deg')
            if tail_metrics['theta_zero_rms_deg'] > TARGET['theta_zero_rms_deg']:
                failures.append('theta_zero_rms_above_2deg')
        if released and any(abs(r['command_permille']) >= 1000 for r in released):
            failures.append('saturated_hands_off_samples')
        met = not insufficient and not failures
        results.append(dict(segment=index, firmware_id=rows[0]['firmware_version'],
            profile=H_LQI_PROFILES[rows[0]['firmware_version']], source_lines=[rows[0]['_source_line'], rows[-1]['_source_line']],
            device_time_ms=[rows[0]['device_time_ms'], rows[-1]['device_time_ms']],
            sample_counter=[rows[0]['sample_counter'], rows[-1]['sample_counter']],
            sequence=[rows[0]['sequence'], rows[-1]['sequence']], end_reason=ending,
            capture_arm_deg=rows[0]['h_capture_arm_q10']*RAD_Q10_TO_DEG,
            continuous_healthy_h_s=duration, hands_off_evidence=confirmation,
            confirmed_hands_off_s=release_duration, whole_segment=_metrics(rows),
            tail_20s_complete=duration >= TARGET['tail_s'], tail_observed=tail_metrics,
            hands_off_metrics=_metrics(released), insufficient_evidence=insufficient,
            failed_targets=failures, logged_target_met=met,
            advanced_logged_target_met=met and tail_metrics['arm_peak_to_peak_deg'] <= TARGET['advanced_arm_peak_to_peak_deg']))
    return dict(schema=1, input_kind='telemetry', target=TARGET.copy(), input_records=total,
                crc_decoded_frames=decoded_count, excluded_records=excluded, first_errors=errors,
                segments=results, any_logged_target_met=any(s['logged_target_met'] for s in results),
                limitations=['50Hz快照无法证明帧间瞬态与连续无饱和；1kHz记录需另行核查。',
                             '角度来自标定零点与名义1040计数/圈，并非独立机械真值。',
                             '松手证据为外部明确输入；该工具不推断操作者是否接触机构。',
                             '满足本日志目标不等于轻推恢复、自动起摆或赛题全部要求已验收。'])


def analyze_trace(archive):
    capture = rebuild(archive)
    meta, rows = capture['metadata'], capture['rows']
    period_ms = meta['period_cycles']/50000
    observed = sum((b['sample_counter']-a['sample_counter']) & 0xffffffff for a, b in zip(rows, rows[1:]))
    longest = run = 1 if rows else 0
    for previous, current in zip(rows, rows[1:]):
        run = run+1 if (current['sample_counter']-previous['sample_counter']) & 0xffffffff == 1 else 1
        longest = max(longest, run)
    # total_committed is overwritten history, never the number of retained samples.
    return dict(schema=1, input_kind='control_trace', target=TARGET.copy(),
                firmware_id=meta['firmware_id'], known_firmware=meta['firmware_id'] in H_LQI_PROFILES,
                transfer_complete=capture['complete'] and capture['archive_complete'],
                row_count=len(rows), retained_time_span_s=observed*period_ms/1000,
                contiguous=capture['sample_counter_continuity']['contiguous'],
                longest_contiguous_observable_s=max(0, longest-1)*period_ms/1000,
                total_committed_not_retained=meta['total_committed'], any_logged_target_met=False,
                insufficient_evidence=['trace_not_full_60s_hands_off_acceptance_record'],
                limitations=['环形缓存保留的最近样本不能代替完整60秒日志与松手证据。',
                             '有缺口的时间跨度不是连续健康控制时长。'])


def analyze_file(path, *, hands_off=None):
    path = Path(path)
    data = path.read_bytes()
    if path.suffix.lower() == '.json':
        if hands_off is not None:
            raise ValueError('hands_off_marker_requires_telemetry_jsonl')
        report = analyze_trace(json.loads(data))
    else:
        report = analyze_telemetry(data.decode('utf-8-sig').splitlines(), hands_off=hands_off)
    report['source'] = dict(path=str(path.resolve()), sha256=hashlib.sha256(data).hexdigest(), bytes=len(data))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path, help='telemetry.jsonl 或 capture.json；只读')
    parser.add_argument('--hands-off-segment', type=int, help='确认松手的连续健康H段序号，从1开始')
    parser.add_argument('--hands-off-offset-s', type=float, help='该段首帧之后的松手时间')
    parser.add_argument('--hands-off-evidence', help='用户确认或独立记录的来源说明，不能凭日志猜测')
    args = parser.parse_args()
    fields = (args.hands_off_segment, args.hands_off_offset_s, args.hands_off_evidence)
    if any(v is not None for v in fields) and not all(v is not None for v in fields):
        parser.error('松手段序号、时刻与外部证据说明必须一起提供')
    confirmation = (dict(segment=fields[0], offset_s=fields[1], evidence=fields[2])
                    if fields[0] is not None else None)
    try:
        report = analyze_file(args.input, hands_off=confirmation)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
