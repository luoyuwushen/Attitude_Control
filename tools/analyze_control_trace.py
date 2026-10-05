"""Read-only analysis of CRC-backed H control captures; never opens a port.

The JSON archive's derived rows are not trusted: reconstruct the original v3
packets first. Only exact firmware 0x00020009/0x00030000 have known models here.
Physical calibration accuracy, encoder scale and mechanical stability remain
outside this consistency check.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from host.control_trace import TraceCaptureCollector

FIRMWARE = 0x00020009
SUPPORTED_FIRMWARE = {FIRMWARE, 0x00030000}
GAINS = (904758, 77909, -34213, -61768)
ENC_COEFF = 6588397 // 1040
LIMIT = 100 << 24
RATE = 335544
TERMS = ('theta_term', 'omega_term', 'arm_term', 'speed_term',
         'integral_term', 'feedback_rounding', 'output_clipping')


def signed32(value):
    return (value + (1 << 31)) % (1 << 32) - (1 << 31)


def clip(value, bound):
    return max(-bound, min(bound, value))


def round_q24(value):
    return ((abs(value) + (1 << 23)) >> 24) * (-1 if value < 0 else 1)


def unwrap_step(delta):
    return delta - 6434 if delta > 3217 else delta + 6434 if delta < -3217 else delta


def rebuild(archive):
    frames = archive.get('raw_frames_hex')
    if not isinstance(frames, list) or len(frames) > 4099:
        raise ValueError('missing_or_unbounded_original_packets')
    collector = TraceCaptureCollector()
    terminal = []
    for raw in frames:
        if not isinstance(raw, str) or len(raw) > 480:
            raise ValueError('invalid_original_packet')
        terminal += collector.feed(bytes.fromhex(raw))
    terminal += collector.finish('analysis_input_missing_end')
    if len(terminal) != 1 or terminal[0]['metadata'] is None:
        raise ValueError('archive_must_contain_one_capture_with_metadata')
    actual = terminal[0]
    if archive.get('complete') and not actual['complete']:
        raise ValueError('declared_complete_but_wire_incomplete')
    if archive.get('metadata') != actual['metadata']:
        raise ValueError('archive_metadata_disagrees_with_wire')
    stored = archive.get('rows')
    if not isinstance(stored, list) or [r.get('raw_hex') for r in stored] != [r['raw_hex'] for r in actual['rows']]:
        raise ValueError('archive_rows_disagree_with_wire')
    # A terminal transport/storage failure may downgrade otherwise valid wire.
    actual['archive_complete'] = archive.get('complete') is True
    actual['archive_storage_complete'] = archive.get('storage_complete')
    return actual


def state_terms(row, target):
    products = [GAINS[0] * row['theta_q10'], GAINS[1] * row['omega_q10'],
                GAINS[2] * (row['arm_q10'] - target), GAINS[3] * row['arm_speed_q10']]
    age = row['control_age_ms']
    arm = products[2]
    # Signed arithmetic shifts are separate in the 3/4 stage, as in RTL.
    products[2] = arm >> 2 if age < 128 else arm >> 1 if age < 256 else (arm >> 1) + (arm >> 2) if age < 384 else arm
    terms = [-p / (1 << 20) for p in products]
    feedback = -(sum(products) >> 20)
    integral = round_q24(row['integral_q24'])
    request = feedback + integral
    command = clip(request, 1000)
    delta = clip(237 * (row['arm_q10'] - target), RATE)
    allowed = (age >= 384 and abs(row['theta_q10']) < 268 and
               abs(row['omega_q10']) < 3584 and abs(row['arm_speed_q10']) < 10240)
    freeze = allowed and ((request >= 1000 and delta > 0) or (request <= -1000 and delta < 0))
    flags = 1 | (2 if allowed and not freeze else 0) | (4 if freeze else 0) | (8 if abs(row['integral_q24']) >= LIMIT else 0)
    next_integral = clip(row['integral_q24'] + (delta if allowed and not freeze else 0), LIMIT)
    result = dict(zip(TERMS[:4], terms))
    result.update(integral_term=integral, feedback_rounding=feedback-sum(terms),
                  output_clipping=command-request, reconstructed_request=request,
                  reconstructed_command=command, expected_flags=flags,
                  expected_next_integral_q24=next_integral)
    return result


def healthy(row, firmware=FIRMWARE):
    if firmware == 0x00030000:
        return (row['sensor_flags'] & 0x2d == 0x0c and
                row['sensor_flags'] & ~0x7f == 0 and row['fault'] == 0 and
                row['h_flags'] & 1 != 0 and row['h_flags'] & ~15 == 0 and
                abs(row['omega_q10']) <= 30720 and abs(row['arm_speed_q10']) <= 20480 and
                abs(row['arm_q10']) <= 6144)
    return (row['sensor_flags'] == 0x0c and row['fault'] == 0 and
            row['adc_quality'] & 0x5f == 0 and row['h_flags'] & 1 != 0 and
            abs(row['omega_q10']) <= 30720 and abs(row['arm_speed_q10']) <= 20480 and
            abs(row['arm_q10']) <= 6144)


def describe(values):
    if not values:
        return dict(n=0, mean=None, stddev=None, rms=None, minimum=None, maximum=None)
    return dict(n=len(values), mean=statistics.fmean(values), stddev=statistics.pstdev(values),
                rms=math.sqrt(statistics.fmean(v*v for v in values)), minimum=min(values), maximum=max(values))


def spectrum(rows, period):
    if period != 50000:
        return dict(available=False, reason='period_not_production_1ms')
    if len(rows) < 512:
        return dict(available=False, reason='less_than_512_contiguous_healthy_rows')
    import numpy as np
    n = len(rows)
    frequency = np.fft.rfftfreq(n, .001)
    weights = np.full(len(frequency), 2.0)
    weights[0] = 0
    if n % 2 == 0:
        weights[-1] = 1
    channels = {}
    for name in ('command_permille', 'omega_q10', 'arm_speed_q10', *TERMS[:4]):
        values = np.array([r[name] for r in rows], dtype=float)
        power = abs(np.fft.rfft((values-values.mean()) * np.hanning(n)))**2 * weights
        total = float(power.sum())
        bands = []
        for low, high in [(0, 2), (2, 5), (5, 20), (20, 100), (100, 500)]:
            mask = (frequency > low) & (frequency <= high)
            bands.append(dict(lower_hz=low, upper_hz=high,
                              power_fraction=float(power[mask].sum()) / total if total > 0 else None))
        channels[name] = dict(bands=bands, demeaned_windowed_power=total)
    return dict(available=True, n=n, sample_rate_hz=1000, resolution_hz=1000/n,
                duration_s=n/1000, channels=channels,
                qualification='Digital signal spectrum, not physical vibration/noise attribution; about 4s cannot establish a stable 2.5s cycle.')


def analyze_capture(archive):
    capture = rebuild(archive)
    meta = capture['metadata']
    known = meta['firmware_id'] in SUPPORTED_FIRMWARE
    down, up = meta['adc_down_q4'], meta['adc_up_q4']
    span = abs(up-down)
    if known and not (512 <= span <= 14400 and 160 < up < 16208):
        raise ValueError('known_firmware_invalid_calibration')
    slope = 3294199 // span if known else None
    checks = {key: dict(checked=0, mismatches=0, first_mismatches=[]) for key in
              ('command', 'h_flags', 'theta_mapping', 'arm_origin', 'omega_iir',
               'arm_speed_iir', 'integral_transition', 'age_transition', 'motor_mapping')}
    rows = []
    origins = set()
    segments = []
    segment = []
    gate_zero = 0

    def check(name, index, actual, expected):
        item = checks[name]
        item['checked'] += 1
        if actual != expected:
            item['mismatches'] += 1
            if len(item['first_mismatches']) < 20:
                item['first_mismatches'].append(dict(index=index, actual=actual, expected=expected))

    for i, source in enumerate(capture['rows']):
        row = dict(source)
        row['healthy'] = healthy(row, meta['firmware_id'])
        previous = rows[-1] if rows else None
        contiguous = previous is not None and ((row['sample_counter']-previous['sample_counter']) & 0xffffffff) == 1
        row['continuous_from_previous'] = contiguous
        if known:
            terms = state_terms(row, meta['capture_arm_q10'])
            row.update(terms)
            check('command', i, row['command_permille'], terms['reconstructed_command'])
            check('h_flags', i, row['h_flags'], terms['expected_flags'])
            expected_theta = ((up-row['adc_mean_q4']) if up > down else (row['adc_mean_q4']-up)) * slope >> 10
            expected_theta = unwrap_step(expected_theta)
            check('theta_mapping', i, row['theta_q10'], expected_theta)
            if row['motor_command_permille'] == 0 and row['command_permille'] != 0:
                gate_zero += 1
            else:
                check('motor_mapping', i, row['motor_command_permille'], -row['command_permille'])
            if row['healthy']:
                low = (row['arm_q10'] * 1024 + ENC_COEFF-1) // ENC_COEFF
                high = ((row['arm_q10']+1) * 1024 + ENC_COEFF-1) // ENC_COEFF-1
                if low == high:
                    origin = signed32(row['encoder_count']-low)
                    check('arm_origin', i, origin, next(iter(origins)) if origins else origin)
                    origins.add(origin)
                    row['inferred_digital_origin'] = origin
                else:
                    check('arm_origin', i, row['arm_q10'], 'not_representable_by_nominal_encoder_scale')
            if contiguous:
                check('integral_transition', i, row['integral_q24'], previous['expected_next_integral_q24'])
                check('age_transition', i, row['control_age_ms'], min(512, previous['control_age_ms']+1))
                if row['healthy'] and previous['healthy']:
                    speed = unwrap_step(row['theta_q10']-previous['theta_q10']) * 1000
                    expected_omega = previous['omega_q10'] + ((speed-previous['omega_q10']) >> 3)
                    step = signed32(row['encoder_count']-previous['encoder_count'])
                    raw_speed = signed32((step*ENC_COEFF*1000) >> 10)
                    expected_speed = previous['arm_speed_q10'] + ((raw_speed-previous['arm_speed_q10']) >> 3)
                    check('omega_iir', i, row['omega_q10'], expected_omega)
                    check('arm_speed_iir', i, row['arm_speed_q10'], expected_speed)
                    row['theta_step_q10'] = unwrap_step(row['theta_q10']-previous['theta_q10'])
                    row['encoder_step'] = step
                    row['adc_step_q4'] = row['adc_mean_q4']-previous['adc_mean_q4']
        if row['healthy'] and known:
            if not contiguous and segment:
                segments.append(segment)
                segment = []
            segment.append(row)
        elif segment:
            segments.append(segment)
            segment = []
        rows.append(row)
    if segment:
        segments.append(segment)
    longest = max(segments, key=len, default=[])
    errors = sum(c['mismatches'] for c in checks.values())
    # Empty/unknown data cannot pass an unperformed digital-model check.
    verified = bool(known and rows and errors == 0)
    report = dict(schema=1, firmware_id=meta['firmware_id'], known_profile=known,
        parameter_source=('static_project-v0.3.0_definition' if meta['firmware_id'] == 0x00030000
                          else 'static_project-v0.2.9_definition') if known else None,
        adc_mean_semantics=('conditioned_control_input_q4' if meta['firmware_id'] == 0x00030000
                            else 'median7_full_window_mean_q4'),
        transfer_complete=capture['complete'] and capture['archive_complete'],
        storage_complete=capture['archive_storage_complete'],
        sample_continuity=capture['sample_counter_continuity'], metadata=meta,
        row_count=len(rows), healthy_rows=sum(r['healthy'] for r in rows),
        command_saturation_rows=sum(abs(r['command_permille']) == 1000 for r in rows),
        digital_model_consistent=verified if rows and known else None, checks=checks,
        zero_gated_nonzero_requests=gate_zero, inferred_digital_origins=sorted(origins),
        calibration_slope=slope, longest_healthy_segment_rows=len(longest),
        terms={key: describe([r[key] for r in rows if r['healthy']]) for key in TERMS} if known else {},
        spectrum=spectrum(longest, meta['period_cycles']) if known and verified else dict(available=False,reason='unknown_or_inconsistent_digital_model'),
        limitations=[
            'CRC and digital consistency do not establish physical angle, CPR, torque, sensor noise cause or stable balance.',
            'First retained filter state is observed, not initialized to zero. Recurrences skip gaps and invalid measurements.',
            'Gate-zero observation alone does not identify which safety input stopped the motor.',
            'Feedback terms are coupled; their RMS or spectral power must not be interpreted as causal variance shares.',
            'Known model fixes the released default signs, nominal 1040 count/rev and estimator 1000 factor; no inference for future firmware.'])
    return report, rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    source = args.capture.resolve()
    if source.stat().st_size > 16 * 1024 * 1024:
        parser.error('capture exceeds bounded 16 MiB input')
    raw = source.read_bytes()
    report, rows = analyze_capture(json.loads(raw))
    output = args.output.resolve()
    if output == source.parent or output.exists():
        parser.error('choose a new analysis directory, separate from the source capture')
    output.mkdir(parents=True)
    report['input'] = dict(path=str(source), sha256=hashlib.sha256(raw).hexdigest())
    (output/'analysis.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    if rows:
        fields = list(dict.fromkeys(key for row in rows for key in row))
        with (output/'contributions.csv').open('w',encoding='utf-8-sig',newline='') as target:
            writer=csv.DictWriter(target,fieldnames=fields)
            writer.writeheader();writer.writerows(rows)
    body=['# H 连续记录分析', '',
          f"记录行数：{report['row_count']}；健康行数：{report['healthy_rows']}。",
          f"导出完整：{report['transfer_complete']}；数字模型一致：{report['digital_model_consistent']}。", '',
          '完整性与数字模型核对不代表实物已经稳定。', '',
          '| 核对项 | 已核对 | 不一致 |', '| --- | ---: | ---: |']
    body += [f"| {key} | {value['checked']} | {value['mismatches']} |" for key,value in report['checks'].items()]
    body += ['', '各控制项保留有符号贡献；不同项可能相互抵消，不能按各项RMS排序认定故障成因。',
             '首行速度以已记录值作为初值；序号缺口两侧不插值，也不强行连接滤波状态。',
             '频谱只使用最长连续健康段，且要求标称1毫秒采样；约4秒记录不足以证明约2.5秒慢周期已经收敛。']
    (output/'report.md').write_text('\n'.join(body)+'\n',encoding='utf-8')
    assert source.read_bytes()==raw
    print(json.dumps(dict(output=str(output),row_count=len(rows),
                          digital_model_consistent=report['digital_model_consistent'],
                          transfer_complete=report['transfer_complete']),ensure_ascii=False))


if __name__ == '__main__':
    main()
