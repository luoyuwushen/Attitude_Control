"""Evidence-preserving measurement helpers for the existing 50 Hz telemetry.

These helpers report what the sensors support, rather than filling unmeasured
physical parameters with model defaults. No third-party numerical dependency is
required. A result from demonstration data remains explicitly synthetic.
"""
from __future__ import annotations

import json
import math
import os
import statistics
import tempfile
from pathlib import Path


class MeasurementError(ValueError):
    """The supplied telemetry cannot support the requested measurement."""


_RAD_TO_DEG = 180.0 / math.pi
_Q10_DEG = _RAD_TO_DEG / 1024


def _number(value, name):
    if (not isinstance(value, (int, float)) or isinstance(value, bool)
            or not math.isfinite(value)):
        raise MeasurementError(f"{name} 必须是有限数值")
    return value


def _integer(value, name, low=None, high=None):
    if not isinstance(value, int) or isinstance(value, bool):
        raise MeasurementError(f"{name} 必须是整数，不能缺失或使用小数")
    if (low is not None and value < low) or (high is not None and value > high):
        raise MeasurementError(f"{name} 超出有效范围")
    return value


def _source_key(record):
    source = record.get("source")
    if not isinstance(source, str) or not source:
        raise MeasurementError("数据来源缺失，不能判断是否为实测")
    original = record.get("original_source")
    if original is not None and (not isinstance(original, str) or not original):
        raise MeasurementError("原始数据来源无效")
    return source, original


def validate_records(records, require_stopped=False, require_calibrated=True):
    """Reject discontinuous or malformed samples; never silently drop samples."""
    records = list(records)
    if len(records) < 2:
        raise MeasurementError("至少需要两个有效样本")
    previous = None
    source_key = None
    parameter_version = None
    for record in records:
        if not isinstance(record, dict):
            raise MeasurementError("遥测样本必须为字段字典")
        for key, value in record.items():
            if isinstance(value, (int, float)) and not isinstance(value, bool) and not math.isfinite(value):
                raise MeasurementError(f"{key} 包含非有限数值")
        time = _number(record.get("elapsed_s"), "elapsed_s")
        sequence = _integer(record.get("sequence"), "sequence", 0, 65535)
        state = _integer(record.get("state"), "state", 0, 4)
        _integer(record.get("fault"), "fault", 0, 255)
        calibrated = _integer(record.get("calibrated"), "calibrated", 0, 1)
        if require_calibrated and calibrated != 1:
            raise MeasurementError("包含未标定样本，请完成 D/U 后重新采集")
        if require_stopped:
            command = _integer(record.get("command_permille"), "command_permille", -1000, 1000)
            if state != 0 or command != 0:
                raise MeasurementError("此次测量要求所有样本处于 IDLE 且驱动指令为零")
        for flag in ("sequence_reset", "sequence_duplicate", "time_reset"):
            if record.get(flag):
                raise MeasurementError("包含复位、重复帧或时间重置，重新采集连续记录")
        for flag in ("origin_reset", "encoder_origin_reset", "calibration_reset", "calibration_changed"):
            if record.get(flag):
                raise MeasurementError("包含原点重设或标定变更，请重新采集连续记录")
        if "sequence_gap" in record and _integer(record["sequence_gap"], "sequence_gap", 0):
            raise MeasurementError("包含遥测丢帧，不能用于连续辨识")
        for key in ("theta_deg", "arm_deg", "omega_rad_s", "arm_speed_rad_s"):
            if key in record:
                _number(record[key], key)
        for key in ("theta_q10", "arm_q10", "omega_q10", "arm_speed_q10"):
            if key in record:
                _integer(record[key], key, -32768, 32767)
        for raw_field, display_field, factor in (
            ("theta_q10", "theta_deg", _Q10_DEG),
            ("arm_q10", "arm_deg", _Q10_DEG),
            ("omega_q10", "omega_rad_s", 1 / 1024),
            ("arm_speed_q10", "arm_speed_rad_s", 1 / 1024),
        ):
            if raw_field in record and display_field in record:
                if abs(record[display_field] - record[raw_field] * factor) > 1e-6:
                    raise MeasurementError(f"原始 {raw_field} 与 {display_field} 不一致，不能用于测量")
        if "adc" in record:
            _integer(record["adc"], "adc", 0, 1023)
        if "command_permille" in record:
            _integer(record["command_permille"], "command_permille", -1000, 1000)
        if "protocol_version" in record:
            _integer(record["protocol_version"], "protocol_version", 1, 2)
        if "encoder_count" in record:
            _integer(record["encoder_count"], "encoder_count", -(2**31), 2**31 - 1)
        for key in ("parameter_version", "device_time_ms"):
            if key in record:
                _integer(record[key], key, 0, 2**32 - 1)
        if "parameter_version" in record:
            current_version = record["parameter_version"]
            if parameter_version is not None and current_version != parameter_version:
                raise MeasurementError("测量区间内参数版本发生变化，请重新采集")
            parameter_version = current_version
        key = _source_key(record)
        if source_key is not None and key != source_key:
            raise MeasurementError("不能混合不同来源或不同回放来源的数据")
        source_key = key
        if previous is not None:
            delta_t = time - previous[0]
            if delta_t <= 0 or delta_t > 0.1 + 1e-9:
                raise MeasurementError("时间须递增且接收间隔不能超过 100 ms")
            if (sequence - previous[1]) % 65536 != 1:
                raise MeasurementError("序号不连续，包含丢帧、复位或重复帧")
        previous = time, sequence
    return records


def _base(records, method):
    source, original = _source_key(records[0])
    synthetic = source == "demo" or original == "demo"
    result = {
        "method": method, "source": source,
        "evidence": "synthetic" if synthetic else (
            "physical_telemetry" if source == "serial" else "recorded_telemetry"),
        "is_physical_measurement": not synthetic and source == "serial",
        "samples": len(records),
        "time_range": [records[0]["elapsed_s"], records[-1]["elapsed_s"]],
        "duration_s": records[-1]["elapsed_s"] - records[0]["elapsed_s"],
    }
    if original is not None:
        result["original_source"] = original
    return result


def _fault_free(records):
    if any(record["fault"] != 0 for record in records):
        raise MeasurementError("包含故障样本，不能用于本项辨识")


def _valid_sensor_codes(records):
    for record in records:
        code = _integer(record.get("adc"), "adc", 0, 1023)
        if code in (0, 1023):
            raise MeasurementError("ADC 到达电压轨，传感器状态存疑，不能用于本项辨识")


def _values(records, field):
    return [_number(record.get(field), field) for record in records]


def _wrap(angle):
    return (angle + 180.0) % 360.0 - 180.0


def static_measurement(records):
    """Static ADC noise from integer averaged codes, also before calibration."""
    records = validate_records(records, require_stopped=True, require_calibrated=False)
    adc = [_integer(record.get("adc"), "adc", 0, 1023) for record in records]
    result = _base(records, "static_adc")
    result.update(adc_mean=statistics.fmean(adc), adc_std=statistics.pstdev(adc),
                  adc_min=min(adc), adc_max=max(adc), adc_ptp=max(adc) - min(adc),
                  fault_samples=sum(record["fault"] != 0 for record in records),
                  calibrated=all(record["calibrated"] == 1 for record in records),
                  quality_warning="ADC 为 16 点平均后截断的整数码；标准差不代表 ADC 新增分辨率。静止需现场确认，停机速度零不能证明静止。")
    if result["calibrated"] and not result["fault_samples"]:
        if all("theta_deg" in record for record in records):
            angles = _values(records, "theta_deg")
            # Unwrap near the first sample so +180/-180 does not average to zero.
            unwrapped = [angles[0] + _wrap(angle - angles[0]) for angle in angles]
            mean = statistics.fmean(unwrapped)
            result["theta_mean_deg"] = _wrap(mean)
            result["theta_rms_deg"] = math.sqrt(statistics.fmean((angle - mean)**2 for angle in unwrapped))
        if all("arm_deg" in record for record in records):
            arm = _values(records, "arm_deg")
            result["arm_change_deg"] = arm[-1] - arm[0]
            result["arm_range_deg"] = max(arm) - min(arm)
    return result


def _arm_q10(record):
    if "arm_q10" in record:
        raw = _integer(record["arm_q10"], "arm_q10", -32768, 32767)
        if "arm_deg" in record and abs(record["arm_deg"] - raw * _Q10_DEG) > 1e-6:
            raise MeasurementError("原始 arm_q10 与角度不一致")
    else:
        angle = _number(record.get("arm_deg"), "arm_deg")
        raw = round(angle / _Q10_DEG)
        if abs(angle - raw * _Q10_DEG) > 1e-6:
            raise MeasurementError("arm_deg 不是完整 Q10 转换值，不能用显示的舍入角度恢复计数")
    if not -32768 < raw < 32767:
        raise MeasurementError("摆臂角度发生 Q10 饱和，不能恢复计数")
    return raw


def _inverted_count(record, coefficient, sign):
    raw = _arm_q10(record)
    count = -((-1024 * raw) // coefficient)
    if count * coefficient // 1024 != raw:
        raise MeasurementError("角度不符合所填固件计数系数，请核对 COUNTS_PER_REV")
    return count * sign


def encoder_measurement(start_records, end_records, configured_cpr=1040,
                        encoder_sign=1, physical_angle_deg=360.0, all_records=None):
    """Recover counts, then compare with an independently marked physical angle."""
    configured_cpr = _integer(configured_cpr, "configured_cpr", 1)
    encoder_sign = _integer(encoder_sign, "encoder_sign", -1, 1)
    if encoder_sign not in (-1, 1):
        raise MeasurementError("ENCODER_SIGN 必须为 +1 或 -1")
    physical_angle_deg = _number(physical_angle_deg, "physical_angle_deg")
    if physical_angle_deg == 0:
        raise MeasurementError("机械参考角度不能为零")
    start = validate_records(start_records, True, True)
    end = validate_records(end_records, True, True)
    endpoints = start + end
    _fault_free(endpoints)
    _valid_sensor_codes(endpoints)
    if _source_key(start[0]) != _source_key(end[0]):
        raise MeasurementError("起点与终点来源不同")
    if end[0]["elapsed_s"] <= start[-1]["elapsed_s"]:
        raise MeasurementError("终点必须位于起点之后")
    direct = all(record.get("protocol_version") == 2 and "encoder_count" in record for record in endpoints)
    coefficient = 6588397 // configured_cpr
    if not direct and coefficient < 1024:
        raise MeasurementError("当前转换系数小于 1024，遥测角度不能唯一逆算计数")
    for record in endpoints:
        _arm_q10(record)
    get_count = (lambda record: _integer(record.get("encoder_count"), "encoder_count", -(2**31), 2**31 - 1)) if direct else (
        lambda record: _inverted_count(record, coefficient, encoder_sign))
    start_counts, end_counts = [get_count(record) for record in start], [get_count(record) for record in end]
    if len(set(start_counts)) != 1 or len(set(end_counts)) != 1:
        raise MeasurementError("起终点窗口内计数未静止，请等待参考位置稳定后重新截取")
    context = None
    if all_records is not None:
        context = validate_records(all_records, True, True)
        _fault_free(context)
        _valid_sensor_codes(context)
        if _source_key(context[0]) != _source_key(start[0]):
            raise MeasurementError("全程记录与端点来源不同")
        context_by_key = {(record["elapsed_s"], record["sequence"]): record for record in context}
        context_keys = set(context_by_key)
        if any((record["elapsed_s"], record["sequence"]) not in context_keys for record in endpoints):
            raise MeasurementError("全程历史不包含完整起终点，不能核验本次测量")
        for record in endpoints:
            corresponding = context_by_key[record["elapsed_s"], record["sequence"]]
            if get_count(corresponding) != get_count(record) or _arm_q10(corresponding) != _arm_q10(record):
                raise MeasurementError("端点计数与全程历史中的对应样本不一致")
        if context[0]["elapsed_s"] > start[0]["elapsed_s"] or context[-1]["elapsed_s"] < end[-1]["elapsed_s"]:
            raise MeasurementError("全程记录未覆盖起终点")
        versions = {record["parameter_version"] for record in context if "parameter_version" in record}
        if len(versions) > 1:
            raise MeasurementError("测量中固件参数版本改变")
        previous_raw = None
        for record in context:
            raw = _arm_q10(record)
            get_count(record)
            if record.get("origin_reset") or record.get("calibration_reset"):
                raise MeasurementError("测量中重新标定原点，记录作废")
            if previous_raw is not None and raw == 0 and abs(previous_raw) * _Q10_DEG > 10:
                raise MeasurementError("发现突然归零，可能重设 U 原点；请核验后重新测量")
            previous_raw = raw
    delta = end_counts[0] - start_counts[0]
    if not delta:
        raise MeasurementError("起终点计数相同，尚未得到有效机械参考角度测量")
    result = _base(context or endpoints, "encoder_cpr")
    result.update(provenance="direct" if direct else "inverted",
                  relative_count_start=start_counts[0], relative_count_end=end_counts[0],
                  delta_count=delta, physical_angle_deg=physical_angle_deg,
                  cpr_estimate=abs(delta) * 360 / abs(physical_angle_deg),
                  configured_cpr=configured_cpr, encoder_sign=encoder_sign,
                  full_interval_checked=context is not None,
                  quality_warning="CPR 依据独立机械参考角度计算；所填固件 CPR 仅用于逆算。原点不能中途重新 D/U；遥测无法证明没有未被采到的板上 U 操作，需人工确认。")
    return result


def blind_zone_scan(records, jump_threshold_codes=32):
    """Flag ADC discontinuities/rails without claiming a blind-zone angle."""
    records = validate_records(records, require_stopped=True, require_calibrated=False)
    threshold = _number(jump_threshold_codes, "jump_threshold_codes")
    if threshold <= 0:
        raise MeasurementError("ADC 跳变阈值必须大于零")
    adc = [_integer(record.get("adc"), "adc", 0, 1023) for record in records]
    suspects = []
    for i, code in enumerate(adc):
        reasons = []
        if code <= 10 or code >= 1013:
            reasons.append("near_rail")
        if i and abs(code - adc[i - 1]) >= threshold:
            reasons.append("adc_jump")
        if records[i]["fault"]:
            reasons.append("reported_fault")
        if reasons:
            suspects.append({"index": i, "elapsed_s": records[i]["elapsed_s"],
                             "adc": code, "previous_adc": adc[i - 1] if i else None,
                             "reasons": reasons})
    result = _base(records, "adc_scan")
    result.update(adc_min=min(adc), adc_max=max(adc), adc_ptp=max(adc)-min(adc),
                  max_adjacent_jump_codes=max(abs(b-a) for a, b in zip(adc, adc[1:])),
                  near_rail_samples=sum(code <= 10 or code >= 1013 for code in adc),
                  fault_samples=sum(record["fault"] != 0 for record in records),
                  jump_threshold_codes=threshold, suspected_segments=suspects,
                  quality_warning="仅标记整数 ADC 的突变、近电压轨和故障区段；需结合机械参考与录像定位。不能由此测得盲区角度或电气行程。")
    return result


def free_decay(records, min_amplitude_deg=0.5, arm_fixed_tolerance_deg=2.0, noise_std_deg=None):
    """Estimate normalized small-angle dynamics around the hanging position."""
    records = validate_records(records, True, True)
    _fault_free(records)
    _valid_sensor_codes(records)
    min_amplitude = _number(min_amplitude_deg, "min_amplitude_deg")
    tolerance = _number(arm_fixed_tolerance_deg, "arm_fixed_tolerance_deg")
    if min_amplitude <= 0 or tolerance < 0:
        raise MeasurementError("峰值阈值应为正，摆臂固定容差不能为负")
    noise = 0.0 if noise_std_deg is None else _number(noise_std_deg, "noise_std_deg")
    if noise < 0:
        raise MeasurementError("静态噪声标准差不能为负")
    threshold = max(min_amplitude, 5 * noise, 3 * _Q10_DEG)
    beta = [_wrap(theta - 180) for theta in _values(records, "theta_deg")]
    arm = _values(records, "arm_deg")
    if max(abs(value) for value in beta) > 15:
        raise MeasurementError("摆杆超出下垂附近 ±15° 小角区，不能套用该衰减模型")
    if max(arm) - min(arm) > tolerance:
        raise MeasurementError("摆臂未近似固定，自由衰减含耦合运动")
    if any(abs(b-a) > 5 for a, b in zip(beta, beta[1:])):
        raise MeasurementError("下垂角存在异常跳变，不能进行衰减拟合")
    times = _values(records, "elapsed_s")
    smoothed = [beta[0]] + [statistics.fmean(beta[i-1:i+2]) for i in range(1, len(beta)-1)] + [beta[-1]]
    candidates = []
    for side in (1, -1):
        peaks = []
        for i in range(1, len(beta)-1):
            amplitude = side * smoothed[i]
            if amplitude < threshold or not (amplitude > side * smoothed[i-1] and amplitude >= side * smoothed[i+1]):
                continue
            if peaks:
                last_index = peaks[-1]["index"]
                if not any(side * value < -threshold for value in smoothed[last_index:i]):
                    # Noise/plateau peaks within the same half-cycle are one peak.
                    if amplitude > peaks[-1]["amplitude_deg"]:
                        peaks[-1] = {"index": i, "time_s": times[i], "amplitude_deg": amplitude}
                    continue
            peaks.append({"index": i, "time_s": times[i], "amplitude_deg": amplitude})
        candidates.append((side, peaks))
    side, peaks = max(candidates, key=lambda item: len(item[1]))
    if len(peaks) < 4:
        raise MeasurementError("至少需要四个高于噪声、同侧且跨过零位的可信峰值；请延长自由衰减记录")
    intervals = [b["time_s"]-a["time_s"] for a, b in zip(peaks, peaks[1:])]
    period = statistics.fmean(intervals)
    period_cv = statistics.pstdev(intervals) / period
    if period_cv > 0.15:
        raise MeasurementError("同侧峰值周期不一致，可能有干扰或峰值不可辨")
    if any(b["amplitude_deg"] > a["amplitude_deg"] + max(3*noise, _Q10_DEG)
           for a, b in zip(peaks, peaks[1:])):
        raise MeasurementError("峰值未呈连续衰减，不能拟合单一阻尼")
    x = [peak["time_s"] for peak in peaks]
    y = [math.log(peak["amplitude_deg"]) for peak in peaks]
    mx, my = statistics.fmean(x), statistics.fmean(y)
    sxx = sum((value-mx)**2 for value in x)
    syy = sum((value-my)**2 for value in y)
    slope = sum((a-mx)*(b-my) for a, b in zip(x, y)) / sxx
    decay = -slope
    residual = sum((b-(my+slope*(a-mx)))**2 for a, b in zip(x, y))
    r2 = 1-residual/syy if syy > 0 else 0.0
    if decay <= 0 or r2 < 0.9:
        raise MeasurementError("衰减不满足指数模型（要求下降且拟合 R²≥0.90），不输出阻尼系数")
    result = _base(records, "free_decay")
    result.update(damped_period_s=period, decay_rate_s_inv=decay,
                  b_over_J_s_inv=2*decay,
                  mgd_over_J_s_inv2=(2*math.pi/period)**2+decay**2,
                  fit_r2=r2, period_cv=period_cv, peak_count=len(peaks), peaks=peaks,
                  peak_side=side, amplitude_threshold_deg=threshold,
                  arm_range_deg=max(arm)-min(arm),
                  quality_warning="采用主机接收时间；约 50 Hz 遥测使峰值时间有一帧量化误差及串口抖动。本结果仅为小角、固定摆臂下的归一化辨识，不是绝对惯量或绝对阻尼。停机速度字段未参与计算。")
    if noise_std_deg is None:
        result["quality_warning"] += "未提供静态角度噪声，使用 0.5° 与 Q10 量化阈值预筛，需补充静态噪声核验。"
    return result


def jog_response(records):
    """Observe one bounded manual-jog episode plus its stopped coast."""
    records = validate_records(records, require_calibrated=True)
    _fault_free(records)
    _valid_sensor_codes(records)
    states = [record["state"] for record in records]
    if any(state not in (0, 4) for state in states):
        raise MeasurementError("点动分析只接受 IDLE 和点动 state=4 的记录")
    jogging = [i for i, state in enumerate(states) if state == 4]
    if not jogging:
        raise MeasurementError("未记录到 state=4 点动；约 50 Hz 遥测可能漏掉很短的点动")
    first, last = jogging[0], jogging[-1]
    if first < 3 or len(records)-1-last < 3:
        raise MeasurementError("需保留点动前与点动后各至少三帧 IDLE 记录")
    if jogging != list(range(first, last+1)):
        raise MeasurementError("只分析一次连续点动；请将多个点动分开采集")
    commands = [_integer(record.get("command_permille"), "command_permille", -1000, 1000) for record in records]
    if any(commands[i] != 0 for i, state in enumerate(states) if state == 0):
        raise MeasurementError("IDLE 样本驱动指令非零")
    active = [commands[i] for i in jogging]
    if any(command == 0 for command in active) or len({1 if command > 0 else -1 for command in active}) != 1:
        raise MeasurementError("点动指令为零或途中换向，不能分析单次方向响应")
    arm = _values(records, "arm_deg")
    for record in records:
        _arm_q10(record)
    times = _values(records, "elapsed_s")
    # Several frames can arrive together. Expand the angular-difference window
    # rather than dividing by microseconds between frames in one serial read.
    speeds = []
    for i in range(1, len(records)-1):
        radius = 1
        left, right = i-1, i+1
        while times[right]-times[left] < 0.04 and (left > 0 or right < len(records)-1):
            radius += 1
            left, right = max(0, i-radius), min(len(records)-1, i+radius)
        if times[right]-times[left] >= 0.04-1e-9:
            speeds.append((arm[right]-arm[left]) / _RAD_TO_DEG / (times[right]-times[left]))
    if not speeds:
        raise MeasurementError("接收时间窗口太短，不能可靠估计点动角度差分速度")
    initial = statistics.fmean(arm[:first])
    final = statistics.fmean(arm[-3:])
    displacement = final-initial
    result = _base(records, "jog_response")
    result.update(command_sign=1 if active[0] > 0 else -1,
                  command_permille_min=min(active), command_permille_max=max(active),
                  arm_change_deg=displacement, jog_change_deg=arm[last]-initial,
                  displacement_sign=1 if displacement > 0 else (-1 if displacement < 0 else 0),
                  direction_matches_command=(displacement * active[0] > 0) if displacement else None,
                  jog_duration_estimate_s=times[last+1]-times[first],
                  jog_duration_bounds_s=[max(0, times[last]-times[first]), times[last+1]-times[first-1]],
                  coast_change_deg=final-arm[last],
                  post_stop_change_deg=final-arm[last+1],
                  peak_speed_from_angle_rad_s=max(abs(speed) for speed in speeds),
                  speed_method="angle_difference_window_min_40ms",
                  quality_warning="点动时长由遥测状态估计，含约一帧误差；滑行起点以最后点动帧近似。速度由至少 40 ms 窗口的角度差分估计，受主机时间抖动和窗口平滑影响，未使用停机清零的速度字段。不据此给出精密机械时间常数、力矩或电机电气参数。")
    return result


def save_measurement(path, result, metadata=None):
    """Atomically save a UTF-8 JSON report; reject non-finite report values."""
    path = Path(path)
    if not isinstance(result, dict):
        raise MeasurementError("测量结果必须为字典")
    payload = {"schema_version": 1, "measurement": result, "metadata": metadata or {}}
    try:
        serialized = json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False)
    except (TypeError, ValueError) as error:
        raise MeasurementError(f"测量报告包含不可保存的数据：{error}") from error
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", newline="\n",
                                         dir=path.parent, prefix=f".{path.name}.", suffix=".tmp", delete=False) as output:
            temporary = Path(output.name)
            output.write(serialized + "\n")
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, path)
        temporary = None
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return path
