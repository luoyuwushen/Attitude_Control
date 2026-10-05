"""Device-reported diagnostics; never infer an acknowledgement from a write."""
import math
from host.adc_status import ADC_CONDITIONING_FIRMWARE, has_adc_conditioning, in_blind_zone
from host.tuning import tuning_values

START_RESULTS = {
    0: '尚未记录启动请求', 1: '启动请求已接纳', 2: '拒绝启动：未完成两点标定',
    3: '拒绝启动：估计器异常', 4: '拒绝启动：接口异常、采样超时、ADC 超量程或测量尚未就绪',
    5: '拒绝启动：停止或标定优先', 6: '拒绝启动：点动或电机测试正在进行',
    7: '拒绝启动：当前状态或直立接管条件不满足，或清故障优先',
}
FAULT_NAMES = {1: '传感器 / 接口异常', 2: '标定失效', 4: '采样丢失',
               8: '位移 / 速度超限', 16: '起摆超时', 32: '平衡跌落'}
DIAGNOSTIC_FIELDS = ['diagnostic_status', 'diagnostic_supported', 'start_result',
                     'stop_pressed', 'adc_over_range', 'input_fault', 'sensor_fault']
MEASUREMENT_FIRMWARE_MIN = 0x00020000
RECOVERABLE_SENSOR_FIRMWARE_MIN = 0x00020001
ADC_OBSERVABILITY_FIRMWARE_MIN = 0x00020002
MOTOR_TEST_FIRMWARE_MIN = 0x00020003
CONTINUOUS_MEAN_FIRMWARE_MIN = 0x00020004
MANUAL_POSITION_INDEPENDENT_FIRMWARE_MIN = 0x00020004
JOG_MEASUREMENT_FIRMWARE_MIN = 0x00020004
HANDOVER_R32_FIRMWARE = 0x00020005
HANDOVER_LQI_FIRMWARE_MIN = 0x00020006
MOTOR_TEST_STATUSES = {0: '无测试记录', 1: '运行中', 2: '达到时限', 3: '达到位移上限',
                       4: '手动停止', 5: '故障终止', 6: '未检测到编码器变化', 7: '请求条件拒收'}
ADC_QUALITY_REASONS = {
    0: '窗口 OTR', 1: '原码到达 0/1023', 2: '采样窗口跨度异常',
    3: '相邻采样突变', 4: '滤波/采样未填满', 5: '原码跨度警示', 6: '块内方差超限',
    8: '相邻均值突变', 9: '角度映射超限', 10: '标定跨度不合格',
    11: '标定样本过期', 12: '样本异常（未提供细因）',
}
MOTION_LIMITS = (
    ('arm_q10', 'arm_deg', '摆臂角', 6144, 180 / math.pi, '°'),
    ('arm_speed_q10', 'arm_speed_rad_s', '摆臂速度', 20480, 1, 'rad/s'),
    ('omega_q10', 'omega_rad_s', '摆杆速度', 30720, 1, 'rad/s'),
)
def manual_position_independent(record):
    version = record.get('firmware_version')
    return type(version) is int and version >= MANUAL_POSITION_INDEPENDENT_FIRMWARE_MIN


def arm_return_hint(record):
    method = ('已核验方向时可用短 F/B 朝原标定零位返回，或断使能后沿原路手动返回'
              if manual_position_independent(record) else '断使能后沿原路手动返回原标定零位附近')
    return ('R 只清故障，不清摆臂原点。先按 S / SW3 停机并确认停稳；' + method +
            '，注意线缆；不要用 U 改原点绕过限位。')


def _motion_limit_value(record, specification):
    raw_key, display_key, name, raw_limit, conversion, unit = specification
    limit = raw_limit / 1024 * conversion
    if raw_key in record:
        raw = record[raw_key]
        if type(raw) is not int or not -32768 <= raw <= 32767:
            return None, limit, None
        return raw / 1024 * conversion, limit, abs(raw) > raw_limit
    # Old imported records may only retain displayed units. Compare directly
    # in those units to avoid rounding an exact Q10 boundary during conversion.
    value = record.get(display_key)
    if type(value) not in (int, float) or not math.isfinite(value):
        return None, limit, None
    return value, limit, abs(value) > limit


def motion_limit_reason(record, command):
    """Mirror strict FPGA absolute limits; S/R and non-motion are unaffected."""
    if command not in ('G', 'H', 'F', 'B', 'J', 'K', 'L', 'M'):
        return ''
    record = record or {}
    specifications = (MOTION_LIMITS if command in ('G', 'H') else
                      MOTION_LIMITS[1:2] if manual_position_independent(record) else MOTION_LIMITS[:2])
    for specification in specifications:
        _, _, name, _, _, unit = specification
        value, limit, exceeded = _motion_limit_value(record, specification)
        if value is None:
            return f'等待有效{name}遥测，无法核验运动限位'
        if exceeded:
            prefix = 'G/H ' if name == '摆杆速度' or (name == '摆臂角' and manual_position_independent(record)) else ''
            detail = f'{prefix}{name} {value:+.3f}{unit} 超出绝对限位 ±{limit:.3f}{unit}'
            return detail + ('。' + arm_return_hint(record) if name == '摆臂角' else '；请停机并等待机构停稳后检查')
    return ''


def motion_limits_text(record):
    """Keep current values visible even after a fault bit has been cleared."""
    parts = []
    for specification in MOTION_LIMITS:
        _, _, name, _, _, unit = specification
        value, limit, exceeded = _motion_limit_value(record or {}, specification)
        precision = 2 if unit == '°' else 3
        current = f'{value:+.{precision}f}' if value is not None else '未提供'
        prefix = 'G/H ' if name == '摆杆速度' or (name == '摆臂角' and manual_position_independent(record or {})) else ''
        parts.append(f'{prefix}{name} {current} / ±{limit:.2f} {unit}' + ('（越限）' if exceeded else ''))
    return '绝对运动限位：' + ' · '.join(parts)


def adc_quality_names(value, record=None, *, version=None):
    if type(value) is not int or not 0 <= value <= 0xffff:
        return '未提供'
    if version is None:
        version = record.get('firmware_version') if isinstance(record, dict) else record
    labels = dict(ADC_QUALITY_REASONS)
    if type(version) is int and version >= CONTINUOUS_MEAN_FIRMWARE_MIN:
        labels.update({2: '均值块窗跨度 >32 code', 3: '相邻均值块步进 >32 code',
                       6: '块内方差超限（RMS >16 code）'})
    elif type(version) is int:
        labels.update({2: '滤后窗口跨度异常', 3: '连续中值采样突变'})
        labels.pop(6)
    if version == ADC_CONDITIONING_FIRMWARE:
        labels.update({0: '原始窗口 OTR 观测', 1: '原码到达 0/1023（盲区/尖峰观测）'})
    names = [label for bit, label in labels.items() if value & (1 << bit)]
    known = sum(1 << bit for bit in labels)
    if value & ~known:
        names.append('未知原因位')
    return '、'.join(names) or '无'


def has_measurement_diagnostics(record):
    version = record.get('firmware_version') if record else None
    return (record is not None and record.get('protocol_version') == 2 and
            type(version) is int and MEASUREMENT_FIRMWARE_MIN <= version <= 0xffffffff)


def measurement_block_reason(record):
    """Require device-reported valid/ready state, never infer it from the plot."""
    flags = record.get('sensor_flags')
    if type(flags) is not int or not 0 <= flags <= 0xffff:
        return '等待 FPGA 提供 ADC 有效与就绪状态'
    if in_blind_zone(record):
        return '摆杆位于水平附近盲区，当前角度不可用；扶离盲区并等待测量就绪后重新接管，无需按 R'
    if has_adc_conditioning(record) and flags & ~0x7f:
        return '传感器状态含未知标志，等待可识别的测量状态'
    if flags & (0x01 if has_adc_conditioning(record) else 0x13):
        return 'ADC 样本异常或窗口 / 实时 OTR 越界；请停机核验采样链'
    if not flags & 4:
        return 'ADC 测量无效；当前角度可能是最后可信值'
    if not flags & 8:
        return 'ADC 测量尚未就绪；请等待连续有效采样'
    return ''


def adc_detail_text(record, prefix='adc_'):
    """Explain received conversion statistics without assigning sensor health."""
    def number(name):
        value = record.get(prefix + name)
        return str(value) if type(value) is int else '未提供'
    flags = record.get(prefix + 'detail_flags')
    if type(flags) is not int:
        return '转换级统计：未提供。'
    baseline = '已建立（不代表窗口健康）' if flags & 1 else '尚未建立'
    saturation = '存在计数或和饱和' if flags & 2 else '未报告计数或和饱和'
    def event(name, bit, shift, title):
        if not flags & (1 << bit):
            return title + '：未报告事件'
        age = record.get(prefix + name + '_edge_ticks')
        if type(age) is not int:
            age_text = '未提供'
        elif age == 0xffff:
            age_text = '≥1310.70 μs 或无先前可追溯边沿（0xFFFF）'
        else:
            age_text = f'{age * .02:.2f} μs'
        tag = (flags >> shift) & 15
        return (f'{title}：中值输出索引 {number(name + "_index")}，距观测驱动边沿 {age_text}；'
                f'EN/BN2/BN1/PWM={tag:04b}')
    count, total = record.get(prefix + 'conversion_count'), record.get(prefix + 'filtered_sum')
    continuous_mean = has_continuous_adc_mean(record)
    mean_label = '全流核对均值' if continuous_mean else '全中值输出均值'
    if flags & 2:
        bypass = '计数或和已饱和，不计算核对均值' if continuous_mean else '计数或和已饱和，不计算旁路均值'
    elif type(count) is int and count > 0 and type(total) is int:
        meaning = ('与控制同一 1 ms 窗独立计算；控制均值按 Q4 舍入' if continuous_mean else
                   '全中值输出旁路均值，不替代控制均值')
        if has_adc_conditioning(record):
            meaning = '核对原始 1 ms 均值，不含后级去尖峰；控制输入见去尖峰控制均值'
        bypass = f'{total / count:.3f} code（{meaning}）'
    else:
        bypass = '未提供有效的和/计数'
    reference = record.get(prefix + 'reference_q4')
    reference_text = f'{reference / 16:.3f} code' if type(reference) is int else '未提供'
    observation = ('首次观察到传感器故障时最近发布窗口；标定失败若恰逢新窗发布，'
                   '可能晚于判失败的输入一窗。\n') if prefix == 'adc_fault_' else ''
    sampling = ('控制使用同一 1 ms 内全部中值输出连续平均（稳态 5000 个）；16 点诊断抽样范围不参与控制平均。'
                '质量检查：40 个输出 / 8 μs 均值块，块窗跨度或相邻块步进 >32 code、块内方差 RMS >16 code。\n'
                if continuous_mean else '')
    if has_adc_conditioning(record):
        sampling = ('1 ms 原均值保留全部 7 点中值输出；去尖峰控制均值另行上报。'
                    '超过 32 code 的变化需连续 64 次转换确认，短尖峰以可信值替代；'
                    '盲区测量不可用，原始 OTR 和质量原因保留供诊断。\n')
    return (observation + adc_filter_text(record) + '。\n' + sampling +
            f'上一完整 1 ms 输出基准：{baseline}；参考 {reference_text}。{saturation}。\n'
            '离群：中值与该参考相差 >16 code；每次转换间隔 0.2 μs。'
            '最长时长按滤后输出序列的连续离群计数计算，不是模拟毛刺宽度。\n' +
            event('first_outlier', 3, 4, '首离群') + '\n' +
            event('max_step', 2, 8, '最大阶跃') + '\n' +
            f'{mean_label}：{bypass}。\n'
            '近边沿指距数字驱动信号变化 ≤2 μs；观察有固定 20 ns 偏移，标签随 ADC 流水及中值选择传播。'
            '索引按本窗中值输出计数，边沿间隔与标签来自被选原码，同一原码可被重复选中。'
            '这些统计不参与控制判据，也不能证明模拟干扰来源。')


def has_continuous_adc_mean(record):
    version = record.get('firmware_version')
    return type(version) is int and version >= CONTINUOUS_MEAN_FIRMWARE_MIN


def adc_contribution_title(record):
    return '16 点诊断抽样' if has_continuous_adc_mean(record) else '贡献'


def adc_filter_text(record):
    version = record.get('firmware_version')
    if type(version) is not int or version < RECOVERABLE_SENSOR_FIRMWARE_MIN:
        return '中值滤波点数：当前固件未提供可识别版本'
    if version == ADC_CONDITIONING_FIRMWARE:
        return '7 点中值 + 64 次异常确认去尖峰 + 1 ms 控制均值'
    if version >= CONTINUOUS_MEAN_FIRMWARE_MIN:
        return '7 点中值 + 1 ms 全窗连续平均'
    return ('7 点中值 + 16 点平均' if version >= MOTOR_TEST_FIRMWARE_MIN else '3 点中值 + 16 点平均')


def adc_observability_text(record):
    """Compact main-panel view; the extension table retains every raw field."""
    failed = bool(record.get('sensor_fault_reason'))
    prefix = 'adc_fault_' if failed else 'adc_'
    def number(name):
        value = record.get(prefix + name)
        return str(value) if type(value) is int else '未提供'
    flags = record.get(prefix + 'detail_flags')
    if failed:
        heading = f'首次观测异常：设备 {number("time_ms")} ms / 窗 {number("sample_counter")}'
    else:
        heading = '当前转换窗'
    if prefix + 'detail_flags' not in record:
        if failed and any(key in record for key in
                ('adc_detail_flags', 'adc_fault_time_ms', 'adc_fault_sample_counter')):
            return f'\n{heading} · 首次异常转换级统计未提供；当前窗不能代替首次异常窗。'
        return ''
    longest = record.get(prefix + 'outlier_longest')
    duration = f'{longest * .2:.1f} μs' if type(longest) is int else '未提供'
    reference = '' if type(flags) is int and flags & 1 else ' · 参考尚未建立'
    saturation = ' · 计数/和存在饱和' if type(flags) is int and flags & 2 else ''
    return (f'\n{heading} · 滤后 {number("filtered_min")}～{number("filtered_max")}'
            f' · {adc_contribution_title(record)} {number("contribution_min")}～{number("contribution_max")}\n'
            f'离群 {number("outlier_count")}/{number("conversion_count")} · 滤后连续最长 {duration}'
            f' · 近驱动边沿 {number("edge_outlier_count")}（≤2 μs，仅相关统计）{reference}{saturation}')


def adc_diagnostic_text(record):
    if not record:
        return 'ADC 诊断：等待 FPGA 遥测。'
    if not any(key in record for key in ('adc_raw', 'adc_mean_q4', 'sensor_flags')):
        return 'ADC 诊断：当前固件未提供原始码、窗口范围与就绪信息。'
    def number(key):
        value = record.get(key)
        return str(value) if type(value) is int else '未提供'
    flags = record.get('sensor_flags')
    def flag(bit):
        return ('是' if flags & (1 << bit) else '否') if type(flags) is int else '未提供'
    mean = record.get('adc_mean_q4')
    mean_text = f'{mean / 16:.3f}' if type(mean) is int else '未提供'
    version = record.get('firmware_version')
    firmware = f'0x{version:08X}' if type(version) is int else '未提供'
    if version == MEASUREMENT_FIRMWARE_MIN:
        firmware = 'project-v0.2.0 · ' + firmware
    elif version == RECOVERABLE_SENSOR_FIRMWARE_MIN:
        firmware = 'project-v0.2.1 · ' + firmware
    elif version == ADC_OBSERVABILITY_FIRMWARE_MIN:
        firmware = 'project-v0.2.2 · ' + firmware
    elif version == MOTOR_TEST_FIRMWARE_MIN:
        firmware = 'project-v0.2.3 · ' + firmware
    elif version == CONTINUOUS_MEAN_FIRMWARE_MIN:
        firmware = 'project-v0.2.4 · ' + firmware
    elif version == HANDOVER_R32_FIRMWARE:
        firmware = 'project-v0.2.5 · ' + firmware
    elif version == HANDOVER_LQI_FIRMWARE_MIN:
        firmware = 'project-v0.2.6 · ' + firmware
    elif version == 0x00020007:
        firmware = 'project-v0.2.7 · ' + firmware
    elif version == 0x00020008:
        firmware = 'project-v0.2.8 · ' + firmware
    elif version == 0x00020009:
        firmware = 'project-v0.2.9 · ' + firmware
    elif version == ADC_CONDITIONING_FIRMWARE:
        firmware = 'project-v0.3.0 · ' + firmware
    reason_text = ''
    if 'adc_quality_reason' in record or 'sensor_fault_reason' in record:
        reason_text = ('\n当前质量：' + adc_quality_names(record.get('adc_quality_reason'), record) +
                       ' · 锁存传感器原因：' + adc_quality_names(record.get('sensor_fault_reason'), record))
        if record.get('sensor_fault_reason') and 'adc_fault_detail_flags' not in record:
            failed_mean = record.get('adc_fault_mean_q4')
            failed_mean_text = f'{failed_mean / 16:.3f}' if type(failed_mean) is int else '未提供'
            reason_text += (f"\n首次传感器故障观测时原码窗口 {number('adc_fault_window_min')}～{number('adc_fault_window_max')}"
                            f' · 滤后均值 {failed_mean_text}')
    control = record.get('adc_control_q4')
    control_text = (f' · 去尖峰控制均值 {control / 16:.3f}' if type(control) is int else
                    ' · 去尖峰控制均值 未提供') if has_adc_conditioning(record) else ''
    condition_text = (f' · 盲区 {flag(5)} · 尖峰拒绝 {flag(6)}（观测）'
                      if has_adc_conditioning(record) else '')
    return (f"ADC 原始码 {number('adc_raw')} · 均值 {mean_text}{control_text} · 原码窗口 {number('adc_window_min')}～{number('adc_window_max')}"
            f" · 标定 D/U {number('adc_down')} / {number('adc_up')}\n"
            f'测量有效 {flag(2)} · 已就绪 {flag(3)} · 样本异常 {flag(0)} · 窗口 OTR {flag(1)} · 实时 OTR {flag(4)}{condition_text}\n'
            f"固件 {firmware} · {adc_filter_text(record)} · 采样窗口计数 {number('sample_counter')} · 设备时间 {number('device_time_ms')} ms" +
            reason_text + adc_observability_text(record))


def motor_test_text(record):
    record = record or {}
    status = record.get('motor_test_status')
    reason = MOTOR_TEST_STATUSES.get(status, '未知状态') if type(status) is int else '未提供'
    delta = record.get('motor_test_delta')
    displacement = f'{delta:+d} count' if type(delta) is int else '未提供'
    return f'测试结果：{reason} · 相对编码器位移 {displacement}'


def handover_control_text(record):
    """Command-aligned H observations, not measured PWM or elapsed run time."""
    record = record or {}
    integral = record.get('h_integral_q8')
    capture = record.get('h_capture_arm_q10')
    age = record.get('h_control_age_ms')
    flags = record.get('h_control_flags')
    integral_text = (f'{integral / 256:+.3f}‰'
                     if type(integral) is int and -32768 <= integral <= 32767 else '未提供')
    capture_text = (f'{capture / 1024 * 180 / math.pi:+.2f}°'
                    if type(capture) is int and -32768 <= capture <= 32767 else '未提供')
    age_text = f'{age} ms' if type(age) is int and 0 <= age <= 65535 else '未提供'
    if type(flags) is int and 0 <= flags <= 255:
        names = ['H 运行' if flags & 1 else 'H 未运行']
        names.extend(name for bit, name in ((2, '积分更新允许'), (4, '抗饱和冻结'),
                                             (8, '积分到达限幅')) if flags & bit)
        if flags & 0xf0:
            names.append('未知标志位')
        flag_text = f'0x{flags:02X}（' + '、'.join(names) + '）'
    else:
        flag_text = '未提供'
    return (f'居中积分修正 {integral_text} · 接管摆臂参考 {capture_text}'
            f' · 位置渐入计时 {age_text} · H 标志 {flag_text}')


def motor_diagnostic_text(record):
    if not record:
        return '请求输出：— · 门控后命令：未提供 · 首次故障：未提供'
    requested, output = record.get('command_permille'), record.get('motor_command_permille')
    request_text = f'{requested}‰' if type(requested) is int else '未提供'
    output_text = f'{output}‰' if type(output) is int else '未提供'
    first = record.get('first_fault')
    cause = (f"0x{first:02X} " + ('、'.join(fault_names(first)) or '无')) if type(first) is int and 0 <= first <= 255 else '未提供'
    version = record.get('firmware_version')
    handover = ('\n' + handover_control_text(record)
                if any(key in record for key in ('h_integral_q8', 'h_capture_arm_q10',
                       'h_control_age_ms', 'h_control_flags')) or
                   (type(version) is int and version >= HANDOVER_LQI_FIRMWARE_MIN) else '')
    if handover:
        tuning = tuning_values(record)
        actual = f'{tuning["actual"]:+.2f}°' if tuning['actual'] is not None else '未提供'
        if not tuning['actual_valid']:
            actual += '（无效或未就绪）'
        target = '0.00°（控制目标）' if tuning['active'] else '未提供'
        handover += f'\n调参 actual {actual} · target {target} · out {request_text} · {tuning["algorithm"]}'
    return (f'请求输出 {request_text} · 门控后命令 {output_text}（软件输出，非引脚实测）\n'
            f'FPGA 锁存首次故障：{cause}\n' + motion_limits_text(record) + handover)


def fault_names(value):
    if type(value) is not int or not 0 <= value <= 255:
        return ['故障字段未提供或无效']
    names = [name for bit, name in FAULT_NAMES.items() if value & bit]
    if value & ~63:
        names.append('未知故障位')
    return names


def decode_diagnostics(value):
    if type(value) is not int or not 0 <= value <= 255:
        return {}
    result = {'diagnostic_status': value, 'diagnostic_supported': int(bool(value & 0x80))}
    if value & 0x80:
        result.update(start_result=(value >> 4) & 7, stop_pressed=(value >> 3) & 1,
                      adc_over_range=(value >> 2) & 1, input_fault=(value >> 1) & 1,
                      sensor_fault=value & 1)
    return result


def diagnostic_text(record):
    if not record:
        return '启动诊断：等待 FPGA 遥测。'
    if record.get('source') == 'demo':
        return '演示数据不含设备启动诊断。'
    if not record.get('diagnostic_supported'):
        return '启动诊断：固件未提供；请核对烧录版本。'
    otr_label = '原始 ADC OTR 观测' if has_adc_conditioning(record) else 'ADC 超量程'
    flags = [text for key, text in [('stop_pressed', 'SW3 按下'),
             ('adc_over_range', otr_label), ('input_fault', '接口异常锁存'),
             ('sensor_fault', '估计器异常锁存')] if record.get(key)]
    if in_blind_zone(record):
        flags.append('水平附近盲区（测量不可用，不锁存 0x01）')
    result = START_RESULTS.get(record.get('start_result'), '未知结果')
    if has_adc_conditioning(record) and record.get('start_result') == 4:
        result = '拒绝启动：接口异常、采样超时、盲区或测量尚未就绪'
    return ('最近 SW2 / G / H：' + result +
            '；板级状态：' + ('、'.join(flags) if flags else '无异常') +
            '。最近结果为锁存记录，实际运行以当前状态为准。')


def recovery_hint(record):
    if not record:
        return '等待 FPGA 遥测。'
    if record.get('stop_pressed'):
        return '请释放 SW3，再检查启动条件。'
    if in_blind_zone(record) and not (record.get('fault') or record.get('sensor_fault') or record.get('input_fault')):
        return '水平附近盲区使测量暂不可用；扶离盲区并等待连续 16 个有效样本后重新 H 接管。无需按 R，不会自动重启。'
    if record.get('adc_over_range') and not has_adc_conditioning(record):
        return 'ADC 超量程：停机检查传感器接线、供电和有效行程。'
    if record.get('sensor_fault') or record.get('input_fault'):
        version = record.get('firmware_version')
        if type(version) is int and version >= RECOVERABLE_SENSOR_FIRMWARE_MIN:
            return '先停机检查锁存传感器原因；恢复连续可信采样后按 R。持续异常不清除，清除不启动电机。'
        return '停机检查 ADC / 编码器信号与标定跨度；排除异常后重新 D、U 标定。R 只清控制器故障。'
    fault = record.get('fault')
    if type(fault) is not int or not 0 <= fault <= 255:
        return '故障字段未提供或无效，无法判断设备是否健康。'
    if fault & 4:
        return '有效采样超时：当前角度可能是旧值。停机检查采样链路，恢复后再重新启动。'
    limit_reason = motion_limit_reason(record, 'G')
    if limit_reason and '超出绝对限位' in limit_reason:
        return limit_reason
    if fault & 8:
        return '位移 / 速度曾超限；确认当前限位已恢复后再清故障。' + arm_return_hint(record)
    if record.get('fault'):
        return '先停止并排除故障原因；需要时重新标定，再清控制器故障 R。'
    if not record.get('calibrated'):
        return '自然下垂时按 D，停机扶至直立时按 U；确认标定完成后再起摆。'
    return '电机未动作时，先查看当前状态和控制量，再核验电机供电、驱动接线与方向。'
