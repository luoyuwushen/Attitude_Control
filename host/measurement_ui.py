"""Manually opened measurement assistance; never sends motion commands."""
from __future__ import annotations

from collections import deque
from copy import deepcopy
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

from PySide6.QtWidgets import (QComboBox, QDoubleSpinBox, QFormLayout, QGroupBox,
    QHBoxLayout, QLabel, QLineEdit, QPushButton, QScrollArea, QSpinBox,
    QStackedWidget, QTextEdit, QVBoxLayout, QWidget)


METHODS = [
    ('static', 'ADC 静态噪声 / 标定辅助',
     '停止电机并保持摆杆静止，计算当前窗口。分别将摆杆保持在下垂、直立位置后保存静态段。保存动作只记录遥测，不会发送 D / U。ADC 是整数遥测码的近似结果，不等于内部 Q4 标定寄存器。'),
    ('encoder', '编码器单圈 / 已知角度计数',
     '停止电机，在摆臂稳定时记录起点；手动转过已测量的机械角度，稳定后记录终点。可使用安全的小角度，无需强行转满一圈。实际转角必须来自人工测量，固件系数仅用于逆算计数。'),
    ('scan', 'ADC 有效区 / 疑似盲区扫描',
     '停止电机，缓慢手动扫描摆杆角度并计算窗口；允许未标定的待机数据。结果给出 ADC 覆盖范围和疑似跳变，不能仅凭跳变确诊传感器盲区。'),
    ('decay', '下垂自由衰减',
     '先完成标定，手动固定摆臂，将摆杆从下垂位置偏离约 5–10° 后轻轻释放。记录至少 4 个同侧峰值，再计算窗口；可填入已测静态角度噪声以筛除噪声峰。'),
    ('jog', '点动 / 滑行响应',
     '通过开发者窗口的 F / B 限时点动操作采集数据；让所选窗口覆盖点动前、运动中及停止后的滑行。这里仅分析已采集的指令和运动响应，不会发出运动命令。'),
]
SOURCE_NAMES = {'idle': '未连接', 'serial': '串口实测遥测',
                'demo': '演示数据（非实测）', 'replay': '回放数据（非本次实测）'}
RESULT_LABELS = {
    'samples': '样本数', 'duration_s': '采样时长（秒）', 'adc_mean': 'ADC 均值（码）',
    'adc_std': 'ADC 噪声标准差（码）', 'adc_min': 'ADC 最小值（码）',
    'adc_max': 'ADC 最大值（码）', 'adc_ptp': 'ADC 跨度（码）',
    'theta_mean_deg': '摆杆平均角度（°）', 'theta_rms_deg': '静态角度噪声 RMS（°）',
    'arm_change_deg': '摆臂总位移（°）', 'arm_range_deg': '摆臂变化范围（°）',
    'fault_samples': '故障样本数', 'relative_count_start': '起点相对计数',
    'relative_count_end': '终点相对计数', 'delta_count': '实测计数差',
    'physical_angle_deg': '人工机械参考角度（°）', 'cpr_estimate': '据机械参考估计的每圈计数',
    'configured_cpr': '当前固件每圈计数', 'encoder_sign': '当前固件编码器方向',
    'full_interval_checked': '起终点间完整记录检查',
    'max_adjacent_jump_codes': '最大相邻 ADC 跳变（码）', 'near_rail_samples': '接近电压轨样本数',
    'jump_threshold_codes': '疑似跳变阈值（码）', 'damped_period_s': '衰减振动周期（秒）',
    'decay_rate_s_inv': '振幅衰减率（1/秒）', 'b_over_J_s_inv': '归一化阻尼 b/J（1/秒）',
    'mgd_over_J_s_inv2': '归一化重力项 mgd/J（1/秒²）', 'fit_r2': '指数衰减拟合 R²',
    'period_cv': '周期变异系数', 'peak_count': '可信同侧峰数', 'peak_side': '峰值方向',
    'amplitude_threshold_deg': '有效峰值幅度阈值（°）',
    'command_sign': '点动指令方向', 'command_permille_min': '最小驱动指令（‰）',
    'command_permille_max': '最大驱动指令（‰）', 'jog_change_deg': '点动期间摆臂位移（°）',
    'displacement_sign': '实际位移方向', 'direction_matches_command': '运动方向与指令一致',
    'jog_duration_estimate_s': '遥测估计点动时长（秒）', 'jog_duration_bounds_s': '点动时长区间（秒）',
    'coast_change_deg': '最后点动帧后位移（°）', 'post_stop_change_deg': '首个停机帧后位移（°）',
    'peak_speed_from_angle_rad_s': '角度差分峰值速度（rad/s）', 'calibrated': '窗口均已标定',
}


def _text(value):
    widget = QLabel(value)
    widget.setWordWrap(True)
    return widget


class MeasurementPanel(QWidget):
    def __init__(self, output_root, parent=None):
        super().__init__(parent)
        self.output_root = Path(output_root)
        self.records = deque(maxlen=30000)
        self.source = 'idle'
        self.fresh = False
        self.endpoints = {}
        self.adc_segments = {}
        self.result = None
        self.evidence_records = []
        self.result_parameters = {}
        self._build_ui()
        self._update_enabled()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        layout.addWidget(scroll)
        content = QWidget()
        scroll.setWidget(content)
        body = QVBoxLayout(content)
        body.addWidget(_text('开发者模式 · 实测与辨识'))
        self.source_label = _text('数据来源：未连接')
        body.addWidget(self.source_label)
        common = QFormLayout()
        self.method = QComboBox()
        for key, title, _ in METHODS:
            self.method.addItem(title, key)
        self.window_seconds = QSpinBox()
        self.window_seconds.setRange(1, 120)
        self.window_seconds.setValue(10)
        self.window_seconds.setSuffix(' 秒')
        common.addRow('测量项目', self.method)
        common.addRow('当前数据窗口', self.window_seconds)
        body.addLayout(common)
        self.instructions = _text(METHODS[0][2])
        body.addWidget(self.instructions)
        self.options = QStackedWidget()
        body.addWidget(self.options)
        adc = QWidget()
        adc_layout = QVBoxLayout(adc)
        adc_buttons = QHBoxLayout()
        self.capture_down = QPushButton('保存下垂静态段（末 1 秒）')
        self.capture_up = QPushButton('保存直立静态段（末 1 秒）')
        self.capture_down.clicked.connect(lambda: self.capture_adc('down'))
        self.capture_up.clicked.connect(lambda: self.capture_adc('up'))
        adc_buttons.addWidget(self.capture_down)
        adc_buttons.addWidget(self.capture_up)
        adc_layout.addLayout(adc_buttons)
        self.adc_status = _text('尚未保存下垂 / 直立静态段')
        adc_layout.addWidget(self.adc_status)
        self.options.addWidget(adc)
        enc = QWidget()
        enc_layout = QVBoxLayout(enc)
        enc_form = QFormLayout()
        self.configured_cpr = QSpinBox()
        self.configured_cpr.setRange(1, 10000000)
        self.configured_cpr.setValue(1040)
        self.encoder_sign = QComboBox()
        self.encoder_sign.addItem('+1', 1)
        self.encoder_sign.addItem('−1', -1)
        self.physical_angle = QDoubleSpinBox()
        self.physical_angle.setRange(-36000, 36000)
        self.physical_angle.setDecimals(3)
        self.physical_angle.setValue(0)
        self.physical_angle.setSuffix(' °')
        enc_form.addRow('当前固件 COUNTS_PER_REV', self.configured_cpr)
        enc_form.addRow('当前固件 ENCODER_SIGN', self.encoder_sign)
        enc_form.addRow('人工测得的实际转角（带方向）', self.physical_angle)
        enc_layout.addLayout(enc_form)
        enc_buttons = QHBoxLayout()
        self.capture_start = QPushButton('记录起点（末 1 秒）')
        self.capture_end = QPushButton('记录终点（末 1 秒）')
        self.capture_start.clicked.connect(lambda: self.capture_endpoint('start'))
        self.capture_end.clicked.connect(lambda: self.capture_endpoint('end'))
        enc_buttons.addWidget(self.capture_start)
        enc_buttons.addWidget(self.capture_end)
        enc_layout.addLayout(enc_buttons)
        self.endpoint_status = _text('尚未记录起点 / 终点')
        enc_layout.addWidget(self.endpoint_status)
        self.options.addWidget(enc)
        scan = QWidget()
        scan_form = QFormLayout(scan)
        self.jump_threshold = QSpinBox()
        self.jump_threshold.setRange(1, 4095)
        self.jump_threshold.setValue(32)
        scan_form.addRow('相邻 ADC 疑似跳变阈值（码）', self.jump_threshold)
        self.options.addWidget(scan)
        decay = QWidget()
        decay_form = QFormLayout(decay)
        self.noise_std = QLineEdit()
        self.noise_std.setPlaceholderText('未知可留空；单位 °')
        self.arm_tolerance = QDoubleSpinBox()
        self.arm_tolerance.setRange(0.01, 30)
        self.arm_tolerance.setValue(2)
        self.arm_tolerance.setSuffix(' °')
        decay_form.addRow('已测静态角度噪声标准差', self.noise_std)
        decay_form.addRow('固定摆臂最大允许变化', self.arm_tolerance)
        self.options.addWidget(decay)
        self.options.addWidget(QWidget())
        self.calculate_button = QPushButton('计算当前窗口')
        self.calculate_button.clicked.connect(self.calculate)
        body.addWidget(self.calculate_button)
        self.result_view = QTextEdit()
        self.result_view.setReadOnly(True)
        self.result_view.setMinimumHeight(230)
        body.addWidget(self.result_view)
        manual = QGroupBox('机械人工测量与追溯信息')
        form = QFormLayout(manual)
        self.manual_fields = {}
        for key, title in [('arm_length_mm', '摆臂长度（mm）'),
                           ('pendulum_length_mm', '摆杆长度（mm）'),
                           ('mass_g', '摆杆质量（g）'),
                           ('center_of_mass_mm', '质心距转轴（mm）'),
                           ('measurement_tool', '测量工具'), ('precision', '工具精度'),
                           ('conditions', '观察 / 测量条件'),
                           ('operator', '记录人'), ('firmware_git', '固件 Git 版本')]:
            entry = QLineEdit()
            entry.setPlaceholderText('未知可留空')
            self.manual_fields[key] = entry
            form.addRow(title, entry)
        body.addWidget(manual)
        self.save_manual_button = QPushButton('保存机械人工测量记录')
        self.save_manual_button.clicked.connect(self.save_manual_report)
        body.addWidget(self.save_manual_button)
        self.save_button = QPushButton('保存测量报告与原始窗口')
        self.save_button.clicked.connect(self.save_report)
        body.addWidget(self.save_button)
        self.save_status = _text('报告保存到测量目录，每次生成独立文件夹。')
        body.addWidget(self.save_status)
        body.addStretch()
        self.method.currentIndexChanged.connect(self._method_changed)
        for widget in (self.window_seconds, self.configured_cpr, self.physical_angle,
                       self.jump_threshold, self.arm_tolerance):
            widget.valueChanged.connect(self._invalidate_result)
        self.encoder_sign.currentIndexChanged.connect(self._invalidate_result)
        self.noise_std.textChanged.connect(self._invalidate_result)

    def _method_changed(self, index):
        self.options.setCurrentIndex(index)
        self.instructions.setText(METHODS[index][2])
        self._invalidate_result()

    def _invalidate_result(self, *_):
        self.result = None
        self.evidence_records = []
        self.result_parameters = {}
        self.result_view.clear()
        self.save_button.setEnabled(False)

    def _update_enabled(self):
        available = self.source in {'serial', 'demo', 'replay'} and bool(self.records)
        if self.source == 'serial' and not self.fresh:
            available = False
        for widget in (self.capture_down, self.capture_up, self.capture_start,
                       self.capture_end, self.calculate_button):
            widget.setEnabled(available)
        self.save_button.setEnabled(self.result is not None)
        suffix = '；数据已过期，请恢复遥测' if self.source == 'serial' and not self.fresh else ''
        self.source_label.setText('数据来源：' + SOURCE_NAMES.get(self.source, self.source) + suffix)

    def set_context(self, source, fresh=True):
        if source != self.source:
            self.reset(source)
        self.fresh = bool(fresh)
        self._update_enabled()

    def reset(self, source):
        self.records.clear()
        self.endpoints.clear()
        self.adc_segments.clear()
        self.source = source
        self.fresh = False
        self.endpoint_status.setText('尚未记录起点 / 终点')
        self.adc_status.setText('尚未保存下垂 / 直立静态段')
        self._invalidate_result()
        self._update_enabled()

    def notify_command(self, command):
        if str(command).strip().upper() in {'D', 'U', 'CONNECT', 'DISCONNECT', '连接', '断开', 'SOURCE', 'RESET'}:
            self.reset(self.source)

    def ingest(self, record):
        if record.get('source') != self.source:
            return
        try:
            elapsed = float(record['elapsed_s'])
        except (KeyError, TypeError, ValueError):
            return
        if not math.isfinite(elapsed):
            return
        if self.records and elapsed <= self.records[-1]['elapsed_s']:
            self.reset(self.source)
        self.records.append(deepcopy(record))
        self.fresh = True
        self._update_enabled()

    def _window(self, seconds=None):
        if not self.records or (self.source == 'serial' and not self.fresh):
            raise ValueError('当前没有新鲜的可用遥测数据。')
        end = self.records[-1]['elapsed_s']
        length = seconds if seconds is not None else self.window_seconds.value()
        records = [deepcopy(row) for row in self.records if row['elapsed_s'] >= end - length]
        if len(records) < 3:
            raise ValueError('至少需要 3 帧有效数据。')
        self._validate_continuity(records)
        return records

    @staticmethod
    def _validate_continuity(records):
        previous = None
        for record in records:
            if record.get('sequence_gap') or record.get('sequence_reset') or record.get('sequence_duplicate') or record.get('time_reset'):
                raise ValueError('数据窗口存在丢帧、重复帧或重置，请重新采集完整窗口。')
            sequence = record.get('sequence')
            if not isinstance(sequence, int):
                raise ValueError('数据缺少有效帧序号。')
            if previous is not None and ((sequence - previous) & 65535) != 1:
                raise ValueError('帧序号不连续，请重新采集完整窗口。')
            previous = sequence

    def _error(self, error):
        self._invalidate_result()
        self.result_view.setPlainText('暂不能计算：' + str(error))

    @staticmethod
    def _stable_duration(rows):
        if rows[-1]['elapsed_s'] - rows[0]['elapsed_s'] < 0.8:
            raise ValueError('静态端点需要至少 0.8 秒连续记录，请保持稳定并继续采集。')

    def capture_adc(self, position):
        from host.measurements import static_measurement
        try:
            rows = self._window(1)
            self._stable_duration(rows)
            result = static_measurement(rows)
            self.adc_segments[position] = {'records': rows, 'result': result}
            self._invalidate_result()
            summaries = []
            for key, title in [('down', '下垂'), ('up', '直立')]:
                if key in self.adc_segments:
                    value = self.adc_segments[key]['result']
                    summaries.append(f"{title}：均值 {value.get('adc_mean', 0):.2f} 码，跨度 {value.get('adc_ptp', 0):.0f} 码，噪声标准差 {value.get('adc_std', 0):.3f} 码")
            self.adc_status.setText('；'.join(summaries))
        except ValueError as error:
            self._error(error)

    def capture_endpoint(self, position):
        from host.measurements import validate_records
        try:
            rows = self._window(1)
            self._stable_duration(rows)
            validate_records(rows, require_stopped=True, require_calibrated=True)
            arm = [float(row['arm_deg']) for row in rows]
            if max(arm) - min(arm) > 0.5:
                raise ValueError('端点摆臂尚未稳定（末 1 秒变化超过 0.5°）。')
            if position == 'end' and 'start' not in self.endpoints:
                raise ValueError('请先记录起点。')
            if position == 'start':
                self.endpoints.clear()
            elif rows[0]['elapsed_s'] <= self.endpoints['start'][-1]['elapsed_s']:
                raise ValueError('起点和终点静态段不能重叠，请完成实际转动后再记录。')
            self.endpoints[position] = rows
            self._invalidate_result()
            self.endpoint_status.setText('；'.join(
                f"{'起点' if key == 'start' else '终点'}：{value[-1]['elapsed_s']:.2f} 秒，{len(value)} 帧"
                for key, value in self.endpoints.items()))
        except (ValueError, KeyError) as error:
            self._error(error)

    def calculate(self):
        from host import measurements as engine
        try:
            rows = self._window()
            method = self.method.currentData()
            parameters = {'window_seconds': self.window_seconds.value()}
            evidence = list(rows)
            if method == 'static':
                result = engine.static_measurement(rows)
                result['saved_static_segments'] = {key: deepcopy(value['result']) for key, value in self.adc_segments.items()}
                for segment in self.adc_segments.values():
                    evidence.extend(segment['records'])
            elif method == 'encoder':
                if not {'start', 'end'}.issubset(self.endpoints):
                    raise ValueError('请记录起点与终点两个稳定静态段。')
                start, end = self.endpoints['start'], self.endpoints['end']
                if self.records[0]['elapsed_s'] > start[0]['elapsed_s']:
                    raise ValueError('起点至终点的完整历史已超出缓存，请重新采集。')
                evidence = [deepcopy(row) for row in self.records if start[0]['elapsed_s'] <= row['elapsed_s'] <= end[-1]['elapsed_s']]
                self._validate_continuity(evidence)
                parameters.update(configured_cpr=self.configured_cpr.value(),
                                  encoder_sign=self.encoder_sign.currentData(),
                                  physical_angle_deg=self.physical_angle.value())
                result = engine.encoder_measurement(start, end, all_records=evidence,
                    configured_cpr=parameters['configured_cpr'], encoder_sign=parameters['encoder_sign'],
                    physical_angle_deg=parameters['physical_angle_deg'])
            elif method == 'scan':
                parameters['jump_threshold_codes'] = self.jump_threshold.value()
                result = engine.blind_zone_scan(rows, jump_threshold_codes=parameters['jump_threshold_codes'])
            elif method == 'decay':
                noise = self.noise_std.text().strip()
                noise_value = float(noise) if noise else None
                if noise_value is not None and (not math.isfinite(noise_value) or noise_value < 0):
                    raise ValueError('静态角度噪声必须为有限的非负值。')
                parameters.update(arm_fixed_tolerance_deg=self.arm_tolerance.value(), noise_std_deg=noise_value)
                result = engine.free_decay(rows, arm_fixed_tolerance_deg=parameters['arm_fixed_tolerance_deg'], noise_std_deg=noise_value)
            else:
                result = engine.jog_response(rows)
            self.result = deepcopy(result)
            self.result_parameters = parameters
            # Preserve all endpoint / calibration evidence; remove only exact duplicate frames.
            seen = set()
            self.evidence_records = []
            for row in sorted(evidence, key=lambda value: value['elapsed_s']):
                key = (row['elapsed_s'], row['sequence'])
                if key not in seen:
                    seen.add(key)
                    self.evidence_records.append(deepcopy(row))
            lines = [SOURCE_NAMES.get(self.source, self.source)]
            if 'provenance' in result:
                lines.append('计数来源：' + ('固件直接发送' if result['provenance'] == 'direct' else '由原始 Q10 角度与固件系数逆算'))
            for key, title in RESULT_LABELS.items():
                if key not in result:
                    continue
                value = result[key]
                if isinstance(value, bool):
                    rendered = '是' if value else '否'
                elif value is None:
                    rendered = '无法判断'
                elif isinstance(value, float):
                    rendered = f'{value:.6g}'
                elif isinstance(value, list):
                    rendered = ' ～ '.join(f'{item:.6g}' for item in value)
                else:
                    rendered = str(value)
                lines.append(f'{title}：{rendered}')
            if 'suspected_segments' in result:
                lines.append(f"疑似异常采样点：{len(result['suspected_segments'])} 个（完整列表保存在报告中）")
            if self.adc_segments and method == 'static':
                lines.append(self.adc_status.text())
            if result.get('quality_warning'):
                lines.extend(['', '结果条件与误差：' + result['quality_warning']])
            self.result_view.setPlainText('\n'.join(lines))
            self._update_enabled()
        except (ValueError, KeyError, TypeError) as error:
            self._error(error)

    def _metadata(self):
        metadata = {key: entry.text().strip() or None for key, entry in self.manual_fields.items()}
        for key in ('arm_length_mm', 'pendulum_length_mm', 'mass_g', 'center_of_mass_mm'):
            if metadata[key] is not None:
                number = float(metadata[key])
                if not math.isfinite(number) or number <= 0:
                    raise ValueError('机械尺寸和质量必须为有限正数；未知请留空。')
                metadata[key] = number
        return metadata

    def _write_report(self, result, records, parameters, source, metadata, static_segments=None):
        directory = None
        directory_created = False
        try:
            report = {'schema_version': 1, 'created_utc': datetime.now(timezone.utc).isoformat(),
                      'source': source, 'result': result, 'parameters': parameters,
                      'metadata': metadata, 'evidence_samples': len(records),
                      'adc_static_segments': static_segments or {}}
            # Validate the whole result before creating any output.
            json.dumps(report, ensure_ascii=False, allow_nan=False)
            directory = self.output_root / 'measurements' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            directory.mkdir(parents=True, exist_ok=False)
            directory_created = True
            if records:
                csv_path = directory / 'samples.csv'
                fields = sorted({key for row in records for key in row})
                with csv_path.open('w', encoding='utf-8-sig', newline='') as stream:
                    writer = csv.DictWriter(stream, fieldnames=fields)
                    writer.writeheader()
                    writer.writerows(records)
                report['evidence_file'] = 'samples.csv'
                report['evidence_sha256'] = hashlib.sha256(csv_path.read_bytes()).hexdigest()
            path = directory / 'report.json'
            path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
            self.save_status.setText('已保存：' + str(path))
            return path
        except (ValueError, TypeError, OSError) as error:
            # Remove only the two files created by this attempt, never saved reports.
            if directory_created and directory is not None and directory.exists():
                try:
                    for name in ('samples.csv', 'report.json'):
                        (directory / name).unlink(missing_ok=True)
                    directory.rmdir()
                except OSError:
                    pass
            self.save_status.setText('保存失败：' + str(error))
            return None

    def save_report(self):
        if self.result is None:
            return None
        try:
            metadata = self._metadata()
        except ValueError as error:
            self.save_status.setText('保存失败：' + str(error))
            return None
        segments = {key: {'result': value['result'],
                    'time_range': [value['records'][0]['elapsed_s'], value['records'][-1]['elapsed_s']],
                    'records': deepcopy(value['records'])}
                    for key, value in self.adc_segments.items()}
        return self._write_report(self.result, self.evidence_records, self.result_parameters,
                                  self.source, metadata, segments)

    def save_manual_report(self):
        try:
            metadata = self._metadata()
            measurements = {key: metadata[key] for key in
                            ('arm_length_mm', 'pendulum_length_mm', 'mass_g', 'center_of_mass_mm')}
            if not any(value is not None for value in measurements.values()):
                raise ValueError('请至少填入一项已测的机械尺寸或质量，未知项保持空白。')
        except ValueError as error:
            self.save_status.setText('保存失败：' + str(error))
            return None
        result = {'method': 'manual_mechanical', 'source': 'manual',
                  'evidence': 'manual_observation', 'measurements': measurements,
                  'quality_warning': '人工记录，未由遥测验证。请保留测量工具、精度、条件与记录人信息；空白项表示未知。'}
        return self._write_report(result, [], {}, 'manual', metadata)
