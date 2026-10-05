"""J280 measurement desk. All widgets present operator data and actions."""
import argparse
from collections import deque
import math
from pathlib import Path
import sys
import time

import numpy as np

from PySide6.QtCore import Qt, QSignalBlocker, QTimer
from PySide6.QtGui import QFont, QFontDatabase, QKeySequence, QShortcut
from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox, QDialog,
    QDialogButtonBox, QDoubleSpinBox, QFileDialog, QFormLayout, QFrame,
    QGridLayout, QHBoxLayout, QLabel, QLineEdit, QMainWindow, QMessageBox,
    QPushButton, QScrollArea, QSpinBox, QSplitter, QTabWidget, QTableWidget,
    QTableWidgetItem, QTextEdit, QVBoxLayout, QWidget, QHeaderView, QStackedWidget)
import pyqtgraph as pg
from serial.tools import list_ports

from host.core import Metrics, Replay, demo_record
from host.recording import AsyncSessionRecorder
from host.transport import SerialWorker
from host.serial_config import SerialConfig
from host.settings import SettingsDialog, load_preferences, save_preferences, load_log_preferences
from host.serial_ui import SerialPanel
from host.operation_log import OperationLog
from host.diagnostics import (diagnostic_text, fault_names, recovery_hint,
    adc_diagnostic_text, motor_diagnostic_text, has_measurement_diagnostics,
    measurement_block_reason, adc_detail_text, motor_test_text, MOTOR_TEST_FIRMWARE_MIN,
    motion_limit_reason, motion_limits_text, adc_contribution_title, JOG_MEASUREMENT_FIRMWARE_MIN,
    handover_control_text)
from host.version import HOST_VERSION, PROJECT_VERSION
from host.handover_status import HANDOVER_STEPS, HandoverStatus
from host.adc_status import in_blind_zone

STATE_NAMES = {0: '待机', 1: '自动起摆', 2: '直立平衡', 3: '故障', 4: '限时点动', 5: '定 PWM 测试'}
MOTION_COMMANDS = 'GHFBJKLM'
EXTENSIONS = [
    ('device_time_ms', '设备时间', 'ms'), ('encoder_count', '编码器累计计数', 'count'),
    ('target_arm_deg', '摆臂目标位置', '°'), ('target_speed_rad_s', '摆臂目标速度', 'rad/s'),
    ('observer_theta_deg', '观测器摆杆角度', '°'),
    ('observer_confidence', '观测置信度', '0–1'), ('energy_error', '归一化能量误差', '1'),
    ('parameter_version', '参数版本', 'ID'), ('control_loop_us', '控制计算耗时', 'μs'),
    ('sensor_flags', '传感器诊断标志', 'bits'), ('adc_down', '下垂标定码', 'code'),
    ('adc_up', '直立标定码', 'code'), ('adc_raw', 'ADC 原始采样码', 'code'),
    ('adc_window_min', 'ADC 窗口最小码', 'code'), ('adc_window_max', 'ADC 窗口最大码', 'code'),
    ('adc_mean_q4', 'ADC 均值 Q4', 'code × 16'),
    ('adc_control_q4', 'ADC 去尖峰控制均值 Q4', 'code × 16'),
    ('motor_command_permille', '门控后电机命令（非实测）', '‰'),
    ('first_fault', 'FPGA 锁存首次故障', 'bits'),
    ('firmware_version', 'FPGA 固件版本', 'ID'), ('sample_counter', '采样窗口计数', 'count'),
    ('adc_quality_reason', '当前 ADC 质量原因', 'bits'),
    ('sensor_fault_reason', '锁存传感器故障原因', 'bits'),
    ('adc_fault_window_min', '故障窗口最小原码', 'code'),
    ('adc_fault_window_max', '故障窗口最大原码', 'code'),
    ('adc_fault_mean_q4', '故障窗口均值 Q4', 'code × 16'),
    ('adc_fault_time_ms', '首次传感器故障观测时刻', 'ms'),
    ('adc_fault_sample_counter', '首次传感器故障观测时窗口计数', 'count'),
    ('motor_test_status', '定 PWM 测试状态', 'ID'),
    ('motor_test_delta', '电机测试相对编码器位移', 'count'),
    ('h_integral_q8', '居中积分修正 Q8', '‰ × 256'),
    ('h_capture_arm_q10', '接管摆臂参考 Q10', 'rad × 1024'),
    ('h_control_age_ms', '位置渐入计时', 'ms'),
    ('h_control_flags', 'H 居中控制标志', 'bits')]
HANDOVER_TOOLTIP = ('居中积分修正是本次控制命令使用的积分项，非独立PWM或实测力矩；'
    'Q8除以256为‰，截断精度为1/256‰。参考位置按名义1040计数/圈换算。'
    '位置渐入计时最大512ms，不是H总运行时长。标志bit0为H运行，bit1为本样本允许积分更新，'
    'bit2为抗饱和冻结，bit3为本次积分已到±100‰限幅；允许更新不等于实际发生非零增量。'
    '停止、故障或非H时这些值清零，不保留上次接管结果。')
ADC_DETAIL_COLUMNS = [
    ('reference_q4', '上一完整窗均值', 'code × 16'),
    ('filtered_min', '滤后最小码', 'code'), ('filtered_max', '滤后最大码', 'code'),
    ('contribution_min', '贡献最小码', 'code'), ('contribution_max', '贡献最大码', 'code'),
    ('max_step', '最大相邻阶跃', 'code'), ('outlier_count', '滤后离群数', 'count'),
    ('outlier_longest', '滤后连续离群最长', '0.2 μs / count'),
    ('first_outlier_index', '首离群中值输出索引', 'index'),
    ('first_outlier_edge_ticks', '首离群距观测边沿', '20 ns / tick'),
    ('max_step_index', '最大阶跃中值输出索引', 'index'),
    ('max_step_edge_ticks', '最大阶跃距观测边沿', '20 ns / tick'),
    ('edge_outlier_count', '近驱动边沿离群数', 'count'),
    ('conversion_count', '中值输出数', 'count'),
    ('detail_flags', '转换统计标志（悬停查看）', 'bits'),
    ('filtered_sum', '连续中值总和', 'code'),
]
ADC_DETAIL_PREFIXES = {}
for _prefix, _title in (('adc_', '当前'), ('adc_fault_', '首次异常')):
    EXTENSIONS.extend((_prefix + key, _title + ' · ' + name, unit)
                      for key, name, unit in ADC_DETAIL_COLUMNS)
    ADC_DETAIL_PREFIXES.update((_prefix + key, _prefix) for key, _, _ in ADC_DETAIL_COLUMNS)

STYLE = """
QWidget { background:#101923; color:#dce6ef; font-family:'Microsoft YaHei UI'; font-size:13px; }
QMainWindow { background:#101923; }
QLabel#title { font-size:25px; font-weight:700; color:#f1f7fc; }
QLabel#subtitle { color:#8da4b8; }
QLabel#badge { background:#173a39; color:#74e6ce; padding:8px 14px; border-radius:7px; }
QFrame#panel { background:#172330; border:1px solid #2b3c4f; border-radius:10px; }
QFrame#panel QLabel { background:transparent; }
QLabel#value { font-size:27px; color:#70e0c8; font-weight:600; }
QLabel#muted { color:#8da4b8; }
QPushButton { background:#25394c; border:1px solid #385168; padding:9px 12px; border-radius:6px; }
QPushButton:hover { background:#304c62; }
QPushButton:disabled { color:#607384; background:#1c2a38; border-color:#2b3c4f; }
QPushButton#primary { background:#127567; color:white; border-color:#299884; }
QPushButton#stop { background:#a13d48; color:white; border-color:#d66b77; font-weight:600; }
QPushButton#primary:disabled, QPushButton#stop:disabled { color:#607384; background:#1c2a38; border-color:#2b3c4f; }
QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox, QTextEdit, QPlainTextEdit { background:#111e2a; border:1px solid #3b5166; padding:6px; border-radius:5px; }
QComboBox QAbstractItemView { background:#172330; selection-background-color:#127567; }
QTabWidget::pane { border:1px solid #2b3c4f; border-radius:8px; }
QTabBar::tab { background:#1c2a38; padding:10px 22px; color:#9db0c3; }
QTabBar::tab:selected { background:#254537; color:#80ecd2; }
QHeaderView::section { background:#243547; color:#b8ccd9; border:0; padding:8px; }
QTableWidget { background:#14202d; gridline-color:#2c3d50; border:0; }
QTableWidget::item { padding:6px; }
QCheckBox { spacing:7px; }
QScrollArea { border:0; }
"""


def label(text, name=None):
    widget = QLabel(text)
    if name:
        widget.setObjectName(name)
    widget.setWordWrap(True)
    return widget


def button(text, callback, name=None):
    widget = QPushButton(text)
    if name:
        widget.setObjectName(name)
    widget.clicked.connect(callback)
    return widget


def fmt(value, precision=2):
    return '—' if value is None else f'{value:.{precision}f}'


class Window(QMainWindow):
    PLOT_INTERVAL_MS = 100

    def __init__(self, output_root=None):
        super().__init__()
        self.setWindowTitle('J280 姿态控制 · 测量工作站')
        self.resize(1440, 960)
        self.setMinimumSize(1000, 750)
        self.output_root = Path(output_root or default_output())
        self.preferences_path = self.output_root / 'host_preferences.json'
        self.serial_config, self.serial_mode, self.baud_presets = load_preferences(self.preferences_path)
        self.developer_page = None
        self.operation_log = None
        self.runtime_log = None
        self.runtime_error = None
        self.runtime_path = None
        self.link_fresh = None
        self.connected_monotonic = None
        self.operator_checks_snapshot = None
        self.control_availability_snapshot = None
        self.exit_pending = False
        self.log_preferences = load_log_preferences(self.preferences_path)
        self.logging_error = None
        self.measurement_panel = None
        self.worker = None
        self.serial_ready = False
        self.source = 'idle'
        self.latest = None
        self.last_received = 0.0
        self.metrics = Metrics()
        self.history = deque(maxlen=15000)
        self.plot_dirty = True
        self.plotted_window_seconds = None
        self.rate_history = deque()
        self.recorder = None
        self.closing_recorder = None
        self.stats = {}
        self.replay_rows = []
        self.replay_index = 0
        self.replay_anchor = 0.0
        self.replay_cursor = 0.0
        self.recovery = None
        self.recovery_start = None
        self.recovery_result = None
        self.event_history = deque(maxlen=300)
        self.plot_curves = {}
        self.cards = {}
        self.command_buttons = {}
        self.last_transition = None
        self.trace_busy = False
        self.trace_export_status = 'idle'
        self.handover = HandoverStatus()
        self.build_ui()
        self.refresh_ports()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(40)
        # Receiving, logging and command gates keep their own cadence. Plotting
        # is a bounded display task, never part of a telemetry/command callback.
        self.plot_timer = QTimer(self)
        self.plot_timer.timeout.connect(self.refresh_plots)
        self.plot_timer.start(self.PLOT_INTERVAL_MS)
        self.shortcut = QShortcut(QKeySequence('Esc'), self)
        self.shortcut.activated.connect(lambda: self.send_command('S'))

    def build_ui(self):
        base = QWidget()
        self.setCentralWidget(base)
        layout = QVBoxLayout(base)
        layout.setContentsMargins(22, 18, 22, 14)
        title_row = QHBoxLayout()
        titles = QVBoxLayout()
        heading = label('J280  姿态控制测量工作站', 'title')
        heading.setWordWrap(False)
        titles.addWidget(heading)
        subtitle = label('实时采集  /  起摆与平衡  /  实验记录', 'subtitle')
        subtitle.setWordWrap(False)
        titles.addWidget(subtitle)
        self.heading, self.subtitle = heading, subtitle
        title_row.addLayout(titles)
        title_row.addStretch()
        self.source_badge = label('未连接', 'badge')
        self.source_badge.setWordWrap(False)
        self.source_badge.setMaximumHeight(40)
        title_row.addWidget(self.source_badge)
        self.back_button = button('返回主界面', self.show_main)
        self.back_button.hide()
        title_row.addWidget(self.back_button)
        self.settings_button = button('设置', self.open_settings)
        title_row.addWidget(self.settings_button)
        self.global_stop = button('停止 S / Esc', lambda: self.send_command('S'), 'stop')
        title_row.addWidget(self.global_stop)
        layout.addLayout(title_row)
        self.handover_steps = label(HANDOVER_STEPS)
        layout.addWidget(self.handover_steps)
        self.handover_status_label = label('H 接管：等待连接')
        self.handover_status_label.setToolTip(
            '已排队或已发送均不代表接管。仅依据新鲜实时遥测显示 H 运行；'
            '设备最近启动结果可能属于先前请求，不能对应本次 H。')
        layout.addWidget(self.handover_status_label)
        self.pages = QStackedWidget()
        self.main_page = QWidget()
        main_layout = QVBoxLayout(self.main_page)
        main_layout.setContentsMargins(0, 0, 0, 0)
        self.pages.addWidget(self.main_page)
        layout.addWidget(self.pages, 1)
        splitter = QSplitter(Qt.Horizontal)
        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setMinimumWidth(250)
        left_scroll.setMaximumWidth(320)
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 8, 12, 0)
        left_layout.addWidget(label('连接设备'))
        self.ports = QComboBox()
        self.ports.setEditable(True)
        left_layout.addWidget(self.ports)
        self.refresh_button = button('刷新串口', self.refresh_ports)
        left_layout.addWidget(self.refresh_button)
        self.serial_config_label = label('', 'muted')
        left_layout.addWidget(self.serial_config_label)
        self.connect_button = button('连接串口', self.connect_serial, 'primary')
        left_layout.addWidget(self.connect_button)
        self.stop_on_close = QCheckBox('断开时发送停止 S')
        self.stop_on_close.setChecked(True)
        left_layout.addWidget(self.stop_on_close)
        self.demo_button = button('查看演示数据', self.start_demo)
        left_layout.addWidget(self.demo_button)
        self.replay_button = button('打开 CSV 回放', self.open_replay)
        left_layout.addWidget(self.replay_button)
        left_layout.addSpacing(18)
        left_layout.addWidget(label('开发板控制'))
        self.board_controls = QWidget()
        board_layout = QVBoxLayout(self.board_controls)
        board_layout.setContentsMargins(0, 0, 0, 0)
        self.jog_check = QCheckBox('已确认点动空间，准备按 SW3')
        board_layout.addWidget(self.jog_check)
        self.direction_check = QCheckBox('已核验方向、盲区与运动范围')
        board_layout.addWidget(self.direction_check)
        for command, text in [('D', 'SW1 · 下垂点标定'), ('U', 'SW4 · 直立点标定'),
                              ('G', 'SW2 · 自动起摆与平衡'), ('H', 'H · 手扶近直立接管'),
                              ('F', '正向点动 · 10% / 150 ms'),
                              ('B', '反向点动 · 10% / 150 ms'), ('R', '清除故障 R')]:
            b = button(text, lambda checked=False, c=command: self.send_command(c),
                       'primary' if command == 'G' else None)
            b.setToolTip(f'发送单字节 {command}；以设备遥测确认执行结果')
            board_layout.addWidget(b)
            self.command_buttons[command] = b
            if command == 'G':
                self.start_gate = label('起摆条件：等待连接', 'muted')
                board_layout.addWidget(self.start_gate)
                board_layout.addWidget(label('未勾选不等于未标定；完成实际方向、盲区与运动范围核验后再勾选。', 'muted'))
            elif command == 'H':
                self.capture_gate = label('直立接管条件：等待连接', 'muted')
                board_layout.addWidget(self.capture_gate)
                board_layout.addWidget(label('H 用于手扶至近直立、低速后接管；自然下垂时使用 G 自动起摆。', 'muted'))
        self.stop_button = button('SW3 · 停止 S / Esc', lambda: self.send_command('S'), 'stop')
        board_layout.addWidget(self.stop_button)
        self.trace_button = button('导出连续控制记录', lambda: self.send_command('T'))
        self.command_buttons['T'] = self.trace_button
        board_layout.addWidget(self.trace_button)
        self.trace_progress = label('H 停止后可导出最近的连续控制记录。', 'muted')
        self.trace_progress.setToolTip('历史记录单独保存为 JSON 和 CSV，不更新实时曲线。导出完整性、采样间断和磁盘保存结果分别显示。')
        board_layout.addWidget(self.trace_progress)
        board_layout.addWidget(label('定 PWM 电机测试'))
        board_layout.addWidget(label('最多 0.5 s / 相对位移 128 count；启动后 0.3 s 仍无编码器变化则停止。固定 PWM 不保证转速。', 'subtitle'))
        self.motor_test_level = QComboBox()
        self.motor_test_level.addItem('15% PWM', 150)
        self.motor_test_level.addItem('22% PWM', 220)
        board_layout.addWidget(self.motor_test_level)
        self.motor_test_buttons = {}
        for forward, text in ((True, '正向测试'), (False, '反向测试')):
            b = button(text, lambda checked=False, f=forward: self.send_motor_test(f))
            board_layout.addWidget(b)
            self.motor_test_buttons[forward] = b
        self.motor_test_gate = label('电机测试条件：等待连接', 'muted')
        board_layout.addWidget(self.motor_test_gate)
        self.motor_test_result = label(motor_test_text(None), 'muted')
        self.motor_test_result.setToolTip('位移为最近一次成功接纳测试起点的原始编码器相对计数；拒收请求不建立新起点。停止后继续更新，便于观察滑行。输出为零不代表机械静止。摆臂速度的编码器每转计数尚待实测核验。')
        board_layout.addWidget(self.motor_test_result)
        board_layout.addWidget(label('自然下垂记录 D；停机扶至直立记录 U。首次核验方向可先确认点动安全，再用 F / B（10%，150 ms）；核验完成后勾选方向项并起摆。SW5 为板上系统复位。', 'muted'))
        left_layout.addWidget(self.board_controls)
        left_layout.addStretch()
        self.health = label('等待数据', 'muted')
        left_layout.addWidget(self.health)
        left_scroll.setWidget(left)
        splitter.addWidget(left_scroll)
        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(8, 8, 0, 0)
        card_row = QHBoxLayout()
        for key, text, unit in [('theta_deg', '摆杆角度', '°'), ('arm_deg', '摆臂角度', '°'),
                               ('command_permille', '请求输出', '‰'), ('state', '运行状态', '')]:
            card = QFrame()
            card.setObjectName('panel')
            box = QVBoxLayout(card)
            box.addWidget(label(text, 'muted'))
            value = label('—', 'value')
            box.addWidget(value)
            detail = label(unit if unit else '等待遥测', 'muted')
            box.addWidget(detail)
            self.cards[key] = (value, detail)
            card_row.addWidget(card)
        right_layout.addLayout(card_row)
        self.fault_label = label('标定：—   故障：—   ADC：—', 'subtitle')
        right_layout.addWidget(self.fault_label)
        self.diagnostic_label = label('启动诊断：等待 FPGA 遥测', 'subtitle')
        right_layout.addWidget(self.diagnostic_label)
        self.adc_diagnostic_label = label(adc_diagnostic_text(None), 'subtitle')
        right_layout.addWidget(self.adc_diagnostic_label)
        self.motor_diagnostic_label = label(motor_diagnostic_text(None), 'subtitle')
        right_layout.addWidget(self.motor_diagnostic_label)
        self.tabs = QTabWidget()
        self.tabs.addTab(self.build_plots(), '实时曲线')
        self.tabs.addTab(self.build_experiments(), '实验与记录')
        self.extension_widget = self.build_extensions()
        self.extension_widget.setParent(self)
        self.extension_widget.hide()
        self.serial_panel = SerialPanel(self.output_root, self.send_raw, self)
        self.tabs.addTab(self.serial_panel, '串口收发')
        right_layout.addWidget(self.tabs)
        splitter.addWidget(right)
        splitter.setStretchFactor(1, 1)
        main_layout.addWidget(splitter)
        self.message = label('连接后自动接收遥测；查看演示和回放可在无板卡时使用。', 'muted')
        layout.addWidget(self.message)
        self.log_status = label('日志保留：未开启', 'muted')
        self.log_status.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.log_status)
        self.runtime_status = label('FPGA 运行日志：项目连接后自动记录', 'muted')
        self.runtime_status.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.runtime_status)
        self.direction_check.toggled.connect(self.update_control_availability)
        self.jog_check.toggled.connect(self.update_control_availability)
        self.motor_test_level.currentIndexChanged.connect(self.update_control_availability)

    def open_settings(self):
        dialog = SettingsDialog(self.serial_config, self.serial_mode, self.baud_presets,
                                connected=self.worker is not None, parent=self,
                                log_preferences=self.log_preferences)
        result = dialog.exec()
        if dialog.developer_requested:
            self.open_developer()
        elif result == QDialog.Accepted:
            try:
                save_preferences(self.preferences_path, dialog.selected_config,
                                 dialog.selected_mode, dialog.presets, dialog.selected_log_preferences)
            except OSError as error:
                self.message.setText(f'设置保存失败：{error}')
                return
            if not self.worker:
                self.serial_config = dialog.selected_config
                self.serial_mode = dialog.selected_mode
                self.baud_presets = dialog.presets
                self.direction_check.setChecked(False)
            if self.source == 'serial':
                self.control_log('logging_settings', enabled=dict(dialog.selected_log_preferences))
            self.log_preferences = dialog.selected_log_preferences
            if self.logging_error or not any(self.log_preferences.values()):
                self.close_operation_log()
            self.logging_error = None
            if self.source == 'serial':
                self.control_log('logging_enabled', enabled=dict(self.log_preferences))
            self.message.setText('设置已保存；日志选项立即生效，串口参数下次连接生效。')
        self.update_display()

    def open_developer(self):
        if self.developer_page is None:
            from host.measurement_ui import MeasurementPanel
            self.developer_page = QWidget()
            layout = QVBoxLayout(self.developer_page)
            layout.addWidget(label('采集并分析项目开发所需的测量数据。控制开发板请返回主界面；切换页面时采集和日志继续。', 'muted'))
            tabs = QTabWidget()
            self.measurement_panel = MeasurementPanel(self.output_root, self.developer_page)
            self.measurement_panel.reset(self.measurement_source())
            for record in self.history:
                self.measurement_panel.ingest(record)
            tabs.addTab(self.measurement_panel, '实测与辨识')
            tabs.addTab(self.experiment_analysis, '实验指标')
            self.experiment_analysis.show()
            tabs.addTab(self.extension_widget, '扩展遥测')
            self.extension_widget.show()
            layout.addWidget(tabs)
            self.pages.addWidget(self.developer_page)
        self.measurement_panel.set_context(self.measurement_source(), self.is_fresh())
        self.pages.setCurrentWidget(self.developer_page)
        self.heading.setText('J280  开发者模式')
        self.subtitle.setText('实测与辨识  /  实验指标  /  扩展遥测')
        self.back_button.show()

    def show_main(self):
        self.pages.setCurrentWidget(self.main_page)
        self.heading.setText('J280  姿态控制测量工作站')
        self.subtitle.setText('实时采集  /  起摆与平衡  /  实验记录')
        self.back_button.hide()

    def measurement_source(self):
        if self.source == 'serial' and (self.serial_mode != 'project' or not self.serial_config.is_project_default):
            return 'idle'
        return self.source

    def send_raw(self, data):
        accepted = bool(self.serial_mode == 'raw' and self.source == 'serial' and
            self.serial_ready and self.worker and self.worker.send_raw(data))
        # Raw byte attempts are never presented as a completed transmission.
        self.serial_log_event('send_queued' if accepted else 'send_rejected',
                              requested_hex=bytes(data).hex(), requested_bytes=len(data))
        return accepted

    def serial_log_event(self, event, **details):
        if self.source != 'serial' or not self.log_preferences['serial'] or self.logging_error:
            return
        try:
            self.ensure_operation_log().write_serial_event(event, **details)
        except (OSError, ValueError) as error:
            self.logging_failed(error)

    def ensure_operation_log(self):
        if self.operation_log is None:
            self.operation_log = OperationLog(self.output_root, {
                'host_version': HOST_VERSION, 'compatible_project_version': PROJECT_VERSION,
                'source': self.source, 'mode': self.serial_mode,
                'port': getattr(self.worker, 'port_name', self.ports.currentText()),
                'serial_config': self.serial_config.as_dict(),
                'enabled_at_start': dict(self.log_preferences),
                'command_acknowledgements': False})
        return self.operation_log

    def logging_failed(self, error):
        self.logging_error = str(error)
        if self.operation_log:
            try:
                self.operation_log.close(timeout=0)
            except OSError:
                pass
        self.operation_log = None
        self.log_status.setText(f'日志异常：{error}；本次日志可能不完整。采集和控制继续，请在设置中重新保存日志选项以重试。')

    def control_log(self, event, **details):
        self.runtime_event(event, **details)
        if self.source != 'serial' or not self.log_preferences['control'] or self.logging_error:
            return
        context = {key: self.latest.get(key) for key in ('sequence', 'state', 'calibrated', 'fault')} if self.latest else {}
        try:
            self.ensure_operation_log().write_control(event, source=self.source, telemetry=context, **details)
        except (OSError, ValueError) as error:
            self.logging_failed(error)

    def ensure_runtime_log(self):
        if self.runtime_log is None:
            self.runtime_log = OperationLog(self.output_root, {
                'host_version': HOST_VERSION, 'compatible_project_version': PROJECT_VERSION,
                'source': 'serial', 'mode': self.serial_mode,
                'port': getattr(self.worker, 'port_name', self.ports.currentText()),
                'serial_config': self.serial_config.as_dict(),
                'command_acknowledgements': False,
                'purpose': 'FPGA telemetry and control evidence'}, directory_name='fpga_logs')
        return self.runtime_log

    def runtime_failed(self, error):
        self.runtime_error = str(error)
        logger, self.runtime_log = self.runtime_log, None
        if logger:
            self.runtime_path = logger.directory
            try:
                logger.close(timeout=0)
            except OSError:
                pass
        self.runtime_status.setText(f'FPGA 运行日志不完整：{error}；排除磁盘问题后重新连接。')

    def runtime_event(self, event, **details):
        if (self.source != 'serial' or self.serial_mode != 'project' or
                not self.serial_config.is_project_default or self.runtime_error):
            return
        context = {key: self.latest[key] for key in
                   ('sequence', 'state', 'calibrated', 'fault', 'diagnostic_status',
                    'first_fault', 'sensor_flags', 'firmware_version') if key in self.latest} if self.latest else {}
        try:
            self.ensure_runtime_log().write_control(event, telemetry=context, **details)
        except (OSError, ValueError) as error:
            self.runtime_failed(error)

    def runtime_sample(self, record):
        if (self.source != 'serial' or self.serial_mode != 'project' or
                not self.serial_config.is_project_default or self.runtime_error):
            return
        try:
            self.ensure_runtime_log().write_telemetry(record)
        except (OSError, ValueError) as error:
            self.runtime_failed(error)

    def close_runtime_log(self):
        logger, self.runtime_log = self.runtime_log, None
        if logger:
            try:
                logger.close()
            except OSError as error:
                self.runtime_failed(error)
            self.runtime_path = logger.directory

    def wire_activity(self, activity):
        if self.sender() is not self.worker or self.source != 'serial':
            return
        if activity.get('direction') == 'TX':
            tx = dict(activity)
            tx['hex'] = tx.pop('data').hex()
            self.runtime_event('serial_tx', **tx)
        if not self.log_preferences['serial'] or self.logging_error:
            return
        details = dict(activity)
        direction, data = details.pop('direction'), details.pop('data')
        try:
            self.ensure_operation_log().write_serial(direction, data, **details)
        except (OSError, ValueError) as error:
            self.logging_failed(error)

    def close_operation_log(self):
        logger, self.operation_log = self.operation_log, None
        if logger:
            try:
                logger.close()
            except OSError as error:
                self.logging_failed(error)

    def receive_raw(self, data):
        if self.source == 'serial' and self.sender() is self.worker:
            self.serial_panel.ingest(data)

    def raw_sent(self, data):
        if self.sender() is self.worker:
            self.serial_panel.sent(data)

    def build_plots(self):
        widget = QWidget()
        box = QVBoxLayout(widget)
        toolbar = QHBoxLayout()
        toolbar.addWidget(label('显示窗口'))
        self.window_seconds = QSpinBox()
        self.window_seconds.setRange(5, 300)
        self.window_seconds.setValue(20)
        self.window_seconds.setSuffix(' s')
        toolbar.addWidget(self.window_seconds)
        self.pause_plots = QCheckBox('冻结曲线')
        toolbar.addWidget(self.pause_plots)
        self.replay_pause = QCheckBox('暂停回放')
        toolbar.addWidget(self.replay_pause)
        self.replay_speed = QComboBox()
        self.replay_speed.addItems(['0.5×', '1×', '2×', '4×'])
        self.replay_speed.setCurrentIndex(1)
        toolbar.addWidget(self.replay_speed)
        toolbar.addStretch()
        toolbar.addWidget(button('保存曲线截图', self.save_screenshot))
        box.addLayout(toolbar)
        plot_scroll = QScrollArea()
        plot_scroll.setWidgetResizable(True)
        plot_content = QWidget()
        grid = QGridLayout(plot_content)
        plot_scroll.setWidget(plot_content)
        pg.setConfigOptions(antialias=False, background='#14202d', foreground='#b9cddd')
        definitions = [
            ('姿态与位置', '°', [('theta_deg', '摆杆', '#6de0c7'), ('arm_deg', '摆臂', '#77abff')]),
            ('角速度', 'rad/s', [('omega_rad_s', '摆杆', '#6de0c7'), ('arm_speed_rad_s', '摆臂', '#77abff')]),
            ('电机命令（非物理输出实测）', '‰', [('command_permille', '请求', '#e9b86e'),
                ('motor_command_permille', '门控后', '#77abff')]),
            ('原始采集', 'code', [('adc', 'ADC', '#be9ef4')])]
        self.plots = []
        for index, (title, unit, series) in enumerate(definitions):
            plot = pg.PlotWidget(title=title)
            plot.setLabel('left', unit)
            plot.setLabel('bottom', '采集时间', units='s')
            # Reserve the tick and caption height even in the smallest window.
            plot.getAxis('bottom').setHeight(48)
            plot.getPlotItem().layout.setContentsMargins(1, 1, 1, 8)
            plot.showGrid(x=True, y=True, alpha=0.13)
            plot.addLegend(offset=(10, 10))
            plot.setMinimumHeight(200)
            for key, name, color in series:
                self.plot_curves[key] = plot.plot(name=name, pen=pg.mkPen(color, width=1),
                                                antialias=False, connect='finite')
            if index == 0:
                for bound in [-2, 2]:
                    plot.addItem(pg.InfiniteLine(bound, angle=0,
                        pen=pg.mkPen('#42635f', style=Qt.DashLine)))
            grid.addWidget(plot, index // 2, index % 2)
            self.plots.append(plot)
        box.addWidget(plot_scroll, 1)
        box.addWidget(label('角速度来自 FPGA 状态估计；电机命令非物理输出实测。虚线为 ±2° 参考带。冻结曲线期间继续采集与记录。', 'muted'))
        return widget

    def build_experiments(self):
        widget = QWidget()
        box = QVBoxLayout(widget)
        row = QHBoxLayout()
        self.record_button = button('开始实验记录', self.toggle_recording, 'primary')
        row.addWidget(self.record_button)
        self.mark_text = QLineEdit()
        self.mark_text.setPlaceholderText('实验事件，例如：第 3 次起摆 / 向右轻推')
        row.addWidget(self.mark_text, 1)
        row.addWidget(button('添加标记', self.mark_event))
        box.addLayout(row)
        self.record_path = label(f'记录目录：{self.output_root}', 'muted')
        self.record_path.setTextInteractionFlags(Qt.TextSelectableByMouse)
        box.addWidget(self.record_path)
        self.experiment_analysis = QWidget(self)
        analysis_box = QVBoxLayout(self.experiment_analysis)
        recover_row = QHBoxLayout()
        recover_row.addWidget(label('恢复判据：角度范围'))
        self.tolerance = QDoubleSpinBox()
        self.tolerance.setRange(0.1, 30)
        self.tolerance.setValue(2)
        self.tolerance.setSuffix(' °')
        recover_row.addWidget(self.tolerance)
        recover_row.addWidget(label('连续保持'))
        self.hold_seconds = QDoubleSpinBox()
        self.hold_seconds.setRange(0.1, 10)
        self.hold_seconds.setValue(0.5)
        self.hold_seconds.setSuffix(' s')
        recover_row.addWidget(self.hold_seconds)
        recover_row.addWidget(button('标记扰动结束', self.mark_recovery))
        recover_row.addStretch()
        analysis_box.addLayout(recover_row)
        self.metric_table = QTableWidget(0, 2)
        self.metric_table.setHorizontalHeaderLabels(['实验指标', '结果'])
        self.metric_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.metric_table.setEditTriggers(QTableWidget.NoEditTriggers)
        analysis_box.addWidget(self.metric_table, 2)
        analysis_box.addWidget(label('平衡误差仅统计已标定、无故障的平衡样本。恢复时间由手动扰动结束标记起算；50 Hz 遥测与接收时间限制时间精度。', 'muted'))
        self.experiment_analysis.hide()
        box.addWidget(label('事件记录'))
        self.events_text = QTextEdit()
        self.events_text.setReadOnly(True)
        box.addWidget(self.events_text, 1)
        box.addWidget(label('实验指标、扰动恢复分析和实测辅助工具可在“设置 → 进入开发者模式”中使用。', 'muted'))
        return widget

    def build_extensions(self):
        widget = QWidget()
        box = QVBoxLayout(widget)
        box.addWidget(label('位置与速度目标、状态观测、在线辨识和运行诊断'))
        box.addWidget(label('设备上报后显示并写入实验记录；未上报的量显示“未提供”。转换统计值可悬停查看解释；近驱动边沿仅表示数字时序相关，不证明模拟干扰。', 'muted'))
        self.ext_table = QTableWidget(len(EXTENSIONS), 3)
        self.ext_table.setHorizontalHeaderLabels(['测量量', '数值', '单位'])
        self.ext_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.ext_table.setEditTriggers(QTableWidget.NoEditTriggers)
        for index, (_, name, unit) in enumerate(EXTENSIONS):
            for col, text in enumerate([name, '未提供', unit]):
                self.ext_table.setItem(index, col, QTableWidgetItem(text))
        box.addWidget(self.ext_table)
        box.addWidget(label('现有套件没有电流、温度和力矩测量接口。现场尺寸、安装相位与人工观察可填写在实验信息中。', 'muted'))
        return widget

    def refresh_ports(self):
        old = self.ports.currentText()
        self.ports.clear()
        for port in list_ports.comports():
            self.ports.addItem(f'{port.device} · {port.description}', port.device)
        if old:
            self.ports.setEditText(old)

    def reset_data(self, source):
        self.close_operation_log()
        self.close_runtime_log()
        self.runtime_error = None
        self.runtime_path = None
        self.link_fresh = None
        self.connected_monotonic = None
        self.operator_checks_snapshot = None
        self.control_availability_snapshot = None
        self.last_received = 0.0
        self.trace_busy = False
        self.trace_export_status = 'idle'
        self.handover.reset()
        self.trace_progress.setText('H 停止后可导出最近的连续控制记录。')
        self.serial_ready = False
        self.source = source
        self.latest = None
        self.history.clear()
        self.plot_dirty = True
        self.plotted_window_seconds = None
        self.rate_history.clear()
        self.metrics = Metrics()
        self.stats = {}
        self.last_transition = None
        # Reconnect resets are one initial snapshot of the new connection,
        # not operator changes emitted into the previous or half-built one.
        with QSignalBlocker(self.direction_check), QSignalBlocker(self.jog_check), QSignalBlocker(self.motor_test_level):
            self.direction_check.setChecked(False)
            self.jog_check.setChecked(False)
            self.motor_test_level.setCurrentIndex(0)
        self.recovery = self.recovery_start = self.recovery_result = None
        self.event_history.clear()
        self.events_text.clear()
        self.pause_plots.setChecked(False)
        self.replay_pause.setChecked(False)
        self.origin = time.monotonic()
        self.last_tick = self.origin
        self.serial_panel.clear()
        if self.measurement_panel:
            self.measurement_panel.reset(self.measurement_source())

    def connect_serial(self):
        if self.worker:
            self.disconnect_source()
            return
        port = self.ports.currentText().split(' · ')[0].strip()
        if not port:
            self.message.setText('请选择或输入串口，例如 COM5。')
            return
        self.finish_recording()
        self.reset_data('serial')
        self.serial_ready = False
        self.worker = SerialWorker(port, self, config=self.serial_config, mode=self.serial_mode,
                                   buffered_delivery=True, trace_directory=self.output_root / 'control_traces')
        self.worker.records.connect(self.receive_serial)
        self.worker.raw_received.connect(self.receive_raw)
        self.worker.raw_sent.connect(self.raw_sent)
        self.worker.wire_activity.connect(self.wire_activity)
        self.worker.status.connect(self.serial_status)
        self.worker.statistics.connect(self.set_stats)
        self.worker.command_sent.connect(self.command_sent)
        self.worker.trace_status.connect(self.receive_trace_status)
        self.worker.failed.connect(self.serial_failed)
        self.worker.delivery_finished.connect(self.worker_finished)
        self.connect_button.setText('正在连接…')
        self.connect_button.setEnabled(False)
        self.control_log('connect_requested')
        self.update_control_availability()
        self.worker.start()

    def serial_status(self, status):
        if self.sender() is not None and self.sender() is not self.worker:
            return
        self.control_log('connection', status=status)
        self.serial_ready = status == 'connected'
        if not self.serial_ready:
            self.handover.reset()
        if not self.serial_ready and self.trace_export_status in ('queued', 'waiting_metadata', 'receiving'):
            self.trace_busy = False
            self.trace_export_status = 'disconnected'
            self.trace_progress.setText('导出未完成：连接已断开。已接收的失败证据将单独保存。')
        if self.serial_ready:
            self.connected_monotonic = time.monotonic()
        self.connect_button.setText('断开串口' if self.serial_ready else '连接串口')
        self.connect_button.setEnabled(True)
        if self.serial_ready:
            self.log_event('connection', f'串口已连接 {self.serial_config.display_label} / {self.serial_mode}')
            self.message.setText('通用收发已连接，可在串口收发页发送文本或 HEX；此模式断开不发送停止字节。' if self.serial_mode == 'raw' else
                '连接后只接收数据。操作命令发送后，请以状态和标定遥测确认结果。' if self.serial_config.is_project_default else
                '当前设置与 J280 固件的 115200 / 8N1 不同，项目命令已禁用。')
        self.update_control_availability()

    def set_stats(self, stats):
        if self.sender() is not None and self.sender() is not self.worker:
            return
        counters = ('crc_errors', 'discarded_bytes', 'unsupported_frames', 'missing_frames',
                    'duplicates', 'resets', 'delivery_overflows')
        if any(stats.get(key, 0) != self.stats.get(key, 0) for key in counters):
            self.runtime_event('communication_diagnostics', counters=stats)
        self.stats = stats

    def serial_failed(self, text):
        if self.sender() is not None and self.sender() is not self.worker:
            return
        self.serial_log_event('serial_error', message=text)
        self.control_log('serial_error', message=text)
        self.message.setText(f'串口异常：{text}。如机构正在运动，请按 SW3。')
        self.log_event('serial_error', text)
        self.serial_ready = False
        self.handover.reset()
        self.update_control_availability()

    def worker_finished(self):
        if self.sender() is not None and self.sender() is not self.worker:
            return
        self.control_log('connection_finished')
        self.close_operation_log()
        self.close_runtime_log()
        self.finish_recording()
        if self.worker:
            self.worker.deleteLater()
        self.worker = None
        self.serial_ready = False
        self.handover.reset()
        self.source = 'idle'
        with QSignalBlocker(self.motor_test_level):
            self.motor_test_level.setCurrentIndex(0)
        self.update_control_availability()
        if self.exit_pending:
            QTimer.singleShot(0, self.close)

    def disconnect_source(self):
        if self.worker:
            self.connect_button.setEnabled(False)
            self.worker.request_close(self.stop_on_close.isChecked())
            self.handover.reset()
            self.control_log('disconnect_requested', stop_requested=self.stop_on_close.isChecked() and
                self.serial_mode == 'project' and self.serial_config.is_project_default)
            self.update_control_availability()
        else:
            self.finish_recording()
            self.source = 'idle'

    def start_demo(self):
        if self.worker:
            return
        self.finish_recording()
        self.reset_data('demo')
        self.demo_sequence = 0
        self.source_badge.setText('演示 · 非实测')
        self.log_event('source', '演示数据，非实物采集')
        self.message.setText('正在显示生成的演示数据；可体验曲线、记录与事件标记。')

    def open_replay(self):
        if self.worker:
            return
        filename, _ = QFileDialog.getOpenFileName(self, '选择遥测 CSV', str(self.output_root), 'CSV (*.csv)')
        if filename:
            self.load_replay(filename)

    def load_replay(self, filename):
        if self.worker:
            self.message.setText('请先断开串口，再打开回放。')
            return False
        try:
            rows = Replay.load(Path(filename))
            if not rows:
                raise ValueError('记录中没有有效样本')
        except (OSError, ValueError, KeyError) as error:
            self.message.setText(f'无法回放：{error}')
            return False
        self.finish_recording()
        self.reset_data('replay')
        self.replay_rows = rows
        self.replay_index = 0
        self.replay_anchor = float(rows[0]['elapsed_s'])
        self.replay_cursor = self.replay_anchor
        self.source_badge.setText('CSV 回放')
        self.message.setText(f'回放：{filename}')
        self.log_event('source', f'回放 {filename}')
        return True

    def is_fresh(self):
        age = time.monotonic() - self.last_received
        return bool(self.latest and math.isfinite(age) and 0 <= age < 1.0)

    def waiting_for_first_frame(self):
        return (self.source == 'serial' and self.serial_mode == 'project' and
                self.serial_ready and self.latest is None and
                self.connected_monotonic is not None and
                0 <= time.monotonic() - self.connected_monotonic < 1.0)

    def record_control_availability(self, reasons=None):
        if (self.source != 'serial' or self.worker is None or self.serial_mode != 'project' or
                not self.serial_config.is_project_default):
            return
        checks = dict(direction_verified=self.direction_check.isChecked(),
                      jog_clearance=self.jog_check.isChecked())
        if checks != self.operator_checks_snapshot:
            self.control_log('operator_checks', **checks, device_confirmed=False,
                             initial_snapshot=self.operator_checks_snapshot is None)
            self.operator_checks_snapshot = dict(checks)
        reasons = reasons if reasons is not None else {
            command: self.command_block_reason(command) for command in MOTION_COMMANDS}
        snapshot = dict(checks, commands={command: dict(available=not reasons[command],
                            reason=reasons[command]) for command in MOTION_COMMANDS},
                        motor_test_permille=self.motor_test_level.currentData())
        if snapshot != self.control_availability_snapshot:
            self.control_log('control_availability', **snapshot, device_confirmed=False,
                             initial_snapshot=self.control_availability_snapshot is None)
            self.control_availability_snapshot = snapshot

    def update_control_availability(self, *_):
        reasons = {command: self.command_block_reason(command)
                   for command in set(self.command_buttons) | set(MOTION_COMMANDS)}
        for command, b in self.command_buttons.items():
            b.setEnabled(not reasons[command])
            b.setToolTip(reasons[command] or f'发送 {command}，执行结果以 FPGA 遥测为准')
        self.start_gate.setText('起摆条件：' + (reasons['G'] or '已满足，可发送 G'))
        self.capture_gate.setText('直立接管条件：' + (reasons['H'] or '已满足，可发送 H；保持近直立、低速'))
        for forward, b in self.motor_test_buttons.items():
            command = self.motor_test_command(forward)
            b.setEnabled(not reasons[command])
            b.setToolTip(reasons[command] or f'发送 {command}；固定 PWM {self.motor_test_level.currentData() / 10:g}%，最多 0.5 s')
        self.motor_test_level.setEnabled(not (self.source == 'serial' and self.worker is not None
            and self.latest and self.latest.get('state') == 5))
        self.motor_test_gate.setText('电机测试条件：' +
            (reasons[self.motor_test_command(True)] or '已满足；准备按 SW3 / S 停止'))
        stop_allowed = self.allowed('S')
        self.stop_button.setEnabled(stop_allowed)
        self.global_stop.setEnabled(stop_allowed)
        self.record_control_availability(reasons)
        self.update_handover_status()

    def update_handover_status(self):
        live = (self.source not in ('demo', 'replay') and self.serial_mode == 'project'
                and self.serial_config.is_project_default)
        connected = bool(self.source == 'serial' and self.serial_ready and self.worker
                         and not self.worker.closing.is_set())
        status, text, severity = self.handover.presentation(
            time.monotonic(), connected=connected, live=live)
        self.handover_status_label.setProperty('handoverStatus', status)
        self.handover_status_label.setText(text)
        color = {'success': '#74e6ce', 'warning': '#ffd184', 'error': '#ff929b',
                 'neutral': '#b8ccd9'}[severity]
        self.handover_status_label.setStyleSheet(
            f'color:{color}; font-weight:700; padding:6px 0;')

    def allowed(self, command):
        return not self.command_block_reason(command)

    def command_block_reason(self, command):
        if command not in ('D', 'U', 'G', 'H', 'S', 'R', 'F', 'B', 'J', 'K', 'L', 'M', 'T'):
            return '未知命令'
        if self.serial_mode != 'project' or not self.serial_config.is_project_default:
            return '请使用 J280 模式及 115200 / 8N1'
        if self.source != 'serial' or not self.serial_ready or not self.worker or self.worker.closing.is_set():
            return '请连接 FPGA 串口'
        if command == 'S':
            return ''
        if not self.is_fresh():
            if self.waiting_for_first_frame():
                return '等待 FPGA 首帧遥测'
            return '等待有效遥测；数据超时，必要时按 SW3 停机'
        state = self.latest.get('state')
        if command == 'T':
            if self.trace_busy:
                return '连续记录正在导出或保存，请等待结果'
            if self.latest.get('firmware_version') not in (0x00020009, 0x00030000):
                return '连续控制记录需要已知 FPGA project-v0.2.9 / project-v0.3.0 固件'
            if state not in (0, 3) or self.latest.get('motor_test_status') == 1:
                return '请先停止，等待待机或故障状态'
            flags = self.latest.get('trace_flags')
            count, capture = self.latest.get('trace_row_count'), self.latest.get('trace_capture_id')
            if (type(flags) is not int or flags & ~15 or type(count) is not int or
                    not 0 <= count <= 4096 or type(capture) is not int or not 0 <= capture <= 65535):
                return '等待完整有效的连续记录状态'
            if flags & 5:
                return '设备正在采集或导出，请先等待冻结'
            return '' if flags & 2 else '尚无冻结记录；完成一次 H 并停止后可导出'
        if command in 'DUR':
            if command in 'DU' and in_blind_zone(self.latest):
                return '盲区内角度不可用，不能标定；扶离盲区并等待测量就绪'
            return '' if state in (0, 3) else '请先停止，等待待机或故障状态'
        if command in MOTION_COMMANDS:
            if self.latest.get('stop_pressed'):
                return '请释放 SW3 停止按键'
            if state != 0:
                return '当前状态：' + STATE_NAMES.get(state, '未知') + '；请先回到待机'
            if self.latest.get('fault') != 0:
                return recovery_hint(self.latest)
            if self.latest.get('calibrated') != 1:
                return '请完成 D 下垂、U 直立两点标定'
            modern = has_measurement_diagnostics(self.latest)
            if command == 'H' and not modern:
                return '直立接管需要 FPGA project-v0.2.0 或更新版本的诊断遥测'
            if command in 'JKLM' and (not modern or self.latest['firmware_version'] < MOTOR_TEST_FIRMWARE_MIN):
                return '定 PWM 电机测试需要 FPGA project-v0.2.3 或更新版本'
            if modern and (command in 'GHJKLM' or
                           self.latest['firmware_version'] >= JOG_MEASUREMENT_FIRMWARE_MIN):
                reason = measurement_block_reason(self.latest)
                if reason:
                    return reason
            reason = motion_limit_reason(self.latest, command)
            if reason:
                return reason
            if command in 'GH' and not self.direction_check.isChecked():
                return '请核验并勾选方向、盲区与运动范围'
            if command == 'H':
                # Match the FPGA's strict Q10 capture envelope. Legacy replay
                # may lack Q10 fields; it remains non-controlling regardless.
                for raw_key, value_key, scale, limit in (
                        ('theta_q10', 'theta_deg', math.pi / 180 * 1024, 268),
                        ('omega_q10', 'omega_rad_s', 1024, 3584)):
                    value = self.latest.get(raw_key)
                    if value is None:
                        value = self.latest.get(value_key)
                        value = value * scale if type(value) in (int, float) else None
                    if type(value) not in (int, float) or not math.isfinite(value) or abs(value) >= limit:
                        return '直立接管需 |摆杆角| < 约15°、|摆杆速度| < 3.5 rad/s'
            if command in 'FBJKLM' and not (self.jog_check.isChecked() or self.direction_check.isChecked()):
                return '请先确认点动运动空间，并准备按 SW3 停止'
        return ''

    def motor_test_command(self, forward):
        return ('J' if forward else 'K') if self.motor_test_level.currentData() == 150 else ('L' if forward else 'M')

    def send_motor_test(self, forward):
        return self.send_command(self.motor_test_command(forward))

    def send_command(self, command):
        if not self.allowed(command):
            reason = self.command_block_reason(command)
            self.message.setText('操作未发送：' + reason)
            self.control_log('command_rejected', command=command, reason=reason)
            self.log_event('command_rejected', f'{command} 未发送：{reason}')
            if command == 'H':
                self.handover.rejected(reason)
                self.update_handover_status()
            return False
        if command == 'T' and hasattr(self.worker, 'set_trace_context'):
            runtime_directory = self.runtime_log.directory if self.runtime_log else self.runtime_path
            self.worker.set_trace_context(dict(
                runtime_log_directory=str(runtime_directory) if runtime_directory else None,
                request_host_monotonic=time.monotonic(),
                request_device_time_ms=self.latest.get('device_time_ms'),
                request_sequence=self.latest.get('sequence')))
        requested_at = time.monotonic()
        if self.worker.send(command):
            self.handover.queued(command, requested_at)
            self.update_handover_status()
            if command == 'T':
                self.trace_busy = True
                self.trace_export_status = 'queued'
                self.trace_progress.setText('导出请求已排队，等待发送与设备元信息。')
                self.update_control_availability()
            self.message.setText(f'命令 {command} 已排队；设备状态以遥测为准。')
            self.control_log('command_queued', command=command, acknowledged=False,
                             jog_clearance=self.jog_check.isChecked(),
                             direction_verified=self.direction_check.isChecked())
            if self.measurement_panel and command in 'DU':
                self.measurement_panel.notify_command(command)
            return True
        self.message.setText('操作未发送：串口正在关闭或命令队列已满。')
        self.control_log('command_rejected', command=command, reason='closing_or_full_queue')
        if command == 'H':
            self.handover.rejected('串口正在关闭或命令队列已满')
            self.update_handover_status()
        return False

    def command_sent(self, command):
        if self.sender() is not None and self.sender() is not self.worker:
            return
        self.control_log('command_written', command=command, acknowledged=False)
        self.log_event('command_sent', f'已写入 {command}；未代表设备确认')
        self.handover.sent(command)
        self.update_handover_status()
        if self.measurement_panel:
            self.measurement_panel.notify_command(command)

    def receive_trace_status(self, status):
        """Only small summaries cross into the GUI; historical rows stay off it."""
        if self.sender() is not None and self.sender() is not self.worker:
            return
        state = status.get('status')
        self.trace_export_status = state
        self.trace_busy = state in ('waiting_metadata', 'receiving', 'saving', 'failed_receiving_tail')
        received, total = status.get('received_rows', 0), status.get('total_rows')
        progress = f'{received}/{total}' if total is not None else str(received)
        if state == 'waiting_metadata':
            text = '请求已写入，等待连续记录元信息。'
        elif state == 'receiving':
            text = f'正在接收连续记录：{progress} 行。'
        elif state == 'failed_receiving_tail':
            text = f'导出校验失败：已可信接收 {progress} 行；正在保留本次剩余证据。'
        elif state == 'saving':
            integrity = '导出校验完整' if status.get('complete') else '导出不完整'
            text = f'{integrity}，{progress} 行；正在后台保存，尚未确认写盘成功。'
        elif state == 'request_cancelled':
            text = '停止命令优先，尚未发送的导出请求已取消。'
        elif state == 'saved' and status.get('storage_complete'):
            integrity = '导出完整' if status.get('complete') else '导出不完整，失败证据'
            gaps = status.get('sample_gap_count', 0)
            continuity = f'；采样序号有 {gaps} 处间断' if gaps else ''
            text = f'{integrity}已保存：{received} 行{continuity}\n{status.get("json_path", "")}'
            self.runtime_event('control_trace_saved', **status)
        else:
            text = '连续记录保存失败：' + str(status.get('error', '未确认文件保存成功'))
            self.runtime_event('control_trace_storage_failed', **status)
        self.trace_progress.setText(text)
        self.update_control_availability()

    def receive_serial(self, records):
        # Ignore queued signals from a connection that has already ended.
        if self.source == 'serial' and self.serial_mode == 'project' and self.sender() is self.worker:
            self.receive(records)

    def receive(self, records):
        for record in records:
            record = dict(record)
            if self.source in ('demo', 'replay'):
                record['source'] = self.source
            self.latest = record
            self.runtime_sample(record)
            now = time.monotonic()
            if self.source == 'serial':
                # GUI/disk stalls must not make old worker telemetry fresh again.
                received = record.get('host_monotonic')
                self.last_received = (received if isinstance(received, (int, float))
                    and not isinstance(received, bool) and math.isfinite(received)
                    and 0 < received <= now else 0.0)
            else:
                self.last_received = now
            if (self.source == 'serial' and self.serial_mode == 'project'
                    and self.serial_config.is_project_default and self.serial_ready
                    and self.worker and not self.worker.closing.is_set()):
                self.handover.observe(record, now)
            self.record_control_availability()
            self.rate_history.append(self.last_received)
            self.history.append(record)
            self.plot_dirty = True
            if self.measurement_panel and self.measurement_source() != 'idle':
                self.measurement_panel.ingest(record)
            self.metrics.add(record)
            if self.recorder:
                try:
                    self.recorder.write(record)
                except (OSError, ValueError) as error:
                    self.message.setText(f'记录失败：{error}。采集继续，请检查磁盘。')
                    self.finish_recording()
            transition = (record.get('state'), record.get('calibrated'), record.get('fault'),
                          record.get('diagnostic_status'), record.get('first_fault'),
                          record.get('motor_test_status'))
            if transition != self.last_transition:
                self.control_log('telemetry_state', device_confirmed_command=False,
                                 state_name=STATE_NAMES.get(record.get('state'), '未知'),
                                 diagnostic=diagnostic_text(record),
                                 faults=fault_names(record.get('fault')),
                                 motor_test_status=record.get('motor_test_status'),
                                 motor_test_delta=record.get('motor_test_delta'),
                                 worker_received_monotonic=record.get('host_monotonic'))
                self.log_event('state', f"{STATE_NAMES.get(transition[0], '未知')} / 标定 {transition[1]} / 故障 {transition[2]} / {diagnostic_text(record)}")
                self.last_transition = transition
            self.update_recovery(record)
        if self.measurement_panel:
            # Queued old serial batches must not temporarily enable capture.
            self.measurement_panel.set_context(self.measurement_source(), self.is_fresh())

    def update_recovery(self, record):
        if not self.recovery or self.recovery_result is not None:
            return
        t = float(record['elapsed_s'])
        if record.get('sequence_duplicate'):
            return
        if record.get('sequence_reset') or record.get('time_reset'):
            self.log_event('recovery_invalid', '数据重启或时间重置，本次恢复计时取消')
            self.recovery = None
            self.recovery_start = None
            return
        if self.recovery.get('last_t') is not None and (
                t - self.recovery['last_t'] > 0.1 or t < self.recovery['last_t'] or
                record.get('sequence_gap', 0)):
            self.recovery_start = None
        self.recovery['last_t'] = t
        valid = (record.get('state') == 2 and record.get('calibrated') == 1 and
                 record.get('fault') == 0 and abs(record['theta_deg']) <= self.recovery['tolerance'])
        if valid:
            if self.recovery_start is None:
                self.recovery_start = t
            if t - self.recovery_start >= self.recovery['hold']:
                self.recovery_result = self.recovery_start - self.recovery['marked_at']
                self.log_event('recovery', f'恢复 {self.recovery_result:.3f} s；已连续保持 {self.recovery["hold"]:.2f} s')
        else:
            self.recovery_start = None

    def mark_recovery(self):
        if not self.latest or (self.source == 'serial' and not self.is_fresh()):
            self.message.setText('请在接收数据时标记扰动结束。')
            return
        self.recovery = {'marked_at': float(self.latest['elapsed_s']),
                         'tolerance': self.tolerance.value(), 'hold': self.hold_seconds.value()}
        self.recovery_start = self.recovery_result = None
        self.log_event('disturbance_end', f'扰动结束；±{self.tolerance.value()}°，保持 {self.hold_seconds.value()} s')

    def mark_event(self):
        detail = self.mark_text.text().strip()
        if detail:
            self.log_event('marker', detail)
            self.mark_text.clear()

    def log_event(self, kind, detail):
        t = self.latest.get('elapsed_s', 0) if self.latest else 0
        text = f'{t:8.3f} s   {detail}'
        self.event_history.append(text)
        self.events_text.setPlainText('\n'.join(self.event_history))
        self.events_text.verticalScrollBar().setValue(self.events_text.verticalScrollBar().maximum())
        if (kind in ('marker', 'disturbance_end') and self.source == 'serial' and
                self.worker is not None and self.serial_ready and not self.worker.closing.is_set()):
            context = {key: self.latest.get(key) if self.latest else None
                       for key in ('sequence', 'device_time_ms', 'elapsed_s')}
            self.control_log(kind, kind=kind, detail=detail, frame_is_fresh=self.is_fresh(), **context)
        if self.recorder:
            try:
                self.recorder.event(kind, detail, self.latest)
            except OSError as error:
                self.message.setText(f'事件记录失败：{error}')
                self.finish_recording()

    def metadata_dialog(self):
        dialog = QDialog(self)
        dialog.setWindowTitle('实验信息')
        dialog.setMinimumWidth(550)
        form = QFormLayout(dialog)
        inputs = {}
        for key, text in [('experiment', '实验名称'), ('board_id', '套件 / 板号'),
                          ('firmware_version', '固件版本 / Git 提交'), ('signs', '方向参数 θ / 编码器 / 电机'),
                          ('counts_per_rev', '单圈编码器计数（实测）'), ('installation', '安装相位 / 运动范围'),
                          ('notes', '测试条件 / 尺寸 / 质量 / 扰动描述')]:
            inputs[key] = QLineEdit()
            form.addRow(text, inputs[key])
        inputs['experiment'].setText('起摆与平衡测量')
        form.addRow(label(f'数据来源：{self.source}；空白项目作为未知项保留。', 'muted'))
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.button(QDialogButtonBox.Ok).setText('开始记录')
        buttons.button(QDialogButtonBox.Cancel).setText('取消')
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        form.addRow(buttons)
        if dialog.exec() != QDialog.Accepted:
            return None
        return {key: item.text().strip() or None for key, item in inputs.items()}

    def toggle_recording(self):
        if self.recorder:
            self.finish_recording()
            return
        if self.source == 'idle' or not self.latest:
            self.message.setText('请先连接并收到遥测，或打开演示 / 回放。')
            return
        metadata = self.metadata_dialog()
        if metadata is not None:
            self.start_recording(metadata)

    def start_recording(self, metadata):
        if self.recorder or self.closing_recorder or self.exit_pending:
            self.message.setText('请等待当前记录保存完成后再开始新记录。')
            return False
        try:
            metadata = dict(metadata, source=self.source, host_version=HOST_VERSION,
                            project_version=PROJECT_VERSION,
                            baud=self.serial_config.baudrate, serial_config=self.serial_config.as_dict(),
                            serial_mode=self.serial_mode, stop_on_disconnect=self.stop_on_close.isChecked() and self.serial_mode == 'project' and self.serial_config.is_project_default,
                            direction_verified=self.direction_check.isChecked())
            self.recorder = AsyncSessionRecorder(self.output_root, metadata)
            self.metrics = Metrics()
            self.recovery = self.recovery_start = self.recovery_result = None
            self.record_button.setText('结束记录并生成摘要')
            self.record_path.setText('正在创建实验记录；采集继续。')
            self.log_event('record_start', '实验记录开始')
            return True
        except OSError as error:
            self.message.setText(f'无法创建记录：{error}')
            return False

    def finish_recording(self):
        if self.recorder:
            recorder, self.recorder = self.recorder, None
            self.closing_recorder = recorder
            recorder.request_close({
                'communication': dict(self.stats),
                'recovery_delay_s': self.recovery_result,
                'recovery_criterion': self.recovery,
            })
            self.record_button.setText('正在保存实验记录…')
            self.record_button.setEnabled(False)
            self.record_path.setText('正在保存尾部记录；采集与停止按钮仍可使用。')
        self.poll_recording()

    def poll_recording(self):
        """Observe writer state only; never wait for files or a writer thread."""
        if self.recorder:
            status = self.recorder.poll()
            if status['error']:
                # Also reached on idle flush/init failures with no further frames.
                self.finish_recording()
                return
            if status['directory'] is not None:
                self.record_path.setText(f'正在记录：{status["directory"]}')
        recorder = self.closing_recorder
        if recorder is None:
            return
        status = recorder.poll()
        if status['error']:
            self.record_path.setText(f'记录不完整：{status["directory"] or self.output_root}')
            self.message.setText(f'结束记录失败：{status["error"]}。采集继续，请检查磁盘。')
        if not recorder.done.is_set():
            return
        # Completion can race the first snapshot; read the final state after done.
        status = recorder.poll()
        if status['error']:
            self.record_path.setText(f'记录不完整：{status["directory"] or self.output_root}')
            self.message.setText(f'结束记录失败：{status["error"]}。采集继续，请检查磁盘。')
        self.closing_recorder = None
        self.record_button.setText('开始实验记录')
        self.record_button.setEnabled(True)
        if not status['error'] and status['state'] == 'closed':
            self.record_path.setText(f'记录已保存：{status["summary"]}')
            self.message.setText(f'实验已保存：{status["summary"]}')
        if self.exit_pending:
            QTimer.singleShot(0, self.close)

    def tick(self):
        now = time.monotonic()
        dt = now - getattr(self, 'last_tick', now)
        self.last_tick = now
        if self.source == 'demo':
            target = int((now - self.origin) * 50)
            records = []
            while self.demo_sequence <= target:
                record = demo_record(self.demo_sequence / 50, self.demo_sequence % 65536)
                record['elapsed_s'] = self.demo_sequence / 50
                records.append(record)
                self.demo_sequence += 1
            self.receive(records)
        elif self.source == 'replay' and not self.replay_pause.isChecked():
            speed = [0.5, 1, 2, 4][self.replay_speed.currentIndex()]
            self.replay_cursor += dt * speed
            batch = []
            while (self.replay_index < len(self.replay_rows) and
                   self.replay_rows[self.replay_index]['elapsed_s'] <= self.replay_cursor):
                batch.append(self.replay_rows[self.replay_index])
                self.replay_index += 1
            if batch:
                self.receive(batch)
            if self.replay_index >= len(self.replay_rows):
                self.replay_pause.setChecked(True)
                self.message.setText('回放结束。可检查指标、保存截图或重新打开记录。')
        self.update_display()

    def update_display(self):
        self.poll_recording()
        busy = self.worker is not None
        project_control = self.serial_mode == 'project' and self.serial_config.is_project_default
        self.serial_config_label.setText(f'{self.serial_config.display_label} · ' + ('J280 遥测' if self.serial_mode == 'project' else '通用收发'))
        self.stop_on_close.setEnabled(project_control)
        self.direction_check.setEnabled(project_control)
        self.jog_check.setEnabled(project_control)
        self.serial_panel.set_send_enabled(bool(self.serial_mode == 'raw' and busy and self.serial_ready and not self.worker.closing.is_set()))
        self.serial_panel.flush()
        if self.measurement_panel:
            self.measurement_panel.set_context(self.measurement_source(), self.is_fresh())
        self.demo_button.setEnabled(not busy)
        self.replay_button.setEnabled(not busy)
        self.ports.setEnabled(not busy)
        self.refresh_button.setEnabled(not busy)
        self.update_control_availability()
        if self.operation_log and not self.logging_error:
            try:
                self.operation_log.check()
            except OSError as error:
                self.logging_failed(error)
        if not self.logging_error:
            names = [name for key, name in [('serial', '串口收发'), ('control', '开发板控制')] if self.log_preferences[key]]
            path = self.operation_log.directory if self.operation_log else None
            self.log_status.setText(('日志保留：' + '、'.join(names) +
                (f' · {path}' if path else ' · 等待串口事件')) if names else '日志保留：未开启')
        self.replay_pause.setEnabled(self.source == 'replay')
        self.replay_speed.setEnabled(self.source == 'replay')
        waiting = self.waiting_for_first_frame()
        stale = self.source == 'serial' and self.serial_ready and not self.is_fresh() and not waiting
        if self.source == 'serial' and self.serial_ready and self.serial_mode == 'project':
            fresh = self.is_fresh()
            if not waiting and fresh != self.link_fresh:
                self.control_log('telemetry_restored' if fresh else 'telemetry_timeout',
                                 last_received_monotonic=self.last_received)
                self.link_fresh = fresh
        if self.runtime_log and not self.runtime_error:
            try:
                self.runtime_log.check()
            except OSError as error:
                self.runtime_failed(error)
        if not self.runtime_error:
            path = self.runtime_log.directory if self.runtime_log else self.runtime_path
            self.runtime_status.setText('FPGA 运行日志：' +
                (f'{path}' if path else '项目连接后自动记录到 captures/fpga_logs'))
        self.source_badge.setText({'idle': '未连接', 'demo': '演示 · 非实测', 'replay': 'CSV 回放',
                                  'serial': '串口 · 连接中' if not self.serial_ready else
                                  '串口 · 等待首帧' if waiting else
                                  '串口 · 数据超时' if stale else '串口 · 实时采集'}[self.source])
        if self.source == 'serial' and self.serial_mode == 'raw':
            self.source_badge.setText('通用串口 · 已连接' if self.serial_ready else '通用串口 · 连接中')
        r = self.latest
        self.diagnostic_label.setText(('上次遥测（已超时） · ' if stale else '') +
            diagnostic_text(r) + '\n' + recovery_hint(r))
        age_prefix = '上次遥测（已超时） · ' if stale else ''
        self.adc_diagnostic_label.setText(age_prefix + adc_diagnostic_text(r))
        self.adc_diagnostic_label.setToolTip(adc_detail_text(r or {},
            'adc_fault_' if r and r.get('sensor_fault_reason') else 'adc_'))
        self.motor_diagnostic_label.setText(age_prefix + motor_diagnostic_text(r))
        self.motor_diagnostic_label.setToolTip(motion_limits_text(r) + '\n' + recovery_hint(r) +
                                               '\n' + HANDOVER_TOOLTIP)
        self.motor_test_result.setText(age_prefix + motor_test_text(r))
        if waiting:
            self.diagnostic_label.setText('等待 FPGA 首帧遥测；收到数据后显示标定、故障和启动诊断。')
        if r:
            for key in ['theta_deg', 'arm_deg', 'command_permille']:
                self.cards[key][0].setText(fmt(r.get(key), 0 if key == 'command_permille' else 2))
            self.cards['state'][0].setText(STATE_NAMES.get(r.get('state'), '未知'))
            self.cards['state'][0].setStyleSheet('font-size:22px')
            self.cards['state'][1].setText('上次数据 · 超时' if stale else f"帧 {r.get('sequence', '—')} · v{r.get('protocol_version', 1)}")
            fault = r.get('fault')
            fault_code = f'0x{fault:02X}' if type(fault) is int and 0 <= fault <= 255 else '—'
            faults = '、'.join(fault_names(fault))
            self.fault_label.setText(f"标定：{'完成' if r.get('calibrated') == 1 else '未完成'}   ADC：{r.get('adc', '—')}   故障：{fault_code} {faults or '无'}")
            if r.get('state') in (0, 3) and not has_measurement_diagnostics(r):
                self.fault_label.setText(self.fault_label.text() + '   速度估计停机清零')
            detail_tooltips = {prefix: adc_detail_text(r, prefix) for prefix in ('adc_', 'adc_fault_')}
            for index, (key, name, _) in enumerate(EXTENSIONS):
                if key.endswith(('contribution_min', 'contribution_max')):
                    self.ext_table.item(index, 0).setText(name.replace('贡献', adc_contribution_title(r)))
                value = r.get(key)
                self.ext_table.item(index, 1).setText('未提供' if value is None else fmt(value, 0 if type(value) is int else 3))
                self.ext_table.item(index, 1).setToolTip(detail_tooltips.get(ADC_DETAIL_PREFIXES.get(key), ''))
                if key.startswith('h_'):
                    self.ext_table.item(index, 1).setToolTip(handover_control_text(r) + '\n' + HANDOVER_TOOLTIP)
        else:
            for value, _ in self.cards.values():
                value.setText('—')
            self.fault_label.setText('标定：—   故障：—   ADC：—')
            for row in range(len(EXTENSIONS)):
                self.ext_table.item(row, 0).setText(EXTENSIONS[row][1])
                self.ext_table.item(row, 1).setText('未提供')
                self.ext_table.item(row, 1).setToolTip('')
        now = time.monotonic()
        while self.rate_history and self.rate_history[0] < now - 2:
            self.rate_history.popleft()
        rate = len(self.rate_history) / 2
        age = now - self.last_received if self.latest else None
        self.health.setText(f"接收 {self.stats.get('frames', self.metrics.snapshot().get('samples', 0))} 帧 · {rate:.1f} 帧/s\n最后帧龄 {fmt(age)} s\nCRC 错误 {self.stats.get('crc_errors', 0)} · 缺帧 {self.stats.get('missing_frames', 0)}\n重复 {self.stats.get('duplicates', 0)} · 重启/倒序 {self.stats.get('resets', 0)}\n未知协议 {self.stats.get('unsupported_frames', 0)}")
        if self.source == 'serial' and self.serial_mode == 'raw':
            self.health.setText(f"通用串口收发\nRX {self.stats.get('rx_bytes', 0)} B · TX {self.stats.get('tx_bytes', 0)} B")
        self.update_metrics()

    def refresh_plots(self):
        # Retain the dirty flag while frozen or hidden so returning to the
        # curves shows the latest complete history, without replaying old draws.
        if self.pause_plots.isChecked() or not self.plots[0].isVisible():
            return
        seconds = self.window_seconds.value()
        if not self.plot_dirty and seconds == self.plotted_window_seconds:
            return
        self.update_plots()
        self.plot_dirty = False
        self.plotted_window_seconds = seconds

    def update_plots(self):
        if not self.history:
            for curve in self.plot_curves.values():
                curve.setData([], [])
            return
        end = self.history[-1]['elapsed_s']
        start = max(0, end - self.window_seconds.value())
        rows = [r for r in self.history if r['elapsed_s'] >= start]
        # Share numeric time/gap arrays across curves. Constructing every
        # point through nested Python loops becomes expensive at 300 s.
        times = np.fromiter((r['elapsed_s'] for r in rows), dtype=float, count=len(rows))
        sequences = np.fromiter((r['sequence'] for r in rows), dtype=np.int64, count=len(rows))
        gaps = np.flatnonzero((np.diff(times) > 0.1) |
                              ((np.diff(sequences) & 65535) != 1)) + 1
        # Keep explicit NaNs: finite connections must never bridge lost frames.
        times = np.insert(times, gaps, times[gaps])
        for key, curve in self.plot_curves.items():
            values = np.fromiter((r.get(key, np.nan) for r in rows), dtype=float, count=len(rows))
            curve.setData(times, np.insert(values, gaps, np.nan))
        for plot in self.plots:
            plot.setXRange(start, max(start + 1, end), padding=0.01)

    def update_metrics(self):
        m = self.metrics.snapshot()
        percent = lambda key: '—' if m.get(key) is None else fmt(m[key] * 100) + ' %'
        rows = [('样本数', str(m.get('samples', 0))), ('记录时长', fmt(m.get('total_duration_s')) + ' s'),
                ('有效平衡样本', str(m.get('balance_samples', 0))),
                ('平衡摆杆 RMS', fmt(m.get('balance_rms_deg'), 3) + ' °'),
                ('平衡最大绝对倾角', fmt(m.get('balance_peak_deg'), 3) + ' °'),
                ('摆臂范围', f"{fmt(m.get('arm_min_deg'))} ～ {fmt(m.get('arm_max_deg'))} °"),
                ('平衡平均角度偏置', fmt(m.get('balance_mean_deg'), 3) + ' °'),
                ('最长连续有效平衡', fmt(m.get('longest_balance_s'), 3) + ' s'),
                ('最长连续 ±2° 保持', fmt(m.get('longest_in_band_s'), 3) + ' s'),
                ('满量程指令占比（1000‰）', percent('saturation_ratio')),
                ('起摆限幅比例（800‰）', percent('swing_saturation_ratio')),
                ('平衡限幅比例（1000‰）', percent('balance_saturation_ratio')),
                ('连续有效捕获次数', str(m.get('capture_count', 0))),
                ('首次起摆至捕获', fmt(m.get('capture_delay_s'), 3) + ' s'),
                ('本次扰动恢复', fmt(self.recovery_result, 3) + ' s')]
        self.metric_table.setRowCount(len(rows))
        for row, pair in enumerate(rows):
            for col, text in enumerate(pair):
                self.metric_table.setItem(row, col, QTableWidgetItem(text))

    def save_screenshot(self):
        filename, _ = QFileDialog.getSaveFileName(self, '保存曲线截图', str(self.output_root / '曲线.png'), 'PNG (*.png)')
        if filename:
            Path(filename).parent.mkdir(parents=True, exist_ok=True)
            if not self.tabs.widget(0).grab().save(filename):
                self.message.setText('截图保存失败，请检查路径。')
            else:
                self.message.setText(f'截图已保存：{filename}')

    def closeEvent(self, event):
        if self.worker and (self.worker.isRunning() or getattr(self.worker, 'delivery_pending', False)):
            self.exit_pending = True
            self.worker.request_close(self.stop_on_close.isChecked())
            event.ignore()
            self.message.setText('正在停止并关闭串口，完成后自动退出；必要时按 SW3。')
            return
        self.finish_recording()
        if self.closing_recorder is not None:
            self.exit_pending = True
            event.ignore()
            # Keep the regular UI timer alive until the writer has drained.
            if not self.timer.isActive():
                self.timer.start()
            if not self.closing_recorder.poll()['error']:
                self.message.setText('正在保存尾部记录，完成后自动退出。')
            return
        self.close_operation_log()
        self.close_runtime_log()
        self.timer.stop()
        self.plot_timer.stop()
        event.accept()


def default_output():
    if getattr(sys, 'frozen', False):
        return Path(sys.executable).resolve().parent / 'captures'
    return Path(__file__).resolve().parents[1] / 'Release' / 'captures'


def main():
    parser = argparse.ArgumentParser(description='J280 姿态控制测量工作站')
    parser.add_argument('--demo', action='store_true')
    parser.add_argument('--replay', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--smoke-test', action='store_true', help='离线启动、绘图和退出验证')
    parser.add_argument('--screenshot', type=Path)
    args = parser.parse_args()
    app = QApplication(sys.argv[:1])
    # The offscreen Windows platform has no automatic system font discovery.
    if not QFontDatabase.families() and sys.platform == 'win32':
        import os
        font_dir = Path(os.environ.get('WINDIR', 'C:/Windows')) / 'Fonts'
        for name in ('msyh.ttc', 'msyhbd.ttc'):
            if (font_dir / name).is_file():
                QFontDatabase.addApplicationFont(str(font_dir / name))
    app.setStyle('Fusion')
    app.setFont(QFont('Microsoft YaHei UI', 10))
    app.setStyleSheet(STYLE)
    window = Window(args.output)
    window.show()
    if args.replay:
        window.load_replay(args.replay)
    elif args.demo or args.smoke_test:
        window.start_demo()
    if args.screenshot:
        def screenshot():
            args.screenshot.parent.mkdir(parents=True, exist_ok=True)
            window.grab().save(str(args.screenshot))
        QTimer.singleShot(7000 if args.demo and not args.smoke_test else 2500, screenshot)
    if args.smoke_test:
        smoke = {'passed': False}
        def verify_smoke():
            smoke['passed'] = bool(window.latest and len(window.history) >= 20
                and window.plot_curves['theta_deg'].getData()[0] is not None
                and not window.allowed('G') and not window.allowed('F')
                and window.pages.currentWidget() is window.main_page)
            window.open_developer()
            smoke['passed'] = bool(smoke['passed'] and window.measurement_panel
                and len(window.measurement_panel.records) >= 20
                and window.pages.currentWidget() is window.developer_page
                and window.developer_page.window() is window)
            window.close()
        QTimer.singleShot(3200, verify_smoke)
    result = app.exec()
    return 1 if args.smoke_test and not smoke['passed'] else result


if __name__ == '__main__':
    raise SystemExit(main())
