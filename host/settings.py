"""Operator serial preferences and the explicit developer-mode entry."""
import json
from pathlib import Path

from PySide6.QtWidgets import (QCheckBox, QComboBox, QDialog, QDialogButtonBox, QFormLayout,
                              QGroupBox, QLabel, QPushButton, QVBoxLayout)
from host.serial_config import SerialConfig

BAUD_PRESETS = [1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200,
                230400, 460800, 921600, 1000000]


def load_preferences(path):
    """Invalid settings cannot silently enable a device command mode."""
    try:
        data = json.loads(Path(path).read_text(encoding='utf-8'))
        config = SerialConfig(**data.get('serial', {}))
        mode = data.get('mode', 'project')
        if mode not in ('project', 'raw'):
            raise ValueError('Invalid mode')
        presets = sorted(set(BAUD_PRESETS + [int(x) for x in data.get('baud_presets', [])
                                             if type(x) is int and 0 < x <= 4000000]))
        return config, mode, presets
    except (OSError, ValueError, TypeError, AttributeError):
        return SerialConfig(), 'project', list(BAUD_PRESETS)


def load_log_preferences(path):
    try:
        data = json.loads(Path(path).read_text(encoding='utf-8')).get('logging', {})
        return {key: data.get(key) is True for key in ('serial', 'control')}
    except (OSError, ValueError, TypeError, AttributeError):
        return {'serial': False, 'control': False}


def save_preferences(path, config, mode, presets, log_preferences=None):
    config.validate()
    if mode not in ('project', 'raw'):
        raise ValueError('Invalid mode')
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    try:
        logging = load_log_preferences(path) if log_preferences is None else {
            key: log_preferences.get(key) is True for key in ('serial', 'control')}
        temporary.write_text(json.dumps({'serial': config.as_dict(), 'mode': mode,
            'baud_presets': sorted(set(presets)), 'logging': logging, 'format_version': 2},
            ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        temporary.replace(path)
    finally:
        if temporary.exists():
            temporary.unlink()


class SettingsDialog(QDialog):
    def __init__(self, config, mode, presets, connected=False, parent=None, log_preferences=None):
        super().__init__(parent)
        self.setWindowTitle('设置')
        self.setMinimumWidth(520)
        self.developer_requested = False
        self.selected_config = config
        self.selected_mode = mode
        self.presets = list(presets)
        self.connected = connected
        self.selected_log_preferences = dict(log_preferences or {'serial': False, 'control': False})
        layout = QVBoxLayout(self)
        group = QGroupBox('串口设置')
        form = QFormLayout(group)
        self.mode_combo = QComboBox()
        self.mode_combo.addItem('J280 遥测与控制', 'project')
        self.mode_combo.addItem('通用串口收发', 'raw')
        self.mode_combo.setCurrentIndex(0 if mode == 'project' else 1)
        form.addRow('工作模式', self.mode_combo)
        self.baud = QComboBox()
        self.baud.setEditable(True)
        self.baud.addItems([str(x) for x in sorted(set(presets + [config.baudrate]))])
        self.baud.setCurrentText(str(config.baudrate))
        form.addRow('波特率（可输入）', self.baud)
        add = QPushButton('加入常用波特率')
        add.clicked.connect(self.add_baud)
        form.addRow('', add)
        self.bits = QComboBox()
        self.bits.addItems(['5', '6', '7', '8'])
        self.bits.setCurrentText(str(config.bytesize))
        form.addRow('数据位', self.bits)
        self.parity = QComboBox()
        for text, value in [('无校验 N', 'N'), ('偶校验 E', 'E'), ('奇校验 O', 'O'),
                            ('标记校验 M', 'M'), ('空校验 S', 'S')]:
            self.parity.addItem(text, value)
        self.parity.setCurrentIndex(self.parity.findData(config.parity))
        form.addRow('校验位', self.parity)
        self.stops = QComboBox()
        self.stops.addItems(['1', '1.5', '2'])
        self.stops.setCurrentText(f'{config.stopbits:g}')
        form.addRow('停止位', self.stops)
        restore = QPushButton('恢复 J280 默认：115200 / 8N1')
        restore.clicked.connect(self.restore_project)
        form.addRow('', restore)
        group.setEnabled(not connected)
        layout.addWidget(group)
        hint = QLabel('连接期间串口设置锁定，断开后可修改。' if connected else
            '当前 J280 固件使用 115200 / 8N1。设置不同参数不会修改固件；'
            '自定义串口可用于通用收发。部分组合是否可用取决于串口驱动。')
        hint.setWordWrap(True)
        layout.addWidget(hint)
        logs = QGroupBox('日志保留')
        logs_box = QVBoxLayout(logs)
        self.serial_log_check = QCheckBox('保留串口收发日志（RX / TX 原始字节）')
        self.control_log_check = QCheckBox('保留开发板控制日志（命令 / 状态 / 异常）')
        self.serial_log_check.setChecked(self.selected_log_preferences['serial'])
        self.control_log_check.setChecked(self.selected_log_preferences['control'])
        logs_box.addWidget(self.serial_log_check)
        logs_box.addWidget(self.control_log_check)
        log_hint = QLabel('两项可分别开启。保存后立即生效，写入 captures/logs；不依赖实验记录按钮。')
        log_hint.setWordWrap(True)
        logs_box.addWidget(log_hint)
        layout.addWidget(logs)
        developer = QGroupBox('开发者模式')
        dev_box = QVBoxLayout(developer)
        description = QLabel('ADC 静态与标定分析、编码器计数、有效区扫描、自由衰减、'
                             '点动响应，以及后续数据接入与现场参数记录。')
        description.setWordWrap(True)
        dev_box.addWidget(description)
        self.developer_button = QPushButton('进入开发者模式')
        self.developer_button.clicked.connect(self.enter_developer)
        dev_box.addWidget(self.developer_button)
        layout.addWidget(developer)
        self.error_label = QLabel('')
        self.error_label.setWordWrap(True)
        layout.addWidget(self.error_label)
        buttons = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel)
        buttons.button(QDialogButtonBox.Save).setText('保存设置')
        buttons.button(QDialogButtonBox.Cancel).setText('取消')
        buttons.accepted.connect(self.apply)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def add_baud(self):
        try:
            rate = int(self.baud.currentText().strip())
            SerialConfig(baudrate=rate).validate()
        except ValueError:
            self.error_label.setText('波特率请输入 1～4000000 的整数。')
            return False
        if rate not in self.presets:
            self.presets.append(rate)
        if self.baud.findText(str(rate)) < 0:
            self.baud.addItem(str(rate))
        self.error_label.setText(f'{rate} 已加入常用列表，保存后保留。')
        return True

    def restore_project(self):
        self.mode_combo.setCurrentIndex(0)
        self.baud.setCurrentText('115200')
        self.bits.setCurrentText('8')
        self.parity.setCurrentIndex(self.parity.findData('N'))
        self.stops.setCurrentText('1')

    def apply(self):
        if not self.connected:
            try:
                self.selected_config = SerialConfig(baudrate=int(self.baud.currentText().strip()),
                    bytesize=int(self.bits.currentText()), parity=self.parity.currentData(),
                    stopbits=float(self.stops.currentText()))
                self.selected_config.validate()
            except ValueError as error:
                self.error_label.setText(f'设置无效：{error}')
                return
            self.selected_mode = self.mode_combo.currentData()
            self.presets = sorted(set(self.presets + [self.selected_config.baudrate]))
        self.selected_log_preferences = {'serial': self.serial_log_check.isChecked(),
                                         'control': self.control_log_check.isChecked()}
        self.accept()

    def enter_developer(self):
        self.developer_requested = True
        # This entry never applies partially edited serial settings.
        self.reject()
