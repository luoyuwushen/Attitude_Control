"""Bounded raw byte terminal, shared by project receive and generic serial mode."""
import codecs
from datetime import datetime
from pathlib import Path

from PySide6.QtWidgets import (QCheckBox, QComboBox, QFileDialog, QHBoxLayout,
    QLabel, QPlainTextEdit, QPushButton, QVBoxLayout, QWidget)


class SerialPanel(QWidget):
    def __init__(self, output_root, send, parent=None):
        super().__init__(parent)
        self.output_root = Path(output_root)
        self.send_callback = send
        self.buffer = bytearray()
        self.pending = bytearray()
        self.rx_total = self.tx_total = 0
        self.limit = 262144
        layout = QVBoxLayout(self)
        row = QHBoxLayout()
        self.hex_view = QCheckBox('HEX 接收显示')
        self.hex_view.setChecked(True)
        row.addWidget(self.hex_view)
        self.encoding = QComboBox()
        self.encoding.addItems(['UTF-8', 'GBK', 'ASCII'])
        row.addWidget(self.encoding)
        clear = QPushButton('清空接收')
        clear.clicked.connect(self.clear)
        row.addWidget(clear)
        save = QPushButton('保存接收字节')
        save.clicked.connect(self.save)
        row.addWidget(save)
        row.addStretch()
        layout.addLayout(row)
        self.count = QLabel('RX 0 B / TX 0 B')
        layout.addWidget(self.count)
        self.view = QPlainTextEdit()
        self.view.setReadOnly(True)
        self.view.document().setMaximumBlockCount(4096)
        layout.addWidget(self.view, 3)
        self.input = QPlainTextEdit()
        self.input.setPlaceholderText('输入文本，或勾选 HEX 后输入例如 AA 55 01 18')
        self.input.setMaximumHeight(100)
        layout.addWidget(self.input)
        row = QHBoxLayout()
        self.hex_send = QCheckBox('HEX 发送')
        row.addWidget(self.hex_send)
        self.ending = QComboBox()
        for name, value in [('不附加换行', ''), ('LF', '\n'), ('CR', '\r'), ('CRLF', '\r\n')]:
            self.ending.addItem(name, value)
        row.addWidget(self.ending)
        self.send_button = QPushButton('发送一次')
        self.send_button.clicked.connect(self.send)
        row.addWidget(self.send_button)
        layout.addLayout(row)
        self.hint = QLabel('项目模式仅查看原始接收；任意字节发送需在设置中选择通用串口收发。')
        self.hint.setWordWrap(True)
        layout.addWidget(self.hint)
        self.hex_view.toggled.connect(self.render)
        self.encoding.currentIndexChanged.connect(self.render)
        self.decoder = self.new_decoder()
        self.set_send_enabled(False)

    def new_decoder(self):
        return codecs.getincrementaldecoder(self.encoding.currentText())(errors='replace')

    def clear(self):
        self.buffer.clear()
        self.pending.clear()
        self.rx_total = self.tx_total = 0
        self.decoder = self.new_decoder()
        self.view.clear()
        self.update_count()

    def ingest(self, data):
        self.rx_total += len(data)
        self.buffer.extend(data)
        del self.buffer[:-self.limit]
        self.pending.extend(data)
        del self.pending[:-self.limit]

    def flush(self):
        if self.pending:
            data = bytes(self.pending)
            self.pending.clear()
            text = data.hex(' ').upper() + '\n' if self.hex_view.isChecked() else self.decoder.decode(data)
            cursor = self.view.textCursor()
            cursor.movePosition(cursor.MoveOperation.End)
            cursor.insertText(text)
            self.view.setTextCursor(cursor)
            # Text without newlines still needs a strict memory bound.
            if self.view.document().characterCount() > self.limit * 3:
                self.render()
        self.update_count()

    def render(self, *_):
        self.pending.clear()
        self.decoder = self.new_decoder()
        data = bytes(self.buffer)
        self.view.setPlainText(data.hex(' ').upper() if self.hex_view.isChecked() else self.decoder.decode(data))

    def update_count(self):
        self.count.setText(f'RX {self.rx_total} B / TX {self.tx_total} B · 接收缓存保留末 {self.limit // 1024} KiB')

    def sent(self, data):
        self.tx_total += len(data)
        self.update_count()

    def set_send_enabled(self, enabled):
        self.send_button.setEnabled(enabled)

    def send(self):
        try:
            text = self.input.toPlainText()
            data = bytes.fromhex(text) if self.hex_send.isChecked() else (text + self.ending.currentData()).encode(self.encoding.currentText())
            if not data or len(data) > 4096:
                raise ValueError('每次发送需为 1～4096 字节')
        except (ValueError, UnicodeError) as error:
            self.hint.setText(f'未发送：{error}')
            return False
        accepted = self.send_callback(data)
        self.hint.setText('数据已排队，TX 统计以完整写入为准。' if accepted else '未发送：请检查通用模式、连接状态和数据位允许的字节范围。')
        return accepted

    def save(self):
        filename, _ = QFileDialog.getSaveFileName(self, '保存当前接收缓存',
            str(self.output_root / f'serial_{datetime.now():%Y%m%d_%H%M%S}.bin'), '原始字节 (*.bin)')
        if filename:
            try:
                Path(filename).parent.mkdir(parents=True, exist_ok=True)
                Path(filename).write_bytes(self.buffer)
                self.hint.setText(f'已保存接收缓存：{filename}')
            except OSError as error:
                self.hint.setText(f'保存失败：{error}')
