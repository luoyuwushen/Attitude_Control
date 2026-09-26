"""Serial ownership stays in one worker; commands never originate from replay."""
import queue
import threading
import time

import serial
from PySide6.QtCore import QThread, Signal

from host.core import StreamDecoder
from host.serial_config import SerialConfig


class SerialWorker(QThread):
    records = Signal(list)
    status = Signal(str)
    statistics = Signal(dict)
    command_sent = Signal(str)
    raw_received = Signal(bytes)
    raw_sent = Signal(bytes)
    failed = Signal(str)

    def __init__(self, port, parent=None, config=None, mode='project'):
        super().__init__(parent)
        if mode not in ('project', 'raw'):
            raise ValueError('串口模式必须为 project 或 raw')
        if config is not None and not isinstance(config, SerialConfig):
            raise ValueError('串口配置必须为 SerialConfig')
        self.port_name = port
        self.config = (config if config is not None else SerialConfig()).validate()
        self.mode = mode
        self.commands = queue.Queue(maxsize=8)
        self.closing = threading.Event()
        self.stop_on_close = mode == 'project' and self.config.is_project_default
        self.decoder = StreamDecoder() if mode == 'project' else None
        self.rx_bytes = self.tx_bytes = 0
        self._dispatch_lock = threading.Lock()
        self._command_generation = 0

    def send(self, command):
        if (self.mode != 'project' or not self.config.is_project_default
                or not isinstance(command, str) or len(command) != 1
                or command not in 'DUGSRFB'):
            return False
        # Acceptance and the start of a write are serialized. An already
        # running write may finish before S is accepted; its write timeout is
        # 250 ms. No queued/dequeued older command may start after acceptance.
        with self._dispatch_lock:
            if self.closing.is_set():
                return False
            if command == 'S':
                self._command_generation += 1
                while True:
                    try:
                        self.commands.get_nowait()
                    except queue.Empty:
                        break
            try:
                self.commands.put_nowait((command, self._command_generation))
                return True
            except queue.Full:
                return False

    def send_raw(self, data):
        if self.mode != 'raw' or not isinstance(data, (bytes, bytearray)):
            return False
        payload = bytes(data)
        if not 0 < len(payload) <= 4096:
            return False
        # Narrow data bits must never silently truncate the user's payload.
        if self.config.bytesize < 8 and any(value >= 1 << self.config.bytesize for value in payload):
            return False
        with self._dispatch_lock:
            if self.closing.is_set():
                return False
            try:
                self.commands.put_nowait((payload, self._command_generation))
                return True
            except queue.Full:
                return False

    def request_close(self, send_stop=True):
        with self._dispatch_lock:
            self.stop_on_close = (bool(send_stop) and self.mode == 'project'
                                  and self.config.is_project_default)
            self._command_generation += 1
            self.closing.set()

    def _statistics(self):
        stats = (dict(self.decoder.stats) if self.decoder else dict.fromkeys(
            ('frames', 'crc_errors', 'discarded_bytes', 'unsupported_frames',
             'missing_frames', 'duplicates', 'resets'), 0))
        stats.update(rx_bytes=self.rx_bytes, tx_bytes=self.tx_bytes)
        return stats

    def _write(self, port, payload, message):
        written = port.write(payload)
        if isinstance(written, int):
            self.tx_bytes += max(0, min(written, len(payload)))
        if written != len(payload):
            raise serial.SerialException(message)

    def run(self):
        port = None
        try:
            # Configure modem lines before opening; no intentional board reset.
            port = serial.Serial(port=None, **self.config.as_dict(), timeout=0.04,
                                 write_timeout=0.25, rtscts=False, dsrdtr=False)
            port.dtr = False
            port.rts = False
            port.port = self.port_name
            port.open()
            self.status.emit('connected')
            last_stats = time.monotonic()
            while not self.closing.is_set():
                try:
                    queued = self.commands.get_nowait()
                except queue.Empty:
                    queued = None
                sent = None
                if queued:
                    command, generation = queued
                    with self._dispatch_lock:
                        if not self.closing.is_set() and generation == self._command_generation:
                            payload = command.encode('ascii') if isinstance(command, str) else command
                            self._write(port, payload, '命令未完整写入' if isinstance(command, str)
                                        else '通用发送数据未完整写入，部分字节可能已送达')
                            sent = command
                if sent:
                    # Callbacks must not execute while the dispatch lock is held.
                    if isinstance(sent, str):
                        self.command_sent.emit(sent)
                    else:
                        self.raw_sent.emit(sent)
                if self.closing.is_set():
                    break
                if queued and not sent:
                    # The dequeued command was superseded; dispatch the queued
                    # stop before entering another potentially blocking read.
                    continue
                data = port.read(min(max(port.in_waiting, 1), 4096))
                if data:
                    self.rx_bytes += len(data)
                    self.raw_received.emit(data)
                    if self.decoder:
                        records = self.decoder.feed(data)
                        if records:
                            self.records.emit(records)
                if time.monotonic() - last_stats > 0.25:
                    self.statistics.emit(self._statistics())
                    last_stats = time.monotonic()
        except (serial.SerialException, OSError, ValueError) as error:
            self.failed.emit(str(error))
        finally:
            with self._dispatch_lock:
                self.closing.set()
                self._command_generation += 1
            if port and port.is_open:
                try:
                    if self.stop_on_close and self.mode == 'project' and self.config.is_project_default:
                        self._write(port, b'S', '断开前停止命令未完整写入，请使用 SW3')
                        self.command_sent.emit('S')
                except (serial.SerialException, OSError) as error:
                    self.failed.emit(f'停止命令未送达：{error}；请使用 SW3')
                finally:
                    try:
                        port.close()
                    except (serial.SerialException, OSError) as error:
                        self.failed.emit(f'串口关闭异常：{error}')
            self.statistics.emit(self._statistics())
            self.status.emit('disconnected')
