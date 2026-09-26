"""Serial ownership stays in one worker; commands never originate from replay."""
import queue
import threading
import time

import serial
from PySide6.QtCore import QThread, Signal

from host.core import StreamDecoder


class SerialWorker(QThread):
    records = Signal(list)
    status = Signal(str)
    statistics = Signal(dict)
    command_sent = Signal(str)
    failed = Signal(str)

    def __init__(self, port, parent=None):
        super().__init__(parent)
        self.port_name = port
        self.commands = queue.Queue(maxsize=8)
        self.closing = threading.Event()
        self.stop_on_close = True
        self.decoder = StreamDecoder()
        self._dispatch_lock = threading.Lock()
        self._command_generation = 0

    def send(self, command):
        if command not in 'DUGSRFB' or len(command) != 1:
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

    def request_close(self, send_stop=True):
        with self._dispatch_lock:
            self.stop_on_close = send_stop
            self._command_generation += 1
            self.closing.set()

    def run(self):
        port = None
        try:
            # Configure modem lines before opening; no intentional board reset.
            port = serial.Serial(port=None, baudrate=115200, timeout=0.04,
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
                            if port.write(command.encode('ascii')) != 1:
                                raise serial.SerialException('命令未完整写入')
                            sent = command
                if sent:
                    # Callbacks must not execute while the dispatch lock is held.
                    self.command_sent.emit(sent)
                if self.closing.is_set():
                    break
                if queued and not sent:
                    # The dequeued command was superseded; dispatch the queued
                    # stop before entering another potentially blocking read.
                    continue
                data = port.read(min(max(port.in_waiting, 1), 4096))
                if data:
                    records = self.decoder.feed(data)
                    if records:
                        self.records.emit(records)
                if time.monotonic() - last_stats > 0.25:
                    self.statistics.emit(dict(self.decoder.stats))
                    last_stats = time.monotonic()
        except (serial.SerialException, OSError, ValueError) as error:
            self.failed.emit(str(error))
        finally:
            with self._dispatch_lock:
                self.closing.set()
                self._command_generation += 1
            if port and port.is_open:
                try:
                    if self.stop_on_close:
                        if port.write(b'S') == 1:
                            self.command_sent.emit('S')
                        else:
                            self.failed.emit('断开前停止命令未完整写入，请使用 SW3')
                except (serial.SerialException, OSError) as error:
                    self.failed.emit(f'停止命令未送达：{error}；请使用 SW3')
                finally:
                    try:
                        port.close()
                    except (serial.SerialException, OSError) as error:
                        self.failed.emit(f'串口关闭异常：{error}')
            self.statistics.emit(dict(self.decoder.stats))
            self.status.emit('disconnected')
