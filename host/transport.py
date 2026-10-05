"""Serial ownership stays in one worker; commands never originate from replay."""
import queue
import threading
import time
from collections import deque
from datetime import datetime, timezone

import serial
from PySide6.QtCore import QThread, Qt, Signal, Slot

from host.core import StreamDecoder
from host.serial_config import SerialConfig
from host.trace_recording import TraceArchiveWriter
from pathlib import Path
from host.version import HOST_VERSION, PROJECT_VERSION


class SerialWorker(QThread):
    records = Signal(list)
    status = Signal(str)
    statistics = Signal(dict)
    command_sent = Signal(str)
    raw_received = Signal(bytes)
    raw_sent = Signal(bytes)
    wire_activity = Signal(dict)
    failed = Signal(str)
    delivery_finished = Signal()
    trace_status = Signal(dict)
    _delivery_ready = Signal()

    # A stalled GUI cannot accumulate an unlimited Qt event queue. Keep raw
    # data and its decoded records together, and spend only a small batch of
    # work in each GUI turn so stop/close input can still be processed.
    MAX_PENDING_BATCHES = 512
    MAX_PENDING_BYTES = 256 * 1024
    DELIVERY_BATCHES_PER_TURN = 4
    RAW_WRITE_SLICE_S = 0.1
    READ_TIMEOUT_S = 0.02
    MIN_READ_BYTES = 64

    def __init__(self, port, parent=None, config=None, mode='project', buffered_delivery=False,
                 trace_directory=None):
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
        self._trace_directory = Path(trace_directory) if trace_directory is not None else (
            Path(__file__).resolve().parents[1] / 'Release' / 'captures' / 'control_traces')
        self._trace_writer = None
        self._trace_context = dict(port=port, host_version=HOST_VERSION,
                                   compatible_project_version=PROJECT_VERSION,
                                   runtime_log_directory=None)
        self.rx_bytes = self.tx_bytes = 0
        self._dispatch_lock = threading.Lock()
        self._command_generation = 0
        self.buffered_delivery = buffered_delivery
        self._delivery_lock = threading.Lock()
        self._pending_delivery = deque()
        self._pending_bytes = 0
        self._delivery_scheduled = False
        self._thread_done = False
        self._delivery_complete = False
        self.delivery_overflows = 0
        self._delivery_ready.connect(self._deliver_pending, Qt.ConnectionType.QueuedConnection)
        self.finished.connect(self._thread_finished)

    def send(self, command):
        if (self.mode != 'project' or not self.config.is_project_default
                or not isinstance(command, str) or len(command) != 1
                or command not in 'DUGHSRFBJKLMT'):
            return False
        # Only reservation of one in-flight write is serialized. Never hold a
        # GUI-shared lock across driver I/O: a slow/removed device must not
        # prevent S or close from being accepted. S invalidates every older
        # command which has not yet been reserved by the worker.
        cancelled_trace = False
        with self._dispatch_lock:
            if self.closing.is_set():
                return False
            if command == 'S':
                self._command_generation += 1
                while True:
                    try:
                        cancelled_trace |= self.commands.get_nowait()[0] == 'T'
                    except queue.Empty:
                        break
            try:
                self.commands.put_nowait((command, self._command_generation))
                accepted = True
            except queue.Full:
                accepted = False
        if cancelled_trace:
            self._publish([('trace_status', dict(status='request_cancelled', reason='stop_priority'))])
        return accepted

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

    def set_trace_context(self, context):
        """Only small connection/request metadata; never historical row copies."""
        with self._dispatch_lock:
            self._trace_context.update(context)

    def request_close(self, send_stop=True):
        with self._dispatch_lock:
            self.stop_on_close = (bool(send_stop) and self.mode == 'project'
                                  and self.config.is_project_default)
            self._command_generation += 1
            self.closing.set()

    @property
    def delivery_pending(self):
        with self._delivery_lock:
            return (bool(self._pending_delivery) or self._delivery_scheduled or
                    (self.isFinished() and not self._delivery_complete))

    def _statistics(self):
        stats = (dict(self.decoder.stats) if self.decoder else dict.fromkeys(
            ('frames', 'crc_errors', 'discarded_bytes', 'unsupported_frames',
             'missing_frames', 'duplicates', 'resets'), 0))
        stats.update(rx_bytes=self.rx_bytes, tx_bytes=self.tx_bytes,
                     delivery_overflows=self.delivery_overflows)
        if self.decoder:
            stats.update({'trace_' + key: value for key, value in self.decoder.trace_stats.items()})
        return stats

    def _publish(self, events, byte_count=0, terminal=False):
        if not self.buffered_delivery:
            for name, value in events:
                getattr(self, name).emit(value)
            return
        with self._delivery_lock:
            if not terminal and (len(self._pending_delivery) >= self.MAX_PENDING_BATCHES or
                    self._pending_bytes + byte_count > self.MAX_PENDING_BYTES):
                self.delivery_overflows += 1
                raise serial.SerialException(
                    f'上位机接收处理积压超过上限，本次 {byte_count} 字节未交付；'
                    '已终止连接，请检查运行日志后重新连接；机构运动时请使用 SW3')
            # Only the fixed-size failure/stop/closing tail may exceed the
            # bound. It is essential evidence and cannot be silently dropped.
            self._pending_delivery.append((events, byte_count))
            self._pending_bytes += byte_count
            wake = not self._delivery_scheduled
            self._delivery_scheduled = True
        if wake:
            self._delivery_ready.emit()

    @Slot()
    def _deliver_pending(self):
        # This slot belongs to the creating (GUI) thread, not run()'s thread.
        # Emit outside both locks; receiver callbacks may send or disconnect.
        with self._delivery_lock:
            batches = []
            for _ in range(min(self.DELIVERY_BATCHES_PER_TURN, len(self._pending_delivery))):
                events, byte_count = self._pending_delivery.popleft()
                self._pending_bytes -= byte_count
                batches.append(events)
        for events in batches:
            for name, value in events:
                getattr(self, name).emit(value)
        with self._delivery_lock:
            more = bool(self._pending_delivery)
            if not more:
                self._delivery_scheduled = False
        if more:
            # Always queued, including when emitted from the GUI itself.
            self._delivery_ready.emit()
        else:
            self._finish_delivery()

    @Slot()
    def _thread_finished(self):
        self._thread_done = True
        self._finish_delivery()

    def _finish_delivery(self):
        with self._delivery_lock:
            done = self._thread_done and not self._pending_delivery and not self._delivery_scheduled
        if done and not self._delivery_complete:
            self._delivery_complete = True
            self.delivery_finished.emit()

    def _write(self, port, payload, message, terminal=False):
        started = time.monotonic()
        try:
            written = port.write(payload)
        except (serial.SerialException, OSError, ValueError) as error:
            # Drivers do not report the partial byte count on timeout. Log the
            # attempted bytes separately instead of claiming they were sent.
            self._publish([('wire_activity', {'direction': 'TX', 'data': b'',
                'requested_hex': payload.hex(), 'requested_bytes': len(payload),
                'complete': False, 'written_bytes_known': False, 'error': str(error),
                'duration_s': time.monotonic() - started,
                'host_monotonic': time.monotonic(),
                'time_utc': datetime.now(timezone.utc).isoformat()})], terminal=True)
            raise
        count = max(0, min(written, len(payload))) if type(written) is int else 0
        self.tx_bytes += count
        self._publish([('wire_activity', {'direction': 'TX', 'data': payload[:count],
            'requested_bytes': len(payload), 'complete': written == len(payload),
            'written_bytes_known': True, 'duration_s': time.monotonic() - started,
            'host_monotonic': time.monotonic(),
            'time_utc': datetime.now(timezone.utc).isoformat()})], count, terminal=terminal)
        if written != len(payload):
            raise serial.SerialException(message)

    def _receive_once(self, port):
        # Waiting for just one byte turns a normal UART stream into thousands
        # of Python/Qt events. A small minimum read coalesces bytes while the
        # serial timeout still returns partial packets within 20 ms.
        data = port.read(min(max(port.in_waiting, self.MIN_READ_BYTES), 4096))
        if not data:
            self._trace_updates()
            return
        self.rx_bytes += len(data)
        events = [('wire_activity', {'direction': 'RX', 'data': data,
            'host_monotonic': time.monotonic(), 'time_utc': datetime.now(timezone.utc).isoformat()}),
            ('raw_received', data)]
        if self.decoder:
            records = self.decoder.feed(data)
            if records:
                events.append(('records', records))
        self._publish(events, len(data))
        self._trace_updates()

    def _trace_updates(self, terminal=False):
        if not self.decoder:
            return
        self.decoder.check_trace_timeout()
        events = []
        for event in self.decoder.take_trace_events():
            if event['type'] == 'progress':
                events.append(('trace_status', {k: v for k, v in event.items() if k != 'type'}))
                continue
            capture = event['capture']
            with self._dispatch_lock:
                capture['connection_context'] = dict(self._trace_context)
            if self._trace_writer is None:
                self._trace_writer = TraceArchiveWriter(self._trace_directory)
            events.append(('trace_status', dict(status='saving', complete=capture['complete'],
                capture_id=capture['capture_id'], received_rows=capture['received_rows'],
                total_rows=capture['total_rows'], errors=list(capture['errors']), storage_complete=False)))
            if not self._trace_writer.submit(capture):
                self._publish(events, terminal=True)
                raise serial.SerialException('连续记录写盘积压，已停止接收；失败证据正在后台保存')
        if self._trace_writer:
            events.extend(('trace_status', event) for event in self._trace_writer.take_events())
        if events:
            self._publish(events, terminal=terminal)

    def _write_raw(self, port, payload):
        # A legal 4096-byte packet already takes 356 ms at 115200/8N1;
        # submitting it as one 250 ms write can time out on Windows. Budget
        # chunks from the configured line rate, and service RX/close between
        # them instead of increasing one blocking driver's timeout.
        frame_bits = 1 + self.config.bytesize + (self.config.parity != 'N') + self.config.stopbits
        chunk_size = max(1, int(self.config.baudrate * self.RAW_WRITE_SLICE_S / frame_bits))
        for offset in range(0, len(payload), chunk_size):
            if self.closing.is_set():
                self._publish([('wire_activity', {'direction': 'TX', 'data': b'',
                    'requested_bytes': len(payload) - offset, 'complete': False,
                    'cancelled': True, 'written_bytes_known': True,
                    'error': '串口关闭，剩余发送已取消', 'host_monotonic': time.monotonic(),
                    'time_utc': datetime.now(timezone.utc).isoformat()})], terminal=True)
                return False
            chunk = payload[offset:offset + chunk_size]
            self._write(port, chunk, '通用发送数据未完整写入，部分字节可能已送达')
            if offset + len(chunk) < len(payload):
                self._receive_once(port)
        return True

    def run(self):
        port = None
        try:
            # Configure modem lines before opening; no intentional board reset.
            port = serial.Serial(port=None, **self.config.as_dict(), timeout=self.READ_TIMEOUT_S,
                                 write_timeout=0.25, rtscts=False, dsrdtr=False)
            port.dtr = False
            port.rts = False
            port.port = self.port_name
            port.open()
            self._publish([('status', 'connected')])
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
                            sent = command
                    if sent:
                        if isinstance(sent, str):
                            self._write(port, sent.encode('ascii'), '命令未完整写入')
                            if sent == 'T' and self.decoder:
                                self.decoder.begin_trace()
                                self._trace_updates()
                        elif not self._write_raw(port, sent):
                            sent = None
                if sent:
                    # Callbacks must not execute while the dispatch lock is held.
                    if isinstance(sent, str):
                        self._publish([('command_sent', sent)])
                    else:
                        self._publish([('raw_sent', sent)], len(sent))
                if self.closing.is_set():
                    break
                if queued and not sent:
                    # The dequeued command was superseded; dispatch the queued
                    # stop before entering another potentially blocking read.
                    continue
                self._receive_once(port)
                if time.monotonic() - last_stats > 0.25:
                    self._publish([('statistics', self._statistics())])
                    last_stats = time.monotonic()
        except (serial.SerialException, OSError, ValueError) as error:
            self._publish([('failed', str(error))], terminal=True)
        finally:
            with self._dispatch_lock:
                self.closing.set()
                self._command_generation += 1
            if port and port.is_open:
                try:
                    if self.stop_on_close and self.mode == 'project' and self.config.is_project_default:
                        self._write(port, b'S', '断开前停止命令未完整写入，请使用 SW3', terminal=True)
                        self._publish([('command_sent', 'S')], terminal=True)
                except (serial.SerialException, OSError) as error:
                    self._publish([('failed', f'停止命令未送达：{error}；请使用 SW3')], terminal=True)
                finally:
                    try:
                        port.close()
                    except (serial.SerialException, OSError) as error:
                        self._publish([('failed', f'串口关闭异常：{error}')], terminal=True)
            if self.decoder:
                self.decoder.finish_trace('disconnected_missing_end')
                try:
                    self._trace_updates(terminal=True)
                except (OSError, serial.SerialException) as error:
                    self._publish([('trace_status', dict(status='storage_failed', storage_complete=False,
                                                        error=str(error)))], terminal=True)
            self._publish([('statistics', self._statistics()), ('status', 'disconnected')], terminal=True)
            if self._trace_writer:
                self._trace_writer.request_close()
                # Port is already closed. Waiting here never blocks the GUI or
                # holds its locks; delivery_finished is deferred until all
                # archive results have reached the UI, including close errors.
                while not self._trace_writer.done.wait(.05):
                    self._trace_updates(terminal=True)
                self._trace_updates(terminal=True)
