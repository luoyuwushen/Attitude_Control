"""Optional per-connection evidence; bounded writing never blocks command dispatch."""
from copy import deepcopy
from datetime import datetime, timezone
import json
from pathlib import Path
import queue
import threading
import time
import uuid


class OperationLog:
    def __init__(self, output_root, metadata):
        self.root = Path(output_root)
        self.metadata = deepcopy(metadata)
        self.directory = None
        self.error = None
        self.closed = False
        self._queue = queue.Queue(maxsize=2048)
        self._closing = threading.Event()
        self._thread = None

    def check(self):
        if self.error:
            raise OSError(f'日志保存失败，记录可能不完整：{self.error}')

    def write_serial(self, direction, data, **details):
        if direction not in ('RX', 'TX') or not isinstance(data, bytes):
            raise ValueError('串口日志需要 RX/TX 与原始 bytes')
        self._put('serial', dict(details, direction=direction, hex=data.hex(), bytes=len(data)))

    def write_control(self, event, **details):
        self._put('control', dict(details, event=event))

    def write_serial_event(self, event, **details):
        self._put('serial', dict(details, event=event))

    def _put(self, channel, event):
        self.check()
        if self.closed:
            raise OSError('日志会话已关闭')
        event = deepcopy(event)
        event.setdefault('time_utc', datetime.now(timezone.utc).isoformat())
        event.setdefault('host_monotonic', time.monotonic())
        # Reject invalid evidence before handing it to the writer.
        json.dumps(event, ensure_ascii=False, allow_nan=False)
        try:
            self._queue.put_nowait((channel, event))
        except queue.Full as error:
            self.error = '写入队列已满；日志不是完整记录'
            self._closing.set()
            raise OSError(self.error) from error
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name='J280 operation log', daemon=True)
            self._thread.start()

    def _run(self):
        streams = {}
        try:
            identifier = datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '_' + uuid.uuid4().hex[:8]
            self.directory = self.root / 'logs' / identifier
            self.directory.mkdir(parents=True, exist_ok=False)
            metadata = dict(self.metadata, schema_version=1,
                            created_utc=datetime.now(timezone.utc).isoformat())
            (self.directory / 'metadata.json').write_text(
                json.dumps(metadata, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
            while not self._closing.is_set() or not self._queue.empty():
                try:
                    channel, event = self._queue.get(timeout=0.05)
                except queue.Empty:
                    continue
                if channel not in streams:
                    streams[channel] = (self.directory / (channel + '.jsonl')).open('x', encoding='utf-8')
                stream = streams[channel]
                stream.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + '\n')
                stream.flush()
        except (OSError, ValueError, TypeError) as error:
            self.error = str(error)
        finally:
            for stream in streams.values():
                try:
                    stream.close()
                except OSError as error:
                    self.error = str(error)

    def close(self, timeout=1.2):
        self.closed = True
        self._closing.set()
        if self._thread:
            self._thread.join(timeout)
            if self._thread.is_alive():
                raise OSError('日志写入尚未完成，尾部记录可能不完整')
        self.check()
