"""Bounded asynchronous experiment recording; file ownership stays in one thread."""
from collections import deque
import json
from pathlib import Path
import threading
import time

from host import core


class AsyncSessionRecorder:
    """Accept immutable JSON snapshots, then persist them in FIFO order.

    Pending bytes count UTF-8 JSON payload bytes, including the in-flight item;
    the separate item limit bounds envelope overhead. ``written`` counts
    successful sample/event calls, not fsync durability. Only ``closed`` with
    a summary means that all accepted items and final file closure succeeded.
    """

    def __init__(self, directory, metadata, capacity=2048,
                 max_pending_bytes=8*1024*1024, *, writer_factory=None):
        if type(capacity) is not int or capacity < 1:
            raise ValueError('Recording capacity must be a positive integer')
        if type(max_pending_bytes) is not int or max_pending_bytes < 1:
            raise ValueError('Recording byte limit must be a positive integer')
        self._root = Path(directory)
        self._metadata = self._snapshot(dict(metadata))
        self._factory = writer_factory or core.SessionRecorder
        self._capacity = capacity
        self._max_pending_bytes = max_pending_bytes
        self._condition = threading.Condition()
        self._queue = deque()
        self._state = 'starting'
        self._directory = self._summary = self._error = None
        self._closing = False
        self._extra_summary = None
        self._pending_count = self._pending_bytes = 0
        self._accepted = self._written = self._rejected = 0
        self.done = threading.Event()
        self._thread = threading.Thread(target=self._run,
                                        name='J280 experiment writer', daemon=True)
        self._thread.start()

    @staticmethod
    def _snapshot(value):
        # JSON bytes form an immutable deep snapshot and a deterministic byte
        # budget. Match the synchronous recorder's Path/nonfinite normalization.
        return json.dumps(core._clean_json(value), ensure_ascii=False,
                          allow_nan=False, separators=(',', ':')).encode('utf-8')

    def _fail_locked(self, error):
        if self._error is None:
            self._error = str(error) or type(error).__name__
        self._state = 'failed'
        self._closing = True
        self._condition.notify_all()

    def _submit(self, kind, payload):
        size = len(payload)
        with self._condition:
            if self._closing:
                self._rejected += 1
                raise OSError(self._error or 'Experiment recording is closing or closed')
            if (self._pending_count >= self._capacity or
                    self._pending_bytes + size > self._max_pending_bytes):
                self._rejected += 1
                self._fail_locked('Experiment recording queue is full; records are incomplete')
                raise OSError(self._error)
            self._queue.append((kind, payload))
            self._pending_count += 1
            self._pending_bytes += size
            self._accepted += 1
            self._condition.notify()

    def write(self, record):
        self._submit('sample', self._snapshot(dict(record)))

    def event(self, kind, detail, record=None):
        payload = {'kind': str(kind), 'detail': detail,
                   'record': dict(record) if record is not None else None,
                   'timestamp': time.time()}
        self._submit('event', self._snapshot(payload))

    def request_close(self, extra_summary=None):
        with self._condition:
            if self._closing:
                return
        snapshot = self._snapshot(dict(extra_summary) if extra_summary is not None else {})
        with self._condition:
            if self._closing:
                return
            self._extra_summary = snapshot
            self._closing = True
            self._state = 'closing'
            self._condition.notify_all()

    def poll(self):
        with self._condition:
            return {'state': self._state, 'directory': self._directory,
                    'error': self._error, 'summary': self._summary,
                    'pending_count': self._pending_count,
                    'pending_bytes': self._pending_bytes,
                    'accepted': self._accepted, 'written': self._written,
                    'rejected': self._rejected}

    def join(self, timeout=None):
        """Wait only in tests/non-GUI callers; return whether cleanup finished."""
        self._thread.join(timeout)
        return self.done.is_set()

    def _run(self):
        writer = None
        try:
            writer = self._factory(self._root, json.loads(self._metadata),
                                   background_flush=False)
            with self._condition:
                self._directory = writer.directory
                if self._error is None:
                    self._state = 'closing' if self._closing else 'recording'
            interval = core.SessionRecorder.FLUSH_INTERVAL_S
            deadline = time.monotonic() + interval
            while True:
                with self._condition:
                    while not self._queue and not self._closing:
                        remaining = deadline-time.monotonic()
                        if remaining <= 0:
                            break
                        self._condition.wait(remaining)
                    if not self._queue and self._closing:
                        break
                    item = self._queue.popleft() if self._queue else None
                if item is not None:
                    kind, payload = item
                    value = json.loads(payload)
                    if kind == 'sample':
                        writer.write(value)
                    else:
                        writer.event(value['kind'], value['detail'], value['record'],
                                     timestamp=value['timestamp'])
                    with self._condition:
                        self._pending_count -= 1
                        self._pending_bytes -= len(payload)
                        self._written += 1
                # Check even with a perpetually non-empty queue: a burst must
                # not starve the one-second flush deadline.
                if time.monotonic() >= deadline:
                    writer.flush()
                    deadline = time.monotonic() + interval
        except Exception as error:
            with self._condition:
                self._fail_locked(error)
                self._queue.clear()
                self._pending_count = self._pending_bytes = 0
        finally:
            if writer is not None:
                with self._condition:
                    error = self._error
                    extra_summary = self._extra_summary
                try:
                    writer.extra_summary = json.loads(extra_summary) if extra_summary else {}
                    if error is not None:
                        writer.invalidate(OSError(error))
                except Exception as cleanup_error:
                    with self._condition:
                        self._fail_locked(cleanup_error)
                try:
                    summary = writer.close()
                except Exception as cleanup_error:
                    with self._condition:
                        self._fail_locked(cleanup_error)
                else:
                    with self._condition:
                        if self._error is None:
                            self._summary = summary
                            self._state = 'closed'
            with self._condition:
                self._queue.clear()
                self._pending_count = self._pending_bytes = 0
            self.done.set()
