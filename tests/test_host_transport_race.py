"""Deterministic serial dispatch races; no physical device is opened."""
import queue
import threading
import unittest
from unittest.mock import patch

try:
    from host.transport import SerialWorker
except ModuleNotFoundError as error:
    if error.name not in {'PySide6', 'serial'}:
        raise
    DEPENDENCY_ERROR = str(error)
else:
    DEPENDENCY_ERROR = None


class InterceptQueue(queue.Queue):
    """Run a UI action after dequeue, before the worker can commit its write."""
    def __init__(self, command, action):
        super().__init__(maxsize=8)
        self.command = command
        self.action = action
        self.fired = False

    def get_nowait(self):
        item = super().get_nowait()
        if item[0] == self.command and not self.fired:
            self.fired = True
            self.action()
        return item


class MemoryPort:
    def __init__(self, worker, on_write=None, on_read=None):
        self.worker = worker
        self.on_write = on_write
        self.on_read = on_read
        self.is_open = False
        self.in_waiting = 0
        self.writes = []

    def open(self):
        self.is_open = True

    def write(self, data):
        self.writes.append(data)
        if self.on_write:
            self.on_write(data)
        return len(data)

    def read(self, count):
        if self.on_read:
            self.on_read()
        elif self.writes and self.worker.commands.empty():
            self.worker.request_close(False)
        return b''

    def close(self):
        self.is_open = False


@unittest.skipIf(DEPENDENCY_ERROR, f'上位机依赖未安装：{DEPENDENCY_ERROR}')
class DispatchRaceTests(unittest.TestCase):
    def run_memory(self, worker, port):
        with patch('host.transport.serial.Serial', return_value=port):
            worker.run()
        self.assertFalse(port.is_open)

    def intercepted(self, command, action):
        worker = SerialWorker('TEST-NO-DEVICE')
        worker.commands = InterceptQueue(command, lambda: action(worker))
        self.assertTrue(worker.send(command))
        port = MemoryPort(worker)
        self.run_memory(worker, port)
        self.assertTrue(worker.commands.fired)
        return port.writes

    def test_stop_cancels_dequeued_start_before_write(self):
        writes = self.intercepted('G', lambda worker: self.assertTrue(worker.send('S')))
        self.assertEqual(writes, [b'S'])

    def test_stop_cancels_dequeued_calibration_before_write(self):
        writes = self.intercepted('D', lambda worker: self.assertTrue(worker.send('S')))
        self.assertEqual(writes, [b'S'])

    def test_close_cancels_dequeued_motion_and_sends_final_stop(self):
        writes = self.intercepted('F', lambda worker: worker.request_close(True))
        self.assertEqual(writes, [b'S'])

    def test_close_without_stop_still_cancels_dequeued_motion(self):
        writes = self.intercepted('B', lambda worker: worker.request_close(False))
        self.assertEqual(writes, [])

    def test_stop_is_accepted_with_full_queue(self):
        worker = SerialWorker('TEST-NO-DEVICE')
        for _ in range(8):
            self.assertTrue(worker.send('G'))
        self.assertFalse(worker.send('F'))
        self.assertTrue(worker.send('S'))
        port = MemoryPort(worker)
        self.run_memory(worker, port)
        self.assertEqual(port.writes, [b'S'])

    def test_new_explicit_command_after_stop_uses_new_generation(self):
        worker = SerialWorker('TEST-NO-DEVICE')
        self.assertTrue(worker.send('G'))
        self.assertTrue(worker.send('S'))
        self.assertTrue(worker.send('F'))
        port = MemoryPort(worker)
        self.run_memory(worker, port)
        self.assertEqual(port.writes, [b'S', b'F'])

    def started_write_race(self, close):
        worker = SerialWorker('TEST-NO-DEVICE')
        self.assertTrue(worker.send('G'))
        if close:
            self.assertTrue(worker.send('F'))
        started, release, attempted, accepted = (threading.Event() for _ in range(4))
        failures = []

        def on_write(data):
            if data == b'G':
                started.set()
                if not release.wait(2):
                    raise AssertionError('test write was not released')

        def on_read():
            if not accepted.wait(2):
                raise AssertionError('UI action did not complete')
            if not close and port.writes == [b'G', b'S']:
                worker.request_close(False)

        port = MemoryPort(worker, on_write, on_read)

        def ui_action():
            attempted.set()
            if close:
                worker.request_close(True)
            elif not worker.send('S'):
                failures.append('S not accepted')
            accepted.set()

        def run_worker():
            try:
                worker.run()
            except Exception as error:
                failures.append(error)

        with patch('host.transport.serial.Serial', return_value=port):
            thread = threading.Thread(target=run_worker, daemon=True)
            thread.start()
            self.assertTrue(started.wait(2))
            sender = threading.Thread(target=ui_action, daemon=True)
            sender.start()
            try:
                self.assertTrue(attempted.wait(2))
                self.assertTrue(accepted.wait(.2), 'UI action blocked behind driver I/O')
            finally:
                release.set()
            self.assertTrue(accepted.wait(2))
            sender.join(2)
            thread.join(2)
            self.assertFalse(sender.is_alive())
            self.assertFalse(thread.is_alive())
        self.assertEqual(failures, [])
        self.assertFalse(port.is_open)
        self.assertEqual(port.writes, [b'G', b'S'])

    def test_stop_acceptance_never_waits_for_already_started_write(self):
        self.started_write_race(close=False)

    def test_close_acceptance_never_waits_for_write_and_discards_next_motion(self):
        self.started_write_race(close=True)

    def test_wire_activity_callback_can_request_stop_without_deadlock(self):
        worker = SerialWorker('TEST-NO-DEVICE')
        self.assertTrue(worker.send('G'))
        self.assertTrue(worker.send('F'))
        actions = []

        def received(activity):
            if activity['direction'] == 'TX' and activity['data'] == b'G':
                actions.append(worker.send('S'))

        # A direct callback is deliberately used to exercise reentrancy. Real
        # GUI delivery also uses this path after its bounded inbox is drained.
        from PySide6.QtCore import Qt
        worker.wire_activity.connect(received, Qt.ConnectionType.DirectConnection)
        port = MemoryPort(worker)
        with patch('host.transport.serial.Serial', return_value=port):
            thread = threading.Thread(target=worker.run, daemon=True)
            thread.start()
            thread.join(2)
        self.assertFalse(thread.is_alive(), 'wire activity emitted under dispatch lock')
        self.assertEqual(actions, [True])
        self.assertEqual(port.writes, [b'G', b'S'])


if __name__ == '__main__':
    unittest.main()
