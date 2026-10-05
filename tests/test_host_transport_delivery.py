"""Real Qt worker/GUI boundary with deterministic ports; no hardware access."""
import os
import time
import unittest
from collections import deque
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

try:
    import serial
    from PySide6.QtWidgets import QApplication
    from host.core import demo_record
    from host.transport import SerialWorker
except ModuleNotFoundError as error:
    if error.name not in {'PySide6', 'serial'}:
        raise
    DEPENDENCY_ERROR = str(error)
else:
    DEPENDENCY_ERROR = None


class BurstPort:
    def __init__(self, worker, chunks=(), write_error=None):
        self.worker = worker
        self.chunks = deque(chunks)
        self.write_error = write_error
        self.is_open = False
        self.writes = []

    @property
    def in_waiting(self):
        return len(self.chunks[0]) if self.chunks else 0

    def open(self):
        self.is_open = True

    def write(self, data):
        self.writes.append(data)
        if self.write_error and data != b'S':
            raise self.write_error
        return len(data)

    def read(self, count):
        if self.chunks:
            data = self.chunks.popleft()
            if len(data) > count:
                self.chunks.appendleft(data[count:])
            return data[:count]
        self.worker.request_close(True)
        return b''

    def close(self):
        self.is_open = False


@unittest.skipIf(DEPENDENCY_ERROR, f'上位机依赖未安装：{DEPENDENCY_ERROR}')
class BufferedDeliveryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.application = QApplication.instance() or QApplication([])

    def observe(self, worker):
        events = []
        for name in ('status', 'wire_activity', 'raw_received', 'records',
                     'command_sent', 'raw_sent', 'statistics', 'failed'):
            getattr(worker, name).connect(lambda value, name=name: events.append((name, value)))
        worker.delivery_finished.connect(lambda: events.append(('delivery_finished', None)))
        return events

    def drain(self, worker, events):
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and not any(name == 'delivery_finished' for name, _ in events):
            self.application.processEvents()
        self.assertEqual(events[-1][0], 'delivery_finished')
        self.assertEqual(events[-2], ('status', 'disconnected'))
        self.assertFalse(worker.delivery_pending)
        self.assertFalse(worker.isRunning())

    def run_without_gui(self, worker, port):
        with patch('host.transport.serial.Serial', return_value=port):
            worker.start()
            self.assertTrue(worker.wait(2000), 'worker blocked by paused GUI')
        self.assertFalse(port.is_open)

    def test_gui_pause_preserves_order_timestamps_and_final_stop_before_finished(self):
        worker = SerialWorker('TEST-NO-DEVICE', buffered_delivery=True)
        events = self.observe(worker)
        chunks = [bytes.fromhex(demo_record(2, index)['raw_hex']) for index in range(12)]
        port = BurstPort(worker, chunks)
        self.run_without_gui(worker, port)
        self.assertEqual(events, [], 'worker bypassed buffered delivery')
        self.assertTrue(worker.delivery_pending)
        resumed = time.monotonic()
        self.drain(worker, events)
        samples = [sample for name, values in events if name == 'records' for sample in values]
        self.assertEqual([sample['sequence'] for sample in samples], list(range(12)))
        self.assertTrue(all(sample['host_monotonic'] <= resumed for sample in samples))
        self.assertEqual([data for name, data in events if name == 'raw_received'], chunks)
        self.assertEqual([value for name, value in events if name == 'command_sent'], ['S'])
        self.assertEqual(port.writes, [b'S'])
        self.assertEqual(events[0], ('status', 'connected'))

    def test_sustained_gui_stall_has_bounded_queue_and_explicit_failure_with_stop(self):
        worker = SerialWorker('TEST-NO-DEVICE', buffered_delivery=True)
        worker.MAX_PENDING_BATCHES = 8
        events = self.observe(worker)
        chunk = bytes.fromhex(demo_record(0, 2)['raw_hex'])
        port = BurstPort(worker, [chunk] * 100)
        self.run_without_gui(worker, port)
        self.assertEqual(events, [])
        # The finite error + stop + closing tail is preserved over the cap.
        self.assertLessEqual(len(worker._pending_delivery), worker.MAX_PENDING_BATCHES + 4)
        self.assertEqual(worker.delivery_overflows, 1)
        self.assertEqual(port.writes, [b'S'])
        self.drain(worker, events)
        failures = [value for name, value in events if name == 'failed']
        self.assertEqual(len(failures), 1)
        self.assertIn('积压超过上限', failures[0])
        self.assertIn('24 字节未交付', failures[0])
        statistics = [value for name, value in events if name == 'statistics'][-1]
        self.assertEqual(statistics['delivery_overflows'], 1)
        self.assertLess(statistics['rx_bytes'], 100 * len(chunk))

    def test_bytewise_uart_arrival_is_coalesced_by_the_read_deadline(self):
        worker = SerialWorker('TEST-NO-DEVICE', buffered_delivery=True)
        events = self.observe(worker)
        wire = b''.join(bytes.fromhex(demo_record(2, index)['raw_hex']) for index in range(10))
        received_counts = []

        class PacedPort(BurstPort):
            def __init__(self):
                super().__init__(worker)
                self.remaining = wire

            @property
            def in_waiting(self):
                # At the start of every read the next UART byte is still in
                # flight; about twenty bytes arrive over the 20 ms deadline.
                return 0

            def read(self, count):
                received_counts.append(count)
                count = min(count, int(worker.READ_TIMEOUT_S / .001))
                data, self.remaining = self.remaining[:count], self.remaining[count:]
                if not self.remaining:
                    worker.request_close(True)
                return data

        port = PacedPort()
        self.run_without_gui(worker, port)
        self.drain(worker, events)
        chunks = [value for name, value in events if name == 'raw_received']
        self.assertEqual(b''.join(chunks), wire)
        self.assertEqual(len(chunks), 12)
        self.assertTrue(all(count >= 64 for count in received_counts))
        self.assertEqual([sample['sequence'] for name, values in events if name == 'records'
                          for sample in values], list(range(10)))
        self.assertEqual(worker.delivery_overflows, 0)

    def test_finished_thread_stays_pending_until_gui_observes_its_finish(self):
        worker = SerialWorker('TEST-NO-DEVICE', buffered_delivery=True)
        events = self.observe(worker)
        self.run_without_gui(worker, BurstPort(worker))
        # Model the small window where all data has drained but QThread's
        # finished MetaCall has not reached the GUI yet.
        while worker._pending_delivery:
            worker._deliver_pending()
        self.assertFalse(worker._delivery_scheduled)
        self.assertTrue(worker.delivery_pending)
        self.drain(worker, events)

    def test_raw_byte_bound_also_applies_with_few_large_reads(self):
        worker = SerialWorker('TEST-NO-DEVICE', mode='raw', buffered_delivery=True)
        worker.MAX_PENDING_BYTES = 6000
        events = self.observe(worker)
        port = BurstPort(worker, [b'x' * 4096] * 4)
        self.run_without_gui(worker, port)
        self.assertLessEqual(worker._pending_bytes, worker.MAX_PENDING_BYTES)
        self.assertEqual(port.writes, [])
        self.drain(worker, events)
        self.assertEqual([len(value) for name, value in events if name == 'raw_received'], [4096])
        self.assertTrue(any(name == 'failed' and '积压' in value for name, value in events))

    def test_each_delivery_turn_has_a_fixed_work_budget(self):
        worker = SerialWorker('TEST-NO-DEVICE', mode='raw', buffered_delivery=True)
        events = self.observe(worker)
        for index in range(20):
            worker._publish([('raw_received', bytes([index]))], 1)
        worker._deliver_pending()
        self.assertEqual(len(events), worker.DELIVERY_BATCHES_PER_TURN)
        self.assertTrue(worker.delivery_pending)
        self.assertEqual(len(worker._pending_delivery), 20 - worker.DELIVERY_BATCHES_PER_TURN)
        # Complete the synthetic session so its queued wakeups cannot leak
        # into another test's event processing.
        worker._thread_finished()
        deadline = time.monotonic() + 2
        while worker.delivery_pending and time.monotonic() < deadline:
            self.application.processEvents()
        self.assertFalse(worker.delivery_pending)
        self.assertEqual(events[-1][0], 'delivery_finished')
        self.assertEqual(len([value for name, value in events if name == 'raw_received']), 20)

    def test_timeout_logs_unknown_partial_write_without_claiming_success(self):
        worker = SerialWorker('TEST-NO-DEVICE', buffered_delivery=True)
        events = self.observe(worker)
        self.assertTrue(worker.send('G'))
        port = BurstPort(worker, write_error=serial.SerialTimeoutException('write timed out'))
        self.run_without_gui(worker, port)
        self.drain(worker, events)
        writes = [value for name, value in events if name == 'wire_activity']
        self.assertEqual(writes[0]['data'], b'')
        self.assertEqual(writes[0]['requested_hex'], b'G'.hex())
        self.assertFalse(writes[0]['written_bytes_known'])
        self.assertFalse(writes[0]['complete'])
        self.assertEqual([value for name, value in events if name == 'command_sent'], ['S'])
        self.assertEqual(port.writes, [b'G', b'S'])
        self.assertTrue(any(name == 'failed' and 'timed out' in value for name, value in events))


if __name__ == '__main__':
    unittest.main()
