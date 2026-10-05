"""Offline Qt acceptance: stalled experiment storage must not own the GUI thread."""
import csv
import json
import os
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PySide6.QtCore import QTimer
from PySide6.QtGui import QCloseEvent
from PySide6.QtWidgets import QApplication

from host.app import Window
from host.core import SessionRecorder, demo_record
from host.recording import AsyncSessionRecorder
from test_host_ui import WorkerStub


class DiskGate:
    """Release on explicit permission; bounded wait makes a sync regression fail safely."""
    def __init__(self, stage, error=None):
        self.stage = stage
        self.error = error
        self.entered = threading.Event()
        self.release = threading.Event()
        self.timed_out = False
        self.calls = []

    def hit(self, stage):
        self.calls.append((stage, threading.get_ident()))
        if stage == self.stage and not self.entered.is_set():
            self.entered.set()
            if not self.release.wait(3):
                self.timed_out = True
                raise OSError('test disk barrier timed out: ' + stage)
            if self.error:
                raise OSError(self.error)

    def factory(self, *args, **kwargs):
        gate = self

        class Stream:
            def __init__(self, stream, kind):
                self.stream, self.kind = stream, kind

            def __getattr__(self, name):
                return getattr(self.stream, name)

            def write(self, value):
                gate.hit(self.kind + '_write')
                return self.stream.write(value)

            def flush(self):
                gate.hit('file_flush')
                return self.stream.flush()

            def close(self):
                gate.hit('close')
                return self.stream.close()

        class GatedSession(SessionRecorder):
            def __init__(self, *a, **kw):
                # Simulate a stalled constructor (mkdir/metadata/open). All actual
                # persistence below still uses the production SessionRecorder.
                gate.hit('init')
                super().__init__(*a, **kw)
                self._wrap_csv()
                self._raw = Stream(self._raw, 'raw')
                self._events = Stream(self._events, 'event')

            def _wrap_csv(self):
                self._csv = Stream(self._csv, 'csv')
                self._writer = csv.DictWriter(self._csv, fieldnames=self._fields)

            def _extend_fields(self, fields):
                gate.hit('extend')
                super()._extend_fields(fields)
                self._wrap_csv()

            def _flush_files(self):
                gate.hit('flush')
                return super()._flush_files()

        return GatedSession(*args, **kwargs)


class RecordingResponsivenessTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.application = QApplication.instance() or QApplication([])

    def setUp(self):
        self.gui_thread = threading.get_ident()
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        with patch('host.app.list_ports.comports', return_value=[]):
            self.window = Window(self.root)
        self.window.timer.stop()
        self.window.plot_timer.stop()
        self.window.reset_data('serial')
        self.worker = WorkerStub()
        self.window.worker = self.worker
        self.window.serial_ready = True
        self.gates = []
        self.recorders = []
        self.window.receive([self.row(0)])

    def tearDown(self):
        for gate in self.gates:
            gate.release.set()
        for recorder in self.recorders:
            recorder.request_close()
            self.assertTrue(recorder.join(2), 'offline writer did not clean up')
        self.window.exit_pending = False
        self.window.poll_recording()
        self.window.worker = None
        self.window.close()
        self.application.processEvents()
        self.window.deleteLater()
        self.application.processEvents()
        self.tmp.cleanup()

    @staticmethod
    def row(sequence):
        row = demo_record(sequence * .02, sequence)
        row.update(source='offline-test', host_monotonic=time.monotonic())
        return row

    def wait_until(self, predicate, timeout=1):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            time.sleep(.001)
        self.assertTrue(predicate(), 'background writer failed to reach expected state')

    def start(self, gate, capacity=2048):
        self.gates.append(gate)

        def factory(*args, **kwargs):
            recorder = AsyncSessionRecorder(*args, **kwargs, writer_factory=gate.factory,
                                            capacity=capacity)
            self.recorders.append(recorder)
            return recorder

        with patch('host.app.AsyncSessionRecorder', side_effect=factory):
            self.assertTrue(self.window.start_recording({'experiment': 'offline disk barrier'}))
        return self.window.recorder

    def assert_blocked(self, gate, recorder):
        self.assertTrue(gate.entered.is_set())
        self.assertFalse(gate.release.is_set())
        self.assertFalse(gate.timed_out)
        self.assertFalse(recorder.done.is_set())
        self.assertTrue(gate.calls)
        writer_threads = {thread for _, thread in gate.calls}
        self.assertEqual(len(writer_threads), 1)
        self.assertNotIn(self.gui_thread, writer_threads)

    def while_blocked(self, gate, recorder, action):
        self.assert_blocked(gate, recorder)
        started = time.monotonic()
        result = action()
        # State/thread assertions are primary; this catches a GUI wait which
        # returns only when the three-second emergency barrier expires.
        self.assertLess(time.monotonic() - started, .5)
        self.assert_blocked(gate, recorder)
        return result

    def stop_callback(self, gate, recorder):
        results = []
        QTimer.singleShot(0, lambda: results.append(
            (threading.get_ident(), self.window.send_command('S'))))
        self.while_blocked(gate, recorder, self.application.processEvents)
        self.assertEqual(results, [(self.gui_thread, True)])
        self.assertEqual(self.worker.sent[-1:], ['S'])

    def read_samples(self, recorder):
        with (recorder.poll()['directory'] / 'samples.csv').open(
                encoding='utf-8-sig', newline='') as stream:
            return list(csv.DictReader(stream))

    def exercise_storage_stage(self, stage):
        gate = DiskGate(stage)
        interval = .02 if stage == 'flush' else 60
        accepted = []
        with patch.object(SessionRecorder, 'FLUSH_INTERVAL_S', interval):
            recorder = self.start(gate)
            if stage != 'init':
                accepted.append(self.row(1))
                self.window.receive([accepted[-1]])
            if stage == 'extend':
                self.wait_until(lambda: recorder.poll()['written'] == 2)
                accepted.append(dict(self.row(2), offline_extra_column=17))
                self.window.receive([accepted[-1]])
            if stage == 'close':
                self.window.mark_text.setText('before finish')
                self.window.mark_event()
                self.window.finish_recording()
            self.assertTrue(gate.entered.wait(1))
            self.assert_blocked(gate, recorder)

            sample = self.row(10)
            if stage != 'close':
                accepted.append(sample)
            self.while_blocked(gate, recorder, lambda: self.window.receive([sample]))
            self.assertEqual(self.window.latest['sequence'], 10)
            self.window.mark_text.setText('released under disk stall')
            self.while_blocked(gate, recorder, self.window.mark_event)
            self.assertEqual(self.window.mark_text.text(), '')
            self.stop_callback(gate, recorder)
            self.while_blocked(gate, recorder, self.window.finish_recording)
            self.assertIsNone(self.window.recorder)
            self.assertIs(self.window.closing_recorder, recorder)
            self.assertFalse(self.window.record_button.isEnabled())
            self.assertFalse(self.while_blocked(gate, recorder, lambda:
                             self.window.start_recording({'experiment': 'must not overlap'})))
            self.assertNotIn('已保存', self.window.record_path.text())

            event = QCloseEvent()
            self.while_blocked(gate, recorder, lambda: self.window.closeEvent(event))
            self.assertFalse(event.isAccepted())
            self.assertTrue(self.window.timer.isActive())
            self.assertIsNone(recorder.poll()['summary'])
            self.assertNotIn('已保存', self.window.message.text())

            gate.release.set()
            self.assertTrue(recorder.done.wait(2))
            self.window.poll_recording()
            status = recorder.poll()
            self.assertEqual(status['state'], 'closed')
            self.assertIsNone(status['error'])
            self.assertEqual(status['accepted'], status['written'])
            self.assertEqual(status['pending_count'], 0)
            self.assertIsNone(self.window.closing_recorder)
            self.assertIn('已保存', self.window.record_path.text())
            summary = json.loads(status['summary'].read_text(encoding='utf-8'))
            self.assertEqual(summary['samples'], len(accepted))
            rows = self.read_samples(recorder)
            self.assertEqual([int(row['sequence']) for row in rows],
                             [row['sequence'] for row in accepted])
            self.assertEqual((status['directory'] / 'raw_frames.bin').read_bytes(),
                             b''.join(bytes.fromhex(row['raw_hex']) for row in accepted))
            if stage == 'extend':
                self.assertEqual(rows[1]['offline_extra_column'], '17')
                self.assertEqual(rows[0]['offline_extra_column'], '')
            events = [json.loads(line) for line in
                      (status['directory'] / 'events.jsonl').read_text(encoding='utf-8').splitlines()]
            self.assertEqual([e['detail'] for e in events if e['kind'] == 'marker'],
                             ['before finish' if stage == 'close' else 'released under disk stall'])
            self.assertEqual({thread for _, thread in gate.calls}, {recorder._thread.ident})
            final = QCloseEvent()
            self.window.closeEvent(final)
            self.assertTrue(final.isAccepted())
            self.assertFalse(self.window.timer.isActive())

    def test_blocked_initialization_keeps_gui_and_stop_live(self):
        self.exercise_storage_stage('init')

    def test_blocked_csv_write_keeps_gui_and_stop_live(self):
        self.exercise_storage_stage('csv_write')

    def test_blocked_raw_write_keeps_gui_and_stop_live(self):
        self.exercise_storage_stage('raw_write')

    def test_blocked_flush_keeps_gui_and_stop_live(self):
        self.exercise_storage_stage('flush')

    def test_blocked_dynamic_column_rewrite_preserves_rows_and_stop(self):
        self.exercise_storage_stage('extend')

    def test_blocked_file_close_defers_exit_without_false_saved_status(self):
        self.exercise_storage_stage('close')

    def test_idle_flush_failure_is_detected_by_display_without_another_sample(self):
        gate = DiskGate('flush', error='offline idle flush failed')
        with patch.object(SessionRecorder, 'FLUSH_INTERVAL_S', .02):
            recorder = self.start(gate)
            self.assertTrue(gate.entered.wait(1))
            self.while_blocked(gate, recorder, self.window.update_display)
            count = len(self.window.history)
            gate.release.set()
            self.assertTrue(recorder.done.wait(2))
            self.window.update_display()
        self.assertEqual(len(self.window.history), count)
        self.assertIsNone(self.window.recorder)
        self.assertIsNone(self.window.closing_recorder)
        self.assertIn('offline idle flush failed', self.window.message.text())
        self.assertIn('不完整', self.window.record_path.text())
        self.assertIsNone(recorder.poll()['summary'])
        self.assertFalse((recorder.poll()['directory'] / 'summary.json').exists())
        self.assertTrue(self.window.send_command('S'))

    def test_source_switch_cannot_mix_tail_records_or_start_an_overlapping_session(self):
        self.window.worker = None
        self.window.reset_data('replay')
        gate = DiskGate('csv_write')
        recorder = self.start(gate)
        self.window.receive([self.row(11)])
        self.assertTrue(gate.entered.wait(1))
        self.window.mark_text.setText('old replay marker')
        self.window.mark_event()
        self.while_blocked(gate, recorder, self.window.start_demo)
        self.assertEqual(self.window.source, 'demo')
        self.window.receive([self.row(900)])
        self.window.mark_text.setText('new demo marker')
        self.window.mark_event()
        self.assertFalse(self.window.start_recording({'experiment': 'overlap rejected'}))
        self.assertIs(self.window.closing_recorder, recorder)
        gate.release.set()
        self.assertTrue(recorder.done.wait(2))
        self.window.poll_recording()
        old_rows = self.read_samples(recorder)
        self.assertEqual([(r['sequence'], r['source']) for r in old_rows], [('11', 'replay')])
        directory = recorder.poll()['directory']
        old_events = [json.loads(line) for line in (directory / 'events.jsonl').read_text(
            encoding='utf-8').splitlines()]
        self.assertEqual([e['detail'] for e in old_events if e['kind'] == 'marker'],
                         ['old replay marker'])
        second = self.start(DiskGate('never'))
        self.window.receive([self.row(901)])
        self.window.finish_recording()
        self.assertTrue(second.done.wait(2))
        self.window.poll_recording()
        self.assertEqual([(r['sequence'], r['source']) for r in self.read_samples(second)],
                         [('901', 'demo')])
        self.assertNotEqual(directory, second.poll()['directory'])

    def test_bounded_backlog_failure_keeps_stop_and_never_certifies_partial_data(self):
        gate = DiskGate('init')
        recorder = self.start(gate, capacity=3)
        self.assertTrue(gate.entered.wait(1))
        self.window.receive([self.row(1)])
        self.window.mark_text.setText('fills third queue slot')
        self.window.mark_event()
        self.assertEqual(recorder.poll()['pending_count'], 3)
        self.while_blocked(gate, recorder, lambda: self.window.receive([self.row(2)]))
        self.assertIsNone(self.window.recorder)
        self.assertIs(self.window.closing_recorder, recorder)
        self.assertEqual(recorder.poll()['accepted'], 3)
        self.assertEqual(recorder.poll()['rejected'], 1)
        self.assertIn('queue is full', recorder.poll()['error'])
        self.stop_callback(gate, recorder)
        self.window.receive([self.row(3)])
        self.assertEqual(self.window.latest['sequence'], 3)
        gate.release.set()
        self.assertTrue(recorder.done.wait(2))
        self.window.poll_recording()
        self.assertEqual(recorder.poll()['state'], 'failed')
        self.assertIsNone(recorder.poll()['summary'])
        self.assertFalse((recorder.poll()['directory'] / 'summary.json').exists())
        self.assertIn('不完整', self.window.record_path.text())
        self.assertNotIn('已保存', self.window.message.text())


if __name__ == '__main__':
    unittest.main()
