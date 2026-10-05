"""A failed persistent sample/event must never become a successful session."""
import json
import os
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PySide6.QtWidgets import QApplication
from host.app import Window
from host.core import Replay, SessionRecorder, demo_record
from host.recording import AsyncSessionRecorder


class SessionWriteFailureTests(unittest.TestCase):
    def test_partial_initialization_closes_already_open_files(self):
        opened = []
        original = Path.open
        def open_file(path, *args, **kwargs):
            if path.name == 'raw_frames.bin':
                raise OSError('raw initialization unavailable')
            stream = original(path, *args, **kwargs)
            if path.name == 'samples.csv':
                opened.append(stream)
            return stream
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(Path, 'open', open_file):
                with self.assertRaisesRegex(OSError, 'raw initialization unavailable'):
                    SessionRecorder(directory, {}, background_flush=False)
            self.assertEqual(len(opened), 1)
            self.assertTrue(opened[0].closed)
            self.assertFalse(list(Path(directory).glob('session_*/summary.json')))

    def assert_sticky_failure(self, recorder, cause):
        for action in (lambda: recorder.write(demo_record(.04, 2)),
                       lambda: recorder.event('marker', 'later marker'),
                       recorder.close, recorder.close,
                       lambda: recorder.write(demo_record(.06, 3))):
            with self.assertRaisesRegex(OSError, cause):
                action()
        self.assertFalse((recorder.directory/'summary.json').exists())
        self.assertFalse(recorder._flush_thread.is_alive())

    def test_csv_write_failure_stays_failed_after_disk_recovers(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = SessionRecorder(directory, {})
            recorder.write(demo_record(0, 0))
            with patch.object(recorder._writer, 'writerow', side_effect=OSError('CSV first failure')):
                with self.assertRaisesRegex(OSError, 'CSV first failure'):
                    recorder.write(demo_record(.02, 1))
            self.assert_sticky_failure(recorder, 'CSV first failure')
            self.assertEqual(len(Replay.load(recorder.directory/'samples.csv')), 1)
            self.assertEqual(len((recorder.directory/'raw_frames.bin').read_bytes()), 24)

    def test_raw_failure_after_csv_append_cannot_generate_success_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = SessionRecorder(directory, {})
            recorder.write(demo_record(0, 0))
            with patch.object(recorder._raw, 'write', side_effect=OSError('raw first failure')):
                with self.assertRaisesRegex(OSError, 'raw first failure'):
                    recorder.write(demo_record(.02, 1))
            self.assert_sticky_failure(recorder, 'raw first failure')
            self.assertEqual(len(Replay.load(recorder.directory/'samples.csv')), 2)
            self.assertEqual(len((recorder.directory/'raw_frames.bin').read_bytes()), 24)

    def test_event_failure_stays_failed_after_disk_recovers(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = SessionRecorder(directory, {})
            recorder.write(demo_record(0, 0))
            with patch.object(recorder._events, 'write', side_effect=OSError('event first failure')):
                with self.assertRaisesRegex(OSError, 'event first failure'):
                    recorder.event('marker', 'release')
            self.assert_sticky_failure(recorder, 'event first failure')
            self.assertEqual(len(Replay.load(recorder.directory/'samples.csv')), 1)

    def test_background_flush_failure_remains_after_patch_removed(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(SessionRecorder, 'FLUSH_INTERVAL_S', .02):
            recorder = SessionRecorder(directory, {})
            recorder.write(demo_record(0, 0))
            entered = threading.Event()
            def failed_flush():
                entered.set()
                raise OSError('flush first failure')
            with patch.object(recorder, '_flush_files', side_effect=failed_flush):
                self.assertTrue(entered.wait(1))
                recorder._flush_thread.join(1)
            self.assert_sticky_failure(recorder, 'flush first failure')

    def test_cleanup_failure_cannot_replace_original_csv_error(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = SessionRecorder(directory, {})
            with patch.object(recorder._writer, 'writerow', side_effect=OSError('original CSV error')):
                with self.assertRaises(OSError):
                    recorder.write(demo_record(0, 0))
            original_close = recorder._csv.close
            def failed_close():
                original_close()
                raise OSError('later cleanup error')
            with patch.object(recorder._csv, 'close', side_effect=failed_close):
                with self.assertRaisesRegex(OSError, 'original CSV error'):
                    recorder.close()
            self.assert_sticky_failure(recorder, 'original CSV error')

    def test_prewrite_programming_error_does_not_fake_disk_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = SessionRecorder(directory, {})
            with self.assertRaises(TypeError):
                recorder.write(None)
            recorder.write(demo_record(0, 0))
            summary = json.loads(recorder.close().read_text(encoding='utf-8'))
            self.assertEqual(summary['samples'], 1)


class SessionFailureUITests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_gui_receive_failure_is_not_overwritten_by_saved_message(self):
        with tempfile.TemporaryDirectory() as directory:
            window = Window(Path(directory))
            window.timer.stop()
            window.plot_timer.stop()
            window.reset_data('demo')
            try:
                def factory(*args, **kwargs):
                    writer = SessionRecorder(*args, **kwargs)
                    def unavailable(_record):
                        raise OSError('CSV write unavailable')
                    writer._writer.writerow = unavailable
                    return writer
                with patch('host.app.AsyncSessionRecorder', side_effect=lambda *a, **kw:
                           AsyncSessionRecorder(*a, **kw, writer_factory=factory)):
                    self.assertTrue(window.start_recording({'experiment': 'offline failure test'}))
                recorder = window.recorder
                window.receive([demo_record(0, 0)])
                self.assertTrue(recorder.done.wait(2))
                window.poll_recording()
                self.assertIsNone(window.recorder)
                self.assertIn('结束记录失败', window.message.text())
                self.assertIn('CSV write unavailable', window.message.text())
                self.assertNotIn('已保存', window.record_path.text())
                self.assertFalse((recorder.poll()['directory']/'summary.json').exists())
                self.assertTrue(recorder.join(1))
                self.assertIsNone(window.worker)
            finally:
                window.close()
                window.deleteLater()
                self.app.processEvents()

    def test_gui_marker_failure_cannot_be_erased_by_later_finish(self):
        with tempfile.TemporaryDirectory() as directory:
            window = Window(Path(directory))
            window.timer.stop()
            window.plot_timer.stop()
            window.reset_data('demo')
            try:
                def factory(*args, **kwargs):
                    writer = SessionRecorder(*args, **kwargs)
                    original = writer.event
                    def event(kind, *args, **kwargs):
                        if kind == 'marker':
                            raise OSError('marker write unavailable')
                        return original(kind, *args, **kwargs)
                    writer.event = event
                    return writer
                with patch('host.app.AsyncSessionRecorder', side_effect=lambda *a, **kw:
                           AsyncSessionRecorder(*a, **kw, writer_factory=factory)):
                    self.assertTrue(window.start_recording({'experiment': 'offline marker failure'}))
                recorder = window.recorder
                window.log_event('marker', 'release')
                self.assertTrue(recorder.done.wait(2))
                window.poll_recording()
                window.finish_recording()
                self.assertIn('结束记录失败', window.message.text())
                self.assertIn('marker write unavailable', window.message.text())
                self.assertFalse((recorder.poll()['directory']/'summary.json').exists())
                self.assertIsNone(window.worker)
            finally:
                window.close()
                window.deleteLater()
                self.app.processEvents()


if __name__ == '__main__':
    unittest.main()
