"""Optional logs preserve actual I/O and never block the command dispatcher."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import json
from pathlib import Path
import queue
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from PySide6.QtWidgets import QApplication, QDialog
from host.app import Window
from host.core import demo_record
from host.operation_log import OperationLog
from host.serial_config import SerialConfig
from host.settings import SettingsDialog, load_log_preferences, save_preferences
from host.transport import SerialWorker
from test_host_serial import MemoryPort


def rows(directory, channel):
    return [json.loads(line) for line in (directory / (channel + '.jsonl')).read_text(encoding='utf-8').splitlines()]


class OperationLogTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def test_unused_logger_creates_no_files(self):
        logger = OperationLog(self.root, {})
        logger.close()
        self.assertEqual(list(self.root.iterdir()), [])

    def test_channels_keep_bytes_times_and_unacknowledged_commands(self):
        logger = OperationLog(self.root, {'serial': SerialConfig().as_dict()})
        logger.write_serial('RX', b'\x00\xffG', host_monotonic=42.0)
        logger.write_serial('TX', b'S', requested_bytes=1, complete=True)
        logger.write_control('command_queued', command='G', acknowledged=False)
        logger.close()
        serial = rows(logger.directory, 'serial')
        self.assertEqual(serial[0]['hex'], '00ff47')
        self.assertEqual(serial[0]['bytes'], 3)
        self.assertEqual(serial[0]['host_monotonic'], 42.0)
        self.assertIn('time_utc', serial[1])
        self.assertFalse(rows(logger.directory, 'control')[0]['acknowledged'])
        self.assertEqual(json.loads((logger.directory / 'metadata.json').read_text())['serial']['baudrate'], 115200)

    def test_independent_channels_and_unique_sessions(self):
        a, b = OperationLog(self.root, {}), OperationLog(self.root, {})
        a.write_serial('RX', b'a')
        b.write_control('connection')
        a.close()
        b.close()
        self.assertNotEqual(a.directory, b.directory)
        self.assertFalse((a.directory / 'control.jsonl').exists())
        self.assertFalse((b.directory / 'serial.jsonl').exists())

    def test_disk_failure_is_explicit(self):
        blocked = self.root / 'file'
        blocked.write_text('blocked')
        logger = OperationLog(blocked, {})
        logger.write_serial('RX', b'frame')
        with self.assertRaises(OSError):
            logger.close()
        with self.assertRaises(OSError):
            logger.write_control('must not silently disappear')

    def test_queue_overflow_does_not_pretend_log_is_complete(self):
        logger = OperationLog(self.root, {})
        logger._queue = queue.Queue(maxsize=1)
        # Prevent consumption so overflow is deterministic.
        with patch.object(threading.Thread, 'start'):
            logger.write_serial('RX', b'first')
            with self.assertRaisesRegex(OSError, '不.*完整'):
                logger.write_serial('RX', b'second')
        logger._thread = None
        with self.assertRaises(OSError):
            logger.close()

    def test_writer_wait_does_not_block_subsequent_entries(self):
        logger = OperationLog(self.root, {})
        entered, release = threading.Event(), threading.Event()
        original = Path.mkdir
        def blocked_mkdir(path, *args, **kwargs):
            entered.set()
            self.assertTrue(release.wait(2))
            return original(path, *args, **kwargs)
        try:
            with patch.object(Path, 'mkdir', blocked_mkdir):
                logger.write_control('first')
                self.assertTrue(entered.wait(1))
                logger.write_control('stop_requested', command='S')
                self.assertEqual(logger._queue.qsize(), 2)
                release.set()
                logger.close()
        finally:
            release.set()
        self.assertEqual(rows(logger.directory, 'control')[-1]['command'], 'S')


class LoggedWindowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.window = Window(self.root)
        self.window.timer.stop()

    def tearDown(self):
        if self.window.worker and self.window.worker.isRunning():
            self.window.worker.request_close(False)
            self.window.worker.wait(1200)
        self.window.worker = None
        self.window.close()
        self.app.processEvents()
        self.tmp.cleanup()

    def attach(self, mode='project'):
        worker = SerialWorker('TEST-NO-HARDWARE', mode=mode)
        self.window.worker = worker
        self.window.serial_mode = mode
        self.window.source = 'serial'
        self.window.serial_ready = True
        worker.wire_activity.connect(self.window.wire_activity)
        worker.command_sent.connect(self.window.command_sent)
        worker.failed.connect(self.window.serial_failed)
        return worker

    def test_default_off_and_demo_never_create_operation_logs(self):
        self.assertEqual(self.window.log_preferences, {'serial': False, 'control': False})
        self.window.control_log('command_queued', command='G')
        self.window.start_demo()
        self.window.log_preferences = dict(serial=True, control=True)
        self.window.receive([demo_record(0, 1)])
        self.window.send_command('G')
        self.assertIsNone(self.window.operation_log)
        self.assertFalse((self.root / 'logs').exists())

    def test_connected_logging_options_persist_without_changing_port(self):
        self.attach()
        def save(dialog):
            self.assertFalse(dialog.baud.isEnabled())
            self.assertTrue(dialog.serial_log_check.isEnabled())
            dialog.baud.setCurrentText('9600')
            dialog.serial_log_check.setChecked(True)
            dialog.control_log_check.setChecked(True)
            dialog.apply()
            return QDialog.Accepted
        with patch.object(SettingsDialog, 'exec', save):
            self.window.open_settings()
        self.assertTrue(self.window.serial_config.is_project_default)
        self.assertEqual(load_log_preferences(self.window.preferences_path), dict(serial=True, control=True))

    def test_preferences_upgrade_and_false_strings_do_not_enable_logging(self):
        path = self.window.preferences_path
        path.write_text('{"logging":{"serial":"false","control":1}}')
        self.assertEqual(load_log_preferences(path), dict(serial=False, control=False))
        save_preferences(path, SerialConfig(), 'raw', [115200], dict(serial=True, control=False))
        save_preferences(path, SerialConfig(9600), 'raw', [9600])
        self.assertEqual(load_log_preferences(path), dict(serial=True, control=False))

    def test_main_controls_and_developer_share_one_window_and_data(self):
        self.window.show()
        self.app.processEvents()
        self.assertEqual(set(self.window.command_buttons), set('DUGHFBRT'))
        for b in self.window.command_buttons.values():
            self.assertTrue(b.isVisible())
        self.window.start_demo()
        self.window.receive([demo_record(0, 1)])
        self.window.open_developer()
        self.assertIs(self.window.developer_page.window(), self.window)
        panel = self.window.measurement_panel
        self.window.receive([demo_record(0.02, 2)])
        self.window.show_main()
        self.window.receive([demo_record(0.04, 3)])
        self.window.open_developer()
        self.assertIs(self.window.measurement_panel, panel)
        self.assertEqual(len(panel.records), 3)
        self.assertTrue(self.window.global_stop.isVisible())

    def test_command_queue_is_distinct_from_actual_uart_write(self):
        worker = self.attach()
        self.window.log_preferences['control'] = True
        self.window.latest = dict(demo_record(0, 1), state=0, calibrated=1, fault=0)
        self.window.last_received = time.monotonic()
        self.window.direction_check.setChecked(True)
        self.assertTrue(self.window.send_command('G'))
        logger = self.window.operation_log
        worker.command_sent.emit('G')
        self.window.close_operation_log()
        events = rows(logger.directory, 'control')
        commands = [row for row in events if row['event'].startswith('command_')]
        self.assertEqual([row['event'] for row in commands], ['command_queued', 'command_written'])
        self.assertTrue(all(row['acknowledged'] is False for row in commands))
        self.assertIn('operator_checks', [row['event'] for row in events])
        self.assertIn('control_availability', [row['event'] for row in events])
        self.assertFalse((logger.directory / 'serial.jsonl').exists())

    def test_raw_short_write_and_error_preserve_only_known_written_bytes(self):
        worker = self.attach('raw')
        self.window.log_preferences['serial'] = True
        self.assertTrue(self.window.send_raw(b'\x00\xffabcd'))
        port = MemoryPort(worker, short_write=2)
        with patch('host.transport.serial.Serial', return_value=port):
            worker.run()
        logger = self.window.operation_log
        self.window.close_operation_log()
        events = rows(logger.directory, 'serial')
        tx = [row for row in events if row.get('direction') == 'TX']
        self.assertEqual(len(tx), 1)
        self.assertEqual((tx[0]['hex'], tx[0]['bytes'], tx[0]['requested_bytes']), ('00ff', 2, 6))
        self.assertFalse(tx[0]['complete'])
        self.assertEqual(events[0]['event'], 'send_queued')
        self.assertEqual(events[-1]['event'], 'serial_error')
        self.assertFalse((logger.directory / 'control.jsonl').exists())

    def test_stale_worker_signals_cannot_contaminate_current_logs(self):
        old = self.attach()
        self.attach()
        self.window.log_preferences = dict(serial=True, control=True)
        old.wire_activity.emit(dict(direction='RX', data=b'old'))
        old.command_sent.emit('G')
        old.failed.emit('old')
        self.assertIsNone(self.window.operation_log)

    def test_logging_failure_keeps_stop_available_and_shows_incomplete_status(self):
        worker = self.attach()
        self.window.log_preferences['control'] = True
        self.window.output_root = self.root / 'blocked'
        self.window.output_root.write_text('file')
        self.window.control_log('connection')
        logger = self.window.operation_log
        with self.assertRaises(OSError):
            logger.close()
        self.window.update_display()
        self.assertIsNotNone(self.window.logging_error)
        self.assertIn('不完整', self.window.log_status.text())
        self.assertTrue(self.window.global_stop.isEnabled())
        self.assertTrue(self.window.send_command('S'))
        self.assertEqual(worker.commands.get_nowait()[0], 'S')

    def test_exit_drains_background_final_stop_before_closing_log(self):
        worker = self.attach()
        self.window.log_preferences = dict(serial=True, control=True)
        class LivePort(MemoryPort):
            def read(port, count):
                time.sleep(0.005)
                return b''
        port = LivePort(worker)
        worker.delivery_finished.connect(self.window.worker_finished)
        with patch('host.transport.serial.Serial', return_value=port):
            worker.start()
            started = time.monotonic()
            self.window.close()
            self.assertLess(time.monotonic() - started, 0.1)
            deadline = time.monotonic() + 2
            while self.window.worker is not None and time.monotonic() < deadline:
                self.app.processEvents()
                time.sleep(0.005)
        self.assertFalse(worker.isRunning())
        self.assertEqual(port.writes, [b'S'])
        directories = list((self.root / 'logs').iterdir())
        self.assertEqual(len(directories), 1)
        self.assertEqual(rows(directories[0], 'serial')[-1]['hex'], '53')
        events = rows(directories[0], 'control')
        self.assertTrue(any(item['event'] == 'command_written' and item['command'] == 'S' for item in events))
        self.assertEqual(events[-1]['event'], 'connection_finished')
