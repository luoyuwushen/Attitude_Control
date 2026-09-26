"""Offline acceptance of persisted serial settings and manually entered developer UI."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import tempfile
from pathlib import Path
import threading
import time
import unittest
from unittest.mock import patch

from PySide6.QtWidgets import QApplication, QDialog
from host.app import Window
from host.core import demo_record
from host.serial_config import SerialConfig
from host.settings import BAUD_PRESETS, SettingsDialog, load_preferences, save_preferences
from host.serial_ui import SerialPanel


class SettingsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.application = QApplication.instance() or QApplication([])

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.window = Window(self.root)

    def tearDown(self):
        self.window.worker = None
        self.window.close()
        self.tmp.cleanup()

    def test_persist_custom_rates_format_and_mode_without_auto_developer(self):
        config = SerialConfig(baudrate=12345, bytesize=7, parity='E', stopbits=1.5)
        save_preferences(self.window.preferences_path, config, 'raw', BAUD_PRESETS + [12345, 250000])
        other = Window(self.root)
        try:
            self.assertEqual(other.serial_config, config)
            self.assertEqual(other.serial_mode, 'raw')
            self.assertIn(250000, other.baud_presets)
            self.assertIsNone(other.developer_window)
            self.assertIsNone(other.measurement_panel)
            self.assertFalse(other.developer_controls.isVisible())
            self.assertFalse(other.experiment_analysis.isVisible())
            self.assertEqual([other.tabs.tabText(i) for i in range(other.tabs.count())],
                             ['实时曲线', '实验与记录', '串口收发'])
        finally:
            other.close()

    def test_corrupt_preferences_restore_safe_project_default(self):
        self.window.preferences_path.write_text('{invalid', encoding='utf-8')
        config, mode, _ = load_preferences(self.window.preferences_path)
        self.assertTrue(config.is_project_default)
        self.assertEqual(mode, 'project')

    def test_custom_baud_and_format_dialog(self):
        dialog = SettingsDialog(SerialConfig(), 'project', BAUD_PRESETS)
        dialog.baud.setCurrentText('250000')
        self.assertTrue(dialog.add_baud())
        dialog.bits.setCurrentText('7')
        dialog.parity.setCurrentIndex(dialog.parity.findData('O'))
        dialog.stops.setCurrentText('2')
        dialog.mode_combo.setCurrentIndex(1)
        dialog.apply()
        self.assertEqual(dialog.result(), QDialog.Accepted)
        self.assertEqual(dialog.selected_config, SerialConfig(250000, 7, 'O', 2))
        self.assertEqual(dialog.selected_mode, 'raw')
        dialog.close()

    def test_developer_entry_does_not_apply_unsaved_format(self):
        def enter(dialog):
            dialog.baud.setCurrentText('9600')
            dialog.enter_developer()
            return QDialog.Rejected
        self.window.start_demo()
        self.window.receive([demo_record(i / 50, i) for i in range(60)])
        with patch.object(SettingsDialog, 'exec', enter):
            self.window.open_settings()
        self.assertTrue(self.window.developer_window.isVisible())
        self.assertTrue(self.window.serial_config.is_project_default)
        self.assertEqual(len(self.window.measurement_panel.records), 60)
        self.assertEqual(self.window.measurement_panel.source, 'demo')
        self.assertIs(self.window.command_buttons['F'].parent(), self.window.developer_controls)
        self.window.developer_window.close()
        self.assertFalse(self.window.developer_window.isVisible())
        self.window.open_developer()
        self.assertTrue(self.window.developer_window.isVisible())

    def test_connected_settings_locked_but_developer_entry_available(self):
        dialog = SettingsDialog(SerialConfig(), 'project', BAUD_PRESETS, connected=True)
        self.assertFalse(dialog.baud.isEnabled())
        self.assertTrue(dialog.developer_button.isEnabled())
        dialog.baud.setCurrentText('9600')
        dialog.apply()
        self.assertTrue(dialog.selected_config.is_project_default)
        dialog.close()

    def test_project_commands_blocked_in_raw_and_mismatched_config(self):
        class Worker:
            closing = threading.Event()
            def send_raw(self, data):
                return data == b'hello'
        self.window.worker = Worker()
        self.window.source = 'serial'
        self.window.serial_ready = True
        self.window.latest = dict(demo_record(0, 1), state=0, calibrated=1, fault=0)
        self.window.last_received = time.monotonic()
        self.window.direction_check.setChecked(True)
        for mode, config in [('raw', SerialConfig()), ('project', SerialConfig(9600))]:
            self.window.serial_mode, self.window.serial_config = mode, config
            for command in 'DUGFBSR':
                self.assertFalse(self.window.allowed(command))
            self.window.update_display()
            self.assertFalse(self.window.stop_on_close.isEnabled())
        self.window.serial_mode = 'raw'
        self.assertTrue(self.window.send_raw(b'hello'))
        self.window.serial_mode = 'project'
        self.assertFalse(self.window.send_raw(b'hello'))

    def test_developer_reset_invalidates_previous_source_and_calibration(self):
        self.window.start_demo()
        self.window.open_developer()
        self.window.receive([demo_record(i / 50, i) for i in range(60)])
        self.assertEqual(len(self.window.measurement_panel.records), 60)
        self.window.command_sent('U')
        self.assertEqual(len(self.window.measurement_panel.records), 0)
        self.window.reset_data('replay')
        self.assertEqual(self.window.measurement_panel.source, 'replay')
        self.assertFalse(self.window.measurement_panel.endpoints)

    def test_raw_hex_text_endings_and_no_automatic_transmission(self):
        sent = []
        panel = SerialPanel(self.root, lambda data: sent.append(data) or True)
        self.assertEqual(sent, [])
        panel.hex_send.setChecked(True)
        panel.input.setPlainText('AA 55 00 FF')
        self.assertTrue(panel.send())
        self.assertEqual(sent[-1], bytes.fromhex('AA5500FF'))
        panel.input.setPlainText('0X ZZ')
        self.assertFalse(panel.send())
        panel.hex_send.setChecked(False)
        panel.input.setPlainText('测量')
        panel.ending.setCurrentIndex(3)
        self.assertTrue(panel.send())
        self.assertEqual(sent[-1], '测量\r\n'.encode('utf-8'))
        panel.close()

    def test_old_serial_batch_cannot_temporarily_enable_developer_capture(self):
        self.window.reset_data('serial')
        self.window.open_developer()
        row = dict(demo_record(0, 1), source='serial', host_monotonic=time.monotonic() - 2)
        self.window.receive([row])
        self.assertFalse(self.window.is_fresh())
        self.assertFalse(self.window.measurement_panel.fresh)
        self.assertFalse(self.window.measurement_panel.capture_down.isEnabled())

    def test_raw_multibyte_stream_and_memory_bound(self):
        panel = SerialPanel(self.root, lambda _: False)
        panel.hex_view.setChecked(False)
        data = '测量'.encode('utf-8')
        panel.ingest(data[:2])
        panel.flush()
        panel.ingest(data[2:])
        panel.flush()
        self.assertEqual(panel.view.toPlainText(), '测量')
        panel.ingest(b'x' * (panel.limit + 100))
        panel.flush()
        self.assertEqual(len(panel.buffer), panel.limit)
        self.assertLessEqual(panel.view.document().characterCount(), panel.limit * 3 + 1)
        panel.clear()
        self.assertEqual(panel.rx_total, 0)
        self.assertEqual(panel.buffer, b'')
        panel.close()

    def test_closed_window_stops_collection_timer(self):
        self.window.close()
        self.assertFalse(self.window.timer.isActive())
