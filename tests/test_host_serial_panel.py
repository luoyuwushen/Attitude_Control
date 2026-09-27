"""Offline regressions for terminal layout after the raw ring fills."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from PySide6.QtWidgets import QApplication
from host.serial_ui import SerialPanel


class SerialPanelLongRunTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.application = QApplication.instance() or QApplication([])

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.panel = SerialPanel(self.directory.name, lambda _: False)

    def tearDown(self):
        self.panel.close()
        self.panel.deleteLater()
        self.directory.cleanup()

    def assert_bounded_display(self):
        self.assertLessEqual(self.panel.view.document().characterCount(),
                             self.panel.character_limit + 1)
        self.assertLessEqual(self.panel.view.document().blockCount(),
                             self.panel.display_limit // 16)

    def test_full_raw_ring_never_rebuilds_hex_during_flush(self):
        self.panel.ingest(bytes(range(256)) * 1024)
        with patch.object(self.panel, 'render', side_effect=AssertionError('full-ring rebuild')):
            self.panel.flush()
            for _ in range(100):
                self.panel.ingest(bytes.fromhex('AA 55 00 24'))
                self.panel.flush()
        self.assertEqual(len(self.panel.buffer), self.panel.limit)
        self.assert_bounded_display()
        self.assertTrue(all(len(line) <= 47 for line in self.panel.view.toPlainText().splitlines()))

    def test_750_seconds_of_50_hz_input_wraps_raw_ring_without_layout_growth(self):
        # 25 GUI flushes/s, two 24-byte telemetry frames per flush. No serial port.
        payload = bytes(range(48))
        with patch.object(self.panel, 'render', side_effect=AssertionError('history rebuild')):
            for index in range(750 * 25):
                self.panel.ingest(payload)
                self.panel.flush()
                if index % 500 == 0:
                    self.assert_bounded_display()
        self.assertEqual(self.panel.rx_total, 750 * 50 * 24)
        self.assertEqual(len(self.panel.buffer), self.panel.limit)
        self.assertEqual(bytes(self.panel.buffer), (payload * (750 * 25))[-self.panel.limit:])
        self.assertEqual(self.panel.pending, b'')
        self.assert_bounded_display()

    def test_display_mode_switch_keeps_full_saved_raw_bytes(self):
        payload = bytes(range(256)) * 1200
        self.panel.ingest(payload)
        self.panel.flush()
        self.panel.hex_view.setChecked(False)
        self.assert_bounded_display()
        self.panel.hex_view.setChecked(True)
        shown = bytes.fromhex(self.panel.view.toPlainText())
        self.assertEqual(shown, payload[-self.panel.display_limit:])
        target = Path(self.directory.name) / 'saved.bin'
        with patch('host.serial_ui.QFileDialog.getSaveFileName', return_value=(str(target), '')):
            self.panel.save()
        self.assertEqual(target.read_bytes(), payload[-self.panel.limit:])
        self.panel.ingest(bytes(range(32)))
        self.panel.flush()
        self.assertTrue(all(len(line) <= 47 for line in self.panel.view.toPlainText().splitlines()))

    def test_newline_free_text_is_bounded_and_split_utf8_is_decoded(self):
        self.panel.hex_view.setChecked(False)
        word = '测量'.encode('utf-8')
        self.panel.ingest(word[:2])
        self.panel.flush()
        self.panel.ingest(word[2:])
        self.panel.flush()
        self.assertEqual(self.panel.view.toPlainText(), '测量')
        with patch.object(self.panel, 'render', side_effect=AssertionError('full-ring rebuild')):
            for _ in range(100):
                self.panel.ingest(b'x' * 8192)
                self.panel.flush()
                self.assert_bounded_display()
        self.assertEqual(len(self.panel.buffer), self.panel.limit)
        self.assertTrue(self.panel.view.toPlainText().endswith('x' * 8192))

    def test_text_trimming_preserves_unicode_character_boundaries(self):
        self.panel.hex_view.setChecked(False)
        self.panel.character_limit = 10
        self.panel.ingest('😀abcdefgh'.encode('utf-8'))
        self.panel.flush()
        self.panel.ingest(b'i')
        self.panel.flush()
        self.assertEqual(self.panel.view.toPlainText(), 'abcdefghi')
        self.assert_bounded_display()


if __name__ == '__main__':
    unittest.main()
