"""Plotting must not pace telemetry, evidence recording or stop requests."""
import json
import math
import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PySide6.QtWidgets import QApplication
from host.app import Window
from host.core import demo_record
from host.operation_log import OperationLog
from host.transport import SerialWorker


class RenderBudgetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        with patch('host.app.list_ports.comports', return_value=[]):
            self.window = Window(self.root)
        self.window.timer.stop()
        self.window.plot_timer.stop()
        self.window.resize(1000, 750)
        self.window.show()
        self.app.processEvents()

    def tearDown(self):
        self.window.worker = None
        self.window.close()
        self.window.deleteLater()
        self.app.processEvents()
        self.tmp.cleanup()

    def rows(self, count, start=0):
        return [demo_record(i / 50, i) for i in range(start, start + count)]

    def test_queued_delivery_and_log_keep_every_sample_without_waiting_for_plot(self):
        w = self.window
        w.reset_data('serial')
        w.worker = SerialWorker('OFFLINE-NO-PORT', buffered_delivery=True)
        w.worker.records.connect(w.receive_serial)
        w.serial_ready = True
        w.runtime_log = OperationLog(self.root, {'source': 'offline_simulation'},
                                     directory_name='logs')
        records = self.rows(200)
        with patch.object(w, 'update_plots', wraps=w.update_plots) as draw:
            for offset in range(0, len(records), 20):
                for original in records[offset:offset + 20]:
                    row = dict(original, source='offline_simulation',
                               host_monotonic=time.monotonic())
                    w.worker._publish([('records', [row])], 24)
                while w.worker.delivery_pending:
                    self.app.processEvents()
                # State/control refreshes must not draw, even with new data.
                w.update_display()
            draw.assert_not_called()
            self.assertEqual(w.metrics.samples, 200)
            self.assertEqual([r['sequence'] for r in w.history], list(range(200)))
            self.assertEqual(w.worker.delivery_overflows, 0)
            self.assertIn('199', w.cards['state'][1].text())
            self.assertTrue(w.global_stop.isEnabled())
            w.global_stop.click()
            self.assertEqual(w.worker.commands.get_nowait()[0], 'S')
            # The independent plot tick consumes the latest complete history.
            w.plot_timer.timeout.emit()
            draw.assert_called_once()
            x, _ = w.plot_curves['theta_deg'].getData()
            self.assertAlmostEqual(float(x[-1]), 199 / 50)
        directory = w.runtime_log.directory
        w.close_runtime_log()
        logged = [json.loads(line) for line in
                  (directory / 'telemetry.jsonl').read_text(encoding='utf-8').splitlines()]
        self.assertEqual([row['sequence'] for row in logged], list(range(200)))
        self.assertFalse((self.root / 'fpga_logs').exists())

    def test_frozen_hidden_and_unchanged_plots_resume_with_current_history(self):
        w = self.window
        w.reset_data('replay')
        w.receive(self.rows(60))
        with patch.object(w, 'update_plots', wraps=w.update_plots) as draw:
            w.refresh_plots()
            self.assertEqual(draw.call_count, 1)
            for _ in range(10):
                w.refresh_plots()
                w.update_display()
            self.assertEqual(draw.call_count, 1)
            w.pause_plots.setChecked(True)
            w.receive(self.rows(20, 60))
            w.refresh_plots()
            self.assertEqual(draw.call_count, 1)
            w.pause_plots.setChecked(False)
            w.refresh_plots()
            self.assertEqual(draw.call_count, 2)
            self.assertAlmostEqual(w.plot_curves['adc'].getData()[0][-1], 79 / 50)
            w.tabs.setCurrentIndex(1)
            w.receive(self.rows(20, 80))
            w.refresh_plots()
            self.assertEqual(draw.call_count, 2)
            w.tabs.setCurrentIndex(0)
            w.refresh_plots()
            self.assertEqual(draw.call_count, 3)
            self.assertAlmostEqual(w.plot_curves['adc'].getData()[0][-1], 99 / 50)
            w.window_seconds.setValue(5)
            w.refresh_plots()
            self.assertEqual(draw.call_count, 4)
            w.reset_data('replay')
            w.refresh_plots()
            self.assertIsNone(w.plot_curves['adc'].getData()[0])

    def test_full_history_retains_peaks_and_breaks_missing_sample_connections(self):
        w = self.window
        w.reset_data('replay')
        w.window_seconds.setValue(300)
        records = self.rows(15000)
        records[7500]['theta_deg'] = 1000.0
        records[7501]['theta_deg'] = -1000.0
        # One lost sample must remain a visible discontinuity in every curve.
        del records[5000]
        w.receive(records)
        w.refresh_plots()
        x, y = w.plot_curves['theta_deg'].getData()
        self.assertEqual(len(w.history), 14999)
        self.assertEqual(max(v for v in y if math.isfinite(v)), 1000.0)
        self.assertEqual(min(v for v in y if math.isfinite(v)), -1000.0)
        self.assertEqual(sum(math.isnan(v) for v in y), 1)
        self.assertAlmostEqual(float(x[-1]), 14999 / 50)
        gap = next(i for i, v in enumerate(y) if math.isnan(v))
        self.assertAlmostEqual(float(x[gap]), 5001 / 50)
        for curve in w.plot_curves.values():
            self.assertEqual(curve.opts['pen'].widthF(), 1.0)
            self.assertFalse(curve.opts['antialias'])
            self.assertEqual(curve.opts['connect'], 'finite')
        self.assertGreaterEqual(w.plot_timer.interval(), 100)
        self.assertLessEqual(w.plot_timer.interval(), 200)


if __name__ == '__main__':
    unittest.main()
