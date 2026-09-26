"""Measurement UI evidence, isolation and read-only control acceptance tests."""
import csv
import json
import hashlib
import math
import os
from pathlib import Path
import tempfile
import unittest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
try:
    from PySide6.QtWidgets import QApplication, QPushButton
    from host.measurement_ui import MeasurementPanel
except ModuleNotFoundError as error:
    if error.name != 'PySide6':
        raise
    DEPENDENCY_ERROR = str(error)
else:
    DEPENDENCY_ERROR = None


def sample(index, count=0, source='serial', **changes):
    arm_q10 = count * (6588397 // 1040) // 1024
    record = dict(sequence=index % 65536, elapsed_s=index * .02,
                  source=source, state=0, fault=0, calibrated=1, adc=700,
                  theta_deg=180, arm_deg=arm_q10 * 180 / math.pi / 1024,
                  arm_q10=arm_q10, omega_rad_s=0, arm_speed_rad_s=0,
                  command_permille=0, sequence_gap=0, sequence_reset=False,
                  sequence_duplicate=False, protocol_version=1, raw_hex='aabb')
    record.update(changes)
    return record


@unittest.skipIf(DEPENDENCY_ERROR, DEPENDENCY_ERROR or '')
class MeasurementPanelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.panel = MeasurementPanel(Path(self.temp.name))
        self.panel.set_context('serial')

    def tearDown(self):
        self.panel.close()
        self.panel.deleteLater()
        self.app.processEvents()
        self.temp.cleanup()

    def feed(self, start=0, stop=60, **changes):
        for index in range(start, stop):
            self.panel.ingest(sample(index, **changes))

    def test_controls_cover_five_methods_and_no_motion_commands(self):
        self.assertEqual(self.panel.method.count(), 5)
        self.assertEqual((self.panel.window_seconds.minimum(), self.panel.window_seconds.maximum()), (1, 120))
        self.assertEqual(self.panel.window_seconds.value(), 10)
        self.assertEqual(self.panel.configured_cpr.value(), 1040)
        self.assertEqual(self.panel.physical_angle.value(), 0)
        self.assertTrue(self.panel.result_view.isReadOnly())
        self.assertFalse(hasattr(self.panel, 'worker'))
        button_names = [button.text() for button in self.panel.findChildren(QPushButton)]
        self.assertFalse(any('点动' in name or '起摆' in name for name in button_names))
        self.assertTrue(all(not entry.text() for entry in self.panel.manual_fields.values()))

    def test_stale_serial_capture_and_calculation_are_rejected(self):
        self.feed()
        self.panel.set_context('serial', fresh=False)
        self.assertFalse(self.panel.capture_down.isEnabled())
        self.assertFalse(self.panel.calculate_button.isEnabled())
        self.panel.capture_adc('down')
        self.assertFalse(self.panel.adc_segments)
        self.panel.calculate()
        self.assertIsNone(self.panel.result)
        self.assertIn('新鲜', self.panel.result_view.toPlainText())

    def test_adc_captures_are_read_only_and_unmarked_adc_is_allowed(self):
        self.feed(calibrated=0)
        before = list(self.panel.records)
        self.panel.capture_adc('down')
        self.panel.capture_adc('up')
        self.assertEqual(set(self.panel.adc_segments), {'down', 'up'})
        self.assertEqual(before, list(self.panel.records))
        self.assertTrue(all(row['calibrated'] == 0 for row in self.panel.records))
        self.panel.calculate()
        self.assertEqual(self.panel.result['adc_mean'], 700)
        self.assertNotIn('theta_mean_deg', self.panel.result)
        self.assertIn('整数', self.panel.instructions.text())

    def test_capture_requires_a_stable_duration_and_stopped_state(self):
        self.feed(stop=5)
        self.panel.capture_adc('down')
        self.assertFalse(self.panel.adc_segments)
        self.assertIn('0.8', self.panel.result_view.toPlainText())
        self.panel.reset('serial')
        self.feed(state=4, command_permille=80)
        self.panel.capture_adc('down')
        self.assertFalse(self.panel.adc_segments)

    def test_gap_inside_selected_window_rejects_measurement(self):
        self.feed(stop=55)
        self.panel.ingest(sample(55, sequence_gap=1))
        self.feed(start=56, stop=65)
        self.panel.calculate()
        self.assertIsNone(self.panel.result)
        self.assertIn('丢帧', self.panel.result_view.toPlainText())

    def test_source_isolation_and_commands_invalidate_saved_captures(self):
        self.feed()
        self.panel.capture_adc('down')
        self.panel.calculate()
        saved = self.panel.save_report()
        self.assertTrue(saved.exists())
        self.panel.notify_command('U')
        self.assertFalse(self.panel.records)
        self.assertFalse(self.panel.adc_segments)
        self.assertIsNone(self.panel.result)
        self.assertTrue(saved.exists())
        self.panel.set_context('demo')
        self.panel.ingest(sample(0, source='serial'))
        self.assertFalse(self.panel.records)
        self.feed(source='demo')
        self.panel.calculate()
        self.assertEqual(self.panel.result['evidence'], 'synthetic')
        self.assertFalse(self.panel.result['is_physical_measurement'])
        self.panel.set_context('replay')
        self.assertFalse(self.panel.records)
        self.feed(source='replay', original_source='demo')
        self.panel.calculate()
        self.assertEqual(self.panel.result['evidence'], 'synthetic')

    def test_report_preserves_raw_evidence_and_unknown_manual_fields(self):
        self.feed()
        self.panel.manual_fields['firmware_git'].setText('abc123')
        self.panel.manual_fields['operator'].setText('测试记录人')
        self.panel.manual_fields['arm_length_mm'].setText('123.5')
        self.panel.calculate()
        path = self.panel.save_report()
        report = json.loads(path.read_text(encoding='utf-8'))
        self.assertEqual(report['source'], 'serial')
        self.assertEqual(report['metadata']['firmware_git'], 'abc123')
        self.assertEqual(report['metadata']['arm_length_mm'], 123.5)
        self.assertIsNone(report['metadata']['mass_g'])
        with (path.parent / report['evidence_file']).open(encoding='utf-8-sig', newline='') as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), report['evidence_samples'])
        self.assertTrue(all(row['raw_hex'] == 'aabb' for row in rows))
        self.assertTrue(all(row['source'] == 'serial' for row in rows))
        self.assertEqual(report['evidence_sha256'], hashlib.sha256((path.parent / 'samples.csv').read_bytes()).hexdigest())

    def test_mechanical_manual_report_needs_no_telemetry_and_preserves_unknowns(self):
        self.assertIsNone(self.panel.save_manual_report())
        self.panel.manual_fields['mass_g'].setText('50')
        self.panel.manual_fields['measurement_tool'].setText('电子秤')
        path = self.panel.save_manual_report()
        report = json.loads(path.read_text(encoding='utf-8'))
        self.assertEqual(report['source'], 'manual')
        self.assertEqual(report['result']['evidence'], 'manual_observation')
        self.assertEqual(report['result']['measurements']['mass_g'], 50)
        self.assertIsNone(report['result']['measurements']['arm_length_mm'])
        self.assertEqual(report['evidence_samples'], 0)
        self.assertNotIn('evidence_file', report)
        self.assertFalse(self.panel.records)

    def test_report_preserves_both_adc_segments_outside_current_window(self):
        self.feed(stop=60, adc=700)
        self.panel.capture_adc('down')
        down = list(self.panel.adc_segments['down']['records'])
        self.feed(start=60, stop=200, adc=800)
        self.panel.capture_adc('up')
        up = list(self.panel.adc_segments['up']['records'])
        self.panel.window_seconds.setValue(1)
        self.panel.manual_fields['firmware_git'].setText('raw-endpoint-check')
        self.panel.calculate()
        self.assertGreater(self.panel.result['time_range'][0], down[-1]['elapsed_s'])
        path = self.panel.save_report()
        report = json.loads(path.read_text(encoding='utf-8'))
        self.assertEqual(report['metadata']['firmware_git'], 'raw-endpoint-check')
        self.assertEqual(report['source'], 'serial')
        for key, expected in [('down', down), ('up', up)]:
            segment = report['adc_static_segments'][key]
            self.assertEqual(segment['records'], expected)
            self.assertEqual(segment['result']['source'], report['source'])
            self.assertTrue(all(row['source'] == 'serial' and row['raw_hex'] == 'aabb'
                                for row in segment['records']))
            self.assertEqual(segment['time_range'],
                             [expected[0]['elapsed_s'], expected[-1]['elapsed_s']])

    def test_encoder_full_interval_not_just_endpoint_windows(self):
        self.panel.method.setCurrentIndex(1)
        self.feed(stop=51)
        self.panel.capture_endpoint('start')
        for index in range(51, 151):
            self.panel.ingest(sample(index, count=index - 50))
        for index in range(151, 203):
            self.panel.ingest(sample(index, count=100))
        self.panel.capture_endpoint('end')
        self.panel.physical_angle.setValue(100 * 360 / 1040)
        self.panel.calculate()
        self.assertIsNotNone(self.panel.result, self.panel.result_view.toPlainText())
        self.assertTrue(self.panel.result['full_interval_checked'])
        self.assertEqual(self.panel.result['delta_count'], 100)
        self.assertAlmostEqual(self.panel.result['cpr_estimate'], 1040, delta=.02)
        self.assertEqual(len(self.panel.evidence_records), 203)

    def test_encoder_requires_explicit_measured_mechanical_angle(self):
        self.panel.method.setCurrentIndex(1)
        self.feed(stop=51)
        self.panel.capture_endpoint('start')
        for index in range(51, 151):
            self.panel.ingest(sample(index, count=index - 50))
        for index in range(151, 203):
            self.panel.ingest(sample(index, count=100))
        self.panel.capture_endpoint('end')
        self.panel.calculate()
        self.assertIsNone(self.panel.result)
        self.assertIn('角度不能为零', self.panel.result_view.toPlainText())

    def test_encoder_rejects_gap_hidden_between_stable_endpoints(self):
        self.panel.method.setCurrentIndex(1)
        self.feed(stop=51)
        self.panel.capture_endpoint('start')
        for index in range(51, 151):
            self.panel.ingest(sample(index, count=index - 50, sequence_gap=int(index == 75)))
        for index in range(151, 203):
            self.panel.ingest(sample(index, count=100))
        self.panel.capture_endpoint('end')
        self.panel.calculate()
        self.assertIsNone(self.panel.result)
        self.assertIn('丢帧', self.panel.result_view.toPlainText())

    def test_parameter_edit_invalidates_prior_report_and_bad_manual_value_not_saved(self):
        self.feed()
        self.panel.calculate()
        self.panel.window_seconds.setValue(2)
        self.assertIsNone(self.panel.result)
        self.assertFalse(self.panel.save_button.isEnabled())
        self.panel.calculate()
        self.panel.manual_fields['mass_g'].setText('NaN')
        self.assertIsNone(self.panel.save_report())
        self.assertFalse((Path(self.temp.name) / 'measurements').exists())


if __name__ == '__main__':
    unittest.main()
