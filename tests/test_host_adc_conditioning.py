"""Conditioned measurements retain raw evidence and separate the blind zone."""
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PySide6.QtWidgets import QApplication

from host.adc_status import measurement_ready
from host.app import Window
from host.core import StreamDecoder, SessionRecorder, Replay
from host.diagnostics import (adc_detail_text, adc_diagnostic_text, diagnostic_text,
                              measurement_block_reason, recovery_hint)
from host.handover_status import HandoverStatus
from host.trace_recording import TraceArchiveWriter
from host.tuning import H_LQI_CONDITIONED_PROFILE, tuning_values
from test_control_trace import row
from test_host_adc_diagnostics import frame, tlv
from test_host_control_trace import collected, live
from test_host_handover_control import group
from test_host_handover_status import sample
from test_host_ui import WorkerStub


VERSION = 0x00030000


def conditioned_frame(*, flags=12, extra=b'', **changes):
    return frame(firmware=VERSION, flags=flags,
                 extra=tlv(34, '<H', 12000) + group(0, 0, 512, 3) + extra, **changes)


class ConditionedProtocolTests(unittest.TestCase):
    def test_control_mean_is_distinct_from_raw_mean_at_every_fragment_and_in_csv(self):
        raw = conditioned_frame()
        for split in range(1, len(raw)):
            decoder = StreamDecoder()
            self.assertEqual(decoder.feed(raw[:split]), [])
            parsed = decoder.feed(raw[split:])[0]
            self.assertEqual(parsed['adc_mean_q4'], 12265)
            self.assertEqual(parsed['adc_control_q4'], 12000)
            self.assertEqual(parsed['adc_raw'], 766)
            self.assertEqual(parsed['raw_hex'], raw.hex())
        with tempfile.TemporaryDirectory() as directory:
            recorder = SessionRecorder(directory, {'source': 'offline_simulation'})
            recorder.write(parsed)
            recorder.close()
            restored = Replay.load(recorder.directory / 'samples.csv')[0]
            self.assertEqual(restored['adc_control_q4'], 12000)
            self.assertEqual(restored['adc_mean_q4'], 12265)
        self.assertNotIn('adc_control_q4', StreamDecoder().feed(frame())[0])

    def test_control_mean_malformed_duplicate_and_truncation_reject_entire_frame(self):
        valid = conditioned_frame(sequence=2)
        for bad in (tlv(34, '<H', 1) * 2, b'\x22\x00', b'\x22\x02\x00', b'\x22\x01\x00'):
            decoder = StreamDecoder()
            parsed = decoder.feed(frame(extensions=False, extra=bad) + valid)
            self.assertEqual([r['sequence'] for r in parsed], [2])
            self.assertEqual(decoder.stats['unsupported_frames'], 1)

    def test_raw_otr_and_rejected_spike_warning_do_not_override_accepted_state(self):
        for flags in (0x0c, 0x4c, 0x0e, 0x1c, 0x5e):
            record = dict(firmware_version=VERSION, sensor_flags=flags)
            self.assertTrue(measurement_ready(record), flags)
            self.assertEqual(measurement_block_reason(record), '')
        for flags in (0, 4, 0x40, 0x4d, 0x20, 0x2c, 0x80 | 12):
            record = dict(firmware_version=VERSION, sensor_flags=flags)
            self.assertFalse(measurement_ready(record), flags)
            self.assertTrue(measurement_block_reason(record), flags)
        for version in (0x20009, 0x30001):
            self.assertTrue(measurement_block_reason(dict(firmware_version=version, sensor_flags=0x1e)))

    def test_blind_zone_is_not_presented_as_latched_wiring_fault(self):
        record = dict(firmware_version=VERSION, sensor_flags=0x33, fault=0,
                      adc_over_range=1, sensor_fault=0, input_fault=0,
                      diagnostic_supported=1, start_result=4)
        self.assertIn('无需按 R', recovery_hint(record))
        self.assertIn('不会自动重启', recovery_hint(record))
        self.assertIn('盲区', diagnostic_text(record))
        self.assertNotIn('ADC 超量程', diagnostic_text(record))
        self.assertNotIn('接线', recovery_hint(record))
        self.assertIn('锁存传感器原因', recovery_hint(dict(record, fault=1, sensor_fault=1)))
        self.assertIn('ADC 超量程', recovery_hint(dict(record, firmware_version=0x20009)))

    def test_new_display_exposes_filter_raw_and_control_without_rewriting_evidence(self):
        record = StreamDecoder().feed(conditioned_frame(flags=0x4c))[0]
        before = dict(record)
        text = adc_diagnostic_text(record)
        for label in ('project-v0.3.0', '均值 766.562', '去尖峰控制均值 750.000',
                      '尖峰拒绝 是', '盲区 否', '64 次异常确认'):
            self.assertIn(label, text)
        record.update(adc_detail_flags=1, adc_conversion_count=5000, adc_filtered_sum=3832812)
        self.assertIn('不含后级去尖峰', adc_detail_text(record))
        self.assertEqual({key: record[key] for key in before}, before)

    def test_handover_and_tuning_use_conditioned_readiness(self):
        record = StreamDecoder().feed(conditioned_frame(state=2, flags=0x5e))[0]
        self.assertEqual(record['parameter_profile'], H_LQI_CONDITIONED_PROFILE)
        self.assertTrue(record['actual_valid'])
        self.assertFalse(tuning_values(dict(record, sensor_flags=0x2c))['actual_valid'])
        status = HandoverStatus()
        status.observe(sample(firmware_version=VERSION, state=2, h_control_flags=3, sensor_flags=0x5e), 100)
        self.assertEqual(status.presentation(100.1, connected=True)[0], 'running')
        status.observe(sample(at=100.2, sequence=11, device_ms=1020,
                              firmware_version=VERSION, state=0, sensor_flags=0x33), 100.2)
        self.assertEqual(status.presentation(100.3, connected=True)[0], 'blind_zone')

    def test_trace_preserves_raw_bytes_but_labels_versioned_control_input(self):
        cases = ((VERSION, 0x5e, 0x5f, True), (VERSION, 0x2c, 0, False),
                 (0x20009, 0x0c, 1, False))
        with tempfile.TemporaryDirectory() as directory:
            writer = TraceArchiveWriter(directory)
            for version, flags, quality, valid in cases:
                writer.submit(collected(firmware=version, rows=[row(sensor_flags=flags, quality=quality)]))
            writer.request_close()
            self.assertTrue(writer.join(3))
            events = writer.take_events()
            self.assertEqual(len(events), len(cases))
            for event, (version, flags, quality, valid) in zip(events, cases):
                saved = json.loads(Path(event['json_path']).read_text(encoding='utf8'))
                item = saved['rows'][0]
                self.assertEqual(item['actual_valid'], valid)
                self.assertEqual(item['sensor_flags'], flags)
                self.assertEqual(item['adc_quality'], quality)
                self.assertEqual(item['raw_hex'], row(sensor_flags=flags, quality=quality).hex())
                if version == VERSION:
                    self.assertEqual(saved['adc_mean_q4_semantics'], 'conditioned_control_input')
                    self.assertEqual(item['adc_control_q4'], item['adc_mean_q4'])
                    self.assertEqual(saved['parameter_profile'], H_LQI_CONDITIONED_PROFILE)
                else:
                    self.assertNotIn('adc_control_q4', item)


class ConditionedUITests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_blind_scan_then_ready_requires_explicit_restart_and_logs_both_means(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch('host.app.list_ports.comports', return_value=[]):
                window = Window(Path(directory))
            window.timer.stop()
            window.plot_timer.stop()
            window.reset_data('serial')
            window.worker = WorkerStub()
            window.serial_ready = True
            window.direction_check.setChecked(True)
            window.receive(StreamDecoder().feed(conditioned_frame(flags=0x33))[0:1])
            for command in 'DUGHFBJKLM':
                self.assertFalse(window.allowed(command), command)
            self.assertTrue(window.allowed('S'))
            self.assertTrue(window.allowed('R'))
            window.receive(StreamDecoder().feed(conditioned_frame(sequence=2, flags=4))[0:1])
            self.assertFalse(window.allowed('H'))
            window.receive(StreamDecoder().feed(conditioned_frame(sequence=3, flags=0x5e))[0:1])
            self.assertTrue(window.allowed('H'))
            self.assertEqual(window.worker.sent, [])
            window.receive(StreamDecoder().feed(live(sequence=4, firmware=VERSION)))
            self.assertTrue(window.allowed('T'))
            runtime = window.runtime_log
            window.close_runtime_log()
            rows = [json.loads(line) for line in (runtime.directory / 'telemetry.jsonl').read_text(encoding='utf8').splitlines()]
            self.assertEqual(rows[0]['adc_mean_q4'], 12265)
            self.assertEqual(rows[0]['adc_control_q4'], 12000)
            self.assertEqual(rows[0]['sensor_flags'], 0x33)
            window.worker = None
            window.close()
            window.deleteLater()
            self.app.processEvents()


if __name__ == '__main__':
    unittest.main()
