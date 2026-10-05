"""Real protocol diagnostics, automatic runtime evidence and operator gates."""
import json
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

from PySide6.QtWidgets import QApplication
from host.app import Window
from host.core import Replay, SessionRecorder, StreamDecoder
from host.diagnostics import START_RESULTS
from test_host_ui import WorkerStub
from test_start_diagnostics_protocol import frame


class FeedbackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.window = Window(self.root)
        self.window.timer.stop()

    def tearDown(self):
        self.window.worker = None
        self.window.close()
        self.window.deleteLater()
        self.app.processEvents()
        self.tmp.cleanup()

    def live(self, diagnostic=0x80, fault=0):
        if not self.window.worker:
            self.window.reset_data('serial')
            self.window.worker = WorkerStub()
            self.window.serial_ready = True
        row = StreamDecoder().feed(frame(diagnostic, fault))[0]
        self.window.receive([row])
        self.window.update_display()
        return row

    def connect_without_device(self):
        self.window.ports.setEditText('TEST-NO-DEVICE')
        # Exercise real session initialization and signal wiring without
        # starting the I/O worker or opening a physical port.
        with patch('host.app.SerialWorker.start') as start:
            self.window.connect_serial()
            start.assert_called_once()
        self.window.serial_status('connected')

    def runtime_events(self):
        logger = self.window.runtime_log
        self.assertIsNotNone(logger)
        self.window.close_runtime_log()
        return [json.loads(line) for line in
                (logger.directory / 'control.jsonl').read_text(encoding='utf-8').splitlines()]

    def test_initial_frame_grace_shows_waiting_without_false_timeout_and_stop_works(self):
        with patch('host.app.time.monotonic', return_value=100.0) as clock:
            self.connect_without_device()
            self.window.update_display()
            self.assertIsNone(self.window.link_fresh)
            self.assertIn('等待首帧', self.window.source_badge.text())
            self.assertIn('首帧', self.window.start_gate.text())
            self.assertNotIn('已超时', self.window.diagnostic_label.text())
            self.assertFalse(self.window.command_buttons['G'].isEnabled())
            self.assertTrue(self.window.global_stop.isEnabled())
            self.assertTrue(self.window.send_command('S'))
            clock.return_value = 100.999
            self.window.update_display()
            self.assertTrue(self.window.waiting_for_first_frame())
            self.live()
            self.window.update_display()
            self.assertTrue(self.window.link_fresh)
            events = self.runtime_events()
        self.assertNotIn('telemetry_timeout', [event['event'] for event in events])
        self.assertEqual(sum(event['event'] == 'telemetry_restored' for event in events), 1)

    def test_first_frame_timeout_at_one_second_and_recovery_each_record_once(self):
        with patch('host.app.time.monotonic', return_value=100.0) as clock:
            self.connect_without_device()
            clock.return_value = 100.999
            self.window.update_display()
            self.assertIsNone(self.window.link_fresh)
            clock.return_value = 101.0
            self.window.update_display()
            self.window.update_display()
            self.assertFalse(self.window.link_fresh)
            self.assertIn('数据超时', self.window.source_badge.text())
            self.assertTrue(self.window.global_stop.isEnabled())
            clock.return_value = 101.5
            self.live()
            self.window.update_display()
            events = self.runtime_events()
        timeout = [event for event in events if event['event'] == 'telemetry_timeout']
        self.assertEqual(len(timeout), 1)
        self.assertEqual(timeout[0]['last_received_monotonic'], 0.0)
        self.assertEqual(sum(event['event'] == 'telemetry_restored' for event in events), 1)

    def test_reconnect_resets_receive_clock_and_checks_without_logging_reset_toggles(self):
        with patch('host.app.time.monotonic', return_value=100.0) as clock:
            self.connect_without_device()
            self.live()
            self.window.direction_check.setChecked(True)
            self.window.jog_check.setChecked(True)
            old_logger = self.window.runtime_log
            self.window.worker_finished()
            self.assertIsNone(self.window.worker)
            clock.return_value = 200.0
            self.connect_without_device()
            self.assertEqual(self.window.last_received, 0.0)
            self.assertEqual(self.window.connected_monotonic, 200.0)
            self.assertIsNone(self.window.latest)
            self.assertIsNone(self.window.link_fresh)
            self.assertFalse(self.window.direction_check.isChecked())
            self.assertFalse(self.window.jog_check.isChecked())
            self.window.update_display()
            self.assertTrue(self.window.waiting_for_first_frame())
            clock.return_value = 201.0
            self.window.update_display()
            new_events = self.runtime_events()
        old_events = [json.loads(line) for line in
                      (old_logger.directory / 'control.jsonl').read_text(encoding='utf-8').splitlines()]
        old_checks = [event for event in old_events if event['event'] == 'operator_checks']
        self.assertEqual(len(old_checks), 3)
        self.assertTrue(old_checks[-1]['direction_verified'])
        self.assertTrue(old_checks[-1]['jog_clearance'])
        new_checks = [event for event in new_events if event['event'] == 'operator_checks']
        self.assertEqual(len(new_checks), 1)
        self.assertTrue(new_checks[0]['initial_snapshot'])
        self.assertFalse(new_checks[0]['direction_verified'])
        self.assertFalse(new_checks[0]['jog_clearance'])
        timeout = next(event for event in new_events if event['event'] == 'telemetry_timeout')
        self.assertEqual(timeout['last_received_monotonic'], 0.0)
        self.assertEqual(len(list((self.root / 'fpga_logs').iterdir())), 2)

    def test_checks_and_each_command_gate_are_recorded_on_change_without_duplicates(self):
        self.live()
        self.window.jog_check.setChecked(True)
        self.assertTrue(self.window.command_buttons['F'].isEnabled())
        self.assertFalse(self.window.command_buttons['G'].isEnabled())
        self.window.update_display()
        self.window.jog_check.setChecked(True)
        self.window.direction_check.setChecked(True)
        self.assertTrue(self.window.command_buttons['G'].isEnabled())
        self.window.update_display()
        self.window.update_display()
        self.live(0x81, fault=1)
        self.window.update_display()
        events = self.runtime_events()
        checks = [event for event in events if event['event'] == 'operator_checks']
        self.assertEqual([(event['direction_verified'], event['jog_clearance']) for event in checks],
                         [(False, False), (False, True), (True, True)])
        self.assertEqual([event['initial_snapshot'] for event in checks], [True, False, False])
        gates = [event for event in events if event['event'] == 'control_availability']
        self.assertEqual(len(gates), 4)
        self.assertEqual([event['commands']['G']['available'] for event in gates], [False, False, True, False])
        self.assertEqual([event['commands']['F']['available'] for event in gates], [False, True, True, False])
        self.assertEqual([event['commands']['B']['available'] for event in gates], [False, True, True, False])
        self.assertIn('方向', gates[0]['commands']['G']['reason'])
        self.assertTrue(gates[-1]['commands']['G']['reason'])
        self.assertTrue(all(event['device_confirmed'] is False for event in checks + gates))

    def test_transient_gate_changes_inside_one_received_batch_are_preserved(self):
        self.live()
        self.window.direction_check.setChecked(True)
        decoder = StreamDecoder()
        rows = decoder.feed(frame(0x81, 1)) + decoder.feed(frame(0x80, 0))
        self.window.receive(rows)
        self.window.update_display()
        events = self.runtime_events()
        gates = [event for event in events if event['event'] == 'control_availability']
        self.assertEqual([event['commands']['G']['available'] for event in gates], [False, True, False, True])

    def test_operator_changes_and_reset_do_not_create_demo_replay_or_raw_runtime_logs(self):
        for source, mode in [('demo', 'project'), ('replay', 'project'), ('serial', 'raw')]:
            self.window.serial_mode = mode
            self.window.reset_data(source)
            self.window.worker = WorkerStub() if source == 'serial' else None
            self.window.serial_ready = source == 'serial'
            self.window.direction_check.setChecked(True)
            self.window.jog_check.setChecked(True)
            self.window.update_control_availability()
            self.window.reset_data(source)
            self.assertIsNone(self.window.runtime_log)
        self.assertFalse((self.root / 'fpga_logs').exists())

    def test_start_gate_is_immediately_below_start_button(self):
        layout = self.window.board_controls.layout()
        self.assertEqual(layout.indexOf(self.window.start_gate),
                         layout.indexOf(self.window.command_buttons['G']) + 1)

    def test_all_start_results_and_board_flags_reach_control_panel(self):
        for result, text in START_RESULTS.items():
            self.live(0x80 | result << 4)
            self.assertIn(text, self.window.diagnostic_label.text())
        row = self.live(0xBF, 1)
        for key in ('sensor_fault', 'input_fault', 'adc_over_range', 'stop_pressed'):
            self.assertEqual(row[key], 1)
        for text in ('SW3 按下', 'ADC 超量程', '接口异常锁存', '估计器异常锁存'):
            self.assertIn(text, self.window.diagnostic_label.text())
        self.assertFalse(self.window.allowed('G'))
        self.assertTrue(self.window.allowed('S'))

    def test_legacy_zero_never_claims_diagnostic_health(self):
        row = self.live(0)
        self.assertEqual(row['diagnostic_supported'], 0)
        self.assertNotIn('start_result', row)
        self.assertNotIn('sensor_fault', row)
        self.assertIn('固件未提供', self.window.diagnostic_label.text())

    def test_disabled_start_explains_direction_and_stop_conditions(self):
        self.live()
        self.assertIn('方向', self.window.start_gate.text())
        self.window.direction_check.setChecked(True)
        self.window.update_display()
        self.assertTrue(self.window.command_buttons['G'].isEnabled())
        self.live(0x88)
        self.assertIn('释放 SW3', self.window.start_gate.text())
        self.assertFalse(self.window.send_command('G'))
        self.assertTrue(self.window.send_command('S'))

    def test_start_latch_never_changes_actual_state_or_claims_motor_motion(self):
        self.live(0x90)
        self.assertEqual(self.window.cards['state'][0].text(), '待机')
        self.assertIn('锁存', self.window.diagnostic_label.text())
        self.assertEqual(self.window.latest['command_permille'], 0)

    def test_first_direction_check_allows_only_limited_jog_after_clearance(self):
        self.live()
        self.assertFalse(self.window.allowed('F'))
        self.window.jog_check.setChecked(True)
        self.assertTrue(self.window.allowed('F'))
        self.assertTrue(self.window.allowed('B'))
        self.assertFalse(self.window.allowed('G'))
        self.live(0x80, fault=4)
        self.assertFalse(self.window.allowed('F'))
        self.assertIn('旧值', self.window.diagnostic_label.text())
        self.assertTrue(self.window.allowed('S'))
        self.window.reset_data('serial')
        self.assertFalse(self.window.jog_check.isChecked())

    def test_runtime_samples_and_same_state_diagnostic_changes_record_without_opt_in(self):
        self.assertEqual(self.window.log_preferences, {'serial': False, 'control': False})
        self.live(0xA0)
        self.live(0xB1, 1)
        self.live(0xC2, 1)
        self.window.send_command('G')
        logger = self.window.runtime_log
        self.window.close_runtime_log()
        self.assertEqual(logger.directory.parent, self.root / 'fpga_logs')
        samples = [json.loads(x) for x in (logger.directory / 'telemetry.jsonl').read_text(encoding='utf-8').splitlines()]
        events = [json.loads(x) for x in (logger.directory / 'control.jsonl').read_text(encoding='utf-8').splitlines()]
        self.assertEqual([x['start_result'] for x in samples], [2, 3, 4])
        self.assertTrue(all(len(x['raw_hex']) == 48 for x in samples))
        self.assertEqual(len([e for e in events if e['event'] == 'telemetry_state']), 3)
        self.assertEqual(events[-1]['event'], 'command_rejected')
        self.assertFalse((self.root / 'logs').exists())

    def test_timeout_and_recovery_are_recorded_once_per_transition(self):
        self.live()
        self.window.last_received = time.monotonic() - 5
        self.window.update_display()
        self.window.update_display()
        self.assertIn('已超时', self.window.diagnostic_label.text())
        self.live()
        logger = self.window.runtime_log
        self.window.close_runtime_log()
        events = [json.loads(x)['event'] for x in (logger.directory / 'control.jsonl').read_text(encoding='utf-8').splitlines()]
        self.assertEqual(events.count('telemetry_timeout'), 1)
        self.assertEqual(events.count('telemetry_restored'), 2)

    def test_disk_failure_does_not_disable_stop_or_claim_complete_log(self):
        blocked = self.root / 'blocked'
        blocked.write_text('file')
        self.window.output_root = blocked
        self.live()
        logger = self.window.runtime_log
        if logger:
            with self.assertRaises(OSError):
                logger.close()
        self.window.update_display()
        self.assertIn('不完整', self.window.runtime_status.text())
        self.assertTrue(self.window.send_command('S'))

    def test_demo_and_raw_connections_do_not_create_fpga_logs(self):
        self.window.start_demo()
        self.window.runtime_event('fake')
        self.window.runtime_sample(StreamDecoder().feed(frame(0x80))[0])
        self.window.source = 'serial'
        self.window.serial_mode = 'raw'
        self.window.control_log('raw_event')
        self.assertIsNone(self.window.runtime_log)
        self.assertFalse((self.root / 'fpga_logs').exists())

    def test_record_and_legacy_csv_replay_recover_diagnostics_from_crc_frame(self):
        row = StreamDecoder().feed(frame(0xC6, 1))[0]
        with SessionRecorder(self.root, {'source': 'serial'}) as recorder:
            recorder.write(row)
        replay = Replay.load(recorder.directory / 'samples.csv')
        self.assertEqual(replay[0]['start_result'], 4)
        legacy = self.root / 'legacy.csv'
        legacy.write_text('elapsed_s,sequence,theta_deg,state,raw_hex\n' +
                          '0,17,0,0,' + row['raw_hex'] + '\n', encoding='utf-8')
        self.assertEqual(Replay.load(legacy)[0]['adc_over_range'], 1)


if __name__ == '__main__':
    unittest.main()
