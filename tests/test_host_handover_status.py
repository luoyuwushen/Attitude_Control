"""H handover is a current observation, never inferred from a UART write."""
import os
from pathlib import Path
import tempfile
import time
import unittest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PySide6.QtWidgets import QApplication, QScrollArea

from host.app import Window
from host.core import StreamDecoder
from host.handover_status import HANDOVER_STEPS, HandoverStatus
from test_host_handover_control import handover_frame
from test_host_ui import WorkerStub


def sample(at=100.0, sequence=10, device_ms=1000, **changes):
    row = dict(source='serial', protocol_version=2, host_monotonic=at,
               sequence=sequence, device_time_ms=device_ms, state=0, fault=0,
               calibrated=1, stop_pressed=0, h_control_flags=0,
               start_result=0, sensor_flags=12,
               sequence_duplicate=False, sequence_reset=False)
    row.update(changes)
    return row


class HandoverStatusTests(unittest.TestCase):
    def setUp(self):
        self.status = HandoverStatus()
        self.status.observe(sample(), 100.0)

    def code(self, now=100.2, **context):
        return self.status.presentation(now, connected=context.pop('connected', True), **context)[0]

    def start(self):
        self.status.queued('H', 100.1)

    def live(self, at=100.2, sequence=11, device_ms=1020, **changes):
        self.status.observe(sample(at, sequence, device_ms, **changes), at)

    def test_queued_and_written_never_mean_handover(self):
        self.start()
        self.assertEqual(self.code(), 'queued')
        self.status.sent('H')
        self.assertEqual(self.code(), 'written')
        self.assertIn('尚未观测到接管', self.status.presentation(100.2, connected=True)[1])

    def test_old_accepted_latch_and_balance_without_h_bit_are_not_handover(self):
        self.start()
        self.status.sent('H')
        self.live(start_result=1)
        self.assertEqual(self.code(), 'waiting')
        self.live(100.3, 12, 1040, state=2, start_result=1)
        self.assertNotEqual(self.code(100.3), 'running')

    def test_rejection_is_recent_device_result_not_attributed_ack(self):
        self.start()
        self.live(start_result=7)
        code, text, severity = self.status.presentation(100.2, connected=True)
        self.assertEqual((code, severity), ('rejected', 'warning'))
        self.assertIn('未观测到接管', text)
        self.assertIn('最近启动结果', text)
        self.assertNotIn('本次', text)

    def test_fresh_advancing_h_state_is_observed_without_using_start_result(self):
        self.start()
        self.live(state=2, h_control_flags=3, start_result=7)
        self.assertEqual(self.code(), 'running')
        self.assertEqual(self.status.presentation(100.2, connected=True)[2], 'success')

    def test_elapsed_freshness_boundary_clears_running(self):
        self.live(state=2, h_control_flags=1)
        self.assertEqual(self.code(101.199), 'running')
        self.assertEqual(self.code(101.2), 'stale')

    def test_buffered_frame_before_click_cannot_prove_handover(self):
        self.start()
        self.status.sent('H')
        self.status.observe(sample(100.05, 11, 1020, state=2, h_control_flags=1), 100.2)
        self.assertEqual(self.code(), 'written')

    def test_invalid_stale_future_duplicate_backward_or_history_cannot_prove_handover(self):
        for changes in (
                {'host_monotonic': 98.0}, {'host_monotonic': 101.0},
                {'host_monotonic': float('nan')}, {'host_monotonic': True},
                {'sequence': 10}, {'sequence': 9}, {'device_time_ms': 1000},
                {'device_time_ms': 999}, {'sequence_duplicate': True},
                {'sequence_reset': True}, {'source': 'replay'}, {'source': 'demo'},
                {'protocol_version': 3}, {'sequence': None}, {'device_time_ms': None}):
            with self.subTest(changes=changes):
                self.setUp()
                self.start()
                row = sample(100.2, 11, 1020, state=2, h_control_flags=1)
                row.update(changes)
                self.status.observe(row, 100.2)
                self.assertNotEqual(self.code(), 'running')

    def test_counter_wrap_and_forward_loss_do_not_prevent_current_observation(self):
        self.status.reset()
        self.status.observe(sample(100.0, 65535, 0xfffffff0), 100.0)
        self.start()
        self.live(100.2, 0, 4, state=2, h_control_flags=1)
        self.assertEqual(self.code(), 'running')
        self.live(100.3, 3, 64, state=2, h_control_flags=1)
        self.assertEqual(self.code(100.3), 'running')

    def test_equal_receive_clock_with_advancing_device_counters_is_allowed(self):
        self.start()
        self.live(100.2, 11, 1020)
        self.live(100.2, 12, 1040, state=2, h_control_flags=1)
        self.assertEqual(self.code(), 'running')

    def test_equal_request_boundary_still_cannot_prove_handover(self):
        self.start()
        self.live(100.1, 11, 1020, state=2, h_control_flags=1)
        self.assertNotEqual(self.code(), 'running')

    def test_missing_or_reserved_flags_fault_calibration_and_stop_cannot_show_success(self):
        for changes in ({'h_control_flags': None}, {'h_control_flags': 0x81},
                        {'fault': 8}, {'fault': None}, {'calibrated': 0},
                        {'stop_pressed': 1}, {'state': 3}, {'sensor_flags': 4},
                        {'sensor_flags': 8}, {'sensor_flags': 13}, {'sensor_flags': 28},
                        {'sensor_flags': None}):
            with self.subTest(changes=changes):
                self.setUp()
                row = sample(100.2, 11, 1020, state=2, h_control_flags=1)
                row.update(changes)
                self.status.observe(row, 100.2)
                self.assertNotEqual(self.code(), 'running')

    def test_wait_timeout_with_current_idle_and_silence(self):
        self.start()
        self.assertEqual(self.code(102.1), 'timeout')
        self.live(102.2, 11, 3000, start_result=1)
        self.assertEqual(self.code(102.2), 'timeout')

    def test_stop_cancels_wait_and_delayed_h_written_cannot_revive_it(self):
        self.start()
        self.status.queued('S', 100.15)
        self.status.sent('H')
        self.assertEqual(self.code(), 'stopping')
        self.live(state=2, h_control_flags=1)
        self.assertEqual(self.code(), 'stopping')
        self.live(100.3, 12, 1040)
        self.assertEqual(self.code(100.3), 'stopped')
        self.status.queued('H', 100.4)
        self.assertEqual(self.code(100.4), 'queued')

    def test_new_non_stop_operation_clears_old_stopping_hint(self):
        for command in 'DUGRFBJKLM':
            with self.subTest(command=command):
                self.status.queued('S', 100.1)
                self.status.queued(command, 100.2)
                self.assertNotEqual(self.code(), 'stopping')

    def test_counter_reset_preserves_stop_and_pre_request_boundaries(self):
        self.start()
        self.live(100.05, 0, 0, sequence_reset=True)
        self.live(100.08, 1, 20, state=2, h_control_flags=1)
        self.assertNotEqual(self.code(), 'running')
        self.status.queued('S', 100.15)
        self.live(100.2, 0, 0, sequence_reset=True)
        self.live(100.3, 1, 20, state=2, h_control_flags=1)
        self.assertEqual(self.code(100.3), 'stopping')

    def test_fault_idle_disconnect_and_offline_clear_running(self):
        self.live(state=2, h_control_flags=1)
        self.assertEqual(self.code(connected=False), 'disconnected')
        self.assertEqual(self.code(live=False), 'offline')
        self.live(100.3, 12, 1040, state=3, fault=1)
        self.assertEqual(self.code(100.3), 'fault')
        self.live(100.4, 13, 1060)
        self.assertEqual(self.code(100.4), 'idle')
        self.status.reset()
        self.assertEqual(self.code(), 'unverified')

    def test_new_device_fault_overrides_old_local_h_rejection(self):
        self.status.rejected('角度不满足接管条件')
        self.assertEqual(self.code(), 'not_sent')
        self.live(state=3, fault=1)
        self.assertEqual(self.code(), 'fault')
        self.assertEqual(self.status.presentation(100.2, connected=True)[2], 'error')

    def test_expired_telemetry_overrides_old_local_h_rejection(self):
        self.status.rejected('角度不满足接管条件')
        self.assertEqual(self.code(), 'not_sent')
        self.assertEqual(self.code(101.0), 'stale')


class HandoverStatusUITests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.window = Window(Path(self.temp.name))
        self.window.timer.stop()
        self.window.plot_timer.stop()
        self.window.reset_data('serial')
        self.window.worker = WorkerStub()
        self.window.serial_ready = True
        self.window.direction_check.setChecked(True)
        self.sequence = 0
        self.receive()

    def tearDown(self):
        loggers = (self.window.runtime_log, self.window.operation_log)
        self.window.worker = None
        self.window.close()
        for logger in loggers:
            if logger:
                logger.close()
        self.window.deleteLater()
        self.app.processEvents()
        self.temp.cleanup()

    def receive(self, **changes):
        # Match the 50 Hz production stream even on coarse Windows clocks.
        time.sleep(.02)
        self.sequence += 1
        raw = handover_frame(sequence=self.sequence, firmware=0x20009,
                             values=(0, 0, 0, 0))
        row = StreamDecoder().feed(raw)[0]
        row['device_time_ms'] = 1000 + self.sequence * 20
        row.update(changes)
        self.window.receive([row])
        self.window.update_display()
        return row

    def code(self):
        return self.window.handover_status_label.property('handoverStatus')

    def test_guidance_is_permanent_above_scrolling_controls(self):
        self.window.show()
        self.app.processEvents()
        self.assertEqual(self.window.handover_steps.text(), HANDOVER_STEPS)
        parent = self.window.handover_status_label.parentWidget()
        while parent:
            self.assertNotIsInstance(parent, QScrollArea)
            parent = parent.parentWidget()
        self.assertLess(self.window.handover_status_label.y(), self.window.pages.y())
        self.assertEqual(self.window.worker.sent, [])

    def test_send_write_and_old_latch_do_not_show_handover_but_current_h_does(self):
        self.assertTrue(self.window.allowed('H'))
        self.assertTrue(self.window.send_command('H'))
        self.assertEqual(self.code(), 'queued')
        self.window.command_sent('H')
        self.assertEqual(self.code(), 'written')
        self.receive(start_result=1)
        self.assertEqual(self.code(), 'waiting')
        self.receive(state=2, h_control_flags=3)
        self.assertEqual(self.code(), 'running')
        self.assertEqual(self.window.worker.sent, ['H'])
        self.assertFalse(self.window.allowed('H'))
        self.assertTrue(self.window.allowed('S'))

    def test_stale_buffered_packet_and_trace_status_cannot_confirm_h(self):
        self.window.send_command('H')
        self.window.command_sent('H')
        self.receive(host_monotonic=time.monotonic() - 2, state=2, h_control_flags=3)
        self.assertNotEqual(self.code(), 'running')
        self.window.receive_trace_status(dict(status='receiving', received_rows=16, total_rows=20))
        self.assertNotEqual(self.code(), 'running')
        self.assertEqual(self.window.worker.sent, ['H'])

    def test_one_delivery_batch_finishes_with_the_newest_live_observation(self):
        baseline = dict(self.window.latest)
        self.window.send_command('H')
        time.sleep(.02)
        old = dict(baseline, sequence=2, device_time_ms=1040,
                   host_monotonic=self.window.handover.requested_at - .01,
                   state=2, h_control_flags=3)
        current = dict(old, sequence=3, device_time_ms=1060, host_monotonic=time.monotonic())
        self.window.receive([old, current])
        self.window.update_display()
        self.assertEqual(self.code(), 'running')
        self.assertEqual(self.window.worker.sent, ['H'])

    def test_refusal_timeout_and_fault_do_not_resend_h(self):
        self.window.send_command('H')
        self.window.command_sent('H')
        self.receive(start_result=7)
        self.assertEqual(self.code(), 'rejected')
        self.window.handover.requested_at = time.monotonic() - 3
        self.window.update_display()
        self.assertEqual(self.code(), 'timeout')
        self.receive(state=3, fault=1)
        self.assertEqual(self.code(), 'fault')
        self.assertEqual(self.window.worker.sent, ['H'])

    def test_s_is_available_during_wait_and_clears_green_before_telemetry(self):
        self.window.send_command('H')
        self.receive(state=2, h_control_flags=3)
        self.assertTrue(self.window.send_command('S'))
        self.assertEqual(self.code(), 'stopping')
        self.window.command_sent('H')
        self.assertEqual(self.code(), 'stopping')
        self.receive()
        self.assertEqual(self.code(), 'stopped')
        self.assertEqual(self.window.worker.sent, ['H', 'S'])

    def test_disconnect_error_and_timeout_revoke_running_prompt(self):
        self.receive(state=2, h_control_flags=3)
        self.window.handover.record['host_monotonic'] = time.monotonic() - 1
        self.window.update_display()
        self.assertEqual(self.code(), 'stale')
        self.receive(state=2, h_control_flags=3)
        self.window.serial_status('disconnected')
        self.assertEqual(self.code(), 'disconnected')
        self.window.serial_ready = True
        self.receive(state=2, h_control_flags=3)
        self.window.serial_failed('test offline failure')
        self.assertEqual(self.code(), 'disconnected')
        self.assertEqual(self.window.worker.sent, [])

    def test_local_h_gate_failure_is_visible_and_s_remains_allowed(self):
        self.window.direction_check.setChecked(False)
        self.assertFalse(self.window.send_command('H'))
        self.assertEqual(self.code(), 'not_sent')
        self.assertTrue(self.window.allowed('S'))
        self.assertEqual(self.window.worker.sent, [])


if __name__ == '__main__':
    unittest.main()
