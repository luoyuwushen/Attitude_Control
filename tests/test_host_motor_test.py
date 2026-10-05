"""Fixed-PWM commands remain explicit, guarded and observable without COM I/O."""
import json
import os
from pathlib import Path
import queue
import tempfile
import time
import unittest
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PySide6.QtWidgets import QApplication

from host.app import Window, EXTENSIONS
from host.core import EXTENDED_FIELDS, Replay, SessionRecorder, StreamDecoder
from host.diagnostics import adc_detail_text, adc_diagnostic_text, motor_test_text
from host.measurements import MeasurementError, jog_response, validate_records
from host.operation_log import OperationLog
from host.transport import SerialWorker
from test_host_adc_diagnostics import frame, tlv, RTL_GOLDEN as GOLDEN_87
from test_host_adc_observability import RTL_GOLDEN_190, OBSERVABLE
from test_host_sensor_recovery import recovery_frame
from test_host_measurements import record as measurement_record
from test_host_ui import WorkerStub, FakePort


# Actual UART bytes from the RTL bench, independently provided after its PASS.
RTL_GOLDEN_199 = bytes.fromhex(
    'aa5502c70000fd0209fd0c007afe6000e803030121950104785634120204fffffeff'
    '0902e8030a0213000b02db000c02ff020d02ff030e0200000f02ff031002e01f'
    '110238ff120120130403000200140498badcfe15012916022913170211001802eb031902f137'
    '1a22010000010080ffff34127856e803e110c3001601a20d88132000feffff80efcdab89'
    '1b22cdabff031000f4016400c8002c019001f4015802bc0220038403e803dcfe10325476'
    '1c043412dcfe1d04cdab65871e01051f04eb32a4f818dd')


def motor_frame(status=0, delta=0, **kwargs):
    kwargs.setdefault('firmware', 0x20003)
    return frame(extra=tlv(30, '<B', status) + tlv(31, '<i', delta), **kwargs)


class MotorTestProtocolTests(unittest.TestCase):
    def test_actual_rtl_uart_golden_every_fragment_and_bytewise(self):
        self.assertEqual(len(RTL_GOLDEN_199), 199)
        for split in range(1, 199):
            decoder = StreamDecoder()
            self.assertEqual(decoder.feed(RTL_GOLDEN_199[:split]), [])
            rows = decoder.feed(RTL_GOLDEN_199[split:])
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]['motor_test_status'], 5)
            self.assertEqual(rows[0]['motor_test_delta'], -123456789)
            self.assertEqual(rows[0]['firmware_version'], 0x20003)
            self.assertEqual({k: rows[0][k] for k in OBSERVABLE}, OBSERVABLE)
        rows, decoder = [], StreamDecoder()
        for byte in RTL_GOLDEN_199:
            rows.extend(decoder.feed(bytes((byte,))))
        self.assertEqual(rows[0]['raw_hex'], RTL_GOLDEN_199.hex())
        self.assertEqual(decoder.stats['crc_errors'], 0)

    def test_legacy_frames_missing_optional_fields_and_crc_recovery(self):
        for raw in (frame(version=1), GOLDEN_87, recovery_frame(), RTL_GOLDEN_190):
            rows = StreamDecoder().feed(raw)
            self.assertEqual(len(rows), 1)
            self.assertNotIn('motor_test_status', rows[0])
            self.assertNotIn('motor_test_delta', rows[0])
            self.assertIn('未提供', motor_test_text(rows[0]))
        corrupted = bytearray(RTL_GOLDEN_199)
        corrupted[100] ^= 1
        decoder = StreamDecoder()
        rows = decoder.feed(corrupted + RTL_GOLDEN_199)
        self.assertEqual(len(rows), 1)
        self.assertEqual(decoder.stats['crc_errors'], 1)

    def test_new_scalar_duplicate_length_and_truncation_rejected(self):
        for kind, fmt in ((30, '<B'), (31, '<i')):
            good = tlv(kind, fmt, 5)
            for bad in (good + good, bytes((kind, 0)), good[:-1],
                        bytes((kind, len(good)-1)) + good[2:] + b'\0'):
                decoder = StreamDecoder()
                rows = decoder.feed(frame(extensions=False, extra=bad) + frame(sequence=2))
                self.assertEqual([r['sequence'] for r in rows], [2])
                self.assertEqual(decoder.stats['unsupported_frames'], 1)

    def test_export_preserves_signed_delta_and_state_five(self):
        rows = StreamDecoder().feed(motor_frame(1, -(2**31), state=5) +
                                    motor_frame(2, 2**31-1, sequence=2))
        for i, row in enumerate(rows):
            row['elapsed_s'] = i * .02
        self.assertTrue({'motor_test_status', 'motor_test_delta'} <= set(EXTENDED_FIELDS))
        self.assertTrue({'motor_test_status', 'motor_test_delta'} <= {x[0] for x in EXTENSIONS})
        with tempfile.TemporaryDirectory() as directory:
            recorder = SessionRecorder(Path(directory), {'source': 'offline_simulation'})
            for row in rows:
                recorder.write(row)
            path = recorder.directory
            recorder.close()
            replay = Replay.load(path / 'samples.csv')
            self.assertEqual([r['motor_test_delta'] for r in replay], [-(2**31), 2**31-1])
            self.assertEqual(replay[0]['state'], 5)
            log = OperationLog(Path(directory) / 'offline', {'source': 'offline_simulation'})
            for row in rows:
                log.write_telemetry(row)
            log.close()
            stored = [json.loads(line) for line in (log.directory/'telemetry.jsonl').read_text(encoding='utf-8').splitlines()]
            self.assertEqual([r['motor_test_delta'] for r in stored], [-(2**31), 2**31-1])

    def test_state_five_is_valid_data_but_not_legacy_jog_experiment(self):
        rows = [measurement_record(i, state=5, command_permille=150) for i in range(30)]
        self.assertEqual(len(validate_records(rows)), 30)
        with self.assertRaisesRegex(MeasurementError, 'state=4'):
            jog_response(rows)
        with self.assertRaises(MeasurementError):
            validate_records(rows, require_stopped=True)

    def test_reason_and_filter_semantics_do_not_claim_constant_speed(self):
        for status, expected in ((0, '无测试记录'), (1, '运行中'), (2, '时限'), (3, '位移'),
                                 (4, '手动停止'), (5, '故障'), (6, '编码器变化'), (7, '拒收')):
            self.assertIn(expected, motor_test_text(dict(motor_test_status=status, motor_test_delta=-7)))
        self.assertIn('-7 count', motor_test_text(dict(motor_test_status=7, motor_test_delta=-7)))
        for version, expected in ((0x20001, '3 点'), (0x20002, '3 点'), (0x20003, '7 点')):
            row = StreamDecoder().feed(motor_frame(firmware=version))[0]
            row['adc_detail_flags'] = 1
            self.assertIn(expected, adc_diagnostic_text(row))
            self.assertIn(expected, adc_detail_text(row))


class MotorTestUITests(unittest.TestCase):
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

    def tearDown(self):
        self.window.worker = None
        self.window.close()
        self.window.deleteLater()
        self.app.processEvents()
        self.temp.cleanup()

    def live(self, **kwargs):
        row = StreamDecoder().feed(motor_frame(**kwargs))[0]
        self.window.receive([row])
        self.window.update_display()
        return row

    def test_default_level_routes_each_direction_and_never_auto_starts(self):
        self.live()
        self.assertEqual(self.window.motor_test_level.currentData(), 150)
        self.assertFalse(self.window.motor_test_buttons[True].isEnabled())
        self.window.jog_check.setChecked(True)
        self.assertEqual(self.window.worker.sent, [])
        for level, commands in ((0, 'JK'), (1, 'LM')):
            self.window.motor_test_level.setCurrentIndex(level)
            for forward in (True, False):
                self.window.motor_test_buttons[forward].click()
            self.assertEqual(self.window.worker.sent[-2:], list(commands))
        self.assertFalse(self.window.direction_check.isChecked())

    def test_old_firmware_and_missing_flags_keep_tests_closed_but_preserve_jog(self):
        self.window.jog_check.setChecked(True)
        for kwargs in ({'version': 1}, {'firmware': None}, {'firmware': 0x20000},
                       {'firmware': 0x20001}, {'firmware': 0x20002}, {'flags': None}):
            self.live(**kwargs)
            for command in 'JKLM':
                self.assertFalse(self.window.allowed(command))
                self.assertFalse(self.window.send_command(command))
            self.assertTrue(self.window.allowed('F'))
            self.assertTrue(self.window.allowed('B'))
            self.assertTrue(self.window.allowed('S'))
        self.assertEqual(self.window.worker.sent, [])

    def test_new_session_resets_level_and_checks_without_transmitting(self):
        self.live()
        self.window.jog_check.setChecked(True)
        self.window.motor_test_level.setCurrentIndex(1)
        self.window.reset_data('serial')
        self.window.serial_ready = True
        self.live()
        self.assertEqual(self.window.motor_test_level.currentData(), 150)
        self.assertFalse(self.window.jog_check.isChecked())
        self.assertFalse(self.window.allowed('J'))
        self.assertEqual(self.window.worker.sent, [])

    def test_normal_and_failed_worker_completion_reset_level_after_tail_delivery(self):
        class IdlePort(FakePort):
            def read(port, count):
                time.sleep(.005)
                return super().read(count)

        for failure in (False, True):
            with self.subTest(failure=failure):
                self.window.worker = None
                self.window.ports.setEditText('OFFLINE-NEVER-OPEN')
                port = IdlePort()
                with patch('host.transport.serial.Serial', return_value=port):
                    self.window.connect_serial()
                    deadline = time.monotonic()+2
                    while not self.window.serial_ready and time.monotonic() < deadline:
                        self.app.processEvents()
                        time.sleep(.005)
                    self.assertTrue(self.window.serial_ready)
                    worker = self.window.worker
                    logger = self.window.runtime_log
                    self.window.motor_test_level.setCurrentIndex(1)
                    self.live(state=5, status=1)
                    self.assertFalse(self.window.motor_test_level.isEnabled())
                    if failure:
                        port.read_error = OSError('simulated unplug')
                    else:
                        self.window.disconnect_source()
                    deadline = time.monotonic()+2
                    while self.window.worker is not None and time.monotonic() < deadline:
                        self.app.processEvents()
                        time.sleep(.005)
                    if worker.isRunning():
                        worker.request_close(True)
                        worker.wait(1000)
                    self.assertIsNone(self.window.worker)
                self.assertEqual(self.window.motor_test_level.currentData(), 150)
                self.assertTrue(self.window.motor_test_level.isEnabled())
                self.assertFalse(self.window.motor_test_buttons[True].isEnabled())
                self.assertEqual(port.writes, [b'S'])
                events = [json.loads(line) for line in
                    (logger.directory/'control.jsonl').read_text(encoding='utf-8').splitlines()]
                self.assertEqual(events[-1]['event'], 'connection_finished')
                self.assertEqual(sum(e['event']=='operator_checks' for e in events), 1)
                self.assertEqual([e['motor_test_permille'] for e in events
                    if e['event']=='control_availability'][-1], 220)
                self.assertTrue(any(e['event']=='command_written' and e['command']=='S' for e in events))
                self.assertEqual(any(e['event']=='serial_error' for e in events), failure)

    def test_bad_unready_fault_busy_and_stale_never_allow_motor_test(self):
        self.window.jog_check.setChecked(True)
        for kwargs in ({'flags': 13}, {'flags': 14}, {'flags': 28}, {'flags': 4}, {'flags': 8},
                       {'fault': 1}, {'diagnostic': 0x88}, {'state': 1}, {'state': 4}, {'state': 5}):
            self.live(**kwargs)
            self.assertTrue(all(not self.window.allowed(c) for c in 'JKLM'))
            self.assertTrue(self.window.allowed('S'))
        self.live()
        self.window.latest['calibrated'] = 0
        self.assertFalse(self.window.allowed('J'))
        self.live()
        self.window.last_received = time.monotonic()-2
        self.assertFalse(self.window.send_motor_test(True))
        self.assertTrue(self.window.send_command('S'))
        self.assertEqual(self.window.worker.sent, ['S'])

    def test_running_blocks_all_other_motion_and_status_does_not_authorize_start(self):
        self.window.direction_check.setChecked(True)
        self.live(state=5, status=1, delta=32)
        for command in 'DURGHFBJKLM':
            self.assertFalse(self.window.allowed(command))
        self.assertTrue(self.window.global_stop.isEnabled())
        self.assertFalse(self.window.motor_test_level.isEnabled())
        self.assertEqual(self.window.cards['state'][0].text(), '定 PWM 测试')
        self.assertIn('+32 count', self.window.motor_test_result.text())
        self.live(status=7, delta=40)
        self.assertIn('拒收', self.window.motor_test_result.text())
        self.assertEqual(self.window.worker.sent, [])
        self.live(status=2, delta=48)
        self.assertIn('+48 count', self.window.motor_test_result.text())
        self.assertTrue(self.window.motor_test_level.isEnabled())

    def test_routes_availability_and_device_status_preserved_in_automatic_log(self):
        log = OperationLog(Path(self.temp.name)/'offline', {'source': 'offline_simulation'})
        self.window.runtime_log = log
        self.live()
        self.window.jog_check.setChecked(True)
        self.window.motor_test_buttons[True].click()
        self.window.command_sent('J')
        self.live(state=5, status=1, delta=12)
        self.assertFalse(self.window.send_motor_test(False))
        self.live(status=6, delta=12)
        self.window.motor_test_level.setCurrentIndex(1)
        self.window.motor_test_buttons[False].click()
        self.window.command_sent('M')
        self.live(status=7, delta=13)
        for _ in range(3):
            self.window.update_control_availability()
        self.window.close_runtime_log()
        events = [json.loads(l) for l in (log.directory/'control.jsonl').read_text(encoding='utf-8').splitlines()]
        self.assertEqual([e['command'] for e in events if e['event']=='command_queued'], ['J', 'M'])
        self.assertEqual([e['command'] for e in events if e['event']=='command_written'], ['J', 'M'])
        self.assertTrue(all(not e['acknowledged'] for e in events if e['event']=='command_written'))
        self.assertIn('K', [e['command'] for e in events if e['event']=='command_rejected'])
        gates = [e for e in events if e['event']=='control_availability']
        self.assertTrue(any(e['commands']['J']['available'] for e in gates))
        self.assertTrue(any(not e['commands']['M']['available'] for e in gates))
        self.assertEqual([e['motor_test_permille'] for e in gates[-2:]], [150, 220])
        statuses = [e['motor_test_status'] for e in events if e['event']=='telemetry_state']
        self.assertEqual(statuses, [0, 1, 6, 7])

    def test_demo_replay_cannot_transmit_even_with_new_firmware(self):
        self.window.jog_check.setChecked(True)
        for source in ('demo', 'replay'):
            self.window.source = source
            self.live()
            for command in 'JKLM':
                self.assertFalse(self.window.send_command(command))
        self.assertEqual(self.window.worker.sent, [])


class MotorTestTransportTests(unittest.TestCase):
    def test_new_letters_use_existing_write_evidence_and_stop_preemption(self):
        worker = SerialWorker('OFFLINE-NO-COM')
        for command in 'JKLM':
            self.assertTrue(worker.send(command))
        self.assertTrue(worker.send('S'))
        self.assertEqual(worker.commands.get_nowait()[0], 'S')
        with self.assertRaises(queue.Empty):
            worker.commands.get_nowait()
        for command in 'jklm':
            self.assertFalse(worker.send(command))
        for command in 'JKLM':
            worker = SerialWorker('OFFLINE-NO-COM')
            self.assertTrue(worker.send(command))
            port = FakePort(on_read=lambda: worker.request_close(True))
            sent, activity = [], []
            worker.command_sent.connect(sent.append)
            worker.wire_activity.connect(activity.append)
            with patch('host.transport.serial.Serial', return_value=port):
                worker.run()
            self.assertEqual(port.writes, [command.encode(), b'S'])
            self.assertEqual(sent, [command, 'S'])
            self.assertTrue(all(e['complete'] for e in activity if e['direction']=='TX'))


if __name__ == '__main__':
    unittest.main()
