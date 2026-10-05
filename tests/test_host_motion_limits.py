"""Host admission mirrors FPGA absolute motion limits without changing origins."""
import json
import math
import os
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PySide6.QtWidgets import QApplication
from host.app import Window
from host.core import StreamDecoder, crc16
from host.diagnostics import motion_limit_reason, motion_limits_text, recovery_hint
from host.operation_log import OperationLog
from test_host_motor_test import motor_frame
from test_host_ui import WorkerStub


def limits(arm=0, speed=0, omega=0):
    return dict(arm_q10=arm, arm_speed_q10=speed, omega_q10=omega, firmware_version=0x20004)


class MotionLimitDiagnosticsTests(unittest.TestCase):
    def test_arm_and_speed_strict_signed_boundaries_apply_to_their_modes(self):
        for name, boundary in (('arm_q10', 6144), ('arm_speed_q10', 20480)):
            for sign in (-1, 1):
                for magnitude in (boundary-1, boundary, boundary+1):
                    record = limits()
                    record[name] = sign*magnitude
                    for command in 'GHFBJKLM':
                        with self.subTest(name=name, sign=sign, magnitude=magnitude, command=command):
                            reason = motion_limit_reason(record, command)
                            self.assertEqual(bool(reason), magnitude > boundary and
                                             (name == 'arm_speed_q10' or command in 'GH'))
                            if reason:
                                self.assertIn('超出绝对限位', reason)
                                self.assertIn('+' if sign > 0 else '-', reason)

    def test_pendulum_speed_limit_only_applies_to_g_and_h(self):
        for sign in (-1, 1):
            for magnitude in (30719, 30720, 30721, 32767):
                record = limits(omega=sign*magnitude)
                for command in 'GH':
                    self.assertEqual(bool(motion_limit_reason(record, command)), magnitude > 30720)
                for command in 'FBJKLM':
                    self.assertEqual(motion_limit_reason(record, command), '')
        self.assertIn('超出', motion_limit_reason(limits(arm=-32768), 'G'))

    def test_stop_clear_and_calibration_are_not_motion_gated(self):
        for record in (None, {}, limits(arm=-32768, speed=32767, omega=32767)):
            for command in 'SRDU':
                self.assertEqual(motion_limit_reason(record, command), '')

    def test_old_firmware_manual_position_limit_is_still_mirrored(self):
        for version in (None, 0x20000, 0x20001, 0x20002, 0x20003):
            record=dict(limits(arm=-6972),firmware_version=version)
            for command in 'FBJKLM':
                self.assertIn('超出绝对限位',motion_limit_reason(record,command))
            self.assertNotIn('短 F/B',recovery_hint(dict(record,fault=0)))
            self.assertNotIn('G/H 摆臂角',motion_limits_text(record))

    def test_legacy_display_fields_work_and_invalid_raw_cannot_override_with_display(self):
        record = dict(arm_deg=6*180/math.pi, arm_speed_rad_s=-20, omega_rad_s=30)
        self.assertEqual(motion_limit_reason(record, 'G'), '')
        for key in record:
            changed = dict(record)
            changed[key] *= 1.0001
            self.assertIn('超出', motion_limit_reason(changed, 'G'))
        for bad in (None, True, 6144.5, 32768, -32769, float('nan'), float('inf')):
            self.assertIn('等待有效', motion_limit_reason(dict(record, arm_q10=bad), 'G'))
        self.assertEqual(motion_limit_reason(dict(record, arm_q10=0), 'G'), '')

    def test_missing_and_nonfinite_required_fields_fail_closed(self):
        for key in ('arm_q10', 'arm_speed_q10', 'omega_q10'):
            record = limits()
            del record[key]
            self.assertIn('等待有效', motion_limit_reason(record, 'G'))
        record = limits()
        del record['omega_q10']
        self.assertEqual(motion_limit_reason(record, 'J'), '')
        del record['arm_q10']
        self.assertEqual(motion_limit_reason(record, 'J'), '')
        for invalid in (True, '0', None, float('nan'), float('inf')):
            self.assertIn('等待有效', motion_limit_reason(dict(arm_deg=0, arm_speed_rad_s=invalid), 'F'))
        self.assertIn('未提供', motion_limits_text({}))

    def test_position_recovery_explains_reference_and_does_not_suggest_resetting_u(self):
        record = dict(limits(arm=-6972), fault=0, calibrated=1)
        reason = motion_limit_reason(record, 'G')
        for text in ('-390.104', '343.775', 'R 只清故障', '不清摆臂原点', '断使能', '沿原路', '原标定零位', '不要用 U'):
            self.assertIn(text, reason)
        self.assertIn('短 F/B', reason)
        self.assertIn('G/H 摆臂角', reason)
        self.assertEqual(recovery_hint(record), reason)
        self.assertIn('（越限）', motion_limits_text(record))


class MotionLimitUITests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        with patch('host.app.list_ports.comports', return_value=[]):
            self.window = Window(Path(self.tmp.name))
        self.window.timer.stop()
        self.window.plot_timer.stop()
        self.window.reset_data('serial')
        self.window.direction_check.setChecked(True)
        self.window.worker = WorkerStub()
        self.window.serial_ready = True
        self.sequence = 0

    def tearDown(self):
        self.window.worker = None
        self.window.close()
        self.window.deleteLater()
        self.app.processEvents()
        self.tmp.cleanup()

    def receive(self, arm=0, speed=0, omega=0, **kwargs):
        self.sequence += 1
        kwargs.setdefault('firmware',0x20004)
        raw = bytearray(motor_frame(sequence=self.sequence, omega=omega, **kwargs))
        struct.pack_into('<h', raw, 12, arm)
        struct.pack_into('<h', raw, 14, speed)
        raw[-2:] = crc16(raw[:-2]).to_bytes(2, 'little')
        row = StreamDecoder().feed(bytes(raw))[0]
        self.window.receive([row])
        self.window.update_display()
        return row

    def test_clear_fault_does_not_bypass_position_limit_and_returning_inside_restores_admission(self):
        self.receive(arm=-6972, fault=0, status=7)
        for command in 'GH':
            self.assertFalse(self.window.allowed(command))
            self.assertIn('摆臂角', self.window.command_block_reason(command))
        for command in 'FBJKLM':
            self.assertTrue(self.window.allowed(command))
        self.assertTrue(self.window.allowed('S'))
        self.assertTrue(self.window.allowed('R'))
        self.assertFalse(self.window.send_command('G'))
        self.assertIn('-390.104', self.window.message.text())
        self.assertIn('G 未发送', self.window.events_text.toPlainText())
        self.assertIn('绝对运动限位', self.window.motor_diagnostic_label.text())
        self.assertIn('343.77', self.window.motor_diagnostic_label.text())
        self.assertIn('R 只清故障', self.window.diagnostic_label.text())
        self.window.command_buttons['R'].click()
        self.assertEqual(self.window.worker.sent, ['R'])
        self.assertEqual(self.window.latest['arm_q10'], -6972)
        self.receive(arm=-6972, fault=0, status=7)
        self.assertFalse(self.window.allowed('G'))
        self.receive(arm=-6144, fault=0)
        for command in 'GHFBJKLM':
            self.assertTrue(self.window.allowed(command), command)
        self.assertEqual(self.window.worker.sent, ['R'])

    def test_arm_speed_and_gh_pendulum_speed_keep_stop_clear_and_h_capture_rules(self):
        for signed in (-20481, 20481):
            self.receive(speed=signed)
            self.assertTrue(all(not self.window.allowed(c) for c in 'GHFBJKLM'))
            self.assertTrue(self.window.allowed('R'))
            self.assertTrue(self.window.allowed('S'))
        self.receive(speed=20480)
        self.assertTrue(self.window.allowed('G'))
        self.receive(omega=30720)
        self.assertTrue(self.window.allowed('G'))
        self.assertFalse(self.window.allowed('H'))
        self.assertIn('直立接管需', self.window.command_block_reason('H'))
        self.receive(omega=-30721)
        for command in 'GH':
            self.assertIn('摆杆速度', self.window.command_block_reason(command))
        self.assertTrue(all(self.window.allowed(c) for c in 'FBJKLM'))
        self.receive(state=5, speed=20481)
        self.assertFalse(self.window.allowed('R'))
        self.assertTrue(self.window.allowed('S'))

    def test_legacy_v1_retains_supported_controls_with_absolute_limits(self):
        self.receive(version=1, arm=6144)
        self.assertTrue(all(self.window.allowed(c) for c in 'GFB'))
        self.assertFalse(self.window.allowed('H'))
        self.assertFalse(self.window.allowed('J'))
        self.receive(version=1, arm=6145)
        self.assertTrue(all(not self.window.allowed(c) for c in 'GFB'))
        self.assertTrue(self.window.allowed('R'))
        self.assertTrue(self.window.allowed('S'))

    def test_new_firmware_jog_requires_valid_mature_measurements_before_enabling_or_sending(self):
        for firmware in (0x20004, 0x20005):
            for flags, reason in ((0, '测量无效'), (4, '尚未就绪'), (None, '提供 ADC 有效与就绪状态')):
                self.receive(firmware=firmware, flags=flags)
                for command in 'FB':
                    with self.subTest(firmware=firmware, flags=flags, command=command):
                        self.assertFalse(self.window.command_buttons[command].isEnabled())
                        self.assertIn(reason, self.window.command_block_reason(command))
                        self.assertFalse(self.window.send_command(command))
                self.assertTrue(self.window.allowed('S'))
                self.assertTrue(self.window.allowed('R'))
            self.receive(firmware=firmware, flags=12)
            for command in 'FB':
                self.assertTrue(self.window.command_buttons[command].isEnabled())
                self.assertTrue(self.window.allowed(command))
        self.assertEqual(self.window.worker.sent, [])
        self.assertTrue(self.window.send_command('F'))
        self.assertTrue(self.window.send_command('B'))
        self.assertEqual(self.window.worker.sent, ['F', 'B'])

    def test_jog_legacy_without_measurement_contract_does_not_require_new_flags(self):
        for kwargs in (dict(version=1), dict(version=2, extensions=False),
                       dict(version=2, firmware=None, flags=None)):
            row = self.receive(**kwargs)
            self.assertNotIn('sensor_flags', row)
            for command in 'FB':
                with self.subTest(frame=kwargs, command=command):
                    self.assertTrue(self.window.command_buttons[command].isEnabled())
                    self.assertEqual(self.window.command_block_reason(command), '')
        # v0.2.0..3 expose diagnostics but their F/B contract predates the
        # valid/ready admission added in v0.2.4. Preserve that explicit boundary.
        for firmware in (0x20000, 0x20001, 0x20002, 0x20003):
            for flags in (0, 4, None):
                self.receive(firmware=firmware, flags=flags)
                for command in 'FB':
                    with self.subTest(firmware=firmware, flags=flags, command=command):
                        self.assertTrue(self.window.command_buttons[command].isEnabled())
                        self.assertEqual(self.window.command_block_reason(command), '')
                for command in 'GHJKLM':
                    self.assertFalse(self.window.allowed(command))
        self.assertEqual(self.window.worker.sent, [])

    def test_log_records_rejection_and_restored_gate_without_fake_device_ack(self):
        logger = OperationLog(Path(self.tmp.name)/'offline', {'source': 'offline_simulation'})
        self.window.runtime_log = logger
        self.receive(arm=6145)
        self.assertFalse(self.window.send_command('G'))
        self.receive(arm=6144)
        self.window.close_runtime_log()
        events = [json.loads(line) for line in (logger.directory/'control.jsonl').read_text(encoding='utf-8').splitlines()]
        gates = [e for e in events if e['event']=='control_availability']
        self.assertFalse(gates[0]['commands']['G']['available'])
        self.assertIn('超出绝对限位', gates[0]['commands']['G']['reason'])
        self.assertTrue(gates[0]['commands']['J']['available'])
        self.assertTrue(gates[-1]['commands']['G']['available'])
        rejected = [e for e in events if e['event']=='command_rejected']
        self.assertEqual([e['command'] for e in rejected], ['G'])
        self.assertFalse(any(e['event'] in ('command_queued','command_written') for e in events))
        self.assertEqual(self.window.worker.sent, [])


if __name__=='__main__':
    unittest.main()
