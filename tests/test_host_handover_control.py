"""H observations are optional evidence, never an extra motion authorization."""
import csv
import json
import math
import os
from pathlib import Path
import struct
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PySide6.QtWidgets import QApplication

from host.app import EXTENSIONS, HANDOVER_TOOLTIP, Window
from host.core import (BASE_FIELDS, EXTENDED_FIELDS, HANDOVER_CONTROL_FIELDS,
                       Replay, SessionRecorder, StreamDecoder, crc16)
from host.diagnostics import adc_diagnostic_text, handover_control_text, motor_diagnostic_text
from host.operation_log import OperationLog
from host.version import HOST_VERSION, PROJECT_VERSION
from host.tuning import (H_LQI_PROFILE, H_LQI_S110_PROFILE, H_LQI_S120_PROFILE, TUNING_CSV_FIELDS,
                         TUNING_SCHEMA, tuning_values)
from tools.telemetry_monitor import decode as cli_decode
from test_host_adc_diagnostics import frame
from test_host_motor_test import RTL_GOLDEN_199
from test_host_ui import WorkerStub

# Actual UART vector supplied by the FPGA bench, not created by host helpers.
RTL_GOLDEN_208 = bytes.fromhex(
    'aa5502d00000fd0209fd0c007afe6000e803030121950104785634120204fffffeff'
    '0902e8030a0213000b02db000c02ff020d02ff030e0200000f02ff031002e01f'
    '110238ff120120130406000200140498badcfe15012916022913170211001802eb031902f137'
    '1a22010000010080ffff34127856e803e110c3001601a20d88132000feffff80efcdab89'
    '1b22cdabff031000f4016400c8002c019001f4015802bc0220038403e803dcfe10325476'
    '1c043412dcfe1d04cdab65871e01051f04eb32a4f82007c7cfc4fe0002030801')


def group(integral=-25601, capture=-6144, age=512, flags=15):
    return b'\x20\x07' + struct.pack('<hhHB', integral, capture, age, flags)


def handover_frame(*, values=(-25601, -6144, 512, 15), **kwargs):
    kwargs.setdefault('firmware', 0x20006)
    return frame(extra=group(*values), **kwargs)


def full_frame(values=(-25601, -6144, 512, 15), *, firmware=0x20006):
    # Host synthetic extension of the independently retained old 199B vector;
    # this fixture is not claimed to have been emitted by the new RTL.
    body = bytearray(RTL_GOLDEN_199[:-2])
    body[3] = 208
    body[75:79] = struct.pack('<I', firmware)
    body.extend(group(*values))
    return bytes(body) + crc16(body).to_bytes(2, 'little')


class HandoverProtocolTests(unittest.TestCase):
    def test_independent_rtl_uart_golden(self):
        row = StreamDecoder().feed(RTL_GOLDEN_208)[0]
        self.assertEqual(len(RTL_GOLDEN_208), 208)
        self.assertEqual(tuple(row[k] for k in HANDOVER_CONTROL_FIELDS), (-12345, -316, 512, 3))
        self.assertEqual(row['firmware_version'], 0x20006)
        self.assertEqual(row['raw_hex'], RTL_GOLDEN_208.hex())

    def test_208_byte_layout_every_fragment_and_one_byte_delivery(self):
        raw = full_frame()
        self.assertEqual(len(raw), 208)
        self.assertEqual(raw[197:206], group())
        expected = dict(zip(HANDOVER_CONTROL_FIELDS, (-25601, -6144, 512, 15)))
        for split in range(1, len(raw)):
            decoder = StreamDecoder()
            self.assertEqual(decoder.feed(raw[:split]), [])
            rows = decoder.feed(raw[split:])
            self.assertEqual({k: rows[0][k] for k in expected}, expected)
            self.assertEqual(rows[0]['firmware_version'], 0x20006)
            self.assertEqual(rows[0]['motor_test_delta'], -123456789)
        decoder = StreamDecoder()
        rows = []
        for byte in raw:
            rows.extend(decoder.feed(bytes((byte,))))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['raw_hex'], raw.hex())
        self.assertEqual(decoder.stats['crc_errors'], 0)

    def test_signed_extrema_unsigned_age_and_flags_are_retained_without_clipping(self):
        for values in [(-32768, 32767, 65535, 255), (32767, -32768, 0, 0), (-1, -1, 384, 3)]:
            with self.subTest(values=values):
                row = StreamDecoder().feed(handover_frame(values=values))[0]
                self.assertEqual(tuple(row[k] for k in HANDOVER_CONTROL_FIELDS), values)
        text = handover_control_text(dict(zip(HANDOVER_CONTROL_FIELDS, (-32768, -1024, 512, 3))))
        self.assertIn('-128.000‰', text)
        self.assertIn('-57.30°', text)
        self.assertIn('位置渐入计时 512 ms', text)

    def test_missing_legacy_v1_v2_and_199_byte_frames_do_not_synthesize_values(self):
        for raw in (frame(version=1), frame(extensions=False), RTL_GOLDEN_199,
                    frame(firmware=0x20005), frame(firmware=0x20006)):
            row = StreamDecoder().feed(raw)[0]
            self.assertTrue(all(k not in row for k in HANDOVER_CONTROL_FIELDS))
            self.assertIn('居中积分修正 未提供', handover_control_text(row))
        self.assertIn('居中积分修正 未提供', motor_diagnostic_text({'firmware_version': 0x20006}))

    def test_malformed_group_and_duplicate_rejected_without_partial_record(self):
        good = group()
        malformed = [good + good, b'\x20\x00', good[:-1], b'\x20\x06' + good[2:-1],
                     b'\x20\x08' + good[2:] + b'\0', b'\x20']
        for bad in malformed:
            with self.subTest(bad=bad.hex()):
                decoder = StreamDecoder()
                rows = decoder.feed(frame(extensions=False, extra=bad) + handover_frame(sequence=2))
                self.assertEqual([r['sequence'] for r in rows], [2])
                self.assertEqual(decoder.stats['unsupported_frames'], 1)

    def test_crc_error_recovers_to_legacy_and_handover(self):
        corrupt = bytearray(full_frame())
        corrupt[204] ^= 0x80
        decoder = StreamDecoder()
        rows = decoder.feed(bytes(corrupt) + frame(sequence=1, version=1) + handover_frame(sequence=2))
        self.assertEqual([r['sequence'] for r in rows], [1, 2])
        self.assertNotIn('h_integral_q8', rows[0])
        self.assertEqual(rows[1]['h_integral_q8'], -25601)
        self.assertEqual(decoder.stats['crc_errors'], 1)

    def test_shared_cli_and_fixed_csv_schema_include_group(self):
        raw = full_frame((25600, 1024, 384, 9))
        row = cli_decode(raw)
        self.assertEqual(tuple(row[k] for k in HANDOVER_CONTROL_FIELDS), (25600, 1024, 384, 9))
        fields = BASE_FIELDS + EXTENDED_FIELDS
        self.assertEqual(len(fields), len(set(fields)))
        self.assertTrue(set(HANDOVER_CONTROL_FIELDS) <= set(fields))
        self.assertTrue(set(HANDOVER_CONTROL_FIELDS) <= {x[0] for x in EXTENSIONS})

    def test_csv_replay_and_json_log_roundtrip_keep_integer_precision_and_missing(self):
        rows = StreamDecoder().feed(frame(sequence=1, version=1) +
             handover_frame(sequence=2, values=(-32768, 32767, 65535, 255)) +
             handover_frame(sequence=3, values=(32767, -32768, 0, 0)))
        for i, row in enumerate(rows):
            row['elapsed_s'] = i * .02
        with tempfile.TemporaryDirectory() as directory:
            recorder = SessionRecorder(directory, {'source': 'offline_simulation'})
            for row in rows:
                recorder.write(row)
            path = recorder.directory
            recorder.close()
            replay = Replay.load(path / 'samples.csv')
            self.assertTrue(all(k not in replay[0] for k in HANDOVER_CONTROL_FIELDS))
            for original, restored in zip(rows[1:], replay[1:]):
                self.assertEqual([restored[k] for k in HANDOVER_CONTROL_FIELDS],
                                 [original[k] for k in HANDOVER_CONTROL_FIELDS])
            with (path / 'samples.csv').open(encoding='utf-8-sig', newline='') as source:
                csv_rows = list(csv.DictReader(source))
            self.assertTrue(all(csv_rows[0][k] == '' for k in HANDOVER_CONTROL_FIELDS))
            log = OperationLog(Path(directory)/'offline', {'source': 'offline_simulation'})
            for row in rows:
                log.write_telemetry(row)
            log.close()
            stored = [json.loads(line) for line in (log.directory/'telemetry.jsonl').read_text(encoding='utf-8').splitlines()]
            self.assertNotIn('h_integral_q8', stored[0])
            for original, restored in zip(rows[1:], stored[1:]):
                self.assertEqual([restored[k] for k in HANDOVER_CONTROL_FIELDS],
                                 [original[k] for k in HANDOVER_CONTROL_FIELDS])

    def test_flags_are_observations_not_ack_or_fault(self):
        for flags, expected in ((0, 'H 未运行'), (1, 'H 运行'), (3, '积分更新允许'),
                                (5, '抗饱和冻结'), (9, '积分到达限幅'), (129, '未知标志位')):
            text = handover_control_text({'h_control_flags': flags})
            self.assertIn(expected, text)
            self.assertIn(f'0x{flags:02X}', text)
            self.assertNotIn('已接纳', text)
        for invalid in (None, True, -1, 256, float('nan')):
            self.assertIn('H 标志 未提供', handover_control_text({'h_control_flags': invalid}))
        self.assertIn('不是H总运行时长', HANDOVER_TOOLTIP)

    def test_explicit_firmware_names_and_host_release(self):
        for fw, name in ((0x20004, 'project-v0.2.4'), (0x20005, 'project-v0.2.5'),
                         (0x20006, 'project-v0.2.6'), (0x20007, 'project-v0.2.7'),
                         (0x20008, 'project-v0.2.8')):
            self.assertIn(name, adc_diagnostic_text({'firmware_version': fw, 'adc_raw': 200}))
        unknown = adc_diagnostic_text({'firmware_version': 0x2000a, 'adc_raw': 200})
        self.assertIn('0x0002000A', unknown)
        self.assertNotIn('project-v', unknown)
        self.assertEqual(HOST_VERSION, 'host-v0.7.0')
        self.assertEqual(PROJECT_VERSION, 'project-v0.3.0')

    def test_s110_208_byte_synthetic_frame_preserves_group_and_selects_exact_profile(self):
        raw = bytearray(full_frame((-320, 1024, 512, 3), firmware=0x20007))
        raw[18], raw[20] = 2, 0
        raw[-2:] = crc16(raw[:-2]).to_bytes(2, 'little')
        decoder = StreamDecoder()
        rows = []
        for byte in raw:
            rows.extend(decoder.feed(bytes((byte,))))
        self.assertEqual(len(raw), 208)
        self.assertEqual(len(rows), 1)
        row = rows[0]
        self.assertEqual(row['firmware_version'], 0x20007)
        self.assertEqual(tuple(row[k] for k in HANDOVER_CONTROL_FIELDS), (-320, 1024, 512, 3))
        self.assertEqual(row['parameter_profile'], H_LQI_S110_PROFILE)
        self.assertEqual(row['algorithm'], 'H_LQI')
        self.assertEqual(row['target'], 0)
        self.assertTrue(row['active'])
        self.assertEqual(row['raw_hex'], raw.hex())
        self.assertEqual(decoder.stats['crc_errors'], 0)

    def test_old_and_s110_definitions_differ_only_in_identity_and_speed_gain(self):
        definitions = TUNING_SCHEMA['profile_definitions']
        old, new = definitions[H_LQI_PROFILE], definitions[H_LQI_S110_PROFILE]
        self.assertEqual(old['gain_q10'], [904758, 77909, -34213, -51473])
        self.assertEqual(new['gain_q10'], [904758, 77909, -34213, -56620])
        self.assertEqual(old['firmware_version'], 0x20006)
        self.assertEqual(new['firmware_version'], 0x20007)
        self.assertEqual(new['source'], 'known_firmware_definition')
        self.assertEqual(new['physical_validation'], 'candidate_not_accepted')
        for key in old:
            if key not in ('firmware_version', 'gain_q10'):
                self.assertEqual(old[key], new[key], key)
        self.assertIsNot(old['gain_q10'], new['gain_q10'])

    def test_s120_synthetic_frame_all_splits_selects_only_its_static_definition(self):
        raw = bytearray(full_frame((-320, -1024, 512, 3), firmware=0x20008))
        raw[18], raw[20] = 2, 0
        raw[-2:] = crc16(raw[:-2]).to_bytes(2, 'little')
        self.assertEqual(len(raw), 208)
        for split in range(1, len(raw)):
            with self.subTest(split=split):
                decoder = StreamDecoder()
                self.assertEqual(decoder.feed(raw[:split]), [])
                rows = decoder.feed(raw[split:])
                self.assertEqual(len(rows), 1)
                row = rows[0]
                self.assertEqual(row['firmware_version'], 0x20008)
                self.assertEqual(tuple(row[k] for k in HANDOVER_CONTROL_FIELDS), (-320, -1024, 512, 3))
                self.assertEqual(row['parameter_profile'], H_LQI_S120_PROFILE)
                self.assertTrue(row['active'])
                self.assertEqual(row['algorithm'], 'H_LQI')
                self.assertEqual(row['target'], 0)
                self.assertEqual(row['raw_hex'], raw.hex())
                self.assertEqual(decoder.stats['crc_errors'], 0)

    def test_s120_static_definition_preserves_old_parameters_and_does_not_alias_them(self):
        definitions = TUNING_SCHEMA['profile_definitions']
        new = definitions[H_LQI_S120_PROFILE]
        self.assertEqual(new['firmware_version'], 0x20008)
        self.assertEqual(new['gain_q10'], [904758, 77909, -34213, -61768])
        self.assertEqual(new['source'], 'known_firmware_definition')
        self.assertEqual(new['physical_validation'], 'candidate_not_accepted')
        for profile, version, gain in ((H_LQI_PROFILE, 0x20006, -51473),
                                       (H_LQI_S110_PROFILE, 0x20007, -56620)):
            old = definitions[profile]
            self.assertEqual(old['firmware_version'], version)
            self.assertEqual(old['gain_q10'], [904758, 77909, -34213, gain])
            self.assertIsNot(new, old)
            self.assertIsNot(new['gain_q10'], old['gain_q10'])
            for key in new:
                if key not in ('firmware_version', 'gain_q10'):
                    self.assertEqual(new[key], old[key], key)

    def test_versioned_profiles_survive_csv_replay_and_automatic_metadata(self):
        rows = [StreamDecoder().feed(handover_frame(sequence=i, firmware=fw, state=2,
                    values=(320, 1024, 512, 3)))[0]
                for i, fw in enumerate((0x20006, 0x20007, 0x20008, 0x2000a), start=1)]
        for i, row in enumerate(rows):
            row['elapsed_s'] = i*.02
        with tempfile.TemporaryDirectory() as directory:
            recorder = SessionRecorder(directory, {'source': 'offline_simulation'})
            log = OperationLog(directory, {'source': 'offline_simulation'}, directory_name='fpga_logs')
            for row in rows:
                recorder.write(row)
                log.write_telemetry(row)
            recorder.close()
            log.close()
            expected = [H_LQI_PROFILE, H_LQI_S110_PROFILE, H_LQI_S120_PROFILE, None]
            restored = Replay.load(recorder.directory/'samples.csv')
            self.assertEqual([row['parameter_profile'] for row in restored], expected)
            stored = [json.loads(line) for line in (log.directory/'telemetry.jsonl').read_text(
                encoding='utf-8').splitlines()]
            self.assertEqual([row['parameter_profile'] for row in stored], expected)
            with (log.directory/'tuning.csv').open(encoding='utf-8-sig', newline='') as stream:
                narrow = list(csv.DictReader(stream))
            self.assertEqual([row['parameter_profile'] for row in narrow], expected[:3]+[''])
            self.assertEqual(narrow[3]['target'], '')
            self.assertEqual(narrow[3]['algorithm'], 'unknown')
            for path in (recorder.directory, log.directory):
                definitions = json.loads((path/'metadata.json').read_text(
                    encoding='utf-8'))['tuning_schema']['profile_definitions']
                self.assertEqual(definitions[H_LQI_PROFILE]['gain_q10'][-1], -51473)
                self.assertEqual(definitions[H_LQI_S110_PROFILE]['gain_q10'][-1], -56620)
                self.assertEqual(definitions[H_LQI_S120_PROFILE]['gain_q10'][-1], -61768)

    def test_tuning_aliases_exact_profile_and_error_semantics(self):
        row = StreamDecoder().feed(handover_frame(state=2, theta=128, values=(-320, 1024, 512, 3)))[0]
        self.assertEqual(row['algorithm'], 'H_LQI')
        self.assertEqual(row['parameter_profile'], H_LQI_PROFILE)
        self.assertTrue(row['active'])
        self.assertTrue(row['actual_valid'])
        self.assertEqual(row['actual'], row['theta_deg'])
        self.assertEqual(row['target'], 0)
        self.assertEqual(row['out'], row['command_permille'])
        self.assertEqual(row['integral_permille'], -1.25)
        self.assertAlmostEqual(row['arm_target_deg'], 180/math.pi)
        self.assertEqual(row['error_deg'], -row['theta_deg'])
        self.assertEqual(row['arm_error_deg'], row['arm_target_deg']-row['arm_deg'])
        self.assertFalse(row['requested_saturated'])
        for out in (-1000, 1000, -999, 999):
            result = tuning_values(dict(row, command_permille=out))
            self.assertEqual(result['requested_saturated'], abs(out) == 1000)

    def test_tuning_does_not_fabricate_old_future_inactive_or_reserved_targets(self):
        for raw in (frame(version=1), frame(firmware=0x20006, state=2),
                    handover_frame(firmware=0x20005, state=2),
                    frame(firmware=0x20007, state=2),
                    handover_frame(firmware=0x2000a, state=2),
                    frame(firmware=0x20008, state=2),
                    handover_frame(firmware=0x20008, state=2, values=(0, 0, 512, 0x83)),
                    handover_frame(firmware=0x20008, state=0),
                    handover_frame(firmware=0x20008, state=3, fault=8),
                    handover_frame(firmware=0x20008, state=2, values=(0, 0, 512, 0)),
                    handover_frame(firmware=0x20007, state=2, values=(0, 0, 512, 0x83)),
                    handover_frame(firmware=0x20007, state=0),
                    handover_frame(firmware=0x20007, state=3, fault=8),
                    handover_frame(state=2, values=(0, 0, 512, 0x83)),
                    handover_frame(state=0), handover_frame(state=3, fault=8),
                    handover_frame(state=2, values=(0, 0, 0, 0))):
            with self.subTest(raw=raw.hex()):
                row = StreamDecoder().feed(raw)[0]
                self.assertEqual(row['actual'], row['theta_deg'])
                self.assertEqual(row['out'], row['command_permille'])
                self.assertIsNone(row['target'])
                self.assertIsNone(row['arm_target_deg'])
                self.assertIsNone(row['error_deg'])
                self.assertIsNone(row['arm_error_deg'])
                self.assertIsNone(row['parameter_profile'])
                self.assertEqual(row['algorithm'], 'unknown')
        row = StreamDecoder().feed(handover_frame(state=0))[0]
        self.assertIs(row['active'], False)
        self.assertIsNone(row['requested_saturated'])

    def test_invalid_measurements_remain_visible_but_not_valid_error(self):
        baseline = StreamDecoder().feed(handover_frame(state=2, theta=100))[0]
        for changes in ({'sensor_flags': 4}, {'sensor_flags': 13}, {'sensor_flags': 28},
                        {'sensor_flags': None}, {'calibrated': 0}, {'fault': 1}):
            result = tuning_values(dict(baseline, **changes))
            self.assertEqual(result['actual'], baseline['theta_deg'])
            self.assertFalse(result['actual_valid'])
            self.assertIsNone(result['error_deg'])
            self.assertIsNone(result['arm_error_deg'])

    def test_automatic_narrow_csv_and_experiment_csv_match_json_on_close(self):
        wires = [frame(sequence=1, version=1),
                 handover_frame(sequence=2, state=2, theta=-100, values=(-320, 1024, 512, 3)),
                 handover_frame(sequence=3, state=0, values=(0, 0, 0, 0))]
        rows = [StreamDecoder().feed(raw)[0] for raw in wires]
        for i, row in enumerate(rows):
            row['elapsed_s'] = i*.02
        with tempfile.TemporaryDirectory() as directory:
            log = OperationLog(directory, {'source': 'offline_simulation'}, directory_name='fpga_logs')
            recorder = SessionRecorder(directory, {'source': 'offline_simulation'})
            for row in rows:
                log.write_telemetry(row)
                recorder.write(row)
            log.close()
            recorder.close()
            stored = [json.loads(x) for x in (log.directory/'telemetry.jsonl').read_text(encoding='utf-8').splitlines()]
            with (log.directory/'tuning.csv').open(encoding='utf-8-sig', newline='') as source:
                reader = csv.DictReader(source)
                narrow = list(reader)
                self.assertEqual(reader.fieldnames, list(TUNING_CSV_FIELDS))
            self.assertEqual(len(narrow), 3)
            self.assertEqual(narrow[0]['target'], '')
            self.assertEqual(narrow[2]['target'], '')
            self.assertEqual(narrow[2]['active'], 'False')
            self.assertEqual(narrow[1]['algorithm'], 'H_LQI')
            for key in ('actual', 'target', 'out', 'error_deg', 'arm_error_deg', 'integral_permille'):
                self.assertEqual(float(narrow[1][key]), stored[1][key])
            restored = Replay.load(recorder.directory/'samples.csv')
            self.assertEqual(restored[1]['algorithm'], 'H_LQI')
            self.assertEqual(restored[1]['parameter_profile'], H_LQI_PROFILE)
            self.assertTrue(restored[1]['active'])
            self.assertTrue(restored[1]['actual_valid'])
            self.assertIsNone(restored[0]['target'])
            for folder in (log.directory, recorder.directory):
                metadata = json.loads((folder/'metadata.json').read_text(encoding='utf-8'))
                self.assertEqual(metadata['tuning_schema'], TUNING_SCHEMA)
                profile = metadata['tuning_schema']['profile_definitions'][H_LQI_PROFILE]
                self.assertEqual(profile['source'], 'known_firmware_definition')
                self.assertEqual(profile['gain_q10'], [904758, 77909, -34213, -51473])
                self.assertAlmostEqual(profile['ki_permille_per_rad_s'], 237/16.384)
                self.assertEqual(profile['integral_bound_permille'], 100)
                self.assertAlmostEqual(profile['integral_rate_permille_per_s'], 335544*1000/2**24)
            self.assertEqual((recorder.directory/'raw_frames.bin').read_bytes(), b''.join(wires))

    def test_tuning_csv_flushes_while_idle_and_reports_disk_errors(self):
        row = StreamDecoder().feed(handover_frame(state=2))[0]
        with tempfile.TemporaryDirectory() as directory:
            log = OperationLog(directory, {}, directory_name='fpga_logs')
            log.write_telemetry(row)
            try:
                deadline = time.monotonic()+3
                visible = []
                while time.monotonic() < deadline:
                    if log.directory and (log.directory/'tuning.csv').exists():
                        with (log.directory/'tuning.csv').open(encoding='utf-8-sig', newline='') as stream:
                            visible = list(csv.DictReader(stream))
                        if visible:
                            break
                    time.sleep(.02)
                self.assertEqual(len(visible), 1)
            finally:
                log.close()
            failing = OperationLog(Path(directory)/'broken', {}, directory_name='fpga_logs')
            with patch('host.operation_log.csv.DictWriter.writerow', side_effect=OSError('CSV disk failed')):
                failing.write_telemetry(row)
                with self.assertRaisesRegex(OSError, 'CSV disk failed'):
                    failing.close()
            with self.assertRaisesRegex(OSError, 'CSV disk failed'):
                failing.check()

    def test_tuning_csv_writer_never_runs_on_calling_thread(self):
        row = StreamDecoder().feed(handover_frame(state=2))[0]
        entered, release = threading.Event(), threading.Event()
        calling = threading.get_ident()
        seen = []
        original = csv.DictWriter.writerow
        def blocked(writer, value):
            seen.append(threading.get_ident())
            entered.set()
            release.wait(2)
            return original(writer, value)
        with tempfile.TemporaryDirectory() as directory:
            log = OperationLog(directory, {}, directory_name='fpga_logs')
            try:
                with patch('host.operation_log.csv.DictWriter.writerow', side_effect=blocked, autospec=True):
                    log.write_telemetry(row)
                    self.assertTrue(entered.wait(1))
                    log.write_telemetry(row)
                    self.assertNotIn(calling, seen)
                    release.set()
                    log.close()
            finally:
                release.set()
                if not log.closed:
                    log.close()


class HandoverUITests(unittest.TestCase):
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

    def receive(self, raw):
        row = StreamDecoder().feed(raw)[0]
        self.window.receive([row])
        self.window.update_display()
        return row

    def test_converted_observations_and_raw_table_values_are_visible(self):
        self.receive(handover_frame(values=(-320, 1024, 512, 5), state=2))
        text = self.window.motor_diagnostic_label.text()
        self.assertIn('居中积分修正 -1.250‰', text)
        self.assertIn('接管摆臂参考 +57.30°', text)
        self.assertIn('抗饱和冻结', text)
        self.assertIn('project-v0.2.6', self.window.adc_diagnostic_label.text())
        for key, expected in zip(HANDOVER_CONTROL_FIELDS, ('-320', '1024', '512', '5')):
            index = next(i for i, x in enumerate(EXTENSIONS) if x[0] == key)
            self.assertEqual(self.window.ext_table.item(index, 1).text(), expected)
            self.assertIn('非独立PWM', self.window.ext_table.item(index, 1).toolTip())
        self.assertEqual(self.window.worker.sent, [])

    def test_old_or_missing_frame_clears_observations_instead_of_leaking_previous(self):
        self.receive(handover_frame())
        self.receive(frame(sequence=2, firmware=0x20005))
        self.assertNotIn('居中积分修正 -100.004', self.window.motor_diagnostic_label.text())
        for key in HANDOVER_CONTROL_FIELDS:
            index = next(i for i, x in enumerate(EXTENSIONS) if x[0] == key)
            self.assertEqual(self.window.ext_table.item(index, 1).text(), '未提供')
        self.receive(frame(sequence=3, firmware=0x20006))
        self.assertIn('居中积分修正 未提供', self.window.motor_diagnostic_label.text())

    def test_s110_identity_does_not_send_a_command_or_change_motion_authorization(self):
        self.window.direction_check.setChecked(True)
        self.receive(handover_frame(firmware=0x20006, state=0, values=(0, 0, 0, 0)))
        old_gates = {c:self.window.allowed(c) for c in 'GHFBJKLMS'}
        self.receive(handover_frame(firmware=0x20007, state=0, values=(0, 0, 0, 0)))
        self.assertEqual({c:self.window.allowed(c) for c in old_gates}, old_gates)
        self.assertIn('project-v0.2.7', self.window.adc_diagnostic_label.text())
        row = self.receive(handover_frame(firmware=0x20007, state=2, values=(320, 1024, 512, 3)))
        self.assertEqual(row['parameter_profile'], H_LQI_S110_PROFILE)
        self.assertIn('居中积分修正 +1.250‰', self.window.motor_diagnostic_label.text())
        self.assertFalse(self.window.allowed('H'))
        self.assertTrue(self.window.allowed('S'))
        self.assertEqual(self.window.worker.sent, [])

    def test_s120_identity_keeps_motion_gates_and_never_sends_a_command(self):
        self.window.direction_check.setChecked(True)
        for state, fault, sensor in ((0, 0, 12), (0, 0, 4), (2, 0, 12), (0, 1, 12)):
            self.receive(handover_frame(firmware=0x20007, state=state, fault=fault, flags=sensor))
            previous = {c: self.window.allowed(c) for c in 'GHFBJKLMS'}
            row = self.receive(handover_frame(firmware=0x20008, state=state, fault=fault, flags=sensor))
            self.assertEqual({c: self.window.allowed(c) for c in previous}, previous)
            self.assertIn('project-v0.2.8', self.window.adc_diagnostic_label.text())
            if state == 2 and fault == 0:
                self.assertEqual(row['parameter_profile'], H_LQI_S120_PROFILE)
            self.assertTrue(self.window.allowed('S'))
        self.assertEqual(self.window.worker.sent, [])

    def test_observation_flags_do_not_change_motion_gates_or_stop_priority(self):
        self.window.direction_check.setChecked(True)
        for state, fault, sensor in ((0, 0, 12), (0, 0, 4), (2, 0, 12), (0, 1, 12)):
            self.receive(frame(firmware=0x20006, state=state, fault=fault, flags=sensor))
            expected = {c: self.window.allowed(c) for c in 'GHFBJKLM'}
            for flags in (0, 3, 5, 15, 255):
                self.receive(handover_frame(values=(25600, 6144, 512, flags), state=state,
                                            fault=fault, flags=sensor))
                self.assertEqual({c: self.window.allowed(c) for c in 'GHFBJKLM'}, expected)
                self.assertTrue(self.window.allowed('S'))
        self.assertEqual(self.window.worker.sent, [])

    def test_automatic_runtime_log_retains_new_fields(self):
        log = OperationLog(Path(self.temp.name)/'offline', {'source': 'offline_simulation'})
        self.window.runtime_log = log
        self.receive(handover_frame(values=(-1, -1024, 384, 3), state=2))
        self.window.close_runtime_log()
        rows = [json.loads(line) for line in (log.directory/'telemetry.jsonl').read_text(encoding='utf-8').splitlines()]
        self.assertEqual(tuple(rows[0][k] for k in HANDOVER_CONTROL_FIELDS), (-1, -1024, 384, 3))
        self.assertEqual(self.window.worker.sent, [])

    def test_manual_markers_reach_automatic_and_optional_logs_without_experiment(self):
        self.window.log_preferences['control'] = True
        row = self.receive(handover_frame(state=2))
        self.assertIsNone(self.window.recorder)
        self.window.mark_text.setText('已松手')
        self.window.mark_event()
        self.window.mark_recovery()
        self.window.log_event('state', 'internal transition must not be duplicated')
        automatic, optional = self.window.runtime_log, self.window.operation_log
        self.window.close_runtime_log()
        self.window.close_operation_log()
        for logger in (automatic, optional):
            events = [json.loads(line) for line in (logger.directory/'control.jsonl').read_text(encoding='utf-8').splitlines()]
            marked = [e for e in events if e['event'] in ('marker', 'disturbance_end')]
            self.assertEqual([e['kind'] for e in marked], ['marker', 'disturbance_end'])
            self.assertEqual(marked[0]['detail'], '已松手')
            for event in marked:
                self.assertEqual(event['sequence'], row['sequence'])
                self.assertEqual(event['device_time_ms'], row['device_time_ms'])
                self.assertEqual(event['elapsed_s'], row['elapsed_s'])
                self.assertTrue(event['frame_is_fresh'])
            self.assertFalse(any(e.get('detail') == 'internal transition must not be duplicated' for e in events))
        self.assertEqual(self.window.worker.sent, [])

    def test_demo_replay_and_disconnected_markers_do_not_create_device_logs(self):
        self.window.worker = None
        self.window.log_preferences['control'] = True
        for source in ('demo', 'replay', 'serial'):
            self.window.reset_data(source)
            self.window.serial_ready = False
            self.window.mark_text.setText('离线示例标记')
            self.window.mark_event()
            self.window.log_event('disturbance_end', '离线示例结束')
            self.assertIsNone(self.window.runtime_log)
            self.assertIsNone(self.window.operation_log)
        self.assertFalse((Path(self.temp.name)/'fpga_logs').exists())
        self.assertFalse((Path(self.temp.name)/'logs').exists())

    def test_stale_manual_marker_labels_latest_frame_as_stale(self):
        row = self.receive(handover_frame(state=2))
        self.window.last_received = time.monotonic()-2
        self.window.mark_text.setText('再次扶住，串口数据已过期')
        self.window.mark_event()
        logger = self.window.runtime_log
        self.window.close_runtime_log()
        events = [json.loads(line) for line in (logger.directory/'control.jsonl').read_text(encoding='utf-8').splitlines()]
        marker = next(e for e in events if e['event'] == 'marker')
        self.assertFalse(marker['frame_is_fresh'])
        self.assertEqual(marker['sequence'], row['sequence'])
        self.assertEqual(marker['device_time_ms'], row['device_time_ms'])


if __name__ == '__main__':
    unittest.main()
